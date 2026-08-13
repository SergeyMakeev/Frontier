#include "frontier/builder.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstring>

#include "frontier/detail/subtree_data.h"

namespace frontier {
namespace {

bool finiteNonEmptyBounds(const AABB& bounds)
{
    return bounds.mn.x <= bounds.mx.x &&
           bounds.mn.y <= bounds.mx.y &&
           bounds.mn.z <= bounds.mx.z &&
           bounds.mx.x - bounds.mn.x < FLT_MAX &&
           bounds.mx.y - bounds.mn.y < FLT_MAX &&
           bounds.mx.z - bounds.mn.z < FLT_MAX;
}

} // namespace

void SubtreeBuilder::reserve(uint32_t nodeCount)
{
    FRONTIER_CHECK(!built_, "SubtreeBuilder: builder already consumed");
    FRONTIER_CHECK(nodeCount < detail::kMaxSubtreeNodes,
                   "SubtreeBuilder: node-count limit exceeded");
    nodes_.reserve(nodeCount);
    roots_.reserve(std::min(nodeCount, detail::kMaxChildren));
}

SubtreeBuilder::NodeId SubtreeBuilder::createNode(const NodeDesc& node)
{
    return createNode(kInvalidIndex, node);
}

SubtreeBuilder::NodeId SubtreeBuilder::createNode(NodeId parent,
                                                   const NodeDesc& node)
{
    using namespace detail;
    FRONTIER_CHECK(!built_, "SubtreeBuilder: builder already consumed");
    FRONTIER_CHECK(nodes_.size() < detail::kMaxSubtreeNodes - 1,
                   "SubtreeBuilder: node-count limit exceeded");
    FRONTIER_CHECK(parent == kInvalidIndex || parent < nodes_.size(),
                   "SubtreeBuilder: invalid parent");
    FRONTIER_CHECK(node.geometricError >= 0.0f &&
                       std::isfinite(node.geometricError),
                   "SubtreeBuilder: invalid geometric error");
    FRONTIER_CHECK((node.flags & ~uint32_t(NodeDesc::FlagMountable)) == 0,
                   "SubtreeBuilder: unknown node flags");
    if (parent != kInvalidIndex)
    {
        FRONTIER_CHECK(!nodes_[parent].mountable(),
                       "SubtreeBuilder: mountable node must stay a leaf");
        FRONTIER_CHECK(nodes_[parent].childCount() < kMaxChildren,
                       "SubtreeBuilder: fanout exceeds limit");
    }
    else
    {
        FRONTIER_CHECK(roots_.size() < kMaxChildren,
                       "SubtreeBuilder: too many top-level nodes");
    }

    BuildNode built;
    built.bounds = node.bounds;
    built.payload = node.payload;
    built.geometricError = node.geometricError;
    built.parent = parent;
    built.setMountable(node.isMountable());
    nodes_.push_back(built);
    const NodeId id = NodeId(nodes_.size() - 1);
    if (parent == kInvalidIndex)
    {
        roots_.push_back(id);
    }
    else
    {
        BuildNode& owner = nodes_[parent];
        if (owner.lastChild == kInvalidIndex)
            owner.firstChild = id;
        else
            nodes_[owner.lastChild].nextSibling = id;
        owner.lastChild = id;
        const uint32_t childCount = owner.childCount() + 1;
        FRONTIER_ASSERT(childCount <= kMaxChildren,
                        "builder fanout preflight failed");
        owner.setChildCount(childCount);
    }
    return id;
}

SubtreeBytes SubtreeBuilder::build(const FrontierContext& context)
{
    using namespace detail;
    FRONTIER_CHECK(!built_, "SubtreeBuilder: builder already consumed");
    FRONTIER_CHECK(!roots_.empty(), "SubtreeBuilder: subtree has no nodes");
    built_ = true;

    const uint32_t total = uint32_t(nodes_.size()) + 1;
    uint64_t wideCount64 = (roots_.size() + kWide - 1) / kWide;
    uint32_t maxChildren = 0;
    for (const BuildNode& node : nodes_)
    {
        wideCount64 += (node.childCount() + kWide - 1) / kWide;
        maxChildren = std::max(maxChildren, node.childCount());
    }
    FRONTIER_CHECK(wideCount64 <= kMaxWideOffset,
                   "SubtreeBuilder: wide offset overflow");
    const uint32_t wideCount = uint32_t(wideCount64);

    const SubtreeLayout layout = subtreeLayout(total, wideCount);
    SubtreeBytes bytes(layout.totalBytes, context);
    std::memset(bytes.data(), 0, bytes.size());

    auto* base = bytes.data();
    auto* header = reinterpret_cast<SubtreeHeader*>(base);
    header->magic = kSubtreeMagic;
    header->version = kSubtreeVersion;
    header->headerBytes = uint16_t(sizeof(SubtreeHeader));
    header->totalBytes = layout.totalBytes;
    header->nodeCount = total;
    header->wideCount = wideCount;
    header->wideOffset = layout.wide;
    header->maskOffset = layout.mask;
    header->bboxOffset = layout.bbox;
    header->payloadOffset = layout.payload;
    header->parentOffset = layout.parent;
    header->subtreeSizeOffset = layout.subtreeSize;
    header->metaOffset = layout.meta;
    header->errorOffset = layout.error;
    header->branchingFactor = kWide;

    auto* wide = reinterpret_cast<WideBlock*>(base + layout.wide);
    auto* blockMask = reinterpret_cast<uint32_t*>(base + layout.mask);
    auto* bbox = reinterpret_cast<AABB*>(base + layout.bbox);
    auto* payload = reinterpret_cast<UserPayload*>(base + layout.payload);
    auto* parent = reinterpret_cast<uint32_t*>(base + layout.parent);
    auto* subtreeSize = reinterpret_cast<uint32_t*>(base + layout.subtreeSize);
    auto* meta = reinterpret_cast<uint32_t*>(base + layout.meta);
    auto* geometricError = reinterpret_cast<float*>(base + layout.error);

    uint32_t emitted = 0;

    const auto emit = [&](uint32_t parentIndex, uint32_t childCount,
                          bool mountable, UserPayload nodePayload,
                          const AABB& nodeBounds, float error) -> uint32_t
    {
        const uint32_t index = emitted++;
        parent[index] = parentIndex;
        subtreeSize[index] = 1;
        meta[index] = childCount | (mountable ? kMetaMountable : 0u);
        payload[index] = nodePayload;
        bbox[index] = nodeBounds;
        geometricError[index] = error;
        return index;
    };

    emit(0, uint32_t(roots_.size()), false, kSentinelPayload, AABB::empty(),
         FLT_MAX);

    std::vector<uint32_t> remap(nodes_.size(), kInvalidIndex);
    std::vector<uint32_t> stack;
    std::vector<uint32_t> siblingScratch;
    stack.reserve(nodes_.size());
    siblingScratch.reserve(maxChildren);
    for (auto it = roots_.rbegin(); it != roots_.rend(); ++it)
        stack.push_back(*it);

    while (!stack.empty())
    {
        const uint32_t source = stack.back();
        stack.pop_back();
        const BuildNode& node = nodes_[source];
        const uint32_t parentIndex =
            node.parent == kInvalidIndex ? 0 : remap[node.parent];
        const uint32_t packed =
            emit(parentIndex, node.childCount(), node.mountable(),
                 node.payload, node.bounds.toAABB(), node.geometricError);
        remap[source] = packed;
        siblingScratch.clear();
        for (NodeId child = node.firstChild; child != kInvalidIndex;
             child = nodes_[child].nextSibling)
            siblingScratch.push_back(child);
        for (auto child = siblingScratch.rbegin();
             child != siblingScratch.rend(); ++child)
            stack.push_back(*child);
    }
    FRONTIER_CHECK(emitted == total,
                   "SubtreeBuilder: internal emission count mismatch");

    // Derive conservative bounds and contiguous subtree extents bottom-up.
    for (uint32_t i = total; i-- > 1;)
    {
        subtreeSize[parent[i]] += subtreeSize[i];
        bbox[parent[i]].expand(bbox[i]);
    }
    for (uint32_t i = 1; i < total; ++i)
        FRONTIER_CHECK(finiteNonEmptyBounds(bbox[i]),
                       "SubtreeBuilder: empty or non-finite node bounds");

    // Clamp error monotonically from the implicit parent down.
    for (uint32_t i = 1; i < total; ++i)
    {
        const float parentError = geometricError[parent[i]];
        if (geometricError[i] > parentError)
            geometricError[i] = parentError;
    }

    uint32_t wideIndex = 0;
    for (uint32_t i = 0; i < total; ++i)
    {
        const uint32_t childCount = metaChildCount(meta[i]);
        if (childCount == 0) continue;

        meta[i] |= wideIndex << kMetaOffsetShift;

        uint32_t child = i + 1;
        for (uint32_t first = 0; first < childCount; first += kWide)
        {
            WideBlock block;
            block.bounds = WideBounds::allEmpty();
            block.error = float8::splat(0.0f);
            for (uint32_t lane = 0; lane < kWide; ++lane)
                block.child[lane] = kInvalidIndex;
            uint32_t valid = 0;
            uint32_t leaf = 0;
            for (uint32_t lane = 0;
                 lane < kWide && first + lane < childCount; ++lane)
            {
                block.bounds.setLane(lane, bbox[child]);
                block.error.v[lane] = geometricError[child];
                block.child[lane] = child;
                valid |= 1u << lane;
                if (metaChildCount(meta[child]) == 0 &&
                    !metaIsMountable(meta[child]))
                    leaf |= 1u << lane;
                child += subtreeSize[child];
            }
            wide[wideIndex] = block;
            blockMask[wideIndex] = valid | (leaf << kBlockLeafShift);
            ++wideIndex;
        }
    }
    FRONTIER_CHECK(wideIndex == wideCount,
                   "SubtreeBuilder: internal wide-block count mismatch");

    for (uint32_t i = 1; i < total; ++i)
    {
        FRONTIER_CHECK(parent[i] < i, "SubtreeBuilder: parent ordering violated");
        FRONTIER_CHECK(i + subtreeSize[i] <= total,
                       "SubtreeBuilder: subtree extent violated");
        FRONTIER_CHECK(metaChildCount(meta[i]) == 0 || parent[i + 1] == i,
                       "SubtreeBuilder: first child is not adjacent");
        FRONTIER_CHECK(bbox[parent[i]].contains(bbox[i]),
                       "SubtreeBuilder: bounds containment violated");
        FRONTIER_CHECK(geometricError[i] <= geometricError[parent[i]],
                       "SubtreeBuilder: error monotonicity violated");
        FRONTIER_CHECK(metaChildCount(meta[i]) == 0 ||
                           !metaIsMountable(meta[i]),
                       "SubtreeBuilder: local and mounted children conflict");
    }

    return bytes;
}

} // namespace frontier
