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
    const PayloadWord nodePayload = encodePayload(node.payload);
    FRONTIER_CHECK(nodePayload != invalidPayloadWord(),
                   "SubtreeBuilder: reserved invalid payload");
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
    built.payload = nodePayload;
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

    // Grow authored bounds bottom-up before packing. The builder nodes are
    // edit-time storage and are consumed here, so this needs no temporary
    // per-packed-node bounds array.
    for (uint32_t source = uint32_t(nodes_.size()); source-- > 0;)
    {
        const AABB nodeBounds = nodes_[source].bounds.toAABB();
        FRONTIER_CHECK(finiteNonEmptyBounds(nodeBounds),
                       "SubtreeBuilder: empty or non-finite node bounds");
        const NodeId owner = nodes_[source].parent;
        if (owner != kInvalidIndex)
        {
            AABB ownerBounds = nodes_[owner].bounds.toAABB();
            ownerBounds.expand(nodeBounds);
            nodes_[owner].bounds = ownerBounds;
        }
    }
    AABB rootBounds = AABB::empty();
    for (const NodeId root : roots_)
        rootBounds.expand(nodes_[root].bounds.toAABB());

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
    header->reservedOffset = 0;
    header->payloadOffset = layout.payload;
    header->parentOffset = layout.parent;
    header->subtreeSizeOffset = layout.subtreeSize;
    header->metaOffset = layout.meta;
    header->errorOffset = layout.error;
    header->branchingFactor = kWide;
    header->invalidPayloadWord = uint64_t(invalidPayloadWord());
    header->payloadBytes = sizeof(PayloadWord);

    auto* wide = reinterpret_cast<WideBlock*>(base + layout.wide);
    auto* blockMask = reinterpret_cast<uint32_t*>(base + layout.mask);
    auto* payload = reinterpret_cast<PayloadWord*>(base + layout.payload);
    auto* parent = reinterpret_cast<uint32_t*>(base + layout.parent);
    auto* subtreeSize = reinterpret_cast<uint32_t*>(base + layout.subtreeSize);
    auto* meta = reinterpret_cast<uint32_t*>(base + layout.meta);
    auto* geometricError = reinterpret_cast<float*>(base + layout.error);

    uint32_t emitted = 0;

    const auto emit = [&](uint32_t packedParent, uint32_t childCount,
                          bool mountable, PayloadWord nodePayload,
                          float error, uint32_t source) -> uint32_t
    {
        const uint32_t index = emitted++;
        parent[index] = packedParent;
        // Temporary packed->builder mapping. Replaced with subtree extents
        // after the wide blocks have consumed the source sibling lists.
        subtreeSize[index] = source;
        meta[index] = childCount | (mountable ? kMetaMountable : 0u);
        payload[index] = nodePayload;
        geometricError[index] = error;
        return index;
    };

    emit(packParent(0, 0), uint32_t(roots_.size()), false,
         invalidPayloadWord(), FLT_MAX, kInvalidIndex);

    std::vector<uint32_t> remap(nodes_.size(), kInvalidIndex);
    // Source node and sibling ordinal fit the same 20+9-bit shape as a
    // serialized parent word. Reusing it keeps the build stack at four bytes
    // per node without adding another temporary structure.
    std::vector<uint32_t> stack;
    std::vector<uint32_t> siblingScratch;
    stack.reserve(nodes_.size());
    siblingScratch.reserve(maxChildren);
    for (uint32_t ordinal = uint32_t(roots_.size()); ordinal-- > 0;)
        stack.push_back(packParent(roots_[ordinal], ordinal));

    while (!stack.empty())
    {
        const uint32_t queued = stack.back();
        stack.pop_back();
        const uint32_t source = packedParentIndex(queued);
        const uint32_t ordinal = packedParentOrdinal(queued);
        const BuildNode& node = nodes_[source];
        const uint32_t parentIndex =
            node.parent == kInvalidIndex ? 0 : remap[node.parent];
        const uint32_t packed =
            emit(packParent(parentIndex, ordinal), node.childCount(),
                 node.mountable(), node.payload, node.geometricError, source);
        remap[source] = packed;
        siblingScratch.clear();
        for (NodeId child = node.firstChild; child != kInvalidIndex;
             child = nodes_[child].nextSibling)
            siblingScratch.push_back(child);
        for (uint32_t childOrdinal = uint32_t(siblingScratch.size());
             childOrdinal-- > 0;)
            stack.push_back(packParent(siblingScratch[childOrdinal],
                                       childOrdinal));
    }
    FRONTIER_CHECK(emitted == total,
                   "SubtreeBuilder: internal emission count mismatch");

    // Clamp error monotonically from the implicit parent down.
    for (uint32_t i = 1; i < total; ++i)
    {
        const float parentError =
            geometricError[packedParentIndex(parent[i])];
        if (geometricError[i] > parentError)
            geometricError[i] = parentError;
    }

    uint32_t wideIndex = 0;
    for (uint32_t i = 0; i < total; ++i)
    {
        const uint32_t childCount = metaChildCount(meta[i]);
        if (childCount == 0) continue;

        meta[i] |= wideIndex << kMetaOffsetShift;

        NodeId childSource =
            i == 0 ? kInvalidIndex : nodes_[subtreeSize[i]].firstChild;
        for (uint32_t first = 0; first < childCount; first += kWide)
        {
            WideBlock block;
            block.bounds = WideBounds::allEmpty();
            block.error = float8::splat(0.0f);
            for (uint32_t lane = 0; lane < kWide; ++lane)
                block.child[lane] = kInvalidIndex;
            uint32_t valid = 0;
            uint32_t leaf = 0;
            uint32_t zeroError = 0;
            for (uint32_t lane = 0;
                 lane < kWide && first + lane < childCount; ++lane)
            {
                const NodeId laneSource =
                    i == 0 ? roots_[first + lane] : childSource;
                FRONTIER_ASSERT(laneSource != kInvalidIndex,
                                "builder child list ended early");
                const uint32_t child = remap[laneSource];
                block.bounds.setLane(
                    lane, nodes_[laneSource].bounds.toAABB());
                block.error.v[lane] = geometricError[child];
                block.child[lane] = child;
                valid |= 1u << lane;
                if (geometricError[child] == 0.0f)
                    zeroError |= 1u << lane;
                if (metaChildCount(meta[child]) == 0 &&
                    !metaIsMountable(meta[child]))
                    leaf |= 1u << lane;
                if (i != 0)
                    childSource = nodes_[laneSource].nextSibling;
            }
            wide[wideIndex] = block;
            blockMask[wideIndex] =
                valid | (leaf << kBlockLeafShift) |
                (zeroError << kBlockZeroErrorShift);
            ++wideIndex;
        }
        FRONTIER_ASSERT(i == 0 || childSource == kInvalidIndex,
                        "builder child list exceeds metadata");
    }
    FRONTIER_CHECK(wideIndex == wideCount,
                   "SubtreeBuilder: internal wide-block count mismatch");

    std::fill(subtreeSize, subtreeSize + total, 1u);
    for (uint32_t i = total; i-- > 1;)
        subtreeSize[packedParentIndex(parent[i])] += subtreeSize[i];

    header->rootBoundsMin[0] = rootBounds.mn.x;
    header->rootBoundsMin[1] = rootBounds.mn.y;
    header->rootBoundsMin[2] = rootBounds.mn.z;
    header->rootBoundsMax[0] = rootBounds.mx.x;
    header->rootBoundsMax[1] = rootBounds.mx.y;
    header->rootBoundsMax[2] = rootBounds.mx.z;

    const auto packedNodeBounds = [&](uint32_t index)
    {
        if (index == 0) return rootBounds;
        const uint32_t owner = packedParentIndex(parent[index]);
        const uint32_t ordinal = packedParentOrdinal(parent[index]);
        return wide[metaWideOffset(meta[owner]) + ordinal / kWide]
            .bounds.lane(ordinal & (kWide - 1u));
    };

    for (uint32_t i = 1; i < total; ++i)
    {
        const uint32_t parentIndex = packedParentIndex(parent[i]);
        FRONTIER_CHECK(parentIndex < i,
                       "SubtreeBuilder: parent ordering violated");
        FRONTIER_CHECK(i + subtreeSize[i] <= total,
                       "SubtreeBuilder: subtree extent violated");
        FRONTIER_CHECK(metaChildCount(meta[i]) == 0 ||
                           packedParentIndex(parent[i + 1]) == i,
                       "SubtreeBuilder: first child is not adjacent");
        const AABB nodeBounds = packedNodeBounds(i);
        const AABB parentBounds = packedNodeBounds(parentIndex);
        FRONTIER_CHECK(parentBounds.contains(nodeBounds),
                       "SubtreeBuilder: bounds containment violated");
        FRONTIER_CHECK(geometricError[i] <= geometricError[parentIndex],
                       "SubtreeBuilder: error monotonicity violated");
        FRONTIER_CHECK(metaChildCount(meta[i]) == 0 ||
                           !metaIsMountable(meta[i]),
                       "SubtreeBuilder: local and mounted children conflict");
    }

    return bytes;
}

} // namespace frontier
