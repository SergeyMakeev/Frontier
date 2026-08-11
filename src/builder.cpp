#include "frontier/builder.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstring>

#include "frontier/detail/subtree_data.h"

namespace frontier {

void SubtreeBuilder::reserve(uint32_t nodeCount)
{
    FRONTIER_CHECK(!built_, "SubtreeBuilder: builder already consumed");
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
    FRONTIER_CHECK(parent == kInvalidIndex || parent < nodes_.size(),
                   "SubtreeBuilder: invalid parent");
    FRONTIER_CHECK(node.geometricError >= 0.0f &&
                       std::isfinite(node.geometricError),
                   "SubtreeBuilder: invalid geometric error");
    if (parent != kInvalidIndex)
        FRONTIER_CHECK(!nodes_[parent].mountable(),
                       "SubtreeBuilder: mountable node must stay a leaf");

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
        FRONTIER_CHECK(roots_.size() <= kMaxChildren,
                       "SubtreeBuilder: too many top-level nodes");
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
        FRONTIER_CHECK(childCount <= kMaxChildren,
                       "SubtreeBuilder: fanout exceeds limit");
        owner.setChildCount(childCount);
    }
    return id;
}

SubtreeBytes SubtreeBuilder::build(const FrontierContext& context)
{
    using namespace detail;
    FRONTIER_CHECK(!built_, "SubtreeBuilder: builder already consumed");
    built_ = true;
    FRONTIER_CHECK(!roots_.empty(), "SubtreeBuilder: subtree has no nodes");

    const uint32_t total = uint32_t(nodes_.size()) + 1;
    std::vector<uint32_t> parent;
    std::vector<uint32_t> subtreeSize;
    std::vector<uint32_t> meta;
    std::vector<UserPayload> payload;
    std::vector<AABB> bbox;
    std::vector<float> geometricError;
    std::vector<WideBlock> wide;
    std::vector<uint32_t> blockMask;

    parent.reserve(total);
    subtreeSize.reserve(total);
    meta.reserve(total);
    payload.reserve(total);
    bbox.reserve(total);
    geometricError.reserve(total);

    const auto emit = [&](uint32_t parentIndex, uint32_t childCount,
                          bool mountable, UserPayload nodePayload,
                          const AABB& nodeBounds, float error) -> uint32_t
    {
        parent.push_back(parentIndex);
        subtreeSize.push_back(1);
        meta.push_back(childCount | (mountable ? kMetaMountable : 0u));
        payload.push_back(nodePayload);
        bbox.push_back(nodeBounds);
        geometricError.push_back(error);
        return uint32_t(parent.size() - 1);
    };

    emit(0, uint32_t(roots_.size()), false, kSentinelPayload, AABB::empty(),
         FLT_MAX);

    std::vector<uint32_t> remap(nodes_.size(), kInvalidIndex);
    std::vector<uint32_t> stack;
    std::vector<uint32_t> siblingScratch;
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
    FRONTIER_CHECK(parent.size() == total,
                   "SubtreeBuilder: internal emission count mismatch");

    // Derive conservative bounds and contiguous subtree extents bottom-up.
    for (uint32_t i = total - 1; i >= 1; --i)
    {
        subtreeSize[parent[i]] += subtreeSize[i];
        bbox[parent[i]].expand(bbox[i]);
    }
    for (uint32_t i = 1; i < total; ++i)
        FRONTIER_CHECK(!bbox[i].isEmpty(),
                       "SubtreeBuilder: leaf node without bounds");

    // Clamp error monotonically from the implicit parent down.
    for (uint32_t i = 1; i < total; ++i)
    {
        const float parentError = geometricError[parent[i]];
        if (geometricError[i] > parentError)
            geometricError[i] = parentError;
    }

    std::vector<uint32_t> children;
    for (uint32_t i = 0; i < total; ++i)
    {
        const uint32_t childCount = metaChildCount(meta[i]);
        if (childCount == 0) continue;

        children.clear();
        uint32_t child = i + 1;
        for (uint32_t c = 0; c < childCount; ++c)
        {
            children.push_back(child);
            child += subtreeSize[child];
        }

        const uint32_t offset = uint32_t(wide.size());
        FRONTIER_CHECK(offset <= kMaxWideOffset,
                       "SubtreeBuilder: wide offset overflow");
        meta[i] |= offset << kMetaOffsetShift;

        for (uint32_t base = 0; base < childCount; base += kWide)
        {
            WideBlock block;
            block.bounds = WideBounds::allEmpty();
            block.error = float8::splat(0.0f);
            for (uint32_t lane = 0; lane < kWide; ++lane)
                block.child[lane] = kInvalidIndex;
            uint32_t valid = 0;
            uint32_t leaf = 0;
            for (uint32_t lane = 0;
                 lane < kWide && base + lane < childCount; ++lane)
            {
                const uint32_t childIndex = children[base + lane];
                block.bounds.setLane(lane, bbox[childIndex]);
                block.error.v[lane] = geometricError[childIndex];
                block.child[lane] = childIndex;
                valid |= 1u << lane;
                if (metaChildCount(meta[childIndex]) == 0 &&
                    !metaIsMountable(meta[childIndex]))
                    leaf |= 1u << lane;
            }
            wide.push_back(block);
            blockMask.push_back(valid | (leaf << kBlockLeafShift));
        }
    }

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

    const uint32_t wideCount = uint32_t(wide.size());
    const size_t byteCount = subtreeBlobBytes(total, wideCount);
    SubtreeBytes bytes(byteCount, context);
    std::memset(bytes.data(), 0, byteCount);

    auto* base = bytes.data();
    auto* header = reinterpret_cast<SubtreeHeader*>(base);
    header->magic = kSubtreeMagic;
    header->version = kSubtreeVersion;
    header->headerBytes = uint16_t(sizeof(SubtreeHeader));
    header->totalBytes = uint32_t(byteCount);
    header->nodeCount = total;
    header->wideCount = wideCount;

    size_t offset = sizeof(SubtreeHeader);
    const auto place = [&](uint32_t& destination, size_t alignment,
                           size_t count, size_t stride, const void* source)
    {
        offset = (offset + alignment - 1) & ~(alignment - 1);
        destination = uint32_t(offset);
        if (count) std::memcpy(base + offset, source, count * stride);
        offset += count * stride;
    };

    place(header->wideOffset, alignof(WideBlock), wideCount,
          sizeof(WideBlock), wide.data());
    place(header->maskOffset, alignof(uint32_t), wideCount,
          sizeof(uint32_t), blockMask.data());
    place(header->bboxOffset, alignof(AABB), total, sizeof(AABB), bbox.data());
    place(header->payloadOffset, alignof(UserPayload), total,
          sizeof(UserPayload), payload.data());
    place(header->parentOffset, alignof(uint32_t), total, sizeof(uint32_t),
          parent.data());
    place(header->subtreeSizeOffset, alignof(uint32_t), total,
          sizeof(uint32_t), subtreeSize.data());
    place(header->metaOffset, alignof(uint32_t), total, sizeof(uint32_t),
          meta.data());
    place(header->errorOffset, alignof(float), total, sizeof(float),
          geometricError.data());

    return bytes;
}

} // namespace frontier
