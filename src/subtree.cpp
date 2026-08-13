#include "frontier/subtree.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "frontier/detail/subtree_data.h"

namespace frontier {
namespace {

constexpr size_t alignUp(size_t value, size_t alignment)
{
    return (value + alignment - 1) & ~(alignment - 1);
}

detail::SubtreeLayout computeLayout(uint32_t nodeCount, uint32_t wideCount)
{
    using namespace detail;
    SubtreeLayout layout;
    size_t offset = sizeof(SubtreeHeader);

    offset = alignUp(offset, alignof(WideBlock));
    layout.wide = uint32_t(offset);
    offset += size_t(wideCount) * sizeof(WideBlock);

    offset = alignUp(offset, alignof(uint32_t));
    layout.mask = uint32_t(offset);
    offset += size_t(wideCount) * sizeof(uint32_t);

    offset = alignUp(offset, alignof(AABB));
    layout.bbox = uint32_t(offset);
    offset += size_t(nodeCount) * sizeof(AABB);

    offset = alignUp(offset, alignof(UserPayload));
    layout.payload = uint32_t(offset);
    offset += size_t(nodeCount) * sizeof(UserPayload);

    offset = alignUp(offset, alignof(uint32_t));
    layout.parent = uint32_t(offset);
    offset += size_t(nodeCount) * sizeof(uint32_t);
    layout.subtreeSize = uint32_t(offset);
    offset += size_t(nodeCount) * sizeof(uint32_t);
    layout.meta = uint32_t(offset);
    offset += size_t(nodeCount) * sizeof(uint32_t);
    layout.error = uint32_t(offset);
    offset += size_t(nodeCount) * sizeof(float);

    layout.totalBytes = uint32_t(alignUp(offset, kSubtreeAlign));
    return layout;
}

#if FRONTIER_VALIDATE_SUBTREES
bool finiteNonEmptyBounds(const AABB& bounds)
{
    return bounds.mn.x <= bounds.mx.x &&
           bounds.mn.y <= bounds.mx.y &&
           bounds.mn.z <= bounds.mx.z &&
           bounds.mx.x - bounds.mn.x < FLT_MAX &&
           bounds.mx.y - bounds.mn.y < FLT_MAX &&
           bounds.mx.z - bounds.mn.z < FLT_MAX;
}

bool sameBounds(const AABB& a, const AABB& b)
{
    return a.mn.x == b.mn.x && a.mn.y == b.mn.y && a.mn.z == b.mn.z &&
           a.mx.x == b.mx.x && a.mx.y == b.mx.y && a.mx.z == b.mx.z;
}

bool canonicalEmptyBounds(const AABB& bounds)
{
    return bounds.mn.x == FLT_MAX && bounds.mn.y == FLT_MAX &&
           bounds.mn.z == FLT_MAX && bounds.mx.x == -FLT_MAX &&
           bounds.mx.y == -FLT_MAX && bounds.mx.z == -FLT_MAX;
}
#endif

} // namespace

SubtreeBytes::SubtreeBytes(size_t size, const FrontierContext& context)
    : size_(size), context_(context)
{
    if (size == 0) return;
    FRONTIER_CHECK(context_.alloc != nullptr && context_.free != nullptr,
                   "SubtreeBytes: allocator callbacks are required");
    data_ = static_cast<std::byte*>(
        context_.alloc(size, kSubtreeByteAlignment, context_.user));
    FRONTIER_CHECK(data_ != nullptr, "SubtreeBytes: allocation failed");
    if ((reinterpret_cast<uintptr_t>(data_) &
         (kSubtreeByteAlignment - 1)) != 0)
    {
        context_.free(data_, context_.user);
        data_ = nullptr;
        size_ = 0;
        FRONTIER_CHECK(false,
                       "SubtreeBytes: allocator returned insufficiently aligned storage");
    }
}

SubtreeBytes::SubtreeBytes(const SubtreeBytes& other)
    : SubtreeBytes(other.size_, other.context_)
{
    if (size_) std::memcpy(data_, other.data_, size_);
}

SubtreeBytes& SubtreeBytes::operator=(const SubtreeBytes& other)
{
    if (this != &other)
    {
        SubtreeBytes copy(other);
        *this = std::move(copy);
    }
    return *this;
}

SubtreeBytes& SubtreeBytes::operator=(SubtreeBytes&& other) noexcept
{
    if (this != &other)
    {
        release();
        moveFrom(other);
    }
    return *this;
}

void SubtreeBytes::release()
{
    if (data_) context_.free(data_, context_.user);
    data_ = nullptr;
    size_ = 0;
}

void SubtreeBytes::moveFrom(SubtreeBytes& other) noexcept
{
    data_ = other.data_;
    size_ = other.size_;
    context_ = other.context_;
    other.data_ = nullptr;
    other.size_ = 0;
}

namespace detail {

SubtreeLayout subtreeLayout(uint32_t nodeCount, uint32_t wideCount)
{
    return computeLayout(nodeCount, wideCount);
}

void validateSubtreeBytes(const SubtreeBytes& bytes)
{
    FRONTIER_CHECK(bytes.data() != nullptr,
                   "registerSubtree: empty byte array");
    FRONTIER_CHECK(bytes.size() >= sizeof(SubtreeHeader),
                   "registerSubtree: truncated header");
    FRONTIER_CHECK(
        (reinterpret_cast<uintptr_t>(bytes.data()) & (kSubtreeAlign - 1)) == 0,
        "registerSubtree: byte array is not sufficiently aligned");

    const auto* header = reinterpret_cast<const SubtreeHeader*>(bytes.data());
    FRONTIER_CHECK(header->magic == kSubtreeMagic,
                   "registerSubtree: bad magic or byte order");
    FRONTIER_CHECK(header->version == kSubtreeVersion,
                   "registerSubtree: format version mismatch");
    FRONTIER_CHECK(header->headerBytes == sizeof(SubtreeHeader),
                   "registerSubtree: header size mismatch");
    FRONTIER_CHECK(header->branchingFactor == kWide,
                   "registerSubtree: branching factor mismatch");
    FRONTIER_CHECK(std::all_of(header->reserved, header->reserved + 18,
                               [](uint32_t value) { return value == 0; }),
                   "registerSubtree: reserved header fields are non-zero");
    FRONTIER_CHECK(header->nodeCount > 1,
                   "registerSubtree: subtree has no renderable nodes");
    FRONTIER_CHECK(header->nodeCount <= kMaxSubtreeNodes,
                   "registerSubtree: node count exceeds handle index space");
    FRONTIER_CHECK(header->wideCount > 0 &&
                       header->wideCount < header->nodeCount,
                   "registerSubtree: invalid wide-block count");
    FRONTIER_CHECK(header->totalBytes == bytes.size(),
                   "registerSubtree: byte-array size mismatch");

    const SubtreeLayout layout =
        computeLayout(header->nodeCount, header->wideCount);
    FRONTIER_CHECK(
        layout.totalBytes == header->totalBytes &&
            layout.wide == header->wideOffset &&
            layout.mask == header->maskOffset &&
            layout.bbox == header->bboxOffset &&
            layout.payload == header->payloadOffset &&
            layout.parent == header->parentOffset &&
            layout.subtreeSize == header->subtreeSizeOffset &&
            layout.meta == header->metaOffset &&
            layout.error == header->errorOffset,
        "registerSubtree: offsets inconsistent with subtree shape");

    const auto* payload = reinterpret_cast<const UserPayload*>(
        bytes.data() + header->payloadOffset);
    FRONTIER_CHECK(payload[0] == kInvalidPayload,
                   "registerSubtree: missing implicit-parent sentinel");

    const auto* parent = reinterpret_cast<const uint32_t*>(
        bytes.data() + header->parentOffset);
    const auto* subtreeSize = reinterpret_cast<const uint32_t*>(
        bytes.data() + header->subtreeSizeOffset);
    const auto* meta = reinterpret_cast<const uint32_t*>(
        bytes.data() + header->metaOffset);
    const auto* error = reinterpret_cast<const float*>(
        bytes.data() + header->errorOffset);

    const uint32_t nodeCount = header->nodeCount;
    FRONTIER_CHECK(parent[0] == 0 && subtreeSize[0] == nodeCount,
                   "registerSubtree: invalid implicit-parent extent");
    const uint32_t rootChildren = metaChildCount(meta[0]);
    FRONTIER_CHECK(!metaIsMountable(meta[0]) && rootChildren != 0,
                   "registerSubtree: invalid implicit-parent metadata");
    const uint32_t rootBlocks = (rootChildren + kWide - 1) / kWide;
    FRONTIER_CHECK(metaWideOffset(meta[0]) == 0 &&
                       rootBlocks <= header->wideCount,
                   "registerSubtree: implicit-parent blocks escape array");
    FRONTIER_CHECK(error[0] == FLT_MAX,
                   "registerSubtree: invalid implicit-parent error");

#if FRONTIER_VALIDATE_SUBTREES
    const auto* wide = reinterpret_cast<const WideBlock*>(
        bytes.data() + header->wideOffset);
    const auto* blockMask = reinterpret_cast<const uint32_t*>(
        bytes.data() + header->maskOffset);
    const auto* bbox = reinterpret_cast<const AABB*>(
        bytes.data() + header->bboxOffset);

    // Validate all scalar arrays before using any serialized index. This is
    // deliberately linear: registration remains zero-copy, while malformed
    // persisted bytes can never become unchecked traversal pointers.
    for (uint32_t node = 0; node < nodeCount; ++node)
    {
        FRONTIER_CHECK(finiteNonEmptyBounds(bbox[node]),
                       "registerSubtree: invalid node bounds");
        if (node != 0)
            FRONTIER_CHECK(payload[node] != kInvalidPayload,
                           "registerSubtree: reserved invalid payload");
        FRONTIER_CHECK(subtreeSize[node] != 0 &&
                           subtreeSize[node] <= nodeCount - node,
                       "registerSubtree: invalid subtree extent");

        const uint32_t children = metaChildCount(meta[node]);
        FRONTIER_CHECK(!metaIsMountable(meta[node]) || children == 0,
                       "registerSubtree: mountable node has local children");
        if (children == 0)
            FRONTIER_CHECK(subtreeSize[node] == 1,
                           "registerSubtree: leaf has descendants");

        if (node == 0) continue;
        FRONTIER_CHECK(parent[node] < node,
                       "registerSubtree: parent ordering violated");
        FRONTIER_CHECK(error[node] >= 0.0f &&
                           std::isfinite(error[node]) &&
                           error[node] <= error[parent[node]],
                       "registerSubtree: invalid geometric error");
        FRONTIER_CHECK(bbox[parent[node]].contains(bbox[node]),
                       "registerSubtree: parent bounds do not contain child");
    }

    uint32_t expectedWide = 0;
    for (uint32_t node = 0; node < nodeCount; ++node)
    {
        const uint32_t children = metaChildCount(meta[node]);
        if (children == 0)
        {
            FRONTIER_CHECK(metaWideOffset(meta[node]) == 0,
                           "registerSubtree: leaf has a wide-block offset");
            continue;
        }

        FRONTIER_CHECK(metaWideOffset(meta[node]) == expectedWide,
                       "registerSubtree: non-canonical wide-block offset");
        const uint32_t blocks = (children + kWide - 1) / kWide;
        FRONTIER_CHECK(expectedWide <= header->wideCount &&
                           blocks <= header->wideCount - expectedWide,
                       "registerSubtree: wide-block range escapes array");

        uint32_t child = node + 1;
        for (uint32_t blockIndex = 0; blockIndex < blocks; ++blockIndex)
        {
            const WideBlock& block = wide[expectedWide + blockIndex];
            const uint32_t first = blockIndex * kWide;
            const uint32_t lanes =
                std::min(kWide, children - first);
            uint32_t expectedValid = 0;
            uint32_t expectedLeaf = 0;

            for (uint32_t lane = 0; lane < kWide; ++lane)
            {
                if (lane < lanes)
                {
                    FRONTIER_CHECK(child < nodeCount &&
                                       parent[child] == node,
                                   "registerSubtree: invalid preorder topology");
                    FRONTIER_CHECK(block.child[lane] == child,
                                   "registerSubtree: wide child index mismatch");
                    FRONTIER_CHECK(sameBounds(block.bounds.lane(lane),
                                              bbox[child]) &&
                                       block.error.v[lane] == error[child],
                                   "registerSubtree: wide child data mismatch");
                    expectedValid |= 1u << lane;
                    if (metaChildCount(meta[child]) == 0 &&
                        !metaIsMountable(meta[child]))
                        expectedLeaf |= 1u << lane;
                    child += subtreeSize[child];
                }
                else
                {
                    FRONTIER_CHECK(block.child[lane] == kInvalidIndex &&
                                       canonicalEmptyBounds(
                                           block.bounds.lane(lane)) &&
                                       block.error.v[lane] == 0.0f,
                                   "registerSubtree: non-canonical unused lane");
                }
            }

            FRONTIER_CHECK(
                blockMask[expectedWide + blockIndex] ==
                    (expectedValid | (expectedLeaf << kBlockLeafShift)),
                "registerSubtree: invalid wide-block lane mask");
        }

        FRONTIER_CHECK(child == node + subtreeSize[node],
                       "registerSubtree: child extents do not cover subtree");
        expectedWide += blocks;
    }
    FRONTIER_CHECK(expectedWide == header->wideCount,
                   "registerSubtree: unused wide blocks");
#endif
}

SubtreeView viewSubtreeBytes(const SubtreeBytes& bytes)
{
    const auto* base = bytes.data();
    const auto* header = reinterpret_cast<const SubtreeHeader*>(base);
    SubtreeView view;
    view.byteSize_ = bytes.size();
    view.packedNodeCount_ = header->nodeCount;
    view.wideCount_ = header->wideCount;
    view.wide_ = reinterpret_cast<const WideBlock*>(base + header->wideOffset);
    view.blockMask_ = reinterpret_cast<const uint32_t*>(base + header->maskOffset);
    view.bbox_ = reinterpret_cast<const AABB*>(base + header->bboxOffset);
    view.payload_ = reinterpret_cast<const UserPayload*>(base + header->payloadOffset);
    view.parent_ = reinterpret_cast<const uint32_t*>(base + header->parentOffset);
    view.subtreeSize_ = reinterpret_cast<const uint32_t*>(
        base + header->subtreeSizeOffset);
    view.meta_ = reinterpret_cast<const uint32_t*>(base + header->metaOffset);
    view.geometricError_ = reinterpret_cast<const float*>(
        base + header->errorOffset);
    return view;
}

} // namespace detail
} // namespace frontier
