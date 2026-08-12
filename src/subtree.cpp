#include "frontier/subtree.h"

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
    FRONTIER_CHECK(payload[0] == kSentinelPayload,
                   "registerSubtree: missing implicit-parent sentinel");
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
