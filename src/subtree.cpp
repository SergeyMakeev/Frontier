#include "frontier/subtree.h"

#include <cstring>

namespace frontier {
namespace {

constexpr size_t alignUp(size_t value, size_t alignment)
{
    return (value + alignment - 1) & ~(alignment - 1);
}

struct Layout
{
    uint32_t wide = 0;
    uint32_t mask = 0;
    uint32_t bbox = 0;
    uint32_t payload = 0;
    uint32_t parent = 0;
    uint32_t subtreeSize = 0;
    uint32_t meta = 0;
    uint32_t error = 0;
    uint32_t dependencies = 0;
    uint32_t expansions = 0;
    uint32_t totalBytes = 0;
};

Layout computeLayout(uint32_t nodeCount, uint32_t wideCount,
                     uint32_t dependencyCount, uint32_t expansionCount)
{
    using namespace detail;
    Layout layout;
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

    offset = alignUp(offset, alignof(SubtreeKey));
    layout.dependencies = uint32_t(offset);
    offset += size_t(dependencyCount) * sizeof(SubtreeKey);

    offset = alignUp(offset, alignof(SubtreeExpansion));
    layout.expansions = uint32_t(offset);
    offset += size_t(expansionCount) * sizeof(SubtreeExpansion);

    layout.totalBytes = uint32_t(alignUp(offset, kSubtreeAlign));
    return layout;
}

} // namespace

namespace detail {

size_t subtreeBlobBytes(uint32_t nodeCount, uint32_t wideCount,
                        uint32_t dependencyCount, uint32_t expansionCount)
{
    return computeLayout(nodeCount, wideCount, dependencyCount,
                         expansionCount)
        .totalBytes;
}

} // namespace detail

void Subtree::validate(const void* blob, size_t bytes)
{
    using namespace detail;
    FRONTIER_CHECK(blob != nullptr, "Subtree::fromBytes: null blob");
    FRONTIER_CHECK(bytes >= sizeof(SubtreeHeader),
                   "Subtree::fromBytes: truncated header");
    FRONTIER_CHECK(
        (reinterpret_cast<uintptr_t>(blob) & (kSubtreeAlign - 1)) == 0,
        "Subtree::fromBytes: blob is not sufficiently aligned");

    const auto* header = static_cast<const SubtreeHeader*>(blob);
    FRONTIER_CHECK(header->magic == kSubtreeMagic,
                   "Subtree::fromBytes: bad magic or byte order");
    FRONTIER_CHECK(header->version == kSubtreeVersion,
                   "Subtree::fromBytes: format version mismatch");
    FRONTIER_CHECK(header->headerBytes == sizeof(SubtreeHeader),
                   "Subtree::fromBytes: header size mismatch");
    FRONTIER_CHECK(header->nodeCount > 1,
                   "Subtree::fromBytes: subtree has no renderable nodes");
    FRONTIER_CHECK(header->key != 0,
                   "Subtree::fromBytes: invalid subtree key");
    FRONTIER_CHECK(header->totalBytes <= bytes,
                   "Subtree::fromBytes: truncated blob");

    const Layout layout =
        computeLayout(header->nodeCount, header->wideCount,
                      header->dependencyCount, header->expansionCount);
    FRONTIER_CHECK(
        layout.totalBytes == header->totalBytes &&
            layout.wide == header->wideOffset &&
            layout.mask == header->maskOffset &&
            layout.bbox == header->bboxOffset &&
            layout.payload == header->payloadOffset &&
            layout.parent == header->parentOffset &&
            layout.subtreeSize == header->subtreeSizeOffset &&
            layout.meta == header->metaOffset &&
            layout.error == header->errorOffset &&
            layout.dependencies == header->dependencyOffset &&
            layout.expansions == header->expansionOffset,
        "Subtree::fromBytes: offsets inconsistent with subtree shape");

    const auto* bytesBase = static_cast<const std::byte*>(blob);
    const auto* payload = reinterpret_cast<const UserPayload*>(
        bytesBase + header->payloadOffset);
    FRONTIER_CHECK(payload[0] == kSentinelPayload,
                   "Subtree::fromBytes: missing implicit-parent sentinel");
}

void Subtree::bind(const void* blob, size_t bytes)
{
    using namespace detail;
    const auto* bytesBase = static_cast<const std::byte*>(blob);
    const auto* header = reinterpret_cast<const SubtreeHeader*>(blob);

    base_ = bytesBase;
    byteSize_ = bytes;
    packedNodeCount_ = header->nodeCount;
    wideCount_ = header->wideCount;
    dependencyCount_ = header->dependencyCount;
    expansionCount_ = header->expansionCount;
    key_ = SubtreeKey{header->key};
    wide_ = reinterpret_cast<const WideBlock*>(bytesBase + header->wideOffset);
    blockMask_ = reinterpret_cast<const uint32_t*>(bytesBase + header->maskOffset);
    bbox_ = reinterpret_cast<const AABB*>(bytesBase + header->bboxOffset);
    payload_ = reinterpret_cast<const UserPayload*>(bytesBase + header->payloadOffset);
    parent_ = reinterpret_cast<const uint32_t*>(bytesBase + header->parentOffset);
    subtreeSize_ = reinterpret_cast<const uint32_t*>(
        bytesBase + header->subtreeSizeOffset);
    meta_ = reinterpret_cast<const uint32_t*>(bytesBase + header->metaOffset);
    geometricError_ = reinterpret_cast<const float*>(bytesBase + header->errorOffset);
    dependencies_ = reinterpret_cast<const SubtreeKey*>(
        bytesBase + header->dependencyOffset);
    expansions_ = reinterpret_cast<const SubtreeExpansion*>(
        bytesBase + header->expansionOffset);
}

Subtree Subtree::fromValidatedBytes(const void* blob, size_t bytes,
                                    const FrontierContext* owner)
{
    Subtree subtree;
    subtree.bind(blob, bytes);
    subtree.context_ = owner;
    return subtree;
}

Subtree Subtree::adopt(void* blob, size_t bytes,
                       const FrontierContext& context)
{
    validate(blob, bytes);
    return fromValidatedBytes(blob, bytes, &context);
}

Subtree Subtree::fromBytes(const void* blob, size_t bytes,
                           const FrontierContext& context)
{
    validate(blob, bytes);
    const auto* header = static_cast<const detail::SubtreeHeader*>(blob);
    void* copy = context.alloc(header->totalBytes, detail::kSubtreeAlign,
                               context.user);
    FRONTIER_CHECK(copy != nullptr, "Subtree::fromBytes: allocation failed");
    std::memcpy(copy, blob, header->totalBytes);
    return fromValidatedBytes(copy, header->totalBytes, &context);
}

Subtree Subtree::borrow(const void* blob, size_t bytes)
{
    validate(blob, bytes);
    const auto* header = static_cast<const detail::SubtreeHeader*>(blob);
    return fromValidatedBytes(blob, header->totalBytes, nullptr);
}

Subtree Subtree::clone(const FrontierContext& context) const
{
    if (!valid()) return {};
    void* copy = context.alloc(byteSize_, detail::kSubtreeAlign, context.user);
    FRONTIER_CHECK(copy != nullptr, "Subtree::clone: allocation failed");
    std::memcpy(copy, base_, byteSize_);
    return fromValidatedBytes(copy, byteSize_, &context);
}

void Subtree::release()
{
    if (base_ && context_)
        context_->free(const_cast<std::byte*>(base_), context_->user);

    parent_ = nullptr;
    subtreeSize_ = nullptr;
    meta_ = nullptr;
    wide_ = nullptr;
    blockMask_ = nullptr;
    payload_ = nullptr;
    bbox_ = nullptr;
    geometricError_ = nullptr;
    dependencies_ = nullptr;
    expansions_ = nullptr;
    base_ = nullptr;
    packedNodeCount_ = 0;
    wideCount_ = 0;
    dependencyCount_ = 0;
    expansionCount_ = 0;
    byteSize_ = 0;
    key_ = {};
    context_ = nullptr;
}

void Subtree::moveFrom(Subtree& other) noexcept
{
    if (!other.valid()) return;
    bind(other.base_, other.byteSize_);
    context_ = other.context_;
    other.context_ = nullptr;
    other.release();
}

} // namespace frontier
