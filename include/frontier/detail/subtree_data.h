#pragma once

#include <cstddef>
#include <cstdint>

#include "../node.h"
#include "../subtree.h"

namespace frontier::detail {

inline constexpr uint32_t kMetaChildBits = 9;
inline constexpr uint32_t kMaxChildren = (1u << kMetaChildBits) - 1;
inline constexpr uint32_t kMaxSubtreeNodes = 1u << 20;
inline constexpr uint32_t kMetaMountable = 1u << kMetaChildBits;
inline constexpr uint32_t kMetaOffsetShift = kMetaChildBits + 1;
inline constexpr uint32_t kMaxWideOffset =
    (1u << (32 - kMetaOffsetShift)) - 1;

inline uint32_t metaChildCount(uint32_t m) { return m & kMaxChildren; }
inline bool metaIsMountable(uint32_t m) { return (m & kMetaMountable) != 0; }
inline uint32_t metaWideOffset(uint32_t m) { return m >> kMetaOffsetShift; }

struct WideBlock
{
    WideBounds bounds;
    float8 error;
    uint32_t child[kWide];
};
static_assert(sizeof(WideBlock) == kWide * 32,
              "WideBlock must stay exactly 32 bytes per lane");

inline constexpr uint32_t kBlockLaneMask = (1u << kWide) - 1u;
inline constexpr uint32_t kBlockLeafShift = kWide;
inline uint32_t blockValidLanes(uint32_t m) { return m & kBlockLaneMask; }
inline uint32_t blockLeafLanes(uint32_t m) { return m >> kBlockLeafShift; }

struct WideBoundsRef
{
    const std::byte* base = nullptr;
    uint32_t stride = 0;

    const WideBounds& operator[](uint32_t i) const
    {
        return *reinterpret_cast<const WideBounds*>(base + size_t(i) * stride);
    }
    bool valid() const { return base != nullptr; }

    static WideBoundsRef interleaved(const WideBlock* blocks)
    {
        static_assert(offsetof(WideBlock, bounds) == 0);
        return {reinterpret_cast<const std::byte*>(blocks),
                uint32_t(sizeof(WideBlock))};
    }

    static WideBoundsRef packed(const WideBounds* bounds)
    {
        return {reinterpret_cast<const std::byte*>(bounds),
                uint32_t(sizeof(WideBounds))};
    }
};

struct MutWideBoundsRef
{
    std::byte* base = nullptr;
    uint32_t stride = 0;

    WideBounds& operator[](uint32_t i) const
    {
        return *reinterpret_cast<WideBounds*>(base + size_t(i) * stride);
    }

    static MutWideBoundsRef interleaved(WideBlock* blocks)
    {
        return {reinterpret_cast<std::byte*>(blocks),
                uint32_t(sizeof(WideBlock))};
    }

    static MutWideBoundsRef packed(WideBounds* bounds)
    {
        return {reinterpret_cast<std::byte*>(bounds),
                uint32_t(sizeof(WideBounds))};
    }
};

inline constexpr uint32_t kSubtreeMagic = 0x42545346u; // 'FSTB'
inline constexpr uint16_t kSubtreeVersion = 5;
inline constexpr size_t kSubtreeAlign = kSubtreeByteAlignment;

// The immutable in-memory layout is also the serialized format. It contains
// only definition-local traversal data; mount targets and transforms are
// runtime assembly state.
struct SubtreeHeader
{
    uint32_t magic;
    uint16_t version;
    uint16_t headerBytes;
    uint32_t totalBytes;
    uint32_t nodeCount;       // includes the implicit-parent sentinel at zero
    uint32_t wideCount;
    uint32_t wideOffset;
    uint32_t maskOffset;
    uint32_t bboxOffset;
    uint32_t payloadOffset;
    uint32_t parentOffset;
    uint32_t subtreeSizeOffset;
    uint32_t metaOffset;
    uint32_t errorOffset;
    uint32_t branchingFactor;
    uint64_t invalidPayloadWord;
    uint32_t payloadBytes;
    uint32_t reserved[15];
};
static_assert(sizeof(SubtreeHeader) == 128,
              "SubtreeHeader must stay two cache lines");

struct SubtreeLayout
{
    uint32_t wide = 0;
    uint32_t mask = 0;
    uint32_t bbox = 0;
    uint32_t payload = 0;
    uint32_t parent = 0;
    uint32_t subtreeSize = 0;
    uint32_t meta = 0;
    uint32_t error = 0;
    uint32_t totalBytes = 0;
};

// Internal zero-copy interpretation of registered bytes. Moving SubtreeBytes
// moves only its allocation owner, so these pointers remain stable.
struct SubtreeView
{
    const uint32_t* parent_ = nullptr;
    const uint32_t* subtreeSize_ = nullptr;
    const uint32_t* meta_ = nullptr;
    const WideBlock* wide_ = nullptr;
    const uint32_t* blockMask_ = nullptr;
    const PayloadWord* payload_ = nullptr;
    const AABB* bbox_ = nullptr;
    const float* geometricError_ = nullptr;
    uint32_t packedNodeCount_ = 0;
    uint32_t wideCount_ = 0;
    size_t byteSize_ = 0;

    bool valid() const { return parent_ != nullptr; }
    uint32_t nodeCount() const
    {
        return packedNodeCount_ ? packedNodeCount_ - 1 : 0;
    }
    uint32_t packedNodeCount() const { return packedNodeCount_; }
    uint32_t wideCount() const { return wideCount_; }
    AABB bounds() const { return valid() ? bbox_[0] : AABB::empty(); }
    uint32_t childCount(uint32_t i) const { return metaChildCount(meta_[i]); }
    bool isMountable(uint32_t i) const { return metaIsMountable(meta_[i]); }
    uint32_t wideOffset(uint32_t i) const { return metaWideOffset(meta_[i]); }
    uint32_t wideBlockCount(uint32_t i) const
    {
        return (childCount(i) + kWide - 1) / kWide;
    }
    uint32_t validLanes(uint32_t block) const
    {
        return blockValidLanes(blockMask_[block]);
    }
    uint32_t leafLanes(uint32_t block) const
    {
        return blockLeafLanes(blockMask_[block]);
    }
    WideBoundsRef wideBounds() const
    {
        return WideBoundsRef::interleaved(wide_);
    }
};

SubtreeLayout subtreeLayout(uint32_t nodeCount, uint32_t wideCount);
void validateSubtreeBytes(const SubtreeBytes& bytes);
SubtreeView viewSubtreeBytes(const SubtreeBytes& bytes);

} // namespace frontier::detail
