#pragma once

#include <cstddef>
#include <cstdint>

#include "../node.h"

namespace frontier::detail {

inline constexpr uint32_t kMetaChildBits = 9;
inline constexpr uint32_t kMaxChildren = (1u << kMetaChildBits) - 1;
inline constexpr uint32_t kMetaExpansion = 1u << kMetaChildBits;
inline constexpr uint32_t kMetaOffsetShift = kMetaChildBits + 1;
inline constexpr uint32_t kMaxWideOffset =
    (1u << (32 - kMetaOffsetShift)) - 1;

inline uint32_t metaChildCount(uint32_t m) { return m & kMaxChildren; }
inline bool metaIsExpansion(uint32_t m) { return (m & kMetaExpansion) != 0; }
inline uint32_t metaWideOffset(uint32_t m) { return m >> kMetaOffsetShift; }

struct WideBlock
{
    WideBounds bounds;
    float8 error;
    uint32_t child[kWide];
};
static_assert(sizeof(WideBlock) == 256,
              "WideBlock must stay exactly four cache lines");

inline constexpr uint32_t kBlockLeafShift = 8;
inline uint32_t blockValidLanes(uint32_t m) { return m & 0xFFu; }
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

// Sparse authored data for one node whose refinement lives in another
// subtree. dependency indexes the definition's deduplicated key table.
struct SubtreeExpansion
{
    float4 pos = float4::point(0.0f, 0.0f, 0.0f);
    float scale = 1.0f;
    uint32_t nodeIndex = kInvalidIndex;
    uint32_t dependency = kInvalidIndex;

    Transform transform() const { return {pos, scale}; }
};
static_assert(sizeof(SubtreeExpansion) == 32,
              "subtree expansion layout changed");

inline constexpr uint32_t kSubtreeMagic = 0x42545346u; // 'FSTB'
inline constexpr uint16_t kSubtreeVersion = 1;
inline constexpr size_t kSubtreeAlign = 64;

// The immutable in-memory layout is also the serialized format. Unlike the
// This header covers the complete reusable definition,
// including dependency and expansion metadata.
struct SubtreeHeader
{
    uint32_t magic;
    uint16_t version;
    uint16_t headerBytes;
    uint32_t totalBytes;
    uint32_t nodeCount;       // includes the implicit-parent sentinel at zero
    uint32_t wideCount;
    uint32_t dependencyCount;
    uint32_t expansionCount;
    uint32_t reserved0;
    uint64_t key;
    uint32_t wideOffset;
    uint32_t maskOffset;
    uint32_t bboxOffset;
    uint32_t payloadOffset;
    uint32_t parentOffset;
    uint32_t subtreeSizeOffset;
    uint32_t metaOffset;
    uint32_t errorOffset;
    uint32_t dependencyOffset;
    uint32_t expansionOffset;
    uint32_t reserved[12];
};
static_assert(sizeof(SubtreeHeader) == 128,
              "SubtreeHeader must stay two cache lines");

size_t subtreeBlobBytes(uint32_t nodeCount, uint32_t wideCount,
                        uint32_t dependencyCount, uint32_t expansionCount);

} // namespace frontier::detail
