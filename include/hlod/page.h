#pragma once
// Immutable flat page: preorder SoA arrays + BVH8-style wide child blocks.
// Produced by HLodBuilder::build(); see hlod_design.md §2.

#include <cstdint>
#include <vector>

#include "math.h"

namespace hlod {

// Opaque 64-bit user payload carried per node: an id, a pointer, an index —
// the World never interprets it, only echoes it back in selectCut outputs.
// Duplicates are the caller's business.
using UserPayload = uint64_t;
inline constexpr UserPayload kSentinelPayload = ~0ull;   // page sentinel [0] only
inline constexpr uint32_t    kInvalidIndex    = 0xFFFFFFFFu;

// meta packing: childCount | EXPANSION flag | wide-block offset.
// Fits in one word because pages are bounded; build() enforces the limits.
inline constexpr uint32_t kMetaChildBits  = 9;                          // <= 511 children
inline constexpr uint32_t kMaxChildren    = (1u << kMetaChildBits) - 1;
inline constexpr uint32_t kMetaExpansion  = 1u << kMetaChildBits;
inline constexpr uint32_t kMetaOffsetShift = kMetaChildBits + 1;
inline constexpr uint32_t kMaxWideOffset  = (1u << (32 - kMetaOffsetShift)) - 1;

inline uint32_t metaChildCount(uint32_t m) { return m & kMaxChildren; }
inline bool     metaIsExpansion(uint32_t m) { return (m & kMetaExpansion) != 0; }
inline uint32_t metaWideOffset(uint32_t m) { return m >> kMetaOffsetShift; }

// One wide child block: up to kWide children of one node, SoA-transposed so a
// single SIMD issue tests them all (frustum + distance + error).
struct WideBlock
{
    WideBounds bounds;              // children's bounds, one child per lane
    float8     error;               // children's geometric error
    uint32_t   child[kWide];        // children's local indices
    uint32_t   validMask = 0;       // bit per used lane
    uint32_t   leafMask  = 0;       // lanes whose child is a plain leaf
                                    // (no local children, not an expansion):
                                    // the walk emits them straight from the
                                    // wide test without visiting them
    uint32_t   _pad[2] = {};        // keep 32-byte cadence
};

struct Page
{
    // ---- hot: read for every visited node ---------------------------------
    std::vector<uint32_t> parent;       // (A) parent[i] < i; parent[0] == 0
    std::vector<uint32_t> subtreeSize;  // (B) subtree == [i, i + subtreeSize[i])
    std::vector<uint32_t> meta;         // packed, see above

    // ---- hot: read only when a node refines --------------------------------
    std::vector<WideBlock> wide;

    // ---- cold: emission, refit, attach — never in the inner loop -----------
    std::vector<UserPayload> payload;      // opaque caller data; [0] == kSentinelPayload
    std::vector<AABB>   bbox;              // (C) source of truth for refit
    std::vector<float>  geometricError;    // (D) source of truth; wide lanes
                                           //     are the hot copies

    uint32_t nodeCount() const { return uint32_t(parent.size()); }

    uint32_t childCount(uint32_t i) const { return metaChildCount(meta[i]); }
    bool     isExpansion(uint32_t i) const { return metaIsExpansion(meta[i]); }
    uint32_t wideOffset(uint32_t i) const { return metaWideOffset(meta[i]); }
    uint32_t wideBlockCount(uint32_t i) const
    {
        return (childCount(i) + kWide - 1) / kWide;
    }

    // Approximate memory footprint; used by tests/GC budgeting.
    size_t byteSize() const
    {
        return parent.size() * (sizeof(uint32_t) * 3 + sizeof(UserPayload) +
                                sizeof(AABB) + sizeof(float)) +
               wide.size() * sizeof(WideBlock);
    }
};

} // namespace hlod
