#pragma once
// Immutable flat page: preorder SoA arrays + BVH8-style wide child blocks,
// laid out as ONE contiguous blob. Produced by HLodBuilder::build();
// see hlod_design.md §3.
//
// The in-memory layout IS the on-disk format. A streamed page is one read
// into one aligned buffer with no parsing, no fixups and no per-array
// allocation; a memory-mapped page is wrapped by PageView with zero copies.
// That is what makes attachPage() allocation-free and what lets thousands of
// instances share a single asset.
//
//   PageView — borrows a blob somebody else owns (mmap, a bundle file, an
//              asset the World already holds). Trivially copyable, 88 bytes
//              (asserted below, because it is embedded in every PageRt).
//   Page     — owns its blob and frees it through the context it came from.
//              Move-only; copies must be spelled out with clone().

#include <cstddef>
#include <cstdint>

#include "config.h"
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
//
// EXACTLY four cache lines, and the blob keeps the array 64-byte aligned, so
// block b occupies lines [4b, 4b+4) and never straddles a fifth. That is the
// whole reason the two lane masks live in a side array (PageView::blockMask)
// instead of here: with them inline the block was 272 bytes, padded to 288 by
// the 32-byte cadence, which is 4.5 lines -- so every block touched five lines
// whatever its parity, 320 bytes of line traffic to read 272 bytes of data.
// The masks are 4 bytes per block in a dense sequential array, sixteen blocks
// to a line, which is far cheaper than the padding they replaced. The cut walk
// is memory-bound, so lines touched is the number that matters.
struct WideBlock
{
    WideBounds bounds;              // children's bounds, one child per lane
    float8     error;               // children's geometric error
    uint32_t   child[kWide];        // children's local indices
#ifdef HLOD_WIDE_PAD_288
    // A/B scaffold only: pads the block back to the 288 bytes it occupied when
    // the lane masks were inline, with every other line of code unchanged. That
    // makes the stride the single variable, so a measurement across the two
    // builds is the footprint effect and nothing else.
    uint32_t   _pad[4] = {};
#endif
};
#ifndef HLOD_WIDE_PAD_288
static_assert(sizeof(WideBlock) == 256,
              "WideBlock must stay exactly four cache lines; see the note above");
#endif

// Lane masks for one wide block, packed into the side array.
inline constexpr uint32_t kBlockLeafShift = 8;
inline uint32_t blockValidLanes(uint32_t m) { return m & 0xFFu; }
inline uint32_t blockLeafLanes(uint32_t m) { return m >> kBlockLeafShift; }

// Strided handle onto a run of WideBounds.
//
// Bounds are the only per-instance mutable page data. A deformed instance gets
// a private copy of just those (a per-instance overlay) while
// still sharing the page's topology, payloads and errors with every other
// instance. The two cases differ only in where the boxes sit: interleaved
// inside the page's WideBlocks, or packed in the overlay. Carrying the stride
// instead of a "do I have an overlay" flag keeps the inner loop branch-free
// and identical for shared and deformed instances alike.
struct WideBoundsRef
{
    const std::byte* base   = nullptr;
    uint32_t         stride = 0;

    const WideBounds& operator[](uint32_t i) const
    {
        return *reinterpret_cast<const WideBounds*>(base + size_t(i) * stride);
    }
    bool valid() const { return base != nullptr; }

    // The page's own bounds, interleaved in its wide blocks.
    static WideBoundsRef interleaved(const WideBlock* w)
    {
        static_assert(offsetof(WideBlock, bounds) == 0,
                      "WideBlock::bounds must lead the block for the strided alias");
        return {reinterpret_cast<const std::byte*>(w), uint32_t(sizeof(WideBlock))};
    }
    // A packed per-instance overlay.
    static WideBoundsRef packed(const WideBounds* b)
    {
        return {reinterpret_cast<const std::byte*>(b), uint32_t(sizeof(WideBounds))};
    }
};

// Mutable twin of WideBoundsRef, for the refit path. Refit writes boxes into
// whichever layout the instance's overlay uses, so carrying the stride keeps
// one code path for both.
struct MutWideBoundsRef
{
    std::byte* base   = nullptr;
    uint32_t   stride = 0;

    WideBounds& operator[](uint32_t i) const
    {
        return *reinterpret_cast<WideBounds*>(base + size_t(i) * stride);
    }

    static MutWideBoundsRef interleaved(WideBlock* w)
    {
        return {reinterpret_cast<std::byte*>(w), uint32_t(sizeof(WideBlock))};
    }
    static MutWideBoundsRef packed(WideBounds* b)
    {
        return {reinterpret_cast<std::byte*>(b), uint32_t(sizeof(WideBounds))};
    }
};

// ---------------------------------------------------------------------------
// Blob format
// ---------------------------------------------------------------------------

inline constexpr uint32_t kPageMagic   = 0x444F4C48u;   // 'HLOD', little-endian
// 2: lane masks moved out of WideBlock into the blockMask side array, taking
//    the block from 288 bytes to 256. Not backward compatible.
inline constexpr uint16_t kPageVersion = 2;

// Blob alignment. 64 keeps the header on its own cache line and every array
// at or above its natural alignment.
inline constexpr size_t kPageAlign = 64;

struct PageHeader
{
    uint32_t magic;         // kPageMagic; also the little-endian marker
    uint16_t version;       // kPageVersion
    uint16_t headerBytes;   // sizeof(PageHeader)
    uint32_t totalBytes;
    uint32_t nodeCount;
    uint32_t wideCount;
    uint32_t wideOffset;
    uint32_t maskOffset;    // wideCount lane-mask words
    uint32_t bboxOffset;
    uint32_t payloadOffset;
    uint32_t parentOffset;
    uint32_t subtreeOffset;
    uint32_t metaOffset;
    uint32_t errorOffset;
    uint32_t reserved[3];
};
static_assert(sizeof(PageHeader) == 64, "PageHeader must stay one cache line");

// Byte size of the blob for a page of the given shape. Used by the builder to
// size its single allocation and by tools to budget.
size_t pageBlobBytes(uint32_t nodeCount, uint32_t wideCount);

// ---------------------------------------------------------------------------
// PageView — a borrowed page
// ---------------------------------------------------------------------------

class PageView
{
public:
    // Hot: read for every visited node.
    const uint32_t* parent      = nullptr;   // (A) parent[i] < i; parent[0] == 0
    const uint32_t* subtreeSize = nullptr;   // (B) subtree == [i, i + subtreeSize[i])
    const uint32_t* meta        = nullptr;   // packed, see above

    // Hot: read only when a node refines. blockMask[b] carries block b's used
    // lanes in its low 8 bits and its plain-leaf lanes in the next 8; the walk
    // wants both on every block test, so they share one word and one load.
    const WideBlock* wide      = nullptr;
    const uint32_t*  blockMask = nullptr;

    // Cold: emission, refit, attach — never in the inner loop.
    const UserPayload* payload        = nullptr;   // opaque; [0] == kSentinelPayload
    const AABB*        bbox           = nullptr;   // (C) source of truth for refit
    const float*       geometricError = nullptr;   // (D) source of truth; wide
                                                   //     lanes are the hot copies

    PageView() = default;

    // Wrap a blob, validating magic, version and every offset/extent against
    // `bytes`. HLOD_FATAL on a malformed or truncated blob — this is the
    // trust boundary for data coming off disk.
    static PageView fromBytes(const void* blob, size_t bytes);

    // Wrap a blob that has already been validated (one the World is holding).
    // No checks; do not point this at untrusted bytes.
    static PageView fromValidatedBytes(const void* blob);

    bool     valid() const { return parent != nullptr; }
    uint32_t nodeCount() const { return nodeCount_; }
    uint32_t wideCount() const { return wideCount_; }

    const void* data() const { return base_; }
    size_t      byteSize() const { return byteSize_; }

    uint32_t childCount(uint32_t i) const { return metaChildCount(meta[i]); }
    bool     isExpansion(uint32_t i) const { return metaIsExpansion(meta[i]); }
    uint32_t wideOffset(uint32_t i) const { return metaWideOffset(meta[i]); }
    uint32_t wideBlockCount(uint32_t i) const
    {
        return (childCount(i) + kWide - 1) / kWide;
    }

    // Lanes of block b that hold a child, and of those the ones that are plain
    // leaves. For cold callers; the walk reads blockMask[b] once and masks it
    // itself rather than loading the word twice.
    uint32_t validLanes(uint32_t b) const { return blockValidLanes(blockMask[b]); }
    uint32_t leafLanes(uint32_t b) const { return blockLeafLanes(blockMask[b]); }

    // The page's own child bounds, for handing to the traversal when the
    // instance has no overlay.
    WideBoundsRef wideBounds() const { return WideBoundsRef::interleaved(wide); }

protected:
    void bind(const void* blob, size_t byteSize);

    const std::byte* base_ = nullptr;
    uint32_t         nodeCount_ = 0;
    uint32_t         wideCount_ = 0;
    size_t           byteSize_ = 0;
};

// Pinned because a PageView is embedded in every PageRt, so its size sets the
// stride of the residency table. The header comment above claimed 64 bytes long
// after it had stopped being true.
static_assert(sizeof(void*) != 8 || sizeof(PageView) == 88,
              "PageView must stay 88 bytes; it is embedded in every PageRt");

// ---------------------------------------------------------------------------
// Page — an owned page
//
// Slicing a Page to a PageView is meaningful and intended: it is exactly
// "borrow this page's bytes".
// ---------------------------------------------------------------------------

class Page : public PageView
{
public:
    Page() = default;
    ~Page() { release(); }

    Page(Page&& o) noexcept { moveFrom(o); }
    Page& operator=(Page&& o) noexcept
    {
        if (this != &o)
        {
            release();
            moveFrom(o);
        }
        return *this;
    }

    // Copies are never implicit: a page is the big object in this library,
    // and silently duplicating it per instance is the mistake this whole
    // design exists to prevent. Say clone() when you mean it.
    Page(const Page&) = delete;
    Page& operator=(const Page&) = delete;

    // Take ownership of a blob that was allocated through `ctx`.
    static Page adopt(void* blob, size_t bytes, const HlodContext& ctx);

    // Validate and copy an external blob (one just read from a file).
    static Page fromBytes(const void* blob, size_t bytes,
                          const HlodContext& ctx = defaultContext());

    Page clone(const HlodContext& ctx = defaultContext()) const;

private:
    void release();
    void moveFrom(Page& o) noexcept;

    const HlodContext* ctx_ = nullptr;
};

} // namespace hlod
