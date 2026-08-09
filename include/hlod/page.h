#pragma once
// Immutable flat page: preorder SoA arrays + BVH8-style wide child blocks,
// laid out as ONE contiguous blob. Produced by HierarchyBuilder::build() or
// the low-level HLodBuilder::build();
// see docs/hlod_design.md §3.
//
// The in-memory layout IS the on-disk format. A streamed page can be read into
// one aligned buffer and validated in place, with no unpacking, fixups, or
// per-array allocation; a memory-mapped page is wrapped by PageView with zero
// copies. Registration and attachment need no page-data transformation, and
// thousands of instances can share a single asset blob.
//
//   PageView — borrows a blob somebody else owns (mmap, a bundle file, an
//              asset the World already holds). Trivially copyable, 88 bytes.
//   Page     — owns its blob and frees it through the context it came from.
//              Move-only; copies must be spelled out with clone().

#include <cstddef>
#include <cstdint>

#include "config.h"
#include "math.h"

namespace hlod {

class World;

// Opaque 64-bit user payload carried per node: an id, a pointer, an index —
// the World never interprets it, only echoes it back in selectCut outputs.
// Duplicates are the caller's business.
using UserPayload = uint64_t;
inline constexpr UserPayload kSentinelPayload = ~0ull;   // page sentinel [0] only
inline constexpr uint32_t    kInvalidIndex    = 0xFFFFFFFFu;

// Stable id assigned to each logical page generated from one hierarchy.
// Page zero is the root page; detail-page ids therefore start at one.
using HierarchyPageId = uint32_t;
inline constexpr HierarchyPageId kInvalidHierarchyPage = kInvalidIndex;

// meta packing: childCount | EXPANSION flag | wide-block offset/detail-page id.
// Nodes with local children use the high bits as their wide-block offset;
// expansion leaves use the same otherwise-idle bits as a generated logical
// detail-page id. Fits in one word because pages are bounded.
inline constexpr uint32_t kMetaChildBits  = 9;                          // <= 511 children
inline constexpr uint32_t kMaxChildren    = (1u << kMetaChildBits) - 1;
static_assert(kMaxChildren <= UINT16_MAX,
              "runtime covered-child count relies on a 16-bit counter");
inline constexpr uint32_t kMetaExpansion  = 1u << kMetaChildBits;
inline constexpr uint32_t kMetaOffsetShift = kMetaChildBits + 1;
inline constexpr uint32_t kMaxWideOffset  = (1u << (32 - kMetaOffsetShift)) - 1;

inline uint32_t metaChildCount(uint32_t m) { return m & kMaxChildren; }
inline bool     metaIsExpansion(uint32_t m) { return (m & kMetaExpansion) != 0; }
inline uint32_t metaWideOffset(uint32_t m) { return m >> kMetaOffsetShift; }
inline HierarchyPageId metaDetailPage(uint32_t m)
{
    const HierarchyPageId encoded = m >> kMetaOffsetShift;
    return encoded == 0 ? kInvalidHierarchyPage : encoded;
}

// One wide child block: up to kWide children of one node, SoA-transposed so a
// single SIMD issue tests them all (frustum + distance + error).
//
// EXACTLY four cache lines, and the blob keeps the array 64-byte aligned, so
// block b occupies lines [4b, 4b+4) and never straddles a fifth. That is the
// two lane masks live in a dense side array (`PageView::blockMask`) so a block
// remains four cache lines without internal padding. Each mask word is 4 bytes,
// placing sixteen consecutive block masks in one 64-byte cache line.
struct WideBlock
{
    WideBounds bounds;              // children's bounds, one child per lane
    float8     error;               // children's geometric error
    uint32_t   child[kWide];        // children's local indices
};
static_assert(sizeof(WideBlock) == 256,
              "WideBlock must stay exactly four cache lines; see the note above");

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
// Readers require an exact format-version match.
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
    // Valid for nodes with local children. Expansion leaves reuse these bits
    // for detailPage(), so callers skip this accessor when childCount(i) == 0.
    uint32_t wideOffset(uint32_t i) const { return metaWideOffset(meta[i]); }
    HierarchyPageId detailPage(uint32_t i) const
    {
        return isExpansion(i) ? metaDetailPage(meta[i])
                              : kInvalidHierarchyPage;
    }
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

// Pinned because every registered asset carries one and accidental growth
// multiplies with asset count.
static_assert(sizeof(void*) != 8 || sizeof(PageView) == 88,
              "PageView must stay 88 bytes");

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
    friend class World;

    // Wrap already-validated external storage without taking ownership. The
    // World uses this so AssetRt needs one Page object for both owned and
    // borrowed assets instead of storing a duplicate PageView beside it.
    static Page borrow(PageView view);

    void release();
    void moveFrom(Page& o) noexcept;

    const HlodContext* ctx_ = nullptr;
};
static_assert(sizeof(void*) != 8 || sizeof(Page) == 96,
              "Page must stay an 88-byte view plus ownership pointer");

} // namespace hlod
