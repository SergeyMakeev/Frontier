#include "frontier/page.h"

#include <cstring>

namespace frontier {

namespace {

constexpr size_t alignUp(size_t v, size_t a) { return (v + a - 1) & ~(a - 1); }

// The one place the blob layout is defined. The builder uses it to place the
// arrays; PageView::fromBytes recomputes it and compares against the header,
// which makes "the offsets agree with the shape" a single check rather than
// seven independent range tests.
struct Layout
{
    uint32_t wide, mask, bbox, payload, parent, subtree, meta, error;
    uint32_t totalBytes;
};

Layout computeLayout(uint32_t nodeCount, uint32_t wideCount)
{
    Layout L{};
    size_t o = sizeof(PageHeader);

    // The wide array leads the blob so that it inherits the blob's 64-byte
    // alignment: sizeof(PageHeader) is 64 and sizeof(WideBlock) is 256, so
    // every block lands on a cache-line boundary. Do not put anything between
    // the header and the blocks.
    o = alignUp(o, alignof(WideBlock));
    L.wide = uint32_t(o);
    o += size_t(wideCount) * sizeof(WideBlock);

    o = alignUp(o, alignof(uint32_t));
    L.mask = uint32_t(o);
    o += size_t(wideCount) * sizeof(uint32_t);

    o = alignUp(o, alignof(AABB));
    L.bbox = uint32_t(o);
    o += size_t(nodeCount) * sizeof(AABB);

    o = alignUp(o, alignof(UserPayload));
    L.payload = uint32_t(o);
    o += size_t(nodeCount) * sizeof(UserPayload);

    o = alignUp(o, alignof(uint32_t));
    L.parent = uint32_t(o);
    o += size_t(nodeCount) * sizeof(uint32_t);

    L.subtree = uint32_t(o);
    o += size_t(nodeCount) * sizeof(uint32_t);

    L.meta = uint32_t(o);
    o += size_t(nodeCount) * sizeof(uint32_t);

    L.error = uint32_t(o);
    o += size_t(nodeCount) * sizeof(float);

    L.totalBytes = uint32_t(alignUp(o, kPageAlign));
    return L;
}

} // namespace

size_t pageBlobBytes(uint32_t nodeCount, uint32_t wideCount)
{
    return computeLayout(nodeCount, wideCount).totalBytes;
}

// ---------------------------------------------------------------------------
// PageView
// ---------------------------------------------------------------------------

void PageView::bind(const void* blob, size_t byteSize)
{
    const auto* base = static_cast<const std::byte*>(blob);
    const auto* h = reinterpret_cast<const PageHeader*>(base);

    base_      = base;
    byteSize_  = byteSize;
    nodeCount_ = h->nodeCount;
    wideCount_ = h->wideCount;

    parent         = reinterpret_cast<const uint32_t*>(base + h->parentOffset);
    subtreeSize    = reinterpret_cast<const uint32_t*>(base + h->subtreeOffset);
    meta           = reinterpret_cast<const uint32_t*>(base + h->metaOffset);
    wide           = reinterpret_cast<const WideBlock*>(base + h->wideOffset);
    blockMask      = reinterpret_cast<const uint32_t*>(base + h->maskOffset);
    payload        = reinterpret_cast<const UserPayload*>(base + h->payloadOffset);
    bbox           = reinterpret_cast<const AABB*>(base + h->bboxOffset);
    geometricError = reinterpret_cast<const float*>(base + h->errorOffset);
}

PageView PageView::fromValidatedBytes(const void* blob)
{
    PageView v;
    v.bind(blob, reinterpret_cast<const PageHeader*>(blob)->totalBytes);
    return v;
}

PageView PageView::fromBytes(const void* blob, size_t bytes)
{
    FRONTIER_CHECK(blob != nullptr, "PageView::fromBytes: null blob");
    FRONTIER_CHECK(bytes >= sizeof(PageHeader), "PageView::fromBytes: truncated header");
    FRONTIER_CHECK((reinterpret_cast<uintptr_t>(blob) & (alignof(WideBlock) - 1)) == 0,
               "PageView::fromBytes: blob is not sufficiently aligned");

    const auto* h = static_cast<const PageHeader*>(blob);
    FRONTIER_CHECK(h->magic == kPageMagic,
               "PageView::fromBytes: bad magic (not a Frontier page, or byte-swapped)");
    FRONTIER_CHECK(h->version == kPageVersion, "PageView::fromBytes: page version mismatch");
    FRONTIER_CHECK(h->headerBytes == sizeof(PageHeader),
               "PageView::fromBytes: header size mismatch");
    FRONTIER_CHECK(h->nodeCount >= 1, "PageView::fromBytes: empty page");
    FRONTIER_CHECK(h->totalBytes <= bytes, "PageView::fromBytes: truncated blob");

    // Every offset is a function of the shape, so one comparison against the
    // recomputed layout covers all of them.
    const Layout L = computeLayout(h->nodeCount, h->wideCount);
    FRONTIER_CHECK(L.totalBytes == h->totalBytes && L.wide == h->wideOffset &&
                   L.mask == h->maskOffset && L.bbox == h->bboxOffset &&
                   L.payload == h->payloadOffset && L.parent == h->parentOffset &&
                   L.subtree == h->subtreeOffset && L.meta == h->metaOffset &&
                   L.error == h->errorOffset,
               "PageView::fromBytes: offsets inconsistent with page shape");

    PageView v;
    v.bind(blob, h->totalBytes);
    FRONTIER_CHECK(v.payload[0] == kSentinelPayload,
               "PageView::fromBytes: missing page sentinel");
    return v;
}

// ---------------------------------------------------------------------------
// Page
// ---------------------------------------------------------------------------

Page Page::adopt(void* blob, size_t bytes, const FrontierContext& ctx)
{
    Page p;
    static_cast<PageView&>(p) = PageView::fromBytes(blob, bytes);
    p.ctx_ = &ctx;
    return p;
}

Page Page::borrow(PageView view)
{
    Page p;
    static_cast<PageView&>(p) = view;
    return p;
}

Page Page::fromBytes(const void* blob, size_t bytes, const FrontierContext& ctx)
{
    const PageView v = PageView::fromBytes(blob, bytes);
    void* copy = ctx.alloc(v.byteSize(), kPageAlign, ctx.user);
    FRONTIER_CHECK(copy != nullptr, "Page::fromBytes: allocation failed");
    std::memcpy(copy, v.data(), v.byteSize());
    return adopt(copy, v.byteSize(), ctx);
}

Page Page::clone(const FrontierContext& ctx) const
{
    if (!valid()) return Page{};
    void* copy = ctx.alloc(byteSize_, kPageAlign, ctx.user);
    FRONTIER_CHECK(copy != nullptr, "Page::clone: allocation failed");
    std::memcpy(copy, base_, byteSize_);
    Page p;
    static_cast<PageView&>(p) = PageView::fromValidatedBytes(copy);
    p.ctx_ = &ctx;
    return p;
}

void Page::release()
{
    if (base_ && ctx_) ctx_->free(const_cast<std::byte*>(base_), ctx_->user);
    static_cast<PageView&>(*this) = PageView{};
    ctx_ = nullptr;
}

void Page::moveFrom(Page& o) noexcept
{
    static_cast<PageView&>(*this) = static_cast<const PageView&>(o);
    ctx_ = o.ctx_;
    static_cast<PageView&>(o) = PageView{};
    o.ctx_ = nullptr;
}

} // namespace frontier
