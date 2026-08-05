#include "hlod/builder.h"

#include <cstring>

namespace hlod {

HLodBuilder::NodeId HLodBuilder::createRoot(UserPayload payload, float geometricError,
                                            const AABB& bbox)
{
    HLOD_CHECK(!built_, "HLodBuilder: builder already consumed");
    BuildNode n;
    n.bbox = bbox;
    n.geometricError = geometricError;
    n.parent = kInvalidIndex;
    n.payload = payload;
    nodes_.push_back(n);
    roots_.push_back(uint32_t(nodes_.size() - 1));
    return NodeId(nodes_.size() - 1);
}

HLodBuilder::NodeId HLodBuilder::createNode(NodeId parent, UserPayload payload,
                                            float geometricError, const AABB& bbox)
{
    HLOD_CHECK(!built_, "HLodBuilder: builder already consumed");
    HLOD_CHECK(parent < nodes_.size(), "HLodBuilder: invalid parent");
    HLOD_CHECK(!nodes_[parent].expansion, "HLodBuilder: expansion point must stay a leaf");
    BuildNode n;
    n.bbox = bbox;
    n.geometricError = geometricError;
    n.parent = parent;
    n.payload = payload;
    nodes_.push_back(n);
    const NodeId me = NodeId(nodes_.size() - 1);
    nodes_[parent].children.push_back(me);
    HLOD_CHECK(nodes_[parent].children.size() <= kMaxChildren,
               "HLodBuilder: fanout exceeds kMaxChildren");
    return me;
}

void HLodBuilder::markExpansion(NodeId node)
{
    HLOD_CHECK(!built_, "HLodBuilder: builder already consumed");
    HLOD_CHECK(node < nodes_.size(), "HLodBuilder: invalid node");
    HLOD_CHECK(nodes_[node].children.empty(),
               "HLodBuilder: expansion point must stay a leaf");
    nodes_[node].expansion = true;
}

Page HLodBuilder::build(const HlodContext& ctx)
{
    HLOD_CHECK(!built_, "HLodBuilder: builder already consumed");
    built_ = true;
    HLOD_CHECK(!roots_.empty(), "HLodBuilder: page has no roots");
    HLOD_CHECK(roots_.size() <= kMaxChildren, "HLodBuilder: too many page roots");

    const uint32_t total = uint32_t(nodes_.size()) + 1;   // +1 sentinel

    // The passes below work in plain scratch arrays; the blob is packed once
    // at the end. Authoring is not perf-sensitive and the final memcpy is a
    // rounding error next to the DFS passes.
    std::vector<uint32_t>    parent;
    std::vector<uint32_t>    subtreeSize;
    std::vector<uint32_t>    meta;
    std::vector<UserPayload> payload;
    std::vector<AABB>        bbox;
    std::vector<float>       geometricError;
    std::vector<WideBlock>   wide;
    std::vector<uint32_t>    blockMask;   // parallel to wide, one word per block

    parent.reserve(total);
    subtreeSize.reserve(total);
    meta.reserve(total);
    payload.reserve(total);
    bbox.reserve(total);
    geometricError.reserve(total);

    auto emit = [&](uint32_t par, uint32_t childCount, bool expansion,
                    UserPayload pay, const AABB& box, float ge) -> uint32_t
    {
        parent.push_back(par);
        subtreeSize.push_back(1);
        meta.push_back(childCount | (expansion ? kMetaExpansion : 0u));
        payload.push_back(pay);
        bbox.push_back(box);
        geometricError.push_back(ge);
        return uint32_t(parent.size() - 1);
    };

    // ---- Sentinel at index 0: stand-in for this page's owner ----------------
    // FLT_MAX error means the roots are never clamped here. A page attached
    // under an expansion point is clamped at runtime by the owner's error,
    // which the World carries as a per-attachment scalar rather than by
    // rewriting the page (that is what lets one page back many attachments).
    emit(0, uint32_t(roots_.size()), false, kSentinelPayload, AABB::empty(), FLT_MAX);

    // ---- Pass A: preorder DFS emission — establishes (A) and (B) ------------
    std::vector<uint32_t> remap(nodes_.size(), kInvalidIndex);
    std::vector<uint32_t> stack;
    for (auto it = roots_.rbegin(); it != roots_.rend(); ++it) stack.push_back(*it);
    while (!stack.empty())
    {
        const uint32_t b = stack.back();
        stack.pop_back();
        const BuildNode& n = nodes_[b];
        const uint32_t par = n.parent == kInvalidIndex ? 0 : remap[n.parent];
        remap[b] = emit(par, uint32_t(n.children.size()), n.expansion,
                        n.payload, n.bbox, n.geometricError);
        for (auto c = n.children.rbegin(); c != n.children.rend(); ++c)
            stack.push_back(*c);
    }
    HLOD_CHECK(parent.size() == total, "HLodBuilder: internal emission count mismatch");

    // ---- Pass B: bottom-up fold — subtree sizes and bounds, one reverse sweep
    // Unioning children into the parent ESTABLISHES (C); an author-supplied
    // bbox is treated as a lower bound.
    for (uint32_t i = total - 1; i >= 1; --i)
    {
        subtreeSize[parent[i]] += subtreeSize[i];
        bbox[parent[i]].expand(bbox[i]);
    }
    for (uint32_t i = 1; i < total; ++i)
        HLOD_CHECK(!bbox[i].isEmpty(), "HLodBuilder: leaf node without bounds");

    // ---- Pass C: enforce monotone error (D), forward sweep ------------------
    for (uint32_t i = 1; i < total; ++i)
    {
        const float pe = geometricError[parent[i]];
        if (geometricError[i] > pe) geometricError[i] = pe;
    }

    // ---- Pass D: emit wide child blocks --------------------------------------
    std::vector<uint32_t> kids;
    for (uint32_t i = 0; i < total; ++i)
    {
        const uint32_t cc = metaChildCount(meta[i]);
        if (cc == 0) continue;

        kids.clear();
        uint32_t c = i + 1;
        for (uint32_t k = 0; k < cc; ++k)
        {
            kids.push_back(c);
            c += subtreeSize[c];
        }

        const uint32_t offset = uint32_t(wide.size());
        HLOD_CHECK(offset <= kMaxWideOffset,
                   "HLodBuilder: page too large: wide offset overflow");
        meta[i] |= offset << kMetaOffsetShift;

        for (uint32_t base = 0; base < cc; base += kWide)
        {
            WideBlock blk;
            blk.bounds = WideBounds::allEmpty();
            blk.error  = float8::splat(0.0f);
            for (uint32_t l = 0; l < kWide; ++l) blk.child[l] = kInvalidIndex;
            uint32_t valid = 0, leaf = 0;
            for (uint32_t l = 0; l < kWide && base + l < cc; ++l)
            {
                const uint32_t ci = kids[base + l];
                blk.bounds.setLane(l, bbox[ci]);
                blk.error.v[l] = geometricError[ci];
                blk.child[l]   = ci;
                valid |= 1u << l;
                if (metaChildCount(meta[ci]) == 0 && !metaIsExpansion(meta[ci]))
                    leaf |= 1u << l;
            }
            wide.push_back(blk);
            blockMask.push_back(valid | (leaf << kBlockLeafShift));
        }
    }

    // ---- Pass E: verify the contract -----------------------------------------
    // Payloads are opaque user data: no uniqueness or reserved-value checks.
    for (uint32_t i = 1; i < total; ++i)
    {
        HLOD_CHECK(parent[i] < i, "HLodBuilder: (A) violated");                    // (A)
        HLOD_CHECK(i + subtreeSize[i] <= total, "HLodBuilder: (B) violated");      // (B)
        HLOD_CHECK(metaChildCount(meta[i]) == 0 || parent[i + 1] == i,
                   "HLodBuilder: (B) first child not adjacent");
        HLOD_CHECK(bbox[parent[i]].contains(bbox[i]), "HLodBuilder: (C) violated"); // (C)
        HLOD_CHECK(geometricError[i] <= geometricError[parent[i]],
                   "HLodBuilder: (D) violated");                                    // (D)
        HLOD_CHECK(metaChildCount(meta[i]) == 0 || !metaIsExpansion(meta[i]),
                   "HLodBuilder: local XOR paged children");
    }

    // ---- Pass F: pack the blob -------------------------------------------------
    const uint32_t wideCount = uint32_t(wide.size());
    const size_t   bytes = pageBlobBytes(total, wideCount);

    void* blob = ctx.alloc(bytes, kPageAlign, ctx.user);
    HLOD_CHECK(blob != nullptr, "HLodBuilder: page allocation failed");
    std::memset(blob, 0, bytes);

    auto* base = static_cast<std::byte*>(blob);
    auto* h = reinterpret_cast<PageHeader*>(base);
    h->magic       = kPageMagic;
    h->version     = kPageVersion;
    h->headerBytes = uint16_t(sizeof(PageHeader));
    h->totalBytes  = uint32_t(bytes);
    h->nodeCount   = total;
    h->wideCount   = wideCount;

    // Mirrors the layout in page.cpp; PageView::fromBytes recomputes it and
    // rejects the blob if the two ever disagree.
    size_t o = sizeof(PageHeader);
    auto place = [&](uint32_t& outOffset, size_t align, size_t count, size_t stride,
                     const void* src)
    {
        o = (o + align - 1) & ~(align - 1);
        outOffset = uint32_t(o);
        if (count) std::memcpy(base + o, src, count * stride);
        o += count * stride;
    };
    place(h->wideOffset, alignof(WideBlock), wideCount, sizeof(WideBlock), wide.data());
    place(h->maskOffset, alignof(uint32_t), wideCount, sizeof(uint32_t),
          blockMask.data());
    place(h->bboxOffset, alignof(AABB), total, sizeof(AABB), bbox.data());
    place(h->payloadOffset, alignof(UserPayload), total, sizeof(UserPayload),
          payload.data());
    place(h->parentOffset, alignof(uint32_t), total, sizeof(uint32_t), parent.data());
    place(h->subtreeOffset, alignof(uint32_t), total, sizeof(uint32_t),
          subtreeSize.data());
    place(h->metaOffset, alignof(uint32_t), total, sizeof(uint32_t), meta.data());
    place(h->errorOffset, alignof(float), total, sizeof(float), geometricError.data());

    return Page::adopt(blob, bytes, ctx);
}

} // namespace hlod
