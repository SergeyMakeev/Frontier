#include "hlod/builder.h"

#include <stdexcept>
#include <string>

namespace hlod {

namespace {
[[noreturn]] void fail(const std::string& msg)
{
    throw std::logic_error("HLodBuilder: " + msg);
}
void check(bool cond, const char* msg)
{
    if (!cond) fail(msg);
}
} // namespace

HLodBuilder::NodeId HLodBuilder::createRoot(UserPayload payload, float geometricError,
                                            const AABB& bbox)
{
    check(!built_, "builder already consumed");
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
    check(!built_, "builder already consumed");
    check(parent < nodes_.size(), "invalid parent");
    check(!nodes_[parent].expansion, "expansion point must stay a leaf");
    BuildNode n;
    n.bbox = bbox;
    n.geometricError = geometricError;
    n.parent = parent;
    n.payload = payload;
    nodes_.push_back(n);
    const NodeId me = NodeId(nodes_.size() - 1);
    nodes_[parent].children.push_back(me);
    check(nodes_[parent].children.size() <= kMaxChildren, "fanout exceeds kMaxChildren");
    return me;
}

void HLodBuilder::markExpansion(NodeId node)
{
    check(!built_, "builder already consumed");
    check(node < nodes_.size(), "invalid node");
    check(nodes_[node].children.empty(), "expansion point must stay a leaf");
    nodes_[node].expansion = true;
}

Page HLodBuilder::build()
{
    check(!built_, "builder already consumed");
    built_ = true;
    check(!roots_.empty(), "page has no roots");
    check(roots_.size() <= kMaxChildren, "too many page roots");

    const uint32_t total = uint32_t(nodes_.size()) + 1;   // +1 sentinel

    Page pg;
    pg.parent.reserve(total);
    pg.subtreeSize.reserve(total);
    pg.meta.reserve(total);
    pg.payload.reserve(total);
    pg.bbox.reserve(total);
    pg.geometricError.reserve(total);

    auto emit = [&](uint32_t parent, uint32_t childCount, bool expansion,
                    UserPayload payload, const AABB& bbox, float ge) -> uint32_t
    {
        pg.parent.push_back(parent);
        pg.subtreeSize.push_back(1);
        pg.meta.push_back(childCount | (expansion ? kMetaExpansion : 0u));
        pg.payload.push_back(payload);
        pg.bbox.push_back(bbox);
        pg.geometricError.push_back(ge);
        return uint32_t(pg.parent.size() - 1);
    };

    // ---- Sentinel at index 0: stand-in for this page's owner ----------------
    // FLT_MAX error means the roots are never clamped here; attachPage()
    // overwrites it with the owning expansion node's error.
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
        const uint32_t parent = n.parent == kInvalidIndex ? 0 : remap[n.parent];
        remap[b] = emit(parent, uint32_t(n.children.size()), n.expansion,
                        n.payload, n.bbox, n.geometricError);
        for (auto c = n.children.rbegin(); c != n.children.rend(); ++c)
            stack.push_back(*c);
    }
    check(pg.nodeCount() == total, "internal: emission count mismatch");

    // ---- Pass B: bottom-up fold — subtree sizes and bounds, one reverse sweep
    // Unioning children into the parent ESTABLISHES (C); an author-supplied
    // bbox is treated as a lower bound.
    for (uint32_t i = total - 1; i >= 1; --i)
    {
        pg.subtreeSize[pg.parent[i]] += pg.subtreeSize[i];
        pg.bbox[pg.parent[i]].expand(pg.bbox[i]);
    }
    for (uint32_t i = 1; i < total; ++i)
        check(!pg.bbox[i].isEmpty(), "leaf node without bounds");

    // ---- Pass C: enforce monotone error (D), forward sweep ------------------
    for (uint32_t i = 1; i < total; ++i)
    {
        const float pe = pg.geometricError[pg.parent[i]];
        if (pg.geometricError[i] > pe) pg.geometricError[i] = pe;
    }

    // ---- Pass D: emit wide child blocks --------------------------------------
    std::vector<uint32_t> kids;
    for (uint32_t i = 0; i < total; ++i)
    {
        const uint32_t cc = pg.childCount(i);
        if (cc == 0) continue;

        kids.clear();
        uint32_t c = i + 1;
        for (uint32_t k = 0; k < cc; ++k)
        {
            kids.push_back(c);
            c += pg.subtreeSize[c];
        }

        const uint32_t offset = uint32_t(pg.wide.size());
        check(offset <= kMaxWideOffset, "page too large: wide offset overflow");
        pg.meta[i] |= offset << kMetaOffsetShift;

        for (uint32_t base = 0; base < cc; base += kWide)
        {
            WideBlock blk;
            blk.bounds = WideBounds::allEmpty();
            blk.error  = float8::splat(0.0f);
            for (uint32_t l = 0; l < kWide; ++l) blk.child[l] = kInvalidIndex;
            for (uint32_t l = 0; l < kWide && base + l < cc; ++l)
            {
                const uint32_t ci = kids[base + l];
                blk.bounds.setLane(l, pg.bbox[ci]);
                blk.error.v[l] = pg.geometricError[ci];
                blk.child[l]   = ci;
                blk.validMask |= 1u << l;
                if (pg.childCount(ci) == 0 && !pg.isExpansion(ci))
                    blk.leafMask |= 1u << l;
            }
            pg.wide.push_back(blk);
        }
    }

    // ---- Pass E: verify the contract -----------------------------------------
    // Payloads are opaque user data: no uniqueness or reserved-value checks.
    for (uint32_t i = 1; i < total; ++i)
    {
        check(pg.parent[i] < i, "(A) violated");                              // (A)
        check(i + pg.subtreeSize[i] <= total, "(B) violated");                // (B)
        check(pg.childCount(i) == 0 || pg.parent[i + 1] == i,
              "(B) first child not adjacent");
        check(pg.bbox[pg.parent[i]].contains(pg.bbox[i]), "(C) violated");    // (C)
        check(pg.geometricError[i] <= pg.geometricError[pg.parent[i]],
              "(D) violated");                                                // (D)
        check(pg.childCount(i) == 0 || !pg.isExpansion(i),
              "local XOR paged children");
    }
    return pg;
}

} // namespace hlod
