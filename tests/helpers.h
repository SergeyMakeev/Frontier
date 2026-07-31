#pragma once
// Shared test utilities: World internals access, a brute-force scalar
// reference implementation of selectCut, and deterministic tree generators.

#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "hlod/builder.h"
#include "hlod/world.h"

namespace hlod {

struct RefResult
{
    std::vector<CutEntry>    cut;
    std::vector<IdealEntry>  ideal;
    std::vector<LoadRequest> requests;
};

// Full access to World internals plus a straightforward recursive scalar
// reference for selectCut (no wide tests, no masks, no epoch stamps, no
// pruning shortcuts beyond the algorithm's own semantics).
// Requires params.hysteresis == 0 (the reference keeps no per-frame state).
//
// The production API is handle-only; the World keeps no payload index. For
// test readability everything here resolves payloads by brute-force scan
// over the attached pages (findByScan) — deliberately slow, tests only.
struct World::TestAccess
{
    static NodeHandle findByScan(World& w, UserPayload payload)
    {
        for (uint32_t s = 0; s < uint32_t(w.slots_.size()); ++s)
        {
            const PageRt& rt = w.slots_[s];
            if (!rt.inUse) continue;
            for (uint32_t i = 1; i < rt.page.nodeCount(); ++i)
                if (rt.page.payload[i] == payload)
                    return NodeHandle{s, i, rt.generation};
        }
        return NodeHandle{};
    }
    static NodeHandle requireByScan(World& w, UserPayload payload)
    {
        const NodeHandle h = findByScan(w, payload);
        if (!h.valid()) throw std::logic_error("TestAccess: unknown payload");
        return h;
    }
    static const Page& pageOf(World& w, UserPayload anyNodeInPage)
    {
        return w.slots_[requireByScan(w, anyNodeInPage).slot].page;
    }
    static uint32_t lastTouched(World& w, UserPayload anyNodeInPage)
    {
        return w.slots_[requireByScan(w, anyNodeInPage).slot].lastTouched;
    }
    static void forceTlasRebuild(World& w) { w.tlasDirty_ = true; }
    static size_t pageRtBytes() { return sizeof(PageRt); }

    static std::vector<std::pair<uint32_t, uint8_t>> tlasQuery(World& w,
                                                               const CullView& v,
                                                               float minPix)
    {
        std::vector<std::pair<uint32_t, uint8_t>> out;
        w.tlasQuery(v, minPix, out);
        return out;
    }

    // Ancestor chain of a node as payloads, walking parent links inside pages
    // and owner links across pages, up to (excluding) the instance sentinel.
    static std::vector<UserPayload> ancestorIds(World& w, UserPayload payload)
    {
        std::vector<UserPayload> out;
        const NodeHandle h = requireByScan(w, payload);
        NodeRef r{h.slot, h.index};
        while (true)
        {
            const PageRt& rt = w.slots_[r.slot];
            uint32_t i = rt.page.parent[r.index];
            while (i != 0)
            {
                out.push_back(rt.page.payload[i]);
                i = rt.page.parent[i];
            }
            if (!rt.owner.valid()) return out;
            r = rt.owner;
            out.push_back(w.slots_[r.slot].page.payload[r.index]);
        }
    }

    static RefResult referenceCut(World& w, const CullView& view, const CutParams& p)
    {
        RefResult out;
        for (const Instance& inst : w.instances_)
        {
            if (!inst.alive) continue;
            uint8_t mask = kAllPlanes;
            if (testAabb(inst.worldBox, view.frustum, mask) == CullState::Outside)
                continue;
            if (p.minPix > 0.0f)
            {
                const float e = screenError(inst.maxErrWorld, view.k,
                                            distanceToBox(inst.worldBox, view.pos));
                if (e < p.minPix) continue;
            }
            const CullView local = toLocal(view, inst.pos, inst.scale);
            refChildren(w, inst.rootSlot, 0, true, local, p, out);
        }
        return out;
    }

private:
    static void refChildren(World& w, uint32_t slot, uint32_t node, bool alive,
                            const CullView& local, const CutParams& p, RefResult& out)
    {
        const Page& pg = w.slots_[slot].page;
        uint32_t c = node + 1;
        for (uint32_t k = 0; k < pg.childCount(node); ++k)
        {
            refNode(w, slot, c, alive, local, p, out);
            c += pg.subtreeSize[c];
        }
    }

    static void refNode(World& w, uint32_t slot, uint32_t i, bool alive,
                        const CullView& local, const CutParams& p, RefResult& out)
    {
        const PageRt& rt = w.slots_[slot];
        const Page& pg = rt.page;

        uint8_t mask = kAllPlanes;
        if (testAabb(pg.bbox[i], local.frustum, mask) == CullState::Outside) return;

        const float err = screenError(pg.geometricError[i], local.k,
                                      distanceToBox(pg.bbox[i], local.pos));
        const uint32_t cc = pg.childCount(i);
        const bool exp = pg.isExpansion(i);
        const bool wants = (cc > 0 || exp) && err > p.threshold;

        const NodeHandle here{slot, i, rt.generation};
        if (!wants)
        {
            if (alive) out.cut.push_back({pg.payload[i], err});
            out.ideal.push_back({pg.payload[i], here, err, IdealTag::Direct});
            return;
        }

        const uint32_t childSlot =
            (exp && !rt.expSlot.empty()) ? rt.expSlot[i] : kInvalidIndex;
        if (exp && childSlot == kInvalidIndex)
        {
            if (alive) out.cut.push_back({pg.payload[i], err});
            out.ideal.push_back({pg.payload[i], here, err, IdealTag::NeedsExpansion});
            return;
        }

        bool ready;
        if (exp)
        {
            const PageRt& crt = w.slots_[childSlot];
            ready = crt.readyChildren[0] == crt.page.childCount(0);
        }
        else
        {
            ready = rt.readyChildren[i] == cc;
        }

        if (!ready && alive)
        {
            out.cut.push_back({pg.payload[i], err});
            const PageRt& target = exp ? w.slots_[childSlot] : rt;
            const uint32_t ts = exp ? childSlot : slot;
            const uint32_t tn = exp ? 0u : i;
            uint32_t c = tn + 1;
            for (uint32_t k = 0; k < target.page.childCount(tn); ++k)
            {
                if (!target.resident[c])
                    out.requests.push_back({target.page.payload[c],
                                            NodeHandle{ts, c, target.generation}, err});
                c += target.page.subtreeSize[c];
            }
        }

        const bool a2 = alive && ready;
        if (exp)
            refChildren(w, childSlot, 0, a2, local, p, out);
        else
            refChildren(w, slot, i, a2, local, p, out);
    }
};

} // namespace hlod

namespace hlodtest {

using namespace hlod;
using TAX = World::TestAccess;

// Tests key their content by numeric ids; those ids travel as the opaque
// payload. The alias keeps test code reading naturally.
using UserId = UserPayload;

inline std::vector<UserPayload> pageIds(const Page& pg)
{
    std::vector<UserPayload> out;
    for (uint32_t i = 1; i < pg.nodeCount(); ++i) out.push_back(pg.payload[i]);
    return out;
}

// ---------------------------------------------------------------------------
// Payload-keyed wrappers over the handle-only production API. Each one is a
// brute-force scan (TestAccess::findByScan) — fine for tests, never for
// production code, which composes handles from attach results and selectCut
// outputs instead.
// ---------------------------------------------------------------------------
inline NodeHandle handleOf(World& w, UserPayload p) { return TAX::requireByScan(w, p); }

inline void markResident(World& w, UserPayload p) { w.markResident(handleOf(w, p)); }
inline void markNonResident(World& w, UserPayload p) { w.markNonResident(handleOf(w, p)); }
inline bool isResident(World& w, UserPayload p) { return w.isResident(handleOf(w, p)); }
inline bool contains(World& w, UserPayload p) { return TAX::findByScan(w, p).valid(); }

inline PageHandle attachPage(World& w, UserPayload expansion, Page page)
{
    return w.attachPage(handleOf(w, expansion), std::move(page));
}
inline void detachPage(World& w, UserPayload expansion)
{
    w.detachPage(handleOf(w, expansion));
}
inline bool isAttached(World& w, UserPayload expansion)
{
    const NodeHandle h = TAX::findByScan(w, expansion);
    return h.valid() && w.isAttached(h);
}
inline void setNodeBounds(World& w, UserPayload p, const AABB& b)
{
    w.setNodeBounds(handleOf(w, p), b);
}

// Deterministic paged-tree generator. Space is subdivided into slabs along
// the longest axis; geometric error halves per level. Page leaves become
// expansion points while pageLevels > 0, with recipes to build child pages
// on demand (a "virtual planet").
struct TreeGen
{
    uint32_t fanout = 4;
    uint32_t depth  = 2;   // edges from a page root to a page leaf
    uint64_t nextId = 1;

    struct Recipe
    {
        AABB     region;
        float    rootErr;
        uint32_t pageLevels;
    };
    std::unordered_map<UserId, Recipe> recipes;   // expansion id -> child page
    std::vector<UserId> lastIds;                  // ids of the last built page

    static AABB slab(const AABB& r, uint32_t idx, uint32_t count)
    {
        const float4 e = r.mx - r.mn;
        const int axis = (e.x >= e.y && e.x >= e.z) ? 0 : (e.y >= e.z ? 1 : 2);
        float4 mn = r.mn, mx = r.mx;
        const float lo = axis == 0 ? r.mn.x : (axis == 1 ? r.mn.y : r.mn.z);
        const float hi = axis == 0 ? r.mx.x : (axis == 1 ? r.mx.y : r.mx.z);
        const float a = lo + (hi - lo) * float(idx) / float(count);
        const float b = lo + (hi - lo) * float(idx + 1) / float(count);
        (axis == 0 ? mn.x : (axis == 1 ? mn.y : mn.z)) = a;
        (axis == 0 ? mx.x : (axis == 1 ? mx.y : mx.z)) = b;
        return AABB::fromMinMax(mn, mx);
    }

    Page makeRootPage(const AABB& region, float rootErr, uint32_t pageLevels)
    {
        HLodBuilder b;
        lastIds.clear();
        addSubtree(b, 0, true, region, rootErr, 0, pageLevels);
        return b.build();
    }

    Page makeChildPage(UserId expansionId)
    {
        const Recipe r = recipes.at(expansionId);
        HLodBuilder b;
        lastIds.clear();
        for (uint32_t f = 0; f < fanout; ++f)
            addSubtree(b, 0, true, slab(r.region, f, fanout), r.rootErr, 0, r.pageLevels);
        return b.build();
    }

private:
    void addSubtree(HLodBuilder& b, HLodBuilder::NodeId parent, bool isRoot,
                    const AABB& region, float err, uint32_t level, uint32_t pageLevels)
    {
        const UserId id = nextId++;
        lastIds.push_back(id);
        const bool pageLeaf = (level == depth);
        const HLodBuilder::NodeId n =
            isRoot ? b.createRoot(id, err, pageLeaf ? region : AABB::empty())
                   : b.createNode(parent, id, err, pageLeaf ? region : AABB::empty());
        if (pageLeaf)
        {
            if (pageLevels > 0)
            {
                b.markExpansion(n);
                recipes[id] = {region, err * 0.5f, pageLevels - 1};
            }
            return;
        }
        for (uint32_t f = 0; f < fanout; ++f)
            addSubtree(b, n, false, slab(region, f, fanout), err * 0.5f,
                       level + 1, pageLevels);
    }
};

inline void markAllResident(World& w, const std::vector<UserPayload>& ids)
{
    for (UserPayload id : ids) markResident(w, id);
}

// Fast path for benchmarks: mark a whole freshly attached page resident by
// composing handles from the PageHandle — no scans.
inline void markAllResident(World& w, PageHandle page, uint32_t nodeCount)
{
    for (uint32_t i = 1; i < nodeCount; ++i) w.markResident(nodeAt(page, i));
}

inline AABB unitRegion(float halfSize = 100.0f)
{
    return AABB::fromMinMax(float4::vec(-halfSize, -halfSize, -halfSize),
                            float4::vec(halfSize, halfSize, halfSize));
}

} // namespace hlodtest
