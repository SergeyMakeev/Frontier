#pragma once
// Shared test utilities: SpatialDatabase internals access, a brute-force scalar
// reference implementation of selectFrontier, and deterministic tree generators.

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "frontier/builder.h"
#include "frontier/spatial_database.h"
#include "deterministic_rng.h"

namespace frontier {

struct RefResult
{
    detail::FrontierBuffers buffers;
    FrontierResultView cut;

    RefResult() { sync(); }
    RefResult(const RefResult& other) : buffers(other.buffers) { sync(); }
    RefResult(RefResult&& other) noexcept : buffers(std::move(other.buffers))
    {
        sync();
        other.sync();
    }
    RefResult& operator=(const RefResult& other)
    {
        buffers = other.buffers;
        sync();
        return *this;
    }
    RefResult& operator=(RefResult&& other) noexcept
    {
        buffers = std::move(other.buffers);
        sync();
        other.sync();
        return *this;
    }

    void sync() { cut = buffers.view(); }
};

// Exercise the uncached public path without carrying a SpatialQuery between calls.
// Most correctness tests want a fresh reference query; tests for reuse own a
// persistent SpatialQuery explicitly.
inline void selectFrontierUncached(SpatialDatabase& world, const Camera& camera,
                              const SelectionParams& params,
                              FrontierResult& outResult)
{
    world.applyUpdates();
    SpatialQuery query;
    query.setReuseEnabled(false);
    query.selectFrontier(world, camera, params, outResult);
}

inline void selectFrontierUncached(SpatialDatabase& world, const Camera& camera,
                              const SelectionParams& params, PageUsageContext& usage,
                              FrontierResult& outResult)
{
    world.applyUpdates();
    SpatialQuery query;
    query.setReuseEnabled(false);
    query.selectFrontier(world, camera, params, usage, outResult);
}

// Full access to SpatialDatabase internals plus a straightforward recursive scalar
// reference for selectFrontier (no wide tests, no masks, no epoch stamps, no
// pruning shortcuts beyond the algorithm's own semantics). It reads the
// camera envelope the same way the production path does, so it stays valid
// under LOD damping — there is no per-frame state on either side to diverge.
//
// The production API is handle-only; the SpatialDatabase keeps no payload index. For
// test readability everything here resolves payloads by brute-force scan
// over the attached pages (findByScan) — deliberately slow, tests only.
struct SpatialDatabase::TestAccess
{
    struct TlasQueryScratch
    {
        std::vector<VisibleItem> visible;
        std::vector<TlasItem> stack;
    };

    static size_t queryTlas(SpatialDatabase& w, const Camera& camera, float minPix,
                            TlasQueryScratch& scratch)
    {
        w.tlasQuery(camera, minPix, -1.0f, scratch.visible, scratch.stack);
        return scratch.visible.size();
    }

    static NodeHandle findByScan(SpatialDatabase& w, UserPayload payload)
    {
        for (uint32_t s = 0; s < uint32_t(w.slots_.size()); ++s)
        {
            const PageRt& rt = w.slots_[s];
            if (!rt.inUse()) continue;
            const PageView& page = w.pageView(rt);
            for (uint32_t i = 1; i < page.nodeCount(); ++i)
                if (page.payload[i] == payload)
                    return NodeHandle{s, i, w.pageStamps_[s].generation()};
        }
        return NodeHandle{};
    }
    static NodeHandle requireByScan(SpatialDatabase& w, UserPayload payload)
    {
        const NodeHandle h = findByScan(w, payload);
        if (!h.valid()) throw std::logic_error("TestAccess: unknown payload");
        return h;
    }
    static const PageView& pageOf(SpatialDatabase& w, UserPayload anyNodeInPage)
    {
        return w.pageView(w.slots_[requireByScan(w, anyNodeInPage).slot()]);
    }
    static uint32_t lastTouched(SpatialDatabase& w, UserPayload anyNodeInPage)
    {
        return w.slots_[requireByScan(w, anyNodeInPage).slot()].lastTouched;
    }
    static void forceTlasRebuild(SpatialDatabase& w) { w.tlasDirty_ = true; }
    // False means the next query will use the tree as it stands. This is how a
    // test asserts that an edit was applied INCREMENTALLY rather than by
    // deferring a full rebuild, which is invisible in the cut either way.
    static bool tlasDirty(SpatialDatabase& w) { return w.tlasDirty_; }
    static uint32_t tlasEscapes(SpatialDatabase& w) { return w.tlasEscapes_; }
    static size_t tlasNodeCount(SpatialDatabase& w) { return w.tlasNodes_.size(); }
    static size_t instanceSlotCount(SpatialDatabase& w) { return w.instances_.size(); }
    static uint32_t instanceLayoutVersion(SpatialDatabase& w)
    {
        return w.instanceLayoutVersion_;
    }
    static size_t assetRtBytes() { return sizeof(AssetRt); }
    static size_t pageRtBytes() { return sizeof(PageRt); }
    static size_t pageStampBytes() { return sizeof(PageStamp); }
    static size_t pageResidencyBytes() { return sizeof(PageResidency); }
    static size_t overlayBytes() { return sizeof(Overlay); }
    static size_t overlayListBytes() { return sizeof(OverlayList); }
    static size_t instanceBytes() { return sizeof(Instance); }
    static size_t instanceTlasBytes() { return sizeof(InstanceTlas); }
    static size_t tlasNodeBytes() { return sizeof(TlasNode); }
    static size_t workItemBytes() { return sizeof(WorkItem); }
    static size_t nodeItemBytes() { return sizeof(NodeItem); }
    static size_t pendingMoveBytes() { return sizeof(PendingMove); }
    static size_t tlasItemBytes() { return sizeof(TlasItem); }
    static size_t mortonItemBytes() { return sizeof(MortonItem); }
    static bool fullyResidentTree(SpatialDatabase& w, UserPayload anyNodeInPage)
    {
        return w.pageTreeFullyResident(requireByScan(w, anyNodeInPage).slot());
    }

    // Invariant (D) across a page boundary is a per-mount scalar now, not a
    // rewrite of the child page's error array (page bytes are immutable).
    static float errClampOf(SpatialDatabase& w, UserPayload anyNodeInPage)
    {
        return w.slots_[requireByScan(w, anyNodeInPage).slot()].errClamp;
    }

    // How many mounts this instance has taken a private copy of the bounds
    // for. Zero unless the instance has been deformed.
    static size_t overlaysOf(SpatialDatabase& w, SpatialDatabase::InstanceRef ref)
    {
        const Instance* inst = w.resolveInstance(ref);
        return inst && inst->hasOverlayList()
                   ? w.overlayLists_[inst->overlayList()].refs.size()
                   : 0;
    }

    static bool overlayIsSparse(SpatialDatabase& w, SpatialDatabase::InstanceRef ref, uint32_t slot)
    {
        const Instance* inst = w.resolveInstance(ref);
        const Overlay* ov = inst ? w.findOverlay(*inst, slot) : nullptr;
        return ov && ov->sparseWide();
    }

    // Bounds AS THIS INSTANCE SEES THEM. Refit no longer writes into the page
    // (it is shared and immutable), so a test that wants to observe motion has
    // to read through the instance, not through the page.
    static const AABB* bboxOf(SpatialDatabase& w, SpatialDatabase::InstanceRef ref, uint32_t slot)
    {
        Instance* inst = w.resolveInstance(ref);
        return inst ? w.boundsFor(*inst, slot, w.slots_[slot]) : nullptr;
    }
    static WideBoundsRef wideOf(SpatialDatabase& w, SpatialDatabase::InstanceRef ref, uint32_t slot)
    {
        Instance* inst = w.resolveInstance(ref);
        if (!inst) return WideBoundsRef{};
        if (const Overlay* found = w.findOverlay(*inst, slot);
            found && found->sparseWide())
        {
            Overlay& ov = w.overlays_[size_t(found - w.overlays_.data())];
            const PageView& page = w.pageView(w.slots_[slot]);
            for (uint32_t b = 0; b < page.wideCount(); ++b)
                (void)w.mutableWideBounds(ov, page, b);
        }
        uint32_t sparse = kInvalidIndex;
        return w.wideBoundsFor(*inst, slot, w.slots_[slot], &sparse);
    }
    static AABB instanceWorldBox(SpatialDatabase& w, SpatialDatabase::InstanceRef ref)
    {
        w.flushBounds();
        const Instance* inst = w.resolveInstance(ref);
        return inst ? w.instanceTlas_[size_t(inst - w.instances_.data())].worldBox
                    : AABB::empty();
    }
    // The runtime state a mount holds, which every instance of the asset
    // shares: used to assert that deforming one instance does not fork it.
    static const void* mountStateOf(SpatialDatabase& w, UserPayload anyNodeInPage)
    {
        return &w.slots_[requireByScan(w, anyNodeInPage).slot()];
    }

    static std::vector<std::pair<uint32_t, uint8_t>> tlasQuery(SpatialDatabase& w,
                                                               const Camera& v,
                                                               float minPix)
    {
        std::vector<std::pair<uint32_t, uint8_t>> out;
        std::vector<SpatialDatabase::VisibleItem> packed;
        std::vector<SpatialDatabase::TlasItem> stack;
        w.tlasQuery(v, minPix, -1.0f, packed, stack);
        out.reserve(packed.size());
        for (const SpatialDatabase::VisibleItem item : packed)
        {
            const uint32_t instance = w.publicInstanceId(item.instance());
            out.emplace_back(instance, item.mask());
        }
        return out;
    }

    // Structural audit of the top-level tree. Returns "" when sound, otherwise
    // the first thing found wrong. Incremental insertion and removal edit this
    // tree in place, so every one of these is a way for a spawn to make an
    // instance invisible or a stale one visible -- exactly the failures a cut
    // comparison can miss when the camera happens not to look there.
    static std::string tlasValidate(SpatialDatabase& w)
    {
        if (w.tlasDirty_) return "";   // a rebuild is pending; nothing to audit
        std::vector<uint32_t> seen(w.instances_.size(), 0);
        size_t alive = 0;
        for (const SpatialDatabase::Instance& i : w.instances_)
            if (i.alive()) ++alive;
        if (w.instanceTlas_.size() != w.instances_.size() ||
            (!w.instanceFlatSlots_.empty() &&
             w.instanceFlatSlots_.size() != w.instances_.size()))
            return "instance hot/cold arrays differ in size";
        size_t flat = 0;
        for (uint32_t id = 0; id < uint32_t(w.instances_.size()); ++id)
            if (!w.instanceFlatSlots_.empty() && w.instances_[id].alive() &&
                w.instanceFlatSlots_[id] != kInvalidIndex)
                ++flat;
        if (flat != w.flatInstanceCount_)
            return "flat-instance count disagrees with instance slots";
        if (w.liveInstances_.size() != alive)
            return "dense live list count disagrees with instance slots";
        std::vector<uint8_t> listed(w.instances_.size(), 0);
        for (uint32_t dense = 0; dense < uint32_t(w.liveInstances_.size()); ++dense)
        {
            const uint32_t id = w.liveInstances_[dense];
            if (id >= w.instances_.size()) return "dense live list names no instance";
            if (!w.instances_[id].alive()) return "dense live list contains dead instance";
            if (listed[id]++) return "dense live list contains an instance twice";
            if (w.instanceTlas_[id].liveIndex != dense)
                return "dense live-list back-pointer disagrees";
        }
        if (w.tlasRoot_ < 0)
            return alive == 0 ? "" : "empty tree with " + std::to_string(alive) +
                                         " live instances";

        std::vector<int32_t> stack{w.tlasRoot_};
        std::vector<uint8_t> visited(w.tlasNodes_.size(), 0);
        size_t found = 0;
        while (!stack.empty())
        {
            const int32_t ni = stack.back();
            stack.pop_back();
            if (visited[size_t(ni)]) return "node reachable twice";
            visited[size_t(ni)] = 1;
            const SpatialDatabase::TlasNode& n = w.tlasNodes_[size_t(ni)];
            if (n.validLanes() == 0) return "reachable node with no valid lane";
            for (uint32_t l = 0; l < kWide; ++l)
            {
                if (!(n.validMask & (1u << l))) continue;
                const AABB lane = n.bounds.lane(l);
                if (n.child[l] < 0)
                {
                    const uint32_t id = uint32_t(~n.child[l]);
                    if (id >= w.instances_.size()) return "lane names no instance";
                    const SpatialDatabase::Instance& inst = w.instances_[id];
                    const SpatialDatabase::InstanceTlas& spatial = w.instanceTlas_[id];
                    if (!inst.alive()) return "dead instance still in the tree";
                    if (seen[id]++) return "instance in the tree twice";
                    if (spatial.tlasNode() != uint32_t(ni) || spatial.tlasLane() != l)
                        return "instance back-pointer disagrees with its lane";
                    if (!lane.contains(spatial.worldBox))
                        return "lane does not contain its instance";
                    if ((n.laneMask[l] & spatial.mask) != spatial.mask)
                        return "lane mask drops layers its instance is on";
                    if (n.maxErr.v[l] < spatial.maxErrWorld)
                        return "lane error below its instance's";
                    if (n.singleRoot(l) != w.instanceHasSingleRoot(id))
                        return "lane single-root flag disagrees with its instance";
                    ++found;
                }
                else
                {
                    if (n.singleRoot(l))
                        return "inner lane carries a leaf single-root flag";
                    const SpatialDatabase::TlasNode& c = w.tlasNodes_[size_t(n.child[l])];
                    if (c.parent != ni) return "child's parent link is wrong";
                    float childErr = 0.0f;
                    uint32_t childMask = 0;
                    const AABB ext = w.tlasNodeExtent(c, childErr, childMask);
                    if (!lane.contains(ext)) return "lane does not contain its subtree";
                    if ((n.laneMask[l] & childMask) != childMask)
                        return "lane mask drops layers its subtree is on";
                    if (n.maxErr.v[l] < childErr) return "lane error below its subtree's";
                    stack.push_back(n.child[l]);
                }
            }
        }
        if (found != alive)
            return "tree holds " + std::to_string(found) + " of " +
                   std::to_string(alive) + " live instances";
        return "";
    }

    // Ancestor chain of a node as payloads, walking parent links inside pages
    // and owner links across pages, up to (excluding) the instance sentinel.
    static std::vector<UserPayload> ancestorIds(SpatialDatabase& w, UserPayload payload)
    {
        std::vector<UserPayload> out;
        const NodeHandle h = requireByScan(w, payload);
        NodeRef r{h.slot(), h.index()};
        while (true)
        {
            const PageRt& rt = w.slots_[r.slot];
            const PageView& page = w.pageView(rt);
            uint32_t i = page.parent[r.index];
            while (i != 0)
            {
                out.push_back(page.payload[i]);
                i = page.parent[i];
            }
            if (!rt.owner.valid()) return out;
            r = rt.owner;
            out.push_back(w.pageView(w.slots_[r.slot]).payload[r.index]);
        }
    }

    static RefResult referenceFrontier(SpatialDatabase& w, const Camera& view, const SelectionParams& p)
    {
        w.flushBounds();
        RefResult out;
        for (uint32_t id = 0; id < uint32_t(w.instances_.size()); ++id)
        {
            const Instance& inst = w.instances_[id];
            if (!inst.alive()) continue;
            const InstanceTlas& spatial = w.instanceTlas_[id];
            uint8_t mask = kAllPlanes;
            if (testAabb(spatial.worldBox, view.frustum, mask) == CullState::Outside)
                continue;
            if (p.minPix > 0.0f)
            {
                const float e = screenError(
                    spatial.maxErrWorld, view.k,
                    distanceToBox(spatial.worldBox, view.queryMin(), view.queryMax()));
                if (e < p.minPix) continue;
            }
            const Camera local = toLocal(view, inst.pos, inst.scale);
            refChildren(w, inst, id, inst.rootSlot, 0, true, true, local, p, out);
        }
        out.sync();
        return out;
    }

private:
    static void refChildren(SpatialDatabase& w, const Instance& inst, InstanceId instance,
                            uint32_t slot,
                            uint32_t node, bool current, bool ideal,
                            const Camera& local,
                            const SelectionParams& p, RefResult& out)
    {
        const PageView& pg = w.pageView(w.slots_[slot]);
        uint32_t c = node + 1;
        for (uint32_t k = 0; k < pg.childCount(node); ++k)
        {
            refNode(w, inst, instance, slot, c, current, ideal, local, p, out);
            c += pg.subtreeSize[c];
        }
    }

    static void refNode(SpatialDatabase& w, const Instance& inst, InstanceId instance,
                        uint32_t slot, uint32_t i,
                        bool current, bool ideal, const Camera& local,
                        const SelectionParams& p,
                        RefResult& out)
    {
        const PageRt& rt = w.slots_[slot];
        const PageView& pg = w.pageView(rt);
        // Whatever bounds this instance sees: its overlay if it has been
        // deformed, the shared page otherwise.
        const AABB* bbox = w.boundsFor(inst, slot, rt);

        uint8_t mask = kAllPlanes;
        if (testAabb(bbox[i], local.frustum, mask) == CullState::Outside) return;

        const float err = screenError(
            std::min(pg.geometricError[i], rt.errClamp), local.k,
            distanceToBox(bbox[i], local.queryMin(), local.queryMax()));
        const uint32_t cc = pg.childCount(i);
        const bool exp = pg.isExpansion(i);
        const bool wants = ideal && (cc > 0 || exp) && err > p.threshold;

        const NodeHandle here{slot, i, w.pageStamps_[slot].generation()};
        if (!ideal)
        {
            if (rt.isResident(i))
            {
                out.buffers.currentOnly.emplace_back(here, err, p.threshold, instance);
                return;
            }
        }
        else if (!wants)
        {
            const bool shared = current && rt.isResident(i);
            (shared ? out.buffers.shared : out.buffers.idealOnly)
                .emplace_back(here, err, p.threshold, instance);
            if (!current || shared) return;
        }

        const uint32_t childSlot =
            (exp && !rt.expSlot.empty()) ? rt.expSlot[i] : kInvalidIndex;
        if (ideal && wants && exp && childSlot == kInvalidIndex)
        {
            const FrontierEntry entry{here, err, p.threshold, instance};
            (current ? out.buffers.shared : out.buffers.idealOnly).push_back(entry);
            return;
        }

        bool nextCurrent = current;
        bool nextIdeal = false;
        if (ideal && wants)
        {
            const bool canDescend =
                !current || w.visibleDescendantsCovered(slot, i, mask, inst, local);
            if (current && !canDescend)
            {
                out.buffers.currentOnly.emplace_back(here, err, p.threshold, instance);
            }
            nextCurrent = current && canDescend;
            nextIdeal = true;
        }

        if (exp)
            refChildren(w, inst, instance, childSlot, 0, nextCurrent, nextIdeal,
                        local, p, out);
        else
            refChildren(w, inst, instance, slot, i, nextCurrent, nextIdeal,
                        local, p, out);
    }
};

} // namespace frontier

namespace frontiertest {

using namespace frontier;
using TAX = SpatialDatabase::TestAccess;

// Tests key their content by numeric ids; those ids travel as the opaque
// payload. The alias keeps test code reading naturally.
using UserId = UserPayload;

inline std::vector<FrontierEntry> currentFrontier(const FrontierResultView& cut)
{
    std::vector<FrontierEntry> out(cut.shared.begin(), cut.shared.end());
    out.insert(out.end(), cut.currentOnly.begin(), cut.currentOnly.end());
    return out;
}

inline std::vector<FrontierEntry> idealFrontier(const FrontierResultView& cut)
{
    std::vector<FrontierEntry> out(cut.shared.begin(), cut.shared.end());
    out.insert(out.end(), cut.idealOnly.begin(), cut.idealOnly.end());
    return out;
}

inline size_t currentFrontierSize(const FrontierResultView& cut)
{
    return cut.currentSize();
}

inline size_t idealFrontierSize(const FrontierResultView& cut)
{
    return cut.idealSize();
}

inline UserPayload payloadOf(const SpatialDatabase& w, const FrontierEntry& entry)
{
    UserPayload payload = 0;
    if (!w.tryGetPayload(entry.nodeHandle, payload))
        throw std::logic_error("payloadOf: stale cut handle");
    return payload;
}

inline std::vector<UserPayload> pageIds(const PageView& pg)
{
    std::vector<UserPayload> out;
    for (uint32_t i = 1; i < pg.nodeCount(); ++i) out.push_back(pg.payload[i]);
    return out;
}

// ---------------------------------------------------------------------------
// Payload-keyed wrappers over the handle-only production API. Each one is a
// brute-force scan (TestAccess::findByScan) — fine for tests, never for
// production code, which composes handles from attach results and selectFrontier
// outputs instead.
// ---------------------------------------------------------------------------
inline NodeHandle handleOf(SpatialDatabase& w, UserPayload p) { return TAX::requireByScan(w, p); }

inline void markResident(SpatialDatabase& w, UserPayload p) { w.markResident(handleOf(w, p)); }
inline void markNonResident(SpatialDatabase& w, UserPayload p) { w.markNonResident(handleOf(w, p)); }
inline bool isResident(SpatialDatabase& w, UserPayload p) { return w.isResident(handleOf(w, p)); }
inline bool contains(SpatialDatabase& w, UserPayload p) { return TAX::findByScan(w, p).valid(); }

inline PageHandle attachPage(SpatialDatabase& w, UserPayload expansion, Page page)
{
    return w.attachPage(handleOf(w, expansion), std::move(page));
}
inline void detachPage(SpatialDatabase& w, UserPayload expansion)
{
    w.detachPage(handleOf(w, expansion));
}
inline bool isAttached(SpatialDatabase& w, UserPayload expansion)
{
    const NodeHandle h = TAX::findByScan(w, expansion);
    return h.valid() && w.isAttached(h);
}
inline void setNodeBounds(SpatialDatabase& w, SpatialDatabase::InstanceRef inst, UserPayload p,
                          const AABB& b)
{
    w.setNodeBounds(inst, handleOf(w, p), b);
}
inline AABB nodeBounds(SpatialDatabase& w, SpatialDatabase::InstanceRef inst, UserPayload p)
{
    return w.nodeBounds(inst, handleOf(w, p));
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
        PageBuilder b;
        lastIds.clear();
        addSubtree(b, 0, true, region, rootErr, 0, pageLevels);
        return b.build();
    }

    Page makeChildPage(UserId expansionId)
    {
        const Recipe r = recipes.at(expansionId);
        PageBuilder b;
        lastIds.clear();
        for (uint32_t f = 0; f < fanout; ++f)
            addSubtree(b, 0, true, slab(r.region, f, fanout), r.rootErr, 0, r.pageLevels);
        return b.build();
    }

private:
    void addSubtree(PageBuilder& b, PageBuilder::NodeId parent, bool isRoot,
                    const AABB& region, float err, uint32_t level, uint32_t pageLevels)
    {
        const UserId id = nextId++;
        lastIds.push_back(id);
        const bool pageLeaf = (level == depth);
        const PageBuilder::NodeId n =
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

inline void markAllResident(SpatialDatabase& w, const std::vector<UserPayload>& ids)
{
    for (UserPayload id : ids) markResident(w, id);
}

// Fast path for benchmarks: mark a whole freshly attached page resident by
// composing handles from the PageHandle — no scans.
inline void markAllResident(SpatialDatabase& w, PageHandle page, uint32_t nodeCount)
{
    for (uint32_t i = 1; i < nodeCount; ++i) w.markResident(nodeAt(page, i));
}

inline AABB unitRegion(float halfSize = 100.0f)
{
    return AABB::fromMinMax(float4::vec(-halfSize, -halfSize, -halfSize),
                            float4::vec(halfSize, halfSize, halfSize));
}

} // namespace frontiertest
