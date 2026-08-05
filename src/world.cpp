#include "hlod/world.h"

#include <algorithm>
#include <atomic>
#include <bit>

namespace hlod {

#ifdef HLOD_STATS
  #define HLOD_STAT(w, field, n) ((w).stats.field += (n))
#else
  #define HLOD_STAT(w, field, n) ((void)sizeof(w), (void)0)
#endif

namespace {

inline float axisOf(float4 v, int axis)
{
    return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);
}

inline float surfaceArea(const AABB& b)
{
    if (b.isEmpty()) return 0.0f;
    const float4 e = b.mx - b.mn;
    return 2.0f * (e.x * e.y + e.y * e.z + e.z * e.x);
}

} // namespace

World::World(const WorldConfig& config) : config_(config)
{
    if (config_.context.workerCount == 0) config_.context.workerCount = 1;
    workers_.resize(1);
}

World::~World() = default;

// ============================================================================
// handle resolution — two loads and three compares, no hashing anywhere
// ============================================================================

const World::PageRt* World::resolve(NodeHandle h) const
{
    if (h.slot >= slots_.size()) return nullptr;
    const PageRt& rt = slots_[h.slot];
    if (!rt.inUse || rt.generation != h.generation) return nullptr;   // stale
    if (h.index == 0 || h.index >= rt.page.nodeCount()) return nullptr;
    return &rt;
}

const World::AssetRt* World::resolveAsset(AssetHandle h) const
{
    if (h.slot >= assets_.size()) return nullptr;
    const AssetRt& as = assets_[h.slot];
    if (!as.inUse || as.generation != h.generation) return nullptr;
    return &as;
}

// ============================================================================
// assets — the unit of sharing
// ============================================================================

uint32_t World::allocAsset()
{
    if (!freeAssets_.empty())
    {
        const uint32_t a = freeAssets_.back();
        freeAssets_.pop_back();
        return a;
    }
    assets_.emplace_back();
    return uint32_t(assets_.size() - 1);
}

uint32_t World::createAsset(Page&& owned, PageView borrowed, bool registered)
{
    const uint32_t a = allocAsset();
    AssetRt& as = assets_[a];
    as = AssetRt{};
    as.owned = std::move(owned);
    // The view points into the blob, not into the AssetRt, so it survives the
    // assets_ vector reallocating underneath it.
    as.view       = as.owned.valid() ? static_cast<const PageView&>(as.owned) : borrowed;
    as.generation = ++generationCounter_;
    as.registered = registered;
    as.inUse      = true;
    ++liveAssets_;
    return a;
}

void World::destroyAssetIfUnused(uint32_t a)
{
    AssetRt& as = assets_[a];
    if (!as.inUse) return;
    if (as.registered || as.mountRefs > 0 || as.instanceRefs > 0) return;
    as = AssetRt{};   // releases the blob if the World owned it
    freeAssets_.push_back(a);
    --liveAssets_;
}

AssetHandle World::registerAsset(Page&& page)
{
    HLOD_CHECK(page.nodeCount() > 1, "World::registerAsset: empty page");
    const uint32_t a = createAsset(std::move(page), PageView{}, true);
    return AssetHandle{a, assets_[a].generation};
}

AssetHandle World::registerAsset(PageView borrowedPage)
{
    HLOD_CHECK(borrowedPage.valid() && borrowedPage.nodeCount() > 1,
               "World::registerAsset: empty page");
    const uint32_t a = createAsset(Page{}, borrowedPage, true);
    return AssetHandle{a, assets_[a].generation};
}

void World::releaseAsset(AssetHandle h)
{
    if (!resolveAsset(h)) return;   // already released; stale handles are quiet
    const uint32_t a = h.slot;
    HLOD_CHECK(assets_[a].instanceRefs == 0,
               "World::releaseAsset: instances still reference this asset");
    assets_[a].registered = false;
    if (assets_[a].rootMount != kInvalidIndex)
    {
        const uint32_t root = assets_[a].rootMount;
        assets_[a].rootMount = kInvalidIndex;
        detachMountTree(root, nullptr);
    }
    destroyAssetIfUnused(a);
}

bool World::isAsset(AssetHandle h) const { return resolveAsset(h) != nullptr; }

PageHandle World::assetRootPage(AssetHandle h) const
{
    const AssetRt* as = resolveAsset(h);
    if (!as || as->rootMount == kInvalidIndex) return PageHandle{};
    return PageHandle{as->rootMount, slots_[as->rootMount].generation};
}

// ============================================================================
// mounts / attach / detach
// ============================================================================

uint32_t World::allocSlot()
{
    if (!freeSlots_.empty())
    {
        uint32_t s = freeSlots_.back();
        freeSlots_.pop_back();
        return s;
    }
    slots_.emplace_back();
    return uint32_t(slots_.size() - 1);
}

uint32_t World::registerPage(uint32_t asset, NodeRef owner, bool pinned)
{
    const uint32_t slot = allocSlot();
    AssetRt& as = assets_[asset];
    PageRt& rt = slots_[slot];

    rt.page     = as.view;
    rt.asset    = asset;
    rt.errClamp = FLT_MAX;
    rt.resident.assign(rt.page.nodeCount(), 0);
    rt.readyChildren.assign(rt.page.nodeCount(), 0);
    rt.reqStamp.clear();
    rt.expSlot.clear();
    rt.generation = ++generationCounter_;
    // Bumped here as well as on every content change, so that a slot which was
    // detached and reused can never present the same contentVersion as before.
    // That is what lets a SelectionContext record identify a page's state with a
    // single word instead of a (generation, version) pair.
    ++rt.contentVersion;
    rt.lastTouched = frame_;
    rt.attachedChildPages = 0;
    rt.pinned = pinned;
    rt.inUse  = true;
    rt.owner = owner;

    ++as.mountRefs;
    ++attachedPages_;
    // Pinned pages can never be collected, so they never enter the LRU list;
    // that turns lruTouch into a single compare for them (a big deal for
    // forests of instances, whose root pages are all pinned).
    if (pinned) ++pinnedPages_;
    else lruPushFront(slot);
    return slot;
}

void World::pinRootPayloads(uint32_t slot)
{
    // Pin the roots' payloads: the base case of invariant (F).
    PageRt& rt = slots_[slot];
    uint32_t c = 1;
    for (uint32_t k = 0; k < rt.page.childCount(0); ++k)
    {
        rt.resident[c] = 1;
        rt.readyChildren[0]++;
        c += rt.page.subtreeSize[c];
    }
}

PageHandle World::attachPage(NodeHandle expansionNode, AssetHandle assetHandle)
{
    const AssetRt* as = resolveAsset(assetHandle);
    HLOD_CHECK(as != nullptr, "World::attachPage: invalid or released asset");

    // Stale expansion handle: the parent page was detached/collected while
    // this page was being streamed. Normal streaming race — reject quietly.
    if (!resolve(expansionNode)) return PageHandle{};

    const uint32_t asset = assetHandle.slot;
    const NodeRef owner{expansionNode.slot, expansionNode.index};
    {
        const PageRt& ownerRt = slots_[owner.slot];
        HLOD_CHECK(ownerRt.page.isExpansion(owner.index),
                   "World::attachPage: not an expansion point");
        HLOD_CHECK(ownerRt.expSlot.empty() || ownerRt.expSlot[owner.index] == kInvalidIndex,
                   "World::attachPage: already attached");
    }
    HLOD_CHECK(assets_[asset].view.nodeCount() > 1, "World::attachPage: empty page");

    // (C) across the boundary: the owner must contain the page's content.
    // Growing the owner here is not an option — its bytes back every instance
    // of the owning asset, so the growth would have to ripple into all of
    // their top-level bounds, and a streaming event would silently become an
    // O(instances) write. Author expansion bounds that contain what attaches.
    HLOD_CHECK(slots_[owner.slot].page.bbox[owner.index].contains(
                   assets_[asset].view.bbox[0]),
               "World::attachPage: the attached page escapes the expansion node's "
               "authored bounds — author conservative expansion bounds at build time");

    // (D) across the boundary: the child page's effective error ceiling is
    // the owner expansion node's own effective error. Carried as a scalar and
    // folded into the wide test, so the page's bytes are never touched — the
    // same asset can hang under a dozen different expansion points, each with
    // its own ceiling, and attach stays O(1) instead of O(nodeCount).
    const float childClamp =
        std::min(slots_[owner.slot].page.geometricError[owner.index],
                 slots_[owner.slot].errClamp);

    // NOTE: registerPage can reallocate slots_, so nothing above may be held
    // as a reference across this call.
    const uint32_t slot = registerPage(asset, owner, false);
    slots_[slot].errClamp = childClamp;

    PageRt& ort = slots_[owner.slot];
    if (ort.expSlot.empty()) ort.expSlot.assign(ort.page.nodeCount(), kInvalidIndex);
    ort.expSlot[owner.index] = slot;
    ort.attachedChildPages++;
    ++ort.contentVersion;   // this page now refines further than it did

    return PageHandle{slot, slots_[slot].generation};
}

PageHandle World::attachPage(NodeHandle expansionNode, Page&& page)
{
    // Check the handle before taking ownership so a stale attach leaves the
    // caller's page untouched (they just drop it).
    if (!resolve(expansionNode)) return PageHandle{};
    HLOD_CHECK(page.nodeCount() > 1, "World::attachPage: empty page");

    const uint32_t a = createAsset(std::move(page), PageView{}, false);
    const PageHandle h = attachPage(expansionNode, AssetHandle{a, assets_[a].generation});
    if (!h.valid()) destroyAssetIfUnused(a);
    return h;
}

void World::detachPage(NodeHandle expansionNode)
{
    const PageRt* ownerRt = resolve(expansionNode);
    if (!ownerRt) return;   // parent page already gone; nothing to detach
    const uint32_t child = ownerRt->expSlot.empty()
                               ? kInvalidIndex
                               : ownerRt->expSlot[expansionNode.index];
    HLOD_CHECK(child != kInvalidIndex, "World::detachPage: not attached");
    PageRt& rt = slots_[child];
    HLOD_CHECK(rt.attachedChildPages == 0,
               "World::detachPage: attached child pages remain");
    HLOD_CHECK(!rt.pinned, "World::detachPage: page is pinned");
    detachSlot(child, nullptr);
}

bool World::isAttached(NodeHandle expansionNode) const
{
    const PageRt* rt = resolve(expansionNode);
    return rt && !rt->expSlot.empty() &&
           rt->expSlot[expansionNode.index] != kInvalidIndex;
}

void World::detachSlot(uint32_t slot, std::vector<UserPayload>* freedPayloads)
{
    PageRt& rt = slots_[slot];
    if (freedPayloads)
        for (uint32_t i = 1; i < rt.page.nodeCount(); ++i)
            if (rt.resident[i]) freedPayloads->push_back(rt.page.payload[i]);
    if (rt.owner.valid())
    {
        PageRt& ownerRt = slots_[rt.owner.slot];
        ownerRt.expSlot[rt.owner.index] = kInvalidIndex;
        ownerRt.attachedChildPages--;
        ++ownerRt.contentVersion;   // it collapses back to a leaf here
    }
    lruUnlink(slot);
    if (rt.pinned) --pinnedPages_;
    const uint32_t asset = rt.asset;
    rt = PageRt{};
    freeSlots_.push_back(slot);
    --attachedPages_;
    // Any per-instance bounds overlay on this mount is now describing a page
    // that no longer exists. They are not hunted down here (that would be a
    // scan of every instance); the generation stamp retires them on the next
    // lookup, and ensureOverlay recycles the storage.

    if (asset != kInvalidIndex)
    {
        AssetRt& as = assets_[asset];
        --as.mountRefs;
        if (as.rootMount == slot) as.rootMount = kInvalidIndex;
        destroyAssetIfUnused(asset);
    }
}

void World::detachMountTree(uint32_t rootSlot, std::vector<UserPayload>* freedPayloads)
{
    if (rootSlot == kInvalidIndex || !slots_[rootSlot].inUse) return;
    // Collect the page tree from its root (preorder via the expansion-slot
    // links), then detach in reverse — children before their owners.
    // O(this tree's pages), independent of the world's size.
    std::vector<uint32_t> order;
    order.push_back(rootSlot);
    for (size_t k = 0; k < order.size(); ++k)
        for (const uint32_t child : slots_[order[k]].expSlot)
            if (child != kInvalidIndex) order.push_back(child);
    for (size_t k = order.size(); k-- > 0;) detachSlot(order[k], freedPayloads);
}

// ============================================================================
// residency
// ============================================================================

void World::markResident(NodeHandle h)
{
    PageRt* rt = resolve(h);
    if (!rt) return;   // page collected while the payload was loading
    if (rt->resident[h.index]) return;
    rt->resident[h.index] = 1;
    rt->readyChildren[rt->page.parent[h.index]]++;
    ++rt->contentVersion;
}

void World::markNonResident(NodeHandle h)
{
    PageRt* rt = resolve(h);
    if (!rt) return;
    HLOD_CHECK(!(rt->pinned && rt->page.parent[h.index] == 0),
               "World::markNonResident: pinned root");
    if (!rt->resident[h.index]) return;
    rt->resident[h.index] = 0;
    rt->readyChildren[rt->page.parent[h.index]]--;
    ++rt->contentVersion;
}

bool World::isResident(NodeHandle h) const
{
    const PageRt* rt = resolve(h);
    return rt && rt->resident[h.index] != 0;
}

// ============================================================================
// instances
// ============================================================================

World::InstanceRef World::addInstanceInternal(uint32_t asset, const InstanceDesc& desc)
{
    HLOD_CHECK(desc.scale > 0.0f, "World::addInstance: non-positive scale");

    InstanceId id;
    if (!freeInstances_.empty())
    {
        id = freeInstances_.back();
        freeInstances_.pop_back();
    }
    else
    {
        instances_.emplace_back();
        instanceTlas_.emplace_back();
        id = InstanceId(instances_.size() - 1);
    }

    // Every instance of an asset walks the SAME root mount: one residency
    // array, one attachment graph, one streaming request set for the whole
    // species. This is the line that makes 10k trees cost one tree.
    if (assets_[asset].rootMount == kInvalidIndex)
    {
        assets_[asset].rootMount = registerPage(asset, NodeRef{}, true);
        pinRootPayloads(assets_[asset].rootMount);
    }
    const uint32_t slot = assets_[asset].rootMount;
    ++assets_[asset].instanceRefs;

    Instance& inst = instances_[id];
    InstanceTlas& spatial = instanceTlas_[id];
    inst = Instance{};
    spatial = InstanceTlas{};
    inst.pos = desc.pos;
    inst.scale = desc.scale;
    inst.rootSlot = slot;
    inst.tag = desc.tag == kAutoTag ? id : desc.tag;
    spatial.asset = asset;
    spatial.mask = desc.mask;
    inst.alive = true;
    inst.generation = ++generationCounter_;
    spatial.liveIndex = uint32_t(liveInstances_.size());
    liveInstances_.push_back(id);
    // Bounds first, then insert: the insert descends on worldBox. Going through
    // refreshInstanceBounds here would hand a not-yet-inserted instance to the
    // motion refit, whose kInvalidIndex case exists to catch exactly that as a
    // bug and would dirty the whole tree.
    recomputeInstanceBounds(id);
    tlasInsert(id);
    markTlasStructuralChange();
    return InstanceRef{id, inst.generation, PageHandle{slot, slots_[slot].generation}};
}

World::InstanceRef World::addInstance(AssetHandle asset, const InstanceDesc& desc)
{
    HLOD_CHECK(resolveAsset(asset) != nullptr,
               "World::addInstance: invalid or released asset");
    return addInstanceInternal(asset.slot, desc);
}

World::InstanceRef World::addInstance(AssetHandle asset, float4 pos, float scale)
{
    InstanceDesc d;
    d.pos = pos;
    d.scale = scale;
    return addInstance(asset, d);
}

World::InstanceRef World::addInstance(Page&& page, const InstanceDesc& desc)
{
    HLOD_CHECK(page.nodeCount() > 1, "World::addInstance: empty page");
    const uint32_t a = createAsset(std::move(page), PageView{}, false);
    return addInstanceInternal(a, desc);
}

World::InstanceRef World::addInstance(Page&& page, float4 pos, float scale)
{
    InstanceDesc d;
    d.pos = pos;
    d.scale = scale;
    return addInstance(std::move(page), d);
}

World::Instance* World::resolveInstance(InstanceRef ref)
{
    if (ref.id >= instances_.size()) return nullptr;
    Instance& inst = instances_[ref.id];
    if (!inst.alive || inst.generation != ref.generation) return nullptr;
    return &inst;
}

// Structural change policy: quality rebuilds are reserved for real population
// drift — world assembly, level load, mass despawn. Under steady churn
// (spawn/despawn at roughly constant population) the instance count barely
// moves, so nothing is forced here at all: the edit is applied incrementally
// and tlasEditFraction decides when the accumulated quality loss is worth a
// rebuild. Only a real change in population size forces one immediately, and
// then it takes the quality tier because it is rare and long-lived.
void World::markTlasStructuralChange()
{
    const uint64_t alive = liveInstances_.size();
    const uint64_t drift = alive > tlasQualityCount_ ? alive - tlasQualityCount_
                                                     : tlasQualityCount_ - alive;
    if (float(drift) > float(tlasQualityCount_) * config_.tlasCountDrift)
    {
        tlasDirty_ = true;
        tlasQualityBuild_ = true;
    }
}

void World::removeInstance(InstanceRef ref)
{
    Instance* inst = resolveInstance(ref);
    if (!inst) return;   // stale ref: the instance is already gone
    const InstanceId id = ref.id;
    const uint32_t   asset = instanceTlas_[id].asset;

    freeOverlays(*inst);
    tlasRemove(id);
    const uint32_t liveIndex = instanceTlas_[id].liveIndex;
    const InstanceId moved = liveInstances_.back();
    liveInstances_[liveIndex] = moved;
    instanceTlas_[moved].liveIndex = liveIndex;
    liveInstances_.pop_back();
    instanceTlas_[id].liveIndex = kInvalidIndex;
    instances_[id].alive = false;
    freeInstances_.push_back(id);
    markTlasStructuralChange();

    AssetRt& as = assets_[asset];
    --as.instanceRefs;

    // An asset's page tree belongs to the asset, not to any one placement: it
    // survives until the last instance goes away (or releaseAsset), so
    // removing one tree out of ten thousand does not throw away the streamed
    // topology the other 9,999 are still using.
    if (as.instanceRefs == 0 && as.rootMount != kInvalidIndex)
    {
        const uint32_t root = as.rootMount;
        as.rootMount = kInvalidIndex;
        detachMountTree(root, nullptr);
    }
    destroyAssetIfUnused(asset);
}

void World::moveInstance(InstanceRef ref, float4 pos, float scale)
{
    HLOD_CHECK(scale > 0.0f, "World::moveInstance: non-positive scale");
    Instance* inst = resolveInstance(ref);
    if (!inst) return;   // stale ref: never touch the slot's new occupant
    inst->pos = pos;
    inst->scale = scale;
    refreshInstanceBounds(ref.id);
}

void World::refreshInstanceBounds(InstanceId id)
{
    recomputeInstanceBounds(id);
    tlasOnInstanceMoved(id);
}

void World::recomputeInstanceBounds(InstanceId id)
{
    Instance& inst = instances_[id];
    InstanceTlas& spatial = instanceTlas_[id];
    // Globally unique rather than per-instance so a recycled slot can never
    // match the previous occupant's SelectionContext record.
    inst.cutVersion = ++generationCounter_;
    const PageRt& rt = slots_[inst.rootSlot];
    const AABB* bbox = boundsFor(inst, inst.rootSlot, rt);
    spatial.worldBox = toWorld(bbox[0], inst.pos, inst.scale);

    float maxErr = 0.0f;
    uint32_t c = 1;
    for (uint32_t k = 0; k < rt.page.childCount(0); ++k)
    {
        maxErr = std::max(maxErr, std::min(rt.page.geometricError[c], rt.errClamp));
        c += rt.page.subtreeSize[c];
    }
    spatial.maxErrWorld = maxErr * inst.scale;
}

// ============================================================================
// copy-on-write bounds overlays
//
// Bounds are the only bytes of a page the runtime ever rewrites. Giving a
// deformed instance a private copy of just those keeps everything that
// actually costs at scale — topology, payloads, errors, residency, the
// attachment graph, the streaming request set — shared with every other
// instance of the same asset. Duplicating the whole page instead would
// duplicate the streaming state too, which is the expensive half.
// ============================================================================

const World::Overlay* World::findOverlay(const Instance& inst, uint32_t slot) const
{
    if (inst.overlays.empty()) return nullptr;   // the common case, one compare
    const auto it = std::lower_bound(
        inst.overlays.begin(), inst.overlays.end(), slot,
        [](const OverlayRef& r, uint32_t s) { return r.slot < s; });
    if (it == inst.overlays.end() || it->slot != slot) return nullptr;
    const Overlay& ov = overlays_[it->index];
    // The mount may have been detached (and its slot reused) since the
    // overlay was taken, in which case it describes a page that is gone.
    if (!ov.inUse || ov.generation != slots_[slot].generation) return nullptr;
    return &ov;
}

void World::initOverlay(Overlay& ov, const PageRt& rt)
{
    const PageView& pg = rt.page;
    ov.generation = rt.generation;
    ov.inUse = true;
#ifdef HLOD_OVERLAY_FULL_PAGE
    ov.page = Page::fromBytes(pg.data(), pg.byteSize(), config_.context);
#else
    ov.bbox.assign(pg.bbox, pg.bbox + pg.nodeCount());
    ov.wide.resize(pg.wideCount());
    for (uint32_t b = 0; b < pg.wideCount(); ++b) ov.wide[b] = pg.wide[b].bounds;
#endif
}

uint32_t World::ensureOverlay(Instance& inst, uint32_t slot)
{
    const auto it = std::lower_bound(
        inst.overlays.begin(), inst.overlays.end(), slot,
        [](const OverlayRef& r, uint32_t s) { return r.slot < s; });

    if (it != inst.overlays.end() && it->slot == slot)
    {
        const uint32_t idx = it->index;
        Overlay& ov = overlays_[idx];
        if (!ov.inUse || ov.generation != slots_[slot].generation)
            initOverlay(ov, slots_[slot]);   // stale: retake from the new page
        return idx;
    }

    uint32_t idx;
    if (!freeOverlays_.empty())
    {
        idx = freeOverlays_.back();
        freeOverlays_.pop_back();
    }
    else
    {
        overlays_.emplace_back();
        idx = uint32_t(overlays_.size() - 1);
    }
    Overlay& ov = overlays_[idx];
    ov.slot = slot;
    initOverlay(ov, slots_[slot]);
    ++liveOverlays_;
    inst.overlays.insert(it, OverlayRef{slot, idx});
    return idx;
}

void World::freeOverlays(Instance& inst)
{
    for (const OverlayRef& r : inst.overlays)
    {
        Overlay& ov = overlays_[r.index];
        if (!ov.inUse) continue;
        ov = Overlay{};
        freeOverlays_.push_back(r.index);
        --liveOverlays_;
    }
    inst.overlays.clear();
}

const AABB* World::boundsFor(const Instance& inst, uint32_t slot, const PageRt& rt) const
{
    if (const Overlay* ov = findOverlay(inst, slot))
#ifdef HLOD_OVERLAY_FULL_PAGE
        return ov->page.bbox;
#else
        return ov->bbox.data();
#endif
    return rt.page.bbox;
}

WideBoundsRef World::wideBoundsFor(const Instance& inst, uint32_t slot,
                                   const PageRt& rt) const
{
    if (const Overlay* ov = findOverlay(inst, slot))
#ifdef HLOD_OVERLAY_FULL_PAGE
        return ov->page.wideBounds();   // interleaved again, like the shared page
#else
        return WideBoundsRef::packed(ov->wide.data());
#endif
    return rt.page.wideBounds();
}

bool World::mountBelongsTo(const Instance& inst, uint32_t slot) const
{
    uint32_t s = slot;
    for (size_t guard = 0; guard <= slots_.size(); ++guard)
    {
        if (s == inst.rootSlot) return true;
        const NodeRef o = slots_[s].owner;
        if (!o.valid()) return false;
        s = o.slot;
    }
    return false;
}

size_t World::overlayBytes() const
{
    size_t n = 0;
    for (const Overlay& ov : overlays_)
        if (ov.inUse)
#ifdef HLOD_OVERLAY_FULL_PAGE
            n += ov.page.byteSize();
#else
            n += ov.bbox.size() * sizeof(AABB) + ov.wide.size() * sizeof(WideBounds);
#endif
    return n;
}

// ============================================================================
// motion: lazy, coalesced, deduplicated conservative grow-only refit
// ============================================================================

void World::setNodeBounds(InstanceRef ref, NodeHandle h, const AABB& localBounds)
{
    // Positive ordering check: rejects empty boxes AND NaN (every NaN
    // comparison is false, so !isEmpty() would let NaN through and poison
    // ancestor boxes forever — grow-only refit never un-grows).
    const AABB& b = localBounds;
    HLOD_CHECK(b.mn.x <= b.mx.x && b.mn.y <= b.mx.y && b.mn.z <= b.mx.z &&
                   b.mx.x - b.mn.x < FLT_MAX && b.mx.y - b.mn.y < FLT_MAX &&
                   b.mx.z - b.mn.z < FLT_MAX,
               "World::setNodeBounds: empty or non-finite bounds");
    if (!h.valid()) return;
    const Instance* inst = resolveInstance(ref);
    if (!inst) return;         // stale instance ref
    if (!resolve(h)) return;   // stale handle: the page was detached or collected
    HLOD_ASSERT(mountBelongsTo(*inst, h.slot),
                "World::setNodeBounds: the node is not in this instance's page tree");
    pendingMoves_.push_back(
        {ref.id, ref.generation, h.slot, h.generation, h.index, localBounds});
}

void World::beginFrame()
{
    ++frame_;
}

void World::flushBounds()
{
    // Applied in submission order, so the last box per node is the final
    // state (last write wins). There is deliberately NO dedup structure:
    // with grow-only refit, a repeated move of the same node rewrites the
    // same hot bbox, patches the same hot lane and early-outs at the parent
    // (~hot-cache cost) — while every dedup scheme we measured (per-node
    // stamps, per-page dirty chains, a transient hash set) paid more in
    // cold cache lines than the walks it skipped. Movers sharing ancestors
    // dedup naturally the same way: the walk stops at the first ancestor
    // that already contains the change, so a shared parent is grown once
    // and merely re-checked by the rest. Stale entries (instance removed,
    // page detached, slot reused) self-invalidate via the generation stamps.
    for (const auto& m : pendingMoves_)
    {
        if (m.instance >= instances_.size()) continue;
        const Instance& inst = instances_[m.instance];
        if (!inst.alive || inst.generation != m.instGeneration) continue;
        if (!resolve(NodeHandle{m.slot, m.index, m.generation})) continue;
        applyBoundsChange(m.instance, m.slot, m.index, m.box);
    }
    pendingMoves_.clear();
}

void World::applyBoundsChange(InstanceId id, uint32_t slot, uint32_t index,
                              const AABB& box)
{
    uint32_t curSlot = slot;
    uint32_t cur     = index;
    AABB     curBox  = box;
    bool     exact   = true;   // the submitted node is SET; ancestors only grow

    // A deform privatises bounds into this instance's own overlay, so it can
    // never reach another instance -- one counter on the instance is the whole
    // invalidation. Bumped here rather than deeper because a change that stops
    // early (the ancestor box already contained it) still moved this node.
    instances_[id].cutVersion = ++generationCounter_;

    while (true)
    {
        // Taking the copy is what makes this instance stop sharing bounds for
        // this page — and only this page. Crossing a boundary below promotes
        // the owner too, so exactly the ancestor path is privatised.
        const uint32_t oi = ensureOverlay(instances_[id], curSlot);
#ifdef HLOD_OVERLAY_FULL_PAGE
        // Topology comes from the private copy too, so the whole walk below
        // stays inside one blob instead of straddling page and overlay.
        const PageView& pg = overlays_[oi].page;
        AABB* bbox = const_cast<AABB*>(overlays_[oi].page.bbox);
        const MutWideBoundsRef wide =
            MutWideBoundsRef::interleaved(const_cast<WideBlock*>(overlays_[oi].page.wide));
#else
        const PageView& pg = slots_[curSlot].page;
        AABB* bbox = overlays_[oi].bbox.data();
        const MutWideBoundsRef wide = MutWideBoundsRef::packed(overlays_[oi].wide.data());
#endif

        if (exact)
        {
            bbox[cur] = curBox;
        }
        else
        {
            if (bbox[cur].contains(curBox)) return;   // ancestors already conservative
            bbox[cur].expand(curBox);
        }
        if (cur != 0) patchParentLane(pg, bbox, wide, cur);

        while (cur != 0)
        {
            const uint32_t p = pg.parent[cur];
            if (bbox[p].contains(bbox[cur])) return;
            bbox[p].expand(bbox[cur]);             // grow immediately, shrink lazily
            if (p != 0) patchParentLane(pg, bbox, wide, p);
            cur = p;
        }

        // The page outgrew its sentinel: cross the boundary.
        const NodeRef owner = slots_[curSlot].owner;
        if (!owner.valid())
        {
            refreshInstanceBounds(id);
            return;
        }
        curBox  = bbox[0];
        curSlot = owner.slot;
        cur     = owner.index;
        exact   = false;
    }
}

// Update a node's lane in its parent's wide block (the hot mirror of bbox).
// Which lane holds the node is immutable authored data, so it is read from
// the shared page; only the box is written, into the instance's overlay.
void World::patchParentLane(const PageView& pg, AABB* bbox, MutWideBoundsRef wide,
                            uint32_t index)
{
    const uint32_t p = pg.parent[index];
    const uint32_t cc = pg.childCount(p);
    uint32_t b = pg.wideOffset(p);
    for (uint32_t base = 0; base < cc; base += kWide, ++b)
    {
        const WideBlock& blk = pg.wide[b];
        const uint32_t   valid = pg.validLanes(b);
        for (uint32_t l = 0; l < kWide; ++l)
        {
            if ((valid & (1u << l)) && blk.child[l] == index)
            {
                wide[b].setLane(l, bbox[index]);
                return;
            }
        }
    }
    HLOD_FATAL("World: internal: child lane not found");
}

AABB World::nodeBounds(InstanceRef ref, NodeHandle h)
{
    flushBounds();
    Instance* inst = resolveInstance(ref);
    const PageRt* rt = resolve(h);
    if (!inst || !rt) return AABB::empty();
    return boundsFor(*inst, h.slot, *rt)[h.index];
}

// ============================================================================
// top-level BVH
// ============================================================================

void World::tlasNoteGrowth(float addedArea)
{
    tlasGrownArea_ += addedArea;
    // Cost drift: a population that stays constant while everything moves
    // never trips the count-drift trigger, but grow-only refit bloats the
    // lanes all the same. Watching the added area catches exactly that case.
    if (tlasBaseArea_ > 0.0f && tlasGrownArea_ > tlasBaseArea_ * config_.tlasAreaDrift)
    {
        tlasDirty_ = true;
        tlasQualityBuild_ = true;
    }
}

// Grow-only propagation up the parent chain, shared by motion refit and by
// incremental insertion. Stops at the first ancestor that already covers the
// box, its error and its layer mask -- which is what keeps a small move O(1)
// rather than O(depth).
//
// The layer mask matters here and does not for a pure move: an ancestor's
// laneMask must be a superset of its subtree's instance masks, or tlasQuery's
// layer filter will cull a visible instance. A move cannot change a mask, so
// that term is always already satisfied on the motion path.
float World::tlasGrowUp(uint32_t nodeIdx, const AABB& box, float maxErr,
                        uint32_t laneMask)
{
    float added = 0.0f;
    TlasNode* node = &tlasNodes_[nodeIdx];
    while (node->parent >= 0)
    {
        const int32_t childIdx = int32_t(nodeIdx);
        nodeIdx = uint32_t(node->parent);
        node = &tlasNodes_[nodeIdx];
        uint32_t l = 0;
        for (; l < kWide; ++l)
            if ((node->validMask & (1u << l)) && node->child[l] == childIdx) break;
        if (l == kWide) break;   // already unlinked; nothing above to grow
        AABB laneBox = node->bounds.lane(l);
        if (laneBox.contains(box) && node->maxErr.v[l] >= maxErr &&
            (node->laneMask[l] & laneMask) == laneMask)
            break;
        const float was = surfaceArea(laneBox);
        laneBox.expand(box);
        node->bounds.setLane(l, laneBox);
        node->maxErr.v[l] = std::max(node->maxErr.v[l], maxErr);
        node->laneMask[l] |= laneMask;
        added += surfaceArea(laneBox) - was;
    }
    return added;
}

AABB World::tlasNodeExtent(const TlasNode& n, float& maxErr, uint32_t& laneMask) const
{
    AABB u = AABB::empty();
    maxErr = 0.0f;
    laneMask = 0;
    for (uint32_t l = 0; l < kWide; ++l)
    {
        if (!(n.validMask & (1u << l))) continue;
        u.expand(n.bounds.lane(l));
        maxErr = std::max(maxErr, n.maxErr.v[l]);
        laneMask |= n.laneMask[l];
    }
    return u;
}

int32_t World::tlasAllocNode()
{
    if (!tlasFreeNodes_.empty())
    {
        const int32_t idx = tlasFreeNodes_.back();
        tlasFreeNodes_.pop_back();
        TlasNode& n = tlasNodes_[uint32_t(idx)];
        n.bounds = WideBounds::allEmpty();
        n.maxErr = float8::splat(0.0f);
        n.validMask = 0;
        n.parent = -1;
        for (uint32_t l = 0; l < kWide; ++l)
        {
            n.child[l] = 0;
            n.laneMask[l] = 0;
        }
        return idx;
    }
    const int32_t idx = int32_t(tlasNodes_.size());
    TlasNode& n = tlasNodes_.emplace_back();
    n.bounds = WideBounds::allEmpty();
    n.maxErr = float8::splat(0.0f);
    n.parent = -1;
    for (uint32_t l = 0; l < kWide; ++l)
    {
        n.child[l] = 0;
        n.laneMask[l] = 0;
    }
    return idx;
}

// Incremental edits trade a little tree quality for O(depth) instead of a full
// rebuild. This is what bounds how much of that accumulates.
void World::tlasNoteEdit()
{
    ++tlasEdits_;
    if (float(tlasEdits_) > float(tlasLeafCount_) * config_.tlasEditFraction)
        tlasDirty_ = true;
}

// Descend to the leaf whose lane box grows least, then either take a free lane
// there or SPLIT it: a new node takes the full leaf's place in its parent and
// holds the leaf plus the new instance. Splitting always succeeds, which is why
// there is no "the tree is full, give up and rebuild" case -- the alternative,
// hunting for a free lane somewhere up the chain, fails immediately on a tree
// that was just built full.
void World::tlasInsert(InstanceId id)
{
    if (tlasDirty_) return;   // the pending rebuild will enumerate this instance
    InstanceTlas& inst = instanceTlas_[id];
    if (tlasRoot_ < 0)
    {
        tlasDirty_ = true;    // no tree yet; let a build make the first one
        return;
    }

    uint32_t cur = uint32_t(tlasRoot_);
    for (;;)
    {
        const TlasNode& n = tlasNodes_[cur];
        int32_t  bestChild = -1;
        float    bestGrowth = FLT_MAX;
        for (uint32_t l = 0; l < kWide; ++l)
        {
            if (!(n.validMask & (1u << l))) continue;
            if (n.child[l] < 0) continue;   // an instance, not a subtree
            AABB box = n.bounds.lane(l);
            const float was = surfaceArea(box);
            box.expand(inst.worldBox);
            const float growth = surfaceArea(box) - was;
            if (growth < bestGrowth)
            {
                bestGrowth = growth;
                bestChild = n.child[l];
            }
        }
        if (bestChild < 0) break;   // this node holds instances: place here
        cur = uint32_t(bestChild);
    }

    const uint32_t full = (1u << kWide) - 1;
    uint32_t host = cur;
    if (tlasNodes_[cur].validMask == full)
    {
        // Split. The new node replaces `cur` wherever `cur` was referenced, and
        // adopts it, so nothing above needs to know the difference.
        const int32_t mIdx = tlasAllocNode();
        TlasNode& m = tlasNodes_[uint32_t(mIdx)];
        TlasNode& l0 = tlasNodes_[cur];

        float    childErr = 0.0f;
        uint32_t childMask = 0;
        const AABB childBox = tlasNodeExtent(l0, childErr, childMask);

        m.parent = l0.parent;
        m.bounds.setLane(0, childBox);
        m.maxErr.v[0] = childErr;
        m.child[0] = int32_t(cur);
        m.laneMask[0] = childMask;
        m.validMask = 1u;
        l0.parent = mIdx;

        if (m.parent < 0)
            tlasRoot_ = mIdx;
        else
        {
            TlasNode& p = tlasNodes_[uint32_t(m.parent)];
            for (uint32_t l = 0; l < kWide; ++l)
                if ((p.validMask & (1u << l)) && p.child[l] == int32_t(cur))
                {
                    p.child[l] = mIdx;
                    break;
                }
        }
        host = uint32_t(mIdx);
    }

    TlasNode& h = tlasNodes_[host];
    const uint32_t lane = uint32_t(std::countr_zero(~h.validMask & full));
    h.bounds.setLane(lane, inst.worldBox);
    h.maxErr.v[lane] = inst.maxErrWorld;
    h.child[lane] = ~int32_t(id);
    h.laneMask[lane] = inst.mask;
    h.validMask |= 1u << lane;
    inst.tlasNode = host;
    inst.tlasLane = lane;
    ++tlasLeafCount_;

    tlasNoteGrowth(tlasGrowUp(host, inst.worldBox, inst.maxErrWorld, inst.mask));
    tlasNoteEdit();
}

// Invalidate the lane and unlink any node that empties. Boxes are left loose,
// which is the same grow-only discipline motion uses: a lane that is larger
// than its contents costs a little traversal and nothing else.
void World::tlasRemove(InstanceId id)
{
    if (tlasDirty_) return;
    InstanceTlas& inst = instanceTlas_[id];
    if (inst.tlasNode == kInvalidIndex) return;

    uint32_t nodeIdx = inst.tlasNode;
    const uint32_t lane = inst.tlasLane;
    if (nodeIdx >= tlasNodes_.size() ||
        !(tlasNodes_[nodeIdx].validMask & (1u << lane)) ||
        tlasNodes_[nodeIdx].child[lane] != ~int32_t(id))
    {
        tlasDirty_ = true;   // bookkeeping disagrees; rebuild rather than guess
        return;
    }

    tlasNodes_[nodeIdx].validMask &= ~(1u << lane);
    inst.tlasNode = kInvalidIndex;
    if (tlasLeafCount_) --tlasLeafCount_;

    while (tlasNodes_[nodeIdx].validMask == 0)
    {
        const int32_t parent = tlasNodes_[nodeIdx].parent;
        if (parent < 0)
        {
            tlasRoot_ = -1;
            tlasFreeNodes_.push_back(int32_t(nodeIdx));
            break;
        }
        TlasNode& p = tlasNodes_[uint32_t(parent)];
        for (uint32_t l = 0; l < kWide; ++l)
            if ((p.validMask & (1u << l)) && p.child[l] == int32_t(nodeIdx))
            {
                p.validMask &= ~(1u << l);
                break;
            }
        tlasFreeNodes_.push_back(int32_t(nodeIdx));
        nodeIdx = uint32_t(parent);
    }

    tlasNoteEdit();
}

void World::tlasOnInstanceMoved(InstanceId id)
{
    if (tlasDirty_) return;
    InstanceTlas& inst = instanceTlas_[id];
    if (inst.tlasNode == kInvalidIndex)
    {
        tlasDirty_ = true;
        return;
    }

    // Grow-only lane refit up the parent chain; escapes trigger a rebuild.
    const uint32_t nodeIdx = inst.tlasNode;
    const uint32_t lane = inst.tlasLane;
    TlasNode& node = tlasNodes_[nodeIdx];
    if (node.bounds.lane(lane).contains(inst.worldBox) &&
        node.maxErr.v[lane] >= inst.maxErrWorld)
        return;

    ++tlasEscapes_;
    AABB grown = node.bounds.lane(lane);
    const float wasArea = surfaceArea(grown);
    grown.expand(inst.worldBox);
    node.bounds.setLane(lane, grown);
    node.maxErr.v[lane] = std::max(node.maxErr.v[lane], inst.maxErrWorld);

    float added = surfaceArea(grown) - wasArea;
    added += tlasGrowUp(nodeIdx, grown, inst.maxErrWorld, inst.mask);

    tlasNoteGrowth(added);
    if (float(tlasEscapes_) > float(tlasLeafCount_) * config_.tlasEscapeFraction)
        tlasDirty_ = true;
}

// 21 bits per axis -> 63-bit Morton key.
static inline uint64_t expandBits21(uint64_t v)
{
    v &= 0x1FFFFFull;
    v = (v | v << 32) & 0x1F00000000FFFFull;
    v = (v | v << 16) & 0x1F0000FF0000FFull;
    v = (v | v << 8)  & 0x100F00F00F00F00Full;
    v = (v | v << 4)  & 0x10C30C30C30C30C3ull;
    v = (v | v << 2)  & 0x1249249249249249ull;
    return v;
}

// Partition items[lo, hi) into [lo, m) and [m, hi). BinnedSAH scans 16 bins on
// all three axes and takes the cheapest plane; Median (and any degenerate SAH
// case, e.g. coincident centroids) falls back to a longest-axis median split,
// which always makes progress.
int World::tlasSplit(std::vector<uint32_t>& items, int lo, int hi)
{
    const int count = hi - lo;
    if (count <= 1) return hi;

    AABB cb = AABB::empty();
    for (int k = lo; k < hi; ++k)
        cb.expand(instanceTlas_[items[k]].worldBox.center());
    const float4 ext = cb.mx - cb.mn;

    if (config_.tlasQuality == TlasQuality::BinnedSAH)
    {
        constexpr int kBins = 16;
        float bestCost = FLT_MAX;
        int   bestAxis = -1, bestBin = -1;

        for (int axis = 0; axis < 3; ++axis)
        {
            const float e = axisOf(ext, axis);
            if (!(e > 0.0f)) continue;
            const float base  = axisOf(cb.mn, axis);
            const float scale = float(kBins) / e;

            AABB binBox[kBins];
            int  binCount[kBins] = {};
            for (int i = 0; i < kBins; ++i) binBox[i] = AABB::empty();
            for (int k = lo; k < hi; ++k)
            {
                const InstanceTlas& in = instanceTlas_[items[k]];
                int b = int((axisOf(in.worldBox.center(), axis) - base) * scale);
                b = b < 0 ? 0 : (b >= kBins ? kBins - 1 : b);
                binBox[b].expand(in.worldBox);
                ++binCount[b];
            }

            float leftArea[kBins];
            int   leftCount[kBins];
            AABB  acc = AABB::empty();
            int   cnt = 0;
            for (int i = 0; i < kBins; ++i)
            {
                acc.expand(binBox[i]);
                cnt += binCount[i];
                leftArea[i]  = surfaceArea(acc);
                leftCount[i] = cnt;
            }

            acc = AABB::empty();
            cnt = 0;
            for (int i = kBins - 1; i >= 1; --i)
            {
                acc.expand(binBox[i]);
                cnt += binCount[i];
                const int l = leftCount[i - 1], r = cnt;
                if (l == 0 || r == 0) continue;
                const float cost =
                    config_.tlasTraversalCost +
                    config_.tlasIntersectCost *
                        (leftArea[i - 1] * float(l) + surfaceArea(acc) * float(r));
                if (cost < bestCost)
                {
                    bestCost = cost;
                    bestAxis = axis;
                    bestBin  = i;
                }
            }
        }

        if (bestAxis >= 0)
        {
            const float base  = axisOf(cb.mn, bestAxis);
            const float scale = float(kBins) / axisOf(ext, bestAxis);
            const auto  mid = std::partition(
                items.begin() + lo, items.begin() + hi,
                [&](uint32_t idx)
                {
                    int b = int(
                        (axisOf(instanceTlas_[idx].worldBox.center(), bestAxis) - base) *
                        scale);
                    b = b < 0 ? 0 : (b >= kBins ? kBins - 1 : b);
                    return b < bestBin;
                });
            const int m = int(mid - items.begin());
            if (m > lo && m < hi) return m;
        }
    }

    const int axis = (ext.x >= ext.y && ext.x >= ext.z) ? 0 : (ext.y >= ext.z ? 1 : 2);
    const int mid = (lo + hi) / 2;
    std::nth_element(items.begin() + lo, items.begin() + mid, items.begin() + hi,
                     [&](uint32_t a, uint32_t b)
                     {
                         return axisOf(instanceTlas_[a].worldBox.center(), axis) <
                                axisOf(instanceTlas_[b].worldBox.center(), axis);
                     });
    return mid;
}

// Recursive 8-way build: three levels of binary splits per node. Slower than
// the Morton path but produces noticeably tighter trees. Used for structural
// rebuilds (instances added/removed), which are rare and long-lived.
int32_t World::tlasBuildRange(std::vector<uint32_t>& items, int lo, int hi, int32_t parent)
{
    const int32_t idx = int32_t(tlasNodes_.size());
    tlasNodes_.emplace_back();
    tlasNodes_[idx].parent = parent;
    for (uint32_t l = 0; l < kWide; ++l)
    {
        tlasNodes_[idx].child[l] = 0;
        tlasNodes_[idx].laneMask[l] = 0;
    }
    tlasNodes_[idx].bounds = WideBounds::allEmpty();
    tlasNodes_[idx].maxErr = float8::splat(0.0f);

    const int count = hi - lo;
    if (count <= int(kWide))
    {
        for (int k = 0; k < count; ++k)
        {
            const uint32_t instIdx = items[lo + k];
            InstanceTlas& inst = instanceTlas_[instIdx];
            TlasNode& n = tlasNodes_[idx];
            n.bounds.setLane(uint32_t(k), inst.worldBox);
            n.maxErr.v[k] = inst.maxErrWorld;
            n.child[k] = ~int32_t(instIdx);
            n.laneMask[k] = inst.mask;
            n.validMask |= 1u << k;
            inst.tlasNode = uint32_t(idx);
            inst.tlasLane = uint32_t(k);
        }
        return idx;
    }

    int cuts[kWide + 1] = {};
    cuts[0] = lo;
    cuts[kWide] = hi;
    cuts[4] = tlasSplit(items, cuts[0], cuts[8]);
    cuts[2] = tlasSplit(items, cuts[0], cuts[4]);
    cuts[6] = tlasSplit(items, cuts[4], cuts[8]);
    cuts[1] = tlasSplit(items, cuts[0], cuts[2]);
    cuts[3] = tlasSplit(items, cuts[2], cuts[4]);
    cuts[5] = tlasSplit(items, cuts[4], cuts[6]);
    cuts[7] = tlasSplit(items, cuts[6], cuts[8]);

    for (uint32_t g = 0; g < kWide; ++g)
    {
        if (cuts[g] >= cuts[g + 1]) continue;
        const int32_t child = tlasBuildRange(items, cuts[g], cuts[g + 1], idx);

        // Union the child's lanes into our lane for it.
        AABB u = AABB::empty();
        float me = 0.0f;
        uint32_t lm = 0;
        const TlasNode& cn = tlasNodes_[child];
        for (uint32_t l = 0; l < kWide; ++l)
        {
            if (!(cn.validMask & (1u << l))) continue;
            u.expand(cn.bounds.lane(l));
            me = std::max(me, cn.maxErr.v[l]);
            lm |= cn.laneMask[l];
        }
        TlasNode& n = tlasNodes_[idx];
        n.bounds.setLane(g, u);
        n.maxErr.v[g] = me;
        n.child[g] = child;
        n.laneMask[g] = lm;
        n.validMask |= 1u << g;
    }
    return idx;
}

// Two-tier rebuild policy:
//  - structural rebuilds (add/remove) take the quality path: rare,
//    long-lived, quality matters (contribution culling leans on tight
//    maxErr/bounds lanes);
//  - motion rebuilds (escape/area threshold) take the Morton path: one sort
//    plus contiguous groups of kWide per level, ~5x faster to build, letting
//    the escape policy rebuild eagerly and keep bloat low under heavy motion.
void World::tlasRebuild()
{
    tlasNodes_.clear();
    tlasFreeNodes_.clear();
    tlasRoot_ = -1;
    tlasEscapes_ = 0;
    tlasEdits_ = 0;
    tlasGrownArea_ = 0.0f;
    tlasDirty_ = false;

    const bool quality = tlasQualityBuild_;
    tlasQualityBuild_ = false;

    if (quality)
    {
        std::vector<uint32_t>& items = tlasItemsTmp_;
        items.assign(liveInstances_.begin(), liveInstances_.end());
        tlasLeafCount_ = uint32_t(items.size());
        tlasQualityCount_ = tlasLeafCount_;
        if (!items.empty())
            tlasRoot_ = tlasBuildRange(items, 0, int(items.size()), -1);
    }
    else
    {
        tlasKeys_.clear();
        AABB cb = AABB::empty();
        for (const InstanceId i : liveInstances_)
            cb.expand(instanceTlas_[i].worldBox.center());
        const float4 lo = cb.mn;
        const float4 ext = cb.extent();
        const float sx = ext.x > 0.0f ? 2097151.0f / ext.x : 0.0f;
        const float sy = ext.y > 0.0f ? 2097151.0f / ext.y : 0.0f;
        const float sz = ext.z > 0.0f ? 2097151.0f / ext.z : 0.0f;
        for (const InstanceId i : liveInstances_)
        {
            const float4 c = instanceTlas_[i].worldBox.center();
            const uint64_t kx = expandBits21(uint64_t((c.x - lo.x) * sx));
            const uint64_t ky = expandBits21(uint64_t((c.y - lo.y) * sy));
            const uint64_t kz = expandBits21(uint64_t((c.z - lo.z) * sz));
            tlasKeys_.push_back({(kx << 2) | (ky << 1) | kz, i});
        }
        tlasLeafCount_ = uint32_t(tlasKeys_.size());
        if (!tlasKeys_.empty())
        {
            std::sort(tlasKeys_.begin(), tlasKeys_.end());

            // Leaf level: consecutive groups of kWide instances.
            std::vector<int32_t>& cur = tlasLevelTmp_;
            cur.clear();
            for (size_t base = 0; base < tlasKeys_.size(); base += kWide)
            {
                const int32_t idx = int32_t(tlasNodes_.size());
                TlasNode& n = tlasNodes_.emplace_back();
                n.bounds = WideBounds::allEmpty();
                n.maxErr = float8::splat(0.0f);
                n.parent = -1;
                for (uint32_t l = 0; l < kWide; ++l)
                {
                    n.child[l] = 0;
                    n.laneMask[l] = 0;
                }
                const uint32_t cnt =
                    uint32_t(std::min<size_t>(kWide, tlasKeys_.size() - base));
                for (uint32_t k = 0; k < cnt; ++k)
                {
                    const uint32_t instIdx = tlasKeys_[base + k].second;
                    InstanceTlas& inst = instanceTlas_[instIdx];
                    n.bounds.setLane(k, inst.worldBox);
                    n.maxErr.v[k] = inst.maxErrWorld;
                    n.child[k] = ~int32_t(instIdx);
                    n.laneMask[k] = inst.mask;
                    n.validMask |= 1u << k;
                    inst.tlasNode = uint32_t(idx);
                    inst.tlasLane = k;
                }
                cur.push_back(idx);
            }

            // Inner levels: group kWide nodes until one remains.
            std::vector<int32_t> next;
            while (cur.size() > 1)
            {
                next.clear();
                for (size_t base = 0; base < cur.size(); base += kWide)
                {
                    const int32_t idx = int32_t(tlasNodes_.size());
                    TlasNode& n = tlasNodes_.emplace_back();
                    n.bounds = WideBounds::allEmpty();
                    n.maxErr = float8::splat(0.0f);
                    n.parent = -1;
                    for (uint32_t l = 0; l < kWide; ++l)
                    {
                        n.child[l] = 0;
                        n.laneMask[l] = 0;
                    }
                    const uint32_t cnt =
                        uint32_t(std::min<size_t>(kWide, cur.size() - base));
                    for (uint32_t k = 0; k < cnt; ++k)
                    {
                        const int32_t childIdx = cur[base + k];
                        TlasNode& cn = tlasNodes_[childIdx];
                        cn.parent = idx;
                        AABB u = AABB::empty();
                        float me = 0.0f;
                        uint32_t lm = 0;
                        for (uint32_t l = 0; l < kWide; ++l)
                        {
                            if (!(cn.validMask & (1u << l))) continue;
                            u.expand(cn.bounds.lane(l));
                            me = std::max(me, cn.maxErr.v[l]);
                            lm |= cn.laneMask[l];
                        }
                        n.bounds.setLane(k, u);
                        n.maxErr.v[k] = me;
                        n.child[k] = childIdx;
                        n.laneMask[k] = lm;
                        n.validMask |= 1u << k;
                    }
                    next.push_back(idx);
                }
                cur.swap(next);
            }
            tlasRoot_ = cur[0];
        }
    }

    // Baseline for the area-drift trigger: the total lane area this build
    // started from. Motion is allowed to add a configured fraction of it
    // before the tree is considered bloated enough to rebuild.
    tlasBaseArea_ = 0.0f;
    for (const TlasNode& n : tlasNodes_)
        for (uint32_t l = 0; l < kWide; ++l)
            if (n.validMask & (1u << l)) tlasBaseArea_ += surfaceArea(n.bounds.lane(l));
}

void World::tlasQuery(const CullView& view, float minPix,
                      std::vector<std::pair<uint32_t, uint8_t>>& outVisible)
{
    outVisible.clear();
    if (tlasDirty_) tlasRebuild();
    if (tlasRoot_ < 0) return;

    const bool useMask = view.viewMask != ~0u;
    const float4 qmn = view.queryMin(), qmx = view.queryMax();

    tlasStack_.clear();
    tlasStack_.push_back({tlasRoot_, kAllPlanes});
    while (!tlasStack_.empty())
    {
        const TlasItem it = tlasStack_.back();
        tlasStack_.pop_back();
        const TlasNode& n = tlasNodes_[it.node];

        uint8_t outMasks[kWide];
        uint32_t survivors =
            testWideAabb(n.bounds, view.frustum, it.mask, outMasks) & n.validMask;
        if (!survivors) continue;

        // Layer visibility. Skipped entirely for the default all-ones view
        // mask, so callers that do not use layers pay one compare per node.
        if (useMask)
        {
            for (uint32_t l = 0; l < kWide; ++l)
                if (!(n.laneMask[l] & view.viewMask)) survivors &= ~(1u << l);
            if (!survivors) continue;
        }

        if (minPix > 0.0f)
        {
            const float8 d2 = distanceToBoxesSq(n.bounds, qmn, qmx);
            const float8 errs = screenErrorFromSq8(n.maxErr, view.k, d2);
            for (uint32_t l = 0; l < kWide; ++l)
                if (errs.v[l] < minPix) survivors &= ~(1u << l);
        }

        while (survivors)
        {
            const uint32_t l = uint32_t(std::countr_zero(survivors));
            survivors &= survivors - 1;
            const int32_t c = n.child[l];
            if (c >= 0)
                tlasStack_.push_back({c, outMasks[l]});
            else
                outVisible.emplace_back(uint32_t(~c), outMasks[l]);
        }
    }
}

// ============================================================================
// garbage collection
// ============================================================================

void World::lruUnlink(uint32_t slot)
{
    PageRt& rt = slots_[slot];
    if (rt.lruPrev != kInvalidIndex) slots_[rt.lruPrev].lruNext = rt.lruNext;
    else if (lruHead_ == slot) lruHead_ = rt.lruNext;
    if (rt.lruNext != kInvalidIndex) slots_[rt.lruNext].lruPrev = rt.lruPrev;
    else if (lruTail_ == slot) lruTail_ = rt.lruPrev;
    rt.lruPrev = rt.lruNext = kInvalidIndex;
}

void World::lruPushFront(uint32_t slot)
{
    PageRt& rt = slots_[slot];
    rt.lruPrev = kInvalidIndex;
    rt.lruNext = lruHead_;
    if (lruHead_ != kInvalidIndex) slots_[lruHead_].lruPrev = slot;
    lruHead_ = slot;
    if (lruTail_ == kInvalidIndex) lruTail_ = slot;
}

void World::lruTouch(uint32_t slot)
{
    PageRt& rt = slots_[slot];
    if (rt.lastTouched == frame_) return;
    rt.lastTouched = frame_;
    if (rt.pinned || lruHead_ == slot) return;
    lruUnlink(slot);
    lruPushFront(slot);
}

size_t World::collect(size_t maxAttachedPages, uint32_t minAge,
                      std::vector<UserPayload>* freedPayloads)
{
    size_t detached = 0;
    uint32_t slot = lruTail_;
    while (streamedPageCount() > maxAttachedPages && slot != kInvalidIndex)
    {
        const uint32_t prev = slots_[slot].lruPrev;
        const PageRt& rt = slots_[slot];
        const bool eligible = rt.inUse && !rt.pinned && rt.attachedChildPages == 0 &&
                              (frame_ - rt.lastTouched) >= minAge;
        if (eligible)
        {
            detachSlot(slot, freedPayloads);
            ++detached;
        }
        slot = prev;
    }
    return detached;
}

// ============================================================================
// cut selection
// ============================================================================

// One SIMD issue per kWide children: masked tri-state frustum, distance and
// screen error, lanes = children. Surviving PLAIN LEAVES are emitted right
// here (they are in both cuts by definition — no visit, no metadata reads);
// surviving interior/expansion nodes go onto the DFS stack with their err and
// narrowed plane mask carried along.
//
// Bounds come through item.wide, which already points at either the page's
// own boxes or this instance's overlay. Everything else is read from the
// shared page. There is no per-block branch for the overlay case: the two
// layouts differ only in stride.
void World::wideVisit(const WorkItem& item, const PageView& pg, float errClamp,
                      uint32_t gen, uint32_t tag, uint32_t node, uint8_t mask,
                      uint8_t aliveKids, const CullView& local, Worker& w,
                      bool wantIdeal)
{
    const uint32_t cc = pg.childCount(node);
    uint32_t b = pg.wideOffset(node);
    const float8 clamp = float8::splat(errClamp);
    // LOD distance is measured to the camera ENVELOPE, not to a point: this
    // is the whole of hysteresis. The envelope collapses to local.pos when
    // damping is off, and then this is bit-identical to a point query.
    const float4 qmn = local.queryMin(), qmx = local.queryMax();
    for (uint32_t base = 0; base < cc; base += kWide, ++b)
    {
        const WideBlock&  blk = pg.wide[b];
        const WideBounds& wb  = item.wide[b];
        // One load carries both lane masks. `survivors` never exceeds 8 bits,
        // so ANDing it with the whole word keeps exactly the valid lanes and
        // the leaf lanes in the high half come along for free.
        const uint32_t lanes = pg.blockMask[b];
        HLOD_STAT(w, wideBlocksTested, 1);
#ifdef HLOD_OPT_MASK64
        uint64_t       outMasks = 0;
        const uint32_t survivors =
            testWideAabbPacked(wb, local.frustum, mask, outMasks) & lanes;
#else
        uint8_t outMasks[kWide];
        const uint32_t survivors =
            testWideAabb(wb, local.frustum, mask, outMasks) & lanes;
#endif
        if (!survivors) continue;
        HLOD_STAT(w, lanesSurvived, uint64_t(std::popcount(survivors)));

        // The clamp is invariant (D) across the page boundary, applied here
        // rather than baked into the page. One vminps; a no-op for root pages
        // where errClamp is FLT_MAX.
        //
        // Squared distance, then one reciprocal square root: never a sqrt and
        // never a divide. See the note on screenErrorFromSq8 for why that is
        // worth more than the arithmetic it saves.
        const float8 eff = min8(blk.error, clamp);
        const float8 d2 = distanceToBoxesSq(wb, qmn, qmx);
        const float8 errs = screenErrorFromSq8(eff, local.k, d2);

        const uint32_t leafLanes = blockLeafLanes(lanes);
        uint32_t leaves = survivors & leafLanes;
        while (leaves)
        {
            const uint32_t l = uint32_t(std::countr_zero(leaves));
            leaves &= leaves - 1;
            const uint32_t c = blk.child[l];
            if (aliveKids) w.cut.push({pg.payload[c], errs.v[l], tag});
            if (wantIdeal)
                w.ideal.push({pg.payload[c], NodeHandle{item.slot, c, gen}, errs.v[l],
                              tag, IdealTag::Direct});
        }

        uint32_t inner = survivors & ~leafLanes;
        while (inner)
        {
            const uint32_t l = uint32_t(std::countr_zero(inner));
            inner &= inner - 1;
            const uint32_t c = blk.child[l];
#ifdef HLOD_OPT_MASK64
            const uint8_t planes = uint8_t(outMasks >> (8 * l));
#else
            const uint8_t planes = outMasks[l];
#endif
            w.nodeStack.push_back({c, errs.v[l], planes, aliveKids});

            // This lane is the only kind that gets DECIDED: runPage will ask
            // whether its error clears the bar, and plain leaves (handled
            // above) are emitted without asking. The answer flips when the
            // distance reaches eff * k / bar, so the gap between where this
            // node is and where that happens is how far the camera may travel
            // before this instance's cut could differ. See SelectionContext.
            if (w.trackMargin)
            {
                w.maxError = std::max(w.maxError, eff.v[l]);
                const float flipAt = eff.v[l] * local.k / w.bar;
                const float d = std::sqrt(d2.v[l]);
                const float slack = d > flipAt ? d - flipAt : flipAt - d;
                if (slack < w.margin) w.margin = slack;
            }
        }
    }
}

void World::pushLoadRequests(PageRt& rt, uint32_t slot, uint32_t node, float priority,
                             Worker& w)
{
    const PageView& pg = rt.page;
    if (rt.reqStamp.size() != pg.nodeCount()) rt.reqStamp.assign(pg.nodeCount(), 0);

    // With a shared asset, every one of its instances reaches this same node
    // and wants the same content. Stamping the node with this pass's epoch
    // collapses those thousands of identical requests into one, keeping the
    // highest priority any instance asked for.
    const uint64_t epoch = uint64_t(w.epoch) << 32;
    uint32_t c = node + 1;
    for (uint32_t k = 0; k < pg.childCount(node); ++k)
    {
        if (!rt.resident[c])
        {
            // Relaxed atomics so the parallel path is race-free; on every
            // target we care about these compile to plain loads and stores.
            std::atomic_ref<uint64_t> stamp(rt.reqStamp[c]);
            const uint64_t prev = stamp.load(std::memory_order_relaxed);
            if ((prev & 0xFFFFFFFF00000000ull) == epoch)
            {
                if (LoadRequest* r = w.requests.at(uint32_t(prev)))
                    if (r->priority < priority) r->priority = priority;
            }
            else
            {
                stamp.store(epoch | uint64_t(w.requests.count()),
                            std::memory_order_relaxed);
                w.requests.push(
                    {pg.payload[c], NodeHandle{slot, c, rt.generation}, priority});
            }
        }
        c += pg.subtreeSize[c];
    }
}

void World::runPage(const WorkItem& item, const Instance& inst, const CullView& local,
                    const CutParams& params, Worker& w, bool wantIdeal,
                    bool wantRequests)
{
    PageRt& rt = slots_[item.slot];
    if (w.deferTouch)
        w.touched.push_back(item.slot);
    else
        lruTouch(item.slot);

    HLOD_STAT(w, pagesVisited, 1);
    const PageView& pg = rt.page;
    const uint32_t gen = rt.generation;
    const uint32_t tag = inst.tag;
    // One bar, no history: damping is already folded into the view's camera
    // envelope, which widened the measured error rather than moving the
    // threshold. That is what makes selection a pure read of the World.
    const float bar = params.threshold;

    w.nodeStack.clear();
    wideVisit(item, pg, rt.errClamp, gen, tag, 0, item.mask, item.alive, local, w,
              wantIdeal);

    while (!w.nodeStack.empty())
    {
        const NodeItem e = w.nodeStack.back();
        w.nodeStack.pop_back();
        const uint32_t i = e.node;
        HLOD_STAT(w, nodesVisited, 1);

        // Every stacked node has kids (plain leaves were emitted by the
        // parent's wide test), so `wants` is just the error test.
        if (!(e.err > bar))   // both cuts end here; a good-enough collapsed proxy is DIRECT
        {
            if (e.alive) w.cut.push({pg.payload[i], e.err, tag});
            if (wantIdeal)
                w.ideal.push({pg.payload[i], NodeHandle{item.slot, i, gen}, e.err, tag,
                              IdealTag::Direct});
            continue;
        }

        const uint32_t m = pg.meta[i];
        const bool exp = metaIsExpansion(m);

        const uint32_t childSlot =
            (exp && !rt.expSlot.empty()) ? rt.expSlot[i] : kInvalidIndex;

        if (exp && childSlot == kInvalidIndex)   // collapsed AND too coarse
        {
            if (e.alive) w.cut.push({pg.payload[i], e.err, tag});
            if (wantIdeal)
                w.ideal.push({pg.payload[i], NodeHandle{item.slot, i, gen}, e.err, tag,
                              IdealTag::NeedsExpansion});
            continue;
        }

        bool ready;
        if (exp)
        {
            const PageRt& crt = slots_[childSlot];
            ready = crt.readyChildren[0] == crt.page.childCount(0);
        }
        else
        {
            ready = rt.readyChildren[i] == metaChildCount(m);
        }

        if (!ready && e.alive)   // actual cut stops here; ideal descends
        {
            w.cut.push({pg.payload[i], e.err, tag});
            if (wantRequests)
                pushLoadRequests(exp ? slots_[childSlot] : rt,
                                 exp ? childSlot : item.slot, exp ? 0u : i, e.err, w);
        }

        const uint8_t a2 = uint8_t(e.alive && ready);
        if (exp)
            w.work.push_back({childSlot,
                              wideBoundsFor(inst, childSlot, slots_[childSlot]), a2,
                              e.planes});
        else
            wideVisit(item, pg, rt.errClamp, gen, tag, i, e.planes, a2, local, w,
                      wantIdeal);
    }
}

void World::runInstance(uint32_t instIdx, const CullView& view, const CutParams& params,
                        uint8_t mask, Worker& w, bool wantIdeal, bool wantRequests)
{
    const Instance& inst = instances_[instIdx];
    const CullView local = toLocal(view, inst.pos, inst.scale);
    HLOD_STAT(w, instancesVisited, 1);
    w.work.push_back({inst.rootSlot,
                      wideBoundsFor(inst, inst.rootSlot, slots_[inst.rootSlot]), 1,
                      mask});
    while (!w.work.empty())
    {
        const WorkItem item = w.work.back();
        w.work.pop_back();
        runPage(item, inst, local, params, w, wantIdeal, wantRequests);
    }
}

void World::selectCut(const CullView& view, const CutParams& params, CutSink& outCut,
                      IdealSink* outIdealCut, RequestSink* outRequests)
{
    // The one place the tree must be up to date: apply pending bounds edits.
    flushBounds();

    ++selectEpoch_;
    stats_ = CutStats{};

    tlasQuery(view, params.minPix, visibleTmp_);

    const uint32_t nVis = uint32_t(visibleTmp_.size());
    const bool wantIdeal = outIdealCut != nullptr;
    const bool wantRequests = outRequests != nullptr;

    const uint32_t workerCount = config_.context.workerCount;
    const bool parallel = config_.parallelInstanceThreshold > 0 && workerCount > 1 &&
                          nVis >= config_.parallelInstanceThreshold;
    // One request-stamp epoch per worker plus one for the post-join merge.
    // The extra stride keeps the merge epoch distinct from the next call's
    // worker zero epoch.
    const uint32_t epochBase = selectEpoch_ * (workerCount + 1);

    if (!parallel)
    {
        Worker& w = workers_[0];
        w.work.clear();
        w.nodeStack.clear();
        w.touched.clear();
        w.deferTouch = false;
        w.epoch = epochBase;
        w.cut = outCut;
        if (wantIdeal) w.ideal = *outIdealCut;
        if (wantRequests) w.requests = *outRequests;
        w.stats = CutStats{};

        // The per-instance walk is a chain of dependent loads (Instance ->
        // page slot -> wide block); with tens of thousands of visible
        // instances that chain is memory-latency-bound. Pipeline it: prefetch
        // instance i+2's record and instance i+1's root page while working on
        // instance i.
        for (uint32_t i = 0; i < nVis; ++i)
        {
            if (i + 2 < nVis) HLOD_PREFETCH(&instances_[visibleTmp_[i + 2].first]);
            if (i + 1 < nVis)
            {
                const Instance& next = instances_[visibleTmp_[i + 1].first];
                const PageRt& nrt = slots_[next.rootSlot];
                HLOD_PREFETCH(nrt.page.wide);
                HLOD_PREFETCH(nrt.page.meta);
                HLOD_PREFETCH(nrt.page.payload);
            }
            runInstance(visibleTmp_[i].first, view, params, visibleTmp_[i].second, w,
                        wantIdeal, wantRequests);
        }

        outCut = w.cut;
        if (wantIdeal) *outIdealCut = w.ideal;
        if (wantRequests) *outRequests = w.requests;
        stats_ = w.stats;
        return;
    }

    // ---- parallel selection -------------------------------------------------
    // Each worker takes a contiguous run of visible instances and fills its
    // own buffers, so concatenating in worker order reproduces the serial
    // order exactly — the cut is bit-identical whether or not this path runs.
    if (workers_.size() < workerCount) workers_.resize(workerCount);

    // Request dedup stamps are written during the walk; size them up front so
    // no worker has to allocate inside the parallel region.
    if (wantRequests)
        for (PageRt& rt : slots_)
            if (rt.inUse && rt.reqStamp.size() != rt.page.nodeCount())
                rt.reqStamp.assign(rt.page.nodeCount(), 0);

    struct Chunk
    {
        World*           world;
        const CullView*  view;
        const CutParams* params;
        uint32_t         nVis;
        uint32_t         workerCount;
        bool             wantIdeal;
        bool             wantRequests;
    } chunk{this, &view, &params, nVis, workerCount, wantIdeal, wantRequests};

    for (uint32_t k = 0; k < workerCount; ++k)
    {
        Worker& w = workers_[k];
        w.work.clear();
        w.nodeStack.clear();
        w.touched.clear();
        w.deferTouch = true;
        w.epoch = epochBase + k;
        w.cutBuf.clear();
        w.idealBuf.clear();
        w.reqBuf.clear();
        w.cut = CutSink(w.cutBuf);
        w.ideal = IdealSink(w.idealBuf);
        w.requests = RequestSink(w.reqBuf);
        w.stats = CutStats{};
    }

    config_.context.parallelFor(
        workerCount,
        [](uint32_t k, void* payload)
        {
            auto* c = static_cast<Chunk*>(payload);
            World& world = *c->world;
            const uint32_t per = (c->nVis + c->workerCount - 1) / c->workerCount;
            const uint32_t lo = std::min(k * per, c->nVis);
            const uint32_t hi = std::min(lo + per, c->nVis);
            Worker& w = world.workers_[k];
            for (uint32_t i = lo; i < hi; ++i)
                world.runInstance(world.visibleTmp_[i].first, *c->view, *c->params,
                                  world.visibleTmp_[i].second, w, c->wantIdeal,
                                  c->wantRequests);
        },
        &chunk, config_.context.user);

    const uint64_t mergeEpoch = uint64_t(epochBase + workerCount) << 32;
    for (uint32_t k = 0; k < workerCount; ++k)
    {
        Worker& w = workers_[k];
        for (const CutEntry& e : w.cutBuf) outCut.push(e);
        if (wantIdeal)
            for (const IdealEntry& e : w.idealBuf) outIdealCut->push(e);
        if (wantRequests)
            for (const LoadRequest& e : w.reqBuf)
            {
                PageRt& rt = slots_[e.node.slot];
                std::atomic_ref<uint64_t> stamp(rt.reqStamp[e.node.index]);
                const uint64_t prev = stamp.load(std::memory_order_relaxed);
                if ((prev & 0xFFFFFFFF00000000ull) == mergeEpoch)
                {
                    if (LoadRequest* prior = outRequests->at(uint32_t(prev)))
                        if (prior->priority < e.priority) prior->priority = e.priority;
                }
                else
                {
                    stamp.store(mergeEpoch | uint64_t(outRequests->count()),
                                std::memory_order_relaxed);
                    outRequests->push(e);
                }
            }
        for (const uint32_t slot : w.touched) lruTouch(slot);
#ifdef HLOD_STATS
        stats_.instancesVisited += w.stats.instancesVisited;
        stats_.pagesVisited += w.stats.pagesVisited;
        stats_.nodesVisited += w.stats.nodesVisited;
        stats_.wideBlocksTested += w.stats.wideBlocksTested;
        stats_.lanesSurvived += w.stats.lanesSurvived;
#endif
    }
}

// ---------------------------------------------------------------------------
// Cached selection
// ---------------------------------------------------------------------------

void SelectionContext::reset()
{
    rec_.clear();
    store_.clear();
    used_ = garbage_ = reused_ = walked_ = 0;
    travel_ = kTravel_ = 0.0f;
    primed_ = false;
    k_ = bar_ = 0.0f;
    ++epoch_;
    // The half-life is configuration and survives; the accumulated window is
    // state and does not. This is the half that reset() exists for: records
    // would have expired on their own, an envelope stretched across a teleport
    // would not.
    damper_.reset();
}

size_t SelectionContext::bytes() const
{
    return rec_.capacity() * sizeof(Rec) + store_.capacity() * sizeof(CutEntry);
}

// Runs are allocated by bumping and abandoned when an instance's cut outgrows
// its block, so the slab accumulates holes. Squeeze them out once the holes
// outweigh the live data. Records keep their contents; only `begin` moves.
void SelectionContext::compact()
{
    // Record ids and slab-allocation order are unrelated (the latter follows
    // TLAS traversal order).  Compacting in-place while iterating rec_ could
    // therefore move one run over the still-unread source of another run.
    // Compaction is deliberately rare, so use a same-sized scratch slab and
    // keep the existing allocation headroom while making the copy order moot.
    std::vector<CutEntry> packed(store_.size());
    uint32_t w = 0;
    for (Rec& r : rec_)
    {
        if (r.capacity == 0) continue;
        if (r.validUntil <= travel_ + r.kSlope * kTravel_ || r.epoch != epoch_)
        {
            // Not reusable anyway: drop the block rather than move it.
            r.capacity = r.count = 0;
            continue;
        }
        if (r.count)
            std::memcpy(packed.data() + w, store_.data() + r.begin,
                        size_t(r.count) * sizeof(CutEntry));
        r.begin = w;
        r.capacity = r.count;
        w += r.count;
    }
    store_.swap(packed);
    used_ = w;
    garbage_ = 0;
}

void World::selectCut(const CullView& view, const CutParams& params,
                      SelectionContext& ctx, CutSink& outCut,
                      IdealSink* outIdealCut, RequestSink* outRequests)
{
    flushBounds();
    ++selectEpoch_;
    stats_ = CutStats{};

    // This overload owns the hysteresis, so it takes the raw view and damps it
    // here. Everything below -- the cull, the walk, the odometer -- sees `dv`
    // and only `dv`, which is what makes the reuse argument about the envelope
    // rather than about the camera.
    const CullView dv = ctx.damper_.damp(view);

    tlasQuery(dv, params.minPix, visibleTmp_);

    const bool wantIdeal = outIdealCut != nullptr;
    const bool wantRequests = outRequests != nullptr;

    IdealSink   ideal;
    RequestSink req;
    if (wantIdeal) ideal = *outIdealCut;
    if (wantRequests) req = *outRequests;

    ctx.reused_ = ctx.walked_ = 0;
    if (ctx.rec_.size() < instances_.size()) ctx.rec_.resize(instances_.size());

    // How far the query envelope moved since the last call, added to this
    // view's odometer. One number for the whole frame; every record's validity
    // is then a single compare against it.
    const float4 qmn = dv.queryMin(), qmx = dv.queryMax();
    if (ctx.primed_)
    {
        const float4 dmn = max4(qmn - ctx.lastQmn_, ctx.lastQmn_ - qmn);
        const float4 dmx = max4(qmx - ctx.lastQmx_, ctx.lastQmx_ - qmx);
        ctx.travel_ += length3(max4(dmn, dmx));
    }
    ctx.lastQmn_ = qmn;
    ctx.lastQmx_ = qmx;
    ctx.primed_ = true;

    // Threshold changes alter every record's slope and still invalidate the
    // cache in O(1). Projection-scale changes instead feed an odometer: each
    // record bounds how far any flip point can move per unit k, so gradual
    // damped zoom consumes its margin instead of voiding the whole cache.
    if (ctx.bar_ != params.threshold)
    {
        ++ctx.epoch_;
        ctx.bar_ = params.threshold;
        ctx.kTravel_ = 0.0f;
    }
    else
        ctx.kTravel_ += std::fabs(dv.k - ctx.k_);
    ctx.k_ = dv.k;

    // The one safe moment to squeeze the slab: before any offset recorded this
    // pass could be moved out from under us.
    if (ctx.garbage_ > ctx.used_ / 2) ctx.compact();

    Worker& w = workers_[0];
    w.work.clear();
    w.nodeStack.clear();
    w.epoch = selectEpoch_ * (config_.context.workerCount + 1);
    w.ideal = ideal;
    w.requests = req;
    w.stats = CutStats{};
    // Pages are LRU-touched by hand below, because a reused instance still
    // has to keep its pages warm: skipping the walk must not make the
    // collector think the page went cold.
    w.deferTouch = true;
    w.bar = params.threshold;

    const uint32_t nVis = uint32_t(visibleTmp_.size());
    for (uint32_t i = 0; i < nVis; ++i)
    {
        const uint32_t instIdx = visibleTmp_[i].first;
        const uint8_t  mask = visibleTmp_[i].second;
        const Instance& inst = instances_[instIdx];
        SelectionContext::Rec& r = ctx.rec_[instIdx];

        // Everything the record was taken under, re-checked, in one cache
        // line. `mask == 0` is the frustum condition: this instance is wholly
        // inside, so no plane was tested anywhere within it and camera
        // rotation cannot matter. `travel_ < validUntil` is the margin.
        bool hit = !wantIdeal && mask == 0 &&
                   ctx.travel_ + r.kSlope * ctx.kTravel_ < r.validUntil &&
                   r.epoch == ctx.epoch_ && r.cutVersion == inst.cutVersion;
        if (hit)
            for (uint32_t d = 0; d < r.depCount; ++d)
            {
                const PageRt& rt = slots_[r.depSlot[d]];
                if (!rt.inUse || rt.contentVersion != r.depVersion[d])
                {
                    hit = false;
                    break;
                }
            }

        if (hit)
        {
            for (uint32_t k = 0; k < r.depCount; ++k) lruTouch(r.depSlot[k]);
            // The whole saving is the walk that did not happen. Copying the
            // recorded entries out is ~1.5% of the call at 80k instances, and
            // handing back a descriptor instead measured no better while
            // costing the caller an indirection: see SelectionContext.
            outCut.pushRange(ctx.store_.data() + r.begin, r.count);
            ++ctx.reused_;
            continue;
        }

        // ---- walk it ----
        w.cutBuf.clear();
        w.cut = CutSink(w.cutBuf);
        w.touched.clear();
        w.margin = FLT_MAX;
        w.maxError = 0.0f;
        w.trackMargin = true;
        const uint32_t reqBefore = wantRequests ? w.requests.count() : 0;
        runInstance(instIdx, dv, params, mask, w, wantIdeal, wantRequests);
        w.trackMargin = false;
        for (const uint32_t slot : w.touched) lruTouch(slot);

        // Eligible only if the instance has converged (nothing left to stream)
        // and it fits the record's fixed dependency list. Otherwise it is
        // emitted now and walked again next frame.
        const bool eligible = !wantIdeal && mask == 0 &&
                              w.touched.size() <= SelectionContext::kMaxDeps &&
                              (!wantRequests || w.requests.count() == reqBefore);

        const uint32_t n = uint32_t(w.cutBuf.size());
        if (r.capacity < n)
        {
            ctx.garbage_ += r.capacity;
            if (size_t(ctx.used_) + n > ctx.store_.size())
                ctx.store_.resize(
                    std::max<size_t>(size_t(ctx.used_) + n, ctx.store_.size() * 2 + 256));
            r.begin = ctx.used_;
            r.capacity = n;
            ctx.used_ += n;
        }
        r.count = n;
        if (n) std::memcpy(ctx.store_.data() + r.begin, w.cutBuf.data(),
                           size_t(n) * sizeof(CutEntry));

        if (eligible)
        {
            // The margin is measured in the instance's own space, where the
            // walk measures distances; the odometer runs in world space, so
            // scale it across. Anything non-finite (nothing was ever decided,
            // so nothing can flip) becomes an unbounded budget.
            const float m = w.margin * inst.scale;
            r.kSlope = w.maxError * inst.scale / params.threshold;
            const float consumed = ctx.travel_ + r.kSlope * ctx.kTravel_;
            r.validUntil = m >= FLT_MAX - consumed ? FLT_MAX : consumed + m;
            r.epoch = ctx.epoch_;
            r.cutVersion = inst.cutVersion;
            r.depCount = uint32_t(w.touched.size());
            for (uint32_t d = 0; d < r.depCount; ++d)
            {
                r.depSlot[d] = w.touched[d];
                r.depVersion[d] = slots_[w.touched[d]].contentVersion;
            }
        }
        else
        {
            r.validUntil = 0.0f;
        }

        // From the walk buffer, not from the slab: same bytes, still hot.
        outCut.pushRange(w.cutBuf.data(), n);
        ++ctx.walked_;
    }

    w.deferTouch = false;
    w.cut = CutSink{};
    if (wantIdeal) *outIdealCut = w.ideal;
    if (wantRequests) *outRequests = w.requests;
    stats_ = w.stats;
}

void World::selectCut(const CullView& view, const CutParams& params,
                      SelectionContext&          ctx,
                      std::vector<CutEntry>&     outCut,
                      std::vector<IdealEntry>*   outIdealCut,
                      std::vector<LoadRequest>*  outRequests)
{
    CutSink     cut(outCut);
    IdealSink   ideal;
    RequestSink req;
    if (outIdealCut) ideal = IdealSink(*outIdealCut);
    if (outRequests) req = RequestSink(*outRequests);
    selectCut(view, params, ctx, cut, outIdealCut ? &ideal : nullptr,
              outRequests ? &req : nullptr);
}

void World::selectCut(const CullView& view, const CutParams& params,
                      std::vector<CutEntry>&    outCut,
                      std::vector<IdealEntry>*  outIdealCut,
                      std::vector<LoadRequest>* outRequests)
{
    CutSink     cut(outCut);
    IdealSink   ideal;
    RequestSink req;
    if (outIdealCut) ideal = IdealSink(*outIdealCut);
    if (outRequests) req = RequestSink(*outRequests);
    selectCut(view, params, cut, outIdealCut ? &ideal : nullptr,
              outRequests ? &req : nullptr);
}

} // namespace hlod
