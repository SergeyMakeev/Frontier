#include "hlod/world.h"

#include <algorithm>
#include <array>
#include <bit>
#include <memory>

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

inline uint32_t nextPageGeneration(uint32_t generation)
{
    generation = (generation + 1u) & NodeHandle::kGenerationMask;
    return generation == 0 ? 1u : generation;
}

inline uint32_t cutCount(uint32_t counts, uint32_t bucket)
{
    return (counts >> (bucket * 10)) & 0x3ffu;
}

inline uint32_t cutTotal(uint32_t counts)
{
    return cutCount(counts, 0) + cutCount(counts, 1) + cutCount(counts, 2);
}

inline uint32_t cutDepCount(uint32_t counts)
{
    return counts >> 30;
}

inline uint32_t packCutCounts(uint32_t shared, uint32_t currentOnly,
                              uint32_t idealOnly, uint32_t depCount)
{
    return shared | (currentOnly << 10) | (idealOnly << 20) | (depCount << 30);
}

inline uint8_t encodeCutErrorRatio(float ratio, bool above)
{
    if (!std::isfinite(ratio)) return above ? 255 : 127;
    if (!(ratio > 0.0f)) return 0;

    const uint32_t bits = std::bit_cast<uint32_t>(ratio);
    const uint32_t biased = (bits >> 23) & 0xffu;
    if (biased == 0) return above ? 128 : 0;
    const int exponent = int(biased) - 127;
    const int mantissa = int((bits >> 20) & 7u);
    int code = 128 + exponent * 8 + mantissa;
    code = std::clamp(code, 0, 255);
    code = above ? std::max(code, 128) : std::min(code, 127);
    return uint8_t(code);
}

} // namespace

uint8_t encodeCutError(float error, float threshold)
{
    if (!(error > 0.0f)) return 0;
    if (!(threshold > 0.0f)) return error > threshold ? 255 : 127;

    const bool above = error > threshold;
    return encodeCutErrorRatio(error * (1.0f / threshold), above);
}

float decodeCutError(uint8_t code, float threshold)
{
    if (!(threshold > 0.0f)) return threshold;
    const int q = code < kCutErrorThreshold ? int(code) - 127
                                             : int(code) - 128;
    return threshold * std::exp2(float(q) * (1.0f / 8.0f));
}

namespace {

inline constexpr uint32_t kFlatZeroError = 1u << 31;

inline CutEntry makeCutEntry(NodeHandle node, float error, float threshold,
                             float thresholdInv, InstanceId instance)
{
    if (!(error > 0.0f)) return CutEntry{node, uint8_t(0), instance};
    if (!(threshold > 0.0f))
        return CutEntry{node, uint8_t(error > threshold ? 255 : 127), instance};
    return CutEntry{node,
                    encodeCutErrorRatio(error * thresholdInv, error > threshold),
                    instance};
}

} // namespace

// Opaque per-view query state. Cached and uncached selection both use it;
// ownership by View is what makes every query a read-only World operation.
struct ViewScratch
{
    std::vector<World::Worker>      workers{1};
    std::vector<World::VisibleItem> visible;
    std::vector<World::TlasItem>    tlasStack;
    detail::CutBuffers              output;

    size_t bytes() const
    {
        size_t n = visible.capacity() * sizeof(visible[0]) +
                   tlasStack.capacity() * sizeof(tlasStack[0]) +
                   workers.capacity() * sizeof(World::Worker) +
                   output.shared.capacity() * sizeof(CutEntry) +
                   output.currentOnly.capacity() * sizeof(CutEntry) +
                   output.idealOnly.capacity() * sizeof(CutEntry);
        for (const World::Worker& w : workers)
        {
            n += w.work.capacity() * sizeof(World::WorkItem);
            n += w.nodeStack.capacity() * sizeof(World::NodeItem);
            n += w.cutBuf.shared.capacity() * sizeof(CutEntry);
            n += w.cutBuf.currentOnly.capacity() * sizeof(CutEntry);
            n += w.cutBuf.idealOnly.capacity() * sizeof(CutEntry);
            n += w.touched.capacity() * sizeof(uint32_t);
        }
        return n;
    }
};

View::View()
    : scratch_(std::make_unique<ViewScratch>())
{}

View::View(float halfLifeFrames)
    : damper_(halfLifeFrames), scratch_(std::make_unique<ViewScratch>())
{}

View::~View() = default;
View::View(View&&) noexcept = default;
View& View::operator=(View&&) noexcept = default;

void PageUsageContext::reset()
{
    world_ = nullptr;
    rec_.clear();
    dirty_.clear();
}

size_t PageUsageContext::bytes() const
{
    return rec_.capacity() * sizeof(Rec) + dirty_.capacity() * sizeof(uint32_t);
}

World::World(const WorldConfig& config) : config_(config)
{
    if (config_.context.workerCount == 0) config_.context.workerCount = 1;
}

World::~World() = default;

// ============================================================================
// handle resolution — two loads and three compares, no hashing anywhere
// ============================================================================

const World::PageRt* World::resolve(NodeHandle h) const
{
    const uint32_t slot = h.slot();
    const uint32_t index = h.index();
    if (slot >= slots_.size()) return nullptr;
    const PageStamp& stamp = pageStamps_[slot];
    if (!stamp.inUse() || stamp.generation() != h.generation()) return nullptr;
    const PageRt& rt = slots_[slot];
    if (index == 0 || index >= pageView(rt).nodeCount()) return nullptr;
    return &rt;
}

const World::AssetRt* World::resolveAsset(AssetHandle h) const
{
    if (h.slot >= assets_.size()) return nullptr;
    const AssetRt& as = assets_[h.slot];
    if (!as.inUse() || as.generation != h.generation) return nullptr;
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

uint32_t World::createAsset(Page&& page, bool registered)
{
    const uint32_t a = allocAsset();
    AssetRt& as = assets_[a];
    as = AssetRt{};
    // Every pointer in Page points into the blob, not into AssetRt, so moving
    // this vector never invalidates mounted page data.
    as.page       = std::move(page);
    as.generation = ++generationCounter_;
    as.setRegistered(registered);
    ++liveAssets_;
    return a;
}

void World::destroyAssetIfUnused(uint32_t a)
{
    AssetRt& as = assets_[a];
    if (!as.inUse()) return;
    if (as.registered() || as.mountRefs() > 0 || as.instanceRefs > 0) return;
    as = AssetRt{};   // releases the blob if the World owned it
    freeAssets_.push_back(a);
    --liveAssets_;
}

AssetHandle World::registerAsset(Page&& page)
{
    HLOD_CHECK(page.nodeCount() > 1, "World::registerAsset: empty page");
    const uint32_t a = createAsset(std::move(page), true);
    return AssetHandle{a, assets_[a].generation};
}

AssetHandle World::registerAsset(PageView borrowedPage)
{
    HLOD_CHECK(borrowedPage.valid() && borrowedPage.nodeCount() > 1,
               "World::registerAsset: empty page");
    const uint32_t a = createAsset(Page::borrow(borrowedPage), true);
    return AssetHandle{a, assets_[a].generation};
}

void World::releaseAsset(AssetHandle h)
{
    if (!resolveAsset(h)) return;   // already released; stale handles are quiet
    const uint32_t a = h.slot;
    HLOD_CHECK(assets_[a].instanceRefs == 0,
               "World::releaseAsset: instances still reference this asset");
    assets_[a].setRegistered(false);
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
    return PageHandle{as->rootMount, pageStamps_[as->rootMount].generation()};
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
    HLOD_CHECK(slots_.size() < NodeHandle::kInvalidSlot,
               "World: exhausted the 20-bit page-mount slot space");
    slots_.emplace_back();
    pageStamps_.emplace_back();
    pageResidency_.emplace_back();
    return uint32_t(slots_.size() - 1);
}

uint32_t World::registerPage(uint32_t asset, NodeRef owner, bool pinned)
{
    const uint32_t slot = allocSlot();
    AssetRt& as = assets_[asset];
    PageRt& rt = slots_[slot];
    PageStamp& stamp = pageStamps_[slot];
    pageResidency_[slot] = PageResidency{};

    HLOD_CHECK(as.page.nodeCount() <= (1u << NodeHandle::kIndexBits),
               "World: page exceeds the 20-bit page-local node space");
    rt.asset    = asset;
    rt.errClamp = FLT_MAX;
    rt.nodeState.assign(as.page.nodeCount(), 0);
    rt.coveredChildren.assign(as.page.nodeCount(), 0);
    rt.expSlot.clear();
    const uint32_t generation = nextPageGeneration(stamp.generation());
    stamp.setGeneration(generation);
    rt.setGeneration(generation);
    // Bumped here as well as on every content change, so that a slot which was
    // detached and reused can never present the same contentVersion as before.
    // That is what lets a View record identify a page's state with a
    // single word instead of a (generation, version) pair.
    ++stamp.contentVersion;
    stamp.setInUse(true);
    rt.lastTouched = frame_;
    rt.attachedChildrenAndPinned = 0;
    rt.setPinned(pinned);
    rt.owner = owner;

    as.addMountRef();
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
    const PageView& page = pageView(rt);
    uint32_t c = 1;
    for (uint32_t k = 0; k < page.childCount(0); ++k)
    {
        rt.setResident(c, true);
        ++pageResidency_[slot].residentNodes;
        propagateCoverage(slot, c);
        c += page.subtreeSize[c];
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
    const NodeRef owner{expansionNode.slot(), expansionNode.index()};
    {
        const PageRt& ownerRt = slots_[owner.slot];
        HLOD_CHECK(pageView(ownerRt).isExpansion(owner.index),
                   "World::attachPage: not an expansion point");
        HLOD_CHECK(ownerRt.expSlot.empty() || ownerRt.expSlot[owner.index] == kInvalidIndex,
                   "World::attachPage: already attached");
    }
    HLOD_CHECK(assets_[asset].page.nodeCount() > 1, "World::attachPage: empty page");

    // (C) across the boundary: the owner must contain the page's content.
    // Growing the owner here is not an option — its bytes back every instance
    // of the owning asset, so the growth would have to ripple into all of
    // their top-level bounds, and a streaming event would silently become an
    // O(instances) write. Author expansion bounds that contain what attaches.
    HLOD_CHECK(pageView(slots_[owner.slot]).bbox[owner.index].contains(
                   assets_[asset].page.bbox[0]),
               "World::attachPage: the attached page escapes the expansion node's "
               "authored bounds — author conservative expansion bounds at build time");

    // (D) across the boundary: the child page's effective error ceiling is
    // the owner expansion node's own effective error. Carried as a scalar and
    // folded into the wide test, so the page's bytes are never touched — the
    // same asset can hang under a dozen different expansion points, each with
    // its own ceiling, and attach stays O(1) instead of O(nodeCount).
    const float childClamp =
        std::min(pageView(slots_[owner.slot]).geometricError[owner.index],
                 slots_[owner.slot].errClamp);

    // NOTE: registerPage can reallocate slots_, so nothing above may be held
    // as a reference across this call.
    const uint32_t slot = registerPage(asset, owner, false);
    slots_[slot].errClamp = childClamp;

    PageRt& ort = slots_[owner.slot];
    const bool ownerWasFullyResident = pageTreeFullyResident(owner.slot);
    if (ort.expSlot.empty())
        ort.expSlot.assign(pageView(ort).nodeCount(), kInvalidIndex);
    ort.expSlot[owner.index] = slot;
    ort.addAttachedChild();
    ++pageResidency_[owner.slot].incompleteChildren;
    propagateFullResidency(owner.slot, ownerWasFullyResident);
    ++pageStamps_[owner.slot].contentVersion;   // this page now refines further

    return PageHandle{slot, pageStamps_[slot].generation()};
}

PageHandle World::attachPage(NodeHandle expansionNode, Page&& page)
{
    // Check the handle before taking ownership so a stale attach leaves the
    // caller's page untouched (they just drop it).
    if (!resolve(expansionNode)) return PageHandle{};
    HLOD_CHECK(page.nodeCount() > 1, "World::attachPage: empty page");

    const uint32_t a = createAsset(std::move(page), false);
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
                               : ownerRt->expSlot[expansionNode.index()];
    HLOD_CHECK(child != kInvalidIndex, "World::detachPage: not attached");
    PageRt& rt = slots_[child];
    HLOD_CHECK(rt.attachedChildPages() == 0,
               "World::detachPage: attached child pages remain");
    HLOD_CHECK(!rt.pinned(), "World::detachPage: page is pinned");
    detachSlot(child, nullptr);
}

bool World::isAttached(NodeHandle expansionNode) const
{
    const PageRt* rt = resolve(expansionNode);
    return rt && !rt->expSlot.empty() &&
           rt->expSlot[expansionNode.index()] != kInvalidIndex;
}

void World::detachSlot(uint32_t slot, AppendBuffer<UserPayload>* freedPayloads)
{
    PageRt& rt = slots_[slot];
    if (freedPayloads)
        for (uint32_t i = 1; i < pageView(rt).nodeCount(); ++i)
            if (rt.isResident(i)) freedPayloads->push_back(pageView(rt).payload[i]);
    if (rt.owner.valid())
    {
        PageRt& ownerRt = slots_[rt.owner.slot];
        const bool ownerWasFullyResident = pageTreeFullyResident(rt.owner.slot);
        if (!pageTreeFullyResident(slot))
        {
            HLOD_CHECK(pageResidency_[rt.owner.slot].incompleteChildren != 0,
                       "World: page residency summary underflow");
            --pageResidency_[rt.owner.slot].incompleteChildren;
        }
        ownerRt.expSlot[rt.owner.index] = kInvalidIndex;
        ownerRt.removeAttachedChild();
        propagateFullResidency(rt.owner.slot, ownerWasFullyResident);
        ++pageStamps_[rt.owner.slot].contentVersion;   // it collapses to a leaf
        propagateCoverage(rt.owner.slot, rt.owner.index);
    }
    lruUnlink(slot);
    if (rt.pinned()) --pinnedPages_;
    const uint32_t asset = rt.asset;
    const uint32_t generation = rt.generation();
    rt = PageRt{};
    rt.setGeneration(generation);
    // The compact stamps belong to the slot rather than the mounted page.
    // Preserve both counters across reset, but make detached slots fail
    // dependency validation until registerPage reuses them with a new version.
    pageStamps_[slot].setInUse(false);
    pageResidency_[slot] = PageResidency{};
    freeSlots_.push_back(slot);
    --attachedPages_;
    // Any per-instance bounds overlay on this mount is now describing a page
    // that no longer exists. They are not hunted down here (that would be a
    // scan of every instance); the generation stamp retires them on the next
    // lookup, and ensureOverlay recycles the storage.

    if (asset != kInvalidIndex)
    {
        AssetRt& as = assets_[asset];
        as.removeMountRef();
        if (as.rootMount == slot) as.rootMount = kInvalidIndex;
        destroyAssetIfUnused(asset);
    }
}

void World::detachMountTree(uint32_t rootSlot,
                            AppendBuffer<UserPayload>* freedPayloads)
{
    if (rootSlot == kInvalidIndex || !slots_[rootSlot].inUse()) return;
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

bool World::pageTreeFullyResident(uint32_t slot) const
{
    const PageResidency& summary = pageResidency_[slot];
    return summary.incompleteChildren == 0 &&
           summary.residentNodes + 1 == pageView(slots_[slot]).nodeCount();
}

void World::propagateFullResidency(uint32_t slot, bool wasFullyResident)
{
    bool fullyResident = pageTreeFullyResident(slot);
    while (fullyResident != wasFullyResident)
    {
        const NodeRef owner = slots_[slot].owner;
        if (!owner.valid()) return;

        slot = owner.slot;
        PageResidency& summary = pageResidency_[slot];
        const bool ownerWasFullyResident = pageTreeFullyResident(slot);
        if (fullyResident)
        {
            HLOD_CHECK(summary.incompleteChildren != 0,
                       "World: page residency summary underflow");
            --summary.incompleteChildren;
        }
        else
            ++summary.incompleteChildren;
        wasFullyResident = ownerWasFullyResident;
        fullyResident = pageTreeFullyResident(slot);
    }
}

bool World::descendantsCovered(uint32_t slot, uint32_t node) const
{
    const PageRt& rt = slots_[slot];
    if (node != 0 && pageView(rt).isExpansion(node))
    {
        const uint32_t child = rt.expSlot.empty() ? kInvalidIndex : rt.expSlot[node];
        return child != kInvalidIndex && slots_[child].inUse() &&
               slots_[child].isCovered(0);
    }
    const uint32_t count = pageView(rt).childCount(node);
    return count != 0 && rt.coveredChildren[node] == count;
}

bool World::computeCovered(uint32_t slot, uint32_t node) const
{
    const PageRt& rt = slots_[slot];
    return (node != 0 && rt.isResident(node)) || descendantsCovered(slot, node);
}

void World::propagateCoverage(uint32_t slot, uint32_t node)
{
    for (;;)
    {
        PageRt& rt = slots_[slot];
        const bool was = rt.isCovered(node);
        const bool now = computeCovered(slot, node);
        if (was == now) return;
        rt.setCovered(node, now);

        if (node == 0)
        {
            if (!rt.owner.valid()) return;
            const NodeRef owner = rt.owner;
            ++pageStamps_[owner.slot].contentVersion;
            slot = owner.slot;
            node = owner.index;
            continue;
        }

        const uint32_t parent = pageView(rt).parent[node];
        if (now)
            ++rt.coveredChildren[parent];
        else
            --rt.coveredChildren[parent];
        node = parent;
    }
}

void World::markResident(NodeHandle h)
{
    PageRt* rt = resolve(h);
    if (!rt) return;   // page collected while the payload was loading
    const uint32_t index = h.index();
    if (rt->isResident(index)) return;
    const bool wasFullyResident = pageTreeFullyResident(h.slot());
    rt->setResident(index, true);
    ++pageResidency_[h.slot()].residentNodes;
    propagateFullResidency(h.slot(), wasFullyResident);
    ++pageStamps_[h.slot()].contentVersion;
    propagateCoverage(h.slot(), index);
}

void World::markNonResident(NodeHandle h)
{
    PageRt* rt = resolve(h);
    if (!rt) return;
    const uint32_t index = h.index();
    HLOD_CHECK(!(rt->pinned() && pageView(*rt).parent[index] == 0),
               "World::markNonResident: pinned root");
    if (!rt->isResident(index)) return;
    const bool wasFullyResident = pageTreeFullyResident(h.slot());
    rt->setResident(index, false);
    HLOD_CHECK(pageResidency_[h.slot()].residentNodes != 0,
               "World: page residency summary underflow");
    --pageResidency_[h.slot()].residentNodes;
    propagateFullResidency(h.slot(), wasFullyResident);
    ++pageStamps_[h.slot()].contentVersion;
    propagateCoverage(h.slot(), index);
}

bool World::isResident(NodeHandle h) const
{
    const PageRt* rt = resolve(h);
    return rt && rt->isResident(h.index());
}

bool World::tryGetPayload(NodeHandle h, UserPayload& outPayload) const
{
    const PageRt* rt = resolve(h);
    if (!rt) return false;
    outPayload = pageView(*rt).payload[h.index()];
    return true;
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
        HLOD_CHECK(instances_.size() < kInvalidInstanceId,
                   "World: exhausted the 24-bit InstanceId space");
        instances_.emplace_back();
        instanceTlas_.emplace_back();
        instanceCutVersions_.emplace_back();
        instanceDenseToHandle_.push_back(kInvalidInstanceId);
        id = InstanceId(instances_.size() - 1);
    }

    InstanceId handle;
    if (!freeInstanceHandles_.empty())
    {
        handle = freeInstanceHandles_.back();
        freeInstanceHandles_.pop_back();
    }
    else
    {
        HLOD_CHECK(instanceHandleToDense_.size() < kInvalidInstanceId,
                   "World: exhausted the 24-bit instance-handle space");
        handle = InstanceId(instanceHandleToDense_.size());
        instanceHandleToDense_.push_back(kInvalidInstanceId);
    }
    instanceHandleToDense_[handle] = id;
    instanceDenseToHandle_[id] = handle;

    // Every instance of an asset walks the SAME root mount: one residency
    // array, one attachment graph, and one residency state for the whole
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
    spatial.mask = desc.mask;
    inst.setAlive(true);
    inst.generation = ++generationCounter_;
    spatial.liveIndex = uint32_t(liveInstances_.size());
    liveInstances_.push_back(id);
    const PageView& rootPage = assets_[asset].page;
    const bool flat = rootPage.nodeCount() == 2 && rootPage.childCount(0) == 1 &&
                      rootPage.childCount(1) == 0 && !rootPage.isExpansion(1);
    if (flat)
    {
        if (instanceFlatSlots_.empty())
            instanceFlatSlots_.resize(instances_.size(), kInvalidIndex);
        else if (instanceFlatSlots_.size() < instances_.size())
            instanceFlatSlots_.resize(instances_.size(), kInvalidIndex);
        instanceFlatSlots_[id] =
            slot | (rootPage.geometricError[1] > 0.0f ? 0u : kFlatZeroError);
        ++flatInstanceCount_;
    }
    else if (!instanceFlatSlots_.empty())
    {
        if (instanceFlatSlots_.size() < instances_.size())
            instanceFlatSlots_.resize(instances_.size(), kInvalidIndex);
        instanceFlatSlots_[id] = kInvalidIndex;
    }
    // Bounds first, then insert: the insert descends on worldBox. Going through
    // refreshInstanceBounds here would hand a not-yet-inserted instance to the
    // motion refit, whose kInvalidIndex case exists to catch exactly that as a
    // bug and would dirty the whole tree.
    recomputeInstanceBounds(id, true);
    tlasInsert(id);
    markTlasStructuralChange();
    return InstanceRef{handle, inst.generation,
                       PageHandle{slot, pageStamps_[slot].generation()}};
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
    const uint32_t a = createAsset(std::move(page), false);
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
    const InstanceId id = denseInstanceId(ref);
    return id == kInvalidInstanceId ? nullptr : &instances_[id];
}

InstanceId World::denseInstanceId(InstanceRef ref) const
{
    if (ref.id >= instanceHandleToDense_.size()) return kInvalidInstanceId;
    const InstanceId id = instanceHandleToDense_[ref.id];
    if (id >= instances_.size()) return kInvalidInstanceId;
    const Instance& inst = instances_[id];
    if (!inst.alive() || inst.generation != ref.generation)
        return kInvalidInstanceId;
    return id;
}

InstanceId World::publicInstanceId(InstanceId dense) const
{
    HLOD_ASSERT(dense < instanceDenseToHandle_.size() &&
                    instanceDenseToHandle_[dense] != kInvalidInstanceId,
                "World: live dense instance has no public handle");
    return instanceDenseToHandle_[dense];
}

World::MotionGroup::MotionGroup(std::span<const InstanceRef> instances)
{
    reset(instances);
}

void World::MotionGroup::reset(std::span<const InstanceRef> instances)
{
    instances_.clear();
    instances_.append(instances.data(), instances.size());
    physicalOrder_.clear();
    layoutVersion_ = 0;
    physicalOrderValid_ = false;
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
    const InstanceId id = InstanceId(inst - instances_.data());
    const uint32_t   asset = slots_[inst->rootSlot].asset;

    if (!instanceFlatSlots_.empty() &&
        instanceFlatSlots_[id] != kInvalidIndex)
    {
        HLOD_CHECK(flatInstanceCount_ != 0, "World: flat-instance count underflow");
        --flatInstanceCount_;
        instanceFlatSlots_[id] = kInvalidIndex;
    }

    freeOverlays(*inst);
    tlasRemove(id);
    const uint32_t liveIndex = instanceTlas_[id].liveIndex;
    const InstanceId moved = liveInstances_.back();
    liveInstances_[liveIndex] = moved;
    instanceTlas_[moved].liveIndex = liveIndex;
    liveInstances_.pop_back();
    if (liveInstances_.empty()) instanceLayoutSpatialized_ = false;
    instanceTlas_[id].liveIndex = kInvalidIndex;
    instances_[id].setAlive(false);
    instanceHandleToDense_[ref.id] = kInvalidInstanceId;
    instanceDenseToHandle_[id] = kInvalidInstanceId;
    freeInstanceHandles_.push_back(ref.id);
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
    const InstanceId dense = denseInstanceId(ref);
    if (dense == kInvalidInstanceId) return;
    moveInstanceDense(dense, pos, scale);
}

void World::moveInstanceDense(InstanceId dense, float4 pos, float scale)
{
    Instance& inst = instances_[dense];
    // Translation and bounds deformation cannot change geometric error.
    // Uniform scale can, so only that uncommon case rescans the root nodes.
    const bool scaleChanged = inst.scale != scale;
    inst.pos = pos;
    inst.scale = scale;
    refreshInstanceBounds(dense, scaleChanged);
}

void World::refreshMotionGroup(MotionGroup& group) const
{
    group.physicalOrder_.clear();
    group.physicalOrder_.reserve(group.instances_.size());
    for (uint32_t source = 0; source < group.instances_.size(); ++source)
    {
        const InstanceId dense = denseInstanceId(group.instances_[source]);
        if (dense != kInvalidInstanceId)
            group.physicalOrder_.push_back({dense, source});
    }
    if (group.physicalOrder_.size() > 1)
        std::sort(group.physicalOrder_.begin(), group.physicalOrder_.end());

    // Duplicate refs retain the last caller position, matching scalar calls.
    size_t out = 0;
    for (size_t i = 0; i < group.physicalOrder_.size();)
    {
        size_t last = i;
        while (last + 1 < group.physicalOrder_.size() &&
               group.physicalOrder_[last + 1].dense ==
                   group.physicalOrder_[i].dense)
            ++last;
        group.physicalOrder_[out++] = group.physicalOrder_[last];
        i = last + 1;
    }
    group.physicalOrder_.resize_uninitialized(out);
    group.layoutVersion_ = instanceLayoutVersion_;
    group.physicalOrderValid_ = true;
}

void World::moveInstances(MotionGroup& group,
                          std::span<const float4> positions,
                          float scale)
{
    HLOD_CHECK(group.instances_.size() == positions.size(),
               "World::moveInstances: motion-group/position count mismatch");
    HLOD_CHECK(scale > 0.0f, "World::moveInstances: non-positive scale");
    if (!group.physicalOrderValid_ ||
        group.layoutVersion_ != instanceLayoutVersion_)
        refreshMotionGroup(group);

    for (const MotionGroup::Slot slot : group.physicalOrder_)
    {
        if (slot.dense >= instances_.size()) continue;
        const InstanceRef ref = group.instances_[slot.source];
        const Instance& inst = instances_[slot.dense];
        if (!inst.alive() || inst.generation != ref.generation ||
            instanceDenseToHandle_[slot.dense] != ref.id)
            continue;
        moveInstanceDense(slot.dense, positions[slot.source], scale);
    }
}

void World::refreshInstanceBounds(InstanceId id, bool recomputeError)
{
    recomputeInstanceBounds(id, recomputeError);
    tlasOnInstanceMoved(id);
}

void World::recomputeInstanceBounds(InstanceId id, bool recomputeError)
{
    Instance& inst = instances_[id];
    InstanceTlas& spatial = instanceTlas_[id];
    // Globally unique rather than per-instance so a recycled slot can never
    // match the previous occupant's View record.
    instanceCutVersions_[id] = ++generationCounter_;
    const PageRt& rt = slots_[inst.rootSlot];
    const AABB* bbox = boundsFor(inst, inst.rootSlot, rt);
    spatial.worldBox = toWorld(bbox[0], inst.pos, inst.scale);

    if (recomputeError)
    {
        float maxErr = 0.0f;
        uint32_t c = 1;
        const PageView& page = pageView(rt);
        for (uint32_t k = 0; k < page.childCount(0); ++k)
        {
            maxErr = std::max(maxErr, std::min(page.geometricError[c], rt.errClamp));
            c += page.subtreeSize[c];
        }
        spatial.maxErrWorld = maxErr * inst.scale;
    }
}

// ============================================================================
// copy-on-write bounds overlays
//
// Bounds are the only bytes of a page the runtime ever rewrites. Giving a
// deformed instance a private copy of just those keeps everything that
// actually costs at scale — topology, payloads, errors, residency, and the
// attachment graph — shared with every other
// instance of the same asset. Duplicating the whole page instead would
// duplicate the streaming state too, which is the expensive half.
// ============================================================================

const World::Overlay* World::findOverlay(const Instance& inst, uint32_t slot) const
{
    if (!inst.hasOverlayList()) return nullptr;   // common case, one compare
    const std::vector<OverlayRef>& refs = overlayLists_[inst.overlayList()].refs;
    const auto it = std::lower_bound(
        refs.begin(), refs.end(), slot,
        [](const OverlayRef& r, uint32_t s) { return r.slot < s; });
    if (it == refs.end() || it->slot != slot) return nullptr;
    const Overlay& ov = overlays_[it->index];
    // The mount may have been detached (and its slot reused) since the
    // overlay was taken, in which case it describes a page that is gone.
    if (!ov.inUse() || ov.generation != pageStamps_[slot].generation()) return nullptr;
    return &ov;
}

void World::initOverlay(Overlay& ov, uint32_t slot, const PageRt& rt)
{
    const PageView& pg = pageView(rt);
    ov.generation = pageStamps_[slot].generation();
    ov.bbox.assign(pg.bbox, pg.bbox + pg.nodeCount());
    if (pg.wideCount() >= Overlay::kSparseWideMinBlocks)
    {
        std::vector<WideBounds>().swap(ov.wide);
        ov.widePatch.assign(pg.wideCount(), kInvalidIndex);
        ov.patchedWide.clear();
    }
    else
    {
        ov.wide.resize(pg.wideCount());
        for (uint32_t b = 0; b < pg.wideCount(); ++b)
            ov.wide[b] = pg.wide[b].bounds;
        std::vector<uint32_t>().swap(ov.widePatch);
        std::vector<WideBounds>().swap(ov.patchedWide);
    }
}

WideBounds& World::mutableWideBounds(Overlay& ov, const PageView& pg,
                                     uint32_t block)
{
    if (!ov.sparseWide()) return ov.wide[block];

    const uint32_t patch = ov.widePatch[block];
    if (patch != kInvalidIndex) return ov.patchedWide[patch];

    // Once edits cover a sixteenth of the blocks, dense storage removes the
    // sparse lookup from future selections and refits.
    if ((ov.patchedWide.size() + 1) *
            Overlay::kSparsePromotionDenominator >
        pg.wideCount())
    {
        ov.wide.resize(pg.wideCount());
        for (uint32_t b = 0; b < pg.wideCount(); ++b)
            ov.wide[b] = pg.wide[b].bounds;
        for (uint32_t b = 0; b < pg.wideCount(); ++b)
            if (ov.widePatch[b] != kInvalidIndex)
                ov.wide[b] = ov.patchedWide[ov.widePatch[b]];
        std::vector<uint32_t>().swap(ov.widePatch);
        std::vector<WideBounds>().swap(ov.patchedWide);
        return ov.wide[block];
    }

    ov.widePatch[block] = uint32_t(ov.patchedWide.size());
    ov.patchedWide.push_back(pg.wide[block].bounds);
    return ov.patchedWide.back();
}

uint32_t World::ensureOverlay(Instance& inst, uint32_t slot)
{
    if (!inst.hasOverlayList())
    {
        uint32_t list;
        if (!freeOverlayLists_.empty())
        {
            list = freeOverlayLists_.back();
            freeOverlayLists_.pop_back();
        }
        else
        {
            HLOD_CHECK(overlayLists_.size() < Instance::kOverlayListMask,
                       "World: exhausted overlay-list index space");
            overlayLists_.emplace_back();
            list = uint32_t(overlayLists_.size() - 1);
        }
        inst.setOverlayList(list);
    }
    std::vector<OverlayRef>& refs = overlayLists_[inst.overlayList()].refs;
    const auto it = std::lower_bound(
        refs.begin(), refs.end(), slot,
        [](const OverlayRef& r, uint32_t s) { return r.slot < s; });

    if (it != refs.end() && it->slot == slot)
    {
        const uint32_t idx = it->index;
        Overlay& ov = overlays_[idx];
        if (!ov.inUse() || ov.generation != pageStamps_[slot].generation())
            initOverlay(ov, slot, slots_[slot]);   // stale: retake from new page
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
    initOverlay(ov, slot, slots_[slot]);
    ++liveOverlays_;
    refs.insert(it, OverlayRef{slot, idx});
    return idx;
}

void World::freeOverlays(Instance& inst)
{
    if (!inst.hasOverlayList()) return;
    std::vector<OverlayRef>& refs = overlayLists_[inst.overlayList()].refs;
    for (const OverlayRef& r : refs)
    {
        Overlay& ov = overlays_[r.index];
        if (!ov.inUse()) continue;
        ov = Overlay{};
        freeOverlays_.push_back(r.index);
        --liveOverlays_;
    }
    refs.clear();
    freeOverlayLists_.push_back(inst.overlayList());
    inst.clearOverlayList();
}

const AABB* World::boundsFor(const Instance& inst, uint32_t slot, const PageRt& rt) const
{
    if (const Overlay* ov = findOverlay(inst, slot))
        return ov->bbox.data();
    return pageView(rt).bbox;
}

WideBoundsRef World::wideBoundsFor(const Instance& inst, uint32_t slot,
                                   const PageRt& rt, uint32_t* sparseOverlay) const
{
    if (const Overlay* ov = findOverlay(inst, slot))
    {
        if (ov->sparseWide())
        {
            *sparseOverlay = uint32_t(ov - overlays_.data());
            return pageView(rt).wideBounds();
        }
        return WideBoundsRef::packed(ov->wide.data());
    }
    return pageView(rt).wideBounds();
}

World::WorkItem World::makeWorkItem(uint32_t slot, const Instance& inst,
                                    uint8_t current, uint8_t ideal,
                                    uint8_t mask) const
{
    uint32_t sparse = kInvalidIndex;
    const WideBoundsRef wide =
        wideBoundsFor(inst, slot, slots_[slot], &sparse);
    return WorkItem{slot, wide, current, ideal, mask, sparse};
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
        if (ov.inUse())
            n += ov.bbox.size() * sizeof(AABB) +
                 ov.wide.size() * sizeof(WideBounds) +
                 ov.widePatch.size() * sizeof(uint32_t) +
                 ov.patchedWide.size() * sizeof(WideBounds);
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
    HLOD_ASSERT(mountBelongsTo(*inst, h.slot()),
                "World::setNodeBounds: the node is not in this instance's page tree");
    pendingMoves_.push_back({localBounds, h, ref.id, ref.generation});
}

void World::applyUpdates()
{
    ++frame_;
    flushBounds();
    if (tlasDirty_)
    {
        const bool firstSpatialization =
            !instanceLayoutSpatialized_ && !liveInstances_.empty();
        tlasRebuild(firstSpatialization);
    }
}

void World::optimize()
{
    flushBounds();
    tlasDirty_ = true;
    tlasQualityBuild_ = true;
    tlasRebuild(true);
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
    for (const PendingMove& m : pendingMoves_)
    {
        if (m.instance >= instanceHandleToDense_.size()) continue;
        const InstanceId dense = instanceHandleToDense_[m.instance];
        if (dense >= instances_.size()) continue;
        const Instance& inst = instances_[dense];
        if (!inst.alive() || inst.generation != m.instGeneration) continue;
        if (!resolve(m.node)) continue;
        applyBoundsChange(dense, m.node.slot(), m.node.index(), m.box);
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
    instanceCutVersions_[id] = ++generationCounter_;

    while (true)
    {
        // Taking the copy is what makes this instance stop sharing bounds for
        // this page — and only this page. Crossing a boundary below promotes
        // the owner too, so exactly the ancestor path is privatised.
        const uint32_t oi = ensureOverlay(instances_[id], curSlot);
        const PageView& pg = pageView(slots_[curSlot]);
        Overlay& overlay = overlays_[oi];
        AABB* bbox = overlay.bbox.data();

        if (exact)
        {
            bbox[cur] = curBox;
        }
        else
        {
            if (bbox[cur].contains(curBox)) return;   // ancestors already conservative
            bbox[cur].expand(curBox);
        }
        if (cur != 0) patchParentLane(pg, bbox, overlay, cur);

        while (cur != 0)
        {
            const uint32_t p = pg.parent[cur];
            if (bbox[p].contains(bbox[cur])) return;
            bbox[p].expand(bbox[cur]);             // grow immediately, shrink lazily
            if (p != 0) patchParentLane(pg, bbox, overlay, p);
            cur = p;
        }

        // The page outgrew its sentinel: cross the boundary.
        const NodeRef owner = slots_[curSlot].owner;
        if (!owner.valid())
        {
            refreshInstanceBounds(id, false);
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
void World::patchParentLane(const PageView& pg, AABB* bbox, Overlay& overlay,
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
                mutableWideBounds(overlay, pg, b).setLane(l, bbox[index]);
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
    return boundsFor(*inst, h.slot(), *rt)[h.index()];
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

bool World::instanceHasSingleRoot(InstanceId id) const
{
    const Instance& inst = instances_[id];
    const bool flat = !instanceFlatSlots_.empty() &&
                      instanceFlatSlots_[id] != kInvalidIndex;
    return !flat && pageView(slots_[inst.rootSlot]).childCount(0) == 1;
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
    HLOD_CHECK(tlasNodes_.size() < kInvalidInstanceId,
               "World: exhausted the 24-bit TLAS node space");
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
    if (tlasNodes_[cur].validLanes() == full)
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
    h.setLeafLane(lane, instanceHasSingleRoot(id));
    inst.setTlasPlacement(host, lane);
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
    if (inst.tlasNode() == kInvalidInstanceId) return;

    uint32_t nodeIdx = inst.tlasNode();
    const uint32_t lane = inst.tlasLane();
    if (nodeIdx >= tlasNodes_.size() ||
        !(tlasNodes_[nodeIdx].validMask & (1u << lane)) ||
        tlasNodes_[nodeIdx].child[lane] != ~int32_t(id))
    {
        tlasDirty_ = true;   // bookkeeping disagrees; rebuild rather than guess
        return;
    }

    if (inst.escapedSinceBuild())
    {
        if (tlasEscapes_) --tlasEscapes_;
        inst.setEscapedSinceBuild(false);
    }
    tlasNodes_[nodeIdx].clearLane(lane);
    inst.clearTlasPlacement();
    if (tlasLeafCount_) --tlasLeafCount_;

    while (tlasNodes_[nodeIdx].validLanes() == 0)
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
                p.clearLane(l);
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
    if (inst.tlasNode() == kInvalidInstanceId)
    {
        tlasDirty_ = true;
        return;
    }

    // Grow-only lane refit up the parent chain. The rebuild budget counts
    // distinct escaped leaves, not escape events: a bounded moving cohort
    // should not force periodic rebuilds merely because it moves every frame.
    const uint32_t nodeIdx = inst.tlasNode();
    const uint32_t lane = inst.tlasLane();
    TlasNode& node = tlasNodes_[nodeIdx];
    if (node.bounds.lane(lane).contains(inst.worldBox) &&
        node.maxErr.v[lane] >= inst.maxErrWorld)
        return;

    if (!inst.escapedSinceBuild())
    {
        inst.setEscapedSinceBuild(true);
        ++tlasEscapes_;
    }
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

// Stable LSD radix sort of the 63-bit Morton coordinate. Up to six 11-bit
// passes keep the histogram in L1 and turn the rebuild's O(N log N) comparison
// sort into linear streaming passes. Equal coordinates retain live-instance
// order; their order is immaterial to the tree and stability keeps it fully
// deterministic without paying for a second comparison sort.
template <class Item>
static void radixSortMorton(std::vector<Item>& keys,
                            std::vector<Item>& scratch,
                            uint64_t keyVariation)
{
    if (keys.size() < 1024)
    {
        std::sort(keys.begin(), keys.end());
        return;
    }
    if (keyVariation == 0) return;

    constexpr uint32_t kBits = 11;
    constexpr uint32_t kBuckets = 1u << kBits;
    constexpr uint64_t kMask = kBuckets - 1;
    std::array<uint32_t, kBuckets> offsets{};
    scratch.resize(keys.size());

    auto* src = &keys;
    auto* dst = &scratch;
    for (uint32_t shift = 0; shift < 63; shift += kBits)
    {
        if (((keyVariation >> shift) & kMask) == 0) continue;
        offsets.fill(0);
        for (const auto& item : *src)
            ++offsets[size_t((item.key() >> shift) & kMask)];

        uint32_t next = 0;
        for (uint32_t& count : offsets)
        {
            const uint32_t begin = next;
            next += count;
            count = begin;
        }

        for (const auto& item : *src)
            (*dst)[offsets[size_t((item.key() >> shift) & kMask)]++] = item;
        std::swap(src, dst);
    }
    if (src != &keys) keys.swap(scratch);
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
    const int32_t idx = tlasAllocNode();
    tlasNodes_[idx].parent = parent;

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
            n.setLeafLane(uint32_t(k), instanceHasSingleRoot(instIdx));
            inst.setTlasPlacement(uint32_t(idx), uint32_t(k));
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

void World::reorderInstancesByTlas()
{
    HLOD_ASSERT(pendingMoves_.empty(),
                "World: cannot reorder instances with queued deformation edits");

    const size_t slotCount = instances_.size();
    const size_t liveCount = liveInstances_.size();
    std::vector<InstanceId> order;
    order.reserve(slotCount);

    // Match tlasQuery's reverse-DFS stack and ascending leaf-lane order
    // exactly. A visible query is therefore a monotonic subsequence of these
    // dense ids; culling can skip ranges but cannot turn the stream random.
    if (tlasRoot_ >= 0)
    {
        std::vector<int32_t> stack;
        stack.push_back(tlasRoot_);
        while (!stack.empty())
        {
            const int32_t node = stack.back();
            stack.pop_back();
            const TlasNode& n = tlasNodes_[uint32_t(node)];
            for (uint32_t lane = 0; lane < kWide; ++lane)
            {
                if (!(n.validMask & (1u << lane))) continue;
                const int32_t child = n.child[lane];
                if (child >= 0)
                    stack.push_back(child);
                else
                    order.push_back(InstanceId(~child));
            }
        }
    }
    HLOD_ASSERT(order.size() == liveCount,
                "World: TLAS traversal did not contain every live instance");

    std::vector<InstanceId> oldToNew(slotCount, kInvalidInstanceId);
    for (InstanceId next = 0; next < liveCount; ++next)
        oldToNew[order[next]] = next;

    // Rewrite the TLAS before moving its parallel instance streams.
    for (TlasNode& n : tlasNodes_)
    {
        uint32_t lanes = n.validLanes();
        while (lanes)
        {
            const uint32_t lane = uint32_t(std::countr_zero(lanes));
            lanes &= lanes - 1;
            if (n.child[lane] < 0)
            {
                const InstanceId old = InstanceId(~n.child[lane]);
                n.child[lane] = ~int32_t(oldToNew[old]);
            }
        }
    }

    // Public handles are independent of dense positions, so dead dense slots
    // no longer need to survive a rebuild. Compacting them here makes both
    // memory and subsequent permutations scale with the live population,
    // rather than with the world's historical peak.
    std::vector<Instance> newInstances(liveCount);
    std::vector<InstanceTlas> newSpatial(liveCount);
    std::vector<uint32_t> newCutVersions(liveCount);
    std::vector<InstanceId> newDenseToHandle(liveCount, kInvalidInstanceId);
    std::vector<uint32_t> newFlat;
    const bool hadFlatStream = !instanceFlatSlots_.empty();
    if (hadFlatStream) newFlat.resize(liveCount, kInvalidIndex);

    for (InstanceId next = 0; next < liveCount; ++next)
    {
        const InstanceId old = order[next];
        newInstances[next] = std::move(instances_[old]);
        newSpatial[next] = std::move(instanceTlas_[old]);
        newCutVersions[next] = instanceCutVersions_[old];
        newDenseToHandle[next] = instanceDenseToHandle_[old];
        if (hadFlatStream) newFlat[next] = instanceFlatSlots_[old];
    }
    instances_.swap(newInstances);
    instanceTlas_.swap(newSpatial);
    instanceCutVersions_.swap(newCutVersions);
    instanceDenseToHandle_.swap(newDenseToHandle);
    if (hadFlatStream) instanceFlatSlots_.swap(newFlat);

    liveInstances_.resize(liveCount);
    for (InstanceId dense = 0; dense < liveCount; ++dense)
    {
        liveInstances_[dense] = dense;
        instanceTlas_[dense].liveIndex = dense;
        const InstanceId handle = instanceDenseToHandle_[dense];
        HLOD_ASSERT(handle < instanceHandleToDense_.size(),
                    "World: dense instance has an invalid public handle");
        instanceHandleToDense_[handle] = dense;
    }
    freeInstances_.clear();

    instanceLayoutSpatialized_ = liveCount != 0;
    if (++instanceLayoutVersion_ == 0) ++instanceLayoutVersion_;
}

// Two-tier rebuild policy:
//  - structural rebuilds (add/remove) take the quality path: rare,
//    long-lived, quality matters (contribution culling leans on tight
//    maxErr/bounds lanes);
//  - motion rebuilds (escape/area threshold) take the Morton path: one sort
//    plus contiguous groups of kWide per level, ~5x faster to build, letting
//    the escape policy rebuild eagerly and keep bloat low under heavy motion.
void World::tlasRebuild(bool reorderInstances)
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
        uint64_t firstMorton = 0;
        uint64_t mortonVariation = 0;
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
            const uint64_t morton = (kx << 2) | (ky << 1) | kz;
            if (tlasKeys_.empty()) firstMorton = morton;
            else mortonVariation |= morton ^ firstMorton;
            tlasKeys_.push_back({morton, i});
        }
        tlasLeafCount_ = uint32_t(tlasKeys_.size());
        if (!tlasKeys_.empty())
        {
            radixSortMorton(tlasKeys_, tlasKeysTmp_, mortonVariation);

            // Leaf level: consecutive groups of kWide instances.
            std::vector<int32_t>& cur = tlasLevelTmp_;
            cur.clear();
            for (size_t base = 0; base < tlasKeys_.size(); base += kWide)
            {
                const int32_t idx = tlasAllocNode();
                TlasNode& n = tlasNodes_[idx];
                const uint32_t cnt =
                    uint32_t(std::min<size_t>(kWide, tlasKeys_.size() - base));
                for (uint32_t k = 0; k < cnt; ++k)
                {
                    const uint32_t instIdx = tlasKeys_[base + k].instance;
                    InstanceTlas& inst = instanceTlas_[instIdx];
                    n.bounds.setLane(k, inst.worldBox);
                    n.maxErr.v[k] = inst.maxErrWorld;
                    n.child[k] = ~int32_t(instIdx);
                    n.laneMask[k] = inst.mask;
                    n.setLeafLane(k, instanceHasSingleRoot(instIdx));
                    inst.setTlasPlacement(uint32_t(idx), k);
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
                    const int32_t idx = tlasAllocNode();
                    TlasNode& n = tlasNodes_[idx];
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

    if (reorderInstances) reorderInstancesByTlas();

    // Baseline for the area-drift trigger: the total lane area this build
    // started from. Motion is allowed to add a configured fraction of it
    // before the tree is considered bloated enough to rebuild.
    tlasBaseArea_ = 0.0f;
    for (const TlasNode& n : tlasNodes_)
        for (uint32_t l = 0; l < kWide; ++l)
            if (n.validMask & (1u << l)) tlasBaseArea_ += surfaceArea(n.bounds.lane(l));
}

template<bool UseMask, bool UseMinPix, bool SelectRoots>
void World::tlasQueryImpl(const Camera& view, float minPix, float rootThreshold,
                          std::vector<VisibleItem>& outVisible,
                          std::vector<TlasItem>& stack) const
{
    outVisible.clear();
    HLOD_CHECK(!tlasDirty_ && pendingMoves_.empty(),
               "View::selectCut: call applyUpdates() after world changes");
    if (tlasRoot_ < 0) return;

    const float4 qmn = view.queryMin(), qmx = view.queryMax();

    stack.clear();
    stack.push_back({tlasRoot_, kAllPlanes});
    while (!stack.empty())
    {
        const TlasItem it = stack.back();
        stack.pop_back();
        const TlasNode& n = tlasNodes_[it.node()];

        const uint8_t inMask = it.mask();
        uint8_t outMasks[kWide];
        uint32_t survivors = inMask
                                 ? testWideAabb(n.bounds, view.frustum, inMask,
                                                outMasks) & n.validMask
                                 : n.validLanes();
        if (!survivors) continue;

        // Query-level dispatch removes this block entirely for the default
        // all-ones view mask.
        if constexpr (UseMask)
        {
            for (uint32_t l = 0; l < kWide; ++l)
                if (!(n.laneMask[l] & view.viewMask)) survivors &= ~(1u << l);
            if (!survivors) continue;
        }

        if constexpr (UseMinPix)
        {
            const float8 d2 = distanceToBoxesSq(n.bounds, qmn, qmx);
            const float8 errs = screenErrorFromSq8(n.maxErr, view.k, d2);
            for (uint32_t l = 0; l < kWide; ++l)
                if (errs.v[l] < minPix) survivors &= ~(1u << l);
        }

        uint32_t rootCandidates = 0;
        if constexpr (SelectRoots)
        {
            uint32_t singleRoots =
                ((n.validMask >> TlasNode::kSingleRootShift) &
                 TlasNode::kValidLaneMask) & survivors;
            if (singleRoots)
            {
                // One vector distance/error evaluation per TLAS leaf node.
                // The exact root test below the query remains authoritative,
                // so the rsqrt approximation can only miss an optimization
                // opportunity, never change the selected cut.
                const float8 d2 = distanceToBoxesSq(n.bounds, qmn, qmx);
                const float8 errs = screenErrorFromSq8(n.maxErr, view.k, d2);
                while (singleRoots)
                {
                    const uint32_t l = uint32_t(std::countr_zero(singleRoots));
                    singleRoots &= singleRoots - 1;
                    if (errs.v[l] <= rootThreshold) rootCandidates |= 1u << l;
                }
            }
        }

        while (survivors)
        {
            const uint32_t l = uint32_t(std::countr_zero(survivors));
            survivors &= survivors - 1;
            const int32_t c = n.child[l];
            if (c >= 0)
                stack.push_back({c, inMask ? outMasks[l] : uint8_t(0)});
            else
            {
                outVisible.emplace_back(uint32_t(~c),
                                        inMask ? outMasks[l] : uint8_t(0),
                                        (rootCandidates & (1u << l)) != 0);
            }
        }
    }
}

// ============================================================================
// garbage collection
// ============================================================================

void World::lruUnlink(uint32_t slot)
{
    PageRt& rt = slots_[slot];
    const uint32_t prev = rt.lruPrev();
    const uint32_t next = rt.lruNext();
    if (prev != kInvalidIndex) slots_[prev].setLruNext(next);
    else if (lruHead_ == slot) lruHead_ = next;
    if (next != kInvalidIndex) slots_[next].setLruPrev(prev);
    else if (lruTail_ == slot) lruTail_ = prev;
    rt.setLruPrev(kInvalidIndex);
    rt.setLruNext(kInvalidIndex);
}

void World::lruPushFront(uint32_t slot)
{
    PageRt& rt = slots_[slot];
    rt.setLruPrev(kInvalidIndex);
    rt.setLruNext(lruHead_);
    if (lruHead_ != kInvalidIndex) slots_[lruHead_].setLruPrev(slot);
    lruHead_ = slot;
    if (lruTail_ == kInvalidIndex) lruTail_ = slot;
}

void World::lruTouch(uint32_t slot, uint32_t epoch)
{
    PageRt& rt = slots_[slot];
    if (rt.lastTouched == epoch || int32_t(epoch - rt.lastTouched) <= 0) return;
    rt.lastTouched = epoch;
    if (rt.pinned() || lruHead_ == slot) return;
    lruUnlink(slot);
    lruPushFront(slot);
}

void World::consumePageUsage(PageUsageContext& usage)
{
    PageUsageContext* usages[] = {&usage};
    consumePageUsage(usages);
}

void World::consumePageUsage(std::span<PageUsageContext* const> usages)
{
    struct Event
    {
        uint32_t slot;
        uint32_t lastUsed;
    };

    size_t eventCapacity = 0;
    for (PageUsageContext* usage : usages)
        if (usage) eventCapacity += usage->dirty_.size();
    std::vector<Event> events;
    events.reserve(eventCapacity);

    for (PageUsageContext* usage : usages)
    {
        if (!usage) continue;
        HLOD_CHECK(usage->world_ == nullptr || usage->world_ == this,
                   "World::collect: PageUsageContext belongs to another World");
        for (const uint32_t slot : usage->dirty_)
        {
            if (slot >= usage->rec_.size()) continue;
            PageUsageContext::Rec& rec = usage->rec_[slot];
            rec.setPending(false);
            if (slot >= slots_.size()) continue;
            const PageStamp& stamp = pageStamps_[slot];
            if (!stamp.inUse() || stamp.generation() != rec.generation()) continue;
            events.push_back({slot, rec.lastUsed});
        }
        usage->dirty_.clear();
    }

    // Feedback may have accumulated for several frames and may come from
    // several cameras. Replay it oldest-to-newest so push-front preserves a
    // true LRU order instead of depending on context-list or discovery order.
    std::stable_sort(events.begin(), events.end(), [this](const Event& a,
                                                          const Event& b)
    {
        return frame_ - a.lastUsed > frame_ - b.lastUsed;
    });
    for (const Event& event : events) lruTouch(event.slot, event.lastUsed);
}

void World::recordPageUsage(PageUsageContext& usage, uint32_t slot) const
{
    HLOD_CHECK(usage.world_ == nullptr || usage.world_ == this,
               "View::selectCut: PageUsageContext belongs to another World");
    usage.world_ = this;
    const PageRt& rt = slots_[slot];
    if (rt.pinned()) return;
    const uint32_t generation = pageStamps_[slot].generation();
    if (usage.rec_.size() <= slot) usage.rec_.resize(size_t(slot) + 1);
    PageUsageContext::Rec& rec = usage.rec_[slot];
    if (rec.generation() != generation)
    {
        rec = PageUsageContext::Rec{};
        rec.setGeneration(generation);
    }
    rec.lastUsed = frame_;
    if (!rec.pending())
    {
        rec.setPending(true);
        usage.dirty_.push_back(slot);
    }
}

void World::tlasQuery(const Camera& view, float minPix, float rootThreshold,
                      std::vector<VisibleItem>& outVisible,
                      std::vector<TlasItem>& stack) const
{
    const bool useMask = view.viewMask != ~0u;
    const bool useMinPix = minPix > 0.0f;
    const bool selectRoots = rootThreshold >= 0.0f;
    if (selectRoots && useMask)
    {
        if (useMinPix)
            tlasQueryImpl<true, true, true>(view, minPix, rootThreshold,
                                            outVisible, stack);
        else
            tlasQueryImpl<true, false, true>(view, minPix, rootThreshold,
                                             outVisible, stack);
    }
    else if (selectRoots)
    {
        if (useMinPix)
            tlasQueryImpl<false, true, true>(view, minPix, rootThreshold,
                                             outVisible, stack);
        else
            tlasQueryImpl<false, false, true>(view, minPix, rootThreshold,
                                              outVisible, stack);
    }
    else if (useMask)
    {
        if (useMinPix)
            tlasQueryImpl<true, true, false>(view, minPix, rootThreshold,
                                             outVisible, stack);
        else
            tlasQueryImpl<true, false, false>(view, minPix, rootThreshold,
                                              outVisible, stack);
    }
    else if (useMinPix)
        tlasQueryImpl<false, true, false>(view, minPix, rootThreshold,
                                          outVisible, stack);
    else
        tlasQueryImpl<false, false, false>(view, minPix, rootThreshold,
                                           outVisible, stack);
}

CollectResult World::collect(size_t maxAttachedPages, uint32_t minAge)
{
    collectPayloads_.clear();
    size_t detached = 0;
    uint32_t slot = lruTail_;
    while (streamedPageCount() > maxAttachedPages && slot != kInvalidIndex)
    {
        const uint32_t prev = slots_[slot].lruPrev();
        const PageRt& rt = slots_[slot];
        const bool eligible = rt.inUse() && !rt.pinned() &&
                              rt.attachedChildPages() == 0 &&
                              (frame_ - rt.lastTouched) >= minAge;
        if (eligible)
        {
            detachSlot(slot, &collectPayloads_);
            ++detached;
        }
        slot = prev;
    }
    return {detached,
            {collectPayloads_.data(), collectPayloads_.size()}};
}

CollectResult World::collect(PageUsageContext& usage, size_t maxAttachedPages,
                             uint32_t minAge)
{
    consumePageUsage(usage);
    return collect(maxAttachedPages, minAge);
}

CollectResult World::collect(std::span<PageUsageContext* const> usage,
                             size_t maxAttachedPages, uint32_t minAge)
{
    consumePageUsage(usage);
    return collect(maxAttachedPages, minAge);
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
// Normal and dense-overlay bounds come through item.wide without a per-block
// branch. The sparse-overlay instantiation consults its compact patch table;
// template dispatch happens once per page rather than once per block.
template<bool FullyResident, bool SparseOverlay>
void World::wideVisit(const WorkItem& item, const PageView& pg, float errClamp,
                      uint32_t gen, InstanceId instance, uint32_t node, uint8_t mask,
                      uint8_t currentKids, uint8_t idealKids,
                      const Camera& local, Worker& w) const
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
        const WideBlock& blk = pg.wide[b];
        const WideBounds& wb = [&]() -> const WideBounds&
        {
            if constexpr (!SparseOverlay)
                return item.wide[b];
            else
            {
                const Overlay& ov = overlays_[item.sparseOverlay];
                const uint32_t patch = ov.widePatch[b];
                return patch == kInvalidIndex ? pg.wide[b].bounds
                                              : ov.patchedWide[patch];
            }
        }();
        // One load carries both lane masks. `survivors` never exceeds 8 bits,
        // so ANDing it with the whole word keeps exactly the valid lanes and
        // the leaf lanes in the high half come along for free.
        const uint32_t lanes = pg.blockMask[b];
        HLOD_STAT(w, wideBlocksTested, 1);
        uint8_t outMasks[kWide];
        const uint32_t survivors =
            testWideAabb(wb, local.frustum, mask, outMasks) & lanes;
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
            const CutEntry entry =
                makeCutEntry(NodeHandle{item.slot(), c, gen}, errs.v[l], w.bar,
                             w.barInv, instance);
            if constexpr (FullyResident)
                w.cut.shared.push(entry);
            else
                w.emit(entry, currentKids != 0, idealKids != 0);
        }

        uint32_t inner = survivors & ~leafLanes;
        while (inner)
        {
            const uint32_t l = uint32_t(std::countr_zero(inner));
            inner &= inner - 1;
            const uint32_t c = blk.child[l];
            const uint8_t planes = outMasks[l];
            if constexpr (FullyResident)
                w.nodeStack.push_back({c, errs.v[l], planes, 1, 1});
            else
                w.nodeStack.push_back(
                    {c, errs.v[l], planes, currentKids, idealKids});

            // This lane is the only kind that gets DECIDED: runPage will ask
            // whether its error clears the bar, and plain leaves (handled
            // above) are emitted without asking. The answer flips when the
            // distance reaches eff * k / bar, so the gap between where this
            // node is and where that happens is how far the camera may travel
            // before this instance's cut could differ. See View.
            if (w.trackMargin && (FullyResident || idealKids))
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

bool World::visibleDescendantsCovered(uint32_t slot, uint32_t node, uint8_t mask,
                                      const Instance& inst,
                                      const Camera& local,
                                      std::vector<uint32_t>* touched,
                                      bool uniqueTouches) const
{
    if (touched &&
        (!uniqueTouches ||
         std::find(touched->begin(), touched->end(), slot) == touched->end()))
        touched->push_back(slot);
    if (descendantsCovered(slot, node)) return true;

    // A fully-inside node has no invisible branch that can excuse a missing
    // payload. The propagated structural summary is therefore definitive.
    if (mask == 0) return false;

    const PageRt* rt = &slots_[slot];
    if (node != 0 && pageView(*rt).isExpansion(node))
    {
        const uint32_t child = rt->expSlot.empty() ? kInvalidIndex : rt->expSlot[node];
        if (child == kInvalidIndex) return false;
        slot = child;
        node = 0;
        rt = &slots_[slot];
        if (touched &&
            (!uniqueTouches ||
             std::find(touched->begin(), touched->end(), slot) == touched->end()))
            touched->push_back(slot);
    }

    const PageView& page = pageView(*rt);
    const uint32_t count = page.childCount(node);
    if (count == 0) return false;
    const AABB* bounds = boundsFor(inst, slot, *rt);

    uint32_t child = node + 1;
    for (uint32_t k = 0; k < count; ++k)
    {
        uint8_t childMask = mask;
        if (testAabb(bounds[child], local.frustum, childMask) != CullState::Outside &&
            !rt->isCovered(child))
        {
            if (!visibleDescendantsCovered(slot, child, childMask, inst, local,
                                           touched, uniqueTouches))
                return false;
        }
        child += page.subtreeSize[child];
    }
    return true;
}

template<bool FullyResident>
void World::runPage(const WorkItem& item, const Instance& inst, const Camera& local,
                    const CutParams& params, Worker& w) const
{
    if (item.sparseOverlay == kInvalidIndex)
        runPageImpl<FullyResident, false>(item, inst, local, params, w);
    else
        runPageImpl<FullyResident, true>(item, inst, local, params, w);
}

template<bool FullyResident, bool SparseOverlay>
void World::runPageImpl(const WorkItem& item, const Instance& inst,
                        const Camera& local, const CutParams& params,
                        Worker& w) const
{
    const PageRt& rt = slots_[item.slot()];
    if (w.trackTouches &&
        (!w.uniqueTouches ||
         std::find(w.touched.begin(), w.touched.end(), item.slot()) == w.touched.end()))
        w.touched.push_back(item.slot());

    HLOD_STAT(w, pagesVisited, 1);
    const PageView& pg = pageView(rt);
    const uint32_t gen = rt.generation();
    const InstanceId instance =
        publicInstanceId(InstanceId(&inst - instances_.data()));
    // One bar, no history: damping is already folded into the view's camera
    // envelope, which widened the measured error rather than moving the
    // threshold. That is what makes selection a pure read of the World.
    const float bar = params.threshold;

    w.nodeStack.clear();
    wideVisit<FullyResident, SparseOverlay>(
        item, pg, rt.errClamp, gen, instance, 0, item.mask(), item.current(),
        item.ideal(), local, w);

    while (!w.nodeStack.empty())
    {
        const NodeItem e = w.nodeStack.back();
        w.nodeStack.pop_back();
        const uint32_t i = e.node();
        HLOD_STAT(w, nodesVisited, 1);

        const NodeHandle here{item.slot(), i, gen};

        if constexpr (FullyResident)
        {
            if (!(e.err > bar))
            {
                w.cut.shared.push(
                    makeCutEntry(here, e.err, bar, w.barInv, instance));
                continue;
            }

            const bool exp = metaIsExpansion(pg.meta[i]);
            const uint32_t childSlot =
                (exp && !rt.expSlot.empty()) ? rt.expSlot[i] : kInvalidIndex;
            if (exp)
            {
                if (childSlot == kInvalidIndex)
                    w.cut.shared.push(
                        makeCutEntry(here, e.err, bar, w.barInv, instance));
                else
                    w.work.push_back(
                        makeWorkItem(childSlot, inst, 1, 1, e.planes()));
            }
            else
                wideVisit<true, SparseOverlay>(item, pg, rt.errClamp, gen,
                                               instance, i, e.planes(), 1, 1,
                                               local, w);
        }
        else
        {
            const bool current = e.current();
            const bool ideal = e.ideal();
            uint8_t nextCurrent = 0;
            uint8_t nextIdeal = 0;

            // Current-only traversal happens when the ideal cut stopped at a
            // non-resident proxy whose descendants nevertheless form a complete
            // resident cover. Stop at the nearest resident descendant; otherwise
            // continue through the precomputed cover.
            if (!ideal)
            {
                if (rt.isResident(i))
                {
                    w.cut.currentOnly.push(
                        makeCutEntry(here, e.err, bar, w.barInv, instance));
                    continue;
                }
                nextCurrent = 1;
            }
            else if (!(e.err > bar))
            {
                const bool shared = current && rt.isResident(i);
                const CutEntry entry =
                    makeCutEntry(here, e.err, bar, w.barInv, instance);
                w.emit(entry, shared, true);
                if (!current || shared) continue;
                // The ideal proxy is missing, but a recursively complete resident
                // descendant cut exists because the current walk reached it.
                nextCurrent = 1;
            }

            const uint32_t m = pg.meta[i];
            const bool exp = metaIsExpansion(m);

            const uint32_t childSlot =
                (exp && !rt.expSlot.empty()) ? rt.expSlot[i] : kInvalidIndex;

            if (ideal && e.err > bar && exp && childSlot == kInvalidIndex)
            {
                HLOD_CHECK(!current || rt.isResident(i),
                           "View::selectCut: non-resident current expansion proxy");
                const CutEntry entry =
                    makeCutEntry(here, e.err, bar, w.barInv, instance);
                w.emit(entry, current, true);
                continue;
            }

            if (ideal && e.err > bar)
            {
                const bool canDescend =
                    !current ||
                    visibleDescendantsCovered(item.slot(), i, e.planes(), inst, local,
                                               w.trackTouches ? &w.touched : nullptr,
                                               w.uniqueTouches);
                if (current && !canDescend)
                {
                    HLOD_CHECK(rt.isResident(i),
                               "View::selectCut: uncovered current subtree");
                    w.cut.currentOnly.push(
                        makeCutEntry(here, e.err, bar, w.barInv, instance));
                }
                nextCurrent = uint8_t(current && canDescend);
                nextIdeal = 1;
            }

            HLOD_CHECK(nextCurrent || nextIdeal,
                       "View::selectCut: node has no current or ideal continuation");
            if (exp)
            {
                HLOD_CHECK(childSlot != kInvalidIndex,
                           "View::selectCut: uncovered expansion subtree");
                w.work.push_back(makeWorkItem(childSlot, inst, nextCurrent,
                                              nextIdeal, e.planes()));
            }
            else
                wideVisit<false, SparseOverlay>(
                    item, pg, rt.errClamp, gen, instance, i, e.planes(),
                    nextCurrent, nextIdeal, local, w);
        }
    }
}

void World::runInstance(uint32_t instIdx, const Camera& view, const CutParams& params,
                        uint8_t mask, bool tryRoot, Worker& w) const
{
    const Instance& inst = instances_[instIdx];
    HLOD_STAT(w, instancesVisited, 1);

    // A root page normally contains one BLAS root. The TLAS already maintains
    // that root's exact world box and maximum effective error, so a distant
    // root can be selected before transforming the view or touching its wide
    // page data. Root payloads are pinned, making this entry shared by the
    // current and ideal cuts. Multi-root page forests take the ordinary walk.
    const PageRt& rootRt = slots_[inst.rootSlot];
    const PageView& rootPage = pageView(rootRt);
    if (tryRoot && rootPage.childCount(0) == 1)
    {
        const InstanceTlas& spatial = instanceTlas_[instIdx];
        if (mask == 0 ||
            testAabb(spatial.worldBox, view.frustum, mask) != CullState::Outside)
        {
            const float worldDistance =
                spatial.maxErrWorld > 0.0f
                    ? distanceToBox(spatial.worldBox, view.queryMin(),
                                    view.queryMax())
                    : 0.0f;
            const float error = spatial.maxErrWorld > 0.0f
                                    ? screenError(spatial.maxErrWorld, view.k,
                                                  worldDistance)
                                    : 0.0f;
            if (error <= params.threshold)
            {
                if (w.trackMargin && spatial.maxErrWorld > 0.0f)
                {
                    const float worldFlip =
                        spatial.maxErrWorld * view.k / w.bar;
                    const float worldSlack = worldDistance > worldFlip
                                                 ? worldDistance - worldFlip
                                                 : worldFlip - worldDistance;
                    w.margin = std::min(w.margin, worldSlack / inst.scale);
                    w.maxError = std::max(
                        w.maxError, spatial.maxErrWorld / inst.scale);
                }
                if (w.trackTouches &&
                    (!w.uniqueTouches ||
                     std::find(w.touched.begin(), w.touched.end(), inst.rootSlot) ==
                         w.touched.end()))
                    w.touched.push_back(inst.rootSlot);
                const InstanceId outputInstance = publicInstanceId(instIdx);
                w.cut.shared.push(makeCutEntry(
                    NodeHandle{inst.rootSlot, 1, rootRt.generation()}, error,
                    w.bar, w.barInv, outputInstance));
                return;
            }
        }
        else
            return;
    }

    const Camera local = toLocal(view, inst.pos, inst.scale);
    w.work.push_back(makeWorkItem(inst.rootSlot, inst, 1, 1, mask));
    const bool fullyResident = pageTreeFullyResident(inst.rootSlot);
    while (!w.work.empty())
    {
        const WorkItem item = w.work.back();
        w.work.pop_back();
        if (fullyResident)
            runPage<true>(item, inst, local, params, w);
        else
            runPage<false>(item, inst, local, params, w);
    }
}

void World::runFlatInstance(uint32_t instIdx, const Camera& view,
                            uint8_t mask, Worker& w) const
{
    const uint32_t flatMarker = instanceFlatSlots_[instIdx];
    const uint32_t slot = flatMarker & NodeHandle::kSlotMask;
    const bool zeroError = (flatMarker & kFlatZeroError) != 0;
    const InstanceTlas* spatial = nullptr;

    // Grow-only TLAS lanes can be looser than an instance after motion. The
    // ordinary page walk retests its exact child box, so the direct path must
    // do the same before emitting the sole node.
    uint8_t exactMask = mask;
    if (exactMask != 0 || !zeroError)
    {
        spatial = &instanceTlas_[instIdx];
        if (exactMask != 0 &&
            testAabb(spatial->worldBox, view.frustum, exactMask) == CullState::Outside)
            return;
    }

    HLOD_STAT(w, instancesVisited, 1);
    if (w.trackTouches &&
        (!w.uniqueTouches ||
         std::find(w.touched.begin(), w.touched.end(), slot) == w.touched.end()))
        w.touched.push_back(slot);

    const float error = !zeroError && spatial->maxErrWorld > 0.0f
                            ? screenError(
                                  spatial->maxErrWorld, view.k,
                                  distanceToBox(spatial->worldBox, view.queryMin(),
                                                view.queryMax()))
                            : 0.0f;
    const InstanceId outputInstance = publicInstanceId(instIdx);
    w.cut.shared.push(makeCutEntry(NodeHandle{slot, 1,
                                             pageStamps_[slot].generation()}, error,
                                   w.bar, w.barInv, outputInstance));
}

void World::selectCutUncached(const Camera& camera, const CutParams& params,
                              View& view, PageUsageContext* usage,
                              CutResultSink& outCut) const
{
    ViewScratch& scratch = *view.scratch_;
    view.stats_ = CutStats{};
    view.reused_ = 0;

    if (usage)
    {
        HLOD_CHECK(usage->world_ == nullptr || usage->world_ == this,
                   "View::selectCut: PageUsageContext belongs to another World");
        usage->world_ = this;
    }

    const Camera damped = view.damper_.damp(camera);
    const bool testRoots = view.rootQueryEnabled_ || view.rootProbeCountdown_ == 0;
    tlasQuery(damped, params.minPix, testRoots ? params.threshold : -1.0f,
              scratch.visible, scratch.tlasStack);

    const uint32_t nVis = uint32_t(scratch.visible.size());
    if (testRoots)
    {
        uint32_t roots = 0;
        for (const VisibleItem item : scratch.visible)
            roots += item.rootSelected() ? 1u : 0u;
        // The query-side vector test costs roughly one eighth of a normal
        // root-page visit. Require a comfortable margin rather than enabling
        // it for a sparse handful of distant instances.
        view.rootQueryEnabled_ = nVis != 0 && roots >= (nVis + 3u) / 4u;
        view.rootProbeCountdown_ = view.rootQueryEnabled_ ? 0u : 31u;
    }
    else
        --view.rootProbeCountdown_;

    view.walked_ = nVis;
    const uint32_t workerCount = config_.context.workerCount;
    const bool parallel = config_.parallelInstanceThreshold > 0 && workerCount > 1 &&
                          nVis >= config_.parallelInstanceThreshold;
    if (!parallel)
    {
        Worker& w = scratch.workers[0];
        w.work.clear();
        w.nodeStack.clear();
        w.touched.clear();
        w.trackTouches = usage != nullptr;
        w.uniqueTouches = false;
        w.cut = outCut;
        w.stats = CutStats{};
        w.bar = params.threshold;
        w.barInv = params.threshold > 0.0f ? 1.0f / params.threshold : 0.0f;

        if (flatInstanceCount_ == 0)
        {
            // Preserve the original hierarchical loop exactly: worlds with
            // no one-node assets pay only this call-level dispatch.
            for (uint32_t i = 0; i < nVis; ++i)
            {
                if (i + 2 < nVis)
                    HLOD_PREFETCH(&instances_[scratch.visible[i + 2].instance()]);
                if (i + 1 < nVis)
                {
                    const Instance& next =
                        instances_[scratch.visible[i + 1].instance()];
                    const PageRt& nrt = slots_[next.rootSlot];
                    const PageView& nextPage = pageView(nrt);
                    HLOD_PREFETCH(nextPage.wide);
                    HLOD_PREFETCH(nextPage.meta);
                    HLOD_PREFETCH(nextPage.payload);
                }
                runInstance(scratch.visible[i].instance(), damped, params,
                            scratch.visible[i].mask(),
                            scratch.visible[i].rootSelected(), w);
            }
        }
        else if (flatInstanceCount_ == liveInstances_.size())
        {
            // A flat-only forest does not need Instance or page-topology
            // prefetches. Lead the one cold stream that direct emission reads.
            constexpr uint32_t kFlatPrefetchDistance = 8;
            for (uint32_t i = 0; i < nVis; ++i)
            {
                if (i + kFlatPrefetchDistance < nVis)
                {
                    const VisibleItem next =
                        scratch.visible[i + kFlatPrefetchDistance];
                    const uint32_t marker = instanceFlatSlots_[next.instance()];
                    if (next.mask() != 0 || (marker & kFlatZeroError) == 0)
                        HLOD_PREFETCH(&instanceTlas_[next.instance()]);
                }
                const uint32_t instIdx = scratch.visible[i].instance();
                runFlatInstance(instIdx, damped, scratch.visible[i].mask(), w);
            }
        }
        else
        {
            // The hierarchical walk is a chain of dependent loads (Instance
            // -> page slot -> wide block). Pipeline it, but in a mixed forest
            // do not fetch those records for flat objects that bypass them.
            for (uint32_t i = 0; i < nVis; ++i)
            {
                if (i + 2 < nVis)
                {
                    const uint32_t next = scratch.visible[i + 2].instance();
                    if (instanceFlatSlots_[next] == kInvalidIndex)
                        HLOD_PREFETCH(&instances_[next]);
                }
                if (i + 1 < nVis)
                {
                    const uint32_t nextIdx = scratch.visible[i + 1].instance();
                    if (instanceFlatSlots_[nextIdx] == kInvalidIndex)
                    {
                        const Instance& next = instances_[nextIdx];
                        const PageRt& nrt = slots_[next.rootSlot];
                        const PageView& nextPage = pageView(nrt);
                        HLOD_PREFETCH(nextPage.wide);
                        HLOD_PREFETCH(nextPage.meta);
                        HLOD_PREFETCH(nextPage.payload);
                    }
                }
                const uint32_t instIdx = scratch.visible[i].instance();
                if (instanceFlatSlots_[instIdx] != kInvalidIndex)
                    runFlatInstance(instIdx, damped, scratch.visible[i].mask(), w);
                else
                    runInstance(instIdx, damped, params,
                                scratch.visible[i].mask(),
                                scratch.visible[i].rootSelected(), w);
            }
        }

        outCut = w.cut;
        w.cut = CutResultSink{};
        if (usage)
            for (const uint32_t slot : w.touched) recordPageUsage(*usage, slot);
        view.stats_ = w.stats;
        return;
    }

    // ---- parallel selection -------------------------------------------------
    // Each worker takes a contiguous run of visible instances and fills its
    // own buffers, so concatenating in worker order reproduces the serial
    // order exactly — the cut is bit-identical whether or not this path runs.
    if (scratch.workers.size() < workerCount) scratch.workers.resize(workerCount);

    struct Chunk
    {
        const World*     world;
        ViewScratch*     scratch;
        const Camera*    camera;
        const CutParams* params;
        uint32_t         nVis;
        uint32_t         workerCount;
        uint32_t         flatMode;   // 0 none, 1 mixed, 2 all
    } chunk{this, &scratch, &damped, &params, nVis, workerCount,
            flatInstanceCount_ == 0
                ? 0u
                : (flatInstanceCount_ == liveInstances_.size() ? 2u : 1u)};

    for (uint32_t k = 0; k < workerCount; ++k)
    {
        Worker& w = scratch.workers[k];
        w.work.clear();
        w.nodeStack.clear();
        w.touched.clear();
        w.trackTouches = usage != nullptr;
        w.uniqueTouches = false;
        w.cutBuf.clear();
        w.cut = makeSink(w.cutBuf);
        w.stats = CutStats{};
        w.bar = params.threshold;
        w.barInv = params.threshold > 0.0f ? 1.0f / params.threshold : 0.0f;
    }

    config_.context.parallelFor(
        workerCount,
        [](uint32_t k, void* payload)
        {
            auto* c = static_cast<Chunk*>(payload);
            const World& world = *c->world;
            const uint32_t per = (c->nVis + c->workerCount - 1) / c->workerCount;
            const uint32_t lo = std::min(k * per, c->nVis);
            const uint32_t hi = std::min(lo + per, c->nVis);
            Worker& w = c->scratch->workers[k];
            if (c->flatMode == 0)
            {
                for (uint32_t i = lo; i < hi; ++i)
                    world.runInstance(c->scratch->visible[i].instance(), *c->camera,
                                      *c->params, c->scratch->visible[i].mask(),
                                      c->scratch->visible[i].rootSelected(), w);
            }
            else if (c->flatMode == 2)
            {
                for (uint32_t i = lo; i < hi; ++i)
                {
                    const uint32_t instIdx = c->scratch->visible[i].instance();
                    world.runFlatInstance(instIdx, *c->camera,
                                          c->scratch->visible[i].mask(), w);
                }
            }
            else
            {
                for (uint32_t i = lo; i < hi; ++i)
                {
                    const uint32_t instIdx = c->scratch->visible[i].instance();
                    if (world.instanceFlatSlots_[instIdx] != kInvalidIndex)
                        world.runFlatInstance(instIdx, *c->camera,
                                              c->scratch->visible[i].mask(), w);
                    else
                        world.runInstance(instIdx, *c->camera, *c->params,
                                          c->scratch->visible[i].mask(),
                                          c->scratch->visible[i].rootSelected(), w);
                }
            }
        },
        &chunk, config_.context.user);

    for (uint32_t k = 0; k < workerCount; ++k)
    {
        Worker& w = scratch.workers[k];
        outCut.shared.pushRange(w.cutBuf.shared.data(),
                                uint32_t(w.cutBuf.shared.size()));
        outCut.currentOnly.pushRange(w.cutBuf.currentOnly.data(),
                                     uint32_t(w.cutBuf.currentOnly.size()));
        outCut.idealOnly.pushRange(w.cutBuf.idealOnly.data(),
                                   uint32_t(w.cutBuf.idealOnly.size()));
        if (usage)
            for (const uint32_t slot : w.touched) recordPageUsage(*usage, slot);
#ifdef HLOD_STATS
        view.stats_.instancesVisited += w.stats.instancesVisited;
        view.stats_.pagesVisited += w.stats.pagesVisited;
        view.stats_.nodesVisited += w.stats.nodesVisited;
        view.stats_.wideBlocksTested += w.stats.wideBlocksTested;
        view.stats_.lanesSurvived += w.stats.lanesSurvived;
#endif
        w.cut = CutResultSink{};
    }
}

// ---------------------------------------------------------------------------
// Cached selection
// ---------------------------------------------------------------------------

void View::reset()
{
    rec_.clear();
    recCold_.clear();
    secondDep_.clear();
    store_.clear();
    used_ = garbage_ = reused_ = walked_ = 0;
    travel_ = kTravel_ = 0.0f;
    primed_ = false;
    k_ = bar_ = 0.0f;
    stats_ = CutStats{};
    world_ = nullptr;
    instanceLayoutVersion_ = 0;
    rootQueryEnabled_ = true;
    rootProbeCountdown_ = 0;
    if (scratch_) scratch_->output.clear();
    ++epoch_;
    // The half-life is configuration and survives; the accumulated window is
    // state and does not. This is the half that reset() exists for: records
    // would have expired on their own, an envelope stretched across a teleport
    // would not.
    damper_.reset();
}

void View::setReuseEnabled(bool enabled)
{
    if (reuseEnabled_ == enabled) return;
    reset();
    reuseEnabled_ = enabled;
}

size_t View::bytes() const
{
    return rec_.capacity() * sizeof(Rec) +
           recCold_.capacity() * sizeof(RecCold) +
           secondDep_.capacity() * sizeof(SecondDep) +
           store_.capacity() * sizeof(CutEntry) +
           (scratch_ ? scratch_->bytes() : 0);
}

// Runs are allocated by bumping and abandoned when an instance's cut outgrows
// its block, so the slab accumulates holes. Squeeze them out once the holes
// outweigh the live data. Records keep their contents; only `begin` moves.
void View::compact()
{
    // Record ids and slab-allocation order are unrelated (the latter follows
    // TLAS traversal order).  Compacting in-place while iterating rec_ could
    // therefore move one run over the still-unread source of another run.
    // Compaction is deliberately rare, so use a same-sized scratch slab and
    // keep the existing allocation headroom while making the copy order moot.
    AppendBuffer<CutEntry> packed;
    packed.resize_uninitialized(store_.size());
    uint32_t w = 0;
    for (size_t i = 0; i < rec_.size(); ++i)
    {
        Rec& r = rec_[i];
        RecCold& cold = recCold_[i];
        if (cold.capacity == 0) continue;
        if (r.validUntil <= travel_ + r.kSlope * kTravel_ || r.epoch != epoch_)
        {
            // Not reusable anyway: drop the block rather than move it.
            cold.capacity = 0;
            r.counts = 0;
            continue;
        }
        const uint32_t count = cutTotal(r.counts);
        if (count)
            std::memcpy(packed.data() + w, store_.data() + r.begin,
                        size_t(count) * sizeof(CutEntry));
        r.begin = w;
        cold.capacity = count;
        w += count;
    }
    store_.swap(packed);
    used_ = w;
    garbage_ = 0;
}

void World::selectCutCached(const Camera& camera, const CutParams& params,
                            View& ctx, PageUsageContext* usage,
                            CutResultSink& outCut) const
{
    ViewScratch& scratch = *ctx.scratch_;
    ctx.stats_ = CutStats{};

    if (usage)
    {
        HLOD_CHECK(usage->world_ == nullptr || usage->world_ == this,
                   "View::selectCut: PageUsageContext belongs to another World");
        usage->world_ = this;
    }

    // The View owns hysteresis, so it takes the raw Camera and damps it
    // here. Everything below -- the cull, the walk, the odometer -- sees `dv`
    // and only `dv`, which is what makes the reuse argument about the envelope
    // rather than about the camera.
    const Camera dv = ctx.damper_.damp(camera);

    // Whole-cut cache hits already avoid the BLAS walk. Keep the universal
    // TLAS query lean and test the root only on the smaller miss population.
    tlasQuery(dv, params.minPix, -1.0f, scratch.visible, scratch.tlasStack);

    ctx.reused_ = ctx.walked_ = 0;
    if (ctx.instanceLayoutVersion_ != instanceLayoutVersion_)
    {
        ctx.rec_.clear();
        ctx.recCold_.clear();
        ctx.secondDep_.clear();
        ctx.store_.clear();
        ctx.used_ = ctx.garbage_ = 0;
        ctx.instanceLayoutVersion_ = instanceLayoutVersion_;
    }
    if (ctx.rec_.size() < instances_.size())
    {
        ctx.rec_.resize(instances_.size());
        ctx.recCold_.resize(instances_.size());
        if (!ctx.secondDep_.empty())
            ctx.secondDep_.resize_uninitialized(instances_.size());
    }

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

    Worker& w = scratch.workers[0];
    w.work.clear();
    w.nodeStack.clear();
    w.stats = CutStats{};
    w.trackTouches = true;
    w.uniqueTouches = true;
    w.bar = params.threshold;
    w.barInv = params.threshold > 0.0f ? 1.0f / params.threshold : 0.0f;

    const uint32_t nVis = uint32_t(scratch.visible.size());

    const auto recordUsage = [&](uint32_t slot)
    {
        if (usage) recordPageUsage(*usage, slot);
    };
    for (uint32_t i = 0; i < nVis; ++i)
    {
        const uint32_t instIdx = scratch.visible[i].instance();
        const uint8_t  mask = scratch.visible[i].mask();
        View::Rec& r = ctx.rec_[instIdx];
        const uint32_t depCount = cutDepCount(r.counts);

        // Everything the record was taken under, re-checked, in one cache
        // line. `mask == 0` is the frustum condition: this instance is wholly
        // inside, so no plane was tested anywhere within it and camera
        // rotation cannot matter. `travel_ < validUntil` is the margin.
        bool hit = mask == 0 &&
                   ctx.travel_ + r.kSlope * ctx.kTravel_ < r.validUntil &&
                   r.epoch == ctx.epoch_ &&
                   r.cutVersion == instanceCutVersions_[instIdx];
        if (hit && depCount != 0)
        {
            const PageStamp& stamp = pageStamps_[r.depSlot];
            hit = stamp.inUse() && stamp.contentVersion == r.depVersion;
        }
        if (hit && depCount == 2)
        {
            const View::SecondDep& dep = ctx.secondDep_[instIdx];
            const PageStamp& stamp = pageStamps_[dep.slot];
            hit = stamp.inUse() && stamp.contentVersion == dep.version;
        }

        if (hit)
        {
            if (depCount != 0) recordUsage(r.depSlot);
            if (depCount == 2) recordUsage(ctx.secondDep_[instIdx].slot);
            // The whole saving is the walk that did not happen. Copying the
            // recorded entries out is ~1.5% of the call at 80k instances, and
            // handing back a descriptor instead measured no better while
            // costing the caller an indirection: see View.
            const uint32_t shared = cutCount(r.counts, 0);
            const uint32_t current = cutCount(r.counts, 1);
            const uint32_t ideal = cutCount(r.counts, 2);
            const CutEntry* entries = ctx.store_.data() + r.begin;
            outCut.shared.pushRange(entries, shared);
            outCut.currentOnly.pushRange(entries + shared, current);
            outCut.idealOnly.pushRange(entries + shared + current, ideal);
            ++ctx.reused_;
            continue;
        }

        // ---- walk it ----
        // Hits deliberately never fetch the 32-byte Instance record. Once a
        // miss is known, start that read before resetting the worker scratch;
        // the bookkeeping below gives the cache line a little useful lead.
        HLOD_PREFETCH(&instances_[instIdx]);
        const Instance& inst = instances_[instIdx];
        w.cutBuf.clear();
        w.cut = makeSink(w.cutBuf);
        w.touched.clear();
        w.margin = FLT_MAX;
        w.maxError = 0.0f;
        w.trackMargin = true;
        if (flatInstanceCount_ != 0 &&
            instanceFlatSlots_[instIdx] != kInvalidIndex)
            runFlatInstance(instIdx, dv, mask, w);
        else
            runInstance(instIdx, dv, params, mask, true, w);
        w.trackMargin = false;
        for (const uint32_t slot : w.touched) recordUsage(slot);

        const uint32_t nShared = uint32_t(w.cutBuf.shared.size());
        const uint32_t nCurrent = uint32_t(w.cutBuf.currentOnly.size());
        const uint32_t nIdeal = uint32_t(w.cutBuf.idealOnly.size());
        const uint32_t n = nShared + nCurrent + nIdeal;
        const bool eligible = mask == 0 &&
                              w.touched.size() <= View::kMaxDeps &&
                              nShared <= 0x3ffu && nCurrent <= 0x3ffu &&
                              nIdeal <= 0x3ffu;
        View::RecCold& cold = ctx.recCold_[instIdx];
        if (cold.capacity < n)
        {
            ctx.garbage_ += cold.capacity;
            if (size_t(ctx.used_) + n > ctx.store_.size())
                ctx.store_.resize_uninitialized(
                    std::max<size_t>(size_t(ctx.used_) + n, ctx.store_.size() * 2 + 256));
            r.begin = ctx.used_;
            cold.capacity = n;
            ctx.used_ += n;
        }
        if (eligible && w.touched.size() == 2 && ctx.secondDep_.empty())
            ctx.secondDep_.resize_uninitialized(instances_.size());
        r.counts = eligible
                       ? packCutCounts(nShared, nCurrent, nIdeal,
                                       uint32_t(w.touched.size()))
                       : 0;
        CutEntry* dst = ctx.store_.data() + r.begin;
        if (nShared)
            std::memcpy(dst, w.cutBuf.shared.data(),
                        size_t(nShared) * sizeof(CutEntry));
        if (nCurrent)
            std::memcpy(dst + nShared, w.cutBuf.currentOnly.data(),
                        size_t(nCurrent) * sizeof(CutEntry));
        if (nIdeal)
            std::memcpy(dst + nShared + nCurrent, w.cutBuf.idealOnly.data(),
                        size_t(nIdeal) * sizeof(CutEntry));

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
            r.cutVersion = instanceCutVersions_[instIdx];
            if (!w.touched.empty())
            {
                r.depSlot = w.touched[0];
                r.depVersion = pageStamps_[w.touched[0]].contentVersion;
            }
            if (w.touched.size() == 2)
            {
                View::SecondDep& dep = ctx.secondDep_[instIdx];
                dep.slot = w.touched[1];
                dep.version = pageStamps_[w.touched[1]].contentVersion;
            }
        }
        else
        {
            r.validUntil = 0.0f;
        }

        // From the walk buffer, not from the slab: same bytes, still hot.
        outCut.shared.pushRange(w.cutBuf.shared.data(), nShared);
        outCut.currentOnly.pushRange(w.cutBuf.currentOnly.data(), nCurrent);
        outCut.idealOnly.pushRange(w.cutBuf.idealOnly.data(), nIdeal);
        ++ctx.walked_;
    }

    w.cut = CutResultSink{};
    ctx.stats_ = w.stats;
}

void View::selectCut(const World& world, const Camera& camera,
                     const CutParams& params, CutResultSink& outCut)
{
    HLOD_CHECK(world_ == nullptr || world_ == &world,
               "View::selectCut: View belongs to another World; call reset()");
    world_ = &world;
    if (reuseEnabled_)
        world.selectCutCached(camera, params, *this, nullptr, outCut);
    else
        world.selectCutUncached(camera, params, *this, nullptr, outCut);
}

void View::selectCut(const World& world, const Camera& camera,
                     const CutParams& params, PageUsageContext& usage,
                     CutResultSink& outCut)
{
    HLOD_CHECK(world_ == nullptr || world_ == &world,
               "View::selectCut: View belongs to another World; call reset()");
    world_ = &world;
    if (reuseEnabled_)
        world.selectCutCached(camera, params, *this, &usage, outCut);
    else
        world.selectCutUncached(camera, params, *this, &usage, outCut);
}

void View::selectCut(const World& world, const Camera& camera,
                     const CutParams& params,
                     CutResults& outCut)
{
    CutResultSink cut = World::makeSink(outCut.buffers_);
    selectCut(world, camera, params, cut);
    outCut.sync();
}

void View::selectCut(const World& world, const Camera& camera,
                     const CutParams& params, PageUsageContext& usage,
                     CutResults& outCut)
{
    CutResultSink cut = World::makeSink(outCut.buffers_);
    selectCut(world, camera, params, usage, cut);
    outCut.sync();
}

CutView View::selectCut(const World& world, const Camera& camera,
                        const CutParams& params)
{
    detail::CutBuffers& output = scratch_->output;
    CutResultSink cut = World::makeSink(output);
    selectCut(world, camera, params, cut);
    return output.view();
}

CutView View::selectCut(const World& world, const Camera& camera,
                        const CutParams& params, PageUsageContext& usage)
{
    detail::CutBuffers& output = scratch_->output;
    CutResultSink cut = World::makeSink(output);
    selectCut(world, camera, params, usage, cut);
    return output.view();
}

} // namespace hlod
