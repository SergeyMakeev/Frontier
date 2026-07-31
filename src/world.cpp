#include "hlod/world.h"

#include <algorithm>
#include <bit>
#include <stdexcept>
#include <string>

namespace hlod {

namespace {
[[noreturn]] void fail(const std::string& msg)
{
    throw std::logic_error("World: " + msg);
}
void check(bool cond, const char* msg)
{
    if (!cond) fail(msg);
}
} // namespace

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

// ============================================================================
// slots / attach / detach
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

uint32_t World::registerPage(Page&& page, InstanceId instance, NodeRef owner, bool pinned)
{
    const uint32_t slot = allocSlot();
    PageRt& rt = slots_[slot];
    rt.page = std::move(page);
    rt.resident.assign(rt.page.nodeCount(), 0);
    rt.readyChildren.assign(rt.page.nodeCount(), 0);
    rt.generation = ++generationCounter_;
    rt.lastTouched = frame_;
    rt.attachedChildPages = 0;
    rt.pinned = pinned;
    rt.inUse = true;
    rt.instance = instance;
    rt.owner = owner;

    ++attachedPages_;
    // Pinned pages can never be collected, so they never enter the LRU list;
    // that turns lruTouch into a single compare for them (a big deal for
    // forests of instances, whose root pages are all pinned).
    if (pinned) ++pinnedPages_;
    else lruPushFront(slot);
    return slot;
}

PageHandle World::attachPage(NodeHandle expansionNode, Page page)
{
    // Stale expansion handle: the parent page was detached/collected while
    // this page was being built. Normal streaming race — reject quietly.
    if (!resolve(expansionNode)) return PageHandle{};

    const NodeRef owner{expansionNode.slot, expansionNode.index};
    PageRt& ownerRt = slots_[owner.slot];
    check(ownerRt.page.isExpansion(owner.index), "attachPage: not an expansion point");
    check(ownerRt.expSlot.empty() || ownerRt.expSlot[owner.index] == kInvalidIndex,
          "attachPage: already attached");
    check(page.nodeCount() > 1, "attachPage: empty page");

    // (D) across the boundary: clamp the child page's errors to the expansion
    // node's error (forward sweep, same as build() pass C), then re-mirror the
    // wide error lanes, which are the hot copies.
    const float e = ownerRt.page.geometricError[owner.index];
    page.geometricError[0] = e;
    for (uint32_t i = 1; i < page.nodeCount(); ++i)
    {
        const float pe = page.geometricError[page.parent[i]];
        if (page.geometricError[i] > pe) page.geometricError[i] = pe;
    }
    for (uint32_t i = 0; i < page.nodeCount(); ++i)
    {
        const uint32_t cc = page.childCount(i);
        if (cc == 0) continue;
        uint32_t b = page.wideOffset(i);
        for (uint32_t base = 0; base < cc; base += kWide, ++b)
        {
            WideBlock& blk = page.wide[b];
            for (uint32_t l = 0; l < kWide; ++l)
                if (blk.validMask & (1u << l))
                    blk.error.v[l] = page.geometricError[blk.child[l]];
        }
    }

    // (C) across the boundary: the owner must contain the page's content.
    // Enforce conservatively: grow the owner chain if authored data is loose.
    const AABB pageBounds = page.bbox[0];   // union of the page's roots
    const uint32_t slot = registerPage(std::move(page), ownerRt.instance, owner, false);

    {
        PageRt& ort = slots_[owner.slot];
        if (ort.expSlot.empty()) ort.expSlot.assign(ort.page.nodeCount(), kInvalidIndex);
        ort.expSlot[owner.index] = slot;
        ort.attachedChildPages++;
    }

    if (!slots_[owner.slot].page.bbox[owner.index].contains(pageBounds))
    {
        AABB grown = slots_[owner.slot].page.bbox[owner.index];
        grown.expand(pageBounds);
        applyBoundsChange(owner.slot, owner.index, grown);
    }
    return PageHandle{slot, slots_[slot].generation};
}

void World::detachPage(NodeHandle expansionNode)
{
    const PageRt* ownerRt = resolve(expansionNode);
    if (!ownerRt) return;   // parent page already gone; nothing to detach
    const uint32_t child = ownerRt->expSlot.empty()
                               ? kInvalidIndex
                               : ownerRt->expSlot[expansionNode.index];
    check(child != kInvalidIndex, "detachPage: not attached");
    PageRt& rt = slots_[child];
    check(rt.attachedChildPages == 0, "detachPage: attached child pages remain");
    check(!rt.pinned, "detachPage: page is pinned");
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
    }
    lruUnlink(slot);
    if (rt.pinned) --pinnedPages_;
    rt = PageRt{};   // also releases the page memory
    freeSlots_.push_back(slot);
    --attachedPages_;
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
}

void World::markNonResident(NodeHandle h)
{
    PageRt* rt = resolve(h);
    if (!rt) return;
    check(!(rt->pinned && rt->page.parent[h.index] == 0),
          "markNonResident: pinned root");
    if (!rt->resident[h.index]) return;
    rt->resident[h.index] = 0;
    rt->readyChildren[rt->page.parent[h.index]]--;
}

bool World::isResident(NodeHandle h) const
{
    const PageRt* rt = resolve(h);
    return rt && rt->resident[h.index] != 0;
}

// ============================================================================
// instances
// ============================================================================

World::InstanceRef World::addInstance(Page rootPage, float4 pos, float scale)
{
    check(rootPage.nodeCount() > 1, "addInstance: empty page");
    check(scale > 0.0f, "addInstance: non-positive scale");

    InstanceId id;
    if (!freeInstances_.empty())
    {
        id = freeInstances_.back();
        freeInstances_.pop_back();
    }
    else
    {
        instances_.emplace_back();
        id = InstanceId(instances_.size() - 1);
    }

    const uint32_t slot = registerPage(std::move(rootPage), id, NodeRef{}, true);

    // Pin the roots' payloads: the base case of invariant (F).
    PageRt& rt = slots_[slot];
    uint32_t c = 1;
    for (uint32_t k = 0; k < rt.page.childCount(0); ++k)
    {
        rt.resident[c] = 1;
        rt.readyChildren[0]++;
        c += rt.page.subtreeSize[c];
    }

    Instance& inst = instances_[id];
    inst.pos = pos;
    inst.scale = scale;
    inst.rootSlot = slot;
    inst.alive = true;
    inst.generation = ++generationCounter_;
    inst.tlasNode = kInvalidIndex;
    refreshInstanceBounds(id);
    markTlasStructuralChange();
    return InstanceRef{id, inst.generation, PageHandle{slot, slots_[slot].generation}};
}

World::Instance* World::resolveInstance(InstanceRef ref)
{
    if (ref.id >= instances_.size()) return nullptr;
    Instance& inst = instances_[ref.id];
    if (!inst.alive || inst.generation != ref.generation) return nullptr;
    return &inst;
}

// Structural change policy: quality (median-split) rebuilds are reserved for
// real population drift — world assembly, level load, mass despawn. Under
// steady churn (spawn/despawn at roughly constant population) the instance
// count barely moves, and the next rebuild takes the Morton path instead:
// ~5x cheaper, same policy motion escapes already use, slightly looser tree
// that the next quality rebuild (>20% drift) re-tightens.
void World::markTlasStructuralChange()
{
    tlasDirty_ = true;
    const uint64_t alive = instances_.size() - freeInstances_.size();
    const uint64_t drift = alive > tlasQualityCount_ ? alive - tlasQualityCount_
                                                     : tlasQualityCount_ - alive;
    if (drift * 5 > tlasQualityCount_) tlasQualityBuild_ = true;
}

void World::removeInstance(InstanceRef ref)
{
    Instance* inst = resolveInstance(ref);
    if (!inst) return;   // stale ref: the instance is already gone
    const InstanceId id = ref.id;

    // Collect the instance's page tree from its root slot (preorder via the
    // expansion-slot links), then detach in reverse — children before their
    // owners. O(this instance's pages), independent of the world's size.
    std::vector<uint32_t> order;
    order.push_back(inst->rootSlot);
    for (size_t k = 0; k < order.size(); ++k)
        for (const uint32_t child : slots_[order[k]].expSlot)
            if (child != kInvalidIndex) order.push_back(child);
    for (size_t k = order.size(); k-- > 0;)
        detachSlot(order[k], nullptr);

    instances_[id].alive = false;
    freeInstances_.push_back(id);
    markTlasStructuralChange();
}

void World::moveInstance(InstanceRef ref, float4 pos, float scale)
{
    check(scale > 0.0f, "moveInstance: non-positive scale");
    Instance* inst = resolveInstance(ref);
    if (!inst) return;   // stale ref: never touch the slot's new occupant
    inst->pos = pos;
    inst->scale = scale;
    refreshInstanceBounds(ref.id);
}

void World::refreshInstanceBounds(InstanceId id)
{
    Instance& inst = instances_[id];
    const PageRt& rt = slots_[inst.rootSlot];
    inst.worldBox = toWorld(rt.page.bbox[0], inst.pos, inst.scale);

    float maxErr = 0.0f;
    uint32_t c = 1;
    for (uint32_t k = 0; k < rt.page.childCount(0); ++k)
    {
        maxErr = std::max(maxErr, rt.page.geometricError[c]);
        c += rt.page.subtreeSize[c];
    }
    inst.maxErrWorld = maxErr * inst.scale;
    tlasOnInstanceMoved(id);
}

// ============================================================================
// motion: lazy, coalesced, deduplicated conservative grow-only refit
// ============================================================================

void World::setNodeBounds(NodeHandle h, const AABB& localBounds)
{
    // Positive ordering check: rejects empty boxes AND NaN (every NaN
    // comparison is false, so !isEmpty() would let NaN through and poison
    // ancestor boxes forever — grow-only refit never un-grows).
    const AABB& b = localBounds;
    check(b.mn.x <= b.mx.x && b.mn.y <= b.mx.y && b.mn.z <= b.mx.z &&
              b.mx.x - b.mn.x < FLT_MAX && b.mx.y - b.mn.y < FLT_MAX &&
              b.mx.z - b.mn.z < FLT_MAX,
          "setNodeBounds: empty or non-finite bounds");
    if (!h.valid()) return;
    pendingMoves_.push_back({h.slot, h.index, h.generation, localBounds});
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
    // and merely re-checked by the rest. Stale entries (page detached,
    // slot reused) self-invalidate via the generation stamp.
    for (const auto& m : pendingMoves_)
    {
        if (!resolve(NodeHandle{m.slot, m.index, m.generation})) continue;
        applyBoundsChange(m.slot, m.index, m.box);
    }
    pendingMoves_.clear();
}

void World::applyBoundsChange(uint32_t slot, uint32_t index, const AABB& box)
{
    PageRt& rt = slots_[slot];
    Page& pg = rt.page;
    pg.bbox[index] = box;

    uint32_t cur = index;
    while (true)
    {
        patchParentLane(rt, cur);
        const uint32_t p = pg.parent[cur];
        if (pg.bbox[p].contains(pg.bbox[cur]))
            return;                            // ancestors already conservative
        pg.bbox[p].expand(pg.bbox[cur]);       // grow immediately, shrink lazily
        if (p == 0)
        {
            // The page outgrew its sentinel: cross the boundary.
            if (rt.owner.valid())
            {
                PageRt& ownerRt = slots_[rt.owner.slot];
                AABB grown = ownerRt.page.bbox[rt.owner.index];
                grown.expand(pg.bbox[0]);
                applyBoundsChange(rt.owner.slot, rt.owner.index, grown);
            }
            else
            {
                refreshInstanceBounds(rt.instance);
            }
            return;
        }
        cur = p;
    }
}

// Update a node's lane in its parent's wide block (the hot mirror of bbox).
void World::patchParentLane(PageRt& rt, uint32_t index)
{
    Page& pg = rt.page;
    const uint32_t p = pg.parent[index];
    const uint32_t cc = pg.childCount(p);
    uint32_t b = pg.wideOffset(p);
    for (uint32_t base = 0; base < cc; base += kWide, ++b)
    {
        WideBlock& blk = pg.wide[b];
        for (uint32_t l = 0; l < kWide; ++l)
        {
            if ((blk.validMask & (1u << l)) && blk.child[l] == index)
            {
                blk.bounds.setLane(l, pg.bbox[index]);
                return;
            }
        }
    }
    fail("internal: child lane not found");
}

// ============================================================================
// top-level BVH
// ============================================================================

void World::tlasOnInstanceMoved(InstanceId id)
{
    if (tlasDirty_) return;
    Instance& inst = instances_[id];
    if (inst.tlasNode == kInvalidIndex)
    {
        tlasDirty_ = true;
        return;
    }

    // Grow-only lane refit up the parent chain; escapes trigger a rebuild.
    uint32_t nodeIdx = inst.tlasNode;
    uint32_t lane = inst.tlasLane;
    TlasNode* node = &tlasNodes_[nodeIdx];
    if (node->bounds.lane(lane).contains(inst.worldBox) &&
        node->maxErr.v[lane] >= inst.maxErrWorld)
        return;

    ++tlasEscapes_;
    AABB grown = node->bounds.lane(lane);
    grown.expand(inst.worldBox);
    node->bounds.setLane(lane, grown);
    node->maxErr.v[lane] = std::max(node->maxErr.v[lane], inst.maxErrWorld);

    while (node->parent >= 0)
    {
        const int32_t childIdx = int32_t(nodeIdx);
        nodeIdx = uint32_t(node->parent);
        node = &tlasNodes_[nodeIdx];
        uint32_t l = 0;
        for (; l < kWide; ++l)
            if ((node->validMask & (1u << l)) && node->child[l] == childIdx) break;
        AABB laneBox = node->bounds.lane(l);
        if (laneBox.contains(grown) && node->maxErr.v[l] >= inst.maxErrWorld) break;
        laneBox.expand(grown);
        node->bounds.setLane(l, laneBox);
        node->maxErr.v[l] = std::max(node->maxErr.v[l], inst.maxErrWorld);
    }

    if (tlasEscapes_ * 4 > tlasLeafCount_) tlasDirty_ = true;
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

// Recursive median-split build: slower (7 nth_element calls per node) but
// produces noticeably tighter trees. Used for structural rebuilds
// (instances added/removed), which are rare and long-lived; motion rebuilds
// use the Morton path below.
int32_t World::tlasBuildRange(std::vector<uint32_t>& items, int lo, int hi, int32_t parent)
{
    const int32_t idx = int32_t(tlasNodes_.size());
    tlasNodes_.emplace_back();
    tlasNodes_[idx].parent = parent;
    for (uint32_t l = 0; l < kWide; ++l) tlasNodes_[idx].child[l] = 0;
    tlasNodes_[idx].bounds = WideBounds::allEmpty();
    tlasNodes_[idx].maxErr = float8::splat(0.0f);

    const int count = hi - lo;
    if (count <= int(kWide))
    {
        for (int k = 0; k < count; ++k)
        {
            const uint32_t instIdx = items[lo + k];
            Instance& inst = instances_[instIdx];
            TlasNode& n = tlasNodes_[idx];
            n.bounds.setLane(uint32_t(k), inst.worldBox);
            n.maxErr.v[k] = inst.maxErrWorld;
            n.child[k] = ~int32_t(instIdx);
            n.validMask |= 1u << k;
            inst.tlasNode = uint32_t(idx);
            inst.tlasLane = uint32_t(k);
        }
        return idx;
    }

    // Split into kWide groups: three levels of longest-axis median splits.
    int cuts[kWide + 1] = {};
    cuts[0] = lo;
    cuts[kWide] = hi;
    auto splitRange = [&](int l, int h, int outMid[1])
    {
        AABB cb = AABB::empty();
        for (int k = l; k < h; ++k)
            cb.expand(instances_[items[k]].worldBox.center());
        const float4 e = cb.extent();
        const int axis = (e.x >= e.y && e.x >= e.z) ? 0 : (e.y >= e.z ? 1 : 2);
        const int mid = (l + h) / 2;
        std::nth_element(items.begin() + l, items.begin() + mid, items.begin() + h,
                         [&](uint32_t a, uint32_t b)
                         {
                             const float4 ca = instances_[a].worldBox.center();
                             const float4 cb2 = instances_[b].worldBox.center();
                             const float va = axis == 0 ? ca.x : (axis == 1 ? ca.y : ca.z);
                             const float vb = axis == 0 ? cb2.x : (axis == 1 ? cb2.y : cb2.z);
                             return va < vb;
                         });
        outMid[0] = mid;
    };
    splitRange(cuts[0], cuts[8], &cuts[4]);
    splitRange(cuts[0], cuts[4], &cuts[2]);
    splitRange(cuts[4], cuts[8], &cuts[6]);
    splitRange(cuts[0], cuts[2], &cuts[1]);
    splitRange(cuts[2], cuts[4], &cuts[3]);
    splitRange(cuts[4], cuts[6], &cuts[5]);
    splitRange(cuts[6], cuts[8], &cuts[7]);

    for (uint32_t g = 0; g < kWide; ++g)
    {
        if (cuts[g] >= cuts[g + 1]) continue;
        const int32_t child = tlasBuildRange(items, cuts[g], cuts[g + 1], idx);

        // Union the child's lanes into our lane for it.
        AABB u = AABB::empty();
        float me = 0.0f;
        const TlasNode& cn = tlasNodes_[child];
        for (uint32_t l = 0; l < kWide; ++l)
        {
            if (!(cn.validMask & (1u << l))) continue;
            u.expand(cn.bounds.lane(l));
            me = std::max(me, cn.maxErr.v[l]);
        }
        TlasNode& n = tlasNodes_[idx];
        n.bounds.setLane(g, u);
        n.maxErr.v[g] = me;
        n.child[g] = child;
        n.validMask |= 1u << g;
    }
    return idx;
}

// Two-tier rebuild policy:
//  - structural rebuilds (add/remove) take the median-split path: rare,
//    long-lived, quality matters (contribution culling leans on tight
//    maxErr/bounds lanes);
//  - motion rebuilds (escape threshold) take the Morton path: one sort plus
//    contiguous groups of kWide per level, ~5x faster to build, letting the
//    escape policy rebuild eagerly and keep bloat low under heavy motion.
void World::tlasRebuild()
{
    if (tlasQualityBuild_)
    {
        tlasNodes_.clear();
        tlasRoot_ = -1;
        tlasEscapes_ = 0;
        tlasDirty_ = false;
        tlasQualityBuild_ = false;

        std::vector<uint32_t> items;
        for (uint32_t i = 0; i < uint32_t(instances_.size()); ++i)
            if (instances_[i].alive) items.push_back(i);
        tlasLeafCount_ = uint32_t(items.size());
        tlasQualityCount_ = tlasLeafCount_;
        if (items.empty()) return;
        tlasRoot_ = tlasBuildRange(items, 0, int(items.size()), -1);
        return;
    }
    tlasNodes_.clear();
    tlasRoot_ = -1;
    tlasEscapes_ = 0;
    tlasDirty_ = false;

    tlasKeys_.clear();
    AABB cb = AABB::empty();
    for (uint32_t i = 0; i < uint32_t(instances_.size()); ++i)
        if (instances_[i].alive) cb.expand(instances_[i].worldBox.center());
    const float4 lo = cb.mn;
    const float4 ext = cb.extent();
    const float sx = ext.x > 0.0f ? 2097151.0f / ext.x : 0.0f;
    const float sy = ext.y > 0.0f ? 2097151.0f / ext.y : 0.0f;
    const float sz = ext.z > 0.0f ? 2097151.0f / ext.z : 0.0f;
    for (uint32_t i = 0; i < uint32_t(instances_.size()); ++i)
    {
        if (!instances_[i].alive) continue;
        const float4 c = instances_[i].worldBox.center();
        const uint64_t kx = expandBits21(uint64_t((c.x - lo.x) * sx));
        const uint64_t ky = expandBits21(uint64_t((c.y - lo.y) * sy));
        const uint64_t kz = expandBits21(uint64_t((c.z - lo.z) * sz));
        tlasKeys_.push_back({(kx << 2) | (ky << 1) | kz, i});
    }
    tlasLeafCount_ = uint32_t(tlasKeys_.size());
    if (tlasKeys_.empty()) return;
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
        for (uint32_t l = 0; l < kWide; ++l) n.child[l] = 0;
        const uint32_t cnt = uint32_t(std::min<size_t>(kWide, tlasKeys_.size() - base));
        for (uint32_t k = 0; k < cnt; ++k)
        {
            const uint32_t instIdx = tlasKeys_[base + k].second;
            Instance& inst = instances_[instIdx];
            n.bounds.setLane(k, inst.worldBox);
            n.maxErr.v[k] = inst.maxErrWorld;
            n.child[k] = ~int32_t(instIdx);
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
            for (uint32_t l = 0; l < kWide; ++l) n.child[l] = 0;
            const uint32_t cnt = uint32_t(std::min<size_t>(kWide, cur.size() - base));
            for (uint32_t k = 0; k < cnt; ++k)
            {
                const int32_t childIdx = cur[base + k];
                TlasNode& cn = tlasNodes_[childIdx];
                cn.parent = idx;
                AABB u = AABB::empty();
                float me = 0.0f;
                for (uint32_t l = 0; l < kWide; ++l)
                {
                    if (!(cn.validMask & (1u << l))) continue;
                    u.expand(cn.bounds.lane(l));
                    me = std::max(me, cn.maxErr.v[l]);
                }
                n.bounds.setLane(k, u);
                n.maxErr.v[k] = me;
                n.child[k] = childIdx;
                n.validMask |= 1u << k;
            }
            next.push_back(idx);
        }
        cur.swap(next);
    }
    tlasRoot_ = cur[0];
}

void World::tlasQuery(const CullView& view, float minPix,
                      std::vector<std::pair<uint32_t, uint8_t>>& outVisible)
{
    outVisible.clear();
    if (tlasDirty_) tlasRebuild();
    if (tlasRoot_ < 0) return;

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

        if (minPix > 0.0f)
        {
            const float8 dist = distanceToBoxes(n.bounds, view.pos);
            const float8 errs = screenError8(n.maxErr, view.k, dist);
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
    if (rt.lastTouched == frame_) return;   // already touched this frame
    rt.lastTouched = frame_;
    if (rt.pinned) return;                  // pinned pages live outside the LRU
    lruUnlink(slot);
    lruPushFront(slot);
}

size_t World::collect(size_t maxAttachedPages, uint32_t minAge,
                      std::vector<UserPayload>* freedPayloads)
{
    // The watermark budgets STREAMED pages: pinned root pages can never be
    // collected, so counting them against the budget would make the collector
    // run unbounded in instance-heavy worlds (every aged page evicted at once
    // instead of keeping a cache up to the budget).
    // Tail-to-head passes; repeat while progress is made, because detaching a
    // leaf page can make its parent eligible (collapse is leaf-pages-first).
    size_t collected = 0;
    bool progress = true;
    while (streamedPageCount() > maxAttachedPages && progress)
    {
        progress = false;
        uint32_t cur = lruTail_;
        while (streamedPageCount() > maxAttachedPages && cur != kInvalidIndex)
        {
            const uint32_t prev = slots_[cur].lruPrev;
            const PageRt& rt = slots_[cur];
            if (!rt.pinned && rt.attachedChildPages == 0 &&
                frame_ - rt.lastTouched >= minAge)
            {
                detachSlot(cur, freedPayloads);
                ++collected;
                progress = true;
            }
            cur = prev;
        }
    }
    return collected;
}

// ============================================================================
// cut selection
// ============================================================================

ViewScratch::PageScratch& World::ensureScratch(ViewScratch& scratch, uint32_t slot,
                                               const PageRt& rt) const
{
    if (scratch.pages_.size() < slots_.size()) scratch.pages_.resize(slots_.size());
    ViewScratch::PageScratch& ps = scratch.pages_[slot];
    if (ps.generation != rt.generation)
    {
        ps.seenSticky.assign(rt.page.nodeCount(), 0);
        ps.generation = rt.generation;
    }
    return ps;
}

// One SIMD issue per kWide children: masked tri-state frustum, distance and
// screen error, lanes = children. Surviving PLAIN LEAVES are emitted right
// here (they are in both cuts by definition — no visit, no metadata reads);
// surviving interior/expansion nodes go onto the DFS stack with their err and
// narrowed plane mask carried along.
void World::wideVisit(const Page& pg, uint32_t slot, uint32_t gen, uint32_t node,
                      uint8_t mask, uint8_t aliveKids, const CullView& local,
                      std::vector<CutEntry>& outCut, std::vector<IdealEntry>* outIdeal)
{
    const uint32_t cc = pg.childCount(node);
    uint32_t b = pg.wideOffset(node);
    for (uint32_t base = 0; base < cc; base += kWide, ++b)
    {
        const WideBlock& blk = pg.wide[b];
        uint8_t outMasks[kWide];
        const uint32_t survivors =
            testWideAabb(blk.bounds, local.frustum, mask, outMasks) & blk.validMask;
        if (!survivors) continue;

        const float8 dist = distanceToBoxes(blk.bounds, local.pos);
        const float8 errs = screenError8(blk.error, local.k, dist);

        uint32_t leaves = survivors & blk.leafMask;
        while (leaves)
        {
            const uint32_t l = uint32_t(std::countr_zero(leaves));
            leaves &= leaves - 1;
            const uint32_t c = blk.child[l];
            if (aliveKids) outCut.push_back({pg.payload[c], errs.v[l]});
            if (outIdeal)
                outIdeal->push_back({pg.payload[c], NodeHandle{slot, c, gen},
                                     errs.v[l], IdealTag::Direct});
        }

        uint32_t inner = survivors & ~blk.leafMask;
        while (inner)
        {
            const uint32_t l = uint32_t(std::countr_zero(inner));
            inner &= inner - 1;
            nodeStack_.push_back({blk.child[l], errs.v[l], outMasks[l], aliveKids});
        }
    }
}

void World::pushLoadRequests(const PageRt& rt, uint32_t slot, uint32_t node,
                             float priority, std::vector<LoadRequest>& out) const
{
    const Page& pg = rt.page;
    uint32_t c = node + 1;
    for (uint32_t k = 0; k < pg.childCount(node); ++k)
    {
        if (!rt.resident[c])
            out.push_back({pg.payload[c], NodeHandle{slot, c, rt.generation}, priority});
        c += pg.subtreeSize[c];
    }
}

void World::runPage(const WorkItem& item, const CullView& local, const CutParams& params,
                    ViewScratch& scratch,
                    std::vector<CutEntry>& outCut, std::vector<IdealEntry>* outIdeal,
                    std::vector<LoadRequest>* outRequests)
{
    PageRt& rt = slots_[item.slot];
    lruTouch(item.slot);
    const Page& pg = rt.page;
    const uint32_t gen = rt.generation;
    const uint32_t frame = scratch.frame_;

    // Hysteresis history is the only per-node state; skip it entirely at h=0.
    const bool useSticky = params.hysteresis > 0.0f;
    ViewScratch::PageScratch* S = useSticky ? &ensureScratch(scratch, item.slot, rt)
                                            : nullptr;
    const float barU = params.threshold * (1.0f + params.hysteresis);
    const float barD = params.threshold * (1.0f - params.hysteresis);

    nodeStack_.clear();
    wideVisit(pg, item.slot, gen, 0, item.mask, item.alive, local, outCut, outIdeal);

    while (!nodeStack_.empty())
    {
        const NodeItem e = nodeStack_.back();
        nodeStack_.pop_back();
        const uint32_t i = e.node;

        // Every stacked node has kids (plain leaves were emitted by the
        // parent's wide test), so `wants` is just the error test.
        float bar = barU;
        if (useSticky)
        {
            const uint32_t ss = (*S).seenSticky[i];
            if ((ss >> 1) == frame - 1 && (ss & 1)) bar = barD;
        }
        const bool wants = e.err > bar;
        if (useSticky) (*S).seenSticky[i] = (frame << 1) | uint32_t(wants);

        if (!wants)   // both cuts end here; a good-enough collapsed proxy is DIRECT
        {
            if (e.alive) outCut.push_back({pg.payload[i], e.err});
            if (outIdeal)
                outIdeal->push_back({pg.payload[i], NodeHandle{item.slot, i, gen},
                                     e.err, IdealTag::Direct});
            continue;
        }

        const uint32_t m = pg.meta[i];
        const bool exp = metaIsExpansion(m);

        const uint32_t childSlot =
            (exp && !rt.expSlot.empty()) ? rt.expSlot[i] : kInvalidIndex;

        if (exp && childSlot == kInvalidIndex)   // collapsed AND too coarse
        {
            if (e.alive) outCut.push_back({pg.payload[i], e.err});
            if (outIdeal)
                outIdeal->push_back({pg.payload[i], NodeHandle{item.slot, i, gen},
                                     e.err, IdealTag::NeedsExpansion});
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
            outCut.push_back({pg.payload[i], e.err});
            if (outRequests)
                pushLoadRequests(exp ? slots_[childSlot] : rt,
                                 exp ? childSlot : item.slot, exp ? 0u : i, e.err,
                                 *outRequests);
        }

        const uint8_t a2 = uint8_t(e.alive && ready);
        if (exp)
            work_.push_back({childSlot, a2, e.planes});
        else
            wideVisit(pg, item.slot, gen, i, e.planes, a2, local, outCut, outIdeal);
    }
}

void World::selectCut(const CullView& view, const CutParams& params, ViewScratch& scratch,
                      std::vector<CutEntry>& outCut,
                      std::vector<IdealEntry>* outIdealCut,
                      std::vector<LoadRequest>* outRequests)
{
    outCut.clear();
    if (outIdealCut) outIdealCut->clear();
    if (outRequests) outRequests->clear();

    // The one place the tree must be up to date: apply pending bounds edits.
    flushBounds();

    ++scratch.frame_;

    tlasQuery(view, params.minPix, visibleTmp_);

    // The per-instance walk is a chain of dependent loads (Instance -> page
    // slot -> wide block); with tens of thousands of visible instances that
    // chain is memory-latency-bound. Pipeline it: prefetch instance i+2's
    // record and instance i+1's root page while working on instance i.
    const size_t nVis = visibleTmp_.size();
    auto prefetchStages = [&](size_t i)
    {
#if HLOD_AVX2
        if (i + 2 < nVis)
            _mm_prefetch(reinterpret_cast<const char*>(&instances_[visibleTmp_[i + 2].first]),
                         _MM_HINT_T0);
        if (i + 1 < nVis)
        {
            const Instance& next = instances_[visibleTmp_[i + 1].first];
            const PageRt& nrt = slots_[next.rootSlot];
            _mm_prefetch(reinterpret_cast<const char*>(nrt.page.wide.data()), _MM_HINT_T0);
            _mm_prefetch(reinterpret_cast<const char*>(nrt.page.meta.data()), _MM_HINT_T0);
            _mm_prefetch(reinterpret_cast<const char*>(nrt.page.payload.data()), _MM_HINT_T0);
        }
#else
        (void)i;
#endif
    };

    for (size_t i = 0; i < nVis; ++i)
    {
        prefetchStages(i);
        const auto& [instIdx, mask] = visibleTmp_[i];
        const Instance& inst = instances_[instIdx];
        const CullView local = toLocal(view, inst.pos, inst.scale);
        work_.push_back({inst.rootSlot, 1, mask});
        while (!work_.empty())
        {
            const WorkItem item = work_.back();
            work_.pop_back();
            runPage(item, local, params, scratch, outCut, outIdealCut, outRequests);
        }
    }
}

} // namespace hlod
