#include <gtest/gtest.h>

#include <memory>
#include <random>
#include <set>

#include "helpers.h"

using namespace hlod;
using namespace hlodtest;
using TA = World::TestAccess;

namespace {

struct Outputs
{
    std::vector<CutEntry> cut;
};

Outputs frame(World& w, const Camera& v, const CutParams& p)
{
    w.applyUpdates();
    Outputs o;
    selectCutUncached(w, v, p, o.cut);
    return o;
}

std::set<UserId> cutIds(const Outputs& o)
{
    std::set<UserId> ids;
    for (const auto& e : o.cut)
        if (inCurrentCut(e)) ids.insert(e.payload);
    return ids;
}

// Verifies invariant (C) conservatively for a page reachable from `anyId`,
// and that the wide lanes mirror bbox[] exactly.
//
// Both are read THROUGH THE INSTANCE: pages are immutable and shared, so a
// deformation lands in that instance's copy-on-write overlay, and reading the
// page directly would only ever show the authored bounds.
void verifyConservativeBounds(World& w, World::InstanceRef inst, UserId anyId)
{
    const NodeHandle h = handleOf(w, anyId);
    const PageView& pg = TA::pageOf(w, anyId);
    const AABB* bbox = TA::bboxOf(w, inst, h.slot);
    const WideBoundsRef wide = TA::wideOf(w, inst, h.slot);
    ASSERT_NE(bbox, nullptr);
    ASSERT_TRUE(wide.valid());

    for (uint32_t i = 1; i < pg.nodeCount(); ++i)
        EXPECT_TRUE(bbox[pg.parent[i]].contains(bbox[i])) << "node " << i;
    for (uint32_t i = 0; i < pg.nodeCount(); ++i)
    {
        const uint32_t cc = pg.childCount(i);
        uint32_t b = pg.wideOffset(i);
        for (uint32_t base = 0; base < cc; base += kWide, ++b)
        {
            const WideBounds& wb = wide[b];
            const WideBlock& blk = pg.wide[b];
            for (uint32_t l = 0; l < kWide; ++l)
            {
                if (!(pg.validLanes(b) & (1u << l))) continue;
                const AABB lane = wb.lane(l);
                const AABB& cold = bbox[blk.child[l]];
                EXPECT_EQ(lane.mn.x, cold.mn.x);
                EXPECT_EQ(lane.mn.y, cold.mn.y);
                EXPECT_EQ(lane.mn.z, cold.mn.z);
                EXPECT_EQ(lane.mx.x, cold.mx.x);
                EXPECT_EQ(lane.mx.y, cold.mx.y);
                EXPECT_EQ(lane.mx.z, cold.mx.z);
            }
        }
    }
}

// Bounds of one payload as `inst` sees them.
AABB boundsOf(World& w, World::InstanceRef inst, UserId id)
{
    return w.nodeBounds(inst, handleOf(w, id));
}

// Bounds of a page's sentinel (node 0) as `inst` sees them.
AABB rootBoundsOf(World& w, World::InstanceRef inst, UserId anyIdInPage)
{
    return TA::bboxOf(w, inst, handleOf(w, anyIdInPage).slot)[0];
}

} // namespace

// ---------------------------------------------------------------------------
// A moving leaf: ancestors grow to contain it, wide lanes stay in sync, and
// the cut follows the node to its new position.
// ---------------------------------------------------------------------------
TEST(Motion, LeafMoveRefitsAncestors)
{
    HLodBuilder b;
    const auto root = b.createRoot(1, 32.0f);
    const auto mid = b.createNode(root, 2, 8.0f);
    b.createNode(mid, 10, 1.0f, AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(1, 1, 1)));
    b.createNode(mid, 11, 1.0f, AABB::fromCenterExtent(float4::vec(4, 0, 0), float4::vec(1, 1, 1)));
    b.createNode(root, 3, 8.0f, AABB::fromCenterExtent(float4::vec(-8, 0, 0), float4::vec(1, 1, 1)));
    Page pg = b.build();
    const auto ids = pageIds(pg);

    World w;
    const auto inst = w.addInstance(std::move(pg), float4::point(0, 0, 0));
    markAllResident(w, ids);
    w.applyUpdates();

    // Move wall 11 far outside every ancestor box.
    const AABB newBox = AABB::fromCenterExtent(float4::vec(200, 50, 0), float4::vec(1, 1, 1));
    setNodeBounds(w, inst, 11, newBox);
    w.flushBounds();

    verifyConservativeBounds(w, inst, 11);
    EXPECT_TRUE(rootBoundsOf(w, inst, 11).contains(newBox));   // grew to the sentinel

    // A camera looking only at the new position sees exactly that wall.
    const Camera vNew = makeLookAtCamera(float4::point(200, 50, -30), float4::point(200, 50, 0));
    auto o = frame(w, vNew, {4.0f, 0.0f});
    EXPECT_TRUE(cutIds(o).count(11));

    // A camera at the old position no longer draws it.
    const Camera vOld = makeLookAtCamera(float4::point(4, 0, -10), float4::point(4, 0, 0));
    o = frame(w, vOld, {4.0f, 0.0f});
    EXPECT_FALSE(cutIds(o).count(11));
}

// ---------------------------------------------------------------------------
// Motion across a page boundary: a leaf inside an attached child page grows
// the owning expansion node in the parent page and the instance bounds. The
// overlay is promoted across the boundary as the growth propagates.
// ---------------------------------------------------------------------------
TEST(Motion, CrossPagePropagation)
{
    TreeGen gen;
    gen.fanout = 2;
    gen.depth = 1;
    Page root = gen.makeRootPage(unitRegion(10.0f), 32.0f, 1);
    const auto rootIds = pageIds(root);

    World w;
    const auto inst = w.addInstance(std::move(root), float4::point(0, 0, 0));
    markAllResident(w, rootIds);

    const UserId expId = gen.recipes.begin()->first;
    Page child = gen.makeChildPage(expId);
    const auto childIds = pageIds(child);
    attachPage(w, expId, std::move(child));
    markAllResident(w, childIds);
    w.applyUpdates();

    // Move a leaf of the child page way outside the whole tree.
    const UserId movedId = childIds.back();
    const AABB newBox = AABB::fromCenterExtent(float4::vec(500, 0, 0), float4::vec(1, 1, 1));
    setNodeBounds(w, inst, movedId, newBox);
    w.flushBounds();

    verifyConservativeBounds(w, inst, movedId);
    verifyConservativeBounds(w, inst, rootIds.front());

    // Both pages on the path were privatised, and only those two.
    EXPECT_EQ(TA::overlaysOf(w, inst), 2u);

    // The expansion node in the parent page now contains the new position.
    EXPECT_TRUE(boundsOf(w, inst, expId).contains(newBox));

    // And the cut can actually find the moved leaf out there.
    const Camera v = makeLookAtCamera(float4::point(500, 0, -20), float4::point(500, 0, 0));
    const auto o = frame(w, v, {1.0f, 0.0f});
    EXPECT_TRUE(cutIds(o).count(movedId));
}

// ---------------------------------------------------------------------------
// Deforming one instance must not disturb its siblings, and must not fork the
// runtime state they share. This is the whole point of copy-on-write bounds.
// ---------------------------------------------------------------------------
TEST(Motion, DeformingOneInstanceLeavesSiblingsAlone)
{
    TreeGen gen;
    gen.fanout = 3;
    gen.depth = 2;
    Page pg = gen.makeRootPage(unitRegion(5.0f), 16.0f, 0);
    const auto ids = pageIds(pg);

    World w;
    const AssetHandle asset = w.registerAsset(std::move(pg));
    const auto a = w.addInstance(asset, float4::point(0, 0, 0));
    const auto bInst = w.addInstance(asset, float4::point(100, 0, 0));
    const auto c = w.addInstance(asset, float4::point(200, 0, 0));
    markAllResident(w, ids);

    // One tree, three placements: one page, one mount, one residency array.
    EXPECT_EQ(w.attachedPageCount(), 1u);
    EXPECT_EQ(w.overlayCount(), 0u);
    const void* sharedMount = TA::mountStateOf(w, ids.front());

    const UserId leaf = ids.back();
    const AABB before = boundsOf(w, bInst, leaf);
    const AABB moved =
        AABB::fromCenterExtent(float4::vec(3, 40, 0), float4::vec(0.5f, 0.5f, 0.5f));
    setNodeBounds(w, a, leaf, moved);
    w.flushBounds();

    // Only the deformed instance took a copy...
    EXPECT_EQ(w.overlayCount(), 1u);
    EXPECT_EQ(TA::overlaysOf(w, a), 1u);
    EXPECT_EQ(TA::overlaysOf(w, bInst), 0u);
    EXPECT_EQ(TA::overlaysOf(w, c), 0u);

    // ...its siblings still see the authored bounds...
    const AABB stillB = boundsOf(w, bInst, leaf);
    EXPECT_EQ(stillB.mn.x, before.mn.x);
    EXPECT_EQ(stillB.mx.y, before.mx.y);
    EXPECT_FALSE(stillB.contains(moved));
    EXPECT_TRUE(boundsOf(w, a, leaf).contains(moved));

    // ...and everything except the bounds is still one shared object.
    EXPECT_EQ(w.attachedPageCount(), 1u);
    EXPECT_EQ(TA::mountStateOf(w, ids.front()), sharedMount);

    // The deformed instance's world box grew; its siblings' did not.
    EXPECT_TRUE(TA::instanceWorldBox(w, a).contains(toWorld(moved, float4::point(0, 0, 0), 1.0f)));
    EXPECT_FALSE(TA::instanceWorldBox(w, bInst)
                     .contains(toWorld(moved, float4::point(100, 0, 0), 1.0f)));

    verifyConservativeBounds(w, a, leaf);
    verifyConservativeBounds(w, bInst, leaf);

    // Removing the deformed instance returns its overlay to the pool.
    w.removeInstance(a);
    EXPECT_EQ(w.overlayCount(), 0u);
    EXPECT_EQ(w.overlayBytes(), 0u);
}

// ---------------------------------------------------------------------------
// Instance motion: the top-level BVH follows rigid moves; the whole tree
// renders at the new location, nothing at the old one.
// ---------------------------------------------------------------------------
TEST(Motion, InstanceMove)
{
    TreeGen gen;
    Page pg = gen.makeRootPage(unitRegion(5.0f), 16.0f, 0);
    const auto ids = pageIds(pg);

    World w;
    const auto inst = w.addInstance(std::move(pg), float4::point(0, 0, 0));
    markAllResident(w, ids);
    w.applyUpdates();

    const Camera vOrigin = makeLookAtCamera(float4::point(0, 0, -30), float4::point(0, 0, 0));
    EXPECT_FALSE(cutIds(frame(w, vOrigin, {4, 0})).empty());

    w.moveInstance(inst, float4::point(10000, 0, 0), 2.0f);
    EXPECT_TRUE(cutIds(frame(w, vOrigin, {4, 0})).empty());

    const Camera vThere = makeLookAtCamera(float4::point(10000, 0, -60), float4::point(10000, 0, 0));
    EXPECT_FALSE(cutIds(frame(w, vThere, {4, 0})).empty());
}

// ---------------------------------------------------------------------------
// TLAS query against a brute-force instance sweep on a freshly built BVH.
// ---------------------------------------------------------------------------
TEST(Motion, TlasMatchesBruteForce)
{
    std::mt19937 rng(4242);
    std::uniform_real_distribution<float> uni(-2000.0f, 2000.0f);

    TreeGen gen;
    World w;
    std::vector<float4> positions;
    for (int i = 0; i < 300; ++i)
    {
        Page pg = gen.makeRootPage(unitRegion(3.0f), 8.0f, 0);
        const auto ids = pageIds(pg);
        const float4 pos = float4::point(uni(rng), uni(rng) * 0.1f, uni(rng));
        positions.push_back(pos);
        w.addInstance(std::move(pg), pos);
        markAllResident(w, ids);
    }
    w.applyUpdates();
    TA::forceTlasRebuild(w);
    w.applyUpdates();

    const Camera v = makeLookAtCamera(float4::point(0, 100, -2500), float4::point(0, 0, 0));
    const auto visible = TA::tlasQuery(w, v, 0.0f);

    std::set<uint32_t> got;
    for (const auto& [inst, mask] : visible) got.insert(inst);

    for (uint32_t i = 0; i < positions.size(); ++i)
    {
        const AABB worldBox = AABB::fromCenterExtent(positions[i], float4::vec(3, 3, 3));
        uint8_t mask = kAllPlanes;
        const bool vis = testAabb(worldBox, v.frustum, mask) != CullState::Outside;
        EXPECT_EQ(got.count(i) != 0, vis) << "instance " << i;
    }
}

// ---------------------------------------------------------------------------
// Many random instance moves keep selectCut equivalent to the brute-force
// reference (the TLAS may go loose or rebuild — results must not change).
// ---------------------------------------------------------------------------
TEST(Motion, ManyMovesStayCorrect)
{
    std::mt19937 rng(99);
    std::uniform_real_distribution<float> uni(-500.0f, 500.0f);

    TreeGen gen;
    World w;
    std::vector<World::InstanceRef> insts;
    for (int i = 0; i < 60; ++i)
    {
        Page pg = gen.makeRootPage(unitRegion(4.0f), 8.0f, 0);
        const auto ids = pageIds(pg);
        insts.push_back(w.addInstance(std::move(pg), float4::point(uni(rng), 0, uni(rng))));
        markAllResident(w, ids);
    }

    const CutParams p{4.0f, 0.0f};
    for (int f = 0; f < 12; ++f)
    {
        for (size_t i = 0; i < insts.size(); i += 3)
            w.moveInstance(insts[i], float4::point(uni(rng), uni(rng) * 0.05f, uni(rng)));
        w.applyUpdates();

        const Camera v = makeLookAtCamera(float4::point(uni(rng), 200, -800), float4::point(0, 0, 0));
        Outputs o;
        selectCutUncached(w, v, p, o.cut);
        const RefResult want = TA::referenceCut(w, v, p);

        std::set<UserId> gotIds, wantIds;
        for (const auto& e : o.cut)
            if (inCurrentCut(e)) gotIds.insert(e.payload);
        for (const auto& e : want.cut)
            if (inCurrentCut(e)) wantIds.insert(e.payload);
        EXPECT_EQ(gotIds, wantIds) << "frame " << f;
    }
}

// ---------------------------------------------------------------------------
// Dynamic instance churn: every frame some trees are removed and new ones
// added while leaves keep moving. The cut must stay equivalent to the
// brute-force reference, and pending moves against removed trees (stale
// instance refs and stale handles alike) must be dropped harmlessly.
// ---------------------------------------------------------------------------
TEST(Motion, InstanceChurnStaysCorrect)
{
    std::mt19937 rng(1234);
    std::uniform_real_distribution<float> uni(-400.0f, 400.0f);

    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 2;

    World w;
    struct Tree
    {
        World::InstanceRef ref;
        PageHandle page;
        std::vector<uint32_t> leafIdx;
    };
    std::vector<Tree> trees;
    auto addTree = [&](float4 pos)
    {
        Page pg = gen.makeRootPage(unitRegion(4.0f), 16.0f, 0);
        const uint32_t n = pg.nodeCount();
        std::vector<uint32_t> leafIdx;
        for (uint32_t i = 1; i < n; ++i)
            if (pg.childCount(i) == 0) leafIdx.push_back(i);
        const auto inst = w.addInstance(std::move(pg), pos);
        markAllResident(w, inst.rootPage, n);
        trees.push_back({inst, inst.rootPage, std::move(leafIdx)});
    };
    for (int i = 0; i < 40; ++i) addTree(float4::point(uni(rng), 0, uni(rng)));

    const CutParams p{4.0f, 0.0f};
    for (int f = 0; f < 10; ++f)
    {
        // Move one leaf of every tree, then remove 20% and add replacements.
        // The removed trees' submissions become stale before the flush.
        for (const Tree& t : trees)
            w.setNodeBounds(t.ref,
                            nodeAt(t.page, t.leafIdx[size_t(f) % t.leafIdx.size()]),
                            AABB::fromCenterExtent(float4::vec(uni(rng) * 0.01f, 0, 0),
                                                   float4::vec(0.5f, 0.5f, 0.5f)));
        for (int k = 0; k < 8; ++k)
        {
            const size_t i = size_t(rng()) % trees.size();
            w.removeInstance(trees[i].ref);
            trees[i] = std::move(trees.back());
            trees.pop_back();
        }
        for (int k = 0; k < 8; ++k) addTree(float4::point(uni(rng), 0, uni(rng)));

        w.applyUpdates();
        const Camera v = makeLookAtCamera(float4::point(uni(rng), 300, -900),
                                          float4::point(0, 0, 0));
        std::vector<CutEntry> cut;
        selectCutUncached(w, v, p, cut);
        const RefResult want = TA::referenceCut(w, v, p);

        std::multiset<UserId> gotIds, wantIds;
        for (const auto& e : cut) gotIds.insert(e.payload);
        for (const auto& e : want.cut)
            if (inCurrentCut(e)) wantIds.insert(e.payload);
        EXPECT_EQ(gotIds, wantIds) << "frame " << f;
    }
    EXPECT_EQ(trees.size(), 40u);
}

// ---------------------------------------------------------------------------
// Moving leaves every frame: bounds stay conservative and the moved node is
// always drawable at its current position.
// ---------------------------------------------------------------------------
TEST(Motion, ManyLeafMovesStayConservative)
{
    std::mt19937 rng(5150);
    std::uniform_real_distribution<float> uni(-80.0f, 80.0f);

    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 3;
    Page pg = gen.makeRootPage(unitRegion(50.0f), 64.0f, 0);
    const auto ids = pageIds(pg);

    World w;
    const auto inst = w.addInstance(std::move(pg), float4::point(0, 0, 0));
    markAllResident(w, ids);

    // Collect the leaves (deepest generated ids are leaves of the page).
    std::vector<UserId> leaves;
    {
        const PageView& page = TA::pageOf(w, ids.front());
        for (uint32_t i = 1; i < page.nodeCount(); ++i)
            if (page.childCount(i) == 0) leaves.push_back(page.payload[i]);
    }
    ASSERT_GT(leaves.size(), 10u);

    for (int f = 0; f < 8; ++f)
    {
        for (size_t k = 0; k < leaves.size(); k += 5)
            setNodeBounds(w, inst, leaves[k],
                          AABB::fromCenterExtent(float4::vec(uni(rng), uni(rng), uni(rng)),
                                                 float4::vec(0.5f, 0.5f, 0.5f)));
        w.flushBounds();
        verifyConservativeBounds(w, inst, ids.front());
    }
}

// ---------------------------------------------------------------------------
// Handle fast path: moving by NodeHandle is exactly equivalent to moving by
// id — same refit, same wide lanes, same cut.
// ---------------------------------------------------------------------------
TEST(Motion, HandleMovesMatchIdMoves)
{
    auto makeWorld = [](std::unique_ptr<World>& w) -> World::InstanceRef
    {
        HLodBuilder b;
        const auto root = b.createRoot(1, 32.0f);
        const auto mid = b.createNode(root, 2, 8.0f);
        b.createNode(mid, 10, 1.0f, AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(1, 1, 1)));
        b.createNode(mid, 11, 1.0f, AABB::fromCenterExtent(float4::vec(4, 0, 0), float4::vec(1, 1, 1)));
        b.createNode(root, 3, 8.0f, AABB::fromCenterExtent(float4::vec(-8, 0, 0), float4::vec(1, 1, 1)));
        Page pg = b.build();
        const auto ids = pageIds(pg);
        w = std::make_unique<World>();
        const auto inst = w->addInstance(std::move(pg), float4::point(0, 0, 0));
        markAllResident(*w, ids);
        w->applyUpdates();
        return inst;
    };

    std::unique_ptr<World> byId, byHandle;
    const auto instId = makeWorld(byId);
    const auto instH = makeWorld(byHandle);
    const NodeHandle h = handleOf(*byHandle, 11);
    EXPECT_TRUE(h.valid());

    const AABB newBox = AABB::fromCenterExtent(float4::vec(200, 50, 0), float4::vec(1, 1, 1));
    setNodeBounds(*byId, instId, 11, newBox);
    byHandle->setNodeBounds(instH, h, newBox);
    byId->flushBounds();
    byHandle->flushBounds();

    verifyConservativeBounds(*byHandle, instH, 11);
    const uint32_t slotA = handleOf(*byId, 11).slot;
    const uint32_t slotB = h.slot;
    const AABB* a = TA::bboxOf(*byId, instId, slotA);
    const AABB* b = TA::bboxOf(*byHandle, instH, slotB);
    const uint32_t n = TA::pageOf(*byId, 11).nodeCount();
    for (uint32_t i = 0; i < n; ++i)
    {
        EXPECT_EQ(a[i].mn.x, b[i].mn.x) << "node " << i;
        EXPECT_EQ(a[i].mx.x, b[i].mx.x) << "node " << i;
    }

    const Camera v = makeLookAtCamera(float4::point(200, 50, -30), float4::point(200, 50, 0));
    auto o = frame(*byHandle, v, {4.0f, 0.0f});
    EXPECT_TRUE(cutIds(o).count(11));
}

// ---------------------------------------------------------------------------
// Handles are invalidated by detach: a stale handle (page detached, even with
// the slot reused by a re-attach) is silently ignored; a fresh handle works.
// ---------------------------------------------------------------------------
TEST(Motion, StaleHandleIgnoredAfterDetach)
{
    auto makeChild = []
    {
        HLodBuilder cb;
        const auto r = cb.createRoot(10, 4.0f);
        cb.createNode(r, 11, 1.0f, AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(1, 1, 1)));
        cb.createNode(r, 12, 1.0f, AABB::fromCenterExtent(float4::vec(3, 0, 0), float4::vec(1, 1, 1)));
        return cb.build();
    };

    HLodBuilder rb;
    const auto r = rb.createRoot(1, 32.0f);
    const auto e = rb.createNode(r, 2, 8.0f,
                                 AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(4, 4, 4)));
    rb.markExpansion(e);
    World w;
    const auto inst = w.addInstance(rb.build(), float4::point(0, 0, 0));

    attachPage(w, 2, makeChild());
    const NodeHandle stale = handleOf(w, 11);
    const AABB before = boundsOf(w, inst, 11);
    detachPage(w, 2);

    // Stale submission is dropped at flush — no crash, no effect.
    w.setNodeBounds(inst, stale,
                    AABB::fromCenterExtent(float4::vec(500, 0, 0), float4::vec(1, 1, 1)));
    w.flushBounds();
    EXPECT_EQ(w.overlayCount(), 0u);

    // Re-attach: same ids, new slot generation. The stale handle must still
    // be ignored; a freshly resolved one must work.
    attachPage(w, 2, makeChild());
    w.setNodeBounds(inst, stale,
                    AABB::fromCenterExtent(float4::vec(500, 0, 0), float4::vec(1, 1, 1)));
    w.flushBounds();
    EXPECT_EQ(boundsOf(w, inst, 11).mn.x, before.mn.x);   // untouched

    const NodeHandle fresh = handleOf(w, 11);
    const AABB moved = AABB::fromCenterExtent(float4::vec(7, 0, 0), float4::vec(1, 1, 1));
    w.setNodeBounds(inst, fresh, moved);
    w.flushBounds();
    EXPECT_TRUE(boundsOf(w, inst, 11).contains(moved));
    verifyConservativeBounds(w, inst, 11);
}

TEST(Motion, HandleOfUnknownIdThrows)
{
    World w;
    EXPECT_THROW((void)handleOf(w, 999), std::logic_error);
}

// ---------------------------------------------------------------------------
// Lazy motion contract: a node may be moved any number of times between
// cuts; its own bbox ends at exactly the LAST submitted box (last write
// wins), ancestors stay conservative (grow-only, they may also cover
// intermediate positions), and no flush is needed before selectCut — the
// applyUpdates publishes the final queued state.
// ---------------------------------------------------------------------------
TEST(Motion, RepeatedMovesLastWins)
{
    HLodBuilder b;
    const auto root = b.createRoot(1, 32.0f);
    const auto mid = b.createNode(root, 2, 8.0f);
    b.createNode(mid, 10, 1.0f, AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(1, 1, 1)));
    b.createNode(mid, 11, 1.0f, AABB::fromCenterExtent(float4::vec(4, 0, 0), float4::vec(1, 1, 1)));
    Page pg = b.build();
    const auto ids = pageIds(pg);

    World w;
    const auto inst = w.addInstance(std::move(pg), float4::point(0, 0, 0));
    markAllResident(w, ids);

    // Thrash node 11 across the map; only the final position may matter.
    for (int i = 0; i < 100; ++i)
        setNodeBounds(w, inst, 11,
                      AABB::fromCenterExtent(float4::vec(float(i * 37 % 900 - 450), 0, 0),
                                             float4::vec(1, 1, 1)));
    const AABB last = AABB::fromCenterExtent(float4::vec(50, 0, 0), float4::vec(1, 1, 1));
    setNodeBounds(w, inst, 11, last);
    w.flushBounds();

    verifyConservativeBounds(w, inst, 11);
    const AABB got = boundsOf(w, inst, 11);
    EXPECT_EQ(got.mn.x, last.mn.x);   // last write won
    EXPECT_EQ(got.mx.x, last.mx.x);
    EXPECT_TRUE(rootBoundsOf(w, inst, 11).contains(last));

    // One page on the path, so exactly one copy was taken no matter how many
    // times the node moved.
    EXPECT_EQ(w.overlayCount(), 1u);
}

TEST(Motion, ApplyUpdatesFlushesPendingBounds)
{
    HLodBuilder b;
    const auto root = b.createRoot(1, 32.0f);
    b.createNode(root, 10, 1.0f, AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(1, 1, 1)));
    b.createNode(root, 11, 1.0f, AABB::fromCenterExtent(float4::vec(4, 0, 0), float4::vec(1, 1, 1)));
    Page pg = b.build();
    const auto ids = pageIds(pg);

    World w;
    const auto inst = w.addInstance(std::move(pg), float4::point(0, 0, 0));
    markAllResident(w, ids);

    // The explicit update barrier applies pending bounds before selection.
    setNodeBounds(w, inst, 11,
                  AABB::fromCenterExtent(float4::vec(300, 0, 0), float4::vec(1, 1, 1)));
    w.applyUpdates();
    const Camera v = makeLookAtCamera(float4::point(300, 0, -20), float4::point(300, 0, 0));
    Outputs o;
    selectCutUncached(w, v, {2.0f, 0.0f}, o.cut);
    std::set<UserId> got;
    for (const auto& e : o.cut)
        if (inCurrentCut(e)) got.insert(e.payload);
    EXPECT_TRUE(got.count(11));
    verifyConservativeBounds(w, inst, 11);
}

TEST(Motion, SharedAncestorsRefitOnce)
{
    // Many siblings under one parent all teleport to the same faraway spot;
    // bounds must be exact-conservative and lanes in sync after one flush.
    HLodBuilder b;
    const auto root = b.createRoot(1, 64.0f);
    const auto mid = b.createNode(root, 2, 16.0f);
    std::vector<UserId> leaves;
    for (int i = 0; i < 12; ++i)
    {
        const UserId id = 100 + i;
        b.createNode(mid, id, 1.0f,
                     AABB::fromCenterExtent(float4::vec(float(i), 0, 0), float4::vec(0.5f, 0.5f, 0.5f)));
        leaves.push_back(id);
    }
    Page pg = b.build();
    const auto ids = pageIds(pg);

    World w;
    const auto inst = w.addInstance(std::move(pg), float4::point(0, 0, 0));
    markAllResident(w, ids);

    for (size_t i = 0; i < leaves.size(); ++i)
        setNodeBounds(w, inst, leaves[i],
                      AABB::fromCenterExtent(float4::vec(700 + float(i), 3, 0),
                                             float4::vec(0.5f, 0.5f, 0.5f)));
    w.flushBounds();
    verifyConservativeBounds(w, inst, ids.front());

    EXPECT_TRUE(rootBoundsOf(w, inst, ids.front())
                    .contains(AABB::fromCenterExtent(float4::vec(711, 3, 0),
                                                     float4::vec(0.5f, 0.5f, 0.5f))));
}
