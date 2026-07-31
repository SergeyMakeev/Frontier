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
    std::vector<CutEntry>    cut;
    std::vector<IdealEntry>  ideal;
    std::vector<LoadRequest> reqs;
};

Outputs frame(World& w, ViewScratch& s, const CullView& v, const CutParams& p)
{
    w.beginFrame();
    Outputs o;
    w.selectCut(v, p, s, o.cut, o.ideal, o.reqs);
    return o;
}

std::set<UserId> cutIds(const Outputs& o)
{
    std::set<UserId> ids;
    for (const auto& e : o.cut) ids.insert(e.payload);
    return ids;
}

// Verifies invariant (C) conservatively for a page reachable from `anyId`,
// and that the wide lanes mirror bbox[] exactly.
void verifyConservativeBounds(World& w, UserId anyId)
{
    const Page& pg = TA::pageOf(w, anyId);
    for (uint32_t i = 1; i < pg.nodeCount(); ++i)
        EXPECT_TRUE(pg.bbox[pg.parent[i]].contains(pg.bbox[i])) << "node " << i;
    for (uint32_t i = 0; i < pg.nodeCount(); ++i)
    {
        const uint32_t cc = pg.childCount(i);
        uint32_t b = pg.wideOffset(i);
        for (uint32_t base = 0; base < cc; base += kWide, ++b)
        {
            const WideBlock& blk = pg.wide[b];
            for (uint32_t l = 0; l < kWide; ++l)
            {
                if (!(blk.validMask & (1u << l))) continue;
                const AABB lane = blk.bounds.lane(l);
                const AABB& cold = pg.bbox[blk.child[l]];
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
    w.addInstance(std::move(pg), float4::point(0, 0, 0));
    markAllResident(w, ids);
    w.beginFrame();

    // Move wall 11 far outside every ancestor box.
    const AABB newBox = AABB::fromCenterExtent(float4::vec(200, 50, 0), float4::vec(1, 1, 1));
    setNodeBounds(w, 11, newBox);
    w.flushBounds();

    verifyConservativeBounds(w, 11);
    const Page& page = TA::pageOf(w, 11);
    EXPECT_TRUE(page.bbox[0].contains(newBox));   // grew all the way to the sentinel

    // A camera looking only at the new position sees exactly that wall.
    ViewScratch s;
    const CullView vNew = makeLookAtView(float4::point(200, 50, -30), float4::point(200, 50, 0));
    auto o = frame(w, s, vNew, {4.0f, 0.0f, 0.0f});
    EXPECT_TRUE(cutIds(o).count(11));

    // A camera at the old position no longer draws it.
    ViewScratch s2;
    const CullView vOld = makeLookAtView(float4::point(4, 0, -10), float4::point(4, 0, 0));
    o = frame(w, s2, vOld, {4.0f, 0.0f, 0.0f});
    EXPECT_FALSE(cutIds(o).count(11));
}

// ---------------------------------------------------------------------------
// Motion across a page boundary: a leaf inside an attached child page grows
// the owning expansion node in the parent page and the instance bounds.
// ---------------------------------------------------------------------------
TEST(Motion, CrossPagePropagation)
{
    TreeGen gen;
    gen.fanout = 2;
    gen.depth = 1;
    Page root = gen.makeRootPage(unitRegion(10.0f), 32.0f, 1);
    const auto rootIds = pageIds(root);

    World w;
    w.addInstance(std::move(root), float4::point(0, 0, 0));
    markAllResident(w, rootIds);

    const UserId expId = gen.recipes.begin()->first;
    Page child = gen.makeChildPage(expId);
    const auto childIds = pageIds(child);
    attachPage(w, expId, std::move(child));
    markAllResident(w, childIds);
    w.beginFrame();

    // Move a leaf of the child page way outside the whole tree.
    const UserId movedId = childIds.back();
    const AABB newBox = AABB::fromCenterExtent(float4::vec(500, 0, 0), float4::vec(1, 1, 1));
    setNodeBounds(w, movedId, newBox);
    w.flushBounds();

    verifyConservativeBounds(w, movedId);
    verifyConservativeBounds(w, rootIds.front());

    // The expansion node in the parent page now contains the new position.
    const Page& rootPage = TA::pageOf(w, expId);
    uint32_t expIdx = 0;
    for (uint32_t i = 1; i < rootPage.nodeCount(); ++i)
        if (rootPage.payload[i] == expId) expIdx = i;
    ASSERT_NE(expIdx, 0u);
    EXPECT_TRUE(rootPage.bbox[expIdx].contains(newBox));

    // And the cut can actually find the moved leaf out there.
    ViewScratch s;
    const CullView v = makeLookAtView(float4::point(500, 0, -20), float4::point(500, 0, 0));
    const auto o = frame(w, s, v, {1.0f, 0.0f, 0.0f});
    EXPECT_TRUE(cutIds(o).count(movedId));
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
    w.beginFrame();

    ViewScratch s;
    const CullView vOrigin = makeLookAtView(float4::point(0, 0, -30), float4::point(0, 0, 0));
    EXPECT_FALSE(cutIds(frame(w, s, vOrigin, {4, 0, 0})).empty());

    w.moveInstance(inst, float4::point(10000, 0, 0), 2.0f);
    ViewScratch s2;
    EXPECT_TRUE(cutIds(frame(w, s2, vOrigin, {4, 0, 0})).empty());

    const CullView vThere = makeLookAtView(float4::point(10000, 0, -60), float4::point(10000, 0, 0));
    ViewScratch s3;
    EXPECT_FALSE(cutIds(frame(w, s3, vThere, {4, 0, 0})).empty());
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
    w.beginFrame();
    TA::forceTlasRebuild(w);

    const CullView v = makeLookAtView(float4::point(0, 100, -2500), float4::point(0, 0, 0));
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

    ViewScratch s;
    const CutParams p{4.0f, 0.0f, 0.0f};
    for (int f = 0; f < 12; ++f)
    {
        for (size_t i = 0; i < insts.size(); i += 3)
            w.moveInstance(insts[i], float4::point(uni(rng), uni(rng) * 0.05f, uni(rng)));
        w.beginFrame();

        const CullView v = makeLookAtView(float4::point(uni(rng), 200, -800), float4::point(0, 0, 0));
        Outputs o;
        w.selectCut(v, p, s, o.cut, o.ideal, o.reqs);
        const RefResult want = TA::referenceCut(w, v, p);

        std::set<UserId> gotIds, wantIds;
        for (const auto& e : o.cut) gotIds.insert(e.payload);
        for (const auto& e : want.cut) wantIds.insert(e.payload);
        EXPECT_EQ(gotIds, wantIds) << "frame " << f;
    }
}

// ---------------------------------------------------------------------------
// Dynamic instance churn: every frame some trees are removed and new ones
// added while leaves keep moving. The cut must stay equivalent to the
// brute-force reference, and pending moves against removed trees (stale
// handles) must be dropped harmlessly.
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

    ViewScratch s;
    const CutParams p{4.0f, 0.0f, 0.0f};
    for (int f = 0; f < 10; ++f)
    {
        // Move one leaf of every tree, then remove 20% and add replacements.
        // The removed trees' submissions become stale before the flush.
        for (const Tree& t : trees)
            w.setNodeBounds(nodeAt(t.page, t.leafIdx[size_t(f) % t.leafIdx.size()]),
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

        w.beginFrame();
        const CullView v = makeLookAtView(float4::point(uni(rng), 300, -900),
                                          float4::point(0, 0, 0));
        std::vector<CutEntry> cut;
        w.selectCut(v, p, s, cut);
        const RefResult want = TA::referenceCut(w, v, p);

        std::multiset<UserId> gotIds, wantIds;
        for (const auto& e : cut) gotIds.insert(e.payload);
        for (const auto& e : want.cut) wantIds.insert(e.payload);
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
    w.addInstance(std::move(pg), float4::point(0, 0, 0));
    markAllResident(w, ids);

    // Collect the leaves (deepest generated ids are leaves of the page).
    std::vector<UserId> leaves;
    {
        const Page& page = TA::pageOf(w, ids.front());
        for (uint32_t i = 1; i < page.nodeCount(); ++i)
            if (page.childCount(i) == 0) leaves.push_back(page.payload[i]);
    }
    ASSERT_GT(leaves.size(), 10u);

    ViewScratch s;
    for (int f = 0; f < 8; ++f)
    {
        for (size_t k = 0; k < leaves.size(); k += 5)
            setNodeBounds(w, leaves[k],
                          AABB::fromCenterExtent(float4::vec(uni(rng), uni(rng), uni(rng)),
                                                 float4::vec(0.5f, 0.5f, 0.5f)));
        w.flushBounds();
        verifyConservativeBounds(w, ids.front());
    }
}

// ---------------------------------------------------------------------------
// Handle fast path: moving by NodeHandle is exactly equivalent to moving by
// id — same refit, same wide lanes, same cut.
// ---------------------------------------------------------------------------
TEST(Motion, HandleMovesMatchIdMoves)
{
    auto makeWorld = []
    {
        HLodBuilder b;
        const auto root = b.createRoot(1, 32.0f);
        const auto mid = b.createNode(root, 2, 8.0f);
        b.createNode(mid, 10, 1.0f, AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(1, 1, 1)));
        b.createNode(mid, 11, 1.0f, AABB::fromCenterExtent(float4::vec(4, 0, 0), float4::vec(1, 1, 1)));
        b.createNode(root, 3, 8.0f, AABB::fromCenterExtent(float4::vec(-8, 0, 0), float4::vec(1, 1, 1)));
        Page pg = b.build();
        const auto ids = pageIds(pg);
        auto w = std::make_unique<World>();
        w->addInstance(std::move(pg), float4::point(0, 0, 0));
        markAllResident(*w, ids);
        w->beginFrame();
        return w;
    };

    auto byId = makeWorld();
    auto byHandle = makeWorld();
    const NodeHandle h = handleOf(*byHandle, 11);
    EXPECT_TRUE(h.valid());

    const AABB newBox = AABB::fromCenterExtent(float4::vec(200, 50, 0), float4::vec(1, 1, 1));
    setNodeBounds(*byId, 11, newBox);
    byHandle->setNodeBounds(h, newBox);
    byId->flushBounds();
    byHandle->flushBounds();

    verifyConservativeBounds(*byHandle, 11);
    const Page& a = TA::pageOf(*byId, 11);
    const Page& b = TA::pageOf(*byHandle, 11);
    for (uint32_t i = 0; i < a.nodeCount(); ++i)
    {
        EXPECT_EQ(a.bbox[i].mn.x, b.bbox[i].mn.x) << "node " << i;
        EXPECT_EQ(a.bbox[i].mx.x, b.bbox[i].mx.x) << "node " << i;
    }

    ViewScratch s;
    const CullView v = makeLookAtView(float4::point(200, 50, -30), float4::point(200, 50, 0));
    auto o = frame(*byHandle, s, v, {4.0f, 0.0f, 0.0f});
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
    w.addInstance(rb.build(), float4::point(0, 0, 0));

    attachPage(w, 2, makeChild());
    const NodeHandle stale = handleOf(w, 11);
    const AABB before = TA::pageOf(w, 11).bbox[stale.index];
    detachPage(w, 2);

    // Stale submission is dropped at flush — no crash, no effect.
    w.setNodeBounds(stale, AABB::fromCenterExtent(float4::vec(500, 0, 0), float4::vec(1, 1, 1)));
    w.flushBounds();

    // Re-attach: same ids, new slot generation. The stale handle must still
    // be ignored; a freshly resolved one must work.
    attachPage(w, 2, makeChild());
    w.setNodeBounds(stale, AABB::fromCenterExtent(float4::vec(500, 0, 0), float4::vec(1, 1, 1)));
    w.flushBounds();
    const Page& pg = TA::pageOf(w, 11);
    EXPECT_EQ(pg.bbox[stale.index].mn.x, before.mn.x);   // untouched

    const NodeHandle fresh = handleOf(w, 11);
    const AABB moved = AABB::fromCenterExtent(float4::vec(7, 0, 0), float4::vec(1, 1, 1));
    w.setNodeBounds(fresh, moved);
    w.flushBounds();
    EXPECT_TRUE(TA::pageOf(w, 11).bbox[fresh.index].contains(moved));
    verifyConservativeBounds(w, 11);
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
// cut flushes internally and sees the final state.
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
    w.addInstance(std::move(pg), float4::point(0, 0, 0));
    markAllResident(w, ids);

    // Thrash node 11 across the map; only the final position may matter.
    for (int i = 0; i < 100; ++i)
        setNodeBounds(w, 11,
                      AABB::fromCenterExtent(float4::vec(float(i * 37 % 900 - 450), 0, 0),
                                             float4::vec(1, 1, 1)));
    const AABB last = AABB::fromCenterExtent(float4::vec(50, 0, 0), float4::vec(1, 1, 1));
    setNodeBounds(w, 11, last);
    w.flushBounds();

    verifyConservativeBounds(w, 11);
    const Page& page = TA::pageOf(w, 11);
    const NodeHandle h = handleOf(w, 11);
    EXPECT_EQ(page.bbox[h.index].mn.x, last.mn.x);   // last write won
    EXPECT_EQ(page.bbox[h.index].mx.x, last.mx.x);
    EXPECT_TRUE(page.bbox[0].contains(last));
}

TEST(Motion, NoFlushNeededBeforeSelectCut)
{
    HLodBuilder b;
    const auto root = b.createRoot(1, 32.0f);
    b.createNode(root, 10, 1.0f, AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(1, 1, 1)));
    b.createNode(root, 11, 1.0f, AABB::fromCenterExtent(float4::vec(4, 0, 0), float4::vec(1, 1, 1)));
    Page pg = b.build();
    const auto ids = pageIds(pg);

    World w;
    w.addInstance(std::move(pg), float4::point(0, 0, 0));
    markAllResident(w, ids);

    // Move and cut with no beginFrame/flushBounds in between.
    setNodeBounds(w, 11, AABB::fromCenterExtent(float4::vec(300, 0, 0), float4::vec(1, 1, 1)));
    ViewScratch s;
    const CullView v = makeLookAtView(float4::point(300, 0, -20), float4::point(300, 0, 0));
    Outputs o;
    w.selectCut(v, {2.0f, 0.0f, 0.0f}, s, o.cut, nullptr, nullptr);
    std::set<UserId> got;
    for (const auto& e : o.cut) got.insert(e.payload);
    EXPECT_TRUE(got.count(11));
    verifyConservativeBounds(w, 11);
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
    w.addInstance(std::move(pg), float4::point(0, 0, 0));
    markAllResident(w, ids);

    for (size_t i = 0; i < leaves.size(); ++i)
        setNodeBounds(w, leaves[i],
                      AABB::fromCenterExtent(float4::vec(700 + float(i), 3, 0),
                                             float4::vec(0.5f, 0.5f, 0.5f)));
    w.flushBounds();
    verifyConservativeBounds(w, ids.front());

    const Page& page = TA::pageOf(w, ids.front());
    EXPECT_TRUE(page.bbox[0].contains(
        AABB::fromCenterExtent(float4::vec(711, 3, 0), float4::vec(0.5f, 0.5f, 0.5f))));
}
