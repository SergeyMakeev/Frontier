#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <set>

#include "helpers.h"

// Cross-cutting API contracts: cut invariants that must hold on any world,
// determinism, multi-view damper isolation, instance-slot reuse (ABA),
// geometric edge cases, and memory budgets.

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

Outputs run(World& w, const CullView& v, const CutParams& p)
{
    Outputs o;
    w.selectCut(v, p, o.cut, o.ideal, o.reqs);
    return o;
}

std::set<UserId> cutIds(const std::vector<CutEntry>& v)
{
    std::set<UserId> s;
    for (const auto& e : v) s.insert(e.payload);
    return s;
}

// Deterministic random world: several instances of a paged tree, a couple of
// rounds of expansion attaches, partial residency. Given the same seed it
// reproduces the exact same World (same slots, same pages, same residency).
struct RandomWorld
{
    World w;
    std::vector<UserId> allIds;

    explicit RandomWorld(uint32_t seed)
    {
        std::mt19937 rng(seed);
        std::uniform_real_distribution<float> uni(0.0f, 1.0f);

        TreeGen gen;
        gen.fanout = 2 + uint32_t(rng() % 3);
        gen.depth = 1 + uint32_t(rng() % 2);

        const int numInstances = 1 + int(rng() % 3);
        for (int inst = 0; inst < numInstances; ++inst)
        {
            Page pg = gen.makeRootPage(unitRegion(40.0f), 64.0f, 2);
            const auto ids = pageIds(pg);
            allIds.insert(allIds.end(), ids.begin(), ids.end());
            const float4 pos = float4::point(uni(rng) * 400 - 200, uni(rng) * 100 - 50,
                                             uni(rng) * 400 - 200);
            w.addInstance(std::move(pg), pos, 0.5f + uni(rng) * 2.0f);
        }

        for (int round = 0; round < 2; ++round)
        {
            std::vector<UserId> exps;
            for (const auto& [id, r] : gen.recipes)
                if (contains(w, id) && !isAttached(w, id)) exps.push_back(id);
            std::sort(exps.begin(), exps.end());
            int budget = 16;
            for (UserId id : exps)
            {
                if (budget <= 0) break;
                if (uni(rng) > 0.4f) continue;
                Page child = gen.makeChildPage(id);
                const auto ids = pageIds(child);
                allIds.insert(allIds.end(), ids.begin(), ids.end());
                attachPage(w, id, std::move(child));
                --budget;
            }
        }

        for (UserId id : allIds)
            if (contains(w, id) && uni(rng) < 0.6f) markResident(w, id);
        w.beginFrame();
    }
};

CullView randomView(std::mt19937& rng)
{
    std::uniform_real_distribution<float> uni(0.0f, 1.0f);
    const float4 pos = float4::point(uni(rng) * 800 - 400, uni(rng) * 300 - 150,
                                     uni(rng) * 800 - 400);
    const float4 tgt = float4::point(uni(rng) * 200 - 100, 0, uni(rng) * 200 - 100);
    return makeLookAtView(pos, tgt);
}

} // namespace

// ---------------------------------------------------------------------------
// Structural invariants that must hold for ANY world and ANY camera:
//   - the actual cut is an antichain (no entry is an ancestor of another)
//     and every entry is resident;
//   - the ideal cut is an antichain; NeedsExpansion entries are unattached
//     expansion points and their handles are live;
//   - every load request targets a non-resident node with a live handle.
// ---------------------------------------------------------------------------
TEST(Contracts, CutInvariantsHoldOnRandomWorlds)
{
    std::mt19937 rng(2024);
    std::uniform_real_distribution<float> uni(0.0f, 1.0f);

    for (int iter = 0; iter < 15; ++iter)
    {
        RandomWorld rw(1000 + uint32_t(iter));
        World& w = rw.w;

        CutParams p;
        p.threshold = 1.0f + uni(rng) * 30.0f;

        for (int frame = 0; frame < 3; ++frame)
        {
            const CullView v = randomView(rng);
            const Outputs o = run(w, v, p);

            const std::set<UserId> cut = cutIds(o.cut);
            ASSERT_EQ(cut.size(), o.cut.size()) << "duplicate payloads in cut";
            for (const auto& e : o.cut)
            {
                EXPECT_TRUE(w.isResident(handleOf(w, e.payload)))
                    << "cut entry " << e.payload << " not resident";
                EXPECT_FALSE(std::isnan(e.err));
                for (UserId anc : TA::ancestorIds(w, e.payload))
                    EXPECT_EQ(cut.count(anc), 0u)
                        << "cut contains ancestor " << anc << " of " << e.payload;
            }

            std::set<UserId> ideal;
            for (const auto& e : o.ideal) ideal.insert(e.payload);
            ASSERT_EQ(ideal.size(), o.ideal.size()) << "duplicate payloads in ideal";
            for (const auto& e : o.ideal)
            {
                for (UserId anc : TA::ancestorIds(w, e.payload))
                    EXPECT_EQ(ideal.count(anc), 0u)
                        << "ideal contains ancestor " << anc << " of " << e.payload;

                const NodeHandle found = TA::requireByScan(w, e.payload);
                EXPECT_EQ(found.slot, e.node.slot);
                EXPECT_EQ(found.index, e.node.index);
                if (e.tag == IdealTag::NeedsExpansion)
                {
                    EXPECT_TRUE(TA::pageOf(w, e.payload).isExpansion(found.index));
                    EXPECT_FALSE(w.isAttached(e.node));
                }
            }

            for (const auto& r : o.reqs)
            {
                const NodeHandle found = TA::requireByScan(w, r.payload);
                EXPECT_EQ(found.slot, r.node.slot);
                EXPECT_EQ(found.index, r.node.index);
                EXPECT_FALSE(w.isResident(r.node))
                    << "request for already-resident " << r.payload;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Identical inputs must give byte-identical outputs: same world construction
// + same camera path => same cut/ideal/request sequences, in the same order,
// with bit-equal errors. (No unordered containers or pointers may leak into
// the traversal order.)
// ---------------------------------------------------------------------------
TEST(Contracts, DeterministicAcrossIdenticalWorlds)
{
    for (uint32_t seed : {7u, 42u, 314u})
    {
        RandomWorld a(seed), b(seed);
        // Damping on: the camera envelope history must match too.
        ViewDamper da(4.0f), db(4.0f);
        std::mt19937 rngA(seed * 3), rngB(seed * 3);
        const CutParams p{6.0f, 0.5f};

        for (int frame = 0; frame < 6; ++frame)
        {
            a.w.beginFrame();
            b.w.beginFrame();
            const CullView va = da.damp(randomView(rngA));
            const CullView vb = db.damp(randomView(rngB));
            const Outputs oa = run(a.w, va, p);
            const Outputs ob = run(b.w, vb, p);

            ASSERT_EQ(oa.cut.size(), ob.cut.size()) << "seed " << seed << " frame " << frame;
            for (size_t i = 0; i < oa.cut.size(); ++i)
            {
                EXPECT_EQ(oa.cut[i].payload, ob.cut[i].payload) << "pos " << i;
                EXPECT_EQ(oa.cut[i].err, ob.cut[i].err) << "pos " << i;   // bit-equal
            }
            ASSERT_EQ(oa.ideal.size(), ob.ideal.size());
            for (size_t i = 0; i < oa.ideal.size(); ++i)
            {
                EXPECT_EQ(oa.ideal[i].payload, ob.ideal[i].payload);
                EXPECT_EQ(int(oa.ideal[i].tag), int(ob.ideal[i].tag));
                EXPECT_EQ(oa.ideal[i].node.slot, ob.ideal[i].node.slot);
                EXPECT_EQ(oa.ideal[i].node.index, ob.ideal[i].node.index);
            }
            ASSERT_EQ(oa.reqs.size(), ob.reqs.size());
            for (size_t i = 0; i < oa.reqs.size(); ++i)
                EXPECT_EQ(oa.reqs[i].payload, ob.reqs[i].payload);
        }
    }
}

// ---------------------------------------------------------------------------
// Multiple damped views share the world but not each other's history: each
// view's memory lives entirely in its own ViewDamper, and selectCut is a pure
// read of the World. Interleaving view B must not change view A's outputs vs
// running A alone on an identical world (and vice versa). This is the
// regression test for moving hysteresis off the nodes onto the camera.
// ---------------------------------------------------------------------------
TEST(Contracts, MultiViewDamperIsolation)
{
    const uint32_t seed = 99;
    RandomWorld both(seed), onlyA(seed), onlyB(seed);
    for (UserId id : both.allIds)
        if (contains(both.w, id))
        {
            markResident(both.w, id);
            markResident(onlyA.w, id);
            markResident(onlyB.w, id);
        }

    // Heavy damping: the envelope history matters.
    ViewDamper dA(8.0f), dB(8.0f), dAlone(8.0f), dBlone(8.0f);
    const CutParams p{6.0f, 0.0f};

    for (int frame = 0; frame < 8; ++frame)
    {
        both.w.beginFrame();
        onlyA.w.beginFrame();
        onlyB.w.beginFrame();

        const float t = float(frame) * 0.35f;
        const CullView vA = makeLookAtView(
            float4::point(std::cos(t) * 300, 120, std::sin(t) * 300), float4::point(0, 0, 0));
        const CullView vB = makeLookAtView(
            float4::point(-std::sin(t) * 90, 25, std::cos(t) * 90), float4::point(30, 0, -20));

        std::vector<CutEntry> cutA, cutB, cutAlone, cutBlone;
        both.w.selectCut(dA.damp(vA), p, cutA);
        both.w.selectCut(dB.damp(vB), p, cutB);   // interleaved with A every frame
        onlyA.w.selectCut(dAlone.damp(vA), p, cutAlone);
        onlyB.w.selectCut(dBlone.damp(vB), p, cutBlone);

        ASSERT_EQ(cutA.size(), cutAlone.size()) << "frame " << frame;
        for (size_t i = 0; i < cutA.size(); ++i)
        {
            EXPECT_EQ(cutA[i].payload, cutAlone[i].payload) << "frame " << frame;
            EXPECT_EQ(cutA[i].err, cutAlone[i].err);
        }
        ASSERT_EQ(cutB.size(), cutBlone.size()) << "frame " << frame;
        for (size_t i = 0; i < cutB.size(); ++i)
            EXPECT_EQ(cutB[i].payload, cutBlone[i].payload) << "frame " << frame;
    }
}

// ---------------------------------------------------------------------------
// Instance slots are recycled; a stale InstanceRef (remove + add reusing the
// slot) must never act on the slot's new occupant — the generation stamp
// makes moveInstance/removeInstance safe no-ops, exactly like stale
// NodeHandles.
// ---------------------------------------------------------------------------
TEST(Contracts, StaleInstanceRefIsIgnored)
{
    TreeGen gen;
    World w;

    Page pgA = gen.makeRootPage(unitRegion(5.0f), 16.0f, 0);
    const auto refA = w.addInstance(std::move(pgA), float4::point(0, 0, 0));
    w.removeInstance(refA);

    Page pgB = gen.makeRootPage(unitRegion(5.0f), 16.0f, 0);
    const auto idsB = pageIds(pgB);
    const auto refB = w.addInstance(std::move(pgB), float4::point(0, 0, 0));
    markAllResident(w, idsB);
    ASSERT_EQ(refA.id, refB.id);   // the slot was recycled (LIFO free list)
    ASSERT_NE(refA.generation, refB.generation);

    w.beginFrame();
    const CullView v = makeLookAtView(float4::point(0, 0, -30), float4::point(0, 0, 0));
    std::vector<CutEntry> cut;
    w.selectCut(v, {4, 0}, cut);
    ASSERT_FALSE(cut.empty());

    // Stale move: B must not teleport.
    w.moveInstance(refA, float4::point(50000, 0, 0));
    cut.clear();
    w.selectCut(v, {4, 0}, cut);
    EXPECT_FALSE(cut.empty()) << "stale moveInstance displaced the new instance";

    // Stale remove: B must survive.
    w.removeInstance(refA);
    cut.clear();
    w.selectCut(v, {4, 0}, cut);
    EXPECT_FALSE(cut.empty()) << "stale removeInstance killed the new instance";

    // The live ref still works.
    w.removeInstance(refB);
    cut.clear();
    w.selectCut(v, {4, 0}, cut);
    EXPECT_TRUE(cut.empty());
}

// ---------------------------------------------------------------------------
// Geometry edge cases
// ---------------------------------------------------------------------------

// Zero-extent (point) leaves: the wide culling path and the distance kernels
// must handle degenerate boxes; the cut matches the scalar reference.
TEST(Contracts, PointLeavesMatchReference)
{
    HLodBuilder b;
    const auto root = b.createRoot(1, 64.0f, AABB::empty());
    for (int i = 0; i < 12; ++i)
    {
        const float4 c = float4::vec(float(i % 4) * 10 - 15, 0, float(i / 4) * 10 - 10);
        b.createNode(root, 100 + uint64_t(i), 0.0f, AABB::fromCenterExtent(c, float4::vec(0, 0, 0)));
    }
    Page pg = b.build();
    const auto ids = pageIds(pg);

    World w;
    w.addInstance(std::move(pg), float4::point(0, 0, 0));
    markAllResident(w, ids);
    w.beginFrame();

    const CullView v = makeLookAtView(float4::point(0, 20, -60), float4::point(0, 0, 0));
    const CutParams p{4.0f, 0.0f};
    const Outputs o = run(w, v, p);
    const RefResult want = TA::referenceCut(w, v, p);

    std::set<UserId> wantIds;
    for (const auto& e : want.cut) wantIds.insert(e.payload);
    EXPECT_EQ(cutIds(o.cut), wantIds);
    EXPECT_FALSE(o.cut.empty());
}

// Camera inside the tree: distances hit zero, screen errors saturate to huge
// finite values, nothing NaNs, and the result still matches the reference.
TEST(Contracts, CameraInsideTreeMatchesReference)
{
    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 2;
    Page pg = gen.makeRootPage(unitRegion(50.0f), 64.0f, 0);
    const auto ids = pageIds(pg);

    World w;
    w.addInstance(std::move(pg), float4::point(0, 0, 0));
    markAllResident(w, ids);
    w.beginFrame();

    const CullView v = makeLookAtView(float4::point(1, 2, 3), float4::point(40, 0, 40));
    const CutParams p{4.0f, 0.0f};
    const Outputs o = run(w, v, p);
    const RefResult want = TA::referenceCut(w, v, p);

    ASSERT_FALSE(o.cut.empty());
    for (const auto& e : o.cut) EXPECT_FALSE(std::isnan(e.err)) << e.payload;

    std::set<UserId> wantIds;
    for (const auto& e : want.cut) wantIds.insert(e.payload);
    EXPECT_EQ(cutIds(o.cut), wantIds);
}

// A world far from the origin (1e6 units out): absolute-coordinate math must
// not diverge between the wide path and the scalar reference.
TEST(Contracts, FarFromOriginMatchesReference)
{
    TreeGen gen;
    Page pg = gen.makeRootPage(unitRegion(20.0f), 32.0f, 0);
    const auto ids = pageIds(pg);

    const float4 farPos = float4::point(1.0e6f, 0, 1.0e6f);
    World w;
    w.addInstance(std::move(pg), farPos);
    markAllResident(w, ids);
    w.beginFrame();

    const CullView v = makeLookAtView(farPos + float4::vec(0, 30, -80), farPos);
    const CutParams p{4.0f, 0.0f};
    const Outputs o = run(w, v, p);
    const RefResult want = TA::referenceCut(w, v, p);

    std::set<UserId> wantIds;
    for (const auto& e : want.cut) wantIds.insert(e.payload);
    EXPECT_EQ(cutIds(o.cut), wantIds);
    EXPECT_FALSE(o.cut.empty());
}

// Extreme instance scale: the local-space transform keeps error selection
// scale-invariant and equivalent to the reference.
TEST(Contracts, ScaledInstanceMatchesReference)
{
    TreeGen gen;
    for (float scale : {0.01f, 1.0f, 250.0f})
    {
        Page pg = gen.makeRootPage(unitRegion(10.0f), 32.0f, 0);
        const auto ids = pageIds(pg);

        World w;
        w.addInstance(std::move(pg), float4::point(0, 0, 0), scale);
        markAllResident(w, ids);
        w.beginFrame();

        const CullView v = makeLookAtView(float4::point(0, 10 * scale, -40 * scale),
                                          float4::point(0, 0, 0));
        const CutParams p{4.0f, 0.0f};
        const Outputs o = run(w, v, p);
        const RefResult want = TA::referenceCut(w, v, p);

        std::set<UserId> wantIds;
        for (const auto& e : want.cut) wantIds.insert(e.payload);
        EXPECT_EQ(cutIds(o.cut), wantIds) << "scale " << scale;
        EXPECT_FALSE(o.cut.empty()) << "scale " << scale;
    }
}

// NaN / empty / infinite boxes are rejected at the API boundary. Grow-only
// refit can never un-grow, so one NaN box would poison ancestors forever.
TEST(Contracts, NonFiniteBoundsRejected)
{
    TreeGen gen;
    Page pg = gen.makeRootPage(unitRegion(5.0f), 16.0f, 0);
    World w;
    const auto inst = w.addInstance(std::move(pg), float4::point(0, 0, 0));
    const NodeHandle leaf = nodeAt(inst.rootPage, 2);

    const float nan = std::nanf("");
    const float inf = std::numeric_limits<float>::infinity();
    EXPECT_THROW(w.setNodeBounds(inst, leaf, AABB::fromCenterExtent(
                     float4::vec(nan, 0, 0), float4::vec(1, 1, 1))), std::logic_error);
    EXPECT_THROW(w.setNodeBounds(inst, leaf, AABB::fromCenterExtent(
                     float4::vec(0, 0, 0), float4::vec(nan, 1, 1))), std::logic_error);
    EXPECT_THROW(w.setNodeBounds(inst, leaf, AABB::fromCenterExtent(
                     float4::vec(0, 0, 0), float4::vec(inf, 1, 1))), std::logic_error);
    EXPECT_THROW(w.setNodeBounds(inst, leaf, AABB::empty()), std::logic_error);

    // Zero-extent is legal (a point object).
    EXPECT_NO_THROW(w.setNodeBounds(inst, leaf, AABB::fromCenterExtent(
                        float4::vec(1, 2, 3), float4::vec(0, 0, 0))));
    w.flushBounds();
}

// ---------------------------------------------------------------------------
// Memory budgets: catch accidental per-node or per-page bloat. Page data is
// the dominant cost at scale; the budgets are deliberately loose enough to
// survive refactors but tight enough to flag a new hot-array or a hash map
// sneaking back in.
// ---------------------------------------------------------------------------
TEST(Contracts, MemoryBudgets)
{
    TreeGen gen8;
    gen8.fanout = 8;
    gen8.depth = 2;
    const Page p8 = gen8.makeRootPage(unitRegion(50.0f), 64.0f, 0);
    RecordProperty("bytes_per_node_fanout8", int(p8.byteSize() / p8.nodeCount()));
    EXPECT_LE(p8.byteSize() / p8.nodeCount(), 128u);

    TreeGen gen4;
    gen4.fanout = 4;
    gen4.depth = 3;
    const Page p4 = gen4.makeRootPage(unitRegion(50.0f), 64.0f, 0);
    RecordProperty("bytes_per_node_fanout4", int(p4.byteSize() / p4.nodeCount()));
    EXPECT_LE(p4.byteSize() / p4.nodeCount(), 176u);

#ifdef NDEBUG
    // Fixed per-page World overhead (slot bookkeeping, not page data).
    RecordProperty("pagert_bytes", int(TA::pageRtBytes()));
    EXPECT_LE(TA::pageRtBytes(), 512u);
#endif
}
