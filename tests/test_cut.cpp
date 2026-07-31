#include <gtest/gtest.h>

#include <algorithm>
#include <map>
#include <random>
#include <set>

#include "helpers.h"

using namespace hlod;
using namespace hlodtest;
using TA = World::TestAccess;

namespace {

std::map<UserId, float> cutMap(const std::vector<CutEntry>& v)
{
    std::map<UserId, float> m;
    for (const auto& e : v) m.emplace(e.payload, e.err);
    return m;
}
std::map<UserId, std::pair<float, IdealTag>> idealMap(const std::vector<IdealEntry>& v)
{
    std::map<UserId, std::pair<float, IdealTag>> m;
    for (const auto& e : v) m.emplace(e.payload, std::make_pair(e.err, e.tag));
    return m;
}
std::set<UserId> reqSet(const std::vector<LoadRequest>& v)
{
    std::set<UserId> s;
    for (const auto& e : v) s.insert(e.payload);
    return s;
}

void expectSameCut(const std::vector<CutEntry>& got, const std::vector<CutEntry>& want)
{
    const auto g = cutMap(got), w = cutMap(want);
    ASSERT_EQ(g.size(), got.size()) << "duplicate ids in cut";
    ASSERT_EQ(g.size(), w.size());
    for (const auto& [id, err] : w)
    {
        auto it = g.find(id);
        ASSERT_NE(it, g.end()) << "missing id " << id;
        EXPECT_NEAR(it->second, err, 1e-3f * std::max(1.0f, err)) << "id " << id;
    }
}

void expectSameIdeal(const std::vector<IdealEntry>& got, const std::vector<IdealEntry>& want)
{
    const auto g = idealMap(got), w = idealMap(want);
    ASSERT_EQ(g.size(), got.size()) << "duplicate ids in ideal cut";
    ASSERT_EQ(g.size(), w.size());
    for (const auto& [id, ew] : w)
    {
        auto it = g.find(id);
        ASSERT_NE(it, g.end()) << "missing id " << id;
        EXPECT_EQ(int(it->second.second), int(ew.second)) << "tag of id " << id;
    }
}

struct Outputs
{
    std::vector<CutEntry>    cut;
    std::vector<IdealEntry>  ideal;
    std::vector<LoadRequest> reqs;
};

Outputs run(World& w, ViewScratch& s, const CullView& v, const CutParams& p)
{
    Outputs o;
    w.selectCut(v, p, s, o.cut, o.ideal, o.reqs);
    return o;
}

} // namespace

// ---------------------------------------------------------------------------
// The doc's town example: near building refines to walls, far one draws whole.
// ---------------------------------------------------------------------------
TEST(Cut, TownExample)
{
    HLodBuilder b;
    const auto town = b.createRoot(1, 64.0f);
    const auto bldA = b.createNode(town, 2, 8.0f);
    b.createNode(bldA, 10, 1.0f, AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(1, 1, 1)));
    b.createNode(bldA, 11, 1.0f, AABB::fromCenterExtent(float4::vec(3, 0, 0), float4::vec(1, 1, 1)));
    b.createNode(bldA, 12, 1.0f, AABB::fromCenterExtent(float4::vec(6, 0, 0), float4::vec(1, 1, 1)));
    b.createNode(town, 3, 8.0f, AABB::fromCenterExtent(float4::vec(500, 0, 0), float4::vec(5, 5, 5)));
    Page pg = b.build();
    const auto ids = pageIds(pg);

    World w;
    w.addInstance(std::move(pg), float4::point(0, 0, 0));
    markAllResident(w, ids);
    w.beginFrame();

    ViewScratch s;
    // Both buildings on screen; A close enough to refine into walls, the
    // walls themselves fine at this distance, B drawn as one proxy.
    const CullView v = makeLookAtView(float4::point(250, 0, -600), float4::point(250, 0, 0));
    const auto o = run(w, s, v, {4.0f, 0.0f, 0.0f});

    std::set<UserId> got;
    for (const auto& e : o.cut) got.insert(e.payload);
    EXPECT_EQ(got, (std::set<UserId>{10, 11, 12, 3}));   // walls + far building
    EXPECT_TRUE(o.reqs.empty());

    // Everything resident: ideal == actual.
    expectSameCut(o.cut, [&] {
        std::vector<CutEntry> c;
        for (const auto& e : o.ideal) c.push_back({e.payload, e.err});
        return c;
    }());
}

// ---------------------------------------------------------------------------
// Distance monotonicity: moving away coarsens the cut down to the root.
// ---------------------------------------------------------------------------
TEST(Cut, CoarsensWithDistance)
{
    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 3;
    Page pg = gen.makeRootPage(unitRegion(50.0f), 64.0f, 0);
    const auto ids = pageIds(pg);

    World w;
    w.addInstance(std::move(pg), float4::point(0, 0, 0));
    markAllResident(w, ids);
    w.beginFrame();

    ViewScratch s;
    size_t lastSize = SIZE_MAX;
    for (float d : {80.0f, 300.0f, 1500.0f, 30000.0f})
    {
        const CullView v = makeLookAtView(float4::point(0, 0, -d), float4::point(0, 0, 0));
        const auto o = run(w, s, v, {4.0f, 0.0f, 0.0f});
        ASSERT_FALSE(o.cut.empty()) << "distance " << d;
        EXPECT_LE(o.cut.size(), lastSize) << "distance " << d;
        lastSize = o.cut.size();
    }
    EXPECT_EQ(lastSize, 1u);   // far enough: just the root
}

// ---------------------------------------------------------------------------
// The cut is an antichain: no drawn node is an ancestor of another drawn node.
// ---------------------------------------------------------------------------
TEST(Cut, IsAntichain)
{
    TreeGen gen;
    gen.fanout = 3;
    gen.depth = 2;
    Page root = gen.makeRootPage(unitRegion(60.0f), 64.0f, 1);
    const auto rootIds = pageIds(root);

    World w;
    w.addInstance(std::move(root), float4::point(0, 0, 0));
    markAllResident(w, rootIds);

    // Attach and fill some child pages, leave others collapsed.
    int n = 0;
    for (const auto& [expId, recipe] : std::map<UserId, TreeGen::Recipe>(
             gen.recipes.begin(), gen.recipes.end()))
    {
        if (++n % 2) continue;
        Page child = gen.makeChildPage(expId);
        const auto childIds = pageIds(child);
        attachPage(w, expId, std::move(child));
        markAllResident(w, childIds);
    }
    w.beginFrame();

    ViewScratch s;
    const CullView v = makeLookAtView(float4::point(10, 20, -100), float4::point(0, 0, 0));
    const auto o = run(w, s, v, {6.0f, 0.0f, 0.0f});

    std::set<UserId> inCut;
    for (const auto& e : o.cut) inCut.insert(e.payload);
    for (const auto& e : o.cut)
        for (UserId anc : TA::ancestorIds(w, e.payload))
            EXPECT_FALSE(inCut.count(anc)) << "node " << e.payload << " and ancestor " << anc;
}

// ---------------------------------------------------------------------------
// Brute-force reference equivalence over randomized worlds: random paged
// trees, random attachments, random residency, random cameras, multi-instance.
// ---------------------------------------------------------------------------
TEST(Cut, MatchesBruteForceReference)
{
    std::mt19937 rng(777);
    std::uniform_real_distribution<float> uni(0.0f, 1.0f);

    for (int iter = 0; iter < 40; ++iter)
    {
        TreeGen gen;
        gen.fanout = 2 + uint32_t(rng() % 3);
        gen.depth = 1 + uint32_t(rng() % 2);

        World w;
        std::vector<UserId> allIds;

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

        // Randomly attach expansion pages, capped per round: the recipe pool
        // grows combinatorially with each attach and would otherwise explode.
        for (int round = 0; round < 2; ++round)
        {
            std::vector<UserId> exps;
            for (const auto& [id, r] : gen.recipes)
                if (contains(w, id) && !isAttached(w, id)) exps.push_back(id);
            std::sort(exps.begin(), exps.end());
            int budget = 24;
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

        // Random residency.
        for (UserId id : allIds)
            if (contains(w, id) && uni(rng) < 0.6f) markResident(w, id);

        w.beginFrame();

        CutParams p;
        p.threshold = 1.0f + uni(rng) * 30.0f;
        p.hysteresis = 0.0f;   // reference keeps no history
        p.minPix = (iter % 4 == 0) ? 0.5f : 0.0f;

        const float4 camPos = float4::point(uni(rng) * 800 - 400, uni(rng) * 300 - 150,
                                            uni(rng) * 800 - 400);
        const float4 camTgt = float4::point(uni(rng) * 200 - 100, 0, uni(rng) * 200 - 100);
        const CullView v = makeLookAtView(camPos, camTgt);

        ViewScratch s;
        const auto got = run(w, s, v, p);
        const RefResult want = TA::referenceCut(w, v, p);

        SCOPED_TRACE("iter " + std::to_string(iter));
        expectSameCut(got.cut, want.cut);
        expectSameIdeal(got.ideal, want.ideal);
        EXPECT_EQ(reqSet(got.reqs), reqSet(want.requests));
    }
}

// ---------------------------------------------------------------------------
// Temporal coherence: identical frames produce identical cuts (epoch stamps
// must not leak stale state between frames).
// ---------------------------------------------------------------------------
TEST(Cut, StableAcrossFrames)
{
    TreeGen gen;
    Page pg = gen.makeRootPage(unitRegion(50.0f), 64.0f, 0);
    const auto ids = pageIds(pg);

    World w;
    w.addInstance(std::move(pg), float4::point(0, 0, 0));
    markAllResident(w, ids);

    ViewScratch s;
    const CullView v = makeLookAtView(float4::point(20, 10, -90), float4::point(0, 0, 0));
    w.beginFrame();
    const auto first = run(w, s, v, {5.0f, 0.1f, 0.0f});
    for (int f = 0; f < 5; ++f)
    {
        w.beginFrame();
        const auto o = run(w, s, v, {5.0f, 0.1f, 0.0f});
        expectSameCut(o.cut, first.cut);
    }
}

// ---------------------------------------------------------------------------
// Hysteresis: refine at threshold*(1+h), un-refine at threshold*(1-h);
// between the bars the previous decision is sticky.
// ---------------------------------------------------------------------------
TEST(Cut, Hysteresis)
{
    HLodBuilder b;
    const auto root = b.createRoot(1, 1.0f);   // ge = 1
    b.createNode(root, 2, 0.001f, AABB::fromCenterExtent(float4::vec(-0.01f, 0, 0),
                                                         float4::vec(0.005f, 0.01f, 0.01f)));
    b.createNode(root, 3, 0.001f, AABB::fromCenterExtent(float4::vec(0.01f, 0, 0),
                                                         float4::vec(0.005f, 0.01f, 0.01f)));
    Page pg = b.build();
    const auto ids = pageIds(pg);

    World w;
    w.addInstance(std::move(pg), float4::point(0, 0, 0));
    markAllResident(w, ids);

    const CutParams p{10.0f, 0.2f, 0.0f};   // bars at 12 and 8
    ViewScratch s;

    // err(root) = ge * k / dist; the root box is tiny around the origin.
    const float k = makeLookAtView(float4::point(0, 0, -1), float4::point(0, 0, 0)).k;
    auto camAtErr = [&](float errTarget)
    {
        const float dist = 1.0f * k / errTarget;
        return makeLookAtView(float4::point(0, 0, -(dist + 0.01f)), float4::point(0, 0, 0));
    };
    auto cutIsRoot = [&](const Outputs& o)
    {
        return o.cut.size() == 1 && o.cut[0].payload == 1;
    };

    w.beginFrame();
    EXPECT_TRUE(cutIsRoot(run(w, s, camAtErr(11.0f), p)));    // below start bar: coarse
    w.beginFrame();
    EXPECT_TRUE(cutIsRoot(run(w, s, camAtErr(11.9f), p)));    // still sticky-coarse
    w.beginFrame();
    EXPECT_FALSE(cutIsRoot(run(w, s, camAtErr(13.0f), p)));   // crossed 12: refined
    w.beginFrame();
    EXPECT_FALSE(cutIsRoot(run(w, s, camAtErr(9.0f), p)));    // sticky-refined above 8
    w.beginFrame();
    EXPECT_TRUE(cutIsRoot(run(w, s, camAtErr(7.0f), p)));     // below 8: coarse again
}

// ---------------------------------------------------------------------------
// Contribution culling: a subpixel instance vanishes entirely; a visible one
// stays untouched.
// ---------------------------------------------------------------------------
TEST(Cut, ContributionCulling)
{
    TreeGen gen;
    Page near = gen.makeRootPage(unitRegion(10.0f), 8.0f, 0);
    const auto nearIds = pageIds(near);
    Page far = gen.makeRootPage(
        AABB::fromMinMax(float4::vec(-10, -10, -10), float4::vec(10, 10, 10)), 8.0f, 0);
    const auto farIds = pageIds(far);

    World w;
    w.addInstance(std::move(near), float4::point(0, 0, 0));
    w.addInstance(std::move(far), float4::point(0, 0, 500000.0f), 0.01f);   // tiny & distant
    markAllResident(w, nearIds);
    markAllResident(w, farIds);
    w.beginFrame();

    ViewScratch s;
    const CullView v = makeLookAtView(float4::point(0, 5, -60), float4::point(0, 0, 100));

    const auto without = run(w, s, v, {4.0f, 0.0f, 0.0f});
    ViewScratch s2;
    const auto with = run(w, s2, v, {4.0f, 0.0f, 0.5f});

    std::set<UserId> withoutIds, withIds;
    for (const auto& e : without.cut) withoutIds.insert(e.payload);
    for (const auto& e : with.cut) withIds.insert(e.payload);

    bool farInWithout = false, farInWith = false;
    for (UserId id : farIds)
    {
        farInWithout |= withoutIds.count(id) != 0;
        farInWith |= withIds.count(id) != 0;
    }
    EXPECT_TRUE(farInWithout);
    EXPECT_FALSE(farInWith);
    for (UserId id : nearIds)
        if (withoutIds.count(id)) EXPECT_TRUE(withIds.count(id)) << id;
}

// ---------------------------------------------------------------------------
// Two views over the same world get independent scratch and correct cuts.
// ---------------------------------------------------------------------------
TEST(Cut, MultiViewIndependence)
{
    TreeGen gen;
    Page pg = gen.makeRootPage(unitRegion(50.0f), 64.0f, 0);
    const auto ids = pageIds(pg);

    World w;
    w.addInstance(std::move(pg), float4::point(0, 0, 0));
    markAllResident(w, ids);

    ViewScratch sNear, sFar;
    const CullView vNear = makeLookAtView(float4::point(0, 0, -70), float4::point(0, 0, 0));
    const CullView vFar = makeLookAtView(float4::point(0, 0, -20000), float4::point(0, 0, 0));

    for (int f = 0; f < 3; ++f)
    {
        w.beginFrame();
        const auto oNear = run(w, sNear, vNear, {4.0f, 0.1f, 0.0f});
        const auto oFar = run(w, sFar, vFar, {4.0f, 0.1f, 0.0f});
        EXPECT_GT(oNear.cut.size(), oFar.cut.size());
        EXPECT_EQ(oFar.cut.size(), 1u);
    }
}
