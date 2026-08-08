#include <gtest/gtest.h>

#include <algorithm>
#include <map>
#include <set>

#include "helpers.h"

using namespace hlod;
using namespace hlodtest;
using TA = World::TestAccess;

namespace {

std::map<UserId, uint8_t> cutMap(World& world, const CutView& cut)
{
    std::map<UserId, uint8_t> m;
    for (const auto& e : currentCut(cut))
        m.emplace(payloadOf(world, e), e.errorCode());
    return m;
}
std::map<UserId, uint8_t> idealMap(World& world, const CutView& cut)
{
    std::map<UserId, uint8_t> m;
    for (const auto& e : idealCut(cut))
        m.emplace(payloadOf(world, e), e.errorCode());
    return m;
}

void expectSameCut(World& world, const CutView& got, const CutView& want)
{
    const auto g = cutMap(world, got), w = cutMap(world, want);
    ASSERT_EQ(g.size(), currentCutSize(got)) << "duplicate ids in current cut";
    ASSERT_EQ(g.size(), w.size());
    for (const auto& [id, error] : w)
    {
        auto it = g.find(id);
        ASSERT_NE(it, g.end()) << "missing id " << id;
        EXPECT_EQ(it->second, error) << "id " << id;
    }
}

void expectSameIdeal(World& world, const CutView& got, const CutView& want)
{
    const auto g = idealMap(world, got), w = idealMap(world, want);
    ASSERT_EQ(g.size(), idealCutSize(got)) << "duplicate ids in ideal cut";
    ASSERT_EQ(g.size(), w.size());
    for (const auto& [id, error] : w)
    {
        auto it = g.find(id);
        ASSERT_NE(it, g.end()) << "missing id " << id;
        EXPECT_EQ(it->second, error) << "error code of id " << id;
    }
}

struct Outputs
{
    CutResults cut;
};

Outputs run(World& w, const Camera& v, const CutParams& p)
{
    Outputs o;
    selectCutUncached(w, v, p, o.cut);
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
    w.applyUpdates();

    // Both buildings on screen; A close enough to refine into walls, the
    // walls themselves fine at this distance, B drawn as one proxy.
    const Camera v = makeLookAtCamera(float4::point(250, 0, -600), float4::point(250, 0, 0));
    const auto o = run(w, v, {4.0f, 0.0f});

    std::set<UserId> got;
    for (const auto& e : currentCut(o.cut)) got.insert(payloadOf(w, e));
    EXPECT_EQ(got, (std::set<UserId>{10, 11, 12, 3}));   // walls + far building

    // Everything resident: ideal == current.
    EXPECT_EQ(cutMap(w, o.cut), idealMap(w, o.cut));
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
    w.applyUpdates();

    size_t lastSize = SIZE_MAX;
    for (float d : {80.0f, 300.0f, 1500.0f, 30000.0f})
    {
        const Camera v = makeLookAtCamera(float4::point(0, 0, -d), float4::point(0, 0, 0));
        const auto o = run(w, v, {4.0f, 0.0f});
        ASSERT_FALSE(o.cut.empty()) << "distance " << d;
        EXPECT_LE(currentCutSize(o.cut), lastSize) << "distance " << d;
        lastSize = currentCutSize(o.cut);
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
    w.applyUpdates();

    const Camera v = makeLookAtCamera(float4::point(10, 20, -100), float4::point(0, 0, 0));
    const auto o = run(w, v, {6.0f, 0.0f});

    std::set<UserId> inCut;
    for (const auto& e : currentCut(o.cut)) inCut.insert(payloadOf(w, e));
    for (const auto& e : currentCut(o.cut))
    {
        const UserId id = payloadOf(w, e);
        for (UserId anc : TA::ancestorIds(w, id))
            EXPECT_FALSE(inCut.count(anc)) << "node " << id << " and ancestor " << anc;
    }
}

// ---------------------------------------------------------------------------
// Brute-force reference equivalence over randomized worlds: random paged
// trees, random attachments, random residency, random cameras, multi-instance.
// ---------------------------------------------------------------------------
TEST(Cut, MatchesBruteForceReference)
{
    DeterministicRng rng(777);
    DeterministicUniformFloat uni(0.0f, 1.0f);

    for (int iter = 0; iter < 40; ++iter)
    {
        TreeGen gen;
        gen.fanout = 2 + rng.index(3);
        gen.depth = 1 + rng.index(2);

        World w;
        std::vector<UserId> allIds;

        const int numInstances = 1 + int(rng.index(3));
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

        w.applyUpdates();

        CutParams p;
        p.threshold = 1.0f + uni(rng) * 30.0f;
        p.minPix = (iter % 4 == 0) ? 0.5f : 0.0f;

        const float4 camPos = float4::point(uni(rng) * 800 - 400, uni(rng) * 300 - 150,
                                            uni(rng) * 800 - 400);
        const float4 camTgt = float4::point(uni(rng) * 200 - 100, 0, uni(rng) * 200 - 100);
        const Camera v = makeLookAtCamera(camPos, camTgt);

        const auto got = run(w, v, p);
        const RefResult want = TA::referenceCut(w, v, p);

        SCOPED_TRACE("iter " + std::to_string(iter));
        expectSameCut(w, got.cut, want.cut);
        expectSameIdeal(w, got.cut, want.cut);
    }
}

// A far hierarchical BLAS can terminate at its renderable root directly from
// the TLAS leaf. Moving the camera closer must still refine normally. This
// guards both sides of the shortcut against the scalar reference.
TEST(Cut, TlasRootDecisionMatchesReference)
{
    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 3;
    Page page = gen.makeRootPage(unitRegion(3.0f), 16.0f, 0);
    const auto ids = pageIds(page);

    World w;
    w.addInstance(std::move(page), float4::point(0, 0, 0));
    markAllResident(w, ids);
    w.applyUpdates();

    const CutParams params{4.0f, 0.0f};
    const Camera far = makeLookAtCamera(float4::point(0, 0, -5000),
                                        float4::point(0, 0, 0));
    const auto farGot = run(w, far, params);
    const RefResult farWant = TA::referenceCut(w, far, params);
    expectSameCut(w, farGot.cut, farWant.cut);
    expectSameIdeal(w, farGot.cut, farWant.cut);
    ASSERT_EQ(farGot.cut.shared.size(), 1u);
    EXPECT_EQ(payloadOf(w, farGot.cut.shared.front()), ids.front());

    const Camera near = makeLookAtCamera(float4::point(0, 0, -1000),
                                         float4::point(0, 0, 0));
    const auto nearGot = run(w, near, params);
    const RefResult nearWant = TA::referenceCut(w, near, params);
    expectSameCut(w, nearGot.cut, nearWant.cut);
    expectSameIdeal(w, nearGot.cut, nearWant.cut);
    EXPECT_GT(nearGot.cut.size(), 1u);
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

    const Camera v = makeLookAtCamera(float4::point(20, 10, -90), float4::point(0, 0, 0));
    w.applyUpdates();
    const auto first = run(w, v, {5.0f, 0.0f});
    for (int f = 0; f < 5; ++f)
    {
        w.applyUpdates();
        const auto o = run(w, v, {5.0f, 0.0f});
        expectSameCut(w, o.cut, first.cut);
    }
}

// ---------------------------------------------------------------------------
// LOD damping. There is no per-node hysteresis state any more: the memory
// lives on the camera as an envelope of where it has recently been, and error
// is measured to the nearest point of that envelope. These tests pin the
// three properties that buys.
// ---------------------------------------------------------------------------
namespace {

// A tiny tree whose root has geometric error 1, so err(root) = k / dist.
struct DampFixture
{
    World w;
    float k;

    DampFixture()
    {
        HLodBuilder b;
        const auto root = b.createRoot(1, 1.0f);
        b.createNode(root, 2, 0.001f,
                     AABB::fromCenterExtent(float4::vec(-0.01f, 0, 0),
                                            float4::vec(0.005f, 0.01f, 0.01f)));
        b.createNode(root, 3, 0.001f,
                     AABB::fromCenterExtent(float4::vec(0.01f, 0, 0),
                                            float4::vec(0.005f, 0.01f, 0.01f)));
        Page pg = b.build();
        const auto ids = pageIds(pg);
        w.addInstance(std::move(pg), float4::point(0, 0, 0));
        markAllResident(w, ids);
        k = makeLookAtCamera(float4::point(0, 0, -1), float4::point(0, 0, 0)).k;
    }

    // A camera placed so the root projects to `errTarget` pixels.
    Camera camAtErr(float errTarget) const
    {
        const float dist = 1.0f * k / errTarget;
        return makeLookAtCamera(float4::point(0, 0, -(dist + 0.01f)),
                              float4::point(0, 0, 0));
    }

    bool coarse(const Camera& v, const CutParams& p)
    {
        w.applyUpdates();
        const Outputs o = run(w, v, p);
        const auto current = currentCut(o.cut);
        return current.size() == 1 && payloadOf(w, current[0]) == 1;
    }
};

} // namespace

// A camera jittering around the threshold is the case hysteresis exists for.
// Undamped it flips every frame; damped, the envelope covers both positions,
// so the measured error is the closer one's and the decision never moves.
TEST(Cut, DampingSurvivesJitter)
{
    const CutParams p{10.0f, 0.0f};

    {
        DampFixture f;
        CameraDamper none(0.0f);
        std::vector<bool> flips;
        for (int i = 0; i < 6; ++i)
            flips.push_back(f.coarse(none.damp(f.camAtErr(i % 2 ? 9.5f : 10.5f)), p));
        // Undamped: strictly alternating, i.e. a visible pop every frame.
        for (size_t i = 1; i < flips.size(); ++i)
            EXPECT_NE(flips[i], flips[i - 1]) << "frame " << i;
    }
    {
        DampFixture f;
        CameraDamper damper(4.0f);
        std::vector<bool> flips;
        for (int i = 0; i < 6; ++i)
            flips.push_back(f.coarse(damper.damp(f.camAtErr(i % 2 ? 9.5f : 10.5f)), p));
        // Damped: one steady decision, and it is the refined one (the envelope
        // is conservative, so damping never gives up detail it already had).
        for (size_t i = 1; i < flips.size(); ++i)
            EXPECT_EQ(flips[i], flips[i - 1]) << "frame " << i;
        EXPECT_FALSE(flips.back());
    }
}

// Detail must arrive the instant the camera gets close enough, and be given up
// only reluctantly. The envelope always contains the current position, so
// approach is undamped by construction; recede is damped by the decay.
TEST(Cut, DampingIsAsymmetric)
{
    const CutParams p{10.0f, 0.0f};
    DampFixture f;
    CameraDamper damper(4.0f);

    // Settle far away and coarse.
    for (int i = 0; i < 12; ++i) EXPECT_TRUE(f.coarse(damper.damp(f.camAtErr(7.0f)), p));

    // Approach: refined on the very first frame past the threshold.
    EXPECT_FALSE(f.coarse(damper.damp(f.camAtErr(13.0f)), p));

    // Recede: still refined immediately after falling below the threshold...
    EXPECT_FALSE(f.coarse(damper.damp(f.camAtErr(7.0f)), p));
    // ...and eventually collapses as the envelope relaxes onto the camera.
    bool collapsed = false;
    for (int i = 0; i < 20 && !collapsed; ++i)
        collapsed = f.coarse(damper.damp(f.camAtErr(7.0f)), p);
    EXPECT_TRUE(collapsed);
}

// Damping off must be bit-identical to no damping at all, so every existing
// expectation about undamped selection stays exactly valid.
TEST(Cut, DampingOffIsExact)
{
    const CutParams p{10.0f, 0.0f};
    DampFixture f;
    CameraDamper off(0.0f);

    for (float err : {11.0f, 9.0f, 13.0f, 7.0f, 10.001f})
    {
        const Camera raw = f.camAtErr(err);
        const Camera damped = off.damp(raw);
        EXPECT_EQ(damped.envLo.x, 0.0f);
        EXPECT_EQ(damped.envHi.x, 0.0f);
        EXPECT_FALSE(damped.damped());
        EXPECT_EQ(damped.k, raw.k);
        // Same view in, same decision out.
        DampFixture a, b;
        EXPECT_EQ(a.coarse(raw, p), b.coarse(damped, p)) << "err " << err;
    }
}

// reset() forgets the window, so a teleport does not stretch the envelope
// across the discontinuity and over-refine everything in between.
TEST(Cut, DamperResetForgetsTheWindow)
{
    const CutParams p{10.0f, 0.0f};
    DampFixture f;
    CameraDamper damper(8.0f);

    for (int i = 0; i < 8; ++i) f.coarse(damper.damp(f.camAtErr(20.0f)), p);
    EXPECT_FALSE(f.coarse(damper.damp(f.camAtErr(3.0f)), p));   // envelope holds detail

    damper.reset();
    EXPECT_TRUE(f.coarse(damper.damp(f.camAtErr(3.0f)), p));    // history gone
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
    w.applyUpdates();

    const Camera v = makeLookAtCamera(float4::point(0, 5, -60), float4::point(0, 0, 100));

    const auto without = run(w, v, {4.0f, 0.0f});
    const auto with = run(w, v, {4.0f, 0.5f});

    std::set<UserId> withoutIds, withIds;
    for (const auto& e : currentCut(without.cut))
        withoutIds.insert(payloadOf(w, e));
    for (const auto& e : currentCut(with.cut)) withIds.insert(payloadOf(w, e));

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
// Two views over the same world produce independent, correct cuts.
// ---------------------------------------------------------------------------
TEST(Cut, MultiViewIndependence)
{
    TreeGen gen;
    Page pg = gen.makeRootPage(unitRegion(50.0f), 64.0f, 0);
    const auto ids = pageIds(pg);

    World w;
    w.addInstance(std::move(pg), float4::point(0, 0, 0));
    markAllResident(w, ids);

    const Camera vNear = makeLookAtCamera(float4::point(0, 0, -70), float4::point(0, 0, 0));
    const Camera vFar = makeLookAtCamera(float4::point(0, 0, -20000), float4::point(0, 0, 0));

    for (int f = 0; f < 3; ++f)
    {
        w.applyUpdates();
        const auto oNear = run(w, vNear, {4.0f, 0.0f});
        const auto oFar = run(w, vFar, {4.0f, 0.0f});
        EXPECT_GT(currentCutSize(oNear.cut), currentCutSize(oFar.cut));
        EXPECT_EQ(currentCutSize(oFar.cut), 1u);
    }
}
