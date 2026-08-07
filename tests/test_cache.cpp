// View: temporal reuse must be invisible.
//
// The whole value of the cache rests on one claim -- that the set of nodes it
// hands back is exactly the set an uncached selectCut would have produced --
// so that claim is what these tests attack, frame after frame, while the
// camera moves and the world churns underneath. The quantized error code is
// deliberately not compared: it is the recorded value, stale within the
// proven margin, and that is the one documented approximation.

#include <algorithm>
#include <array>
#include <barrier>
#include <exception>
#include <random>
#include <thread>
#include <tuple>

#include <gtest/gtest.h>

#include "helpers.h"

using namespace hlodtest;

namespace {

// Bucketed entry identities, sorted as a multiset; only the intentionally stale
// cached error and emission order are dropped.
using Keys = std::vector<std::tuple<uint32_t, uint32_t, uint32_t, uint32_t>>;

Keys keysOf(const CutResults& cut)
{
    Keys k;
    k.reserve(cut.size());
    const auto append = [&](const auto& entries, uint32_t bucket)
    {
        for (const CutEntry& e : entries)
            k.push_back({bucket, e.instance(), e.nodeHandle.lo, e.nodeHandle.hi});
    };
    append(cut.shared, 0);
    append(cut.currentOnly, 1);
    append(cut.idealOnly, 2);
    std::sort(k.begin(), k.end());
    return k;
}

Camera viewAt(float4 pos)
{
    return makePerspectiveCamera(pos, float4::vec(0, 0, 1), float4::vec(0, 1, 0), 1.2f,
                               1.7778f, 1080.0f, 0.5f, 40000.0f);
}

// A grid of instances of one shared asset, sitting well down the +Z axis so
// most of them are wholly inside the frustum (which is what makes them
// cacheable at all).
struct Scene
{
    TreeGen                       gen;
    std::unique_ptr<World>        world;
    AssetHandle                   asset;
    std::vector<World::InstanceRef> inst;

    explicit Scene(uint32_t side = 6, uint32_t depth = 3, uint32_t fanout = 4,
                   const WorldConfig& config = WorldConfig{})
    {
        gen.fanout = fanout;
        gen.depth = depth;
        world = std::make_unique<World>(config);
        Page p = gen.makeRootPage(unitRegion(60.0f), 512.0f, 0);
        const uint32_t nodes = p.nodeCount();
        asset = world->registerAsset(std::move(p));
        for (uint32_t y = 0; y < side; ++y)
            for (uint32_t x = 0; x < side; ++x)
                inst.push_back(world->addInstance(
                    asset, float4::vec(float(x) * 300.0f - float(side) * 150.0f,
                                       float(y) * 300.0f - float(side) * 150.0f,
                                       2500.0f + float((x + y) % 3) * 400.0f)));
        // After the instances: the asset's root mount, which is where residency
        // lives, is created by the first addInstance.
        markAllResident(*world, world->assetRootPage(asset), nodes);
        world->applyUpdates();
    }
};

} // namespace

// The core claim, over a continuous flight: every frame, the cached node set
// equals the uncached node set. And the cache has to actually be doing
// something -- a cache that never hits would pass trivially.
TEST(Cache, MatchesUncachedOnMovingCamera)
{
    Scene sc;
    View cache;
    CutResults cut;
    CutParams p{6.0f, 0.0f};
    uint32_t totalReused = 0;

    for (int f = 0; f < 120; ++f)
    {
        // Continuous motion, no teleports: a few units of drift per frame.
        const Camera v = viewAt(float4::vec(float(f) * 1.5f, float(f) * 0.4f,
                                             float(f) * 6.0f));
        CutResults ref;
        selectCutUncached(*sc.world, v, p, ref);
        cache.selectCut(*sc.world, v, p, cut);
        ASSERT_EQ(keysOf(cut), keysOf(ref)) << "frame " << f;
        ASSERT_EQ(cut.size(), ref.size()) << "frame " << f;
        totalReused += cache.reused();
    }
    EXPECT_GT(totalReused, 0u) << "the cache never hit; the test proves nothing";
}

// A camera that barely moves should reuse nearly everything: this is the case
// the whole mechanism exists for.
TEST(Cache, StationaryCameraReusesAlmostEverything)
{
    Scene sc;
    View cache;
    CutResults cut;
    CutParams p{6.0f, 0.0f};
    const Camera v = viewAt(float4::vec(0, 0, 0));

    cache.selectCut(*sc.world, v, p, cut);          // cold: records everything
    const uint32_t visible = cache.reused() + cache.walked();
    ASSERT_GT(visible, 10u);

    cache.selectCut(*sc.world, v, p, cut);          // warm: nothing changed at all
    EXPECT_EQ(cache.walked(), 0u);
    EXPECT_EQ(cache.reused(), visible);
}

// Moving one instance must invalidate that instance and nothing else.
TEST(Cache, InstanceMotionInvalidatesOnlyThatInstance)
{
    Scene sc;
    View cache;
    CutResults cut;
    CutParams p{6.0f, 0.0f};
    const Camera v = viewAt(float4::vec(0, 0, 0));

    cache.selectCut(*sc.world, v, p, cut);
    cache.selectCut(*sc.world, v, p, cut);
    ASSERT_EQ(cache.walked(), 0u);

    const AABB before = TAX::instanceWorldBox(*sc.world, sc.inst[3]);
    sc.world->moveInstance(sc.inst[3], before.mn + float4::vec(0, 0, 40.0f));
    cache.selectCut(*sc.world, v, p, cut);
    EXPECT_EQ(cache.walked(), 1u);

    CutResults ref;
    selectCutUncached(*sc.world, v, p, ref);
    cache.selectCut(*sc.world, v, p, cut);
    EXPECT_EQ(keysOf(cut), keysOf(ref));
}

// Deforming one instance must not disturb the others' records, exactly as
// deformation does not disturb their bounds.
TEST(Cache, DeformInvalidatesOnlyThatInstance)
{
    Scene sc;
    View cache;
    CutResults cut;
    CutParams p{6.0f, 0.0f};
    const Camera v = viewAt(float4::vec(0, 0, 0));

    cache.selectCut(*sc.world, v, p, cut);
    cache.selectCut(*sc.world, v, p, cut);
    ASSERT_EQ(cache.walked(), 0u);

    const UserPayload leaf = sc.gen.lastIds.back();
    const NodeHandle h = handleOf(*sc.world, leaf);
    AABB b = sc.world->nodeBounds(sc.inst[5], h);
    b.expand(b.mx + float4::vec(3, 3, 3));
    sc.world->setNodeBounds(sc.inst[5], h, b);
    sc.world->applyUpdates();

    cache.selectCut(*sc.world, v, p, cut);
    EXPECT_EQ(cache.walked(), 1u);

    CutResults ref;
    selectCutUncached(*sc.world, v, p, ref);
    cache.selectCut(*sc.world, v, p, cut);
    EXPECT_EQ(keysOf(cut), keysOf(ref));
}

// Residency and topology changes reach every instance of a shared asset, since
// they change the shared page rather than any one placement.
TEST(Cache, SharedPageChangeInvalidatesEveryInstanceOfIt)
{
    Scene sc;
    View cache;
    CutResults cut;
    CutParams p{6.0f, 0.0f};
    const Camera v = viewAt(float4::vec(0, 0, 0));

    cache.selectCut(*sc.world, v, p, cut);
    cache.selectCut(*sc.world, v, p, cut);
    ASSERT_EQ(cache.walked(), 0u);
    const uint32_t visible = cache.reused();

    markNonResident(*sc.world, sc.gen.lastIds.back());
    cache.selectCut(*sc.world, v, p, cut);
    EXPECT_EQ(cache.walked(), visible) << "one shared page changed; all of them stale";

    CutResults ref;
    selectCutUncached(*sc.world, v, p, ref);
    cache.selectCut(*sc.world, v, p, cut);
    EXPECT_EQ(keysOf(cut), keysOf(ref));
}

// Under damping the query is a box that grows and shrinks, not a point, so the
// odometer has to bound the movement of the whole envelope rather than of the
// eye. Jitter is the case that would expose a bound that only tracked position.
//
// The View damps internally, so the reference arm drives an external
// CameraDamper with the same half-life over the same raw views. Equal node sets
// therefore assert two things at once: that reuse is invisible, and that the
// contained damper is the same mechanism the caller could have run by hand.
TEST(Cache, MatchesUncachedUnderDamping)
{
    Scene sc;
    View cache(6.0f);
    CutResults cut;
    CameraDamper refDamper(6.0f);
    CutParams p{6.0f, 0.0f};
    std::mt19937 rng(99);
    std::uniform_real_distribution<float> jit(-2.0f, 2.0f);

    ASSERT_EQ(cache.halfLife(), 6.0f);

    for (int f = 0; f < 120; ++f)
    {
        // Drift plus jitter: the envelope both travels and breathes.
        Camera raw = viewAt(float4::vec(float(f) * 1.5f + jit(rng), jit(rng),
                                          float(f) * 6.0f + jit(rng)));
        // Continuous zoom exercises the projection-scale odometer at the same
        // time as camera and damping-envelope travel.
        raw.k *= 0.75f + 0.2f * std::sin(float(f) * 0.13f);

        CutResults ref;
        selectCutUncached(*sc.world, refDamper.damp(raw), p, ref);
        cache.selectCut(*sc.world, raw, p, cut);
        ASSERT_EQ(keysOf(cut), keysOf(ref)) << "frame " << f;
    }
}

// Damping off must stay exactly damping off: a default-constructed View has
// to leave the view alone, or every other test in this file is comparing
// against the wrong reference.
TEST(Cache, UndampedByDefault)
{
    Scene sc;
    View cache;
    CutResults cut;
    CutParams p{6.0f, 0.0f};
    EXPECT_EQ(cache.halfLife(), 0.0f);

    for (int f = 0; f < 8; ++f)
    {
        const Camera raw = viewAt(float4::vec(float(f) * 40.0f, 0.0f, float(f) * 90.0f));
        CutResults ref;
        selectCutUncached(*sc.world, raw, p, ref);
        cache.selectCut(*sc.world, raw, p, cut);
        ASSERT_EQ(keysOf(cut), keysOf(ref)) << "frame " << f;
    }
}

// reset() has to clear the damping window too, not just the records: that is
// the half of it that is required rather than merely tidy. After a teleport +
// reset the envelope must be a point again, which is what an undamped View
// fed the same view produces.
TEST(Cache, ResetClearsTheDampingWindow)
{
    Scene sc;
    View damped(8.0f), fresh;
    CutResults dampedCut, freshCut;
    CutParams p{6.0f, 0.0f};

    for (int f = 0; f < 12; ++f)
        damped.selectCut(*sc.world, viewAt(float4::vec(0.0f, 0.0f, float(f) * -120.0f)), p, dampedCut);

    const Camera jumped = viewAt(float4::vec(0, 0, 2600.0f));
    damped.reset();
    damped.selectCut(*sc.world, jumped, p, dampedCut);
    fresh.selectCut(*sc.world, jumped, p, freshCut);

    // A stale envelope would still reach back down the flight path and refine
    // nodes the point query does not.
    EXPECT_EQ(keysOf(dampedCut), keysOf(freshCut));
}

// A teleport is a discontinuity the odometer absorbs as one huge step, so
// almost everything must be re-walked -- and the answer must be right either
// way. Note "almost": a handful of instances legitimately survive a 2.4 km jump
// because they sit further than that from any flip point of their own, and the
// margin says so. That is the bound being sound rather than lucky, and the
// keys check below is what holds it honest.
TEST(Cache, TeleportRewalksAndStaysCorrect)
{
    Scene sc;
    View cache;
    CutResults cut;
    CutParams p{6.0f, 0.0f};

    cache.selectCut(*sc.world, viewAt(float4::vec(0, 0, 0)), p, cut);
    cache.selectCut(*sc.world, viewAt(float4::vec(0, 0, 0)), p, cut);
    ASSERT_EQ(cache.walked(), 0u);

    const Camera jumped = viewAt(float4::vec(0, 0, 2400.0f));
    CutResults ref;
    selectCutUncached(*sc.world, jumped, p, ref);
    cache.selectCut(*sc.world, jumped, p, cut);
    EXPECT_GT(cache.walked(), cache.reused());
    EXPECT_EQ(keysOf(cut), keysOf(ref));
}

// Two cameras, two caches, on the same world in the same frame: neither may
// see the other's state, and each must match its own uncached answer. This is
// the reason the cache is a separate object.
TEST(Cache, MultipleViewsDoNotInterfere)
{
    Scene sc;
    View main, shadow;
    CutResults mainCut, shadowCut;
    CutParams pm{6.0f, 0.0f}, ps{24.0f, 0.0f};

    for (int f = 0; f < 40; ++f)
    {
        const Camera vm = viewAt(float4::vec(float(f) * 2.0f, 0, float(f) * 5.0f));
        const Camera vs = viewAt(float4::vec(-float(f) * 3.0f, 120.0f, float(f) * 2.0f));

        CutResults rm, rs;
        selectCutUncached(*sc.world, vm, pm, rm);
        selectCutUncached(*sc.world, vs, ps, rs);
        main.selectCut(*sc.world, vm, pm, mainCut);
        shadow.selectCut(*sc.world, vs, ps, shadowCut);

        ASSERT_EQ(keysOf(mainCut), keysOf(rm)) << "main, frame " << f;
        ASSERT_EQ(keysOf(shadowCut), keysOf(rs)) << "shadow, frame " << f;
    }
}

// A threshold change alters every slope bound, so it invalidates every record
// in O(1). Projection changes are budgeted and covered by the damping test.
TEST(Cache, ThresholdChangesInvalidate)
{
    Scene sc;
    View cache;
    CutResults cut;
    const Camera v = viewAt(float4::vec(0, 0, 0));

    cache.selectCut(*sc.world, v, CutParams{6.0f, 0.0f}, cut);
    cache.selectCut(*sc.world, v, CutParams{6.0f, 0.0f}, cut);
    ASSERT_EQ(cache.walked(), 0u);

    cache.selectCut(*sc.world, v, CutParams{3.0f, 0.0f}, cut);
    EXPECT_EQ(cache.reused(), 0u);

    CutResults ref;
    selectCutUncached(*sc.world, v, CutParams{3.0f, 0.0f}, ref);
    cache.selectCut(*sc.world, v, CutParams{3.0f, 0.0f}, cut);
    EXPECT_EQ(keysOf(cut), keysOf(ref));
}

// Streaming churn: instances appearing and disappearing, pages attaching and
// detaching, payloads going in and out of residency, all while the camera
// flies. The cache must track every one of those.
TEST(Cache, SurvivesStreamingAndInstanceChurn)
{
    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 2;
    World world;
    Page root = gen.makeRootPage(unitRegion(80.0f), 400.0f, 1);
    std::vector<UserId> rootIds = gen.lastIds;
    const AssetHandle asset = world.registerAsset(std::move(root));

    std::vector<World::InstanceRef> inst;
    for (uint32_t i = 0; i < 24; ++i)
        inst.push_back(world.addInstance(
            asset, float4::vec(float(i % 6) * 250.0f - 750.0f,
                               float(i / 6) * 250.0f - 400.0f, 1800.0f)));
    markAllResident(world, rootIds);

    View cache;
    CutResults cut;
    CutParams p{5.0f, 0.0f};
    uint32_t reused = 0, attached = 0;

    for (int f = 0; f < 60; ++f)
    {
        const Camera v = viewAt(float4::vec(float(f) * 2.0f, float(f), float(f) * 8.0f));

        CutResults ref;
        selectCutUncached(world, v, p, ref);

        cache.selectCut(world, v, p, cut);
        ASSERT_EQ(keysOf(cut), keysOf(ref)) << "frame " << f;
        reused += cache.reused();

        // Act on streaming the way a real frame would.
        for (const CutEntry& entry : idealCut(cut))
        {
            const UserId id = payloadOf(world, entry);
            const bool expandable = entry.overThreshold() &&
                                    gen.recipes.count(id) &&
                                    !world.isAttached(entry.nodeHandle);
            if (!expandable && !world.isResident(entry.nodeHandle))
                world.markResident(entry.nodeHandle);
        }
        if (f % 4 == 0)
            for (const CutEntry& e : idealCut(cut))
                if (e.overThreshold() && attached < 8)
                {
                    const UserId id = payloadOf(world, e);
                    if (gen.recipes.count(id) && !world.isAttached(e.nodeHandle))
                    {
                        Page child = gen.makeChildPage(id);
                        const uint32_t n = child.nodeCount();
                        const PageHandle ph = world.attachPage(e.nodeHandle,
                                                               std::move(child));
                        if (ph.valid())
                        {
                            markAllResident(world, ph, n);
                            ++attached;
                        }
                    }
                    break;
                }
        if (f == 30)
        {
            world.removeInstance(inst[2]);
            inst.push_back(world.addInstance(asset, float4::vec(200.0f, 0.0f, 2200.0f)));
        }
        if (f % 11 == 5) markNonResident(world, rootIds.back());
        if (f % 11 == 6) markResident(world, rootIds.back());
        world.applyUpdates();
    }
    EXPECT_GT(reused, 0u);
    EXPECT_GT(attached, 0u) << "no pages attached; churn was not exercised";
}

TEST(Cache, SurvivesFullTlasRebuild)
{
    WorldConfig config;
    config.tlasEscapeFraction = 0.0f;
    Scene sc(8, 3, 4, config);
    View cache;
    CutResults cut;
    const Camera v = viewAt(float4::vec(0, 0, 0));
    const CutParams p{6.0f, 0.0f};

    cache.selectCut(*sc.world, v, p, cut);
    cache.selectCut(*sc.world, v, p, cut);
    ASSERT_GT(cache.reused(), 0u);
    const uint32_t initialLayout = TAX::instanceLayoutVersion(*sc.world);

    // Escaping the exact lane with a zero budget forces a Morton rebuild. A
    // routine rebuild must leave dense positions and warm View records alone.
    const uint32_t center = 4u * 8u + 4u;
    sc.world->moveInstance(sc.inst[center], float4::point(10.0f, 0.0f, 3300.0f));
    sc.world->applyUpdates();
    EXPECT_EQ(TAX::instanceLayoutVersion(*sc.world), initialLayout);

    CutResults ref;
    selectCutUncached(*sc.world, v, p, ref);
    cache.selectCut(*sc.world, v, p, cut);
    EXPECT_EQ(keysOf(cut), keysOf(ref));
    EXPECT_GT(cache.reused(), 0u);

    // optimize() is the explicit expensive synchronization point. It changes
    // dense positions, so the next cached selection must forget the old
    // record indexing while public InstanceRefs remain usable.
    sc.world->optimize();
    EXPECT_NE(TAX::instanceLayoutVersion(*sc.world), initialLayout);
    selectCutUncached(*sc.world, v, p, ref);
    cache.selectCut(*sc.world, v, p, cut);
    EXPECT_EQ(keysOf(cut), keysOf(ref));
    EXPECT_EQ(cache.reused(), 0u);

    cache.selectCut(*sc.world, v, p, cut);
    EXPECT_GT(cache.reused(), 0u);

    const uint32_t optimizedLayout = TAX::instanceLayoutVersion(*sc.world);
    sc.world->moveInstance(sc.inst[center], float4::point(20.0f, 0.0f, 3300.0f));
    sc.world->applyUpdates();
    EXPECT_EQ(TAX::instanceLayoutVersion(*sc.world), optimizedLayout);
    selectCutUncached(*sc.world, v, p, ref);
    cache.selectCut(*sc.world, v, p, cut);
    EXPECT_EQ(keysOf(cut), keysOf(ref));
}

// Residency feedback is optional: a view that does not influence collection
// pays no PageUsageContext storage and still gets normal cut reuse.
TEST(Cache, ReuseDoesNotRequirePageUsage)
{
    Scene sc;
    View cache;
    CutResults cut;
    CutParams p{6.0f, 0.0f};
    const Camera v = viewAt(float4::vec(0, 0, 0));

    cache.selectCut(*sc.world, v, p, cut);
    sc.world->applyUpdates();
    cache.selectCut(*sc.world, v, p, cut);
    ASSERT_EQ(cache.walked(), 0u);
}

TEST(Cache, SixViewsSelectConcurrentlyFromOnePublishedWorld)
{
    constexpr size_t kViews = 6;
    Scene sc(20);
    const World& published = *sc.world;
    const CutParams params{6.0f, 0.0f};

    std::array<View, kViews> serialCtx;
    std::array<View, kViews> parallelCtx;
    std::array<CutResults, kViews> serialCut;
    std::array<CutResults, kViews> parallelCut;

    for (int frame = 0; frame < 12; ++frame)
    {
        for (size_t i = 0; i < 20; ++i)
            sc.world->moveInstance(
                sc.inst[i],
                float4::vec(float(i % 5) * 300.0f - 3000.0f + float(frame),
                            float(i / 5) * 300.0f - 3000.0f, 2500.0f));
        sc.world->applyUpdates();

        std::array<Camera, kViews> views;
        for (size_t v = 0; v < kViews; ++v)
            views[v] = viewAt(float4::vec((float(v) - 2.5f) * 35.0f,
                                          float(v) * 8.0f, float(frame) * 4.0f));

        for (size_t v = 0; v < kViews; ++v)
            serialCtx[v].selectCut(published, views[v], params, serialCut[v]);

        std::barrier<> start(static_cast<std::ptrdiff_t>(kViews + 1));
        std::array<std::exception_ptr, kViews> errors{};
        std::array<std::thread, kViews> threads;
        for (size_t v = 0; v < kViews; ++v)
            threads[v] = std::thread([&, v]
            {
                start.arrive_and_wait();
                try
                {
                    parallelCtx[v].selectCut(published, views[v], params,
                                        parallelCut[v]);
                }
                catch (...)
                {
                    errors[v] = std::current_exception();
                }
            });
        start.arrive_and_wait();
        for (std::thread& thread : threads) thread.join();

        for (size_t v = 0; v < kViews; ++v)
        {
            if (errors[v]) std::rethrow_exception(errors[v]);
            EXPECT_EQ(keysOf(parallelCut[v]), keysOf(serialCut[v]))
                << "frame " << frame << ", view " << v;
            EXPECT_EQ(parallelCtx[v].reused(), serialCtx[v].reused());
            EXPECT_EQ(parallelCtx[v].walked(), serialCtx[v].walked());
        }
    }
}

TEST(Cache, ConcurrentViewsClassifyBucketedCutsIndependently)
{
    constexpr size_t kViews = 6;
    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 3;
    Page page = gen.makeRootPage(unitRegion(40.0f), 128.0f, 0);

    World world;
    const AssetHandle asset = world.registerAsset(std::move(page));
    for (uint32_t i = 0; i < 256; ++i)
        world.addInstance(asset,
                          float4::point(float(i % 16) * 20.0f - 150.0f, 0.0f,
                                        float(i / 16) * 20.0f + 1000.0f));
    world.applyUpdates();
    const World& published = world;

    const CutParams params{0.25f, 0.0f};
    std::array<Camera, kViews> views;
    std::array<CutResults, kViews> serialCut, parallelCut;
    std::array<View, kViews> contexts;
    for (size_t v = 0; v < kViews; ++v)
    {
        views[v] = viewAt(float4::vec((float(v) - 2.5f) * 10.0f, 0.0f, 0.0f));
        selectCutUncached(world, views[v], params, serialCut[v]);
    }

    std::array<std::thread, kViews> threads;
    for (size_t v = 0; v < kViews; ++v)
        threads[v] = std::thread([&, v]
        {
            contexts[v].selectCut(published, views[v], params, parallelCut[v]);
        });
    for (std::thread& thread : threads) thread.join();

    for (size_t v = 0; v < kViews; ++v)
    {
        EXPECT_EQ(keysOf(parallelCut[v]), keysOf(serialCut[v]));
    }
}

TEST(Cache, ResetForgetsEverythingAndStaysCorrect)
{
    Scene sc;
    View cache;
    CutResults cut;
    CutParams p{6.0f, 0.0f};
    const Camera v = viewAt(float4::vec(0, 0, 0));

    cache.selectCut(*sc.world, v, p, cut);
    cache.selectCut(*sc.world, v, p, cut);
    ASSERT_EQ(cache.walked(), 0u);

    const size_t allocated = cache.bytes();
    ASSERT_GT(allocated, 0u);
    cache.reset();
    EXPECT_EQ(cache.bytes(), allocated);   // forget state, retain reusable storage
    EXPECT_EQ(cache.reused(), 0u);
    EXPECT_EQ(cache.walked(), 0u);
    EXPECT_EQ(cache.lastCutStats().instancesVisited, 0u);

    CutResults ref;
    selectCutUncached(*sc.world, v, p, ref);
    cache.selectCut(*sc.world, v, p, cut);
    EXPECT_EQ(cache.reused(), 0u);
    EXPECT_EQ(keysOf(cut), keysOf(ref));
}
