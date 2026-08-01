// Performance tests for the HLodTree corner cases:
//   - one very deep tree, static camera and fly-through
//   - deep paged tree with live streaming (attach/resident/collect loop)
//   - many shallow trees (forests), with and without contribution culling
//   - moving top-level instances
//   - moving leaf nodes (bounds refit)
//   - payload residency churn
//   - dynamic instance churn (spawn/despawn every frame)
//   - HLodBuilder page composition (runtime tree creation)
//   - output-sensitivity scaling (cost vs cut size at fixed world size)
//   - camera teleport (cold-frame latency spike)
//   - multi-view frames (marginal cost of extra views)
//   - streaming convergence latency (frames to ideal == actual)
//   - TLAS at 200k-500k instances + level-load burst
//   - adversarial shapes (stacked instances, max-width node, deep page chain)
//
// All setup happens outside the timed loop; iterations time selectCut plus
// whatever per-frame work the scenario prescribes.

#include <benchmark/benchmark.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <random>
#include <vector>

#include "helpers.h"   // TreeGen etc. (shared with the unit tests)

using namespace hlod;
using namespace hlodtest;

namespace {

struct Outputs
{
    std::vector<CutEntry>    cut;
    std::vector<IdealEntry>  ideal;
    std::vector<LoadRequest> reqs;
};

CullView orbitView(float t, float dist, float4 center = float4::point(0, 0, 0))
{
    const float4 pos = center + float4::vec(std::cos(t) * dist, dist * 0.35f,
                                            std::sin(t) * dist);
    return makeLookAtView(pos, center);
}

// Cheap deterministic RNG so the timed loops don't measure std::mt19937.
struct XorShift32
{
    uint32_t s = 0x9E3779B9u;
    uint32_t next()
    {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        return s;
    }
    float uniform(float lo, float hi)
    {
        return lo + (hi - lo) * float(next() >> 8) * (1.0f / 16777216.0f);
    }
};

// Attach up to `budget` requested expansions, most-visible (largest screen
// error) first. The order of ideal-cut entries is traversal-defined, so a
// streaming policy must prioritize by err — picking "the first N" makes the
// attach set depend on walk order and over-refines wherever the walk went
// deep first (measurably worse: the hot set churns instead of converging).
size_t attachTopByPriority(World& w, TreeGen& gen,
                           const std::vector<IdealEntry>& ideal, size_t budget)
{
    static std::vector<uint32_t> idx;
    idx.clear();
    for (uint32_t i = 0; i < ideal.size(); ++i)
        if (ideal[i].tag == IdealTag::NeedsExpansion && gen.recipes.count(ideal[i].payload))
            idx.push_back(i);
    const size_t take = idx.size() < budget ? idx.size() : budget;
    std::partial_sort(idx.begin(), idx.begin() + ptrdiff_t(take), idx.end(),
                      [&](uint32_t a, uint32_t b) { return ideal[a].err > ideal[b].err; });
    for (size_t j = 0; j < take; ++j)
    {
        // Production flow: content is keyed by payload, attach by handle.
        const IdealEntry& e = ideal[idx[j]];
        Page child = gen.makeChildPage(e.payload);
        const uint32_t n = child.nodeCount();
        const PageHandle ph = w.attachPage(e.node, std::move(child));
        if (ph.valid()) markAllResident(w, ph, n);   // payload streaming is instant
    }
    return take;
}

// Add a root page and mark every node resident (handles composed from the
// attach result — no lookups anywhere).
World::InstanceRef addResidentInstance(World& w, Page pg, float4 pos, float scale = 1.0f)
{
    const uint32_t n = pg.nodeCount();
    const auto inst = w.addInstance(std::move(pg), pos, scale);
    markAllResident(w, inst.rootPage, n);
    return inst;
}

// One fully resident deep tree. depth 6 / fanout 8 is ~300k nodes.
World makeDeepWorld(uint32_t fanout, uint32_t depth, TreeGen* genOut = nullptr)
{
    TreeGen gen;
    gen.fanout = fanout;
    gen.depth = depth;
    World w;
    addResidentInstance(w, gen.makeRootPage(unitRegion(1000.0f), 4096.0f, 0),
                        float4::point(0, 0, 0));
    if (genOut) *genOut = std::move(gen);
    return w;
}

} // namespace

// ---------------------------------------------------------------------------
// Deep tree, static camera: steady-state cost of the pruned single pass.
// ---------------------------------------------------------------------------
static void BM_DeepTree_StaticCamera(benchmark::State& state)
{
    World w = makeDeepWorld(8, uint32_t(state.range(0)));
    ViewScratch s;
    Outputs o;
    const CullView v = orbitView(0.7f, 2500.0f);
    const CutParams p{4.0f, 0.1f, 0.0f};

    for (auto _ : state)
    {
        w.beginFrame();
        w.selectCut(v, p, s, o.cut, o.ideal, o.reqs);
        benchmark::DoNotOptimize(o.cut.data());
    }
    state.counters["cut"] = double(o.cut.size());
}
BENCHMARK(BM_DeepTree_StaticCamera)->Arg(4)->Arg(5)->Arg(6)->Unit(benchmark::kMicrosecond);

// Same scene, but the caller opts out of the ideal cut and load requests
// (nullptr outputs) — the fully-resident static case pays for exactly one
// output vector instead of two.
static void BM_DeepTree_CutOnly(benchmark::State& state)
{
    World w = makeDeepWorld(8, uint32_t(state.range(0)));
    ViewScratch s;
    std::vector<CutEntry> cut;
    const CullView v = orbitView(0.7f, 2500.0f);
    const CutParams p{4.0f, 0.1f, 0.0f};

    for (auto _ : state)
    {
        w.beginFrame();
        w.selectCut(v, p, s, cut);
        benchmark::DoNotOptimize(cut.data());
    }
    state.counters["cut"] = double(cut.size());
}
BENCHMARK(BM_DeepTree_CutOnly)->Arg(6)->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Deep tree, flying camera: refinement churn, hysteresis, epoch reuse.
// ---------------------------------------------------------------------------
static void BM_DeepTree_FlyThrough(benchmark::State& state)
{
    World w = makeDeepWorld(8, uint32_t(state.range(0)));
    ViewScratch s;
    Outputs o;
    const CutParams p{4.0f, 0.1f, 0.0f};

    float t = 0.0f;
    size_t cutTotal = 0, frames = 0;
    for (auto _ : state)
    {
        t += 0.02f;
        // Swoop in and out while orbiting.
        const float dist = 1800.0f + 1500.0f * std::sin(t * 3.1f);
        const CullView v = orbitView(t, dist);
        w.beginFrame();
        w.selectCut(v, p, s, o.cut, o.ideal, o.reqs);
        benchmark::DoNotOptimize(o.cut.data());
        cutTotal += o.cut.size();
        ++frames;
    }
    state.counters["avg_cut"] = double(cutTotal) / double(frames ? frames : 1);
}
BENCHMARK(BM_DeepTree_FlyThrough)->Arg(5)->Arg(6)->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Paged "planet": fly in/out while streaming topology and payloads with the
// cut's own outputs, plus garbage collection back to a page budget.
// ---------------------------------------------------------------------------
static void BM_PagedPlanet_StreamingFly(benchmark::State& state)
{
    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 2;
    World w;
    addResidentInstance(w, gen.makeRootPage(unitRegion(100000.0f), 1u << 20, 4),
                        float4::point(0, 0, 0));

    ViewScratch s;
    Outputs o;
    const CutParams p{8.0f, 0.1f, 0.0f};
    const size_t pageBudget = size_t(state.range(0));

    float t = 0.0f;
    size_t attaches = 0;
    for (auto _ : state)
    {
        t += 0.03f;
        const float dist = 250000.0f * std::exp(-2.5f * (0.5f + 0.5f * std::sin(t)));
        const CullView v = orbitView(t * 0.2f, 20000.0f + dist);

        w.beginFrame();
        w.selectCut(v, p, s, o.cut, o.ideal, o.reqs);

        attaches += attachTopByPriority(w, gen, o.ideal, 8);
        for (const auto& r : o.reqs) w.markResident(r.node);
        w.collect(pageBudget, 16);
        benchmark::DoNotOptimize(o.cut.data());
    }
    state.counters["attached"] = double(w.attachedPageCount());
    state.counters["attaches"] = double(attaches);
}
BENCHMARK(BM_PagedPlanet_StreamingFly)->Arg(64)->Arg(512)->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// GC stress: a camera racing at low altitude through a huge, dense paged
// world. Pages are constantly expanded ahead of the camera and collapsed
// behind it; the flight path wraps around, so already-collected regions get
// re-expanded — full expand/collapse churn, forever.
// arg0 = GC page budget (low watermark).
// ---------------------------------------------------------------------------
static void BM_GcStress_FastFlythrough(benchmark::State& state)
{
    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 2;
    World w;
    const float half = 100000.0f;
    addResidentInstance(w, gen.makeRootPage(unitRegion(half), float(1u << 20), 4),
                        float4::point(0, 0, 0));

    ViewScratch s;
    Outputs o;
    const CutParams p{8.0f, 0.1f, 1.0f};
    const size_t pageBudget = size_t(state.range(0));
    const uint32_t minAge = 8;
    const float speed = half / 250.0f;   // crosses the whole world in ~500 frames

    float x = -half;
    size_t attaches = 0, collected = 0, frames = 0, cutTotal = 0;
    for (auto _ : state)
    {
        x += speed;
        if (x > half) x = -half;   // wrap: revisit collapsed regions
        const CullView v = makePerspectiveView(
            float4::point(x, half * 0.02f, 0), float4::vec(1.0f, -0.15f, 0.0f),
            float4::vec(0, 1, 0), 1.0f, 16.0f / 9.0f, 1080.0f, 0.1f, 1.0e9f);

        w.beginFrame();
        w.selectCut(v, p, s, o.cut, o.ideal, o.reqs);

        attaches += attachTopByPriority(w, gen, o.ideal, 16);
        for (const auto& r : o.reqs) w.markResident(r.node);
        collected += w.collect(pageBudget, minAge);

        cutTotal += o.cut.size();
        ++frames;
        benchmark::DoNotOptimize(o.cut.data());
    }
    state.counters["attached"] = double(w.attachedPageCount());
    state.counters["attach_pf"] = double(attaches) / double(frames ? frames : 1);
    state.counters["collect_pf"] = double(collected) / double(frames ? frames : 1);
    state.counters["avg_cut"] = double(cutTotal) / double(frames ? frames : 1);
}
BENCHMARK(BM_GcStress_FastFlythrough)->Arg(96)->Arg(768)->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Forests: many shallow multi-root trees. arg0 = instances, arg1 = minPix on.
// ---------------------------------------------------------------------------
static void BM_ManyShallowTrees(benchmark::State& state)
{
    std::mt19937 rng(1234);
    const int count = int(state.range(0));
    const float area = 60.0f * std::sqrt(float(count));
    std::uniform_real_distribution<float> uni(-area, area);

    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 1;
    World w;
    for (int i = 0; i < count; ++i)
        addResidentInstance(w, gen.makeRootPage(unitRegion(4.0f), 8.0f, 0),
                            float4::point(uni(rng), 0, uni(rng)));

    ViewScratch s;
    Outputs o;
    const CutParams p{4.0f, 0.1f, state.range(1) ? 1.0f : 0.0f};

    float t = 0.0f;
    for (auto _ : state)
    {
        t += 0.01f;
        const CullView v = orbitView(t, area * 0.4f);
        w.beginFrame();
        w.selectCut(v, p, s, o.cut, o.ideal, o.reqs);
        benchmark::DoNotOptimize(o.cut.data());
    }
    state.counters["cut"] = double(o.cut.size());
}
BENCHMARK(BM_ManyShallowTrees)
    ->Args({1000, 0})->Args({10000, 0})->Args({50000, 0})
    ->Args({10000, 1})->Args({50000, 1})
    ->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Top-level motion: move a slice of the instances every frame.
// arg0 = instances, arg1 = movers per frame.
// ---------------------------------------------------------------------------
static void BM_MovingInstances(benchmark::State& state)
{
    std::mt19937 rng(555);
    const int count = int(state.range(0));
    const int movers = int(state.range(1));
    const float area = 60.0f * std::sqrt(float(count));
    std::uniform_real_distribution<float> uni(-area, area);

    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 1;
    World w;
    std::vector<World::InstanceRef> insts;
    for (int i = 0; i < count; ++i)
        insts.push_back(addResidentInstance(w, gen.makeRootPage(unitRegion(4.0f), 8.0f, 0),
                                            float4::point(uni(rng), 0, uni(rng))));

    ViewScratch s;
    Outputs o;
    const CutParams p{4.0f, 0.1f, 0.0f};
    XorShift32 fast;

    float t = 0.0f;
    size_t cursor = 0;
    for (auto _ : state)
    {
        t += 0.01f;
        for (int m = 0; m < movers; ++m)
        {
            w.moveInstance(insts[cursor++ % insts.size()],
                           float4::point(fast.uniform(-area, area), 0,
                                         fast.uniform(-area, area)));
        }
        const CullView v = orbitView(t, area * 0.4f);
        w.beginFrame();
        w.selectCut(v, p, s, o.cut, o.ideal, o.reqs);
        benchmark::DoNotOptimize(o.cut.data());
    }
}
BENCHMARK(BM_MovingInstances)
    ->Args({10000, 100})->Args({10000, 1000})->Args({50000, 1000})
    ->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Movable/resizable leaf stress: ~250k leaves in one wide tree. The refit-only
// benches time setNodeBounds + flushBounds (the bbox hierarchy + wide-lane
// update) in isolation; the WithCut variant adds selectCut on top.
// ---------------------------------------------------------------------------
struct MoverWorld
{
    World world;
    std::vector<NodeHandle> leaves;   // composed from the attach result
    std::vector<float4>     home;     // original leaf centers
    float half = 2000.0f;
};

MoverWorld makeMoverWorld()
{
    MoverWorld mw;
    TreeGen gen;
    gen.fanout = 12;   // wide: 12^5 = 248832 leaves, ~271k nodes
    gen.depth = 5;
    Page pg = gen.makeRootPage(unitRegion(mw.half), 4096.0f, 0);
    std::vector<uint32_t> leafIdx;
    for (uint32_t i = 1; i < pg.nodeCount(); ++i)
    {
        if (pg.childCount(i) != 0) continue;
        leafIdx.push_back(i);
        mw.home.push_back(pg.bbox[i].center());
    }
    const auto inst = addResidentInstance(mw.world, std::move(pg), float4::point(0, 0, 0));
    for (uint32_t i : leafIdx) mw.leaves.push_back(nodeAt(inst.rootPage, i));
    return mw;
}

// Common case: objects jitter and resize near where they are. After a brief
// warm-up the grown parent boxes contain the jitter and every refit
// early-outs at the immediate parent.
static void BM_LeafRefit_LocalJitter(benchmark::State& state)
{
    MoverWorld mw = makeMoverWorld();
    const int movers = int(state.range(0));
    XorShift32 rng;
    size_t cursor = 0;
    for (auto _ : state)
    {
        for (int m = 0; m < movers; ++m)
        {
            const size_t i = cursor++ % mw.leaves.size();
            const float4 c = mw.home[i] + float4::vec(rng.uniform(-1, 1),
                                                      rng.uniform(-1, 1),
                                                      rng.uniform(-1, 1));
            const float e = rng.uniform(0.3f, 1.5f);   // resize as well
            mw.world.setNodeBounds(mw.leaves[i],
                                   AABB::fromCenterExtent(c, float4::vec(e, e, e)));
        }
        mw.world.flushBounds();
    }
    state.SetItemsProcessed(int64_t(state.iterations()) * movers);
}
BENCHMARK(BM_LeafRefit_LocalJitter)->Arg(10000)->Arg(100000)
    ->Unit(benchmark::kMicrosecond);

// Worst case: objects teleport across the whole world, forcing multi-level
// grow-refits and wide-lane patches high up the tree.
static void BM_LeafRefit_Teleport(benchmark::State& state)
{
    MoverWorld mw = makeMoverWorld();
    const int movers = int(state.range(0));
    XorShift32 rng;
    size_t cursor = 0;
    for (auto _ : state)
    {
        for (int m = 0; m < movers; ++m)
        {
            const size_t i = cursor++ % mw.leaves.size();
            const float r = mw.half * 0.98f;
            const float4 c = float4::vec(rng.uniform(-r, r), rng.uniform(-r, r),
                                         rng.uniform(-r, r));
            const float e = rng.uniform(0.3f, 3.0f);
            mw.world.setNodeBounds(mw.leaves[i],
                                   AABB::fromCenterExtent(c, float4::vec(e, e, e)));
        }
        mw.world.flushBounds();
    }
    state.SetItemsProcessed(int64_t(state.iterations()) * movers);
}
BENCHMARK(BM_LeafRefit_Teleport)->Arg(10000)->Arg(100000)
    ->Unit(benchmark::kMicrosecond);

// Repeated-submission case: distinct objects each submit many bounds updates
// per frame (animation ticks, physics substeps). Repeats apply against hot
// cache lines and early-out at the already-grown parent, so they cost a
// fraction of a first move.
static void BM_LeafRefit_RepeatedMoves(benchmark::State& state)
{
    MoverWorld mw = makeMoverWorld();
    const int distinct = int(state.range(0));
    const int repeats = int(state.range(1));
    XorShift32 rng;
    size_t cursor = 0;
    for (auto _ : state)
    {
        for (int rep = 0; rep < repeats; ++rep)
        {
            for (int m = 0; m < distinct; ++m)
            {
                const size_t i = (cursor + size_t(m)) % mw.leaves.size();
                const float4 c = mw.home[i] + float4::vec(rng.uniform(-1, 1),
                                                          rng.uniform(-1, 1),
                                                          rng.uniform(-1, 1));
                mw.world.setNodeBounds(mw.leaves[i],
                                       AABB::fromCenterExtent(c, float4::vec(1, 1, 1)));
            }
        }
        cursor += size_t(distinct);
        mw.world.flushBounds();
    }
    state.SetItemsProcessed(int64_t(state.iterations()) * distinct * repeats);
}
BENCHMARK(BM_LeafRefit_RepeatedMoves)
    ->Args({10000, 10})->Args({100000, 10})
    ->Unit(benchmark::kMicrosecond);

// Full frame under teleport chaos: refit + cut selection with a moving
// camera. Note the ancestor boxes degrade toward the whole region (grow-only,
// shrink is lazy), so this also measures the cut under worst-case bounds.
static void BM_LeafMotion_TeleportWithCut(benchmark::State& state)
{
    MoverWorld mw = makeMoverWorld();
    const int movers = int(state.range(0));
    XorShift32 rng;
    ViewScratch s;
    Outputs o;
    const CutParams p{16.0f, 0.1f, 0.0f};

    float t = 0.0f;
    size_t cursor = 0, cutTotal = 0, frames = 0;
    for (auto _ : state)
    {
        t += 0.02f;
        for (int m = 0; m < movers; ++m)
        {
            const size_t i = cursor++ % mw.leaves.size();
            const float r = mw.half * 0.98f;
            const float4 c = float4::vec(rng.uniform(-r, r), rng.uniform(-r, r),
                                         rng.uniform(-r, r));
            mw.world.setNodeBounds(mw.leaves[i],
                                   AABB::fromCenterExtent(c, float4::vec(1, 1, 1)));
        }
        const CullView v = orbitView(t, mw.half * 2.0f);
        mw.world.beginFrame();
        mw.world.selectCut(v, p, s, o.cut, o.ideal, o.reqs);
        benchmark::DoNotOptimize(o.cut.data());
        cutTotal += o.cut.size();
        ++frames;
    }
    state.SetItemsProcessed(int64_t(state.iterations()) * movers);
    state.counters["avg_cut"] = double(cutTotal) / double(frames ? frames : 1);
}
BENCHMARK(BM_LeafMotion_TeleportWithCut)->Arg(10000)->Arg(100000)
    ->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// In-tree motion: leaf nodes of a deep tree change bounds every frame; the
// lazy refit + wide-lane patching runs inside selectCut.
// ---------------------------------------------------------------------------
static void BM_MovingLeafNodes(benchmark::State& state)
{
    TreeGen gen;
    gen.fanout = 8;
    gen.depth = 5;
    World w;
    std::vector<NodeHandle> leaves;
    {
        Page pg = gen.makeRootPage(unitRegion(1000.0f), 4096.0f, 0);
        std::vector<uint32_t> leafIdx;
        for (uint32_t i = 1; i < pg.nodeCount(); ++i)
            if (pg.childCount(i) == 0) leafIdx.push_back(i);
        const auto inst = addResidentInstance(w, std::move(pg), float4::point(0, 0, 0));
        for (uint32_t i : leafIdx) leaves.push_back(nodeAt(inst.rootPage, i));
    }

    XorShift32 rng;
    const int movers = int(state.range(0));

    ViewScratch s;
    Outputs o;
    const CutParams p{4.0f, 0.1f, 0.0f};

    float t = 0.0f;
    size_t cursor = 0;
    for (auto _ : state)
    {
        t += 0.02f;
        for (int m = 0; m < movers; ++m)
        {
            const NodeHandle h = leaves[cursor++ % leaves.size()];
            w.setNodeBounds(h, AABB::fromCenterExtent(
                                   float4::vec(rng.uniform(-900, 900),
                                               rng.uniform(-90, 90),
                                               rng.uniform(-900, 900)),
                                   float4::vec(2, 2, 2)));
        }
        const CullView v = orbitView(t, 2500.0f);
        w.beginFrame();
        w.selectCut(v, p, s, o.cut, o.ideal, o.reqs);
        benchmark::DoNotOptimize(o.cut.data());
    }
}
BENCHMARK(BM_MovingLeafNodes)->Arg(100)->Arg(1000)->Arg(10000)
    ->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Residency churn: payloads stream in and out under a live camera.
// ---------------------------------------------------------------------------
static void BM_ResidencyChurn(benchmark::State& state)
{
    TreeGen gen;
    gen.fanout = 8;
    gen.depth = 5;
    World w;
    Page pg = gen.makeRootPage(unitRegion(1000.0f), 4096.0f, 0);
    const uint32_t nodeCount = pg.nodeCount();
    const auto inst = w.addInstance(std::move(pg), float4::point(0, 0, 0));
    markAllResident(w, inst.rootPage, nodeCount);

    // Handles for every node except the pinned root (index 1).
    std::vector<NodeHandle> nodes;
    for (uint32_t i = 2; i < nodeCount; ++i) nodes.push_back(nodeAt(inst.rootPage, i));

    std::mt19937 rng(2020);
    const int churn = int(state.range(0));

    ViewScratch s;
    Outputs o;
    const CutParams p{4.0f, 0.1f, 0.0f};

    float t = 0.0f;
    size_t cursor = 0;
    for (auto _ : state)
    {
        t += 0.02f;
        for (int c = 0; c < churn; ++c)
        {
            const NodeHandle h = nodes[cursor++ % nodes.size()];
            if (w.isResident(h)) w.markNonResident(h);
            else w.markResident(h);
        }
        const CullView v = orbitView(t, 2500.0f);
        w.beginFrame();
        w.selectCut(v, p, s, o.cut, o.ideal, o.reqs);
        benchmark::DoNotOptimize(o.cut.data());
    }
}
BENCHMARK(BM_ResidencyChurn)->Arg(100)->Arg(10000)->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Kitchen sink: every dynamic feature at once in one world.
//   - a paged planet streamed under a fast low-altitude camera (attach + GC)
//   - a forest of 20k shallow prop instances, 500 drifting per frame (TLAS)
//   - a deep tree whose leaves jitter, 2000 refits per frame
//   - payload residency streamed with one frame of latency
// This is the "real game" shape: nothing is isolated, every subsystem's
// bookkeeping has to coexist with the others in the same selectCut.
// ---------------------------------------------------------------------------
static void BM_Combined_KitchenSink(benchmark::State& state)
{
    const float half = 50000.0f;
    TreeGen planetGen;
    planetGen.fanout = 4;
    planetGen.depth = 2;

    World w;
    addResidentInstance(w, planetGen.makeRootPage(unitRegion(half), float(1u << 19), 3),
                        float4::point(0, 0, 0));

    // Props: shallow trees scattered on the ground plane.
    TreeGen propGen;
    propGen.fanout = 4;
    propGen.depth = 1;
    propGen.nextId = 1u << 20;   // avoid id collisions with the planet
    std::mt19937 rng(4242);
    std::uniform_real_distribution<float> uni(-half, half);
    std::vector<World::InstanceRef> props;
    std::vector<float4> propHome;
    for (int i = 0; i < 20000; ++i)
    {
        propHome.push_back(float4::point(uni(rng), 0, uni(rng)));
        props.push_back(addResidentInstance(w, propGen.makeRootPage(unitRegion(6.0f), 24.0f, 0),
                                            propHome.back()));
    }

    // Movers: one wide deep tree with jittering leaves.
    TreeGen moverGen;
    moverGen.fanout = 10;
    moverGen.depth = 4;   // 10^4 = 10k leaves
    moverGen.nextId = 1u << 24;
    std::vector<NodeHandle> moverLeaves;
    std::vector<float4> moverHome;
    {
        Page pg = moverGen.makeRootPage(unitRegion(2000.0f), 2048.0f, 0);
        std::vector<uint32_t> leafIdx;
        for (uint32_t i = 1; i < pg.nodeCount(); ++i)
        {
            if (pg.childCount(i) != 0) continue;
            leafIdx.push_back(i);
            moverHome.push_back(pg.bbox[i].center());
        }
        const auto inst = addResidentInstance(w, std::move(pg), float4::point(0, 500.0f, 0));
        for (uint32_t i : leafIdx) moverLeaves.push_back(nodeAt(inst.rootPage, i));
    }

    ViewScratch s;
    Outputs o;
    const CutParams p{8.0f, 0.1f, 0.5f};
    XorShift32 fast;

    float x = -half;
    size_t frames = 0, cutTotal = 0, attaches = 0, collected = 0;
    size_t propCursor = 0, leafCursor = 0;
    for (auto _ : state)
    {
        x += half / 300.0f;
        if (x > half) x = -half;

        // 500 props drift near home, a handful teleport across the world.
        for (int m = 0; m < 500; ++m)
        {
            const size_t i = propCursor++ % props.size();
            if (m < 5)
                propHome[i] = float4::point(fast.uniform(-half, half), 0,
                                            fast.uniform(-half, half));
            const float4 d = float4::vec(fast.uniform(-10, 10), 0,
                                         fast.uniform(-10, 10));
            w.moveInstance(props[i], propHome[i] + d);
        }
        // 2000 mover leaves jitter near home.
        for (int m = 0; m < 2000; ++m)
        {
            const size_t i = leafCursor++ % moverLeaves.size();
            const float4 c = moverHome[i] + float4::vec(fast.uniform(-2, 2),
                                                        fast.uniform(-2, 2),
                                                        fast.uniform(-2, 2));
            w.setNodeBounds(moverLeaves[i],
                            AABB::fromCenterExtent(c, float4::vec(1, 1, 1)));
        }

        const CullView v = makePerspectiveView(
            float4::point(x, half * 0.03f, 0), float4::vec(1.0f, -0.2f, 0.1f),
            float4::vec(0, 1, 0), 1.0f, 16.0f / 9.0f, 1080.0f, 0.1f, 1.0e9f);

        w.beginFrame();
        w.selectCut(v, p, s, o.cut, o.ideal, o.reqs);

        attaches += attachTopByPriority(w, planetGen, o.ideal, 12);
        for (const auto& r : o.reqs) w.markResident(r.node);
        collected += w.collect(300, 8);

        cutTotal += o.cut.size();
        ++frames;
        benchmark::DoNotOptimize(o.cut.data());
    }
    state.counters["avg_cut"] = double(cutTotal) / double(frames ? frames : 1);
    state.counters["attach_pf"] = double(attaches) / double(frames ? frames : 1);
    state.counters["collect_pf"] = double(collected) / double(frames ? frames : 1);
    state.counters["attached"] = double(w.attachedPageCount());
}
BENCHMARK(BM_Combined_KitchenSink)->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// "Typical game forest" with a per-phase breakdown:
//   - arg0 trees on a ground plane, 80% shallow (fanout 4, depth 1 -> 4
//     leaves) and 20% deep (fanout 4, depth 3 -> 64 leaves)
//   - 10% of ALL leaves sway slowly (small local drift) every frame
//   - camera at eye height runs through the forest (~5 m/s at 60 fps)
//   - fully resident, no streaming: cut-only selectCut
// Movers are addressed by NodeHandle, composed at setup from the attach
// result plus the authored leaf indices — the production pattern.
// Phase times are reported as counters, in microseconds per frame:
//   move_us  = setNodeBounds submission (handle check + queue push)
//   refit_us = flushBounds (coalesce + bbox hierarchy + wide-lane patching)
//   cut_us   = TLAS query + per-instance walks + emission
// ---------------------------------------------------------------------------
static void BM_TypicalForest_Breakdown(benchmark::State& state)
{
    using clock = std::chrono::steady_clock;
    const int count = int(state.range(0));
    // ~20 m spacing: 10k trees live on a ~2x2 km map.
    const float half = 10.0f * std::sqrt(float(count));
    std::mt19937 rng(777);
    std::uniform_real_distribution<float> uni(-half, half);

    TreeGen gen;
    World w;
    std::vector<NodeHandle> leaves;   // all leaf handles
    std::vector<float4>     home;     // leaf home centers, local space
    std::vector<uint32_t>   leafIdx;
    for (int i = 0; i < count; ++i)
    {
        const bool deep = (i % 5) == 0;   // 20% deep
        gen.fanout = 4;
        gen.depth = deep ? 3 : 1;
        // Crown ~4 m, proxy error 1 m: refines within ~250 m at 4 px.
        Page pg = gen.makeRootPage(unitRegion(2.0f), 1.0f, 0);
        leafIdx.clear();
        for (uint32_t n = 1; n < pg.nodeCount(); ++n)
        {
            if (pg.childCount(n) != 0) continue;
            leafIdx.push_back(n);
            home.push_back(pg.bbox[n].center());
        }
        const auto inst = addResidentInstance(w, std::move(pg),
                                              float4::point(uni(rng), 0, uni(rng)));
        for (uint32_t n : leafIdx) leaves.push_back(nodeAt(inst.rootPage, n));
    }

    // 10% of all leaves are movers, spread across all trees.
    std::vector<uint32_t> movers;
    for (uint32_t i = 0; i < leaves.size(); i += 10) movers.push_back(i);

    ViewScratch s;
    std::vector<CutEntry> cut;
    const CutParams p{4.0f, 0.1f, 1.0f};
    XorShift32 fast;

    double moveNs = 0, refitNs = 0, cutNs = 0;
    size_t frames = 0, cutTotal = 0;
    float x = -half * 0.7f;
    for (auto _ : state)
    {
        x += 5.0f / 60.0f;   // run speed at 60 fps
        if (x > half * 0.7f) x = -half * 0.7f;

        const auto t0 = clock::now();
        for (const uint32_t i : movers)
        {
            const float4 c = home[i] + float4::vec(fast.uniform(-0.3f, 0.3f),
                                                   fast.uniform(-0.1f, 0.1f),
                                                   fast.uniform(-0.3f, 0.3f));
            w.setNodeBounds(leaves[i],
                            AABB::fromCenterExtent(c, float4::vec(0.5f, 0.5f, 0.5f)));
        }
        const auto t1 = clock::now();
        w.beginFrame();
        w.flushBounds();   // explicit here so refit shows up as its own phase
        const auto t2 = clock::now();
        const CullView v = makePerspectiveView(
            float4::point(x, 1.7f, 0), float4::vec(1.0f, 0.0f, 0.05f),
            float4::vec(0, 1, 0), 1.0f, 16.0f / 9.0f, 1080.0f, 0.1f, 1.0e9f);
        w.selectCut(v, p, s, cut);
        const auto t3 = clock::now();

        moveNs += std::chrono::duration<double, std::nano>(t1 - t0).count();
        refitNs += std::chrono::duration<double, std::nano>(t2 - t1).count();
        cutNs += std::chrono::duration<double, std::nano>(t3 - t2).count();
        cutTotal += cut.size();
        ++frames;
        benchmark::DoNotOptimize(cut.data());
    }
    const double f = double(frames ? frames : 1);
    state.counters["move_us"] = moveNs / f / 1000.0;
    state.counters["refit_us"] = refitNs / f / 1000.0;
    state.counters["cut_us"] = cutNs / f / 1000.0;
    state.counters["movers"] = double(movers.size());
    state.counters["leaves"] = double(leaves.size());
    state.counters["avg_cut"] = double(cutTotal) / f;
}
BENCHMARK(BM_TypicalForest_Breakdown)->Arg(10000)->Arg(50000)
    ->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Dynamic forest: the TypicalForest scenario plus per-frame instance churn —
// arg1 percent of all trees are removed and the same number of new trees
// added at fresh positions, every frame, while 10% of leaves keep swaying
// and the camera keeps running. Runtime adds copy one of two immutable
// prototype pages (payloads are opaque and may repeat, so one authored page
// serves every tree of its shape — the realistic spawn pattern).
// Phase counters, microseconds per frame:
//   churn_us = removeInstance + page copy + addInstance + residency
//   move_us  = setNodeBounds submissions
//   refit_us = flushBounds
//   cut_us   = selectCut (includes any TLAS rebuild the churn forced)
// ---------------------------------------------------------------------------
static void BM_TypicalForest_Churn(benchmark::State& state)
{
    using clock = std::chrono::steady_clock;
    const int count = int(state.range(0));
    const int churnPct = int(state.range(1));
    const float half = 10.0f * std::sqrt(float(count));
    std::mt19937 rng(777);
    std::uniform_real_distribution<float> uni(-half, half);

    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 1;
    const Page protoShallow = gen.makeRootPage(unitRegion(2.0f), 1.0f, 0);
    gen.depth = 3;
    const Page protoDeep = gen.makeRootPage(unitRegion(2.0f), 1.0f, 0);

    struct Proto
    {
        const Page* page;
        std::vector<uint32_t> leafIdx;
        std::vector<float4>   home;
    };
    auto makeProto = [](const Page& pg)
    {
        Proto p{&pg, {}, {}};
        for (uint32_t i = 1; i < pg.nodeCount(); ++i)
        {
            if (pg.childCount(i) != 0) continue;
            p.leafIdx.push_back(i);
            p.home.push_back(pg.bbox[i].center());
        }
        return p;
    };
    const Proto protos[2] = {makeProto(protoShallow), makeProto(protoDeep)};

    World w;
    struct Tree
    {
        World::InstanceRef ref;
        PageHandle  page;
        const Proto* proto;
    };
    std::vector<Tree> trees;
    trees.reserve(size_t(count) * 2);
    auto addTree = [&](bool deep, float4 pos)
    {
        const Proto& pr = protos[deep ? 1 : 0];
        const auto inst = addResidentInstance(w, Page(*pr.page), pos);
        trees.push_back({inst, inst.rootPage, &pr});
    };
    for (int i = 0; i < count; ++i)
        addTree((i % 5) == 0, float4::point(uni(rng), 0, uni(rng)));

    ViewScratch s;
    std::vector<CutEntry> cut;
    const CutParams p{4.0f, 0.1f, 1.0f};
    XorShift32 fast;
    const size_t churnN = size_t(count) * size_t(churnPct) / 100;

    double churnNs = 0, moveNs = 0, refitNs = 0, cutNs = 0;
    size_t frames = 0, cutTotal = 0;
    float x = -half * 0.7f;
    for (auto _ : state)
    {
        x += 5.0f / 60.0f;
        if (x > half * 0.7f) x = -half * 0.7f;

        const auto t0 = clock::now();
        for (size_t k = 0; k < churnN; ++k)
        {
            const size_t i = fast.next() % trees.size();
            w.removeInstance(trees[i].ref);
            trees[i] = trees.back();
            trees.pop_back();
        }
        for (size_t k = 0; k < churnN; ++k)
            addTree((fast.next() % 5) == 0,
                    float4::point(fast.uniform(-half, half), 0,
                                  fast.uniform(-half, half)));
        const auto t1 = clock::now();

        // Every 10th leaf sways, rotating which ones frame to frame.
        for (const Tree& t : trees)
        {
            const Proto& pr = *t.proto;
            for (size_t j = frames % 10; j < pr.leafIdx.size(); j += 10)
            {
                const float4 c = pr.home[j] + float4::vec(fast.uniform(-0.3f, 0.3f),
                                                          fast.uniform(-0.1f, 0.1f),
                                                          fast.uniform(-0.3f, 0.3f));
                w.setNodeBounds(nodeAt(t.page, pr.leafIdx[j]),
                                AABB::fromCenterExtent(c, float4::vec(0.5f, 0.5f, 0.5f)));
            }
        }
        const auto t2 = clock::now();
        w.beginFrame();
        w.flushBounds();
        const auto t3 = clock::now();
        const CullView v = makePerspectiveView(
            float4::point(x, 1.7f, 0), float4::vec(1.0f, 0.0f, 0.05f),
            float4::vec(0, 1, 0), 1.0f, 16.0f / 9.0f, 1080.0f, 0.1f, 1.0e9f);
        w.selectCut(v, p, s, cut);
        const auto t4 = clock::now();

        churnNs += std::chrono::duration<double, std::nano>(t1 - t0).count();
        moveNs += std::chrono::duration<double, std::nano>(t2 - t1).count();
        refitNs += std::chrono::duration<double, std::nano>(t3 - t2).count();
        cutNs += std::chrono::duration<double, std::nano>(t4 - t3).count();
        cutTotal += cut.size();
        ++frames;
        benchmark::DoNotOptimize(cut.data());
    }
    const double f = double(frames ? frames : 1);
    state.counters["churn_us"] = churnNs / f / 1000.0;
    state.counters["move_us"] = moveNs / f / 1000.0;
    state.counters["refit_us"] = refitNs / f / 1000.0;
    state.counters["cut_us"] = cutNs / f / 1000.0;
    state.counters["churn_pf"] = double(churnN) * 2.0;
    state.counters["avg_cut"] = double(cutTotal) / f;
}
BENCHMARK(BM_TypicalForest_Churn)
    ->Args({10000, 5})->Args({50000, 5})
    ->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// HLodBuilder cost: composing a page at runtime (spawned structures,
// procedural content). Builds a complete fanout^depth tree — createRoot +
// createNode per node + build() (preorder layout, wide-block packing,
// invariant checks) — per iteration.
//   4^1 =     5 nodes: a shallow prop
//   4^3 =    85 nodes: a typical deep tree page
//   8^4 =  4681 nodes: a large streamed page
// ---------------------------------------------------------------------------
static void BM_Builder_BuildPage(benchmark::State& state)
{
    const uint32_t fanout = uint32_t(state.range(0));
    const uint32_t depth = uint32_t(state.range(1));

    uint64_t nodes = 0;
    for (auto _ : state)
    {
        HLodBuilder b;
        uint64_t payload = 1;
        const auto root = b.createRoot(payload++, float(1u << (2 * depth)),
                                       AABB::fromCenterExtent(float4::vec(0, 0, 0),
                                                              float4::vec(100, 100, 100)));
        // Iterative BFS expansion; children boxes subdivide the parent's.
        struct Item { HLodBuilder::NodeId node; AABB box; uint32_t level; };
        std::vector<Item> queue;
        queue.push_back({root, AABB::fromCenterExtent(float4::vec(0, 0, 0),
                                                      float4::vec(100, 100, 100)), 0});
        nodes = 1;
        for (size_t qi = 0; qi < queue.size(); ++qi)
        {
            const Item it = queue[qi];
            if (it.level >= depth) continue;
            const float err = float(1u << (2 * (depth - it.level - 1)));
            const float4 c = it.box.center();
            const float4 e = it.box.extent() * 0.25f;
            for (uint32_t k = 0; k < fanout; ++k)
            {
                const float4 off = float4::vec((k & 1 ? 1.0f : -1.0f) * e.x,
                                               (k & 2 ? 1.0f : -1.0f) * e.y,
                                               (k & 4 ? 1.0f : -1.0f) * e.z);
                const AABB cb = AABB::fromCenterExtent(c + off, e);
                const auto n = b.createNode(it.node, payload++, err, cb);
                queue.push_back({n, cb, it.level + 1});
                ++nodes;
            }
        }
        Page pg = b.build();
        benchmark::DoNotOptimize(pg.nodeCount());
    }
    state.SetItemsProcessed(int64_t(state.iterations()) * int64_t(nodes));
    state.counters["nodes"] = double(nodes);
}
BENCHMARK(BM_Builder_BuildPage)
    ->Args({4, 1})->Args({4, 3})->Args({8, 4})
    ->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Output sensitivity: the same 300k-node world queried at different error
// thresholds. Cost must track the OUTPUT size (ns_per_entry roughly flat),
// not the world size — that is the core complexity claim.
// ---------------------------------------------------------------------------
static void BM_CutScaling_OutputSensitivity(benchmark::State& state)
{
    World w = makeDeepWorld(8, 6);
    ViewScratch s;
    std::vector<CutEntry> cut;
    const CullView v = orbitView(0.7f, 2500.0f);
    const CutParams p{float(state.range(0)), 0.0f, 0.0f};

    for (auto _ : state)
    {
        w.beginFrame();
        w.selectCut(v, p, s, cut);
        benchmark::DoNotOptimize(cut.data());
    }
    state.counters["cut"] = double(cut.size());
    // Inverted iteration-invariant rate: seconds per cut entry (SI-suffixed).
    state.counters["per_entry"] = benchmark::Counter(
        double(cut.size()),
        benchmark::Counter::kIsIterationInvariantRate | benchmark::Counter::kInvert);
}
BENCHMARK(BM_CutScaling_OutputSensitivity)
    ->Arg(64)->Arg(16)->Arg(4)->Arg(1)
    ->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Camera teleport: worst-case frame latency when every locality assumption
// breaks at once — scratch stamps stale, hysteresis history useless, page
// memory cold. Steady frames orbit smoothly; every 16th frame jumps to a
// random spot. steady_us vs teleport_us is the spike factor.
// ---------------------------------------------------------------------------
static void BM_CameraTeleport_ColdFrame(benchmark::State& state)
{
    using clock = std::chrono::steady_clock;
    std::mt19937 rng(777);
    const float half = 6000.0f;
    std::uniform_real_distribution<float> uni(-half, half);

    TreeGen propGen;
    propGen.fanout = 4;
    propGen.depth = 1;
    World w;
    for (int i = 0; i < 20000; ++i)
        addResidentInstance(w, propGen.makeRootPage(unitRegion(6.0f), 24.0f, 0),
                            float4::point(uni(rng), 0, uni(rng)));
    TreeGen deepGen;
    deepGen.fanout = 8;
    deepGen.depth = 5;
    deepGen.nextId = 1u << 24;
    addResidentInstance(w, deepGen.makeRootPage(unitRegion(800.0f), 2048.0f, 0),
                        float4::point(0, 300.0f, 0));

    ViewScratch s;
    std::vector<CutEntry> cut;
    const CutParams p{4.0f, 0.1f, 1.0f};
    XorShift32 fast;

    double steadyNs = 0, teleportNs = 0;
    size_t steadyFrames = 0, teleportFrames = 0;
    float4 center = float4::point(0, 0, 0);
    float t = 0.0f;
    for (auto _ : state)
    {
        t += 0.01f;
        const bool jump = ((steadyFrames + teleportFrames) % 16) == 15;
        if (jump) center = float4::point(fast.uniform(-half, half), 0,
                                         fast.uniform(-half, half));
        const CullView v = orbitView(t, 400.0f, center);

        const auto t0 = clock::now();
        w.beginFrame();
        w.selectCut(v, p, s, cut);
        const auto t1 = clock::now();
        benchmark::DoNotOptimize(cut.data());

        const double ns = std::chrono::duration<double, std::nano>(t1 - t0).count();
        if (jump) { teleportNs += ns; ++teleportFrames; }
        else      { steadyNs += ns; ++steadyFrames; }
    }
    state.counters["steady_us"] = steadyNs / double(steadyFrames ? steadyFrames : 1) / 1000.0;
    state.counters["teleport_us"] =
        teleportNs / double(teleportFrames ? teleportFrames : 1) / 1000.0;
}
BENCHMARK(BM_CameraTeleport_ColdFrame)->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Multi-view frame: main camera + 3 extra views (shadow cascades, a mirror)
// against one world, each with its own scratch. The marginal cost of an
// extra view should be well below the main view (shared world data is hot).
// ---------------------------------------------------------------------------
static void BM_MultiView(benchmark::State& state)
{
    using clock = std::chrono::steady_clock;
    std::mt19937 rng(4321);
    const float half = 4000.0f;
    std::uniform_real_distribution<float> uni(-half, half);

    TreeGen propGen;
    propGen.fanout = 4;
    propGen.depth = 1;
    World w;
    for (int i = 0; i < 10000; ++i)
        addResidentInstance(w, propGen.makeRootPage(unitRegion(6.0f), 24.0f, 0),
                            float4::point(uni(rng), 0, uni(rng)));
    TreeGen deepGen;
    deepGen.fanout = 8;
    deepGen.depth = 5;
    deepGen.nextId = 1u << 24;
    addResidentInstance(w, deepGen.makeRootPage(unitRegion(600.0f), 2048.0f, 0),
                        float4::point(0, 250.0f, 0));

    ViewScratch scr[4];
    std::vector<CutEntry> cut[4];
    const CutParams p{4.0f, 0.1f, 1.0f};

    double mainNs = 0, extraNs = 0;
    size_t frames = 0;
    float t = 0.0f;
    for (auto _ : state)
    {
        t += 0.01f;
        w.beginFrame();
        const float4 eye = float4::vec(std::cos(t) * 900.0f, 300.0f, std::sin(t) * 900.0f);

        const auto t0 = clock::now();
        w.selectCut(makeLookAtView(eye, float4::point(0, 0, 0)), p, scr[0], cut[0]);
        const auto t1 = clock::now();
        // Extra views: same eye, different directions (cascade-style).
        w.selectCut(makeLookAtView(eye, float4::point(2000, 0, 0)), p, scr[1], cut[1]);
        w.selectCut(makeLookAtView(eye, float4::point(-1000, 0, 1500)), p, scr[2], cut[2]);
        w.selectCut(makeLookAtView(eye + float4::vec(0, 1200, 0), eye), p, scr[3], cut[3]);
        const auto t2 = clock::now();

        for (auto& c : cut) benchmark::DoNotOptimize(c.data());
        mainNs += std::chrono::duration<double, std::nano>(t1 - t0).count();
        extraNs += std::chrono::duration<double, std::nano>(t2 - t1).count();
        ++frames;
    }
    const double f = double(frames ? frames : 1);
    state.counters["main_us"] = mainNs / f / 1000.0;
    state.counters["extra_view_us"] = extraNs / f / 3.0 / 1000.0;
}
BENCHMARK(BM_MultiView)->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Streaming convergence as ERROR DECAY: the camera teleports into an
// unexpanded region of an unbounded view (no far-plane trick — a collapsed
// node is never culled, it renders coarse until its page arrives). The
// metric is the worst residual screen error among still-expandable
// NEEDS_EXPANSION entries, sampled 1/2/4/8/16/32 frames after each teleport.
// Near-field detail must land in a few frames; the far field may take
// seconds — that distance-proportional tolerance is exactly what the
// err-priority ordering produces.
//
// Two attach policies at the same total page budget:
//   discovery (arg1 = 0): attach the requested level only, largest error
//     first; the next level is discovered by the next walk — a chain D
//     pages deep costs D frames of latency regardless of budget.
//   predictive (arg1 = 1): the walk cannot see below a missing page, but
//     the entry's error already says how deep the chain goes:
//         levels ~= ceil(log2(err / threshold) / halvings_per_page)
//     A 100 px entry at a 4 px threshold is >1 level with certainty. The
//     streamer keeps a max-heap of candidates keyed by (estimated) screen
//     error: it pops the globally worst one, attaches its page, estimates
//     the fresh expansions from the page's own data (bbox + geomError vs
//     the camera), and pushes the over-threshold ones back into the heap —
//     the same greedy order as discovery, but new levels become attachable
//     in the SAME frame instead of after the next walk. Mispredicted pages
//     just age out through the GC.
//
//     (A first cut of this policy did plain depth-first recursion instead
//     of a global heap; it was WORSE than discovery, because one entry's
//     subtree — x16 candidates per level — swallowed the whole frame budget
//     and starved every other teleport hotspot. The heap restores global
//     priority order; the lookahead is the only difference that remains.)
// ---------------------------------------------------------------------------
namespace {

// One frame of the predictive policy, entirely content-side (the streamer
// owns the page data and the recipes; the World only sees attachPage).
void predictiveAttachFrame(World& w, TreeGen& gen, const std::vector<IdealEntry>& ideal,
                           const CullView& view, float threshold, size_t budget)
{
    struct Cand
    {
        float       err;
        UserPayload payload;
        NodeHandle  node;
        bool operator<(const Cand& o) const { return err < o.err; }
    };
    static std::priority_queue<Cand> heap;   // max-heap by (estimated) error
    while (!heap.empty()) heap.pop();

    for (const IdealEntry& e : ideal)
        if (e.tag == IdealTag::NeedsExpansion && gen.recipes.count(e.payload))
            heap.push({e.err, e.payload, e.node});

    while (budget != 0 && !heap.empty())
    {
        const Cand c = heap.top();
        heap.pop();
        Page child = gen.makeChildPage(c.payload);

        // Estimate the fresh expansion points BEFORE handing the page over.
        struct Fresh
        {
            uint32_t idx;
            UserPayload payload;
            float    est;
        };
        static std::vector<Fresh> fresh;
        fresh.clear();
        for (uint32_t i = 1; i < child.nodeCount(); ++i)
        {
            if (!child.isExpansion(i)) continue;
            const float est = screenError(child.geometricError[i], view.k,
                                          distanceToBox(child.bbox[i], view.pos));
            if (est > threshold && gen.recipes.count(child.payload[i]))
                fresh.push_back({i, child.payload[i], est});
        }

        const uint32_t n = child.nodeCount();
        const PageHandle ph = w.attachPage(c.node, std::move(child));
        if (!ph.valid()) continue;
        markAllResident(w, ph, n);
        --budget;

        for (const Fresh& f : fresh)
            heap.push({f.est, f.payload, nodeAt(ph, f.idx)});
    }
}

} // namespace

static void BM_StreamingConvergence(benchmark::State& state)
{
    const size_t attachBudget = size_t(state.range(0));
    const bool predictive = state.range(1) != 0;

    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 2;
    World w;
    addResidentInstance(w, gen.makeRootPage(unitRegion(4000.0f), float(1u << 17), 3),
                        float4::point(0, 0, 0));

    ViewScratch s;
    Outputs o;
    const CutParams p{4.0f, 0.1f, 0.0f};
    XorShift32 fast;

    constexpr int kPeriod = 33;                          // teleport every 33 frames
    constexpr int kSampleAt[6] = {1, 2, 4, 8, 16, 32};   // frames after teleport
    double resSum[6] = {};
    size_t resCnt[6] = {};

    int frameInCycle = kPeriod;   // force a teleport on the first frame
    float4 eye;
    for (auto _ : state)
    {
        if (frameInCycle >= kPeriod)
        {
            eye = float4::point(fast.uniform(-3500, 3500), 60,
                                fast.uniform(-3500, 3500));
            frameInCycle = 0;
        }
        ++frameInCycle;

        const CullView v = makeLookAtView(eye, eye + float4::vec(1, -0.15f, 0.3f));
        w.beginFrame();
        w.selectCut(v, p, s, o.cut, o.ideal, o.reqs);

        // Residual: worst screen error still waiting on an expansion.
        // Clamped: the camera inside a collapsed box saturates err toward
        // 1e33+ (distance 0), and one such frame would swamp the average —
        // 1e6 px already means "the box you are standing in is unexpanded".
        float worst = 0.0f;
        for (const auto& e : o.ideal)
            if (e.tag == IdealTag::NeedsExpansion && e.err > worst &&
                gen.recipes.count(e.payload))
                worst = e.err;
        worst = worst < 1.0e6f ? worst : 1.0e6f;
        for (int b = 0; b < 6; ++b)
            if (frameInCycle == kSampleAt[b])
            {
                resSum[b] += worst;
                ++resCnt[b];
            }

        if (predictive)
            predictiveAttachFrame(w, gen, o.ideal, v, p.threshold, attachBudget);
        else
            attachTopByPriority(w, gen, o.ideal, attachBudget);
        for (const auto& r : o.reqs) w.markResident(r.node);
        w.collect(20000, 30);
        benchmark::DoNotOptimize(o.cut.data());
    }
    state.counters["px_f1"] = resSum[0] / double(resCnt[0] ? resCnt[0] : 1);
    state.counters["px_f2"] = resSum[1] / double(resCnt[1] ? resCnt[1] : 1);
    state.counters["px_f4"] = resSum[2] / double(resCnt[2] ? resCnt[2] : 1);
    state.counters["px_f8"] = resSum[3] / double(resCnt[3] ? resCnt[3] : 1);
    state.counters["px_f16"] = resSum[4] / double(resCnt[4] ? resCnt[4] : 1);
    state.counters["px_f32"] = resSum[5] / double(resCnt[5] ? resCnt[5] : 1);
    state.counters["pages"] = double(w.attachedPageCount());
}
BENCHMARK(BM_StreamingConvergence)
    ->Args({32, 0})->Args({32, 1})->Args({128, 0})->Args({128, 1})
    ->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// TLAS at scale: hundreds of thousands of instances. Steady cost is the
// output-sensitive TLAS query + visible-instance walks; firstcut_ms is the
// level-load burst (quality TLAS build + first full query) paid once.
// ---------------------------------------------------------------------------
static void BM_TlasScale(benchmark::State& state)
{
    using clock = std::chrono::steady_clock;
    const int count = int(state.range(0));
    const float half = 40.0f * std::sqrt(float(count));
    std::mt19937 rng(1717);
    std::uniform_real_distribution<float> uni(-half, half);

    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 1;
    const Page proto = gen.makeRootPage(unitRegion(5.0f), 16.0f, 0);

    World w;
    for (int i = 0; i < count; ++i)
        addResidentInstance(w, Page(proto), float4::point(uni(rng), 0, uni(rng)));

    ViewScratch s;
    std::vector<CutEntry> cut;
    const CutParams p{4.0f, 0.1f, 1.0f};

    // Level-load burst: the first cut pays the quality TLAS build.
    const auto b0 = clock::now();
    w.beginFrame();
    w.selectCut(orbitView(0.0f, half * 0.25f), p, s, cut);
    const auto b1 = clock::now();
    const double firstMs = std::chrono::duration<double, std::milli>(b1 - b0).count();

    float t = 0.0f;
    for (auto _ : state)
    {
        t += 0.01f;
        w.beginFrame();
        w.selectCut(orbitView(t, half * 0.25f), p, s, cut);
        benchmark::DoNotOptimize(cut.data());
    }
    state.counters["cut"] = double(cut.size());
    state.counters["firstcut_ms"] = firstMs;
}
BENCHMARK(BM_TlasScale)
    ->Arg(200000)->Arg(500000)
    ->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Adversarial: 10k instances stacked in the same spot. The TLAS cannot
// separate them spatially — every query wades through all of them. This is
// the pathological floor, not a target scenario.
// ---------------------------------------------------------------------------
static void BM_Adversarial_StackedInstances(benchmark::State& state)
{
    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 1;
    const Page proto = gen.makeRootPage(unitRegion(5.0f), 16.0f, 0);

    World w;
    XorShift32 fast;
    for (int i = 0; i < 10000; ++i)
        addResidentInstance(w, Page(proto),
                            float4::point(fast.uniform(-0.01f, 0.01f), 0,
                                          fast.uniform(-0.01f, 0.01f)));

    ViewScratch s;
    std::vector<CutEntry> cut;
    const CutParams p{4.0f, 0.1f, 0.0f};
    const CullView v = makeLookAtView(float4::point(0, 20, -60), float4::point(0, 0, 0));

    for (auto _ : state)
    {
        w.beginFrame();
        w.selectCut(v, p, s, cut);
        benchmark::DoNotOptimize(cut.data());
    }
    state.counters["cut"] = double(cut.size());
}
BENCHMARK(BM_Adversarial_StackedInstances)->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Adversarial: one node with the maximum 511 children — 64 wide blocks under
// a single parent, no hierarchy to prune with. Exercises the wide-test loop
// at its widest.
// ---------------------------------------------------------------------------
static void BM_Adversarial_WideNode(benchmark::State& state)
{
    HLodBuilder b;
    const auto root = b.createRoot(1, 512.0f, AABB::empty());
    XorShift32 fast;
    for (uint32_t i = 0; i < kMaxChildren; ++i)
    {
        const float4 c = float4::vec(fast.uniform(-200, 200), fast.uniform(-20, 20),
                                     fast.uniform(-200, 200));
        b.createNode(root, 100 + i, 0.0f, AABB::fromCenterExtent(c, float4::vec(1, 1, 1)));
    }
    World w;
    addResidentInstance(w, b.build(), float4::point(0, 0, 0));

    ViewScratch s;
    std::vector<CutEntry> cut;
    const CutParams p{4.0f, 0.1f, 0.0f};

    float t = 0.0f;
    for (auto _ : state)
    {
        t += 0.02f;
        w.beginFrame();
        w.selectCut(orbitView(t, 300.0f), p, s, cut);
        benchmark::DoNotOptimize(cut.data());
    }
    state.counters["cut"] = double(cut.size());
}
BENCHMARK(BM_Adversarial_WideNode)->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Adversarial: a 25-page-deep expansion chain (fanout 1) fully attached, with
// the camera close enough to force the walk through every page boundary for
// a single-entry cut — the maximum page-crossing overhead per output.
// ---------------------------------------------------------------------------
static void BM_Adversarial_DeepPageChain(benchmark::State& state)
{
    TreeGen gen;
    gen.fanout = 1;
    gen.depth = 1;
    World w;
    // Astronomical root error so refinement pressure survives 25 halving
    // pages and the walk is forced through the entire chain.
    addResidentInstance(w, gen.makeRootPage(unitRegion(50.0f), 1.0e15f, 25),
                        float4::point(0, 0, 0));

    // Attach the whole chain: each page exposes exactly one expansion point.
    size_t depth = 0;
    for (;;)
    {
        UserPayload next = 0;
        bool found = false;
        for (const auto& [id, rec] : gen.recipes)
            if (contains(w, id) && !isAttached(w, id)) { next = id; found = true; break; }
        if (!found) break;
        Page child = gen.makeChildPage(next);
        const uint32_t n = child.nodeCount();
        const PageHandle ph = attachPage(w, next, std::move(child));
        markAllResident(w, ph, n);
        ++depth;
    }

    ViewScratch s;
    std::vector<CutEntry> cut;
    const CutParams p{4.0f, 0.1f, 0.0f};
    const CullView v = makeLookAtView(float4::point(0, 3, -8), float4::point(0, 0, 0));

    for (auto _ : state)
    {
        w.beginFrame();
        w.selectCut(v, p, s, cut);
        benchmark::DoNotOptimize(cut.data());
    }
    state.counters["pages"] = double(depth + 1);
    state.counters["cut"] = double(cut.size());
}
BENCHMARK(BM_Adversarial_DeepPageChain)->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Harness self-check: the cost of the random numbers themselves, arg = draws
// per iteration. Measures the generator, not the library.
//
// std::uniform_real_distribution over libstdc++ is pathologically slow under
// clang: generate_canonical() computes logl() of a constant, which GCC folds
// at compile time but clang calls at runtime, per sample — ~90 ns on x86-64
// (x87 fp80) and ~480 ns on aarch64 where long double is software binary128
// (llvm/llvm-project#19916). BM_MovingLeafNodes/10000 used to draw 30k such
// samples per frame inside its timed loop, which made it look 6-25x slower
// on linux-clang. All timed loops now use XorShift32; these two benches keep
// the harness cost visible so a regression of this kind is obvious.
// ---------------------------------------------------------------------------
static void BM_Harness_StdUniformReal(benchmark::State& state)
{
    std::mt19937 rng(31337);
    std::uniform_real_distribution<float> uni(-900.0f, 900.0f);
    const int draws = int(state.range(0));
    float acc = 0.0f;
    for (auto _ : state)
    {
        for (int i = 0; i < draws; ++i) acc += uni(rng);
        benchmark::DoNotOptimize(acc);
    }
}
BENCHMARK(BM_Harness_StdUniformReal)->Arg(30000)->Unit(benchmark::kMicrosecond);

static void BM_Harness_XorShift(benchmark::State& state)
{
    XorShift32 rng;
    const int draws = int(state.range(0));
    float acc = 0.0f;
    for (auto _ : state)
    {
        for (int i = 0; i < draws; ++i) acc += rng.uniform(-900.0f, 900.0f);
        benchmark::DoNotOptimize(acc);
    }
}
BENCHMARK(BM_Harness_XorShift)->Arg(30000)->Unit(benchmark::kMicrosecond);
