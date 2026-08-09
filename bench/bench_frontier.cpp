// Performance tests for Frontier corner cases:
//   - one very deep tree, static camera and fly-through
//   - deep paged tree with live streaming (attach/resident/collect loop)
//   - many shallow trees (forests), with and without contribution culling
//   - moving top-level instances
//   - moving leaf nodes (bounds refit)
//   - payload residency churn
//   - dynamic instance churn (spawn/despawn every frame)
//   - PageBuilder page composition (runtime tree creation)
//   - output-sensitivity scaling (cost vs cut size at fixed world size)
//   - camera teleport (cold-frame latency spike)
//   - multi-view frames (marginal cost of extra views)
//   - streaming convergence latency (frames to ideal == current)
//   - TLAS at 200k-500k instances + level-load burst
//   - 100k flat forests: TLAS-only vs cached/uncached result materialization
//   - adversarial shapes (stacked instances, max-width node, deep page chain)
//
// All setup happens outside the timed loop; iterations time selectFrontier plus
// whatever per-frame work the scenario prescribes.

#include <benchmark/benchmark.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <barrier>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <queue>
#include <thread>
#include <vector>

#include "helpers.h"   // TreeGen etc. (shared with the unit tests)

using namespace frontier;
using namespace frontiertest;

namespace {

struct UncachedSpatialQuery : SpatialQuery
{
    UncachedSpatialQuery() { setReuseEnabled(false); }
};

struct Outputs
{
    UncachedSpatialQuery query;
    FrontierResult cut;
};

void consumeFrontier(const FrontierResultView& cut)
{
    benchmark::DoNotOptimize(cut.shared.data());
    benchmark::DoNotOptimize(cut.currentOnly.data());
    benchmark::DoNotOptimize(cut.idealOnly.data());
}

Camera orbitView(float t, float dist, float4 center = float4::point(0, 0, 0))
{
    const float4 pos = center + float4::vec(std::cos(t) * dist, dist * 0.35f,
                                            std::sin(t) * dist);
    return makeLookAtCamera(pos, center);
}

constexpr uint32_t kDynamicWorkloadSeed = 0x9e3779b9u;

// Attach up to `budget` candidate expansions, most-visible (largest screen
// error) first. The order of ideal-cut entries is traversal-defined, so a
// streaming policy must prioritize by err — picking "the first N" makes the
// attach set depend on walk order and over-refines wherever the walk went
// deep first (measurably worse: the hot set churns instead of converging).
size_t attachTopByPriority(SpatialDatabase& w, TreeGen& gen,
                           const FrontierResultView& cut, size_t budget)
{
    static std::vector<const FrontierEntry*> candidates;
    candidates.clear();
    const auto gather = [&](const auto& entries)
    {
        for (const FrontierEntry& entry : entries)
        {
            const UserId id = payloadOf(w, entry);
            if (entry.overThreshold() && gen.recipes.count(id) &&
                !w.isAttached(entry.nodeHandle))
                candidates.push_back(&entry);
        }
    };
    gather(cut.shared);
    gather(cut.idealOnly);
    const size_t take = candidates.size() < budget ? candidates.size() : budget;
    std::partial_sort(candidates.begin(), candidates.begin() + ptrdiff_t(take),
                      candidates.end(),
                      [](const FrontierEntry* a, const FrontierEntry* b) {
                          return a->errorCode() > b->errorCode();
                      });
    for (size_t j = 0; j < take; ++j)
    {
        // Production flow: content is keyed by payload, attach by handle.
        const FrontierEntry& e = *candidates[j];
        Page child = gen.makeChildPage(payloadOf(w, e));
        const uint32_t n = child.nodeCount();
        const PageHandle ph = w.attachPage(e.nodeHandle, std::move(child));
        if (ph.valid()) markAllResident(w, ph, n);   // payload streaming is instant
    }
    return take;
}

// Example streaming policy: make every ideal-cut payload resident.
// A production streamer would normally deduplicate shared payload ids, apply
// priorities/budgets, and complete these calls asynchronously.
size_t makeIdealResident(SpatialDatabase& w, const FrontierResultView& cut)
{
    const auto mark = [&](const auto& entries)
    {
        for (const FrontierEntry& e : entries)
            if (!w.isResident(e.nodeHandle)) w.markResident(e.nodeHandle);
    };
    mark(cut.shared);
    mark(cut.idealOnly);
    return cut.currentSize();
}

// Add a root page and mark every node resident (handles composed from the
// attach result — no lookups anywhere).
SpatialDatabase::InstanceRef addResidentInstance(SpatialDatabase& w, Page pg, float4 pos, float scale = 1.0f)
{
    const uint32_t n = pg.nodeCount();
    const auto inst = w.addInstance(std::move(pg), pos, scale);
    markAllResident(w, inst.rootPage, n);
    return inst;
}

// One fully resident deep tree. depth 6 / fanout 8 is ~300k nodes.
// (unique_ptr because SpatialDatabase is neither copyable nor movable: mounts and
// instances refer to each other by slot, and owned pages point into the
// SpatialDatabase's context.)
std::unique_ptr<SpatialDatabase> makeDeepWorld(uint32_t fanout, uint32_t depth,
                                     TreeGen* genOut = nullptr)
{
    TreeGen gen;
    gen.fanout = fanout;
    gen.depth = depth;
    auto w = std::make_unique<SpatialDatabase>();
    addResidentInstance(*w, gen.makeRootPage(unitRegion(1000.0f), 4096.0f, 0),
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
    const auto wp = makeDeepWorld(8, uint32_t(state.range(0)));
    SpatialDatabase& w = *wp;
    Outputs o;
    const Camera v = orbitView(0.7f, 2500.0f);
    const SelectionParams p{4.0f, 0.0f};

    for (auto _ : state)
    {
        w.applyUpdates();
        o.query.selectFrontier(w, v, p, o.cut);
        consumeFrontier(o.cut);
    }
    state.counters["frontier"] = double(o.cut.size());
}
BENCHMARK(BM_DeepTree_StaticCamera)->Arg(4)->Arg(5)->Arg(6)->Unit(benchmark::kMicrosecond);

// Same scene through an explicit owning snapshot. The fully resident result is
// entirely Shared.
static void BM_DeepTree_FrontierOnly(benchmark::State& state)
{
    UncachedSpatialQuery selection;
    const auto wp = makeDeepWorld(8, uint32_t(state.range(0)));
    SpatialDatabase& w = *wp;
    FrontierResult cut;
    const Camera v = orbitView(0.7f, 2500.0f);
    const SelectionParams p{4.0f, 0.0f};

    for (auto _ : state)
    {
        w.applyUpdates();
        selection.selectFrontier(w, v, p, cut);
        consumeFrontier(cut);
    }
    state.counters["frontier"] = double(cut.size());
}
BENCHMARK(BM_DeepTree_FrontierOnly)->Arg(6)->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Deep tree, flying camera: refinement churn, epoch reuse.
// ---------------------------------------------------------------------------
static void BM_DeepTree_FlyThrough(benchmark::State& state)
{
    const auto wp = makeDeepWorld(8, uint32_t(state.range(0)));
    SpatialDatabase& w = *wp;
    Outputs o;
    const SelectionParams p{4.0f, 0.0f};

    float t = 0.0f;
    size_t frontierTotal = 0, frames = 0;
    for (auto _ : state)
    {
        t += 0.02f;
        // Swoop in and out while orbiting.
        const float dist = 1800.0f + 1500.0f * std::sin(t * 3.1f);
        const Camera v = orbitView(t, dist);
        w.applyUpdates();
        o.query.selectFrontier(w, v, p, o.cut);
        consumeFrontier(o.cut);
        frontierTotal += o.cut.size();
        ++frames;
    }
    state.counters["avg_frontier"] = double(frontierTotal) / double(frames ? frames : 1);
}
BENCHMARK(BM_DeepTree_FlyThrough)->Arg(5)->Arg(6)->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// LOD damping cost: the camera envelope is conservative, so a damped cut is
// larger than an undamped one — that growth is the honest price of damping
// (the cut is output-bound). arg0 = depth, arg1 = damper half-life in frames
// (0 = envelope collapsed, i.e. off). Compare avg_frontier across arg1; time per
// emitted entry should not move (the damped and undamped walks execute the
// same instruction sequence).
// The flight swoops through 30k–90k units, where the refinement boundary of
// the mid tree levels actually lives (leaf refine distance for this world is
// ~63k units at 4 px) — the close-in FlyThrough orbit leaves the whole
// visible tree fully refined, and then damping has nothing to damp.
// Iterations are FIXED so every variant flies the identical path — avg_frontier
// would otherwise depend on how many iterations the harness happens to pick.
// ---------------------------------------------------------------------------
static void BM_DeepTree_FlyThroughDamped(benchmark::State& state)
{
    const auto wp = makeDeepWorld(8, uint32_t(state.range(0)));
    SpatialDatabase& w = *wp;
    Outputs o;
    const SelectionParams p{4.0f, 0.0f};
    CameraDamper damper(float(state.range(1)));

    float t = 0.0f;
    size_t frontierTotal = 0, frames = 0;
    for (auto _ : state)
    {
        t += 0.02f;
        const float dist = 60000.0f + 30000.0f * std::sin(t * 3.1f);
        const Camera v = damper.damp(orbitView(t, dist));
        w.applyUpdates();
        o.query.selectFrontier(w, v, p, o.cut);
        consumeFrontier(o.cut);
        frontierTotal += o.cut.size();
        ++frames;
    }
    state.counters["avg_frontier"] = double(frontierTotal) / double(frames ? frames : 1);
}
BENCHMARK(BM_DeepTree_FlyThroughDamped)
    ->Args({6, 0})->Args({6, 4})->Args({6, 16})
    ->Iterations(512)
    ->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Paged "planet": fly in/out while streaming topology and payloads with the
// cut's own outputs, plus garbage collection back to a page budget.
// ---------------------------------------------------------------------------
static void BM_PagedPlanet_StreamingFly(benchmark::State& state)
{
    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 2;
    SpatialDatabase w;
    addResidentInstance(w, gen.makeRootPage(unitRegion(100000.0f), 1u << 20, 4),
                        float4::point(0, 0, 0));

    Outputs o;
    PageUsageContext usage;
    const SelectionParams p{8.0f, 0.0f};
    const size_t pageBudget = size_t(state.range(0));

    float t = 0.0f;
    size_t attaches = 0;
    for (auto _ : state)
    {
        t += 0.03f;
        const float dist = 250000.0f * std::exp(-2.5f * (0.5f + 0.5f * std::sin(t)));
        const Camera v = orbitView(t * 0.2f, 20000.0f + dist);

        w.applyUpdates();
        o.query.selectFrontier(w, v, p, usage, o.cut);

        attaches += attachTopByPriority(w, gen, o.cut, 8);
        makeIdealResident(w, o.cut);
        w.collect(usage, pageBudget, 16);
        consumeFrontier(o.cut);
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
    SpatialDatabase w;
    const float half = 100000.0f;
    addResidentInstance(w, gen.makeRootPage(unitRegion(half), float(1u << 20), 4),
                        float4::point(0, 0, 0));

    Outputs o;
    PageUsageContext usage;
    const SelectionParams p{8.0f, 1.0f};
    const size_t pageBudget = size_t(state.range(0));
    const uint32_t minAge = 8;
    const float speed = half / 250.0f;   // crosses the whole world in ~500 frames

    float x = -half;
    size_t attaches = 0, collected = 0, frames = 0, frontierTotal = 0;
    for (auto _ : state)
    {
        x += speed;
        if (x > half) x = -half;   // wrap: revisit collapsed regions
        const Camera v = makePerspectiveCamera(
            float4::point(x, half * 0.02f, 0), float4::vec(1.0f, -0.15f, 0.0f),
            float4::vec(0, 1, 0), 1.0f, 16.0f / 9.0f, 1080.0f, 0.1f, 1.0e9f);

        w.applyUpdates();
        o.query.selectFrontier(w, v, p, usage, o.cut);

        attaches += attachTopByPriority(w, gen, o.cut, 16);
        const size_t currentCount = makeIdealResident(w, o.cut);
        collected += w.collect(usage, pageBudget, minAge).detachedPages;

        frontierTotal += currentCount;
        ++frames;
        consumeFrontier(o.cut);
    }
    state.counters["attached"] = double(w.attachedPageCount());
    state.counters["attach_pf"] = double(attaches) / double(frames ? frames : 1);
    state.counters["collect_pf"] = double(collected) / double(frames ? frames : 1);
    state.counters["avg_frontier"] = double(frontierTotal) / double(frames ? frames : 1);
}
BENCHMARK(BM_GcStress_FastFlythrough)->Arg(96)->Arg(768)->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Forests: many shallow multi-root trees. arg0 = instances, arg1 = minPix on.
// ---------------------------------------------------------------------------
static void BM_ManyShallowTrees(benchmark::State& state)
{
    DeterministicRng rng(1234);
    const int count = int(state.range(0));
    const float area = 60.0f * std::sqrt(float(count));
    DeterministicUniformFloat uni(-area, area);

    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 1;
    SpatialDatabase w;
    for (int i = 0; i < count; ++i)
        addResidentInstance(w, gen.makeRootPage(unitRegion(4.0f), 8.0f, 0),
                            float4::point(uni(rng), 0, uni(rng)));

    Outputs o;
    const SelectionParams p{4.0f, state.range(1) ? 1.0f : 0.0f};

    float t = 0.0f;
    for (auto _ : state)
    {
        t += 0.01f;
        const Camera v = orbitView(t, area * 0.4f);
        w.applyUpdates();
        o.query.selectFrontier(w, v, p, o.cut);
        consumeFrontier(o.cut);
    }
    state.counters["frontier"] = double(o.cut.size());
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
    DeterministicRng rng(555);
    const int count = int(state.range(0));
    const int movers = int(state.range(1));
    const float area = 60.0f * std::sqrt(float(count));
    DeterministicUniformFloat uni(-area, area);

    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 1;
    SpatialDatabase w;
    std::vector<SpatialDatabase::InstanceRef> insts;
    for (int i = 0; i < count; ++i)
        insts.push_back(addResidentInstance(w, gen.makeRootPage(unitRegion(4.0f), 8.0f, 0),
                                            float4::point(uni(rng), 0, uni(rng))));

    Outputs o;
    const SelectionParams p{4.0f, 0.0f};
    DeterministicRng fast(kDynamicWorkloadSeed);

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
        const Camera v = orbitView(t, area * 0.4f);
        w.applyUpdates();
        o.query.selectFrontier(w, v, p, o.cut);
        consumeFrontier(o.cut);
    }
}
BENCHMARK(BM_MovingInstances)
    ->Args({10000, 100})->Args({10000, 1000})->Args({50000, 1000})
    ->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Movable/resizable leaf stress: ~250k leaves in one wide tree. The refit-only
// benches time setNodeBounds + flushBounds (the bbox hierarchy + wide-lane
// update) in isolation; the WithCut variant adds selectFrontier on top.
// ---------------------------------------------------------------------------
struct MoverWorld
{
    SpatialDatabase world;
    SpatialDatabase::InstanceRef      inst;     // setNodeBounds is per instance now
    std::vector<NodeHandle> leaves;   // composed from the attach result
    std::vector<float4>     home;     // original leaf centers
    float half = 2000.0f;
};

// unique_ptr: SpatialDatabase (and so MoverWorld) is neither copyable nor movable.
std::unique_ptr<MoverWorld> makeMoverWorld()
{
    auto mw = std::make_unique<MoverWorld>();
    TreeGen gen;
    gen.fanout = 12;   // wide: 12^5 = 248832 leaves, ~271k nodes
    gen.depth = 5;
    Page pg = gen.makeRootPage(unitRegion(mw->half), 4096.0f, 0);
    std::vector<uint32_t> leafIdx;
    for (uint32_t i = 1; i < pg.nodeCount(); ++i)
    {
        if (pg.childCount(i) != 0) continue;
        leafIdx.push_back(i);
        mw->home.push_back(pg.bbox[i].center());
    }
    mw->inst = addResidentInstance(mw->world, std::move(pg), float4::point(0, 0, 0));
    for (uint32_t i : leafIdx) mw->leaves.push_back(nodeAt(mw->inst.rootPage, i));
    return mw;
}

// Common case: objects jitter and resize near where they are. After a brief
// warm-up the grown parent boxes contain the jitter and every refit
// early-outs at the immediate parent.
static void BM_LeafRefit_LocalJitter(benchmark::State& state)
{
    const auto mwp = makeMoverWorld();
    MoverWorld& mw = *mwp;
    const int movers = int(state.range(0));
    DeterministicRng rng(kDynamicWorkloadSeed);
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
            mw.world.setNodeBounds(mw.inst, mw.leaves[i],
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
    const auto mwp = makeMoverWorld();
    MoverWorld& mw = *mwp;
    const int movers = int(state.range(0));
    DeterministicRng rng(kDynamicWorkloadSeed);
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
            mw.world.setNodeBounds(mw.inst, mw.leaves[i],
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
    const auto mwp = makeMoverWorld();
    MoverWorld& mw = *mwp;
    const int distinct = int(state.range(0));
    const int repeats = int(state.range(1));
    DeterministicRng rng(kDynamicWorkloadSeed);
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
                mw.world.setNodeBounds(mw.inst, mw.leaves[i],
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
static void BM_LeafMotion_TeleportWithFrontier(benchmark::State& state)
{
    const auto mwp = makeMoverWorld();
    MoverWorld& mw = *mwp;
    const int movers = int(state.range(0));
    DeterministicRng rng(kDynamicWorkloadSeed);
    Outputs o;
    const SelectionParams p{16.0f, 0.0f};

    float t = 0.0f;
    size_t cursor = 0, frontierTotal = 0, frames = 0;
    for (auto _ : state)
    {
        t += 0.02f;
        for (int m = 0; m < movers; ++m)
        {
            const size_t i = cursor++ % mw.leaves.size();
            const float r = mw.half * 0.98f;
            const float4 c = float4::vec(rng.uniform(-r, r), rng.uniform(-r, r),
                                         rng.uniform(-r, r));
            mw.world.setNodeBounds(mw.inst, mw.leaves[i],
                                   AABB::fromCenterExtent(c, float4::vec(1, 1, 1)));
        }
        const Camera v = orbitView(t, mw.half * 2.0f);
        mw.world.applyUpdates();
        o.query.selectFrontier(mw.world, v, p, o.cut);
        consumeFrontier(o.cut);
        frontierTotal += o.cut.size();
        ++frames;
    }
    state.SetItemsProcessed(int64_t(state.iterations()) * movers);
    state.counters["avg_frontier"] = double(frontierTotal) / double(frames ? frames : 1);
}
BENCHMARK(BM_LeafMotion_TeleportWithFrontier)->Arg(10000)->Arg(100000)
    ->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// In-tree motion: leaf nodes of a deep tree change bounds every frame; the
// lazy refit + wide-lane patching runs inside selectFrontier.
// ---------------------------------------------------------------------------
static void BM_MovingLeafNodes(benchmark::State& state)
{
    TreeGen gen;
    gen.fanout = 8;
    gen.depth = 5;
    SpatialDatabase w;
    SpatialDatabase::InstanceRef inst;
    std::vector<NodeHandle> leaves;
    {
        Page pg = gen.makeRootPage(unitRegion(1000.0f), 4096.0f, 0);
        std::vector<uint32_t> leafIdx;
        for (uint32_t i = 1; i < pg.nodeCount(); ++i)
            if (pg.childCount(i) == 0) leafIdx.push_back(i);
        inst = addResidentInstance(w, std::move(pg), float4::point(0, 0, 0));
        for (uint32_t i : leafIdx) leaves.push_back(nodeAt(inst.rootPage, i));
    }

    DeterministicRng rng(kDynamicWorkloadSeed);
    const int movers = int(state.range(0));

    Outputs o;
    const SelectionParams p{4.0f, 0.0f};

    float t = 0.0f;
    size_t cursor = 0;
    for (auto _ : state)
    {
        t += 0.02f;
        for (int m = 0; m < movers; ++m)
        {
            const NodeHandle h = leaves[cursor++ % leaves.size()];
            w.setNodeBounds(inst, h, AABB::fromCenterExtent(
                                         float4::vec(rng.uniform(-900, 900),
                                                     rng.uniform(-90, 90),
                                                     rng.uniform(-900, 900)),
                                         float4::vec(2, 2, 2)));
        }
        const Camera v = orbitView(t, 2500.0f);
        w.applyUpdates();
        o.query.selectFrontier(w, v, p, o.cut);
        consumeFrontier(o.cut);
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
    SpatialDatabase w;
    Page pg = gen.makeRootPage(unitRegion(1000.0f), 4096.0f, 0);
    const uint32_t nodeCount = pg.nodeCount();
    const auto inst = w.addInstance(std::move(pg), float4::point(0, 0, 0));
    markAllResident(w, inst.rootPage, nodeCount);

    // Handles for every node except the pinned root (index 1).
    std::vector<NodeHandle> nodes;
    for (uint32_t i = 2; i < nodeCount; ++i) nodes.push_back(nodeAt(inst.rootPage, i));

    DeterministicRng rng(2020);
    const int churn = int(state.range(0));

    Outputs o;
    const SelectionParams p{4.0f, 0.0f};

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
        const Camera v = orbitView(t, 2500.0f);
        w.applyUpdates();
        o.query.selectFrontier(w, v, p, o.cut);
        consumeFrontier(o.cut);
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
// bookkeeping has to coexist with the others in the same selectFrontier.
// ---------------------------------------------------------------------------
static void BM_Combined_KitchenSink(benchmark::State& state)
{
    const float half = 50000.0f;
    TreeGen planetGen;
    planetGen.fanout = 4;
    planetGen.depth = 2;

    SpatialDatabase w;
    addResidentInstance(w, planetGen.makeRootPage(unitRegion(half), float(1u << 19), 3),
                        float4::point(0, 0, 0));

    // Props: shallow trees scattered on the ground plane.
    TreeGen propGen;
    propGen.fanout = 4;
    propGen.depth = 1;
    propGen.nextId = 1u << 20;   // avoid id collisions with the planet
    DeterministicRng rng(4242);
    DeterministicUniformFloat uni(-half, half);
    std::vector<SpatialDatabase::InstanceRef> props;
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
    SpatialDatabase::InstanceRef moverInst;
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
        moverInst = addResidentInstance(w, std::move(pg), float4::point(0, 500.0f, 0));
        for (uint32_t i : leafIdx) moverLeaves.push_back(nodeAt(moverInst.rootPage, i));
    }

    Outputs o;
    PageUsageContext usage;
    const SelectionParams p{8.0f, 0.5f};
    DeterministicRng fast(kDynamicWorkloadSeed);

    float x = -half;
    size_t frames = 0, frontierTotal = 0, attaches = 0, collected = 0;
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
            w.setNodeBounds(moverInst, moverLeaves[i],
                            AABB::fromCenterExtent(c, float4::vec(1, 1, 1)));
        }

        const Camera v = makePerspectiveCamera(
            float4::point(x, half * 0.03f, 0), float4::vec(1.0f, -0.2f, 0.1f),
            float4::vec(0, 1, 0), 1.0f, 16.0f / 9.0f, 1080.0f, 0.1f, 1.0e9f);

        w.applyUpdates();
        o.query.selectFrontier(w, v, p, usage, o.cut);

        attaches += attachTopByPriority(w, planetGen, o.cut, 12);
        const size_t currentCount = makeIdealResident(w, o.cut);
        collected += w.collect(usage, 300, 8).detachedPages;

        frontierTotal += currentCount;
        ++frames;
        consumeFrontier(o.cut);
    }
    state.counters["avg_frontier"] = double(frontierTotal) / double(frames ? frames : 1);
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
//   - fully resident, no streaming: cut-only selectFrontier
// Movers are addressed by NodeHandle, composed at setup from the attach
// result plus the authored leaf indices — the production pattern.
// Phase times are reported as counters, in microseconds per frame:
//   move_us  = setNodeBounds submission (handle check + queue push)
//   refit_us = flushBounds (coalesce + bbox hierarchy + wide-lane patching)
//   frontier_us   = TLAS query + per-instance walks + emission
// ---------------------------------------------------------------------------
static void BM_TypicalForest_Breakdown(benchmark::State& state)
{
    UncachedSpatialQuery selection;
    using clock = std::chrono::steady_clock;
    const int count = int(state.range(0));
    // ~20 m spacing: 10k trees live on a ~2x2 km map.
    const float half = 10.0f * std::sqrt(float(count));
    DeterministicRng rng(777);
    DeterministicUniformFloat uni(-half, half);

    TreeGen gen;
    SpatialDatabase w;
    std::vector<NodeHandle> leaves;   // all leaf handles
    std::vector<SpatialDatabase::InstanceRef> leafInst;   // owning instance per leaf
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
        for (uint32_t n : leafIdx)
        {
            leaves.push_back(nodeAt(inst.rootPage, n));
            leafInst.push_back(inst);
        }
    }

    // 10% of all leaves are movers, spread across all trees.
    std::vector<uint32_t> movers;
    for (uint32_t i = 0; i < leaves.size(); i += 10) movers.push_back(i);

    FrontierResult cut;
    const SelectionParams p{4.0f, 1.0f};
    DeterministicRng fast(kDynamicWorkloadSeed);

    double moveNs = 0, refitNs = 0, frontierNs = 0;
    size_t frames = 0, frontierTotal = 0;
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
            w.setNodeBounds(leafInst[i], leaves[i],
                            AABB::fromCenterExtent(c, float4::vec(0.5f, 0.5f, 0.5f)));
        }
        const auto t1 = clock::now();
        w.applyUpdates();   // refit and TLAS work are measured in this phase
        const auto t2 = clock::now();
        const Camera v = makePerspectiveCamera(
            float4::point(x, 1.7f, 0), float4::vec(1.0f, 0.0f, 0.05f),
            float4::vec(0, 1, 0), 1.0f, 16.0f / 9.0f, 1080.0f, 0.1f, 1.0e9f);
        selection.selectFrontier(w, v, p, cut);
        const auto t3 = clock::now();

        moveNs += std::chrono::duration<double, std::nano>(t1 - t0).count();
        refitNs += std::chrono::duration<double, std::nano>(t2 - t1).count();
        frontierNs += std::chrono::duration<double, std::nano>(t3 - t2).count();
        frontierTotal += cut.size();
        ++frames;
        consumeFrontier(cut);
    }
    const double f = double(frames ? frames : 1);
    state.counters["move_us"] = moveNs / f / 1000.0;
    state.counters["refit_us"] = refitNs / f / 1000.0;
    state.counters["frontier_us"] = frontierNs / f / 1000.0;
    state.counters["movers"] = double(movers.size());
    state.counters["leaves"] = double(leaves.size());
    state.counters["avg_frontier"] = double(frontierTotal) / f;
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
//   frontier_us   = selectFrontier (includes any TLAS rebuild the churn forced)
// ---------------------------------------------------------------------------
static void BM_TypicalForest_Churn(benchmark::State& state)
{
    UncachedSpatialQuery selection;
    using clock = std::chrono::steady_clock;
    const int count = int(state.range(0));
    const int churnPct = int(state.range(1));
    const float half = 10.0f * std::sqrt(float(count));
    DeterministicRng rng(777);
    DeterministicUniformFloat uni(-half, half);

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

    SpatialDatabase w;
    struct Tree
    {
        SpatialDatabase::InstanceRef ref;
        PageHandle  page;
        const Proto* proto;
    };
    std::vector<Tree> trees;
    trees.reserve(size_t(count) * 2);
    auto addTree = [&](bool deep, float4 pos)
    {
        const Proto& pr = protos[deep ? 1 : 0];
        const auto inst = addResidentInstance(w, pr.page->clone(), pos);
        trees.push_back({inst, inst.rootPage, &pr});
    };
    for (int i = 0; i < count; ++i)
        addTree((i % 5) == 0, float4::point(uni(rng), 0, uni(rng)));

    FrontierResult cut;
    const SelectionParams p{4.0f, 1.0f};
    DeterministicRng fast(kDynamicWorkloadSeed);
    const size_t churnN = size_t(count) * size_t(churnPct) / 100;

    double churnNs = 0, moveNs = 0, refitNs = 0, frontierNs = 0;
    size_t frames = 0, frontierTotal = 0;
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
                w.setNodeBounds(t.ref, nodeAt(t.page, pr.leafIdx[j]),
                                AABB::fromCenterExtent(c, float4::vec(0.5f, 0.5f, 0.5f)));
            }
        }
        const auto t2 = clock::now();
        w.applyUpdates();
        const auto t3 = clock::now();
        const Camera v = makePerspectiveCamera(
            float4::point(x, 1.7f, 0), float4::vec(1.0f, 0.0f, 0.05f),
            float4::vec(0, 1, 0), 1.0f, 16.0f / 9.0f, 1080.0f, 0.1f, 1.0e9f);
        selection.selectFrontier(w, v, p, cut);
        const auto t4 = clock::now();

        churnNs += std::chrono::duration<double, std::nano>(t1 - t0).count();
        moveNs += std::chrono::duration<double, std::nano>(t2 - t1).count();
        refitNs += std::chrono::duration<double, std::nano>(t3 - t2).count();
        frontierNs += std::chrono::duration<double, std::nano>(t4 - t3).count();
        frontierTotal += cut.size();
        ++frames;
        consumeFrontier(cut);
    }
    const double f = double(frames ? frames : 1);
    state.counters["churn_us"] = churnNs / f / 1000.0;
    state.counters["move_us"] = moveNs / f / 1000.0;
    state.counters["refit_us"] = refitNs / f / 1000.0;
    state.counters["frontier_us"] = frontierNs / f / 1000.0;
    state.counters["churn_pf"] = double(churnN) * 2.0;
    state.counters["avg_frontier"] = double(frontierTotal) / f;
}
BENCHMARK(BM_TypicalForest_Churn)
    ->Args({10000, 5})->Args({50000, 5})
    ->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// PageBuilder cost: composing a page at runtime (spawned structures,
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
        PageBuilder b;
        uint64_t payload = 1;
        const auto root = b.createRoot(payload++, float(1u << (2 * depth)),
                                       AABB::fromCenterExtent(float4::vec(0, 0, 0),
                                                              float4::vec(100, 100, 100)));
        // Iterative BFS expansion; children boxes subdivide the parent's.
        struct Item { PageBuilder::NodeId node; AABB box; uint32_t level; };
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
static void BM_FrontierScaling_OutputSensitivity(benchmark::State& state)
{
    UncachedSpatialQuery selection;
    const auto wp = makeDeepWorld(8, 6);
    SpatialDatabase& w = *wp;
    FrontierResult cut;
    const Camera v = orbitView(0.7f, 2500.0f);
    const SelectionParams p{float(state.range(0)), 0.0f};

    for (auto _ : state)
    {
        w.applyUpdates();
        selection.selectFrontier(w, v, p, cut);
        consumeFrontier(cut);
    }
    state.counters["frontier"] = double(cut.size());
    // Inverted iteration-invariant rate: seconds per frontier entry (SI-suffixed).
    state.counters["per_entry"] = benchmark::Counter(
        double(cut.size()),
        benchmark::Counter::kIsIterationInvariantRate | benchmark::Counter::kInvert);
}
BENCHMARK(BM_FrontierScaling_OutputSensitivity)
    ->Arg(64)->Arg(16)->Arg(4)->Arg(1)
    ->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Camera teleport: worst-case frame latency when every locality assumption
// breaks at once — epoch stamps stale, page memory cold. Steady frames orbit
// smoothly; every 16th frame jumps to a random spot. steady_us vs teleport_us
// is the spike factor.
// ---------------------------------------------------------------------------
static void BM_CameraTeleport_ColdFrame(benchmark::State& state)
{
    UncachedSpatialQuery selection;
    using clock = std::chrono::steady_clock;
    DeterministicRng rng(777);
    const float half = 6000.0f;
    DeterministicUniformFloat uni(-half, half);

    TreeGen propGen;
    propGen.fanout = 4;
    propGen.depth = 1;
    SpatialDatabase w;
    for (int i = 0; i < 20000; ++i)
        addResidentInstance(w, propGen.makeRootPage(unitRegion(6.0f), 24.0f, 0),
                            float4::point(uni(rng), 0, uni(rng)));
    TreeGen deepGen;
    deepGen.fanout = 8;
    deepGen.depth = 5;
    deepGen.nextId = 1u << 24;
    addResidentInstance(w, deepGen.makeRootPage(unitRegion(800.0f), 2048.0f, 0),
                        float4::point(0, 300.0f, 0));

    FrontierResult cut;
    const SelectionParams p{4.0f, 1.0f};
    DeterministicRng fast(kDynamicWorkloadSeed);

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
        const Camera v = orbitView(t, 400.0f, center);

        const auto t0 = clock::now();
        w.applyUpdates();
        selection.selectFrontier(w, v, p, cut);
        const auto t1 = clock::now();
        consumeFrontier(cut);

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
// against one world. The marginal cost of an extra view should be well below
// the main view (shared world data is hot).
// ---------------------------------------------------------------------------
static void BM_MultiView(benchmark::State& state)
{
    UncachedSpatialQuery selection;
    using clock = std::chrono::steady_clock;
    DeterministicRng rng(4321);
    const float half = 4000.0f;
    DeterministicUniformFloat uni(-half, half);

    TreeGen propGen;
    propGen.fanout = 4;
    propGen.depth = 1;
    SpatialDatabase w;
    for (int i = 0; i < 10000; ++i)
        addResidentInstance(w, propGen.makeRootPage(unitRegion(6.0f), 24.0f, 0),
                            float4::point(uni(rng), 0, uni(rng)));
    TreeGen deepGen;
    deepGen.fanout = 8;
    deepGen.depth = 5;
    deepGen.nextId = 1u << 24;
    addResidentInstance(w, deepGen.makeRootPage(unitRegion(600.0f), 2048.0f, 0),
                        float4::point(0, 250.0f, 0));

    FrontierResult cut[4];
    const SelectionParams p{4.0f, 1.0f};

    double mainNs = 0, extraNs = 0;
    size_t frames = 0;
    float t = 0.0f;
    for (auto _ : state)
    {
        t += 0.01f;
        w.applyUpdates();
        const float4 eye = float4::vec(std::cos(t) * 900.0f, 300.0f, std::sin(t) * 900.0f);

        const auto t0 = clock::now();
        selection.selectFrontier(w, makeLookAtCamera(eye, float4::point(0, 0, 0)), p, cut[0]);
        const auto t1 = clock::now();
        // Extra views: same eye, different directions (cascade-style).
        selection.selectFrontier(w, makeLookAtCamera(eye, float4::point(2000, 0, 0)), p, cut[1]);
        selection.selectFrontier(w, makeLookAtCamera(eye, float4::point(-1000, 0, 1500)), p, cut[2]);
        selection.selectFrontier(w, makeLookAtCamera(eye + float4::vec(0, 1200, 0), eye), p, cut[3]);
        const auto t2 = clock::now();

        for (auto& c : cut) consumeFrontier(c);
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
// metric is the worst residual screen error among high-error ideal-side leaves
// that the content graph can still expand, sampled 1/2/4/8/16/32 frames after
// each teleport.
// Near-field detail must land in a few frames; the far field may take
// seconds — that distance-proportional tolerance is exactly what the
// err-priority ordering produces.
//
// Two attach policies at the same total page budget:
//   discovery (arg1 = 0): attach the discovered level only, largest error
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
// owns the page data and the recipes; the SpatialDatabase only sees attachPage).
void predictiveAttachFrame(SpatialDatabase& w, TreeGen& gen, const FrontierResultView& cut,
                           const Camera& view, float threshold, size_t budget)
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

    const auto gather = [&](const auto& entries)
    {
        for (const FrontierEntry& e : entries)
        {
            const UserPayload payload = payloadOf(w, e);
            if (e.overThreshold() && gen.recipes.count(payload) &&
                !w.isAttached(e.nodeHandle))
                heap.push({e.approximateError(threshold), payload, e.nodeHandle});
        }
    };
    gather(cut.shared);
    gather(cut.idealOnly);

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
    SpatialDatabase w;
    addResidentInstance(w, gen.makeRootPage(unitRegion(4000.0f), float(1u << 17), 3),
                        float4::point(0, 0, 0));

    Outputs o;
    PageUsageContext usage;
    const SelectionParams p{4.0f, 0.0f};
    DeterministicRng fast(kDynamicWorkloadSeed);

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

        const Camera v = makeLookAtCamera(eye, eye + float4::vec(1, -0.15f, 0.3f));
        w.applyUpdates();
        o.query.selectFrontier(w, v, p, usage, o.cut);

        // Residual: worst screen error still waiting on an expansion.
        // Clamped: the camera inside a collapsed box saturates err toward
        // 1e33+ (distance 0), and one such frame would swamp the average —
        // 1e6 px already means "the box you are standing in is unexpanded".
        float worst = 0.0f;
        const auto measureResidual = [&](const auto& entries)
        {
            for (const FrontierEntry& e : entries)
            {
                const UserPayload payload = payloadOf(w, e);
                if (e.overThreshold() && gen.recipes.count(payload))
                    worst = std::max(worst, e.approximateError(p.threshold));
            }
        };
        measureResidual(o.cut.shared);
        measureResidual(o.cut.idealOnly);
        worst = worst < 1.0e6f ? worst : 1.0e6f;
        for (int b = 0; b < 6; ++b)
            if (frameInCycle == kSampleAt[b])
            {
                resSum[b] += worst;
                ++resCnt[b];
            }

        if (predictive)
            predictiveAttachFrame(w, gen, o.cut, v, p.threshold, attachBudget);
        else
            attachTopByPriority(w, gen, o.cut, attachBudget);
        makeIdealResident(w, o.cut);
        w.collect(usage, 20000, 30);
        consumeFrontier(o.cut);
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
// output-sensitive TLAS query + visible-instance walks; first_frontier_ms is the
// level-load burst (quality TLAS build + first full query) paid once.
// ---------------------------------------------------------------------------
static void BM_TlasScale(benchmark::State& state)
{
    UncachedSpatialQuery selection;
    using clock = std::chrono::steady_clock;
    const int count = int(state.range(0));
    const float half = 40.0f * std::sqrt(float(count));
    DeterministicRng rng(1717);
    DeterministicUniformFloat uni(-half, half);

    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 1;
    const Page proto = gen.makeRootPage(unitRegion(5.0f), 16.0f, 0);

    SpatialDatabase w;
    for (int i = 0; i < count; ++i)
        addResidentInstance(w, proto.clone(), float4::point(uni(rng), 0, uni(rng)));

    FrontierResult cut;
    const SelectionParams p{4.0f, 1.0f};

    // Level-load burst: the first cut pays the quality TLAS build.
    const auto b0 = clock::now();
    w.applyUpdates();
    selection.selectFrontier(w, orbitView(0.0f, half * 0.25f), p, cut);
    const auto b1 = clock::now();
    const double firstMs = std::chrono::duration<double, std::milli>(b1 - b0).count();

    float t = 0.0f;
    for (auto _ : state)
    {
        t += 0.01f;
        w.applyUpdates();
        selection.selectFrontier(w, orbitView(t, half * 0.25f), p, cut);
        consumeFrontier(cut);
    }
    state.counters["frontier"] = double(cut.size());
    state.counters["first_frontier_ms"] = firstMs;
}
BENCHMARK(BM_TlasScale)
    ->Arg(200000)->Arg(500000)
    ->Unit(benchmark::kMicrosecond);

// Flat-forest control: 100k instances whose entire hierarchy is one
// renderable node. This separates the BVH8 query from the per-visible
// materialization that currently still enters the page traversal.
//
// arg0 = 0: one anonymous flat asset per instance, 1: one shared flat asset,
//        2: one renderable node stored directly in the TLAS
// arg1 = target visible population percentage (25 or 100)
// arg2 = 0: TLAS query only, 1: uncached selectFrontier, 2: cached selectFrontier
static void BM_FlatForest100k(benchmark::State& state)
{
    constexpr uint32_t kInstances = 100000;
    const int representation = int(state.range(0));
    const bool sharedAsset = representation == 1;
    const uint32_t visibleTarget =
        kInstances * uint32_t(state.range(1)) / 100u;
    const int mode = int(state.range(2));

    PageBuilder builder;
    builder.createRoot(
        1, 0.0f,
        AABB::fromCenterExtent(float4::point(0, 0, 0), float4::vec(1, 1, 1)));
    Page prototype = builder.build();

    SpatialDatabase world;
    AssetHandle shared;
    if (sharedAsset) shared = world.registerAsset(prototype.clone());
    const RootNodeDesc tlasRoot{
        1, 0.0f,
        AABB::fromCenterExtent(float4::point(0, 0, 0),
                               float4::vec(1, 1, 1)),
        {}};

    const auto gridPosition = [](uint32_t index, uint32_t count, float zBase)
    {
        const uint32_t columns =
            uint32_t(std::ceil(std::sqrt(double(std::max(1u, count)))));
        const uint32_t row = index / columns;
        const uint32_t column = index - row * columns;
        const float x = (float(column) - 0.5f * float(columns - 1)) * 6.0f;
        return float4::point(x, 0.0f, zBase + float(row) * 6.0f);
    };

    const auto assembly0 = std::chrono::steady_clock::now();
    for (uint32_t i = 0; i < kInstances; ++i)
    {
        float4 position;
        if (i < visibleTarget)
            position = gridPosition(i, visibleTarget, 0.0f);
        else
            // A separate cluster behind the camera lets the quality TLAS
            // reject the invisible 75% high in the tree.
            position = gridPosition(i - visibleTarget,
                                    kInstances - visibleTarget, -10000.0f);
        if (representation == 2)
            world.instantiate(tlasRoot, position);
        else if (sharedAsset)
            world.addInstance(shared, position);
        else
            world.addInstance(prototype.clone(), position);
    }

    const auto assembly1 = std::chrono::steady_clock::now();
    const double assemblyMs =
        std::chrono::duration<double, std::milli>(assembly1 - assembly0).count();

    const Camera camera = makePerspectiveCamera(
        float4::point(0, 100, -2000), float4::vec(0, -0.05f, 1),
        float4::vec(0, 1, 0), 1.2f, 16.0f / 9.0f, 1080.0f, 0.1f, 5000.0f);
    const SelectionParams params{4.0f, 0.0f};

    const auto build0 = std::chrono::steady_clock::now();
    world.applyUpdates();
    const auto build1 = std::chrono::steady_clock::now();
    const double buildMs =
        std::chrono::duration<double, std::milli>(build1 - build0).count();
    SpatialDatabase::TestAccess::TlasQueryScratch tlasScratch;
    SpatialQuery query;
    query.setReuseEnabled(mode == 2);
    FrontierResultView cut;

    size_t visible = 0;
    if (mode == 0)
        visible = SpatialDatabase::TestAccess::queryTlas(world, camera, params.minPix,
                                               tlasScratch);
    else
    {
        // Warm output and SpatialQuery capacities; the benchmark measures steady
        // culling/materialization rather than first-use allocation.
        cut = query.selectFrontier(world, camera, params);
        visible = query.reused() + query.walked();
    }

    for (auto _ : state)
    {
        if (mode == 0)
        {
            visible = SpatialDatabase::TestAccess::queryTlas(world, camera, params.minPix,
                                                   tlasScratch);
            benchmark::DoNotOptimize(tlasScratch.visible.data());
        }
        else
        {
            cut = query.selectFrontier(world, camera, params);
            visible = query.reused() + query.walked();
            consumeFrontier(cut);
        }
    }

    state.SetItemsProcessed(state.iterations() * int64_t(visible));
    state.counters["visible"] = double(visible);
    state.counters["visible%"] = 100.0 * double(visible) / double(kInstances);
    state.counters["frontier"] = mode == 0 ? 0.0 : double(cut.size());
    state.counters["query_MB"] =
        mode == 2 ? double(query.bytes()) / (1024.0 * 1024.0) : 0.0;
    state.counters["build_ms"] = buildMs;
    state.counters["assembly_ms"] = assemblyMs;
    state.counters["pages"] = double(world.attachedPageCount());
    state.counters["tlas_nodes"] =
        double(SpatialDatabase::TestAccess::tlasNodeCount(world));
}
BENCHMARK(BM_FlatForest100k)
    ->Args({1, 25, 0})->Args({1, 25, 1})->Args({1, 25, 2})
    ->Args({1, 100, 0})->Args({1, 100, 1})->Args({1, 100, 2})
    ->Args({2, 25, 0})->Args({2, 25, 1})->Args({2, 25, 2})
    ->Args({2, 100, 0})->Args({2, 100, 1})->Args({2, 100, 2})
    ->Args({0, 25, 0})->Args({0, 25, 1})->Args({0, 25, 2})
    ->Args({0, 100, 0})->Args({0, 100, 1})->Args({0, 100, 2})
    ->ArgNames({"representation", "visible_pct", "mode"})
    ->Unit(benchmark::kMicrosecond);

// Mixed-forest control: one shared flat asset and one shared, fully resident
// 85-node hierarchy. Exactly 25% of both populations are visible, and the
// flat/hierarchical assignment is interleaved so every ratio sees the same
// spatial distribution.
//
// arg0 = percentage of instances using the single-node asset
// arg1 = 0: uncached SpatialQuery, 1: warm cached SpatialQuery
static void BM_MixedForest100k(benchmark::State& state)
{
    constexpr uint32_t kInstances = 100000;
    constexpr uint32_t kVisible = kInstances / 4;
    const uint32_t flatPercent = uint32_t(state.range(0));
    const bool cached = state.range(1) != 0;

    PageBuilder flatBuilder;
    flatBuilder.createRoot(
        1, 0.0f,
        AABB::fromCenterExtent(float4::point(0, 0, 0), float4::vec(1, 1, 1)));

    TreeGen treeGen;
    treeGen.fanout = 4;
    treeGen.depth = 3;
    Page hierarchy = treeGen.makeRootPage(unitRegion(3.0f), 16.0f, 0);
    const uint32_t hierarchyNodes = hierarchy.nodeCount();

    SpatialDatabase world;
    const AssetHandle flatAsset = world.registerAsset(flatBuilder.build());
    const AssetHandle hierarchyAsset = world.registerAsset(std::move(hierarchy));

    const auto gridPosition = [](uint32_t index, uint32_t count, float zBase)
    {
        const uint32_t columns =
            uint32_t(std::ceil(std::sqrt(double(std::max(1u, count)))));
        const uint32_t row = index / columns;
        const uint32_t column = index - row * columns;
        const float x = (float(column) - 0.5f * float(columns - 1)) * 6.0f;
        return float4::point(x, 0.0f, zBase + float(row) * 6.0f);
    };

    uint32_t visibleFlat = 0;
    for (uint32_t i = 0; i < kInstances; ++i)
    {
        const bool visible = i < kVisible;
        const uint32_t clusterIndex = visible ? i : i - kVisible;
        const uint32_t clusterCount = visible ? kVisible : kInstances - kVisible;
        const float zBase = visible ? 0.0f : -10000.0f;
        const float4 position = gridPosition(clusterIndex, clusterCount, zBase);
        const bool flat = (i % 100u) < flatPercent;
        if (visible && flat) ++visibleFlat;
        world.addInstance(flat ? flatAsset : hierarchyAsset, position);
    }
    if (flatPercent != 100)
        markAllResident(world, world.assetRootPage(hierarchyAsset), hierarchyNodes);

    const Camera camera = makePerspectiveCamera(
        float4::point(0, 100, -2000), float4::vec(0, -0.05f, 1),
        float4::vec(0, 1, 0), 1.2f, 16.0f / 9.0f, 1080.0f, 0.1f, 5000.0f);
    const SelectionParams params{4.0f, 0.0f};

    const auto build0 = std::chrono::steady_clock::now();
    world.applyUpdates();
    const auto build1 = std::chrono::steady_clock::now();
    const double buildMs =
        std::chrono::duration<double, std::milli>(build1 - build0).count();

    SpatialQuery query;
    query.setReuseEnabled(cached);
    FrontierResultView cut = query.selectFrontier(world, camera, params);
    size_t visible = query.reused() + query.walked();

    for (auto _ : state)
    {
        cut = query.selectFrontier(world, camera, params);
        visible = query.reused() + query.walked();
        consumeFrontier(cut);
    }

    state.SetItemsProcessed(state.iterations() * int64_t(visible));
    state.counters["visible"] = double(visible);
    state.counters["flat_visible"] = double(visibleFlat);
    state.counters["frontier"] = double(cut.size());
    state.counters["entries/visible"] =
        visible != 0 ? double(cut.size()) / double(visible) : 0.0;
    state.counters["query_MB"] =
        cached ? double(query.bytes()) / (1024.0 * 1024.0) : 0.0;
    state.counters["build_ms"] = buildMs;
}
BENCHMARK(BM_MixedForest100k)
    ->Args({0, 0})->Args({0, 1})
    ->Args({20, 0})->Args({20, 1})
    ->Args({50, 0})->Args({50, 1})
    ->Args({80, 0})->Args({80, 1})
    ->Args({100, 0})->Args({100, 1})
    ->ArgNames({"flat_pct", "cached"})
    ->Unit(benchmark::kMicrosecond);

// Root-decision control: the same 100k-instance, 25%-visible hierarchy as the
// hierarchical arm above. The near camera refines every visible root to four
// entries; the far camera accepts the one renderable BLAS root. This separates
// a TLAS-leaf root decision from the ordinary page walk while keeping output
// size and SpatialQuery behavior explicit.
//
// arg0 = 0: near/refined roots, 1: far/accepted roots
// arg1 = 0: uncached SpatialQuery, 1: warm cached SpatialQuery
static void BM_RootDecisionForest100k(benchmark::State& state)
{
    constexpr uint32_t kInstances = 100000;
    constexpr uint32_t kVisible = kInstances / 4;
    const bool far = state.range(0) != 0;
    const bool cached = state.range(1) != 0;

    TreeGen treeGen;
    treeGen.fanout = 4;
    treeGen.depth = 3;
    Page hierarchy = treeGen.makeRootPage(unitRegion(3.0f), 16.0f, 0);
    const uint32_t hierarchyNodes = hierarchy.nodeCount();

    SpatialDatabase world;
    const AssetHandle hierarchyAsset = world.registerAsset(std::move(hierarchy));
    const auto gridPosition = [](uint32_t index, uint32_t count, float zBase)
    {
        const uint32_t columns =
            uint32_t(std::ceil(std::sqrt(double(std::max(1u, count)))));
        const uint32_t row = index / columns;
        const uint32_t column = index - row * columns;
        const float x = (float(column) - 0.5f * float(columns - 1)) * 6.0f;
        return float4::point(x, 0.0f, zBase + float(row) * 6.0f);
    };

    for (uint32_t i = 0; i < kInstances; ++i)
    {
        const bool visible = i < kVisible;
        const uint32_t clusterIndex = visible ? i : i - kVisible;
        const uint32_t clusterCount = visible ? kVisible : kInstances - kVisible;
        world.addInstance(
            hierarchyAsset,
            gridPosition(clusterIndex, clusterCount, visible ? 0.0f : -10000.0f));
    }
    markAllResident(world, world.assetRootPage(hierarchyAsset), hierarchyNodes);

    const Camera camera = makePerspectiveCamera(
        float4::point(0, 100, far ? -4000.0f : -2000.0f),
        float4::vec(0, -0.05f, 1), float4::vec(0, 1, 0), 1.2f,
        16.0f / 9.0f, 1080.0f, 0.1f, 6000.0f);
    const SelectionParams params{4.0f, 0.0f};

    world.applyUpdates();
    SpatialQuery query;
    query.setReuseEnabled(cached);
    FrontierResultView cut = query.selectFrontier(world, camera, params);
    size_t visible = query.reused() + query.walked();

    for (auto _ : state)
    {
        cut = query.selectFrontier(world, camera, params);
        visible = query.reused() + query.walked();
        consumeFrontier(cut);
    }

    state.SetItemsProcessed(state.iterations() * int64_t(visible));
    state.counters["visible"] = double(visible);
    state.counters["frontier"] = double(cut.size());
    state.counters["entries/visible"] =
        visible != 0 ? double(cut.size()) / double(visible) : 0.0;
    state.counters["query_MB"] =
        cached ? double(query.bytes()) / (1024.0 * 1024.0) : 0.0;
}
BENCHMARK(BM_RootDecisionForest100k)
    ->Args({0, 0})->Args({0, 1})
    ->Args({1, 0})->Args({1, 1})
    ->ArgNames({"far", "cached"})
    ->Unit(benchmark::kMicrosecond);

// End-to-end Morton rebuild cost. After one quality build, move one instance
// between opposite corners on every iteration. A zero escape budget forces the
// next cut to rebuild through the fast Morton tier; the area budget is disabled
// so it cannot promote that rebuild back to the quality tier.
static void BM_TlasMortonRebuild(benchmark::State& state)
{
    UncachedSpatialQuery selection;
    const int count = int(state.range(0));
    const bool stacked = state.range(1) != 0;
    const float half = 40.0f * std::sqrt(float(count));

    SpatialDatabaseConfig config;
    config.tlasEscapeFraction = 0.0f;
    config.tlasAreaDrift = std::numeric_limits<float>::max();
    SpatialDatabase w(config);

    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 1;
    Page proto = gen.makeRootPage(unitRegion(5.0f), 16.0f, 0);
    const uint32_t nodes = proto.nodeCount();
    const AssetHandle asset = w.registerAsset(std::move(proto));

    DeterministicRng rng(1919);
    DeterministicUniformFloat uni(-half, half);
    std::vector<SpatialDatabase::InstanceRef> refs;
    refs.reserve(count);
    for (int i = 0; i < count; ++i)
    {
        const float4 pos = stacked ? float4::point(0, 0, 0)
                                   : float4::point(uni(rng), 0, uni(rng));
        refs.push_back(w.addInstance(asset, pos));
    }
    markAllResident(w, w.assetRootPage(asset), nodes);

    FrontierResult cut;
    const SelectionParams params{4.0f, 1.0f};
    const Camera view = orbitView(0.0f, half * 0.25f);
    w.applyUpdates();
    selection.selectFrontier(w, view, params, cut);   // establish the quality tree

    bool high = false;
    for (auto _ : state)
    {
        high = !high;
        const float corner = high ? half * 0.95f : -half * 0.95f;
        w.moveInstance(refs[0], float4::point(corner, 0, corner));
        w.applyUpdates();
        selection.selectFrontier(w, view, params, cut);
        consumeFrontier(cut);
    }
    state.counters["frontier"] = double(cut.size());
    state.counters["instances"] = double(count);
    state.counters["stacked"] = double(stacked);
}
BENCHMARK(BM_TlasMortonRebuild)
    ->Args({100000, 0})->Args({500000, 0})->Args({100000, 1})
    ->Unit(benchmark::kMillisecond);

// Explicit maintenance point: rebuild the quality TLAS and, with the default
// spatial layout, compact/reorder all dense instance streams. Applications
// call this rarely (loading screen, menu, teleport), so it is intentionally
// separate from the routine Morton rebuild benchmark above.
static void BM_TlasOptimize(benchmark::State& state)
{
    const int count = int(state.range(0));
    const float half = 40.0f * std::sqrt(float(count));

    SpatialDatabase w;
    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 1;
    Page proto = gen.makeRootPage(unitRegion(5.0f), 16.0f, 0);
    const AssetHandle asset = w.registerAsset(std::move(proto));

    DeterministicRng rng(9191);
    DeterministicUniformFloat uni(-half, half);
    for (int i = 0; i < count; ++i)
        w.addInstance(asset, float4::point(uni(rng), 0, uni(rng)));
    w.applyUpdates();

    for (auto _ : state)
    {
        w.optimize();
        benchmark::DoNotOptimize(SpatialDatabase::TestAccess::tlasNodeCount(w));
    }
    state.counters["instances"] = double(count);
}
BENCHMARK(BM_TlasOptimize)->Arg(100000)->Unit(benchmark::kMillisecond);

// Selection after disruptive motion has randomized the relationship between
// dense storage and TLAS traversal. The optimized arm pays SpatialDatabase::optimize()
// outside the timed loop, so the delta measures the locality it recovers.
static void BM_LayoutRecovery100k(benchmark::State& state)
{
    constexpr uint32_t kInstances = 100000;
    constexpr uint32_t kVisible = kInstances / 4;
    const bool optimized = state.range(0) != 0;

    SpatialDatabaseConfig config;
    config.tlasEscapeFraction = 0.0f;
    config.tlasAreaDrift = std::numeric_limits<float>::max();
    SpatialDatabase world(config);

    PageBuilder builder;
    builder.createRoot(
        1, 0.0f,
        AABB::fromCenterExtent(float4::point(0, 0, 0), float4::vec(1, 1, 1)));
    const AssetHandle asset = world.registerAsset(builder.build());

    const auto gridPosition = [](uint32_t index, uint32_t count, float zBase)
    {
        const uint32_t columns =
            uint32_t(std::ceil(std::sqrt(double(std::max(1u, count)))));
        const uint32_t row = index / columns;
        const uint32_t column = index - row * columns;
        const float x = (float(column) - 0.5f * float(columns - 1)) * 6.0f;
        return float4::point(x, 0.0f, zBase + float(row) * 6.0f);
    };

    std::vector<SpatialDatabase::InstanceRef> refs;
    std::vector<float4> positions;
    refs.reserve(kInstances);
    positions.reserve(kInstances);
    for (uint32_t i = 0; i < kInstances; ++i)
    {
        const bool visible = i < kVisible;
        const uint32_t clusterIndex = visible ? i : i - kVisible;
        const uint32_t clusterCount = visible ? kVisible : kInstances - kVisible;
        positions.push_back(gridPosition(
            clusterIndex, clusterCount, visible ? 0.0f : -10000.0f));
        refs.push_back(world.addInstance(asset, positions.back()));
    }
    world.applyUpdates();

    std::vector<uint32_t> permutation(kInstances);
    std::iota(permutation.begin(), permutation.end(), 0u);
    DeterministicRng rng(5252);
    deterministicShuffle(permutation.begin(), permutation.end(), rng);
    for (uint32_t i = 0; i < kInstances; ++i)
        world.moveInstance(refs[i], positions[permutation[i]]);
    world.applyUpdates();
    double optimizeMs = 0.0;
    if (optimized)
    {
        const auto begin = std::chrono::steady_clock::now();
        world.optimize();
        optimizeMs = std::chrono::duration<double, std::milli>(
                         std::chrono::steady_clock::now() - begin).count();
    }

    const Camera camera = makePerspectiveCamera(
        float4::point(0, 100, -2000), float4::vec(0, -0.05f, 1),
        float4::vec(0, 1, 0), 1.2f, 16.0f / 9.0f, 1080.0f, 0.1f, 5000.0f);
    const SelectionParams params{4.0f, 0.0f};
    SpatialQuery query;
    FrontierResultView cut;
    cut = query.selectFrontier(world, camera, params);

    for (auto _ : state)
    {
        cut = query.selectFrontier(world, camera, params);
        consumeFrontier(cut);
    }
    state.SetItemsProcessed(state.iterations() * int64_t(kVisible));
    state.counters["frontier"] = double(cut.size());
    state.counters["optimize_ms"] = optimizeMs;
    state.counters["optimized"] = optimized ? 1.0 : 0.0;
}
BENCHMARK(BM_LayoutRecovery100k)
    ->Args({0})->Args({1})
    ->ArgName("optimized")
    ->Unit(benchmark::kMicrosecond);

// Rebuild after a world has shrunk far below its historical peak. Instance
// slots are recycled rather than erased, so a rebuild that scans capacity pays
// for the peak population forever; the dense live-id list should make this
// scale with the 10% that remain.
static void BM_TlasSparseRebuild(benchmark::State& state)
{
    UncachedSpatialQuery selection;
    using clock = std::chrono::steady_clock;
    const int peak = int(state.range(0));
    const int live = peak / 10;

    SpatialDatabaseConfig config;
    config.tlasCountDrift = 2.0f;   // let edit budget request a Morton rebuild
    SpatialDatabase w(config);

    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 1;
    Page proto = gen.makeRootPage(unitRegion(5.0f), 16.0f, 0);
    const uint32_t nodes = proto.nodeCount();
    const AssetHandle asset = w.registerAsset(std::move(proto));

    const float half = 40.0f * std::sqrt(float(peak));
    DeterministicRng rng(1717);
    DeterministicUniformFloat uni(-half, half);
    std::vector<SpatialDatabase::InstanceRef> refs;
    refs.reserve(peak);
    for (int i = 0; i < peak; ++i)
        refs.push_back(w.addInstance(asset, float4::point(uni(rng), 0, uni(rng))));
    markAllResident(w, w.assetRootPage(asset), nodes);

    FrontierResult cut;
    const SelectionParams params{4.0f, 1.0f};
    const Camera view = orbitView(0.0f, half * 0.25f);
    w.applyUpdates();
    selection.selectFrontier(w, view, params, cut);   // establish the peak-quality tree
    for (int i = live; i < peak; ++i) w.removeInstance(refs[i]);

    const auto t0 = clock::now();
    w.applyUpdates();
    selection.selectFrontier(w, view, params, cut);   // sparse Morton rebuild
    const auto t1 = clock::now();
    const double rebuildMs =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

    for (auto _ : state)
    {
        w.applyUpdates();
        selection.selectFrontier(w, view, params, cut);
        consumeFrontier(cut);
    }
    state.counters["frontier"] = double(cut.size());
    state.counters["live"] = double(live);
    state.counters["peak"] = double(peak);
    state.counters["rebuild_ms"] = rebuildMs;
}
BENCHMARK(BM_TlasSparseRebuild)->Arg(100000)->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Reusable subtree assembly versus a flattened authored city. Both arms have
// one city root, N house proxies, eight fully resident detail leaves per house,
// identical bounds/errors, and the same fully refined frontier. The assembled
// arm stores the house detail bytes once and crosses one transformed mount per
// house; the flat arm bakes every detail node into the city page.
//
// arg0 0 = flattened city page; 1 = shared mounted house subtree
// arg1 = house count (kept below the packed per-node fanout cap)
// arg2 0 = raw traversal; 1 = stationary warm SpatialQuery reuse
// ---------------------------------------------------------------------------
static void BM_SubtreeAssembly_FrontierCost(benchmark::State& state)
{
    const bool assembled = state.range(0) != 0;
    const uint32_t count = uint32_t(state.range(1));
    const bool cached = state.range(2) != 0;
    SpatialQuery selection;
    selection.setReuseEnabled(cached);
    constexpr uint32_t detailCount = 8;
    const uint32_t side = uint32_t(std::ceil(std::sqrt(double(count))));
    const float pitch = 12.0f;
    const auto position = [&](uint32_t i) {
        return float4::point(float(int(i % side) - int(side / 2)) * pitch,
                             0.0f,
                             float(int(i / side) - int(side / 2)) * pitch);
    };
    const auto detailBounds = [](uint32_t i) {
        const float x = (float(i) - 3.5f) * 0.8f;
        return AABB::fromCenterExtent(float4::point(x, 0, 0),
                                      float4::vec(0.3f, 1.0f, 0.3f));
    };

    SpatialDatabase world;
    size_t immutableBytes = 0;
    if (assembled)
    {
        constexpr SubtreeKey houseKey{0xB001};
        constexpr SubtreeKey cityKey{0xC001};
        SubtreeBuilder house(houseKey);
        for (uint32_t d = 0; d < detailCount; ++d)
            house.createNode(house.root(), 1000 + d, 0.0f, detailBounds(d));
        Subtree houseSubtree = house.build();
        const uint32_t houseNodes = houseSubtree.page().nodeCount();
        immutableBytes += houseSubtree.page().byteSize();

        SubtreeBuilder city(cityKey);
        for (uint32_t i = 0; i < count; ++i)
        {
            const float4 pos = position(i);
            const auto proxy = city.createNode(
                city.root(), 10 + i, 16.0f,
                AABB::fromCenterExtent(pos, float4::vec(4, 2, 2)));
            city.setExpansion(proxy, houseKey,
                              SubtreeTransform{pos, 1.0f});
        }
        Subtree citySubtree = city.build();
        const AABB cityBounds = citySubtree.page().bbox[0];
        immutableBytes += citySubtree.page().byteSize() +
                          citySubtree.expansions().size() * sizeof(SubtreeExpansion) +
                          citySubtree.dependencies().size() * sizeof(SubtreeKey);

        const SubtreeHandle houseAsset =
            world.registerSubtree(std::move(houseSubtree));
        const SubtreeHandle cityAsset =
            world.registerSubtree(std::move(citySubtree));
        const auto instance =
            world.instantiate(RootNodeDesc{1, 64.0f, cityBounds, cityKey},
                              float4::point(0, 0, 0));
        const MountHandle cityMount = world.mount(instance.rootNode(), cityAsset);
        markAllResident(world, cityMount, count + 1);
        for (uint32_t i = 0; i < count; ++i)
        {
            const MountHandle mount =
                world.mount(nodeAt(cityMount, i + 1), houseAsset);
            markAllResident(world, mount, houseNodes);
        }
    }
    else
    {
        PageBuilder city;
        const auto root = city.createRoot(1, 64.0f, AABB::empty());
        for (uint32_t i = 0; i < count; ++i)
        {
            const float4 pos = position(i);
            const auto proxy = city.createNode(
                root, 10 + i, 16.0f,
                AABB::fromCenterExtent(pos, float4::vec(4, 2, 2)));
            for (uint32_t d = 0; d < detailCount; ++d)
                city.createNode(proxy, 1000 + d, 0.0f,
                                toWorld(detailBounds(d), pos, 1.0f));
        }
        Page page = city.build();
        const uint32_t nodes = page.nodeCount();
        immutableBytes = page.byteSize();
        const AssetHandle asset = world.registerAsset(std::move(page));
        const auto instance = world.addInstance(asset, float4::point(0, 0, 0));
        markAllResident(world, instance.rootPage, nodes);
    }

    const float span = float(side) * pitch;
    const Camera view = makeLookAtCamera(
        float4::point(0, span * 0.8f, -span * 0.8f),
        float4::point(0, 0, 0));
    FrontierResult cut;
    const SelectionParams params{1.0f, 0.0f};
    if (cached)
    {
        world.applyUpdates();
        selection.selectFrontier(world, view, params, cut);
        consumeFrontier(cut);
    }
    for (auto _ : state)
    {
        world.applyUpdates();
        selection.selectFrontier(world, view, params, cut);
        consumeFrontier(cut);
    }
    state.counters["frontier"] = double(cut.size());
    state.counters["mounts"] = double(world.attachedPageCount());
    state.counters["immutable_KB"] = double(immutableBytes) / 1024.0;
    state.counters["mount_state_KB"] =
        double(world.mountStateBytes()) / 1024.0;
    state.counters["total_KB"] =
        double(immutableBytes + world.mountStateBytes()) / 1024.0;
}
BENCHMARK(BM_SubtreeAssembly_FrontierCost)
    ->Args({0, 32, 0})->Args({1, 32, 0})
    ->Args({0, 128, 0})->Args({1, 128, 0})
    ->Args({0, 400, 0})->Args({1, 400, 0})
    ->Args({0, 32, 1})->Args({1, 32, 1})
    ->Args({0, 128, 1})->Args({1, 128, 1})
    ->Args({0, 400, 1})->Args({1, 400, 1})
    ->ArgNames({"assembled", "houses", "cached"})
    ->Unit(benchmark::kMicrosecond);

static void BM_SubtreeAssembly_ConstructCost(benchmark::State& state)
{
    const bool assembled = state.range(0) != 0;
    const uint32_t count = uint32_t(state.range(1));
    constexpr uint32_t detailCount = 8;
    const uint32_t side = uint32_t(std::ceil(std::sqrt(double(count))));
    const auto position = [&](uint32_t i) {
        return float4::point(float(int(i % side) - int(side / 2)) * 12.0f,
                             0.0f,
                             float(int(i / side) - int(side / 2)) * 12.0f);
    };
    const auto detailBounds = [](uint32_t i) {
        return AABB::fromCenterExtent(
            float4::point((float(i) - 3.5f) * 0.8f, 0, 0),
            float4::vec(0.3f, 1.0f, 0.3f));
    };

    for (auto _ : state)
    {
        SpatialDatabase world;
        if (assembled)
        {
            constexpr SubtreeKey houseKey{0xB101};
            constexpr SubtreeKey cityKey{0xC101};
            SubtreeBuilder house(houseKey);
            for (uint32_t d = 0; d < detailCount; ++d)
                house.createNode(house.root(), 1000 + d, 0.0f,
                                 detailBounds(d));

            SubtreeBuilder city(cityKey);
            for (uint32_t i = 0; i < count; ++i)
            {
                const float4 pos = position(i);
                const auto proxy = city.createNode(
                    city.root(), 10 + i, 16.0f,
                    AABB::fromCenterExtent(pos, float4::vec(4, 2, 2)));
                city.setExpansion(proxy, houseKey,
                                  SubtreeTransform{pos, 1.0f});
            }

            const SubtreeHandle houseAsset =
                world.registerSubtree(house.build());
            Subtree citySubtree = city.build();
            const AABB cityBounds = citySubtree.page().bbox[0];
            const SubtreeHandle cityAsset =
                world.registerSubtree(std::move(citySubtree));
            const auto instance =
                world.instantiate(RootNodeDesc{1, 64.0f, cityBounds, cityKey},
                                  float4::point(0, 0, 0));
            const MountHandle cityMount =
                world.mount(instance.rootNode(), cityAsset);
            for (uint32_t i = 0; i < count; ++i)
                world.mount(nodeAt(cityMount, i + 1), houseAsset);
        }
        else
        {
            PageBuilder city;
            const auto root = city.createRoot(1, 64.0f, AABB::empty());
            for (uint32_t i = 0; i < count; ++i)
            {
                const float4 pos = position(i);
                const auto proxy = city.createNode(
                    root, 10 + i, 16.0f,
                    AABB::fromCenterExtent(pos, float4::vec(4, 2, 2)));
                for (uint32_t d = 0; d < detailCount; ++d)
                    city.createNode(proxy, 1000 + d, 0.0f,
                                    toWorld(detailBounds(d), pos, 1.0f));
            }
            const AssetHandle asset = world.registerAsset(city.build());
            world.addInstance(asset, float4::point(0, 0, 0));
        }
        benchmark::DoNotOptimize(world.attachedPageCount());
    }
}
BENCHMARK(BM_SubtreeAssembly_ConstructCost)
    ->Args({0, 32})->Args({1, 32})
    ->Args({0, 128})->Args({1, 128})
    ->Args({0, 400})->Args({1, 400})
    ->ArgNames({"assembled", "houses"})
    ->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Asset sharing vs. cloning: the SAME scene and the SAME cut, built two ways.
// arg0 0 = every instance owns a private copy of the page (N assets, N
// mounts); arg0 1 = one registered asset instanced N times (1 asset, 1 mount).
// Positions, geometry and camera are identical, so the cut is identical and
// the delta is purely the cost of the data layout: cache residency of the page
// bytes and of the per-mount residency/coverage arrays.
//
// This is the measurement the sharing design was never given: every other
// bench here builds a unique page per instance (or proto.clone()s one), so
// they all model the unshared case.
// ---------------------------------------------------------------------------
static void BM_AssetSharing_FrontierCost(benchmark::State& state)
{
    UncachedSpatialQuery selection;
    const bool shared = state.range(0) != 0;
    const int  count  = int(state.range(1));

    TreeGen gen;
    gen.fanout = 8;
    gen.depth = 3;
    const Page proto = gen.makeRootPage(unitRegion(5.0f), 64.0f, 0);
    const size_t pageBytes = proto.byteSize();

    // Loose grid so every instance is visible at once and fully refines.
    const int   side  = int(std::ceil(std::sqrt(double(count))));
    const float pitch = 14.0f;
    const auto  at = [&](int i) {
        return float4::point(float(i % side - side / 2) * pitch, 0,
                             float(i / side - side / 2) * pitch);
    };

    SpatialDatabase w;
    AssetHandle asset{};
    if (shared)
    {
        asset = w.registerAsset(proto.clone());
        for (int i = 0; i < count; ++i) w.addInstance(asset, at(i));
        markAllResident(w, w.assetRootPage(asset), proto.nodeCount());
    }
    else
    {
        for (int i = 0; i < count; ++i) addResidentInstance(w, proto.clone(), at(i));
    }

    FrontierResult cut;
    const SelectionParams p{4.0f, 0.0f};
    const float span = float(side) * pitch;
    const Camera v = makeLookAtCamera(float4::point(0, span * 0.8f, -span * 0.8f),
                                      float4::point(0, 0, 0));

    for (auto _ : state)
    {
        w.applyUpdates();
        selection.selectFrontier(w, v, p, cut);
        consumeFrontier(cut);
    }
    state.counters["frontier"] = double(cut.size());
    state.counters["mounts"] = double(w.attachedPageCount());
    state.counters["page_KB"] = double(pageBytes) / 1024.0;
    state.counters["pages_MB"] =
        double(pageBytes) * (shared ? 1.0 : double(count)) / (1024.0 * 1024.0);
}
BENCHMARK(BM_AssetSharing_FrontierCost)
    ->Args({0, 100})->Args({1, 100})       // cloned set ~5 MB: fits in L3
    ->Args({0, 300})->Args({1, 300})       // ~16 MB: half of L3
    ->Args({0, 600})->Args({1, 600})       // ~32 MB: at L3
    ->Args({0, 1000})->Args({1, 1000})     // ~52 MB: over L3
    ->Args({0, 4000})->Args({1, 4000})     // ~207 MB: far over L3
    ->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// The deformed CUT path: what an overlay costs the walk, not the refit.
//
// One shared asset instanced N times, then arg1 percent of the instances are
// deformed by rewriting one leaf to the box it ALREADY HAS. applyBoundsChange
// takes the overlay before it inspects anything, so that allocates the copy
// without moving any geometry: the cut is identical at every deform fraction
// and the only variable is where the walk reads bounds from — interleaved in
// the shared WideBlocks, or sparse/dense in a per-instance overlay.
//
// Bounds are 192 of a WideBlock's 256 bytes, so a deformed instance stops
// sharing about two thirds of the hot block and its walk splits into two
// streams. BM_AssetSharing_FrontierCost showed that sharing is worth up to 2.6x;
// this asks how much of that a deformed instance gives back.
// ---------------------------------------------------------------------------
static void BM_DeformedFrontierCost(benchmark::State& state)
{
    UncachedSpatialQuery selection;
    const int count = int(state.range(0));
    const int pct   = int(state.range(1));

    TreeGen gen;
    gen.fanout = 8;
    gen.depth = 3;
    const Page proto = gen.makeRootPage(unitRegion(5.0f), 64.0f, 0);

    // Any leaf of the shared page; node indices are identical for all instances.
    uint32_t leaf = 0;
    for (uint32_t n = 1; n < proto.nodeCount(); ++n)
        if (proto.childCount(n) == 0) { leaf = n; break; }

    const int   side  = int(std::ceil(std::sqrt(double(count))));
    const float pitch = 14.0f;

    SpatialDatabase w;
    const AssetHandle asset = w.registerAsset(proto.clone());
    std::vector<SpatialDatabase::InstanceRef> insts;
    insts.reserve(size_t(count));
    for (int i = 0; i < count; ++i)
        insts.push_back(w.addInstance(
            asset, float4::point(float(i % side - side / 2) * pitch, 0,
                                 float(i / side - side / 2) * pitch)));
    markAllResident(w, w.assetRootPage(asset), proto.nodeCount());

    // Identity rewrite: takes the overlay, moves nothing.
    const int deform = count * pct / 100;
    for (int i = 0; i < deform; ++i)
    {
        const NodeHandle h = nodeAt(insts[i].rootPage, leaf);
        w.setNodeBounds(insts[i], h, w.nodeBounds(insts[i], h));
    }
    w.flushBounds();

    FrontierResult cut;
    const SelectionParams p{4.0f, 0.0f};
    const float span = float(side) * pitch;
    const Camera v = makeLookAtCamera(float4::point(0, span * 0.8f, -span * 0.8f),
                                      float4::point(0, 0, 0));

    for (auto _ : state)
    {
        w.applyUpdates();
        selection.selectFrontier(w, v, p, cut);
        consumeFrontier(cut);
    }
    state.counters["frontier"] = double(cut.size());
    state.counters["overlays"] = double(w.overlayCount());
    state.counters["ov_MB"] = double(w.overlayBytes()) / (1024.0 * 1024.0);
}
BENCHMARK(BM_DeformedFrontierCost)
    ->Args({1000, 0})->Args({1000, 1})->Args({1000, 10})
    ->Args({1000, 50})->Args({1000, 100})
    ->Args({4000, 0})->Args({4000, 10})->Args({4000, 100})
    ->Unit(benchmark::kMicrosecond);

// Submission-locality control for deformation refit. Overlay storage is
// allocated in instance order before timing; the timed stream then visits it
// either in that same order or through one fixed random permutation.
static void BM_DeformationSubmissionOrder(benchmark::State& state)
{
    using clock = std::chrono::steady_clock;
    const int count = int(state.range(0));
    const bool shuffled = state.range(1) != 0;

    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 1;
    const Page proto = gen.makeRootPage(unitRegion(2.0f), 4.0f, 0);
    uint32_t leaf = 1;
    while (leaf < proto.nodeCount() && proto.childCount(leaf) != 0) ++leaf;
    const AABB leafBox = proto.bbox[leaf];

    SpatialDatabase w;
    const AssetHandle asset = w.registerAsset(proto.clone());
    std::vector<SpatialDatabase::InstanceRef> insts;
    insts.reserve(size_t(count));
    for (int i = 0; i < count; ++i)
        insts.push_back(w.addInstance(asset, float4::point(float(i) * 5.0f, 0, 0)));
    w.applyUpdates();

    // Warm overlays in dense instance order so their pool layout is known.
    for (int i = 0; i < count; ++i)
        w.setNodeBounds(insts[i], nodeAt(insts[i].rootPage, leaf), leafBox);
    w.flushBounds();

    std::vector<uint32_t> order(static_cast<size_t>(count), 0u);
    for (uint32_t i = 0; i < uint32_t(count); ++i) order[i] = i;
    if (shuffled)
    {
        DeterministicRng rng(7331);
        deterministicShuffle(order.begin(), order.end(), rng);
    }

    double submitNs = 0.0;
    double flushNs = 0.0;
    for (auto _ : state)
    {
        const auto t0 = clock::now();
        for (const uint32_t i : order)
            w.setNodeBounds(insts[i], nodeAt(insts[i].rootPage, leaf), leafBox);
        const auto t1 = clock::now();
        w.flushBounds();
        const auto t2 = clock::now();
        submitNs += std::chrono::duration<double, std::nano>(t1 - t0).count();
        flushNs += std::chrono::duration<double, std::nano>(t2 - t1).count();
    }
    state.SetItemsProcessed(int64_t(state.iterations()) * count);
    state.counters["submit_us"] = submitNs / (1000.0 * double(state.iterations()));
    state.counters["flush_us"] = flushNs / (1000.0 * double(state.iterations()));
    state.counters["ov_MB"] = double(w.overlayBytes()) / (1024.0 * 1024.0);
}
BENCHMARK(BM_DeformationSubmissionOrder)
    ->Args({20000, 0})->Args({20000, 1})
    ->Args({80000, 0})->Args({80000, 1})
    ->ArgNames({"instances", "shuffled"})
    ->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// SpatialQuery: what temporal reuse is worth on the workload it targets.
//
// A large instanced world seen by a camera that moves continuously and never
// teleports, with a configurable slice of the instances moving every frame.
// arg0 = instances, arg1 = 0 uncached / 1 cached, arg2 = percent of instances
// that move each frame.
//
// The uncached and cached arms do the same work per frame from the caller's
// point of view: same TLAS query, same view, same cut. The difference is that
// the cached arm skips the per-instance walk wherever it can prove the answer
// has not moved, and hands back the entries where they already sit instead of
// rewriting them.
//
// reuse% is the number to read alongside the time: it says how much of the
// population the margin actually covered, and it is the mechanism's own
// report on whether this world suits it.
// ---------------------------------------------------------------------------
static void BM_SpatialQuery_FlyThrough(benchmark::State& state)
{
    const int count = int(state.range(0));
    const bool cached = state.range(1) != 0;
    const int movePct = int(state.range(2));

    // One shared asset, as a real forest or city would be: the cache's records
    // are per instance, so sharing the asset does not share the records.
    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 3;
    Page proto = gen.makeRootPage(unitRegion(3.0f), 2.0f, 0);
    const uint32_t nodes = proto.nodeCount();

    SpatialDatabase w;
    const AssetHandle asset = w.registerAsset(std::move(proto));
    const float half = 12.0f * std::sqrt(float(count));
    DeterministicRng rng(4242);
    DeterministicUniformFloat uni(-half, half);
    std::vector<SpatialDatabase::InstanceRef> insts;
    std::vector<float4>             home;
    insts.reserve(count);
    home.reserve(count);
    for (int i = 0; i < count; ++i)
    {
        home.push_back(float4::point(uni(rng), 0, uni(rng)));
        insts.push_back(w.addInstance(asset, home.back()));
    }
    markAllResident(w, w.assetRootPage(asset), nodes);

    const int movers = count * movePct / 100;

    FrontierResultView cut;
    SpatialQuery cache;
    cache.setReuseEnabled(cached);
    const SelectionParams p{4.0f, 0.0f};
    DeterministicRng fast(kDynamicWorkloadSeed);
    std::vector<float4> movingPositions(static_cast<size_t>(movers));
    SpatialDatabase::MotionGroup movingGroup(
        std::span<const SpatialDatabase::InstanceRef>(insts.data(), size_t(movers)));

    // Fixed iteration count, and the camera's path is a function of the frame
    // index: both arms therefore fly exactly the same route past exactly the
    // same instances, which is what makes their averages comparable. Letting
    // Google Benchmark choose the count would let the faster arm sweep a
    // different part of the map and quietly change what it is averaging.
    using clock = std::chrono::steady_clock;
    double frontierNs = 0, reuse = 0, entries = 0, visible = 0;
    size_t frames = 0;
    for (auto _ : state)
    {
        // Continuous motion, no teleports: the camera drifts a little under a
        // metre per frame and the movers wander about their homes.
        const float t = float(frames % 600) / 600.0f;
        const float z = (t * 2.0f - 1.0f) * half * 0.8f;
        for (int i = 0; i < movers; ++i)
            movingPositions[size_t(i)] =
                home[i] + float4::vec(fast.uniform(-0.4f, 0.4f), 0,
                                      fast.uniform(-0.4f, 0.4f));
        w.moveInstances(movingGroup, movingPositions);
        const Camera v = makePerspectiveCamera(
            float4::point(0, 2.0f, z), float4::vec(0.0f, 0.0f, 1.0f),
            float4::vec(0, 1, 0), 1.0f, 16.0f / 9.0f, 1080.0f, 0.1f, 1.0e9f);
        w.applyUpdates();

        const auto t0 = clock::now();
        cut = cache.selectFrontier(w, v, p);
        const auto t1 = clock::now();
        frontierNs += std::chrono::duration<double, std::nano>(t1 - t0).count();

        if (cached)
        {
            const double n = double(cache.reused() + cache.walked());
            reuse += n > 0 ? 100.0 * double(cache.reused()) / n : 0.0;
            entries += double(cut.size());
            visible += n;
        consumeFrontier(cut);
        }
        else
        {
            entries += double(cut.size());
        consumeFrontier(cut);
        }
        ++frames;
    }
    const double f = double(frames ? frames : 1);
    state.counters["frontier_us"] = frontierNs / f / 1000.0;
    state.counters["reuse%"] = reuse / f;
    state.counters["avg_frontier"] = entries / f;
    state.counters["cache_MB"] = double(cache.bytes()) / (1024.0 * 1024.0);
    // Entries per visible instance: how shallow a per-instance cut really is.
    // At ~1 this is what killed the span-per-instance output -- the descriptor
    // array came out the same size as the data it described.
    state.counters["ent_inst"] = visible > 0 ? entries / visible : 0.0;
}
BENCHMARK(BM_SpatialQuery_FlyThrough)
    ->Args({20000, 0, 0})->Args({20000, 1, 0})
    ->Args({20000, 0, 5})->Args({20000, 1, 5})
    ->Args({20000, 0, 100})->Args({20000, 1, 100})
    ->Args({80000, 0, 5})->Args({80000, 1, 5})
    ->Iterations(600)
    ->Unit(benchmark::kMicrosecond);

// Operation-level breakdown for the representative README workloads. This is
// deliberately SpatialQuery-only: three arms at each scale separate object
// motion from camera motion while keeping the view and 600-frame route fixed.
//
// arg0 = instance count
// arg1 = separately registered assets shared by those instances
// arg2 = instances moved per frame
// arg3 = 0 fixed camera / 1 continuous fly-through
static void BM_SpatialQuery_Breakdown(benchmark::State& state)
{
    using clock = std::chrono::steady_clock;

    const int count = int(state.range(0));
    const int assetCount = int(state.range(1));
    const int movers = int(state.range(2));
    const bool cameraMoves = state.range(3) != 0;

    const auto init0 = clock::now();
    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 3;

    SpatialDatabase w;
    std::vector<AssetHandle> assets;
    assets.reserve(size_t(assetCount));
    uint32_t nodes = 0;
    for (int i = 0; i < assetCount; ++i)
    {
        Page proto = gen.makeRootPage(unitRegion(3.0f), 2.0f, 0);
        if (i == 0) nodes = proto.nodeCount();
        assets.push_back(w.registerAsset(std::move(proto)));
    }
    const float half = 12.0f * std::sqrt(float(count));
    DeterministicRng rng(4242);
    DeterministicUniformFloat uni(-half, half);
    std::vector<SpatialDatabase::InstanceRef> insts;
    std::vector<float4> home;
    insts.reserve(count);
    home.reserve(count);
    for (int i = 0; i < count; ++i)
    {
        home.push_back(float4::point(uni(rng), 0, uni(rng)));
        insts.push_back(w.addInstance(assets[size_t(i) % assets.size()], home.back()));
    }
    for (const AssetHandle asset : assets)
        markAllResident(w, w.assetRootPage(asset), nodes);
    const auto init1 = clock::now();

    const auto viewAt = [&](size_t frame)
    {
        const float t = float(frame % 600) / 600.0f;
        const float z = cameraMoves ? (t * 2.0f - 1.0f) * half * 0.8f : 0.0f;
        return makePerspectiveCamera(
            float4::point(0, 2.0f, z), float4::vec(0.0f, 0.0f, 1.0f),
            float4::vec(0, 1, 0), 1.0f, 16.0f / 9.0f, 1080.0f, 0.1f, 1.0e9f);
    };

    SpatialQuery query;
    FrontierResultView cut;
    const SelectionParams params{4.0f, 0.0f};

    // The first published selection cycle builds the initial TLAS and
    // populates the SpatialQuery. Steady-frame counters start from a warm state.
    const auto cold0 = clock::now();
    w.applyUpdates();
    const Camera coldView = viewAt(0);
    cut = query.selectFrontier(w, coldView, params);
    const auto cold1 = clock::now();

    DeterministicRng fast(kDynamicWorkloadSeed);
    std::vector<float4> movingPositions(static_cast<size_t>(movers));
    SpatialDatabase::MotionGroup movingGroup(
        std::span<const SpatialDatabase::InstanceRef>(insts.data(), size_t(movers)));
    double moveNs = 0.0;
    double maintenanceNs = 0.0;
    double frontierNs = 0.0;
    double totalNs = 0.0;
    double reuse = 0.0;
    double entries = 0.0;
    double visible = 0.0;
    size_t frames = 0;

    for (auto _ : state)
    {
        const Camera view = viewAt(frames);
        const auto frame0 = clock::now();

        const auto move0 = frame0;
        for (int i = 0; i < movers; ++i)
            movingPositions[size_t(i)] =
                home[i] + float4::vec(fast.uniform(-0.4f, 0.4f), 0,
                                      fast.uniform(-0.4f, 0.4f));
        w.moveInstances(movingGroup, movingPositions);
        const auto move1 = clock::now();

        w.applyUpdates();
        const auto maintenance1 = clock::now();

        cut = query.selectFrontier(w, view, params);
        const auto cut1 = clock::now();

        moveNs += std::chrono::duration<double, std::nano>(move1 - move0).count();
        maintenanceNs +=
            std::chrono::duration<double, std::nano>(maintenance1 - move1).count();
        frontierNs += std::chrono::duration<double, std::nano>(cut1 - maintenance1).count();
        totalNs += std::chrono::duration<double, std::nano>(cut1 - frame0).count();

        const double n = double(query.reused() + query.walked());
        reuse += n > 0.0 ? 100.0 * double(query.reused()) / n : 0.0;
        entries += double(cut.size());
        visible += n;
        consumeFrontier(cut);
        ++frames;
    }

    const double f = double(frames ? frames : 1);
    state.counters["init_ms"] =
        std::chrono::duration<double, std::milli>(init1 - init0).count();
    state.counters["cold_ms"] =
        std::chrono::duration<double, std::milli>(cold1 - cold0).count();
    state.counters["move_us"] = moveNs / f / 1000.0;
    state.counters["maint_us"] = maintenanceNs / f / 1000.0;
    state.counters["frontier_us"] = frontierNs / f / 1000.0;
    state.counters["total_us"] = totalNs / f / 1000.0;
    state.counters["reuse%"] = reuse / f;
    state.counters["visible"] = visible / f;
    state.counters["avg_frontier"] = entries / f;
    state.counters["cache_MB"] = double(query.bytes()) / (1024.0 * 1024.0);
}
BENCHMARK(BM_SpatialQuery_Breakdown)
    ->Args({80000, 1, 4000, 1})    // large: moving camera and objects
    ->Args({80000, 1, 4000, 0})    // large: static camera, moving objects
    ->Args({80000, 1, 0, 1})       // large: moving camera, static objects
    ->Args({10000, 700, 1000, 1})  // small: moving camera and objects
    ->Args({10000, 700, 1000, 0})  // small: static camera, moving objects
    ->Args({10000, 700, 0, 1})     // small: moving camera, static objects
    ->ArgNames({"instances", "assets", "movers", "camera_moves"})
    ->Iterations(600)
    ->Unit(benchmark::kMicrosecond);

// Multi-view scaling for the same representative world. Object updates happen
// once per frame; each nearby view owns its own SpatialQuery and output.
// `select_us` is wall time for all six views. The MT arms use six persistent
// worker threads, so the measurement excludes thread creation and directly
// compares serial and concurrent selection of the same published snapshot.
//
// arg0 = percent of instances moved per frame
// arg1 = number of views
// arg2 = 0 serial views / 1 concurrent views on persistent worker threads
static void BM_SpatialQuery_MultiQuery(benchmark::State& state)
{
    using clock = std::chrono::steady_clock;

    constexpr int count = 80000;
    const int movePct = int(state.range(0));
    const int viewCount = int(state.range(1));
    const bool concurrent = state.range(2) != 0;

    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 3;
    Page proto = gen.makeRootPage(unitRegion(3.0f), 2.0f, 0);
    const uint32_t nodes = proto.nodeCount();

    SpatialDatabase w;
    const AssetHandle asset = w.registerAsset(std::move(proto));
    const float half = 12.0f * std::sqrt(float(count));
    DeterministicRng rng(4242);
    DeterministicUniformFloat uni(-half, half);
    std::vector<SpatialDatabase::InstanceRef> insts;
    std::vector<float4> home;
    insts.reserve(count);
    home.reserve(count);
    for (int i = 0; i < count; ++i)
    {
        home.push_back(float4::point(uni(rng), 0, uni(rng)));
        insts.push_back(w.addInstance(asset, home.back()));
    }
    markAllResident(w, w.assetRootPage(asset), nodes);

    const auto viewAt = [&](size_t frame, int viewIndex)
    {
        const float t = float(frame % 600) / 600.0f;
        const float z = (t * 2.0f - 1.0f) * half * 0.8f;
        const float x = (float(viewIndex) - float(viewCount - 1) * 0.5f) * 24.0f;
        return makePerspectiveCamera(
            float4::point(x, 2.0f, z), float4::vec(0.0f, 0.0f, 1.0f),
            float4::vec(0, 1, 0), 1.0f, 16.0f / 9.0f, 1080.0f, 0.1f, 1.0e9f);
    };

    std::vector<SpatialQuery> contexts(static_cast<size_t>(viewCount));
    std::vector<FrontierResultView> cuts(static_cast<size_t>(viewCount));
    const SelectionParams params{4.0f, 0.0f};

    w.applyUpdates();
    for (int v = 0; v < viewCount; ++v)
        cuts[size_t(v)] = contexts[size_t(v)].selectFrontier(w, viewAt(0, v), params);

    std::vector<Camera> currentViews(static_cast<size_t>(viewCount));
    std::atomic<bool> stop{false};
    std::barrier<> start(viewCount + 1);
    std::barrier<> done(viewCount + 1);
    std::vector<std::thread> viewThreads;
    if (concurrent)
    {
        viewThreads.reserve(static_cast<size_t>(viewCount));
        for (int v = 0; v < viewCount; ++v)
            viewThreads.emplace_back([&, v]
            {
                for (;;)
                {
                    start.arrive_and_wait();
                    if (stop.load(std::memory_order_relaxed)) return;
                    cuts[size_t(v)] = contexts[size_t(v)].selectFrontier(
                        static_cast<const SpatialDatabase&>(w), currentViews[size_t(v)], params);
                    done.arrive_and_wait();
                }
            });
    }

    const int movers = count * movePct / 100;
    DeterministicRng fast(kDynamicWorkloadSeed);
    std::vector<float4> movingPositions(static_cast<size_t>(movers));
    SpatialDatabase::MotionGroup movingGroup(
        std::span<const SpatialDatabase::InstanceRef>(insts.data(), size_t(movers)));
    double selectNs = 0.0;
    double entries = 0.0;
    double reuse = 0.0;
    size_t frames = 0;

    for (auto _ : state)
    {
        for (int i = 0; i < movers; ++i)
            movingPositions[size_t(i)] =
                home[i] + float4::vec(fast.uniform(-0.4f, 0.4f), 0,
                                      fast.uniform(-0.4f, 0.4f));
        w.moveInstances(movingGroup, movingPositions);
        w.applyUpdates();

        if (concurrent)
        {
            for (int v = 0; v < viewCount; ++v)
                currentViews[size_t(v)] = viewAt(frames, v);
            const auto select0 = clock::now();
            start.arrive_and_wait();
            done.arrive_and_wait();
            const auto select1 = clock::now();
            selectNs +=
                std::chrono::duration<double, std::nano>(select1 - select0).count();
        }
        else
        {
            const auto select0 = clock::now();
            for (int v = 0; v < viewCount; ++v)
                cuts[size_t(v)] =
                    contexts[size_t(v)].selectFrontier(w, viewAt(frames, v), params);
            const auto select1 = clock::now();
            selectNs +=
                std::chrono::duration<double, std::nano>(select1 - select0).count();
        }

        for (int v = 0; v < viewCount; ++v)
        {
            const double n = double(contexts[size_t(v)].reused() +
                                    contexts[size_t(v)].walked());
            reuse += n > 0.0 ? 100.0 * double(contexts[size_t(v)].reused()) / n : 0.0;
            entries += double(cuts[size_t(v)].size());
            consumeFrontier(cuts[size_t(v)]);
        }
        ++frames;
    }

    if (concurrent)
    {
        stop.store(true, std::memory_order_relaxed);
        start.arrive_and_wait();
        for (std::thread& thread : viewThreads) thread.join();
    }

    const double f = double(frames ? frames : 1);
    const double calls = f * double(viewCount);
    const double selectUs = selectNs / f / 1000.0;
    state.counters["select_us"] = selectUs;
    state.counters["per_view"] = selectUs / double(viewCount);
    state.counters["view_mt"] = concurrent ? 1.0 : 0.0;
    state.counters["reuse%"] = reuse / calls;
    state.counters["avg_frontier"] = entries / calls;
}
BENCHMARK(BM_SpatialQuery_MultiQuery)
    ->Args({0, 6, 0})
    ->Args({0, 6, 1})
    ->Args({5, 6, 0})
    ->Args({5, 6, 1})
    ->ArgNames({"move_pct", "views", "view_mt"})
    ->Iterations(600)
    ->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Projection-scale motion under damping. Every flip point sits at
// eff * k / threshold; the cache stores a conservative per-record slope and
// charges absolute k travel against the recorded margin. This benchmark guards
// the old failure mode where every tiny relaxation step bumped a global epoch.
//
// arg0 = damper half-life in frames, arg1 = frames between zoom steps (0 =
// never). Nothing moves; every instance is otherwise permanently reusable, so
// what is left in the numbers is the zoom and only the zoom.
//
// Read stall_f as frames that reused nothing at all, out of 600. With the slope
// budget it should reflect actual exhausted margins, not the full ~190-frame
// relaxation tail of a halfLife-8 zoom-out.
//
// Do NOT read Time across different zoom periods. Zooming in narrows the
// frustum, so fewer instances are visible and the arm is cheaper for a reason
// that has nothing to do with the cache. Compare within a period, across
// half-lives: /0/120 against /8/120.
// ---------------------------------------------------------------------------
static void BM_SpatialQuery_Zoom(benchmark::State& state)
{
    const float halfLife = float(state.range(0));
    const int   zoomEvery = int(state.range(1));
    const int   count = 20000;

    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 3;
    Page proto = gen.makeRootPage(unitRegion(3.0f), 2.0f, 0);
    const uint32_t nodes = proto.nodeCount();

    SpatialDatabase w;
    const AssetHandle asset = w.registerAsset(std::move(proto));
    const float half = 12.0f * std::sqrt(float(count));
    DeterministicRng rng(4242);
    DeterministicUniformFloat uni(-half, half);
    for (int i = 0; i < count; ++i)
        w.addInstance(asset, float4::point(uni(rng), 0, uni(rng)));
    markAllResident(w, w.assetRootPage(asset), nodes);

    SpatialQuery ctx(halfLife);
    FrontierResult cut;
    const SelectionParams p{4.0f, 0.0f};

    using clock = std::chrono::steady_clock;
    double frontierNs = 0, reuse = 0;
    size_t frames = 0, stalls = 0;
    for (auto _ : state)
    {
        const size_t fi = frames % 600;
        const float t = float(fi) / 600.0f;
        const float z = (t * 2.0f - 1.0f) * half * 0.8f;
        // A discrete zoom step, held until the next one: one keystroke, not a
        // continuous sweep. Undamped, k is then constant between steps.
        const bool zoomedIn = zoomEvery > 0 && ((fi / size_t(zoomEvery)) & 1u) != 0;
        const float fovY = zoomedIn ? 0.5f : 1.0f;
        const Camera v = makePerspectiveCamera(
            float4::point(0, 2.0f, z), float4::vec(0.0f, 0.0f, 1.0f),
            float4::vec(0, 1, 0), fovY, 16.0f / 9.0f, 1080.0f, 0.1f, 1.0e9f);
        w.applyUpdates();

        const auto t0 = clock::now();
        ctx.selectFrontier(w, v, p, cut);
        const auto t1 = clock::now();
        frontierNs += std::chrono::duration<double, std::nano>(t1 - t0).count();

        const double n = double(ctx.reused() + ctx.walked());
        reuse += n > 0 ? 100.0 * double(ctx.reused()) / n : 0.0;
        if (ctx.reused() == 0 && ctx.walked() > 0) ++stalls;
        consumeFrontier(cut);
        ++frames;
    }
    const double f = double(frames ? frames : 1);
    state.counters["frontier_us"] = frontierNs / f / 1000.0;
    state.counters["reuse%"] = reuse / f;
    state.counters["stall_f"] = double(stalls);
}
BENCHMARK(BM_SpatialQuery_Zoom)
    ->Args({0, 0})->Args({0, 120})
    ->Args({8, 0})->Args({8, 120})
    // 300 isolates the direction: with this period the run contains exactly one
    // transition, a zoom IN at frame 300. Zoom in raises k, and kMax_ takes the
    // max, so it arrives in one frame; only zoom OUT relaxes.
    ->Args({0, 300})->Args({8, 300})
    ->Iterations(600)
    ->Unit(benchmark::kMicrosecond);

// Camera-cut/reset latency. A reset forgets reuse and damping state, but the
// the same SpatialQuery immediately starts filling its records again; retaining its
// buffers avoids turning that normal cycle into allocator traffic.
static void BM_SpatialQuery_Reset(benchmark::State& state)
{
    const int count = int(state.range(0));
    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 3;
    Page proto = gen.makeRootPage(unitRegion(3.0f), 2.0f, 0);
    const uint32_t nodes = proto.nodeCount();

    SpatialDatabase w;
    const AssetHandle asset = w.registerAsset(std::move(proto));
    const float half = 12.0f * std::sqrt(float(count));
    DeterministicRng rng(4242);
    DeterministicUniformFloat uni(-half, half);
    for (int i = 0; i < count; ++i)
        w.addInstance(asset, float4::point(uni(rng), 0, uni(rng)));
    markAllResident(w, w.assetRootPage(asset), nodes);

    SpatialQuery ctx(6.0f);
    FrontierResult cut;
    const SelectionParams params{4.0f, 0.0f};
    const Camera view = makePerspectiveCamera(
        float4::point(0, 2.0f, 0), float4::vec(0, 0, 1), float4::vec(0, 1, 0),
        1.0f, 16.0f / 9.0f, 1080.0f, 0.1f, 1.0e9f);

    // Allocate once before timing. The benchmark measures whether reset keeps
    // or discards that reusable storage.
    w.applyUpdates();
    ctx.selectFrontier(w, view, params, cut);
    for (auto _ : state)
    {
        ctx.reset();
        w.applyUpdates();
        ctx.selectFrontier(w, view, params, cut);
        consumeFrontier(cut);
    }
    state.counters["frontier"] = double(cut.size());
    state.counters["ctx_MB"] = double(ctx.bytes()) / (1024.0 * 1024.0);
}
BENCHMARK(BM_SpatialQuery_Reset)->Arg(20000)->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// What does ONE spawn cost?
//
// The stated workload has a small number of instances appearing and
// disappearing every frame against a large static population. Every existing
// churn benchmark moves a PERCENTAGE of the world, which never separates the
// marginal cost of a spawn from the bulk cost of thousands of them.
//
// arg0 = population, arg1 = spawns AND removes per frame, as an absolute count.
// arg1 = 0 is the control: no structural change at all. Read frontier_us across the
// arg1 series -- if one spawn costs what five hundred cost, the cost is not the
// spawn.
//
// arg2 is tlasEditFraction in per-mille, or 0 for the default. It exists to
// check the trade the budget makes: a smaller budget rebuilds more often
// (amortised cost up) but keeps the tree tighter (cut down), and only
// measurement says where the minimum is.
//
// The asset is shared, and resident before the loop, so a newly spawned
// instance needs no streaming: this measures structure, not I/O.
// ---------------------------------------------------------------------------
static void BM_Spawn_MarginalCost(benchmark::State& state)
{
    UncachedSpatialQuery selection;
    using clock = std::chrono::steady_clock;
    const int count = int(state.range(0));
    const int churn = int(state.range(1));
    const int editPerMille = int(state.range(2));   // 0 = library default

    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 3;
    Page proto = gen.makeRootPage(unitRegion(3.0f), 2.0f, 0);
    const uint32_t nodes = proto.nodeCount();

    SpatialDatabaseConfig cfg;
    if (editPerMille > 0) cfg.tlasEditFraction = float(editPerMille) / 1000.0f;
    SpatialDatabase w(cfg);
    const AssetHandle asset = w.registerAsset(std::move(proto));
    const float half = 12.0f * std::sqrt(float(count));
    DeterministicRng rng(4242);
    DeterministicUniformFloat uni(-half, half);
    std::vector<SpatialDatabase::InstanceRef> insts;
    insts.reserve(size_t(count) + size_t(churn) + 1);
    for (int i = 0; i < count; ++i)
        insts.push_back(w.addInstance(asset, float4::point(uni(rng), 0, uni(rng))));
    markAllResident(w, w.assetRootPage(asset), nodes);

    FrontierResult cut;
    const SelectionParams p{6.0f, 0.0f};
    DeterministicRng fast(kDynamicWorkloadSeed);
    double churnNs = 0, frontierNs = 0;
    size_t frames = 0, frontierTotal = 0;

    for (auto _ : state)
    {
        const float t = float(frames);
        const Camera v = makePerspectiveCamera(
            float4::point(std::sin(t * 0.01f) * half * 0.5f, 40.0f,
                          std::cos(t * 0.01f) * half * 0.5f),
            float4::vec(0, -0.3f, 1), float4::vec(0, 1, 0), 1.1f, 16.0f / 9.0f,
            1080.0f, 0.1f, 1.0e9f);

        const auto t0 = clock::now();
        for (int k = 0; k < churn; ++k)
        {
            const size_t i = fast.next() % insts.size();
            w.removeInstance(insts[i]);
            insts[i] = insts.back();
            insts.pop_back();
            insts.push_back(
                w.addInstance(asset, float4::point(fast.uniform(-half, half), 0,
                                                   fast.uniform(-half, half))));
        }
        const auto t1 = clock::now();
        w.applyUpdates();
        selection.selectFrontier(w, v, p, cut);
        const auto t2 = clock::now();

        churnNs += std::chrono::duration<double, std::nano>(t1 - t0).count();
        frontierNs += std::chrono::duration<double, std::nano>(t2 - t1).count();
        frontierTotal += cut.size();
        ++frames;
        consumeFrontier(cut);
    }
    const double f = double(frames ? frames : 1);
    state.counters["churn_us"] = churnNs / f / 1000.0;
    state.counters["frontier_us"] = frontierNs / f / 1000.0;
    state.counters["avg_frontier"] = double(frontierTotal) / f;
}
BENCHMARK(BM_Spawn_MarginalCost)
    ->Args({20000, 0, 0})->Args({20000, 1, 0})->Args({20000, 8, 0})
    ->Args({20000, 500, 0})
    ->Args({80000, 0, 0})->Args({80000, 1, 0})->Args({80000, 8, 0})
    // Edit-budget sweep at the churn rate where degradation is visible.
    ->Args({20000, 8, 5})->Args({20000, 8, 10})->Args({20000, 8, 25})
    ->Args({20000, 8, 50})->Args({20000, 8, 100})->Args({20000, 8, 250})
    ->Iterations(400)
    ->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Adversarial: 10k instances stacked in the same spot. The TLAS cannot
// separate them spatially — every query wades through all of them. This is
// the pathological floor, not a target scenario.
// ---------------------------------------------------------------------------
static void BM_Adversarial_StackedInstances(benchmark::State& state)
{
    UncachedSpatialQuery selection;
    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 1;
    const Page proto = gen.makeRootPage(unitRegion(5.0f), 16.0f, 0);

    SpatialDatabase w;
    DeterministicRng fast(kDynamicWorkloadSeed);
    for (int i = 0; i < 10000; ++i)
        addResidentInstance(w, proto.clone(),
                            float4::point(fast.uniform(-0.01f, 0.01f), 0,
                                          fast.uniform(-0.01f, 0.01f)));

    FrontierResult cut;
    const SelectionParams p{4.0f, 0.0f};
    const Camera v = makeLookAtCamera(float4::point(0, 20, -60), float4::point(0, 0, 0));

    for (auto _ : state)
    {
        w.applyUpdates();
        selection.selectFrontier(w, v, p, cut);
        consumeFrontier(cut);
    }
    state.counters["frontier"] = double(cut.size());
}
BENCHMARK(BM_Adversarial_StackedInstances)->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Adversarial: one node with the maximum 511 children — 64 wide blocks under
// a single parent, no hierarchy to prune with. Exercises the wide-test loop
// at its widest.
// ---------------------------------------------------------------------------
static void BM_Adversarial_WideNode(benchmark::State& state)
{
    UncachedSpatialQuery selection;
    PageBuilder b;
    const auto root = b.createRoot(1, 512.0f, AABB::empty());
    DeterministicRng fast(kDynamicWorkloadSeed);
    for (uint32_t i = 0; i < kMaxChildren; ++i)
    {
        const float4 c = float4::vec(fast.uniform(-200, 200), fast.uniform(-20, 20),
                                     fast.uniform(-200, 200));
        b.createNode(root, 100 + i, 0.0f, AABB::fromCenterExtent(c, float4::vec(1, 1, 1)));
    }
    SpatialDatabase w;
    addResidentInstance(w, b.build(), float4::point(0, 0, 0));

    FrontierResult cut;
    const SelectionParams p{4.0f, 0.0f};

    float t = 0.0f;
    for (auto _ : state)
    {
        t += 0.02f;
        w.applyUpdates();
        selection.selectFrontier(w, orbitView(t, 300.0f), p, cut);
        consumeFrontier(cut);
    }
    state.counters["frontier"] = double(cut.size());
}
BENCHMARK(BM_Adversarial_WideNode)->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Adversarial: a 25-page-deep expansion chain (fanout 1) fully attached, with
// the camera close enough to force the walk through every page boundary for
// a single-entry cut — the maximum page-crossing overhead per output.
// ---------------------------------------------------------------------------
static void BM_Adversarial_DeepPageChain(benchmark::State& state)
{
    UncachedSpatialQuery selection;
    TreeGen gen;
    gen.fanout = 1;
    gen.depth = 1;
    SpatialDatabase w;
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

    FrontierResult cut;
    const SelectionParams p{4.0f, 0.0f};
    const Camera v = makeLookAtCamera(float4::point(0, 3, -8), float4::point(0, 0, 0));

    for (auto _ : state)
    {
        w.applyUpdates();
        selection.selectFrontier(w, v, p, cut);
        consumeFrontier(cut);
    }
    state.counters["pages"] = double(depth + 1);
    state.counters["frontier"] = double(cut.size());
}
BENCHMARK(BM_Adversarial_DeepPageChain)->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Harness self-check: cost of the deterministic generator used by workloads,
// arg = draws per iteration. Measures the generator, not the library.
// ---------------------------------------------------------------------------
static void BM_Harness_DeterministicRng(benchmark::State& state)
{
    DeterministicRng rng(kDynamicWorkloadSeed);
    const int draws = int(state.range(0));
    float acc = 0.0f;
    for (auto _ : state)
    {
        for (int i = 0; i < draws; ++i) acc += rng.uniform(-900.0f, 900.0f);
        benchmark::DoNotOptimize(acc);
    }
}
BENCHMARK(BM_Harness_DeterministicRng)->Arg(30000)->Unit(benchmark::kMicrosecond);
