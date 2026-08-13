#include <benchmark/benchmark.h>

#include <cmath>
#include <memory>
#include <vector>

#include "helpers.h"

using namespace frontier;
using namespace frontiertest;

namespace {

constexpr uint32_t kDetailCount = 8;

float4 housePosition(uint32_t index, uint32_t side)
{
    constexpr float pitch = 12.0f;
    return float4::point(float(int(index % side) - int(side / 2)) * pitch,
                         0.0f,
                         float(int(index / side) - int(side / 2)) * pitch);
}

AABB detailBounds(uint32_t index)
{
    return AABB::fromCenterExtent(
        float4::point((float(index) - 3.5f) * 0.8f, 0, 0),
        float4::vec(0.3f, 1.0f, 0.3f));
}

struct AssemblyScene
{
    SpatialDatabase world;
    size_t immutableBytes = 0;
};

std::unique_ptr<AssemblyScene> buildScene(bool assembled, uint32_t count,
                                          bool makeReady,
                                          uint32_t detailCount = kDetailCount)
{
    auto scene = std::make_unique<AssemblyScene>();
    const uint32_t side =
        uint32_t(std::ceil(std::sqrt(double(count))));
    SubtreeBuilder cityBuilder;
    cityBuilder.reserve(assembled ? count : count * (detailCount + 1));
    SubtreeHandle houseHandle;
    if (assembled)
    {
        SubtreeBuilder houseBuilder;
        houseBuilder.reserve(detailCount);
        for (uint32_t detail = 0; detail < detailCount; ++detail)
            houseBuilder.createNode(
                node(1000 + detail, 0.0f, detailBounds(detail)));
        SubtreeBytes house = houseBuilder.build();
        scene->immutableBytes += house.size();
        houseHandle = scene->world.registerSubtree(std::move(house));

        for (uint32_t i = 0; i < count; ++i)
        {
            const float4 position = housePosition(i, side);
            cityBuilder.createNode(node(
                10 + i, 16.0f,
                AABB::fromCenterExtent(position, float4::vec(4, 2, 2)),
                true));
        }
    }
    else
    {
        for (uint32_t i = 0; i < count; ++i)
        {
            const float4 position = housePosition(i, side);
            const auto proxy = cityBuilder.createNode(node(
                10 + i, 16.0f,
                AABB::fromCenterExtent(position, float4::vec(4, 2, 2))));
            for (uint32_t detail = 0; detail < detailCount; ++detail)
                cityBuilder.createNode(
                    proxy,
                    node(1000 + detail, 0.0f,
                         toWorld(detailBounds(detail), position, 1.0f)));
        }
    }

    SubtreeBytes city = cityBuilder.build();
    const AABB cityBounds = detail::viewSubtreeBytes(city).bounds();
    scene->immutableBytes += city.size();
    const SubtreeHandle cityHandle =
        scene->world.registerSubtree(std::move(city));
    const InstanceHandle instance = scene->world.instantiate(
        node(1, 64.0f, cityBounds, true));
    const SubtreeInstanceHandle cityMount =
        scene->world.mountSubtree(instance.rootNode(), cityHandle);

    if (assembled)
        for (uint32_t i = 0; i < count; ++i)
            scene->world.mountSubtree(
                TestAccess::nodeAt(scene->world, cityMount, i + 1),
                houseHandle,
                Transform{housePosition(i, side), 1.0f});

    if (makeReady)
        TestAccess::markAllNodesReady(scene->world);
    return scene;
}

std::unique_ptr<AssemblyScene> buildMixedReadinessScene(uint32_t count)
{
    auto scene = std::make_unique<AssemblyScene>();
    const uint32_t side =
        uint32_t(std::ceil(std::sqrt(double(count))));

    // A ready coarse house C refines into ready F plus unavailable ideal node
    // G. G's descendants K/M/N/O form a complete ready cut. Ancestor mode
    // therefore emits C; descendant mode emits F/K/M/N/O.
    SubtreeBuilder houseBuilder;
    const auto c = houseBuilder.createNode(node(1000, 64.0f, box(4.0f)));
    houseBuilder.createNode(c, node(1001, 0.0f, box(4.0f)));
    const auto g = houseBuilder.createNode(c, node(1002, 0.0f, box(4.0f)));
    houseBuilder.createNode(g, node(1003, 0.0f, box(4.0f)));
    const auto l = houseBuilder.createNode(g, node(1004, 0.0f, box(4.0f)));
    houseBuilder.createNode(l, node(1005, 0.0f, box(4.0f)));
    houseBuilder.createNode(l, node(1006, 0.0f, box(4.0f)));
    houseBuilder.createNode(l, node(1007, 0.0f, box(4.0f)));
    SubtreeBytes houseBytes = houseBuilder.build();
    scene->immutableBytes += houseBytes.size();
    const SubtreeHandle house =
        scene->world.registerSubtree(std::move(houseBytes));

    SubtreeBuilder cityBuilder;
    cityBuilder.reserve(count);
    for (uint32_t i = 0; i < count; ++i)
    {
        const float4 position = housePosition(i, side);
        cityBuilder.createNode(node(
            10 + i, 16.0f,
            AABB::fromCenterExtent(position, float4::vec(4, 4, 4)), true));
    }
    SubtreeBytes city = cityBuilder.build();
    const AABB cityBounds = detail::viewSubtreeBytes(city).bounds();
    scene->immutableBytes += city.size();
    const SubtreeHandle cityHandle =
        scene->world.registerSubtree(std::move(city));
    const InstanceHandle instance = scene->world.instantiate(
        node(1, 64.0f, cityBounds, true));
    const SubtreeInstanceHandle cityMount =
        scene->world.mountSubtree(instance.rootNode(), cityHandle);

    for (uint32_t i = 0; i < count; ++i)
        scene->world.mountSubtree(
            TestAccess::nodeAt(scene->world, cityMount, i + 1),
            house,
            Transform{housePosition(i, side), 1.0f});

    TestAccess::markAllNodesReady(scene->world);
    scene->world.markNodeUnavailable(handleOf(scene->world, 1002));
    scene->world.markNodeUnavailable(handleOf(scene->world, 1004));
    return scene;
}

SubtreeBytes makeRegistrationBytes(uint32_t count)
{
    SubtreeBuilder builder;
    builder.reserve(count);
    std::vector<SubtreeBuilder::NodeId> ids;
    ids.reserve(count);
    for (uint32_t i = 0; i < count; ++i)
    {
        const NodeDesc desc = node(1000 + i, 0.0f, box(1.0f));
        ids.push_back(i == 0
                          ? builder.createNode(desc)
                          : builder.createNode(ids[(i - 1) / 8], desc));
    }
    return builder.build();
}

std::unique_ptr<AssemblyScene> buildFlatReadinessFanoutScene(uint32_t count)
{
    auto scene = std::make_unique<AssemblyScene>();
    const SubtreeHandle detail = scene->world.registerSubtree(
        makeLeafSubtree(1000));
    const uint32_t side =
        uint32_t(std::ceil(std::sqrt(double(count))));
    for (uint32_t i = 0; i < count; ++i)
    {
        InstanceDesc desc;
        desc.pos = housePosition(i, side);
        const InstanceHandle root = scene->world.instantiate(
            node(i + 1, 16.0f, box(1.0f), true), desc);
        scene->world.mountSubtree(root.rootNode(), detail);
    }
    return scene;
}

void consume(const FrontierResultView& cut)
{
    benchmark::DoNotOptimize(cut.shared.data());
    benchmark::DoNotOptimize(cut.currentOnly.data());
    benchmark::DoNotOptimize(cut.idealOnly.data());
    benchmark::DoNotOptimize(cut.size());
}

} // namespace

static void BM_SubtreeAssembly_FrontierCost(benchmark::State& state)
{
    const bool assembled = state.range(0) != 0;
    const uint32_t count = uint32_t(state.range(1));
    const bool cached = state.range(2) != 0;
    auto scene = buildScene(assembled, count, true);
    const uint32_t side =
        uint32_t(std::ceil(std::sqrt(double(count))));
    const float span = float(side) * 12.0f;
    const Camera view = makeLookAtCamera(
        float4::point(0, span * 0.8f, -span * 0.8f),
        float4::point(0, 0, 0));
    const SelectionParams params{1.0f, 0.0f};
    SpatialQuery query;
    query.setReuseEnabled(cached);
    scene->world.applyUpdates();
    if (cached) consume(query.selectFrontier(scene->world, view, params));

    FrontierResultView cut;
    for (auto _ : state)
    {
        scene->world.applyUpdates();
        cut = query.selectFrontier(scene->world, view, params);
        consume(cut);
    }
    state.counters["frontier"] = double(cut.size());
    state.counters["mounts"] =
        double(scene->world.mountedSubtreeCount());
    state.counters["immutable_KB"] =
        double(scene->immutableBytes) / 1024.0;
    state.counters["mount_state_KB"] =
        double(scene->world.subtreeInstanceStateBytes()) / 1024.0;
    state.counters["total_KB"] =
        double(scene->immutableBytes +
               scene->world.subtreeInstanceStateBytes()) /
        1024.0;
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

// Same assembled city and shared-house workflow with controlled logical
// fanout. This is the direct BVH4/BVH8 occupancy experiment: each house visits
// exactly `children` renderable roots in its mounted definition.
static void BM_BranchWidthOccupancy(benchmark::State& state)
{
    constexpr uint32_t count = 400;
    const uint32_t children = uint32_t(state.range(0));
    auto scene = buildScene(true, count, true, children);
    const uint32_t side =
        uint32_t(std::ceil(std::sqrt(double(count))));
    const float span = float(side) * 12.0f;
    const Camera view = makeLookAtCamera(
        float4::point(0, span * 0.8f, -span * 0.8f),
        float4::point(0, 0, 0));
    const SelectionParams params{1.0f, 0.0f};
    SpatialQuery query;
    query.setReuseEnabled(false);
    scene->world.applyUpdates();

    FrontierResultView cut;
    for (auto _ : state)
    {
        cut = query.selectFrontier(scene->world, view, params);
        consume(cut);
    }
    state.counters["branching_factor"] = double(kWide);
    state.counters["children"] = double(children);
    state.counters["frontier"] = double(cut.size());
    state.counters["immutable_KB"] =
        double(scene->immutableBytes) / 1024.0;
}

BENCHMARK(BM_BranchWidthOccupancy)
    ->Arg(2)->Arg(4)->Arg(6)->Arg(8)
    ->ArgName("children")
    ->Unit(benchmark::kMicrosecond);

static void BM_SubtreeAssembly_ConstructCost(benchmark::State& state)
{
    const bool assembled = state.range(0) != 0;
    const uint32_t count = uint32_t(state.range(1));
    for (auto _ : state)
    {
        auto scene = buildScene(assembled, count, false);
        benchmark::DoNotOptimize(scene->world.mountedSubtreeCount());
    }
}

BENCHMARK(BM_SubtreeAssembly_ConstructCost)
    ->Args({0, 32})->Args({1, 32})
    ->Args({0, 128})->Args({1, 128})
    ->Args({0, 400})->Args({1, 400})
    ->ArgNames({"assembled", "houses"})
    ->Unit(benchmark::kMicrosecond);

static void BM_SharedNodeReadinessFanout(benchmark::State& state)
{
    const uint32_t count = uint32_t(state.range(0));
    auto scene = buildScene(true, count, false);
    const NodeHandle sharedNode = handleOf(scene->world, 1000);
    bool ready = false;
    for (auto _ : state)
    {
        if (ready)
            scene->world.markNodeUnavailable(sharedNode);
        else
            scene->world.markNodeReady(sharedNode);
        ready = !ready;
        benchmark::ClobberMemory();
    }
    state.counters["affected_mounts"] = double(count);
}

BENCHMARK(BM_SharedNodeReadinessFanout)
    ->Arg(32)->Arg(128)->Arg(400)
    ->Unit(benchmark::kMicrosecond);

static void BM_SharedNodeReadinessLargeFanout(benchmark::State& state)
{
    const uint32_t count = uint32_t(state.range(0));
    auto scene = buildFlatReadinessFanoutScene(count);
    const NodeHandle sharedNode = handleOf(scene->world, 1000);
    bool ready = false;
    for (auto _ : state)
    {
        if (ready)
            scene->world.markNodeUnavailable(sharedNode);
        else
            scene->world.markNodeReady(sharedNode);
        ready = !ready;
        benchmark::ClobberMemory();
    }
    state.counters["affected_mounts"] = double(count);
}

BENCHMARK(BM_SharedNodeReadinessLargeFanout)
    ->Arg(1024)->Arg(10000)
    ->Unit(benchmark::kMicrosecond);

static void BM_MountUnmountLifecycle(benchmark::State& state)
{
    SpatialDatabase world;

    SubtreeBuilder childBuilder;
    for (uint32_t detail = 0; detail < kDetailCount; ++detail)
        childBuilder.createNode(
            node(1000 + detail, 0.0f, detailBounds(detail)));
    const SubtreeHandle child =
        world.registerSubtree(childBuilder.build());

    SubtreeBuilder ownerBuilder;
    ownerBuilder.createNode(node(
        10, 16.0f,
        AABB::fromCenterExtent(float4::point(0, 0, 0),
                               float4::vec(4, 2, 2)),
        true));
    SubtreeBytes ownerBytes = ownerBuilder.build();
    const AABB ownerBounds = detail::viewSubtreeBytes(ownerBytes).bounds();
    const SubtreeHandle owner =
        world.registerSubtree(std::move(ownerBytes));
    const InstanceHandle instance =
        world.instantiate(node(1, 64.0f, ownerBounds, true));
    const SubtreeInstanceHandle ownerMount =
        world.mountSubtree(instance.rootNode(), owner);
    const NodeHandle parent = TestAccess::nodeAt(world, ownerMount, 1);

    for (auto _ : state)
    {
        const SubtreeInstanceHandle mounted =
            world.mountSubtree(parent, child);
        benchmark::DoNotOptimize(mounted);
        world.unmountSubtree(mounted);
    }
}

BENCHMARK(BM_MountUnmountLifecycle)->Unit(benchmark::kNanosecond);

static void BM_MountUsageConsumption(benchmark::State& state)
{
    const uint32_t count = uint32_t(state.range(0));
    auto scene = buildScene(true, count, true);
    const uint32_t side =
        uint32_t(std::ceil(std::sqrt(double(count))));
    const float span = float(side) * 12.0f;
    const Camera view = makeLookAtCamera(
        float4::point(0, span * 0.8f, -span * 0.8f),
        float4::point(0, 0, 0));
    const SelectionParams params{1.0f, 0.0f};
    SpatialQuery query;
    query.setReuseEnabled(false);
    query.setMountUsageEnabled(true);

    for (auto _ : state)
    {
        state.PauseTiming();
        scene->world.applyUpdates();
        consume(query.selectFrontier(scene->world, view, params));
        state.ResumeTiming();
        benchmark::DoNotOptimize(scene->world.collect(
            query, scene->world.mountedSubtreeCount(), 0));
    }
    state.counters["mounts"] = double(scene->world.mountedSubtreeCount());
}

BENCHMARK(BM_MountUsageConsumption)
    ->Arg(128)->Arg(400)
    ->Unit(benchmark::kMicrosecond);

static void BM_MotionGroupSteady(benchmark::State& state)
{
    const uint32_t count = uint32_t(state.range(0));
    const bool unchanged = state.range(1) != 0;
    SpatialDatabase world;
    std::vector<InstanceHandle> handles;
    std::vector<float4> positions;
    handles.reserve(count);
    positions.reserve(count);
    for (uint32_t i = 0; i < count; ++i)
    {
        const float4 position = float4::point(float(i) * 2.0f, 0, 0);
        InstanceDesc desc;
        desc.pos = position;
        handles.push_back(world.instantiate(
            node(1000 + i, 0.0f, box(0.5f)), desc));
        positions.push_back(position);
    }
    SpatialDatabase::MotionGroup group(handles);

    bool raised = false;
    for (auto _ : state)
    {
        if (!unchanged) raised = !raised;
        const float y = raised ? 0.25f : 0.0f;
        for (float4& position : positions) position.y = y;
        world.moveInstances(group, positions);
        benchmark::ClobberMemory();
    }
    state.counters["instances"] = double(count);
}

BENCHMARK(BM_MotionGroupSteady)
    ->Args({128, 0})->Args({400, 0})
    ->Args({128, 1})->Args({400, 1})
    ->ArgNames({"instances", "unchanged"})
    ->Unit(benchmark::kMicrosecond);

static void BM_MixedReadinessFrontier(benchmark::State& state)
{
    const uint32_t count = uint32_t(state.range(0));
    const bool preferDescendants = state.range(1) != 0;
    auto scene = buildMixedReadinessScene(count);
    const uint32_t side =
        uint32_t(std::ceil(std::sqrt(double(count))));
    const float span = float(side) * 12.0f;
    const Camera view = makeLookAtCamera(
        float4::point(0, span * 0.8f, -span * 0.8f),
        float4::point(0, 0, 0));
    SelectionParams params{1.0f, 0.0f};
    params.currentCutPolicy =
        preferDescendants
            ? CurrentCutPolicy::PreferReadyDescendants
            : CurrentCutPolicy::PreferReadyAncestors;
    SpatialQuery query;
    query.setReuseEnabled(false);
    scene->world.applyUpdates();

    FrontierResultView cut;
    for (auto _ : state)
    {
        scene->world.applyUpdates();
        cut = query.selectFrontier(scene->world, view, params);
        consume(cut);
    }
    state.counters["stored_entries"] = double(cut.size());
    state.counters["current"] = double(cut.currentSize());
    state.counters["ideal"] = double(cut.idealSize());
    state.counters["query_bytes"] = double(query.bytes());
}

BENCHMARK(BM_MixedReadinessFrontier)
    ->Args({128, 0})->Args({128, 1})
    ->Args({400, 0})->Args({400, 1})
    ->ArgNames({"houses", "ready_descendants"})
    ->Unit(benchmark::kMicrosecond);

static void BM_SubtreeBuilder_ConstructCost(benchmark::State& state)
{
    const bool assembled = state.range(0) != 0;
    const uint32_t count = uint32_t(state.range(1));
    const uint32_t side =
        uint32_t(std::ceil(std::sqrt(double(count))));
    for (auto _ : state)
    {
        SubtreeBuilder builder;
        builder.reserve(assembled ? count : count * (kDetailCount + 1));
        for (uint32_t i = 0; i < count; ++i)
        {
            const float4 position = housePosition(i, side);
            const auto proxy = builder.createNode(node(
                10 + i, 16.0f,
                AABB::fromCenterExtent(position, float4::vec(4, 2, 2)),
                assembled));
            if (!assembled)
                for (uint32_t detail = 0; detail < kDetailCount; ++detail)
                    builder.createNode(
                        proxy,
                        node(1000 + detail, 0.0f,
                             toWorld(detailBounds(detail), position, 1.0f)));
        }
        SubtreeBytes bytes = builder.build();
        benchmark::DoNotOptimize(bytes.data());
        benchmark::DoNotOptimize(bytes.size());
    }
}

BENCHMARK(BM_SubtreeBuilder_ConstructCost)
    ->Args({0, 400})->Args({1, 400})
    ->ArgNames({"assembled", "houses"})
    ->Unit(benchmark::kMicrosecond);

static void BM_SubtreeRegistration(benchmark::State& state)
{
    const uint32_t count = uint32_t(state.range(0));
    const SubtreeBytes source = makeRegistrationBytes(count);
    SpatialDatabase world;
    for (auto _ : state)
    {
        state.PauseTiming();
        SubtreeBytes bytes = source;
        state.ResumeTiming();
        const SubtreeHandle handle =
            world.registerSubtree(std::move(bytes));
        benchmark::DoNotOptimize(handle);
        state.PauseTiming();
        world.releaseSubtree(handle);
        state.ResumeTiming();
    }
    state.SetBytesProcessed(
        int64_t(state.iterations()) * int64_t(source.size()));
    state.counters["nodes"] = double(count);
}

BENCHMARK(BM_SubtreeRegistration)
    ->Arg(128)->Arg(4096)
    ->Unit(benchmark::kMicrosecond);

static void BM_FlatTlasSelectionScale(benchmark::State& state)
{
    const uint32_t count = uint32_t(state.range(0));
    const bool cached = state.range(1) != 0;
    SpatialDatabase world;
    const uint32_t side =
        uint32_t(std::ceil(std::sqrt(double(count))));
    for (uint32_t i = 0; i < count; ++i)
    {
        InstanceDesc desc;
        desc.pos = float4::point(
            float(int(i % side) - int(side / 2)) * 3.0f,
            float(int(i / side) - int(side / 2)) * 3.0f, 0.0f);
        world.instantiate(node(1000 + i, 0.0f, box(0.5f)), desc);
    }
    world.applyUpdates();
    const Camera camera = cameraAt(-2000.0f);
    SpatialQuery query;
    query.setReuseEnabled(cached);
    if (cached) consume(query.selectFrontier(world, camera, {}));

    FrontierResultView result;
    for (auto _ : state)
    {
        result = query.selectFrontier(world, camera, {});
        consume(result);
    }
    state.counters["entries"] = double(result.size());
    state.counters["instances"] = double(count);
    state.counters["branching_factor"] = double(kWide);
    state.counters["tlas_nodes"] =
        double(TestAccess::tlasNodeCount(world));
    state.counters["tlas_KB"] =
        double(TestAccess::tlasNodeCount(world) *
               TestAccess::tlasNodeBytes()) /
        1024.0;
}

BENCHMARK(BM_FlatTlasSelectionScale)
    ->Args({1000, 0})->Args({1000, 1})
    ->Args({10000, 0})->Args({10000, 1})
    ->ArgNames({"instances", "cached"})
    ->Unit(benchmark::kMicrosecond);

static void BM_FlatInstanceLifecycle(benchmark::State& state)
{
    constexpr uint32_t population = 1024;
    SpatialDatabase world;
    for (uint32_t i = 0; i < population; ++i)
    {
        InstanceDesc desc;
        desc.pos = float4::point(float(i % 32) * 2.0f,
                                 float(i / 32) * 2.0f, 0.0f);
        world.instantiate(node(i + 1, 0.0f, box(0.5f)), desc);
    }
    world.applyUpdates();

    for (auto _ : state)
    {
        const InstanceHandle transient = world.instantiate(
            node(50000, 0.0f, box(0.5f)),
            InstanceDesc{.pos = float4::point(100, 100, 0)});
        world.removeInstance(transient);
        world.applyUpdates();
    }
    state.counters["steady_population"] = population;
}

BENCHMARK(BM_FlatInstanceLifecycle)->Unit(benchmark::kMicrosecond);

static void BM_BoundsOverrideBatch(benchmark::State& state)
{
    constexpr uint32_t nodeCount = detail::kMaxChildren;
    const uint32_t changedCount = uint32_t(state.range(0));
    SpatialDatabase world;
    SubtreeBuilder builder;
    std::vector<AABB> firstBounds;
    std::vector<AABB> secondBounds;
    firstBounds.reserve(nodeCount);
    secondBounds.reserve(nodeCount);
    for (uint32_t i = 0; i < nodeCount; ++i)
    {
        const float4 center = float4::point(
            float(int(i % 16) - 8) * 3.0f,
            float(int(i / 16) - 8) * 3.0f, 0.0f);
        const AABB authored = box(1.0f, center);
        builder.createNode(node(1000 + i, 0.0f, authored));
        firstBounds.push_back(box(0.75f, center + float4::vec(0.1f, 0, 0)));
        secondBounds.push_back(box(0.75f, center - float4::vec(0.1f, 0, 0)));
    }
    SubtreeBytes bytes = builder.build();
    const AABB definitionBounds = detail::viewSubtreeBytes(bytes).bounds();
    const SubtreeHandle definition =
        world.registerSubtree(std::move(bytes));
    const InstanceHandle instance = world.instantiate(
        node(1, 16.0f, definitionBounds, true));
    const SubtreeInstanceHandle placement =
        world.mountSubtree(instance.rootNode(), definition);
    std::vector<NodeHandle> handles;
    handles.reserve(nodeCount);
    for (uint32_t i = 0; i < nodeCount; ++i)
        handles.push_back(TestAccess::nodeAt(world, placement, i + 1));

    bool alternate = false;
    for (auto _ : state)
    {
        const std::vector<AABB>& bounds =
            alternate ? firstBounds : secondBounds;
        for (uint32_t i = 0; i < changedCount; ++i)
            world.setNodeBounds(instance, handles[i], bounds[i]);
        world.flushBounds();
        alternate = !alternate;
        benchmark::ClobberMemory();
    }
    state.counters["changed_nodes"] = double(changedCount);
    state.counters["overlays"] = double(world.overlayCount());
}

BENCHMARK(BM_BoundsOverrideBatch)
    ->Arg(1)->Arg(32)->Arg(64)->Arg(256)
    ->Unit(benchmark::kMicrosecond);
