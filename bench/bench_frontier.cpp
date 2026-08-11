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
                                          bool makeResident)
{
    auto scene = std::make_unique<AssemblyScene>();
    const uint32_t side =
        uint32_t(std::ceil(std::sqrt(double(count))));
    SubtreeBuilder cityBuilder;
    cityBuilder.reserve(assembled ? count : count * (kDetailCount + 1));
    SubtreeHandle houseHandle;
    if (assembled)
    {
        SubtreeBuilder houseBuilder;
        houseBuilder.reserve(kDetailCount);
        for (uint32_t detail = 0; detail < kDetailCount; ++detail)
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
            for (uint32_t detail = 0; detail < kDetailCount; ++detail)
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

    if (makeResident)
        TestAccess::markAllPayloadsResident(scene->world);
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
