#include <benchmark/benchmark.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>

#include "helpers.h"

using namespace frontier;
using namespace frontiertest;

namespace {

struct PayloadLodScene
{
    SpatialDatabase world;
    size_t immutableBytes = 0;
    uint32_t storedNodes = 0;
};

std::unique_ptr<PayloadLodScene> buildPayloadLodScene(bool multiPayload,
                                                      uint32_t count)
{
    constexpr std::array<float, kMaxNodePayloads> errors{
        2048.0f, 1024.0f, 512.0f, 256.0f,
        128.0f, 64.0f, 32.0f, 0.0f};
    constexpr uint32_t groupSize = 256;
    auto scene = std::make_unique<PayloadLodScene>();
    SubtreeBuilder builder;
    const uint32_t side =
        uint32_t(std::ceil(std::sqrt(double(count))));
    const uint32_t groupCount = (count + groupSize - 1) / groupSize;
    builder.reserve(1 + groupCount +
                    count * (multiPayload ? 1u : kMaxNodePayloads));
    const AABB sceneBounds = box(float(side) * 3.0f);
    const auto hierarchyRoot = builder.createNode(
        node(2, 1.0e6f, sceneBounds));
    for (uint32_t group = 0; group < groupCount; ++group)
    {
        const uint32_t first = group * groupSize;
        const uint32_t last = std::min(first + groupSize, count);
        AABB groupBounds = AABB::empty();
        for (uint32_t i = first; i < last; ++i)
        {
            const float4 position = float4::point(
                float(int(i % side) - int(side / 2)) * 3.0f,
                float(int(i / side) - int(side / 2)) * 3.0f, 0.0f);
            groupBounds.expand(box(0.5f, position));
        }
        const auto groupNode = builder.createNode(
            hierarchyRoot,
            node(UserPayload(3 + group), 1.0e6f, groupBounds));
        for (uint32_t i = first; i < last; ++i)
        {
            const float4 position = float4::point(
                float(int(i % side) - int(side / 2)) * 3.0f,
                float(int(i / side) - int(side / 2)) * 3.0f, 0.0f);
            const AABB bounds = box(0.5f, position);
            const UserPayload firstPayload = UserPayload(1000 + i * 8);
            if (multiPayload)
            {
                std::array<PayloadLodDesc,
                           kMaxNodePayloads - 1> additional{};
                for (uint32_t payloadIndex = 1;
                     payloadIndex < kMaxNodePayloads; ++payloadIndex)
                    additional[payloadIndex - 1] = {
                        UserPayload(firstPayload + payloadIndex),
                        errors[payloadIndex]};
                builder.createNode(
                    groupNode,
                    node(firstPayload, errors[0], bounds), additional);
            }
            else
            {
                auto parent = builder.createNode(
                    groupNode,
                    node(firstPayload, errors[0], bounds));
                for (uint32_t payloadIndex = 1;
                     payloadIndex < kMaxNodePayloads; ++payloadIndex)
                    parent = builder.createNode(
                        parent,
                        node(UserPayload(firstPayload + payloadIndex),
                             errors[payloadIndex], bounds));
            }
        }
    }

    SubtreeBytes bytes = builder.build();
    scene->storedNodes = detail::viewSubtreeBytes(bytes).nodeCount();
    scene->immutableBytes = bytes.size();
    const SubtreeHandle definition =
        scene->world.registerSubtree(std::move(bytes));
    const InstanceHandle root = scene->world.instantiate(
        node(1, 1.0e6f, sceneBounds, true));
    scene->world.mountSubtree(root.rootNode(), definition);
    TestAccess::markAllPayloadsReady(scene->world);
    scene->world.applyUpdates(0);
    return scene;
}

void consume(const FrontierResultView& cut)
{
    benchmark::DoNotOptimize(cut.entries.data());
    benchmark::DoNotOptimize(cut.size());
}

// The motivating case for node-local payload LODs: eight representations with
// one shared bound, authored either as an eight-node unary chain or one node
// with eight payload slots. Both cases produce one entry per logical object.
static void BM_SharedBoundPayloadLods(benchmark::State& state)
{
    const bool multiPayload = state.range(0) != 0;
    const uint32_t count = uint32_t(state.range(1));
    auto scene = buildPayloadLodScene(multiPayload, count);
    const uint32_t side =
        uint32_t(std::ceil(std::sqrt(double(count))));
    const float span = float(side) * 3.0f;
    const float4 openPlanes[6] = {
        {0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f, 1.0f},
        {0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f, 1.0f},
        {0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f, 1.0f}};
    const Camera camera = cameraFromPlanes(
        openPlanes, float4::point(0.0f, 0.0f, -span * 2.0f), 1080.0f);
    SpatialQuery query;
    query.setReuseEnabled(false);
    const SelectionParams params{4.0f, 0.0f};

    FrontierResultView result =
        query.selectFrontier(scene->world, camera, params);
    if (result.size() != count)
    {
        const std::string error =
            "payload LOD scene expected " + std::to_string(count) +
            " entries, got " + std::to_string(result.size());
        state.SkipWithError(error);
        return;
    }
    for (auto _ : state)
    {
        result = query.selectFrontier(scene->world, camera, params);
        consume(result);
    }
    state.counters["frontier"] = double(result.size());
    state.counters["immutable_KB"] =
        double(scene->immutableBytes) / 1024.0;
    state.counters["logical_lods"] =
        double(count * kMaxNodePayloads);
    state.counters["stored_nodes"] = double(scene->storedNodes);
}

BENCHMARK(BM_SharedBoundPayloadLods)
    ->Args({0, 4096})->Args({1, 4096})
    ->Args({0, 32768})->Args({1, 32768})
    ->ArgNames({"multi_payload", "objects"})
    ->Unit(benchmark::kMicrosecond);

} // namespace

int main(int argc, char** argv)
{
    benchmark::MaybeReenterWithoutASLR(argc, argv);
    benchmark::Initialize(&argc, argv);
    benchmark::AddCustomContext(
        "frontier_payload_bytes", std::to_string(sizeof(UserPayload)));
    if (benchmark::ReportUnrecognizedArguments(argc, argv)) return 1;
    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();
    return 0;
}
