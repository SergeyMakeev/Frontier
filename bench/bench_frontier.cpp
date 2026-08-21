#include <benchmark/benchmark.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "helpers.h"

using namespace frontier;
using namespace frontiertest;

namespace {

constexpr uint32_t kDetailCount = 8;

constexpr uint32_t kLiveCityStaticBlocks = 83;
constexpr uint32_t kLiveCityStaticLeavesPerBlock = 1024;
constexpr uint32_t kLiveCityStaticSingles = 8;
constexpr uint32_t kLiveCityCars = 100;
constexpr uint32_t kLiveCityCarLeaves = 50;
constexpr uint32_t kLiveCityPedestrians = 1000;
constexpr uint32_t kLiveCityPedestrianLeaves = 10;
constexpr uint32_t kLiveCityStaticDepth = 5;
constexpr uint32_t kLiveCityFrames = 4096;
constexpr uint32_t kLiveCityFrameMask = kLiveCityFrames - 1;
constexpr float kLiveCityFrameRate = 60.0f;
constexpr float kMetersPerSecondPerMph = 0.44704f;
constexpr float kPi = 3.14159265358979323846f;
constexpr uint32_t kLiveCityTotalLeaves =
    kLiveCityStaticBlocks * kLiveCityStaticLeavesPerBlock +
    kLiveCityStaticSingles + kLiveCityCars * kLiveCityCarLeaves +
    kLiveCityPedestrians * kLiveCityPedestrianLeaves;
static_assert(kLiveCityTotalLeaves == 100000);
static_assert((kLiveCityFrames & kLiveCityFrameMask) == 0);

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

AABB liveCityBounds(float x, float z, float half, float halfHeight)
{
    return AABB::fromCenterExtent(float4::point(x, 0.0f, z),
                                  float4::vec(half, halfHeight, half));
}

SubtreeBytes makeLiveCityStaticBlock(uint32_t firstPayload)
{
    struct LevelNode
    {
        SubtreeBuilder::NodeId id;
        float x;
        float z;
        float half;
    };

    SubtreeBuilder builder;
    builder.reserve(1365); // 1 + 4 + 16 + 64 + 256 + 1024.
    uint32_t payload = firstPayload;
    constexpr float rootHalf = 48.0f;
    constexpr float refineError = 10000.0f;
    std::vector<LevelNode> level;
    level.push_back({builder.createNode(node(
                         payload++, refineError,
                         liveCityBounds(0.0f, 0.0f, rootHalf, 20.0f))),
                     0.0f, 0.0f, rootHalf});

    for (uint32_t depth = 1; depth <= kLiveCityStaticDepth; ++depth)
    {
        std::vector<LevelNode> next;
        next.reserve(level.size() * 4);
        const bool leaf = depth == kLiveCityStaticDepth;
        for (const LevelNode& parent : level)
        {
            const float childHalf = parent.half * 0.5f;
            for (int dz : {-1, 1})
                for (int dx : {-1, 1})
                {
                    const float x = parent.x + float(dx) * childHalf;
                    const float z = parent.z + float(dz) * childHalf;
                    next.push_back({builder.createNode(
                                        parent.id,
                                        node(payload++, leaf ? 0.0f
                                                             : refineError,
                                             liveCityBounds(x, z, childHalf,
                                                            20.0f))),
                                    x, z, childHalf});
                }
        }
        level = std::move(next);
    }
    return builder.build();
}

SubtreeBytes makeLiveCityActor(uint32_t firstPayload, uint32_t leafCount,
                               float xPitch, float zPitch,
                               uint32_t columns, float halfExtent)
{
    SubtreeBuilder builder;
    builder.reserve(leafCount);
    const uint32_t rows = (leafCount + columns - 1) / columns;
    for (uint32_t i = 0; i < leafCount; ++i)
    {
        const float x = (float(i % columns) -
                         0.5f * float(columns - 1)) * xPitch;
        const float z = (float(i / columns) -
                         0.5f * float(rows - 1)) * zPitch;
        builder.createNode(node(firstPayload + i, 0.0f,
                                box(halfExtent, float4::point(x, 0.0f, z))));
    }
    return builder.build();
}

struct LiveCityScene
{
    SpatialDatabase world;
    SpatialDatabase::RigidMotionGroup carMotion;
    SpatialDatabase::RigidMotionGroup pedestrianMotion;
    SubtreeHandle carDefinition;
    SubtreeHandle pedestrianDefinition;
    std::vector<float4> unitCircle;
    std::vector<float4> carCenters;
    std::vector<float4> pedestrianCenters;
    std::vector<float4> carPositions;
    std::vector<float4> pedestrianPositions;
    std::vector<YawRotation> carYaws;
    std::vector<YawRotation> pedestrianYaws;
    std::vector<TerminalInstanceCluster> carClusters;
    std::vector<TerminalInstanceCluster> pedestrianClusters;
    std::vector<AABB> carClusterBounds;
    std::vector<AABB> pedestrianClusterBounds;
    std::vector<Camera> cameras;
    size_t immutableBytes = 0;
};

void buildLiveCityActorLayout(
    uint32_t count, uint32_t columns, uint32_t clusterWidth, float pitch,
    std::vector<float4>& centers,
    std::vector<TerminalInstanceCluster>& clusters)
{
    const uint32_t rows = (count + columns - 1) / columns;
    centers.reserve(count);
    clusters.reserve(rows * ((columns + clusterWidth - 1) / clusterWidth));
    for (uint32_t z = 0; z < rows; ++z)
    {
        for (uint32_t clusterX = 0; clusterX < columns;
             clusterX += clusterWidth)
        {
            const uint32_t first = uint32_t(centers.size());
            for (uint32_t dx = 0; dx < clusterWidth; ++dx)
            {
                const uint32_t x = clusterX + dx;
                const uint32_t logical = z * columns + x;
                if (x >= columns || logical >= count) continue;
                centers.push_back(float4::point(
                    (float(x) - 0.5f * float(columns - 1)) * pitch,
                    0.0f,
                    (float(z) - 0.5f * float(rows - 1)) * pitch));
            }
            const uint32_t clusterCount =
                uint32_t(centers.size()) - first;
            if (clusterCount != 0)
                clusters.push_back({first, clusterCount});
        }
    }
}

float liveCityTrackRadius(float mph, uint32_t loops)
{
    const float metersPerFrame = mph * kMetersPerSecondPerMph /
                                 kLiveCityFrameRate;
    const float radiansPerFrame =
        2.0f * kPi * float(loops) / float(kLiveCityFrames);
    return metersPerFrame / radiansPerFrame;
}

uint32_t liveCityTrackSample(uint32_t frame, uint32_t loops,
                             uint32_t phase, bool reverse)
{
    const uint32_t progress = (frame * loops) & kLiveCityFrameMask;
    return (phase + (reverse ? (kLiveCityFrames - progress) : progress)) &
           kLiveCityFrameMask;
}

void updateLiveCityActorPositions(LiveCityScene& scene, uint32_t frame)
{
    constexpr uint32_t carLoops = 4;
    constexpr uint32_t pedestrianLoops = 1;
    const float carRadius = liveCityTrackRadius(40.0f, carLoops);
    const float pedestrianRadius = liveCityTrackRadius(1.5f, pedestrianLoops);

    for (uint32_t i = 0; i < kLiveCityCars; ++i)
    {
        const uint32_t phase = i * kLiveCityFrames / kLiveCityCars;
        const uint32_t sample = liveCityTrackSample(
            frame, carLoops, phase, (i & 1u) != 0);
        const float4 u = scene.unitCircle[sample];
        const float4 c = scene.carCenters[i];
        scene.carPositions[i] = float4::point(
            c.x + carRadius * u.x, 0.0f, c.z + carRadius * u.z);
        const float direction = (i & 1u) != 0 ? -1.0f : 1.0f;
        // Actor +Z follows the exact tangent of the circular road.
        scene.carYaws[i] = {direction * u.x, direction * u.z};
    }

    for (uint32_t i = 0; i < kLiveCityPedestrians; ++i)
    {
        const uint32_t phase =
            uint32_t((uint64_t(i) * kLiveCityFrames) /
                     kLiveCityPedestrians);
        const uint32_t sample = liveCityTrackSample(
            frame, pedestrianLoops, phase, (i & 1u) != 0);
        const float4 u = scene.unitCircle[sample];
        const float4 c = scene.pedestrianCenters[i];
        scene.pedestrianPositions[i] = float4::point(
            c.x + pedestrianRadius * u.x, 0.0f,
            c.z + pedestrianRadius * u.z);
        const float direction = (i & 1u) != 0 ? -1.0f : 1.0f;
        scene.pedestrianYaws[i] = {direction * u.x, direction * u.z};
    }
}

void buildLiveCityClusterEnvelopes(
    std::span<const float4> centers,
    std::span<const TerminalInstanceCluster> clusters,
    float motionRadius, const AABB& localBounds,
    std::vector<AABB>& envelopes)
{
    envelopes.resize(clusters.size());
    const float4 motionExtent =
        float4::vec(motionRadius, 0.0f, motionRadius);
    for (size_t clusterIndex = 0; clusterIndex < clusters.size();
         ++clusterIndex)
    {
        const TerminalInstanceCluster cluster = clusters[clusterIndex];
        float4 minCenter = centers[cluster.first];
        float4 maxCenter = minCenter;
        const size_t end = size_t(cluster.first) + cluster.count;
        for (size_t i = cluster.first + 1; i < end; ++i)
        {
            minCenter = min4(minCenter, centers[i]);
            maxCenter = max4(maxCenter, centers[i]);
        }
        envelopes[clusterIndex] = AABB::fromMinMax(
            localBounds.mn + minCenter - motionExtent,
            localBounds.mx + maxCenter + motionExtent);
    }
}

std::unique_ptr<LiveCityScene> buildLiveCityScene(
    bool terminalBatches = false)
{
    auto scene = std::make_unique<LiveCityScene>();
    scene->unitCircle.reserve(kLiveCityFrames);
    scene->cameras.reserve(kLiveCityFrames);
    for (uint32_t frame = 0; frame < kLiveCityFrames; ++frame)
    {
        const float angle = 2.0f * kPi * float(frame) /
                            float(kLiveCityFrames);
        scene->unitCircle.push_back(
            float4::vec(std::cos(angle), 0.0f, std::sin(angle)));
    }

    constexpr uint32_t cameraLoops = 2;
    const float cameraRadius = liveCityTrackRadius(40.0f, cameraLoops);
    for (uint32_t frame = 0; frame < kLiveCityFrames; ++frame)
    {
        const uint32_t sample = (frame * cameraLoops) & kLiveCityFrameMask;
        const float4 u = scene->unitCircle[sample];
        const float4 eye = float4::point(cameraRadius * u.x, 2.0f,
                                         cameraRadius * u.z);
        const float4 target = float4::point(
            eye.x - 15.0f * u.z, eye.y, eye.z + 15.0f * u.x);
        scene->cameras.push_back(makeLookAtCamera(
            eye, target, 1.0f, 16.0f / 9.0f, 1080.0f, 0.1f, 1500.0f));
    }

    for (uint32_t i = 0; i < kLiveCityStaticBlocks; ++i)
    {
        SubtreeBytes staticBytes = makeLiveCityStaticBlock(1000000 + i * 2000);
        scene->immutableBytes += staticBytes.size();
        const SubtreeHandle staticBlock =
            scene->world.registerSubtree(std::move(staticBytes));
        InstanceDesc desc;
        desc.pos = float4::point((float(i % 10) - 4.5f) * 100.0f, 0.0f,
                                 (float(i / 10) - 4.0f) * 100.0f);
        const InstanceHandle block = scene->world.instantiate(
            node(10000 + i, 10000.0f,
                 liveCityBounds(0.0f, 0.0f, 50.0f, 25.0f), true), desc);
        scene->world.mountSubtree(block.rootNode(), staticBlock);
    }
    for (uint32_t i = 0; i < kLiveCityStaticSingles; ++i)
    {
        InstanceDesc desc;
        desc.pos = float4::point((float(i) - 3.5f) * 35.0f, 0.0f, -440.0f);
        scene->world.instantiate(node(11000 + i, 0.0f, box(2.0f)), desc);
    }

    SubtreeBytes carBytes = makeLiveCityActor(
        2000000, kLiveCityCarLeaves, 0.8f, 0.8f, 5, 0.3f);
    scene->immutableBytes += carBytes.size();
    const SubtreeHandle car = scene->world.registerSubtree(std::move(carBytes));
    scene->carDefinition = car;
    SubtreeBytes pedestrianBytes = makeLiveCityActor(
        3000000, kLiveCityPedestrianLeaves, 0.4f, 0.4f, 5, 0.15f);
    scene->immutableBytes += pedestrianBytes.size();
    const SubtreeHandle pedestrian =
        scene->world.registerSubtree(std::move(pedestrianBytes));
    scene->pedestrianDefinition = pedestrian;

    buildLiveCityActorLayout(
        kLiveCityCars, 10, 2, 82.0f, scene->carCenters,
        scene->carClusters);
    scene->carPositions.resize(kLiveCityCars);
    scene->carYaws.resize(kLiveCityCars);
    buildLiveCityClusterEnvelopes(
        scene->carCenters, scene->carClusters,
        liveCityTrackRadius(40.0f, 4), box(5.0f),
        scene->carClusterBounds);
    buildLiveCityActorLayout(
        kLiveCityPedestrians, 32, 8, 27.0f, scene->pedestrianCenters,
        scene->pedestrianClusters);
    scene->pedestrianPositions.resize(kLiveCityPedestrians);
    scene->pedestrianYaws.resize(kLiveCityPedestrians);
    buildLiveCityClusterEnvelopes(
        scene->pedestrianCenters, scene->pedestrianClusters,
        liveCityTrackRadius(1.5f, 1), box(1.5f),
        scene->pedestrianClusterBounds);
    updateLiveCityActorPositions(*scene, 0);

    std::vector<InstanceHandle> carHandles;
    std::vector<InstanceHandle> pedestrianHandles;
    if (!terminalBatches)
    {
        carHandles.reserve(kLiveCityCars);
        for (uint32_t i = 0; i < kLiveCityCars; ++i)
        {
            InstanceDesc desc;
            desc.pos = scene->carPositions[i];
            desc.yaw = scene->carYaws[i];
            NodeDesc carRoot =
                node(20000 + i, 10000.0f, box(5.0f), true);
            carRoot.flags |= NodeDesc::FlagYawInvariantBounds;
            const InstanceHandle instance = scene->world.instantiate(
                carRoot, desc);
            scene->world.setInstanceRenderAsUnit(instance);
            scene->world.mountSubtree(instance.rootNode(), car);
            carHandles.push_back(instance);
        }

        pedestrianHandles.reserve(kLiveCityPedestrians);
        for (uint32_t i = 0; i < kLiveCityPedestrians; ++i)
        {
            InstanceDesc desc;
            desc.pos = scene->pedestrianPositions[i];
            desc.yaw = scene->pedestrianYaws[i];
            NodeDesc pedestrianRoot =
                node(30000 + i, 10000.0f, box(1.5f), true);
            pedestrianRoot.flags |= NodeDesc::FlagYawInvariantBounds;
            const InstanceHandle instance = scene->world.instantiate(
                pedestrianRoot, desc);
            scene->world.setInstanceRenderAsUnit(instance);
            scene->world.mountSubtree(instance.rootNode(), pedestrian);
            pedestrianHandles.push_back(instance);
        }
    }

    TestAccess::markAllNodesReady(scene->world);
    scene->world.optimize();
    if (!terminalBatches)
    {
        scene->carMotion.reset(carHandles);
        scene->pedestrianMotion.reset(pedestrianHandles);
    }
    return scene;
}

std::array<TerminalInstanceBatch, 2> liveCityActorBatches(
    const LiveCityScene& scene)
{
    std::array<TerminalInstanceBatch, 2> batches;
    batches[0].definition = scene.carDefinition;
    batches[0].localBounds = box(5.0f);
    batches[0].positions = scene.carPositions;
    batches[0].yaws = scene.carYaws;
    batches[0].clusters = scene.carClusters;
    batches[0].clusterBounds = scene.carClusterBounds;
    batches[0].firstInstance =
        kLiveCityStaticBlocks + kLiveCityStaticSingles;
    batches[0].yawInvariantBounds = true;

    batches[1].definition = scene.pedestrianDefinition;
    batches[1].localBounds = box(1.5f);
    batches[1].positions = scene.pedestrianPositions;
    batches[1].yaws = scene.pedestrianYaws;
    batches[1].clusters = scene.pedestrianClusters;
    batches[1].clusterBounds = scene.pedestrianClusterBounds;
    batches[1].firstInstance = batches[0].firstInstance + kLiveCityCars;
    batches[1].yawInvariantBounds = true;
    return batches;
}

void consume(const FrontierResultView& cut)
{
    benchmark::DoNotOptimize(cut.shared.data());
    benchmark::DoNotOptimize(cut.currentOnly.data());
    benchmark::DoNotOptimize(cut.idealOnly.data());
    benchmark::DoNotOptimize(cut.size());
}

#ifndef FRONTIER_OMIT_SUBMISSION_BENCH
void consumeLiveCitySubmissions(const RenderFrontierView& result)
{
    uint64_t checksum = 0;
    for (const RenderFrontierRun run : result.runs())
    {
        const RenderFrontierSpan leaves = result[run];
        checksum += leaves.instance;
        for (size_t i = 0; i < leaves.size(); ++i)
            checksum += uint64_t(leaves.payloads[i]) + leaves.errors[i];
    }
    benchmark::DoNotOptimize(checksum);
    benchmark::ClobberMemory();
}

void consumeLiveCitySubmissions(const TerminalRenderView& result)
{
    uint64_t checksum = 0;
    for (const TerminalRenderRun run : result.runs())
    {
        checksum += run.instance();
        for (const UserPayload payload : run.payloadSpan())
            checksum += uint64_t(payload) + run.errorCode();
    }
    benchmark::DoNotOptimize(checksum);
    benchmark::ClobberMemory();
}
#endif

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
    scene->world.applyUpdates(0);
    if (cached) consume(query.selectFrontier(scene->world, view, params));

    FrontierResultView cut;
    for (auto _ : state)
    {
        scene->world.applyUpdates(0);
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
    scene->world.applyUpdates(0);

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
        scene->world.applyUpdates(0);
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
    handles.reserve(count);
    for (uint32_t i = 0; i < count; ++i)
    {
        const float4 position = float4::point(float(i) * 2.0f, 0, 0);
        InstanceDesc desc;
        desc.pos = position;
        handles.push_back(world.instantiate(
            node(1000 + i, 0.0f, box(0.5f)), desc));
    }
    SpatialDatabase::MotionGroup group(handles);

    bool raised = false;
    for (auto _ : state)
    {
        float dy = 0.0f;
        if (!unchanged)
        {
            raised = !raised;
            dy = raised ? 0.25f : -0.25f;
        }
        world.translateInstances(group, float4::vec(0.0f, dy, 0.0f));
        benchmark::ClobberMemory();
    }
    state.counters["instances"] = double(count);
}

BENCHMARK(BM_MotionGroupSteady)
    ->Args({128, 0})->Args({400, 0})
    ->Args({128, 1})->Args({400, 1})
    ->ArgNames({"instances", "unchanged"})
    ->Unit(benchmark::kMicrosecond);

static void BM_TlasIncrementalMaintenance(benchmark::State& state)
{
    const uint32_t count = uint32_t(state.range(0));
    const uint32_t movingCount = uint32_t(state.range(1));
    const uint32_t nodeBudget = uint32_t(state.range(2));
    SpatialDatabase world;
    std::vector<InstanceHandle> movers;
    movers.reserve(movingCount);
    const uint32_t side =
        uint32_t(std::ceil(std::sqrt(double(count))));
    for (uint32_t i = 0; i < count; ++i)
    {
        InstanceDesc desc;
        desc.pos = float4::point(float(i % side) * 2.0f, 0.0f,
                                 float(i / side) * 2.0f);
        const InstanceHandle handle = world.instantiate(
            node(1000 + i, 0.0f, box(0.5f)), desc);
        if (movers.size() < movingCount &&
            (uint64_t(i) * movingCount) / count == movers.size())
            movers.push_back(handle);
    }
    world.applyUpdates(0);
    world.optimize();
    SpatialDatabase::MotionGroup motion(movers);

    bool raised = false;
    uint64_t totalProcessed = 0;
    uint64_t totalPending = 0;
    double totalAreaGrowth = 0.0;
    UpdateReport report;
    for (auto _ : state)
    {
        raised = !raised;
        world.translateInstances(
            motion, float4::vec(0.0f, raised ? 0.25f : -0.25f, 0.0f));
        report = world.applyUpdates(nodeBudget);
        benchmark::DoNotOptimize(report.areaGrowthRatio);
        totalProcessed += report.maintenanceNodesProcessed;
        totalPending += report.maintenanceNodesPending;
        totalAreaGrowth += report.areaGrowthRatio;
    }
    const double calls = double(state.iterations());
    state.counters["processed_per_call"] = double(totalProcessed) / calls;
    state.counters["pending_per_call"] = double(totalPending) / calls;
    state.counters["area_growth_per_call"] = totalAreaGrowth / calls;
    state.SetItemsProcessed(state.iterations() * int64_t(movingCount));
}

BENCHMARK(BM_TlasIncrementalMaintenance)
    ->Args({10000, 100, 0})
    ->Args({10000, 100, 16})
    ->Args({10000, 100, 64})
    ->Args({10000, 100, 256})
    ->ArgNames({"instances", "moving", "repair_nodes"})
    ->Unit(benchmark::kMicrosecond);

// End-to-end dynamic-frame cost: move a distributed subset of TLAS roots,
// publish the exact leaf changes, then select a mounted hierarchy. Static
// roots remain eligible for exact-cut reuse while moved roots are invalidated
// by their frontier version.
static void BM_MovingObjectsSelectionScale(benchmark::State& state)
{
    const uint32_t count = uint32_t(state.range(0));
    const uint32_t movingPercent = uint32_t(state.range(1));

    SpatialDatabase world;
    const SubtreeHandle definition =
        world.registerSubtree(makeLodSubtree(50000, 50001, 50002));
    const uint32_t side =
        uint32_t(std::ceil(std::sqrt(double(count))));
    constexpr float pitch = 12.0f;
    std::vector<InstanceHandle> movingHandles;
    movingHandles.reserve(size_t(count) * movingPercent / 100u);
    for (uint32_t i = 0; i < count; ++i)
    {
        const float4 position = float4::point(
            float(int(i % side) - int(side / 2)) * pitch,
            float(int(i / side) - int(side / 2)) * pitch, 0.0f);
        InstanceDesc desc;
        desc.pos = position;
        const InstanceHandle instance = world.instantiate(
            node(1000 + i, 64.0f, box(4.0f), true), desc);
        world.mountSubtree(instance.rootNode(), definition);
        if ((i % 100u) < movingPercent)
            movingHandles.push_back(instance);
    }
    TestAccess::markAllNodesReady(world);
    world.applyUpdates(0);

    SpatialDatabase::MotionGroup motion(movingHandles);
    const float span = float(side) * pitch;
    const Camera camera = makeLookAtCamera(
        float4::point(0.0f, 0.0f, -span),
        float4::point(0.0f, 0.0f, 0.0f));
    SpatialQuery query;
    consume(query.selectFrontier(world, camera, {}));

    bool raised = false;
    uint64_t calls = 0;
    uint64_t totalReused = 0;
    uint64_t totalWalked = 0;
    FrontierResultView result;
    for (auto _ : state)
    {
        raised = !raised;
        world.translateInstances(
            motion, float4::vec(0.0f, raised ? 0.25f : -0.25f, 0.0f));
        world.applyUpdates(0);
        result = query.selectFrontier(world, camera, {});
        consume(result);
        ++calls;
        totalReused += query.reused();
        totalWalked += query.walked();
    }

    const double callCount = double(calls);
    const double visited = double(totalReused + totalWalked);
    state.counters["entries"] = double(result.size());
    state.counters["moved"] = double(movingHandles.size());
    state.counters["reused_per_call"] = double(totalReused) / callCount;
    state.counters["walked_per_call"] = double(totalWalked) / callCount;
    state.counters["reuse_percent"] =
        visited == 0.0 ? 0.0 : 100.0 * double(totalReused) / visited;
}

BENCHMARK(BM_MovingObjectsSelectionScale)
    ->Args({1000, 10})->Args({1000, 100})
    ->Args({10000, 10})->Args({10000, 100})
    ->ArgNames({"instances", "moving_percent"})
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
    scene->world.applyUpdates(0);

    FrontierResultView cut;
    for (auto _ : state)
    {
        scene->world.applyUpdates(0);
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
    world.applyUpdates(0);
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

// Separates TLAS topology quality from BLAS work. The distant camera sees the
// complete grid and measures traversal/occupancy; the close camera rejects
// most instances and measures whether tighter bounds repay a larger tree.
static void BM_TlasQualitySelection(benchmark::State& state)
{
    constexpr uint32_t count = 10000;
    SpatialDatabaseConfig config;
    config.tlasQuality = TlasQuality(state.range(0));
    SpatialDatabase world(config);
    constexpr uint32_t side = 100;
    for (uint32_t i = 0; i < count; ++i)
    {
        InstanceDesc desc;
        desc.pos = float4::point(
            float(int(i % side) - int(side / 2)) * 3.0f,
            float(int(i / side) - int(side / 2)) * 3.0f, 0.0f);
        world.instantiate(node(1000 + i, 0.0f, box(0.5f)), desc);
    }
    world.applyUpdates(0);
    const bool closeCamera = state.range(1) != 0;
    const Camera camera = cameraAt(closeCamera ? -50.0f : -2000.0f);
    SpatialQuery query;
    query.setReuseEnabled(false);

    FrontierResultView result;
    for (auto _ : state)
    {
        result = query.selectFrontier(world, camera, {});
        consume(result);
    }
    state.counters["entries"] = double(result.size());
    state.counters["tlas_nodes"] =
        double(TestAccess::tlasNodeCount(world));
    state.counters["tlas_KB"] =
        double(TestAccess::tlasNodeCount(world) *
               TestAccess::tlasNodeBytes()) /
        1024.0;
}

BENCHMARK(BM_TlasQualitySelection)
    ->Args({int64_t(TlasQuality::Morton), 0})
    ->Args({int64_t(TlasQuality::Median), 0})
    ->Args({int64_t(TlasQuality::BinnedSAH), 0})
    ->Args({int64_t(TlasQuality::Morton), 1})
    ->Args({int64_t(TlasQuality::Median), 1})
    ->Args({int64_t(TlasQuality::BinnedSAH), 1})
    ->ArgNames({"quality", "close_camera"})
    ->Unit(benchmark::kMicrosecond);

// Exercises the indexed/dependent-load pipelines between the TLAS result,
// Instance records, mount slots, and shared subtree arrays. Reuse mode 0 is
// uncached, 1 measures stable whole-view hits, and 2 cycles three thresholds
// to invalidate every record and force the cached-miss path deterministically.
// Three distinct keys are intentional: the exact-view memo owns two entries,
// so a two-state policy cycle would memoize both complete results and stop
// exercising record validation or hierarchy traversal.
static void BM_InstanceForestSelectionScale(benchmark::State& state)
{
    const uint32_t count = uint32_t(state.range(0));
    const uint32_t hierarchicalPercent = uint32_t(state.range(1));
    const uint32_t reuseMode = uint32_t(state.range(2));

    SpatialDatabase world;
    const SubtreeHandle definition =
        world.registerSubtree(makeLodSubtree(50000, 50001, 50002));
    const uint32_t side =
        uint32_t(std::ceil(std::sqrt(double(count))));
    constexpr float pitch = 12.0f;
    for (uint32_t i = 0; i < count; ++i)
    {
        InstanceDesc desc;
        desc.pos = float4::point(
            float(int(i % side) - int(side / 2)) * pitch,
            float(int(i / side) - int(side / 2)) * pitch, 0.0f);
        const bool hierarchical =
            (i % 100) < hierarchicalPercent;
        const InstanceHandle instance = world.instantiate(
            node(1000 + i, hierarchical ? 64.0f : 0.0f,
                 box(4.0f), hierarchical),
            desc);
        if (hierarchical)
            world.mountSubtree(instance.rootNode(), definition);
    }
    TestAccess::markAllNodesReady(world);
    world.applyUpdates(0);

    const float span = float(side) * pitch;
    const Camera stable = makeLookAtCamera(
        float4::point(0.0f, 0.0f, -span),
        float4::point(0.0f, 0.0f, 0.0f));
    SpatialQuery query;
    query.setReuseEnabled(reuseMode != 0);
    if (reuseMode != 0)
        consume(query.selectFrontier(world, stable, {}));

    uint64_t call = 0;
    FrontierResultView result;
    for (auto _ : state)
    {
        SelectionParams params;
        if (reuseMode == 2)
        {
            constexpr float missThresholds[] = {3.75f, 4.0f, 4.25f};
            params.threshold = missThresholds[++call % std::size(missThresholds)];
        }
        result = query.selectFrontier(world, stable, params);
        consume(result);
    }
    state.counters["entries"] = double(result.size());
    state.counters["reused"] = double(query.reused());
    state.counters["walked"] = double(query.walked());
}

BENCHMARK(BM_InstanceForestSelectionScale)
    ->Args({1000, 50, 0})->Args({1000, 50, 1})->Args({1000, 50, 2})
    ->Args({1000, 100, 0})->Args({1000, 100, 1})->Args({1000, 100, 2})
    ->Args({10000, 50, 0})->Args({10000, 50, 1})->Args({10000, 50, 2})
    ->Args({10000, 100, 0})->Args({10000, 100, 1})->Args({10000, 100, 2})
    ->ArgNames({"instances", "hierarchical_percent", "reuse_mode"})
    ->Unit(benchmark::kMicrosecond);

// Measures an exact recurring-camera workload rather than invalidating the
// cache through a parameter change. Two prebuilt, identically oriented cameras
// alternate by the requested translation, so the timed region contains only
// selection. Once both recurring views have been admitted, the two-entry
// whole-cut memo returns exact snapshots. This intentionally does not represent
// a stream of continuously unique camera poses.
static void BM_MovingCameraSelectionScale(benchmark::State& state)
{
    const uint32_t count = uint32_t(state.range(0));
    const float cameraStep = float(state.range(1)) * 0.01f;

    SpatialDatabase world;
    const SubtreeHandle definition =
        world.registerSubtree(makeLodSubtree(50000, 50001, 50002));
    const uint32_t side =
        uint32_t(std::ceil(std::sqrt(double(count))));
    constexpr float pitch = 12.0f;
    for (uint32_t i = 0; i < count; ++i)
    {
        InstanceDesc desc;
        desc.pos = float4::point(
            float(int(i % side) - int(side / 2)) * pitch,
            float(int(i / side) - int(side / 2)) * pitch, 0.0f);
        const InstanceHandle instance = world.instantiate(
            node(1000 + i, 64.0f, box(4.0f), true), desc);
        world.mountSubtree(instance.rootNode(), definition);
    }
    TestAccess::markAllNodesReady(world);
    world.applyUpdates(0);

    const float span = float(side) * pitch;
    const float4 baseEye = float4::point(0.0f, 0.0f, -span);
    const float4 baseTarget = float4::point(0.0f, 0.0f, 0.0f);
    const float4 offset = float4::vec(cameraStep, 0.0f, 0.0f);
    const Camera cameras[2] = {
        makeLookAtCamera(baseEye, baseTarget),
        makeLookAtCamera(baseEye + offset, baseTarget + offset),
    };

    SpatialQuery query;
    consume(query.selectFrontier(world, cameras[0], {}));

    uint64_t calls = 0;
    uint64_t totalReused = 0;
    uint64_t totalWalked = 0;
    FrontierResultView result;
    for (auto _ : state)
    {
        result = query.selectFrontier(world, cameras[++calls & 1u], {});
        consume(result);
        totalReused += query.reused();
        totalWalked += query.walked();
    }

    const double callCount = double(calls);
    const double visited = double(totalReused + totalWalked);
    state.counters["camera_step"] = double(cameraStep);
    state.counters["entries"] = double(result.size());
    state.counters["reused_per_call"] = double(totalReused) / callCount;
    state.counters["walked_per_call"] = double(totalWalked) / callCount;
    state.counters["reuse_percent"] =
        visited == 0.0 ? 0.0 : 100.0 * double(totalReused) / visited;
}

BENCHMARK(BM_MovingCameraSelectionScale)
    ->Args({1000, 0})->Args({1000, 10})
    ->Args({1000, 1600})->Args({1000, 25600})
    ->Args({10000, 0})->Args({10000, 10})
    ->Args({10000, 1600})->Args({10000, 25600})
    ->ArgNames({"instances", "step_x100"})
    ->Unit(benchmark::kMicrosecond);

// One complete 60 Hz city-simulation frame. The camera follows a contiguous
// 40 mph circular road trajectory with no adjacent repeated pose. One hundred
// car placements, each owning 50 detail leaves, travel at 40 mph; 1,000
// pedestrian placements, each owning 10 part leaves, travel at 1.5 mph. The
// remaining 85,000 leaves are static, primarily in 83 independent depth-five
// blocks. Simulation-owned actor transforms and exact descendant-culling
// terminal-range selection are timed. The actor SoA spans are consumed in
// place, so no duplicate per-instance publication or dynamic TLAS refit exists;
// immutable scene/camera and one-time definition-plan construction are not
// timed.
static void BM_LiveCityDrivingFrame(benchmark::State& state)
{
    auto scene = buildLiveCityScene(true);
    const auto batches = liveCityActorBatches(*scene);
    size_t referenceSize = 0;
    {
        auto referenceScene = buildLiveCityScene(false);
        SpatialQuery reference;
        reference.setReuseEnabled(false);
        referenceSize = reference.selectFrontier(
            referenceScene->world, referenceScene->cameras[0], {}).size();
    }
    TerminalRenderQuery query;
    const TerminalRenderView first =
        query.select(scene->world, scene->cameras[0], batches, 4.0f, false);
    if (first.size() != referenceSize)
    {
        state.SkipWithError("batched actors changed the exact cut");
        return;
    }
    benchmark::DoNotOptimize(first.runs().data());

    uint32_t frame = 0;
    uint64_t calls = 0;
    uint64_t totalEntries = 0;
    uint64_t totalSegments = 0;
    uint64_t minEntries = uint64_t(-1);
    uint64_t maxEntries = 0;
    TerminalRenderView result;
    for (auto _ : state)
    {
        frame = (frame + 1) & kLiveCityFrameMask;
        updateLiveCityActorPositions(*scene, frame);
        result = query.select(scene->world, scene->cameras[frame], batches,
                              4.0f, false);
        benchmark::DoNotOptimize(result.runs().data());
        benchmark::DoNotOptimize(result.size());
        ++calls;
        totalEntries += result.size();
        totalSegments += result.segmentCount();
        minEntries = std::min<uint64_t>(minEntries, result.size());
        maxEntries = std::max<uint64_t>(maxEntries, result.size());
    }

    const double callCount = double(calls);
    state.counters["camera_mph"] = 40.0;
    state.counters["car_mph"] = 40.0;
    state.counters["entries_per_call"] = double(totalEntries) / callCount;
    state.counters["immutable_KB"] = double(scene->immutableBytes) / 1024.0;
    state.counters["orientation_KB"] =
        double(scene->world.instanceOrientationStateBytes()) / 1024.0;
    state.counters["max_entries"] = double(maxEntries);
    state.counters["min_entries"] = double(minEntries);
    state.counters["mount_state_KB"] =
        double(scene->world.subtreeInstanceStateBytes()) / 1024.0;
    state.counters["moving_roots"] =
        double(kLiveCityCars + kLiveCityPedestrians);
    state.counters["pedestrian_mph"] = 1.5;
    state.counters["potential_leaves"] = double(kLiveCityTotalLeaves);
    state.counters["query_KB"] = double(query.bytes()) / 1024.0;
    state.counters["segments_per_call"] =
        double(totalSegments) / callCount;
    state.counters["simulated_seconds"] = callCount / kLiveCityFrameRate;
    state.counters["static_depth"] = double(kLiveCityStaticDepth);
    state.counters["tlas_roots"] =
        double(kLiveCityStaticBlocks + kLiveCityStaticSingles +
               kLiveCityCars + kLiveCityPedestrians);
    state.SetItemsProcessed(int64_t(calls));
}

BENCHMARK(BM_LiveCityDrivingFrame)
    ->Iterations(kLiveCityFrames * 2)
    ->Unit(benchmark::kMicrosecond);

// Isolates simulation transform generation for the terminal actor-batch
// architecture. The query consumes these arrays directly, so this is the
// complete moving-object writer cost: there is no second transform copy or
// dynamic TLAS publication hidden outside the benchmark.
static void BM_LiveCityMotionFrame(benchmark::State& state)
{
    auto scene = buildLiveCityScene(true);

    uint32_t frame = 0;
    uint64_t calls = 0;
    for (auto _ : state)
    {
        frame = (frame + 1) & kLiveCityFrameMask;
        updateLiveCityActorPositions(*scene, frame);
        benchmark::DoNotOptimize(scene->carPositions.data());
        benchmark::DoNotOptimize(scene->pedestrianPositions.data());
        benchmark::ClobberMemory();
        ++calls;
    }

    state.counters["moving_roots"] =
        double(kLiveCityCars + kLiveCityPedestrians);
    state.counters["orientation_KB"] =
        double(scene->world.instanceOrientationStateBytes()) / 1024.0;
    state.counters["simulated_seconds"] =
        double(calls) / kLiveCityFrameRate;
    state.SetItemsProcessed(int64_t(calls));
}

BENCHMARK(BM_LiveCityMotionFrame)
    ->Iterations(kLiveCityFrames * 2)
    ->Unit(benchmark::kMicrosecond);

// End-to-end CPU frame companion to BM_LiveCityDrivingFrame. In addition to
// actor motion and terminal-range production, this scans the payload and
// metadata of every logical selected leaf. It represents
// the minimum downstream iteration cost that a selection-only benchmark
// intentionally excludes; no allocator or graphics-driver work is timed.
#ifndef FRONTIER_OMIT_SUBMISSION_BENCH
static void BM_LiveCityRenderSubmissionFrame(benchmark::State& state)
{
    auto scene = buildLiveCityScene(true);
    const auto batches = liveCityActorBatches(*scene);
    size_t referenceSize = 0;
    {
        auto referenceScene = buildLiveCityScene(false);
        SpatialQuery reference;
        reference.setReuseEnabled(true);
        referenceSize = reference.selectRenderFrontier(
            referenceScene->world, referenceScene->cameras[0], {}).size();
    }
    TerminalRenderQuery query;
    TerminalRenderView result =
        query.select(scene->world, scene->cameras[0], batches);
    if (result.size() != referenceSize)
    {
        state.SkipWithError("batched actors changed the render cut");
        return;
    }
    consumeLiveCitySubmissions(result);

    uint32_t frame = 0;
    uint64_t calls = 0;
    uint64_t totalEntries = 0;
    uint64_t totalSubmissions = 0;
    uint64_t totalSegments = 0;
    uint64_t minEntries = uint64_t(-1);
    uint64_t maxEntries = 0;
    for (auto _ : state)
    {
        frame = (frame + 1) & kLiveCityFrameMask;
        updateLiveCityActorPositions(*scene, frame);
        result = query.select(scene->world, scene->cameras[frame], batches);
        consumeLiveCitySubmissions(result);
        ++calls;
        totalEntries += result.size();
        totalSubmissions += result.size();
        totalSegments += result.segmentCount();
        minEntries = std::min<uint64_t>(minEntries, result.size());
        maxEntries = std::max<uint64_t>(maxEntries, result.size());
    }

    const double callCount = double(calls);
    state.counters["camera_mph"] = 40.0;
    state.counters["car_mph"] = 40.0;
    state.counters["entries_per_call"] = double(totalEntries) / callCount;
    state.counters["immutable_KB"] = double(scene->immutableBytes) / 1024.0;
    state.counters["max_entries"] = double(maxEntries);
    state.counters["min_entries"] = double(minEntries);
    state.counters["moving_roots"] =
        double(kLiveCityCars + kLiveCityPedestrians);
    state.counters["pedestrian_mph"] = 1.5;
    state.counters["potential_leaves"] = double(kLiveCityTotalLeaves);
    state.counters["query_KB"] = double(query.bytes()) / 1024.0;
    state.counters["simulated_seconds"] = callCount / kLiveCityFrameRate;
    state.counters["segments_per_call"] =
        double(totalSegments) / callCount;
    state.counters["static_depth"] = double(kLiveCityStaticDepth);
    state.counters["submissions_per_call"] =
        double(totalSubmissions) / callCount;
    state.SetItemsProcessed(int64_t(calls));
}

BENCHMARK(BM_LiveCityRenderSubmissionFrame)
    ->Iterations(kLiveCityFrames * 2)
    ->Unit(benchmark::kMicrosecond);
#endif

// Same mounted population as the refined forest, viewed far enough away that
// selection stops at every renderable TLAS root. Together the two benchmarks
// separate top-level query/dispatch from mounted BLAS traversal.
static void BM_InstanceForestRootSelectionScale(benchmark::State& state)
{
    const uint32_t count = uint32_t(state.range(0));
    SpatialDatabase world;
    const SubtreeHandle definition =
        world.registerSubtree(makeLodSubtree(50000, 50001, 50002));
    const uint32_t side =
        uint32_t(std::ceil(std::sqrt(double(count))));
    constexpr float pitch = 12.0f;
    for (uint32_t i = 0; i < count; ++i)
    {
        InstanceDesc desc;
        desc.pos = float4::point(
            float(int(i % side) - int(side / 2)) * pitch,
            float(int(i / side) - int(side / 2)) * pitch, 0.0f);
        const InstanceHandle instance = world.instantiate(
            node(1000 + i, 64.0f, box(4.0f), true), desc);
        world.mountSubtree(instance.rootNode(), definition);
    }
    TestAccess::markAllNodesReady(world);
    world.applyUpdates(0);

    const Camera camera = makeLookAtCamera(
        float4::point(0.0f, 0.0f, -20000.0f),
        float4::point(0.0f, 0.0f, 0.0f));
    SpatialQuery query;
    query.setReuseEnabled(false);
    FrontierResultView result;
    for (auto _ : state)
    {
        result = query.selectFrontier(world, camera, {});
        consume(result);
    }
    state.counters["entries"] = double(result.size());
    state.counters["tlas_nodes"] =
        double(TestAccess::tlasNodeCount(world));
}

BENCHMARK(BM_InstanceForestRootSelectionScale)
    ->Arg(1000)->Arg(10000)
    ->ArgName("instances")
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
    world.applyUpdates(0);

    for (auto _ : state)
    {
        const InstanceHandle transient = world.instantiate(
            node(50000, 0.0f, box(0.5f)),
            InstanceDesc{.pos = float4::point(100, 100, 0)});
        world.removeInstance(transient);
        world.applyUpdates(0);
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
    state.counters["overlay_KB"] = double(world.overlayBytes()) / 1024.0;
}

BENCHMARK(BM_BoundsOverrideBatch)
    ->Arg(1)->Arg(32)->Arg(64)->Arg(256)
    ->Unit(benchmark::kMicrosecond);

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
