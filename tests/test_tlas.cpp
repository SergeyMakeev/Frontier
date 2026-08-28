#include <gtest/gtest.h>

#include <array>
#include <vector>

#include "helpers.h"

using namespace frontier;
using namespace frontiertest;

TEST(Tlas, ManySingleNodesNeedNoBlasAllocation)
{
    SpatialDatabase database;
    constexpr uint32_t count = 1000;
    for (uint32_t i = 0; i < count; ++i)
    {
        InstanceDesc desc;
        desc.pos = float4::point(float(i % 20) * 4.0f, 0,
                                 float(i / 20) * 4.0f);
        database.instantiate(node(1000 + i, 0.0f, box()), desc);
    }
    database.applyUpdates(0);
    EXPECT_EQ(TestAccess::liveInstanceSlots(database), count);
    EXPECT_EQ(database.subtreeCount(), 0u);
    EXPECT_EQ(database.mountedSubtreeCount(), 0u);
}

TEST(Tlas, InstanceHandlesSurviveOptimize)
{
    SpatialDatabase database;
    std::vector<InstanceHandle> handles;
    for (uint32_t i = 0; i < 64; ++i)
    {
        InstanceDesc desc;
        desc.pos = float4::point(float(i) * 3.0f, 0, 0);
        handles.push_back(database.instantiate(node(i + 1, 0.0f, box()), desc));
    }
    for (uint32_t i = 0; i < 64; i += 3)
        database.removeInstance(handles[i]);
    database.optimize(OptimizationMode::TopologyAndLayout);

    database.moveInstance(handles[1],
                          Transform{float4::point(500, 0, 0), 1.0f});
    database.applyUpdates(0);
    EXPECT_NEAR(TestAccess::instanceBounds(database, handles[1]).center().x,
                500.0f, 0.001f);
    EXPECT_TRUE(TestAccess::instanceBounds(database, handles[0]).isEmpty());
}

TEST(Tlas, LayerMaskCullsAtTopLevel)
{
    SpatialDatabase database;
    InstanceDesc visible;
    visible.mask = 0x1;
    InstanceDesc hidden;
    hidden.mask = 0x2;
    database.instantiate(node(1, 0.0f, box()), visible);
    database.instantiate(node(2, 0.0f, box()), hidden);

    Camera camera = cameraAt();
    camera.viewMask = 0x1;
    SpatialQuery query;
    const FrontierResultView cut = select(database, query, camera);
    EXPECT_EQ(payloads(database, cut), (std::vector<UserPayload>{1}));
}

TEST(Tlas, ContributionCullUsesProjectedBoundsNotGeometricError)
{
    SpatialDatabase database;
    // The large zero-error object must remain visible, while the tiny object
    // with an enormous authored error must be culled. Geometric error chooses
    // an LOD; it is not a measure of the object's image contribution.
    database.instantiate(node(1, 0.0f, box(10.0f)));
    database.instantiate(node(2, 10000.0f, box(0.1f)));
    SpatialQuery query;
    SelectionParams params;
    params.minPix = 10.0f;
    const FrontierResultView cut =
        select(database, query, cameraAt(-1000.0f), params);
    EXPECT_EQ(payloads(database, cut), (std::vector<UserPayload>{1}));
}

TEST(Tlas, ContributionCullPreservesLargeObjectThroughInternalNodes)
{
    SpatialDatabase database;
    constexpr uint32_t largePayload = 1042;
    for (uint32_t i = 0; i < 80; ++i)
    {
        InstanceDesc desc;
        desc.pos = float4::point(float(int(i % 10) - 5) * 8.0f,
                                 float(int(i / 10) - 4) * 8.0f, 0.0f);
        const bool large = i == 42;
        database.instantiate(node(1000 + i, 0.0f,
                                  box(large ? 10.0f : 0.1f)),
                             desc);
    }

    SpatialQuery query;
    SelectionParams params;
    params.minPix = 10.0f;
    const FrontierResultView cut =
        select(database, query, cameraAt(-1000.0f), params);
    EXPECT_EQ(payloads(database, cut),
              (std::vector<UserPayload>{largePayload}));
}

#ifdef FRONTIER_DEBUG_TOOLS
TEST(Tlas, DebugToolsExposeHealthAndDepthBoxes)
{
    SpatialDatabase database;
    constexpr uint32_t count = 80;
    for (uint32_t i = 0; i < count; ++i)
    {
        InstanceDesc desc;
        desc.pos = i + 1 == count
                       ? float4::point(10000.0f, 0.0f, 0.0f)
                       : float4::point(float(i % 10) * 4.0f, 0.0f,
                                       float(i / 10) * 4.0f);
        database.instantiate(node(1000 + i, 0.0f, box()), desc);
    }
    database.applyUpdates(0);

    const TlasDebugSummary summary = database.debugTlasSummary();
    EXPECT_FALSE(summary.buildRequired);
    EXPECT_EQ(summary.instanceCount, count);
    EXPECT_EQ(summary.instanceLaneCount, count);
    EXPECT_GT(summary.activeNodes, 1u);
    EXPECT_GT(summary.maxDepth, 1u);
    EXPECT_GT(summary.averageLaneOccupancy, 0.0f);
    EXPECT_LE(summary.averageLaneOccupancy, 1.0f);

    std::array<TlasDebugBox, 1> root{};
    ASSERT_EQ(database.debugTlasBoxes(0, root), 1u);
    EXPECT_EQ(root[0].kind, TlasDebugBoxKind::Root);
    EXPECT_EQ(root[0].depth, 0u);

    std::vector<TlasDebugBox> boxes(count + summary.activeNodes);
    const size_t total =
        database.debugTlasBoxes(summary.maxDepth, boxes);
    ASSERT_EQ(total, count);
    bool foundShallowLeaf = false;
    for (size_t i = 0; i < total; ++i)
    {
        EXPECT_EQ(boxes[i].kind, TlasDebugBoxKind::Instance);
        EXPECT_LE(boxes[i].depth, summary.maxDepth);
        foundShallowLeaf |= boxes[i].depth < summary.maxDepth;
    }
    EXPECT_TRUE(foundShallowLeaf);
}
#endif

TEST(Tlas, AdaptiveFatLeavesCollapseOnlyWhenTheCostModelWins)
{
    constexpr uint32_t count = 3u * kWide;

    SpatialDatabaseConfig baselineConfig;
    baselineConfig.tlasMaxLeafBlocks = 1;
    SpatialDatabase baseline(baselineConfig);
    SpatialDatabaseConfig fatConfig;
    fatConfig.tlasMaxLeafBlocks = 4;
    SpatialDatabase fat(fatConfig);
    for (uint32_t i = 0; i < count; ++i)
    {
        baseline.instantiate(node(1000 + i, 0.0f, box()));
        fat.instantiate(node(1000 + i, 0.0f, box()));
    }
    baseline.applyUpdates(0);
    fat.applyUpdates(0);

    EXPECT_EQ(TestAccess::tlasFatLeafCount(baseline), 0u);
    EXPECT_EQ(TestAccess::tlasMaxDepth(baseline), 2u);
    EXPECT_EQ(TestAccess::tlasFatLeafCount(fat), 1u);
    EXPECT_EQ(TestAccess::tlasLeafBlockCount(fat), 3u);
    EXPECT_EQ(TestAccess::tlasMaxLeafBlocks(fat), 3u);
    EXPECT_EQ(TestAccess::tlasMaxDepth(fat), 1u);
    EXPECT_EQ(TestAccess::tlasNodeCount(fat) + 1u,
              TestAccess::tlasNodeCount(baseline));

    Camera camera = cameraAt(-1000.0f);
    camera.viewMask = 1u; // bypass the all-population overview shortcut
    SpatialQuery baselineQuery;
    SpatialQuery fatQuery;
    std::vector<UserPayload> baselinePayloads =
        payloads(baseline, select(baseline, baselineQuery, camera));
    std::vector<UserPayload> fatPayloads =
        payloads(fat, select(fat, fatQuery, camera));
    std::sort(baselinePayloads.begin(), baselinePayloads.end());
    std::sort(fatPayloads.begin(), fatPayloads.end());
    EXPECT_EQ(fatPayloads, baselinePayloads);

    SpatialDatabase spread(fatConfig);
    for (uint32_t i = 0; i < count; ++i)
    {
        InstanceDesc desc;
        desc.pos = float4::point(float(i) * 100.0f, 0.0f, 0.0f);
        spread.instantiate(node(2000 + i, 0.0f, box()), desc);
    }
    spread.applyUpdates(0);
    EXPECT_EQ(TestAccess::tlasFatLeafCount(spread), 0u);
    EXPECT_EQ(TestAccess::tlasMaxDepth(spread), 2u);
}

TEST(Tlas, AdaptiveFatLeavesSupportAFullFinalLevel)
{
    constexpr uint32_t count = kWide * kWide;
    SpatialDatabaseConfig config;
    config.tlasMaxLeafBlocks = kWide;
    SpatialDatabase database(config);
    for (uint32_t i = 0; i < count; ++i)
        database.instantiate(node(2500 + i, 0.0f, box()));
    database.applyUpdates(0);

    EXPECT_EQ(TestAccess::tlasFatLeafCount(database), 1u);
    EXPECT_EQ(TestAccess::tlasLeafBlockCount(database), kWide);
    EXPECT_EQ(TestAccess::tlasMaxLeafBlocks(database), kWide);
    EXPECT_EQ(TestAccess::tlasMaxDepth(database), 1u);

    Camera camera = cameraAt(-1000.0f);
    camera.viewMask = 1u;
    SpatialQuery query;
    EXPECT_EQ(select(database, query, camera).size(), count);
}

TEST(Tlas, FatLeafChainsSurviveMoveRemoveInsertAndLayoutRebuild)
{
    SpatialDatabaseConfig config;
    config.tlasMaxLeafBlocks = 4;
    SpatialDatabase database(config);
    constexpr uint32_t count = 3u * kWide - 1u;
    std::vector<InstanceHandle> handles;
    for (uint32_t i = 0; i < count; ++i)
        handles.push_back(
            database.instantiate(node(3000 + i, 0.0f, box())));
    database.applyUpdates(0);
    ASSERT_EQ(TestAccess::tlasFatLeafCount(database), 1u);

    Camera camera = cameraAt(-1000.0f);
    camera.viewMask = 1u;
    SpatialQuery query;
    EXPECT_EQ(select(database, query, camera).size(), count);

    database.moveInstance(
        handles[kWide + 1u],
        Transform{float4::point(100.0f, 0.0f, 0.0f), 1.0f});
    database.applyUpdates(kUnlimitedTlasMaintenance);
    EXPECT_EQ(select(database, query, camera).size(), count);

    // Empty the head block while later blocks remain linked and visible.
    for (uint32_t i = 0; i < kWide; ++i)
        database.removeInstance(handles[i]);
    database.applyUpdates(kUnlimitedTlasMaintenance);
    EXPECT_EQ(select(database, query, camera).size(), count - kWide);
    EXPECT_EQ(TestAccess::tlasFatLeafCount(database), 1u);

    const InstanceHandle inserted =
        database.instantiate(node(9999, 0.0f, box()));
    database.applyUpdates(kUnlimitedTlasMaintenance);
    EXPECT_EQ(select(database, query, camera).size(), count - kWide + 1u);
    EXPECT_FALSE(TestAccess::instanceBounds(database, inserted).isEmpty());

    database.optimize(OptimizationMode::TopologyAndLayout);
    EXPECT_EQ(select(database, query, camera).size(), count - kWide + 1u);
    EXPECT_GT(TestAccess::tlasFatLeafCount(database), 0u);
}

TEST(Tlas, StaleInstanceHandleCannotMoveReusedSlot)
{
    SpatialDatabase database;
    InstanceHandle stale = database.instantiate(node(1, 0.0f, box()));
    database.removeInstance(stale);
    InstanceHandle live = database.instantiate(node(2, 0.0f, box()));
    ASSERT_EQ(stale.id, live.id);
    ASSERT_NE(stale.generation, live.generation);

    database.moveInstance(stale,
                          Transform{float4::point(100, 0, 0), 1.0f});
    database.applyUpdates(0);
    EXPECT_NEAR(TestAccess::instanceBounds(database, live).center().x,
                0.0f, 0.001f);
}

TEST(Tlas, TopologyDriftIsAdvisoryUntilARebuildIsExplicit)
{
    SpatialDatabaseConfig config;
    config.tlasCountDrift = 0.0f;
    config.tlasEditFraction = 0.0f;
    config.tlasAreaDrift = 1000.0f;
    SpatialDatabase database(config);
    for (uint32_t i = 0; i < 16; ++i)
    {
        InstanceDesc desc;
        desc.pos = float4::point(float(i) * 4.0f, 0.0f, 0.0f);
        database.instantiate(node(100 + i, 0.0f, box()), desc);
    }
    EXPECT_TRUE(database.applyUpdates(0).requiredBuildPerformed);

    database.instantiate(
        node(999, 0.0f, box()),
        InstanceDesc{.pos = float4::point(80.0f, 0.0f, 0.0f)});
    const UpdateReport report = database.applyUpdates(0);
    EXPECT_FALSE(report.requiredBuildPerformed);
    EXPECT_TRUE(report.topologyRebuildRecommended);

    SpatialQuery query;
    EXPECT_EQ(query.selectFrontier(database, cameraAt(-1000.0f), {}).size(),
              17u);
#ifdef FRONTIER_DEBUG_TOOLS
    const TlasDebugSummary before = database.debugTlasSummary();
    EXPECT_FALSE(before.buildRequired);
    EXPECT_TRUE(before.topologyRebuildRecommended);
#endif

    database.optimize(OptimizationMode::TopologyAndLayout);
#ifdef FRONTIER_DEBUG_TOOLS
    const TlasDebugSummary after = database.debugTlasSummary();
    EXPECT_FALSE(after.buildRequired);
    EXPECT_FALSE(after.topologyRebuildRecommended);
#endif
}

TEST(Tlas, FirstPopulationAfterAnEmptyPublicationBuildsAtConfiguredQuality)
{
    SpatialDatabaseConfig config;
    config.tlasCountDrift = 0.0f;
    config.tlasEditFraction = 1000.0f;
    config.tlasAreaDrift = 1000.0f;
    SpatialDatabase database(config);
    EXPECT_TRUE(database.applyUpdates(0).requiredBuildPerformed);

    for (uint32_t i = 0; i < 16; ++i)
    {
        InstanceDesc desc;
        desc.pos = float4::point(float(i) * 4.0f, 0.0f, 0.0f);
        database.instantiate(node(300 + i, 0.0f, box()), desc);
    }
    const UpdateReport report = database.applyUpdates(0);
    EXPECT_TRUE(report.requiredBuildPerformed);
    EXPECT_FALSE(report.topologyRebuildRecommended);
#ifdef FRONTIER_DEBUG_TOOLS
    EXPECT_EQ(database.debugTlasSummary().rebuildBaselineInstances, 16u);
#endif
}

TEST(Tlas, TopologyOnlyOptimizationPreservesDenseLayout)
{
    SpatialDatabaseConfig config;
    config.tlasQuality = TlasQuality::BinnedSAH;
    config.tlasCountDrift = 0.0f;
    config.tlasEditFraction = 0.0f;
    config.tlasAreaDrift = 0.0f;
    SpatialDatabase database(config);

    std::vector<InstanceHandle> handles;
    for (uint32_t i = 0; i < 64; ++i)
    {
        InstanceDesc desc;
        desc.pos = float4::point(float(i) * 3.0f, 0.0f, 0.0f);
        handles.push_back(database.instantiate(node(500 + i, 0.0f, box()),
                                               desc));
    }
    database.applyUpdates(0);
    for (uint32_t i = 0; i < 64; i += 3)
        database.removeInstance(handles[i]);
    ASSERT_TRUE(database.applyUpdates(0).topologyRebuildRecommended);
    database.moveInstance(handles[1],
                          Transform{float4::point(500.0f, 0.0f, 0.0f), 1.0f});

    const size_t allocatedBefore = TestAccess::allocatedInstanceSlots(database);
    const uint32_t layoutBefore = TestAccess::instanceLayoutVersion(database);
    const uint32_t mappingBefore = TestAccess::instanceMappingVersion(database);
    const uint32_t frameBefore = database.frame();
    const InstanceId denseBefore = TestAccess::denseInstanceId(database,
                                                                handles[1]);

    database.optimize(OptimizationMode::TopologyOnly);

    EXPECT_EQ(TestAccess::allocatedInstanceSlots(database), allocatedBefore);
    EXPECT_GT(allocatedBefore, TestAccess::liveInstanceSlots(database));
    EXPECT_EQ(TestAccess::instanceLayoutVersion(database), layoutBefore);
    EXPECT_EQ(TestAccess::instanceMappingVersion(database), mappingBefore);
    EXPECT_EQ(database.frame(), frameBefore);
    EXPECT_EQ(TestAccess::denseInstanceId(database, handles[1]), denseBefore);
    EXPECT_TRUE(TestAccess::instanceBounds(database, handles[0]).isEmpty());
    EXPECT_NEAR(TestAccess::tlasLeafBounds(database, handles[1]).center().x,
                500.0f, 0.001f);
#ifdef FRONTIER_DEBUG_TOOLS
    const TlasDebugSummary summary = database.debugTlasSummary();
    EXPECT_EQ(summary.activeQuality, TlasQuality::SpatialBins);
    EXPECT_EQ(summary.configuredQuality, TlasQuality::BinnedSAH);
    EXPECT_EQ(summary.rebuildBaselineInstances,
              TestAccess::liveInstanceSlots(database));
    EXPECT_EQ(summary.maintenanceNodesPending, 0u);
    EXPECT_FALSE(summary.topologyRebuildRecommended);
#endif

    SpatialQuery query;
    EXPECT_EQ(query.selectFrontier(database, cameraAt(-1000.0f), {}).size(),
              TestAccess::liveInstanceSlots(database));
}

TEST(Tlas, EveryQualityTierReturnsTheSameVisibleSet)
{
    constexpr uint32_t count = 1200;
    std::vector<UserPayload> reference;
    std::vector<size_t> nodeCounts;
    for (const TlasQuality quality : {TlasQuality::SpatialBins,
                                      TlasQuality::Median,
                                      TlasQuality::MeanSplit,
                                      TlasQuality::BinnedSAH})
    {
        SpatialDatabaseConfig config;
        config.tlasQuality = quality;
        SpatialDatabase database(config);
        for (uint32_t i = 0; i < count; ++i)
        {
            InstanceDesc desc;
            desc.pos = float4::point(
                float(int((i * 37) % 101) - 50),
                float(int((i * 53) % 97) - 48), 0.0f);
            database.instantiate(node(1000 + i, 0.0f, box(0.1f)), desc);
        }

        SpatialQuery query;
        const std::vector<UserPayload> selected = payloads(
            database, select(database, query, cameraAt(-1000.0f)));
        nodeCounts.push_back(TestAccess::tlasNodeCount(database));
        ASSERT_EQ(selected.size(), count);
        if (reference.empty())
            reference = selected;
        else
            EXPECT_EQ(selected, reference);
    }
    ASSERT_EQ(nodeCounts.size(), 4u);
    EXPECT_LT(nodeCounts[0], nodeCounts[1]);
}

TEST(Tlas, MeanSplitUsesThePrincipalCovarianceDirectionAtTheMean)
{
    SpatialDatabaseConfig config;
    config.tlasQuality = TlasQuality::MeanSplit;
    SpatialDatabase database(config);

    // A thin rectangle rotated 45 degrees. Its principal direction is
    // (1, 1), so the root plane classifies by x + y (the local u coordinate),
    // rather than by either world axis independently.
    constexpr int uRadius = 15;
    constexpr int vRadius = 3;
    for (int u = -uRadius; u <= uRadius; ++u)
        for (int v = -vRadius; v <= vRadius; ++v)
        {
            InstanceDesc desc;
            desc.pos = float4::point(float(u + v), float(u - v), 0.0f);
            database.instantiate(
                node(uint32_t((u + uRadius) * (2 * vRadius + 1) +
                              (v + vRadius) + 1),
                     0.0f, box(0.1f)),
                desc);
        }
    database.applyUpdates(0);

    const std::vector<float4> first =
        TestAccess::tlasRootHalfInstanceCenters(database, false);
    const std::vector<float4> second =
        TestAccess::tlasRootHalfInstanceCenters(database, true);
    ASSERT_FALSE(first.empty());
    ASSERT_FALSE(second.empty());

    // Eigenvector sign is canonicalized, so the first half is the negative
    // side of the x + y plane. Points exactly on the theoretical plane may
    // land on either side after the numerical eigensolve.
    for (const float4 center : first)
        EXPECT_LE(center.x + center.y, 0.0f);
    for (const float4 center : second)
        EXPECT_GE(center.x + center.y, 0.0f);
#ifdef FRONTIER_DEBUG_TOOLS
    EXPECT_EQ(database.debugTlasSummary().activeQuality,
              TlasQuality::MeanSplit);
#endif
}

#ifdef FRONTIER_DEBUG_TOOLS
TEST(Tlas, TopologyOnlyOptimizationAvoidsCitySpanningNearLeafBounds)
{
    constexpr uint32_t side = 100;
    constexpr uint32_t count = side * side;
    SpatialDatabase database;
    for (uint32_t i = 0; i < count; ++i)
    {
        InstanceDesc desc;
        desc.pos = float4::point(
            float(int(i % side) - int(side / 2)) * 3.0f,
            float(int(i / side) - int(side / 2)) * 3.0f, 0.0f);
        database.instantiate(node(2000 + i, 0.0f, box(0.5f)), desc);
    }
    database.applyUpdates(0);
    database.optimize(OptimizationMode::TopologyOnly);

    const TlasDebugSummary summary = database.debugTlasSummary();
    ASSERT_EQ(summary.activeQuality, TlasQuality::SpatialBins);
    ASSERT_GT(summary.maxDepth, 2u);
    const uint32_t depth = summary.maxDepth - 1;
    std::vector<TlasDebugBox> boxes(count + summary.activeNodes);
    const size_t total = database.debugTlasBoxes(depth, boxes);
    ASSERT_LE(total, boxes.size());

    size_t internalCount = 0;
    for (size_t i = 0; i < total; ++i)
    {
        if (boxes[i].kind != TlasDebugBoxKind::Internal) continue;
        ++internalCount;
        const float4 extent = boxes[i].bounds.extent();
        EXPECT_LT(std::max(extent.x, extent.y), 100.0f);
    }
    EXPECT_GT(internalCount, 0u);
}
#endif

TEST(Tlas, CoincidentCentroidsStillBuildACompleteTree)
{
    constexpr uint32_t count = 128;
    for (const TlasQuality quality : {TlasQuality::SpatialBins,
                                      TlasQuality::Median,
                                      TlasQuality::MeanSplit,
                                      TlasQuality::BinnedSAH})
    {
        SpatialDatabaseConfig config;
        config.tlasQuality = quality;
        SpatialDatabase database(config);
        for (uint32_t i = 0; i < count; ++i)
            database.instantiate(node(1000 + i, 0.0f, box()));

        SpatialQuery query;
        const FrontierResultView result =
            select(database, query, cameraAt(-1000.0f));
        EXPECT_EQ(result.entries.size(), count);
    }
}
