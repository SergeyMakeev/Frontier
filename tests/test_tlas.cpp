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
    database.optimize();

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

TEST(Tlas, QualityDriftIsAdvisoryUntilOptimizeIsExplicit)
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
    EXPECT_TRUE(report.optimizeRecommended);

    SpatialQuery query;
    EXPECT_EQ(query.selectFrontier(database, cameraAt(-1000.0f), {}).size(),
              17u);
#ifdef FRONTIER_DEBUG_TOOLS
    const TlasDebugSummary before = database.debugTlasSummary();
    EXPECT_FALSE(before.buildRequired);
    EXPECT_TRUE(before.optimizeRecommended);
#endif

    database.optimize();
#ifdef FRONTIER_DEBUG_TOOLS
    const TlasDebugSummary after = database.debugTlasSummary();
    EXPECT_FALSE(after.buildRequired);
    EXPECT_FALSE(after.optimizeRecommended);
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
    EXPECT_FALSE(report.optimizeRecommended);
#ifdef FRONTIER_DEBUG_TOOLS
    EXPECT_EQ(database.debugTlasSummary().qualityBaselineInstances, 16u);
#endif
}

TEST(Tlas, EveryQualityTierReturnsTheSameVisibleSet)
{
    constexpr uint32_t count = 1200;
    std::vector<UserPayload> reference;
    std::vector<size_t> nodeCounts;
    for (const TlasQuality quality : {TlasQuality::Morton,
                                      TlasQuality::Median,
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
            database, select(database, query, cameraAt(-1000.0f)), false);
        nodeCounts.push_back(TestAccess::tlasNodeCount(database));
        ASSERT_EQ(selected.size(), count);
        if (reference.empty())
            reference = selected;
        else
            EXPECT_EQ(selected, reference);
    }
    ASSERT_EQ(nodeCounts.size(), 3u);
    EXPECT_LT(nodeCounts[0], nodeCounts[1]);
}

TEST(Tlas, CoincidentCentroidsStillBuildACompleteTree)
{
    SpatialDatabase database;
    constexpr uint32_t count = 128;
    for (uint32_t i = 0; i < count; ++i)
        database.instantiate(node(1000 + i, 0.0f, box()));

    SpatialQuery query;
    const FrontierResultView result =
        select(database, query, cameraAt(-1000.0f));
    EXPECT_EQ(result.shared.size(), count);
}
