#include <gtest/gtest.h>

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
    database.applyUpdates();
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
    database.applyUpdates();
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
    database.applyUpdates();
    EXPECT_NEAR(TestAccess::instanceBounds(database, live).center().x,
                0.0f, 0.001f);
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
