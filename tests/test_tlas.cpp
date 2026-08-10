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

TEST(Tlas, ContributionCullUsesRootError)
{
    SpatialDatabase database;
    database.instantiate(node(1, 1.0f, box()));
    SpatialQuery query;
    SelectionParams params;
    params.minPix = 100.0f;
    const FrontierResultView cut =
        select(database, query, cameraAt(-1000.0f), params);
    EXPECT_TRUE(cut.empty());
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
