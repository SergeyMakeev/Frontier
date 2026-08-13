#include <gtest/gtest.h>

#include "helpers.h"

using namespace frontier;
using namespace frontiertest;

TEST(QueryCache, ReusesStableFrontiers)
{
    SpatialDatabase database;
    const SubtreeHandle subtree =
        database.registerSubtree(makeLeafSubtree(1000));
    for (uint32_t i = 0; i < 128; ++i)
    {
        InstanceDesc desc;
        desc.pos = float4::point(float(i % 16) * 4.0f, 0,
                                 float(i / 16) * 4.0f);
        instantiateFor(database, subtree, box(), 64.0f, desc);
    }
    database.applyUpdates();
    SpatialQuery query;
    (void)query.selectFrontier(database, cameraAt(-100), {});
    EXPECT_GT(query.walked(), 0u);
    (void)query.selectFrontier(database, cameraAt(-100), {});
    EXPECT_GT(query.reused(), 0u);
}

TEST(QueryCache, MountedStateMutationInvalidatesRecordedCut)
{
    SpatialDatabase database;
    SubtreeHandle subtree = database.registerSubtree(makeLodSubtree());
    instantiateFor(database, subtree, box(5.0f));
    database.applyUpdates();

    SpatialQuery query;
    SelectionParams params;
    params.threshold = 0.01f;
    const Camera camera = cameraAt(-100);
    (void)query.selectFrontier(database, camera, params);
    (void)query.selectFrontier(database, camera, params);
    ASSERT_GT(query.reused(), 0u);

    database.markNodeReady(handleOf(database, 11));
    database.applyUpdates();
    (void)query.selectFrontier(database, camera, params);
    EXPECT_GT(query.walked(), 0u);
}

TEST(QueryCache, QueriesOwnIndependentDampingAndReuseState)
{
    SpatialDatabase database;
    const SubtreeHandle subtree =
        database.registerSubtree(makeLeafSubtree(2));
    instantiateFor(database, subtree, box(), 4.0f);
    database.applyUpdates();
    SpatialQuery mainView(4.0f);
    SpatialQuery shadowView(0.0f);

    (void)mainView.selectFrontier(database, cameraAt(-20), {});
    (void)shadowView.selectFrontier(database, cameraAt(-20), {});
    (void)mainView.selectFrontier(database, cameraAt(-20), {});
    EXPECT_GT(mainView.reused(), 0u);
    EXPECT_EQ(shadowView.reused(), 0u);

    mainView.reset();
    EXPECT_EQ(mainView.reused(), 0u);
    EXPECT_FLOAT_EQ(mainView.halfLife(), 4.0f);
}

TEST(QueryCache, DisableReuseKeepsTraversalCorrect)
{
    SpatialDatabase database;
    database.instantiate(node(9, 0.0f, box()));
    SpatialQuery query;
    query.setReuseEnabled(false);
    const FrontierResultView first = select(database, query, cameraAt());
    EXPECT_EQ(payloads(database, first), (std::vector<UserPayload>{9}));
    const FrontierResultView second = query.selectFrontier(database, cameraAt(), {});
    EXPECT_EQ(payloads(database, second), (std::vector<UserPayload>{9}));
    EXPECT_EQ(query.reused(), 0u);
}

TEST(QueryCache, TlasOnlyObjectsBypassEntryCaching)
{
    SpatialDatabase database;
    constexpr uint32_t count = 64;
    for (uint32_t i = 0; i < count; ++i)
        database.instantiate(node(1000 + i, 0.0f, box()));
    database.applyUpdates();

    SpatialQuery query;
    ASSERT_TRUE(query.reuseEnabled());
    const Camera camera = cameraAt(-1000.0f);
    for (uint32_t iteration = 0; iteration < 2; ++iteration)
    {
        const FrontierResultView result =
            query.selectFrontier(database, camera, {});
        EXPECT_EQ(result.shared.size(), count);
        EXPECT_EQ(query.reused(), 0u);
        EXPECT_EQ(query.walked(), count);
    }
}
