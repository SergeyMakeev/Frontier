#include <gtest/gtest.h>

#include <array>

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

TEST(QueryCache, RetainsWholeInternalResultButStillFillsExternalSinks)
{
    SpatialDatabase database;
    const SubtreeHandle subtree =
        database.registerSubtree(makeLeafSubtree(1000));
    InstanceHandle moved;
    for (uint32_t i = 0; i < 32; ++i)
    {
        InstanceDesc desc;
        desc.pos = float4::point(float(i) * 2.0f, 0.0f, 0.0f);
        const InstanceHandle instance =
            instantiateFor(database, subtree, box(), 64.0f, desc);
        if (i == 0) moved = instance;
    }
    TestAccess::markAllNodesReady(database);
    database.applyUpdates();

    SpatialQuery query;
    const Camera camera = cameraAt(-1000.0f);
    const FrontierResultView first =
        query.selectFrontier(database, camera, {});
    const FrontierEntry* retained = first.shared.data();
    const FrontierResultView second =
        query.selectFrontier(database, camera, {});
    EXPECT_EQ(second.shared.data(), retained);
    EXPECT_EQ(query.reused(), 32u);

    database.moveInstance(
        moved, Transform{float4::point(0.0f, 0.25f, 0.0f), 1.0f});
    database.applyUpdates();
    const FrontierResultView patched =
        query.selectFrontier(database, camera, {});
    EXPECT_EQ(patched.shared.data(), retained);
    EXPECT_EQ(patched.shared.size(), 32u);
    EXPECT_EQ(query.reused(), 31u);
    EXPECT_EQ(query.walked(), 1u);

    std::array<FrontierEntry, 32> shared{};
    FrontierResultSink sink{Sink<FrontierEntry>{shared},
                            Sink<FrontierEntry>{{}},
                            Sink<FrontierEntry>{{}}};
    query.selectFrontier(database, camera, {}, sink);
    EXPECT_EQ(sink.shared.count(), shared.size());
    EXPECT_EQ(sink.shared.dropped(), 0u);

    const FrontierResultView afterExternal =
        query.selectFrontier(database, camera, {});
    EXPECT_EQ(afterExternal.shared.size(), shared.size());
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
