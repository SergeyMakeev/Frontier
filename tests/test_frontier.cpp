#include <gtest/gtest.h>

#include "helpers.h"

using namespace frontier;
using namespace frontiertest;

namespace {

struct Scene
{
    SpatialDatabase database;
    InstanceHandle instance;

    Scene()
    {
        SubtreeHandle subtree = database.registerSubtree(makeLodSubtree());
        instance = instantiateFor(database, subtree, box(5.0f), 64.0f);
    }
};

} // namespace

TEST(Frontier, PermanentRootCoversMissingMountedPayloads)
{
    Scene scene;
    SpatialQuery query;
    SelectionParams params;
    params.threshold = 1.0f;
    const FrontierResultView cut =
        select(scene.database, query, cameraAt(-8.0f), params);

    EXPECT_EQ(payloads(scene.database, cut, false),
              (std::vector<UserPayload>{1}));
    EXPECT_EQ(payloads(scene.database, cut, true),
              (std::vector<UserPayload>{11, 12}));
}

TEST(Frontier, ResidentLeavesBecomeCurrentCut)
{
    Scene scene;
    scene.database.markPayloadResident(handleOf(scene.database, 11));
    scene.database.markPayloadResident(handleOf(scene.database, 12));

    SpatialQuery query;
    SelectionParams params;
    params.threshold = 1.0f;
    const FrontierResultView cut =
        select(scene.database, query, cameraAt(-8.0f), params);
    EXPECT_EQ(payloads(scene.database, cut, false),
              (std::vector<UserPayload>{11, 12}));
    EXPECT_EQ(payloads(scene.database, cut, true),
              (std::vector<UserPayload>{11, 12}));
}

TEST(Frontier, DistantInstanceSelectsTlasRootWithoutWalkingSubtree)
{
    Scene scene;
    SpatialQuery query;
    query.setReuseEnabled(false);
    SelectionParams params;
    params.threshold = 4.0f;
    const FrontierResultView cut =
        select(scene.database, query, cameraAt(-100000.0f), params);
    EXPECT_EQ(payloads(scene.database, cut),
              (std::vector<UserPayload>{1}));
}

TEST(Frontier, PayloadValuesAreDataNotIdentity)
{
    SpatialDatabase database;
    SubtreeBuilder builder;
    builder.createNode(node(77, 0.0f, box(1.0f,
                                           float4::point(-2, 0, 0))));
    builder.createNode(node(77, 0.0f, box(1.0f,
                                           float4::point(2, 0, 0))));
    SubtreeHandle subtree = database.registerSubtree(builder.build());
    instantiateFor(database, subtree, box(4.0f), 64.0f);

    SpatialQuery query;
    SelectionParams params;
    params.threshold = 1.0f;
    const FrontierResultView cut = select(database, query, cameraAt(-8), params);
    EXPECT_EQ(payloads(database, cut),
              (std::vector<UserPayload>{77, 77}));
}

TEST(Frontier, StaleNodeHandlesAreSafe)
{
    Scene scene;
    const NodeHandle stale = handleOf(scene.database, 11);
    scene.database.removeInstance(scene.instance);
    scene.database.markPayloadResident(stale);
    scene.database.markPayloadNonResident(stale);
    EXPECT_FALSE(scene.database.isPayloadResident(stale));
    UserPayload payload = 0;
    EXPECT_FALSE(scene.database.tryGetPayload(stale, payload));
}
