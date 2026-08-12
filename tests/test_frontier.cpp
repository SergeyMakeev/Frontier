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

TEST(Frontier, ReadyLeavesBecomeCurrentCut)
{
    Scene scene;
    scene.database.markNodeReady(handleOf(scene.database, 11));
    scene.database.markNodeReady(handleOf(scene.database, 12));

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

TEST(Frontier, EqualPayloadValuesDoNotCoupleDefinitionNodes)
{
    SpatialDatabase database;
    SubtreeBuilder builder;
    builder.createNode(node(77, 0.0f, box(1.0f,
                                           float4::point(-2, 0, 0))));
    builder.createNode(node(77, 0.0f, box(1.0f,
                                           float4::point(2, 0, 0))));
    SubtreeHandle subtree = database.registerSubtree(builder.build());
    const InstanceHandle root = database.instantiate(
        node(1, 64.0f, box(4.0f), true));
    const SubtreeInstanceHandle placement =
        database.mountSubtree(root.rootNode(), subtree);
    const NodeHandle first = TestAccess::nodeAt(database, placement, 1);
    const NodeHandle second = TestAccess::nodeAt(database, placement, 2);

    database.markNodeReady(first);
    EXPECT_TRUE(database.isNodeReady(first));
    EXPECT_FALSE(database.isNodeReady(second));

    database.markNodeReady(second);

    SpatialQuery query;
    SelectionParams params;
    params.threshold = 1.0f;
    const FrontierResultView cut = select(database, query, cameraAt(-8), params);
    EXPECT_EQ(payloads(database, cut, false),
              (std::vector<UserPayload>{77, 77}));
}

TEST(Frontier, EqualPayloadValuesDoNotCoupleDefinitions)
{
    SpatialDatabase database;
    const SubtreeHandle first =
        database.registerSubtree(makeLeafSubtree(77));
    const SubtreeHandle second =
        database.registerSubtree(makeLeafSubtree(77));
    const InstanceHandle firstRoot = database.instantiate(
        node(1, 64.0f, box(2.0f), true));
    const SubtreeInstanceHandle firstPlacement =
        database.mountSubtree(firstRoot.rootNode(), first);
    const InstanceHandle secondRoot = database.instantiate(
        node(1, 64.0f, box(2.0f), true));
    const SubtreeInstanceHandle secondPlacement =
        database.mountSubtree(secondRoot.rootNode(), second);
    const NodeHandle firstNode =
        TestAccess::nodeAt(database, firstPlacement, 1);
    const NodeHandle secondNode =
        TestAccess::nodeAt(database, secondPlacement, 1);

    database.markNodeReady(firstNode);
    EXPECT_TRUE(database.isNodeReady(firstNode));
    EXPECT_FALSE(database.isNodeReady(secondNode));

    SpatialQuery query;
    SelectionParams params{.threshold = 1.0f};
    FrontierResultView cut = select(database, query, cameraAt(-8), params);
    EXPECT_EQ(payloads(database, cut, false),
              (std::vector<UserPayload>{1, 77}));

    database.markNodeReady(secondNode);
    cut = select(database, query, cameraAt(-8), params);
    EXPECT_EQ(payloads(database, cut, false),
              (std::vector<UserPayload>{77, 77}));
}

TEST(Frontier, ReadinessAppliesToExistingAndFuturePlacements)
{
    SpatialDatabase database;
    SubtreeHandle subtree = database.registerSubtree(makeLeafSubtree(55));

    InstanceHandle firstRoot = database.instantiate(
        node(1, 16.0f, box(2.0f), true));
    const SubtreeInstanceHandle firstPlacement =
        database.mountSubtree(firstRoot.rootNode(), subtree);
    const NodeHandle firstNode =
        TestAccess::nodeAt(database, firstPlacement, 1);
    database.markNodeReady(firstNode);
    InstanceHandle secondRoot = database.instantiate(
        node(2, 16.0f, box(2.0f), true));
    const SubtreeInstanceHandle secondPlacement =
        database.mountSubtree(secondRoot.rootNode(), subtree);
    const NodeHandle secondNode =
        TestAccess::nodeAt(database, secondPlacement, 1);
    EXPECT_TRUE(database.isNodeReady(secondNode));

    SpatialQuery query;
    SelectionParams params{.threshold = 1.0f};
    FrontierResultView ready = select(database, query, cameraAt(-8), params);
    EXPECT_EQ(payloads(database, ready, false),
              (std::vector<UserPayload>{55, 55}));

    database.markNodeUnavailable(secondNode);
    FrontierResultView unavailable =
        select(database, query, cameraAt(-8), params);
    EXPECT_EQ(payloads(database, unavailable, false),
              (std::vector<UserPayload>{1, 2}));
}

TEST(Frontier, NestedCoverageTracksSharedReadiness)
{
    SpatialDatabase database;

    SubtreeBuilder parentBuilder;
    parentBuilder.createNode(node(20, 16.0f, box(2.0f), true));
    const SubtreeHandle parent =
        database.registerSubtree(parentBuilder.build());
    const SubtreeHandle detail =
        database.registerSubtree(makeLeafSubtree(99));

    const InstanceHandle root = database.instantiate(
        node(1, 64.0f, box(4.0f), true));
    database.mountSubtree(root.rootNode(), parent);
    const NodeHandle parentNode = handleOf(database, 20);
    const SubtreeInstanceHandle detailPlacement =
        database.mountSubtree(parentNode, detail);
    const NodeHandle detailNode =
        TestAccess::nodeAt(database, detailPlacement, 1);
    database.markNodeReady(detailNode);

    SpatialQuery query;
    SelectionParams params{.threshold = 1.0f};
    const auto currentPayloads = [&]
    {
        return payloads(database,
                        select(database, query, cameraAt(-8), params), false);
    };

    EXPECT_EQ(currentPayloads(), (std::vector<UserPayload>{99}));

    database.markNodeUnavailable(detailNode);
    EXPECT_EQ(currentPayloads(), (std::vector<UserPayload>{1}));

    database.markNodeReady(parentNode);
    EXPECT_EQ(currentPayloads(), (std::vector<UserPayload>{20}));

    database.markNodeReady(detailNode);
    EXPECT_EQ(currentPayloads(), (std::vector<UserPayload>{99}));

    database.markNodeUnavailable(parentNode);
    EXPECT_EQ(currentPayloads(), (std::vector<UserPayload>{99}));
}

TEST(Frontier, ReleasedDefinitionsDiscardReadiness)
{
    SpatialDatabase database;
    const SubtreeHandle released =
        database.registerSubtree(makeLeafSubtree(66));
    const InstanceHandle oldRoot = database.instantiate(
        node(1, 64.0f, box(2.0f), true));
    const SubtreeInstanceHandle oldPlacement =
        database.mountSubtree(oldRoot.rootNode(), released);
    const NodeHandle stale =
        TestAccess::nodeAt(database, oldPlacement, 1);
    database.markNodeReady(stale);
    database.removeInstance(oldRoot);
    database.releaseSubtree(released);

    const SubtreeHandle replacement =
        database.registerSubtree(makeLeafSubtree(66));
    const InstanceHandle newRoot = database.instantiate(
        node(1, 64.0f, box(2.0f), true));
    const SubtreeInstanceHandle newPlacement =
        database.mountSubtree(newRoot.rootNode(), replacement);
    const NodeHandle replacementNode =
        TestAccess::nodeAt(database, newPlacement, 1);
    EXPECT_FALSE(database.isNodeReady(replacementNode));
    database.markNodeReady(stale);
    EXPECT_FALSE(database.isNodeReady(replacementNode));

    SpatialQuery query;
    SelectionParams params{.threshold = 1.0f};
    FrontierResultView cut = select(database, query, cameraAt(-8), params);
    EXPECT_EQ(payloads(database, cut, false),
              (std::vector<UserPayload>{1}));

    database.markNodeReady(replacementNode);
    cut = select(database, query, cameraAt(-8), params);
    EXPECT_EQ(payloads(database, cut, false),
              (std::vector<UserPayload>{66}));
}

TEST(Frontier, StaleNodeHandlesRemainSafeForPayloadLookup)
{
    Scene scene;
    const NodeHandle stale = handleOf(scene.database, 11);
    scene.database.removeInstance(scene.instance);
    UserPayload payload = 0;
    EXPECT_FALSE(scene.database.tryGetPayload(stale, payload));
}
