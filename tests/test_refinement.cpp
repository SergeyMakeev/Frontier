#include <gtest/gtest.h>

#include <array>

#include "helpers.h"

using namespace frontier;
using namespace frontiertest;

namespace {

SubtreeBytes makeRefinementTree()
{
    SubtreeBuilder builder;
    const auto coarse =
        builder.createNode(node(10, 64.0f, box(4.0f)));
    const auto left =
        builder.createNode(coarse, node(20, 32.0f, box(4.0f)));
    builder.createNode(left, node(21, 0.0f, box(4.0f)));
    builder.createNode(left, node(22, 0.0f, box(4.0f)));
    builder.createNode(coarse, node(30, 0.0f, box(4.0f)));
    return builder.build();
}

std::vector<UserPayload> groupPayloads(
    const SpatialDatabase& database,
    std::span<const FrontierEntry> entries)
{
    std::vector<UserPayload> result;
    for (const FrontierEntry& entry : entries)
        result.push_back(database.tryGetPayload(entry.nodeHandle));
    std::sort(result.begin(), result.end());
    return result;
}

struct RefinementScene
{
    SpatialDatabase database;
    InstanceHandle instance;
    SpatialQuery query;
    Camera camera = cameraAt(-8.0f);
    SelectionParams params{.threshold = 1.0f};

    RefinementScene()
    {
        const SubtreeHandle subtree =
            database.registerSubtree(makeRefinementTree());
        instance = instantiateFor(database, subtree, box(4.0f), 128.0f);
        database.applyUpdates(0);
    }

    FrontierResultView selectCurrent()
    {
        return query.selectFrontier(database, camera, params);
    }
};

} // namespace

TEST(Refinement, UnlimitedTraversalReturnsCompleteBreadthFirstGroups)
{
    RefinementScene scene;
    const FrontierResultView current = scene.selectCurrent();
    ASSERT_EQ(current.size(), 1u);
    EXPECT_EQ(scene.database.tryGetPayload(current.entries[0].nodeHandle), 1u);

    const FrontierRefinementView refinement =
        scene.query.computeFrontierRefinement(
            scene.database, current, SpatialQuery::UnlimitedDepth);

    ASSERT_EQ(refinement.groupCount(), 3u);
    EXPECT_TRUE(refinement.complete());
    EXPECT_FALSE(refinement.depthLimitReached());
    EXPECT_FALSE(refinement.nodeLimitReached());
    EXPECT_FLOAT_EQ(refinement.threshold(), scene.params.threshold);
    EXPECT_EQ(refinement.entries().size(), 5u);

    EXPECT_EQ(refinement.parent(0), current.entries[0].nodeHandle);
    EXPECT_EQ(refinement.depth(0), 1u);
    EXPECT_EQ(groupPayloads(scene.database, refinement.children(0)),
              (std::vector<UserPayload>{10}));

    const NodeHandle coarse = refinement.children(0)[0].nodeHandle;
    EXPECT_EQ(refinement.findGroup(coarse), 1u);
    EXPECT_EQ(refinement.depth(1), 2u);
    EXPECT_EQ(groupPayloads(scene.database, refinement.children(1)),
              (std::vector<UserPayload>{20, 30}));

    const auto secondLevel = refinement.children(1);
    const auto left = std::find_if(
        secondLevel.begin(), secondLevel.end(), [&](const FrontierEntry& entry)
        { return scene.database.tryGetPayload(entry.nodeHandle) == 20; });
    ASSERT_NE(left, secondLevel.end());
    EXPECT_EQ(refinement.findGroup(left->nodeHandle), 2u);
    EXPECT_EQ(refinement.depth(2), 3u);
    EXPECT_EQ(groupPayloads(scene.database, refinement.children(2)),
              (std::vector<UserPayload>{21, 22}));
    EXPECT_EQ(refinement.findGroup(NodeHandle{}), kInvalidIndex);

    EXPECT_EQ(refinedPayloads(scene.database, scene.query, current),
              (std::vector<UserPayload>{21, 22, 30}));
}

TEST(Refinement, FiniteDepthProvidesACompleteDecisionHorizon)
{
    RefinementScene scene;
    const FrontierResultView current = scene.selectCurrent();

    const FrontierRefinementView oneLevel =
        scene.query.computeFrontierRefinement(scene.database, current, 1);
    ASSERT_EQ(oneLevel.groupCount(), 1u);
    EXPECT_EQ(oneLevel.depth(0), 1u);
    EXPECT_EQ(groupPayloads(scene.database, oneLevel.children(0)),
              (std::vector<UserPayload>{10}));
    EXPECT_TRUE(oneLevel.depthLimitReached());
    EXPECT_FALSE(oneLevel.nodeLimitReached());
    EXPECT_FALSE(oneLevel.complete());

    const FrontierRefinementView twoLevels =
        scene.query.computeFrontierRefinement(scene.database, current, 2);
    ASSERT_EQ(twoLevels.groupCount(), 2u);
    EXPECT_EQ(twoLevels.depth(0), 1u);
    EXPECT_EQ(twoLevels.depth(1), 2u);
    EXPECT_TRUE(twoLevels.depthLimitReached());
    EXPECT_FALSE(twoLevels.complete());
}

TEST(Refinement, NodeLimitNeverSplitsAChildGroup)
{
    RefinementScene scene;
    const FrontierResultView current = scene.selectCurrent();

    const FrontierRefinementView none =
        scene.query.computeFrontierRefinement(
            scene.database, current, SpatialQuery::UnlimitedDepth, 0);
    EXPECT_TRUE(none.empty());
    EXPECT_TRUE(none.nodeLimitReached());
    EXPECT_FALSE(none.complete());

    const FrontierRefinementView firstOnly =
        scene.query.computeFrontierRefinement(
            scene.database, current, SpatialQuery::UnlimitedDepth, 2);
    ASSERT_EQ(firstOnly.groupCount(), 1u);
    EXPECT_EQ(firstOnly.children(0).size(), 1u);
    EXPECT_EQ(firstOnly.entries().size(), 1u);
    EXPECT_TRUE(firstOnly.nodeLimitReached());

    const FrontierRefinementView firstTwo =
        scene.query.computeFrontierRefinement(
            scene.database, current, SpatialQuery::UnlimitedDepth, 3);
    ASSERT_EQ(firstTwo.groupCount(), 2u);
    EXPECT_EQ(firstTwo.children(0).size(), 1u);
    EXPECT_EQ(firstTwo.children(1).size(), 2u);
    EXPECT_EQ(firstTwo.entries().size(), 3u);
    EXPECT_TRUE(firstTwo.nodeLimitReached());

    const FrontierRefinementView exact =
        scene.query.computeFrontierRefinement(
            scene.database, current, SpatialQuery::UnlimitedDepth, 5);
    EXPECT_EQ(exact.groupCount(), 3u);
    EXPECT_EQ(exact.entries().size(), 5u);
    EXPECT_TRUE(exact.complete());
}

TEST(Refinement, MountedBoundaryUsesTheMountPointAsGroupParent)
{
    SpatialDatabase database;

    SubtreeBuilder ownerBuilder;
    ownerBuilder.createNode(node(20, 64.0f, box(4.0f), true));
    const SubtreeHandle owner =
        database.registerSubtree(ownerBuilder.build());

    SubtreeBuilder detailBuilder;
    detailBuilder.createNode(node(31, 0.0f, box(2.0f)));
    detailBuilder.createNode(node(32, 0.0f, box(2.0f)));
    const SubtreeHandle detail =
        database.registerSubtree(detailBuilder.build());

    const InstanceHandle instance = database.instantiate(
        node(1, 128.0f, box(4.0f), true));
    const SubtreeInstanceHandle ownerPlacement =
        database.mountSubtree(instance.rootNode(), owner);
    const NodeHandle mountPoint =
        TestAccess::nodeAt(database, ownerPlacement, 1);
    ASSERT_TRUE(database.mountSubtree(mountPoint, detail).valid());
    database.applyUpdates(0);

    SpatialQuery query;
    const SelectionParams params{.threshold = 1.0f};
    const FrontierResultView current =
        query.selectFrontier(database, cameraAt(-8.0f), params);
    const FrontierRefinementView refinement =
        query.computeFrontierRefinement(
            database, current, SpatialQuery::UnlimitedDepth);

    ASSERT_EQ(refinement.groupCount(), 2u);
    EXPECT_EQ(groupPayloads(database, refinement.children(0)),
              (std::vector<UserPayload>{20}));
    EXPECT_EQ(refinement.parent(1), mountPoint);
    EXPECT_EQ(groupPayloads(database, refinement.children(1)),
              (std::vector<UserPayload>{31, 32}));
}

TEST(Refinement, ACurrentCutAlreadyFinerThanThresholdIsNotCoarsened)
{
    SpatialDatabase database;
    const SubtreeHandle subtree =
        database.registerSubtree(makeLodSubtree());
    instantiateFor(database, subtree, box(5.0f), 64.0f);
    database.markNodeReady(handleOf(database, 11));
    database.markNodeReady(handleOf(database, 12));
    database.applyUpdates(0);

    SpatialQuery query;
    const SelectionParams params{.threshold = 1.0f};
    const FrontierResultView current =
        query.selectFrontier(database, cameraAt(-20000.0f), params);
    ASSERT_EQ(payloads(database, current),
              (std::vector<UserPayload>{11, 12}));

    const FrontierRefinementView refinement =
        query.computeFrontierRefinement(
            database, current, SpatialQuery::UnlimitedDepth);
    EXPECT_TRUE(refinement.empty());
    EXPECT_TRUE(refinement.complete());
    EXPECT_EQ(refinedPayloads(database, query, current),
              (std::vector<UserPayload>{11, 12}));
}

TEST(Refinement, CompleteFixedAndOwningResultsCanBeAnalyzed)
{
    RefinementScene scene;

    std::array<FrontierEntry, 1> storage{};
    Sink<FrontierEntry> fixed{storage};
    scene.query.selectFrontier(scene.database, scene.camera, scene.params,
                               fixed);
    ASSERT_FALSE(fixed.overflowed());
    const FrontierResultView fixedCurrent{
        std::span<const FrontierEntry>{storage.data(), fixed.count()}};
    EXPECT_EQ(scene.query
                  .computeFrontierRefinement(scene.database, fixedCurrent, 1)
                  .groupCount(),
              1u);

    FrontierResult owning;
    scene.query.selectFrontier(scene.database, scene.camera, scene.params,
                               owning);
    EXPECT_EQ(scene.query
                  .computeFrontierRefinement(scene.database, owning, 1)
                  .groupCount(),
              1u);
}

TEST(Refinement, VisibilityUsesPlacedOverlayBoundsFromTheRetainedView)
{
    SpatialDatabase database;
    SubtreeBuilder builder;
    const auto coarse = builder.createNode(node(10, 64.0f, box(30.0f)));
    builder.createNode(
        coarse, node(11, 0.0f,
                     box(1.0f, float4::point(-20.0f, 0.0f, 0.0f))));
    builder.createNode(
        coarse, node(12, 0.0f,
                     box(1.0f, float4::point(20.0f, 0.0f, 0.0f))));
    const SubtreeHandle subtree =
        database.registerSubtree(builder.build());
    const InstanceHandle instance =
        instantiateFor(database, subtree, box(30.0f), 128.0f);
    database.applyUpdates(0);

    SpatialQuery query;
    const Camera camera =
        cameraAt(-8.0f, float4::point(-20.0f, 0.0f, 0.0f));
    const SelectionParams params{.threshold = 1.0f};
    FrontierResultView current =
        query.selectFrontier(database, camera, params);
    FrontierRefinementView refinement =
        query.computeFrontierRefinement(database, current, 2);
    ASSERT_EQ(refinement.groupCount(), 2u);
    EXPECT_EQ(groupPayloads(database, refinement.children(1)),
              (std::vector<UserPayload>{11}));

    const NodeHandle right =
        TestAccess::requireNode(database, instance, 12);
    database.setNodeBounds(
        instance, right,
        box(1.0f, float4::point(-18.0f, 0.0f, 0.0f)));
    database.applyUpdates(0);

    current = query.selectFrontier(database, camera, params);
    refinement = query.computeFrontierRefinement(database, current, 2);
    ASSERT_EQ(refinement.groupCount(), 2u);
    EXPECT_EQ(groupPayloads(database, refinement.children(1)),
              (std::vector<UserPayload>{11, 12}));
}

TEST(Refinement, UnmountedBoundariesRemainIndependentAcrossPlacements)
{
    SpatialDatabase database;
    SubtreeBuilder builder;
    builder.createNode(node(20, 64.0f, box(4.0f), true));
    const SubtreeHandle subtree =
        database.registerSubtree(builder.build());
    const InstanceHandle first =
        instantiateFor(database, subtree, box(4.0f), 128.0f);
    const InstanceHandle second = instantiateFor(
        database, subtree, box(4.0f), 128.0f,
        InstanceDesc{.pos = float4::point(6.0f, 0.0f, 0.0f)});
    database.applyUpdates(0);

    SpatialQuery query;
    const FrontierResultView current = query.selectFrontier(
        database, cameraAt(-12.0f, float4::point(3.0f, 0.0f, 0.0f)),
        SelectionParams{.threshold = 1.0f});
    ASSERT_EQ(current.size(), 2u);
    const FrontierRefinementView refinement =
        query.computeFrontierRefinement(
            database, current, SpatialQuery::UnlimitedDepth);

    ASSERT_EQ(refinement.groupCount(), 2u);
    EXPECT_TRUE(refinement.complete());
    EXPECT_NE(refinement.children(0)[0].nodeHandle,
              refinement.children(1)[0].nodeHandle);
    EXPECT_EQ(database.tryGetPayload(refinement.children(0)[0].nodeHandle),
              20u);
    EXPECT_EQ(database.tryGetPayload(refinement.children(1)[0].nodeHandle),
              20u);
    EXPECT_EQ(refinement.findGroup(refinement.children(0)[0].nodeHandle),
              kInvalidIndex);
    EXPECT_EQ(refinement.parent(0), first.rootNode());
    EXPECT_EQ(refinement.parent(1), second.rootNode());
}

TEST(Refinement, OrientedInstancesUseTheRetainedLocalView)
{
    SpatialDatabase database;
    SubtreeBuilder builder;
    const auto coarse = builder.createNode(node(10, 64.0f, box(30.0f)));
    builder.createNode(
        coarse, node(11, 0.0f,
                     box(1.0f, float4::point(-20.0f, 0.0f, 0.0f))));
    builder.createNode(
        coarse, node(12, 0.0f,
                     box(1.0f, float4::point(20.0f, 0.0f, 0.0f))));
    const SubtreeHandle subtree =
        database.registerSubtree(builder.build());
    instantiateFor(
        database, subtree, box(30.0f), 128.0f,
        InstanceDesc{.yaw = YawRotation{0.0f, 1.0f}});
    database.applyUpdates(0);

    SpatialQuery query;
    const Camera camera =
        cameraAt(-12.0f, float4::point(0.0f, 0.0f, -20.0f));
    const FrontierResultView current = query.selectFrontier(
        database, camera, SelectionParams{.threshold = 1.0f});
    const FrontierRefinementView refinement =
        query.computeFrontierRefinement(database, current, 2);

    ASSERT_EQ(refinement.groupCount(), 2u);
    EXPECT_EQ(groupPayloads(database, refinement.children(1)),
              (std::vector<UserPayload>{11}));
}

TEST(Refinement, ComputationDoesNotChangeSelectionAccounting)
{
    RefinementScene scene;
    scene.selectCurrent();
    const FrontierResultView current = scene.selectCurrent();
    const uint32_t reused = scene.query.reused();
    const uint32_t walked = scene.query.walked();
    const SelectionStats stats = scene.query.lastSelectionStats();
    const NodeHandle coarse = handleOf(scene.database, 10);
    ASSERT_FALSE(scene.database.isNodeReady(coarse));

    const FrontierRefinementView refinement =
        scene.query.computeFrontierRefinement(scene.database, current, 2);
    EXPECT_FALSE(refinement.empty());
    EXPECT_EQ(scene.query.reused(), reused);
    EXPECT_EQ(scene.query.walked(), walked);
    EXPECT_EQ(scene.query.lastSelectionStats().instancesVisited,
              stats.instancesVisited);
    EXPECT_EQ(scene.query.lastSelectionStats().nodesVisited,
              stats.nodesVisited);
    EXPECT_FALSE(scene.database.isNodeReady(coarse));
}

TEST(Refinement, CachedCurrentStillGetsFreshRefinementErrors)
{
    RefinementScene scene;
    scene.database.markNodeReady(handleOf(scene.database, 10));
    scene.database.applyUpdates(0);
    FrontierResultView current = scene.query.selectFrontier(
        scene.database, cameraAt(-100.0f), scene.params);
    ASSERT_EQ(payloads(scene.database, current),
              (std::vector<UserPayload>{10}));
    FrontierRefinementView refinement =
        scene.query.computeFrontierRefinement(scene.database, current, 1);
    ASSERT_EQ(refinement.groupCount(), 1u);
    const uint8_t nearError = refinement.children(0)[0].errorCode();

    scene.database.moveInstance(
        scene.instance,
        Transform{float4::point(0.0f, 0.0f, 10.0f), 1.0f});
    scene.database.applyUpdates(0);
    current = scene.query.selectFrontier(
        scene.database, cameraAt(-100.0f), scene.params);
    ASSERT_EQ(scene.query.reused(), 1u);
    refinement =
        scene.query.computeFrontierRefinement(scene.database, current, 1);
    ASSERT_EQ(refinement.groupCount(), 1u);
    const uint8_t farError = refinement.children(0)[0].errorCode();
    EXPECT_GT(nearError, farError);
}

TEST(Refinement, RejectsInvalidHorizonOverflowAndStaleSelectionContext)
{
    RefinementScene scene;
    const FrontierResultView current = scene.selectCurrent();
    EXPECT_THROW(scene.query.computeFrontierRefinement(
                     scene.database, current, 0),
                 std::logic_error);

    SpatialQuery unrelated;
    EXPECT_THROW(unrelated.computeFrontierRefinement(
                     scene.database, current, 1),
                 std::logic_error);

    std::array<FrontierEntry, 0> noStorage{};
    Sink<FrontierEntry> overflow{noStorage};
    scene.query.selectFrontier(scene.database, scene.camera, scene.params,
                               overflow);
    const FrontierResultView incomplete{
        std::span<const FrontierEntry>{noStorage.data(), overflow.count()}};
    EXPECT_TRUE(overflow.overflowed());
    EXPECT_THROW(scene.query.computeFrontierRefinement(
                     scene.database, incomplete, 1),
                 std::logic_error);

    const FrontierResultView selected = scene.selectCurrent();
    scene.database.moveInstance(
        scene.instance,
        Transform{float4::point(0.25f, 0.0f, 0.0f), 1.0f});
    scene.database.applyUpdates(0);
    EXPECT_THROW(scene.query.computeFrontierRefinement(
                     scene.database, selected, 1),
                 std::logic_error);
}
