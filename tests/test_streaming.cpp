#include <gtest/gtest.h>

#include <array>

#include "helpers.h"

using namespace frontier;
using namespace frontiertest;

TEST(Streaming, MountAndUnmountNestedSubtree)
{
    SpatialDatabase database;
    SubtreeHandle detail = database.registerSubtree(
        makeLeafSubtree(99));

    SubtreeBuilder parentBuilder;
    parentBuilder.createNode(node(20, 16.0f, box(2.0f), true));
    SubtreeHandle parent = database.registerSubtree(parentBuilder.build());
    InstanceHandle instance = instantiateFor(database, parent, box(4.0f));
    NodeHandle mountPoint = handleOf(database, 20);

    SubtreeInstanceHandle mounted = database.mountSubtree(mountPoint, detail);
    ASSERT_TRUE(mounted.valid());
    EXPECT_TRUE(database.hasMountedSubtree(mountPoint));
    EXPECT_EQ(database.mountedSubtreeCount(), 2u);

    database.unmountSubtree(mounted);
    EXPECT_FALSE(database.isMounted(mounted));
    EXPECT_FALSE(database.hasMountedSubtree(mountPoint));
    EXPECT_EQ(database.mountedSubtreeCount(), 1u);

    database.removeInstance(instance);
    EXPECT_EQ(database.mountedSubtreeCount(), 0u);
}

TEST(Streaming, NestedMountMustFitMountPointBounds)
{
    SpatialDatabase database;
    SubtreeHandle detail = database.registerSubtree(
        makeLeafSubtree(99, 10.0f));
    SubtreeBuilder parentBuilder;
    parentBuilder.createNode(node(20, 16.0f, box(1.0f), true));
    SubtreeHandle parent = database.registerSubtree(parentBuilder.build());
    instantiateFor(database, parent, box(4.0f));
    EXPECT_THROW(database.mountSubtree(handleOf(database, 20), detail),
                 std::logic_error);
}

TEST(Streaming, QueryOwnsOptionalRetentionFeedback)
{
    SpatialDatabase database;
    SubtreeHandle subtree = database.registerSubtree(makeLodSubtree());
    instantiateFor(database, subtree, box(5.0f), 64.0f);

    SpatialQuery primary;
    primary.setMountUsageEnabled(true);
    SelectionParams params;
    params.threshold = 1.0f;
    (void)select(database, primary, cameraAt(-8), params);
    EXPECT_TRUE(primary.mountUsageEnabled());

    // The consumed query keeps its mounted tree young.
    CollectResult kept = database.collect(primary, 0, 1);
    EXPECT_EQ(kept.unmountedSubtrees, 0u);
    EXPECT_EQ(database.mountedSubtreeCount(), 1u);

    database.applyUpdates();
    CollectResult collected = database.collect(0, 1);
    EXPECT_EQ(collected.unmountedSubtrees, 1u);
    EXPECT_EQ(database.mountedSubtreeCount(), 0u);
}

TEST(Streaming, MixedDatabaseCollectionFailureDoesNotConsumeEarlierFeedback)
{
    SpatialDatabase first;
    const SubtreeHandle subtree =
        first.registerSubtree(makeLodSubtree());
    instantiateFor(first, subtree, box(5.0f), 64.0f);

    SpatialQuery firstQuery;
    firstQuery.setMountUsageEnabled(true);
    (void)select(first, firstQuery, cameraAt(-8),
                 SelectionParams{.threshold = 1.0f});

    SpatialDatabase second;
    second.instantiate(node(100, 0.0f, box()));
    SpatialQuery foreignQuery;
    (void)select(second, foreignQuery, cameraAt());

    std::array<SpatialQuery*, 2> mixed{&firstQuery, &foreignQuery};
    EXPECT_THROW(first.collect(mixed, 0, 1), std::logic_error);

    // The first query's touch must still be pending after the failed batch.
    // Consuming it now keeps the just-used mount alive for this frame.
    const CollectResult result = first.collect(firstQuery, 0, 1);
    EXPECT_EQ(result.unmountedSubtrees, 0u);
    EXPECT_EQ(first.mountedSubtreeCount(), 1u);
}

TEST(Streaming, ResetMountUsageDropsUnconsumedRetentionFeedback)
{
    SpatialDatabase database;
    const SubtreeHandle subtree =
        database.registerSubtree(makeLodSubtree());
    instantiateFor(database, subtree, box(5.0f), 64.0f);

    SpatialQuery query;
    query.setMountUsageEnabled(true);
    (void)select(database, query, cameraAt(-8),
                 SelectionParams{.threshold = 1.0f});
    query.resetMountUsage();
    database.applyUpdates();

    const CollectResult result = database.collect(query, 0, 1);
    EXPECT_EQ(result.unmountedSubtrees, 1u);
    EXPECT_EQ(database.mountedSubtreeCount(), 0u);
}

TEST(Streaming, CollectionDoesNotChangeSharedDefinitionNodeReadiness)
{
    SpatialDatabase database;
    SubtreeHandle subtree = database.registerSubtree(
        makeLeafSubtree(55));
    const InstanceHandle root = database.instantiate(
        node(1, 64.0f, box(2.0f), true));
    const SubtreeInstanceHandle firstPlacement =
        database.mountSubtree(root.rootNode(), subtree);
    const NodeHandle stale =
        TestAccess::nodeAt(database, firstPlacement, 1);
    database.markNodeReady(stale);
    database.applyUpdates();
    database.applyUpdates();

    CollectResult result = database.collect(0, 1);
    ASSERT_EQ(result.unmountedSubtrees, 1u);
    EXPECT_FALSE(database.isNodeReady(stale));

    const SubtreeInstanceHandle secondPlacement =
        database.mountSubtree(root.rootNode(), subtree);
    const NodeHandle replacement =
        TestAccess::nodeAt(database, secondPlacement, 1);
    EXPECT_TRUE(database.isNodeReady(replacement));
}

TEST(Streaming, StaleSubtreeInstanceHandleDoesNotAffectReplacement)
{
    SpatialDatabase database;
    SubtreeHandle subtree = database.registerSubtree(
        makeLeafSubtree(55));
    InstanceHandle first = database.instantiate(
        node(1, 16.0f, box(2.0f), true));
    SubtreeInstanceHandle old =
        database.mountSubtree(first.rootNode(), subtree);
    database.removeInstance(first);

    InstanceHandle second = database.instantiate(
        node(2, 16.0f, box(2.0f), true));
    SubtreeInstanceHandle replacement =
        database.mountSubtree(second.rootNode(), subtree);
    ASSERT_TRUE(replacement.valid());
    EXPECT_NE(old.generation, replacement.generation);
    database.unmountSubtree(old);
    EXPECT_TRUE(database.isMounted(replacement));
}
