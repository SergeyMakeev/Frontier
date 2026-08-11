#include <gtest/gtest.h>

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

TEST(Streaming, CollectionReturnsResidentPayloads)
{
    SpatialDatabase database;
    SubtreeHandle subtree = database.registerSubtree(
        makeLeafSubtree(55));
    instantiateFor(database, subtree, box(2.0f));
    database.markPayloadResident(handleOf(database, 55));
    database.applyUpdates();
    database.applyUpdates();

    CollectResult result = database.collect(0, 1);
    ASSERT_EQ(result.unmountedSubtrees, 1u);
    ASSERT_EQ(result.freedPayloads.size(), 1u);
    EXPECT_EQ(result.freedPayloads[0], 55u);
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
