#include <gtest/gtest.h>

#include "helpers.h"

using namespace frontier;
using namespace frontiertest;

TEST(Streaming, MountAndUnmountNestedSubtree)
{
    const SubtreeKey detailKey{601};
    const SubtreeKey parentKey{602};
    SpatialDatabase database;
    SubtreeHandle detail = database.registerSubtree(
        makeLeafSubtree(detailKey, 99));

    SubtreeBuilder parentBuilder(parentKey);
    parentBuilder.createNode(node(20, 16.0f, box(2.0f), detailKey));
    SubtreeHandle parent = database.registerSubtree(parentBuilder.build());
    InstanceHandle instance = instantiateFor(database, parent, parentKey,
                                             box(4.0f));
    NodeHandle expansion = handleOf(database, 20);
    EXPECT_EQ(database.subtreeTarget(expansion), detailKey);

    SubtreeInstanceHandle mounted = database.mountSubtree(expansion, detail);
    ASSERT_TRUE(mounted.valid());
    EXPECT_TRUE(database.hasMountedSubtree(expansion));
    EXPECT_EQ(database.mountedSubtreeCount(), 2u);

    database.unmountSubtree(mounted);
    EXPECT_FALSE(database.isMounted(mounted));
    EXPECT_FALSE(database.hasMountedSubtree(expansion));
    EXPECT_EQ(database.mountedSubtreeCount(), 1u);

    database.removeInstance(instance);
    EXPECT_EQ(database.mountedSubtreeCount(), 0u);
}

TEST(Streaming, NestedMountMustFitExpansionBounds)
{
    const SubtreeKey detailKey{603};
    const SubtreeKey parentKey{604};
    SpatialDatabase database;
    SubtreeHandle detail = database.registerSubtree(
        makeLeafSubtree(detailKey, 99, 10.0f));
    SubtreeBuilder parentBuilder(parentKey);
    parentBuilder.createNode(node(20, 16.0f, box(1.0f), detailKey));
    SubtreeHandle parent = database.registerSubtree(parentBuilder.build());
    instantiateFor(database, parent, parentKey, box(4.0f));
    EXPECT_THROW(database.mountSubtree(handleOf(database, 20), detail),
                 std::logic_error);
}

TEST(Streaming, QueryOwnsOptionalRetentionFeedback)
{
    const SubtreeKey key{605};
    SpatialDatabase database;
    SubtreeHandle subtree = database.registerSubtree(makeLodSubtree(key));
    instantiateFor(database, subtree, key, box(5.0f), 64.0f);

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
    const SubtreeKey key{606};
    SpatialDatabase database;
    SubtreeHandle subtree = database.registerSubtree(
        makeLeafSubtree(key, 55));
    instantiateFor(database, subtree, key, box(2.0f));
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
    const SubtreeKey key{607};
    SpatialDatabase database;
    SubtreeHandle subtree = database.registerSubtree(
        makeLeafSubtree(key, 55));
    InstanceHandle first = database.instantiate(
        node(1, 16.0f, box(2.0f), key));
    SubtreeInstanceHandle old =
        database.mountSubtree(first.rootNode(), subtree);
    database.removeInstance(first);

    InstanceHandle second = database.instantiate(
        node(2, 16.0f, box(2.0f), key));
    SubtreeInstanceHandle replacement =
        database.mountSubtree(second.rootNode(), subtree);
    ASSERT_TRUE(replacement.valid());
    EXPECT_NE(old.generation, replacement.generation);
    database.unmountSubtree(old);
    EXPECT_TRUE(database.isMounted(replacement));
}
