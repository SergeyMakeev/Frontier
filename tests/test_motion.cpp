#include <gtest/gtest.h>

#include <array>

#include "helpers.h"

using namespace frontier;
using namespace frontiertest;

TEST(Motion, MovesTlasOwnedNodeWithoutSubtreeState)
{
    SpatialDatabase database;
    InstanceHandle instance =
        database.instantiate(node(7, 0.0f, box(1.0f)));
    database.moveInstance(
        instance, Transform{float4::point(100, 2, 3), 2.0f});
    database.applyUpdates();

    const AABB world = TestAccess::instanceBounds(database, instance);
    EXPECT_TRUE(world.contains(box(2.0f, float4::point(100, 2, 3))));
    EXPECT_EQ(database.mountedSubtreeCount(), 0u);
}

TEST(Motion, NodeBoundsUseCopyOnWritePerTopLevelInstance)
{
    SpatialDatabase database;
    SubtreeHandle subtree = database.registerSubtree(makeLodSubtree());
    InstanceHandle first = instantiateFor(database, subtree, box(5.0f));
    InstanceDesc shifted;
    shifted.pos = float4::point(100, 0, 0);
    InstanceHandle second = instantiateFor(database, subtree, box(5.0f),
                                           64.0f, shifted);

    NodeHandle firstLeaf = TestAccess::requireNode(database, first, 11);
    NodeHandle secondLeaf = TestAccess::requireNode(database, second, 11);
    const AABB authored = database.nodeBounds(second, secondLeaf);
    const AABB moved = box(1.0f, float4::point(20, 0, 0));
    database.setNodeBounds(first, firstLeaf, moved);
    database.flushBounds();

    EXPECT_TRUE(database.nodeBounds(first, firstLeaf).contains(moved));
    EXPECT_TRUE(database.nodeBounds(second, secondLeaf).contains(authored));
    EXPECT_FALSE(database.nodeBounds(second, secondLeaf).contains(moved));
    EXPECT_GT(database.overlayCount(), 0u);
}

TEST(Motion, MountedTransformsCompose)
{
    SpatialDatabase database;
    SubtreeHandle detail = database.registerSubtree(
        makeLeafSubtree(30));
    SubtreeBuilder parentBuilder;
    parentBuilder.createNode(node(
        20, 8.0f, box(2.0f, float4::point(5, 0, 0)), true));
    SubtreeHandle parent = database.registerSubtree(parentBuilder.build());

    Transform parentTransform{float4::point(10, 0, 0), 3.0f};
    instantiateFor(database, parent, box(40.0f), 64.0f, {},
                   parentTransform);
    database.mountSubtree(handleOf(database, 20), detail,
        Transform{float4::point(5, 0, 0), 2.0f});

    Transform transform;
    ASSERT_TRUE(database.tryGetNodeTransform(handleOf(database, 30), transform));
    EXPECT_FLOAT_EQ(transform.scale, 6.0f);
    EXPECT_FLOAT_EQ(transform.pos.x, 25.0f);
}

TEST(Motion, MotionGroupIgnoresStaleHandles)
{
    SpatialDatabase database;
    InstanceHandle a = database.instantiate(node(1, 0.0f, box()));
    InstanceHandle b = database.instantiate(node(2, 0.0f, box()));
    std::array<InstanceHandle, 2> handles{a, b};
    SpatialDatabase::MotionGroup group(handles);
    database.removeInstance(a);
    std::array<float4, 2> positions{
        float4::point(100, 0, 0), float4::point(20, 0, 0)};
    database.moveInstances(group, positions);
    database.applyUpdates();

    const AABB moved = TestAccess::instanceBounds(database, b);
    EXPECT_NEAR(moved.center().x, 20.0f, 0.001f);
}
