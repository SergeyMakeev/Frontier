#include <gtest/gtest.h>

#include <array>
#include <limits>

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

TEST(Motion, LargeBoundsOverlayPromotesFromSparseToDense)
{
    SpatialDatabase database;
    SubtreeBuilder builder;
    constexpr uint32_t count = detail::kMaxChildren;
    for (uint32_t i = 0; i < count; ++i)
        builder.createNode(node(1000 + i, 0.0f, box()));
    const SubtreeHandle definition =
        database.registerSubtree(builder.build());
    const InstanceHandle instance = database.instantiate(
        node(1, 16.0f, box(), true));
    const SubtreeInstanceHandle placement =
        database.mountSubtree(instance.rootNode(), definition);
    ASSERT_TRUE(placement.valid());

    const NodeHandle first = TestAccess::nodeAt(database, placement, 1);
    database.setNodeBounds(instance, first, box(0.75f));
    database.flushBounds();
    EXPECT_TRUE(TestAccess::overlayIsSparse(database, instance, first));

    SpatialQuery query;
    const FrontierResultView sparseResult =
        select(database, query, cameraAt(-1000.0f));
    EXPECT_EQ(sparseResult.idealSize(), count);

    // Cross the one-sixteenth promotion threshold. Block zero was patched
    // above; BVH4 has twice as many half-sized blocks as BVH8.
    const uint32_t wideBlocks = (count + kWide - 1) / kWide;
    const uint32_t patchesToPromote =
        wideBlocks / 16 + 1;
    for (uint32_t block = 1; block < patchesToPromote; ++block)
        database.setNodeBounds(
            instance,
            TestAccess::nodeAt(database, placement, 1 + block * kWide),
            box(0.75f));
    database.flushBounds();
    EXPECT_FALSE(TestAccess::overlayIsSparse(database, instance, first));

    const FrontierResultView denseResult =
        select(database, query, cameraAt(-1000.0f));
    EXPECT_EQ(denseResult.idealSize(), count);
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

TEST(Motion, RejectsUnrepresentableAccumulatedMountTransform)
{
    SpatialDatabase database;

    SubtreeBuilder detailBuilder;
    detailBuilder.createNode(node(30, 0.0f, box(1.0e-20f)));
    const SubtreeHandle detail =
        database.registerSubtree(detailBuilder.build());

    SubtreeBuilder ownerBuilder;
    ownerBuilder.createNode(node(20, 8.0f, box(1.0f), true));
    const SubtreeHandle owner =
        database.registerSubtree(ownerBuilder.build());

    const InstanceHandle root = database.instantiate(
        node(1, 64.0f, box(1.0e30f), true));
    ASSERT_TRUE(database.mountSubtree(
        root.rootNode(), owner,
        Transform{float4::point(0, 0, 0), 1.0e30f}).valid());

    EXPECT_THROW(database.mountSubtree(
                     handleOf(database, 20), detail,
                     Transform{float4::point(0, 0, 0), 1.0e10f}),
                 std::logic_error);
    EXPECT_EQ(database.mountedSubtreeCount(), 1u);
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

TEST(Motion, MotionGroupDuplicateHandlesUseTheFinalCallerPosition)
{
    SpatialDatabase database;
    const InstanceHandle instance =
        database.instantiate(node(1, 0.0f, box()));
    const std::array<InstanceHandle, 3> handles{
        instance, instance, instance};
    SpatialDatabase::MotionGroup group(handles);
    const std::array<float4, 3> positions{
        float4::point(10, 0, 0),
        float4::point(20, 0, 0),
        float4::point(30, 0, 0)};

    database.moveInstances(group, positions);
    database.applyUpdates();
    EXPECT_NEAR(TestAccess::instanceBounds(database, instance).center().x,
                30.0f, 0.001f);
}

TEST(Motion, MotionGroupResetReplacesTheCallerCohort)
{
    SpatialDatabase database;
    const InstanceHandle first =
        database.instantiate(node(1, 0.0f, box()));
    const InstanceHandle second =
        database.instantiate(node(2, 0.0f, box()));

    const std::array<InstanceHandle, 1> firstCohort{first};
    SpatialDatabase::MotionGroup group(firstCohort);
    const std::array<float4, 1> firstPosition{
        float4::point(10, 0, 0)};
    database.moveInstances(group, firstPosition);

    const std::array<InstanceHandle, 1> secondCohort{second};
    group.reset(secondCohort);
    EXPECT_EQ(group.size(), 1u);
    const std::array<float4, 1> secondPosition{
        float4::point(20, 0, 0)};
    database.moveInstances(group, secondPosition);
    database.applyUpdates();

    EXPECT_NEAR(TestAccess::instanceBounds(database, first).center().x,
                10.0f, 0.001f);
    EXPECT_NEAR(TestAccess::instanceBounds(database, second).center().x,
                20.0f, 0.001f);
}

TEST(Motion, RejectsInvalidTransformsWithoutCorruptingTlas)
{
    SpatialDatabase database;
    NodeDesc invalidFlags = node(1, 0.0f, box());
    invalidFlags.flags = 1u << 31;
    EXPECT_THROW(database.instantiate(invalidFlags), std::logic_error);

    InstanceDesc invalidPosition;
    invalidPosition.pos.x = std::numeric_limits<float>::infinity();
    EXPECT_THROW(database.instantiate(node(2, 0.0f, box()), invalidPosition),
                 std::logic_error);

    InstanceHandle instance =
        database.instantiate(node(3, 0.0f, box()));
    const AABB before = TestAccess::instanceBounds(database, instance);
    Transform invalidMove;
    invalidMove.pos.x = std::numeric_limits<float>::infinity();
    EXPECT_THROW(database.moveInstance(instance, invalidMove),
                 std::logic_error);
    const AABB after = TestAccess::instanceBounds(database, instance);
    EXPECT_FLOAT_EQ(after.mn.x, before.mn.x);
    EXPECT_FLOAT_EQ(after.mx.x, before.mx.x);

    const float tooSmall = std::numeric_limits<float>::denorm_min();
    EXPECT_THROW(database.instantiate(
                     node(4, 0.0f, box()),
                     InstanceDesc{.scale = tooSmall}),
                 std::logic_error);
    EXPECT_THROW(database.moveInstance(
                     instance,
                     Transform{float4::point(0, 0, 0), tooSmall}),
                 std::logic_error);

    const SubtreeHandle detail =
        database.registerSubtree(makeLeafSubtree(5));
    const InstanceHandle mountable = database.instantiate(
        node(6, 8.0f, box(), true));
    EXPECT_THROW(database.mountSubtree(
                     mountable.rootNode(), detail,
                     Transform{float4::point(0, 0, 0), tooSmall}),
                 std::logic_error);
}

TEST(Motion, RejectsBoundsThatOverflowTheInstanceTransform)
{
    SpatialDatabase database;
    const InstanceHandle instance = database.instantiate(
        node(1, 0.0f, box(1.0e-10f)),
        InstanceDesc{.scale = 1.0e30f});
    const AABB before = TestAccess::instanceBounds(database, instance);

    database.setNodeBounds(instance, instance.rootNode(), box(1.0e20f));
    EXPECT_THROW(database.flushBounds(), std::logic_error);

    const AABB after =
        TestAccess::unflushedInstanceBounds(database, instance);
    EXPECT_FLOAT_EQ(after.mn.x, before.mn.x);
    EXPECT_FLOAT_EQ(after.mx.x, before.mx.x);
}

TEST(Motion, BoundsOverrideRequiresTheOwningInstance)
{
    SpatialDatabase database;
    const SubtreeHandle subtree =
        database.registerSubtree(makeLeafSubtree(10));
    const InstanceHandle first = instantiateFor(database, subtree, box(2.0f));
    const InstanceHandle second = instantiateFor(database, subtree, box(2.0f));
    const NodeHandle secondNode =
        TestAccess::requireNode(database, second, 10);

    EXPECT_THROW(database.setNodeBounds(first, secondNode, box(0.5f)),
                 std::logic_error);
    EXPECT_THROW(database.setNodeBounds(first, second.rootNode(), box(0.5f)),
                 std::logic_error);
    EXPECT_THROW(database.nodeBounds(first, secondNode), std::logic_error);
    EXPECT_EQ(database.overlayCount(), 0u);
}
