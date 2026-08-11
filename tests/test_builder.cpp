#include <gtest/gtest.h>

#include <cstddef>
#include <cstring>
#include <utility>

#include "frontier/detail/subtree_data.h"
#include "helpers.h"

using namespace frontier;
using namespace frontiertest;

TEST(SubtreeBuilder, BuildsTraversalReadySerializableBytes)
{
    SubtreeBuilder builder;
    const auto coarse = builder.createNode(node(10, 32.0f, box(5.0f)));
    builder.createNode(coarse, node(11, 0.0f, box(2.0f,
                                                   float4::point(-2, 0, 0))));
    builder.createNode(coarse, node(12, 8.0f, box(2.0f,
                                                   float4::point(2, 0, 0)),
                                     true));

    SubtreeBytes bytes = builder.build();
    ASSERT_FALSE(bytes.empty());
    EXPECT_EQ(reinterpret_cast<uintptr_t>(bytes.data()) %
                  kSubtreeByteAlignment,
              0u);
    detail::validateSubtreeBytes(bytes);
    const detail::SubtreeView view = detail::viewSubtreeBytes(bytes);
    EXPECT_EQ(view.nodeCount(), 3u);
    EXPECT_TRUE(view.isMountable(3));
    EXPECT_TRUE(view.bounds().contains(box(5.0f)));

    SubtreeBytes copy = bytes;
    EXPECT_NE(copy.data(), bytes.data());
    EXPECT_EQ(copy.size(), bytes.size());
    EXPECT_EQ(std::memcmp(copy.data(), bytes.data(), bytes.size()), 0);

    SpatialDatabase database;
    const size_t byteCount = bytes.size();
    const void* allocation = bytes.data();
    SubtreeHandle handle = database.registerSubtree(std::move(bytes));
    EXPECT_TRUE(handle.valid());
    EXPECT_TRUE(bytes.empty());
    EXPECT_EQ(TestAccess::definitionData(database, handle), allocation);
    EXPECT_EQ(copy.size(), byteCount);
}

TEST(SubtreeBuilder, RejectsMixedLocalAndMountedChildren)
{
    SubtreeBuilder builder;
    const auto mountable = builder.createNode(node(1, 8.0f, box(), true));
    EXPECT_THROW(builder.createNode(mountable, node(2, 0.0f, box(0.5f))),
                 std::logic_error);
}

TEST(Assembly, ReusesOneHouseDefinitionAtManyCityNodes)
{
    SpatialDatabase database;

    SubtreeHandle house = database.registerSubtree(
        makeLeafSubtree(100, 1.0f));

    SubtreeBuilder cityBuilder;
    cityBuilder.createNode(node(
        10, 8.0f, box(1.0f, float4::point(-3, 0, 0)), true));
    cityBuilder.createNode(node(
        11, 8.0f, box(1.0f, float4::point(3, 0, 0)), true));
    SubtreeHandle city = database.registerSubtree(cityBuilder.build());

    const InstanceHandle instance = instantiateFor(database, city, box(8.0f));
    const SubtreeInstanceHandle left = database.mountSubtree(
        handleOf(database, 10), house,
        Transform{float4::point(-3, 0, 0), 1.0f});
    const SubtreeInstanceHandle right = database.mountSubtree(
        handleOf(database, 11), house,
        Transform{float4::point(3, 0, 0), 1.0f});

    EXPECT_TRUE(left.valid());
    EXPECT_TRUE(right.valid());
    EXPECT_NE(left.slot, right.slot);
    EXPECT_EQ(database.subtreeCount(), 2u);
    EXPECT_EQ(database.mountedSubtreeCount(), 3u);
    EXPECT_TRUE(database.hasMountedSubtree(instance.rootNode()));

    Transform leftTransform;
    ASSERT_TRUE(database.tryGetNodeTransform(
        TestAccess::nodeAt(database, left, 1), leftTransform));
    EXPECT_FLOAT_EQ(leftTransform.pos.x, -3.0f);
    EXPECT_TRUE(database.isMounted(right));
}

TEST(Assembly, MountTargetIsChosenByHandleAtAssemblyTime)
{
    SpatialDatabase database;
    SubtreeHandle subtree = database.registerSubtree(makeLeafSubtree(2));
    InstanceHandle instance = database.instantiate(
        node(1, 16.0f, box(4.0f), true));
    EXPECT_TRUE(database.mountSubtree(instance.rootNode(), subtree).valid());
}

TEST(Assembly, SingleNodeLivesOnlyInTlas)
{
    SpatialDatabase database;
    InstanceHandle instance =
        database.instantiate(node(42, 0.0f, box(2.0f)));
    EXPECT_TRUE(instance.valid());
    EXPECT_TRUE(instance.rootNode().isTlasRoot());
    EXPECT_EQ(database.subtreeCount(), 0u);
    EXPECT_EQ(database.mountedSubtreeCount(), 0u);

    SpatialQuery query;
    const FrontierResultView cut = select(database, query, cameraAt());
    ASSERT_EQ(cut.shared.size(), 1u);
    UserPayload payload = 0;
    ASSERT_TRUE(database.tryGetPayload(cut.shared[0].nodeHandle, payload));
    EXPECT_EQ(payload, 42u);
}
