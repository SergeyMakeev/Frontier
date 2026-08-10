#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

#include "helpers.h"

using namespace frontier;
using namespace frontiertest;

TEST(SubtreeBuilder, BuildsCompleteSerializableBlob)
{
    const SubtreeKey houseKey{101};
    const SubtreeKey detailKey{102};
    SubtreeBuilder builder(houseKey);
    const auto coarse = builder.createNode(node(10, 32.0f, box(5.0f)));
    builder.createNode(coarse, node(11, 0.0f, box(2.0f,
                                                   float4::point(-2, 0, 0))));
    builder.createNode(coarse, node(12, 8.0f, box(2.0f,
                                                   float4::point(2, 0, 0)),
                                     detailKey,
                                     Transform{float4::point(2, 0, 0), 0.5f}));

    Subtree subtree = builder.build();
    ASSERT_TRUE(subtree.valid());
    EXPECT_EQ(subtree.key(), houseKey);
    EXPECT_EQ(subtree.nodeCount(), 3u);
    ASSERT_EQ(subtree.dependencies().size(), 1u);
    EXPECT_EQ(subtree.dependencies()[0], detailKey);
    EXPECT_TRUE(subtree.bounds().contains(box(5.0f)));

    Subtree copy = Subtree::fromBytes(subtree.data(), subtree.byteSize());
    EXPECT_EQ(copy.key(), houseKey);
    EXPECT_EQ(copy.nodeCount(), subtree.nodeCount());
    EXPECT_EQ(copy.byteSize(), subtree.byteSize());

    Subtree clone = subtree.clone();
    EXPECT_NE(clone.data(), subtree.data());
    EXPECT_EQ(clone.byteSize(), subtree.byteSize());

    Subtree borrowed = Subtree::borrow(subtree.data(), subtree.byteSize());
    EXPECT_EQ(borrowed.data(), subtree.data());
    EXPECT_EQ(borrowed.key(), houseKey);
}

TEST(SubtreeBuilder, RejectsMixedLocalAndMountedChildren)
{
    SubtreeBuilder builder(SubtreeKey{200});
    const auto expansion = builder.createNode(
        node(1, 8.0f, box(), SubtreeKey{201}));
    EXPECT_THROW(builder.createNode(expansion, node(2, 0.0f, box(0.5f))),
                 std::logic_error);
}

TEST(Assembly, ReusesOneHouseDefinitionAtManyCityNodes)
{
    const SubtreeKey houseKey{301};
    const SubtreeKey cityKey{302};
    SpatialDatabase database;

    SubtreeHandle house = database.registerSubtree(
        makeLeafSubtree(houseKey, 100, 1.0f));

    SubtreeBuilder cityBuilder(cityKey);
    cityBuilder.createNode(node(
        10, 8.0f, box(1.0f, float4::point(-3, 0, 0)), houseKey,
        Transform{float4::point(-3, 0, 0), 1.0f}));
    cityBuilder.createNode(node(
        11, 8.0f, box(1.0f, float4::point(3, 0, 0)), houseKey,
        Transform{float4::point(3, 0, 0), 1.0f}));
    SubtreeHandle city = database.registerSubtree(cityBuilder.build());

    const InstanceHandle instance = instantiateFor(
        database, city, cityKey, box(8.0f));
    const SubtreeInstanceHandle left =
        database.mountSubtree(handleOf(database, 10), house);
    const SubtreeInstanceHandle right =
        database.mountSubtree(handleOf(database, 11), house);

    EXPECT_TRUE(left.valid());
    EXPECT_TRUE(right.valid());
    EXPECT_NE(left.slot, right.slot);
    EXPECT_EQ(database.subtreeCount(), 2u);
    EXPECT_EQ(database.mountedSubtreeCount(), 3u);
    EXPECT_TRUE(database.hasMountedSubtree(instance.rootNode()));

    Transform leftTransform;
    Transform rightTransform;
    ASSERT_TRUE(database.tryGetNodeTransform(handleOf(database, 100),
                                             leftTransform));
    // Duplicate payloads deliberately have no identity semantics. The mount
    // handles remain distinct and both point at the same registered bytes.
    EXPECT_TRUE(database.isMounted(left));
    EXPECT_TRUE(database.isMounted(right));
}

TEST(Assembly, RequiresAuthoredTargetToMatch)
{
    SpatialDatabase database;
    const SubtreeKey expected{401};
    const SubtreeKey wrong{402};
    SubtreeHandle subtree = database.registerSubtree(
        makeLeafSubtree(wrong, 2));
    InstanceHandle instance = database.instantiate(
        node(1, 16.0f, box(4.0f), expected));
    EXPECT_THROW(database.mountSubtree(instance.rootNode(), subtree),
                 std::logic_error);
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
