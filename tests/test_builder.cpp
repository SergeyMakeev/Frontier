#include <gtest/gtest.h>

#include <bit>
#include <cstddef>
#include <cstring>
#include <limits>
#include <utility>

#include "frontier/detail/subtree_data.h"
#include "helpers.h"

using namespace frontier;
using namespace frontiertest;

TEST(NodeDesc, ScalarBoundsRoundTripExactly)
{
    EXPECT_EQ(sizeof(ScalarAABB), 24u);
    EXPECT_EQ(alignof(ScalarAABB), alignof(float));
    EXPECT_EQ(sizeof(NodeDesc), sizeof(UserPayload) + 32u);
    EXPECT_EQ(alignof(NodeDesc), alignof(UserPayload));
    EXPECT_EQ(offsetof(NodeDesc, flags), sizeof(UserPayload) + 4u);
    EXPECT_EQ(offsetof(NodeDesc, bounds), sizeof(UserPayload) + 8u);

    ScalarAABB inverted = AABB::fromMinMax(float4::vec(0, 1, 0),
                                           float4::vec(1, 0, 1));
    EXPECT_TRUE(inverted.isEmpty());

    const AABB source = AABB::fromMinMax(
        float4::point(-0.0f, -12345.625f, 1.0e-20f),
        float4::point(98765.5f, 0.0f, 1.0e20f));
    const detail::PayloadWord payloadWord =
        sizeof(detail::PayloadWord) == 8
            ? static_cast<detail::PayloadWord>(0xfedcba9876543210ull)
            : static_cast<detail::PayloadWord>(0xfedcba98u);
    const UserPayload authoredPayload = detail::decodePayload(payloadWord);
    const NodeDesc desc{
        .payload = authoredPayload,
        .geometricError = 7.25f,
        .flags = NodeDesc::FlagMountable,
        .bounds = source,
    };
    const AABB decoded = desc.bounds;

    const auto expectSameBits = [](float actual, float expected)
    {
        EXPECT_EQ(std::bit_cast<uint32_t>(actual),
                  std::bit_cast<uint32_t>(expected));
    };
    expectSameBits(decoded.mn.x, source.mn.x);
    expectSameBits(decoded.mn.y, source.mn.y);
    expectSameBits(decoded.mn.z, source.mn.z);
    expectSameBits(decoded.mx.x, source.mx.x);
    expectSameBits(decoded.mx.y, source.mx.y);
    expectSameBits(decoded.mx.z, source.mx.z);
    EXPECT_EQ(desc.payload, authoredPayload);
    EXPECT_FLOAT_EQ(desc.geometricError, 7.25f);
    EXPECT_EQ(desc.flags, NodeDesc::FlagMountable);
    EXPECT_TRUE(desc.isMountable());

    ScalarAABB empty;
    EXPECT_TRUE(empty.isEmpty());
    const AABB decodedEmpty = empty;
    EXPECT_TRUE(decodedEmpty.isEmpty());
}

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
    const auto* header = reinterpret_cast<const detail::SubtreeHeader*>(
        bytes.data());
    EXPECT_EQ(header->branchingFactor, kWide);
    EXPECT_EQ(header->payloadBytes, sizeof(detail::PayloadWord));
    EXPECT_EQ(header->invalidPayloadWord,
              uint64_t(detail::invalidPayloadWord()));
    detail::validateSubtreeBytes(bytes);
    const detail::SubtreeView view = detail::viewSubtreeBytes(bytes);
    EXPECT_EQ(view.nodeCount(), 3u);
    EXPECT_TRUE(view.isMountable(3));
    EXPECT_TRUE(view.bounds().contains(box(5.0f)));
    EXPECT_EQ(view.parent(1), 0u);
    EXPECT_EQ(view.siblingOrdinal(1), 0u);
    EXPECT_EQ(view.parent(2), 1u);
    EXPECT_EQ(view.siblingOrdinal(2), 0u);
    EXPECT_EQ(view.parent(3), 1u);
    EXPECT_EQ(view.siblingOrdinal(3), 1u);
    EXPECT_TRUE(view.nodeBounds(1).contains(view.nodeBounds(2)));
    EXPECT_TRUE(view.nodeBounds(1).contains(view.nodeBounds(3)));
    const uint32_t rootBlock = view.wideOffset(0);
    const uint32_t childBlock = view.wideOffset(1);
    EXPECT_EQ(detail::blockZeroErrorLanes(view.blockMask_[rootBlock]), 0u);
    EXPECT_EQ(detail::blockZeroErrorLanes(view.blockMask_[childBlock]), 1u);
    EXPECT_EQ(view.leafLanes(childBlock), 1u);

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

TEST(SubtreeBuilder, AcceptsLargestNonReservedPayload)
{
    constexpr UserPayload payload = kInvalidPayload - 1;
    SpatialDatabase database;
    const SubtreeHandle definition =
        database.registerSubtree(makeLeafSubtree(payload));
    const InstanceHandle instance =
        instantiateFor(database, definition, box(2.0f));

    const UserPayload resolved = database.tryGetPayload(
        TestAccess::requireNode(database, instance, payload));
    EXPECT_EQ(resolved, payload);
}

TEST(SubtreeBuilder, RejectsReservedInvalidPayload)
{
    SubtreeBuilder builder;
    EXPECT_THROW(builder.createNode(
                     node(kInvalidPayload, 0.0f, box())),
                 std::logic_error);

    SpatialDatabase database;
    EXPECT_THROW(database.instantiate(
                     node(kInvalidPayload, 0.0f, box())),
                 std::logic_error);
}

TEST(SubtreeBuilder, RejectsReservedFlagsAndMalformedBounds)
{
    SubtreeBuilder oversized;
    EXPECT_THROW(oversized.reserve(detail::kMaxSubtreeNodes),
                 std::logic_error);

    SubtreeBuilder flagsBuilder;
    NodeDesc unknownFlags = node(1, 0.0f, box());
    unknownFlags.flags = 1u << 31;
    EXPECT_THROW(flagsBuilder.createNode(unknownFlags), std::logic_error);
    NodeDesc topLevelOnly = node(1, 0.0f, box());
    topLevelOnly.flags = NodeDesc::FlagYawInvariantBounds;
    EXPECT_THROW(flagsBuilder.createNode(topLevelOnly), std::logic_error);

    SubtreeBuilder boundsBuilder;
    NodeDesc malformed = node(2, 0.0f, box());
    malformed.bounds = AABB::fromMinMax(
        float4::point(-1.0f, 1.0f, -1.0f),
        float4::point(1.0f, -1.0f, 1.0f));
    boundsBuilder.createNode(malformed);
    EXPECT_THROW(boundsBuilder.build(), std::logic_error);
}

TEST(SubtreeBuilder, ContractFailuresDoNotPartiallyConsumeTheBuilder)
{
    SubtreeBuilder empty;
    EXPECT_THROW(empty.build(), std::logic_error);
    empty.createNode(node(1, 0.0f, box()));
    EXPECT_NO_THROW((void)empty.build());

    SubtreeBuilder roots;
    for (uint32_t i = 0; i < detail::kMaxChildren; ++i)
        roots.createNode(node(i + 1, 0.0f, box()));
    EXPECT_THROW(roots.createNode(
                     node(detail::kMaxChildren + 1, 0.0f, box())),
                 std::logic_error);
    const SubtreeBytes rootBytes = roots.build();
    EXPECT_EQ(detail::viewSubtreeBytes(rootBytes).nodeCount(),
              detail::kMaxChildren);

    SubtreeBuilder fanout;
    const auto parent = fanout.createNode(node(1, 1.0f, box()));
    for (uint32_t i = 0; i < detail::kMaxChildren; ++i)
        fanout.createNode(parent, node(i + 2, 0.0f, box()));
    EXPECT_THROW(fanout.createNode(
                     parent,
                     node(detail::kMaxChildren + 2, 0.0f, box())),
                 std::logic_error);
    const SubtreeBytes fanoutBytes = fanout.build();
    EXPECT_EQ(detail::viewSubtreeBytes(fanoutBytes).nodeCount(),
              detail::kMaxChildren + 1);
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
    const UserPayload payload =
        database.tryGetPayload(cut.shared[0].nodeHandle);
    EXPECT_EQ(payload, 42u);
}
