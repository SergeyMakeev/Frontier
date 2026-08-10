#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstring>
#include <vector>

#include "helpers.h"

using namespace frontier;
using namespace frontiertest;

TEST(Contracts, SerializedSubtreeRejectsCorruption)
{
    Subtree source = makeLeafSubtree(SubtreeKey{901}, 7);
    std::vector<std::byte> bytes(source.byteSize());
    std::memcpy(bytes.data(), source.data(), source.byteSize());
    bytes[0] ^= std::byte{0xff};
    EXPECT_THROW(Subtree::fromBytes(bytes.data(), bytes.size()),
                 std::logic_error);
}

TEST(Contracts, DuplicateKeysAreRejected)
{
    SpatialDatabase database;
    database.registerSubtree(makeLeafSubtree(SubtreeKey{902}, 1));
    EXPECT_THROW(
        database.registerSubtree(makeLeafSubtree(SubtreeKey{902}, 2)),
        std::logic_error);
}

TEST(Contracts, DefinitionCannotBeReleasedWhileMounted)
{
    SpatialDatabase database;
    const SubtreeKey key{903};
    SubtreeHandle subtree = database.registerSubtree(
        makeLeafSubtree(key, 1));
    InstanceHandle instance = instantiateFor(database, subtree, key, box(2));
    EXPECT_THROW(database.releaseSubtree(subtree), std::logic_error);
    database.removeInstance(instance);
    database.releaseSubtree(subtree);
    EXPECT_FALSE(database.isSubtree(subtree));
}

TEST(Contracts, OnlyExtendableNodesAcceptMounts)
{
    SpatialDatabase database;
    const SubtreeKey parentKey{904};
    const SubtreeKey childKey{905};
    SubtreeHandle parent = database.registerSubtree(
        makeLeafSubtree(parentKey, 10));
    SubtreeHandle child = database.registerSubtree(
        makeLeafSubtree(childKey, 20));
    instantiateFor(database, parent, parentKey, box(2));
    EXPECT_THROW(database.mountSubtree(handleOf(database, 10), child),
                 std::logic_error);
}

TEST(Contracts, FixedOutputReportsOverflow)
{
    SpatialDatabase database;
    for (uint32_t i = 0; i < 8; ++i)
        database.instantiate(node(i + 1, 0.0f, box()));
    database.applyUpdates();

    std::array<FrontierEntry, 3> shared{};
    FrontierResultSink sink{Sink<FrontierEntry>{shared},
                            Sink<FrontierEntry>{{}},
                            Sink<FrontierEntry>{{}}};
    SpatialQuery query;
    query.selectFrontier(database, cameraAt(), {}, sink);
    EXPECT_EQ(sink.shared.count(), shared.size());
    EXPECT_EQ(sink.shared.dropped(), 5u);
}

TEST(Contracts, HotTypesStayCompact)
{
    EXPECT_EQ(sizeof(NodeHandle), 8u);
    EXPECT_EQ(sizeof(InstanceHandle), 8u);
    EXPECT_EQ(sizeof(SubtreeHandle), 8u);
    EXPECT_EQ(sizeof(SubtreeInstanceHandle), 8u);
    EXPECT_EQ(sizeof(FrontierEntry), 12u);
    EXPECT_LE(TestAccess::definitionBytes(), 160u);
    EXPECT_LE(TestAccess::mountedStateBytes(), 64u);
    EXPECT_EQ(TestAccess::mountStampBytes(), 8u);
    EXPECT_EQ(TestAccess::mountResidencyBytes(), 8u);
    EXPECT_LE(TestAccess::instanceBytes(), 80u);
}
