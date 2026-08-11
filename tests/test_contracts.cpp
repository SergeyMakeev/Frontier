#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <utility>

#include "helpers.h"

using namespace frontier;
using namespace frontiertest;

TEST(Contracts, SerializedSubtreeRejectsCorruption)
{
    SubtreeBytes bytes = makeLeafSubtree(7);
    bytes.data()[0] ^= std::byte{0xff};
    SpatialDatabase database;
    EXPECT_THROW(database.registerSubtree(std::move(bytes)), std::logic_error);
}

TEST(Contracts, DefinitionsHaveHandleIdentityNotContentKeys)
{
    SpatialDatabase database;
    SubtreeHandle first = database.registerSubtree(makeLeafSubtree(1));
    SubtreeHandle second = database.registerSubtree(makeLeafSubtree(1));
    EXPECT_NE(first.slot, second.slot);
}

TEST(Contracts, DefinitionCannotBeReleasedWhileMounted)
{
    SpatialDatabase database;
    SubtreeHandle subtree = database.registerSubtree(
        makeLeafSubtree(1));
    InstanceHandle instance = instantiateFor(database, subtree, box(2));
    EXPECT_THROW(database.releaseSubtree(subtree), std::logic_error);
    database.removeInstance(instance);
    database.releaseSubtree(subtree);
    EXPECT_FALSE(database.isSubtree(subtree));
}

TEST(Contracts, OnlyMountableNodesAcceptMounts)
{
    SpatialDatabase database;
    SubtreeHandle parent = database.registerSubtree(
        makeLeafSubtree(10));
    SubtreeHandle child = database.registerSubtree(
        makeLeafSubtree(20));
    instantiateFor(database, parent, box(2));
    EXPECT_THROW(database.mountSubtree(handleOf(database, 10), child),
                 std::logic_error);

    InstanceHandle flatRoot = database.instantiate(node(30, 4.0f, box(2)));
    EXPECT_THROW(database.mountSubtree(flatRoot.rootNode(), child),
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
