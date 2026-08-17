#include <gtest/gtest.h>

#include <array>

#include "helpers.h"

using namespace frontier;
using namespace frontiertest;

namespace {

struct Scene
{
    SpatialDatabase database;
    InstanceHandle instance;

    Scene()
    {
        SubtreeHandle subtree = database.registerSubtree(makeLodSubtree());
        instance = instantiateFor(database, subtree, box(5.0f), 64.0f);
    }
};

} // namespace

TEST(Frontier, PermanentRootCoversMissingMountedPayloads)
{
    Scene scene;
    SpatialQuery query;
    SelectionParams params;
    params.threshold = 1.0f;
    const FrontierResultView cut =
        select(scene.database, query, cameraAt(-8.0f), params);

    EXPECT_EQ(payloads(scene.database, cut, false),
              (std::vector<UserPayload>{1}));
    EXPECT_EQ(payloads(scene.database, cut, true),
              (std::vector<UserPayload>{11, 12}));
}

TEST(Frontier, ReadyLeavesBecomeCurrentCut)
{
    Scene scene;
    scene.database.markNodeReady(handleOf(scene.database, 11));
    scene.database.markNodeReady(handleOf(scene.database, 12));

    SpatialQuery query;
    SelectionParams params;
    params.threshold = 1.0f;
    const FrontierResultView cut =
        select(scene.database, query, cameraAt(-8.0f), params);
    EXPECT_EQ(payloads(scene.database, cut, false),
              (std::vector<UserPayload>{11, 12}));
    EXPECT_EQ(payloads(scene.database, cut, true),
              (std::vector<UserPayload>{11, 12}));
}

TEST(Frontier, DistantInstanceSelectsTlasRootWithoutWalkingSubtree)
{
    Scene scene;
    SpatialQuery query;
    query.setReuseEnabled(false);
    SelectionParams params;
    params.threshold = 4.0f;
    const FrontierResultView cut =
        select(scene.database, query, cameraAt(-100000.0f), params);
    EXPECT_EQ(payloads(scene.database, cut),
              (std::vector<UserPayload>{1}));
}

TEST(Frontier, EqualPayloadValuesDoNotCoupleDefinitionNodes)
{
    SpatialDatabase database;
    SubtreeBuilder builder;
    builder.createNode(node(77, 0.0f, box(1.0f,
                                           float4::point(-2, 0, 0))));
    builder.createNode(node(77, 0.0f, box(1.0f,
                                           float4::point(2, 0, 0))));
    SubtreeHandle subtree = database.registerSubtree(builder.build());
    const InstanceHandle root = database.instantiate(
        node(1, 64.0f, box(4.0f), true));
    const SubtreeInstanceHandle placement =
        database.mountSubtree(root.rootNode(), subtree);
    const NodeHandle first = TestAccess::nodeAt(database, placement, 1);
    const NodeHandle second = TestAccess::nodeAt(database, placement, 2);

    database.markNodeReady(first);
    EXPECT_TRUE(database.isNodeReady(first));
    EXPECT_FALSE(database.isNodeReady(second));

    database.markNodeReady(second);

    SpatialQuery query;
    SelectionParams params;
    params.threshold = 1.0f;
    const FrontierResultView cut = select(database, query, cameraAt(-8), params);
    EXPECT_EQ(payloads(database, cut, false),
              (std::vector<UserPayload>{77, 77}));
}

TEST(Frontier, EqualPayloadValuesDoNotCoupleDefinitions)
{
    SpatialDatabase database;
    const SubtreeHandle first =
        database.registerSubtree(makeLeafSubtree(77));
    const SubtreeHandle second =
        database.registerSubtree(makeLeafSubtree(77));
    const InstanceHandle firstRoot = database.instantiate(
        node(1, 64.0f, box(2.0f), true));
    const SubtreeInstanceHandle firstPlacement =
        database.mountSubtree(firstRoot.rootNode(), first);
    const InstanceHandle secondRoot = database.instantiate(
        node(1, 64.0f, box(2.0f), true));
    const SubtreeInstanceHandle secondPlacement =
        database.mountSubtree(secondRoot.rootNode(), second);
    const NodeHandle firstNode =
        TestAccess::nodeAt(database, firstPlacement, 1);
    const NodeHandle secondNode =
        TestAccess::nodeAt(database, secondPlacement, 1);

    database.markNodeReady(firstNode);
    EXPECT_TRUE(database.isNodeReady(firstNode));
    EXPECT_FALSE(database.isNodeReady(secondNode));

    SpatialQuery query;
    SelectionParams params{.threshold = 1.0f};
    FrontierResultView cut = select(database, query, cameraAt(-8), params);
    EXPECT_EQ(payloads(database, cut, false),
              (std::vector<UserPayload>{1, 77}));

    database.markNodeReady(secondNode);
    cut = select(database, query, cameraAt(-8), params);
    EXPECT_EQ(payloads(database, cut, false),
              (std::vector<UserPayload>{77, 77}));
}

TEST(Frontier, ReadinessAppliesToExistingAndFuturePlacements)
{
    SpatialDatabase database;
    SubtreeHandle subtree = database.registerSubtree(makeLeafSubtree(55));

    InstanceHandle firstRoot = database.instantiate(
        node(1, 16.0f, box(2.0f), true));
    const SubtreeInstanceHandle firstPlacement =
        database.mountSubtree(firstRoot.rootNode(), subtree);
    const NodeHandle firstNode =
        TestAccess::nodeAt(database, firstPlacement, 1);
    database.markNodeReady(firstNode);
    InstanceHandle secondRoot = database.instantiate(
        node(2, 16.0f, box(2.0f), true));
    const SubtreeInstanceHandle secondPlacement =
        database.mountSubtree(secondRoot.rootNode(), subtree);
    const NodeHandle secondNode =
        TestAccess::nodeAt(database, secondPlacement, 1);
    EXPECT_TRUE(database.isNodeReady(secondNode));

    SpatialQuery query;
    SelectionParams params{.threshold = 1.0f};
    FrontierResultView ready = select(database, query, cameraAt(-8), params);
    EXPECT_EQ(payloads(database, ready, false),
              (std::vector<UserPayload>{55, 55}));

    database.markNodeUnavailable(secondNode);
    FrontierResultView unavailable =
        select(database, query, cameraAt(-8), params);
    EXPECT_EQ(payloads(database, unavailable, false),
              (std::vector<UserPayload>{1, 2}));
}

TEST(Frontier, NestedCoverageTracksSharedReadiness)
{
    SpatialDatabase database;

    SubtreeBuilder parentBuilder;
    parentBuilder.createNode(node(20, 16.0f, box(2.0f), true));
    const SubtreeHandle parent =
        database.registerSubtree(parentBuilder.build());
    const SubtreeHandle detail =
        database.registerSubtree(makeLeafSubtree(99));

    const InstanceHandle root = database.instantiate(
        node(1, 64.0f, box(4.0f), true));
    database.mountSubtree(root.rootNode(), parent);
    const NodeHandle parentNode = handleOf(database, 20);
    const SubtreeInstanceHandle detailPlacement =
        database.mountSubtree(parentNode, detail);
    const NodeHandle detailNode =
        TestAccess::nodeAt(database, detailPlacement, 1);
    database.markNodeReady(detailNode);

    SpatialQuery query;
    SelectionParams params{.threshold = 1.0f};
    const auto currentPayloads = [&]
    {
        return payloads(database,
                        select(database, query, cameraAt(-8), params), false);
    };

    EXPECT_EQ(currentPayloads(), (std::vector<UserPayload>{99}));

    database.markNodeUnavailable(detailNode);
    EXPECT_EQ(currentPayloads(), (std::vector<UserPayload>{1}));

    database.markNodeReady(parentNode);
    EXPECT_EQ(currentPayloads(), (std::vector<UserPayload>{20}));

    database.markNodeReady(detailNode);
    EXPECT_EQ(currentPayloads(), (std::vector<UserPayload>{99}));

    database.markNodeUnavailable(parentNode);
    EXPECT_EQ(currentPayloads(), (std::vector<UserPayload>{99}));
}

TEST(Frontier, CurrentCutPolicyChoosesAncestorOrDescendantFallback)
{
    SpatialDatabase database;
    SubtreeBuilder builder;

    // Matches docs/images/cuts: A is the permanent TLAS root and B..O are
    // definition nodes. High-error interior nodes refine at this camera; G
    // is the unavailable ideal choice whose ready descendants distinguish the
    // two current-cut policies.
    const auto b = builder.createNode(node(2, 64.0f, box(4.0f)));       // B
    builder.createNode(b, node(4, 0.0f, box(4.0f)));                    // D
    const auto e = builder.createNode(b, node(5, 32.0f, box(4.0f)));    // E
    builder.createNode(e, node(8, 0.0f, box(4.0f)));                    // H
    builder.createNode(e, node(9, 0.0f, box(4.0f)));                    // I
    builder.createNode(e, node(10, 0.0f, box(4.0f)));                   // J

    const auto c = builder.createNode(node(3, 64.0f, box(4.0f)));       // C
    builder.createNode(c, node(6, 0.0f, box(4.0f)));                    // F
    const auto g = builder.createNode(c, node(7, 0.0f, box(4.0f)));     // G
    builder.createNode(g, node(11, 0.0f, box(4.0f)));                   // K
    const auto l = builder.createNode(g, node(12, 0.0f, box(4.0f)));    // L
    builder.createNode(l, node(13, 0.0f, box(4.0f)));                   // M
    builder.createNode(l, node(14, 0.0f, box(4.0f)));                   // N
    builder.createNode(l, node(15, 0.0f, box(4.0f)));                   // O

    const SubtreeHandle definition =
        database.registerSubtree(builder.build());
    const InstanceHandle root = database.instantiate(
        node(1, 128.0f, box(4.0f), true));                              // A
    database.mountSubtree(root.rootNode(), definition);

    for (const UserPayload ready : {3u, 4u, 5u, 6u, 11u, 13u, 14u, 15u})
        database.markNodeReady(handleOf(database, ready));

    SpatialQuery query;
    SelectionParams params{.threshold = 1.0f};
    const Camera camera = cameraAt(-20.0f);

    FrontierResultView cut = select(database, query, camera, params);
    EXPECT_EQ(payloads(database, cut, true),
              (std::vector<UserPayload>{4, 6, 7, 8, 9, 10}));
    EXPECT_EQ(payloads(database, cut, false),
              (std::vector<UserPayload>{4, 5, 6, 11, 13, 14, 15}));

    // Establish a cache hit before changing only the policy.
    cut = select(database, query, camera, params);
    EXPECT_EQ(query.reused(), 1u);

    params.currentCutPolicy = CurrentCutPolicy::PreferReadyAncestors;
    cut = select(database, query, camera, params);
    EXPECT_EQ(payloads(database, cut, true),
              (std::vector<UserPayload>{4, 6, 7, 8, 9, 10}));
    EXPECT_EQ(payloads(database, cut, false),
              (std::vector<UserPayload>{3, 4, 5}));
    EXPECT_EQ(query.walked(), 1u);

    params.currentCutPolicy = CurrentCutPolicy::PreferReadyDescendants;
    cut = select(database, query, camera, params);
    EXPECT_EQ(payloads(database, cut, false),
              (std::vector<UserPayload>{4, 5, 6, 11, 13, 14, 15}));
    EXPECT_EQ(query.walked(), 1u);
}

TEST(Frontier, AncestorPolicyTracksCandidatesAcrossMountedSubtrees)
{
    SpatialDatabase database;

    SubtreeBuilder ownerBuilder;
    const auto coarse =
        ownerBuilder.createNode(node(20, 64.0f, box(4.0f)));       // C
    ownerBuilder.createNode(coarse, node(21, 0.0f, box(4.0f)));    // F
    ownerBuilder.createNode(coarse,
                            node(22, 32.0f, box(4.0f), true));     // G
    const SubtreeHandle owner =
        database.registerSubtree(ownerBuilder.build());
    const SubtreeHandle detail =
        database.registerSubtree(makeLeafSubtree(23, 4.0f));       // H

    const InstanceHandle root = database.instantiate(
        node(1, 128.0f, box(4.0f), true));
    const SubtreeInstanceHandle ownerPlacement =
        database.mountSubtree(root.rootNode(), owner);
    const NodeHandle mountPoint =
        TestAccess::nodeAt(database, ownerPlacement, 3);
    database.mountSubtree(mountPoint, detail);

    for (const UserPayload ready : {20u, 21u, 22u})
        database.markNodeReady(handleOf(database, ready));

    SpatialQuery query;
    SelectionParams params{
        .threshold = 1.0f,
        .currentCutPolicy = CurrentCutPolicy::PreferReadyAncestors,
    };
    const Camera camera = cameraAt(-20.0f);

    FrontierResultView cut = select(database, query, camera, params);
    EXPECT_EQ(payloads(database, cut, true),
              (std::vector<UserPayload>{21, 23}));
    EXPECT_EQ(payloads(database, cut, false),
              (std::vector<UserPayload>{21, 22}));

    // With G unavailable, its mounted ideal descendant must retreat to C.
    // C then replaces the ready sibling F as well.
    database.markNodeUnavailable(mountPoint);
    cut = select(database, query, camera, params);
    EXPECT_EQ(payloads(database, cut, true),
              (std::vector<UserPayload>{21, 23}));
    EXPECT_EQ(payloads(database, cut, false),
              (std::vector<UserPayload>{20}));
}

TEST(Frontier, MountedCoverageStaysPlacementLocal)
{
    SpatialDatabase database;

    SubtreeBuilder parentBuilder;
    parentBuilder.createNode(node(20, 16.0f, box(2.0f), true));
    const SubtreeHandle parent =
        database.registerSubtree(parentBuilder.build());
    const SubtreeHandle detail =
        database.registerSubtree(makeLeafSubtree(99));

    const InstanceHandle firstRoot = database.instantiate(
        node(1, 64.0f, box(4.0f), true));
    const InstanceHandle secondRoot = database.instantiate(
        node(2, 64.0f, box(4.0f), true));
    const SubtreeInstanceHandle firstParent =
        database.mountSubtree(firstRoot.rootNode(), parent);
    database.mountSubtree(secondRoot.rootNode(), parent);

    const NodeHandle firstParentNode =
        TestAccess::nodeAt(database, firstParent, 1);
    const SubtreeInstanceHandle mountedDetail =
        database.mountSubtree(firstParentNode, detail);
    const NodeHandle detailNode =
        TestAccess::nodeAt(database, mountedDetail, 1);
    database.markNodeReady(detailNode);

    SpatialQuery query;
    SelectionParams params{.threshold = 1.0f};
    const auto currentPayloads = [&]
    {
        std::vector<UserPayload> result = payloads(
            database, select(database, query, cameraAt(-8), params), false);
        std::sort(result.begin(), result.end());
        return result;
    };

    EXPECT_EQ(currentPayloads(), (std::vector<UserPayload>{2, 99}));

    database.unmountSubtree(mountedDetail);
    EXPECT_EQ(currentPayloads(), (std::vector<UserPayload>{1, 2}));

    database.markNodeReady(firstParentNode);
    EXPECT_EQ(currentPayloads(), (std::vector<UserPayload>{20, 20}));
}

TEST(Frontier, ReleasedDefinitionsDiscardReadiness)
{
    SpatialDatabase database;
    const SubtreeHandle released =
        database.registerSubtree(makeLeafSubtree(66));
    const InstanceHandle oldRoot = database.instantiate(
        node(1, 64.0f, box(2.0f), true));
    const SubtreeInstanceHandle oldPlacement =
        database.mountSubtree(oldRoot.rootNode(), released);
    const NodeHandle stale =
        TestAccess::nodeAt(database, oldPlacement, 1);
    database.markNodeReady(stale);
    database.removeInstance(oldRoot);
    database.releaseSubtree(released);

    const SubtreeHandle replacement =
        database.registerSubtree(makeLeafSubtree(66));
    const InstanceHandle newRoot = database.instantiate(
        node(1, 64.0f, box(2.0f), true));
    const SubtreeInstanceHandle newPlacement =
        database.mountSubtree(newRoot.rootNode(), replacement);
    const NodeHandle replacementNode =
        TestAccess::nodeAt(database, newPlacement, 1);
    EXPECT_FALSE(database.isNodeReady(replacementNode));
    database.markNodeReady(stale);
    EXPECT_FALSE(database.isNodeReady(replacementNode));

    SpatialQuery query;
    SelectionParams params{.threshold = 1.0f};
    FrontierResultView cut = select(database, query, cameraAt(-8), params);
    EXPECT_EQ(payloads(database, cut, false),
              (std::vector<UserPayload>{1}));

    database.markNodeReady(replacementNode);
    cut = select(database, query, cameraAt(-8), params);
    EXPECT_EQ(payloads(database, cut, false),
              (std::vector<UserPayload>{66}));
}

TEST(Frontier, StaleNodeHandlesRemainSafeForPayloadLookup)
{
    Scene scene;
    const NodeHandle stale = handleOf(scene.database, 11);
    scene.database.removeInstance(scene.instance);
    EXPECT_EQ(scene.database.tryGetPayload(stale), kInvalidPayload);
}

TEST(Frontier, BulkResolutionPreservesOrderMetadataAndStaleSafety)
{
    SpatialDatabase database;
    const SubtreeHandle subtree =
        database.registerSubtree(makeLeafSubtree(66));
    const InstanceHandle mountedRoot =
        instantiateFor(database, subtree, box(2.0f));
    const NodeHandle mountedNode = handleOf(database, 66);
    const InstanceHandle flatRoot =
        database.instantiate(node(77, 0.0f, box(1.0f)));

    const std::array<FrontierEntry, 2> first{
        FrontierEntry{mountedNode, uint8_t(17), 3},
        FrontierEntry{mountedNode, uint8_t(129), 4}};
    const std::array<FrontierEntry, 1> second{
        FrontierEntry{flatRoot.rootNode(), uint8_t(8), 5}};
    const FrontierCutView cut{first, second};
    std::array<ResolvedFrontierEntry, 3> output{};

    const std::span<ResolvedFrontierEntry> resolved =
        database.resolveFrontier(cut, output);
    ASSERT_EQ(resolved.size(), 3u);
    EXPECT_EQ(resolved[0].payload, UserPayload(66));
    EXPECT_EQ(resolved[0].instance(), 3u);
    EXPECT_EQ(resolved[0].errorCode(), 17u);
    EXPECT_EQ(resolved[1].payload, UserPayload(66));
    EXPECT_EQ(resolved[1].instance(), 4u);
    EXPECT_EQ(resolved[1].errorCode(), 129u);
    EXPECT_TRUE(resolved[1].overThreshold());
    EXPECT_EQ(resolved[2].payload, UserPayload(77));
    EXPECT_EQ(resolved[2].instance(), 5u);
    EXPECT_EQ(resolved[2].errorCode(), 8u);

    std::array<ResolvedFrontierEntry, 2> undersized{};
    EXPECT_TRUE(database.resolveFrontier(cut, undersized).empty());

    database.removeInstance(mountedRoot);
    database.removeInstance(flatRoot);
    const std::span<ResolvedFrontierEntry> stale =
        database.resolveFrontier(cut, output);
    ASSERT_EQ(stale.size(), 3u);
    EXPECT_EQ(stale[0].payload, kInvalidPayload);
    EXPECT_EQ(stale[1].payload, kInvalidPayload);
    EXPECT_EQ(stale[2].payload, kInvalidPayload);
}

TEST(Frontier, RenderQueryTracksCachedRebuildsAndApiSwitches)
{
    Scene scene;
    TestAccess::markAllNodesReady(scene.database);
    scene.database.applyUpdates();
    SpatialQuery query;
    SpatialQuery referenceQuery;
    const Camera camera = cameraAt(-8.0f);

    const auto expectExact = [&](const Camera& view)
    {
        const FrontierResultView handles =
            referenceQuery.selectFrontier(scene.database, view, {});
        std::vector<ResolvedFrontierEntry> expected(handles.currentSize());
        const std::span<ResolvedFrontierEntry> resolved =
            scene.database.resolveFrontier(handles.current(), expected);
        ASSERT_EQ(resolved.size(), expected.size());

        const RenderFrontierView render =
            query.selectRenderFrontier(scene.database, view, {});
        std::vector<ResolvedFrontierEntry> actual;
        actual.reserve(render.size());
        size_t runEntries = 0;
        for (const RenderFrontierRun run : render.runs())
        {
            ASSERT_LE(size_t(run.begin) + run.count,
                      render.storage().size());
            const std::span<const ResolvedFrontierEntry> entries =
                render[run];
            actual.insert(actual.end(), entries.begin(), entries.end());
            runEntries += entries.size();
        }
        EXPECT_EQ(runEntries, render.size());
        ASSERT_EQ(actual.size(), expected.size());

        const auto less = [](const ResolvedFrontierEntry& a,
                             const ResolvedFrontierEntry& b)
        {
            if (a.payload != b.payload) return a.payload < b.payload;
            return a.instanceAndError < b.instanceAndError;
        };
        std::sort(actual.begin(), actual.end(), less);
        std::sort(expected.begin(), expected.end(), less);
        for (size_t i = 0; i < expected.size(); ++i)
        {
            EXPECT_EQ(actual[i].payload, expected[i].payload);
            EXPECT_EQ(actual[i].instanceAndError,
                      expected[i].instanceAndError);
        }
    };

    expectExact(camera);
    expectExact(camera);

    const NodeHandle detail = handleOf(scene.database, 12);
    scene.database.markNodeUnavailable(detail);
    expectExact(camera);

    // A handle-only call deliberately invalidates the retained renderer view.
    // The next render query must rebuild it rather than expose stale payloads.
    scene.database.markNodeReady(detail);
    const FrontierResultView handlesOnly =
        query.selectFrontier(scene.database, cameraAt(-7.5f), {});
    EXPECT_FALSE(handlesOnly.empty());
    expectExact(camera);

    // Uncached/all-direct selection rebuilds the complete output every call.
    query.setReuseEnabled(false);
    expectExact(camera);
    expectExact(cameraAt(-7.0f));

    query.reset();
    expectExact(camera);
}
