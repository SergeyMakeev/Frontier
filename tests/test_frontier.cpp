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

SubtreeBytes makeFullyRefinedReferenceSubtree(float leafError)
{
    SubtreeBuilder builder;
    const auto root = builder.createNode(node(10, 16.0f, box(4.0f)));
    UserPayload payload = 11;
    for (int z : {-1, 1})
        for (int x : {-1, 1})
        {
            const float4 center = float4::point(float(x) * 2.0f, 0.0f,
                                                float(z) * 2.0f);
            const auto interior = builder.createNode(
                root, node(payload++, 16.0f, box(2.0f, center)));
            for (int leafZ : {-1, 1})
                for (int leafX : {-1, 1})
                {
                    const float4 leafCenter =
                        center + float4::vec(float(leafX) * 0.75f, 0.0f,
                                             float(leafZ) * 0.75f);
                    builder.createNode(
                        interior,
                        node(payload++, leafError,
                             box(0.5f, leafCenter)));
                }
        }
    return builder.build();
}

} // namespace

TEST(Frontier, PermanentRootCoversMissingMountedPayloads)
{
    Scene scene;
    SpatialQuery query;
    SelectionParams params;
    params.threshold = 1.0f;
    const FrontierResultView cut =
        select(scene.database, query, cameraAt(-8.0f), params);

    EXPECT_EQ(payloads(scene.database, cut),
              (std::vector<UserPayload>{1}));
    EXPECT_EQ(refinedPayloads(scene.database, query, cut),
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
    EXPECT_EQ(payloads(scene.database, cut),
              (std::vector<UserPayload>{11, 12}));
    EXPECT_EQ(refinedPayloads(scene.database, query, cut),
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
    EXPECT_EQ(payloads(database, cut),
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
    EXPECT_EQ(payloads(database, cut),
              (std::vector<UserPayload>{1, 77}));

    database.markNodeReady(secondNode);
    cut = select(database, query, cameraAt(-8), params);
    EXPECT_EQ(payloads(database, cut),
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
    EXPECT_EQ(payloads(database, ready),
              (std::vector<UserPayload>{55, 55}));

    database.markNodeUnavailable(secondNode);
    FrontierResultView unavailable =
        select(database, query, cameraAt(-8), params);
    EXPECT_EQ(payloads(database, unavailable),
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
                        select(database, query, cameraAt(-8), params));
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
    // is the unavailable threshold-target choice whose ready descendants distinguish the
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
    EXPECT_EQ(payloads(database, cut),
              (std::vector<UserPayload>{4, 5, 6, 11, 13, 14, 15}));

    // Establish a cache hit before changing only the policy.
    cut = select(database, query, camera, params);
    EXPECT_EQ(query.reused(), 1u);

    params.currentCutPolicy = CurrentCutPolicy::PreferReadyAncestors;
    cut = select(database, query, camera, params);
    EXPECT_EQ(refinedPayloads(database, query, cut),
              (std::vector<UserPayload>{4, 6, 7, 8, 9, 10}));
    EXPECT_EQ(payloads(database, cut),
              (std::vector<UserPayload>{3, 4, 5}));
    EXPECT_EQ(query.walked(), 1u);

    params.currentCutPolicy = CurrentCutPolicy::PreferReadyDescendants;
    cut = select(database, query, camera, params);
    EXPECT_EQ(payloads(database, cut),
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
    EXPECT_EQ(refinedPayloads(database, query, cut),
              (std::vector<UserPayload>{21, 23}));
    EXPECT_EQ(payloads(database, cut),
              (std::vector<UserPayload>{21, 22}));

    // With G unavailable, its mounted target descendant must retreat to C.
    // C then replaces the ready sibling F as well.
    database.markNodeUnavailable(mountPoint);
    cut = select(database, query, camera, params);
    EXPECT_EQ(refinedPayloads(database, query, cut),
              (std::vector<UserPayload>{21, 23}));
    EXPECT_EQ(payloads(database, cut),
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
            database, select(database, query, cameraAt(-8), params));
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
    EXPECT_EQ(payloads(database, cut),
              (std::vector<UserPayload>{1}));

    database.markNodeReady(replacementNode);
    cut = select(database, query, cameraAt(-8), params);
    EXPECT_EQ(payloads(database, cut),
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
    const std::array<FrontierEntry, 3> cut{first[0], first[1], second[0]};
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
    scene.database.applyUpdates(0);
    SpatialQuery query;
    SpatialQuery referenceQuery;
    const Camera camera = cameraAt(-8.0f);

    const auto expectExact = [&](const Camera& view)
    {
        const FrontierResultView handles =
            referenceQuery.selectFrontier(scene.database, view, {});
        std::vector<ResolvedFrontierEntry> expected(handles.size());
        const std::span<ResolvedFrontierEntry> resolved =
            scene.database.resolveFrontier(handles.entries, expected);
        ASSERT_EQ(resolved.size(), expected.size());

        const RenderFrontierView render =
            query.selectRenderFrontier(scene.database, view, {});
        std::vector<ResolvedFrontierEntry> actual;
        actual.reserve(render.size());
        size_t runEntries = 0;
        for (const RenderFrontierRun run : render.runs())
        {
            ASSERT_LE(size_t(run.begin) + run.count,
                      render.payloadStorage().size());
            ASSERT_LE(size_t(run.begin) + run.count,
                      render.errorStorage().size());
            const RenderFrontierSpan entries = render[run];
            ASSERT_EQ(entries.payloads.size(), entries.errors.size());
            for (size_t i = 0; i < entries.size(); ++i)
                actual.push_back(
                    {entries.payloads[i],
                     uint32_t(entries.instance) |
                         (uint32_t(entries.errors[i]) << kInstanceIdBits)});
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

TEST(Frontier, RenderAsUnitCoarsensOnlyDescendantFrustumCulling)
{
    Scene scene;
    TestAccess::markAllNodesReady(scene.database);
    scene.database.applyUpdates(0);
    const Camera camera = cameraAt(-8.0f);
    SpatialQuery exactQuery;
    exactQuery.setReuseEnabled(false);

    std::vector<UserPayload> exactPayloads;
    float boundaryX = 0.0f;
    for (uint32_t step = 1; step <= 160; ++step)
    {
        boundaryX = float(step) * 0.125f;
        InstanceTransform transform;
        transform.pos = float4::point(boundaryX, 0.0f, 0.0f);
        scene.database.moveInstance(scene.instance, transform);
    scene.database.applyUpdates(0);
        exactPayloads = payloads(
            scene.database,
            exactQuery.selectFrontier(scene.database, camera, {}));
        if (exactPayloads.size() == 1) break;
    }
    ASSERT_EQ(exactPayloads.size(), 1u)
        << "test scene never reached a one-child frustum intersection";

    const auto renderPayloads = [&](SpatialQuery& query)
    {
        const RenderFrontierView render =
            query.selectRenderFrontier(scene.database, camera, {});
        std::vector<UserPayload> result;
        result.reserve(render.size());
        for (const RenderFrontierRun run : render.runs())
        {
            const RenderFrontierSpan leaves = render[run];
            result.insert(result.end(), leaves.payloads.begin(),
                          leaves.payloads.end());
        }
        std::sort(result.begin(), result.end());
        return result;
    };

    SpatialQuery renderQuery;
    scene.database.setInstanceRenderAsUnit(scene.instance);
    EXPECT_EQ(renderPayloads(renderQuery),
              (std::vector<UserPayload>{11, 12}));

    TerminalRenderQuery terminalQuery;
    const auto terminalPayloads = [&](bool coarsenRenderUnits)
    {
        const TerminalRenderView terminal = terminalQuery.select(
            scene.database, camera, 4.0f, coarsenRenderUnits);
        std::vector<UserPayload> result;
        result.reserve(terminal.size());
        for (const TerminalRenderRun run : terminal.runs())
            result.insert(result.end(), run.payloadSpan().begin(),
                          run.payloadSpan().end());
        std::sort(result.begin(), result.end());
        return result;
    };
    EXPECT_EQ(terminalPayloads(true),
              (std::vector<UserPayload>{11, 12}));
    EXPECT_EQ(terminalPayloads(false), exactPayloads);

    // The normal handle API remains descendant-exact despite the render hint.
    EXPECT_EQ(payloads(scene.database,
                       exactQuery.selectFrontier(scene.database, camera, {})),
              exactPayloads);

    // Policy changes invalidate the cached render record immediately.
    scene.database.setInstanceRenderAsUnit(scene.instance, false);
    EXPECT_EQ(renderPayloads(renderQuery), exactPayloads);
}

TEST(Frontier, FullyRefinedBoundaryTraversalMatchesTheGeneralWalker)
{
    SpatialDatabase fastDatabase;
    SpatialDatabase referenceDatabase;
    const SubtreeHandle fastSubtree = fastDatabase.registerSubtree(
        makeFullyRefinedReferenceSubtree(0.0f));
    // A nonzero terminal error deliberately disables the specialized path;
    // terminal leaves are still selected unconditionally, so payload
    // visibility remains an independent reference for the same hierarchy.
    const SubtreeHandle referenceSubtree = referenceDatabase.registerSubtree(
        makeFullyRefinedReferenceSubtree(1.0e-20f));
    const InstanceHandle fastInstance = instantiateFor(
        fastDatabase, fastSubtree, box(5.0f), 64.0f);
    const InstanceHandle referenceInstance = instantiateFor(
        referenceDatabase, referenceSubtree, box(5.0f), 64.0f);
    TestAccess::markAllNodesReady(fastDatabase);
    TestAccess::markAllNodesReady(referenceDatabase);

    SpatialQuery fastQuery;
    SpatialQuery referenceQuery;
    fastQuery.setReuseEnabled(false);
    referenceQuery.setReuseEnabled(false);
    const Camera camera = cameraAt(-8.0f);

    bool reachedBoundary = false;
    for (uint32_t step = 1; step <= 160; ++step)
    {
        InstanceTransform transform;
        transform.pos = float4::point(float(step) * 0.125f, 0.0f, 0.0f);
        fastDatabase.moveInstance(fastInstance, transform);
        referenceDatabase.moveInstance(referenceInstance, transform);
        fastDatabase.applyUpdates(0);
        referenceDatabase.applyUpdates(0);

        const std::vector<UserPayload> fast = payloads(
            fastDatabase,
            fastQuery.selectFrontier(fastDatabase, camera, {}));
        const std::vector<UserPayload> reference = payloads(
            referenceDatabase,
            referenceQuery.selectFrontier(referenceDatabase, camera, {}));
        EXPECT_EQ(fast, reference);
        if (fast.empty() || fast.size() == 16) continue;

        reachedBoundary = true;
#ifdef FRONTIER_STATS
        EXPECT_GT(fastQuery.lastSelectionStats().fullyRefinedSubtrees, 0u);
        EXPECT_EQ(referenceQuery.lastSelectionStats().fullyRefinedSubtrees,
                  0u);
#endif
        break;
    }
    EXPECT_TRUE(reachedBoundary)
        << "test scene never reached a partial-leaf frustum intersection";
}

TEST(Frontier, TerminalRenderRangesMatchTheFullyRefinedCurrentCut)
{
    SpatialDatabase database;
    const SubtreeHandle subtree = database.registerSubtree(
        makeFullyRefinedReferenceSubtree(0.0f));
    const InstanceHandle instance = instantiateFor(
        database, subtree, box(5.0f), 64.0f);
    TestAccess::markAllNodesReady(database);

    SpatialQuery referenceQuery;
    referenceQuery.setReuseEnabled(false);
    TerminalRenderQuery terminalQuery;
    const Camera camera = cameraAt(-8.0f);

    bool reachedBoundary = false;
    for (uint32_t step = 1; step <= 160; ++step)
    {
        InstanceTransform transform;
        transform.pos = float4::point(float(step) * 0.125f, 0.0f, 0.0f);
        database.moveInstance(instance, transform);
        database.applyUpdates(0);

        std::vector<UserPayload> reference = payloads(
            database,
            referenceQuery.selectFrontier(database, camera, {}));
        const TerminalRenderView terminal =
            terminalQuery.select(database, camera);
        std::vector<UserPayload> ranged;
        ranged.reserve(terminal.size());
        for (const TerminalRenderRun run : terminal.runs())
        {
            EXPECT_EQ(run.errorCode(), 0u);
            ranged.insert(ranged.end(), run.payloadSpan().begin(),
                          run.payloadSpan().end());
        }
        std::sort(reference.begin(), reference.end());
        std::sort(ranged.begin(), ranged.end());
        EXPECT_EQ(ranged, reference);
        EXPECT_EQ(terminal.size(), reference.size());
        if (!ranged.empty() && ranged.size() != 16)
        {
            reachedBoundary = true;
            EXPECT_LT(terminal.segmentCount(), terminal.size());
            break;
        }
    }
    EXPECT_TRUE(reachedBoundary)
        << "test scene never reached a partial terminal range";
}

TEST(Frontier, TerminalActorBatchMatchesMountedYawedInstance)
{
    SpatialDatabase mountedDatabase;
    SpatialDatabase batchDatabase;
    const SubtreeHandle mountedDefinition = mountedDatabase.registerSubtree(
        makeFullyRefinedReferenceSubtree(0.0f));
    const SubtreeHandle batchDefinition = batchDatabase.registerSubtree(
        makeFullyRefinedReferenceSubtree(0.0f));

    InstanceDesc desc;
    desc.yaw = yawRotation(0.6f);
    NodeDesc root = node(9000, 64.0f, box(5.0f), true);
    root.flags |= NodeDesc::FlagYawInvariantBounds;
    const InstanceHandle instance = mountedDatabase.instantiate(root, desc);
    mountedDatabase.setInstanceRenderAsUnit(instance);
    mountedDatabase.mountSubtree(instance.rootNode(), mountedDefinition);
    TestAccess::markAllNodesReady(mountedDatabase);
    TestAccess::markAllNodesReady(batchDatabase);
    mountedDatabase.optimize();
    batchDatabase.applyUpdates(0);

    std::array<float4, 1> positions{desc.pos};
    std::array<YawRotation, 1> yaws{desc.yaw};
    TerminalInstanceBatch batch;
    batch.definition = batchDefinition;
    batch.localBounds = box(5.0f);
    batch.positions = positions;
    batch.yaws = yaws;
    batch.firstInstance = 17;
    batch.yawInvariantBounds = true;

    const auto terminalPayloads = [&](TerminalRenderQuery& query,
                                      const Camera& camera,
                                      bool coarsen)
    {
        const TerminalRenderView view = query.select(
            batchDatabase, camera,
            std::span<const TerminalInstanceBatch>(&batch, 1), 4.0f,
            coarsen);
        std::vector<UserPayload> result;
        result.reserve(view.size());
        for (const TerminalRenderRun run : view.runs())
        {
            EXPECT_EQ(run.instance(), 17u);
            result.insert(result.end(), run.payloadSpan().begin(),
                          run.payloadSpan().end());
        }
        std::sort(result.begin(), result.end());
        return result;
    };

    SpatialQuery mountedQuery;
    mountedQuery.setReuseEnabled(false);
    TerminalRenderQuery batchQuery;
    const Camera camera = cameraAt(-8.0f);
    std::vector<UserPayload> exact;
    bool reachedBoundary = false;
    for (uint32_t step = 1; step <= 160; ++step)
    {
        InstanceTransform transform;
        transform.pos = float4::point(float(step) * 0.125f, 0.0f, 0.0f);
        transform.yaw = desc.yaw;
        positions[0] = transform.pos;
        mountedDatabase.moveInstance(instance, transform);
        mountedDatabase.applyUpdates(0);
        exact = payloads(
            mountedDatabase,
            mountedQuery.selectFrontier(mountedDatabase, camera, {}));
        std::sort(exact.begin(), exact.end());
        EXPECT_EQ(terminalPayloads(batchQuery, camera, false), exact);
        if (!exact.empty() && exact.size() != 16)
        {
            reachedBoundary = true;
            break;
        }
    }
    ASSERT_TRUE(reachedBoundary);

    TerminalRenderQuery mountedTerminalQuery;
    const TerminalRenderView mountedRender = mountedTerminalQuery.select(
        mountedDatabase, camera, 4.0f, true);
    std::vector<UserPayload> renderPayloads;
    renderPayloads.reserve(mountedRender.size());
    for (const TerminalRenderRun run : mountedRender.runs())
        renderPayloads.insert(renderPayloads.end(), run.payloadSpan().begin(),
                              run.payloadSpan().end());
    std::sort(renderPayloads.begin(), renderPayloads.end());
    EXPECT_EQ(terminalPayloads(batchQuery, camera, true), renderPayloads);
}

TEST(Frontier, TerminalActorClustersMatchUngroupedPlacementStream)
{
    SpatialDatabase database;
    const SubtreeHandle definition = database.registerSubtree(
        makeFullyRefinedReferenceSubtree(0.0f));
    TestAccess::markAllNodesReady(database);
        database.applyUpdates(0);

    std::array<float4, 6> positions{
        float4::point(-18.0f, 0.0f, -2.0f),
        float4::point(-9.0f, 0.0f, 1.0f),
        float4::point(-1.0f, 0.0f, 0.0f),
        float4::point(7.0f, 0.0f, -1.0f),
        float4::point(15.0f, 0.0f, 2.0f),
        float4::point(28.0f, 0.0f, 0.0f)};
    std::array<YawRotation, 6> yaws{
        yawRotation(0.1f), yawRotation(0.7f), yawRotation(-0.3f),
        yawRotation(1.1f), yawRotation(-0.9f), yawRotation(0.4f)};
    std::array<TerminalInstanceCluster, 2> clusters{{{0, 3}, {3, 3}}};

    TerminalInstanceBatch ungrouped;
    ungrouped.definition = definition;
    ungrouped.localBounds = box(3.0f);
    ungrouped.positions = positions;
    ungrouped.yaws = yaws;
    ungrouped.firstInstance = 41;
    ungrouped.renderAsUnit = true;
    TerminalInstanceBatch clustered = ungrouped;
    clustered.clusters = clusters;
    std::array<AABB, 2> clusterBounds;
    for (size_t clusterIndex = 0; clusterIndex < clusters.size();
         ++clusterIndex)
    {
        clusterBounds[clusterIndex] = AABB::empty();
        const TerminalInstanceCluster cluster = clusters[clusterIndex];
        for (size_t i = cluster.first; i < cluster.first + cluster.count; ++i)
            clusterBounds[clusterIndex].expand(
                toWorld(clustered.localBounds, positions[i],
                        clustered.scale, yaws[i]));
    }
    TerminalInstanceBatch published = clustered;
    published.clusterBounds = clusterBounds;

    const auto selected = [&](TerminalRenderQuery& query,
                              const TerminalInstanceBatch& batch,
                              const Camera& camera, bool coarsen)
    {
        const TerminalRenderView view = query.select(
            database, camera,
            std::span<const TerminalInstanceBatch>(&batch, 1), 4.0f,
            coarsen);
        std::vector<std::pair<InstanceId, UserPayload>> result;
        result.reserve(view.size());
        for (const TerminalRenderRun run : view.runs())
            for (const UserPayload payload : run.payloadSpan())
                result.emplace_back(run.instance(), payload);
        std::sort(result.begin(), result.end());
        return result;
    };

    const std::array<Camera, 3> cameras{
        cameraAt(-12.0f),
        cameraAt(-25.0f, float4::point(-12.0f, 0.0f, 0.0f)),
        cameraAt(-35.0f, float4::point(18.0f, 0.0f, 0.0f))};
    TerminalRenderQuery ungroupedQuery;
    TerminalRenderQuery clusteredQuery;
    TerminalRenderQuery publishedQuery;
    for (const Camera& camera : cameras)
    {
        EXPECT_EQ(selected(clusteredQuery, clustered, camera, false),
                  selected(ungroupedQuery, ungrouped, camera, false));
        EXPECT_EQ(selected(publishedQuery, published, camera, false),
                  selected(ungroupedQuery, ungrouped, camera, false));
        EXPECT_EQ(selected(clusteredQuery, clustered, camera, true),
                  selected(ungroupedQuery, ungrouped, camera, true));
        EXPECT_EQ(selected(publishedQuery, published, camera, true),
                  selected(ungroupedQuery, ungrouped, camera, true));
    }

    std::array<AABB, 2> underBounds = clusterBounds;
    underBounds[0] = box(0.1f, positions[0]);
    TerminalInstanceBatch invalidPublished = published;
    invalidPublished.clusterBounds = underBounds;
    TerminalRenderQuery invalidQuery;
    EXPECT_THROW(
        invalidQuery.select(
            database, cameras[0],
            std::span<const TerminalInstanceBatch>(&invalidPublished, 1)),
        std::logic_error);
}

TEST(Frontier, TerminalRenderRejectsNonzeroTerminalError)
{
    SpatialDatabase database;
    const SubtreeHandle subtree = database.registerSubtree(
        makeFullyRefinedReferenceSubtree(1.0f));
    instantiateFor(database, subtree, box(5.0f), 64.0f);
    TestAccess::markAllNodesReady(database);
    database.applyUpdates(0);

    TerminalRenderQuery query;
    EXPECT_THROW(query.select(database, cameraAt(-8.0f)), std::logic_error);
}
