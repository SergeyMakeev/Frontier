#include <gtest/gtest.h>

#include "helpers.h"

using namespace frontier;
using namespace frontiertest;

namespace {

void verifyInvariants(const Page& pg)
{
    const uint32_t n = pg.nodeCount();
    ASSERT_GE(n, 2u);
    EXPECT_EQ(pg.payload[0], kSentinelPayload);
    EXPECT_GT(pg.childCount(0), 0u);

    for (uint32_t i = 1; i < n; ++i)
    {
        EXPECT_LT(pg.parent[i], i) << "(A) node " << i;                        // (A)
        EXPECT_LE(i + pg.subtreeSize[i], n) << "(B) node " << i;               // (B)
        if (pg.childCount(i) > 0)
            EXPECT_EQ(pg.parent[i + 1], i) << "(B) first child adjacency " << i;
        EXPECT_TRUE(pg.bbox[pg.parent[i]].contains(pg.bbox[i])) << "(C) " << i; // (C)
        EXPECT_LE(pg.geometricError[i], pg.geometricError[pg.parent[i]])
            << "(D) node " << i;                                               // (D)
        if (pg.isExpansion(i)) EXPECT_EQ(pg.childCount(i), 0u);
    }

    // Subtree windows partition each node's child ranges exactly.
    for (uint32_t i = 0; i < n; ++i)
    {
        uint32_t sum = 1;
        uint32_t c = i + 1;
        for (uint32_t k = 0; k < pg.childCount(i); ++k)
        {
            EXPECT_EQ(pg.parent[c], i);
            sum += pg.subtreeSize[c];
            c += pg.subtreeSize[c];
        }
        if (i > 0) EXPECT_EQ(sum, pg.subtreeSize[i]) << "window of node " << i;
    }

    // Wide lanes mirror the cold arrays exactly.
    for (uint32_t i = 0; i < n; ++i)
    {
        const uint32_t cc = pg.childCount(i);
        if (cc == 0) continue;
        uint32_t b = pg.wideOffset(i);
        uint32_t seen = 0;
        for (uint32_t base = 0; base < cc; base += kWide, ++b)
        {
            const WideBlock& blk = pg.wide[b];
            for (uint32_t l = 0; l < kWide; ++l)
            {
                if (!(pg.validLanes(b) & (1u << l)))
                {
                    EXPECT_EQ(blk.child[l], kInvalidIndex);
                    continue;
                }
                const uint32_t c = blk.child[l];
                ++seen;
                EXPECT_EQ(pg.parent[c], i);
                EXPECT_EQ((pg.leafLanes(b) >> l) & 1u,
                          (pg.childCount(c) == 0 && !pg.isExpansion(c)) ? 1u : 0u);
                const AABB lane = blk.bounds.lane(l);
                EXPECT_EQ(lane.mn.x, pg.bbox[c].mn.x);
                EXPECT_EQ(lane.mx.z, pg.bbox[c].mx.z);
                EXPECT_EQ(blk.error.v[l], pg.geometricError[c]);
            }
        }
        EXPECT_EQ(seen, cc);
    }
}

} // namespace

TEST(Builder, TownExample)
{
    PageBuilder b;
    const auto town = b.createRoot(1, 16.0f);
    const auto bldA = b.createNode(town, 2, 8.0f);
    b.createNode(bldA, 10, 1.0f, AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(1, 1, 1)));
    b.createNode(bldA, 11, 1.0f, AABB::fromCenterExtent(float4::vec(3, 0, 0), float4::vec(1, 1, 1)));
    b.createNode(bldA, 12, 1.0f, AABB::fromCenterExtent(float4::vec(6, 0, 0), float4::vec(1, 1, 1)));
    const auto bldB = b.createNode(town, 3, 8.0f, AABB::fromCenterExtent(float4::vec(50, 0, 0), float4::vec(5, 5, 5)));
    (void)bldB;

    const Page pg = b.build();
    EXPECT_EQ(pg.nodeCount(), 7u);   // sentinel + 6
    verifyInvariants(pg);

    // Derived bounds: buildingA's bbox is the union of its walls.
    EXPECT_TRUE(pg.bbox[2].contains(pg.bbox[3]));
    EXPECT_FLOAT_EQ(pg.bbox[2].mn.x, -1.0f);
    EXPECT_FLOAT_EQ(pg.bbox[2].mx.x, 7.0f);
}

TEST(Builder, ErrorClampEstablishesMonotonicity)
{
    PageBuilder b;
    const auto root = b.createRoot(1, 4.0f);
    b.createNode(root, 2, 9.0f,   // bad authored error: larger than parent
                 AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(1, 1, 1)));
    const Page pg = b.build();
    EXPECT_FLOAT_EQ(pg.geometricError[2], 4.0f);   // clamped
    verifyInvariants(pg);
}

TEST(Builder, MultiRootForestPage)
{
    PageBuilder b;
    for (int r = 0; r < 5; ++r)
        b.createRoot(uint64_t(100 + r), 2.0f,
                     AABB::fromCenterExtent(float4::vec(float(r) * 10, 0, 0), float4::vec(1, 1, 1)));
    const Page pg = b.build();
    EXPECT_EQ(pg.childCount(0), 5u);
    verifyInvariants(pg);
}

TEST(SubtreeBuilder, MountSentinelAndRepeatedDependencyArePackedOnce)
{
    constexpr SubtreeKey cityKey{100};
    constexpr SubtreeKey houseKey{200};

    SubtreeBuilder city(cityKey);
    const auto cityProxy = city.createNode(
        city.root(), 1, 64.0f, AABB::empty());
    for (uint32_t i = 0; i < 128; ++i)
    {
        const float x = float(i) * 8.0f;
        const auto house = city.createNode(
            cityProxy, 10 + i, 16.0f,
            AABB::fromCenterExtent(float4::point(x, 0, 0),
                                   float4::vec(2, 2, 2)));
        city.setExpansion(
            house, houseKey,
            SubtreeTransform{float4::point(x, 0, 0), 1.0f});
    }

    Subtree subtree = city.build();
    ASSERT_TRUE(subtree.valid());
    EXPECT_EQ(subtree.key(), cityKey);
    EXPECT_EQ(subtree.page().childCount(0), 1u); // one real root below anchor
    EXPECT_EQ(subtree.page().nodeCount(), 130u); // sentinel + city + houses
    ASSERT_EQ(subtree.dependencies().size(), 1u);
    EXPECT_EQ(subtree.dependencies()[0], houseKey);
    ASSERT_EQ(subtree.expansions().size(), 128u);
    for (uint32_t i = 0; i < subtree.expansions().size(); ++i)
    {
        EXPECT_EQ(subtree.expansions()[i].nodeIndex, i + 2);
        EXPECT_EQ(subtree.expansions()[i].dependency, 0u);
    }
}

TEST(SubtreeAssembly, SharedHouseMountsKeepDistinctPlacements)
{
    constexpr SubtreeKey houseKey{1001};
    constexpr SubtreeKey cityKey{1002};

    SubtreeBuilder house(houseKey);
    house.createNode(
        house.root(), 100, 0.0f,
        AABB::fromCenterExtent(float4::point(-0.75f, 0, 0),
                               float4::vec(0.25f, 1, 1)));
    house.createNode(
        house.root(), 101, 0.0f,
        AABB::fromCenterExtent(float4::point(0.75f, 0, 0),
                               float4::vec(0.25f, 1, 1)));
    Subtree houseSubtree = house.build();
    const uint32_t houseNodes = houseSubtree.page().nodeCount();

    SubtreeBuilder city(cityKey);
    const auto left = city.createNode(
        city.root(), 10, 16.0f,
        AABB::fromCenterExtent(float4::point(-10, 0, 0),
                               float4::vec(2, 2, 2)));
    const auto right = city.createNode(
        city.root(), 11, 16.0f,
        AABB::fromCenterExtent(float4::point(10, 0, 0),
                               float4::vec(2, 2, 2)));
    city.setExpansion(
        left, houseKey,
        SubtreeTransform{float4::point(-10, 0, 0), 1.0f});
    city.setExpansion(
        right, houseKey,
        SubtreeTransform{float4::point(10, 0, 0), 1.0f});

    SpatialDatabase world;
    const SubtreeHandle houseHandle =
        world.registerSubtree(std::move(houseSubtree));
    Subtree citySubtree = city.build();
    const AABB cityBounds = citySubtree.page().bbox[0];
    const SubtreeHandle cityHandle =
        world.registerSubtree(std::move(citySubtree));
    const SpatialDatabase::InstanceRef instance =
        world.instantiate(RootNodeDesc{1, 64.0f, cityBounds, cityKey},
                          float4::point(0, 0, 0));
    ASSERT_TRUE(instance.rootNode().valid());
    const MountHandle cityMount = world.mount(instance.rootNode(), cityHandle);
    ASSERT_TRUE(cityMount.valid());

    const NodeHandle leftNode = nodeAt(cityMount, 1);
    const NodeHandle rightNode = nodeAt(cityMount, 2);
    EXPECT_EQ(world.expansionTarget(leftNode), houseKey);
    EXPECT_EQ(world.expansionTarget(rightNode), houseKey);

    const MountHandle leftMount = world.mount(leftNode, houseHandle);
    const MountHandle rightMount = world.mount(rightNode, houseHandle);
    ASSERT_TRUE(leftMount.valid());
    ASSERT_TRUE(rightMount.valid());
    EXPECT_NE(leftMount.slot, rightMount.slot);
    EXPECT_EQ(world.assetCount(), 2u);       // city + one shared house definition
    EXPECT_EQ(world.attachedPageCount(), 3u); // city + two placements

    world.markResident(leftNode);
    world.markResident(rightNode);
    markAllResident(world, leftMount, houseNodes);
    markAllResident(world, rightMount, houseNodes);

    SubtreeTransform leftTransform, rightTransform;
    ASSERT_TRUE(world.tryGetNodeTransform(nodeAt(leftMount, 1), leftTransform));
    ASSERT_TRUE(world.tryGetNodeTransform(nodeAt(rightMount, 1), rightTransform));
    EXPECT_FLOAT_EQ(leftTransform.pos.x, -10.0f);
    EXPECT_FLOAT_EQ(rightTransform.pos.x, 10.0f);

    world.applyUpdates();
    SpatialQuery query;
    query.setReuseEnabled(false);
    const Camera camera = makeLookAtCamera(float4::point(0, 4, -25),
                                           float4::point(0, 0, 0));
    const FrontierResultView result =
        query.selectFrontier(world, camera, SelectionParams{1.0f, 0.0f});
    uint32_t wall100 = 0, wall101 = 0;
    for (const FrontierEntry& entry : currentFrontier(result))
    {
        const UserPayload payload = payloadOf(world, entry);
        wall100 += payload == 100;
        wall101 += payload == 101;
    }
    EXPECT_EQ(wall100, 2u);
    EXPECT_EQ(wall101, 2u);

    world.detachPage(leftNode);
    EXPECT_FALSE(world.isAttached(leftNode));
    EXPECT_TRUE(world.isAttached(rightNode));
}

TEST(SubtreeAssembly, TransformedBoundsPropagateInOwnerSpacePerInstance)
{
    constexpr SubtreeKey detailKey{2001};
    constexpr SubtreeKey parentKey{2002};

    SubtreeBuilder detail(detailKey);
    detail.createNode(
        detail.root(), 100, 1.0f,
        AABB::fromCenterExtent(float4::point(0, 0, 0),
                               float4::vec(1, 1, 1)));

    SubtreeBuilder parent(parentKey);
    const auto proxy = parent.createNode(
        parent.root(), 10, 8.0f,
        AABB::fromCenterExtent(float4::point(-10, 0, 0),
                               float4::vec(2, 2, 2)));
    parent.setExpansion(
        proxy, detailKey,
        SubtreeTransform{float4::point(-10, 0, 0), 1.0f});

    SpatialDatabase world;
    const SubtreeHandle detailHandle = world.registerSubtree(detail.build());
    Subtree parentSubtree = parent.build();
    const AABB parentBounds = parentSubtree.page().bbox[0];
    const SubtreeHandle parentHandle =
        world.registerSubtree(std::move(parentSubtree));
    const RootNodeDesc rootDesc{1, 32.0f, parentBounds, parentKey};
    const auto first = world.instantiate(rootDesc, float4::point(0, 0, 0));
    const auto second = world.instantiate(rootDesc, float4::point(100, 0, 0));
    const MountHandle firstParent = world.mount(first.rootNode(), parentHandle);
    const MountHandle secondParent = world.mount(second.rootNode(), parentHandle);
    ASSERT_TRUE(firstParent.valid());
    ASSERT_TRUE(secondParent.valid());
    const NodeHandle firstProxy = nodeAt(firstParent, 1);
    const NodeHandle secondProxy = nodeAt(secondParent, 1);
    const MountHandle firstDetail = world.mount(firstProxy, detailHandle);
    const MountHandle secondDetail = world.mount(secondProxy, detailHandle);
    ASSERT_TRUE(firstDetail.valid());
    ASSERT_TRUE(secondDetail.valid());

    const AABB moved = AABB::fromCenterExtent(float4::point(-4, 0, 0),
                                               float4::vec(1, 1, 1));
    world.setNodeBounds(first, nodeAt(firstDetail, 1), moved);
    world.flushBounds();

    const AABB firstProxyBounds = world.nodeBounds(first, firstProxy);
    const AABB secondProxyBounds = world.nodeBounds(second, secondProxy);
    EXPECT_FLOAT_EQ(firstProxyBounds.mn.x, -15.0f); // child-local -5, mounted at -10
    EXPECT_FLOAT_EQ(secondProxyBounds.mn.x, -12.0f);
    EXPECT_FLOAT_EQ(secondProxyBounds.mx.x, -8.0f);
}

TEST(SubtreeAssembly, NestedMountTransformsComposeThroughMountSentinels)
{
    constexpr SubtreeKey detailKey{2501};
    constexpr SubtreeKey blockKey{2502};

    SubtreeBuilder detail(detailKey);
    detail.createNode(
        detail.root(), 100, 0.0f,
        AABB::fromCenterExtent(float4::point(0, 0, 0),
                               float4::vec(1, 1, 1)));

    SubtreeBuilder block(blockKey);
    const auto building = block.createNode(
        block.root(), 10, 8.0f,
        AABB::fromCenterExtent(float4::point(3, 0, 0),
                               float4::vec(1, 1, 1)));
    block.setExpansion(
        building, detailKey,
        SubtreeTransform{float4::point(3, 0, 0), 0.5f});

    SpatialDatabase world;
    const SubtreeHandle detailHandle = world.registerSubtree(detail.build());
    const SubtreeHandle blockHandle = world.registerSubtree(block.build());
    const auto instance =
        world.instantiate(
            RootNodeDesc{1, 32.0f,
                         AABB::fromCenterExtent(float4::point(16, 0, 0),
                                                float4::vec(3, 3, 3)),
                         blockKey},
            float4::point(100, 0, 0));

    const MountHandle blockMount =
        world.mount(instance.rootNode(), blockHandle,
                    SubtreeTransform{float4::point(10, 0, 0), 2.0f});
    ASSERT_TRUE(blockMount.valid());
    const MountHandle detailMount =
        world.mount(nodeAt(blockMount, 1), detailHandle);
    ASSERT_TRUE(detailMount.valid());

    SubtreeTransform transform;
    ASSERT_TRUE(
        world.tryGetNodeTransform(nodeAt(detailMount, 1), transform));
    EXPECT_FLOAT_EQ(transform.pos.x, 16.0f); // 10 + 3 * 2
    EXPECT_FLOAT_EQ(transform.scale, 1.0f);  // 2 * 0.5
}

TEST(SubtreeAssembly, MountRejectsTheWrongPermanentTarget)
{
    constexpr SubtreeKey expectedKey{3001};
    constexpr SubtreeKey wrongKey{3002};

    auto makeLeaf = [](SubtreeKey key, UserPayload payload) {
        SubtreeBuilder builder(key);
        builder.createNode(
            builder.root(), payload, 0.0f,
            AABB::fromCenterExtent(float4::point(0, 0, 0),
                                   float4::vec(1, 1, 1)));
        return builder.build();
    };

    SpatialDatabase world;
    const SubtreeHandle wrong = world.registerSubtree(makeLeaf(wrongKey, 20));
    const auto instance = world.instantiate(
        RootNodeDesc{1, 8.0f,
                     AABB::fromCenterExtent(float4::point(0, 0, 0),
                                            float4::vec(2, 2, 2)),
                     expectedKey},
        float4::point(0, 0, 0));
    EXPECT_THROW(world.mount(instance.rootNode(), wrong),
                 std::logic_error);
}

TEST(TlasRoot, SingleNodeUsesNoPageAndKeepsAUniformRenderableHandle)
{
    SpatialDatabase world;
    const AABB authored = AABB::fromCenterExtent(
        float4::point(0, 0, 0), float4::vec(2, 1, 1));
    const auto instance = world.instantiate(
        RootNodeDesc{42, 0.0f, authored, {}},
        float4::point(10, 0, 0));
    const NodeHandle root = instance.rootNode();

    EXPECT_TRUE(root.valid());
    EXPECT_TRUE(root.isTlasRoot());
    EXPECT_FALSE(instance.rootPage.valid());
    EXPECT_EQ(world.attachedPageCount(), 0u);
    EXPECT_TRUE(world.isResident(root));
    EXPECT_FALSE(world.expansionTarget(root).valid());

    UserPayload payload = 0;
    ASSERT_TRUE(world.tryGetPayload(root, payload));
    EXPECT_EQ(payload, 42u);
    SubtreeTransform transform;
    ASSERT_TRUE(world.tryGetNodeTransform(root, transform));
    EXPECT_FLOAT_EQ(transform.pos.x, 0.0f);
    EXPECT_FLOAT_EQ(transform.scale, 1.0f);
    EXPECT_THROW(world.markNonResident(root), std::logic_error);

    world.applyUpdates();
    SpatialQuery query;
    query.setReuseEnabled(false);
    const FrontierResultView cut = query.selectFrontier(
        world,
        makeLookAtCamera(float4::point(10, 4, -20),
                         float4::point(10, 0, 0)),
        SelectionParams{1.0f, 0.0f});
    ASSERT_EQ(currentFrontier(cut).size(), 1u);
    EXPECT_EQ(currentFrontier(cut)[0].nodeHandle, root);

    const AABB moved = AABB::fromCenterExtent(
        float4::point(3, 0, 0), float4::vec(1, 1, 1));
    world.setNodeBounds(instance, root, moved);
    EXPECT_FLOAT_EQ(world.nodeBounds(instance, root).mn.x, 2.0f);
    world.moveInstance(instance, float4::point(20, 0, 0), 2.0f);
    EXPECT_FLOAT_EQ(world.nodeBounds(instance, root).mn.x, 2.0f);

    world.removeInstance(instance);
    EXPECT_FALSE(world.isResident(root));
    EXPECT_FALSE(world.tryGetPayload(root, payload));
    EXPECT_TRUE(world.nodeBounds(instance, root).isEmpty());
}

TEST(TlasRoot, CollectedMountedSubtreeFallsBackToPermanentRoot)
{
    constexpr SubtreeKey detailKey{0xD001};
    SubtreeBuilder detail(detailKey);
    detail.createNode(
        detail.root(), 100, 0.0f,
        AABB::fromCenterExtent(float4::point(0, 0, 0),
                               float4::vec(1, 1, 1)));
    Subtree detailSubtree = detail.build();
    const uint32_t detailNodes = detailSubtree.page().nodeCount();

    SpatialDatabase world;
    const SubtreeHandle detailHandle =
        world.registerSubtree(std::move(detailSubtree));
    const auto instance = world.instantiate(
        RootNodeDesc{1, 32.0f,
                     AABB::fromCenterExtent(float4::point(0, 0, 0),
                                            float4::vec(2, 2, 2)),
                     detailKey});
    const MountHandle mounted = world.mount(instance.rootNode(), detailHandle);
    ASSERT_TRUE(mounted.valid());
    markAllResident(world, mounted, detailNodes);
    world.applyUpdates();

    SpatialQuery query;
    query.setReuseEnabled(false);
    const Camera camera = makeLookAtCamera(float4::point(0, 2, -8),
                                           float4::point(0, 0, 0));
    const SelectionParams params{1.0f, 0.0f};
    FrontierResultView cut = query.selectFrontier(world, camera, params);
    ASSERT_EQ(currentFrontier(cut).size(), 1u);
    EXPECT_EQ(payloadOf(world, currentFrontier(cut)[0]), 100u);

    const CollectResult collected = world.collect(0, 0);
    EXPECT_EQ(collected.detachedPages, 1u);
    EXPECT_FALSE(world.isAttached(instance.rootNode()));
    EXPECT_EQ(world.attachedPageCount(), 0u);

    cut = query.selectFrontier(world, camera, params);
    ASSERT_EQ(currentFrontier(cut).size(), 1u);
    EXPECT_EQ(currentFrontier(cut)[0].nodeHandle, instance.rootNode());
    EXPECT_EQ(payloadOf(world, currentFrontier(cut)[0]), 1u);
}

TEST(Builder, WideFanoutChainsBlocks)
{
    PageBuilder b;
    const auto root = b.createRoot(1, 100.0f);
    for (int c = 0; c < 21; ++c)   // 21 children -> 3 wide blocks
        b.createNode(root, uint64_t(10 + c), 1.0f,
                     AABB::fromCenterExtent(float4::vec(float(c), 0, 0),
                                            float4::vec(0.4f, 0.4f, 0.4f)));
    const Page pg = b.build();
    EXPECT_EQ(pg.childCount(1), 21u);
    EXPECT_EQ(pg.wideBlockCount(1), 3u);
    verifyInvariants(pg);
}

TEST(Builder, GeneratedPagesSatisfyInvariants)
{
    TreeGen gen;
    gen.fanout = 5;
    gen.depth = 3;
    const Page pg = gen.makeRootPage(unitRegion(), 64.0f, 1);
    verifyInvariants(pg);

    // Some leaf became an expansion point with a recipe.
    ASSERT_FALSE(gen.recipes.empty());
    const Page child = gen.makeChildPage(gen.recipes.begin()->first);
    verifyInvariants(child);
    EXPECT_EQ(child.childCount(0), gen.fanout);
}

TEST(HierarchyBuilder, NaturalEntitySplitsProduceLogicalSingleRootPages)
{
    HierarchyBuilder builder;
    const HierarchyBuilder::NodeId town =
        builder.createRoot(1, 32.0f, AABB::empty());
    const HierarchyBuilder::NodeId building1 =
        builder.createNode(town, 2, 8.0f, AABB::empty());
    builder.createNode(
        building1, 10, 1.0f,
        AABB::fromCenterExtent(float4::vec(-2, 0, 0), float4::vec(1, 1, 1)));
    builder.createNode(
        building1, 11, 1.0f,
        AABB::fromCenterExtent(float4::vec(2, 0, 0), float4::vec(1, 1, 1)));

    const HierarchyBuilder::NodeId building2 =
        builder.createNode(town, 2, 8.0f, AABB::empty()); // duplicate payload
    builder.createNode(
        building2, 20, 1.0f,
        AABB::fromCenterExtent(float4::vec(20, 0, 0), float4::vec(1, 1, 1)));

    builder.splitBelow(building1);
    builder.splitBelow(building2);
    Hierarchy hierarchy = builder.build();

    ASSERT_EQ(hierarchy.pageCount(), 3u);
    EXPECT_EQ(hierarchy.rootPage(), 0u);

    const PageView townPage = hierarchy.page(0);
    ASSERT_EQ(townPage.childCount(0), 1u); // exactly one physical root
    EXPECT_EQ(townPage.payload[1], 1u);
    EXPECT_TRUE(townPage.isExpansion(2));
    EXPECT_TRUE(townPage.isExpansion(3));

    const Hierarchy::PageId building1Page = townPage.detailPage(2);
    const Hierarchy::PageId building2Page = townPage.detailPage(3);
    ASSERT_NE(building1Page, Hierarchy::kInvalidPage);
    ASSERT_NE(building2Page, Hierarchy::kInvalidPage);
    EXPECT_NE(building1Page, building2Page);
    EXPECT_EQ(building1Page, 1u);
    EXPECT_EQ(building2Page, 2u);

    const Page rootCopy = hierarchy.clonePage(0);
    const Page roundTrip = Page::fromBytes(rootCopy.data(), rootCopy.byteSize());
    EXPECT_EQ(roundTrip.detailPage(2), building1Page);
    EXPECT_EQ(roundTrip.detailPage(3), building2Page);

    const PageView building1Details = hierarchy.page(building1Page);
    EXPECT_EQ(building1Details.childCount(0), 2u);
    EXPECT_EQ(building1Details.payload[1], 10u);
    EXPECT_EQ(building1Details.payload[2], 11u);
    EXPECT_TRUE(townPage.bbox[2].contains(building1Details.bbox[0]));
}

TEST(HierarchyBuilder, NestedSplitsEmbedDetailPageIds)
{
    HierarchyBuilder builder;
    const HierarchyBuilder::NodeId town =
        builder.createRoot(1, 64.0f, AABB::empty());
    const HierarchyBuilder::NodeId building =
        builder.createNode(town, 2, 16.0f, AABB::empty());
    const HierarchyBuilder::NodeId floor =
        builder.createNode(building, 3, 4.0f, AABB::empty());
    builder.createNode(
        floor, 4, 1.0f,
        AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(1, 1, 1)));

    builder.splitBelow(building);
    builder.splitBelow(floor);
    Hierarchy hierarchy = builder.build();

    ASSERT_EQ(hierarchy.pageCount(), 3u);
    const Hierarchy::PageId buildingPage = hierarchy.page(0).detailPage(2);
    ASSERT_NE(buildingPage, Hierarchy::kInvalidPage);

    const PageView buildingDetails = hierarchy.page(buildingPage);
    ASSERT_EQ(buildingDetails.childCount(0), 1u);
    EXPECT_EQ(buildingDetails.payload[1], 3u);
    EXPECT_TRUE(buildingDetails.isExpansion(1));

    const Hierarchy::PageId floorPage = buildingDetails.detailPage(1);
    ASSERT_NE(floorPage, Hierarchy::kInvalidPage);
    EXPECT_EQ(hierarchy.page(floorPage).payload[1], 4u);
}

TEST(HierarchyBuilder, GeneratedDetailPageAttachesToItsLogicalRoot)
{
    HierarchyBuilder builder;
    const HierarchyBuilder::NodeId root =
        builder.createRoot(1, 32.0f, AABB::empty());
    const HierarchyBuilder::NodeId building =
        builder.createNode(root, 2, 8.0f, AABB::empty());
    const HierarchyBuilder::NodeId floor =
        builder.createNode(building, 3, 4.0f, AABB::empty());
    builder.createNode(
        floor, 4, 1.0f,
        AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(1, 1, 1)));
    builder.splitBelow(building);
    builder.splitBelow(floor);
    Hierarchy hierarchy = builder.build();

    SpatialDatabase world;
    const AssetHandle rootAsset = world.registerAsset(hierarchy.page(0));
    const SpatialDatabase::InstanceRef instance =
        world.addInstance(rootAsset, float4::point(0, 0, 0));

    const Hierarchy::PageId details = hierarchy.page(0).detailPage(2);
    ASSERT_NE(details, Hierarchy::kInvalidPage);
    const NodeHandle expansion = nodeAt(instance.rootPage, 2);
    const DetailPageRef detailRef = world.detailPage(expansion);
    EXPECT_TRUE(detailRef.valid());
    EXPECT_EQ(detailRef.page, details);
    EXPECT_EQ(detailRef.rootAsset.slot, rootAsset.slot);
    EXPECT_EQ(detailRef.rootAsset.generation, rootAsset.generation);
    const PageHandle mount =
        world.attachPage(expansion, hierarchy.clonePage(details));
    EXPECT_TRUE(mount.valid());
    EXPECT_TRUE(world.isAttached(expansion));

    const DetailPageRef nestedDetail = world.detailPage(nodeAt(mount, 1));
    EXPECT_TRUE(nestedDetail.valid());
    EXPECT_EQ(nestedDetail.page, hierarchy.page(details).detailPage(1));
    EXPECT_EQ(nestedDetail.rootAsset.slot, rootAsset.slot);
    EXPECT_EQ(nestedDetail.rootAsset.generation, rootAsset.generation);

    world.removeInstance(instance);
    world.releaseAsset(rootAsset);
}

TEST(HierarchyBuilder, DetailPageReferencesAreScopedByRootAsset)
{
    HierarchyBuilder firstBuilder;
    const HierarchyBuilder::NodeId firstRoot =
        firstBuilder.createRoot(1, 8.0f, AABB::empty());
    const HierarchyBuilder::NodeId firstBranch =
        firstBuilder.createNode(firstRoot, 2, 4.0f, AABB::empty());
    firstBuilder.createNode(
        firstBranch, 3, 1.0f,
        AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(1, 1, 1)));
    firstBuilder.splitBelow(firstBranch);
    Hierarchy firstHierarchy = firstBuilder.build();

    HierarchyBuilder secondBuilder;
    const HierarchyBuilder::NodeId secondRoot =
        secondBuilder.createRoot(10, 8.0f, AABB::empty());
    const HierarchyBuilder::NodeId secondBranch =
        secondBuilder.createNode(secondRoot, 20, 4.0f, AABB::empty());
    secondBuilder.createNode(
        secondBranch, 30, 1.0f,
        AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(1, 1, 1)));
    secondBuilder.splitBelow(secondBranch);
    Hierarchy secondHierarchy = secondBuilder.build();

    SpatialDatabase world;
    const AssetHandle firstAsset = world.registerAsset(firstHierarchy.page(0));
    const AssetHandle secondAsset =
        world.registerAsset(secondHierarchy.page(0));
    const SpatialDatabase::InstanceRef firstInstance =
        world.addInstance(firstAsset, float4::point(0, 0, 0));
    const SpatialDatabase::InstanceRef secondInstance =
        world.addInstance(secondAsset, float4::point(10, 0, 0));

    const DetailPageRef firstDetail =
        world.detailPage(nodeAt(firstInstance.rootPage, 2));
    const DetailPageRef secondDetail =
        world.detailPage(nodeAt(secondInstance.rootPage, 2));
    ASSERT_TRUE(firstDetail.valid());
    ASSERT_TRUE(secondDetail.valid());
    EXPECT_EQ(firstDetail.page, 1u);
    EXPECT_EQ(secondDetail.page, 1u);
    EXPECT_NE(firstDetail.rootAsset.slot, secondDetail.rootAsset.slot);
    EXPECT_EQ(firstDetail.rootAsset.slot, firstAsset.slot);
    EXPECT_EQ(secondDetail.rootAsset.slot, secondAsset.slot);

    world.removeInstance(secondInstance);
    world.removeInstance(firstInstance);
    world.releaseAsset(secondAsset);
    world.releaseAsset(firstAsset);
}

TEST(Builder, ContractViolationsThrow)
{
    {
        PageBuilder b;
        EXPECT_THROW((void)b.build(), std::logic_error);   // no roots
    }
    {
        PageBuilder b;
        const auto r = b.createRoot(1, 1.0f);
        EXPECT_THROW((void)b.createNode(99, 2, 1.0f), std::logic_error);   // bad parent
        (void)r;
    }
    {
        PageBuilder b;
        const auto r = b.createRoot(1, 1.0f);
        const auto c = b.createNode(r, 2, 1.0f,
                                    AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(1, 1, 1)));
        b.markExpansion(c);
        EXPECT_THROW((void)b.createNode(c, 3, 1.0f), std::logic_error);   // child under expansion
    }
    {
        // Payloads are opaque user data: duplicates are allowed by design.
        PageBuilder b;
        const auto r = b.createRoot(1, 1.0f);
        b.createNode(r, 7, 1.0f, AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(1, 1, 1)));
        b.createNode(r, 7, 1.0f, AABB::fromCenterExtent(float4::vec(2, 0, 0), float4::vec(1, 1, 1)));
        EXPECT_NO_THROW((void)b.build());
    }
    {
        PageBuilder b;
        b.createRoot(1, 1.0f);   // leaf root with no bbox
        EXPECT_THROW((void)b.build(), std::logic_error);
    }
}

TEST(HierarchyBuilder, ContractViolationsThrow)
{
    {
        HierarchyBuilder builder;
        EXPECT_THROW((void)builder.build(), std::logic_error);
    }
    {
        HierarchyBuilder builder;
        builder.createRoot(
            1, 1.0f,
            AABB::fromCenterExtent(float4::vec(0, 0, 0),
                                   float4::vec(1, 1, 1)));
        EXPECT_THROW(
            (void)builder.createRoot(
                2, 1.0f,
                AABB::fromCenterExtent(float4::vec(0, 0, 0),
                                       float4::vec(1, 1, 1))),
            std::logic_error);
    }
    {
        HierarchyBuilder builder;
        const HierarchyBuilder::NodeId leaf = builder.createRoot(
            1, 1.0f,
            AABB::fromCenterExtent(float4::vec(0, 0, 0),
                                   float4::vec(1, 1, 1)));
        builder.splitBelow(leaf);
        EXPECT_THROW((void)builder.build(), std::logic_error);
    }
}
