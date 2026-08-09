#include <gtest/gtest.h>

#include "helpers.h"

using namespace hlod;
using namespace hlodtest;

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
    HLodBuilder b;
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
    HLodBuilder b;
    const auto root = b.createRoot(1, 4.0f);
    b.createNode(root, 2, 9.0f,   // bad authored error: larger than parent
                 AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(1, 1, 1)));
    const Page pg = b.build();
    EXPECT_FLOAT_EQ(pg.geometricError[2], 4.0f);   // clamped
    verifyInvariants(pg);
}

TEST(Builder, MultiRootForestPage)
{
    HLodBuilder b;
    for (int r = 0; r < 5; ++r)
        b.createRoot(uint64_t(100 + r), 2.0f,
                     AABB::fromCenterExtent(float4::vec(float(r) * 10, 0, 0), float4::vec(1, 1, 1)));
    const Page pg = b.build();
    EXPECT_EQ(pg.childCount(0), 5u);
    verifyInvariants(pg);
}

TEST(Builder, WideFanoutChainsBlocks)
{
    HLodBuilder b;
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

    World world;
    const AssetHandle rootAsset = world.registerAsset(hierarchy.page(0));
    const World::InstanceRef instance =
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

    World world;
    const AssetHandle firstAsset = world.registerAsset(firstHierarchy.page(0));
    const AssetHandle secondAsset =
        world.registerAsset(secondHierarchy.page(0));
    const World::InstanceRef firstInstance =
        world.addInstance(firstAsset, float4::point(0, 0, 0));
    const World::InstanceRef secondInstance =
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
        HLodBuilder b;
        EXPECT_THROW((void)b.build(), std::logic_error);   // no roots
    }
    {
        HLodBuilder b;
        const auto r = b.createRoot(1, 1.0f);
        EXPECT_THROW((void)b.createNode(99, 2, 1.0f), std::logic_error);   // bad parent
        (void)r;
    }
    {
        HLodBuilder b;
        const auto r = b.createRoot(1, 1.0f);
        const auto c = b.createNode(r, 2, 1.0f,
                                    AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(1, 1, 1)));
        b.markExpansion(c);
        EXPECT_THROW((void)b.createNode(c, 3, 1.0f), std::logic_error);   // child under expansion
    }
    {
        // Payloads are opaque user data: duplicates are allowed by design.
        HLodBuilder b;
        const auto r = b.createRoot(1, 1.0f);
        b.createNode(r, 7, 1.0f, AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(1, 1, 1)));
        b.createNode(r, 7, 1.0f, AABB::fromCenterExtent(float4::vec(2, 0, 0), float4::vec(1, 1, 1)));
        EXPECT_NO_THROW((void)b.build());
    }
    {
        HLodBuilder b;
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
