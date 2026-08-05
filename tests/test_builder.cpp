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
