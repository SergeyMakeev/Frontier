#include <gtest/gtest.h>

#include <algorithm>
#include <random>

#include "hlod/math.h"

using namespace hlod;

TEST(Float8, LaneWiseOps)
{
    float8 a, b;
    for (uint32_t l = 0; l < kWide; ++l)
    {
        a.v[l] = float(l + 1);
        b.v[l] = 2.0f;
    }
    const float8 s = a + b, d = a - b, m = a * b, q = a / b;
    const float8 lo = min8(a, b), hi = max8(a, b);
    for (uint32_t l = 0; l < kWide; ++l)
    {
        EXPECT_FLOAT_EQ(s.v[l], float(l + 1) + 2.0f);
        EXPECT_FLOAT_EQ(d.v[l], float(l + 1) - 2.0f);
        EXPECT_FLOAT_EQ(m.v[l], float(l + 1) * 2.0f);
        EXPECT_FLOAT_EQ(q.v[l], float(l + 1) / 2.0f);
        EXPECT_FLOAT_EQ(lo.v[l], std::min(float(l + 1), 2.0f));
        EXPECT_FLOAT_EQ(hi.v[l], std::max(float(l + 1), 2.0f));
    }
}

TEST(Aabb, ExpandContains)
{
    AABB a = AABB::empty();
    EXPECT_TRUE(a.isEmpty());
    a.expand(AABB::fromMinMax(float4::vec(0, 0, 0), float4::vec(1, 1, 1)));
    a.expand(AABB::fromMinMax(float4::vec(-1, 0, 0), float4::vec(0.5f, 2, 1)));
    EXPECT_FALSE(a.isEmpty());
    EXPECT_TRUE(a.contains(AABB::fromMinMax(float4::vec(0, 0, 0), float4::vec(1, 1, 1))));
    EXPECT_FALSE(a.contains(AABB::fromMinMax(float4::vec(0, 0, 0), float4::vec(3, 1, 1))));
    EXPECT_TRUE(a.contains(AABB::empty()));
}

TEST(Aabb, DistanceToBox)
{
    const AABB b = AABB::fromMinMax(float4::vec(-1, -1, -1), float4::vec(1, 1, 1));
    EXPECT_FLOAT_EQ(distanceToBox(b, float4::point(0, 0, 0)), 0.0f);      // inside
    EXPECT_FLOAT_EQ(distanceToBox(b, float4::point(3, 0, 0)), 2.0f);      // face
    EXPECT_FLOAT_EQ(distanceToBox(b, float4::point(4, 5, 1)),
                    std::sqrt(9.0f + 16.0f));                             // edge
}

TEST(Frustum, TriStateScalar)
{
    const Camera v = makeLookAtCamera(float4::point(0, 0, -10), float4::point(0, 0, 0));

    uint8_t mask = kAllPlanes;
    EXPECT_EQ(testAabb(AABB::fromMinMax(float4::vec(-1, -1, -1), float4::vec(1, 1, 1)),
                       v.frustum, mask),
              CullState::Inside);
    EXPECT_EQ(mask, 0);

    mask = kAllPlanes;
    EXPECT_EQ(testAabb(AABB::fromMinMax(float4::vec(-1, -1, -30), float4::vec(1, 1, -25)),
                       v.frustum, mask),
              CullState::Outside);   // behind the camera

    mask = kAllPlanes;
    EXPECT_EQ(testAabb(AABB::fromMinMax(float4::vec(-1000, -1, -1), float4::vec(1, 1, 1)),
                       v.frustum, mask),
              CullState::Partial);   // pokes out one side
    EXPECT_NE(mask, 0);
    EXPECT_NE(mask, kAllPlanes);     // some planes were settled
}

TEST(Frustum, MaskedTestSkipsClearedPlanes)
{
    const Camera v = makeLookAtCamera(float4::point(0, 0, -10), float4::point(0, 0, 0));
    // With an empty mask, even a far-away box "passes" — the caller
    // guarantees an ancestor was fully inside.
    uint8_t mask = 0;
    EXPECT_EQ(testAabb(AABB::fromMinMax(float4::vec(500, 500, 500), float4::vec(501, 501, 501)),
                       v.frustum, mask),
              CullState::Inside);
}

TEST(Frustum, WideMatchesScalarOnRandomBoxes)
{
    std::mt19937 rng(12345);
    std::uniform_real_distribution<float> uni(-50.0f, 50.0f);
    std::uniform_real_distribution<float> ext(0.1f, 20.0f);
    std::uniform_int_distribution<int> maskDist(0, kAllPlanes);

    const Camera v = makeLookAtCamera(float4::point(5, -3, -40), float4::point(0, 0, 0));

    for (int iter = 0; iter < 500; ++iter)
    {
        WideBounds wb = WideBounds::allEmpty();
        AABB boxes[kWide];
        const uint8_t inMask = uint8_t(maskDist(rng));
        for (uint32_t l = 0; l < kWide; ++l)
        {
            const float4 c = float4::vec(uni(rng), uni(rng), uni(rng));
            const float4 e = float4::vec(ext(rng), ext(rng), ext(rng));
            boxes[l] = AABB::fromCenterExtent(c, e);
            wb.setLane(l, boxes[l]);
        }

        uint8_t outMasks[kWide];
        const uint32_t survivors = testWideAabb(wb, v.frustum, inMask, outMasks);
        const float8 dist = distanceToBoxes(wb, v.pos);

        for (uint32_t l = 0; l < kWide; ++l)
        {
            uint8_t m = inMask;
            const CullState st = testAabb(boxes[l], v.frustum, m);
            EXPECT_EQ((survivors >> l) & 1u, st != CullState::Outside ? 1u : 0u)
                << "iter " << iter << " lane " << l;
            if (st != CullState::Outside)
                EXPECT_EQ(outMasks[l], m) << "iter " << iter << " lane " << l;
            EXPECT_FLOAT_EQ(dist.v[l], distanceToBox(boxes[l], v.pos))
                << "iter " << iter << " lane " << l;
        }
    }
}

TEST(Frustum, WideMatchesScalarOnDegenerateBoxes)
{
    // Zero-extent (point) boxes, camera-enclosing boxes, and huge boxes must
    // agree between the scalar and wide paths, with exact tri-state masks.
    const Camera v = makeLookAtCamera(float4::point(0, 0, -10), float4::point(0, 0, 0));

    WideBounds wb = WideBounds::allEmpty();
    AABB boxes[kWide] = {
        AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(0, 0, 0)),        // point in view
        AABB::fromCenterExtent(float4::vec(0, 0, -100), float4::vec(0, 0, 0)),     // point behind
        AABB::fromCenterExtent(float4::vec(0, 0, -10), float4::vec(0, 0, 0)),      // point at camera
        AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(1e6f, 1e6f, 1e6f)),// encloses all
        AABB::fromCenterExtent(float4::vec(0, 0, -10), float4::vec(1, 1, 1)),      // encloses camera
        AABB::fromCenterExtent(float4::vec(5000, 0, 0), float4::vec(1, 1, 1)),     // far off-axis
        AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(1e-20f, 1e-20f, 1e-20f)),
        AABB::fromCenterExtent(float4::vec(1e7f, 1e7f, 1e7f), float4::vec(1, 1, 1)),
    };
    for (uint32_t l = 0; l < kWide; ++l) wb.setLane(l, boxes[l]);

    uint8_t outMasks[kWide];
    const uint32_t survivors = testWideAabb(wb, v.frustum, kAllPlanes, outMasks);
    const float8 dist = distanceToBoxes(wb, v.pos);

    for (uint32_t l = 0; l < kWide; ++l)
    {
        uint8_t m = kAllPlanes;
        const CullState st = testAabb(boxes[l], v.frustum, m);
        EXPECT_EQ((survivors >> l) & 1u, st != CullState::Outside ? 1u : 0u) << "lane " << l;
        if (st != CullState::Outside) EXPECT_EQ(outMasks[l], m) << "lane " << l;
        EXPECT_FLOAT_EQ(dist.v[l], distanceToBox(boxes[l], v.pos)) << "lane " << l;
    }
    // Camera inside a box: distance saturates to zero, never negative/NaN.
    EXPECT_FLOAT_EQ(distanceToBox(boxes[4], v.pos), 0.0f);
}

TEST(ScreenError, Wide8MatchesScalarFuzz)
{
    std::mt19937 rng(31337);
    std::uniform_real_distribution<float> errDist(0.0f, 1.0e5f);
    std::uniform_real_distribution<float> distDist(0.0f, 1.0e6f);

    for (int iter = 0; iter < 500; ++iter)
    {
        const float k = 100.0f + float(iter);
        float8 ge, d;
        for (uint32_t l = 0; l < kWide; ++l)
        {
            ge.v[l] = errDist(rng);
            d.v[l] = (iter % 7 == 0 && l == 3) ? 0.0f : distDist(rng);   // exercise the clamp
        }
        const float8 wide = screenError8(ge, k, d);
        for (uint32_t l = 0; l < kWide; ++l)
            EXPECT_FLOAT_EQ(wide.v[l], screenError(ge.v[l], k, d.v[l]))
                << "iter " << iter << " lane " << l;
    }
}

TEST(ScreenError, ZeroDistanceSaturatesFinitePositive)
{
    // Camera sitting exactly on a node: the error must saturate to a huge
    // positive value (forces refinement), not NaN or a division fault.
    const float e = screenError(4.0f, 1000.0f, 0.0f);
    EXPECT_GT(e, 1.0e30f);
    EXPECT_FALSE(std::isnan(e));
}

TEST(ScreenError, SquaredWideMatchesScalarFuzz)
{
    std::mt19937 rng(98761);
    std::uniform_real_distribution<float> errDist(0.01f, 1.0e4f);
    std::uniform_real_distribution<float> distDist(1.0e-4f, 1.0e5f);

    for (int iter = 0; iter < 500; ++iter)
    {
        const float k = 10.0f + float(iter);
        float8 ge, d2;
        for (uint32_t l = 0; l < kWide; ++l)
        {
            ge.v[l] = errDist(rng);
            const float d = distDist(rng);
            d2.v[l] = d * d;
        }
        if ((iter % 7) == 0) d2.v[3] = 0.0f;

        const float8 wide = screenErrorFromSq8(ge, k, d2);
        for (uint32_t l = 0; l < kWide; ++l)
        {
            const float expected = screenError(ge.v[l], k, std::sqrt(d2.v[l]));
            const float tolerance = std::max(1.0e-5f, std::abs(expected) * 5.0e-5f);
            EXPECT_NEAR(wide.v[l], expected, tolerance)
                << "iter " << iter << " lane " << l;
            EXPECT_TRUE(std::isfinite(wide.v[l]))
                << "iter " << iter << " lane " << l;
        }
    }
}

TEST(Aabb, SquaredWideDistanceMatchesScalarFuzz)
{
    std::mt19937 rng(45123);
    std::uniform_real_distribution<float> pos(-1000.0f, 1000.0f);
    std::uniform_real_distribution<float> ext(0.0f, 30.0f);

    for (int iter = 0; iter < 500; ++iter)
    {
        WideBounds wb = WideBounds::allEmpty();
        AABB boxes[kWide];
        for (uint32_t l = 0; l < kWide; ++l)
        {
            boxes[l] = AABB::fromCenterExtent(
                float4::vec(pos(rng), pos(rng), pos(rng)),
                float4::vec(ext(rng), ext(rng), ext(rng)));
            wb.setLane(l, boxes[l]);
        }
        const float4 q = float4::point(pos(rng), pos(rng), pos(rng));
        const float8 wide = distanceToBoxesSq(wb, q, q);
        for (uint32_t l = 0; l < kWide; ++l)
        {
            const float d = distanceToBox(boxes[l], q);
            const float expected = d * d;
            const float tolerance = std::max(1.0e-4f, expected * 2.0e-6f);
            EXPECT_NEAR(wide.v[l], expected, tolerance)
                << "iter " << iter << " lane " << l;
        }
    }
}

TEST(Camera, LocalTransformIsScaleInvariant)
{
    const Camera v = makeLookAtCamera(float4::point(0, 10, -40), float4::point(0, 0, 0));
    const float4 instPos = float4::point(3, -2, 7);
    const float scale = 2.5f;
    const Camera local = toLocal(v, instPos, scale);

    // A local box and its world image must agree on culling and screen error.
    const AABB localBox = AABB::fromCenterExtent(float4::vec(1, 2, 3), float4::vec(1, 1, 1));
    const AABB worldBox = toWorld(localBox, instPos, scale);

    uint8_t m0 = kAllPlanes, m1 = kAllPlanes;
    EXPECT_EQ(testAabb(worldBox, v.frustum, m0), testAabb(localBox, local.frustum, m1));

    const float ge = 0.7f;
    const float errWorld = screenError(ge * scale, v.k, distanceToBox(worldBox, v.pos));
    const float errLocal = screenError(ge, local.k, distanceToBox(localBox, local.pos));
    EXPECT_NEAR(errWorld, errLocal, 1e-3f * errWorld);
}
