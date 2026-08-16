#include <gtest/gtest.h>

#include <array>
#include <limits>

#include "frontier/math.h"
#include "deterministic_rng.h"

using namespace frontier;
using namespace frontiertest;

TEST(DeterministicRandom, StableSequenceMappingAndShuffle)
{
    DeterministicRng integers(0x12345678u);
    constexpr std::array<uint32_t, 5> expected = {
        2274908837u, 358294691u, 1210119364u, 2176035992u, 1882851208u};
    for (uint32_t value : expected) EXPECT_EQ(integers.next(), value);

    DeterministicRng floats(0x12345678u);
    EXPECT_FLOAT_EQ(floats.uniform(-2.0f, 2.0f), 0.11867380142211914f);

    std::array<uint32_t, 8> order = {0, 1, 2, 3, 4, 5, 6, 7};
    DeterministicRng shuffle(0x42424242u);
    deterministicShuffle(order.begin(), order.end(), shuffle);
    constexpr std::array<uint32_t, 8> expectedOrder = {1, 5, 3, 2, 0, 6, 4, 7};
    EXPECT_EQ(order, expectedOrder);
}

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

TEST(Aabb, EmptyDetectionChecksEveryAxis)
{
    EXPECT_TRUE(AABB::fromMinMax(float4::vec(0, 1, 0),
                                 float4::vec(1, 0, 1)).isEmpty());
    EXPECT_TRUE(AABB::fromMinMax(float4::vec(0, 0, 1),
                                 float4::vec(1, 1, 0)).isEmpty());
}

TEST(Aabb, SquaredWideMatchesScalarEnvelopes)
{
    DeterministicRng rng(0xd157u);
    DeterministicUniformFloat centerDist(-1000.0f, 1000.0f);
    DeterministicUniformFloat extentDist(0.0f, 100.0f);

    for (int iteration = 0; iteration < 1000; ++iteration)
    {
        WideBounds bounds = WideBounds::allEmpty();
        AABB boxes[kWide];
        for (uint32_t lane = 0; lane < kWide; ++lane)
        {
            const float4 center = float4::vec(
                centerDist(rng), centerDist(rng), centerDist(rng));
            const float4 extent = float4::vec(
                extentDist(rng), extentDist(rng), extentDist(rng));
            boxes[lane] = AABB::fromCenterExtent(center, extent);
            bounds.setLane(lane, boxes[lane]);
        }

        const float4 queryCenter = float4::vec(
            centerDist(rng), centerDist(rng), centerDist(rng));
        const float4 queryExtent = float4::vec(
            extentDist(rng), extentDist(rng), extentDist(rng));
        const float4 queryMin = queryCenter - queryExtent;
        const float4 queryMax = queryCenter + queryExtent;
        const float8 actual = distanceToBoxesSq(bounds, queryMin, queryMax);

        for (uint32_t lane = 0; lane < kWide; ++lane)
        {
            const AABB& box = boxes[lane];
            float cx = std::max(std::max(box.mn.x - queryMax.x,
                                         queryMin.x - box.mx.x), 0.0f);
            float cy = std::max(std::max(box.mn.y - queryMax.y,
                                         queryMin.y - box.mx.y), 0.0f);
            float cz = std::max(std::max(box.mn.z - queryMax.z,
                                         queryMin.z - box.mx.z), 0.0f);
            const float expected =
                fmadd(cx, cx, fmadd(cy, cy, cz * cz));
            EXPECT_FLOAT_EQ(actual.v[lane], expected)
                << "iteration " << iteration << " lane " << lane;
        }
    }
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
    DeterministicRng rng(12345);
    DeterministicUniformFloat uni(-50.0f, 50.0f);
    DeterministicUniformFloat ext(0.1f, 20.0f);
    DeterministicUniformInt<int> maskDist(0, int(kAllPlanes));

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
    const AABB cases[8] = {
        AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(0, 0, 0)),        // point in view
        AABB::fromCenterExtent(float4::vec(0, 0, -100), float4::vec(0, 0, 0)),     // point behind
        AABB::fromCenterExtent(float4::vec(0, 0, -10), float4::vec(0, 0, 0)),      // point at camera
        AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(1e6f, 1e6f, 1e6f)),// encloses all
        AABB::fromCenterExtent(float4::vec(0, 0, -10), float4::vec(1, 1, 1)),      // encloses camera
        AABB::fromCenterExtent(float4::vec(5000, 0, 0), float4::vec(1, 1, 1)),     // far off-axis
        AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(1e-20f, 1e-20f, 1e-20f)),
        AABB::fromCenterExtent(float4::vec(1e7f, 1e7f, 1e7f), float4::vec(1, 1, 1)),
    };
    AABB boxes[kWide];
    for (uint32_t l = 0; l < kWide; ++l)
    {
        boxes[l] = cases[l];
        wb.setLane(l, boxes[l]);
    }

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
    EXPECT_FLOAT_EQ(distanceToBox(boxes[2], v.pos), 0.0f);
}

TEST(ScreenError, WideMatchesScalarFuzz)
{
    DeterministicRng rng(31337);
    DeterministicUniformFloat errDist(0.0f, 1.0e5f);
    DeterministicUniformFloat distDist(0.0f, 1.0e6f);

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

TEST(ScreenError, SquaredWideMatchesScalarWithinTolerance)
{
    DeterministicRng rng(0x51a7u);
    DeterministicUniformFloat errorDist(0.01f, 1000.0f);
    DeterministicUniformFloat exponentDist(-20.0f, 20.0f);

    float maxRelativeError = 0.0f;
    for (int iteration = 0; iteration < 2000; ++iteration)
    {
        const float k = 0.25f + float(iteration % 100);
        float8 geometricError, d2;
        for (uint32_t lane = 0; lane < kWide; ++lane)
        {
            geometricError.v[lane] = errorDist(rng);
            const float distance = std::exp2(exponentDist(rng));
            d2.v[lane] = distance * distance;
        }

        const float8 actual = screenErrorFromSq8(geometricError, k, d2);
        for (uint32_t lane = 0; lane < kWide; ++lane)
        {
            const float expected = geometricError.v[lane] * k /
                                   std::sqrt(d2.v[lane]);
            const float relative =
                std::fabs(actual.v[lane] - expected) / expected;
            maxRelativeError = std::max(maxRelativeError, relative);
        }
    }

    EXPECT_LT(maxRelativeError, 2.0e-5f);
}

TEST(ScreenError, ZeroDistanceSaturatesFinitePositive)
{
    // Camera sitting exactly on a node: the error must saturate to a huge
    // positive value (forces refinement), not NaN or a division fault.
    const float e = screenError(4.0f, 1000.0f, 0.0f);
    EXPECT_GT(e, 1.0e30f);
    EXPECT_FALSE(std::isnan(e));
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

TEST(Camera, PlanarYawTransformsBoundsPositionPlanesAndEnvelope)
{
    const AABB localBox = AABB::fromMinMax(
        float4::point(-1.0f, -0.5f, -3.0f),
        float4::point(2.0f, 0.5f, 4.0f));
    const float4 position = float4::point(10.0f, 1.0f, 20.0f);
    const YawRotation quarterTurn{0.0f, 1.0f};
    const AABB worldBox = toWorld(localBox, position, 2.0f, quarterTurn);
    EXPECT_FLOAT_EQ(worldBox.mn.x, 2.0f);
    EXPECT_FLOAT_EQ(worldBox.mx.x, 16.0f);
    EXPECT_FLOAT_EQ(worldBox.mn.y, 0.0f);
    EXPECT_FLOAT_EQ(worldBox.mx.y, 2.0f);
    EXPECT_FLOAT_EQ(worldBox.mn.z, 18.0f);
    EXPECT_FLOAT_EQ(worldBox.mx.z, 24.0f);

    Camera world{};
    world.pos = float4::point(14.0f, 5.0f, 26.0f);
    world.k = 600.0f;
    world.envLo = float4::vec(1.0f, 2.0f, 3.0f);
    world.envHi = float4::vec(5.0f, 4.0f, 7.0f);
    world.frustum.plane[0] = {1.0f, 0.0f, 0.0f, -5.0f};
    const Camera local = toLocal(world, position, 2.0f, quarterTurn, 1u);
    EXPECT_FLOAT_EQ(local.pos.x, 3.0f);
    EXPECT_FLOAT_EQ(local.pos.y, 2.0f);
    EXPECT_FLOAT_EQ(local.pos.z, -2.0f);
    EXPECT_FLOAT_EQ(local.frustum.plane[0].x, 0.0f);
    EXPECT_FLOAT_EQ(local.frustum.plane[0].z, -1.0f);
    EXPECT_FLOAT_EQ(local.frustum.plane[0].w, 2.5f);
    EXPECT_FLOAT_EQ(local.envLo.x, 1.5f);
    EXPECT_FLOAT_EQ(local.envHi.x, 3.5f);
    EXPECT_FLOAT_EQ(local.envLo.y, 1.0f);
    EXPECT_FLOAT_EQ(local.envHi.y, 2.0f);
    EXPECT_FLOAT_EQ(local.envLo.z, 2.5f);
    EXPECT_FLOAT_EQ(local.envHi.z, 0.5f);
}

TEST(Camera, DamperConstructorNormalizesInvalidHalfLife)
{
    CameraDamper negative(-4.0f);
    CameraDamper notANumber(std::numeric_limits<float>::quiet_NaN());
    CameraDamper infinite(std::numeric_limits<float>::infinity());
    EXPECT_FLOAT_EQ(negative.halfLife(), 0.0f);
    EXPECT_FLOAT_EQ(notANumber.halfLife(), 0.0f);
    EXPECT_FLOAT_EQ(infinite.halfLife(), 0.0f);
    EXPECT_EQ(sizeof(CameraDamper), 48u);
}

TEST(Camera, PointerHelpersRejectNullInput)
{
    EXPECT_THROW(float4x4::fromMemory(nullptr), std::logic_error);
    EXPECT_THROW(float4x4{}.coeffs(4), std::logic_error);
    EXPECT_THROW(cameraFromViewProjection(nullptr, {}, 1080.0f, 1.0f),
                 std::logic_error);
    EXPECT_THROW(cameraFromPlanes(nullptr, {}, 1.0f), std::logic_error);

    const float4x4 matrix{};
    EXPECT_THROW(cameraFromViewProjection(
                     matrix, {}, 1080.0f, 1.0f,
                     static_cast<ClipRange>(255)),
                 std::logic_error);
}
