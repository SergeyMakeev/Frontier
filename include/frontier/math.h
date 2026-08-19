#pragma once
// Minimal SIMD-friendly math for Frontier. float4 is the public vector
// interface; float8 and WideBounds form the build-configured four- or
// eight-lane working set used by the AVX2, SSE2, NEON, and scalar backends.

#include <bit>
#include <cassert>
#include <cfloat>
#include <cmath>
#include <cstdint>

#include "config.h"

// ---------------------------------------------------------------------------
// Bring your own vector types.
//
// Define FRONTIER_USE_CUSTOM_VECTOR_TYPES and declare frontier::float4 and
// frontier::float4x4 (typically `using` aliases for your engine's types) before
// including any frontier header. A conforming float4 must be 16 bytes, 16-byte
// aligned, expose public float x, y, z, w, and support the free functions
// declared in the block below. The static_asserts after the block catch the
// common mistakes.
//
// float8 / WideBounds are deliberately NOT swappable: they are the internal
// SIMD working set, not an interface type.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// SIMD backend selection. At most one of FRONTIER_SIMD_AVX2 / FRONTIER_SIMD_NEON /
// FRONTIER_SIMD_SSE2 is defined; with none of them the wide paths fall back to
// portable scalar loops. Define FRONTIER_FORCE_SCALAR (CMake option of the same
// name) to force the scalar fallback everywhere. FRONTIER_SSE2_ONLY selects
// only the SSE2 backend on x86 even when ambient compiler macros expose a
// higher ISA; CMake also constrains compiler-generated code in that mode.
// ---------------------------------------------------------------------------

#if !defined(FRONTIER_FORCE_SCALAR)
  #if FRONTIER_SSE2_ONLY
    #include <immintrin.h>
    #define FRONTIER_SIMD_SSE2 1
  #elif FRONTIER_BVH_WIDTH == 8 && defined(__AVX2__)
    #include <immintrin.h>
    #define FRONTIER_SIMD_AVX2 1
  #elif defined(__aarch64__) || defined(_M_ARM64) || defined(_M_ARM64EC)
    // 64-bit ARM only: the wide paths use vaddvq/vdivq/vsqrtq, which 32-bit
    // NEON lacks. armv7 falls back to the scalar loops.
    #include <arm_neon.h>
    #define FRONTIER_SIMD_NEON 1
  #elif defined(__SSE2__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
    #include <immintrin.h>
    #define FRONTIER_SIMD_SSE2 1
    #if defined(__SSE4_1__) || defined(__AVX__)
      #define FRONTIER_SIMD_SSE41 1
    #endif
  #endif
#endif

namespace frontier {

// Fused multiply-add matching the active wide path. The unit tests require
// scalar and wide results to be bit-identical, so the scalar code must fuse
// exactly when the wide intrinsics fuse. The only backend without hardware
// FMA is plain SSE; there mul+add is used on both sides (and the target has
// no fma instruction the compiler could contract to, keeping them in sync).
#if FRONTIER_SIMD_SSE2 && (!defined(__FMA__) || FRONTIER_SSE2_ONLY)
inline float fmadd(float a, float b, float c) { return a * b + c; }
#else
inline float fmadd(float a, float b, float c) { return std::fma(a, b, c); }
#endif

inline constexpr uint32_t kWide = FRONTIER_BVH_WIDTH;

// ---------------------------------------------------------------------------
// float4
// ---------------------------------------------------------------------------

#ifndef FRONTIER_USE_CUSTOM_VECTOR_TYPES

struct alignas(16) float4
{
    float x = 0.0f, y = 0.0f, z = 0.0f, w = 0.0f;

    static float4 splat(float s) { return {s, s, s, s}; }
    static float4 point(float px, float py, float pz) { return {px, py, pz, 1.0f}; }
    static float4 vec(float vx, float vy, float vz) { return {vx, vy, vz, 0.0f}; }
};

inline float4 operator+(float4 a, float4 b) { return {a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w}; }
inline float4 operator-(float4 a, float4 b) { return {a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w}; }
inline float4 operator*(float4 a, float s)  { return {a.x * s, a.y * s, a.z * s, a.w * s}; }
inline float4 operator/(float4 a, float s)  { return a * (1.0f / s); }

inline float4 min4(float4 a, float4 b)
{
    return {a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y,
            a.z < b.z ? a.z : b.z, a.w < b.w ? a.w : b.w};
}
inline float4 max4(float4 a, float4 b)
{
    return {a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y,
            a.z > b.z ? a.z : b.z, a.w > b.w ? a.w : b.w};
}

inline float dot3(float4 a, float4 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline float4 cross3(float4 a, float4 b)
{
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x, 0.0f};
}
inline float length3(float4 a) { return std::sqrt(dot3(a, a)); }
inline float4 normalize3(float4 a) { return a / length3(a); }

// 4x4 matrix, stored as the 16 floats a graphics API hands you.
//
// The storage convention is deliberately the one that row-vector row-major
// (DirectXMath) and column-vector column-major (glm/GL) agree on: in BOTH,
// the coefficients that produce clip.x sit at memory indices 0, 4, 8, 12. So
// a single fromMemory() serves both conventions and there is no flag
// to get wrong.
struct alignas(16) float4x4
{
    float m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

    static float4x4 fromMemory(const float* m16)
    {
        FRONTIER_CHECK(m16 != nullptr, "float4x4::fromMemory: null matrix");
        float4x4 r;
        for (int i = 0; i < 16; ++i) r.m[i] = m16[i];
        return r;
    }

    // Coefficient vector for clip-space component c (0=x, 1=y, 2=z, 3=w):
    // clip[c] = dot(coeffs(c), (p.x, p.y, p.z, 1)).
    float4 coeffs(int c) const
    {
        FRONTIER_CHECK(c >= 0 && c < 4,
                       "float4x4::coeffs: component out of range");
        return {m[c], m[c + 4], m[c + 8], m[c + 12]};
    }
};

#endif // FRONTIER_USE_CUSTOM_VECTOR_TYPES

static_assert(sizeof(float4) == 16, "frontier::float4 must be 16 bytes");
static_assert(alignof(float4) >= 16, "frontier::float4 must be 16-byte aligned");

// ---------------------------------------------------------------------------
// float8: lane-wise; lanes carry independent values.
// ---------------------------------------------------------------------------

// The historical name remains source-compatible; the actual lane count is
// kWide, so a BVH4 build stores one 128-bit vector here.
struct alignas(kWide * sizeof(float)) float8
{
    float v[kWide];

    static float8 splat(float s)
    {
        float8 r;
        for (uint32_t l = 0; l < kWide; ++l) r.v[l] = s;
        return r;
    }
    float  operator[](uint32_t l) const { return v[l]; }
    float& operator[](uint32_t l)       { return v[l]; }
};

inline float8 operator+(const float8& a, const float8& b)
{
    float8 r;
    for (uint32_t l = 0; l < kWide; ++l) r.v[l] = a.v[l] + b.v[l];
    return r;
}
inline float8 operator-(const float8& a, const float8& b)
{
    float8 r;
    for (uint32_t l = 0; l < kWide; ++l) r.v[l] = a.v[l] - b.v[l];
    return r;
}
inline float8 operator*(const float8& a, const float8& b)
{
    float8 r;
    for (uint32_t l = 0; l < kWide; ++l) r.v[l] = a.v[l] * b.v[l];
    return r;
}
inline float8 operator/(const float8& a, const float8& b)
{
    float8 r;
    for (uint32_t l = 0; l < kWide; ++l) r.v[l] = a.v[l] / b.v[l];
    return r;
}
inline float8 min8(const float8& a, const float8& b)
{
    float8 r;
    for (uint32_t l = 0; l < kWide; ++l) r.v[l] = a.v[l] < b.v[l] ? a.v[l] : b.v[l];
    return r;
}
inline float8 max8(const float8& a, const float8& b)
{
    float8 r;
    for (uint32_t l = 0; l < kWide; ++l) r.v[l] = a.v[l] > b.v[l] ? a.v[l] : b.v[l];
    return r;
}
inline float8 sqrt8(const float8& a)
{
    float8 r;
    for (uint32_t l = 0; l < kWide; ++l) r.v[l] = std::sqrt(a.v[l]);
    return r;
}

// ---------------------------------------------------------------------------
// AABB
// ---------------------------------------------------------------------------

struct AABB
{
    float4 mn = { FLT_MAX,  FLT_MAX,  FLT_MAX, 0.0f};
    float4 mx = {-FLT_MAX, -FLT_MAX, -FLT_MAX, 0.0f};

    static AABB empty() { return {}; }
    static AABB fromMinMax(float4 lo, float4 hi) { return {lo, hi}; }
    static AABB fromCenterExtent(float4 c, float4 e) { return {c - e, c + e}; }

    bool isEmpty() const
    {
        return mn.x > mx.x || mn.y > mx.y || mn.z > mx.z;
    }

    void expand(const AABB& o)
    {
        mn = min4(mn, o.mn);
        mx = max4(mx, o.mx);
    }
    void expand(float4 p)
    {
        mn = min4(mn, p);
        mx = max4(mx, p);
    }

    // True containment (empty contains nothing, everything contains empty).
    bool contains(const AABB& o) const
    {
        if (o.isEmpty()) return true;
        return mn.x <= o.mn.x && mn.y <= o.mn.y && mn.z <= o.mn.z &&
               mx.x >= o.mx.x && mx.y >= o.mx.y && mx.z >= o.mx.z;
    }

    float4 center() const { return (mn + mx) * 0.5f; }
    float4 extent() const { return (mx - mn) * 0.5f; }
};

// Box-to-box separation: distance from [qmn, qmx] to b, 0 when they touch or
// overlap. The LOD path queries with the camera *envelope* (where the camera
// has been recently) rather than a bare position; see CameraDamper below.
//
// Collapsing the query box to a point (qmn == qmx == p) reduces this to the
// textbook point-to-box distance term for term, so the damped and undamped
// paths are the same code and cannot drift apart.
//
// NOTE: written with fmadd in exactly the order the wide path uses, so
// scalar and wide results are bit-identical (the unit tests rely on it).
inline float distanceToBox(const AABB& b, float4 qmn, float4 qmx)
{
    float dx = b.mn.x - qmx.x; float ex = qmn.x - b.mx.x;
    float dy = b.mn.y - qmx.y; float ey = qmn.y - b.mx.y;
    float dz = b.mn.z - qmx.z; float ez = qmn.z - b.mx.z;
    float cx = dx > ex ? dx : ex; cx = cx > 0.0f ? cx : 0.0f;
    float cy = dy > ey ? dy : ey; cy = cy > 0.0f ? cy : 0.0f;
    float cz = dz > ez ? dz : ez; cz = cz > 0.0f ? cz : 0.0f;
    return std::sqrt(fmadd(cx, cx, fmadd(cy, cy, cz * cz)));
}

// Compact planar orientation used by top-level instance placements. Keeping
// cosine/sine instead of an angle removes trigonometry from per-frame motion
// submission and gives traversal the inverse rotation directly. The pair is
// required to be finite and unit length.
struct YawRotation
{
    float cosine = 1.0f;
    float sine = 0.0f;
};
static_assert(sizeof(YawRotation) == 8, "yaw rotation must stay two floats");

inline YawRotation yawRotation(float radians)
{
    return {std::cos(radians), std::sin(radians)};
}

// Point-to-box distance; 0 when the point is inside.
inline float distanceToBox(const AABB& b, float4 p) { return distanceToBox(b, p, p); }

// ---------------------------------------------------------------------------
// Frustum: 6 planes as float4 (xyz = inward normal, w = offset).
// A point p is inside plane i when dot3(n, p) + w >= 0.
// ---------------------------------------------------------------------------

struct Frustum
{
    float4 plane[6] = {};
};

inline constexpr uint8_t kAllPlanes = 0x3F;

enum class CullState : uint8_t { Outside, Partial, Inside };

// Scalar tri-state test against the planes still set in ioMask.
//   Outside: some active plane fully rejects the box.
//   Inside:  every active plane fully contains it (ioMask becomes 0).
//   Partial: otherwise; ioMask is narrowed to the still-undecided planes.
inline CullState testAabb(const AABB& b, const Frustum& fr, uint8_t& ioMask)
{
    uint8_t mask = ioMask;
    for (uint32_t p = 0; p < 6; ++p)
    {
        if (!((mask >> p) & 1)) continue;
        const float4 pl = fr.plane[p];
        // p-vertex: corner maximizing the signed distance. The fmadd chain
        // matches the wide path bit-for-bit.
        const float px = pl.x >= 0.0f ? b.mx.x : b.mn.x;
        const float py = pl.y >= 0.0f ? b.mx.y : b.mn.y;
        const float pz = pl.z >= 0.0f ? b.mx.z : b.mn.z;
        if (fmadd(pl.x, px, fmadd(pl.y, py, fmadd(pl.z, pz, pl.w))) < 0.0f)
        {
            ioMask = mask;
            return CullState::Outside;
        }
        // n-vertex: corner minimizing it. Inside this plane => stop testing it.
        const float nx = pl.x >= 0.0f ? b.mn.x : b.mx.x;
        const float ny = pl.y >= 0.0f ? b.mn.y : b.mx.y;
        const float nz = pl.z >= 0.0f ? b.mn.z : b.mx.z;
        if (fmadd(pl.x, nx, fmadd(pl.y, ny, fmadd(pl.z, nz, pl.w))) >= 0.0f)
            mask = uint8_t(mask & ~(1u << p));
    }
    ioMask = mask;
    return mask ? CullState::Partial : CullState::Inside;
}

// ---------------------------------------------------------------------------
// Wide bounds: kWide boxes, SoA-transposed (lanes = boxes)
// ---------------------------------------------------------------------------

struct WideBounds
{
    float8 mnx, mny, mnz;
    float8 mxx, mxy, mxz;

    static WideBounds allEmpty()
    {
        WideBounds w;
        w.mnx = w.mny = w.mnz = float8::splat(FLT_MAX);
        w.mxx = w.mxy = w.mxz = float8::splat(-FLT_MAX);
        return w;
    }
    void setLane(uint32_t l, const AABB& b)
    {
        mnx.v[l] = b.mn.x; mny.v[l] = b.mn.y; mnz.v[l] = b.mn.z;
        mxx.v[l] = b.mx.x; mxy.v[l] = b.mx.y; mxz.v[l] = b.mx.z;
    }
    AABB lane(uint32_t l) const
    {
        return AABB::fromMinMax({mnx.v[l], mny.v[l], mnz.v[l], 0.0f},
                                {mxx.v[l], mxy.v[l], mxz.v[l], 0.0f});
    }
};

#if FRONTIER_SIMD_NEON
namespace detail {
// Lane bitmask (bit l set when lane l is all-ones), like _mm_movemask_ps.
// (vld1q rather than brace-init: MSVC has no NEON vector literals.)
inline uint32_t movemask4(uint32x4_t m)
{
    alignas(16) static constexpr uint32_t kBits[4] = {1u, 2u, 4u, 8u};
    return vaddvq_u32(vandq_u32(m, vld1q_u32(kBits)));
}
} // namespace detail
#endif

#if FRONTIER_SIMD_SSE2
namespace detail {
// mask ? a : b, per lane; mask lanes are all-ones or all-zeros.
inline __m128 select4(__m128 mask, __m128 a, __m128 b)
{
#if FRONTIER_SIMD_SSE41
    return _mm_blendv_ps(b, a, mask);
#else
    return _mm_or_ps(_mm_and_ps(mask, a), _mm_andnot_ps(mask, b));
#endif
}
// Fused only when the target has FMA, mirroring frontier::fmadd.
inline __m128 fmadd4(__m128 a, __m128 b, __m128 c)
{
#if defined(__FMA__) && !FRONTIER_SSE2_ONLY
    return _mm_fmadd_ps(a, b, c);
#else
    return _mm_add_ps(_mm_mul_ps(a, b), c);
#endif
}
} // namespace detail
#endif

// Wide masked tri-state: kWide boxes against the planes set in inMask.
// Returns the survivor lane bitmask; outMasks[l] is lane l's narrowed plane
// mask (valid only for surviving lanes). inMask == 0 means an ancestor was
// fully inside: every lane survives untested with mask 0.
#if FRONTIER_SIMD_AVX2
inline uint32_t testWideAabb(const WideBounds& b, const Frustum& fr,
                             uint8_t inMask, uint8_t outMasks[kWide])
{
    for (uint32_t l = 0; l < kWide; ++l) outMasks[l] = inMask;
    if (!inMask) return (1u << kWide) - 1;

    const __m256 mnx = _mm256_load_ps(b.mnx.v), mny = _mm256_load_ps(b.mny.v);
    const __m256 mnz = _mm256_load_ps(b.mnz.v), mxx = _mm256_load_ps(b.mxx.v);
    const __m256 mxy = _mm256_load_ps(b.mxy.v), mxz = _mm256_load_ps(b.mxz.v);
    const __m256 zero = _mm256_setzero_ps();

    uint32_t alive = (1u << kWide) - 1;
    for (uint32_t p = 0; p < 6; ++p)
    {
        if (!((inMask >> p) & 1)) continue;
        const float4 pl = fr.plane[p];
        const __m256 nx = _mm256_set1_ps(pl.x), ny = _mm256_set1_ps(pl.y);
        const __m256 nz = _mm256_set1_ps(pl.z), d = _mm256_set1_ps(pl.w);
        // Sign masks select the p-/n-vertex per axis (constant per plane).
        const __m256 sx = _mm256_cmp_ps(nx, zero, _CMP_LT_OQ);
        const __m256 sy = _mm256_cmp_ps(ny, zero, _CMP_LT_OQ);
        const __m256 sz = _mm256_cmp_ps(nz, zero, _CMP_LT_OQ);

        const __m256 px = _mm256_blendv_ps(mxx, mnx, sx);
        const __m256 py = _mm256_blendv_ps(mxy, mny, sy);
        const __m256 pz = _mm256_blendv_ps(mxz, mnz, sz);
        const __m256 dp = _mm256_fmadd_ps(
            nx, px, _mm256_fmadd_ps(ny, py, _mm256_fmadd_ps(nz, pz, d)));
        alive &= ~uint32_t(_mm256_movemask_ps(_mm256_cmp_ps(dp, zero, _CMP_LT_OQ)));

        const __m256 qx = _mm256_blendv_ps(mnx, mxx, sx);
        const __m256 qy = _mm256_blendv_ps(mny, mxy, sy);
        const __m256 qz = _mm256_blendv_ps(mnz, mxz, sz);
        const __m256 dn = _mm256_fmadd_ps(
            nx, qx, _mm256_fmadd_ps(ny, qy, _mm256_fmadd_ps(nz, qz, d)));
        uint32_t inside =
            uint32_t(_mm256_movemask_ps(_mm256_cmp_ps(dn, zero, _CMP_GE_OQ)));
        while (inside)
        {
            const uint32_t l = uint32_t(std::countr_zero(inside));
            inside &= inside - 1;
            outMasks[l] = uint8_t(outMasks[l] & ~(1u << p));
        }
    }
    return alive & ((1u << kWide) - 1);
}

#elif FRONTIER_SIMD_NEON
inline uint32_t testWideAabb(const WideBounds& b, const Frustum& fr,
                             uint8_t inMask, uint8_t outMasks[kWide])
{
    for (uint32_t l = 0; l < kWide; ++l) outMasks[l] = inMask;
    if (!inMask) return (1u << kWide) - 1;

    constexpr uint32_t kSimdGroups = kWide / 4;
    float32x4_t mnx[kSimdGroups], mny[kSimdGroups], mnz[kSimdGroups];
    float32x4_t mxx[kSimdGroups], mxy[kSimdGroups], mxz[kSimdGroups];
    for (uint32_t h = 0; h < kSimdGroups; ++h)
    {
        mnx[h] = vld1q_f32(b.mnx.v + 4 * h);
        mny[h] = vld1q_f32(b.mny.v + 4 * h);
        mnz[h] = vld1q_f32(b.mnz.v + 4 * h);
        mxx[h] = vld1q_f32(b.mxx.v + 4 * h);
        mxy[h] = vld1q_f32(b.mxy.v + 4 * h);
        mxz[h] = vld1q_f32(b.mxz.v + 4 * h);
    }
    const float32x4_t zero = vdupq_n_f32(0.0f);

    // Keep both survivor state and the narrowed per-lane plane masks in NEON
    // registers for the whole test. Scalarizing every comparison costs a
    // horizontal reduction plus a vector-to-GPR transfer on AArch64; doing it
    // twice per half, per plane, dominated this otherwise compact kernel.
    uint32x4_t alive[kSimdGroups];
    uint32x4_t remaining[kSimdGroups];
    for (uint32_t h = 0; h < kSimdGroups; ++h)
    {
        alive[h] = vdupq_n_u32(~0u);
        remaining[h] = vdupq_n_u32(inMask);
    }
    for (uint32_t p = 0; p < 6; ++p)
    {
        if (!((inMask >> p) & 1)) continue;
        const float4 pl = fr.plane[p];
        const float32x4_t nx = vdupq_n_f32(pl.x), ny = vdupq_n_f32(pl.y);
        const float32x4_t nz = vdupq_n_f32(pl.z), d = vdupq_n_f32(pl.w);
        const uint32x4_t planeBit = vdupq_n_u32(1u << p);
        // Sign masks select the p-/n-vertex per axis (constant per plane).
        const uint32x4_t sx = vcltq_f32(nx, zero);
        const uint32x4_t sy = vcltq_f32(ny, zero);
        const uint32x4_t sz = vcltq_f32(nz, zero);

        for (uint32_t h = 0; h < kSimdGroups; ++h)
        {
            const float32x4_t px = vbslq_f32(sx, mnx[h], mxx[h]);
            const float32x4_t py = vbslq_f32(sy, mny[h], mxy[h]);
            const float32x4_t pz = vbslq_f32(sz, mnz[h], mxz[h]);
            // vfmaq_f32(c, a, b) = c + a*b, fused: matches std::fma.
            const float32x4_t dp =
                vfmaq_f32(vfmaq_f32(vfmaq_f32(d, nz, pz), ny, py), nx, px);
            alive[h] = vandq_u32(alive[h], vcgeq_f32(dp, zero));

            const float32x4_t qx = vbslq_f32(sx, mxx[h], mnx[h]);
            const float32x4_t qy = vbslq_f32(sy, mxy[h], mny[h]);
            const float32x4_t qz = vbslq_f32(sz, mxz[h], mnz[h]);
            const float32x4_t dn =
                vfmaq_f32(vfmaq_f32(vfmaq_f32(d, nz, qz), ny, qy), nx, qx);
            const uint32x4_t inside = vcgeq_f32(dn, zero);
            remaining[h] = vbicq_u32(
                remaining[h], vandq_u32(inside, planeBit));
        }
    }

#if FRONTIER_BVH_WIDTH == 8
    const uint16x8_t remaining16 = vcombine_u16(
        vmovn_u32(remaining[0]), vmovn_u32(remaining[1]));
    vst1_u8(outMasks, vmovn_u16(remaining16));
    return detail::movemask4(alive[0]) |
           (detail::movemask4(alive[1]) << 4);
#else
    alignas(16) uint32_t remainingLanes[4];
    vst1q_u32(remainingLanes, remaining[0]);
    for (uint32_t lane = 0; lane < 4; ++lane)
        outMasks[lane] = uint8_t(remainingLanes[lane]);
    return detail::movemask4(alive[0]);
#endif
}
#elif FRONTIER_SIMD_SSE2
inline uint32_t testWideAabb(const WideBounds& b, const Frustum& fr,
                             uint8_t inMask, uint8_t outMasks[kWide])
{
    for (uint32_t l = 0; l < kWide; ++l) outMasks[l] = inMask;
    if (!inMask) return (1u << kWide) - 1;

    constexpr uint32_t kSimdGroups = kWide / 4;
    __m128 mnx[kSimdGroups], mny[kSimdGroups], mnz[kSimdGroups];
    __m128 mxx[kSimdGroups], mxy[kSimdGroups], mxz[kSimdGroups];
    for (uint32_t h = 0; h < kSimdGroups; ++h)
    {
        mnx[h] = _mm_load_ps(b.mnx.v + 4 * h);
        mny[h] = _mm_load_ps(b.mny.v + 4 * h);
        mnz[h] = _mm_load_ps(b.mnz.v + 4 * h);
        mxx[h] = _mm_load_ps(b.mxx.v + 4 * h);
        mxy[h] = _mm_load_ps(b.mxy.v + 4 * h);
        mxz[h] = _mm_load_ps(b.mxz.v + 4 * h);
    }
    const __m128 zero = _mm_setzero_ps();

    uint32_t alive = (1u << kWide) - 1;
    for (uint32_t p = 0; p < 6; ++p)
    {
        if (!((inMask >> p) & 1)) continue;
        const float4 pl = fr.plane[p];
        const __m128 nx = _mm_set1_ps(pl.x), ny = _mm_set1_ps(pl.y);
        const __m128 nz = _mm_set1_ps(pl.z), d = _mm_set1_ps(pl.w);
        // Sign masks select the p-/n-vertex per axis (constant per plane).
        const __m128 sx = _mm_cmplt_ps(nx, zero);
        const __m128 sy = _mm_cmplt_ps(ny, zero);
        const __m128 sz = _mm_cmplt_ps(nz, zero);

        for (uint32_t h = 0; h < kSimdGroups; ++h)
        {
            const __m128 px = detail::select4(sx, mnx[h], mxx[h]);
            const __m128 py = detail::select4(sy, mny[h], mxy[h]);
            const __m128 pz = detail::select4(sz, mnz[h], mxz[h]);
            const __m128 dp = detail::fmadd4(
                nx, px, detail::fmadd4(ny, py, detail::fmadd4(nz, pz, d)));
            alive &= ~(uint32_t(_mm_movemask_ps(_mm_cmplt_ps(dp, zero))) << (4 * h));

            const __m128 qx = detail::select4(sx, mxx[h], mnx[h]);
            const __m128 qy = detail::select4(sy, mxy[h], mny[h]);
            const __m128 qz = detail::select4(sz, mxz[h], mnz[h]);
            const __m128 dn = detail::fmadd4(
                nx, qx, detail::fmadd4(ny, qy, detail::fmadd4(nz, qz, d)));
            uint32_t inside =
                uint32_t(_mm_movemask_ps(_mm_cmpge_ps(dn, zero))) << (4 * h);
            while (inside)
            {
                const uint32_t l = uint32_t(std::countr_zero(inside));
                inside &= inside - 1;
                outMasks[l] = uint8_t(outMasks[l] & ~(1u << p));
            }
        }
    }
    return alive & ((1u << kWide) - 1);
}
#else
inline uint32_t testWideAabb(const WideBounds& b, const Frustum& fr,
                             uint8_t inMask, uint8_t outMasks[kWide])
{
    uint32_t alive = (1u << kWide) - 1;
    for (uint32_t l = 0; l < kWide; ++l) outMasks[l] = inMask;
    if (!inMask) return alive;

    for (uint32_t p = 0; p < 6; ++p)
    {
        if (!((inMask >> p) & 1)) continue;
        const float4 pl = fr.plane[p];
        for (uint32_t l = 0; l < kWide; ++l)
        {
            const float px = pl.x >= 0.0f ? b.mxx.v[l] : b.mnx.v[l];
            const float py = pl.y >= 0.0f ? b.mxy.v[l] : b.mny.v[l];
            const float pz = pl.z >= 0.0f ? b.mxz.v[l] : b.mnz.v[l];
            const float dp = fmadd(pl.x, px, fmadd(pl.y, py, fmadd(pl.z, pz, pl.w)));

            const float nx = pl.x >= 0.0f ? b.mnx.v[l] : b.mxx.v[l];
            const float ny = pl.y >= 0.0f ? b.mny.v[l] : b.mxy.v[l];
            const float nz = pl.z >= 0.0f ? b.mnz.v[l] : b.mxz.v[l];
            const float dn = fmadd(pl.x, nx, fmadd(pl.y, ny, fmadd(pl.z, nz, pl.w)));

            alive &= dp < 0.0f ? ~(1u << l) : ~0u;
            outMasks[l] = uint8_t(outMasks[l] & (dn >= 0.0f ? ~(1u << p) : 0xFFu));
        }
    }
    return alive;
}
#endif

// Wide box-to-box separation, one query box vs kWide boxes; 0 when they overlap.
//
// This is the same instruction count as the point query it replaces: the two
// broadcasts differ, nothing else. That is what makes envelope-based LOD
// damping free in the hot loop.
#if FRONTIER_SIMD_AVX2
inline float8 distanceToBoxes(const WideBounds& b, float4 qmn, float4 qmx)
{
    const __m256 lox = _mm256_set1_ps(qmn.x), loy = _mm256_set1_ps(qmn.y),
                 loz = _mm256_set1_ps(qmn.z);
    const __m256 hix = _mm256_set1_ps(qmx.x), hiy = _mm256_set1_ps(qmx.y),
                 hiz = _mm256_set1_ps(qmx.z);
    const __m256 zero = _mm256_setzero_ps();
    const __m256 cx = _mm256_max_ps(
        _mm256_max_ps(_mm256_sub_ps(_mm256_load_ps(b.mnx.v), hix),
                      _mm256_sub_ps(lox, _mm256_load_ps(b.mxx.v))), zero);
    const __m256 cy = _mm256_max_ps(
        _mm256_max_ps(_mm256_sub_ps(_mm256_load_ps(b.mny.v), hiy),
                      _mm256_sub_ps(loy, _mm256_load_ps(b.mxy.v))), zero);
    const __m256 cz = _mm256_max_ps(
        _mm256_max_ps(_mm256_sub_ps(_mm256_load_ps(b.mnz.v), hiz),
                      _mm256_sub_ps(loz, _mm256_load_ps(b.mxz.v))), zero);
    const __m256 d2 =
        _mm256_fmadd_ps(cx, cx, _mm256_fmadd_ps(cy, cy, _mm256_mul_ps(cz, cz)));
    float8 r;
    _mm256_store_ps(r.v, _mm256_sqrt_ps(d2));
    return r;
}
#elif FRONTIER_SIMD_NEON
inline float8 distanceToBoxes(const WideBounds& b, float4 qmn, float4 qmx)
{
    const float32x4_t lox = vdupq_n_f32(qmn.x), loy = vdupq_n_f32(qmn.y),
                      loz = vdupq_n_f32(qmn.z);
    const float32x4_t hix = vdupq_n_f32(qmx.x), hiy = vdupq_n_f32(qmx.y),
                      hiz = vdupq_n_f32(qmx.z);
    const float32x4_t zero = vdupq_n_f32(0.0f);
    float8 r;
    for (uint32_t h = 0; h < kWide / 4; ++h)
    {
        const float32x4_t cx = vmaxq_f32(
            vmaxq_f32(vsubq_f32(vld1q_f32(b.mnx.v + 4 * h), hix),
                      vsubq_f32(lox, vld1q_f32(b.mxx.v + 4 * h))), zero);
        const float32x4_t cy = vmaxq_f32(
            vmaxq_f32(vsubq_f32(vld1q_f32(b.mny.v + 4 * h), hiy),
                      vsubq_f32(loy, vld1q_f32(b.mxy.v + 4 * h))), zero);
        const float32x4_t cz = vmaxq_f32(
            vmaxq_f32(vsubq_f32(vld1q_f32(b.mnz.v + 4 * h), hiz),
                      vsubq_f32(loz, vld1q_f32(b.mxz.v + 4 * h))), zero);
        const float32x4_t d2 =
            vfmaq_f32(vfmaq_f32(vmulq_f32(cz, cz), cy, cy), cx, cx);
        vst1q_f32(r.v + 4 * h, vsqrtq_f32(d2));
    }
    return r;
}
#elif FRONTIER_SIMD_SSE2
inline float8 distanceToBoxes(const WideBounds& b, float4 qmn, float4 qmx)
{
    const __m128 lox = _mm_set1_ps(qmn.x), loy = _mm_set1_ps(qmn.y),
                 loz = _mm_set1_ps(qmn.z);
    const __m128 hix = _mm_set1_ps(qmx.x), hiy = _mm_set1_ps(qmx.y),
                 hiz = _mm_set1_ps(qmx.z);
    const __m128 zero = _mm_setzero_ps();
    float8 r;
    for (uint32_t h = 0; h < kWide / 4; ++h)
    {
        const __m128 cx = _mm_max_ps(
            _mm_max_ps(_mm_sub_ps(_mm_load_ps(b.mnx.v + 4 * h), hix),
                       _mm_sub_ps(lox, _mm_load_ps(b.mxx.v + 4 * h))), zero);
        const __m128 cy = _mm_max_ps(
            _mm_max_ps(_mm_sub_ps(_mm_load_ps(b.mny.v + 4 * h), hiy),
                       _mm_sub_ps(loy, _mm_load_ps(b.mxy.v + 4 * h))), zero);
        const __m128 cz = _mm_max_ps(
            _mm_max_ps(_mm_sub_ps(_mm_load_ps(b.mnz.v + 4 * h), hiz),
                       _mm_sub_ps(loz, _mm_load_ps(b.mxz.v + 4 * h))), zero);
        const __m128 d2 =
            detail::fmadd4(cx, cx, detail::fmadd4(cy, cy, _mm_mul_ps(cz, cz)));
        _mm_store_ps(r.v + 4 * h, _mm_sqrt_ps(d2));
    }
    return r;
}
#else
inline float8 distanceToBoxes(const WideBounds& b, float4 qmn, float4 qmx)
{
    float8 r;
    for (uint32_t l = 0; l < kWide; ++l)
    {
        float dx = b.mnx.v[l] - qmx.x; float ex = qmn.x - b.mxx.v[l];
        float dy = b.mny.v[l] - qmx.y; float ey = qmn.y - b.mxy.v[l];
        float dz = b.mnz.v[l] - qmx.z; float ez = qmn.z - b.mxz.v[l];
        float cx = dx > ex ? dx : ex; cx = cx > 0.0f ? cx : 0.0f;
        float cy = dy > ey ? dy : ey; cy = cy > 0.0f ? cy : 0.0f;
        float cz = dz > ez ? dz : ez; cz = cz > 0.0f ? cz : 0.0f;
        r.v[l] = std::sqrt(fmadd(cx, cx, fmadd(cy, cy, cz * cz)));
    }
    return r;
}
#endif

// Point query, for callers with no camera envelope.
inline float8 distanceToBoxes(const WideBounds& b, float4 p)
{
    return distanceToBoxes(b, p, p);
}

// ---------------------------------------------------------------------------
// Camera: position, frustum, and the error scale k so that
// screenErrorPx = geometricError * k / distance.
// ---------------------------------------------------------------------------

struct Camera
{
    float4  pos;
    Frustum frustum;
    float   k = 1.0f;

    // Instances whose mask ANDs to zero with this are skipped outright. Use it
    // for per-view layers: shadow casters, first-person body, editor gizmos.
    uint32_t viewMask = ~0u;

    // LOD damping ("hysteresis"), expressed as where the camera has been
    // recently rather than as per-node history. Distances for the screen-error
    // test are measured to the nearest point of [pos - envLo, pos + envHi]
    // instead of to pos, i.e. a node is levelled as if the camera were at the
    // closest place it has occupied lately. A camera that jitters in place has
    // a tiny envelope and stops changing level at all; a camera in transit
    // drags the envelope behind it, so detail arrives immediately on approach
    // and is given up a few frames late on recede -- which is the asymmetry
    // you want, popping in being far worse than holding detail too long.
    //
    // Both components are non-negative and default to zero, which collapses
    // the envelope onto pos and makes the arithmetic bit-identical to an
    // undamped point query. Any hand-built Camera is therefore correct
    // without knowing this field exists. SpatialQuery's internal CameraDamper
    // maintains it during selection.
    float4 envLo{}, envHi{};

    float4 queryMin() const { return pos - envLo; }
    float4 queryMax() const { return pos + envHi; }
    bool   damped()   const { return envLo.x != 0.0f || envLo.y != 0.0f || envLo.z != 0.0f ||
                                     envHi.x != 0.0f || envHi.y != 0.0f || envHi.z != 0.0f; }
};

// Widen v's envelope to also cover otherPos -- the literal two-camera form of
// damping, for callers who keep last frame's view around and do not want the
// longer window CameraDamper provides.
inline Camera withEnvelope(const Camera& v, float4 otherPos)
{
    Camera r = v;
    const float4 zero{};
    r.envLo = max4(r.envLo, max4(v.pos - otherPos, zero));
    r.envHi = max4(r.envHi, max4(otherPos - v.pos, zero));
    return r;
}

inline Camera makePerspectiveCamera(float4 pos, float4 forward, float4 up,
                                    float fovY, float aspect,
                                    float viewportHeightPx,
                                    float nearD, float farD)
{
    const float4 f = normalize3(forward);
    const float4 r = normalize3(cross3(f, up));
    const float4 u = cross3(r, f);
    const float tanY = std::tan(fovY * 0.5f);
    const float tanX = tanY * aspect;

    Camera v;
    v.pos = pos;
    v.k   = viewportHeightPx / (2.0f * tanY);

    auto plane = [&](float4 n) -> float4
    {
        const float4 nn = normalize3(n);
        return {nn.x, nn.y, nn.z, -dot3(nn, pos)};
    };
    v.frustum.plane[0] = {f.x, f.y, f.z, -(dot3(f, pos) + nearD)};              // near
    v.frustum.plane[1] = {-f.x, -f.y, -f.z, dot3(f, pos) + farD};               // far
    v.frustum.plane[2] = plane(r + f * tanX);                                   // left
    v.frustum.plane[3] = plane((r * -1.0f) + f * tanX);                         // right
    v.frustum.plane[4] = plane(u + f * tanY);                                   // bottom
    v.frustum.plane[5] = plane((u * -1.0f) + f * tanY);                         // top
    return v;
}

// Clip-space depth range of the projection the matrix came from. Reverse-Z
// needs no special handling: it swaps which physical plane is "near" and
// which is "far", and both formulations extract the same pair of half-spaces.
enum class ClipRange : uint8_t
{
    ZeroToOne,     // D3D, Metal, Vulkan
    MinusOneToOne, // OpenGL
};

// Build a Camera straight from a combined view-projection matrix
// (Gribb-Hartmann plane extraction) — the form engines already have on hand.
//
// m16 is the 16 floats as DirectXMath (row-vector, row-major) or glm/GL
// (column-vector, column-major) store them; both agree on this layout.
//
// projYScale is the projection matrix's [1][1] element, i.e. 1/tan(fovY/2).
// It cannot be recovered from the combined matrix (the view rotation is
// entangled with it), and it is what turns a geometric error into pixels.
inline Camera cameraFromViewProjection(const float* m16, float4 cameraPos,
                                       float viewportHeightPx, float projYScale,
                                       ClipRange range = ClipRange::ZeroToOne)
{
    FRONTIER_CHECK(m16 != nullptr,
                   "cameraFromViewProjection: null matrix");
    FRONTIER_CHECK(range == ClipRange::ZeroToOne ||
                       range == ClipRange::MinusOneToOne,
                   "cameraFromViewProjection: invalid clip range");
    auto coeffs = [&](int c) -> float4
    { return {m16[c], m16[c + 4], m16[c + 8], m16[c + 12]}; };

    const float4 cx = coeffs(0), cy = coeffs(1), cz = coeffs(2), cw = coeffs(3);

    Camera v;
    v.pos = cameraPos;
    v.k   = viewportHeightPx * 0.5f * projYScale;

    // Each clip-space inequality becomes an inward half-space. Normalizing is
    // not required by the culling tests (they only look at the sign) but it
    // keeps plane.w a real distance, which the wide distance path and any
    // caller inspecting the frustum both expect.
    auto plane = [](float4 p) -> float4
    {
        const float len = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        const float inv = len > 0.0f ? 1.0f / len : 0.0f;
        return {p.x * inv, p.y * inv, p.z * inv, p.w * inv};
    };

    v.frustum.plane[0] = plane(range == ClipRange::ZeroToOne ? cz : cw + cz);  // near
    v.frustum.plane[1] = plane(cw - cz);                                       // far
    v.frustum.plane[2] = plane(cw + cx);                                       // left
    v.frustum.plane[3] = plane(cw - cx);                                       // right
    v.frustum.plane[4] = plane(cw + cy);                                       // bottom
    v.frustum.plane[5] = plane(cw - cy);                                       // top
    return v;
}

inline Camera cameraFromViewProjection(const float4x4& viewProj, float4 cameraPos,
                                       float viewportHeightPx, float projYScale,
                                       ClipRange range = ClipRange::ZeroToOne)
{
    return cameraFromViewProjection(viewProj.m, cameraPos, viewportHeightPx,
                                    projYScale, range);
}

// Escape hatch for callers that already have the six planes (a cascaded
// shadow split, a portal-clipped volume, a hand-built culling volume).
// Planes are inward: a point p is inside plane i when dot3(n, p) + w >= 0.
inline Camera cameraFromPlanes(const float4 planes[6], float4 cameraPos, float errorScaleK)
{
    FRONTIER_CHECK(planes != nullptr, "cameraFromPlanes: null plane array");
    Camera v;
    v.pos = cameraPos;
    v.k   = errorScaleK;
    for (uint32_t p = 0; p < 6; ++p) v.frustum.plane[p] = planes[p];
    return v;
}

inline Camera makeLookAtCamera(float4 pos, float4 target,
                               float fovY = 1.0f, float aspect = 16.0f / 9.0f,
                               float viewportHeightPx = 1080.0f,
                               float nearD = 0.1f, float farD = 1.0e9f)
{
    float4 fwd = target - pos;
    float4 up  = std::fabs(fwd.y) > 0.99f * length3(fwd) ? float4::vec(1, 0, 0)
                                                         : float4::vec(0, 1, 0);
    return makePerspectiveCamera(pos, fwd, up, fovY, aspect, viewportHeightPx, nearD, farD);
}

// Transform a world-space view into an instance's local space.
// This translation + uniform-scale overload is also the identity-yaw fast path.
// The error ratio geomError / distance is scale-invariant, so k is unchanged.
inline Camera toLocal(const Camera& v, float4 instPos, float instScale,
                      uint8_t activePlanes)
{
    assert(instScale > 0.0f);
    const float invScale = 1.0f / instScale;
    Camera local;
    local.pos = (v.pos - instPos) * invScale;
    local.k   = v.k;
    local.viewMask = v.viewMask;
    // The envelope is a pair of offsets from pos, so the translation cancels
    // and only the uniform scale applies. Non-negativity survives (scale > 0).
    local.envLo = v.envLo * invScale;
    local.envHi = v.envHi * invScale;
    for (uint32_t p = 0; p < 6; ++p)
    {
        if (!(activePlanes & (1u << p))) continue;
        const float4 pl = v.frustum.plane[p];
        local.frustum.plane[p] = {pl.x, pl.y, pl.z,
                                  (dot3(pl, instPos) + pl.w) * invScale};
    }
    return local;
}

inline Camera toLocal(const Camera& v, float4 instPos, float instScale)
{
    return toLocal(v, instPos, instScale, kAllPlanes);
}

// Inverse-transform a world-space view through a planar instance placement.
// World = position + scale * yaw * local. The axis-aligned damping envelope
// becomes an oriented box under the inverse yaw, so retain its exact AABB
// enclosure in local space.
inline Camera toLocal(const Camera& v, float4 instPos, float instScale,
                      YawRotation yaw, uint8_t activePlanes)
{
    assert(instScale > 0.0f);
    const float invScale = 1.0f / instScale;
    const float c = yaw.cosine;
    const float s = yaw.sine;
    const float4 delta = (v.pos - instPos) * invScale;

    Camera local;
    local.pos = {c * delta.x + s * delta.z, delta.y,
                 -s * delta.x + c * delta.z, delta.w};
    local.k = v.k;
    local.viewMask = v.viewMask;

    const float4 envelopeCenter = (v.envHi - v.envLo) * (0.5f * invScale);
    const float4 envelopeExtent = (v.envHi + v.envLo) * (0.5f * invScale);
    const float ac = std::fabs(c);
    const float as = std::fabs(s);
    const float4 localCenter = {
        c * envelopeCenter.x + s * envelopeCenter.z,
        envelopeCenter.y,
        -s * envelopeCenter.x + c * envelopeCenter.z, 0.0f};
    const float4 localExtent = {
        ac * envelopeExtent.x + as * envelopeExtent.z,
        envelopeExtent.y,
        as * envelopeExtent.x + ac * envelopeExtent.z, 0.0f};
    local.envLo = localExtent - localCenter;
    local.envHi = localExtent + localCenter;

    for (uint32_t p = 0; p < 6; ++p)
    {
        if (!(activePlanes & (1u << p))) continue;
        const float4 pl = v.frustum.plane[p];
        local.frustum.plane[p] = {
            c * pl.x + s * pl.z, pl.y, -s * pl.x + c * pl.z,
            (dot3(pl, instPos) + pl.w) * invScale};
    }
    return local;
}

inline Camera toLocal(const Camera& v, float4 instPos, float instScale,
                      YawRotation yaw)
{
    return toLocal(v, instPos, instScale, yaw, kAllPlanes);
}

// Transform a local AABB to world space (translation + uniform scale).
inline AABB toWorld(const AABB& b, float4 instPos, float instScale)
{
    if (b.isEmpty()) return AABB::empty();
    return AABB::fromMinMax(b.mn * instScale + instPos, b.mx * instScale + instPos);
}

// Exact world AABB of a local AABB after planar yaw, uniform scale, and
// translation. Center/extent form avoids transforming eight corners.
inline AABB toWorld(const AABB& b, float4 instPos, float instScale,
                    YawRotation yaw)
{
    if (b.isEmpty()) return AABB::empty();
    const float4 center = b.center();
    const float4 extent = (b.mx - b.mn) * 0.5f;
    const float c = yaw.cosine;
    const float s = yaw.sine;
    const float ac = std::fabs(c);
    const float as = std::fabs(s);
    const float4 worldCenter = float4::point(
        instPos.x + instScale * (c * center.x - s * center.z),
        instPos.y + instScale * center.y,
        instPos.z + instScale * (s * center.x + c * center.z));
    const float4 worldExtent = float4::vec(
        instScale * (ac * extent.x + as * extent.z),
        instScale * extent.y,
        instScale * (as * extent.x + ac * extent.z));
    return AABB::fromCenterExtent(worldCenter, worldExtent);
}

// screenErrorPx = geometricError * k / distance, saturating when inside.
inline float screenError(float geomError, float k, float dist)
{
    return geomError * k / (dist > 1.0e-30f ? dist : 1.0e-30f);
}
#if FRONTIER_SIMD_AVX2
inline float8 screenError8(const float8& geomError, float k, const float8& dist)
{
    const __m256 d = _mm256_max_ps(_mm256_load_ps(dist.v), _mm256_set1_ps(1.0e-30f));
    const __m256 e = _mm256_mul_ps(_mm256_load_ps(geomError.v), _mm256_set1_ps(k));
    float8 r;
    _mm256_store_ps(r.v, _mm256_div_ps(e, d));
    return r;
}
#elif FRONTIER_SIMD_NEON
inline float8 screenError8(const float8& geomError, float k, const float8& dist)
{
    const float32x4_t kk = vdupq_n_f32(k), eps = vdupq_n_f32(1.0e-30f);
    float8 r;
    for (uint32_t h = 0; h < kWide / 4; ++h)
    {
        const float32x4_t d = vmaxq_f32(vld1q_f32(dist.v + 4 * h), eps);
        const float32x4_t e = vmulq_f32(vld1q_f32(geomError.v + 4 * h), kk);
        vst1q_f32(r.v + 4 * h, vdivq_f32(e, d));
    }
    return r;
}
#elif FRONTIER_SIMD_SSE2
inline float8 screenError8(const float8& geomError, float k, const float8& dist)
{
    const __m128 kk = _mm_set1_ps(k), eps = _mm_set1_ps(1.0e-30f);
    float8 r;
    for (uint32_t h = 0; h < kWide / 4; ++h)
    {
        const __m128 d = _mm_max_ps(_mm_load_ps(dist.v + 4 * h), eps);
        const __m128 e = _mm_mul_ps(_mm_load_ps(geomError.v + 4 * h), kk);
        _mm_store_ps(r.v + 4 * h, _mm_div_ps(e, d));
    }
    return r;
}
#else
inline float8 screenError8(const float8& geomError, float k, const float8& dist)
{
    const float8 d = max8(dist, float8::splat(1.0e-30f));
    return (geomError * float8::splat(k)) / d;
}
#endif

// ---------------------------------------------------------------------------
// Squared-distance variants.
//
// The walk only ever wants geomError * k / distance. Keeping squared distance
// avoids computing a square root only to square the result again before the
// error calculation. Each SIMD backend can then choose its fastest final
// operation: err = e * k * rsqrt(d2), or the exact sqrt/divide equivalent.
//
// AVX2 has a fast reciprocal-square-root seed, so one Newton-Raphson step is
// profitable there. Apple Silicon instead executes its exact vector
// sqrt/divide path faster than either tested NEON estimate/refinement path.
// Both implementations reproduce the max(dist, 1e-30) zero-distance floor.
// ---------------------------------------------------------------------------

#if FRONTIER_SIMD_AVX2
inline float8 distanceToBoxesSq(const WideBounds& b, float4 qmn, float4 qmx)
{
    const __m256 lox = _mm256_set1_ps(qmn.x), loy = _mm256_set1_ps(qmn.y),
                 loz = _mm256_set1_ps(qmn.z);
    const __m256 hix = _mm256_set1_ps(qmx.x), hiy = _mm256_set1_ps(qmx.y),
                 hiz = _mm256_set1_ps(qmx.z);
    const __m256 zero = _mm256_setzero_ps();
    const __m256 cx = _mm256_max_ps(
        _mm256_max_ps(_mm256_sub_ps(_mm256_load_ps(b.mnx.v), hix),
                      _mm256_sub_ps(lox, _mm256_load_ps(b.mxx.v))), zero);
    const __m256 cy = _mm256_max_ps(
        _mm256_max_ps(_mm256_sub_ps(_mm256_load_ps(b.mny.v), hiy),
                      _mm256_sub_ps(loy, _mm256_load_ps(b.mxy.v))), zero);
    const __m256 cz = _mm256_max_ps(
        _mm256_max_ps(_mm256_sub_ps(_mm256_load_ps(b.mnz.v), hiz),
                      _mm256_sub_ps(loz, _mm256_load_ps(b.mxz.v))), zero);
    float8 r;
    _mm256_store_ps(
        r.v, _mm256_fmadd_ps(cx, cx, _mm256_fmadd_ps(cy, cy, _mm256_mul_ps(cz, cz))));
    return r;
}

inline float8 screenErrorFromSq8(const float8& geomError, float k, const float8& d2)
{
    const __m256 x = _mm256_load_ps(d2.v);
    // 12-bit seed, then one Newton-Raphson step: y *= 1.5 - 0.5*x*y*y.
    const __m256 y0 = _mm256_rsqrt_ps(x);
    const __m256 y = _mm256_mul_ps(
        y0, _mm256_fnmadd_ps(_mm256_mul_ps(_mm256_set1_ps(0.5f), x),
                             _mm256_mul_ps(y0, y0), _mm256_set1_ps(1.5f)));
    // rsqrt(0) is +inf; the exact path floors distance at 1e-30, i.e. caps the
    // reciprocal at 1e30. Also scrubs the NaN that NR would produce from inf.
    const __m256 inv = _mm256_min_ps(_mm256_set1_ps(1.0e30f),
                                     _mm256_blendv_ps(y, y0, _mm256_cmp_ps(
                                         y, y, _CMP_UNORD_Q)));
    float8 r;
    _mm256_store_ps(r.v, _mm256_mul_ps(
        _mm256_mul_ps(_mm256_load_ps(geomError.v), _mm256_set1_ps(k)), inv));
    return r;
}
#elif FRONTIER_SIMD_NEON
inline float8 distanceToBoxesSq(const WideBounds& b, float4 qmn, float4 qmx)
{
    const float32x4_t lox = vdupq_n_f32(qmn.x), loy = vdupq_n_f32(qmn.y),
                      loz = vdupq_n_f32(qmn.z);
    const float32x4_t hix = vdupq_n_f32(qmx.x), hiy = vdupq_n_f32(qmx.y),
                      hiz = vdupq_n_f32(qmx.z);
    const float32x4_t zero = vdupq_n_f32(0.0f);
    float8 r;
    for (uint32_t h = 0; h < kWide / 4; ++h)
    {
        const float32x4_t cx = vmaxq_f32(
            vmaxq_f32(vsubq_f32(vld1q_f32(b.mnx.v + 4 * h), hix),
                      vsubq_f32(lox, vld1q_f32(b.mxx.v + 4 * h))), zero);
        const float32x4_t cy = vmaxq_f32(
            vmaxq_f32(vsubq_f32(vld1q_f32(b.mny.v + 4 * h), hiy),
                      vsubq_f32(loy, vld1q_f32(b.mxy.v + 4 * h))), zero);
        const float32x4_t cz = vmaxq_f32(
            vmaxq_f32(vsubq_f32(vld1q_f32(b.mnz.v + 4 * h), hiz),
                      vsubq_f32(loz, vld1q_f32(b.mxz.v + 4 * h))), zero);
        vst1q_f32(r.v + 4 * h,
                  vfmaq_f32(vfmaq_f32(vmulq_f32(cz, cz), cy, cy), cx, cx));
    }
    return r;
}

inline float8 screenErrorFromSq8(const float8& geomError, float k,
                                 const float8& d2)
{
    const float32x4_t eps = vdupq_n_f32(1.0e-30f);
    const float32x4_t kk = vdupq_n_f32(k);
    float8 r;
    for (uint32_t h = 0; h < kWide / 4; ++h)
    {
        const float32x4_t d = vmaxq_f32(
            vsqrtq_f32(vld1q_f32(d2.v + 4 * h)), eps);
        const float32x4_t e = vmulq_f32(
            vld1q_f32(geomError.v + 4 * h), kk);
        vst1q_f32(r.v + 4 * h, vdivq_f32(e, d));
    }
    return r;
}
#else
inline float8 distanceToBoxesSq(const WideBounds& b, float4 qmn, float4 qmx)
{
    const float8 d = distanceToBoxes(b, qmn, qmx);
    float8       r;
    for (uint32_t l = 0; l < kWide; ++l) r.v[l] = d.v[l] * d.v[l];
    return r;
}

inline float8 screenErrorFromSq8(const float8& geomError, float k, const float8& d2)
{
    float8 d;
    for (uint32_t l = 0; l < kWide; ++l) d.v[l] = std::sqrt(d2.v[l]);
    return screenError8(geomError, k, d);
}
#endif


// ---------------------------------------------------------------------------
// CameraDamper — LOD hysteresis, held per view instead of per node.
//
// Hysteresis is inherently stateful: a Schmitt trigger has to remember which
// way it last flipped, and reconstructing that from just the previous frame is
// not enough (a node whose error alternates 4.1, 3.9 around a threshold of 4
// flips every single frame under a one-frame reconstruction, which is exactly
// the popping the mechanism exists to prevent). The trick is that the state
// does not have to live on the nodes. Frame-to-frame error is a smooth,
// deterministic function of camera position, so all the instability enters
// through one variable -- and that variable can carry the memory for the whole
// tree. One envelope per view replaces one sticky bit per materialized node
// per view, and because a box-to-box separation costs exactly what a
// point-to-box separation costs, the hot loop does not notice.
//
// halfLife is in frames: the envelope relaxes toward the current position by
// half every halfLife frames, so a node stays refined for roughly that long
// after the camera pulls back past the threshold. 0 disables damping, and then
// selection is bit-identical to an undamped run.
//
// SpatialQuery owns one of these and calls damp() from SpatialQuery::selectFrontier. The class is
// also public for code that needs a damped Camera outside selection. Do not
// pre-damp a Camera and also enable damping on its SpatialQuery, or damping is applied
// twice.
// ---------------------------------------------------------------------------

class CameraDamper
{
public:
    CameraDamper() = default;
    explicit CameraDamper(float halfLifeFrames)
    {
        setHalfLife(halfLifeFrames);
    }

    float halfLife() const { return halfLife_; }
    void setHalfLife(float frames)
    {
        halfLife_ = frames > 0.0f && std::isfinite(frames) ? frames : 0.0f;
    }

    // Forget the accumulated window. Call on a teleport or camera cut, so the
    // envelope does not stretch across the discontinuity and over-refine
    // everything between the two positions.
    void reset() { primed_ = false; }

    Camera damp(const Camera& v)
    {
        Camera out = v;
        if (halfLife_ <= 0.0f) { primed_ = false; return out; }

        if (!primed_)
        {
            mn_ = mx_ = v.pos;
            kMax_ = v.k;
            primed_ = true;
        }
        else
        {
            // Relax the envelope toward the current position, then re-include
            // it. A camera that stops sees the window close exponentially; a
            // camera that overshoots an old extreme becomes the new extreme.
            const float decay = std::exp2(-1.0f / halfLife_);
            mn_ = min4(v.pos + (mn_ - v.pos) * decay, v.pos);
            mx_ = max4(v.pos + (mx_ - v.pos) * decay, v.pos);
            const float kRelaxed = v.k + (kMax_ - v.k) * decay;
            kMax_ = kRelaxed > v.k ? kRelaxed : v.k;
        }

        const float4 zero{};
        out.envLo = max4(v.pos - mn_, zero);
        out.envHi = max4(mx_ - v.pos, zero);
        out.k     = kMax_;   // a zoom-out should not drop detail any faster
        return out;
    }

private:
    float4 mn_{}, mx_{};
    float  halfLife_ = 0.0f;
    float  kMax_ = 0.0f;
    bool   primed_ = false;
};
static_assert(sizeof(CameraDamper) == 48,
              "CameraDamper should occupy three SIMD words");

} // namespace frontier
