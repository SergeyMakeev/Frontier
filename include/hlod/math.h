#pragma once
// Minimal SIMD-friendly math for HLodTree: float4 / float8 only.
//
// Everything is written as branch-free lane loops over fixed-size arrays so
// the compiler can auto-vectorize; the types and call sites are shaped so the
// implementation can later be swapped for a real SIMD math library without
// touching the callers.

#include <bit>
#include <cassert>
#include <cfloat>
#include <cmath>
#include <cstdint>

#include "config.h"

// ---------------------------------------------------------------------------
// Bring your own vector types.
//
// Define HLOD_USE_CUSTOM_VECTOR_TYPES and declare hlod::float4 and
// hlod::float4x4 (typically `using` aliases for your engine's types) before
// including any hlod header. A conforming float4 must be 16 bytes, 16-byte
// aligned, expose public float x, y, z, w, and support the free functions
// declared in the block below. The static_asserts after the block catch the
// common mistakes.
//
// float8 / WideBounds are deliberately NOT swappable: they are the internal
// SIMD working set, not an interface type.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// SIMD backend selection. At most one of HLOD_SIMD_AVX2 / HLOD_SIMD_NEON /
// HLOD_SIMD_SSE2 is defined; with none of them the wide paths fall back to
// portable scalar loops. Define HLOD_FORCE_SCALAR (CMake option of the same
// name) to force the scalar fallback everywhere.
// ---------------------------------------------------------------------------

#if !defined(HLOD_FORCE_SCALAR)
  #if defined(__AVX2__)
    #include <immintrin.h>
    #define HLOD_SIMD_AVX2 1
  #elif defined(__aarch64__) || defined(_M_ARM64) || defined(_M_ARM64EC)
    // 64-bit ARM only: the wide paths use vaddvq/vdivq/vsqrtq, which 32-bit
    // NEON lacks. armv7 falls back to the scalar loops.
    #include <arm_neon.h>
    #define HLOD_SIMD_NEON 1
  #elif defined(__SSE2__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
    #include <immintrin.h>
    #define HLOD_SIMD_SSE2 1
    #if defined(__SSE4_1__) || defined(__AVX__)
      #define HLOD_SIMD_SSE41 1
    #endif
  #endif
#endif

// Best-effort cache prefetch hint.
#if HLOD_SIMD_AVX2 || HLOD_SIMD_SSE2
  #define HLOD_PREFETCH(p) _mm_prefetch(reinterpret_cast<const char*>(p), _MM_HINT_T0)
#elif defined(__GNUC__) || defined(__clang__)
  #define HLOD_PREFETCH(p) __builtin_prefetch(p)
#elif defined(_MSC_VER) && (defined(_M_ARM64) || defined(_M_ARM64EC))
  #include <intrin.h>
  #define HLOD_PREFETCH(p) __prefetch(reinterpret_cast<const void*>(p))
#else
  #define HLOD_PREFETCH(p) ((void)(p))
#endif

namespace hlod {

// Fused multiply-add matching the active wide path. The unit tests require
// scalar and wide results to be bit-identical, so the scalar code must fuse
// exactly when the wide intrinsics fuse. The only backend without hardware
// FMA is plain SSE; there mul+add is used on both sides (and the target has
// no fma instruction the compiler could contract to, keeping them in sync).
#if HLOD_SIMD_SSE2 && !defined(__FMA__)
inline float fmadd(float a, float b, float c) { return a * b + c; }
#else
inline float fmadd(float a, float b, float c) { return std::fma(a, b, c); }
#endif

inline constexpr uint32_t kWide = 8;   // SIMD width of the wide paths

// ---------------------------------------------------------------------------
// float4
// ---------------------------------------------------------------------------

#ifndef HLOD_USE_CUSTOM_VECTOR_TYPES

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
// a single fromMemory() serves both worlds and there is no convention flag
// to get wrong.
struct alignas(16) float4x4
{
    float m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

    static float4x4 fromMemory(const float* m16)
    {
        float4x4 r;
        for (int i = 0; i < 16; ++i) r.m[i] = m16[i];
        return r;
    }

    // Coefficient vector for clip-space component c (0=x, 1=y, 2=z, 3=w):
    // clip[c] = dot(coeffs(c), (p.x, p.y, p.z, 1)).
    float4 coeffs(int c) const
    {
        return {m[c], m[c + 4], m[c + 8], m[c + 12]};
    }
};

#endif // HLOD_USE_CUSTOM_VECTOR_TYPES

static_assert(sizeof(float4) == 16, "hlod::float4 must be 16 bytes");
static_assert(alignof(float4) >= 16, "hlod::float4 must be 16-byte aligned");

// ---------------------------------------------------------------------------
// float8 — lane-wise; lanes carry independent values (typically 8 boxes)
// ---------------------------------------------------------------------------

struct alignas(32) float8
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

    bool isEmpty() const { return mn.x > mx.x; }

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
// has been recently) rather than a bare position; see ViewDamper below.
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
// Wide bounds: 8 boxes, SoA-transposed (lanes = boxes)
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

#if HLOD_SIMD_NEON
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

#if HLOD_SIMD_SSE2
namespace detail {
// mask ? a : b, per lane; mask lanes are all-ones or all-zeros.
inline __m128 select4(__m128 mask, __m128 a, __m128 b)
{
#if HLOD_SIMD_SSE41
    return _mm_blendv_ps(b, a, mask);
#else
    return _mm_or_ps(_mm_and_ps(mask, a), _mm_andnot_ps(mask, b));
#endif
}
// Fused only when the target has FMA, mirroring hlod::fmadd.
inline __m128 fmadd4(__m128 a, __m128 b, __m128 c)
{
#if defined(__FMA__)
    return _mm_fmadd_ps(a, b, c);
#else
    return _mm_add_ps(_mm_mul_ps(a, b), c);
#endif
}
} // namespace detail
#endif

// Wide masked tri-state: 8 boxes against the planes set in inMask.
// Returns the survivor lane bitmask; outMasks[l] is lane l's narrowed plane
// mask (valid only for surviving lanes). inMask == 0 means an ancestor was
// fully inside: every lane survives untested with mask 0.
#if HLOD_SIMD_AVX2
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

// Same test, but the eight per-lane plane masks are returned packed one byte
// per lane in a single uint64 instead of written through a byte array.
//
// The byte-array form spends up to 8 dependent load-modify-stores per plane
// clearing one bit at a time (48 per block in the worst case). Packed, the
// same update is a lane-mask expand and one AND: `inside` (a bit per lane)
// becomes 0xFF per set lane, and that selects which bytes lose plane bit p.
// One 0xFF byte per set lane. Plain ALU rather than pdep: pdep is a single
// fast op on some x86 generations and microcoded (tens of cycles) on others,
// and this sits in the innermost loop.
inline uint64_t laneBytes(uint32_t laneMask)
{
    // Broadcast, keep bit l in byte l, then saturate each nonzero byte to 0xFF.
    uint64_t x = (uint64_t(laneMask) * 0x0101010101010101ull) & 0x8040201008040201ull;
    x = (x + 0x7F7F7F7F7F7F7F7Full) & 0x8080808080808080ull;
    return (x >> 7) * 0xFFull;
}

inline uint32_t testWideAabbPacked(const WideBounds& b, const Frustum& fr,
                                  uint8_t inMask, uint64_t& outMasks)
{
    outMasks = laneBytes((1u << kWide) - 1) & (0x0101010101010101ull * inMask);
    if (!inMask) return (1u << kWide) - 1;

    const __m256 mnx = _mm256_load_ps(b.mnx.v), mny = _mm256_load_ps(b.mny.v);
    const __m256 mnz = _mm256_load_ps(b.mnz.v), mxx = _mm256_load_ps(b.mxx.v);
    const __m256 mxy = _mm256_load_ps(b.mxy.v), mxz = _mm256_load_ps(b.mxz.v);
    const __m256 zero = _mm256_setzero_ps();

    uint32_t alive = (1u << kWide) - 1;
    uint64_t masks = outMasks;
    for (uint32_t p = 0; p < 6; ++p)
    {
        if (!((inMask >> p) & 1)) continue;
        const float4 pl = fr.plane[p];
        const __m256 nx = _mm256_set1_ps(pl.x), ny = _mm256_set1_ps(pl.y);
        const __m256 nz = _mm256_set1_ps(pl.z), d = _mm256_set1_ps(pl.w);
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
        const uint32_t inside =
            uint32_t(_mm256_movemask_ps(_mm256_cmp_ps(dn, zero, _CMP_GE_OQ)));
        masks &= ~(laneBytes(inside) & (0x0101010101010101ull << p));
    }
    outMasks = masks;
    return alive & ((1u << kWide) - 1);
}
#define HLOD_HAVE_PACKED_WIDE_TEST 1
#elif HLOD_SIMD_NEON
inline uint32_t testWideAabb(const WideBounds& b, const Frustum& fr,
                             uint8_t inMask, uint8_t outMasks[kWide])
{
    for (uint32_t l = 0; l < kWide; ++l) outMasks[l] = inMask;
    if (!inMask) return (1u << kWide) - 1;

    const float32x4_t mnx[2] = {vld1q_f32(b.mnx.v), vld1q_f32(b.mnx.v + 4)};
    const float32x4_t mny[2] = {vld1q_f32(b.mny.v), vld1q_f32(b.mny.v + 4)};
    const float32x4_t mnz[2] = {vld1q_f32(b.mnz.v), vld1q_f32(b.mnz.v + 4)};
    const float32x4_t mxx[2] = {vld1q_f32(b.mxx.v), vld1q_f32(b.mxx.v + 4)};
    const float32x4_t mxy[2] = {vld1q_f32(b.mxy.v), vld1q_f32(b.mxy.v + 4)};
    const float32x4_t mxz[2] = {vld1q_f32(b.mxz.v), vld1q_f32(b.mxz.v + 4)};
    const float32x4_t zero = vdupq_n_f32(0.0f);

    uint32_t alive = (1u << kWide) - 1;
    for (uint32_t p = 0; p < 6; ++p)
    {
        if (!((inMask >> p) & 1)) continue;
        const float4 pl = fr.plane[p];
        const float32x4_t nx = vdupq_n_f32(pl.x), ny = vdupq_n_f32(pl.y);
        const float32x4_t nz = vdupq_n_f32(pl.z), d = vdupq_n_f32(pl.w);
        // Sign masks select the p-/n-vertex per axis (constant per plane).
        const uint32x4_t sx = vcltq_f32(nx, zero);
        const uint32x4_t sy = vcltq_f32(ny, zero);
        const uint32x4_t sz = vcltq_f32(nz, zero);

        for (uint32_t h = 0; h < 2; ++h)
        {
            const float32x4_t px = vbslq_f32(sx, mnx[h], mxx[h]);
            const float32x4_t py = vbslq_f32(sy, mny[h], mxy[h]);
            const float32x4_t pz = vbslq_f32(sz, mnz[h], mxz[h]);
            // vfmaq_f32(c, a, b) = c + a*b, fused: matches std::fma.
            const float32x4_t dp =
                vfmaq_f32(vfmaq_f32(vfmaq_f32(d, nz, pz), ny, py), nx, px);
            alive &= ~(detail::movemask4(vcltq_f32(dp, zero)) << (4 * h));

            const float32x4_t qx = vbslq_f32(sx, mxx[h], mnx[h]);
            const float32x4_t qy = vbslq_f32(sy, mxy[h], mny[h]);
            const float32x4_t qz = vbslq_f32(sz, mxz[h], mnz[h]);
            const float32x4_t dn =
                vfmaq_f32(vfmaq_f32(vfmaq_f32(d, nz, qz), ny, qy), nx, qx);
            uint32_t inside = detail::movemask4(vcgeq_f32(dn, zero)) << (4 * h);
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
#elif HLOD_SIMD_SSE2
inline uint32_t testWideAabb(const WideBounds& b, const Frustum& fr,
                             uint8_t inMask, uint8_t outMasks[kWide])
{
    for (uint32_t l = 0; l < kWide; ++l) outMasks[l] = inMask;
    if (!inMask) return (1u << kWide) - 1;

    const __m128 mnx[2] = {_mm_load_ps(b.mnx.v), _mm_load_ps(b.mnx.v + 4)};
    const __m128 mny[2] = {_mm_load_ps(b.mny.v), _mm_load_ps(b.mny.v + 4)};
    const __m128 mnz[2] = {_mm_load_ps(b.mnz.v), _mm_load_ps(b.mnz.v + 4)};
    const __m128 mxx[2] = {_mm_load_ps(b.mxx.v), _mm_load_ps(b.mxx.v + 4)};
    const __m128 mxy[2] = {_mm_load_ps(b.mxy.v), _mm_load_ps(b.mxy.v + 4)};
    const __m128 mxz[2] = {_mm_load_ps(b.mxz.v), _mm_load_ps(b.mxz.v + 4)};
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

        for (uint32_t h = 0; h < 2; ++h)
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

#ifndef HLOD_HAVE_PACKED_WIDE_TEST
// Portable stand-in: the byte array, then packed. Correct everywhere; the
// point of the packed form is the AVX2 path above, where it is native.
inline uint32_t testWideAabbPacked(const WideBounds& b, const Frustum& fr,
                                   uint8_t inMask, uint64_t& outMasks)
{
    uint8_t        m[kWide];
    const uint32_t alive = testWideAabb(b, fr, inMask, m);
    uint64_t       packed = 0;
    for (uint32_t l = 0; l < kWide; ++l) packed |= uint64_t(m[l]) << (8 * l);
    outMasks = packed;
    return alive;
}
#endif

// Wide box-to-box separation, one query box vs 8 boxes; 0 when they overlap.
//
// This is the same instruction count as the point query it replaces: the two
// broadcasts differ, nothing else. That is what makes envelope-based LOD
// damping free in the hot loop.
#if HLOD_SIMD_AVX2
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
#elif HLOD_SIMD_NEON
inline float8 distanceToBoxes(const WideBounds& b, float4 qmn, float4 qmx)
{
    const float32x4_t lox = vdupq_n_f32(qmn.x), loy = vdupq_n_f32(qmn.y),
                      loz = vdupq_n_f32(qmn.z);
    const float32x4_t hix = vdupq_n_f32(qmx.x), hiy = vdupq_n_f32(qmx.y),
                      hiz = vdupq_n_f32(qmx.z);
    const float32x4_t zero = vdupq_n_f32(0.0f);
    float8 r;
    for (uint32_t h = 0; h < 2; ++h)
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
#elif HLOD_SIMD_SSE2
inline float8 distanceToBoxes(const WideBounds& b, float4 qmn, float4 qmx)
{
    const __m128 lox = _mm_set1_ps(qmn.x), loy = _mm_set1_ps(qmn.y),
                 loz = _mm_set1_ps(qmn.z);
    const __m128 hix = _mm_set1_ps(qmx.x), hiy = _mm_set1_ps(qmx.y),
                 hiz = _mm_set1_ps(qmx.z);
    const __m128 zero = _mm_setzero_ps();
    float8 r;
    for (uint32_t h = 0; h < 2; ++h)
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
// Cull view: camera position, frustum, and the error scale k so that
// screenErrorPx = geometricError * k / distance.
// ---------------------------------------------------------------------------

struct CullView
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
    // undamped point query. Any hand-built CullView is therefore correct
    // without knowing this field exists. ViewDamper maintains it for you.
    float4 envLo{}, envHi{};

    float4 queryMin() const { return pos - envLo; }
    float4 queryMax() const { return pos + envHi; }
    bool   damped()   const { return envLo.x != 0.0f || envLo.y != 0.0f || envLo.z != 0.0f ||
                                     envHi.x != 0.0f || envHi.y != 0.0f || envHi.z != 0.0f; }
};

// Widen v's envelope to also cover otherPos -- the literal two-camera form of
// damping, for callers who keep last frame's view around and do not want the
// longer window ViewDamper provides.
inline CullView withEnvelope(const CullView& v, float4 otherPos)
{
    CullView r = v;
    const float4 zero{};
    r.envLo = max4(r.envLo, max4(v.pos - otherPos, zero));
    r.envHi = max4(r.envHi, max4(otherPos - v.pos, zero));
    return r;
}

inline CullView makePerspectiveView(float4 pos, float4 forward, float4 up,
                                    float fovY, float aspect,
                                    float viewportHeightPx,
                                    float nearD, float farD)
{
    const float4 f = normalize3(forward);
    const float4 r = normalize3(cross3(f, up));
    const float4 u = cross3(r, f);
    const float tanY = std::tan(fovY * 0.5f);
    const float tanX = tanY * aspect;

    CullView v;
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

// Build a CullView straight from a combined view-projection matrix
// (Gribb-Hartmann plane extraction) — the form engines already have on hand.
//
// m16 is the 16 floats as DirectXMath (row-vector, row-major) or glm/GL
// (column-vector, column-major) store them; both agree on this layout.
//
// projYScale is the projection matrix's [1][1] element, i.e. 1/tan(fovY/2).
// It cannot be recovered from the combined matrix (the view rotation is
// entangled with it), and it is what turns a geometric error into pixels.
inline CullView fromViewProj(const float* m16, float4 cameraPos,
                             float viewportHeightPx, float projYScale,
                             ClipRange range = ClipRange::ZeroToOne)
{
    auto coeffs = [&](int c) -> float4
    { return {m16[c], m16[c + 4], m16[c + 8], m16[c + 12]}; };

    const float4 cx = coeffs(0), cy = coeffs(1), cz = coeffs(2), cw = coeffs(3);

    CullView v;
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

inline CullView fromViewProj(const float4x4& viewProj, float4 cameraPos,
                             float viewportHeightPx, float projYScale,
                             ClipRange range = ClipRange::ZeroToOne)
{
    return fromViewProj(viewProj.m, cameraPos, viewportHeightPx, projYScale, range);
}

// Escape hatch for callers that already have the six planes (a cascaded
// shadow split, a portal-clipped volume, a hand-built culling volume).
// Planes are inward: a point p is inside plane i when dot3(n, p) + w >= 0.
inline CullView fromPlanes(const float4 planes[6], float4 cameraPos, float errorScaleK)
{
    CullView v;
    v.pos = cameraPos;
    v.k   = errorScaleK;
    for (uint32_t p = 0; p < 6; ++p) v.frustum.plane[p] = planes[p];
    return v;
}

inline CullView makeLookAtView(float4 pos, float4 target,
                               float fovY = 1.0f, float aspect = 16.0f / 9.0f,
                               float viewportHeightPx = 1080.0f,
                               float nearD = 0.1f, float farD = 1.0e9f)
{
    float4 fwd = target - pos;
    float4 up  = std::fabs(fwd.y) > 0.99f * length3(fwd) ? float4::vec(1, 0, 0)
                                                         : float4::vec(0, 1, 0);
    return makePerspectiveView(pos, fwd, up, fovY, aspect, viewportHeightPx, nearD, farD);
}

// Transform a world-space view into an instance's local space.
// Instances are positioned by translation + uniform scale (no rotation).
// The error ratio geomError / distance is scale-invariant, so k is unchanged.
inline CullView toLocal(const CullView& v, float4 instPos, float instScale)
{
    assert(instScale > 0.0f);
    CullView local;
    local.pos = (v.pos - instPos) / instScale;
    local.k   = v.k;
    local.viewMask = v.viewMask;
    // The envelope is a pair of offsets from pos, so the translation cancels
    // and only the uniform scale applies. Non-negativity survives (scale > 0).
    local.envLo = v.envLo / instScale;
    local.envHi = v.envHi / instScale;
    for (uint32_t p = 0; p < 6; ++p)
    {
        const float4 pl = v.frustum.plane[p];
        local.frustum.plane[p] = {pl.x, pl.y, pl.z,
                                  (dot3(pl, instPos) + pl.w) / instScale};
    }
    return local;
}

// Transform a local AABB to world space (translation + uniform scale).
inline AABB toWorld(const AABB& b, float4 instPos, float instScale)
{
    if (b.isEmpty()) return AABB::empty();
    return AABB::fromMinMax(b.mn * instScale + instPos, b.mx * instScale + instPos);
}

// screenErrorPx = geometricError * k / distance, saturating when inside.
inline float screenError(float geomError, float k, float dist)
{
    return geomError * k / (dist > 1.0e-30f ? dist : 1.0e-30f);
}
#if HLOD_SIMD_AVX2
inline float8 screenError8(const float8& geomError, float k, const float8& dist)
{
    const __m256 d = _mm256_max_ps(_mm256_load_ps(dist.v), _mm256_set1_ps(1.0e-30f));
    const __m256 e = _mm256_mul_ps(_mm256_load_ps(geomError.v), _mm256_set1_ps(k));
    float8 r;
    _mm256_store_ps(r.v, _mm256_div_ps(e, d));
    return r;
}
#elif HLOD_SIMD_NEON
inline float8 screenError8(const float8& geomError, float k, const float8& dist)
{
    const float32x4_t kk = vdupq_n_f32(k), eps = vdupq_n_f32(1.0e-30f);
    float8 r;
    for (uint32_t h = 0; h < 2; ++h)
    {
        const float32x4_t d = vmaxq_f32(vld1q_f32(dist.v + 4 * h), eps);
        const float32x4_t e = vmulq_f32(vld1q_f32(geomError.v + 4 * h), kk);
        vst1q_f32(r.v + 4 * h, vdivq_f32(e, d));
    }
    return r;
}
#elif HLOD_SIMD_SSE2
inline float8 screenError8(const float8& geomError, float k, const float8& dist)
{
    const __m128 kk = _mm_set1_ps(k), eps = _mm_set1_ps(1.0e-30f);
    float8 r;
    for (uint32_t h = 0; h < 2; ++h)
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
// The walk only ever wants geomError * k / distance. Going through a real
// distance costs a full-precision square root AND a full-precision divide per
// block -- the two longest-latency instructions in the inner loop, back to
// back on the same dependency chain. Folding them into one reciprocal square
// root removes both: err = e * k * rsqrt(d2).
//
// The hardware rsqrt is 12-bit; one Newton-Raphson step takes it to within an
// ulp or two of the exact reciprocal square root, which is the precision the
// screen-error comparison actually needs. The min() reproduces the
// max(dist, 1e-30) floor of the exact path exactly, including d2 == 0.
// ---------------------------------------------------------------------------

#if HLOD_SIMD_AVX2
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
// ViewDamper — LOD hysteresis, held per view instead of per node.
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
// Hold one of these per view (main camera, each shadow cascade, ...) and call
// damp() once per frame; feed the result to World::selectCut.
// ---------------------------------------------------------------------------

class ViewDamper
{
public:
    ViewDamper() = default;
    explicit ViewDamper(float halfLifeFrames) : halfLife_(halfLifeFrames) {}

    float halfLife() const { return halfLife_; }
    void  setHalfLife(float frames) { halfLife_ = frames > 0.0f ? frames : 0.0f; }

    // Forget the accumulated window. Call on a teleport or camera cut, so the
    // envelope does not stretch across the discontinuity and over-refine
    // everything between the two positions.
    void reset() { primed_ = false; }

    CullView damp(const CullView& v)
    {
        CullView out = v;
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
    float  halfLife_ = 0.0f;
    bool   primed_   = false;
    float4 mn_{}, mx_{};
    float  kMax_ = 0.0f;
};

} // namespace hlod
