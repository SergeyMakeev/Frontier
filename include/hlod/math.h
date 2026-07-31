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

#if defined(__AVX2__)
#include <immintrin.h>
#define HLOD_AVX2 1
#endif

namespace hlod {

inline constexpr uint32_t kWide = 8;   // SIMD width of the wide paths

// ---------------------------------------------------------------------------
// float4
// ---------------------------------------------------------------------------

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

// Point-to-box distance; 0 when the point is inside.
// NOTE: written with std::fma in exactly the order the AVX2 wide path uses,
// so scalar and wide results are bit-identical (the unit tests rely on it).
inline float distanceToBox(const AABB& b, float4 p)
{
    float dx = b.mn.x - p.x; float ex = p.x - b.mx.x;
    float dy = b.mn.y - p.y; float ey = p.y - b.mx.y;
    float dz = b.mn.z - p.z; float ez = p.z - b.mx.z;
    float cx = dx > ex ? dx : ex; cx = cx > 0.0f ? cx : 0.0f;
    float cy = dy > ey ? dy : ey; cy = cy > 0.0f ? cy : 0.0f;
    float cz = dz > ez ? dz : ez; cz = cz > 0.0f ? cz : 0.0f;
    return std::sqrt(std::fma(cx, cx, std::fma(cy, cy, cz * cz)));
}

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
        // p-vertex: corner maximizing the signed distance. The fma chain
        // matches the AVX2 wide path bit-for-bit.
        const float px = pl.x >= 0.0f ? b.mx.x : b.mn.x;
        const float py = pl.y >= 0.0f ? b.mx.y : b.mn.y;
        const float pz = pl.z >= 0.0f ? b.mx.z : b.mn.z;
        if (std::fma(pl.x, px, std::fma(pl.y, py, std::fma(pl.z, pz, pl.w))) < 0.0f)
        {
            ioMask = mask;
            return CullState::Outside;
        }
        // n-vertex: corner minimizing it. Inside this plane => stop testing it.
        const float nx = pl.x >= 0.0f ? b.mn.x : b.mx.x;
        const float ny = pl.y >= 0.0f ? b.mn.y : b.mx.y;
        const float nz = pl.z >= 0.0f ? b.mn.z : b.mx.z;
        if (std::fma(pl.x, nx, std::fma(pl.y, ny, std::fma(pl.z, nz, pl.w))) >= 0.0f)
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

// Wide masked tri-state: 8 boxes against the planes set in inMask.
// Returns the survivor lane bitmask; outMasks[l] is lane l's narrowed plane
// mask (valid only for surviving lanes). inMask == 0 means an ancestor was
// fully inside: every lane survives untested with mask 0.
#if HLOD_AVX2
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
            const float dp = std::fma(pl.x, px, std::fma(pl.y, py, std::fma(pl.z, pz, pl.w)));

            const float nx = pl.x >= 0.0f ? b.mnx.v[l] : b.mxx.v[l];
            const float ny = pl.y >= 0.0f ? b.mny.v[l] : b.mxy.v[l];
            const float nz = pl.z >= 0.0f ? b.mnz.v[l] : b.mxz.v[l];
            const float dn = std::fma(pl.x, nx, std::fma(pl.y, ny, std::fma(pl.z, nz, pl.w)));

            alive &= dp < 0.0f ? ~(1u << l) : ~0u;
            outMasks[l] = uint8_t(outMasks[l] & (dn >= 0.0f ? ~(1u << p) : 0xFFu));
        }
    }
    return alive;
}
#endif

// Wide point-to-box distance, one point vs 8 boxes; 0 when inside.
#if HLOD_AVX2
inline float8 distanceToBoxes(const WideBounds& b, float4 p)
{
    const __m256 px = _mm256_set1_ps(p.x), py = _mm256_set1_ps(p.y),
                 pz = _mm256_set1_ps(p.z);
    const __m256 zero = _mm256_setzero_ps();
    const __m256 cx = _mm256_max_ps(
        _mm256_max_ps(_mm256_sub_ps(_mm256_load_ps(b.mnx.v), px),
                      _mm256_sub_ps(px, _mm256_load_ps(b.mxx.v))), zero);
    const __m256 cy = _mm256_max_ps(
        _mm256_max_ps(_mm256_sub_ps(_mm256_load_ps(b.mny.v), py),
                      _mm256_sub_ps(py, _mm256_load_ps(b.mxy.v))), zero);
    const __m256 cz = _mm256_max_ps(
        _mm256_max_ps(_mm256_sub_ps(_mm256_load_ps(b.mnz.v), pz),
                      _mm256_sub_ps(pz, _mm256_load_ps(b.mxz.v))), zero);
    const __m256 d2 =
        _mm256_fmadd_ps(cx, cx, _mm256_fmadd_ps(cy, cy, _mm256_mul_ps(cz, cz)));
    float8 r;
    _mm256_store_ps(r.v, _mm256_sqrt_ps(d2));
    return r;
}
#else
inline float8 distanceToBoxes(const WideBounds& b, float4 p)
{
    float8 r;
    for (uint32_t l = 0; l < kWide; ++l)
    {
        float dx = b.mnx.v[l] - p.x; float ex = p.x - b.mxx.v[l];
        float dy = b.mny.v[l] - p.y; float ey = p.y - b.mxy.v[l];
        float dz = b.mnz.v[l] - p.z; float ez = p.z - b.mxz.v[l];
        float cx = dx > ex ? dx : ex; cx = cx > 0.0f ? cx : 0.0f;
        float cy = dy > ey ? dy : ey; cy = cy > 0.0f ? cy : 0.0f;
        float cz = dz > ez ? dz : ez; cz = cz > 0.0f ? cz : 0.0f;
        r.v[l] = std::sqrt(std::fma(cx, cx, std::fma(cy, cy, cz * cz)));
    }
    return r;
}
#endif

// ---------------------------------------------------------------------------
// Cull view: camera position, frustum, and the error scale k so that
// screenErrorPx = geometricError * k / distance.
// ---------------------------------------------------------------------------

struct CullView
{
    float4  pos;
    Frustum frustum;
    float   k = 1.0f;
};

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
#if HLOD_AVX2
inline float8 screenError8(const float8& geomError, float k, const float8& dist)
{
    const __m256 d = _mm256_max_ps(_mm256_load_ps(dist.v), _mm256_set1_ps(1.0e-30f));
    const __m256 e = _mm256_mul_ps(_mm256_load_ps(geomError.v), _mm256_set1_ps(k));
    float8 r;
    _mm256_store_ps(r.v, _mm256_div_ps(e, d));
    return r;
}
#else
inline float8 screenError8(const float8& geomError, float k, const float8& dist)
{
    const float8 d = max8(dist, float8::splat(1.0e-30f));
    return (geomError * float8::splat(k)) / d;
}
#endif

} // namespace hlod
