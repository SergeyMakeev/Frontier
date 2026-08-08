// Machine characterization for interpreting HLodTree results, plus a small
// group of production-kernel probes. This is a separate executable so adding
// diagnostics cannot perturb the end-to-end benchmark's code layout.

#include <benchmark/benchmark.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

#include "hlod/math.h"
#include "hlod/world.h"
#include "deterministic_rng.h"

#if defined(__aarch64__) || defined(_M_ARM64) || defined(_M_ARM64EC)
#include <arm_neon.h>
#define HLOD_MACHINE_NEON 1
#elif defined(__SSE2__) || defined(_M_X64) || \
      (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
#include <immintrin.h>
#define HLOD_MACHINE_SSE2 1
#endif

#if defined(_MSC_VER)
#define HLOD_MACHINE_NOINLINE __declspec(noinline)
#else
#define HLOD_MACHINE_NOINLINE __attribute__((noinline))
#endif

#define HLOD_STRINGIZE_DETAIL(x) #x
#define HLOD_STRINGIZE(x) HLOD_STRINGIZE_DETAIL(x)

namespace {

constexpr int64_t kKiB = 1024;
constexpr int64_t kMiB = 1024 * kKiB;

#if HLOD_SIMD_AVX2
constexpr const char* kKernelBackend = "AVX2 256-bit";
#elif HLOD_SIMD_NEON
constexpr const char* kKernelBackend = "NEON 128-bit x2";
#elif HLOD_SIMD_SSE2
constexpr const char* kKernelBackend = "SSE2 128-bit x2";
#else
constexpr const char* kKernelBackend = "portable scalar x8";
#endif

#if HLOD_MACHINE_NEON
using Vec4 = float32x4_t;
constexpr const char* kVectorBackend = "NEON 128-bit";
inline Vec4 vset(float x) { return vdupq_n_f32(x); }
inline Vec4 vadd(Vec4 a, Vec4 b) { return vaddq_f32(a, b); }
inline Vec4 vmul(Vec4 a, Vec4 b) { return vmulq_f32(a, b); }
inline Vec4 vdiv(Vec4 a, Vec4 b) { return vdivq_f32(a, b); }
inline Vec4 vsqrt(Vec4 a) { return vsqrtq_f32(a); }
inline Vec4 vload(const float* p) { return vld1q_f32(p); }
inline void vstore(float* p, Vec4 v) { vst1q_f32(p, v); }
inline uint32_t compareMask(Vec4 a, Vec4 b)
{
    // MSVC represents NEON vectors as __n128 and has no vector literals.
    alignas(16) static constexpr uint32_t kWeights[4] = {1u, 2u, 4u, 8u};
    const uint32x4_t bits = vshrq_n_u32(vcltq_f32(a, b), 31);
    return vaddvq_u32(vmulq_u32(bits, vld1q_u32(kWeights)));
}
#elif HLOD_MACHINE_SSE2
using Vec4 = __m128;
constexpr const char* kVectorBackend = "SSE2 128-bit";
inline Vec4 vset(float x) { return _mm_set1_ps(x); }
inline Vec4 vadd(Vec4 a, Vec4 b) { return _mm_add_ps(a, b); }
inline Vec4 vmul(Vec4 a, Vec4 b) { return _mm_mul_ps(a, b); }
inline Vec4 vdiv(Vec4 a, Vec4 b) { return _mm_div_ps(a, b); }
inline Vec4 vsqrt(Vec4 a) { return _mm_sqrt_ps(a); }
inline Vec4 vload(const float* p) { return _mm_loadu_ps(p); }
inline void vstore(float* p, Vec4 v) { _mm_storeu_ps(p, v); }
inline uint32_t compareMask(Vec4 a, Vec4 b)
{
    return uint32_t(_mm_movemask_ps(_mm_cmplt_ps(a, b)));
}
#else
struct Vec4 { float v[4]; };
constexpr const char* kVectorBackend = "portable scalar x4";
inline Vec4 vset(float x) { return {{x, x, x, x}}; }
inline Vec4 vadd(Vec4 a, Vec4 b)
{
    return {{a.v[0] + b.v[0], a.v[1] + b.v[1],
             a.v[2] + b.v[2], a.v[3] + b.v[3]}};
}
inline Vec4 vmul(Vec4 a, Vec4 b)
{
    return {{a.v[0] * b.v[0], a.v[1] * b.v[1],
             a.v[2] * b.v[2], a.v[3] * b.v[3]}};
}
inline Vec4 vdiv(Vec4 a, Vec4 b)
{
    return {{a.v[0] / b.v[0], a.v[1] / b.v[1],
             a.v[2] / b.v[2], a.v[3] / b.v[3]}};
}
inline Vec4 vsqrt(Vec4 a)
{
    return {{std::sqrt(a.v[0]), std::sqrt(a.v[1]),
             std::sqrt(a.v[2]), std::sqrt(a.v[3])}};
}
inline Vec4 vload(const float* p) { return {{p[0], p[1], p[2], p[3]}}; }
inline void vstore(float* p, Vec4 v)
{
    for (uint32_t i = 0; i < 4; ++i) p[i] = v.v[i];
}
inline uint32_t compareMask(Vec4 a, Vec4 b)
{
    uint32_t mask = 0;
    for (uint32_t i = 0; i < 4; ++i) mask |= uint32_t(a.v[i] < b.v[i]) << i;
    return mask;
}
#endif

#if defined(__clang__)
constexpr const char* kCompiler = "Clang " __clang_version__;
#elif defined(__GNUC__)
constexpr const char* kCompiler =
    "GCC " HLOD_STRINGIZE(__GNUC__) "." HLOD_STRINGIZE(__GNUC_MINOR__);
#elif defined(_MSC_VER)
constexpr const char* kCompiler = "MSVC " HLOD_STRINGIZE(_MSC_VER);
#else
constexpr const char* kCompiler = "unknown";
#endif

inline void consume(Vec4 v)
{
    alignas(16) float lanes[4];
    vstore(lanes, v);
    benchmark::DoNotOptimize(lanes[0]);
    benchmark::DoNotOptimize(lanes[1]);
    benchmark::DoNotOptimize(lanes[2]);
    benchmark::DoNotOptimize(lanes[3]);
}

// ---- scalar and vector execution ------------------------------------------

static void BM_ScalarFpDependency(benchmark::State& state)
{
    constexpr int kUpdates = 1024;
    float x = 0.25f;
    for (auto _ : state)
    {
        for (int i = 0; i < kUpdates; ++i) x = x * 0.99991f + 0.00013f;
        benchmark::DoNotOptimize(x);
    }
    state.SetItemsProcessed(state.iterations() * kUpdates);
}
BENCHMARK(BM_ScalarFpDependency);

static void BM_ScalarFpThroughput(benchmark::State& state)
{
    constexpr int kRounds = 128;
    float x0 = 0.11f, x1 = 0.22f, x2 = 0.33f, x3 = 0.44f;
    float x4 = 0.55f, x5 = 0.66f, x6 = 0.77f, x7 = 0.88f;
    for (auto _ : state)
    {
        for (int i = 0; i < kRounds; ++i)
        {
            x0 = x0 * 0.99991f + 0.00011f; x1 = x1 * 0.99989f + 0.00012f;
            x2 = x2 * 0.99987f + 0.00013f; x3 = x3 * 0.99985f + 0.00014f;
            x4 = x4 * 0.99983f + 0.00015f; x5 = x5 * 0.99981f + 0.00016f;
            x6 = x6 * 0.99979f + 0.00017f; x7 = x7 * 0.99977f + 0.00018f;
        }
        benchmark::DoNotOptimize(x0); benchmark::DoNotOptimize(x1);
        benchmark::DoNotOptimize(x2); benchmark::DoNotOptimize(x3);
        benchmark::DoNotOptimize(x4); benchmark::DoNotOptimize(x5);
        benchmark::DoNotOptimize(x6); benchmark::DoNotOptimize(x7);
    }
    state.SetItemsProcessed(state.iterations() * kRounds * 8);
}
BENCHMARK(BM_ScalarFpThroughput);

inline uint64_t integerMix(uint64_t x)
{
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    return x * 0x2545F4914F6CDD1Dull;
}

static void BM_IntegerDependency(benchmark::State& state)
{
    constexpr int kUpdates = 512;
    uint64_t x = 0x123456789abcdef0ull;
    for (auto _ : state)
    {
        for (int i = 0; i < kUpdates; ++i) x = integerMix(x);
        benchmark::DoNotOptimize(x);
    }
    state.SetItemsProcessed(state.iterations() * kUpdates);
}
BENCHMARK(BM_IntegerDependency);

static void BM_IntegerThroughput(benchmark::State& state)
{
    constexpr int kRounds = 128;
    uint64_t x0 = 1, x1 = 2, x2 = 3, x3 = 4;
    for (auto _ : state)
    {
        for (int i = 0; i < kRounds; ++i)
        {
            x0 = integerMix(x0); x1 = integerMix(x1);
            x2 = integerMix(x2); x3 = integerMix(x3);
        }
        benchmark::DoNotOptimize(x0); benchmark::DoNotOptimize(x1);
        benchmark::DoNotOptimize(x2); benchmark::DoNotOptimize(x3);
    }
    state.SetItemsProcessed(state.iterations() * kRounds * 4);
}
BENCHMARK(BM_IntegerThroughput);

static void BM_VectorMulAddDependency128(benchmark::State& state)
{
    constexpr int kUpdates = 1024;
    Vec4 x = vset(0.25f), a = vset(0.99991f), b = vset(0.00013f);
    for (auto _ : state)
    {
        for (int i = 0; i < kUpdates; ++i) x = vadd(vmul(x, a), b);
        consume(x);
    }
    state.SetItemsProcessed(state.iterations() * kUpdates * 4);
}
BENCHMARK(BM_VectorMulAddDependency128);

static void BM_VectorMulAddThroughput128(benchmark::State& state)
{
    constexpr int kRounds = 128;
    const Vec4 a = vset(0.99991f), b = vset(0.00013f);
    Vec4 x0 = vset(0.11f), x1 = vset(0.22f), x2 = vset(0.33f), x3 = vset(0.44f);
    Vec4 x4 = vset(0.55f), x5 = vset(0.66f), x6 = vset(0.77f), x7 = vset(0.88f);
    for (auto _ : state)
    {
        for (int i = 0; i < kRounds; ++i)
        {
            x0 = vadd(vmul(x0, a), b); x1 = vadd(vmul(x1, a), b);
            x2 = vadd(vmul(x2, a), b); x3 = vadd(vmul(x3, a), b);
            x4 = vadd(vmul(x4, a), b); x5 = vadd(vmul(x5, a), b);
            x6 = vadd(vmul(x6, a), b); x7 = vadd(vmul(x7, a), b);
        }
        consume(x0); consume(x1); consume(x2); consume(x3);
        consume(x4); consume(x5); consume(x6); consume(x7);
    }
    state.SetItemsProcessed(state.iterations() * kRounds * 8 * 4);
}
BENCHMARK(BM_VectorMulAddThroughput128);

static void BM_ScalarSqrtDependency(benchmark::State& state)
{
    constexpr int kUpdates = 512;
    float x = 1.25f;
    for (auto _ : state)
    {
        for (int i = 0; i < kUpdates; ++i) x = std::sqrt(x + 0.75f);
        benchmark::DoNotOptimize(x);
    }
    state.SetItemsProcessed(state.iterations() * kUpdates);
}
BENCHMARK(BM_ScalarSqrtDependency);

static void BM_VectorSqrtDependency128(benchmark::State& state)
{
    constexpr int kUpdates = 512;
    Vec4 x = vset(1.25f), c = vset(0.75f);
    for (auto _ : state)
    {
        for (int i = 0; i < kUpdates; ++i) x = vsqrt(vadd(x, c));
        consume(x);
    }
    state.SetItemsProcessed(state.iterations() * kUpdates * 4);
}
BENCHMARK(BM_VectorSqrtDependency128);

static void BM_VectorSqrtThroughput128(benchmark::State& state)
{
    constexpr int kRounds = 128;
    const Vec4 c = vset(0.75f);
    Vec4 x0 = vset(1.1f), x1 = vset(1.2f), x2 = vset(1.3f), x3 = vset(1.4f);
    for (auto _ : state)
    {
        for (int i = 0; i < kRounds; ++i)
        {
            x0 = vsqrt(vadd(x0, c)); x1 = vsqrt(vadd(x1, c));
            x2 = vsqrt(vadd(x2, c)); x3 = vsqrt(vadd(x3, c));
        }
        consume(x0); consume(x1); consume(x2); consume(x3);
    }
    state.SetItemsProcessed(state.iterations() * kRounds * 4 * 4);
}
BENCHMARK(BM_VectorSqrtThroughput128);

static void BM_ScalarDivideDependency(benchmark::State& state)
{
    constexpr int kUpdates = 512;
    float x = 1.25f;
    for (auto _ : state)
    {
        for (int i = 0; i < kUpdates; ++i) x = (x + 0.0013f) / 1.00091f;
        benchmark::DoNotOptimize(x);
    }
    state.SetItemsProcessed(state.iterations() * kUpdates);
}
BENCHMARK(BM_ScalarDivideDependency);

static void BM_VectorDivideDependency128(benchmark::State& state)
{
    constexpr int kUpdates = 512;
    Vec4 x = vset(1.25f), a = vset(1.00091f), b = vset(0.0013f);
    for (auto _ : state)
    {
        for (int i = 0; i < kUpdates; ++i) x = vdiv(vadd(x, b), a);
        consume(x);
    }
    state.SetItemsProcessed(state.iterations() * kUpdates * 4);
}
BENCHMARK(BM_VectorDivideDependency128);

static void BM_VectorDivideThroughput128(benchmark::State& state)
{
    constexpr int kRounds = 128;
    const Vec4 a = vset(1.00091f), b = vset(0.0013f);
    Vec4 x0 = vset(1.1f), x1 = vset(1.2f), x2 = vset(1.3f), x3 = vset(1.4f);
    for (auto _ : state)
    {
        for (int i = 0; i < kRounds; ++i)
        {
            x0 = vdiv(vadd(x0, b), a); x1 = vdiv(vadd(x1, b), a);
            x2 = vdiv(vadd(x2, b), a); x3 = vdiv(vadd(x3, b), a);
        }
        consume(x0); consume(x1); consume(x2); consume(x3);
    }
    state.SetItemsProcessed(state.iterations() * kRounds * 4 * 4);
}
BENCHMARK(BM_VectorDivideThroughput128);

static void BM_VectorCompareMask128(benchmark::State& state)
{
    constexpr size_t kLanes = 4096;
    std::vector<float> a(kLanes), b(kLanes);
    hlodtest::DeterministicRng rng(0x12345678u);
    for (size_t i = 0; i < kLanes; ++i)
    {
        a[i] = float(int32_t(rng.next())) * (1.0f / 2147483648.0f);
        b[i] = float(int32_t(rng.next())) * (1.0f / 2147483648.0f);
    }
    uint32_t result = 0;
    for (auto _ : state)
    {
        benchmark::ClobberMemory();
        for (size_t i = 0; i < kLanes; i += 4)
            result ^= compareMask(vload(a.data() + i), vload(b.data() + i));
        benchmark::DoNotOptimize(result);
    }
    state.SetItemsProcessed(state.iterations() * int64_t(kLanes));
}
BENCHMARK(BM_VectorCompareMask128);

// ---- production traversal kernels ----------------------------------------

inline uint32_t wideAabbProduction(const hlod::WideBounds& b,
                                   const hlod::Frustum& fr,
                                   uint8_t inMask, uint8_t* outMasks)
{
    return hlod::testWideAabb(b, fr, inMask, outMasks);
}

struct WideAabbFixture
{
    static constexpr size_t kCases = 128;
    std::array<hlod::WideBounds, kCases> bounds{};
    std::array<hlod::float8, kCases> geometricError{};
    hlod::Frustum frustum{};
};

WideAabbFixture makeWideAabbFixture()
{
    WideAabbFixture f;
    f.frustum.plane[0] = { 0.98058f,  0.00000f,  0.19612f, 25.0f};
    f.frustum.plane[1] = {-0.98058f,  0.00000f,  0.19612f, 25.0f};
    f.frustum.plane[2] = { 0.00000f,  0.97014f,  0.24254f, 20.0f};
    f.frustum.plane[3] = { 0.00000f, -0.97014f,  0.24254f, 20.0f};
    f.frustum.plane[4] = { 0.00000f,  0.00000f,  1.00000f,  5.0f};
    f.frustum.plane[5] = { 0.00000f,  0.00000f, -1.00000f, 80.0f};

    hlodtest::DeterministicRng rng(0x6d2b79f5u);
    const auto random = [&rng](float lo, float hi) { return rng.uniform(lo, hi); };
    for (size_t i = 0; i < f.bounds.size(); ++i)
    {
        hlod::WideBounds& wide = f.bounds[i];
        for (uint32_t lane = 0; lane < hlod::kWide; ++lane)
        {
            const float cx = random(-60.0f, 60.0f);
            const float cy = random(-50.0f, 50.0f);
            const float cz = random(-20.0f, 100.0f);
            const float ex = random(0.05f, 8.0f);
            const float ey = random(0.05f, 8.0f);
            const float ez = random(0.05f, 8.0f);
            wide.setLane(lane, hlod::AABB::fromMinMax(
                {cx - ex, cy - ey, cz - ez, 0.0f},
                {cx + ex, cy + ey, cz + ez, 0.0f}));
            f.geometricError[i].v[lane] = random(0.1f, 50.0f);
        }
    }
    return f;
}

static void BM_KernelWideAabb(benchmark::State& state)
{
    constexpr int kCalls = 4096;
    const WideAabbFixture fixture = makeWideAabbFixture();
    const uint32_t activePlanes = uint32_t(state.range(0));
    const uint8_t inMask = uint8_t((1u << activePlanes) - 1u);

    uint64_t checksum[4] = {};
    for (auto _ : state)
    {
        for (int i = 0; i < kCalls; ++i)
        {
            uint8_t masks[hlod::kWide];
            const uint32_t alive = wideAabbProduction(
                fixture.bounds[size_t(i) & (WideAabbFixture::kCases - 1)],
                fixture.frustum, inMask, masks);
            uint64_t packed;
            std::memcpy(&packed, masks, sizeof(packed));
            checksum[i & 3] += packed ^ alive;
        }
        benchmark::DoNotOptimize(checksum);
    }
    state.SetItemsProcessed(state.iterations() * kCalls * hlod::kWide);
}
BENCHMARK(BM_KernelWideAabb)
    ->Args({1})->Args({3})->Args({6})->ArgName("active_planes");

static void BM_KernelDistanceErrorCurrent(benchmark::State& state)
{
    constexpr int kCalls = 4096;
    const WideAabbFixture fixture = makeWideAabbFixture();
    const hlod::float4 queryMin = hlod::float4::point(-1.0f, -1.0f, -1.0f);
    const hlod::float4 queryMax = hlod::float4::point(1.0f, 1.0f, 1.0f);
    std::array<hlod::float8, WideAabbFixture::kCases> output{};

    for (auto _ : state)
    {
        for (int i = 0; i < kCalls; ++i)
        {
            const size_t index = size_t(i) & (WideAabbFixture::kCases - 1);
            const hlod::float8 d2 = hlod::distanceToBoxesSq(
                fixture.bounds[index], queryMin, queryMax);
            output[index] = hlod::screenErrorFromSq8(
                fixture.geometricError[index], 935.0f, d2);
        }
        benchmark::ClobberMemory();
    }
    benchmark::DoNotOptimize(output.data());
    state.SetItemsProcessed(state.iterations() * kCalls * hlod::kWide);
}
BENCHMARK(BM_KernelDistanceErrorCurrent);

struct alignas(32) CacheHitRecord
{
    float validUntil;
    float kSlope;
    uint32_t epoch;
    uint32_t cutVersion;
    uint32_t begin;
    uint32_t counts;
    uint32_t depSlot;
    uint32_t depVersion;
};
static_assert(sizeof(CacheHitRecord) == 32);

struct CacheHitStamp
{
    uint32_t contentVersion;
    uint32_t inUse;
};

// 0 = exact production short-circuit expression
// 1 = eager independent predicates
// 2 = fixed-projection specialization (kTravel == 0)
template <uint32_t Mode>
static void runCacheHitValidation(benchmark::State& state)
{
    constexpr uint32_t kRecords = 25'000;
    constexpr uint32_t kStampCount = 64;
    std::vector<uint32_t> visible(kRecords);
    std::vector<CacheHitRecord> records(kRecords);
    std::vector<uint32_t> cutVersions(kRecords, 11u);
    std::array<CacheHitStamp, kStampCount> stamps{};

    for (CacheHitStamp& stamp : stamps) stamp = {7u, 1u};
    for (uint32_t i = 0; i < kRecords; ++i)
    {
        // Five percent of records conservatively miss because their instance
        // is not wholly inside the frustum; the rest take the common one-page
        // cache-hit path used by the cached forest benchmarks.
        const uint32_t mask = (i % 20u) == 0 ? 1u : 0u;
        visible[i] = i | (mask << 24);
        records[i] = {100.0f, 0.125f, 5u, 11u, i * 4u,
                      (1u << 30) | 4u, i & (kStampCount - 1u), 7u};
    }

    uint64_t checksum = 0;
    uint32_t misses = 0;
    for (auto _ : state)
    {
        benchmark::ClobberMemory();
        for (const uint32_t packed : visible)
        {
            const uint32_t instance = packed & 0x00ffffffu;
            const uint32_t mask = packed >> 24;
            const CacheHitRecord& r = records[instance];
            const uint32_t depCount = r.counts >> 30;

            bool hit;
            if constexpr (Mode == 1)
            {
                // Same predicates as the production path, but no
                // short-circuit chain: expose independent comparisons to the
                // out-of-order core and combine their boolean results once.
                hit = mask == 0;
                hit &= 1.0f + r.kSlope * 0.5f < r.validUntil;
                hit &= r.epoch == 5u;
                hit &= r.cutVersion == cutVersions[instance];
            }
            else if constexpr (Mode == 2)
            {
                hit = mask == 0 &&
                      1.0f < r.validUntil &&
                      r.epoch == 5u &&
                      r.cutVersion == cutVersions[instance];
            }
            else
            {
                hit = mask == 0 &&
                      1.0f + r.kSlope * 0.5f < r.validUntil &&
                      r.epoch == 5u &&
                      r.cutVersion == cutVersions[instance];
            }
            if (hit && depCount != 0)
            {
                const CacheHitStamp& stamp = stamps[r.depSlot];
                hit = stamp.inUse && stamp.contentVersion == r.depVersion;
            }
            if (hit)
                checksum += uint64_t(r.begin) + (r.counts & 0x3fffffffu);
            else
                ++misses;
        }
        benchmark::DoNotOptimize(checksum);
        benchmark::DoNotOptimize(misses);
    }
    state.SetItemsProcessed(state.iterations() * kRecords);
}

static void BM_KernelCacheHitShortCircuit(benchmark::State& state)
{
    runCacheHitValidation<0>(state);
}
BENCHMARK(BM_KernelCacheHitShortCircuit);

static void BM_KernelCacheHitEager(benchmark::State& state)
{
    runCacheHitValidation<1>(state);
}
BENCHMARK(BM_KernelCacheHitEager);

static void BM_KernelCacheHitFixedProjection(benchmark::State& state)
{
    runCacheHitValidation<2>(state);
}
BENCHMARK(BM_KernelCacheHitFixedProjection);

// The cached hierarchical workload emits four CutEntries for each of 25k
// visible instances. Keep the production append path visible as a standalone
// cross-machine bandwidth/loop-overhead probe.
static void BM_OutputAppendBuffer(benchmark::State& state)
{
    constexpr uint32_t kRanges = 25'000;
    const uint32_t entriesPerRange = uint32_t(state.range(0));
    const size_t entryCount = size_t(kRanges) * entriesPerRange;
    std::vector<hlod::CutEntry> input(entryCount);
    for (size_t i = 0; i < entryCount; ++i)
    {
        input[i] = hlod::CutEntry(
            hlod::NodeHandle{uint32_t(i) & 0xffffu, uint32_t(i) & 0xffu, 1u},
            uint8_t(i), uint32_t(i) & hlod::kInstanceIdMask);
    }

    hlod::AppendBuffer<hlod::CutEntry> output;
    output.reserve(entryCount);
    for (auto _ : state)
    {
        output.clear();
        for (uint32_t range = 0; range < kRanges; ++range)
        {
            const hlod::CutEntry* p =
                input.data() + size_t(range) * entriesPerRange;
            output.append(p, entriesPerRange);
        }
        benchmark::DoNotOptimize(output.data());
        benchmark::ClobberMemory();
    }
    state.SetItemsProcessed(state.iterations() * int64_t(entryCount));
    state.SetBytesProcessed(
        state.iterations() * int64_t(entryCount * sizeof(hlod::CutEntry)));
}
BENCHMARK(BM_OutputAppendBuffer)
    ->Arg(1)
    ->Arg(4)
    ->Arg(8)
    ->Arg(16)
    ->Arg(64)
    ->ArgName("entries_per_range");

// ---- memory hierarchy ------------------------------------------------------

struct alignas(64) CacheLine
{
    uint64_t word[8];
};
static_assert(sizeof(CacheLine) == 64);

static void BM_SequentialRead(benchmark::State& state)
{
    const size_t bytes = size_t(state.range(0));
    std::vector<CacheLine> data(std::max<size_t>(1, bytes / sizeof(CacheLine)));
    for (size_t i = 0; i < data.size(); ++i)
        for (uint32_t j = 0; j < 8; ++j)
            data[i].word[j] = uint64_t(i * 8 + j + 1);

    uint64_t result = 0;
    for (auto _ : state)
    {
        benchmark::ClobberMemory();
        uint64_t s0 = 0, s1 = 0, s2 = 0, s3 = 0;
        uint64_t s4 = 0, s5 = 0, s6 = 0, s7 = 0;
        for (const CacheLine& line : data)
        {
            s0 += line.word[0]; s1 += line.word[1];
            s2 += line.word[2]; s3 += line.word[3];
            s4 += line.word[4]; s5 += line.word[5];
            s6 += line.word[6]; s7 += line.word[7];
        }
        result ^= s0 ^ s1 ^ s2 ^ s3 ^ s4 ^ s5 ^ s6 ^ s7;
        benchmark::DoNotOptimize(result);
    }
    state.SetBytesProcessed(state.iterations() * int64_t(data.size() * 64));
}
BENCHMARK(BM_SequentialRead)
    ->Arg(32 * kKiB)->Arg(128 * kKiB)->Arg(512 * kKiB)
    ->Arg(2 * kMiB)->Arg(8 * kMiB)->Arg(64 * kMiB)
    ->ArgName("bytes");

static void BM_Memcpy(benchmark::State& state)
{
    const size_t bytes = size_t(state.range(0));
    std::vector<uint8_t> source(bytes), destination(bytes);
    std::iota(source.begin(), source.end(), uint8_t(0));
    for (auto _ : state)
    {
        std::memcpy(destination.data(), source.data(), bytes);
        benchmark::ClobberMemory();
    }
    benchmark::DoNotOptimize(destination.data());
    state.SetBytesProcessed(state.iterations() * int64_t(bytes));
}
BENCHMARK(BM_Memcpy)
    ->Arg(32 * kKiB)->Arg(128 * kKiB)->Arg(512 * kKiB)
    ->Arg(2 * kMiB)->Arg(8 * kMiB)->Arg(64 * kMiB)
    ->ArgName("bytes");

// Constant-stride traversal over every cache line in a power-of-two working
// set. Odd strides make one full cycle; increasing gaps show how far each
// platform's hardware prefetcher can stay ahead without software hints.
static void BM_StridedCacheLineRead(benchmark::State& state)
{
    const size_t bytes = size_t(state.range(0));
    const size_t stride = size_t(state.range(1));
    const size_t count = std::bit_floor(
        std::max<size_t>(2, bytes / sizeof(CacheLine)));
    std::vector<CacheLine> data(count);
    for (size_t i = 0; i < count; ++i) data[i].word[0] = uint64_t(i + 1);

    size_t index = 0;
    uint64_t result = 0;
    for (auto _ : state)
    {
        benchmark::ClobberMemory();
        uint64_t sum = 0;
        for (size_t i = 0; i < count; ++i)
        {
            sum += data[index].word[0];
            index = (index + stride) & (count - 1);
        }
        result ^= sum;
        benchmark::DoNotOptimize(result);
    }
    state.SetItemsProcessed(state.iterations() * int64_t(count));
    state.counters["stride_bytes"] = double(stride * sizeof(CacheLine));
}
BENCHMARK(BM_StridedCacheLineRead)
    ->Args({32 * kMiB, 1})->Args({32 * kMiB, 3})
    ->Args({32 * kMiB, 7})->Args({32 * kMiB, 15})
    ->Args({32 * kMiB, 31})->Args({32 * kMiB, 63})
    ->ArgNames({"bytes", "stride_lines"});

inline uint32_t reverseBits(uint32_t x)
{
    x = ((x >> 1) & 0x55555555u) | ((x & 0x55555555u) << 1);
    x = ((x >> 2) & 0x33333333u) | ((x & 0x33333333u) << 2);
    x = ((x >> 4) & 0x0f0f0f0fu) | ((x & 0x0f0f0f0fu) << 4);
    x = ((x >> 8) & 0x00ff00ffu) | ((x & 0x00ff00ffu) << 8);
    return (x >> 16) | (x << 16);
}

static std::vector<uint32_t> makePointerCycle(size_t bytes)
{
    const uint32_t count = uint32_t(std::bit_floor(
        std::max<size_t>(2, bytes / sizeof(uint32_t))));
    const uint32_t bits = uint32_t(std::countr_zero(count));
    const auto order = [bits](uint32_t i) { return reverseBits(i) >> (32 - bits); };
    std::vector<uint32_t> next(count);
    for (uint32_t i = 0; i < count; ++i)
        next[order(i)] = order((i + 1) & (count - 1));
    return next;
}

template<int Chains>
static void BM_PointerChase(benchmark::State& state)
{
    constexpr int kSteps = 65536;
    std::vector<uint32_t> next = makePointerCycle(size_t(state.range(0)));
    std::array<uint32_t, Chains> index{};
    for (int c = 0; c < Chains; ++c)
        index[c] = reverseBits(uint32_t(c * next.size() / Chains)) >>
                   (32 - uint32_t(std::countr_zero(uint32_t(next.size()))));

    for (auto _ : state)
    {
        for (int step = 0; step < kSteps; ++step)
            for (int c = 0; c < Chains; ++c) index[c] = next[index[c]];
        benchmark::DoNotOptimize(index.data());
    }
    state.SetItemsProcessed(state.iterations() * int64_t(kSteps) * Chains);
    state.counters["chains"] = Chains;
}
BENCHMARK_TEMPLATE(BM_PointerChase, 1)
    ->Arg(32 * kKiB)->Arg(128 * kKiB)->Arg(512 * kKiB)
    ->Arg(2 * kMiB)->Arg(8 * kMiB)->Arg(32 * kMiB)
    ->ArgName("bytes");
BENCHMARK_TEMPLATE(BM_PointerChase, 4)
    ->Arg(8 * kMiB)->Arg(32 * kMiB)->ArgName("bytes");
BENCHMARK_TEMPLATE(BM_PointerChase, 8)
    ->Arg(8 * kMiB)->Arg(32 * kMiB)->ArgName("bytes");

struct alignas(64) Record64
{
    uint64_t word[8];
};
static_assert(sizeof(Record64) == 64);

static void BM_IndexedRecordRead(benchmark::State& state)
{
    const size_t bytes = size_t(state.range(0));
    const bool randomOrder = state.range(1) != 0;
    const int streams = int(state.range(2));
    const uint32_t count = uint32_t(std::bit_floor(
        std::max<size_t>(2, bytes / sizeof(Record64))));
    const uint32_t bits = uint32_t(std::countr_zero(count));
    std::vector<Record64> first(count), second(count);
    std::vector<uint32_t> order(count);
    for (uint32_t i = 0; i < count; ++i)
    {
        first[i].word[0] = uint64_t(i) * 17 + 1;
        second[i].word[0] = uint64_t(i) * 31 + 3;
        order[i] = randomOrder ? reverseBits(i) >> (32 - bits) : i;
    }

    uint64_t result = 0;
    for (auto _ : state)
    {
        benchmark::ClobberMemory();
        uint64_t sum = 0;
        if (streams == 1)
            for (const uint32_t i : order) sum += first[i].word[0];
        else
            for (const uint32_t i : order)
                sum += first[i].word[0] + second[i].word[0];
        result ^= sum;
        benchmark::DoNotOptimize(result);
    }
    state.SetItemsProcessed(state.iterations() * int64_t(count) * streams);
    state.counters["random"] = randomOrder ? 1.0 : 0.0;
    state.counters["streams"] = streams;
}
BENCHMARK(BM_IndexedRecordRead)
    ->Args({8 * kMiB, 0, 1})->Args({8 * kMiB, 1, 1})
    ->Args({8 * kMiB, 0, 2})->Args({8 * kMiB, 1, 2})
    ->Args({32 * kMiB, 0, 1})->Args({32 * kMiB, 1, 1})
    ->Args({32 * kMiB, 0, 2})->Args({32 * kMiB, 1, 2})
    ->ArgNames({"bytes", "random", "streams"});

// ---- control flow and mask handling ---------------------------------------

HLOD_MACHINE_NOINLINE uint32_t branchArmA(uint32_t x)
{
    return (x * 1664525u + 1013904223u) ^ 0x9e3779b9u;
}

HLOD_MACHINE_NOINLINE uint32_t branchArmB(uint32_t x)
{
    return std::rotl(x ^ 0x85ebca6bu, 13) + 0xc2b2ae35u;
}

static void BM_BranchDispatch(benchmark::State& state)
{
    constexpr size_t kCount = 16384;
    const int pattern = int(state.range(0)); // 0 constant, 1 alternating, 2 random
    std::vector<uint8_t> choices(kCount);
    hlodtest::DeterministicRng rng(0x31415926u);
    for (size_t i = 0; i < kCount; ++i)
    {
        choices[i] = pattern == 0 ? 0 : pattern == 1 ? uint8_t(i & 1)
                                                      : uint8_t(rng.next() >> 31);
    }

    uint32_t value = 1;
    for (auto _ : state)
    {
        for (const uint8_t choice : choices)
            value = choice ? branchArmA(value) : branchArmB(value);
        benchmark::DoNotOptimize(value);
    }
    state.SetItemsProcessed(state.iterations() * int64_t(kCount));
}
BENCHMARK(BM_BranchDispatch)
    ->Args({0})->Args({1})->Args({2})->ArgName("pattern");

static void BM_BitMaskIteration(benchmark::State& state)
{
    constexpr size_t kCount = 4096;
    const uint32_t setBits = uint32_t(state.range(0));
    std::vector<uint32_t> masks(kCount);
    for (uint32_t i = 0; i < kCount; ++i)
    {
        uint32_t mask = setBits == 32 ? 0xffffffffu : (1u << setBits) - 1u;
        masks[i] = std::rotl(mask, int(i & 31));
    }

    uint64_t result = 0;
    for (auto _ : state)
    {
        for (uint32_t mask : masks)
            while (mask)
            {
                result += std::countr_zero(mask);
                mask &= mask - 1;
            }
        benchmark::DoNotOptimize(result);
    }
    state.SetItemsProcessed(state.iterations() * int64_t(kCount) * setBits);
}
BENCHMARK(BM_BitMaskIteration)
    ->Args({1})->Args({4})->Args({8})->Args({16})->ArgName("set_bits");

} // namespace

int main(int argc, char** argv)
{
    benchmark::MaybeReenterWithoutASLR(argc, argv);
    benchmark::Initialize(&argc, argv);
    benchmark::AddCustomContext("machine_vector_backend", kVectorBackend);
    benchmark::AddCustomContext("hlod_kernel_backend", kKernelBackend);
    benchmark::AddCustomContext("machine_compiler", kCompiler);
    if (benchmark::ReportUnrecognizedArguments(argc, argv)) return 1;
    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();
    return 0;
}
