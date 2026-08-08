# HLodTree

[![CI](https://github.com/SergeyMakeev/HLod-tree/actions/workflows/ci.yml/badge.svg)](https://github.com/SergeyMakeev/HLod-tree/actions/workflows/ci.yml)

HLodTree is a C++20 library that chooses a view-dependent hierarchical-LOD
cut. You give it a hierarchy whose nodes are renderable proxies, tell it which
payloads and topology pages are resident, and receive one compact result that
separates what to draw now from what complete residency would select.

It exists for scenes where per-object LOD is not enough. Replacing every wall
with a cheaper wall still leaves a distant town with thousands of submissions;
a hierarchical cut can replace the entire town with one proxy, refine nearby
buildings, and stream deeper topology only where the camera needs it. The
library owns no meshes, materials, jobs, or renderer state, so it can sit in
front of an existing asset and rendering system.

### BLAS and TLAS terminology

BLAS describes an independently rooted **renderable hierarchy**, not
necessarily one object or one reusable mesh. A BLAS can represent a city
block, a skyscraper whose children are floors and walls, a terrain region, or
a conventional reusable asset. Every node has its own renderable
`UserPayload`, so selection may stop at any level. A one-node BLAS is equally
valid and is useful for things such as vehicles, characters, and other content
with no authored hierarchy.

The TLAS is the dynamic spatial acceleration structure over placements of
those independent BLAS roots. Its internal nodes provide bounds, layer masks,
and coarse contribution information; they are not renderable proxies and do
not appear in a cut. This lets a world mix large authored regions, reusable
hierarchies, and flat objects without imposing one artificial root for the
whole map. In API names such as `AssetHandle`, *asset* means a registered unit
of immutable page storage and sharing; it does not constrain what part of the
world that hierarchy represents.

Streaming has two independent dimensions. **Residency** says whether a known
node's render payload is loaded. The nearest resident fallback remains in the
current cut until more detailed resident nodes completely cover the visible
region; intermediate proxies need not be resident, and partial loads never
create holes or parent/child overlap. **Expansion and collapse** control
whether deeper topology is known at all: a collapsed expansion point is a
renderable coarse proxy. Attaching a child page expands that branch, while
explicit detach or cold-page garbage collection collapses it again. One
traversal returns a shared cut plus the current-only and ideal-only
differences, so the host can choose what to expand or load without forcing
HLodTree to own an IO system.

## Minimal example

This one-node BLAS is intentionally small so the complete build, instance,
view, and selection loop is visible. Real hierarchies add children with
`HLodBuilder::createNode` and split large hierarchies at expansion points.

```cpp
#include "hlod/builder.h"
#include "hlod/world.h"

int main()
{
    using namespace hlod;

    HLodBuilder builder;
    builder.createRoot(
        42,                       // opaque payload, e.g. a mesh-table index
        0.0f,                     // geometric error in world units
        AABB::fromCenterExtent(float4::point(0, 0, 0),
                               float4::vec(1, 1, 1)));

    World world;
    world.addInstance(builder.build(), float4::point(0, 0, 0));

    const Camera camera = makeLookAtCamera(
        float4::point(0, 2, -8), float4::point(0, 0, 0));

    View view;                            // owns cache, scratch, and cut storage
    world.applyUpdates();                 // publish a stable read-only snapshot
    const World& published = world;
    const CutView cut =
        view.selectCut(published, camera, CutParams{4.0f, 0.0f});

    const auto draw = [&](const CutEntry& entry)
    {
        UserPayload payloadToDraw;
        if (published.tryGetPayload(entry.nodeHandle, payloadToDraw))
            (void)payloadToDraw; // submit with entry.instance()
    };
    for (const CutEntry& entry : cut.shared) draw(entry);
    for (const CutEntry& entry : cut.currentOnly) draw(entry);
}
```

`CutParams::threshold` is the permitted projected error in pixels.
`CutParams::minPix` optionally culls entire instances whose maximum
contribution is smaller than that value; zero disables contribution culling.

## Performance at a glance

The representative workload resembles a forest, city, or prop field:

- 80,000 instances share one fully resident 85-node asset and are spread over
  a roughly 6.8 km square.
- A per-view `View` is always used. Selection returns a `CutView` with a
  4-pixel error threshold and `minPix=0`; because this workload is fully
  resident, every entry is in `shared`.
- Moving-camera cases use the same continuous 600-frame fly-through at 1080p
  and 16:9, with no camera cuts or teleports.
- Moving-object cases update 5% of the population (4,000 transforms) every
  frame.
- Single-view cases use one thread. The multiple-view section compares serial
  calls with six persistent worker threads. Measurements exclude rendering,
  asset IO, residency changes, and instance spawning or removal.

All steady-frame values below are median wall times from five 600-frame runs.
The four reports use source revision `e2c9c91`, deterministic workload seeds,
Release builds, registration-order benchmarks, and scheduler-default affinity.
The Windows builds use MSVC `/O2 /arch:AVX2`; both Arm builds use the AArch64
NEON backend. Real frame time can be higher under host load.

| Label | Hardware | Compiler | Captured (UTC) |
|---|---|---|---|
| EPYC | EPYC 9654, 96C/192T, 768 GiB DDR5-4800, Windows | MSVC 19.51 | 2026-08-08 |
| i9-12900K | Core i9-12900K, 16C/24T, 128 GiB DDR4-2667, Windows | MSVC 19.44 | 2026-08-08 |
| M2 Max | M2 Max, 8 performance + 4 efficiency cores, 64 GiB unified memory, macOS | Apple Clang 21 | 2026-08-08 |
| Mobile-class Arm SBC | 4× Cortex-A72 + 4× Cortex-A53, 7.7 GiB, Linux | GCC 13.3 | 2026-08-08 |

Across all 36 real-world cases, the geometric mean of median wall time is:

| Machine | Geometric mean | Throughput relative to i9-12900K |
|---|---:|---:|
| EPYC | 0.489 ms | 1.40× |
| i9-12900K | 0.684 ms | 1.00× |
| M2 Max | 0.315 ms | 2.17× |
| Mobile-class Arm SBC | 2.367 ms | 0.29× |

M2 Max is 1.55× faster than EPYC and 7.52× faster than the mobile-class
SBC in this aggregate. The SBC is 3.46× slower than the i9. This is not an ALU
ranking: the i9 still leads representative scalar and 128-bit multiply/add
throughput probes. Dense memory and production-like probes explain the results
better, although no single synthetic benchmark predicts the whole workload:

| 64 MiB machine probe | EPYC | i9-12900K | M2 Max | Mobile Arm SBC |
|---|---:|---:|---:|---:|
| Sequential read | 43.7 GiB/s | 35.7 GiB/s | 59.0 GiB/s | 7.1 GiB/s |
| `memcpy` | 24.4 GiB/s | 17.6 GiB/s | 53.5 GiB/s | 4.8 GiB/s |

The current workloads therefore remain substantially data-movement bound, with
SIMD kernel throughput becoming important on the smaller Arm cores. Relative
to the i9, EPYC is 1.40× faster end to end and has 1.22× sequential-read and
1.38× copy bandwidth. Relative to the M2 Max, the SBC is 7.52× slower end to
end, 8.26× slower on sequential reads, 11.1× slower on `memcpy`, and 6.27×
slower in the six-plane wide-AABB kernel. Its dense traversal order still lets
ordinary hardware prefetchers work; there is no new Arm-specific performance
cliff in these results.

The i9 ran nominal DDR4-3600 DIMMs at 2667 MT/s. Across the 36 end-to-end
medians, one EPYC case, two i9 cases, and one SBC case had coefficient of
variation above 5%; only one i9 case exceeded 10%, while none of the M2 cases
exceeded 5%. These remain normal-scheduler measurements, not pinned best-core
limits. The SBC additionally used its `ondemand` governor and could schedule on
either Cortex-A72 or Cortex-A53 cores, so treat it as a useful mobile-class
proxy rather than an Android device guarantee.

### Startup

These startup counters come from the camera-and-objects-moving arm at each
world size; startup does not depend on the later steady-frame motion mode.

| Operation | EPYC | i9-12900K | M2 Max | Mobile Arm SBC | Included work |
|---|---:|---:|---:|---:|---|
| Create the 80,000-instance world | 6.7 ms | 7.3 ms | 1.6 ms | 17.7 ms | Build and register the shared asset, add instances, and mark payloads resident |
| First 80,000-instance selection cycle | 55.1 ms | 95.7 ms | 30.9 ms | 321.3 ms | Build the initial quality TLAS, publish, select the first cut, and populate the `View` |
| Create the 10,000-instance / 700-asset world | 13.4 ms | 18.9 ms | 4.3 ms | 25.4 ms | Build and register 700 assets, add instances, and mark payloads resident |
| First 10,000-instance selection cycle | 5.9 ms | 10.5 ms | 3.4 ms | 24.3 ms | Build the initial quality TLAS, publish, select the first cut, and populate the `View` |

World creation does not force the initial quality TLAS build; the first
`applyUpdates` performs it before publishing the read-only snapshot. Treat the
first published selection cycle as level warm-up rather than steady latency.

That first non-empty build also places the dense instance streams in TLAS
traversal order, allowing cached selection to use ordinary hardware
prefetching instead of architecture-specific prefetch instructions. Routine
motion and structural TLAS rebuilds preserve the physical order: rebuilding a
tree around existing dense ids is cheap, while copying the entire population
after a small change is not. After disruptive motion or heavy spawn/despawn
churn, call `World::optimize()` at an occasional synchronization point such as
a loading screen, menu, or teleport. It flushes pending edits, performs a
quality rebuild, compacts dead instance slots, restores traversal order, and
keeps public `InstanceRef` and `CutEntry::instance()` ids stable. Existing
`View` objects remain valid and discard their indexed records on the next
selection only when this physical reorder actually occurs.

### Steady-frame breakdown: 80,000-instance world

| Machine / HLodTree work per frame | Camera and 4,000 objects moving | Static camera, 4,000 objects moving | Moving camera, static objects |
|---|---:|---:|---:|
| EPYC — submit transforms | 0.185 ms | 0.183 ms | 0 calls |
| EPYC — publish updates and maintain TLAS | <0.001 ms | <0.001 ms | <0.001 ms |
| EPYC — `selectCut` | 0.398 ms | 0.300 ms | 0.337 ms |
| **EPYC — total HLodTree work** | **0.583 ms** | **0.482 ms** | **0.337 ms** |
| i9-12900K — submit transforms | 0.489 ms | 0.515 ms | 0 calls |
| i9-12900K — publish updates and maintain TLAS | <0.001 ms | <0.001 ms | <0.001 ms |
| i9-12900K — `selectCut` | 0.570 ms | 0.474 ms | 0.450 ms |
| **i9-12900K — total HLodTree work** | **1.053 ms** | **0.994 ms** | **0.450 ms** |
| M2 Max — submit transforms | 0.058 ms | 0.058 ms | 0 calls |
| M2 Max — publish updates and maintain TLAS | <0.001 ms | <0.001 ms | <0.001 ms |
| M2 Max — `selectCut` | 0.289 ms | 0.240 ms | 0.255 ms |
| **M2 Max — total HLodTree work** | **0.346 ms** | **0.298 ms** | **0.256 ms** |
| Mobile Arm SBC — submit transforms | 2.485 ms | 2.488 ms | 0 calls |
| Mobile Arm SBC — publish updates and maintain TLAS | <0.001 ms | <0.001 ms | <0.001 ms |
| Mobile Arm SBC — `selectCut` | 1.897 ms | 1.645 ms | 1.261 ms |
| **Mobile Arm SBC — total HLodTree work** | **4.384 ms** | **4.130 ms** | **1.261 ms** |

The moving-camera cases average roughly 21,750-21,860 visible instances and a
24,780-25,020-entry render cut. With objects moving, the `View` reuses about
92.5%; with static objects it reuses 97.6%. The fixed-camera case averages
roughly 19,340-19,600 visible instances, a 22,060-22,680-entry cut, and 94%
reuse. The ranges reflect small architecture-specific floating-point changes at
LOD decision boundaries; the seeded worlds and camera paths are identical.

Bounded motion of the same 5% cohort stays on the grow-only refit path in these
runs. The escape budget counts distinct leaves since the last TLAS build, so the
same movers do not periodically force a rebuild merely by moving every frame.
If enough different instances escape, or accumulated lane area grows too far,
the next `applyUpdates` repairs the TLAS before selection begins.

### Smaller 10,000-instance world

The smaller test uses 10,000 instances spread over a roughly 2.4 km square.
They draw from 700 separately registered, fully resident 85-node assets with
maximum depth 3, instead of sharing one asset across the entire world. The
moving-object cases update exactly 1,000 instances per frame.

| Machine / HLodTree work per frame | Camera and 1,000 objects moving | Static camera, 1,000 objects moving | Moving camera, static objects |
|---|---:|---:|---:|
| EPYC — submit transforms | 44 µs | 44 µs | 0 calls |
| EPYC — publish updates and maintain TLAS | <0.1 µs | <0.1 µs | <0.1 µs |
| EPYC — `selectCut` | 86 µs | 60 µs | 63 µs |
| **EPYC — total HLodTree work** | **130 µs** | **104 µs** | **63 µs** |
| i9-12900K — submit transforms | 74 µs | 63 µs | 0 calls |
| i9-12900K — publish updates and maintain TLAS | <0.1 µs | <0.1 µs | <0.1 µs |
| i9-12900K — `selectCut` | 128 µs | 84 µs | 86 µs |
| **i9-12900K — total HLodTree work** | **202 µs** | **147 µs** | **86 µs** |
| M2 Max — submit transforms | 17 µs | 17 µs | 0 calls |
| M2 Max — publish updates and maintain TLAS | <0.1 µs | <0.1 µs | <0.1 µs |
| M2 Max — `selectCut` | 52 µs | 35 µs | 39 µs |
| **M2 Max — total HLodTree work** | **69 µs** | **53 µs** | **39 µs** |
| Mobile Arm SBC — submit transforms | 735 µs | 741 µs | 0 calls |
| Mobile Arm SBC — publish updates and maintain TLAS | 0.2 µs | 0.2 µs | 0.1 µs |
| Mobile Arm SBC — `selectCut` | 515 µs | 356 µs | 265 µs |
| **Mobile Arm SBC — total HLodTree work** | **1,259 µs** | **1,095 µs** | **265 µs** |

The moving-camera cases average roughly 2,740-2,810 visible instances and a
5,700-6,070-entry cut. Reuse is about 83-84% with 1,000 movers and 93% with
static objects. As above, close LOD decisions can differ slightly between the
x86 and Arm floating-point kernels.

### Multiple views

Each view needs its own `View` and output. With the same
moving-camera route and nearby view origins 24 metres apart, wall time for all
six selections is:

| Machine | Object motion | Serial | Concurrent | Speedup |
|---|---|---:|---:|---:|
| EPYC | Static objects | 2.019 ms | 0.387 ms | 5.21× |
| EPYC | 4,000 moving objects | 2.386 ms | 0.578 ms | 4.13× |
| i9-12900K | Static objects | 2.723 ms | 1.478 ms | 1.84× |
| i9-12900K | 4,000 moving objects | 3.395 ms | 1.498 ms | 2.27× |
| M2 Max | Static objects | 1.520 ms | 0.452 ms | 3.36× |
| M2 Max | 4,000 moving objects | 1.746 ms | 0.720 ms | 2.43× |
| Mobile Arm SBC | Static objects | 7.979 ms | 3.406 ms | 2.34× |
| Mobile Arm SBC | 4,000 moving objects | 11.705 ms | 4.796 ms | 2.44× |

Object transforms are applied once before these selections and are not included
in the table. The concurrent arms use six persistent worker threads; thread
creation is excluded. Scaling is below 6× because the views share memory
bandwidth and cache capacity, but the read-only selection phase removes
serialization between them. The EPYC static concurrent arm, i9 moving
concurrent arm, and SBC moving concurrent arm had coefficients of variation of
6.2%, 8.6%, and 7.5%, respectively; the remaining entries were below 5%.

On the mobile-class SBC, the representative 10,000-instance cases consume
0.265-1.259 ms per frame and the 80,000-instance cases consume 1.261-4.384 ms.
That is useful headroom inside a 16.7 ms frame, but the figures exclude the rest
of an engine and sustained mobile thermal throttling.

The main distinction is visible immediately: creating the quality TLAS is a
one-time cost, and object motion makes selection rewalk only the affected
instances. In these `e2c9c91` captures, scalar instance-motion updates account
for roughly 57-68% of the SBC moving-object totals. `World::MotionGroup`, added
after these captures, processes a persistent moving cohort in physical TLAS
order; the numbers above do not yet include that optimization. These are scale
estimates rather than platform promises; selection remains output-sensitive.
See [ARCHITECTURE.md](ARCHITECTURE.md) for specialized measurements and
methodology.

## What selection returns

One traversal returns a `CutView` over three disjoint, contiguous buffers:

- `shared`: selected in both the current render cut and fully-resident ideal
  cut;
- `currentOnly`: a resident fallback selected only because some more detailed
  ideal entries are not resident; and
- `idealOnly`: selected only by the fully-resident ideal cut.

Each field is a `std::span<const CutEntry>`. The owning storage lives in the
`View`, retains capacity between queries, and is valid until the next selection
or reset on that View. Use the explicit `CutResults` snapshot overload only
when a cut must outlive the next query. Engines that already own output memory
can instead use `CutResultSink` to write directly into fixed spans.

Render `shared + currentOnly`; inspect the fully-resident frontier as
`shared + idealOnly`. A high-error leaf on the ideal side is the point where
the caller may consult its external content graph and attach a child page. If
that graph has no children, the leaf is simply the finest authored
representation. There is deliberately no load-request type or built-in
deduplication: shared assets can produce the same node in many placements, and
the host owns IO priority, budgets, and content identity. Output order is
traversal-defined, not a priority order.

`CutEntry` is 12 bytes: a generation-stamped 64-bit `nodeHandle`, plus a packed
24-bit dense `instance()` and 8-bit `errorCode()`. Codes below 128 are at or
below the query threshold; codes at least 128 are above it. Use
`overThreshold()` for the exact refinement decision and
`approximateError(threshold)` when a pixel-scale estimate helps prioritize IO.
Resolve an opaque application payload only when needed with
`World::tryGetPayload`; keep renderer entity data in a caller table indexed by
`instance()`.

Payloads are values, not identities. Duplicates are legal and the `World`
never indexes them. Operations use generation-stamped `AssetHandle`,
`PageHandle`, `NodeHandle`, and `World::InstanceRef` values. A page collected while an
asynchronous load is in flight simply makes the returned node handle stale;
mutations using it become safe no-ops.

## Runtime model

- Immutable, versioned page blobs hold each BLAS's preorder node arrays and
  8-lane wide child blocks. A registered hierarchy can be instanced thousands
  of times while sharing page bytes, residency, and its attachment graph.
- A dynamic wide TLAS owns placement of independent BLAS roots, layer masks,
  coarse frustum and contribution culling, and incremental spawn/remove edits.
  TLAS leaves may reference either deep hierarchies or one-node BLASes. When a
  hierarchical root already satisfies the view's error threshold, selection
  can emit that renderable root before entering its page hierarchy; the TLAS
  itself remains non-renderable.
- Expansion points connect independently streamed pages. Residency changes
  propagate complete descendant coverage upward. Selection can therefore skip
  a missing intermediate proxy when resident descendants cover the visible
  region, while retaining the nearest resident fallback wherever they do not;
  the current cut stays free of holes and parent/child overlap.
- `setNodeBounds(instance, node, bounds)` creates bounds-only copy-on-write
  overlays for the affected instance. Refits are queued and published by
  `applyUpdates`. Submission order does not affect correctness, but it can
  materially affect large-batch throughput: group updates by instance and
  page when practical so refitting walks overlay memory locally. The library
  preserves caller order and deliberately does not sort the queue, because
  sorting can cost more than the refit. In the 80,000-update locality benchmark,
  grouped submission reduced submit-plus-refit time from 11.12 ms to 4.02 ms
  on the test i9.
- For a persistent cohort of rigid instances that moves every frame, construct
  a `World::MotionGroup` from its `InstanceRef` values and pass its positions
  to `World::moveInstances`. Positions retain the group's caller-visible order;
  the group caches a physical TLAS-friendly order internally and refreshes it
  automatically after `optimize()`. Use scalar `moveInstance` for occasional
  or continually changing cohorts.
- Every camera or shadow cascade owns a `View`. It contains damping, reusable
  cut records, traversal scratch, and selection statistics. Reuse is enabled
  by default and can be disabled with `setReuseEnabled(false)` for highly
  dynamic workloads.
- `View::selectCut` reads a published `const World`. Calls may run concurrently
  when every call has a distinct `View`, optional
  `PageUsageContext`, and output objects. Mutations and `collect` remain serial
  and must happen outside that selection phase.
- `PageUsageContext` is optional per view. It records page-use feedback without
  touching the World; `collect` later consumes only the contexts the caller
  chooses, so a primary camera can influence page retention while shadow
  cameras do not.
- An uncached `View` can fan out over visible instances through the host
  `parallelFor`; its scratch and statistics still remain view-owned, so it has
  the same threading contract as cached selection.

The current contracts, algorithms, lifecycle rules, and complexity bounds are
in [hlod_design.md](hlod_design.md).

## Integrating

The repository currently exposes a CMake target rather than an installed
package:

```cmake
add_subdirectory(path/to/HLod-tree)
target_link_libraries(your_target PRIVATE hlod)
```

Include `hlod/world.h` for runtime use and `hlod/builder.h` when producing page
blobs. `HlodContext` lets an engine provide aligned allocation and a blocking
`parallelFor`. Define `HLOD_FATAL(msg)` before the first HLodTree header if the
default `std::logic_error` policy does not suit the host.

An `HlodContext` used to allocate a `Page` must outlive that page, including
after ownership moves into a `World`. The `user` data referenced by
`WorldConfig::context` must likewise remain valid until the world is destroyed.

The page blob is the versioned disk representation. `Page::fromBytes` validates
and copies external bytes into aligned owned storage; `PageView::fromBytes`
validates borrowed storage, which must outlive the registered asset. Blob
format version 2 is not compatible with version 1.

## Building and testing

CMake 3.24 or newer and a C++20 compiler are required. GoogleTest and Google
Benchmark are fetched only when their corresponding targets are enabled.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
```

Run `build/bench/hlod_bench` on single-config systems or
`build\bench\Release\hlod_bench.exe` on a Visual Studio build.

On Windows, `run_perf_bench.bat` configures a dedicated `build-perf` directory,
builds only `hlod_bench` in Release with AVX2 enabled, and runs the documented
performance suite five times. Google Benchmark arguments replace that default,
for example:

```bat
run_perf_bench.bat
run_perf_bench.bat --benchmark_filter=BM_RootDecisionForest100k --benchmark_repetitions=7
```

On macOS, `run_perf_bench.sh` provides the same workflow and benchmark defaults.
On Apple Silicon it explicitly targets native `arm64` (even when launched from
a shell running under Rosetta), selecting the library's NEON path. Intel Macs
use AVX2 when available, with SSE2 as the fallback. Pass Google Benchmark
arguments in the same way:

```sh
./run_perf_bench.sh
./run_perf_bench.sh --benchmark_filter=BM_DeformationSubmissionOrder --benchmark_repetitions=7
```

Set `HLOD_PERF_BUILD_DIR` to use a build directory other than `build-perf`.

### Complete cross-machine report

Use `run_all_perf.sh` or `run_all_perf.bat` when comparing machines. The
collector builds Release tests and both benchmark executables, runs the full
correctness suite, and captures all three performance datasets used in this
project: real-world workloads, machine-characterization probes, and the
longer-running production-kernel probes. It also records the CPU, memory, OS,
power mode, compiler, build options, Git revision, and exact commands.

Give each run a short machine label:

```sh
./run_all_perf.sh m2-max
./run_all_perf.sh epyc
```

```bat
run_all_perf.bat i9
```

The scripts use normal scheduler placement without an affinity mask. Run them
while the machine is plugged in, in its normal high-performance power mode,
and otherwise idle. A complete run takes roughly 15 minutes on the reference
i9 and can vary with machine speed. Each invocation creates a timestamped
directory under `perf_reports/` and packages it as one `.zip` (`.tar.gz` only
when `zip` is not installed). Send the archives for comparison; each
contains:

- `real_world_perf.json`, `machine_perf.json`, and `arch_kernel_perf.json`;
- a concise `REPORT.md` and machine-readable `manifest.txt`;
- hardware, source, toolchain, command, build, test, and benchmark logs.

All randomized benchmark and test workloads use repository-owned xorshift32
generation with explicit fixed seeds. Float mapping, bounded integers, and
shuffle order are also implemented locally rather than through the standard
library, whose distribution and shuffle algorithms may differ by platform.
Thus a seed and source commit describe the same generated workload on MSVC,
libc++, and libstdc++. Google Benchmark's unseeded random-interleaving mode is
disabled as well, so benchmark cases run in registration order. The policy
identifier is stored in every report manifest.

Set `HLOD_ALL_PERF_BUILD_DIR`, `HLOD_PERF_REPORT_ROOT`, or `HLOD_PERF_LABEL` to
override the unified build directory, report location, or label respectively.

For cross-machine diagnosis, `run_machine_bench.sh` builds a separate
`hlod_machine_bench` executable and writes `machine_perf.json`. Its synthetic
probes isolate scalar dependency and throughput, 128-bit SIMD arithmetic and
compare-to-mask cost, square root/divide, sequential cache and memory
bandwidth, constant-stride hardware prefetch, dependent-load latency,
random-load parallelism, branch prediction, and sparse-mask iteration. It also
contains a small `BM_Kernel*` group using the active production SIMD backend.
Run it once on each machine under the same power conditions:

```sh
./run_machine_bench.sh
```

The executable is separate so diagnostic additions cannot change the code
layout of `hlod_bench`. Set `HLOD_MACHINE_BUILD_DIR` to override its dedicated
`build-machine-perf` directory. SIMD probes deliberately use one 128-bit vector
on both platforms (SSE2 on x86-64 and NEON on arm64); `BM_Kernel*` instead uses
the library backend reported as `hlod_kernel_backend` in the JSON context.

`run_arch_bench.sh` runs only those production-kernel probes and focused
candidate experiments, writing `arch_kernel_perf.json` for an i9/M-series
comparison:

```sh
./run_arch_bench.sh
```

On Apple silicon, `profile_macos_cpu.sh` records the cached hierarchical
workload using real hardware CPU counters and optimized source line tables. It
automatically selects Xcode 26's **CPU Bottlenecks** template or the older
**CPU Counters** template. If a current Xcode publishes the counter instrument
but not its GUI template to `xctrace`, the script composes the recording from
that instrument directly. It also discovers a suitable full Xcode installed
beside the currently selected command-line tools. It writes an Instruments
`.trace`, an exported table of contents, capture metadata, and a small
attachable `_summary.zip` under `profile_results/`. The summary contains the
process/thread bottleneck metrics, counter samples, core placement, and
time-profile hotspots. When supported by `xctrace`, export excludes the first
five seconds of process setup; older exporters include the full interval, which
can be filtered during analysis. The raw trace remains local because
hardware-counter captures can be hundreds of megabytes:

```sh
./profile_macos_cpu.sh
```

If recording produced a trace but post-processing failed, process the newest
existing trace without recording again:

```sh
./profile_macos_cpu.sh --process
```

Pass a trace path after `--process` to select a file other than the newest one.
Set `HLOD_PROFILE_EXPORT_START` to change the default `5s` export start.

Set `HLOD_DEVELOPER_DIR` to force a particular Xcode installation,
`HLOD_XCTRACE_TEMPLATE` to use a custom counter template, or
`HLOD_PROFILE_FILTER` to profile a different benchmark. Hardware-counter
recording may require enabling the terminal under **Privacy & Security >
Developer Tools**.

Normal configuration options:

| Option | Default | Meaning |
|---|---:|---|
| `HLOD_BUILD_TESTS` | `ON` | Build the correctness suite |
| `HLOD_BUILD_BENCH` | `ON` | Build the Google Benchmark suite |
| `HLOD_AVX2` | `ON` | Use AVX2/FMA on x86-64 when available |
| `HLOD_FORCE_SCALAR` | `OFF` | Disable intrinsic implementations |
| `HLOD_PROFILE_SYMBOLS` | `OFF` | Add optimized Clang source line tables for profiling |

## SIMD and CI

The packed format always has eight logical lanes. AVX2 processes all eight at
once; SSE2 and NEON use two four-lane halves; the fallback uses scalar loops.

| Backend | Selected when |
|---|---|
| AVX2 + FMA | `HLOD_AVX2=ON` on x86-64 |
| SSE2, with SSE4.1 blends when available | x86 without AVX2 |
| NEON | 64-bit ARM |
| scalar | `HLOD_FORCE_SCALAR=ON` or unsupported architectures |

CI builds and runs unit and performance tests on Linux, Windows, and macOS;
x86-64 and arm64; GCC, Clang, MSVC, clang-cl, and AppleClang; and every SIMD
fallback listed above.

## Documentation map

- [hlod_design.md](hlod_design.md): current API contracts and design.
- [ARCHITECTURE.md](ARCHITECTURE.md): implementation details and performance
  experiment journal.
- [docs/archive/HANDOFF-2026-08-05.md](docs/archive/HANDOFF-2026-08-05.md):
  archived rework record; historical, not current guidance.

Dependencies: GoogleTest v1.17.0 plus upstream commit `fa8438ae` for the Clang
21 build fix, and Google Benchmark v1.9.4. The library itself has no runtime
third-party dependency. HLodTree is available under the [MIT License](LICENSE).
