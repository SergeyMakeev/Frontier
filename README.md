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

    View view;                            // persistent state for this camera
    CutResults cut;
    world.applyUpdates();                 // publish a stable read-only snapshot
    const World& published = world;
    view.selectCut(published, camera, CutParams{4.0f, 0.0f}, cut);

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
- A per-view `View` is always used. Selection returns `CutResults` with a
  4-pixel error threshold and `minPix=0`; because this workload is fully
  resident, every entry is in `shared`.
- Moving-camera cases use the same continuous 600-frame fly-through at 1080p
  and 16:9, with no camera cuts or teleports.
- Moving-object cases update 5% of the population (4,000 transforms) every
  frame.
- Single-view cases use one thread. The multiple-view section compares serial
  calls with six persistent worker threads. Measurements exclude rendering,
  asset IO, residency changes, and instance spawning or removal.

The primary tables are the best observed values from at least five 600-frame
runs on a noisy shared 64-hardware-thread, 2.4 GHz EPYC, using one thread,
MSVC 19.51, Release `/O2 /arch:AVX2`, on 2026-08-05. Taking the best result
estimates the algorithm's uncontended cost; real frame time can be higher under
host load. A second machine is reported after the primary tables.

### Startup

| Operation | Time | Included work |
|---|---:|---|
| Create the world | 10.2-11.4 ms | Build and register the shared asset, add 80,000 instances, and mark its payloads resident |
| First published selection cycle | 53.1-58.0 ms | `applyUpdates` builds the initial quality TLAS; `selectCut` queries it, produces the first cut, and populates the `View` |

World creation does not force the initial quality TLAS build; the first
`applyUpdates` performs it before publishing the read-only snapshot. Treat the
first published selection cycle as level warm-up rather than steady latency.

### Steady-frame breakdown

| HLodTree work per frame | Camera and 4,000 objects moving | Static camera, 4,000 objects moving | Moving camera, static objects |
|---|---:|---:|---:|
| Submit 4,000 instance transforms | 0.144 ms | 0.139 ms | n/a |
| Publish updates and maintain the TLAS | <0.001 ms | <0.001 ms | <0.001 ms |
| `selectCut` | 0.400 ms | 0.296 ms | 0.330 ms |
| **Total HLodTree frame work** | **0.544 ms** | **0.435 ms** | **0.330 ms** |

The moving-camera cases average about 21,919 visible instances and a
24,986-entry render cut. With objects moving, the `View` reuses 92.6% of
visible instance cuts; with static objects it reuses 97.6%. The fixed-camera
case averages 19,602 visible instances, a 22,872-entry cut, and 94.1% reuse.
The `View` occupies 5.98 MiB after the fly-through and 3.22 MiB for the fixed
view.

Bounded motion of the same 5% cohort stays on the grow-only refit path in this
run. The escape budget counts distinct leaves since the last TLAS build, so the
same movers do not periodically force a rebuild merely by moving every frame.
If enough different instances escape, or accumulated lane area grows too far,
the next `applyUpdates` repairs the TLAS before selection begins.

### Smaller 10,000-instance world

The smaller test uses 10,000 instances spread over a roughly 2.4 km square.
They draw from 700 separately registered, fully resident 85-node assets with
maximum depth 3, instead of sharing one asset across the entire world. The
moving-object cases update exactly 1,000 instances per frame. Creating and
populating this world takes 13.1-14.5 ms; its first published selection cycle,
including the initial quality TLAS build and `View` population, takes
5.4-6.3 ms.

| HLodTree work per frame | Camera and 1,000 objects moving | Static camera, 1,000 objects moving | Moving camera, static objects |
|---|---:|---:|---:|
| Submit 1,000 instance transforms | 34 µs | 34 µs | n/a |
| Publish updates and maintain the TLAS | <0.1 µs | <0.1 µs | <0.1 µs |
| `selectCut` | 89 µs | 74 µs | 63 µs |
| **Total HLodTree frame work** | **123 µs** | **108 µs** | **63 µs** |

The moving-camera cases average about 2,782 visible instances (27.8% of the
world) and a 5,920-entry cut. Reuse is 82.4% with 1,000 movers and 93.3% with
static objects; the `View` occupies 1.87 MiB. The fixed-camera case averages
2,501 visible instances (25.0%), a 5,792-entry cut, 85.8% reuse, and a 0.45 MiB
of view state.

### Multiple views

Each view needs its own `View` and output. With the same
moving-camera route and nearby view origins 24 metres apart, wall time for all
six selections is:

| Execution | Static objects | 4,000 moving objects |
|---|---:|---:|
| Six views, serial | 2.50 ms | 2.99 ms |
| Six views, concurrent | 0.445 ms | 0.538 ms |
| **Speedup** | **5.6×** | **5.6×** |

Object transforms are applied once before these selections and are not included
in the table. The concurrent arms use six persistent worker threads; thread
creation is excluded. Six `View` objects occupy about 36 MiB. Scaling is below 6×
because the views share memory bandwidth and cache capacity, but the read-only
selection phase removes serialization between them.

### Second machine: Core i9-12900K, 128 GB

The same committed benchmark suite was run on a Core i9-12900K with 128 GB of
memory, Windows, MSVC 19.44, and Release `/O2 /arch:AVX2` on 2026-08-06. The
run showed substantial scheduler and CPU-state variance, so these are the best
of five captured repetitions, consistent with the methodology above.

| Startup operation | Best time |
|---|---:|
| Create the 80,000-instance world | 8.1 ms |
| First 80,000-instance published selection cycle | 52.8 ms |
| Create the 10,000-instance / 700-asset world | 15.2 ms |
| First 10,000-instance published selection cycle | 9.7 ms |

Steady-frame results use the same cuts, visibility, reuse rates, and moving
cohorts as the primary tables. `applyUpdates` remained below 0.001 ms per
frame.

| Scenario | Submit transforms | `selectCut` | Total HLodTree work |
|---|---:|---:|---:|
| 80k, moving camera and 4,000 objects | 0.667 ms | 0.659 ms | 1.326 ms |
| 80k, static camera and 4,000 objects | 0.501 ms | 0.442 ms | 0.943 ms |
| 80k, moving camera and static objects | n/a | 0.269 ms | 0.269 ms |
| 10k, moving camera and 1,000 objects | 76 µs | 139 µs | 216 µs |
| 10k, static camera and 1,000 objects | 82 µs | 128 µs | 210 µs |
| 10k, moving camera and static objects | n/a | 91 µs | 91 µs |

| Six-view selection | Serial | Concurrent | Speedup |
|---|---:|---:|---:|
| Static objects | 2.366 ms | 1.654 ms | 1.43× |
| 4,000 moving objects | 4.161 ms | 1.765 ms | 2.36× |

The main distinction is visible immediately: creating the quality TLAS is a
one-time cost, transform updates are relatively small, and object motion makes
selection rewalk only the affected instances. These are scale estimates rather
than platform promises; selection remains output-sensitive. See
[ARCHITECTURE.md](ARCHITECTURE.md) for specialized measurements and methodology.

## What selection returns

One traversal fills three disjoint `CutResults` vectors:

- `shared`: selected in both the current render cut and fully-resident ideal
  cut;
- `currentOnly`: a resident fallback selected only because some more detailed
  ideal entries are not resident; and
- `idealOnly`: selected only by the fully-resident ideal cut.

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
  `applyUpdates`.
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

Normal configuration options:

| Option | Default | Meaning |
|---|---:|---|
| `HLOD_BUILD_TESTS` | `ON` | Build the 100-test correctness suite |
| `HLOD_BUILD_BENCH` | `ON` | Build the Google Benchmark suite |
| `HLOD_AVX2` | `ON` | Use AVX2/FMA on x86-64 when available |
| `HLOD_FORCE_SCALAR` | `OFF` | Disable intrinsic implementations |

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
