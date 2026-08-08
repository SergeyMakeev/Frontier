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

See [API.md](docs/API.md) for the complete integration guide and lifetime rules.

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

Median wall time per frame, lower is better:

| Architecture / CPU | 10k static | 10k + 1k movers | 80k + 4k movers |
|---|---:|---:|---:|
| x86-64 — EPYC 9654 | 0.063 ms | 0.129 ms | 0.548 ms |
| x86-64 — i9-12900K | 0.087 ms | 0.178 ms | 0.882 ms |
| arm64 — Apple M2 Max | 0.040 ms | 0.073 ms | 0.356 ms |
| arm64 — Cortex-A72/A53 SBC | 0.261 ms | 1.047 ms | 3.736 ms |

Each case uses one continuously moving 1080p view over fully resident 85-node
hierarchies. The 10k world contains 700 assets; the 80k world shares one asset.
Dynamic cases include transform submission through `MotionGroup`, TLAS updates,
and `selectCut`; the static case measures `selectCut` with no object motion.

These are deterministic 600-frame Release benchmarks using scheduler-default
placement: AVX2 on x86-64 and NEON on arm64. They exclude rendering, streaming
IO, and other engine work, so treat them as scale indicators rather than frame
time guarantees. The captures use revision `bf60f39`. See
[ARCHITECTURE.md](docs/ARCHITECTURE.md) for design analysis, or use
`run_all_perf.sh` / `run_all_perf.bat` to collect a complete report on another
machine.

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
24-bit stable `instance()` and 8-bit `errorCode()`. Codes below 128 are at or
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
  `applyUpdates`. Group large batches by instance and page when practical so
  consecutive refits reuse nearby overlay data; submission order does not
  affect correctness.
- For a persistent cohort of rigid instances that moves every frame, construct
  a `World::MotionGroup` from its `InstanceRef` values and pass its positions
  to `World::moveInstances`. Positions retain the group's caller-visible order;
  the group caches a physical TLAS-friendly order internally and refreshes it
  automatically after `optimize()`. Use scalar `moveInstance` for occasional
  or continually changing cohorts.
- The first TLAS build establishes spatial instance layout and routine updates
  preserve it. Call `World::optimize()` only at an occasional safe point after
  disruptive changes such as a teleport, level transition, or heavy churn. It
  compacts and restores spatial locality without invalidating public handles.
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
in [hlod_design.md](docs/hlod_design.md).

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
validates borrowed storage, which must outlive the registered asset. The
current reader accepts page blob version 2.

## Building and testing

CMake 3.24 or newer and a C++20 compiler are required:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
```

Run `run_perf_bench.sh` or `run_perf_bench.bat` for a focused Release
benchmark. Run `run_all_perf.sh <machine-label>` on macOS/Linux or
`run_all_perf.bat <machine-label>` on Windows to produce a complete,
self-contained cross-machine report. See [BENCHMARKING.md](docs/BENCHMARKING.md) for
the datasets, platform behavior, environment overrides, and macOS CPU-counter
capture.

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

- [API.md](docs/API.md): public API, examples, ownership, and lifetime rules.
- [ARCHITECTURE.md](docs/ARCHITECTURE.md): current implementation architecture.
- [hlod_design.md](docs/hlod_design.md): behavioral invariants and complexity.
- [BENCHMARKING.md](docs/BENCHMARKING.md): reproducible performance collection and profiling.
- [HISTORY.md](docs/HISTORY.md): implementation history and experimental evidence.
- [docs/archive](docs/archive): archived engineering records.

Dependencies: GoogleTest v1.17.0 plus upstream commit `fa8438ae` for the Clang
21 build fix, and Google Benchmark v1.9.4. The library itself has no runtime
third-party dependency. HLodTree is available under the [MIT License](LICENSE).
