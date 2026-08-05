# HLodTree

[![CI](https://github.com/SergeyMakeev/HLod-tree/actions/workflows/ci.yml/badge.svg)](https://github.com/SergeyMakeev/HLod-tree/actions/workflows/ci.yml)

HLodTree is a C++20 library that chooses a view-dependent hierarchical-LOD
cut. You give it a hierarchy whose nodes are renderable proxies, tell it which
payloads and topology pages are resident, and receive the opaque 64-bit
payloads to draw.

It exists for scenes where per-object LOD is not enough. Replacing every wall
with a cheaper wall still leaves a distant town with thousands of submissions;
a hierarchical cut can replace the entire town with one proxy, refine nearby
buildings, and stream deeper topology only where the camera needs it. The
library owns no meshes, materials, jobs, or renderer state, so it can sit in
front of an existing asset and rendering system.

Streaming has two independent dimensions. **Residency** says whether a known
node's render payload is loaded. A parent remains in the actual cut until every
child needed to replace it is resident, so partial loads never create holes or
parent/child overlap. **Expansion and collapse** control whether deeper topology
is known at all: a collapsed expansion point is a renderable coarse proxy;
attaching a child page expands that branch, while explicit detach or cold-page
garbage collection collapses it again. The ideal cut and load requests tell the
host what to expand and load next without forcing HLodTree to own an IO system.

## Minimal example

This one-node asset is intentionally small so the complete build, instance,
view, and selection loop is visible. Real assets add children with
`HLodBuilder::createNode` and split large hierarchies at expansion points.

```cpp
#include <utility>
#include <vector>

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

    const CullView view = makeLookAtView(
        float4::point(0, 2, -8), float4::point(0, 0, 0));

    SelectionContext selection;          // one per view
    std::vector<CutEntry> cut;
    world.applyUpdates();                 // publish a stable read-only snapshot
    const World& published = world;
    published.selectCut(view, CutParams{4.0f, 0.0f}, selection, cut);

    for (const CutEntry& entry : cut)
    {
        const UserPayload payloadToDraw = entry.payload;
        (void)payloadToDraw; // submit through your renderer
    }
}
```

`CutParams::threshold` is the permitted projected error in pixels.
`CutParams::minPix` optionally culls entire instances whose maximum
contribution is smaller than that value; zero disables contribution culling.

## Performance at a glance

The representative workload resembles a forest, city, or prop field:

- 80,000 instances share one fully resident 85-node asset and are spread over
  a roughly 6.8 km square.
- A per-view `SelectionContext` is always used. Selection requests only the
  actual render cut, with a 4-pixel error threshold and `minPix=0`.
- Moving-camera cases use the same continuous 600-frame fly-through at 1080p
  and 16:9, with no camera cuts or teleports.
- Moving-object cases update 5% of the population (4,000 transforms) every
  frame.
- Single-view cases use one thread. The multiple-view section compares serial
  calls with six persistent worker threads. Measurements exclude rendering,
  asset IO, residency changes, and instance spawning or removal.

Measurements are the best repeat from at least ten 600-frame runs on a noisy
shared 64-hardware-thread, 2.4 GHz EPYC, using one thread, MSVC 19.51, Release
`/O2 /arch:AVX2`, on 2026-08-05. Taking the best recurring result estimates the
algorithm's uncontended cost; real frame time can be higher under host load.

### Startup

| Operation | Time | Included work |
|---|---:|---|
| Create the world | 10.4-11.4 ms | Build and register the shared asset, add 80,000 instances, and mark its payloads resident |
| First published selection cycle | 50-55 ms | `applyUpdates` builds the initial quality TLAS; `selectCut` queries it, produces the first cut, and populates the `SelectionContext` |

World creation does not force the initial quality TLAS build; the first
`applyUpdates` performs it before publishing the read-only snapshot. Treat the
first published selection cycle as level warm-up rather than steady latency.

### Steady-frame breakdown

| HLodTree work per frame | Camera and 4,000 objects moving | Static camera, 4,000 objects moving | Moving camera, static objects |
|---|---:|---:|---:|
| Apply transform updates and maintain the TLAS | 0.15 ms | 0.14 ms | n/a |
| Publish queued node-bound changes with `applyUpdates` | <0.001 ms | <0.001 ms | <0.001 ms |
| `selectCut` | 0.50 ms | 0.40 ms | 0.40 ms |
| **Total HLodTree frame work** | **0.65 ms** | **0.54 ms** | **0.40 ms** |

The moving-camera cases average about 21,919 visible instances and a
24,986-entry render cut. With objects moving, the context reuses 92.6% of
visible instance cuts; with static objects it reuses 97.6%. The fixed-camera
case averages 19,602 visible instances, a 22,872-entry cut, and 94.1% reuse.
The context occupies 8.13 MiB after the fly-through and 4.37 MiB for the fixed
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
populating this world takes 12.5-13.8 ms; its first published selection cycle,
including the initial quality TLAS build and context population, takes
5.3-6.4 ms.

| HLodTree work per frame | Camera and 1,000 objects moving | Static camera, 1,000 objects moving | Moving camera, static objects |
|---|---:|---:|---:|
| Apply transform updates and maintain the TLAS | 34 µs | 33 µs | n/a |
| Publish queued node-bound changes with `applyUpdates` | <0.1 µs | <0.1 µs | <0.1 µs |
| `selectCut` | 121 µs | 105 µs | 80 µs |
| **Total HLodTree frame work** | **155 µs** | **139 µs** | **80 µs** |

The moving-camera cases average about 2,782 visible instances (27.8% of the
world) and a 5,920-entry cut. Reuse is 82.4% with 1,000 movers and 93.3% with
static objects; the context occupies 2.52 MiB. The fixed-camera case averages
2,501 visible instances (25.0%), a 5,792-entry cut, 85.8% reuse, and a 0.61 MiB
context.

### Multiple views

Each view needs its own `SelectionContext` and output. With the same
moving-camera route and nearby view origins 24 metres apart, wall time for all
six selections is:

| Execution | Static objects | 4,000 moving objects |
|---|---:|---:|
| Six views, serial | 3.75 ms | 4.23 ms |
| Six views, concurrent | 0.94 ms | 1.08 ms |
| **Speedup** | **4.0×** | **3.9×** |

Object transforms are applied once before these selections and are not included
in the table. The concurrent arms use six persistent worker threads; thread
creation is excluded. Six contexts occupy about 49 MiB. Scaling is below 6×
because the views share memory bandwidth and cache capacity, but the read-only
selection phase removes serialization between them.

The main distinction is visible immediately: creating the quality TLAS is a
one-time cost, transform updates are relatively small, and object motion makes
selection rewalk only the affected instances. These are scale estimates rather
than platform promises; selection remains output-sensitive. See
[ARCHITECTURE.md](ARCHITECTURE.md) for specialized measurements and methodology.

## What selection returns

One traversal can produce three outputs:

- `CutEntry`: the hole-free actual cut to draw with current residency.
- `IdealEntry`: the cut within known topology if every payload were resident.
  `IdealTag::NeedsExpansion` is also a topology request.
- `LoadRequest`: payloads missing at the actual cut's refinement frontier.

The ideal cut and requests are optional. Pass `nullptr` for a static,
fully-resident world to avoid their emission cost. Output order is defined by
traversal and is not a priority order; stream the largest screen errors first.

Payloads are values, not identities. Duplicates are legal and the `World`
never indexes them. Operations use generation-stamped `AssetHandle`,
`PageHandle`, `NodeHandle`, and `World::InstanceRef` values. A page collected while an
asynchronous load is in flight simply makes the returned node handle stale;
mutations using it become safe no-ops.

## Runtime model

- Immutable, versioned page blobs hold preorder node arrays and 8-lane wide
  child blocks. A registered asset can be instanced thousands of times while
  sharing page bytes, residency, and its attachment graph.
- A dynamic wide TLAS owns instance placement, layer masks, coarse frustum and
  contribution culling, and incremental spawn/remove edits.
- Expansion points connect independently streamed pages. All-or-nothing child
  readiness keeps the actual cut free of holes and parent/child overlap.
- `setNodeBounds(instance, node, bounds)` creates bounds-only copy-on-write
  overlays for the affected instance. Refits are queued and published by
  `applyUpdates`.
- `SelectionContext` is optional, one per view. It owns that view's damping
  envelope and reuses instance cuts only while a conservative camera/projection
  margin proves their node set unchanged.
- Contextual `selectCut` is a read-only `World` operation. Calls may run
  concurrently when every call has a distinct `SelectionContext`, optional
  `PageUsageContext`, and output objects. Mutations and `collect` remain serial
  and must happen outside that selection phase.
- `PageUsageContext` is optional per view. It records page-use feedback without
  touching the World; `collect` later consumes only the contexts the caller
  chooses, so a primary camera can influence residency while shadow cameras do
  not.
- Stateless selection can fan out over visible instances through a host
  `parallelFor`; it remains an externally serial compatibility path.

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

Normal configuration options:

| Option | Default | Meaning |
|---|---:|---|
| `HLOD_BUILD_TESTS` | `ON` | Build the 91-test correctness suite |
| `HLOD_BUILD_BENCH` | `ON` | Build the Google Benchmark suite |
| `HLOD_AVX2` | `ON` | Use AVX2/FMA on x86-64 when available |
| `HLOD_FORCE_SCALAR` | `OFF` | Disable intrinsic implementations |

Additional internal benchmark toggles are documented where they are declared;
they are not supported integration settings.

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
