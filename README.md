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

    std::vector<CutEntry> cut;
    world.beginFrame();
    world.selectCut(view, CutParams{4.0f, 0.0f}, cut);

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

Representative current measurements on a 64-hardware-thread, 2.4 GHz EPYC,
using one thread, MSVC 19.51, Release `/O2 /arch:AVX2`, on 2026-08-05:

| Scenario | Current result |
|---|---:|
| 2.4M-node deep hierarchy, 259,933-entry cut | 1.72 ms |
| 20k static shared instances, stateless selection | 0.538 ms |
| Same 20k scene with `SelectionContext` (94.8% reused) | 0.152 ms |
| 80k shared instances, 5% moving, stateless / cached | 2.62 / 2.03 ms |
| TLAS steady selection, 200k / 500k instances | 5.48 / 6.14 ms |
| First quality TLAS build plus selection, 200k / 500k | 134 / 417 ms |
| Forced Morton rebuild plus cut, 100k / 500k instances | 7.88 / 44.5 ms |
| 4k instances of a 51 KiB asset, cloned / shared | 35.4 / 12.9 ms |
| Immutable page bytes in that cloned / shared case | 199 MiB / 51 KiB |

These are point estimates for scale, not platform promises. Selection is
output-sensitive, so cut size, visibility, residency, page layout, and memory
locality matter more than total authored node count. The latest controlled A/B
runs found that the retained Morton radix sort reduced forced rebuild-plus-cut
time by 36.2% at 100k instances and 42.0% at 500k. See
[ARCHITECTURE.md](ARCHITECTURE.md) for methodology, historical baselines, and
reverted experiments.

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
  overlays for the affected instance. Refits are queued and flushed lazily by
  the next selection.
- `SelectionContext` is optional, one per view. It owns that view's damping
  envelope and reuses instance cuts only while a conservative camera/projection
  margin proves their node set unchanged.
- Stateless selection can fan out over visible instances through a host
  `parallelFor`. Contextual selection is currently serial. All other world
  mutation is single-writer.

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
| `HLOD_BUILD_TESTS` | `ON` | Build the 88-test correctness suite |
| `HLOD_BUILD_BENCH` | `ON` | Build the Google Benchmark suite |
| `HLOD_AVX2` | `ON` | Use AVX2/FMA on x86-64 when available |
| `HLOD_FORCE_SCALAR` | `OFF` | Disable intrinsic implementations |

The remaining CMake options are A/B measurement scaffolds documented where
they are declared; they are not normal integration settings.

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
