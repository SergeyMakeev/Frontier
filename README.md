# Frontier

Frontier is a C++20 library that chooses which level-of-detail (LOD) nodes a
renderer should draw from a large dynamic scene. It indexes independently
movable objects, mounts reusable local hierarchies below them, and accounts for
render resources that are still streaming.

A bounding-volume hierarchy (BVH) groups spatial bounds so many objects can be
rejected at once. Frontier's top-level acceleration structure (TLAS) is a
build-configured 4- or 8-wide world-space BVH; each TLAS leaf is also a
permanent renderable fallback.
A registered subtree definition is the closest equivalent to a bottom-level
acceleration structure (BLAS): it stores the reusable local hierarchy, while a
mount supplies a placement below a renderable node. Frontier does not expose a
`BLAS` type because definitions can be mounted recursively, and a one-node
instance needs no lower hierarchy. A cut, or frontier, is the ancestor-free set
of nodes selected to cover the visible scene. The current cut is renderable now
and hole-free: it never replaces a parent until ready descendants cover every
visible branch represented by that parent. The ideal cut assumes every node in
currently mounted topology is ready. Here, hole-free describes logical
hierarchy coverage; it does not guarantee crack-free mesh boundaries.

Readiness need not form an unbroken path from root to leaf. If an unavailable
node has a complete ready descendant cut, Frontier renders those descendants
directly; it falls back to a ready ancestor only when descendant coverage is
incomplete. This detailed behavior is the default; set
`SelectionParams::currentCutPolicy` to
`CurrentCutPolicy::PreferReadyAncestors` when a caller instead wants the
smaller, coarser parent-only fallback cut. The illustrated comparison is in
the [API guide](docs/API.md#two-current-cut-policies).

The data model is deliberately small:

- Every top-level instance is one permanent, renderable node stored directly in
  the TLAS.
- `SubtreeBuilder::build()` produces an aligned serialized byte array. There is
  no public semantic subtree object and no content key.
- `registerSubtree(SubtreeBytes&&)` consumes that array without copying it and
  returns its opaque definition handle.
- A definition can only be mounted beneath a renderable TLAS root or a
  `mountable` leaf in another mounted definition.
- Top-level instances support translation, positive uniform scale, and planar
  yaw; mounted-subtree placements support translation and uniform scale.
  Immutable payload, error, and authored bounds remain in registered bytes;
  runtime bound changes use copy-on-write overlays.

This makes a one-node object exceptionally cheap: it has no definition bytes or
mount state. Deep assemblies remain composable—a city definition can contain a
million mountable house nodes, all populated from the same house handle.

`UserPayload` defaults to `uint64_t`. Applications may instead define
`FRONTIER_USER_PAYLOAD` and `FRONTIER_INVALID_PAYLOAD` build-wide; for example,
`uint32_t` and `UINT32_MAX`, or `void*` and `nullptr`. Four-byte payloads halve
serialized and TLAS-root payload storage. The invalid value is reserved so
`tryGetPayload()` can return a payload directly and report a stale handle
without an out-parameter or `std::optional`. See the
[API reference](docs/API_REFERENCE.md#3-node-authoring-types) for the exact type
and serialization contract.

## Example

```cpp
#include <frontier/builder.h>
#include <frontier/spatial_database.h>

using namespace frontier;

SubtreeBuilder houseBuilder;
houseBuilder.createNode(NodeDesc{
    .payload = 100,
    .geometricError = 0.0f,
    .bounds = houseLocalBounds,
});

SpatialDatabase world;
const SubtreeHandle house =
    world.registerSubtree(houseBuilder.build());

SubtreeBuilder cityBuilder;
cityBuilder.createNode(NodeDesc{
    .payload = 10,
    .geometricError = 16.0f,
    .flags = NodeDesc::FlagMountable,
    .bounds = leftHouseBoundsInCity,
});
cityBuilder.createNode(NodeDesc{
    .payload = 11,
    .geometricError = 16.0f,
    .flags = NodeDesc::FlagMountable,
    .bounds = rightHouseBoundsInCity,
});
const SubtreeHandle city =
    world.registerSubtree(cityBuilder.build());

const InstanceHandle cityInstance = world.instantiate(NodeDesc{
    .payload = 1,
    .geometricError = 64.0f,
    .flags = NodeDesc::FlagMountable,
    .bounds = cityBounds,
});
world.mountSubtree(cityInstance.rootNode(), city);
world.applyUpdates();

SpatialQuery query;
const Camera camera = currentCamera(); // application function
FrontierResultView cut = query.selectFrontier(world, camera,
                                               SelectionParams{});

// Mountable runtime nodes are discovered through the ideal frontier. The
// application payload maps each proxy to its child definition and placement.
for (const FrontierEntry& entry : cut.ideal()) {
    if (!entry.overThreshold() ||
        world.hasMountedSubtree(entry.nodeHandle))
        continue;

    if (UserPayload payload = world.tryGetPayload(entry.nodeHandle);
        payload != kInvalidPayload) {
        if (payload == 10)
            world.mountSubtree(entry.nodeHandle, house,
                               Transform{leftHousePosition, 1.0f});
        else if (payload == 11)
            world.mountSubtree(entry.nodeHandle, house,
                               Transform{rightHousePosition, 1.0f});
    }
}
world.applyUpdates(); // publish the newly mounted houses
```

Builder `NodeId` values are authoring-local and are never converted to runtime
handles. Nested mount points are deliberately discovered as `NodeHandle`
values in frontier results, then retained while asynchronous loading runs.

`cut.current()` is always a complete render frontier; it iterates `shared`
followed by `currentOnly` without copying either bucket. `cut.ideal()` similarly
iterates `shared` followed by `idealOnly`, the frontier the mounted topology
would choose with every known node ready. Readiness means the renderer has every GPU resource
needed to dispatch a node's payload. It belongs to a node in a registered
definition and is shared by that node across every placement of the definition.
Equal payload values in different nodes are independent; applications that use
them for the same GPU resource may publish readiness to each corresponding
node. Applications decide which definition handle belongs at
each mountable node; Frontier deliberately stores no content lookup key.

## Bytes, ownership, and handles

`SubtreeBytes` is an owning, 64-byte-aligned byte array. The bytes emitted by
the builder are the traversal representation and serialization format. They can
be written directly to disk, or passed directly to registration. A named array
requires an explicit move:

```cpp
SubtreeBytes bytes = builder.build();
save(bytes.bytes());
SubtreeHandle definition = world.registerSubtree(std::move(bytes));
```

To load saved data, allocate `SubtreeBytes(fileSize, context)`, read into
`bytes()`, then move it into `registerSubtree()`. By default, registration
validates the complete structure in linear time and takes over the existing
allocation without unpacking or copying its node arrays; there are no copy and
borrowed registration variants. Trusted-content builds can configure
`FRONTIER_VALIDATE_SUBTREES=OFF` to retain only constant-time format-envelope
checks and remove the structural scan. The performance runners additionally set
`FRONTIER_CONTRACT_CHECKS=OFF`, which assumes even that envelope is valid and
must only be used with trusted benchmark inputs.

The public handles are intentionally distinct:

- `SubtreeHandle`: registered immutable bytes;
- `SubtreeInstanceHandle`: one mounted placement;
- `InstanceHandle`: one permanent TLAS root;
- `NodeHandle`: one live renderable node.

All are generation-stamped. Stale topology and readiness completions are
harmless: mutating operations ignore stale node/instance handles, and mounting
returns an invalid placement when its parent disappeared. A readiness completion
uses the `NodeHandle` that requested the payload; if its placement disappeared,
the application can publish a later completion from any live placement of the
same registered definition node. Invalid live topology, mounting below a
non-mountable node, duplicate children, invalid transforms, and bounds escapes
are contract errors routed through `FRONTIER_FATAL`.

## Query lifecycle

`SpatialDatabase` is single-writer. Apply mutations, call `applyUpdates()`, then
run any number of concurrent reads with distinct `SpatialQuery` objects. All
reads must finish before the next mutation or collection.

Each query owns damping, reuse records, scratch, output, optional instrumented
statistics, and optional mount-retention feedback. Enable the latter with
`query.setMountUsageEnabled(true)` and pass the query to `collect()` when its
camera should influence retention.

## Measured release performance

The current format-v3 release snapshot measures an eight-byte-payload,
fully hierarchical 10,000-instance scene that emits 20,000 frontier entries:

| Platform | Stable cached selection | 16-unit camera step | Move 10% + publish + select |
|---|---:|---:|---:|
| Apple M2 Max | 108 us | 112 us | 171 us |
| RK3399 | 524 us | 575 us | 1,084 us |
| Intel i9-12900K | 125 us | 137 us | 263 us |
| AMD EPYC 9654 | 112 us | 123 us | 245 us |

The 16-unit step retains about 99.4% root reuse. Moving and invalidating all
10,000 roots costs 0.913-9.243 ms across these targets. In a separate
400-house scene, reusable assembly reduces construction latency by 55-81% and
retained memory by 63-64%. Payload32 saves memory but is not consistently
faster. These medians describe specific Release workloads, not latency
guarantees; see the
[cross-platform performance snapshot](docs/PERFORMANCE_RESULTS_2026_08_15.md)
for raw traversal, forced misses, stage attribution, payload comparison, and
measurement conditions.

## Building

```sh
bash ./run_unit_tests.sh  # Debug, checks enabled, BVH4 + BVH8
bash ./run_perf_bench.sh  # Release, native BVH width, 4/8-byte payload comparison
```

For a maximum-throughput native ARM64 deployment, GCC profile-guided
optimization can train on the realistic moving-camera/moving-actor city
trajectory and rebuild the public archive under LTO:

```sh
FRONTIER_PGO_CPU=4 FRONTIER_PGO_JOBS=4 bash ./run_arm_pgo.sh build-arm-pgo
```

The script creates a fresh profile corpus on every invocation and trains the
public library plus both benchmark payload layouts. The equivalent manual
CMake phases are `FRONTIER_PGO_MODE=GENERATE` followed by `USE`, with both
pointing at the same `FRONTIER_PGO_DIR`. PGO currently requires GCC.

On Windows, use `run_unit_tests.bat` and `run_perf_bench.bat`.

The full correctness matrix, deterministic torture tests, sanitizer jobs, and
release verification commands are described in [docs/TESTING.md](docs/TESTING.md).

Important options are `FRONTIER_BUILD_TESTS`, `FRONTIER_BUILD_BENCH`,
`FRONTIER_BVH_WIDTH`, `FRONTIER_AVX2`, `FRONTIER_FORCE_SCALAR`,
`FRONTIER_IPO`, `FRONTIER_PGO_MODE`, `FRONTIER_PGO_DIR`, `FRONTIER_STATS`,
`FRONTIER_CONTRACT_CHECKS`, and
`FRONTIER_VALIDATE_SUBTREES`.

The CMake setting `FRONTIER_BVH_WIDTH` accepts `AUTO`, `4`, or `8` and defaults
to `AUTO`. CMake resolves `AUTO` to BVH8 with AVX2's eight-lane backend and
BVH4 with four-lane SSE2/NEON. Forced-scalar builds also default to the compact
BVH4 layout. An explicit `4` or `8` always overrides this policy. The resulting
preprocessor macro is numeric (`4` or `8`), and branch width is a build-wide
layout choice, including serialized subtrees.

On x86-64, `FRONTIER_AVX2=ON` with BVH8 produces an AVX2/FMA-targeted binary
without runtime dispatch. Set it to `OFF` when the executable must run on the
SSE2 baseline or when the host performs its own per-ISA library dispatch. BVH4
uses the 128-bit backend and does not require AVX2.

Tests default on only for a standalone Frontier checkout; benchmarks default
off because they require a separate optimized build. When the project is
included with `add_subdirectory()`, both default off and the
`frontier` target propagates its C++20 requirement to consumers.

See the progressive [API guide](docs/API.md) for the integration flow, the
exhaustive [API reference](docs/API_REFERENCE.md) for exact contracts,
[ARCHITECTURE.md](docs/ARCHITECTURE.md) for implementation details, and
[BENCHMARKING.md](docs/BENCHMARKING.md) for measurement guidance. The latest
optimization campaign, including rejected experiments and raw-result names,
is recorded in
[PERFORMANCE_OPTIMIZATION_2026.md](docs/PERFORMANCE_OPTIMIZATION_2026.md).
The current release candidate's M2 Max, RK3399, i9-12900K, and EPYC 9654
results are summarized separately in the
[cross-platform performance snapshot](docs/PERFORMANCE_RESULTS_2026_08_15.md).
