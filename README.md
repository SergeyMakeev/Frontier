# Frontier

Frontier is a C++20 hierarchical-LOD selection library. It maintains a dynamic
8-wide top-level acceleration structure (TLAS), mounts reusable hierarchy
fragments, tracks payload residency, and returns both the renderable current cut
and the fully available ideal cut.

The data model is deliberately small:

- Every top-level instance is one permanent, renderable node stored directly in
  the TLAS.
- `SubtreeBuilder::build()` produces an aligned serialized byte array. There is
  no public semantic subtree object and no content key.
- `registerSubtree(SubtreeBytes&&)` consumes that array without copying it and
  returns its opaque definition handle.
- A definition can only be mounted beneath a renderable TLAS root or a
  `mountable` leaf in another mounted definition.
- Transforms belong to instances and mounts. Immutable payload, error, and
  authored bounds remain in registered bytes; runtime bound changes use
  copy-on-write overlays.

This makes a one-node object exceptionally cheap: it has no definition bytes or
mount state. Deep assemblies remain composable—a city definition can contain a
million mountable house nodes, all populated from the same house handle.

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
const auto leftHouse = cityBuilder.createNode(NodeDesc{
    .payload = 10,
    .geometricError = 16.0f,
    .flags = NodeDesc::FlagMountable,
    .bounds = leftHouseBoundsInCity,
});
const auto rightHouse = cityBuilder.createNode(NodeDesc{
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
const SubtreeInstanceHandle cityPlacement =
    world.mountSubtree(cityInstance.rootNode(), city);

NodeHandle leftHouseNode = /* retained from frontier/application state */;
NodeHandle rightHouseNode = /* retained from frontier/application state */;
world.mountSubtree(leftHouseNode, house,
                   Transform{leftHousePosition, 1.0f});
world.mountSubtree(rightHouseNode, house,
                   Transform{rightHousePosition, 1.0f});
```

Runtime `NodeHandle` values normally come from frontier results or retained
assembly state; builder `NodeId` values are authoring-local and are not runtime
handles.

`shared + currentOnly` is always a hole-free render frontier.
`shared + idealOnly` is the frontier the mounted topology would choose with
every payload resident. Applications decide which definition handle belongs at
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
`bytes()`, then move it into `registerSubtree()`. Registration validates and
takes over the existing allocation; there are no copy and borrowed registration
variants.

The public handles are intentionally distinct:

- `SubtreeHandle`: registered immutable bytes;
- `SubtreeInstanceHandle`: one mounted placement;
- `InstanceHandle`: one permanent TLAS root;
- `NodeHandle`: one live renderable node.

All are generation-stamped. Stale streaming completions are harmless: mutating
operations ignore stale node/instance handles, and mounting returns an invalid
placement when its parent disappeared. Invalid live topology, mounting below a
non-mountable node, duplicate children, invalid transforms, and bounds escapes
are contract errors routed through `FRONTIER_FATAL`.

## Query lifecycle

`SpatialDatabase` is single-writer. Apply mutations, call `applyUpdates()`, then
run any number of concurrent reads with distinct `SpatialQuery` objects. All
reads must finish before the next mutation or collection.

Each query owns damping, reuse records, scratch, output, statistics, and optional
mount-retention feedback. Enable the latter with
`query.setMountUsageEnabled(true)` and pass the query to `collect()` when its
camera should influence retention.

## Building

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

Important options are `FRONTIER_BUILD_TESTS`, `FRONTIER_BUILD_BENCH`,
`FRONTIER_AVX2`, and `FRONTIER_FORCE_SCALAR`.

See [API.md](docs/API.md) for the complete workflow,
[ARCHITECTURE.md](docs/ARCHITECTURE.md) for implementation details, and
[BENCHMARKING.md](docs/BENCHMARKING.md) for measurement guidance.
