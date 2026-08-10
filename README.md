# Frontier

Frontier is a C++20 hierarchical-LOD selection library. It maintains a dynamic
8-wide top-level acceleration structure (TLAS), streams reusable hierarchy
fragments, tracks payload residency, and returns both the renderable current cut
and the fully available ideal cut.

The data model is deliberately small:

- Every top-level instance is one permanent, renderable node stored directly in
  the TLAS.
- A `Subtree` is one immutable reusable descendant forest. Its parent is not
  stored in the blob.
- A subtree can only be mounted beneath a renderable TLAS root or an extendable
  node in another mounted subtree.
- `NodeDesc` describes both TLAS roots and authored subtree nodes.
- transforms belong to instances and mounts; immutable payload, error, and
  authored bounds remain in the reusable subtree blob; runtime bound changes
  use copy-on-write overlays.

This makes a one-node object exceptionally cheap: it has no subtree definition
or mount state. Deep assemblies remain composable—a city subtree can contain a
million extendable house nodes that all target the same house definition.

## Example

```cpp
#include <frontier/builder.h>
#include <frontier/spatial_database.h>

using namespace frontier;

constexpr SubtreeKey houseKey{0x1001};
constexpr SubtreeKey cityKey{0x2001};

SubtreeBuilder houseBuilder(houseKey);
houseBuilder.createNode(NodeDesc{
    .payload = 100,
    .geometricError = 0.0f,
    .bounds = houseBounds,
});
Subtree house = houseBuilder.build();

SubtreeBuilder cityBuilder(cityKey);
cityBuilder.createNode(NodeDesc{
    .payload = 10,
    .geometricError = 16.0f,
    .bounds = translatedHouseBounds,
    .childSubtree = houseKey,
    .childTransform = Transform{housePosition, 1.0f},
});
Subtree city = cityBuilder.build();
const AABB cityBounds = city.bounds();

SpatialDatabase world;
const SubtreeHandle houseDefinition =
    world.registerSubtree(std::move(house));
const SubtreeHandle cityDefinition =
    world.registerSubtree(std::move(city));

const InstanceHandle cityInstance = world.instantiate(NodeDesc{
    .payload = 1,
    .geometricError = 64.0f,
    .bounds = cityBounds,
    .childSubtree = cityKey,
});
world.mountSubtree(cityInstance.rootNode(), cityDefinition);

world.applyUpdates();
SpatialQuery query;
SelectionParams selection{.threshold = 1.0f};
FrontierResultView cut = query.selectFrontier(world, camera, selection);

// An application streamer resolves authored targets from ideal-side handles.
for (const FrontierEntry& entry : cut.idealOnly)
    if (world.subtreeTarget(entry.nodeHandle) == houseKey)
        world.mountSubtree(entry.nodeHandle, houseDefinition);
```

`shared + currentOnly` is always a hole-free render frontier.
`shared + idealOnly` is the frontier the known topology would choose with every
payload resident. A high-error ideal node with a valid `subtreeTarget()` is a
topology-streaming request. Other ideal nodes can drive payload IO through
`markPayloadResident()`.

## Ownership and handles

`Subtree` is move-only. `Subtree::fromBytes()` validates and copies a complete
serialized blob; `Subtree::borrow()` validates without copying; `clone()` makes
an explicit owned copy. A borrowed blob must outlive the `Subtree` or database
that owns it.

The public handles are intentionally distinct:

- `SubtreeHandle`: registered immutable definition;
- `SubtreeInstanceHandle`: one mounted placement;
- `InstanceHandle`: one permanent TLAS root;
- `NodeHandle`: one live renderable node.

All are generation-stamped. Stale streaming completions are harmless: mutating
operations ignore stale node/instance handles, and mounting returns an invalid
placement when its parent disappeared. Contract errors—wrong target key,
mounting below a non-extendable node, or escaping authored bounds—fail through
`FRONTIER_FATAL`.

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
