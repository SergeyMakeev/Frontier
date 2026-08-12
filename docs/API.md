# Frontier API Guide

Frontier is a C++20 library that selects a renderable hierarchical-LOD cut from
a large dynamic scene. It owns the spatial index, reusable hierarchy topology,
shared definition-node readiness, and the rules that keep the current cut
complete while content streams. The application still owns rendering, resource loading,
payload interpretation, and the policy that decides which reusable component
belongs under an expandable node.

This guide introduces the system in the order an integration normally uses it.
The exhaustive type and function contracts are in
[API_REFERENCE.md](API_REFERENCE.md).

The normal entry points are:

```cpp
#include <frontier/builder.h>
#include <frontier/spatial_database.h>

using namespace frontier;
```

## 1. What Frontier provides

A conventional LOD system can choose one mesh for one object. Frontier makes
the same decision across an assembled hierarchy: a city can refine into blocks,
a block into reusable buildings, and a building into reusable detail trees.
Definitions are shared, placements are independent, and missing content falls
back to the nearest ready ancestor without leaving holes.

The smallest useful scene needs only a permanent top-level node:

```cpp
SpatialDatabase database;

InstanceHandle rock = database.instantiate(
    NodeDesc{
        .payload = 1001,              // application resource/data id
        .geometricError = 0.0f,       // no finer representation
        .bounds = rockLocalBounds,
    },
    InstanceDesc{
        .pos = float4::point(40.0f, 0.0f, -12.0f),
        .scale = 1.0f,
        .mask = ~0u,
    });

database.applyUpdates();

SpatialQuery query;
Camera camera = makeLookAtCamera(cameraPosition, cameraTarget);
FrontierResultView cut = query.selectFrontier(
    database, camera, SelectionParams{.threshold = 4.0f});

for (const FrontierEntry& entry : cut.shared)
    submitToRenderer(entry);
for (const FrontierEntry& entry : cut.currentOnly)
    submitToRenderer(entry);
```

The root lives directly in Frontier's top-level acceleration structure (TLAS).
A one-node object allocates no subtree definition and no mounted-placement
state.

## 2. The architecture in one pass

Frontier separates immutable authored data from mutable placement data:

| Concept | What it represents | Ownership and mutability |
|---|---|---|
| node | one renderable LOD choice | payload, error, flags, and authored bounds |
| top-level instance | one permanent renderable root in the TLAS | mutable translation, uniform scale, mask, and world bound |
| subtree definition | one reusable descendant hierarchy | immutable registered serialized bytes |
| mounted placement | one definition attached below one renderable node | mutable coverage, mount links, and accumulated transform |
| spatial query | one view's damping, reuse cache, scratch, and output | mutable and owned by the calling view |

An expandable node is called a **mountable node** by the API. It is both a real
renderable fallback and a legal attachment point. It must be a leaf within its
own definition; a different registered definition may be mounted below it at
runtime.

```cpp
SubtreeHandle houseDefinition = /* registered once */;

NodeHandle firstHouseProxy  = /* discovered in an ideal frontier */;
NodeHandle secondHouseProxy = /* another placement's proxy */;

SubtreeInstanceHandle firstHouse =
    database.mountSubtree(firstHouseProxy, houseDefinition,
                          firstHouseLocalTransform);
SubtreeInstanceHandle secondHouse =
    database.mountSubtree(secondHouseProxy, houseDefinition,
                          secondHouseLocalTransform);
```

Both placements share the house's immutable topology, bounds, errors, payload
values, and definition-node readiness. Each placement has its own mounted
descendants and derived coverage state.

Four rules explain most of the architecture:

1. Every top-level instance has exactly one permanent renderable root.
2. A subtree definition contains descendants, not a replacement for that root.
3. Definitions are immutable and shareable; placements are mutable and unique.
4. Topology availability and render readiness are independent.

## 3. Describe one renderable node

`NodeDesc` is used both for TLAS roots and for nodes authored by a
`SubtreeBuilder`:

```cpp
NodeDesc proxy{
    .payload = buildingProxyPayload,
    .geometricError = 32.0f,
    .flags = NodeDesc::FlagMountable,
    .bounds = buildingBounds,
};
```

- `payload` is an opaque application render-resource identifier. Equal values
  may identify the same resource, but do not couple library readiness state.
- `geometricError` is expressed in the node hierarchy's local units.
- `FlagMountable` makes the node an expandable assembly boundary.
- `bounds` is exact six-float authoring storage and accepts an `AABB` directly.

The descriptor occupies 40 bytes. The 32-bit flag word currently defines only
`FlagMountable`; the remaining bits are reserved for future node properties.

```cpp
if (proxy.isMountable()) {
    AABB exactBounds = proxy.bounds;  // exact conversion, no quantization
    scheduleDefinitionLookup(proxy.payload, exactBounds);
}
```

## 4. Author a reusable subtree

`SubtreeBuilder` creates a hierarchy in edit mode. A builder node id exists only
while authoring and is unrelated to a runtime `NodeHandle`.

```cpp
SubtreeBuilder buildingBuilder;
buildingBuilder.reserve(3); // optional allocation hint

SubtreeBuilder::NodeId coarse = buildingBuilder.createNode(NodeDesc{
    .payload = buildingCoarsePayload,
    .geometricError = 24.0f,
    .bounds = buildingBounds,
});

buildingBuilder.createNode(coarse, NodeDesc{
    .payload = buildingLeftPayload,
    .geometricError = 0.0f,
    .bounds = buildingLeftBounds,
});

buildingBuilder.createNode(coarse, NodeDesc{
    .payload = buildingRightPayload,
    .geometricError = 0.0f,
    .bounds = buildingRightBounds,
});

SubtreeBytes buildingBytes = buildingBuilder.build();
```

`createNode(desc)` creates a direct child of the node on which the eventual
definition is mounted. `createNode(parent, desc)` creates a local child of an
earlier builder node. A definition may therefore have several direct nodes; the
serialized representation uses an internal implicit-parent sentinel to keep
those roots contiguous. The sentinel is never renderable and has no public
handle.

`build()` consumes the builder and verifies the authored hierarchy:

- at least one renderable node exists;
- error values are finite and non-negative;
- every final node bound is non-empty;
- no local child exists below a mountable node;
- local fanout is at most 511;
- child error is clamped monotonically to its parent's error.

An interior node may start with empty bounds if its children establish a
non-empty result:

```cpp
SubtreeBuilder generated;
auto parent = generated.createNode(NodeDesc{
    .payload = generatedProxy,
    .geometricError = 64.0f,
    .bounds = AABB::empty(),
});
for (const GeneratedPart& part : parts)
    generated.createNode(parent, part.nodeDesc());

SubtreeBytes bytes = generated.build(); // parent bounds include its children
```

## 5. Serialized subtrees and memory ownership

`SubtreeBytes` is an owning, 64-byte-aligned byte array. The builder's output is
simultaneously:

- the serialized file representation;
- the registration input;
- the immutable in-memory traversal representation.

Registration validates and moves the existing allocation; it does not unpack
the definition into a second semantic object. It does build a compact zeroed
readiness bitset, so registration remains linear in node count for validation
even though ownership transfer of the byte array is constant-time.

```cpp
SubtreeBytes bytes = buildBuildingDefinition().build(context);
writeAll("building.frontier", bytes.bytes()); // application file function

SubtreeHandle building = database.registerSubtree(std::move(bytes));
```

A temporary can be registered directly:

```cpp
SubtreeHandle building =
    database.registerSubtree(buildBuildingDefinition().build(context));
```

Loading allocates the final array before reading:

```cpp
SubtreeBytes bytes(fileSize("building.frontier"), context);
readAll("building.frontier", bytes.bytes()); // application file function

SubtreeHandle building = database.registerSubtree(std::move(bytes));
```

Persisted bytes are a versioned native traversal format, not a long-term
interchange schema. Registration rejects incompatible format versions, layout,
size, alignment, or byte order; rebuild authored assets when those change. The
validation is a compatibility check, not a hardened parser for untrusted
input, so load bytes produced by a trusted authoring pipeline.

Important ownership rules:

- a named `SubtreeBytes` requires `std::move` at registration;
- registering identical bytes twice creates two independent definitions;
- `releaseSubtree()` requires that no mounted placements still reference the
  definition;
- the state referenced by `FrontierContext::user` must outlive every byte array
  and registered definition allocated with that context.

`SubtreeBytes` is explicitly copyable for tools that need duplicate byte
arrays, but registration exposes only the ownership-taking rvalue overload.

## 6. Create a root and mount a definition

Every independently movable object begins with `instantiate()`. Set
`FlagMountable` when a deeper definition will attach below the root.

```cpp
InstanceHandle city = database.instantiate(
    NodeDesc{
        .payload = cityFallbackPayload,
        .geometricError = 128.0f,
        .flags = NodeDesc::FlagMountable,
        .bounds = cityLocalBounds,
    },
    InstanceDesc{
        .pos = cityWorldPosition,
        .scale = 1.0f,
        .mask = cityLayerMask,
    });

SubtreeInstanceHandle cityPlacement = database.mountSubtree(
    city.rootNode(), cityDefinition,
    Transform{.pos = float4::point(0, 0, 0), .scale = 1.0f});
```

A mount parent can be either a mountable TLAS root or a mountable leaf in an
existing placement. Mounting verifies that the transformed definition bounds
fit inside the parent and applies the parent's effective error as a ceiling.
The definition bytes are never rewritten.

```cpp
if (!database.hasMountedSubtree(houseProxy)) {
    SubtreeInstanceHandle house = database.mountSubtree(
        houseProxy, houseDefinition,
        Transform{.pos = houseOffsetInBlock, .scale = houseScale});

    if (!house.valid())
        handleExpectedStreamingRace(); // proxy became stale while loading
}
```

An invalid result caused by a stale parent is an expected asynchronous race.
Mounting on a live non-mountable parent, mounting twice, escaping the parent
bounds, or using an invalid definition is a contract violation.

Mount transforms are translation plus positive uniform scale and accumulate
across nested placements.

## 7. Select the current and ideal frontiers

Mutations become queryable at an update barrier:

```cpp
database.applyUpdates();

SpatialQuery mainView;
mainView.setHalfLife(3.0f); // optional view-local LOD damping

FrontierResultView cut = mainView.selectFrontier(
    database,
    cameraFromViewProjection(viewProjection, cameraPosition,
                             viewportHeight, projectionYScale),
    SelectionParams{
        .threshold = 4.0f,
        .minPix = 0.0f,
    });
```

One traversal returns two logical cuts in three disjoint buckets:

- `shared` belongs to both current and ideal cuts;
- `currentOnly` contains ready fallbacks used only now;
- `idealOnly` contains choices wanted if all known definition nodes were ready.

Render `shared + currentOnly`:

```cpp
auto render = [&](std::span<const FrontierEntry> entries) {
    for (const FrontierEntry& entry : entries) {
        UserPayload payload;
        if (database.tryGetPayload(entry.nodeHandle, payload))
            submitPayload(payload, entry.instance());
    }
};

render(cut.shared);
render(cut.currentOnly);
```

Use `shared + idealOnly` to drive demand:

```cpp
auto request = [&](std::span<const FrontierEntry> entries) {
    for (const FrontierEntry& entry : entries) {
        UserPayload payload;
        if (database.tryGetPayload(entry.nodeHandle, payload) &&
            !database.isNodeReady(entry.nodeHandle))
            requestPayload(entry.nodeHandle, payload);

        if (entry.overThreshold() &&
            !database.hasMountedSubtree(entry.nodeHandle))
            requestChildDefinition(entry.nodeHandle);
    }
};

request(cut.shared);
request(cut.idealOnly);
```

`FrontierResultView` points into its `SpatialQuery` and remains valid until that
query's next selection, `reset()`, or destruction. Use `FrontierResult` for an
owning copy or `FrontierResultSink` to write directly into fixed caller memory.

Each `FrontierEntry` carries a generation-stamped `NodeHandle`, an
`InstanceId` stable for the lifetime of its top-level instance, and a
threshold-relative error code. The instance id is appropriate for indexing the
application's top-level transform or entity table while that instance is live.

## 8. Stream topology and render readiness independently

Mounting makes finer topology known. Marking a node ready says the renderer has
every GPU resource needed to dispatch that node's payload. These operations
intentionally happen at different times.

Readiness belongs to one node in one registered definition. A `NodeHandle` from
any live placement identifies that definition node, and the change applies to
every current and future placement of the same definition:

```cpp
void nodeUploadCompleted(NodeHandle node)
{
    database.markNodeReady(node);
}

void makeNodeUnavailable(NodeHandle node)
{
    database.markNodeUnavailable(node);
}
```

Equal `UserPayload` values in different nodes or definitions remain independent.
If those values refer to one GPU allocation, the integration may mark each
corresponding definition node together. Frontier deliberately does not build a
payload index or impose resource-identity policy.

Once published, readiness remains on the registered definition even if all of
its placements are temporarily unmounted. A later mount inherits it. Releasing
the definition discards it. Publishing requires a live mounted `NodeHandle`;
stale handles are ignored and `isNodeReady()` returns `false` for them.

Each placement stores only derived coverage and covered-child counts. Coverage
propagates toward the root so the current cut remains complete while
intermediate nodes are unavailable.

TLAS roots are always ready because they are the permanent fallback:

```cpp
bool rootReady = database.isNodeReady(instance.rootNode()); // true
database.markNodeReady(instance.rootNode());                 // no-op
```

Calling `markNodeUnavailable()` on a live TLAS root is a contract violation.

An asynchronous topology completion normally retains only the parent handle:

```cpp
void definitionLoadCompleted(NodeHandle parent, SubtreeBytes bytes,
                             Transform placement)
{
    SubtreeHandle definition = database.registerSubtree(std::move(bytes));
    SubtreeInstanceHandle mounted =
        database.mountSubtree(parent, definition, placement);

    if (!mounted.valid() && database.isSubtree(definition))
        database.releaseSubtree(definition); // parent disappeared before mount
}
```

Applications commonly cache definitions separately instead of releasing them
after one stale request.

## 9. Move roots, placements, and individual nodes

Frontier distinguishes three kinds of movement.

### Move an entire top-level object

`moveInstance()` changes the translation and uniform scale of the permanent
root and everything mounted below it:

```cpp
database.moveInstance(carInstance, Transform{
    .pos = newCarPosition,
    .scale = 1.0f,
});
```

For a stable cohort, `MotionGroup` preserves caller order while caching the
database's physical order:

```cpp
SpatialDatabase::MotionGroup trafficGroup(trafficInstances);

// positions[i] corresponds to trafficInstances[i].
database.moveInstances(trafficGroup, positions, 1.0f);
```

### Place a mounted definition

The `Transform` passed to `mountSubtree()` is fixed for that placement and is
accumulated into top-level instance-local coordinates. The current API does not
mutate a mount transform in place. Replace a rigid placement by unmounting and
mounting it again:

```cpp
database.unmountSubtree(oldPlacement);
SubtreeInstanceHandle replacement =
    database.mountSubtree(parentNode, definition, newPlacementTransform);
```

That replacement starts with fresh coverage and inherits the registered
definition nodes' current readiness.

### Change one node's effective bound

Node animation and deformation remain application-owned. Submit the resulting
local bound for the affected top-level instance:

```cpp
database.setNodeBounds(carInstance, movingDoorNode,
                       animatedDoorBoundsInDefinitionSpace);

// Optional immediate tool/readback barrier; applyUpdates() also flushes.
database.flushBounds();
AABB effective = database.nodeBounds(carInstance, movingDoorNode);
```

The first edit to an `(instance, mounted placement)` pair creates a private
copy-on-write bounds overlay. Topology, payloads, errors, readiness, and every
other placement remain shared. Ancestor propagation is conservative and
grow-only.

`setNodeBounds()` does not create a render transform. Use
`tryGetNodeTransform()` to obtain the containing mount's accumulated transform,
then compose it with the application-owned node pose and top-level transform:

```cpp
Transform mountToInstance;
if (database.tryGetNodeTransform(movingDoorNode, mountToInstance))
    updateRenderTransform(movingDoorNode, mountToInstance, animatedDoorPose);
```

## 10. Reclaim cold mounted placements

Definitions and TLAS roots are explicit-lifetime objects. Mounted placements
can additionally be collected by an LRU policy. A query contributes retention
feedback only when enabled:

```cpp
SpatialQuery streamingView;
streamingView.setMountUsageEnabled(true);

database.applyUpdates();
streamingView.selectFrontier(database, camera, params);

CollectResult collected = database.collect(
    streamingView,
    maxMountedSubtrees,
    minimumUnusedEpochs);
```

Collection removes eligible leaf placements from the LRU tail until the mount
budget is met. A placement must be old enough and have no mounted children.
Collection changes topology only. It never changes or reports definition-node
readiness; a later placement of the same registered definition inherits the
retained state. GPU resource eviction remains an application-level decision.

Several views can contribute usage before one collection pass:

```cpp
std::array<SpatialQuery*, 2> retentionViews{&mainView, &shadowView};
CollectResult result = database.collect(retentionViews, mountBudget, minAge);
```

Use `subtreeInstanceStateBytes()`, `overlayCount()`, and `overlayBytes()` to
track placement-local mutable hierarchy cost. The first metric excludes
registered bytes and definition-local readiness bits.

## 11. Update and threading model

`SpatialDatabase` uses a publish/read discipline:

1. one writer performs registration, assembly, motion, readiness changes, and
   collection;
2. the writer calls `applyUpdates()`;
3. any number of readers select concurrently, each with a distinct
   `SpatialQuery`;
4. all reads finish before the next write.

```cpp
// Single-writer phase.
applyStreamingCompletions(database);
applySimulationMotion(database);
database.applyUpdates();

// Read-only phase. Each task owns a different query.
runConcurrently(
    [&] { mainCut = mainQuery.selectFrontier(database, mainCamera, params); },
    [&] { shadowCut = shadowQuery.selectFrontier(database, shadowCamera, params); });

// Join before mutating again.
consumeCuts(mainCut, shadowCut);
database.collect(mainQuery, mountBudget, minAge);
```

A `SpatialQuery` is mutable and cannot be used concurrently, even against the
same database. It binds to the first database it reads; `reset()` releases that
binding and clears damping/reuse history while retaining allocations.

Call `applyUpdates()` once per publication group even if no content changed; it
also advances the epoch used by collection aging. `optimize()` is a heavier
safe-point operation that flushes bounds, compacts dead dense instance slots,
and performs a quality TLAS rebuild. Public handles and `FrontierEntry` instance
ids remain stable.

## 12. Configure allocation, TLAS quality, and parallel selection

`FrontierContext` supplies aligned allocation for serialized subtrees and an
optional blocking `parallelFor` callback. `SpatialDatabaseConfig` selects TLAS
quality and maintenance thresholds.

```cpp
FrontierContext context{
    .alloc = &engineAlignedAlloc,
    .free = &engineAlignedFree,
    .parallelFor = &engineParallelFor,
    .workerCount = workerCount,
    .user = allocatorAndSchedulerState,
};

SpatialDatabaseConfig config{
    .context = context,
    .tlasQuality = TlasQuality::BinnedSAH,
    .tlasTraversalCost = 1.0f,
    .tlasIntersectCost = 1.0f,
    .tlasCountDrift = 0.2f,
    .tlasAreaDrift = 0.5f,
    .tlasEscapeFraction = 0.25f,
    .tlasEditFraction = 0.05f,
    .parallelInstanceThreshold = 4096,
};

SpatialDatabase database(config);
```

`parallelFor` must block until all requested tasks finish. Internal parallel
selection is used only for uncached queries, above the configured visible
instance threshold, and when `workerCount > 1`:

```cpp
SpatialQuery uncached;
uncached.setReuseEnabled(false);
FrontierResultView cut = uncached.selectFrontier(database, camera, params);
```

Contract violations route through `FRONTIER_FATAL`, which throws
`std::logic_error` by default. Exception-free hosts can define it before any
Frontier include.

## 13. End-to-end example: reusable houses in a streamed city

This example builds one reusable house definition and a city definition whose
house proxies are mountable. The application decides that each proxy maps to
the same house handle.

```cpp
// Build and register the reusable house.
SubtreeBuilder houseBuilder;
auto houseCoarse = houseBuilder.createNode(NodeDesc{
    .payload = houseCoarsePayload,
    .geometricError = 8.0f,
    .bounds = houseBounds,
});
houseBuilder.createNode(houseCoarse, NodeDesc{
    .payload = houseFinePayload,
    .geometricError = 0.0f,
    .bounds = houseBounds,
});
SubtreeHandle houseDefinition =
    database.registerSubtree(houseBuilder.build());

// Build the city from renderable district/block LODs and lightweight
// mountable house proxies. Each authored fanout stays at or below 511.
SubtreeBuilder cityBuilder;
for (const District& district : authoredCity.districts) {
    auto districtNode = cityBuilder.createNode(NodeDesc{
        .payload = district.coarsePayload,
        .geometricError = 96.0f,
        .bounds = district.boundsInCity,
    });

    for (const Block& block : district.blocks) {
        auto blockNode = cityBuilder.createNode(districtNode, NodeDesc{
            .payload = block.coarsePayload,
            .geometricError = 48.0f,
            .bounds = block.boundsInCity,
        });

        for (const HousePlacement& house : block.houses) {
            cityBuilder.createNode(blockNode, NodeDesc{
                // In this example the payload indexes the house's authored
                // placement metadata as well as its proxy render data.
                .payload = house.proxyPayload,
                .geometricError = 32.0f,
                .flags = NodeDesc::FlagMountable,
                .bounds = house.conservativeBoundsInCity,
            });
        }
    }
}
SubtreeHandle cityDefinition =
    database.registerSubtree(cityBuilder.build());

// One permanent root owns the assembled runtime tree.
InstanceHandle city = database.instantiate(
    NodeDesc{
        .payload = cityFallbackPayload,
        .geometricError = 128.0f,
        .flags = NodeDesc::FlagMountable,
        .bounds = authoredCity.bounds,
    },
    InstanceDesc{.pos = cityWorldPosition});

database.mountSubtree(city.rootNode(), cityDefinition);
```

The streaming loop discovers mountable proxies from the ideal frontier:

```cpp
void processIdeal(std::span<const FrontierEntry> entries)
{
    for (const FrontierEntry& entry : entries) {
        UserPayload payload;
        if (!database.tryGetPayload(entry.nodeHandle, payload))
            continue;

        if (!database.isNodeReady(entry.nodeHandle))
            payloadStreamer.request(entry.nodeHandle, payload);

        if (entry.overThreshold() &&
            !database.hasMountedSubtree(entry.nodeHandle) &&
            applicationSaysHouseProxy(payload)) {
            // The application authored the proxy-to-house placement together
            // with the proxy payload. Different proxies reuse the same bytes
            // with different local transforms.
            const Transform placement =
                authoredCity.housePlacementFor(payload);
            database.mountSubtree(entry.nodeHandle, houseDefinition,
                                  placement);
        }
    }
}

database.applyUpdates();
FrontierResultView cut = cityQuery.selectFrontier(database, camera, params);
processIdeal(cut.shared);
processIdeal(cut.idealOnly);
```

The city can contain a million proxies without duplicating the house
definition. Only proxies that actually receive a mounted house allocate
per-placement coverage and mount state; each house definition node's readiness
is shared across all house placements.

## 14. End-to-end example: asynchronous payload and topology streaming

Keep generation-stamped handles in asynchronous topology and readiness
requests. A stale readiness completion is safely ignored. If the underlying GPU
resource is shared by several definition nodes, the integration can track those
nodes by payload and publish the completion to each live representative. Queue
completions and apply them during the next single-writer phase.

```cpp
struct PayloadRequest {
    NodeHandle node;
    UserPayload payload;
};

struct DefinitionRequest {
    NodeHandle parent;
    AssetId asset;
    Transform placement;
};

void applyCompletions()
{
    for (PayloadRequest& done : payloadStreamer.completed()) {
        uploadToGpu(done.payload);
        database.markNodeReady(done.node);
    }

    for (DefinitionRequest& done : definitionStreamer.completed()) {
        SubtreeHandle definition = definitionCache.lookup(done.asset);
        if (!definition.valid())
            definition = definitionCache.registerLoaded(done.asset, database);

        database.mountSubtree(done.parent, definition, done.placement);
        // A stale parent simply returns an invalid placement.
    }

    database.applyUpdates();
}
```

Making definition nodes unavailable does not change topology. GPU eviction is
separate because one resource may be referenced by several independent nodes:

```cpp
for (NodeHandle node : readinessPolicy.nodesToDisable())
    database.markNodeUnavailable(node);

for (UserPayload payload : payloadCache.unreferencedResources())
    evictFromGpu(payload);
```

Topology can remain mounted so that the ideal cut continues to express future
demand, or collection can remove cold leaf placements later.

## 15. End-to-end example: moving multiview scene

Use one query per logical view, move top-level cohorts in one writer phase, and
combine view usage during collection.

```cpp
SpatialDatabase::MotionGroup vehicles(vehicleInstances);
SpatialQuery mainQuery;
SpatialQuery shadowQuery;
mainQuery.setMountUsageEnabled(true);
shadowQuery.setMountUsageEnabled(true);

void frame(std::span<const float4> vehiclePositions)
{
    // Writer phase.
    database.moveInstances(vehicles, vehiclePositions, 1.0f);
    for (const AnimatedBound& edit : animatedBounds)
        database.setNodeBounds(edit.instance, edit.node, edit.localBounds);
    database.applyUpdates();

    // Concurrent read phase.
    FrontierResult mainResult;
    FrontierResult shadowResult;
    runConcurrently(
        [&] {
            mainQuery.selectFrontier(database, mainCamera, mainParams,
                                     mainResult);
        },
        [&] {
            shadowQuery.selectFrontier(database, shadowCamera, shadowParams,
                                       shadowResult);
        });

    render(mainResult);
    renderShadow(shadowResult);

    // Writer phase resumes after both queries finish.
    std::array<SpatialQuery*, 2> views{&mainQuery, &shadowQuery};
    database.collect(views, mountedSubtreeBudget, minimumUnusedEpochs);
}
```

Owning `FrontierResult` objects are used because the results survive past the
selection calls and are consumed together.

## 16. Handle lifetimes and current limits

All runtime handles carry generation stamps:

```cpp
if (database.isMounted(placement))
    database.unmountSubtree(placement);

if (database.isSubtree(definition))
    useDefinition(definition);
```

| Handle | Refers to | Becomes stale when |
|---|---|---|
| `SubtreeHandle` | registered immutable definition | `releaseSubtree()` |
| `SubtreeInstanceHandle` | one mounted placement | unmount, collection, or owning root removal |
| `InstanceHandle` | one permanent TLAS root | `removeInstance()` |
| `NodeHandle` | one root or mounted node | root or containing placement removal |

Current limits:

- transforms are translation plus positive uniform scale;
- one authored node has at most 511 local children;
- mounted placement slots and definition-local node indices use 20 bits;
- mounted-node generations use 24 bits;
- TLAS-root generations use 20 bits;
- public instance ids use 24 bits;
- rendering, IO, content lookup, and streaming policy remain application
  responsibilities.

For exact signatures, parameter contracts, stale-handle behavior, output
lifetimes, and all support types, continue with
[API_REFERENCE.md](API_REFERENCE.md).
