# Frontier API

Frontier is a C++20 library. The normal public entry points are:

```cpp
#include <frontier/builder.h>
#include <frontier/spatial_database.h>
```

## Core model

Every top-level instance is exactly one permanent, renderable node stored in
the TLAS. A deeper hierarchy is assembled by mounting immutable reusable
`Subtree` definitions beneath that root or beneath extendable nodes in other
mounted subtrees.

There is only one node descriptor:

```cpp
struct NodeDesc {
    UserPayload payload = 0;
    float geometricError = 0.0f;
    AABB bounds = AABB::empty();
    SubtreeKey childSubtree{};
    Transform childTransform{};
};
```

`SpatialDatabase::instantiate()` stores a `NodeDesc` as a TLAS root.
`SubtreeBuilder::createNode()` stores the same descriptor in a reusable
definition.

`bounds` and `geometricError` use the containing hierarchy's local units. A
TLAS-root descriptor is instance-local and its world placement comes from
`InstanceDesc`; mounted placement comes from `childTransform`. Data authored in
a `Subtree` remains immutable. `setNodeBounds()` gives one top-level instance a
copy-on-write runtime bounds overlay instead of rewriting that definition.

A valid `childSubtree` makes the node extendable. The node remains the
renderable fallback at that boundary; the referenced definition is mounted
beneath it when finer topology is needed. An extendable node cannot also have
locally authored children. `childTransform` places the referenced definition
in the node's containing coordinate system and is meaningful only when
`childSubtree` is valid.

A `Subtree` is a descendant forest, not an independently instantiable object.
Its implicit parent is not a renderable node in the definition. When mounted,
its direct nodes become children of the real renderable node passed to
`mountSubtree()`. Definitions are authored independently and composed only
through `childSubtree` plus `mountSubtree()`; the builder never partitions a
monolithic hierarchy.

## Authoring reusable definitions

Build each reusable component separately. This example authors one house once
and makes two city nodes target that same definition:

```cpp
constexpr SubtreeKey houseKey{0x1001};
constexpr SubtreeKey cityKey{0x2001};

SubtreeBuilder houseBuilder(houseKey);
houseBuilder.createNode(NodeDesc{
    .payload = houseDetailPayload,
    .geometricError = 0.0f,
    .bounds = houseLocalBounds,
});
Subtree house = houseBuilder.build();

SubtreeBuilder cityBuilder(cityKey);
cityBuilder.reserve(2, 2); // optional capacity hint only
cityBuilder.createNode(NodeDesc{
    .payload = leftHouseProxy,
    .geometricError = 16.0f,
    .bounds = leftHouseBoundsInCity,
    .childSubtree = houseKey,
    .childTransform = Transform{leftHousePosition, 1.0f},
});
cityBuilder.createNode(NodeDesc{
    .payload = rightHouseProxy,
    .geometricError = 16.0f,
    .bounds = rightHouseBoundsInCity,
    .childSubtree = houseKey,
    .childTransform = Transform{rightHousePosition, 1.0f},
});
Subtree city = cityBuilder.build();
const AABB cityBounds = city.bounds();
```

`createNode(desc)` creates a direct child of the implicit parent.
`createNode(parent, desc)` creates a local child of an earlier builder node.
Creation order is authoring state only; returned `NodeId` values never become
runtime handles.

`build()` consumes the builder and emits packed preorder, subtree extents,
8-wide child blocks, conservative ancestor bounds, monotonic errors,
deduplicated dependency keys, and expansion metadata. Authored child errors
are clamped to their local parent's error. The builder checks:

- a valid, nonzero definition key and at least one renderable node;
- finite, non-negative geometric error;
- non-empty node bounds;
- positive finite uniform transforms on expansion nodes;
- no local children on an extendable node;
- no direct reference to the definition's own key;
- at most 511 local children or direct nodes under one implicit parent.

The bounds of an extendable node must contain the referenced subtree's bounds
after `childTransform` is applied. That cross-definition condition is checked
when the child is mounted because the two definitions are authored and
registered independently.

## Serialized `Subtree` ownership

`Subtree` is move-only and is the complete immutable serialized unit. Its blob
contains topology, wide bounds, payloads, authored bounds, errors, dependency
keys, and expansion transforms; there is no sidecar object.

```cpp
Subtree copied = Subtree::fromBytes(data, byteCount, context);
Subtree mapped = Subtree::borrow(data, byteCount);
Subtree duplicate = copied.clone(context);
```

`fromBytes()` validates and copies. `borrow()` validates without copying, and
`clone()` is the explicit owned-copy operation. The blob address supplied to
`fromBytes()` or `borrow()` must be 64-byte aligned. Borrowed bytes must remain
alive and unchanged until the `Subtree`, or the database it is moved into, is
destroyed or releases the definition.

Owned blobs use the `FrontierContext` allocator. A custom allocator must honor
the requested alignment, and the referenced `FrontierContext` object plus its
callback state must outlive every owned `Subtree` created with it. The process
default context has static lifetime.

Read-only metadata is exposed through `valid()`, `key()`, `nodeCount()`,
`bounds()`, `dependencies()`, `data()`, and `byteSize()`. `nodeCount()` excludes
the internal implicit-parent record.

## Registration and top-level instances

```cpp
SpatialDatabase database;

SubtreeHandle houseDefinition =
    database.registerSubtree(std::move(house));
SubtreeHandle cityDefinition =
    database.registerSubtree(std::move(city));

InstanceHandle cityInstance = database.instantiate(NodeDesc{
    .payload = cityProxyPayload,
    .geometricError = 64.0f,
    .bounds = cityBounds,
    .childSubtree = cityKey,
}, InstanceDesc{
    .pos = worldPosition,
    .scale = worldScale,
    .mask = layerMask,
});

SubtreeInstanceHandle cityPlacement =
    database.mountSubtree(cityInstance.rootNode(), cityDefinition);
```

`registerSubtree()` moves one owned or borrowed definition into the database.
Every live definition must have a unique `SubtreeKey`; duplicate registration
is a contract violation. Registration order does not need to follow dependency
order.

`instantiate()` creates only the permanent TLAS node. If its `childSubtree` is
invalid, the entire object lives in the TLAS and allocates no subtree or mount
state. `InstanceDesc::pos` and `scale` place its local bounds in world space;
`mask` is ANDed with `Camera::viewMask` for top-level layer culling.

`removeInstance()` removes the permanent root and recursively unmounts
everything below it. `moveInstance()` changes its translation and positive
uniform scale. Both ignore stale `InstanceHandle` values.

## Mounting and topology streaming

`mountSubtree(parent, definition)` accepts either an extendable TLAS root or an
extendable node in a mounted subtree. It checks that:

- the parent is live, extendable, and does not already have a mounted child;
- the registered definition's key matches the parent's `childSubtree`;
- the transformed child bounds fit inside the parent's authored bounds;
- the authored transform is finite and has positive uniform scale.

A nested parent's `childTransform` is used automatically. Transforms accumulate
across mount boundaries without rewriting shared definition bytes. A stale
parent handle, which is an expected streaming race, returns an invalid
`SubtreeInstanceHandle`. An invalid or released definition handle is a contract
error. Mounted child errors receive the parent's effective error as a runtime
ceiling; shared authored errors are not rewritten.

Use `hasMountedSubtree(node)` to suppress duplicate requests and
`subtreeTarget(node)` to obtain the stable authored key. `subtreeTarget()`
returns an invalid key for stale or non-extendable handles.

`unmountSubtree(placement)` recursively removes that placement and its mounted
descendants; a stale placement is a no-op. Removing an instance does the same
for its root placement. `releaseSubtree()` requires zero live placements and
otherwise reports a contract violation. `isSubtree()` and `isMounted()` test
generation-stamped handles.

The topology-demand loop must examine both buckets of the ideal cut. A missing
high-error expansion can be in `shared` when its proxy is already resident, or
in `idealOnly` when the current cut is using another fallback:

```cpp
auto requestIdeal = [&](std::span<const FrontierEntry> entries) {
    for (const FrontierEntry& entry : entries) {
        if (!database.isPayloadResident(entry.nodeHandle))
            requestPayload(entry.nodeHandle); // mark resident after IO completes

        if (!entry.overThreshold())
            continue;

        const SubtreeKey target = database.subtreeTarget(entry.nodeHandle);
        if (target.valid() &&
            !database.hasMountedSubtree(entry.nodeHandle)) {
            const SubtreeHandle definition = findRegisteredDefinition(target);
            if (definition.valid())
                database.mountSubtree(entry.nodeHandle, definition);
            else
                requestSubtree(target, entry.nodeHandle);
        }
    }
};

requestIdeal(cut.shared);
requestIdeal(cut.idealOnly);
```

`requestPayload`, `findRegisteredDefinition`, and `requestSubtree` above are
application policy hooks, not Frontier functions. An asynchronous subtree
completion registers the definition once and attempts each retained parent
handle; `mountSubtree()` safely rejects any parent that became stale meanwhile.

Call `applyUpdates()` after these mutations and before the next group of
queries. `UserPayload` is application data, not node identity; duplicate values
are valid. Resolve a live handle with `tryGetPayload()`.

## Frontier selection and output

Publish writes before querying:

```cpp
database.applyUpdates();

SpatialQuery query;
FrontierResultView cut = query.selectFrontier(
    database, camera,
    SelectionParams{.threshold = 4.0f, .minPix = 0.0f});
```

`threshold` is the screen-error refinement threshold in pixels. `minPix` is an
optional top-level contribution cutoff; zero disables it.

The three spans are disjoint:

- `shared`: nodes present in both cuts;
- `currentOnly`: resident fallback nodes needed only by the current cut;
- `idealOnly`: nodes needed only by the fully resident ideal cut over the
  currently mounted topology.

Render `shared + currentOnly`. Use `shared + idealOnly` to drive payload and
topology demand. `currentSize()`, `idealSize()`, `size()`, and `empty()` report
the corresponding result sizes.

`FrontierEntry` is 12 bytes: an 8-byte `NodeHandle`, a stable 24-bit public
instance id, and an 8-bit threshold-relative error code. `instance()` remains
stable across `optimize()` and identifies the application's top-level transform
entry while the instance is alive. `overThreshold()` preserves the exact LOD
classification; `approximateError(threshold)` decodes the quantized magnitude.
Cached entries can retain an older magnitude while their classification remains
provably valid.

The default `FrontierResultView` points into its `SpatialQuery` and remains
valid until that query's next selection, `reset()`, or destruction. Use
`FrontierResult` for an owning copy. The `FrontierResultSink` overload writes
into three caller-provided `Sink<FrontierEntry>` objects; `count()`,
`dropped()`, and `overflowed()` report fixed-storage overflow.

`SpatialQuery` owns camera damping, reuse records, output storage, statistics,
and optional mount-retention feedback:

- `setHalfLife()` controls damping; zero disables it exactly;
- `setReuseEnabled(false)` selects uncached traversal and permits configured
  instance-level parallelism;
- `reset()` clears damping, reuse, database binding, and unconsumed usage state
  while retaining allocations and the configured half-life;
- `reused()`, `walked()`, and `lastSelectionStats()` describe the last call;
- `bytes()` reports retained query allocations.

Selection counters are populated only in builds that define `FRONTIER_STATS`.

## Payload residency

TLAS-root payloads are permanent and always resident. Mounted nodes begin
non-resident:

```cpp
database.markPayloadResident(node);     // after upload completes
database.markPayloadNonResident(node);  // before unloading
bool ready = database.isPayloadResident(node);
```

Marking a TLAS root resident is a no-op; marking it non-resident is a contract
violation. Stale mounted-node handles are safe no-ops and report non-resident.
Residency is independent of topology. Coverage propagates incrementally so the
current render cut remains hole-free while finer payloads are missing.

## Collection and query-owned usage

Mount recency tracking is opt-in per query:

```cpp
query.setMountUsageEnabled(true);
query.selectFrontier(database, camera, params);

CollectResult result =
    database.collect(query, maxMountedSubtrees, minAge);
```

Only enabled queries passed to a `collect()` overload contribute new retention
feedback. The span overload combines several cameras. `resetMountUsage()`
discards feedback not yet consumed by collection without resetting frontier
reuse.

Collection walks the existing LRU order and unmounts eligible leaves until the
mount budget is met. A placement must be at least `minAge` published update
epochs old and have no mounted children. Definitions and permanent TLAS roots
remain alive.

`CollectResult::unmountedSubtrees` reports the number removed.
`freedPayloads` contains the payload value of every resident node made
unreachable; values can repeat. Its span points into database-owned storage and
remains valid only until the next `collect()` call or database destruction.

## Transforms, motion, and copy-on-write bounds

`moveInstance(instance, Transform)` updates one top-level translation and
positive uniform scale. `MotionGroup` retains a caller-ordered cohort;
`MotionGroup::reset()` replaces that cohort, and
`moveInstances(group, positions, scale)` updates its positions using one shared
scale while caching the database's current physical order.

`tryGetNodeTransform(node, out)` maps the containing subtree definition's local
coordinates into top-level instance-local space. It returns identity for a TLAS
root and `false` for a stale node. The application composes it with the
top-level transform indexed by `FrontierEntry::instance()`.

`setNodeBounds(instance, node, bounds)` queues a runtime local-bound change for
one top-level instance. A TLAS-root bound is instance-local; a mounted-node
bound uses its definition's local coordinates. The first edit to an
`(instance, mount)` pair creates a private bounds overlay. Payloads, errors,
topology, residency, and other instances remain shared. Large wide-bound arrays
stay sparse until edits justify promotion.

`flushBounds()` applies queued changes immediately. `applyUpdates()` also
flushes them. The submitted node bound is exact, while ancestor propagation is
conservative grow-only. `nodeBounds()` first flushes all pending bound edits and
then returns the effective authored or overlaid bound for that instance.
`overlayCount()` and `overlayBytes()` expose retained overlay cost.

## Configuration and host integration

`FrontierContext` provides aligned allocation callbacks plus an optional
blocking `parallelFor` and its maximum `workerCount`. `SubtreeBuilder` and owned
`Subtree` factories use its allocator. A `SpatialDatabaseConfig` copies its
context and controls:

- `tlasQuality`: `Morton`, `Median`, or `BinnedSAH`;
- `tlasTraversalCost` and `tlasIntersectCost` for SAH builds;
- count, area, escape, and incremental-edit rebuild thresholds;
- `parallelInstanceThreshold` for uncached selection (`0` disables it).

`parallelFor` must not return until every requested task completes. Parallel
selection also requires `workerCount > 1`; results remain deterministic.
The database's effective copied configuration is available through `config()`.

Contract violations route through `FRONTIER_FATAL`, which throws
`std::logic_error` by default. Exception-free hosts can define it before
including Frontier headers.

## Concurrency and update barriers

`SpatialDatabase` is single-writer. Mutation, registration, mounting,
collection, and optimization cannot overlap selection. Apply mutations, call
`applyUpdates()`, and then run any number of concurrent reads using distinct
`SpatialQuery` objects. All reads must finish before the next write. A single
query is mutable and cannot be used concurrently.

Call `applyUpdates()` once per publication group even when topology did not
change; it also advances the epoch used by collection aging. `optimize()` is a
heavier explicit safe point that flushes bounds, compacts dead dense instance
slots, performs a quality TLAS rebuild, and restores spatial order. Public
instance handles and frontier instance ids remain stable.

## Handles and introspection

| Handle | Names | Becomes stale when |
|---|---|---|
| `SubtreeHandle` | registered immutable definition | `releaseSubtree()` |
| `SubtreeInstanceHandle` | one mounted placement | unmount, collection, or owning instance removal |
| `InstanceHandle` | one permanent TLAS root | `removeInstance()` |
| `NodeHandle` | one TLAS root or mounted node | root removal or containing placement removal |

Slots can be reused, but generation stamps prevent old handles from acting on
new occupants. Expected races use stale-safe behavior described above; wrong
keys, invalid live topology, bounds escapes, and illegal lifetime operations
are contract violations.

Database accounting is available through `subtreeCount()`,
`mountedSubtreeCount()` (`streamedSubtreeCount()` is the same count), `frame()`,
`subtreeInstanceStateBytes()`, `overlayCount()`, and `overlayBytes()`.

## Current limits

- transforms support translation plus positive uniform scale;
- one authored node has at most 511 local children;
- mounted placement slots and subtree-local node indices use 20 bits;
- mounted-node generations use 24 bits; TLAS-root generations use 20 bits;
- public instance ids use 24 bits;
- rendering, asynchronous IO, payload interpretation, content lookup, and
  streaming policy are application responsibilities.
