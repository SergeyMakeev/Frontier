# Frontier API

Frontier is a C++20 library. The normal public entry points are:

```cpp
#include <frontier/builder.h>
#include <frontier/spatial_database.h>
```

## Core model

Every top-level instance is exactly one permanent, renderable node stored in
the TLAS. Deeper hierarchies are assembled by mounting registered immutable
definitions beneath that root or beneath mountable leaves in other mounted
definitions.

There is one node descriptor:

```cpp
struct NodeDesc {
    UserPayload payload = 0;
    float geometricError = 0.0f;
    AABB bounds = AABB::empty();
    bool mountable = false;
};
```

`SpatialDatabase::instantiate()` stores a `NodeDesc` directly in the TLAS.
`SubtreeBuilder::createNode()` stores the same descriptor in serialized
definition bytes.

`bounds` and `geometricError` use the containing hierarchy's local units. A
TLAS root is placed in world space by `InstanceDesc`; a mounted definition is
placed in its parent's coordinate system by the `Transform` passed to
`mountSubtree()`.

A mountable node is the renderable fallback at an assembly boundary. It must be
a local leaf. A registered definition may later be mounted beneath it, making
the definition's direct nodes its children. The runtime stores no authored
content key or target: the application chooses the definition handle and mount
transform at assembly time.

The serialized bytes contain an internal implicit-parent sentinel so roots and
nested definitions use one layout. The sentinel is not renderable or publicly
addressable. When mounted, the real renderable parent is always the TLAS root or
mountable node supplied by the caller.

## Authoring reusable definitions

Build reusable components independently:

```cpp
SubtreeBuilder houseBuilder;
houseBuilder.createNode(NodeDesc{
    .payload = houseDetailPayload,
    .geometricError = 0.0f,
    .bounds = houseLocalBounds,
});
SubtreeBytes houseBytes = houseBuilder.build();

SubtreeBuilder cityBuilder;
cityBuilder.reserve(2); // optional node-capacity hint
cityBuilder.createNode(NodeDesc{
    .payload = leftHouseProxy,
    .geometricError = 16.0f,
    .bounds = leftHouseBoundsInCity,
    .mountable = true,
});
cityBuilder.createNode(NodeDesc{
    .payload = rightHouseProxy,
    .geometricError = 16.0f,
    .bounds = rightHouseBoundsInCity,
    .mountable = true,
});
SubtreeBytes cityBytes = cityBuilder.build();
```

`createNode(desc)` creates a direct child of the implicit parent.
`createNode(parent, desc)` creates a local child of an earlier builder node.
Returned `NodeId` values exist only while authoring; they are not runtime
`NodeHandle` values.

`build()` consumes the builder and emits the traversal-ready serialized byte
array: packed preorder, subtree extents, 8-wide child blocks, conservative
ancestor bounds, payloads, mountable bits, and monotonic errors. Authored child
errors are clamped to their local parent's error. The builder checks:

- at least one renderable node;
- finite, non-negative geometric error;
- a non-empty resulting bound for every node (an interior node may derive an
  initially empty bound from its children);
- no local children beneath a mountable node;
- at most 511 local children or direct nodes beneath one implicit parent.

The builder does not know which definition will eventually be mounted at a
mountable node. The runtime therefore checks that the selected definition's
bounds fit after the caller's mount transform is applied.

## Serialized bytes and ownership

`SubtreeBytes` is an owning, 64-byte-aligned byte array. The representation
emitted by `build()` is both the on-disk format and the in-memory traversal
format; registration does not unpack it into another semantic object.

```cpp
SubtreeBytes bytes = builder.build();
writeFile(bytes.bytes());

SubtreeHandle definition =
    database.registerSubtree(std::move(bytes));
```

The temporary returned by `builder.build()` can be consumed directly:

```cpp
SubtreeHandle definition = database.registerSubtree(builder.build());
```

To load serialized data, allocate the final aligned array and read into it:

```cpp
SubtreeBytes bytes(fileSize, context);
readFile(bytes.bytes());
SubtreeHandle definition =
    database.registerSubtree(std::move(bytes));
```

The relevant array operations are `data()`, `size()`, `empty()`, and `bytes()`.
`SubtreeBytes` can be explicitly copied when an application really wants a
second byte array, but registration has one ownership contract only:

```cpp
SubtreeHandle registerSubtree(SubtreeBytes&& bytes);
```

A named array therefore requires `std::move`. Registration validates the
header, version, size, alignment, layout offsets, and sentinel, then moves the
existing allocation in O(1). There are no copy-registration or borrowed-storage
variants. Registering identical bytes twice is legal and returns independent
handles; content deduplication is application policy.

`SubtreeBytes` copies its `FrontierContext` callbacks by value. A custom
allocator must honor the requested alignment, and the state referenced by
`context.user` must outlive every byte array or registered definition using it.

## Registration and top-level instances

```cpp
SpatialDatabase database;

SubtreeHandle house =
    database.registerSubtree(std::move(houseBytes));
SubtreeHandle city =
    database.registerSubtree(std::move(cityBytes));

InstanceHandle cityInstance = database.instantiate(NodeDesc{
    .payload = cityProxyPayload,
    .geometricError = 64.0f,
    .bounds = cityBounds,
    .mountable = true,
}, InstanceDesc{
    .pos = worldPosition,
    .scale = worldScale,
    .mask = layerMask,
});

SubtreeInstanceHandle cityPlacement =
    database.mountSubtree(cityInstance.rootNode(), city);
```

`instantiate()` creates only the permanent TLAS node. With `mountable == false`,
a one-node object lives entirely in the TLAS and allocates no definition or
mount state. `InstanceDesc::pos` and `scale` place its local bounds in world
space; `mask` is ANDed with `Camera::viewMask` for top-level layer culling.

`removeInstance()` removes the permanent root and recursively unmounts
everything beneath it. `moveInstance()` changes its translation and positive
uniform scale. Both ignore stale `InstanceHandle` values.

## Mounting and topology streaming

```cpp
SubtreeInstanceHandle mountSubtree(
    NodeHandle parent,
    SubtreeHandle definition,
    const Transform& transform = {});
```

The parent may be a mountable TLAS root or a mountable leaf in an existing
placement. The operation checks that:

- the definition handle is live;
- the parent is live, mountable, and has no mounted child;
- the transform is finite and has positive uniform scale;
- the transformed definition bounds fit inside the parent's authored or
  overlaid bounds.

Transforms accumulate across mount boundaries without rewriting registered
bytes. A stale parent, which is an expected streaming race, returns an invalid
`SubtreeInstanceHandle`. Invalid definition handles and invalid live topology
are contract errors. Mounted child errors receive the parent's effective error
as a runtime ceiling; shared authored errors are never rewritten.

Use `hasMountedSubtree(parent)` to suppress duplicate requests.
`unmountSubtree(placement)` recursively removes a placement and its mounted
descendants; a stale placement is a no-op. `releaseSubtree()` requires zero live
placements. `isSubtree()` and `isMounted()` validate generation-stamped handles.

Definition lookup is deliberately outside Frontier. An application can use its
own asset graph, payload metadata, or retained assembly records to map a
mountable `NodeHandle` to `(SubtreeHandle, Transform)`. A streaming loop usually
examines both ideal buckets because an already-resident proxy can be in
`shared`, while another fallback can put it in `idealOnly`:

```cpp
auto requestIdeal = [&](std::span<const FrontierEntry> entries) {
    for (const FrontierEntry& entry : entries) {
        if (!database.isPayloadResident(entry.nodeHandle))
            requestPayload(entry.nodeHandle);

        if (!entry.overThreshold() ||
            database.hasMountedSubtree(entry.nodeHandle))
            continue;

        MountRequest request = applicationMountRequest(entry.nodeHandle);
        if (request.definition.valid())
            database.mountSubtree(entry.nodeHandle,
                                  request.definition,
                                  request.transform);
    }
};

requestIdeal(cut.shared);
requestIdeal(cut.idealOnly);
```

The application hook must return a valid definition only for authored
mountable nodes. Asynchronous completions retain the parent `NodeHandle` and
attempt the mount when bytes are registered; a parent removed meanwhile is
safely rejected as stale.

Call `applyUpdates()` after mutations and before the next query group.
`UserPayload` is application data, not node identity; duplicate values are
valid. Resolve a live handle with `tryGetPayload()`.

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
database.markPayloadResident(node);
database.markPayloadNonResident(node);
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

`tryGetNodeTransform(node, out)` maps the containing definition's local
coordinates into top-level instance-local space. It returns identity for a TLAS
root and `false` for a stale node. The application composes it with the
top-level transform indexed by `FrontierEntry::instance()`.

`setNodeBounds(instance, node, bounds)` queues a runtime local-bound change for
one top-level instance. A TLAS-root bound is instance-local; a mounted-node
bound uses its definition's local coordinates. The first edit to an
`(instance, placement)` pair creates a private bounds overlay. Payloads, errors,
topology, residency, and other instances remain shared. Large wide-bound arrays
stay sparse until edits justify promotion.

`flushBounds()` applies queued changes immediately. `applyUpdates()` also
flushes them. The submitted node bound is exact, while ancestor propagation is
conservative grow-only. `nodeBounds()` returns the effective authored or
overlaid bound for that instance. `overlayCount()` and `overlayBytes()` expose
retained overlay cost.

## Configuration and host integration

`FrontierContext` provides aligned allocation callbacks plus an optional
blocking `parallelFor` and its maximum `workerCount`. `SubtreeBuilder` and
`SubtreeBytes` use its allocator. A `SpatialDatabaseConfig` copies its context
and controls:

- `tlasQuality`: `Morton`, `Median`, or `BinnedSAH`;
- `tlasTraversalCost` and `tlasIntersectCost` for SAH builds;
- count, area, escape, and incremental-edit rebuild thresholds;
- `parallelInstanceThreshold` for uncached selection (`0` disables it).

`parallelFor` must not return until every requested task completes. Parallel
selection also requires `workerCount > 1`; results remain deterministic. The
database's effective copied configuration is available through `config()`.

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
| `SubtreeHandle` | registered immutable byte array | `releaseSubtree()` |
| `SubtreeInstanceHandle` | one mounted placement | unmount, collection, or owning instance removal |
| `InstanceHandle` | one permanent TLAS root | `removeInstance()` |
| `NodeHandle` | one TLAS root or mounted node | root removal or containing placement removal |

Slots can be reused, but generation stamps prevent old handles from acting on
new occupants. Expected races use stale-safe behavior described above; invalid
live topology, bounds escapes, and illegal lifetime operations are contract
violations.

Database accounting is available through `subtreeCount()`,
`mountedSubtreeCount()` (`streamedSubtreeCount()` is the same count), `frame()`,
`subtreeInstanceStateBytes()`, `overlayCount()`, and `overlayBytes()`.

## Current limits

- transforms support translation plus positive uniform scale;
- one authored node has at most 511 local children;
- mounted placement slots and definition-local node indices use 20 bits;
- mounted-node generations use 24 bits; TLAS-root generations use 20 bits;
- public instance ids use 24 bits;
- rendering, asynchronous IO, payload interpretation, content lookup, and
  streaming policy are application responsibilities.
