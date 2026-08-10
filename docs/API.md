# Frontier API

## Core model

A `NodeDesc` is one renderable LOD choice:

```cpp
struct NodeDesc {
    UserPayload payload;
    float geometricError;
    AABB bounds;
    SubtreeKey childSubtree;
    Transform childTransform;
};
```

The descriptor has the same meaning in both authoring locations:

- `SpatialDatabase::instantiate()` stores it as a permanent renderable TLAS
  root.
- `SubtreeBuilder::createNode()` stores it in an immutable reusable `Subtree`.

A valid `childSubtree` makes the node extendable. An extendable subtree node is
a leaf in its own definition; its mounted children belong to the referenced
definition. A TLAS root may also be extendable. There is no separate root-node
type.

Every `Subtree` has an implicit parent, not an implicit render node. Its direct
children become children of the renderable node passed to `mountSubtree()`.
Consequently every runtime hierarchy has one real, valid, renderable parent at
every mount boundary.

## Authoring a subtree

```cpp
constexpr SubtreeKey buildingKey{17};
constexpr SubtreeKey floorKey{18};

SubtreeBuilder builder(buildingKey);
builder.reserve(2, 1); // optional: exact generated-content capacity hint
auto building = builder.createNode(NodeDesc{
    .payload = buildingPayload,
    .geometricError = 32.0f,
    .bounds = buildingBounds,
});
builder.createNode(building, NodeDesc{
    .payload = floorProxyPayload,
    .geometricError = 8.0f,
    .bounds = floorBounds,
    .childSubtree = floorKey,
});
Subtree buildingDefinition = builder.build();
```

Creation order is authoring state only. `build()` derives packed preorder,
subtree extents, child blocks, conservative ancestor bounds, monotonic errors,
deduplicated dependencies, and expansion metadata. Builder node ids do not
become runtime handles.

The builder enforces:

- valid, finite, non-negative error;
- positive finite uniform mount scale;
- at most `kMaxChildren` children;
- no local children below an extendable node;
- no direct self-reference;
- non-empty bounds for every renderable node.

## Serialization and ownership

`Subtree` is the complete serialized unit. The blob contains topology, wide
child blocks, payloads, authored bounds, errors, dependency keys, and mount
transforms. No sidecar is required.

```cpp
Subtree copied = Subtree::fromBytes(data, byteCount, context);
Subtree mapped = Subtree::borrow(data, byteCount);
Subtree duplicate = copied.clone(context);
```

`fromBytes()` validates and copies. `borrow()` validates and borrows aligned
storage; that storage must outlive the object or database receiving it.
`clone()` is the explicit copy operation. All owned storage is allocated and
freed through the associated `FrontierContext`.

Useful read-only metadata is available through `valid()`, `key()`,
`nodeCount()`, `bounds()`, `dependencies()`, `data()`, and `byteSize()`.

## Registration, instantiation, and mounting

```cpp
SubtreeHandle definition =
    database.registerSubtree(std::move(subtree));

InstanceHandle instance = database.instantiate(NodeDesc{
    .payload = rootPayload,
    .geometricError = rootError,
    .bounds = rootBounds,
    .childSubtree = subtreeKey,
}, InstanceDesc{
    .pos = worldPosition,
    .scale = worldScale,
    .mask = layerMask,
});

SubtreeInstanceHandle placement =
    database.mountSubtree(instance.rootNode(), definition);
```

`instantiate()` never allocates a subtree. A node without `childSubtree` exists
entirely in the TLAS. This is the canonical flat-object path.

`mountSubtree()` accepts either a TLAS root handle or an extendable node handle.
It verifies the authored `SubtreeKey`, the transformed bound containment, and
that no child is already mounted. Nested transforms accumulate into the
top-level instance-local transform returned by `tryGetNodeTransform()`.

`unmountSubtree()` removes the named placement and its descendants.
`releaseSubtree()` requires that no live placements still reference the
definition. `isSubtree()` and `isMounted()` validate generation-stamped handles.

The application normally receives nested parent handles from ideal-frontier
entries. `subtreeTarget(node)` returns the authored key used by the content
resolver. Frontier never indexes or assigns identity to `UserPayload`; duplicate
payload values are valid.

## Frontier selection

Publish writes before reading:

```cpp
database.applyUpdates();
SpatialQuery query;
FrontierResultView cut =
    query.selectFrontier(database, camera, SelectionParams{
        .threshold = 4.0f,
        .minPix = 0.0f,
    });
```

The three result spans are disjoint:

- `shared`: nodes present in both cuts;
- `currentOnly`: resident fallback nodes needed only by the current cut;
- `idealOnly`: nodes needed only by the fully available ideal cut.

Render `shared + currentOnly`. Use `shared + idealOnly` for payload and
topology demand. `FrontierEntry` contains a `NodeHandle`, a stable 24-bit public
instance id, and a compact threshold-relative error code. Resolve application
data with `tryGetPayload()`.

The returned spans remain valid until the next selection/reset on that query.
Use `FrontierResult` to retain an owning copy, or `FrontierResultSink` to write
directly into fixed caller memory and inspect overflow counts.

`SpatialQuery` owns temporal reuse and camera damping. `reset()` clears state
after a camera cut while preserving the configured half-life. Disable reuse
with `setReuseEnabled(false)` for dynamic or internally parallel uncached
queries. `reused()`, `walked()`, and `lastSelectionStats()` describe the last
call.

## Payload residency

TLAS roots are permanent and always resident. Mounted payload nodes begin
non-resident:

```cpp
database.markPayloadResident(entry.nodeHandle);
database.markPayloadNonResident(entry.nodeHandle);
bool ready = database.isPayloadResident(entry.nodeHandle);
```

Residency is independent of topology. The runtime incrementally propagates
complete descendant coverage so the current cut never contains a hole. Stale
node handles are safe no-ops and report non-resident.

## Collection and query-owned usage

Mount recency tracking is optional and belongs to `SpatialQuery`:

```cpp
query.setMountUsageEnabled(true);
query.selectFrontier(database, camera, params);

CollectResult result = database.collect(query, mountBudget, minAge);
```

Only queries explicitly passed to collection affect retention. A span of query
pointers can combine several cameras. Collection removes old mounted leaves
from the LRU tail until the budget is met and returns resident payload values
that became unreachable. Definitions remain registered.

## Transforms and bounds

`moveInstance(instance, Transform)` updates translation and positive uniform
scale. `MotionGroup` batches a persistent cohort and caches physical order.

`setNodeBounds(instance, node, bounds)` records a local-space runtime bound
change. `flushBounds()` applies pending changes immediately; `applyUpdates()`
also flushes them. Immutable authored bounds are never rewritten. The first
change to an `(instance, mount)` pair creates a copy-on-write overlay, with
large wide-bound arrays remaining sparse until edits justify promotion.

`nodeBounds()` reads the effective bound seen by one top-level instance.
`overlayCount()` and `overlayBytes()` expose retained overlay cost.

## Concurrency and update barriers

`SpatialDatabase` is single-writer. Mutations, collection, registration, and
mount changes cannot overlap selection. After `applyUpdates()`, distinct
`SpatialQuery` objects may read the same `const SpatialDatabase` concurrently.
All reads must complete before the next write.

`optimize()` is an explicit heavy safe-point: it compacts dead dense instance
slots, performs a quality TLAS rebuild, and restores spatial order. Public
`InstanceHandle` values and frontier instance ids remain stable.

## Handle lifetimes

| Handle | Names | Becomes stale when |
|---|---|---|
| `SubtreeHandle` | registered definition | `releaseSubtree()` |
| `SubtreeInstanceHandle` | mounted placement | unmount, collection, or owning instance removal |
| `InstanceHandle` | permanent TLAS root | `removeInstance()` |
| `NodeHandle` | TLAS root or mounted node | root removal or placement removal |

Slots may be reused, but generations prevent an old handle from acting on the
new occupant.

## Current constraints

- instance and mount transforms support translation plus positive uniform
  scale;
- mounted placement slots and local node indices use 20 bits;
- public instance ids use 24 bits;
- rendering, asynchronous IO, content lookup, and streaming policy remain
  application responsibilities.
