# Frontier design contracts

This document states the behavioral invariants behind the public API. See
[API.md](API.md) for usage and [ARCHITECTURE.md](ARCHITECTURE.md) for layout.

## 1. One real root

Every top-level instance owns exactly one permanent renderable node in the
TLAS. It is the root of the instance's LOD tree and is always a valid fallback.
The TLAS's internal BVH nodes are spatial acceleration only and never appear in
a frontier.

A one-node tree ends here. No separate hierarchy object, definition, mount, or
sentinel is allocated.

## 2. Serialized definitions have an implicit parent

Registered subtree bytes describe a reusable immutable descendant forest. Its
serialized index zero is an implementation anchor used for packed child ranges;
it is not a renderable node and is never returned. At runtime its direct
renderable nodes are children of the node on which the definition is mounted.

This is the only legal way to instantiate a subtree. A mount therefore always
has one valid renderable parent: either a TLAS root or a mountable node in
another placement.

## 3. Definitions form a DAG; placements form trees

A mountable `NodeDesc` stores only a structural promise that it will remain a
local leaf. The application chooses a registered definition handle and local
translation/uniform scale when it calls `mountSubtree()`. Many city nodes can
mount the same house handle; each call creates a separate placement, so every
top-level instance still owns an ordinary runtime tree.

The database stores no authored content graph or stable content key. Preventing
cycles in application assembly policy remains the content pipeline's
responsibility.

## 4. Immutable data and instance data

One registered serialized byte array permanently owns:

- preorder topology and subtree extents;
- wide child blocks and lane masks;
- application payloads;
- geometric errors;
- authored local bounds;
- mountable-node bits.

Runtime placements own accumulated transforms, residency/coverage flags,
mounted-child links, content stamps, and LRU state. Runtime deformation never
rewrites the definition. Effective bounds use a copy-on-write overlay scoped to
the top-level instance and placement.

## 5. Boundary invariants

Builder output and every mount boundary maintain:

1. parents precede descendants in packed order;
2. each subtree is a contiguous range;
3. a parent bound contains every local child bound;
4. effective error never increases below a parent;
5. a node has local children or a mounted child, never both;
6. a transformed mounted definition fits inside the mount point's authored
   bound.

The runtime carries the parent's effective error ceiling as a per-placement
scalar. It does not modify shared error arrays.

## 6. Two frontiers, one traversal

Selection returns three sequences:

```text
current = shared + currentOnly
ideal   = shared + idealOnly
```

The current cut contains only resident payloads plus permanent TLAS roots and
is guaranteed hole-free. The ideal cut assumes all known payloads are
available. A missing mounted definition stops both cuts at its mountable parent;
application metadata decides which definition handle to request.

Plain leaves emit directly. Interior and mountable nodes are decided by
projected geometric error. Frustum plane masks narrow as traversal descends.

## 7. Residency coverage

Each mounted node records resident and covered flags plus a covered-child
count. A node is covered when its own payload is resident or its visible
descendants provide a complete resident cut. Changes propagate toward the
mount root incrementally. A fully resident mounted tree has a constant-time
summary used to select the lean traversal path.

Topology and payload residency are independent. Mounting exposes finer known
topology; marking payload resident makes one render choice available.

## 8. Handle safety

Runtime slots are recycled. `SubtreeHandle`, `SubtreeInstanceHandle`,
`InstanceHandle`, and `NodeHandle` carry generations. A stale completion cannot
modify a replacement occupying the same numeric slot.

Expected asynchronous races are non-fatal:

- payload residency calls on a stale node do nothing;
- stale queries report absence;
- mounting below a parent collected during IO returns an invalid placement;
- unmounting/removing an already stale handle does nothing.

Incorrect live operations remain contract failures: invalid transform,
non-mountable parent, duplicate child, or bound escape.

## 9. TLAS contracts

The TLAS stores exact world bounds and maximum root error for live instances.
It supports incremental insertion, removal, and grow-only refit. Population
drift, edit count, escaped leaves, and accumulated area growth schedule repair.
`optimize()` performs explicit compaction and a quality rebuild while retaining
public instance ids.

Flat TLAS roots have specialized emission paths and never touch mounted-state
streams. Hierarchical roots may terminate directly before a local camera
transform when their projected error is acceptable.

## 10. Query ownership and reuse

All mutable read-side state belongs to `SpatialQuery`: camera damping, cache
records, traversal scratch, result storage, counters, and optional mount-usage
feedback. The database remains read-only during selection.

Reuse is exact for node membership. A cached record is valid only while:

- the query's conservative position/projection travel stays inside the
  measured decision margin;
- the threshold epoch matches;
- the top-level instance version matches;
- every recorded mounted-tree content stamp matches;
- no frustum-plane decision was required for the instance.

Compact encoded error magnitude can age within that proven interval, but its
above/below-threshold classification remains correct.

## 11. Collection

Mount retention is an application policy. A query records usage only when
explicitly enabled, and collection consumes only the queries selected by the
host. Cold mounted leaves older than the minimum age are removed from the LRU
tail until the placement budget is met. Removing a placement invalidates its
node handles and reports resident payload values that became unreachable.

Definitions are not collected implicitly. The host releases them explicitly
after all placements are gone.

## 12. Complexity

| Operation | Expected cost |
|---|---:|
| build definition | O(nodes) |
| register/release unused definition | O(1), excluding byte-array validation/destruction |
| mount | O(1) plus node-state initialization |
| unmount mounted tree | O(placements and resident nodes removed) |
| payload residency change | O(ancestor depth) until stable |
| submit bound change | O(1) |
| flush bound change | O(ancestor depth) until contained |
| insert/remove/move instance | O(TLAS depth), excluding scheduled rebuild |
| selection | output-sensitive TLAS plus surviving hierarchy work |
| cache hit | O(recorded output plus dependency validation) |
