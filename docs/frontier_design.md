# Frontier design contracts

This document states the behavioral invariants behind the public API. See the
[API guide](API.md) for usage, [API reference](API_REFERENCE.md) for exact
signatures, and [ARCHITECTURE.md](ARCHITECTURE.md) for layout.

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

Each registered definition owns one readiness bit per renderable node. That bit
is shared across every placement of the definition. Runtime placements own
accumulated transforms, derived coverage, mounted-child links, content stamps,
and LRU state. Runtime deformation never rewrites the definition. Effective
bounds use a copy-on-write overlay scoped to the top-level instance and
placement.

## 5. Boundary invariants

Builder output and every mount boundary maintain:

1. parents precede descendants in packed order;
2. each subtree is a contiguous range;
3. a parent bound contains every local child bound;
4. effective error never increases below a parent;
5. a node has local children or a mounted child, never both;
6. a transformed mounted definition fits inside the shared authored bound of a
   definition node, or the current instance-local root bound of a TLAS root.

The runtime carries the parent's effective error ceiling as a per-placement
scalar. It does not modify shared error arrays.

## 6. Two frontiers, one traversal

Selection returns three sequences:

```text
current = shared + currentOnly
ideal   = shared + idealOnly
```

The current cut contains only ready definition nodes plus permanent TLAS roots
and has complete hierarchy coverage. By default, complete ready descendants
may replace an unavailable node; otherwise a ready parent remains selected.
`CurrentCutPolicy::PreferReadyAncestors` disables the descendant substitution
and permits only upward fallback, producing a smaller, coarser cut. This is the
meaning of the hole-free guarantee; it does not concern mesh seams or
rasterization. The ideal cut assumes all known nodes are available. A missing
mounted definition stops both cuts at its mountable parent; application
metadata decides which definition handle to request.

Plain leaves emit directly. Interior and mountable nodes are decided by
projected geometric error. Frustum plane masks narrow as traversal descends.

## 7. Readiness and coverage

Readiness means that every GPU resource required to dispatch one node's
`UserPayload` is available. It belongs to the node in its registered definition,
not to the payload value or placement. Every placement of that definition node
shares the bit. Equal payload values in other nodes are independent; an
integration may update them together when they identify one resource.

Each mounted node records only derived coverage and a covered-child count. A
node is covered when it is ready or its visible descendants provide
a complete ready cut. Changes propagate toward each affected mount root
incrementally. A fully ready mounted tree has a constant-time summary used to
select the lean traversal path.

Topology and readiness are independent. Mounting exposes finer known topology;
marking a node ready makes that definition node available in every placement.

## 8. Handle safety

Runtime slots are recycled. `SubtreeHandle`, `SubtreeInstanceHandle`,
`InstanceHandle`, and `NodeHandle` carry generations. A stale topology
completion cannot modify a replacement occupying the same numeric slot.

Expected asynchronous races are non-fatal:

- readiness completions retain a `NodeHandle`; stale completions are ignored
  rather than affecting a recycled placement;
- stale queries report absence;
- mounting below a parent collected during IO returns an invalid placement;
- unmounting/removing an already stale handle does nothing.

Incorrect live operations remain contract failures: invalid transform,
non-mountable parent, duplicate child, cross-instance bounds access, or bound
escape.

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

The four-platform Release snapshot confirms the intended payoff: stable reuse
is 3.7-8.5 times faster than raw traversal for a 10,000-root hierarchical
scene, and a 16-unit camera step still reuses about 99.4% of roots. A caller
that knows every record is invalid should disable reuse; deliberately forcing
all cache records to miss costs 37-51% more than a reuse-disabled raw walk in
the same snapshot. See
[PERFORMANCE_RESULTS_2026_08_15.md](PERFORMANCE_RESULTS_2026_08_15.md).

## 11. Collection

Mount retention is an application policy. A query records usage only when
explicitly enabled, and collection consumes only the queries selected by the
host. Cold mounted leaves older than the minimum age are removed from the LRU
tail until the placement budget is met. Removing a placement invalidates its
node handles but never changes the registered definition's readiness bits.

Definitions are not collected implicitly. The host releases them explicitly
after all placements are gone.

## 12. Complexity

| Operation | Expected cost |
|---|---:|
| build definition | O(nodes) |
| register definition | default: O(nodes + wide blocks) validation; `FRONTIER_VALIDATE_SUBTREES=0`: O(1) trusted registration |
| release unused definition | O(1), excluding allocator cost |
| mount | O(definition nodes) on its first mount; O(1) for later childless placements; the first nested child copies its owner's coverage state |
| unmount mounted tree | O(placements removed) |
| node readiness change | placements of one definition and ancestor paths until stable |
| submit bound change | O(1) |
| flush bound change | O(ancestor depth) until contained |
| insert/remove/move instance | O(TLAS depth), excluding scheduled rebuild |
| selection | output-sensitive TLAS plus surviving hierarchy work |
| cache hit | O(recorded output plus dependency validation) |

The complexity bounds describe scaling, not constants. Current representative
latencies and the measured split between TLAS work, mounted refinement,
publication, and selection are maintained in the
[cross-platform performance snapshot](PERFORMANCE_RESULTS_2026_08_15.md).
