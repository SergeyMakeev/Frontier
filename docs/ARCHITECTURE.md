# Frontier architecture

This document describes the current implementation. Public workflows are in
the [API guide](API.md), exact signatures are in the
[API reference](API_REFERENCE.md), and behavioral contracts are in
[frontier_design.md](frontier_design.md).

## Spatial structure

The dynamic TLAS is an 8-wide BVH. Every leaf lane names a live top-level
instance whose permanent renderable root data is stored in dense instance
streams. Internal TLAS nodes contain wide bounds, maximum error, layer masks,
child references, and a flag indicating whether a hierarchical root can be
tested directly.

Each mounted definition is a BLAS fragment below one renderable parent. A
definition can have several direct roots because the parent is external. A
definition reference is immutable content-DAG data; a `SubtreeInstanceRt` is a
single placement in one runtime tree.

## Serialized definition bytes

`SubtreeBuilder` emits one aligned, versioned allocation with:

- a 128-byte header;
- interleaved `WideBlock` data;
- valid/leaf lane masks;
- authored bounds and payload arrays;
- parent, contiguous-subtree-size, and packed metadata arrays;
- geometric errors.

Mountability is one bit in packed node metadata. Definition targets and
transforms are deliberately absent: they are supplied when a handle is mounted.

The public `SubtreeBytes` type owns the aligned array and a copied allocator
context. Registration validates and O(1)-moves that allocation into a cold
`SubtreeDefinitionRt`; an internal pointer-only `SubtreeView` binds traversal
streams directly over the same bytes. No deserialized copy or public semantic
definition object exists. Registration also creates a definition-local
geometric slab allocator for per-placement node state.

## Mounted placement state

The placement hot/cold split is:

| Record | Size | Contents |
|---|---:|---|
| `MountTransformRt` | 32 B | accumulated translation/scale, error clamp, generation, definition id, root-leaf flag |
| `MountStamp` | 8 B | content version, generation, live flag |
| `MountResidency` | 8 B | resident-node count, incomplete-child count |
| `SubtreeInstanceRt` | 48 B | definition, node-state pointer, LRU, owner, mount links, mounted-tree root |
| node state | 2 B/node | resident/covered flags and covered-child count |

Mount-link arrays are allocated only for placements that gain mounted
children. Node-state blocks come from definition-local geometric slabs, avoiding
one heap allocation per placement.

Each placement stores the root slot of its mounted tree. Descendant topology or
residency mutations bump the root content stamp, so a cached assembled-city cut
normally validates one exact dependency rather than every house placement.

## Traversal

Selection first queries the TLAS using 8-lane bound/error tests. Flat roots use
a direct path that does not read mounted-state arrays. Hierarchical roots either
terminate in the TLAS or enqueue their first mounted placement.

A `WorkItem` carries placement slot, effective wide bounds, current/ideal
liveness, and narrowed frustum mask. One dispatch selects dense authored/COW
bounds or sparse-overlay lookup for the whole subtree walk. `wideVisit()` tests
up to eight children together. Plain leaves emit immediately; other survivors
go onto a compact DFS stack.

Reusable definitions whose direct roots are all leaves have a batched
fully-resident path. Consecutive placements of the same definition reuse the
resolved immutable block while transform, error clamp, and generation advance
through a dense stream.

## Current and ideal coverage

Mounted nodes carry resident and covered state. Per-placement summaries make a
fully resident mounted tree a constant-time test. The lean traversal then emits
only the shared bucket.

For partial residency, current/ideal liveness travels with the DFS item. An
ideal proxy can be missing while a recursively complete resident descendant cut
keeps the current traversal alive. Visibility-aware coverage checks ignore
unseen missing branches without allowing a visible hole.

## TLAS maintenance

Initial and quality builds use the configured `BinnedSAH`, `Median`, or `Morton`
tier. Incremental insertion descends by least bound growth and splits a full
leaf. Removal invalidates a lane. Motion grows ancestor lanes until already
contained. Population drift, escapes, edit fraction, and added surface area
schedule rebuilds.

An 80-byte `Instance` keeps transform, exact world bound, maximum root error,
mask, mounted-root slot, generation, overlay-list index, TLAS back-pointer, and
dense-list index together. Public ids map through stable handle-to-dense tables.
`optimize()` compacts dead dense slots and rewrites physical back-pointers while
preserving those ids.

## Copy-on-write bounds

Authored bounds are immutable and usually read directly from the definition.
The first `setNodeBounds()` touching an `(instance, placement)` pair creates an
overlay. Small definitions materialize dense wide bounds. Large definitions
start with a dense block-to-patch index plus only modified wide blocks, then
promote when edits reach the configured fraction.

Queued edits retain caller order. Ancestor propagation stops as soon as an
existing bound contains the change, crosses mount boundaries through owner
links, and updates the instance/TLAS bound. Runtime overlay ancestors are
grow-only; top-level rebuilds do not shrink them.

## Query reuse and concurrency

Each `SpatialQuery` owns a camera damper, 32-byte hot reuse records, 4-byte cold
allocation records, a compact output slab, optional dependency spill, scratch,
statistics, and optional mount-use records. Cache validity is proved using a
position/projection travel budget plus instance and mounted-tree versions.

Only instances fully inside the frustum are reusable because no plane decision
then depends on camera rotation. Uncached selection can split visible instance
runs across the host's blocking `parallelFor`; workers own all temporary output
and are concatenated deterministically.

`SpatialDatabase::applyUpdates()` is the writer/read barrier. It flushes bound
edits, performs scheduled TLAS maintenance, and advances the collection epoch.
After publication, distinct queries may read concurrently until the next write.

## Memory and performance result

The city/house benchmark compares 400 houses with eight fully refined detail
nodes each. With the keyless bytes API, the assembled representation occupies
91.8 KiB versus 318.6 KiB flattened (71% less). Median raw traversal is 10.8 us
versus 11.8 us, construction is 63.2 us versus 206 us, and warm cached
traversal is approximately 0.7 us for both.

Against the immediately preceding keyed assembly API, assembled memory fell
from 104.3 KiB to 91.8 KiB (12.0%), construction fell from 80.5 us to 63.2 us
(21.5%), and raw traversal improved about 2.0%. Cached traversal moved by less
than 1%, which is measurement noise at this scale.

The comparison is retained under `bench_results/bytes_api_before.json` and
`bench_results/bytes_api_after.json`.
