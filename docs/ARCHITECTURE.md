# Frontier architecture

This document describes the current implementation. Public workflows are in
the [API guide](API.md), exact signatures are in the
[API reference](API_REFERENCE.md), and behavioral contracts are in
[frontier_design.md](frontier_design.md).

## Spatial structure

The dynamic TLAS uses the build-wide `FRONTIER_BVH_WIDTH`: four or eight lanes.
Every leaf lane names a live top-level instance whose permanent renderable root
data is stored in dense instance streams. Query-hot TLAS nodes contain wide
bounds, child references, valid/leaf flags, and the parent index. Maximum error
and layer masks live in a parallel cold metadata stream used only when the
query requests contribution or layer filtering. BVH4 uses 128 hot + 32 cold
bytes per node; BVH8 uses 256 hot + 64 cold bytes per node.

Each mounted definition is a BLAS fragment below one renderable parent. A
definition can have several direct roots because the parent is external. A
definition reference is immutable content-DAG data; a `SubtreeInstanceRt` is a
single placement in one runtime tree.

## Serialized definition bytes

`SubtreeBuilder` emits one aligned, versioned allocation with:

- a 128-byte header;
- interleaved `WideBlock` data (128 bytes in BVH4, 256 bytes in BVH8);
- valid/leaf/zero-error lane masks;
- payload, parent, contiguous-subtree-size, metadata, and error arrays;
- the definition aggregate bound in the header.

Per-node bounds have no scalar duplicate. Each real node's canonical authored
bound is its lane in its parent's `WideBlock`, so traversal consumes the
serialized representation directly. The 32-bit parent word packs the 20-bit
parent index with the node's sibling ordinal; this addresses the canonical
wide lane in O(1) for inspection and copy-on-write refits. The implicit parent
has no parent lane, so its aggregate bound occupies otherwise reserved header
space.

Mountability is one bit in packed node metadata. Definition targets and
transforms are deliberately absent: they are supplied when a handle is mounted.
The header records the branching factor, so registration rejects bytes built
for the other width even when complete structural validation is disabled.

The public `SubtreeBytes` type owns the aligned array and a copied allocator
context. Registration O(1)-moves that allocation into a cold
`SubtreeDefinitionRt`; an internal pointer-only `SubtreeView` binds directly
over the same bytes. By default it first validates every scalar and wide
traversal stream in O(nodes + wide blocks). `FRONTIER_VALIDATE_SUBTREES=0`
compiles out that complete scan for trusted builder output while retaining
constant-time format-envelope and root-range checks. No deserialized copy or
public semantic definition object exists. The first mount lazily allocates one
shared 16-bit node-state word per packed node. That word combines readiness
with derived coverage; definition-local geometric slabs supply uncommon
private coverage copies without one allocation per placement.

The definition-node bit is the authoritative ready/unavailable state and is
shared by every placement of that exact registered definition. Equal
`UserPayload` values in other nodes are not indexed or coupled. Live placements
of one definition form an intrusive list, so one node-readiness change updates
every affected coverage tree without scanning unrelated definitions or mounts.

## Mounted placement state

The placement hot/cold split is:

| Record | Size | Contents |
|---|---:|---|
| `MountTransformRt` | 32 B | accumulated translation/scale, error clamp, generation, definition id, root-leaf flag |
| `MountStamp` | 8 B | content version, generation, live flag |
| `MountReadiness` | 4 B | fully-ready bit, incomplete-child count |
| `SubtreeInstanceRt` | 56 B | definition, node-state pointer, LRU, owner, mount links, mounted-tree root, definition-list links |
| node state | 2 B/node | covered flag and covered-child count |

Mount-link arrays are allocated only for placements that gain mounted
children. Node-state blocks come from definition-local geometric slabs, avoiding
one heap allocation per placement.

Each placement stores the root slot of its mounted tree. Descendant topology or
readiness mutations bump the root content stamp, so a cached assembled-city cut
normally validates one exact dependency rather than every house placement.

## Traversal

Selection first queries the TLAS using build-width bound/error tests. Flat
roots use a direct path that does not read mounted-state arrays. An all-flat snapshot
also bypasses per-instance frontier caching even when reuse is enabled: there
is no hierarchy walk to save, so recording and copying one cached entry would
only add work. Hierarchical roots either terminate in the TLAS or enqueue their
first mounted placement.

A `WorkItem` carries placement slot, effective wide bounds, current/ideal
liveness, and narrowed frustum mask. One dispatch selects dense authored/COW
bounds or sparse-overlay lookup for the whole subtree walk. `wideVisit()` tests
up to `kWide` children (four or eight) together. Plain leaves emit immediately;
other survivors go onto a compact DFS stack.

Reusable definitions whose direct roots are all leaves have a batched
fully-ready path. Consecutive placements of the same definition reuse the
resolved immutable block while transform, error clamp, and generation advance
through a dense stream.

The fully resident terminal-render path changes the output unit. Its query
builds one cold definition plan containing decoded terminal payloads and one
`{begin,count}` range per definition node. During selection, a frustum-inside
branch appends one pointer-plus-two-word `TerminalRenderRun` (16 bytes on a
64-bit target) that references the plan range;
only partial boundary branches visit lower wide blocks. Instance id and the
constant zero-error code are stored once per run. This avoids constructing and
then resolving one handle record per terminal leaf while retaining the same
logical payload iteration downstream. Terminal leaf nodes reuse their own
one-element range as the node-to-payload index, so the plan needs no duplicate
node mapping stream.

This is a separate archive member and a separate strict query type. The
ordinary LOD/readiness walker and its flat three-bucket API are unchanged.
Scenes with streaming readiness, nested mounts, nonzero terminal error, or
deformed bounds remain on `SpatialQuery`; the range path deliberately trades
those capabilities for a compact max-detail representation.

Homogeneous terminal actors may bypass general instance publication entirely.
A `TerminalInstanceBatch` references immutable definition data plus
caller-owned position and yaw streams. The query first traverses the normal
TLAS for static/heterogeneous roots, then scans batch roots directly. Root
inside/coarsened actors append the plan's complete payload range; only partial
actors transform the camera and visit definition blocks. This static-TLAS plus
flat-dynamic split avoids copying simulation transforms, allocating mount
placements, updating orientation records, and refitting dynamic TLAS leaves.
An optional immutable partition of spatially contiguous actor ranges adds an
exact two-level broadphase. By default the query reduces current member
transforms into one cluster union, rejects or accepts whole clusters, and
passes only a boundary cluster's unresolved plane mask to member tests. A
parallel optional AABB stream can replace that reduction with conservative
caller-authored envelopes. Lifetime envelopes are immutable and free per
frame; published snapshots follow the normal publish-before-query rule and are
coverage-checked in contract builds. Conservative looseness changes only the
amount of member work, while under-bounds violate correctness. The trade is
spatially ordered actor storage, optional envelope memory, and a deliberately
narrower cohort contract: consecutive external ids, constant bounds/scale/mask
and one definition, no per-actor handles, streaming state, or deformed bounds.

## Current and ideal coverage

Registered definitions own readiness by node. Mounted nodes carry only derived
coverage. A 4-byte
per-placement summary makes a fully ready mounted tree a constant-time test;
the lean traversal then emits only the shared bucket.

For partial readiness, current/ideal liveness travels with the DFS item. An
ideal proxy can be unavailable while a recursively complete ready descendant cut
keeps the current traversal alive. Visibility-aware coverage checks ignore
unseen missing branches without allowing a visible hole.

That descendant-cover path implements
`CurrentCutPolicy::PreferReadyDescendants`. Under
`PreferReadyAncestors`, the single ideal traversal carries the nearest ready
ancestor candidate across both local children and mount boundaries. An
unavailable ideal choice marks its candidate for fallback. A parent-first
resolution pass selects the outermost marked candidates and partitions the
recorded ideal entries into `shared` and `idealOnly`; it needs no speculative
readiness walk. The compact candidate records are query scratch rather than
persistent per-node state, and changing policy invalidates cached cuts through
the existing query epoch.

## TLAS maintenance

Initial and quality builds use the configured `BinnedSAH`, `Median`, or `Morton`
tier. Incremental insertion descends by least bound growth and splits a full
leaf. Removal invalidates a lane. Instance motion updates exact dense instance
state immediately but defers TLAS writes to `applyUpdates()`. A cohort smaller
than one quarter of the TLAS population uses conservative grow-only leaf and
ancestor propagation. At or above that threshold, publication streams the
complete TLAS once in retained bottom-up order, copies exact leaf state from
the dense instance array, recomputes interior lanes, and clears loose-leaf
flags. This trades scattered mover-to-root walks for sequential O(TLAS nodes)
work and shrinks old swept envelopes at the same time. Population drift, edit
fraction, and added surface area still schedule quality rebuilds.

The large-batch path reuses mutually exclusive build scratch: one 32-bit array
holds pending dense instance ids and the temporary DFS stack, while another
retains the TLAS postorder until topology changes. It adds no field or
allocation to `SpatialDatabase`; a structural edit invalidates the retained
order. Pending node-bound edits are folded into exact instance boxes before the
actor-motion publication pass.

An 80-byte `Instance` keeps transform, exact world bound, maximum root error,
mask, mounted-root slot, generation, overlay-list index, TLAS back-pointer, and
dense-list index together. Public ids map through stable handle-to-dense tables.
`optimize()` compacts dead dense slots and rewrites physical back-pointers while
preserving those ids.

Rigid actor animation has a separate archive-level publication module.
`RigidMotionGroup` caches the same caller-to-dense mapping plus a proof that all
members use authored yaw-invariant broadphase bounds. Positions and yaws arrive
as independent contiguous streams; stable-scale actors translate their exact
world boxes in place while orientation and travel update in parallel dense
arrays. The general AoS transform path remains the fallback for scale changes
or ordinary oriented bounds. Keeping this kernel in its own static-library
object means applications that never call it do not link it; the generic
spatial-database object and its query code remain byte-identical.

## Copy-on-write bounds

Authored bounds are immutable and usually read directly from the definition.
The first `setNodeBounds()` touching an `(instance, placement)` pair creates an
overlay. The overlay keeps only changed wide bounds; it does not create a
parallel scalar-bounds array. Small definitions materialize dense wide bounds.
Large definitions start with a dense block-to-patch index plus only modified
wide blocks, then promote when edits reach the configured fraction.

Queued edits retain caller order. Ancestor propagation stops as soon as an
existing bound contains the change, crosses mount boundaries through owner
links, and updates the instance/TLAS bound. Runtime overlay ancestors are
grow-only; top-level rebuilds do not shrink them.

## Query reuse and concurrency

Each `SpatialQuery` owns a camera damper, 32-byte hot reuse records, 4-byte cold
allocation records, a compact output slab, optional dependency spill, scratch,
optional compile-time statistics, and optional mount-use records. Cache validity is proved using a
position/projection travel budget plus instance and mounted-tree versions.

Only instances fully inside the frustum are reusable because no plane decision
then depends on camera rotation. Uncached selection can split visible instance
runs across the host's blocking `parallelFor`; workers own all temporary output
and are concatenated deterministically.

`SpatialDatabase::applyUpdates()` is the writer/read barrier. It flushes bound
edits, performs scheduled TLAS maintenance, and advances the collection epoch.
After publication, distinct queries may read concurrently until the next write.

## Memory and performance checkpoints

The city/house benchmark compares 400 houses with eight fully refined detail
nodes each. The 2026-08-18 cross-platform Release reports measured these
eight-byte-payload footprints:

| Native layout | Representation | Immutable definitions | Placement state | Total retained | Reduction vs flat |
| --- | --- | ---: | ---: | ---: | ---: |
| BVH4 | Flattened | 200.6 KiB | 7.2 KiB | 207.8 KiB | - |
| BVH4 | Assembled | 23.1 KiB | 54.1 KiB | 77.2 KiB | 62.8% |
| BVH8 | Flattened | 198.8 KiB | 7.2 KiB | 206.0 KiB | - |
| BVH8 | Assembled | 22.9 KiB | 50.4 KiB | 73.3 KiB | 64.4% |

Assembly also reduced complete construction latency by 60-83% across the M2
Max, Cortex-A72 SBC, i9-12900K, and EPYC 9654 reports. Once the exact assembly
cut was admitted to the whole-cut memo, flat and assembled lookup medians
differed by at most 0.3 ns. Raw uncached traversal remained target-sensitive:
assembly was 37% faster on the EPYC and 15-46% slower on the other machines.
Definition sharing reduces the immutable working set but adds mount
indirection, so shipping targets should measure that uncached balance rather
than assuming one direction. A recurring exact 10,000-root view is now a
10-68 ns whole-cut memo lookup, but that does not consume its 20,000 entries
and is not a continuous-camera measurement. The realistic 100,000-leaf city
times continuous camera motion, 1,100 moving roots, publication, and exact
selection directly at 18.254-69.866 us per payload64 frame; motion and
publication alone take 1.953-8.140 us. See the full
[cross-platform snapshot](PERFORMANCE_RESULTS_2026_08_18.md).

The placement-state counter includes definition-local shared
readiness/coverage words and retained private coverage slabs; it is
intentionally larger for the 400 mounted house placements even though their
immutable definition bytes are shared.

Canonical wide-lane bounds are responsible for much of the current immutable
footprint. Every real node bound is stored once in its parent's `WideBlock`
lane, with the aggregate definition bound in the serialized header; there is
no duplicate scalar bound stream. In the local before/after experiment this
reduced immutable bytes by about 36%, improved construction by roughly
32--48%, and improved batched copy-on-write bound updates by 25--77% for
32--256 edits. Selection moved by at most about 3%, within run-to-run noise.

These numbers are implementation checkpoints rather than guarantees for a new
scene or machine. Reproduce them with the matched profiles in
[BENCHMARKING.md](BENCHMARKING.md); historical API and layout measurements are
kept in [HISTORY.md](HISTORY.md) and `bench_results/`.
