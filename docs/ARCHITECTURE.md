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

Root payload slot zero and its error remain in dense instance streams.
Additional root slots use a lazy dense-to-sparse index plus one padded runtime
record per multi-payload root; its readiness mask is per instance.

Each mounted definition is a BLAS fragment below one renderable parent. A
definition can have several direct roots because the parent is external. A
definition reference is immutable content-DAG data; a `SubtreeInstanceRt` is a
single placement in one runtime tree.

## Serialized definition bytes

`SubtreeBuilder` emits one aligned, versioned allocation with:

- a 128-byte header;
- interleaved `WideBlock` data (128 bytes in BVH4, 256 bytes in BVH8);
- valid/leaf/zero-error/multi-payload lane masks;
- payload, parent, contiguous-subtree-size, metadata, and error arrays;
- when present, a sparse per-node extra-record index and one 32-byte-aligned
  padded payload/error record per multi-payload node;
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
constant-time format-envelope and root-range checks. Registration still scans
the definition to classify specialized paths and may build an eligible terminal
plan. No deserialized copy or public semantic definition object exists.

The first mount lazily allocates one shared 16-bit node-state word per packed
node. For scalar nodes that word combines slot-zero readiness with derived
coverage. Multi-payload nodes instead use one shared byte readiness mask per
sparse record while the node-state word continues to hold coverage. Definition
slot state is shared by every placement of that exact registered definition.
Equal `UserPayload` values in other nodes are not indexed or coupled. Live
placements form an intrusive list, so one slot-readiness change updates every
affected coverage tree without scanning unrelated definitions or mounts.

## Mounted placement state

The placement hot/cold split is:

| Record | Size | Contents |
|---|---:|---|
| `MountTransformRt` | 32 B | accumulated translation/scale, error clamp, generation, definition id, root-leaf flag |
| `MountStamp` | 8 B | content version, generation, live flag |
| `MountReadiness` | 4 B | fully-ready/multi-payload flags, incomplete-child count |
| `SubtreeInstanceRt` | 56 B | definition, node-state pointer, LRU, owner, mount links, mounted-tree root, definition-list links |
| node state | 2 B/node | covered flag and covered-child count |

Mount-link arrays are allocated only for placements that gain mounted
children. Node-state blocks come from definition-local geometric slabs, avoiding
one heap allocation per placement.

Each placement stores the root slot of its mounted tree. A cached cut records
one dependency for that root. Descendant topology or readiness mutations bump
the root content stamp used to validate the dependency.

## Traversal

Selection first queries the TLAS using build-width bound/error tests. Flat
roots use a direct path that does not read mounted-state arrays. An all-flat snapshot
also bypasses per-instance frontier caching even when reuse is enabled: there
is no hierarchy walk to save, so recording and copying one cached entry would
only add work. Hierarchical roots either terminate in the TLAS or enqueue their
first mounted placement.

A `WorkItem` carries placement slot, effective wide bounds, one implicit
threshold-target bit, and a narrowed frustum mask. Every queued branch produces
the current cover. One dispatch selects dense authored/COW bounds or
sparse-overlay lookup for the whole subtree walk. `wideVisit()` tests
up to `kWide` children (four or eight) together. Plain scalar leaves emit
immediately; other survivors go onto a compact DFS stack.

The selector dispatches scalar versus multi-payload behavior once per
placement. Multi-payload lanes choose the first coarse-to-fine slot satisfying
the projected-error threshold; structural descent starts only after the finest
slot fails. The selected current-cut policy controls ready-slot substitution.

Scalar-only reusable definitions whose direct roots are all leaves have a
batched fully-ready path. Consecutive placements of the same definition reuse
the resolved immutable block while transform, error clamp, and generation
advance through a dense stream. Registration also excludes multi-payload
definitions from the general fully-refined terminal-handle plan.

The fully resident terminal-render path changes the output unit. Its query
builds one cold definition plan containing each terminal node's finest decoded
payload and one `{begin,count}` range per definition node. During selection, a
frustum-inside
branch appends one pointer-plus-two-word `TerminalRenderRun` (16 bytes on a
64-bit target) that references the plan range;
only partial boundary branches visit lower wide blocks. Instance id and the
constant zero-error code are stored once per run. This avoids constructing and
then resolving one handle record per terminal leaf while retaining the same
logical payload iteration downstream. Terminal leaf nodes reuse their own
one-element range as the node-to-payload index, so the plan needs no duplicate
node mapping stream.

This is a separate translation/link unit and a separate strict query type. The
ordinary LOD/readiness walker returns one current handle sequence.
Scenes with streaming readiness, nested mounts, nonzero finest-terminal error,
or deformed bounds remain on `SpatialQuery`; the range path deliberately trades
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
narrower cohort contract: consecutive external ids within the 21-bit frontier
range, constant bounds/scale/mask
and one definition, no per-actor handles, streaming state, or deformed bounds.

## Current coverage and the implicit target

Registered definitions own readiness by node and payload slot. Mounted nodes
carry only derived coverage. Additional TLAS-root slots keep per-instance
readiness beside their sparse runtime payload record. A 4-byte
per-placement summary makes a fully ready mounted tree a constant-time test;
the lean traversal then emits directly to the current sequence.

For partial readiness, one internal threshold-target bit travels with the DFS
item. A set bit continues LOD decisions toward the threshold; a clear bit means
an unavailable target has already been reached and traversal stops at the
nearest ready descendant cover. At a multi-payload node the search can use a
finer ready slot before descending, or a coarser fallback when descendant
coverage is incomplete. There is no second liveness bit or second
result.
Visibility-aware coverage checks ignore unseen missing branches without
allowing a visible hole.

That descendant-cover path implements
`CurrentCutPolicy::PreferReadyDescendants`. Under
`PreferReadyAncestors`, the threshold-directed traversal carries the nearest ready
ancestor candidate across both local children and mount boundaries. An
unavailable target choice marks its candidate for fallback. A parent-first
resolution pass selects the outermost marked candidates and emits one current
cut; it needs no speculative readiness walk. The compact candidate records are query scratch rather than
persistent per-node state, and changing policy invalidates cached cuts through
the existing query epoch.

`computeFrontierRefinement()` is a later, policy-free local walk. Query scratch
retains the exact effective camera and selection parameters, plus provenance
and database versions for the immediately preceding handle result. The walk
starts at those current entries, carries placement transforms and error clamps
across mounts, and commits complete visible immediate-child groups
breadth-first. Depth and node-limit flags distinguish bounded results from an
exhaustive threshold-directed closure. Each group stores its complete parent
entry. Parallel 32-bit expansion streams map the source current cut and every
returned child entry directly to the group that expands it, or `kInvalidIndex`
at a horizon endpoint. Streaming planners can consume the forest linearly
without constructing handle maps or sorted parent-error indexes.
Additional payload slots refine as singleton groups before structural children;
such a group preserves the `NodeHandle` and advances `payloadIndex()`.

## TLAS maintenance

Initial builds and `optimize(OptimizationMode::TopologyAndLayout)` use the
configured `BinnedSAH`, `Median`, or `SpatialBins` tier.
`optimize(OptimizationMode::TopologyOnly)` uses the SpatialBins builder and
does not change dense instance layout. Incremental
insertion descends by least bound growth and splits a full leaf. Removal
invalidates a lane. Instance motion updates exact dense instance state
immediately but defers TLAS writes to `applyUpdates(maintenanceNodeBudget)`.

A cohort smaller than one quarter of the TLAS population uses conservative
grow-only leaf and ancestor propagation. Each changed leaf is queued once for
optional tightening. One maintenance unit recomputes one queued node from its
exact instance lanes or conservative child extents. If the node shrinks, its
parent is queued, so a finite node budget naturally spreads repair toward the
root across later updates. A zero budget leaves the conservative envelopes in
place; `kUnlimitedTlasMaintenance` drains the queue. Every partial result is a
valid broadphase.

At or above the quarter-population threshold, publication streams the complete
TLAS once in retained bottom-up order, copies exact leaf state from the dense
instance array, recomputes interior lanes, and clears the repair queue. This
mandatory publication strategy trades scattered mover-to-root writes for
sequential O(TLAS nodes) work. It is separate from the caller-budgeted repair
queue.

Population drift, edit fraction, and current lane-area growth set
`UpdateReport::topologyRebuildRecommended`; they never schedule an implicit
topology rebuild. The application calls
`optimize(OptimizationMode::TopologyOnly)` for an exact linear-pass spatial-bin
rebuild that preserves dense layout, or
`optimize(OptimizationMode::TopologyAndLayout)` for configured-quality rebuild
plus compaction and spatial reordering.

The large-batch path reuses mutually exclusive build scratch: one 32-bit array
holds pending dense instance ids and the temporary DFS stack, while another is
the SpatialBins scatter during rebuild and retains the TLAS postorder until
topology changes. It adds no field or allocation to `SpatialDatabase`; a
structural edit invalidates the retained
order. Pending node-bound edits are folded into exact instance boxes before the
actor-motion publication pass.

An 80-byte `Instance` keeps transform, exact world bound, maximum root error,
mask, mounted-root slot, generation, overlay-list index, TLAS back-pointer, and
dense-list index together. Public ids map through stable handle-to-dense tables.
`optimize(OptimizationMode::TopologyAndLayout)` compacts dead dense slots and
rewrites physical back-pointers while preserving those ids.

Rigid actor animation has a separate publication translation unit.
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

`SpatialDatabase::applyUpdates(maintenanceNodeBudget)` is the writer/read
barrier. It flushes bound edits, publishes motion, performs up to the requested
amount of optional TLAS tightening, and advances the collection epoch.
After publication, distinct queries may read concurrently until the next write.

## Memory and performance characteristics

Definition sharing reduces immutable topology and payload duplication but adds
mounted-placement records and an indirection at definition boundaries. The
placement-state counter includes shared readiness/coverage words, retained
private coverage slabs, transforms, links, stamps, and slab capacity; immutable
registered `SubtreeBytes` are reported separately by the application.

Canonical wide-lane bounds dominate immutable topology storage. Every real
node bound is stored once in its parent's `WideBlock` lane, with the aggregate
definition bound in the serialized header; there is no duplicate scalar bound
stream. A BVH4 node uses a 128-byte hot bound block plus 32 bytes of cold
metadata. BVH8 uses 256 hot plus 64 cold bytes. The wider layout can reduce
tree depth while increasing cache footprint, so neither width nor reusable
assembly has a universal traversal advantage.

`SpatialQuery::bytes()`, `SpatialDatabase::subtreeInstanceStateBytes()`,
`overlayBytes()`, and `instanceOrientationStateBytes()` expose retained runtime
capacity. The subtree-state counter excludes sparse multi-payload definition
masks and TLAS-root extra-payload records. Benchmark the current code and
complete result-consumption path on each shipping target with
[BENCHMARKING.md](BENCHMARKING.md).
