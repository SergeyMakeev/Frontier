# HLodTree architecture

This document describes the current implementation. See the
[README](../README.md) for an overview. For public API usage and
lifetime rules, see [API.md](API.md). For behavioral invariants and complexity,
see [hlod_design.md](hlod_design.md). Historical measurements and superseded
designs are kept separately in [HISTORY.md](HISTORY.md).

## Spatial model

HLodTree has two spatial levels. The names mirror ray-tracing terminology but
describe runtime roles rather than object size.

- A BLAS is an independently rooted renderable hierarchy. It may represent a
  city block, a building, a terrain region, a reusable prop, or one flat
  object. Every real node carries a `UserPayload`, so selection may stop at
  any level.
- The TLAS is a dynamic 8-wide BVH over translated, uniformly scaled BLAS
  instances. It performs coarse frustum, layer, and contribution culling. Its
  internal nodes are acceleration data and never appear in a cut.

`HierarchyBuilder` accepts one logical tree and turns natural `splitBelow()`
boundaries into immutable, deterministically indexed pages. Each generated page
has one logical root; continuation roots inside a packed detail blob remain an
implementation detail. Expansion metadata carries the generated local
detail-page id, eliminating a separate mount manifest. Runtime lookup scopes
that id with the hierarchy's root asset, so streaming does not depend on
payload uniqueness and independent hierarchies may reuse local ids. Instances
of a registered root asset share page bytes, residency, and the attached-page
graph; per-instance deformation allocates bounds-only copy-on-write overlays.

## Published-world lifecycle

`World` is single-writer. Mutations, streaming completions, residency changes,
and collection happen outside selection. `World::applyUpdates()` flushes queued
bounds edits, performs required TLAS maintenance, and publishes a stable
snapshot. Distinct `View` objects may then query the same `const World`
concurrently. All queries must finish before the next mutation or collection.

Each `View` owns its camera damper, temporal reuse records, traversal scratch,
output storage, and statistics. A view therefore has no mutable query state in
the world and cannot interfere with another camera. Cached selection is the
default. An uncached view can use the host's blocking `parallelFor` when
`parallelInstanceThreshold` and `workerCount` enable it.

## Selection pipeline

Selection is output-sensitive:

1. Query the TLAS and discard instances outside the frustum, layer mask, or
   optional minimum projected contribution.
2. Emit exact one-node BLASes and acceptable hierarchical roots directly when
   possible.
3. Transform the camera into each remaining instance's local space.
4. Walk attached pages with an explicit DFS carrying the undecided frustum
   planes plus current- and ideal-cut liveness.
5. Test up to eight children together. Plain leaves emit directly; interior
   and expansion nodes continue through the stack.

The walk returns three disjoint sequences. `shared + currentOnly` is the
resident, hole-free render cut. `shared + idealOnly` is the frontier known
topology would choose if every payload were resident. Membership lives in the
sequence rather than in each entry, keeping `CutEntry` at 12 bytes.

Residency changes maintain a complete-descendant-cover summary. A fully
covered subtree passes in constant time. A partially visible uncovered branch
examines only the branches that survive frustum culling, so resident descendants
can replace a missing intermediate proxy without creating a hole.

## Data layout and SIMD

Page blobs always use eight logical lanes. A `WideBlock` is 256 bytes and holds
child bounds, errors, and local indices; valid and leaf masks live in a compact
side array. AVX2 processes eight lanes directly. SSE2 and NEON process two
four-lane halves. Scalar builds keep the same serialized format.

Hot multiplied data is deliberately compact:

| Structure | 64-bit size | Purpose |
|---|---:|---|
| `CutEntry` | 12 B | node handle, 24-bit instance id, encoded error |
| `View` reuse record | 32 B hot + 4 B cold | cut validity proof and slab location |
| `PageUsageContext` record | 8 B | generation, pending flag, last-use epoch |
| instance cut / TLAS state | 32 B + 48 B | selection state separated from maintenance |
| visible hit / TLAS stack item | 4 B / 4 B | packed instance or node index and flags |
| mounted-node residency state | 3 B | flags plus covered-child count |

Immutable pages keep shared working sets hot. Output contains handles rather
than duplicated payloads. Generation stamps make stale asynchronous streaming
completions cheap to reject without a hash table or payload index.

## TLAS maintenance and locality

The first published snapshot builds the configured TLAS quality tier:
`BinnedSAH`, `Median`, or `Morton`. Later insertions and removals update the
tree in expected O(depth). Count drift, escaped leaves, accumulated edit cost,
and area growth trigger repair or rebuild according to `WorldConfig`.

The first TLAS build spatially reorders physical instance storage while public
`InstanceRef` values and cut instance ids remain stable. Routine repair builds
keep that physical order to avoid turning maintenance into a data shuffle.
`World::optimize()` is the explicit safe-point operation that compacts dead
slots, performs a quality rebuild, and restores spatial instance/View-record
locality after disruptive changes such as a teleport or level transition.

`MotionGroup` serves a persistent cohort whose caller-visible order is fixed.
It caches the corresponding physical order so per-frame transform submission
touches instance and TLAS state coherently, and refreshes that mapping after a
layout change.

Queued node-bound edits preserve caller order. Grouping them by instance and
page improves overlay locality; the runtime does not sort the queue. Ancestors
grow only until a box already contains the change. Internal overlay bounds do
not shrink, so sustained large deformation can loosen page culling without
affecting correctness.

## Streaming and collection

Topology attachment is shared per mounted asset graph. Attaching a child page
under a shared root makes it available to every instance of that root. Payload
residency is tracked independently from topology.

Selection does not mutate the world to update recency. An optional
`PageUsageContext` records pages needed by a view. `World::collect()` consumes
only the contexts selected by the host, updates the intrusive LRU, and detaches
old leaf mounts until the requested streamed-page budget is met. Pinned root
mounts are never collected and do not count toward that budget.

The host retains policy control over page sizing, IO scheduling,
deduplication, attach budgets, residency, and which cameras influence page
retention.

## Current constraints

- Instances support translation and positive uniform scale. Rotation and
  non-uniform scale must be baked or adapted by the integration layer.
- Cached parallelism is across independent views. Parallel work within one
  view is available on the uncached selection path.
- View reuse records at most two page dependencies per instance; deeper walks
  are simply re-evaluated.
- Page overlay refits are grow-only. TLAS rebuilds retighten the top level but
  do not shrink internal overlay ancestors.
- Compact identifiers cap mounted page slots and page entries at 20 bits and
  live instance ids at 24 bits.
- Rendering, content identity, asynchronous IO, and streaming policy remain
  application responsibilities.
