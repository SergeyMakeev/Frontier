# Frontier design and runtime contract

This is the current design implemented by `include/frontier` and `src`. Start with
the [README](../README.md) for the general overview, a compilable minimal example,
and representative performance numbers. See [API.md](API.md) for detailed
integration guidance and [ARCHITECTURE.md](ARCHITECTURE.md) for the current
implementation structure.

## 1. Problem and model

Per-object LOD reduces triangles but not object count. A distant town still
costs one submission per wall. In a hierarchical LOD tree, every node is a
complete proxy for its descendants: the root may draw the town, a child may
draw one building, and leaves may draw individual walls.

This document calls each independently rooted renderable hierarchy a BLAS.
That is a role, not an object-size restriction: one BLAS may describe a city
block, a skyscraper with floors and interiors, a terrain region, a reusable
asset, or one flat vehicle or character. Every real node carries a renderable
`UserPayload`, so selection can stop anywhere in the hierarchy. The dynamic
TLAS spatially indexes placements of these independent BLAS roots. TLAS
internal nodes are acceleration data rather than renderable representations;
they do not appear in the frontier. Consequently the database needs no common
whole-map hierarchy and can freely mix BLASes of very different scopes.

A selection is a ragged antichain through that tree:

```text
                     town
                  /        \
           building A      building B  <- draw the far building proxy
             /  |  \
          wall wall wall                 <- draw nearby walls
```

No selected node is an ancestor of another selected node. This antichain is the
hierarchical **frontier** (also called a cut). Refinement is replace-only:
either a node draws, or its children replace it, so the frontier is hole-free
without parent/child overdraw.

The library is deliberately external to rendering. A node stores an opaque
`uint64_t UserPayload`; selection carries a compact node handle and the caller
may resolve that payload with `SpatialDatabase::tryGetPayload` only when needed. A
payload can be a mesh-table index, entity key, pointer-sized token, or any
other value. Duplicate payloads are legal. Every node must nevertheless have a
renderable representation because any node can become part of the current frontier.

The projected error test is conceptually:

```text
screenErrorPx = geometricError * projectionScale / distance(camera, bounds)
refine when screenErrorPx > SelectionParams::threshold
```

Distance is measured to the node box, not its center. The camera constructors
compute `projectionScale` from vertical FOV and viewport height. A `SpatialQuery` uses
its internal `CameraDamper` to replace the camera point with a decaying camera
envelope, providing LOD hysteresis without sticky state on every node.

## 2. Public objects and handles

The runtime has six distinct concepts:

- A `Page` or `PageView` is an immutable, packed BLAS fragment.
- An `AssetHandle` names page bytes registered with a `SpatialDatabase` for reuse. Here
  *asset* is an API/storage term and does not mean that the BLAS represents one
  object.
- A `Subtree` owns one mount-sentinel page plus permanent external expansion
  targets. It is a descendant fragment, not an independently instantiable
  root. `SubtreeHandle` names its registered logical definition.
- A page mount places an asset below a TLAS root or another expansion point;
  a `PageHandle` names that mount.
- `MountHandle` is the assembly-facing alias for a page mount. A subtree mount
  also carries an accumulated translation and positive uniform scale.
- An instance owns one permanent renderable TLAS root and applies a translation
  and positive uniform scale to it. `SpatialDatabase::InstanceRef` contains a
  generation and exposes that node through `rootNode()`; `rootPage` remains for
  legacy page/asset instances.

`NodeHandle` packs either a mounted-page node or a TLAS root into 64 bits.
Mounted nodes use 20-bit mount and page-local indices plus a 24-bit generation.
The reserved mount-slot code tags a TLAS root and leaves 24 bits for its stable
public instance id plus 20 bits for its generation.
`nodeAt(page, index)` composes one from a `PageHandle` and a packed page-local
index. Generations prevent ABA bugs when page or instance slots are recycled.
An asynchronous completion using a handle whose page was collected is an
expected race: mutating calls ignore it and queries report it absent.

The `SpatialDatabase` keeps no payload-to-node index and no hash map. Persistent systems
should retain handles from `FrontierEntry`, `InstanceRef::rootNode`, legacy
`InstanceRef::rootPage`, or
`attachPage`. Generated expansion nodes embed their `HierarchyPageId`; only a
low-level custom partitioner needs to own additional page-index metadata.

Assembly streaming uses `expansionTarget(NodeHandle)` instead. The target is a
stable `SubtreeKey` stored once in the parent's deduplicated dependency table;
`mount(node, subtreeHandle)` validates that key before creating placement
state.

`InstanceDesc` contains:

- `pos` and positive uniform `scale`;
- `mask`, ANDed with `Camera::viewMask` for cheap layer filtering.

`SpatialDatabase::instantiate(RootNodeDesc, ...)` returns the generation-stamped
owner reference. Legacy page assets use `addInstance`. Selection
packs its stable 24-bit public `InstanceId` into each `FrontierEntry`; callers
normally use that id to index the same placement/transform table that stores
the returned `InstanceRef`.

Rotation and non-uniform scale are not represented by `InstanceDesc`. Bake
them into authored bounds/proxies or place such objects in an integration layer
that presents translation plus uniform scale to Frontier.

### Typical runtime flow

The following uses the actual API. Page loading is application-specific and is
shown as placeholders:

```cpp
void renderEntry(const SpatialDatabase& database, const FrontierEntry& entry)
{
    UserPayload payload;
    if (database.tryGetPayload(entry.nodeHandle, payload))
        submit(payload, transforms[entry.instance()]);
}

void updateStreaming(SpatialDatabase& database, const FrontierEntry& entry)
{
    UserPayload payload;
    if (!database.tryGetPayload(entry.nodeHandle, payload))
        return;

    const DetailPageRef detail = database.detailPage(entry.nodeHandle);
    if (entry.overThreshold() && detail.valid() &&
        !database.isAttached(entry.nodeHandle))
        database.attachPage(entry.nodeHandle, loadHierarchyPage(detail));

    if (!database.isResident(entry.nodeHandle) && payloadFinishedLoading(payload))
        database.markResident(entry.nodeHandle);
}

SpatialDatabase database;
AssetHandle tree = database.registerAsset(loadRootPage());

InstanceDesc desc;
desc.pos = float4::point(100, 0, 20);
desc.scale = 1.0f;
SpatialDatabase::InstanceRef instance = database.addInstance(tree, desc);

Camera camera = makeLookAtCamera(eye, target);
SpatialQuery query;                // owns cache, scratch, and result storage
PageUsageContext primaryUsage;     // only retention-relevant queries need one

database.applyUpdates();              // apply changes and publish the read-only database
const SpatialDatabase& publishedDatabase = database;
const FrontierResultView result = query.selectFrontier(
    publishedDatabase, camera, SelectionParams{4.0f, 0.0f}, primaryUsage);

for (const FrontierEntry& entry : result.shared)
    renderEntry(publishedDatabase, entry);
for (const FrontierEntry& entry : result.currentOnly)
    renderEntry(publishedDatabase, entry);

// After all spatial queries join, streaming and SpatialDatabase mutation are serial again.
for (const FrontierEntry& entry : result.shared)
    updateStreaming(database, entry);
for (const FrontierEntry& entry : result.idealOnly)
    updateStreaming(database, entry);

const CollectResult collected =
    database.collect(primaryUsage, pageBudget, minPageAge);
for (UserPayload payload : collected.freedPayloads) releasePayload(payload);
```

In a real asynchronous streamer, completions normally arrive in later frames.
It normally deduplicates content identities and applies IO/page budgets before
scheduling work; `SpatialDatabase` deliberately does neither. The loop above shows the
state transitions, not a production scheduler. Generated expansion metadata
distinguishes a high-error terminal leaf from one with a detail page.

The generation check is why no extra page-lifetime lock is needed at completion
time. A stale `attachPage` returns an invalid `PageHandle` and leaves the caller
free to discard or cache the loaded page.

## 3. Logical hierarchy pages and packed-page invariants

`SubtreeBuilder::root()` names the packed mount sentinel, not a hierarchy root.
Every real top-level descendant is created with it as the build parent. At
runtime the node passed to `mount()` is the fragment's real renderable parent. An
expansion leaf may reference an independently built `SubtreeKey` without
authoring or copying that child's descendants. The content definitions form a
DAG, but every runtime reference produces a distinct mount, so traversal still
sees a tree and node handles remain placement-specific.

The parent stores one `SubtreeKey` per unique dependency and one 32-byte
expansion record per site. That record contains the packed parent-node index,
a dependency-table index, and the child's relative translation/uniform scale.
The current `Subtree` package owns one Page and this sidecar; raw Page blob
serialization does not include the sidecar.

`HierarchyBuilder` consumes one arbitrary insertion-order, single-root logical
tree. `splitBelow(node)` defines a natural entity boundary: the renderable node
stays in its parent page as an expansion point, while the builder generates one
detail page containing its descendants. Boundaries may nest. The result is a
`Hierarchy` containing deterministically indexed packed blobs. Each expansion
stores its detail-page id, so the same relationship is not duplicated in a
separate manifest.

Every generated page has one logical root. The root page physically contains
that node. A detail page physically begins with the logical root's children;
index zero acts as their common continuation sentinel. Users author and stream
the single logical root, not this packed multi-root representation.

The generated local detail-page id is stored in otherwise unused expansion
metadata. `SpatialDatabase::detailPage(NodeHandle)` scopes it with the hierarchy's root
asset and can therefore resolve an unambiguous streaming request without a
payload lookup or a map for every mounted page. Payloads remain non-unique
opaque application data.

`PageBuilder` is the low-level escape hatch for a content pipeline that already
owns physical partitioning. It emits one packed blob and may explicitly mark a
leaf with `markExpansion(node, detailPageId)`.

The blob begins with a 64-byte `PageHeader` and contains aligned arrays for:

- 8-lane `WideBlock` child data and packed valid/leaf masks;
- source `AABB` values and opaque payloads;
- preorder `parent`, `subtreeSize`, and packed `meta` words; and
- geometric errors.

Index zero is a sentinel representing the page owner. Real page roots begin at
index one. The sentinel gives the runtime one uniform root-child block and one
coverage summary for both instance roots and attached pages.

The current blob version is 2. It is little-endian and deliberately versioned.
`Page::fromBytes` validates and copies untrusted bytes into aligned owned
storage. `PageView::fromBytes` validates a borrowed blob without taking
ownership; the storage must remain valid and suitably aligned for the lifetime
of every asset registered from it.

Both builders establish these packed-page invariants:

| ID | Invariant | Purpose |
|---|---|---|
| A | `parent[i] < i` | Preorder parent lookup and bottom-up refit |
| B | A subtree occupies `[i, i + subtreeSize[i])` | Constant-time subtree range |
| C | Parent bounds conservatively contain descendants | Sound frustum and distance tests |
| D | Effective geometric error never increases downward | Both frontiers remain antichains |
| E | An expansion point has no local children | One unambiguous refinement source |

`HierarchyBuilder` derives bounds and clamps errors over the complete logical
tree before partitioning, so the same contracts hold across generated
boundaries. An authored parent box acts as a conservative lower bound. Every
leaf must have a non-empty box. Payload uniqueness is not a requirement.

Cross-page invariant C is checked when attaching: the attached page's sentinel
bounds must fit inside the expansion node's authored box. The runtime cannot
grow a shared expansion node during attach without touching every instance, so
content must author conservative expansion bounds.

Cross-page invariant D is applied as a per-mount scalar error ceiling. The page
bytes are never rewritten. The same asset can therefore be mounted under
different expansion points with different effective error ceilings.

### Wide layout

The packed format always uses `kWide == 8`. A normal `WideBlock` is exactly 256
bytes (four 64-byte cache lines) and contains child bounds, errors, and local
indices. Valid- and plain-leaf masks live in a dense side array so the block
does not grow to a five-cache-line access pattern.

AVX2 evaluates eight lanes directly. SSE2 and NEON evaluate the same eight
logical lanes as two four-lane halves. Scalar builds preserve the format and
execute lane loops, so page blobs do not vary by CPU backend.

## 4. Assets, mounts, and sharing

`registerSubtree(Subtree&&)` transfers the immutable page and logical
dependency metadata together. `instantiate(RootNodeDesc, ...)` creates one
permanent renderable TLAS root; it does not accept a `SubtreeHandle`.
`mount()` places a registered definition beneath that root or beneath an
authored nested expansion site and validates transformed child bounds plus the
permanent target key.

Different sites mounting one subtree share page bytes but have distinct owner,
transform, error clamp, residency coverage, and LRU records. Accumulated mount
transforms live in a dense slot-parallel array. Selection transforms the
instance-local camera when it crosses a non-identity mount; page bounds remain
canonical and immutable. Bounds-only COW propagation converts a child page's
sentinel box back into its owner coordinate system at each boundary.

`registerAsset(Page&&)` transfers ownership of a blob to the `SpatialDatabase`.
`registerAsset(PageView)` borrows one. Instancing a registered root asset reuses
one root mount, including its residency and attached child-page graph. Ten
thousand identical trees can therefore share one hierarchy rather than clone
it ten thousand times.

Reusing an asset at unrelated attachment sites can create separate mounts.
Those mounts share immutable bytes but have separate placement-dependent
runtime state such as owner link, error ceiling, residency coverage, and LRU
position. Instances of the same root asset share the same root mount and all
descendants attached beneath it.

The convenience `addInstance(Page&&, ...)` creates an anonymous asset. It is
appropriate for one-off content; repeated content should be registered once
and instanced by `AssetHandle`.

`releaseAsset` drops the database's registered ownership. It is a contract error
while live instances still reference the asset. Anonymous assets disappear
when their last mount and instance reference disappear.

## 5. Residency and topology streaming

Each mounted page stores one 16-bit word per node. Two bits hold `resident`
(the caller has the payload available) and `covered` (that payload or its
descendants provide a complete resident structural cover); nine bits hold the
covered-child count. The authored fanout cap is 511, so the count cannot
overflow. Keeping these together removes one allocation per mount.

Attached-child slot arrays live in a sparse pool and are created only when a
mount actually gains a child. Per-node state comes from geometric, asset-local
slabs rather than one allocation per placement. Consequently a reusable leaf
placement carries no vector headers. `PageRt` is 48 bytes. A slot-parallel
32-byte hot mount record holds its transform, error clamp, generation, and asset id.

Each mount also has two compact 8-byte records outside `PageRt`.
`PageStamp` holds the content version plus the generation/live stamp used by
handles and cached-query dependency checks. Each mount stores its mounted-tree
root slot in `PageRt` tail padding, and descendant residency or topology changes
bump that root version. A cached assembled city therefore validates one exact
tree dependency regardless of its placement count. The residency summary holds a
resident node count and a count of recursively incomplete attached child
mounts. It changes only with residency or topology and propagates upward only
when a mount tree crosses the fully-resident boundary. Selection uses that
proof to take a shared-only traversal that skips residency checks and
current/ideal branching for the common fully-resident case.

Residency changes propagate coverage toward the root. A current-frontier node may
refine whenever more detailed resident nodes completely cover the region
selection needs. The intermediate proxy itself need not be resident: if a
parent `P` and leaf descendants `L` are resident while an intermediate `C` is
not, selection may emit `L` directly. If any needed branch lacks a resident
cover, the nearest resident ancestor remains as `CurrentOnly`. This preserves
replace-only selection without holes or parent/child overlap.

The structural summary is an O(1) early exit. A node entirely inside the
frustum needs full structural coverage; a node crossing a frustum plane may
ignore missing branches that are outside the view, so selection recursively
checks only surviving uncovered branches. Those pages count as used for
optional `PageUsageContext` feedback. Instances touching a frustum plane are
re-walked rather than cached by `SpatialQuery`.

An expansion point is a renderable collapsed proxy whose children live in a
different page. If its error is acceptable it remains a normal ideal-frontier entry.
If it is too coarse and no child page is attached, it appears as a high-error
ideal-side leaf. Attaching a child page makes traversal able to cross the
boundary on the next selection. No per-entry expansion tag is needed:
`SpatialDatabase::detailPage()` reports whether a generated detail page exists, while
the quantized error says whether loading it is currently useful.

Instance-root payloads are pinned resident. This is the base case for the
runtime invariant:

> Every node drawn by the current frontier is resident, and every region refined
> through has a complete resident descendant cover.

Payload residency, page attachment, instance updates, and collection are
single-writer operations. Finish them before `applyUpdates`, then treat the
published SpatialDatabase as read-only until every spatial query has joined.

### Streamer policy

Output order is traversal-defined. It is stable for a given state but is not a
priority order. An attach budget should choose expandable ideal-side entries
(`shared` and `idealOnly`) by descending `errorCode()`, not by vector position.
Call `approximateError(threshold)` when a pixel-scale value is more convenient.

Pure discovery can add one frame of latency per missing page level. A streamer
that owns page metadata can use the observed error to look ahead:

```text
levels ~= ceil(log2(error / threshold) / errorHalvingsPerPage)
```

Keep candidates from all regions in one max-heap, attach the globally worst,
inspect the new page, estimate the next expansion errors, and continue until
the page budget is exhausted. A depth-first lookahead can starve other regions
under fanout, while the global heap applies the budget to the worst remaining
error across the whole view.

## 6. Selection outputs and traversal

`selectFrontier` returns a `FrontierResultView` with three spans:

- `shared` belongs to both the current and ideal frontiers;
- `currentOnly` is a resident fallback needed only by the current frontier; and
- `idealOnly` belongs only to the fully-resident ideal frontier.

Thus `shared + currentOnly` is the hole-free render frontier, while
`shared + idealOnly` is the frontier known topology would choose if every
payload were resident. Keeping membership in the container rather than every
entry avoids two per-entry tags and makes each `FrontierEntry` exactly 12 bytes:

- an 8-byte `NodeHandle`;
- a stable 24-bit public `InstanceId`; and
- an 8-bit threshold-relative screen-error code.

Error codes 0 through 127 mean at or below the query threshold; 128 through
255 mean above it. `overThreshold()` preserves the exact comparison result.
Magnitude is logarithmically quantized at roughly eight steps per octave and
`approximateError(threshold)` decodes an estimate for prioritization. Payloads
are not repeated in the output; use `tryGetPayload(nodeHandle, payload)` or the
caller's external graph when needed.

The spans point into retained storage owned by the `SpatialQuery` and remain valid
until its next selection or reset. `FrontierResult` is an explicit owning result
for the less common case that a frontier must survive another query. There is no
request output. The caller inspects ideal-side entries, tests
residency, deduplicates whatever it considers the same content, and applies its
own IO priorities and budgets. Each member of `FrontierResultSink` targets fixed
caller memory and reports dropped entries, so the caller can grow its capacity
without an allocation inside the traversal.

`applyUpdates` first flushes queued bounds and performs any requested TLAS
build or repair. Selection then proceeds against that stable snapshot:

1. Walk the wide TLAS with tri-state frustum and optional `minPix` contribution
   culling.
2. A TLAS leaf's renderable root record carries permanent payload identity.
   Retest its precise world box and error; if it satisfies the threshold, emit
   the tagged root handle directly into `shared` without touching a page.
3. For a root with no mounted subtree, emit through the exact one-node fast
   path. A compact per-instance marker contains the prepacked handle bits, so
   this path fetches neither normal instance traversal state nor a page stamp.
4. For hierarchical BLASes that did not terminate at the root, transform the
   view into each surviving instance's local space.
5. Walk attached pages with an explicit DFS stack, carrying current- and
   ideal-frontier liveness together. Propagated coverage answers most current-frontier
   descent decisions in O(1); only partially visible uncovered regions need a
   recursive visible-coverage probe. A recursively fully-resident mount tree
   instead takes a specialized path in which every emitted entry is `shared`.
6. Test up to eight children together. Fully outside lanes disappear; fully
   inside lanes clear their remaining plane masks; partial lanes carry only
   undecided planes.
7. Emit plain leaves directly from their parent's wide test. Interior and
   expansion nodes carry error, plane mask, and both membership paths on the
   DFS stack. Shared nodes are emitted once.

There is no required per-node per-query scratch and no per-frame clear over all
materialized nodes. Work is bounded by the visible current/ideal frontier region
rather than the full authored hierarchy:

```text
O(TLAS query + visible instances + visible frontier region + output size)
```

The more compact shorthand `O(log R + hits + frontier region)` assumes a
reasonably separating TLAS. In the adversarial case where all R instances
overlap, the TLAS must report all R and the query is necessarily O(R). No
algorithm can cost less than the output it must emit.

## 7. SpatialQuery state and damping

Every logical camera, shadow cascade, or reflection probe owns a `SpatialQuery`.
`SpatialQuery::selectFrontier` accepts the frame's raw `Camera`, damps it internally, and
queries a published `const SpatialDatabase`. Keeping the damper, reuse records, traversal
scratch, and statistics together prevents state from one camera being paired
with another by mistake.

For a fully-frustum-inside instance, every LOD decision changes only when the
camera envelope or projection scale crosses a recorded flip point. The query
records a conservative margin, transform/content versions, up to two page
dependencies, and the emitted frontier. A later call copies that frontier without walking
the instance only while every proof condition still holds.

Important limits are explicit:

- An instance crossing a frustum plane is always walked because camera
  rotation can change culling independently of camera translation.
- Ordinary selection coalesces every visited page to the mounted-tree root, so
  an assembled city has one dependency. The overload that records exact
  `PageUsageContext` feedback enumerates physical pages; a record needing more
  than two of those is not cached. Streaming does not otherwise disable reuse:
  descendant residency and topology changes bump the root version.
- The emitted node set matches uncached selection exactly. `errorCode()` on a
  reused entry is the recorded quantized value and can be stale within the
  proven margin; use its magnitude for prioritization or dithering, not
  bit-exact comparison with a fresh traversal.
- A cached call walks its camera serially, but different queries can run
  concurrently because all mutable query state is query-owned.

Each per-instance cache record is split into 32 hot bytes and 4 cold bytes,
plus an optional 8-byte second dependency allocated only after a cacheable walk
actually needs two stamps. Counts up to 1,023 per output bucket stay inline;
larger cacheable frontiers allocate one sparse 16-byte full-width count record.
The common hit reads the hot record
and a parallel 4-byte instance-version stream; it does not fetch the 32-byte
instance record. Recorded frontier entries remain separate slab storage.
`reset()` clears logical state and its damping window but retains capacity,
which is appropriate for camera cuts and teleports. `setHalfLife(0)` disables
damping exactly. `setReuseEnabled(false)` disables temporal frontier reuse while
retaining the same API and ownership model.

## 8. Parallel selection and threading

There are two independent forms of parallelism.

For multiple queries, call `applyUpdates()` once, then invoke `SpatialQuery::selectFrontier`
concurrently on the published `const SpatialDatabase`. Every in-flight call
must own a distinct `SpatialQuery`, optional `PageUsageContext`, and output
buffers. Those objects are deliberately unsynchronized; sharing one between
threads is a caller error. Selection mutates only those caller-owned objects.

After all query tasks join, the caller may resume SpatialDatabase mutation and call
`collect`. Passing only important cameras' `PageUsageContext` objects lets a
primary camera influence page retention without giving the same weight to shadow
or probe queries.

For one uncached query, `FrontierContext` supplies aligned allocation and an
optional blocking `parallelFor`. Call `query.setReuseEnabled(false)`, set
`workerCount > 1`, and set
`SpatialDatabaseConfig::parallelInstanceThreshold > 0` to fan that call out once the
visible instance count reaches the threshold.

The runtime gives each worker a contiguous visible-instance range and private
output buffers, then concatenates each bucket in serial order. Serial and
parallel results match entry-for-entry for the same backend and input.

`parallelFor` may run task indices in any order but must return only after all
tasks finish. Its scratch and aggregate statistics live in the `SpatialQuery`, so
distinct uncached queries have the same external concurrency contract as cached
ones. No selection may overlap a SpatialDatabase mutation, another `applyUpdates`, or
`collect`.

## 9. TLAS lifecycle

The top level is an 8-wide dynamic BVH over live placements of independent
renderable roots, including page-free one-node hierarchies. Parallel root
records own payload identity; maintenance arrays store world bounds, maximum
effective error, layer masks, and parent/lane back-pointers separately from the
selection-path instance record. Internal TLAS BVH nodes have no renderable
payload and are never selected. A surviving leaf identifies its renderable root
and optional mounted descendant hierarchy.

The first `applyUpdates` builds the configured quality tier before publishing
the selection snapshot:

- `TlasQuality::BinnedSAH` (default) favors traversal quality;
- `TlasQuality::Median` uses recursive longest-axis median splits; and
- `TlasQuality::Morton` is the cheapest, loosest quality choice.

That first non-empty build also compacts and spatially orders the physical
instance streams. Public `InstanceRef` values and `FrontierEntry::instance()` ids
are mapped separately and remain stable.

After the tree exists, individual adds and removes are applied incrementally in
O(depth). Inserts descend toward the least-growing leaf and use a free lane or
split the leaf; removals invalidate a lane. `tlasEditFraction` bounds how many
such quality-losing edits accumulate before a Morton rebuild. Large population
drift (`tlasCountDrift`) promotes the rebuild to the configured quality tier.

Instance motion updates its lane and grows ancestors only as needed. The escape
budget counts each distinct instance at most once between builds; crossing the
configured fraction can request a fast Morton rebuild without letting a small
bounded cohort consume the budget repeatedly. Sufficient aggregate area growth
can promote a later rebuild to restore quality. These thresholds are exposed as
`tlasEscapeFraction` and `tlasAreaDrift`.

Routine repairs preserve physical instance order. `SpatialDatabase::optimize()` is the
explicit synchronization-point operation that flushes pending bounds, compacts
dead dense slots, performs a quality rebuild, and restores TLAS traversal order
to the physical instance and query-record streams. Call it after disruptive
database changes at a menu, loading screen, teleport, or level transition rather
than as per-frame maintenance.

Morton builds quantize each centroid to 21 bits per axis for a 63-bit key.
Populations of at least 1,024 use a stable LSD radix sort with up to six 11-bit
passes and retained scratch; smaller populations use `std::sort`. The dense
live-instance list makes rebuild enumeration proportional to the current
population rather than the allocated-slot high-water mark.

Quality builds can be a visible level-load cost at hundreds of thousands of
instances. Build the initial database before an interactive frame and keep the
incremental edit path active for steady churn.

## 10. Motion and copy-on-write bounds

`moveInstance` updates translation and uniform scale and refits its TLAS path.
For a persistent fixed-order cohort, `MotionGroup` retains the caller's order
and caches the corresponding physical instance order for `moveInstances`.
Stale references are skipped, duplicate references use the last supplied
position, and the physical-order cache refreshes after a database layout change.

`setNodeBounds(instance, node, localBounds)` deforms one node for one instance.
The instance argument is essential: immutable topology and payload data remain
shared, while bounds become instance-specific.

The first deformation of an `(instance, page mount)` pair copies source bounds
into an overlay. Small-page wide bounds are copied densely. Pages with at least
64 wide blocks instead start with only a block-to-patch table and modified wide
blocks, then promote to dense storage after more than one sixteenth changes.
Further edits reuse the overlay. Propagation across a page boundary creates
overlays only along the path to that instance root; siblings that use the same
asset keep seeing the original bounds. Overlay-reference vector headers live in
a cold pool, so undeformed instances retain only a 32-byte hot record.

Submissions are appended to a flat queue. `applyUpdates`, `nodeBounds`, or an
explicit `flushBounds()` applies them in order. A node's own box ends at the
last submitted value. Ancestors grow conservatively and stop at the first box
that already contains the change; that early-out cheaply coalesces shared
ancestors and repeated moves without a dirty-node table. Grouping submissions
by instance or page preserves overlay locality; the library does not sort them
because sorting adds whole-batch work and discards useful locality already
provided by the caller.

Ancestor page bounds do not currently shrink after deformation. Long-running
large teleports can therefore make culling looser, never incorrect. TLAS
rebuilds re-tighten the top level, but a future page-level re-tightening policy
would be needed to recover tight internal overlay bounds.

Bounds correctness cannot update baked proxy geometry. If a child moves, its
ancestor mesh may be visually stale even though culling stays conservative.
Heavily dynamic objects should normally be separate root instances or use
caller-generated dynamic proxies.

## 11. Garbage collection

Non-pinned attached pages participate in an intrusive database LRU, but selection
never mutates it. `applyUpdates` advances the age epoch. A
`PageUsageContext` records the latest epoch in which its query needed each page
and accumulates that feedback until collection.

`collect(usageContexts, maxAttachedPages, minAge)` first consumes
only the supplied queries' accumulated feedback, then detaches cold leaf mounts
until `streamedPageCount()` is no larger than the requested budget. The
single-`PageUsageContext` and no-feedback overloads are conveniences. Pinned root mounts
do not count because they cannot be collected. A candidate must:

- be non-pinned;
- have no attached child pages; and
- have been untouched for at least `minAge` frames.

`CollectResult::freedPayloads` is a span over SpatialDatabase-owned resident payload
values that became unreachable. It remains valid until the next collection.
Collection works from the cold tail and is proportional to feedback consumed,
candidates examined, and pages detached; it does not scan the entire database.

## 12. Complexity and current limits

| Operation | Expected cost |
|---|---|
| Uncached `SpatialQuery::selectFrontier` | TLAS query + visible current/ideal frontier region + outputs |
| SpatialQuery reuse hit | TLAS query + record validation + copied frontier entries |
| `markResident` / `markNonResident` | O(changed coverage path to the mount root), with early-out |
| `attachPage` | O(page nodes) runtime-state initialization |
| `detachPage` | O(detached mount state); child mounts must be detached first |
| Incremental add / move / remove instance | O(TLAS depth), excluding asset teardown |
| Bounds submission | O(1) queue append |
| `applyUpdates` | O(changed ancestor paths + any scheduled TLAS repair) |
| `optimize` | O(live instances + quality TLAS rebuild + physical reorder) |
| `collect` | O(feedback consumed + cold candidates examined + mounts detached) |

Current integration limits and tuning decisions are:

- page size is content-dependent; hundreds to low thousands of nodes is the
  intended starting range, but boundary frequency and streaming granularity
  should be profiled;
- compact identifiers cap a database at 1,048,575 simultaneously mounted page
  slots, a page at 1,048,576 entries including its sentinel, and the dense
  instance table at 16,777,215 live slots; a mount generation wraps only after
  that same slot has been recycled more than 16 million times;
- cached queries parallelize across calls; one cached call does not fan out
  internally;
- translation plus uniform scale is the built-in instance transform;
- internal deformation needs caller-maintained proxy fidelity and has no
  page-level shrink pass; and
- GC watermarks, dwell, attach budget, and error-priority policy belong to the
  host because they depend on IO latency and content.
