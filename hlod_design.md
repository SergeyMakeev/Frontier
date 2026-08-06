# HLodTree design and runtime contract

This is the current design implemented by `include/hlod` and `src`. Start with
the [README](README.md) for the general overview, a compilable minimal example,
and representative performance numbers. The performance history and rejected
experiments live in [ARCHITECTURE.md](ARCHITECTURE.md).

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
they do not appear in the cut. Consequently the world needs no common
whole-map hierarchy and can freely mix BLASes of very different scopes.

A selection is a ragged antichain through that tree:

```text
                     town
                  /        \
           building A      building B  <- draw the far building proxy
             /  |  \
          wall wall wall                 <- draw nearby walls
```

No selected node is an ancestor of another selected node. Refinement is
replace-only: either a node draws, or its children replace it. This makes a cut
hole-free without parent/child overdraw.

The library is deliberately external to rendering. A node stores an opaque
`uint64_t UserPayload`; selection carries a compact node handle and the caller
may resolve that payload with `World::tryGetPayload` only when needed. A
payload can be a mesh-table index, entity key, pointer-sized token, or any
other value. Duplicate payloads are legal. Every node must nevertheless have a
renderable representation because any node can become part of the current cut.

The projected error test is conceptually:

```text
screenErrorPx = geometricError * projectionScale / distance(camera, bounds)
refine when screenErrorPx > CutParams::threshold
```

Distance is measured to the node box, not its center. The camera constructors
compute `projectionScale` from vertical FOV and viewport height. A `View` uses
its internal `CameraDamper` to replace the camera point with a decaying camera
envelope, providing LOD hysteresis without sticky state on every node.

## 2. Public objects and handles

The runtime has four distinct concepts:

- A `Page` or `PageView` is an immutable, packed BLAS fragment.
- An `AssetHandle` names page bytes registered with a `World` for reuse. Here
  *asset* is an API/storage term and does not mean that the BLAS represents one
  object.
- A page mount places an asset at an instance root or below an expansion point;
  a `PageHandle` names that mount.
- An instance applies a translation and positive uniform scale to one
  independent BLAS root. `World::InstanceRef` also contains a generation and
  its root page handle.

`NodeHandle{slot, index, generation}` names a node in a mounted page and packs
those fields into 64 bits: 20 bits each for mount slot and page-local index,
plus a 24-bit per-slot generation.
`nodeAt(page, index)` composes one from a `PageHandle` and a packed page-local
index. Generations prevent ABA bugs when page or instance slots are recycled.
An asynchronous completion using a handle whose page was collected is an
expected race: mutating calls ignore it and queries report it absent.

The `World` keeps no payload-to-node index and no hash map. Persistent systems
should retain handles from `CutEntry`, `InstanceRef::rootPage`, or
`attachPage`. If a content pipeline needs a payload-to-packed-index table, it
owns that table outside the library.

`InstanceDesc` contains:

- `pos` and positive uniform `scale`;
- `mask`, ANDed with `Camera::viewMask` for cheap layer filtering.

`World::addInstance` returns the generation-stamped owner reference. Selection
packs its dense 24-bit `InstanceId` into each `CutEntry`; callers normally use
that id to index the same placement/transform table that stores the returned
`InstanceRef`.

Rotation and non-uniform scale are not represented by `InstanceDesc`. Bake
them into authored bounds/proxies or place such objects in an integration layer
that presents translation plus uniform scale to HLodTree.

### Typical runtime flow

The following uses the actual API. Page loading is application-specific and is
shown as placeholders:

```cpp
World world;
AssetHandle tree = world.registerAsset(loadRootPage());

InstanceDesc desc;
desc.pos = float4::point(100, 0, 20);
desc.scale = 1.0f;
World::InstanceRef instance = world.addInstance(tree, desc);

CutResults cut;
Camera camera = makeLookAtCamera(eye, target);
View view;                         // persistent state for this camera
PageUsageContext primaryUsage;    // only retention-relevant views need one

world.applyUpdates();              // apply changes and publish the read-only world
const World& published = world;
view.selectCut(published, camera, CutParams{4.0f, 0.0f}, primaryUsage, cut);

const auto render = [&](const CutEntry& entry)
{
    UserPayload payload;
    if (published.tryGetPayload(entry.nodeHandle, payload))
        submit(payload, transforms[entry.instance()]);
};
for (const CutEntry& entry : cut.shared) render(entry);
for (const CutEntry& entry : cut.currentOnly) render(entry);

// After all view queries join, streaming and World mutation are serial again.
const auto streamIdeal = [&](const CutEntry& entry)
{
    UserPayload payload;
    if (!world.tryGetPayload(entry.nodeHandle, payload)) return;

    if (entry.overThreshold() && contentGraph.hasChildren(payload) &&
        !world.isAttached(entry.nodeHandle))
        world.attachPage(entry.nodeHandle, loadChildPage(payload));
    else if (!world.isResident(entry.nodeHandle) &&
             payloadFinishedLoading(payload))
        world.markResident(entry.nodeHandle);
};
for (const CutEntry& entry : cut.shared) streamIdeal(entry);
for (const CutEntry& entry : cut.idealOnly) streamIdeal(entry);

world.collect(primaryUsage, pageBudget, minPageAge);
```

In a real asynchronous streamer, completions normally arrive in later frames.
It normally deduplicates content identities and applies IO/page budgets before
scheduling work; `World` deliberately does neither. The loop above shows the
state transitions, not a production scheduler. The caller's content graph is
also what distinguishes a high-error terminal leaf from an expandable leaf.

The generation check is why no extra page-lifetime lock is needed at completion
time. A stale `attachPage` returns an invalid `PageHandle` and leaves the caller
free to discard or cache the loaded page.

## 3. Page format and builder invariants

`HLodBuilder` consumes an arbitrary insertion-order authoring tree and emits
one packed page. The content pipeline decides page boundaries and marks a leaf
with `markExpansion(node)` when its children live in another page.

The blob begins with a 64-byte `PageHeader` and contains aligned arrays for:

- 8-lane `WideBlock` child data and packed valid/leaf masks;
- source `AABB` values and opaque payloads;
- preorder `parent`, `subtreeSize`, and packed `meta` words; and
- geometric errors.

Index zero is a sentinel representing the page owner. Real page roots begin at
index one. The sentinel gives the runtime one uniform root-child block and one
coverage summary for both instance roots and attached pages.

The current blob version is 2. It is little-endian and deliberately versioned;
version 1 is not compatible because lane masks moved out of `WideBlock`.
`Page::fromBytes` validates and copies untrusted bytes into aligned owned
storage. `PageView::fromBytes` validates a borrowed blob without taking
ownership; the storage must remain valid and suitably aligned for the lifetime
of every asset registered from it.

`HLodBuilder::build()` establishes these invariants:

| ID | Invariant | Purpose |
|---|---|---|
| A | `parent[i] < i` | Preorder parent lookup and bottom-up refit |
| B | A subtree occupies `[i, i + subtreeSize[i])` | Constant-time subtree range |
| C | Parent bounds conservatively contain descendants | Sound frustum and distance tests |
| D | Effective geometric error never increases downward | Both cuts remain antichains |
| E | An expansion point has no local children | One unambiguous refinement source |

The builder derives parent bounds by unioning children; an authored parent box
acts as a conservative lower bound. Every leaf must have a non-empty box.
Errors that exceed the parent are clamped downward. Payload uniqueness is not a
requirement.

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

`registerAsset(Page&&)` transfers ownership of a blob to the `World`.
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

`releaseAsset` drops the world's registered ownership. It is a contract error
while live instances still reference the asset. Anonymous assets disappear
when their last mount and instance reference disappear.

## 5. Residency and topology streaming

Each mounted page has compact runtime arrays parallel to its nodes:

- one byte holds both `resident` (the caller has the payload available) and
  `covered` (that payload or its descendants provide a complete resident
  structural cover); and
- one 16-bit `coveredChildren` count incrementally maintains that summary from
  immediate children, including attached child pages. The authored fanout cap
  is 511, so the count cannot overflow.

Each mount also has two compact 8-byte records outside the 104-byte `PageRt`.
`PageStamp` holds the content version plus the generation/live stamp used by
handles and cached-view dependency checks. The residency summary holds a
resident node count and a count of recursively incomplete attached child
mounts. It changes only with residency or topology and propagates upward only
when a mount tree crosses the fully-resident boundary. Selection uses that
proof to take a shared-only traversal that skips residency checks and
current/ideal branching for the common fully-resident case.

Residency changes propagate coverage toward the root. A current-cut node may
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
re-walked rather than cached by `View`.

An expansion point is a renderable collapsed proxy whose children live in a
different page. If its error is acceptable it remains a normal ideal-cut entry.
If it is too coarse and no child page is attached, it appears as a high-error
ideal-side leaf. Attaching a child page makes traversal able to cross the
boundary on the next cut. No per-entry expansion tag is needed: the caller's
external content graph says whether that node has another page, while the
quantized error says whether expansion is currently useful.

Instance-root payloads are pinned resident. This is the base case for the
runtime invariant:

> Every node drawn by the current cut is resident, and every region refined
> through has a complete resident descendant cover.

Payload residency, page attachment, instance updates, and collection are
single-writer operations. Finish them before `applyUpdates`, then treat the
published World as read-only until every view selection has joined.

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
under fanout. The measured global-heap policy reduced worst residual error two
frames after a teleport by about 120x at the same page budget; see experiment N
in [ARCHITECTURE.md](ARCHITECTURE.md).

## 6. Selection outputs and traversal

`selectCut` fills a `CutResults` with three vectors:

- `shared` belongs to both the current and ideal cuts;
- `currentOnly` is a resident fallback needed only by the current cut; and
- `idealOnly` belongs only to the fully-resident ideal cut.

Thus `shared + currentOnly` is the hole-free render cut, while
`shared + idealOnly` is the frontier known topology would choose if every
payload were resident. Keeping membership in the container rather than every
entry avoids two per-entry tags and makes each `CutEntry` exactly 12 bytes:

- an 8-byte `NodeHandle`;
- a dense 24-bit `InstanceId`; and
- an 8-bit threshold-relative screen-error code.

Error codes 0 through 127 mean at or below the query threshold; 128 through
255 mean above it. `overThreshold()` preserves the exact comparison result.
Magnitude is logarithmically quantized at roughly eight steps per octave and
`approximateError(threshold)` decodes an estimate for prioritization. Payloads
are not repeated in the output; use `tryGetPayload(nodeHandle, payload)` or the
caller's external graph when needed.

There is no request output. The caller inspects ideal-side entries, tests
residency, deduplicates whatever it considers the same content, and applies its
own IO priorities and budgets. Each member of `CutResultSink` can target a
growable `std::vector` or fixed caller memory; a fixed sink reports dropped
entries, so the caller can grow its capacity without an allocation inside the
traversal. Vector-backed sinks may grow like any vector.

`applyUpdates` first flushes queued bounds and performs any requested TLAS
build or repair. Selection then proceeds against that stable snapshot:

1. Walk the wide TLAS with tri-state frustum and optional `minPix` contribution
   culling.
2. A TLAS leaf compactly marks a hierarchical BLAS whose root page has one
   renderable root. When a vector error test says that root may satisfy the
   threshold, retest its precise world box and error and emit the pinned root
   directly into `shared`. Uncached views enable this query work adaptively;
   cached views do the exact root test only for cache misses.
3. For an exact one-node BLAS, retest its precise world box and emit its pinned
   root directly into `shared`. A compact per-instance marker identifies this
   case without fetching the normal instance/page traversal state.
4. For hierarchical BLASes that did not terminate at the root, transform the
   view into each surviving instance's local space.
5. Walk attached pages with an explicit DFS stack, carrying current- and
   ideal-cut liveness together. Propagated coverage answers most current-cut
   descent decisions in O(1); only partially visible uncovered regions need a
   recursive visible-coverage probe. A recursively fully-resident mount tree
   instead takes a specialized path in which every emitted entry is `shared`.
6. Test up to eight children together. Fully outside lanes disappear; fully
   inside lanes clear their remaining plane masks; partial lanes carry only
   undecided planes.
7. Emit plain leaves directly from their parent's wide test. Interior and
   expansion nodes carry error, plane mask, and both membership paths on the
   DFS stack. Shared nodes are emitted once.

There is no required per-node per-view scratch and no per-frame clear over all
materialized nodes. Work is bounded by the visible current/ideal cut region
rather than the full authored hierarchy:

```text
O(TLAS query + visible instances + visible cut region + output size)
```

The more compact shorthand `O(log R + hits + cut region)` assumes a
reasonably separating TLAS. In the adversarial case where all R instances
overlap, the TLAS must report all R and the query is necessarily O(R). No
algorithm can cost less than the output it must emit.

## 7. View state and damping

Every logical camera, shadow cascade, or reflection probe owns a `View`.
`View::selectCut` accepts the frame's raw `Camera`, damps it internally, and
queries a published `const World`. Keeping the damper, reuse records, traversal
scratch, and statistics together prevents state from one camera being paired
with another by mistake.

For a fully-frustum-inside instance, every LOD decision changes only when the
camera envelope or projection scale crosses a recorded flip point. The view
records a conservative margin, transform/content versions, up to two page
dependencies, and the emitted cut. A later call copies that cut without walking
the instance only while every proof condition still holds.

Important limits are explicit:

- An instance crossing a frustum plane is always walked because camera
  rotation can change culling independently of camera translation.
- An instance with more than two distinct page dependencies is not cached.
  Streaming does not otherwise disable reuse: residency and topology versions
  invalidate records that depend on changed pages.
- The emitted node set matches uncached selection exactly. `errorCode()` on a
  reused entry is the recorded quantized value and can be stale within the
  proven margin; use its magnitude for prioritization or dithering, not
  bit-exact comparison with a freshly walked view.
- A cached call walks its camera serially, but different views can run
  concurrently because all mutable query state is view-owned.

Each per-instance cache record is split into 32 hot bytes and 4 cold bytes,
plus an optional 8-byte second-page dependency allocated only after a
cacheable walk actually touches two pages. The common hit reads the hot record
and a parallel 4-byte instance-version stream; it does not fetch the 32-byte
instance record. Recorded cut entries remain separate slab storage.
`reset()` clears logical state and its damping window but retains capacity,
which is appropriate for camera cuts and teleports. `setHalfLife(0)` disables
damping exactly. `setReuseEnabled(false)` disables temporal cut reuse while
retaining the same API and ownership model.

## 8. Parallel selection and threading

There are two independent forms of parallelism.

For multiple views, call `applyUpdates()` once, then invoke `View::selectCut`
concurrently on the published `const World`. Every in-flight call
must own a distinct `View`, optional `PageUsageContext`, and output
buffers. Those objects are deliberately unsynchronized; sharing one between
threads is a caller error. Selection mutates only those caller-owned objects.

After all view tasks join, the caller may resume World mutation and call
`collect`. Passing only important cameras' `PageUsageContext` objects lets a
primary view influence page retention without giving the same weight to shadow
or probe views.

For one uncached view, `HlodContext` supplies aligned allocation and an
optional blocking `parallelFor`. Call `view.setReuseEnabled(false)`, set
`workerCount > 1`, and set
`WorldConfig::parallelInstanceThreshold > 0` to fan that call out once the
visible instance count reaches the threshold.

The runtime gives each worker a contiguous visible-instance range and private
output buffers, then concatenates each bucket in serial order. Serial and
parallel results match entry-for-entry for the same backend and input.

`parallelFor` may run task indices in any order but must return only after all
tasks finish. Its scratch and aggregate statistics live in the `View`, so
distinct uncached views have the same external concurrency contract as cached
ones. No selection may overlap a World mutation, another `applyUpdates`, or
`collect`.

## 9. TLAS lifecycle

The top level is an 8-wide dynamic BVH over live placements of independent
BLAS roots, including one-node BLASes. It stores world bounds, maximum
effective error, layer masks, and parent/lane back-pointers in maintenance
arrays separate from the cut-path instance record. TLAS nodes have no
renderable payload and are never selected; after coarse culling, a surviving
leaf identifies the BLAS instance whose renderable hierarchy is evaluated.

The first `applyUpdates` builds the configured quality tier before publishing
the selection snapshot:

- `TlasQuality::BinnedSAH` (default) favors traversal quality;
- `TlasQuality::Median` uses recursive longest-axis median splits; and
- `TlasQuality::Morton` is the cheapest, loosest quality choice.

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

Morton builds quantize each centroid to 21 bits per axis for a 63-bit key.
Populations of at least 1,024 use a stable LSD radix sort with up to six 11-bit
passes and retained scratch; smaller populations use `std::sort`. The dense
live-instance list makes rebuild enumeration proportional to the current
population rather than the historical slot high-water mark.

Quality builds can be a visible level-load cost at hundreds of thousands of
instances. Build the world before an interactive frame, keep the incremental
edit path active for steady churn, and use the measured first-cut numbers in
the README when budgeting a loading transition.

## 10. Motion and copy-on-write bounds

`moveInstance` updates translation and uniform scale and refits its TLAS path.
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
because the measured sort cost exceeded the recovered locality.

Ancestor page bounds do not currently shrink after deformation. Long-running
large teleports can therefore make culling looser, never incorrect. TLAS
rebuilds re-tighten the top level, but a future page-level re-tightening policy
would be needed to recover tight internal overlay bounds.

Bounds correctness cannot update baked proxy geometry. If a child moves, its
ancestor mesh may be visually stale even though culling stays conservative.
Heavily dynamic objects should normally be separate root instances or use
caller-generated dynamic proxies.

## 11. Garbage collection

Non-pinned attached pages participate in an intrusive world LRU, but selection
never mutates it. `applyUpdates` advances the age epoch. A
`PageUsageContext` records the latest epoch in which its view needed each page
and accumulates that feedback until collection.

`collect(usageContexts, maxAttachedPages, minAge, freedPayloads)` first consumes
only the supplied views' accumulated feedback, then detaches cold leaf mounts
until `streamedPageCount()` is no larger than the requested budget. The
single-`PageUsageContext` and no-feedback overloads are conveniences. Pinned root mounts
do not count because they cannot be collected. A candidate must:

- be non-pinned;
- have no attached child pages; and
- have been untouched for at least `minAge` frames.

The optional output receives resident payload values that became unreachable.
Collection works from the cold tail and is proportional to feedback consumed,
candidates examined, and pages detached; it does not scan the entire world.

## 12. Complexity and current limits

| Operation | Expected cost |
|---|---|
| Uncached `View::selectCut` | TLAS query + visible current/ideal cut region + outputs |
| View reuse hit | TLAS query + record validation + copied cut entries |
| `markResident` / `markNonResident` | O(changed coverage path to the mount root), with early-out |
| `attachPage` | O(page nodes) runtime-state initialization |
| `detachPage` | O(detached mount state); child mounts must be detached first |
| Incremental add / move / remove instance | O(TLAS depth), excluding asset teardown |
| Bounds submission | O(1) queue append |
| `applyUpdates` | O(changed ancestor paths + any scheduled TLAS repair) |
| `collect` | O(feedback consumed + cold candidates examined + mounts detached) |

Current integration limits and tuning decisions are:

- page size is content-dependent; hundreds to low thousands of nodes is the
  intended starting range, but boundary frequency and streaming granularity
  should be profiled;
- compact identifiers cap a world at 1,048,575 simultaneously mounted page
  slots, a page at 1,048,576 entries including its sentinel, and the dense
  instance table at 16,777,215 live slots; a mount generation wraps only after
  that same slot has been recycled more than 16 million times;
- cached views parallelize across calls; parallelizing within one cached call
  remains unnecessary for the measured multi-view workload;
- translation plus uniform scale is the built-in instance transform;
- internal deformation needs caller-maintained proxy fidelity and has no
  page-level shrink pass; and
- GC watermarks, dwell, attach budget, and error-priority policy belong to the
  host because they depend on IO latency and content.
