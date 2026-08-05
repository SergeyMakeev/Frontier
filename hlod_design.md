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
`uint64_t UserPayload`; selection echoes it without interpreting or indexing
it. It can be a mesh-table index, an entity key, a pointer-sized token, or any
other value. Duplicate payloads are legal. Every node must nevertheless have a
renderable representation because any node can become part of the actual cut.

The projected error test is conceptually:

```text
screenErrorPx = geometricError * projectionScale / distance(camera, bounds)
refine when screenErrorPx > CutParams::threshold
```

Distance is measured to the node box, not its center. The view constructors
compute `projectionScale` from vertical FOV and viewport height. A
`ViewDamper` or `SelectionContext` replaces the camera point with a decaying
camera envelope to provide LOD hysteresis without storing sticky state on every
node.

## 2. Public objects and handles

The runtime has four distinct concepts:

- A `Page` or `PageView` is an immutable, packed hierarchy fragment.
- An `AssetHandle` names page bytes registered with a `World` for reuse.
- A page mount places an asset at an instance root or below an expansion point;
  a `PageHandle` names that mount.
- An instance applies a translation and positive uniform scale to one root
  asset. `World::InstanceRef` also contains a generation and its root page
  handle.

`NodeHandle{slot, index, generation}` names a node in a mounted page.
`nodeAt(page, index)` composes one from a `PageHandle` and a packed page-local
index. Generations prevent ABA bugs when page or instance slots are recycled.
An asynchronous completion using a handle whose page was collected is an
expected race: mutating calls ignore it and queries report it absent.

The `World` keeps no payload-to-node index and no hash map. Persistent systems
should retain handles from `LoadRequest`, `IdealEntry`, `InstanceRef::rootPage`,
or `attachPage`. If a content pipeline needs a payload-to-packed-index table,
it owns that table outside the library.

`InstanceDesc` contains:

- `pos` and positive uniform `scale`;
- `tag`, echoed as `CutEntry::instance` and `IdealEntry::instance`; and
- `mask`, ANDed with `CullView::viewMask` for cheap layer filtering.

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
desc.tag = entityIndex;
World::InstanceRef instance = world.addInstance(tree, desc);

std::vector<CutEntry> cut;
std::vector<IdealEntry> ideal;
std::vector<LoadRequest> requests;

world.beginFrame();
world.selectCut(view, CutParams{4.0f, 0.0f}, cut, &ideal, &requests);

for (const LoadRequest& request : requests)
    if (payloadFinishedLoading(request.payload))
        world.markResident(request.node);

for (const IdealEntry& entry : ideal)
    if (entry.tag == IdealTag::NeedsExpansion)
        world.attachPage(entry.node, loadChildPage(entry.payload));
```

In a real asynchronous streamer, completions normally arrive in later frames.
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
readiness counter for both instance roots and attached pages.

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
runtime state such as owner link, error ceiling, residency counters, and LRU
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

- `resident[i]` says the caller has the payload available; and
- `readyChildren[i]` counts resident immediate children.

The actual cut uses all-or-nothing refinement. A node refines only when its
children exist and all immediate child payloads are resident. Otherwise that
node remains in the actual cut and missing children are requested. This avoids
both holes and parent/child overlap.

An expansion point is a renderable collapsed proxy whose children live in a
different page. If its error is acceptable it remains a normal ideal-cut entry.
If it is too coarse and no child page is attached, it is emitted as
`IdealTag::NeedsExpansion`. Attaching a child page makes traversal able to cross
the boundary on the next cut.

Instance-root payloads are pinned resident. This is the base case for the
runtime invariant:

> A node that is drawn, or refined through, is resident and materialized.

Payload residency and page attachment should be mutated between external
selection calls. The `World` is a single-writer object; it does not make
concurrent public mutations safe.

### Streamer policy

Output order is traversal-defined. It is stable for a given state but is not a
priority order. An attach budget must choose `NeedsExpansion` entries by
descending `IdealEntry::err`, not by vector position.

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
in experiment N of [ARCHITECTURE.md](ARCHITECTURE.md).

## 6. Selection outputs and traversal

`selectCut` can fill three sinks in one traversal:

- `CutEntry{payload, err, instance}` is the actual cut to draw now.
- `IdealEntry{payload, node, err, instance, tag}` is the cut if all payloads in
  known topology were resident. `NeedsExpansion` stands in for unknown
  descendants.
- `LoadRequest{payload, node, priority}` names missing immediate-frontier
  payloads. Requests have no instance tag because shared-asset residency is not
  instance-specific.

The ideal cut and request sink are optional. If they are null, no entries are
emitted for them. `Sink<T>` can target a growable `std::vector` or fixed caller
memory; a fixed sink reports dropped entries, so the caller can grow its
capacity without an allocation inside the traversal. Vector-backed sinks may
grow like any vector.

Selection proceeds as follows:

1. Flush pending bounds changes.
2. Rebuild the TLAS if a configured quality/escape/edit budget requested it.
3. Walk the wide TLAS with tri-state frustum and optional `minPix` contribution
   culling.
4. Transform the view into each surviving instance's local space.
5. Walk attached pages with an explicit DFS stack, descending only while the
   ideal decision wants refinement.
6. Test up to eight children together. Fully outside lanes disappear; fully
   inside lanes clear their remaining plane masks; partial lanes carry only
   undecided planes.
7. Emit plain leaves directly from their parent's wide test. Interior and
   expansion nodes carry error, plane mask, and actual-cut liveness on the DFS
   stack.

There is no required per-node per-view scratch and no per-frame clear over all
materialized nodes. Work is bounded by the visible ideal-cut region rather than
the full authored hierarchy:

```text
O(TLAS query + visible instances + visible ideal-cut region + output size)
```

The more compact shorthand `O(log R + hits + ideal-cut region)` assumes a
reasonably separating TLAS. In the adversarial case where all R instances
overlap, the TLAS must report all R and the query is necessarily O(R). No
algorithm can cost less than the output it must emit.

## 7. Stateful selection and damping

Stateless selection accepts an already prepared `CullView`. Callers that want
only damping can keep one `ViewDamper` per view and pass `damper.damp(rawView)`.

`SelectionContext` combines damping with conservative cut reuse. It is also
one object per view. Pass the raw view to its overload; the context damps it
internally so its validity odometers and the query envelope cannot be paired
with different cameras by mistake.

For a fully-frustum-inside instance, every LOD decision changes only when the
camera envelope or projection scale crosses a recorded flip point. The context
records a conservative margin, transform/content versions, up to two page
dependencies, and the emitted cut. A later call copies that cut without walking
the instance only while every proof condition still holds.

Important limits are explicit:

- An instance crossing a frustum plane is always walked because camera
  rotation can change culling independently of camera translation.
- An instance with pending streaming requests or more than two page
  dependencies is not cached.
- Requesting the ideal cut bypasses reuse for that call.
- The emitted node set matches stateless selection exactly. `CutEntry::err` on
  a reused entry is the recorded value and can be stale within the proven
  margin; use it for prioritization or dithering, not bit-exact comparison.
- Contextual selection is serial in the current implementation.

Each per-instance record is 48 bytes plus storage for recorded cut entries.
`reset()` clears logical state and its damping window but retains capacity,
which is appropriate for camera cuts and teleports. `setHalfLife(0)` disables
damping exactly.

## 8. Parallel selection and threading

`HlodContext` supplies aligned allocation and an optional blocking
`parallelFor`. Set `workerCount > 1` and
`WorldConfig::parallelInstanceThreshold > 0` to enable stateless parallel
selection once the visible instance count reaches that threshold.

The runtime gives each worker a contiguous visible-instance range and private
output buffers. It concatenates ranges in serial order, then deduplicates load
requests while preserving the first occurrence and maximum priority. Serial
and parallel cuts, ideal cuts, and request ordering are intended to match for
the same backend and input.

`parallelFor` may run task indices in any order but must return only after all
tasks finish. The `World` itself remains single-writer: do not invoke public
operations or multiple selections concurrently on the same world. Internal
parallelism is owned and joined by one calling thread.

## 9. TLAS lifecycle

The top level is an 8-wide dynamic BVH over live instances. It stores world
bounds, maximum effective error, layer masks, and parent/lane back-pointers in
maintenance arrays separate from the cut-path instance record.

The first query builds the configured quality tier:

- `TlasQuality::BinnedSAH` (default) favors traversal quality;
- `TlasQuality::Median` uses recursive longest-axis median splits; and
- `TlasQuality::Morton` is the cheapest, loosest quality choice.

After the tree exists, individual adds and removes are applied incrementally in
O(depth). Inserts descend toward the least-growing leaf and use a free lane or
split the leaf; removals invalidate a lane. `tlasEditFraction` bounds how many
such quality-losing edits accumulate before a Morton rebuild. Large population
drift (`tlasCountDrift`) promotes the rebuild to the configured quality tier.

Instance motion updates its lane and grows ancestors only as needed. Escape
count can request a fast Morton rebuild; sufficient aggregate area growth can
promote a later rebuild to restore quality. These thresholds are exposed as
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

The first deformation of an `(instance, page mount)` pair copies only source
and wide bounds into an overlay. Further edits reuse it. Propagation across a
page boundary creates overlays only along the path to that instance root;
siblings that use the same asset keep seeing the original bounds.

Submissions are appended to a flat queue. The next `selectCut`, `nodeBounds`,
or explicit `flushBounds()` applies them in order. A node's own box ends at the
last submitted value. Ancestors grow conservatively and stop at the first box
that already contains the change; that early-out cheaply coalesces shared
ancestors and repeated moves without a dirty-node table.

Ancestor page bounds do not currently shrink after deformation. Long-running
large teleports can therefore make culling looser, never incorrect. TLAS
rebuilds re-tighten the top level, but a future page-level re-tightening policy
would be needed to recover tight internal overlay bounds.

Bounds correctness cannot update baked proxy geometry. If a child moves, its
ancestor mesh may be visually stale even though culling stays conservative.
Heavily dynamic objects should normally be separate root instances or use
caller-generated dynamic proxies.

## 11. Garbage collection

Non-pinned attached pages participate in one intrusive world LRU. `beginFrame`
advances the age clock. A page touched repeatedly in one frame is linked only
once; all views contribute to the same recency because any view needing the
page should keep it alive.

`collect(maxAttachedPages, minAge, freedPayloads)` detaches cold leaf mounts
until `streamedPageCount()` is no larger than the requested budget. Pinned root
mounts do not count because they cannot be collected. A candidate must:

- be non-pinned;
- have no attached child pages; and
- have been untouched for at least `minAge` frames.

The optional output receives resident payload values that became unreachable.
Collection works from the cold tail and is proportional to candidates examined
and pages detached; it does not scan the entire world.

## 12. Complexity and current limits

| Operation | Expected cost |
|---|---|
| Stateless `selectCut` | TLAS query + visible ideal-cut region + outputs |
| Context hit | TLAS query + record validation + copied cut entries |
| `markResident` / `markNonResident` | O(1) |
| `attachPage` | O(page nodes) runtime-state initialization |
| `detachPage` | O(detached mount state); child mounts must be detached first |
| Incremental add / move / remove instance | O(TLAS depth), excluding asset teardown |
| Bounds submission | O(1) queue append |
| Bounds flush | O(changed ancestor paths), with containment early-outs |
| `collect` | O(cold candidates examined + mounts detached) |

Current integration limits and tuning decisions are:

- page size is content-dependent; hundreds to low thousands of nodes is the
  intended starting range, but boundary frequency and streaming granularity
  should be profiled;
- `SelectionContext` reuse and parallel instance selection are separate paths;
- translation plus uniform scale is the built-in instance transform;
- internal deformation needs caller-maintained proxy fidelity and has no
  page-level shrink pass; and
- GC watermarks, dwell, attach budget, and error-priority policy belong to the
  host because they depend on IO latency and content.
