# Frontier maintainer onboarding

This is the implementation guide for engineers changing Frontier's hot paths.
It describes the checked-in design, not a generic BVH or LOD algorithm. Public
usage is covered by [API.md](API.md), exact signatures by
[API_REFERENCE.md](API_REFERENCE.md), and behavioral contracts by
[frontier_design.md](frontier_design.md). This document concentrates on the
algorithm, data representation, fast paths, cache proofs, and the assumptions
that a performance change must preserve.

## 1. The short version

Frontier selects a renderable **cut** through a forest of level-of-detail
trees. Every top-level instance has one permanent, always-renderable root in a
dynamic world-space TLAS. Optional immutable descendant forests are registered
once and mounted below those roots or below mountable descendant nodes.

For every visible node, the selection criterion is approximately:

```cpp
effectiveError = min(authoredError, inheritedMountErrorCeiling);
pixelError = effectiveError * camera.k /
             distance(cameraDampingEnvelope, nodeBounds);
refine = pixelError > params.threshold;
```

The traversal returns two cuts at once:

```text
current = shared + currentOnly   // ready, hole-free renderable cover
ideal   = shared + idealOnly     // desired cut if all known nodes were ready
```

The most important implementation facts are:

- The TLAS and serialized subtree blocks are wide SoA BVHs. One SIMD operation
  tests four or eight child boxes.
- Registered subtree bytes are already the traversal representation. There is
  no deserialization and no per-placement copy of topology, bounds, errors, or
  payloads.
- Preorder node numbering, contiguous subtree extents, packed parent ordinals,
  and lane masks remove pointer-heavy semantic objects from the walk.
- A narrowed six-bit frustum mask travels down the tree. Mask zero means the
  entire branch is inside and no descendant plane tests are needed.
- The common fully-ready walk is a separate template instantiation that does
  not read readiness state per node.
- Plain leaves are emitted in `wideVisit()` without becoming scalar DFS work.
- Flat TLAS roots, zero-error roots, root-leaf definitions, fully refined
  definitions, and render-native terminal ranges have progressively cheaper
  paths.
- A `SpatialQuery` can retain an exact per-instance cut. A geometric validity
  margin plus camera, projection, instance-motion, epoch, and mounted-content
  stamps proves when the node set cannot have changed.
- The view-returning API can reuse the entire already-materialized answer
  without probing per-instance records. A separate two-entry exact-view memo
  handles recurring undamped views.
- Mutation is single-writer and deferred to `applyUpdates()`. Small motion uses
  grow-only propagation; large motion streams one exact bottom-up refit; a
  rigid translation of the whole population is represented by one offset.
- Hot/cold splitting and packed records are deliberate. Several `static_assert`
  size checks are performance contracts, not cosmetic assertions.

When changing selection, first ask which proof or specialization your change
invalidates. Most regressions in this code would come from making a common
path fetch one more stream, widening one indexed record, or silently making a
cache certificate incomplete.

## 2. Object model and terminology

The runtime has two spatial levels:

```text
world-space wide TLAS
  └─ permanent renderable instance root
       └─ mounted immutable definition placement (optional)
            ├─ ordinary local descendants
            └─ mounted definition placement below a mountable leaf (optional)
```

The important distinctions are:

| Term | Meaning |
|---|---|
| Definition | One registered immutable `SubtreeBytes` allocation. Definitions form an application-managed DAG and can be shared by many placements. |
| Placement or mount | One runtime use of a definition in one instance tree. Placements form trees and own transforms, coverage, child links, and stamps. |
| TLAS root | The permanent renderable root of one top-level instance. It is always ready and is a valid fallback even when nothing is mounted. |
| Plain leaf | A definition node with no local children and no mount point. It is terminal and needs no LOD decision below it. |
| Mountable node | A local leaf that may acquire one mounted definition. It remains a renderable proxy when no child is mounted. |
| Ideal cut | The geometric LOD choice assuming all known definition nodes are renderable. |
| Current cut | A ready, hole-free cover produced from the readiness actually published. |
| Coverage | Derived state saying a node is ready itself or has a complete ready descendant cover. It is not the same as readiness. |
| Fully ready | Every definition node and every mounted descendant below a placement is ready. |

Payload values are deliberately not identities. Readiness is indexed by
`(registered definition, node index)`. Two equal `UserPayload` values do not
couple nodes or definitions.

## 3. Frame and ownership model

The intended lifecycle is:

1. A single writer creates/removes instances, mounts/unmounts definitions,
   publishes readiness, submits transforms, or queues deformed bounds.
2. `SpatialDatabase::applyUpdates()` flushes bound overlays and instance
   motion, repairs or rebuilds the TLAS as needed, and publishes a stable
   snapshot.
3. Any number of readers select concurrently from that snapshot, but every
   reader has its own `SpatialQuery` or `TerminalRenderQuery`.
4. The caller consumes the returned query-owned views.
5. All readers finish before the next writer phase or collection pass.

Selection is a pure read of `SpatialDatabase`; damping, caches, scratch stacks,
output storage, optional statistics, and optional mount-use feedback all live
in the query. A query binds to the first database it reads until `reset()`.

Returned views are non-owning. They remain valid only until the same query is
selected or reset again. `FrontierResult` is the explicit owning alternative.

## 4. Representation and memory layout

### 4.1 Serialized definitions are executable traversal data

`SubtreeBuilder` produces one 64-byte-aligned `SubtreeBytes` allocation in this
order:

```text
128-byte SubtreeHeader
WideBlock[wideCount]
uint32_t blockMask[wideCount]
PayloadWord payload[nodeCount]
uint32_t parent[nodeCount]
uint32_t subtreeSize[nodeCount]
uint32_t meta[nodeCount]
float geometricError[nodeCount]
padding to 64-byte allocation alignment
```

Index zero is an implicit, non-renderable parent for the definition's direct
roots. Real nodes start at one. The header stores the definition-wide root
bound because the implicit parent has no lane in another block.

One `WideBlock` is exactly 32 bytes per lane:

```cpp
struct WideBlock {
    WideBounds bounds;       // six SoA float vectors: min/max x/y/z
    float8 error;            // one child error per lane
    uint32_t child[kWide];   // packed node indices
};
```

It is 128 bytes in BVH4 and 256 bytes in BVH8. A real node's authored bound is
stored exactly once: in its lane of its parent's block. There is no parallel
scalar bound array.

The scalar arrays are packed for constant-time navigation:

- `parent` uses 20 bits for the parent index and nine bits for sibling ordinal.
  The ordinal locates the node's canonical parent block and lane without
  another lookup array.
- `meta` uses nine bits for child count, one mountable bit, and the remaining
  bits for the first wide-block offset.
- `subtreeSize[node]` makes every subtree a contiguous preorder range. Scalar
  coverage walks skip a child with `child += subtreeSize[child]`.
- One block-mask word packs valid lanes, plain-leaf lanes, and zero-error lanes.
  The low `kWide` bits are the valid mask, so a survivor mask can be ANDed with
  the complete word without first extracting those bits.

The builder grows bounds bottom-up in its temporary authoring nodes, emits a
preorder stream with a four-byte packed DFS stack, clamps error monotonically
from parent to child, and then writes the wide blocks. Registration validates
the representation and O(1)-moves the allocation into the database.
`SubtreeView` is only a set of pointers into those bytes.

`FRONTIER_VALIDATE_SUBTREES=0` removes the linear structural scan but retains
format, width, payload-size, layout, and root-range checks. That mode is safe
only for trusted bytes from a compatible builder. It does **not** currently
make the complete `registerSubtree()` call O(1): registration still scans
blocks/nodes to classify root-leaf, zero-terminal-error, and fully-refined fast
paths, and may build a terminal-node plan. The allocation transfer itself is
zero-copy.

### 4.2 Hot runtime records

The sizes below are guarded in the source and should be treated as design
constraints.

| Record | Size | Hot-path reason |
|---|---:|---|
| `NodeHandle` | 8 B | 20-bit mount slot, 20-bit node, 24-bit generation; a reserved slot encoding also carries TLAS roots. |
| `FrontierEntry` | 12 B | Handle plus packed 24-bit stable instance id and 8-bit error code. |
| `VisibleItem` / `TlasItem` | 4 B | 24-bit dense id or node plus six-bit plane mask. |
| `NodeItem` | 8 B | Node index, pixel error, plane mask, current/ideal liveness. |
| `WorkItem` | 16 B | Wide-bounds base, packed mount/mask/liveness/stride state, optional sparse-overlay index. |
| `TlasNode` | 128 B BVH4 / 256 B BVH8 | Wide bounds, child references, valid mask, parent; aligned to 64 bytes. |
| `TlasMeta` | 32 B BVH4 / 64 B BVH8 | Cold maximum-contribution and layer-mask lanes. |
| `Instance` | 80 B | Transform, exact world bound, error, mask, mount root, generations, overlay and TLAS back-pointers. |
| `InstanceOrientation` | 36 B | Lazily allocated local bound, yaw cosine/sine, signed XZ radius. |
| `MountTransformRt` | 32 B | Accumulated root-local transform, error clamp, generation, definition and fast-path flags. |
| `MountStamp` | 8 B | Content version, generation, liveness. |
| `MountReadiness` | 4 B | Fully-ready flag plus recursively incomplete-child count. |
| `SubtreeInstanceRt` | 56 B | Cold placement ownership, coverage pointer, LRU, links, and definition-list state. |
| Placement node state | 2 B/node | Covered bit, covered-child count, and—in shared state only—the authoritative ready bit. |
| `SpatialQuery::Rec` | 32 B | The random-access per-instance cache-hit record. |
| `SpatialQuery::RecCold` | 16 B | Slab allocation and output offsets, fetched only on misses/rebuilds. |
| `RenderFrontierRun` | 12 B | Slab begin/count plus one instance id for a cached render run. |
| `TerminalRenderRun` | 16 B on 64-bit | Payload pointer/count plus one packed instance/error word. |
| `MortonItem` | 12 B | Two halves of a 63-bit Morton key plus instance id; avoids normal pair padding. |

Top-level public instance ids are stable. The database separately maps stable
handle ids to dense physical ids, so `optimize()` can reorder and compact all
parallel instance streams without invalidating application handles.

### 4.3 Hot/cold and sparse allocation rules

Several absent allocations are part of the common-path design:

- `TlasMeta` is separate from `TlasNode`, so a frustum-only query does not read
  layer or contribution data.
- The orientation stream is empty until any instance uses non-identity yaw or
  yaw-invariant bounds.
- The flat-root marker stream is compact and exists only after the scene has
  contained a non-mountable root.
- Mount-child arrays exist only for placements that actually gain mounted
  children.
- Childless placements point at one definition-shared coverage block.
  Private coverage blocks come from definition-local geometric slabs instead
  of one allocation per placement.
- Bounds overlays and per-instance overlay-list headers do not exist until a
  caller deforms a node.
- A cache's second dependency array, overflow-count records, resolved render
  mirror, and mount-use records are all lazy.
- The fully-refined plan store is a nullable pointer and is created only when
  a qualifying definition is registered.

Do not turn one of these into unconditional per-instance or per-placement
state without measuring the new memory stream in a realistic scene.

## 5. The selection algorithm

At a high level, `SpatialQuery::selectFrontier()` does this:

```cpp
Camera view = damper.damp(rawCamera);
view = accountForDeferredWholePopulationTranslation(view);
visible = queryWideTlas(view, params.minPix, view.viewMask);

for (VisibleItem item : visible) {
    if (canReuseExactInstanceCut(item)) {
        appendRecordedBuckets(item.instance());
        continue;
    }

    if (isFlatRoot(item.instance()))
        emitPermanentRoot();
    else
        selectMountedInstance(item.instance(), item.planeMask());

    recordCutAndItsValidityCertificate();
}
```

There are cached and uncached orchestration functions, but both call the same
root and subtree walkers and produce the same three sequences.

### 5.1 Camera damping and the LOD test

`CameraDamper` implements hysteresis with one decaying axis-aligned camera
envelope per query, not sticky state per node. `Camera::queryMin()` and
`queryMax()` describe that envelope. Box-to-box distance costs the same wide
instruction pattern as point-to-box distance; with damping disabled the
envelope collapses to a point and the arithmetic is bit-identical.

The damper also retains a decaying maximum projection scale `k`, so zooming
out cannot discard detail faster than the configured half-life. Call
`SpatialQuery::reset()` on a teleport or camera cut; otherwise the envelope
spans the discontinuity and conservatively over-refines the space between.

For an interior child lane, `wideVisit()` computes:

```text
e  = min(block.error[lane], mount.errClamp)
d2 = squared separation(child AABB, camera envelope)
px = e * camera.k / sqrt(d2)
```

Refinement uses strict `px > threshold`. Plain leaves do not make this
decision: if visible they must be emitted, so the walker avoids scalar work
for them.

The error ceiling maintains monotonic LOD across a mount boundary. Mounting a
child stores:

```text
child.errClamp = min(parent.authoredError, parent.errClamp)
                 / relativeChildScale
```

The child definition stays immutable and the clamp is a single vector `min`
in its wide error test. The placement transform is accumulated all the way to
top-level-instance local space, so a nested mount can transform the root-local
camera directly instead of walking a transform chain.

### 5.2 TLAS traversal

`tlasQuery()` dispatches once to one of four template instantiations for
`UseMask` and `UseMinPix`. The default all-layers/no-contribution query contains
neither optional block and does not touch `TlasMeta`.

For each TLAS node:

1. `testWideAabb()` tests all child boxes against the active frustum planes.
2. The returned survivor bits are intersected with the node's valid lanes.
3. Optional layer masks and projected-contribution tests clear more lanes.
4. Interior lanes push four-byte stack items; instance lanes append four-byte
   visible items with their narrowed plane masks.

Contribution culling is deliberately separate from LOD error. `TlasMeta`
stores the maximum world-space bounds diagonal below each lane; `minPix` asks
whether the represented object can affect enough pixels at all, independent of
which geometric approximation would represent it.

If the root's lanes prove that the entire population is inside, the default
query emits `liveInstances_` directly. Cached selection can retain this exact
all-visible stream and prove it again with one root-width test instead of
rewriting and comparing the complete id array.

Moving instances may have a conservative grow-only TLAS leaf larger than the
exact `Instance::worldBox`. Such instances carry one `instanceTlasLoose_` byte.
Only a boundary hit needs an exact scalar root retest; a wholly-inside loose
envelope also wholly contains its exact box.

### 5.3 Permanent-root decision

`runTlasRootInstance()` first computes projected error for the permanent root.
If the root is below threshold or no descendant definition is mounted, it
emits the root into `shared` and stops. The TLAS root is always ready.

If refinement is required, the camera is transformed into instance-local
space. Identity yaw and the globally absent orientation stream have their own
branch-free path. The root placement then dispatches by readiness:

- A fully-ready mounted tree uses `runSubtree<true>()` and emits only `shared`.
- A partially-ready tree uses the selected current-cut policy.

### 5.4 Wide subtree traversal

`wideVisit<FullyReady, SparseOverlay, TrackAncestor>()` is the central kernel.
The template parameters remove readiness, overlay, and ancestor-policy work
from paths that do not need it.

For every child block it:

1. Resolves authored, dense-overlay, or sparse-patched wide bounds.
2. Performs one masked wide frustum test.
3. Uses the packed leaf and zero-error masks to recognize blocks that can emit
   immediately without loading errors or computing distance.
4. Computes clamped wide errors and squared distances only for the remaining
   block.
5. Emits surviving plain leaves directly.
6. Pushes only surviving interior or mountable nodes as eight-byte `NodeItem`s.

Bitsets are iterated with `countr_zero(mask)` and `mask &= mask - 1`, so cost is
proportional to surviving lanes rather than branch width.

The scalar DFS loop pops a `NodeItem` and either selects the node or refines
through its local children or mounted child. Mounted descendants use a separate
16-byte `WorkItem` stack. Both stacks retain capacity in query scratch.

The known root placement is entered by a direct call; it is not pushed merely
to be popped again. The placement work stack is needed only for mounted
descendants discovered later. An identity mount aliases the root-local camera
instead of copying/transforming it. A non-identity mount transforms only the
frustum planes still present in its mask—dismissed planes stay dismissed.

The plane mask is monotonic: planes are removed once a node is wholly inside
them. A zero mask avoids all lower frustum work and is also the crucial proof
for query reuse.

### 5.5 Producing current and ideal cuts together

The three physical buckets avoid a per-entry current/ideal tag and avoid two
independent traversals:

```cpp
void emit(entry, bool current, bool ideal) {
    if (current && ideal) shared.push(entry);
    else if (current)     currentOnly.push(entry);
    else                  idealOnly.push(entry);
}
```

For `PreferReadyDescendants`, each DFS item carries whether current and ideal
are still live on that branch:

- If an ideal node is below threshold and ready, it is shared.
- If that ideal node is unavailable but its visible descendants have a
  complete ready cover, it goes to `idealOnly` and a current-only traversal
  continues below it until ready descendants are found.
- If the requested refinement has no mounted definition, the mountable proxy
  ends the ideal cut; no unknown topology is invented.
- If visible descendants cannot cover an ideal refinement, the nearest ready
  node already on the branch is emitted to `currentOnly` and only ideal
  continues.

`visibleDescendantsCovered()` first consults propagated structural coverage.
That is a constant-time success for the normal covered branch. If coverage is
incomplete and the node is partially visible, it recursively ignores missing
branches that are outside the frustum. If the node is wholly inside, structural
coverage is definitive and no visibility walk is needed.

For `PreferReadyAncestors`, the implementation avoids a speculative second
readiness traversal:

1. The ideal traversal carries the nearest ready ancestor candidate across
   local and mount boundaries.
2. Every terminal ideal choice records its candidate and ready bit.
3. An unavailable ideal choice marks its candidate as collapsed.
4. Candidates are allocated parent-first. One forward resolution pass
   propagates the outermost collapse, emits each selected fallback once, and
   partitions ideal entries into `shared` or `idealOnly`.

The candidate vectors exist only for this policy. `NodeItem` and `WorkItem`
remain unchanged for the default descendant policy.

## 6. Fast-path inventory

The table below is a practical map of work that the implementation can skip.

| Predicate | Fast path | Work avoided |
|---|---|---|
| TLAS root lanes contain the population, no layer/min-pixel filter | Emit `liveInstances_` with zero masks | Complete TLAS walk. |
| Query uses all layers or `minPix == 0` | Template specialization | Cold `TlasMeta` loads and inner-loop feature branches. |
| Every live instance is a flat root | Automatic uncached/direct mode | Per-instance cache records and every mounted-state read. |
| Every flat root also has zero error | `runZeroErrorTlasFlatInstance()` | `Instance` fetch, distance, divide, and error encoding work. |
| Root error is below threshold or has no mount | Emit TLAS root | Local camera transform and all definition traversal. |
| Orientation stream is absent / yaw is identity | Translation-scale camera path | Yaw transform and cold orientation reads. |
| Mounted tree is fully ready | `runSubtree<true>()` | Per-node readiness and current/ideal branch logic. |
| All surviving lanes are plain zero-error leaves, or clamp is zero | `wideVisit()` block shortcut | Error-vector load, distance, reciprocal/sqrt, and scalar metadata reads. |
| Definition direct roots are all leaves | Root-leaves-only path | General scalar DFS loop. |
| Root-leaf placement is wholly inside | `emitMountedRootLeavesInside()` | Frustum tests and node stack. |
| Consecutive inside mount points use the same root-leaf definition | `emitMountedLeafBatchInside()` | Re-resolving immutable definition blocks and general work-item dispatch. |
| Eligible definition is guaranteed to refine every interior node | Fully-refined boundary path | All error decisions; wholly-inside branches bulk-generate preplanned terminal handles. |
| Visible instance is wholly inside and its certificate is valid | Per-instance cache hit | Instance record fetch, local transform, mount resolution, and hierarchy walk. |
| Visible stream and global certificate are unchanged | Retained whole-cut return | All per-instance record probes and output appends. |
| Same visible stream, only some records miss, bucket counts stay fixed | In-place output patch | Complete output reconstruction. |
| Exact undamped view recurs in a stable scene | Two-entry view memo | TLAS query, record checks, traversal, and answer reconstruction. |
| Cached render API is used | `RenderFrontierRun` scatter/gather | Copying resolved payload/error leaves on hits and repeating instance id per leaf. |
| Uncached visible count exceeds configured threshold | Per-worker contiguous ranges | Serial instance traversal while preserving deterministic output order. |
| Motion batch is at least one quarter of TLAS population | Exact streaming refit | Scattered mover-to-root propagation and later loose-root retests. |
| Small moved root remains inside its swept leaf envelope | Grow-only early-out | Leaf write and every ancestor write. |
| Whole population shares one translation | Deferred `tlasGlobalOffset_` | All instance and TLAS writes; mutation commits a few scalars. |
| Rigid group has yaw-invariant authored bounds | Specialized SoA motion kernel | General bound rotation/reconstruction and repeated eligibility checks. |
| Node was never deformed | Authored wide bounds | Overlay allocation, lookup, and private bounds memory. |
| Large overlay has few edits | Sparse patch mode | Copying every `WideBounds`; dense mode is promoted only past 1/16 edited blocks. |
| Terminal query accepts a whole branch | One `TerminalRenderRun` | Per-leaf handles, resolution, instance/error repetition, and descendant traversal. |
| Terminal actors form accepted clusters | Cluster broadphase | Per-actor root culling and descendant traversal for accepted/rejected clusters. |

Several thresholds are measurements encoded in the implementation:

- Fully-refined plans are created only for definitions with at least 16
  authored nodes; below that, the proof costs more than the normal walk.
- Sparse overlays start at 64 wide blocks and promote after more than 1/16 of
  blocks are patched.
- Large motion switches to exact full refit at one quarter of TLAS leaves.
- The Morton radix path uses comparison sort below 1,024 items.
- Cache slab compaction runs when abandoned capacity exceeds half of used
  capacity.

Treat these as target-sensitive tuning constants. Change them only with a
paired workload and machine-level explanation.

## 7. Exact frontier reuse

The cache does not approximate node membership. It records a proof that none
of the LOD decisions used to build one instance's cut can have flipped.

### 7.1 Geometric certificate

An interior LOD decision flips when distance reaches:

```text
flipDistance = effectiveWorldError * k / threshold
```

During a cacheable walk, Frontier records the minimum absolute gap between the
actual distance and this flip distance over every decision. That gap is the
record's world-space validity margin. It also records:

```text
kSlope = maximumEffectiveWorldError / threshold
```

The query and database maintain conservative monotonic odometers. A record is
geometrically valid while:

```text
queryEnvelopeTravel
+ wholePopulationUniformTravel
+ thisInstanceMotionTravel
+ kSlope * projectionScaleTravel
< validUntil
```

`validUntil` is the consumed amount at recording time plus the measured
margin. Doubling back remains conservative because these are path lengths, not
displacements. Instance translation uses an L1 bound to avoid a square root;
rotation charges a conservative radius-times-yaw-chord point-travel bound.
Scale changes alter the error field and invalidate by version instead.

Only instances with plane mask zero are cacheable. Once a root is wholly
inside, no descendant frustum decision depends on camera orientation; the cut
is a function of position, projection scale, topology, bounds, and readiness.
Boundary instances form a shell and are re-walked.

If a new code path makes an LOD decision, it must contribute to `w.margin` and
`w.maxError`, or the path must be declared uncacheable.

### 7.2 Non-geometric certificate

A hit also requires:

- the query epoch to match (`threshold` or current-cut policy changes bump it);
- the instance frontier version to match (scale, topology, deformation, and
  other non-travel changes bump it);
- every recorded mounted-tree content stamp to match;
- no more than two dependencies, the compact record limit;
- a live, same-layout dense instance record.

Ordinary selection coalesces all descendant placements in one mounted tree to
the root placement's content stamp. Descendant mutations propagate a bump to
that root. Mount-use feedback needs physical placement touches, so it can
disable a coalesced hit and force the uncommon exact usage walk.

### 7.3 Record and slab layout

`SpatialQuery::Rec` is exactly 32 hot bytes and is indexed by dense instance
id. It contains only the hit-path certificate, a slab offset, compact bucket
counts, and one dependency. Allocation capacity and old output offsets live in
the 16-byte cold record.

Three 10-bit bucket counts and a two-bit dependency count share one word. A
dependency count of three is an escape marker whose low 30 bits index a sparse
16-byte full-width count record. One dependency is inline; the second-dependency
array is allocated only if a real cacheable cut needs it.

Recorded entries for one instance are contiguous in `store_` as
`shared/currentOnly/idealOnly`. A replacement reuses its old block if capacity
fits. If it grows, the old block becomes garbage and a new block is bump
allocated. Rare compaction copies live runs through a same-sized scratch slab;
in-place compaction would be unsafe because record id order and allocation
order differ.

The custom `AppendBuffer` retains capacity, grows by roughly 1.5x, supports
uninitialized resize for POD output, uses unrolled copies for at most four
elements, and bulk `memcpy` otherwise. Sources passed to `append()` must not
alias the buffer because growth may relocate it.

### 7.4 Whole-answer reuse and exact-view memo

The view-returning API marks its output as retained. If the visible sequence is
unchanged and the aggregate minimum remaining margin, maximum projection
slope, content generation, mapping, epoch, and global motion odometer all
match, the existing contiguous buffers are already the answer. Selection
returns before reading the 32-byte records.

When that global test fails but the visible sequence is unchanged, misses can
patch old bucket ranges in place if their three counts did not change. One
count change falls back to a deterministic rebuild from all per-instance slab
runs.

A separate two-entry LRU memo recognizes bit-exact raw camera and parameter
keys. It is enabled only for undamped, reuse-enabled, hierarchical queries with
mount-use tracking off, and only while mapping, content, and spatial versions
are stable. The current admission logic observes a pose recurrence before
retaining a full answer, so one-off views do not immediately duplicate a large
cut.

### 7.5 Error-code caveat

The node set on a cache hit is exact, but the stored eight-bit error magnitude
is not recomputed inside the proven interval. Codes are logarithmic relative
to the selection threshold, about eight steps per power of two. Codes 0–127
are at or below threshold and 128–255 are above it, so the threshold
classification remains exact even when the approximate magnitude ages.

## 8. Readiness and coverage maintenance

The authoritative ready bit lives in the definition-shared 16-bit node-state
block. The same word also contains intrinsic covered state and a covered-child
count. A childless placement points directly at this block.

When a placement gains a mounted child, it needs placement-specific coverage.
`ensurePrivateCoverage()` takes a block from a definition-local slab and copies
only coverage/count bits; authoritative ready bits remain in the definition.
The slabs grow geometrically up to roughly 1 MiB and recycle blocks through a
free list.

A readiness change:

1. Changes the definition's node ready bit and ready-node count.
2. Incrementally propagates shared intrinsic coverage toward the definition
   root until state stops changing.
3. Walks the intrusive list of placements of that definition—never unrelated
   definitions or payloads.
4. Propagates private coverage where mounted children make it placement-local.
5. Bumps placement and coalesced root content stamps.
6. Updates the four-byte fully-ready summary and propagates only summary
   transitions through owner mounts.

A node is covered when it is ready itself or all of its children are covered.
For a mountable node, “children” means the mounted child's implicit root. The
TLAS root needs no ready bit because it is permanently ready.

The separation between `MountReadiness` and detailed coverage matters:
fully-ready selection is one load and then a lean traversal; partial-readiness
selection pays detailed coverage only where necessary.

Mount retention is also opt-in. A query with mount-use tracking disabled pays
no use-recording cost. When enabled, it records physical placement touches;
the writer later consumes only dirty query records into the database LRU.
Collection walks from the LRU tail and removes only old leaf placements until
the placement budget is met. It never implicitly releases registered
definitions, and readiness bits survive the removal of placements.

## 9. TLAS construction and maintenance

### 9.1 Build tiers

The configured quality tier is `Morton`, `Median`, or 16-bin `BinnedSAH`.
Initial builds, population-drift rebuilds, area-drift rebuilds, and explicit
quality repairs use the configured tier. Edit-budget and defensive escape
repairs use Morton unless another trigger promotes the same rebuild.

The quality builder recursively produces a `kWide`-way tree using
`log2(kWide)` binary splits per node. SAH scans 16 bins on all three axes; a
degenerate SAH choice falls back to a longest-axis median so progress is
guaranteed. Ranges of at most `kWide * kWide` are packed explicitly into full
leaf groups, avoiding an underfilled final internal level.

The Morton builder:

- quantizes centroids to 21 bits per axis and interleaves a 63-bit key;
- stores each item in 12 bytes rather than a padded key/id pair;
- uses stable LSD radix passes of 11 bits, keeping the 2,048-entry histogram in
  L1;
- skips digit passes with no key variation;
- uses `std::sort` for fewer than 1,024 items;
- groups consecutive `kWide` instances into leaves and then groups nodes
  bottom-up in streaming passes.

### 9.2 Incremental structural edits

Insertion descends through internal lanes by least surface-area growth. It
uses a free leaf lane when possible. If the leaf is full, a new node replaces
that leaf in its parent and adopts the old leaf plus the new instance. This
always succeeds without scanning upward for a free lane.

Removal clears the instance lane and unlinks newly empty nodes. It deliberately
leaves ancestor boxes loose. `tlasEditFraction` bounds accumulated quality
loss; `tlasCountDrift` promotes a rebuild only for real population drift, so
steady spawn/despawn churn can remain incremental.

### 9.3 Motion publication

Exact instance transforms and bounds change at submission time, but TLAS writes
are coalesced until `applyUpdates()`.

For a small cohort, each leaf retains a grow-only motion envelope. If the exact
new box is already inside, no TLAS memory is written. Otherwise the leaf and
ancestors grow until the first ancestor already containing the new box,
maximum contribution, and layer mask. Added lane area is accumulated; crossing
`tlasAreaDrift` schedules a quality rebuild.

At one quarter of the population, scattered leaf/ancestor traffic is more
expensive than streaming the tree. `tlasRefitAllExact()` retains a bottom-up
node order until topology changes, copies exact dense instance state into leaf
lanes, recomputes interiors sequentially, and clears loose flags. The pending
instance-id vector and retained postorder reuse build scratch that is mutually
exclusive with publication.

Repeated identical transforms exit before changing cache or TLAS state.
Stable-scale translation updates an exact AABB by adding delta rather than
retransforming it. Yaw-invariant authored root bounds allow yaw changes to keep
the same translated broadphase envelope. L1 motion length and a chord bound
avoid square roots and trigonometry in cache travel accounting; yaw is already
submitted as cosine/sine.

`MotionGroup` resolves caller handles, sorts and deduplicates them into dense
order once, and retains the mapping until the instance-mapping epoch changes.
Batch-wide invalidation can reuse one mutation generation. `RigidMotionGroup`
also proves yaw-invariant eligibility once per mapping and accepts separate
contiguous position and yaw streams.

If a motion group covers every live instance with one common translation, the
database leaves all instance and TLAS bytes in a common base space and updates
`tlasGlobalOffset_` plus travel/version scalars. Selection transforms the
camera by the offset. A later differential edit materializes it in one pass.

### 9.4 Physical reordering

The first spatial build and explicit `optimize()` reorder dense database
instance streams into TLAS traversal order. A query detects the layout-version
change, clears old records, and subsequently indexes new cache records in that
same dense order. Visible ids then tend to be a monotonic spatial subsequence,
improving locality in `Instance`, cache record, motion, and parallel arrays.
Stable public handle maps and TLAS leaf ids are rewritten around the
permutation. Dead dense slots are compacted at the same time.

Any new per-instance parallel stream must be handled in allocation, removal,
reordering, and compaction code. Missing one of those sites usually creates a
correctness bug that appears only after `optimize()`.

## 10. Copy-on-write deformed bounds

Authored topology, errors, payloads, and bounds remain immutable. A
`setNodeBounds(instance, node, box)` call queues a generation-stamped edit; it
does not mutate definition bytes.

On flush, the first edit to an `(instance, placement)` creates an overlay:

- Definitions below 64 wide blocks copy all `WideBounds` immediately.
- Larger definitions allocate a dense block-to-patch index plus only modified
  wide blocks.
- Once more than 1/16 of blocks are touched, the overlay materializes dense
  bounds and releases sparse storage.

An instance has a sorted `(mount slot, overlay index)` list. The common
undeformed path checks one flag and has no list. A `WorkItem` records whether
its bounds stride is interleaved authored `WideBlock` or packed overlay
`WideBounds`; that bit occupies packed state rather than another word. Sparse
lookup is a separate `runSubtreeImpl` template instantiation, so dense/authored
walks do not branch per block.

The submitted node is set exactly and may shrink. Ancestors are grow-only:
propagation stops at the first containing bound, crosses mount boundaries
through owner links, and finally grows the exact instance box and TLAS
envelope. Only overlays along that ancestor path are privatized.

Submission order is preserved and the last write to a node wins. The queue is
intentionally not sorted or deduplicated: repeated edits hit the same hot lane
and naturally stop at an already-grown ancestor, while measured stamp/hash/sort
schemes cost more cold traffic. Stale queued edits self-discard through instance
and node generations.

## 11. Renderer-facing output paths

### 11.1 General handle output

`FrontierEntry` omits `UserPayload`; payloads stay in immutable definition
arrays. The packed 24-bit stable instance id avoids repeating application
entity data. Fixed caller `Sink`s write what fits and count overflow; internal
sinks append to retained buffers. `pushRange()` turns a cached instance hit
into one bulk copy.

Error encoding does not call `log2` on the hot path. It extracts the IEEE-754
exponent and three leading mantissa bits from the threshold-relative ratio,
then clamps the code to the correct side of 128. `decodeFrontierError()` uses
`exp2` only when a caller explicitly asks for an approximate magnitude.

`resolveFrontier()` and the cached render resolver exploit emission locality:
consecutive handles from the same mount validate slot/generation once and then
stream the definition payload array. Stale handles produce the reserved invalid
payload rather than touching recycled state.

`selectRenderFrontier()` goes further in cached hierarchical mode. It retains
resolved payload and one-byte error arrays at the same per-instance slab
offsets. A hit appends only a 12-byte `{begin,count,instance}` run. Entering or
leaving instances changes the run list, not cached leaves. `renderAsUnit` can
coarsen descendant frustum culling for small boundary actors, setting their
mask to zero after the TLAS accepts the root and making their complete LOD cut
reusable.

### 11.2 Fully-refined general fast path

Registration identifies definitions with:

- at least 16 nodes;
- no mountable nodes;
- zero error on every terminal leaf; and
- a positive minimum error across interior nodes.

It precomputes terminal node ids and one contiguous terminal range per node in
the ordinary LIFO output order. For a fully-ready, non-deformed boundary
instance, a farthest-distance proof can show that every interior error exceeds
threshold. The walker then performs frustum work only. A wholly-inside branch
bulk-generates zero-error handles from its precomputed terminal range.

The proof must remain conservative. If topology, overlays, terminal error, or
output order changes, update eligibility and plan construction together.

### 11.3 `TerminalRenderQuery`

`TerminalRenderQuery` is a deliberately narrower max-detail algorithm. It
requires fully ready mounted definitions, no nested mounts, no deformed bounds,
and zero error on every terminal leaf. It lazily builds a generation-stamped
definition plan containing decoded payloads and a `{begin,count}` range for
every node.

When a branch is wholly inside, the query emits one 16-byte run pointing at
the plan's payload range. Only boundary branches descend through wide blocks.
Adjacent payload ranges with the same instance/error word are coalesced. This
avoids materializing and resolving one 12-byte handle per leaf and stores the
constant instance/error once per run.

Homogeneous `TerminalInstanceBatch` actors can stay outside the general TLAS
and mutable mount population. The query consumes caller-owned SoA position/yaw
streams, one shared definition, constant bounds/scale/mask, and consecutive
external ids. Optional ordered, gap-free clusters add a two-level broadphase:

- Bounds can be reduced from current members.
- Yaw-invariant clusters reduce just the min/max position stream.
- Caller-published conservative cluster bounds avoid reduction entirely.
- An outside cluster rejects all actors; an inside cluster emits every actor's
  full definition range; only boundary clusters test individual roots.

Published cluster bounds must contain every actor for the snapshot. Contract
builds verify this; under-bounds are a correctness failure, not a quality
tradeoff.

`rigid_motion.cpp` and `terminal_render.cpp` deliberately remain separate
static-library archive members. Programs that never call those specialized
APIs need not pull their code into the final link, and the generic
`spatial_database.cpp` selector is not duplicated or rearranged to host them.

## 12. SIMD and low-level arithmetic choices

Branch width and SIMD backend are compile-time choices. There is no runtime ISA
dispatch:

- BVH8 + AVX2 uses one 256-bit group and FMA.
- SSE2 uses one or two 128-bit groups for BVH4/BVH8; SSE4.1 blend is used when
  available without violating an SSE2-only build.
- AArch64 NEON uses one or two 128-bit groups.
- Forced-scalar uses the same wide layout and loop semantics.

Serialized bytes record branch width and payload word size. Both are ABI and
format choices that must match every translation unit and stored definition.
BVH8 is not universally faster: it tests more lanes and may make a shallower
tree, but doubles a wide block and combined TLAS node footprint. Lane occupancy
and cache behavior decide; compare complete BVH4/BVH8 workflows on each
shipping architecture.

`WideBounds` is SoA-transposed: all child min-x values are contiguous, then
min-y, and so on. Frustum tests select p/n vertices per plane and calculate all
lane dot products together. On NEON, survivor and narrowed plane masks stay in
vector registers through all planes because scalarizing each comparison was
measured to dominate the kernel.

The hot error path keeps squared distance. AVX2 uses `rsqrt` plus one
Newton–Raphson refinement and explicitly handles zero-distance infinity/NaN,
avoiding a vector square root followed by division. AArch64 uses exact vector
sqrt/divide because that measured faster than its estimate/refinement paths.
SSE/scalar use their matching exact implementation.

Scalar and wide FMA behavior is deliberately matched because tests require
bit-identical scalar/wide results. Do not allow contraction on only one path or
replace arithmetic based solely on instruction count; verify both numerical
contracts and the machine kernels.

### 12.1 Measured traps worth knowing

The archived optimization rounds are useful because several plausible changes
lost to memory traffic or code layout:

- A separate SIMD stream of TLAS root errors added a cold load and regressed
  large uncached queries; scalar root arithmetic was cheaper.
- A scalar special case for one surviving interior lane did not justify its
  extra branch beside the established wide kernel.
- Extra zero-error branches inside the straight-line mounted-leaf batch made
  that batch much slower even though they appeared to remove arithmetic.
- Sorting, hashing, or stamping queued bounds edits cost more cold traffic than
  repeated hot-lane updates and containment early-outs.
- Returning an indirection descriptor for an ordinary cache hit measured no
  better than bulk-copying its small recorded ranges, while complicating the
  caller. The render-native API earns zero-copy through a materially different
  run layout instead.
- A forced cache miss does the raw walk **and** records it. If nearly every
  instance is known to miss, `setReuseEnabled(false)` is the optimization.

These are not timeless laws, but they explain seemingly non-minimal code. Reopen
one only with a target workload, machine-kernel evidence, and a paired
end-to-end gate; do not infer a win from deleted instructions alone.

## 13. Determinism, concurrency, and stale safety

Uncached parallel selection divides the visible array into contiguous worker
ranges. Each worker owns stacks and three output buffers. Concatenating workers
in index order reproduces serial visible-instance order exactly, so serial and
parallel cuts are bit-identical.

Runtime slots are recycled, so every public handle carries a generation.
Expected asynchronous races are benign: a completion for an unmounted node is
ignored, a stale query reports absence, and mounting below a collected node
returns an invalid placement. True live precondition failures still go through
`FRONTIER_CHECK`.

`FRONTIER_CONTRACT_CHECKS` guards caller contracts and may be disabled only for
trusted production profiles. `FRONTIER_ASSERT` protects internal invariants in
Debug. They are intentionally separate.

## 14. How to change the code without breaking the proofs

### Adding or changing a selection parameter

- Decide whether it changes TLAS visibility, LOD decisions, current/ideal
  membership, output encoding, or only metadata.
- Update cache epoch logic or add a certificate term. “The camera is the same”
  is not enough for per-instance reuse.
- Update `sameMemoParams()` for the exact-view memo.
- Consider retained whole-cut validity and in-place output patching.
- Add the parameter to terminal/render-native paths if their contract includes
  it, or reject it explicitly.

### Changing traversal decisions

- Preserve strict threshold classification unless intentionally changing the
  public cut.
- Any new interior decision on a cacheable path must update validity margin
  and maximum error.
- Preserve plane-mask narrowing and the meaning of mask zero.
- Check fully-ready, partial descendant, ancestor fallback, sparse-overlay,
  oriented-instance, and missing-mount variants.
- Keep plain-leaf detection consistent between builder masks, validation, and
  every specialized plan.

### Changing serialized data

- Bump `kSubtreeVersion` for any incompatible layout or semantic change.
- Update builder, layout computation, validator, and zero-copy view together.
- Preserve 64-byte allocation alignment and backend-required vector alignment.
- Re-evaluate `WideBlock` size, width/payload compatibility checks, and stored
  fixtures or persisted assets.
- Do not add a duplicate per-node bound merely for convenient scalar access;
  the canonical parent lane and packed ordinal are intentional.

### Adding per-instance or per-placement state

- Put hit-path fields only in records actually fetched by the hit.
- Prefer a parallel cold/lazy stream to widening `Instance`, `Rec`,
  `MountTransformRt`, `MountStamp`, or `WorkItem`.
- Update slot allocation, recycling, byte accounting, optimize/reorder,
  generation validation, and reset paths.
- Measure the scene-wide retained footprint, not only an isolated operation.

### Changing readiness or topology

- Keep readiness definition-local and coverage placement-local.
- Update both detailed coverage and the four-byte fully-ready summary.
- Bump the exact placement/root/instance content stamps used by cache hits and
  whole-answer reuse.
- Preserve mount bounds containment and inherited error monotonicity.
- Test shared-definition fanout, nested mounts, unmount/remount slot reuse, and
  both current-cut policies.

### Changing motion or bounds

- Distinguish translation travel, rotation travel, and changes that must bump
  `frontierVersion`.
- Preserve exact instance bounds even when TLAS envelopes are grow-only.
- Update loose-root handling and exact-refit postorder invalidation on topology
  change.
- Keep deferred whole-population offset materialization transactional.
- For overlays, update both dense and sparse template paths and promotion.

### Changing output order or representation

- The fully-refined and terminal plans are built to reproduce the ordinary
  LIFO traversal order. Update plan construction with the walker.
- Update three-bucket counts, cache slab layout, output offsets, compaction,
  bulk resolution, renderer mirrors, and tests.
- Keep `FrontierCutView`'s two-span order: `shared` is always iterated before
  the cut-specific bucket.

## 15. Verification and performance workflow

For correctness, run:

```bat
run_unit_tests.bat
```

The repository suite covers both BVH widths and includes model-based readiness
and TLAS churn, scalar/SIMD equivalence, cache invalidation, overlays,
streaming races, parallel determinism, and concurrent snapshot readers. For a
layout or SIMD change, also build explicit SSE2-only, forced-scalar, BVH4, and
instrumented variants as described in [TESTING.md](TESTING.md).

Use `FRONTIER_STATS` to explain traversal changes with instances, subtrees,
nodes, wide blocks, survivor lanes, and fully-refined counts. Do not ship it by
accident; normal builds retain no hot counter state. `FRONTIER_DEBUG_TOOLS`
provides on-demand TLAS and cache summaries without selection instrumentation.

For performance, use Release with the production profile and compare repeated
medians. The most diagnostic cases are:

| Change area | Benchmarks |
|---|---|
| Wide math / branch width | `BM_KernelWideAabb`, `BM_KernelDistanceErrorCurrent`, `BM_BranchWidthOccupancy` |
| Per-instance cache | `BM_KernelCacheHit*`, `BM_InstanceForestSelectionScale`, `BM_MovingCameraSelectionScale` |
| Flat/direct paths | `BM_FlatTlasSelectionScale`, `BM_InstanceForestRootSelectionScale` |
| Readiness policies | `BM_MixedReadinessFrontier`, `BM_SharedNodeReadiness*` |
| TLAS quality | `BM_TlasQualitySelection`, `BM_FlatInstanceLifecycle` |
| Motion/publication | `BM_MotionGroupSteady`, `BM_MovingObjectsSelectionScale`, `BM_LiveCityMotionFrame` |
| End-to-end selection | `BM_LiveCityDrivingFrame` |
| Result consumption | `BM_LiveCityRenderSubmissionFrame` |
| COW bounds | `BM_BoundsOverrideBatch` |
| Definition sharing/build | `BM_SubtreeAssembly_*`, `BM_SubtreeRegistration`, `BM_SubtreeBuilder_ConstructCost` |

For small deltas, use the ABBA paired runner in
[BENCHMARKING.md](BENCHMARKING.md). A hot-loop improvement that moves the
machine controls or an unrelated decomposition case by the same amount is not
evidence of a Frontier improvement.

## 16. Source-reading map

Read these symbols in this order when debugging selection:

1. `NodeHandle`, `FrontierEntry`, `SelectionParams`, and `SpatialQuery::Rec` in
   `include/frontier/spatial_database.h`.
2. `WideBounds`, `testWideAabb()`, `distanceToBoxesSq()`,
   `screenErrorFromSq8()`, and `CameraDamper` in `include/frontier/math.h`.
3. `WideBlock`, `SubtreeHeader`, and `SubtreeView` in
   `include/frontier/detail/subtree_data.h`.
4. `SpatialDatabase::tlasQueryImpl()` and `wideVisit()` in
   `src/spatial_database.cpp`.
5. `runSubtreeImpl()`, `runSubtreeAncestorImpl()`, and
   `runTlasRootInstance()` in the same file.
6. `selectFrontierUncached()` and `selectFrontierCached()` for orchestration.
7. `setDefinitionNodeReadiness()` and the coverage propagation helpers for
   streaming behavior.
8. `tlasOnInstanceMoved()`, `tlasRefitAllExact()`, and `tlasRebuild()` for
   publication behavior.
9. `src/terminal_render.cpp` for the strict range-output path.

For serialized authoring, start with `SubtreeBuilder::build()` in
`src/builder.cpp` and then read `validateSubtreeBytes()` in `src/subtree.cpp`.
For performance intent and current measurements, use
[ARCHITECTURE.md](ARCHITECTURE.md), [PERFORMANCE.md](PERFORMANCE.md), and the
historical optimization notes under `docs/archive/performance/`; the checked-in
code and its tests remain authoritative when an old note disagrees.

## 17. Complexity and the real cost model

The nominal costs are:

| Operation | Cost |
|---|---:|
| Build definition | O(definition nodes) |
| Register definition | O(nodes + wide blocks) worst case for validation and fast-path classification/plan construction; disabling structural validation removes only that scan, not classification |
| Mount after shared state exists | O(1) for a childless placement; first nested child copies that definition's coverage block |
| Readiness change | Placements of one definition plus changed ancestor paths |
| Submit bound change | O(1) queue append |
| Flush bound change | O(changed ancestor depth), stopping at containment |
| Incremental TLAS insert/remove | O(TLAS depth), excluding scheduled repair |
| Selection | Output-sensitive TLAS plus surviving hierarchy work |
| Per-instance cache hit | O(recorded output + at most two dependency checks) |
| Whole retained answer | O(1) after the visible stream has been proven or compared; validating a boundary-visible stream still costs its TLAS query/comparison |
| Admitted exact-view memo | O(1) metadata and exact-key checks |

The useful performance model is more concrete:

- Wide blocks tested are expensive but sequential and SIMD-friendly.
- Scalar survivors, random 32-byte cache records, mount indirections, and cold
  readiness/overlay streams determine the constants.
- The best path is often the one that proves a block, instance, or entire
  answer does not need to be visited.
- Definition sharing reduces immutable working set but introduces placement
  indirection. Raw traversal can move either direction by target and scene;
  measure it.
- Cache misses can be more expensive than reuse-disabled traversal because
  they both walk and record. Disable reuse for a query known to invalidate
  nearly every hierarchical instance.
- Returned-view timing does not include consuming every entry. Benchmark the
  render submission path when optimizing end-to-end frame cost.

That is the central maintenance principle: preserve exactness with compact
proofs, then arrange the data so the common proof is cheaper than the work it
eliminates.
