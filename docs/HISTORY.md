# Frontier implementation history and performance journal

> This is a historical engineering record. Names, measurements, APIs, and
> implementation details below describe the revision under discussion and may
> no longer be current. Use [README.md](../README.md) for the overview,
> the [API guide](API.md) for integration, the
> [API reference](API_REFERENCE.md) for exact signatures,
> [ARCHITECTURE.md](ARCHITECTURE.md) for the current implementation, and
> [frontier_design.md](frontier_design.md) for the current behavioral contract.
> Current measurements are in the
> [cross-platform performance snapshot](PERFORMANCE_RESULTS_2026_08_18.md).

This document preserves the optimization experiments and measurements that
shaped the library. It is evidence and rationale, not integration guidance.

## 1. Architecture snapshot at the time of this journal

The runtime has two spatial levels, analogous in structure to a ray tracer's
BLAS/TLAS split. The names describe runtime roles, not object-versus-world
scale:

- **Renderable BLAS hierarchies in immutable pages.** A BLAS is any
  independently rooted authored hierarchy: it may be a city block, a building,
  a terrain region, a reusable prop, or a single flat object. Every real node
  carries its own renderable `UserPayload`, so the cut may stop at any level.
  `HLodBuilder` emits versioned page blobs containing preorder arrays and
  8-lane child blocks; expansion points connect separately streamed pages.
  Instances of one registered root share its page bytes, residency, and
  attachment graph. Deformed instances keep sharing those bytes and acquire
  bounds-only copy-on-write overlays. The API term *asset* names this unit of
  registered storage and sharing; it does not imply one object.
- **Non-renderable dynamic wide TLAS.** An 8-wide BVH over translated,
  uniformly scaled placements of independent BLAS roots handles coarse
  frustum/layer/contribution culling, motion, and incremental insert/remove.
  Its internal nodes are acceleration data and never appear in a cut. Keeping
  many BLAS roots avoids an artificial whole-map root and allows large
  hierarchical regions to coexist with one-node vehicles, characters, and
  props. Initial and repair builds can use a quality splitter; frequent
  motion/edit rebuilds use a cheaper Morton hierarchy.

`View::selectCut` queries the TLAS, walks only surviving page regions, and emits
three compact vectors: entries shared by the current and ideal cuts, plus each
side's additive difference. A high-error ideal-side leaf exposes a useful
expansion frontier; the caller's content graph decides whether deeper topology
exists and derives IO work from those entries. `View` owns per-camera damping,
conservative cut reuse, traversal scratch, and statistics. After
`applyUpdates` publishes a stable snapshot, distinct views read only the World
and can run concurrently.
Disabling reuse on a `View` enables the internally parallel uncached path
through the host's blocking `parallelFor` without changing that contract.

### Why it is fast

1. **Output-sensitive descent.** An explicit DFS carries error, undecided
   frustum planes, and current/ideal liveness together. Nothing below either
   cut, outside the frustum, or behind unattached
   topology is visited, and no all-nodes per-frame clear exists.
2. **Eight-lane layout on every backend.** A parent tests eight child bounds and
   errors from one 256-byte `WideBlock`. AVX2 handles eight lanes directly;
   SSE2 and NEON use two four-lane halves; scalar builds retain the same blob
   layout. The TLAS uses the same logical width, dispatches optional layer and
   `minPix` policy once per query, and skips child-mask materialization after a
   subtree becomes fully inside the frustum.
3. **Leaf fast path.** A side-array mask identifies plain leaves, which are
   emitted directly from the parent's wide test without a metadata read or DFS
   stack round trip. At fanout eight, most authored nodes are leaves.
4. **Shared immutable working sets.** Thousands of identical instances can walk
   one hot page. The cut-path `Instance` record is 32 bytes, while TLAS
   maintenance state occupies a separate 48-byte array. Uncached selection
   prefetches the next instance and root page; cached selection pipelines
   its spatially ordered random instance/view-record reads eight entries ahead.
   World-to-local camera conversion computes one reciprocal per instance and
   uses multiplication for every position, envelope, and plane transform.
5. **Compact handle-only output.** The world has no payload index or node hash
   table. A cut entry is 12 bytes: an 8-byte generation-stamped handle plus a
   packed 24-bit instance id and 8-bit error. Stale asynchronous completions
   fail one generation check and are ignored.
6. **Propagated resident coverage.** Residency changes maintain a complete
   descendant-cover summary through local and attached-page ancestors. Fully
   covered subtrees pass in O(1); partially visible uncovered subtrees inspect
   only surviving branches, allowing resident descendants to bypass a missing
   intermediate proxy without opening a hole.
7. **Lazy, bounded updates.** Bounds edits queue until `applyUpdates` and grow
   ancestors only until containment permits an early-out. Instance adds/removes
   edit the TLAS in O(depth); configured edit/escape/area budgets decide when a
   rebuild earns its cost. Escape pressure counts distinct leaves since the
   last build, so repeated motion of one bounded cohort is charged once.
8. **Cheap Morton repair builds.** 63-bit keys use a stable LSD radix sort for
   populations of at least 1,024, with retained scratch and only the 11-bit
   passes required by key variation. A dense live-id list avoids scanning dead
   historical slots.
9. **Frame-coherent reuse.** A 32-byte hot `View` record proves when a
   fully-inside instance's bucketed cut cannot change under camera-envelope or
   projection-scale travel. A 4-byte cold allocation record and an optional
   second-page dependency stay off the hit path; a compact instance-version
   stream avoids fetching the 64-byte instance record on hits.
10. **Fully-resident fast path.** An 8-byte per-mount summary propagates whether
   every attached page is recursively resident. Such instances use a
   shared-only traversal with no residency or current/ideal branching.
11. **TLAS root decisions and direct flat-instance emission.** A leaf lane
    compactly marks a hierarchical BLAS with one renderable root. Uncached
    queries vector-test those roots and, when the result is promising, retest
    the exact instance box and emit the pinned root without transforming the
    camera or entering a page. The query enables this work only while enough
    visible roots terminate there and otherwise probes periodically. Cached
    queries keep the universal TLAS loop lean and test roots only on misses.
    Exact one-node BLASes retain their still-cheaper path: after the first one,
    the World lazily allocates a 4-byte root marker per instance and emits it
    without loading the 64-byte `Instance` or its 256-byte page block.
12. **Deferred, camera-selective GC feedback.** Optional `PageUsageContext`
    objects collect page touches without writing the World. `collect` consumes
    only the important cameras chosen by the caller, updates the intrusive LRU,
    and detaches aged leaf mounts; pinned roots are outside the budget and list.

### Memory-sensitive runtime layouts

The runtime's multiplied structures have exact compile-time size contracts on
64-bit builds, and `Contracts.MemoryBudgets` reports them in the test suite:

| Structure | Size | Why it has that shape |
|---|---:|---|
| `CutEntry` | 12 B | 64-bit node handle + packed 24-bit instance id / 8-bit error |
| `View::Rec` hot / cold | 32 B / 4 B | hits read only proof, version, counts, and the common one-page dependency |
| optional second page dependency | 8 B/instance | allocated only after a cacheable record actually touches two pages |
| `PageUsageContext::Rec` | 8 B | 24-bit page generation and pending flag share one word |
| `AssetRt` | 112 B | one owning-or-borrowing `Page`; no duplicate `PageView` |
| `PageRt` | 104 B | cold mount state; LRU links and the 24-bit handle generation share one word |
| per mounted page validation stamp | 8 B | dense content version plus generation/live flag for cache-friendly validation |
| per mounted page residency summary | 8 B | recursively identifies fully-resident mount trees without scanning nodes |
| per mounted node | 3 B | one packed residency/coverage byte + 16-bit covered-child count |
| `Instance` + `InstanceTlas` | 32 B + 48 B | overlay-list headers and spatial-maintenance state stay out of the common cut stream |
| instance cut version / flat-root marker | 4 B / 0 or 4 B | the flat marker stream is allocated only after the first flat asset |
| `Overlay` header | 104 B | allocated only after deformation; large-page wide bounds start sparse and promote to dense |
| per-deformed-instance overlay-list header | 24 B | cold pooled vector; undeformed instances allocate none |
| `TlasNode` | 320 B | 296 B of SIMD lanes/metadata rounded to 32-byte alignment; single-root leaf flags occupy unused valid-mask bits |
| visible hit / TLAS stack item | 4 B / 4 B | 24-bit id or node index + plane mask; a spare hit bit carries the root candidate |
| node / page DFS item | 8 B / 24 B | existing 20-bit node and mount limits carry traversal flags |
| queued bounds edit | 48 B | one `AABB`, compact `NodeHandle`, instance id and generation |
| Morton sort item | 12 B | 63-bit key as two words + instance id, without pair padding |

Mounted residency used to require separate byte arrays for `resident` and
`covered` plus a 32-bit child count. The current packed state halves the two
booleans, halves the counter, and removes one allocation. A mount also no
longer copies the asset's 88-byte `PageView`; lookup through the already-stored
asset index happens once when a page walk begins.

### Measurements at the time of this journal

Point estimates below were rerun on 2026-08-05 with MSVC 19.51, Release
`/O2 /arch:AVX2`, on a noisy shared 64-hardware-thread 2.4 GHz EPYC.
Single-view cases use one benchmark thread; the concurrent six-view cases use
six persistent workers. Values are the best observed repeat, which estimates
uncontended algorithm cost. Every fixed-trajectory case uses 600 frames and at
least five repetitions.

| Scenario | Current time | Relevant output/state |
|---|---:|---|
| 10k instances, 700 assets, moving camera / 1k moving | 0.089 ms selection | 82.4% reused; 27.8% visible; 5,920-entry cut; 1.87 MiB `View` |
| Same 10k world, static camera / 1k moving | 0.074 ms selection | 85.8% reused; 25.0% visible; 5,792 entries; 0.45 MiB `View` |
| Same 10k world, moving camera / static objects | 0.063 ms selection | 93.3% reused; 5,920 entries |
| 80k instances, moving camera / 5% moving | 0.400 ms selection | 92.6% reused; 24,986 entries; 5.98 MiB `View` |
| Same 80k world, static camera / 5% moving | 0.296 ms selection | 94.1% reused; 22,872 entries; 3.22 MiB `View` |
| Same 80k world, moving camera / static objects | 0.330 ms selection | 97.6% reused; 24,986 entries |
| Six 80k views, static objects, serial / concurrent | 2.50 / 0.445 ms wall time | 5.6× on six persistent workers |
| Six 80k views, 5% moving, serial / concurrent | 2.99 / 0.538 ms wall time | 5.6× on six persistent workers |

The `BM_FlatForest100k` diagnostic isolates a forest in which every instance
contains exactly one fully-resident renderable node. The view is static, output
and cache capacity are warm, and setup is excluded. `TLAS only` calls the
internal BVH8 query without materializing cut entries; the other columns run
the public selection path. Values are the best of three repeats after direct
flat-instance emission:

| Flat forest | Visible | TLAS only | Uncached selection | Warm `View` selection |
|---|---:|---:|---:|---:|
| One shared asset | 25,000 (25%) | 0.133 ms | 0.265 ms | 0.291 ms |
| 100k unique pages | 25,000 (25%) | 0.133 ms | 0.304 ms | 0.346 ms |
| One shared asset | 100,000 (100%) | 0.324 ms | 0.872 ms | 0.967 ms |
| 100k unique pages | 100,000 (100%) | 0.324 ms | 1.42 ms | 1.96 ms |

The initial 100k quality TLAS build takes 38.7-41.4 ms and produces
37,449-39,675 BVH8 nodes for these two spatial layouts. The 25%-visible warm
`View` uses 3.91 MiB; the all-visible case uses 5.46 MiB. The equal TLAS-only
times for shared and unique content confirm that page identity does not affect
spatial culling. The remaining gap above TLAS-only is precise-box validation,
compact entry construction, and output writes; unique mounts additionally
require scattered page-generation reads for the returned node handles.

`BM_MixedForest100k` holds geometry and visibility constant while varying the
content mix. Exactly 25,000 instances are visible. The hierarchical half uses
one shared, fully resident 85-node, 4-ary depth-3 asset; the flat half uses one
shared one-node asset. The 4-pixel threshold selects four entries per visible
hierarchical instance and one per flat instance. Best of five repeats:

| One-node instances | Hierarchical instances | Cut entries | Uncached selection | Warm `View` selection |
|---:|---:|---:|---:|---:|
| 0% | 100% | 100,000 | 3.30 ms | 0.444 ms |
| 20% | 80% | 85,000 | 2.75 ms | 0.432 ms |
| 50% | 50% | 62,500 | 1.83 ms | 0.377 ms |
| 80% | 20% | 40,000 | 0.902 ms | 0.330 ms |
| 100% | 0% | 25,000 | 0.265 ms | 0.290 ms |

`BM_RootDecisionForest100k` isolates the hierarchical-root shortcut with
25,000 visible depth-3 BLAS instances. The near arm must refine each root to
four entries; the far arm accepts one renderable root. Five-repeat medians
before and after the shortcut show both the win and its no-win control:

| Root decision | Output entries | Before | Current | Change |
|---|---:|---:|---:|---:|
| Near, refine roots, uncached | 100,000 | 3.432 ms | 3.432 ms | flat |
| Far, accept roots, uncached | 25,000 | 1.923 ms | 0.649 ms | -66.3% |
| Near, warm `View` | 100,000 | 0.439 ms | 0.436 ms | -0.7% |
| Far, warm `View` | 25,000 | 0.289 ms | 0.277 ms | -4.2% |

These are scale indicators, not guarantees. Output size and cache locality can
dominate population size, and contended runs on this host can be much slower.
Optimization claims in the historical journal use interleaved baselines,
medians, win counts, and controls; headline current costs use the best recurring
repeat to show the speed-of-light floor. The tables above preserve the detailed
operation breakdown captured at this revision.

---

## 2. Historical experiment log

The entries below are chronological evidence. Names such as `UserId`,
`ViewScratch`, or an older timing describe the revision under test, not the
current API. Later entries explicitly supersede earlier mechanisms. The
current sources of truth are linked at the top of this document.

One entry per experiment; kept/reverted and why.

### A. Explicit AVX2 intrinsics for the wide kernels — KEPT

Theory: MSVC's autovectorizer handles the per-plane sign-select lane loops
poorly (scalar blends, no FMA). Rewrote `testWideAabb`, `distanceToBoxes`,
`screenError8` with explicit AVX2 (`blendv` by plane-sign masks, fmadd chains,
`movemask` for lane verdicts), scalar fallback preserved behind `#if`.
The scalar single-box functions were rewritten with `std::fma` in exactly the
wide path's operation order, so scalar reference and SIMD path remain
bit-identical and the brute-force equivalence tests still hold exactly.

Results (`bench_results/01_avx2.txt`): DeepTree −14..20%, GcStress −10%,
TeleportWithCut −33% (it is plane-test-bound: degraded bounds keep every
plane undecided), ManyShallowTrees unchanged (its cost is per-instance
overhead, not math). Kept.

### B. DFS walk with carried state + leaf fast path — KEPT

Theory: the original walk was a forward scan over the page's preorder arrays
gated by epoch stamps. That costs six scratch arrays (`live/err/planes`
scattered by the parent's wide test, `alive/seen/sticky` written at the visit),
a read-modify skip (`subtreeSize`) for every culled subtree, and `parent[]`
reads on the hot path. Replaced it with an explicit DFS stack whose entries
carry everything the visit needs (`node, err, planes, alive`) — computed once
by the parent and never round-tripped through memory.

Two extra structural wins fell out:

- **Leaf fast path.** `WideBlock` gained a `leafMask` (lanes whose child has
  no children and is not an expansion point). Surviving leaf lanes are
  emitted into the cuts straight from the parent's SIMD test — no stack push,
  no `meta`/`parent`/`subtreeSize` read, no scratch access. In a fanout-8
  tree ~87% of all nodes are leaves, so most of the tree never gets "visited"
  at all. A stacked node is therefore known to have children, which also
  removes the `hasKids` check from the visit.
- **Scratch shrank to one word.** The only state that must persist across
  frames is the hysteresis history; it is now a single packed
  `seenSticky = (frame << 1) | lastDecision` per interior node, and is not
  even allocated when the caller runs with `hysteresis = 0`.

Results (`bench_results/02_dfs.txt`): ManyShallowTrees/50k −45% (9.85 → 5.44
ms), DeepTree_FlyThrough/6 −16%, DeepTree_Static/6 −7%. Tiny-page streaming
benches (GcStress, PagedPlanet) regressed ~5–8% — their per-node cost is
dominated by the expansion-link hash lookup, which the DFS does nothing
about (addressed in experiment C). Kept.

### B2. The ordering trap (a correctness scare worth documenting)

After B, `GcStress` reported a 10× smaller average cut. Not a speedup: the
attach trajectory had changed. Frames 1–2 from identical state were identical
(the walks are equivalent — the brute-force reference tests also prove this
per-state), but the DFS emits ideal-cut entries deepest-first, and the bench
attached "the first N" `NEEDS_EXPANSION` entries in output order. Under DFS
order that policy blew the whole per-frame budget over-refining one small
region near the camera, which the camera then left behind — pages aged out,
were collected, re-demanded: permanent churn (~230 attached pages, never
converging) versus the old preorder's accidental breadth-ish coverage
(~1900 attached pages, converging).

Fix: streaming policy must attach by **priority (screen error), descending**,
not by output order — the order of ideal-cut entries is traversal-defined and
unspecified. With a priority policy both implementations produce *identical*
trajectories (`attached=205`, `avg_cut=18.8k`, same attach/collect rates).
The benches now model that policy (`attachTopByPriority`), and the API
contract documents that ideal order must not be relied upon.

### C. Expansion-slot side array — KEPT

Theory: after B, the tiny-page streaming benches (GcStress, PagedPlanet) were
~5–8% behind the old walk. Their pages are 21 nodes with 16 expansion points,
and every visited expansion point did an `unordered_map` lookup
(`expansionLink_`) per frame. Added `PageRt::expSlot` — a per-node array of
attached child slots, allocated lazily on a page's first attach — so the walk
does one indexed load instead of a hash probe. The hash map remained as the
by-id index for the cold paths (`detachPage`, `isAttached`) until experiment
J deleted it outright.

Result: GcStress 1855 → 1682 µs (now ahead of the pre-B code at 1712 µs),
PagedPlanet back to parity. Kept.

### D. Pinned pages outside the LRU + prefetch pipeline — KEPT

Two parts:

- **Pinned pages never enter the LRU list.** Root pages of instances cannot
  be collected, yet every walked page did an LRU unlink/relink once per
  frame. Measured effect: none (below this machine's noise floor) — kept
  anyway because it is strictly less work and makes `lruTouch` a single
  compare for pinned pages.
- **Software prefetch across visible instances.** The forest case
  (50k instances) spends its time in a chain of dependent loads per instance:
  `Instance` record → page slot (`PageRt`) → wide block / meta / userId
  arrays. `selectCut` now prefetches instance i+2's record and instance i+1's
  root-page arrays while walking instance i.
  Result: ManyShallowTrees/50000 median 6.55 → **3.18 ms** (−51%), and the
  run-to-run variance collapsed (CV 13% → 2–5%), confirming the walk was
  memory-latency-bound, not compute-bound.

### E. Prefetch on DFS push (deep trees) — REVERTED

Theory: mirror experiment D inside a page — when the walk pushes an interior
child onto the DFS stack, prefetch that child's wide block and id line, to be
consumed at pop time. Result: no measurable change on DeepTree/6 (±3%,
inside noise). Explanation: pages are preorder-laid-out and the DFS visits
them in near-linear address order, so the hardware prefetcher already has
the lines; unlike the forest case there is no pointer-chase between
unrelated allocations. Reverted to keep the walk clean.

### F. Two-tier TLAS rebuild (median-split vs Morton) — KEPT

`MovingInstances/50000/1000` (1000 random teleports per frame — worst case
for any BVH) sat at 9.2 ms. Splitting the cost with policy extremes:
never rebuilding gives 12.0 ms (queries degrade against bloated grow-only
lanes), rebuilding every frame gives 26.3 ms (the recursive median-split
build alone costs ~17 ms at 50k leaves). The escape-threshold policy
(rebuild at a 25% escape threshold) was already the best of the three.

Change: two build paths.

- **Structural rebuilds** (instance added/removed) keep the median-split
  build: they are rare, the result is long-lived, and build quality earns
  its keep — with a Morton-only build the contribution-culled forest bench
  (which leans on tight interior `maxErr`/bounds lanes) regressed 431 →
  902 µs, measured stable at CV 1%.
- **Motion rebuilds** (distinct-leaf escape threshold) use a Morton build: one 63-bit-key
  sort, then contiguous groups of kWide per level, bottom-up. ~5x cheaper to
  build; slightly looser tree, but it only has to survive until the next
  motion rebuild anyway.

Also: the TLAS query stack became a reused member (it heap-allocated per
`selectCut` call).

Result: MovingInstances/50000/1000 9.2 → 6.2 ms (−33%, CV 2.7%), static
forest quality unchanged (ManyShallowTrees/50000/1 back at ~431 µs). Kept.

### G. Optional outputs — KEPT AT THE TIME, SUPERSEDED BY THE UNIFIED CUT

A fully-resident static scene emits an ideal cut identical to the actual cut:
pure wasted bandwidth, and at 260k entries per frame the outputs *are* a
material part of the walk. `selectCut` now takes the ideal cut and load
requests as nullable pointers (a reference overload keeps existing call sites
source-compatible). Passing nullptr skips those emissions entirely.

Result (controlled, 3 reps): DeepTree/6 4.77 → 3.07 ms (−36%) for cut-only
callers. Zero cost for callers that keep all outputs. Kept.

### H. Kitchen-sink benchmark + GC watermark fix

Added `BM_Combined_KitchenSink`: one world holding a streamed paged planet
(12 attaches + GC per frame), 20k prop instances (500 drifting, 5
teleporting per frame), a 10k-leaf jitter tree (2000 refits per frame), and
instant payload streaming — 38k-entry cut, ~3.8 ms per frame all-in.

The bench exposed an API wart: `collect`'s watermark compared against
`attachedPageCount()`, which includes pinned root pages (20k of them here),
so the collector ran unbounded — every aged page was evicted immediately
instead of keeping a cache up to budget. The watermark now budgets
`streamedPageCount()` (attached minus pinned), which is the set the
collector can actually influence.

### I. Handle-based motion API — KEPT

`setNodeBounds(UserId, …)` pays one `nodeMap_` hash lookup per move; at 800k
map entries that lookup is a guaranteed cache miss (~320 ns) and dominated
the typical-forest breakdown at 80k movers/frame. New entry point:
`handleOf(id)` resolves once to an opaque `NodeHandle{slot, index,
generation}`, and `setNodeBounds(handle, …)` queues the move with no lookup
at all. Pending moves are now stored pre-resolved with the generation stamp,
so the flush validates each entry with two loads and no hashing; a handle
whose page detached (or whose slot was reused by a later attach) fails the
generation check and is skipped — the same semantics the id path always had.

`BM_TypicalForest_Breakdown` A/B (`arg1` selects id vs handle path):

| scale | by id | by handle |
| --- | --- | --- |
| 10k trees, 16k movers | 1.88 ms (0.78 move + 0.75 refit + 0.34 cut) | 1.18 ms (0.25 move + 0.62 refit + 0.30 cut) |
| 50k trees, 80k movers | 28.8 ms (12.5 move + 12.8 refit + 3.5 cut) | 16.4 ms (1.3 move + 12.3 refit + 2.8 cut) |

Submission drops to ~16 ns/mover (10× at 50k). Note the id-path hash cost
now shows up in `move_us` rather than `refit_us` because resolution moved to
submission time. The remaining ~12 ms refit at 80k movers is real grow-only
propagation work (cold page touches), not lookups — experiment K chased it
with explicit dedup and found the walk was already optimal.

(Superseded by experiment J: the id-taking overloads and `handleOf` no
longer exist; handles are composed from attach results instead of resolved.)

### J. Fully handle-based API — hash maps deleted — KEPT

Experiment I proved handles beat hash lookups for motion; the obvious next
question was why keep hash lookups at all. Answer: no reason. Almost every
id the caller passes *into* the World originally came *out* of `selectCut`
— and at emission time the walk already knows the node's (slot, index). The
rest are known at attach time, because page-local indices are immutable
authored data. So:

- `LoadRequest` and `IdealEntry` now carry a `NodeHandle` next to the
  payload. The streaming round trip is: request comes out with a handle →
  content is loaded by payload (the caller's naming, opaque to us) →
  completion calls `markResident(req.node)` / `attachPage(entry.node, page)`.
  Zero lookups; a page collected mid-load makes the handle stale, the
  generation check catches it, the completion is safely dropped.
- `addInstance` / `attachPage` return a `PageHandle{slot, generation}`;
  handles for movers (or anything else known up front) are composed as
  `nodeAt(pageHandle, authoredIndex)`. No resolution step exists at all.
- The former `UserId` is renamed `UserPayload` and is fully opaque: an id,
  a pointer — echoed in outputs, never interpreted, never indexed, and
  duplicates are legal (the builder's uniqueness check is gone too).
- `nodeMap_` (id → node) and `expansionLink_` (id → attached child slot)
  are DELETED. Attach/detach no longer do O(page) hash inserts/erases —
  that was a hidden per-churn tax on every page cycled through the cache —
  and the map's ~40 B/node footprint is gone. The expansion link lives
  solely in the per-slot `expSlot` array from experiment C.
- Tests keep addressing nodes by payload through a deliberately-slow
  brute-force scan in `TestAccess` (`findByScan`), which is not part of the
  production API.

Controlled A/B (previous commit in a git worktree, binaries interleaved,
3 repetitions, medians):

| bench | with hash maps | handle-only | delta |
| --- | --- | --- | --- |
| GcStress_FastFlythrough/96 | 1.93 ms | 1.43 ms | −26% |
| LeafRefit_Teleport/100k | 7.36 ms | 3.00 ms | −59% (2.4×) |
| ResidencyChurn/10000 | 577 µs | 431 µs | −25% |
| Combined_KitchenSink | 3.41 ms | 2.89 ms | −15% |

The refit and residency wins are the per-call hash misses; the GC-stress
and kitchen-sink wins are mostly the attach/detach map maintenance. The
API also got *simpler*: one opaque currency flows out of `selectCut` and
back into every mutating call, and there is no id index to keep coherent.

### K. Lazy motion queue — KEPT; select-time flush — SUPERSEDED; explicit refit dedup — REJECTED

Two API asks landed together: a node can move many times per frame, so
refit-per-move is waste; and the tree only has to be correct at a publish
boundary. The flat queue and refit algorithm remain current. At this point in
the history, however, the boundary was `selectCut`:

- `setNodeBounds` stays a ~16 ns flat queue push, but nothing was applied by
  the frame clock. The pending queue was flushed inside `selectCut` or by an
  explicit `flushBounds()` for tools and tests.
- Contract: after the flush, a node's own bbox equals exactly the *last*
  submitted box; ancestors are conservative (grow-only) over everything
  submitted. Multiple views flushed once because later calls found an empty
  queue.

The current API moves that same work to explicit `applyUpdates`, before any
view selection. This is what lets `View::selectCut` accept a const World and
run concurrently across distinct views.

The second half — replacing the per-mover ancestor walk with an explicitly
deduplicated bottom-up sweep — was implemented three different ways, all
measured SLOWER than the walk they replaced, and all reverted:

| flush variant | forest 50k refit | teleport 100k | 1M repeat subs |
| --- | --- | --- | --- |
| eager walk, contains() early-out (baseline I+J) | 11.5 ms | 3.6 ms | — |
| per-page dirty lists + max-heap sweep | 20.5 ms | 7.3 ms | 23.7 ms |
| per-page move chains + depth-bucket sweep | 18.8 ms | 3.6 ms | 22.9 ms |
| per-node epoch stamps (newest-first scan) | 14.5 ms | 3.5 ms | 23.0 ms |
| transient hash-set coalescing | 13.3 ms | 4.4 ms | 28.0 ms |
| **flat queue, walk per submission (KEPT)** | **11.7 ms** | **3.1 ms** | **29.9 ms** |

Why the fancy versions lose: every dedup scheme needs at least one extra
cold cache line per mover (a stamp, a dirty flag, a chain head — scattered
across 50k pages) or a large transient table, while the thing it saves —
the ancestor walk — already terminates at the first ancestor whose box
contains the change. Grow-only bounds make that termination almost
immediate in steady state: a shared parent is grown by the first mover and
merely *re-checked* (one hot contains()) by the other 999. Repeated moves
of one node are the same story — the second application rewrites the same
hot bbox and lane and stops at the parent, ~30 ns measured at 1M
submissions/frame. The contains() early-out IS the dedup; bookkeeping to
avoid a nearly-free walk costs more than the walk.

What did stick from the exercise besides the lazy timing: nothing — the
final flush is still the experiment-I loop in `flushBounds()`, now invoked by
`applyUpdates`. Zero new state on pages, zero new state per node.

### L. Dynamic instance churn (spawn/despawn) + builder cost — two cliffs FIXED

Question: what does it cost to add/remove whole trees at runtime — 5% of
the forest removed and 5% spawned fresh, every frame, on top of the usual
movers and camera? `BM_TypicalForest_Churn` answers it, and the first run
found two policy bugs that only continuous churn could expose:

1. **`removeInstance` scanned every slot in the world** (twice) to find the
   instance's pages — O(total pages) per removal, ~16 ms/frame at just 10k
   trees. Fixed: pages of an instance form a tree hanging off its root
   slot via the expansion-slot links, so removal now walks exactly its own
   pages (preorder collect, reverse-order detach — children before
   owners). This also makes the design doc's stated removal complexity
   actually true.
2. **Every add/remove forced the quality (median-split) TLAS rebuild** — a
   policy written when structural changes meant "level load", not "50
   spawns per frame". Fixed with a drift threshold: structural changes
   still mark the TLAS dirty, but the quality build runs only when the
   population moved >20% since the last one; steady churn takes the Morton
   path (~5x cheaper), same as motion escapes. Assembly and mass despawn
   still get the tight tree.

   That fix made the rebuild *cheaper* and left the rebuild in place, which a
   later audit found was the whole problem: one spawn still cost a full
   rebuild — 2.1 ms at 20k instances and 9.5 ms at 80k, and exactly the same
   as five hundred spawns. Add and remove are now applied to the tree in
   place, in O(depth), and no longer mark it dirty immediately. An edit budget
   still requests a repair rebuild after enough accumulated changes. See the
   archived [handoff, section 11.1](archive/HANDOFF-2026-08-05.md)
   for the measurement and `tests/test_tlas.cpp` for the invariants an in-place
   edit has to maintain.

| churn bench at this experiment's revision (5% out + 5% in/frame) | before | after |
| --- | --- | --- |
| 10k trees (500+500 churn/frame) | 21.6 ms | 4.0 ms (0.8 churn + 0.25 move + 1.8 refit + 1.2 cut) |
| 50k trees (2500+2500 churn/frame) | not viable (seconds) | 31.5 ms (6.4 churn + 1.4 move + 16.9 refit + 6.8 cut) |

At that revision, non-churn scenarios were unchanged and a spawn or despawn
cost ~1.3 µs all-in (page copy, registration, residency, TLAS share). Fresh
pages made refit and cut a little hotter, and the TLAS re-sorted every frame
under 5% churn. The later incremental-edit change removed the remaining
single-spawn rebuild cliff; use the current `BM_Spawn_MarginalCost` rather than
this historical churn table for production budgeting.

**Builder cost** (`BM_Builder_BuildPage`), for composing trees at runtime —
createRoot/createNode per node plus `build()` (preorder layout, wide-block
packing, invariant checks):

| page shape | nodes | build time | per node |
| --- | --- | --- | --- |
| shallow prop (4^1) | 5 | 1.5 µs | ~300 ns |
| typical deep tree (4^3) | 85 | 11 µs | ~130 ns |
| large streamed page (8^4) | 4681 | 0.55 ms | ~118 ns |

Composing a tree at runtime is dominated by everything *around* the build
(content, IO), not the build: ~8M nodes/s steady. A spawned forest tree is
1.5–11 µs of builder time, and the prototype-copy pattern in the churn
bench (author once, copy the immutable `Page` per spawn — legal because
payloads are opaque and may repeat) skips even that.

### M. Test hardening: contracts, edge cases, and the corner-case benches

Closing the audit list produced two real API fixes and a battery of tests
that pin the invariants down, plus seven benches that measure the corners no
existing scenario covered. All numbers are in the final table above.

**Fix 1 — InstanceRef ABA.** Instance slots are recycled via a LIFO free
list, so `remove(A); add(B)` reuses A's id — and a stale id kept by the
caller (a pooled prop's cached ref, an in-flight despawn) would move or kill
B. `InstanceRef` now carries a generation stamp exactly like `NodeHandle`;
`moveInstance`/`removeInstance` resolve it and no-op when stale.
`Contracts.StaleInstanceRefIsIgnored` locks the behavior in. No hot-path
cost: the stamp is checked once per instance-level call, not per node.

**Fix 2 — NaN bounds rejection.** `setNodeBounds` validated boxes with
`!isEmpty()`, i.e. `!(mn.x > mx.x)` — every NaN comparison is false, so a
NaN box sailed through, and grow-only refit means one poisoned box corrupts
ancestor bounds *forever* (nothing ever un-grows). The check is now a
positive ordering + finite-extent test that rejects NaN, infinities, and
empty boxes in one predicate. `LeafRefit` throughput is unchanged (~32 M
submissions/s).

**New unit tests** (`test_contracts.cpp` + additions elsewhere, 14 total):
cut/ideal antichain + residency invariants fuzzed over random paged worlds;
byte-identical determinism across identically built worlds (the then-current
hysteresis path); multi-view state isolation (originally scratch, now covered
by independent dampers and `View` objects);
`collect()` minAge boundary and exact `freedPayloads` accounting;
`screenError8` and degenerate-box (zero-extent, camera-inside, far-off-axis)
wide-vs-scalar equivalence; camera-inside-tree, point leaves, 1e6-offset
worlds, and 0.01×–250× instance scales against the brute-force reference;
memory budgets (bytes/node ≤ 128 at fanout 8; `PageRt` ≤ 512 B) so per-node
bloat or a returning hash map fails CI.

**New benches, and what they showed:**

- *Output sensitivity* (`BM_CutScaling_OutputSensitivity`): same 2.4M-node
  world at thresholds 64→1 px. Cost tracks output size at ~11–19 ns/entry;
  the claim "output-sensitive, not N-sensitive" is now measured, not argued.
- *Teleport spike* (`BM_CameraTeleport_ColdFrame`): every 16th frame jumps
  to a cold region. The spike is 1.36 ms vs 1.05 ms steady — only 1.3×,
  because the walk has no per-frame caches to rebuild; epoch stamps make
  stale scratch free to ignore.
- *Multi-view* (`BM_MultiView`): 3 extra views cost 0.84 ms each vs 1.45 ms
  for the main view — page data is hot after the first walk.
- *Streaming convergence* (`BM_StreamingConvergence`): originally "frames
  until ideal == actual" — a metric that turned out to be wrong twice, and
  was replaced entirely; see experiment N.
- *TLAS scale* (`BM_TlasScale`): 200k → 500k instances leaves the steady
  frame at ~10 ms for a 34k-entry cut (output-bound); the level-load first
  cut (quality TLAS build) grows 121 → 416 ms and is the number a loading
  screen must hide.
- *Adversarial* (`BM_Adversarial_*`): 10k co-located instances cost 1.5 ms
  (37 ns/entry — the no-spatial-separation floor); a maximal 511-child node
  cuts in 4.7 µs; a 26-page expansion chain costs ~170 ns per page crossing
  for a single-entry cut.

### N. Teleport convergence: error-decay metric + predictive expand depth — KEPT

The convergence bench went through three formulations; the first two were
measuring the wrong thing, and the fix ended up producing a streamer policy
worth documenting as the recommended pattern.

**Formulation 1 (wrong): binary "ideal == actual", infinite view.** With a
10⁹ far plane the frustum holds the whole 8 km world; screen error falls as
1/distance, so nodes 7 km away still project thousands of pixels and demand
expansion. "Converged" meant expanding half the planet: budgets 8/32/128
gave 26.75/3.6/30 frames — non-monotone garbage from 2–5 survivor-biased
samples (cycles that never converged inside the 200-frame cap were silently
excluded, flattering the low budgets).

**Formulation 2 (also wrong, differently): finite far plane.** Clipping at
1.5 km bounded the frontier and made the numbers scale cleanly with budget
(70/18/5.6 frames) — but it models the wrong world. A collapsed node is
never culled by this system; it renders coarse until its page arrives, and
a planet game *shows* the far field at low detail rather than clipping it.
Hard-culling distance in the bench threw away exactly the case that
matters: distant objects deserve *slower convergence*, not none.

**Formulation 3 (kept): error decay.** Unbounded view restored. The metric
is the worst residual screen error among still-expandable NEEDS_EXPANSION
entries, sampled 1/2/4/8/16/32 frames after each teleport (clamped at 10⁶ px
— the camera standing inside an unexpanded box saturates err through the
1/distance term, and one such frame would swamp the average). Convergence
is not a binary event; it is a decay curve, and "near field in frames, far
field in seconds" falls out of the err-priority attach order by itself.

**The predictive policy.** The walk fundamentally cannot see below a
missing page, so pure discovery pays one frame of latency per page level: a
D-page chain costs D frames at any budget. But the entry's error already
says how deep the chain goes — `levels ≈ ceil(log2(err/threshold) / d)`
with d halvings per page level; a 100 px entry at a 4 px threshold is more
than one level with certainty. The bench's predictive streamer keeps a
max-heap of candidates keyed by measured (from the walk) or estimated (from
freshly built page data: geomError × k / distance-to-bbox) screen error,
pops the globally worst, attaches, and pushes the new page's over-threshold
expansions — multiple levels per frame, still in global priority order, no
World changes at all (`attachPage` returns the handle; `nodeAt` composes
the next level's expansion handles).

**A first cut of the predictive policy lost, instructively.** Plain
depth-first recursion (expand this entry's chain fully, then the next
entry) was *worse* than discovery beyond frame 2: each level fans out ×16
candidates, so the first teleport hotspot swallowed the entire 32-page
frame budget and starved every other hotspot. The global heap fixes it;
lookahead must not override priority order.

**Results** (worst residual px after a teleport, budget 32 pages/frame,
chains 3 pages deep in this world):

| frames after teleport | 1 | 2 | 4 | 8 | 32 |
|---|---|---|---|---|---|
| discovery  | 387k | 97k | 284 | 190 | 95 |
| predictive | 387k | **800** | 269 | 190 | 95 |

Frame 1 is identical by construction (sampled before the frame's attach).
At frame 2 discovery is still a blob world — the camera's own chain has two
levels to go — while predictive has already landed the whole near field:
**~120× lower residual at the same page budget**, converging to identical
steady state by frame 4 (D = 3 here; deeper worlds widen the gap since
discovery needs D frames, predictive still needs ~1). Attached-page count
and frame cost are equal within noise; misprediction waste just ages out
through the GC. The formula and the heap pattern are documented in
hlod_design.md as the recommended streamer behavior.

### Measurement honesty note

This machine's noise floor is high: the same binary measured 20 minutes apart
moved 9.9 → 12.0 ms on `LeafRefit_Teleport/100000` (±20%). A scare that
experiments B–D had regressed the refit path by ~28% dissolved under
bisection — the refit code is byte-identical across those commits and all
variants sit inside the same noisy band. Only deltas well above 20%, or ones
confirmed with repetitions and low CV, are treated as real in this journal.

### O. Follow-up hot-path audit -- KEPT, OUTPUT PATH LATER SUPERSEDED

The 2026-08-05 audit tightened the architecture that existed at that revision.
The request-deduplication and separate-output details below were later replaced
by the unified cut; the other listed mechanisms remain:

- `Instance` is now one 64-byte cut-path cache line. Bounds, masks, asset id and
  TLAS back-pointers occupy a parallel 64-byte `InstanceTlas` stream, so a cut
  does not fetch spatial-maintenance state it never reads. This improved the
  measured 80k selection cases by 5.8-6.7% and a 50k motion case by 7.8%.
- A dense `liveInstances_` list makes TLAS rebuild enumeration proportional to
  current population rather than historical slot count. With 100k peak slots
  and 10k survivors, rebuild time fell 9.0%; selection remained neutral.
- Morton rebuilds use a stable six-pass, 11-bit LSD radix sort with retained
  scratch. End-to-end rebuild cost fell 36.2% at 100k instances and 42.0% at
  500k; a 100k coincident-centroid stress case improved 6.4%. Equal keys retain
  deterministic live-instance order rather than paying for a second sort.
- TLAS contribution culling now uses squared box distance and
  `screenErrorFromSq8`, matching the page walk's reciprocal-square-root path and
  improving the focused 50k-200k cases by 3.0-3.5%.
- Request deduplication at that revision lived in per-selection scratch rather
  than mutable page state. Parallel uncached workers merge their private
  request sets; concurrent cached views cannot overwrite one another's
  epochs.
- `View` budgets both camera-envelope travel and projection-scale
  travel. Each 48-byte record stores a conservative flip-point slope, avoiding
  the old all-cache invalidation throughout damped zoom-out. The affected 20k
  benchmark improved 38.9%. Reset retains warm storage, improving reset+cold
  selection 21.9%.
- Contextual selection prefetches its spatially ordered random instance and
  record reads eight entries ahead, and the output sink directly pushes its
  dominant one-entry runs. The then-current 20k fly-through reached 0.121 ms.
- TLAS stacks, visible lists, traversal workers, request stamps, and
  statistics are owned by each `View`. `applyUpdates` performs the
  serial refit/rebuild work first; `View::selectCut` then accepts a const World
  and independent views may select concurrently.
- Page touches accumulate in an optional `PageUsageContext`. Selection does
  not mutate the intrusive World LRU; `collect` consumes only the important
  cameras supplied by the caller.
- The TLAS escape budget now counts each escaped instance once per rebuild,
  rather than counting every later growth of the same lane. A bounded 5% moving
  cohort therefore stays on incremental refit instead of periodically charging
  a repair to a view; `applyUpdates` owns any repair. At that revision the
  representative 80k selection was 0.50 ms, and six nearby moving-object views
  measured 4.23 ms serial and 1.08 ms on six persistent workers.

### P. Unified cut and recursive resident cover — COVER KEPT, OUTPUT LATER SUPERSEDED

At this revision selection emitted one sequence instead of separate current,
ideal, and load-request streams. `Shared`, `CurrentOnly`, and `IdealOnly`
encoded the additive differences, while `Direct` and `NeedsExpansion`
distinguished known payloads from unknown topology. Section Q superseded that
record layout without changing the traversal or resident-cover algorithm.

Residency is no longer restricted to resident immediate children. Each mount
propagates complete descendant coverage upward, so a resident detailed node can
replace an unavailable intermediate proxy. The propagated bit is the common
O(1) decision. At a frustum boundary, an uncovered branch outside the view need
not block refinement, so selection recursively validates only visible branches
and records every inspected page for optional page-use feedback. Frustum-edge
instances are deliberately not retained by `View`.

The resulting traversal carries current and ideal liveness together, preserves
the replace-only antichain for both cuts, and keeps selection read-only with
respect to the World. Current representative measurements are in
section 1 and the README; older output-path timings above are historical.

The archived [2026-08-05 handoff](archive/HANDOFF-2026-08-05.md), section
14, preserves the interleaved A/B tables, rejected margin vectorization,
radix-sort tradeoffs, and full measurements. It is historical evidence, not API
guidance.

### Q. Compact bucketed cut output — KEPT

Membership moved from every entry into three output vectors: `shared`,
`currentOnly`, and `idealOnly`. The expansion tag also disappeared. A
high-error ideal-side leaf is enough for the caller to consult its external
content graph; terminal and expandable leaves no longer require different hot
records. Payloads are resolved on demand instead of copied into every view.

`NodeHandle` is now a logical 64-bit value (20-bit mount slot, 20-bit node
index, 24-bit generation). The dense instance id and threshold-relative error
share one 32-bit word (24 + 8 bits), making `CutEntry` 12 bytes. The quantized
error preserves the exact above/below-threshold decision and retains roughly
eight priority levels per octave. After the later cache-state split, the 80k
moving-camera `View` footprint is 5.98 MiB; the tagged output at its measured
revision used 12.13 MiB in the same scenario.

### R. Runtime layout audit and dead experiment removal — KEPT

The follow-up memory audit removed storage that multiplied by assets, mounts,
instances, visible hits, queued edits, or traversal depth. Exact 64-bit layout
changes were:

| Structure/state | Before | After audit R |
|---|---:|---:|
| `AssetRt` | 208 B | 112 B |
| `PageRt` | 232 B | 112 B |
| mounted-node residency state | 6 B/node | 3 B/node |
| `InstanceTlas` | 64 B | 48 B |
| `View::Rec` | 48 B | 44 B |
| `PageUsageContext::Rec` | 12 B | 8 B |
| visible hit / TLAS stack item | 8 B / 8 B | 4 B / 4 B |
| node / page DFS item | 12 B / 32 B | 8 B / 24 B |
| queued bounds edit | 64 B | 48 B |
| Morton sort item (two retained buffers) | 16 B | 12 B |

The largest fixed duplication was an 88-byte `PageView` beside every owned
asset and another in every mount. `Page` now represents owned or borrowed
storage with one object per asset; mounts reach it through the asset index they
already held. Residency and complete-cover bits share one byte, while the
covered-child count is 16-bit under the authored fanout-511 limit. Existing
20-bit page/node and 24-bit instance limits also carry traversal state instead
of widening stack records. Instance TLAS placement, lane, and escape state
share one word. Morton key halves avoid four bytes of pair tail padding in both
retained radix-sort buffers. These encodings do not lower any published
capacity limit.

Three completed measurement scaffolds were removed from production code:
whole-page deformation overlays, packed-64 plane-mask propagation, and the
288-byte `WideBlock` padding control. The retained design is bounds-only
overlays, byte-array plane masks, and the 256-byte block. Their original A/B
evidence remains in the archived handoff; keeping dormant branches would make
the supported implementation and its size contracts ambiguous.

The representative measurements at this revision were recorded before the
later cache-state and fully-resident traversal work. Besides the memory
reduction, the compact arrays improved the best observed 80k selection cases
from 0.586/0.433/0.460 ms to 0.516/0.382/0.418 ms for moving-camera+movers,
fixed-camera+movers, and moving-camera+static respectively. The 10k cases are
0.121/0.105/0.074 ms. As throughout this journal, those are repeated best-case
floors on a noisy shared host, not latency guarantees.

### S. Split View hit state — KEPT

The per-instance cache record was split into a 32-byte hot proof record and a
4-byte cold slab-capacity record. The common first page dependency remains
inline; an 8-byte second dependency array is allocated only if a cacheable walk
actually touches two pages. A parallel 4-byte instance-version stream lets a
cache hit avoid fetching the 64-byte `Instance`; misses prefetch that record
after the hit decision.

Interleaved A/B results on the representative 80k workload were deliberately
mixed by reuse rate: moving camera plus 5% movers was effectively flat (best
+0.1%, median -1.3%), while moving camera plus static objects improved 4.7-5.2%.
Across six serial views, static objects improved 11.4-13.6% and 5% movers
improved 7.0-7.7%. The retained state fell from 6.59 to 5.98 MiB per moving
view. Uncached selection regressed about 1%, an accepted trade because it does
not use the cache hit path this change targets.

### T. TLAS hot/cold split — REVERTED

Reordering the existing 320-byte node to place default-query validity in the
first four cache lines was neutral. A stronger experiment split it into a
256-byte bounds/child stream and a parallel 64-byte mask/error stream, keeping
total memory unchanged. End-to-end cached and uncached best times regressed by
0.6% and 0.5%; medians were effectively flat. The added parallel-state
maintenance was therefore removed, and `TlasNode` remains one 320-byte record.

### U. Fully-resident shared-only traversal — KEPT

Each mounted page now has an 8-byte parallel summary: resident node count and
number of recursively incomplete attached children. Residency and attach/detach
transitions propagate only when full-tree status changes. A fully-resident
instance can therefore use a compile-time-specialized traversal that emits only
`shared` entries and skips residency tests, cover probes, and current/ideal
branching.

The broad 80k moving-camera benchmark improved about 1-2%, because TLAS and
cache work dominate it. The targeted deep fully-resident fly-through improved
16.1% in both best and median time and won all 8 paired runs. The summary costs
8 bytes per mounted page, not per node or instance. A streaming-convergence
control that forced the generic traversal showed no transition penalty: the
enabled build was 5.1% faster at best and 2.7% faster at median as resident
regions became eligible during the run.

### V. Direct one-node instance emission — KEPT; broader cache/TLAS variants REVERTED

Theory: a one-node asset has no cut decision to make. The TLAS already performs
coarse instance culling, its root payload is pinned, and the sole authored node
is necessarily in `shared`. Re-entering the normal page traversal therefore
loaded a 64-byte `Instance`, the then-112-byte mount header, and part of a 256-byte
wide block only to emit one known node.

The kept path lazily records a 4-byte flat-root marker per instance. Uncached walks
and cache misses use the exact world box already maintained for the TLAS,
retest it against any unresolved frustum planes (required because grow-only
TLAS lanes can be loose after motion), compute the node's error, and emit its
generation-stamped handle directly. A call-level dispatch preserves the old
loop exactly when the world contains no flat instances. Zero-error roots also
skip the distance calculation.

On the fixed 100k-instance, 25%-visible mixed benchmark, best-of-five uncached
times changed as follows; warm `View` times were effectively unchanged:

| Flat / hierarchical mix | Before | After | Change |
|---|---:|---:|---:|
| 0% / 100% control | 3.41 ms | 3.41 ms | noise |
| 20% / 80% | 3.06 ms | 2.84 ms | -7.0% |
| 50% / 50% | 2.53 ms | 1.91 ms | -24.6% |
| 80% / 20% | 1.98 ms | 0.958 ms | -51.7% |
| 100% / 0% | 1.63 ms | 0.307 ms | -81.1% |

The unique-page stress control improved even more: 25k visible flat instances
fell from 2.53 ms to 0.353 ms, while 100k visible fell from 20.97 ms to
1.47 ms. A variant that bypassed `View` caching for flat hits was reverted: it
made warm selection 12-61% slower across the mixed ratios. Tagging every TLAS
leaf and visible hit as flat was also reverted; the sub-1% mixed-case change
did not justify adding work to the universal TLAS query.

### W. Five-query hot-path audit — THREE KEPT, TWO REVERTED

Five independent theories were implemented and measured against interleaved
baseline binaries. Each focused comparison used seven paired rounds and an
unaffected or weakly affected control; the final absolute tables above use
fresh three- or five-repeat runs.

1. **Skip instance-version reads when the World is unchanged — REVERTED.** A
   call-level generation snapshot avoided the random `instanceCutVersions_`
   stream on static frames. Static fly-through was neutral (1.0007×, 3/7
   wins), the static mixed forest regressed 0.9%, and the moving control
   regressed 2.0%. The extra state and branch did not repay the cached read.
2. **Dispatch TLAS query policy once — KEPT.** Four template instantiations
   hoist default/layer-mask and zero/nonzero-`minPix` policy out of the node
   loop. The 25%-visible and all-visible flat TLAS queries improved 21.1% and
   16.6%; the `minPix` scale case improved 2.2%. All three won 7/7 rounds.
3. **Bypass zero-plane mask materialization — TLAS KEPT, PAGE WALK REVERTED.**
   Once a TLAS item is fully inside the frustum, its children inherit mask zero
   without initializing eight output masks. The narrowed TLAS-only change won
   7/7 rounds and improved the focused query 3.6%, while hierarchical and RNG
   controls were neutral. Applying the same branch inside every page block
   regressed the 100k hierarchical walk 2.1%, so that half was removed.
4. **Hoist instance-scale reciprocal — KEPT.** `toLocal` now computes one
   reciprocal and multiplies position, damping envelope, and six plane offsets.
   The 100k hierarchical walk improved 2.1%, and the 20k uncached fly-through
   improved 3.2% with static objects and 3.0% with movers; every subject won
   7/7 rounds. The flat-instance control, which bypasses `toLocal`, was flat.
5. **Specialize fully resident pages with no expansion nodes — REVERTED.** An
   immutable per-asset flag selected a metadata-light traversal. The 100k
   hierarchical case was 1.0003×, fly-through was 1.0025×, and the
   expansion-heavy streaming control was neutral. The flag, asset scan, and
   extra template instantiation were removed.

Against the original pre-audit binary, the retained combination improves the
25%-visible flat TLAS query 24.1% (7/7), the 100k hierarchical uncached case
2.7% (7/7), the 20k uncached fly-through 3.3% (7/7), and its cached arm 2.3%
(6/7). The deep single-instance control moved +3.1% with only 2/7 wins for the
final binary, consistent with the host's known whole-binary layout noise; the
focused retained experiments have their own controls and unanimous wins.

### X. TLAS hierarchical-root decision kept; temporal frontier reverted

Two larger selection changes were tested together and then separated.

**Renderable BLAS roots in TLAS leaves -- KEPT.** A hierarchical root does not
need a page walk when it already meets the error threshold. Single-root flags
fit in unused high bits of each TLAS node's valid-lane word, and the candidate
bit fits in the existing 4-byte visible hit. The query tests eight candidate
errors together, followed by an exact box/error check before emission; no
`TlasNode`, hit, instance, or cut-entry layout grew. Uncached views disable the
extra query work unless at least one quarter of visible instances terminate at
the root, probing every 32 calls for a coherent move back to distance. Cached
views test the root only on misses. The focused 25k-visible result is the
66.3% far-root win in the current table above, while the near/refined control
is unchanged.

**Search around the previous cut -- REVERTED.** A temporal-frontier prototype
started cache misses near the last selected nodes for fully resident,
single-page hierarchies. The tempting version was not a valid proof: a leaf
can remain spatially valid while the parent's LOD decision has changed. The
correct version revalidated the relevant ancestor decision before descending.
Only about 0.5-1.2% of visible instances then used the shortcut in the moving
camera workloads. The 20k cases were effectively flat versus the root-only
binary, while the 80k moving-camera/5%-mover case rose to about 0.519 ms from
roughly 0.47 ms. All frontier state and traversal branches were removed.

### Y. Compact page validation stamps -- KEPT

The 100k unique-page flat benchmark exposed a host-dependent cache cliff. A
warm `View` validated each visible dependency by reading `contentVersion` from
a 112-byte-stride `PageRt` array. The same logical validation now reads an
8-byte `PageStamp` stream containing content version, handle generation, and a
live flag. `PageRt` fell from 112 to 104 bytes, so adding the stamp does not
increase total per-mount fixed state. Generation remains available in the
already-loaded `PageRt` during a real hierarchy walk by sharing one 64-bit word
with the two 20-bit LRU links.

On an i9-12900K, seven alternating baseline/candidate rounds were pinned to
logical CPUs 0-15 and run at high priority. The median paired high-resolution
wall-time changes for unique flat pages were:

| Visible instances | Uncached selection | Warm cached selection |
|---:|---:|---:|
| 25k | -6.7% (7/7 wins) | -14.7% (7/7 wins) |
| 100k | -31.8% (7/7 wins) | -51.5% (7/7 wins) |

The primary 100k cached case moved from a 1.623 ms baseline median to 0.803 ms
for the new binary. Pure-hierarchy and pure-flat uncached mixed-forest controls
changed +0.2% and +0.1%; six representative view-update controls ranged from
-1.4% to +0.2%. The largest root-decision control movement was +1.4% with only
2/7 candidate wins, consistent with run/layout noise rather than a systematic
regression.

### Z. Mobile-memory follow-up -- TWO KEPT, AUTOMATIC GROUPING REVERTED

Three isolated experiments followed the page-stamp result. First, the
24-byte overlay-reference vector header moved out of every `Instance` and into
a cold pool allocated only for deformed instances. The live flag shares the
high bit of that pool index, reducing `Instance` from 64 to 32 bytes. The first
isolated seven-round pass improved the 80k cached fly-through 7.3% (6/7 wins),
the 50k moving-instance case 3.7% (6/7), and the 200k TLAS-scale case 16.1%
(6/7). A stricter final combined single-P-core run was more conservative:
80k cached/uncached fly-through improved 1.7%/2.1%, 200k TLAS scale improved
4.6%, and the moving-instance case was neutral. The direction plus halved
common state justified the localized pool; hierarchy controls stayed flat.

Second, large bounds overlays now keep a block-to-patch table and only the
modified `WideBounds`. Pages below 64 wide blocks stay on the original dense
path; large sparse overlays promote to dense after more than one sixteenth of
their blocks change. `WorkItem` stores the sparse overlay index in existing
tail padding, and template dispatch keeps the normal and dense inner loops
branch-free. In the 4k-instance one-leaf deformation case, overlay storage fell
from 125.7 to 73.4 MiB and selection improved 22.1% in 7/7 rounds. A stricter
single-P-core check measured 25.7% in 9/9. Contiguous 100k refits and ordinary
small-page forest controls were neutral.

Finally, a submission-order benchmark proved the remaining locality effect but
rejected automatic sorting. With 80k warmed overlays, grouped submissions took
4.02 ms while one fixed random permutation took 11.12 ms; flush alone was
3.38 versus 9.35 ms. Stable grouping inside `flushBounds` made 20k shuffled
updates 73% slower and did not improve the 80k flush (9.35 to 9.48 ms), so it
was removed. Callers should submit deformation updates grouped by instance or
page when practical; the library preserves submission order and pays no sort.

---

## 3. Constraints and follow-up recorded with this journal

- **Cached parallelism is across views, not within one cached walk.**
  Six representative 80k views fall from 2.50-2.99 ms serial to 0.445-0.538 ms
  on six persistent workers and occupy about 36 MiB of view state. An uncached
  View can still parallelize one call across visible instances. Combining
  both forms would add nested scheduling and merge costs; profile a production
  need before doing so.
- **Internal overlay bounds do not shrink.** Grow-only refit means long-running
  large teleports can loosen page ancestors. This costs culling efficiency, not
  correctness; TLAS rebuilds retighten only the top level. A budgeted bottom-up
  overlay pass is the remaining mechanism if real content exhibits degradation.
- **Selection uses compact bucketed output.** `shared`, `currentOnly`, and
  `idealOnly` avoid per-entry membership and expansion tags. Each 12-byte entry
  holds a 64-bit node handle, 24-bit dense instance id, and 8-bit
  threshold-relative error. Payload resolution and external topology lookup
  are intentionally caller-directed; callers should retain vector capacity.
- **Transform scope.** Instances support translation and uniform positive scale.
  Rotation and non-uniform scale need authored baking or an integration-layer
  representation; adding them directly would change bound transforms and hot
  instance state.
- **Host policy remains host policy.** Page size, GC watermarks/dwell, attach
  budget, predictive lookahead, and whether a `View` enables reuse are
  content- and IO-dependent decisions. The benchmark suite supplies the
  mechanisms' scale but cannot choose production values.
