# HLodTree — implementation architecture & performance journal

This document describes the implemented architecture (see `hlod_design.md` for
the full design rationale), explains why it is fast, and keeps an honest
journal of every optimization experiment — including the ones that did NOT
work and were reverted.

Measurement setup: MSVC 19.51 `/O2 /arch:AVX2`, 64-core EPYC @ 2.4 GHz,
single-threaded, Google Benchmark `--benchmark_min_time=0.4s`.

---

## 1. Architecture overview

Two levels, in the spirit of a ray tracer's TLAS/BLAS split:

- **Pages (bottom level).** The tree is a forest of immutable, flat,
  SoA **pages** — preorder node arrays built offline by `HLodBuilder`,
  loaded as one blob. Every node's children are mirrored into
  BVH8-style **wide blocks** (`WideBlock`: 8 child bounds, errors, indices,
  a valid mask and a leaf mask, SoA-transposed), which is what the walk
  actually reads. Pages link to child pages through **expansion points**;
  attaching/detaching pages is how planet-scale topology streams in and out.
- **TLAS (top level).** A small dynamic wide BVH over root instances
  (position + uniform scale). It owns multi-root worlds, instance motion and
  coarse culling, including error-based contribution culling (`minPix`).

One call — `selectCut` — walks the TLAS with tri-state frustum culling, then
runs one pruned pass per surviving instance and emits three outputs in a
single walk: the **actual cut** (what to draw now), the **ideal cut** (what
would be drawn if everything were resident; `NEEDS_EXPANSION` entries double
as topology requests), and **load requests** (payload residency wanted at the
actual cut's frontier). The ideal cut and requests are optional — callers
that don't stream pass nullptr and don't pay for them.

### Why it is fast

Everything below follows two principles the design was steered by:
**group work** (test 8 things per instruction, stream over flat arrays,
one walk for all outputs) and **be lazy** (touch nothing outside the output
region, apply updates only when a cut needs them, never clear per-frame
state).

1. **Output-sensitive traversal.** The walk descends only while a node wants
   to refine *in the ideal sense*; everything below the ideal cut and
   everything outside the frustum is never touched. Cost is
   `O(log R + visible instances + ideal-cut region)`, independent of total
   node count. There are no per-frame O(N) clears anywhere — per-view state
   is epoch-stamped, and traversal state is *carried* on an explicit DFS
   stack (`node, err, planes, alive`) instead of being scattered through
   memory by one pass and gathered by another.
2. **Wide everything.** A parent tests all its children with one AVX2 issue:
   masked tri-state frustum test (early-reject / early-accept per plane, so
   interior nodes of a mostly-visible scene test almost no planes), then
   8-wide distance and screen error. The TLAS uses the same wide layout and
   the same kernels.
3. **Leaves never get visited.** Wide blocks carry a `leafMask`; surviving
   plain-leaf lanes are emitted into the cuts directly from the parent's
   SIMD test. In a fanout-8 tree that's ~87% of all nodes skipping the
   visit machinery (no metadata reads, no stack traffic, no scratch).
4. **No hashes anywhere — the API is handle-only.** The World keeps no id
   index at all (experiment J): every entry point takes a
   `NodeHandle{slot, index, generation}`, composed from attach results plus
   authored node indices or emitted by `selectCut` itself; the 64-bit
   per-node payload is opaque and merely echoed in outputs. The walk runs
   on page-local indices, a per-page expansion-slot array, and slot
   indices; the forest walk software-prefetches the next instances'
   records to hide the one unavoidable dependent-load chain. Stale handles
   (page detached or collected since) fail a generation check and are
   safely ignored — the normal streaming-completion-vs-GC race.
5. **All-or-nothing refinement is O(1).** Every node keeps a
   `readyChildren` counter maintained by residency changes; "can I refine"
   is one compare, both within a page and across a page boundary.
6. **Motion is sublinear and lazy.** `setNodeBounds` is a ~16 ns queue
   push, callable any number of times per node per frame; the queue is
   applied only when the tree actually has to be current — inside the next
   `selectCut` (or an explicit `flushBounds()`). Each refit walks up only
   until an ancestor already contains the new box (grow-only; shrink is
   lazy) — that early-out is also what dedups movers sharing ancestors and
   repeated moves of one node (experiment K). Instance motion refits TLAS
   lanes and defers restructuring to an escape threshold; rebuilds
   triggered by motion use a Morton build ~5x cheaper than the quality
   build reserved for structural changes. **Spawning and removing are
   incremental too**: an insert descends to the leaf whose box grows least and
   either takes a free lane or splits that leaf, a removal invalidates a lane,
   and an edit budget (`tlasEditFraction`) bounds the accumulated quality loss.
   Nothing about a single instance costs work proportional to the world.
7. **GC is O(collected).** Pages touch an intrusive LRU once per frame;
   `collect` walks the cold tail only, respects a dwell (`minAge`), a
   budget of *streamed* (non-pinned) pages, and never collects pages with
   attached children — collapse is leaf-pages-first, and pinned pages don't
   even live in the list.

### Final numbers (this machine, single thread)

| Scenario | Baseline | Final | Notes |
|---|---|---|---|
| Deep tree 2.4M nodes, 260k-entry cut | 5.2 ms | 3.1 ms (cut-only) / 4.6 ms (all outputs) | ~12 ns per emitted entry |
| Deep tree fly-through (210k cut) | 4.4 ms | 3.6 ms | hysteresis + churn |
| GC stress (16 attach + 15 collect/frame) | 2.0 ms* | 1.4 ms* | *policy-corrected trajectory, see B2 |
| 50k shallow instances, no minPix | 9.6 ms | 3.2–4.1 ms | 21k-entry cut |
| 50k shallow instances, minPix | 0.68 ms | 0.41 ms | |
| 50k instances, 1000 teleports/frame | 15.1 ms | 6.2–7.5 ms | TLAS two-tier rebuild |
| 100k leaf refits/frame | 10.5 ms | 3.1 ms | ~32 M refits/s after experiments I + J + K |
| 1M move submissions/frame (100k nodes × 10) | — | 30 ms | ~30 ns per submission, lazy flush (K) |
| Kitchen sink (streaming + GC + 500 instance moves + 2000 leaf refits + 38k cut) | — | 2.9–3.6 ms | |
| Typical forest: 10k trees (80% shallow / 20% deep), 16k swaying leaves, running camera | 1.88 ms | 1.3–1.4 ms | 0.25 move + 0.78 refit + 0.34 cut |
| Typical forest at 50k trees, 80k movers | 28.8 ms | 16.5 ms | 1.3 move + 11.7 refit + 3.4 cut |
| Typical forest 10k + 5%/frame spawn-despawn churn | 21.6 ms | 4.0 ms | experiment L; ~1.3 µs per spawn/despawn |
| Builder: compose a page at runtime | — | 1.5 µs (5 nodes) / 0.55 ms (4681 nodes) | ~8 M nodes/s, experiment L |
| Output sensitivity (2.4M nodes, 60k → 260k cut) | — | 1.1 ms → 3.1 ms | ~11–19 ns/entry, flat; experiment M |
| Camera teleport into cold region (20k props + deep tree) | — | 1.05 ms steady / 1.36 ms spike | worst frame is 1.3× steady |
| Multi-view: main + 3 extra views | — | 1.45 ms + 0.84 ms/extra | extra view ≈ 0.6× main (hot shared data) |
| Teleport residual error, 2 frames after (budget 32/frame) | 97,000 px (discovery) | 800 px (predictive descent) | experiment N; both ~280 px by frame 4 |
| TLAS at 200k / 500k instances | — | 10 ms steady (34k cut), 121 / 416 ms level-load first cut | steady cost is output-bound, not N-bound |
| Adversarial: 10k fully stacked instances | — | 1.5 ms (40k cut) | TLAS gives zero separation; the floor |
| Adversarial: 511-child node / 26-page chain per 1-entry cut | — | 4.7 µs / 4.3 µs | ~170 ns per page crossing |

`BM_TypicalForest_Breakdown` reports per-phase counters (`move_us`,
`refit_us`, `cut_us`); movers are addressed by cached `NodeHandle`s, the
production pattern.

Cross-run absolute numbers on this box drift up to ±20% (see the honesty
note); per-experiment deltas below were validated with controlled A/B runs.

---

## 2. Experiment log

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
(rebuild at 25% escaped lanes) was already the best of the three.

Change: two build paths.

- **Structural rebuilds** (instance added/removed) keep the median-split
  build: they are rare, the result is long-lived, and build quality earns
  its keep — with a Morton-only build the contribution-culled forest bench
  (which leans on tight interior `maxErr`/bounds lanes) regressed 431 →
  902 µs, measured stable at CV 1%.
- **Motion rebuilds** (escape threshold) use a Morton build: one 63-bit-key
  sort, then contiguous groups of kWide per level, bottom-up. ~5x cheaper to
  build; slightly looser tree, but it only has to survive until the next
  motion rebuild anyway.

Also: the TLAS query stack became a reused member (it heap-allocated per
`selectCut` call).

Result: MovingInstances/50000/1000 9.2 → 6.2 ms (−33%, CV 2.7%), static
forest quality unchanged (ManyShallowTrees/50000/1 back at ~431 µs). Kept.

### G. Optional outputs — KEPT

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

### K. Lazy motion (flush at selectCut) — KEPT; explicit refit dedup — REJECTED, three ways

Two API asks landed together: a node can move many times per frame, so
refit-per-move is waste; and the tree only has to be correct at one moment
— `selectCut`. The first half is an API/semantics change and it stuck:

- `setNodeBounds` stays a ~16 ns flat queue push, but nothing is applied by
  `beginFrame` anymore. The pending queue is flushed inside `selectCut` (the
  one place that needs current bounds) or by an explicit `flushBounds()` for
  tools and tests. `beginFrame` is now purely the GC/LRU clock.
- Contract: after the flush, a node's own bbox equals exactly the *last*
  submitted box; ancestors are conservative (grow-only) over everything
  submitted. Multiple views flush once — the second `selectCut` finds an
  empty queue.

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
final flush is the experiment-I loop moved verbatim from `beginFrame` into
`flushBounds()`. Zero new state on pages, zero new state per node.

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
   place, in O(depth), and no longer mark it dirty at all. See HANDOFF.md
   11.1 for the measurement and `tests/test_tlas.cpp` for the invariants an
   in-place edit has to maintain.

| churn bench (5% out + 5% in, per frame) | before | after |
| --- | --- | --- |
| 10k trees (500+500 churn/frame) | 21.6 ms | 4.0 ms (0.8 churn + 0.25 move + 1.8 refit + 1.2 cut) |
| 50k trees (2500+2500 churn/frame) | not viable (seconds) | 31.5 ms (6.4 churn + 1.4 move + 16.9 refit + 6.8 cut) |

Non-churn scenarios are unchanged (the threshold keeps the same behavior
for assembly-time adds). Reading the after-numbers: a spawn or despawn
costs ~1.3 µs all-in (page copy, registration, residency, TLAS share);
refit and cut run a little hotter than the static forest because fresh
pages are cache-cold and the TLAS re-sorts every frame under churn. The
5%/frame rate is deliberately brutal — at a realistic 0.1–0.5%/frame the
churn phase is tens of microseconds.

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
byte-identical determinism across identically built worlds (hysteresis on);
multi-view scratch isolation (interleaving view B must not perturb view A);
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

---

## 3. Ideas considered and left for later

- **Radix sort for Morton rebuilds.** `std::sort` on 63-bit keys is the
  bulk of the motion rebuild; an LSD radix sort would roughly halve it.
  Not done: the two-tier policy already made rebuilds a minor term.
- **Parallelism.** The whole design is single-writer by construction, but
  `selectCut` is embarrassingly parallel across instances (forest case) and
  across independent subtrees (deep case): the DFS stack is already the
  natural work-stealing unit, and outputs could be emitted per-worker and
  concatenated. Left out to keep this version simple; nothing in the data
  layout blocks it.
- **Lazy shrink of grown bounds.** Grow-only refit means long-running
  teleport chaos degrades ancestor boxes toward the whole region (visible in
  `LeafMotion_TeleportWithCut`, where the cut stops shrinking). A trickle
  "re-tighten K nodes per frame" pass (bottom-up, budgeted) would restore
  tightness without a spike. The wide-lane mirror makes this a
  straightforward addition.
- **Ideal-cut-free streaming mode.** `NEEDS_EXPANSION` entries could be
  emitted into a dedicated small vector (they are rare) so streamers could
  skip the full ideal cut too. Cheap; do it when a real consumer appears.
