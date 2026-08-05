# HLodTree implementation architecture and performance journal

This document explains the current implementation and records the optimization
experiments that produced it. See [README.md](README.md) first for the overview,
minimal example, and headline scale; see [hlod_design.md](hlod_design.md) for
the current API contract and lifecycle rules.

## 1. Current architecture

The runtime has two spatial levels, analogous to a ray tracer's BLAS/TLAS
split:

- **Immutable page assets.** `HLodBuilder` emits a versioned blob containing
  preorder arrays and 8-lane child blocks. Expansion points connect separately
  streamed pages. Instances of one registered root asset share its page bytes,
  residency, and attachment graph. Deformed instances keep sharing those bytes
  and acquire bounds-only copy-on-write overlays.
- **Dynamic wide TLAS.** An 8-wide BVH over translated, uniformly scaled root
  instances handles coarse frustum/layer/contribution culling, motion, and
  incremental insert/remove. Initial and repair builds can use a quality
  splitter; frequent motion/edit rebuilds use a cheaper Morton hierarchy.

`selectCut` queries the TLAS, walks only surviving page regions, and can emit
the actual cut, ideal cut, and load requests in one pass. The latter two are
optional. `SelectionContext` adds per-view damping and conservative cut reuse;
stateless selection can instead fan out over visible instances through the
host's blocking `parallelFor`.

### Why it is fast

1. **Output-sensitive descent.** An explicit DFS stops at the ideal cut and
   carries error, undecided frustum planes, and actual-cut liveness on its
   stack. Nothing below the cut, outside the frustum, or behind unattached
   topology is visited, and no all-nodes per-frame clear exists.
2. **Eight-lane layout on every backend.** A parent tests eight child bounds and
   errors from one 256-byte `WideBlock`. AVX2 handles eight lanes directly;
   SSE2 and NEON use two four-lane halves; scalar builds retain the same blob
   layout. The TLAS uses the same logical width.
3. **Leaf fast path.** A side-array mask identifies plain leaves, which are
   emitted directly from the parent's wide test without a metadata read or DFS
   stack round trip. At fanout eight, most authored nodes are leaves.
4. **Shared immutable working sets.** Thousands of identical instances can walk
   one hot page. The cut-path `Instance` record is one cache line, while TLAS
   maintenance state occupies a separate cache line. The serial forest loop
   prefetches the next instance and root page.
5. **Handle-only mutation.** The world has no payload index or node hash table.
   Requests already carry generation-stamped handles; stale asynchronous
   completions fail one generation check and are ignored.
6. **O(1) readiness decisions.** Residency changes maintain immediate-child
   counters, so the all-or-nothing refine check is one comparison within and
   across pages.
7. **Lazy, bounded updates.** Bounds edits queue until a cut needs them and grow
   ancestors only until containment permits an early-out. Instance adds/removes
   edit the TLAS in O(depth); configured edit/escape/area budgets decide when a
   rebuild earns its cost.
8. **Cheap Morton repair builds.** 63-bit keys use a stable LSD radix sort for
   populations of at least 1,024, with retained scratch and only the 11-bit
   passes required by key variation. A dense live-id list avoids scanning dead
   historical slots.
9. **Frame-coherent reuse.** A 48-byte `SelectionContext` record proves when a
   fully-inside, converged instance's cut cannot change under camera-envelope or
   projection-scale travel. Reused entries are copied without rewalking pages.
10. **Cold-tail GC.** Non-pinned mounts touch an intrusive LRU at most once per
    frame. Collection examines the cold tail and detaches only aged leaf mounts;
    pinned roots are outside the budget and list.

### Current measurements

Point estimates below were rerun on 2026-08-05 with MSVC 19.51, Release
`/O2 /arch:AVX2`, one benchmark thread, on a 64-hardware-thread 2.4 GHz EPYC.
Google Benchmark used a 0.2 s minimum for the ordinary benchmarks; fixed-trajectory
`SelectionContext` cases use 600 iterations.

| Scenario | Current time | Relevant output/state |
|---|---:|---|
| `DeepTree_CutOnly/6` | 1.72 ms | 2.4M nodes authored; 259,933 cut entries |
| `SelectionContext_FlyThrough/20000`, stateless | 0.538 ms selection | 8,587 average cut entries |
| Same 20k case, contextual | 0.152 ms selection | 94.8% instances reused; 2.91 MiB context |
| 80k instances, 5% moving, stateless / contextual | 2.62 / 2.03 ms selection | 92.4% reused in contextual arm |
| `TlasScale`, 200k / 500k steady | 5.48 / 6.14 ms | 34,573 / 30,975 cut entries |
| `TlasScale`, first quality build + cut | 134 / 417 ms | level-load cost, not steady frame cost |
| Forced Morton rebuild + cut, 100k / 500k | 7.88 / 44.5 ms | random centroids |
| Sparse rebuild after 100k peak, 10k live | 0.741 ms rebuild | steady follow-up selection 0.254 ms |
| 4k cloned / shared 51 KiB assets | 35.4 / 12.9 ms | identical 2.048M-entry cut |
| Immutable page bytes in that cloned / shared case | 199 MiB / 51 KiB | 4,000 mounts / 1 mount |

These are scale indicators, not guarantees. Output size and cache locality can
dominate population size, and absolute results on this host have moved by up to
20% between runs. Optimization claims in the journal use interleaved baselines,
medians, win counts, and controls rather than unrelated absolute runs. The
final radix A/B measured 36.2% lower end-to-end rebuild cost at 100k random
instances and 42.0% at 500k.

---

## 2. Historical experiment log

The entries below are chronological evidence. Names such as `UserId`,
`ViewScratch`, or an older timing describe the revision under test, not the
current API. Later entries explicitly supersede earlier mechanisms. Sections 1
and 3, the README, and the design document are the current sources of truth.

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
   place, in O(depth), and no longer mark it dirty immediately. An edit budget
   still requests a repair rebuild after enough accumulated changes. See the
   archived [handoff, section 11.1](docs/archive/HANDOFF-2026-08-05.md)
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
by independent dampers and `SelectionContext` objects);
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

### O. Follow-up hot-path audit -- KEPT

The 2026-08-05 audit tightened the architecture without changing its external
model:

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
- A page already LRU-touched this frame is left linked in place, eliminating
  repeated unlink/relink work when many reused instances share an asset.
- Parallel workers still use private request epochs while walking, but the join
  performs a page-stamped deduplication. Request order and priority now match
  serial selection as well as the cut itself.
- `SelectionContext` budgets both camera-envelope travel and projection-scale
  travel. Each 48-byte record stores a conservative flip-point slope, avoiding
  the old all-cache invalidation throughout damped zoom-out. The affected 20k
  benchmark improved 38.9%. Reset retains warm storage, improving reset+cold
  selection 21.9%.

The archived [2026-08-05 handoff](docs/archive/HANDOFF-2026-08-05.md), section
14, preserves the interleaved A/B tables, rejected margin vectorization,
radix-sort tradeoffs, and full measurements. It is historical evidence, not API
guidance.

---

## 3. Current constraints and possible follow-up

- **Context reuse and parallel selection are separate paths.** Stateless
  selection already parallelizes across visible instances and preserves serial
  output/request order. `SelectionContext` remains serial because reuse records,
  the cut slab, and dependency validation are optimized as one compact stream.
  Combine them only if a production profile shows a net win after per-worker
  merge and record-write costs.
- **Internal overlay bounds do not shrink.** Grow-only refit means long-running
  large teleports can loosen page ancestors. This costs culling efficiency, not
  correctness; TLAS rebuilds retighten only the top level. A budgeted bottom-up
  overlay pass is the remaining mechanism if real content exhibits degradation.
- **Ideal-cut-free streaming mode.** `NeedsExpansion` entries could be emitted
  into a dedicated small sink so a streamer can request topology without a full
  ideal-cut vector. The current API emits them as tagged ideal entries; add a
  fourth mode only for a measured consumer need.
- **Transform scope.** Instances support translation and uniform positive scale.
  Rotation and non-uniform scale need authored baking or an integration-layer
  representation; adding them directly would change bound transforms and hot
  instance state.
- **Host policy remains host policy.** Page size, GC watermarks/dwell, attach
  budget, predictive lookahead, and whether to retain a `SelectionContext` are
  content- and IO-dependent decisions. The benchmark suite supplies the
  mechanisms' scale but cannot choose production values.
