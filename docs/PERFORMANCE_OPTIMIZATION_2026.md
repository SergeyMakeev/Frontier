# Performance optimization campaign (2026-08)

This report records the autonomous optimization campaign whose target is at
least a 20% speed increase on representative Frontier workloads. It is a
working engineering log, not a list of wins: rejected experiments and invalid
measurements are kept so that the same ideas are not repeated without new
evidence.

## Scope and success criterion

The primary score covers runtime work that a production integration performs
per frame:

- uncached and reused hierarchy selection;
- flat TLAS selection;
- mixed flat/hierarchical forests;
- readiness changes shared by many placements;
- instance motion and node-bound overrides.

Construction and subtree registration are measured separately because they
are edit/streaming operations and must not hide a runtime regression. The
release candidate must preserve correctness for BVH4 and BVH8 and for 32-bit
and 64-bit payload configurations.

The campaign target is a 20% or greater reduction in a geometric-mean runtime
score over the representative workload set. Individual workloads and update
costs are also reported so that a large win cannot conceal an important
regression.

## Measurement protocol

Baseline source: commit `0113052` (`Harden pre-release validation matrix`).
The immutable baseline executable is in `build-perf-goal-baseline`; experiments
are built in `build-perf-goal-exp`.

Performance configuration:

- MSVC Release with IPO enabled;
- AVX2 and BVH8 on the test machine;
- 64-bit payload unless a payload-width experiment says otherwise;
- contract checks, serialized-subtree validation, and statistics disabled;
- Google Benchmark real time, not reported CPU time;
- retained-capacity warm state;
- repeated medians, with important decisions measured A/B/A;
- benchmark process pinned to logical processor 0.

The machine is a 24-logical-processor hybrid-core Intel system with 48 KiB L1D
per physical core, 1.25 MiB L2 per physical core, and 30 MiB shared L3. Windows
CPU-time samples are quantized/noisy on this machine, while pinned real-time
samples are repeatable.

Raw Google Benchmark JSON is stored under
`perf_reports/optimization-goal/` (an ignored results directory). A pinned run
is launched by starting the benchmark process, assigning
`ProcessorAffinity = 1`, then waiting for completion. The core comparison uses
`--benchmark_min_time=0.10s`, nine repetitions, and aggregate medians; focused
experiments use up to `0.20s` and fifteen repetitions. The final score uses
0.20-second minimum samples and fifteen repetitions.

### Invalid initial measurement

The first unpinned baseline/experiment comparison is retained as
`baseline-core-payload64.json` and
`exp1-exact-tlas-leaves-payload64.json`, but is invalid for decisions. Windows
moved the single benchmark thread between different core classes: unrelated
tests changed by 2-3x and the same 400-house test moved from about 11 to 22
microseconds. Pinning produced approximately 1.2% real-time CV in the first
probe and reduced the flat-10k baseline from a scheduler-dependent 75
microseconds to a repeatable 43-45 microseconds.

## Baseline

Pinned A/B/A measurements established these representative baseline medians.
The reference for an experiment is the geometric mean of its bracketing
baseline medians.

| Workload | Baseline median |
|---|---:|
| Flat TLAS, 1k, uncached | 5.14 us |
| Flat TLAS, 10k, uncached | 45.09 us |
| Mixed forest, 10k, 50% hierarchical, uncached | 387.90 us |
| Hierarchical forest, 10k, uncached | 734.11 us |
| Mixed forest, 10k, stable cache hits | 71.61 us |
| Hierarchical forest, 10k, stable cache hits | 78.06 us |
| Hierarchical forest, 10k, forced cache misses | 966.35 us |
| MotionGroup, 400 changed transforms | 2.05 us |
| Assembled city, 400 houses, uncached | 10.60 us |

Full pinned files for the first diagnostic set are
`exp1-baseline-pinned-a.json` and `exp1-baseline-pinned-b.json`.

## Experiment 1: exact TLAS leaf lanes and definitive leaf culling

**Status: retained.**

### Theory

TLAS traversal tested every leaf lane against the frustum with SIMD, then the
instance walker repeated the test scalarly against `Instance::worldBox`.
The second test was required because movement kept grow-only TLAS leaf boxes;
the leaf lane was only a conservative superset of the current instance.

Keep leaf lanes exact on every move, while allowing inner lanes to retain the
existing conservative grow-only refit. A visible leaf lane is then definitive,
so the scalar per-instance cull can be removed from root, flat, and zero-error
flat dispatch. The update path writes six exact leaf-bound floats on movement;
ancestors are still touched only when the instance escapes their existing
bound.

### Correctness

The full Debug matrix passed before performance evaluation: 360/360 tests at
that point in the campaign (BVH4/BVH8 times payload32/payload64), including
randomized TLAS churn, motion, all quality tiers, and serial/parallel identity.
The current consolidated Debug matrix also passes 180/180 tests for BVH8 and
its alternate BVH4 build.

### Result

Pinned A/B/A, 0.10-second minimum samples, nine repetitions:

| Workload | Baseline | Experiment | Speedup |
|---|---:|---:|---:|
| Flat TLAS, 1k, uncached | 5.141 us | 4.891 us | 5.1% |
| Flat TLAS, 10k, uncached | 45.088 us | 41.489 us | 8.7% |
| Flat TLAS, 10k, direct/reuse-enabled | 45.492 us | 41.123 us | 10.6% |
| Hierarchical forest, 10k, uncached | 734.111 us | 703.049 us | 4.4% |
| Hierarchical forest, 10k, forced misses | 966.351 us | 928.663 us | 4.1% |
| MotionGroup, 400 changed transforms | 2.052 us | 1.997 us | 2.8% |
| Assembled city, 400 houses, uncached | 10.595 us | 10.316 us | 2.7% |

The geometric-mean improvement across all 22 diagnostic cases was 2.3%.
There was no measured movement penalty, so this is a useful foundation but not
the campaign's 20% result. Raw result:
`exp1-experiment-pinned.json`.

## Experiment 2: fuse zero-error flat TLAS traversal and output

**Status: rejected and reverted.**

### Theory

An all-flat, zero-error query writes a compact `VisibleItem` array during TLAS
traversal, then immediately walks that array to build one `FrontierEntry` per
instance. Emitting entries directly from TLAS leaves would eliminate the
intermediate buffer and second loop.

### Result

Pinned, 0.20-second minimum samples, eleven repetitions:

| Workload | Baseline | Fused | Change |
|---|---:|---:|---:|
| Flat TLAS, 1k, uncached | 5.115 us | 5.380 us | -4.9% |
| Flat TLAS, 1k, direct/reuse-enabled | 5.128 us | 5.356 us | -4.3% |
| Flat TLAS, 10k, uncached | 45.138 us | 51.292 us | -12.0% |
| Flat TLAS, 10k, direct/reuse-enabled | 45.311 us | 51.673 us | -12.3% |

The compact two-phase path has better locality. Keeping TLAS traversal focused
on TLAS nodes and four-byte visible records is faster than interleaving random
instance-id/flat-slot loads and twelve-byte frontier writes with the traversal.
Raw results: `exp2-fused-flat-pinned.json` and
`exp2-baseline-pinned-b.json`.

## Experiment 3: split hot TLAS traversal data from cold metadata

**Status: retained.**

### Theory

Every TLAS query needs child bounds, child indices, the valid-lane mask, and
the parent index. Most queries do not need the lane contribution masks or
maximum-error vectors. Keeping both sets in one `TlasNode` made the default
walk pull cold data into the cache.

The node was split into a 256-byte BVH8 hot record (`WideBounds`, children,
valid mask, parent) and a 64-byte cold `TlasMeta` record (maximum errors and
lane masks). BVH8 total storage is unchanged at 320 bytes per node, but the
query working set is 20% smaller. BVH4 total storage falls from 192 to 160
bytes per node.

### Result

The split added about 2% to the already-retained flat-selection gain and
improved large hierarchical cases by 1-3%, without a repeatable update
regression. Raw results: `exp3-split-tlas-meta-pinned.json` and
`exp3-baseline-pinned-b.json`.

## Experiment 4: transform only active frustum planes

**Status: retained.**

### Theory

TLAS traversal propagates a six-bit active-plane mask. A placement that is
already fully inside some world-space planes cannot become outside those
planes after an exact coordinate transform, yet `toLocal` transformed all six
planes for every visible instance.

An active-mask overload now transforms only planes still required by the
query. The original overload remains and delegates with all planes active.

### Result

The largest mixed-forest case improved 5.2%; other large hierarchy cases
improved 1.2-2.5%. Raw results:
`exp4-active-plane-local-view-pinned.json` and
`exp4-baseline-pinned-b.json`.

## Experiment 5: alias the root camera for identity mounts

**Status: retained.**

### Theory

The overwhelmingly common root placement has identity translation and scale.
Traversal nevertheless copied a 160-byte `Camera` into a local temporary for
every placement. Use a reference to the instance-local root camera for
identity mounts and materialize transformed storage only for non-identity
mounts.

### Result

The 10k all-hierarchical uncached case improved 2.0%; other cases were neutral
to slightly positive. Raw result: `exp5-alias-identity-camera-pinned.json`.

## Experiment 6: carry TLAS root errors as a SIMD stream

**Status: rejected and reverted.**

### Theory

The TLAS already evaluates wide bounds. Computing all root screen errors there
could remove scalar distance/square-root/divide work from instance dispatch.

### Result

The extra error stream increased cold data traffic and made large uncached
forests 2.2-2.7% slower. Forced-miss cases were effectively unchanged. The
scalar arithmetic is cheaper than another stream of memory traffic, so the
experiment was fully reverted. Raw results:
`exp6-simd-tlas-root-errors-pinned.json` and
`exp6-baseline-pinned-b.json`.

## Experiment 7: adaptive bottom-level TLAS leaf packing

**Status: retained.**

### Theory

The old recursive builder always created `kWide` child ranges. At the bottom
of a BVH8 this frequently produced eight sparse leaves instead of the minimum
number of full leaves. Those extra nodes cost memory, traversal tests, and
refit work.

For ranges no larger than `kWide * kWide`, the builder now spatially sorts the
range along its longest centroid axis and creates exactly
`ceil(count / kWide)` full leaf children. Larger recursive levels retain the
selected median or binned-SAH strategy; Morton uses its separate radix/group
builder.

### Result

For 10k instances, BVH8 median node count fell from 4,681 to 2,121 and binned
SAH reached 2,185 nodes. Default binned-SAH TLAS storage fell from about 1,463
KiB to 683 KiB, a 53% reduction.

| Query | Median | Binned SAH |
|---|---:|---:|
| All 10k instances visible | 21.4% | 19.7% |
| Close camera, 627 outputs | 20.0% | 14.3% |

The new `BM_TlasQualitySelection` benchmark records selection time, entry
count, node count, and TLAS bytes for each quality tier. Raw results:
`exp7-quality-baseline-pinned.json` and
`exp7-adaptive-leaf-packing-pinned.json`.

## Experiment 8: zero-error terminal-leaf fast path

**Status: retained, with one rejected extension.**

### Theory

Terminal render leaves with zero geometric error need frustum culling and
output, but not distance-to-box, reciprocal-square-root, threshold, or error
encoding work. This is common in the finest level of authored LOD trees.

`wideVisit` now emits a surviving all-terminal, all-zero-error lane set
directly. The serialized block mask was extended with a per-lane zero-error
mask, so the proof costs one already-hot word and bit operations rather than a
runtime scan of `WideBlock::error`. The mask consumes previously unused bits,
adds no bytes, is structurally validated, and advances the native subtree
format to version 7.

### Result

The initial exact fast path improved 10k forest traversal by 16.8-19.6% and
forced-miss cases by 12.7-15.2%. Precomputing the lane invariant added another
1.5-3.2% on the large hierarchy cases.

Applying similar branches inside the specialized mounted-leaf batch was
rejected: exact repeated measurement showed assembled 400-house traversal
regressing from 10.33 to 16.73 microseconds. Removing only those branches
restored 10.54 microseconds. The batch's existing straight-line SIMD path has
better code layout even when some arithmetic appears redundant.

Raw results: `exp8-zero-error-leaf-fast-path-pinned.json`,
`exp8c-mask-widevisit-only-forest-pinned.json`,
`exp8b-assembly-exact-pinned.json`, and
`exp8c-mask-widevisit-only-assembly-pinned.json`.

## Experiment 9: scalar single-inner-lane traversal

**Status: rejected and reverted.**

### Theory

An implicit subtree root often occupies only lane zero. A scalar bounds/error
path could avoid evaluating an underfilled BVH8 block.

### Result

Three of four large forest cases improved 1.0-2.4%, but the all-hierarchical
uncached case regressed 0.8%. The small, inconsistent gain did not justify
duplicating backend-specific reciprocal-square-root behavior and traversal
bookkeeping. The experiment was fully reverted. Raw result:
`exp9-scalar-single-inner-lane-pinned.json`.

## Experiment 10: enter the known root placement directly

**Status: retained.**

### Theory

Instance dispatch already has the first mounted placement in registers, but
the old implementation appended a 16-byte `WorkItem` to a heap-backed vector,
entered a loop, immediately loaded and removed the same item, and only then
called subtree traversal. The stack is necessary for mounted descendants, not
for the known root.

All three paths (fully ready, descendant fallback, and ancestor fallback) now
call the first subtree directly. The existing work stack is used only for
mounted descendants discovered during that traversal. This removes one
vector mutation and one dispatch loop round trip per refined instance and
also gives the optimizer a tighter root call path.

### Result

Relative to the state immediately before this change:

| 10k forest | Speedup |
|---|---:|
| 50% hierarchical, uncached | 11.5% |
| 50% hierarchical, forced miss | 5.3% |
| 100% hierarchical, uncached | 20.4% |
| 100% hierarchical, forced miss | 13.7% |

The non-ready current-cut paths also improved 1.8-4.2% in the mixed-readiness
benchmark. Repeated pinned measurements reproduced the result. Raw results:
`exp10-direct-root-dispatch-pinned.json` and
`exp10b-direct-root-all-policies-pinned.json`.

## Experiment 11: honor the configured Morton quality tier

**Status: retained correctness/performance fix.**

### Finding

The final benchmark audit exposed an old configuration bug: an initial or
promoted build with `TlasQuality::Morton` took the generic "quality" branch,
where only binned SAH is distinct and every other value falls back to median.
The public API therefore did not provide the documented cheap Morton initial
build.

### Change and result

Promoted builds now take the radix-sorted Morton path when Morton is the
configured quality tier, and record that build as the population baseline.
The existing visible-set test now also proves that Morton allocates fewer
nodes than median.

| 10k quality tier | All-visible time | Close-camera time | Nodes | TLAS KiB |
|---|---:|---:|---:|---:|
| Morton | 35.04 us | 24.78 us | 1,431 | 447.2 |
| Median | 35.71 us | 3.10 us | 2,121 | 662.8 |
| Binned SAH | 36.12 us | 3.35 us | 2,185 | 682.8 |

This confirms the documented tradeoff: Morton is smallest and best when most
objects are visible, but its loose hierarchy is much worse for selective
culling. The default remains binned SAH. Raw result:
`final-tlas-quality-pinned.json`.

## Final result

**The target was reached: 22.7% geometric-mean speedup across the complete
22-workload runtime core.** The score includes flat and hierarchical
selection, stable cache hits, deterministic cache misses, assembled and
flattened subtree traversal, and both changed and unchanged motion groups.

The immutable pre-change reference was measured throughout the campaign (most
recent early file: `exp6-baseline-pinned-b.json`). Final current files and the
post-current immutable-baseline rerun are:

- `final-current-core-a-pinned.json`;
- `final-current-core-b-pinned.json`;
- `final-baseline-core-a-pinned.json`;
- `final-baseline-core-b-pinned.json`.

The `a` and `b` suffixes split the core workload groups; they are not temporal
run labels.

The second immutable-baseline pass was within about 0-2% of the earlier
baseline on the important selection cases, so the result is not explained by
core-class migration or long-term machine drift.

| Representative workload | Baseline | Final | Speedup |
|---|---:|---:|---:|
| Flat TLAS, 1k, uncached | 5.08 us | 3.58 us | 41.6% |
| Flat TLAS, 10k, uncached | 44.59 us | 37.22 us | 19.8% |
| Mixed forest, 1k, uncached | 38.49 us | 25.75 us | 49.5% |
| Hierarchical forest, 1k, uncached | 71.78 us | 45.30 us | 58.5% |
| Mixed forest, 10k, uncached | 394.63 us | 250.93 us | 57.3% |
| Hierarchical forest, 10k, uncached | 719.26 us | 468.02 us | 53.7% |
| Mixed forest, 10k, stable cache hits | 71.39 us | 61.09 us | 16.8% |
| Hierarchical forest, 10k, stable cache hits | 76.99 us | 67.27 us | 14.5% |
| Mixed forest, 10k, forced misses | 603.38 us | 461.11 us | 30.9% |
| Hierarchical forest, 10k, forced misses | 949.03 us | 685.24 us | 38.5% |

The four motion cases varied from 1.0% to 4.8% slower in the final bracketing
comparison (roughly 10-50 ns absolute). Exact TLAS leaf maintenance trades
those extra writes for the much larger query savings; earlier brackets moved
the same cases in both directions. No construction or streaming regression
was hidden in the runtime score: builder, zero-copy registration, readiness
fanout, mount/unmount, and bounds-override medians all remained within 3.4% of
baseline. Raw peripheral results use the `final-*-construction-pinned.json`
and `final-*-updates-pinned.json` files.

### Cross-platform release snapshot

The optimized `d3f12a0` library implementation was subsequently collected on
an Apple M2 Max, an RK3399, an Intel i9-12900K, and an AMD EPYC 9654. The
latest format-v3 bundles report commit `35e7b3f`; the intervening changes add
documentation, benchmark coverage, the comprehensive collector, and an
ignore-list entry rather than changing the library. All four reports completed
and passed the 360-test payload32/payload64 by BVH4/BVH8 Debug matrix. Their
matched assembly workload found a 55-81% construction-latency reduction and a
63-64% retained-memory reduction for a shared 400-house scene. A cached
assembled cut differed from the flattened cut by at most 0.275 us; uncached
assembly ranged from 39% faster to 43% slower as the mount-indirection versus
working-set tradeoff changed by target.

The comprehensive suite also measured a fully hierarchical 10,000-root scene
that emits 20,000 entries. Stable cached selection takes 108-524 us across the
four targets; a 16-unit camera step retains about 99.4% reuse and takes
112-575 us. Moving 10% of roots, publishing, and selecting takes 171-1,084 us,
while moving every root takes 913-9,243 us. These absolute ranges are
cross-platform current-state context, not part of the pinned optimization
score.

Those bundles contain only the final implementation and ran without affinity,
so they are a current-state portability snapshot, not an independent
recalculation of the pinned 22.7% before/after score above. Complete per-machine
selection, moving-camera, moving-object, assembly, payload-width, kernel, and
lifecycle tables plus measurement caveats are in
[PERFORMANCE_RESULTS_2026_08_15.md](PERFORMANCE_RESULTS_2026_08_15.md).

## Final validation

- Debug BVH8/BVH4: 180/180 tests passed.
- Debug payload64/payload32 times BVH8/BVH4: 360/360 tests passed.
- Forced-scalar + statistics BVH4/BVH8: 180/180 tests passed.
- Serialized structural validation disabled, BVH4/BVH8: 180/180 tests passed.
- Randomized TLAS churn, randomized readiness/complete-cover validation,
  parallel/serial bit identity, and concurrent query torture tests passed in
  every matrix above.
- MSVC Release built the normal library and both 32-bit and 64-bit payload
  benchmark libraries with IPO, checks, validation, statistics disabled, and
  AVX2/BVH8 enabled.
- The serialized zero-error lane mask is asserted directly by the builder
  test and checked by full structural registration validation.
