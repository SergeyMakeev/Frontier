# Dynamic-scene performance campaign (round 2)

## Goal

Improve moving-camera and moving-object frame performance by at least 2x
relative to repository commit `15d8e65`. The primary workloads are the
10,000-root `BM_MovingCameraSelectionScale`,
`BM_MovingObjectsSelectionScale`, `BM_InstanceForestSelectionScale`, and
`BM_MotionGroupSteady` cases with an eight-byte payload.

Every retained change must pass correctness tests and show a repeatable
same-machine improvement. Raw benchmark JSON is stored under the ignored
`perf_reports/optimization-round2/` directory. The pre-change executable is
preserved there as `frontier_bench_baseline.exe`.

## Measurement protocol

- Windows 11, MSVC Release, AVX2/BVH8, IPO enabled.
- Contract checks, statistics, and full serialized-subtree validation disabled.
- Process pinned to logical processor 0.
- Google Benchmark real time, seven repetitions, 0.20-second minimum samples,
  aggregate median.
- The final result will use longer interleaved baseline/current runs.

## Baseline

Baseline file: `baseline-a.json`.

| Workload | Median time |
|---|---:|
| MotionGroup, 400 changed roots | 2.18 us |
| MotionGroup, 400 unchanged roots | 2.22 us |
| Move/publish/select 10% of 10k | 157 us |
| Move/publish/select 100% of 10k | 1,167 us |
| Forest 10k, uncached | 503 us |
| Forest 10k, stable cached | 71.4 us |
| Forest 10k, forced cache miss | 736 us |
| Camera stationary | 72.3 us |
| Camera step 0.1 | 72.2 us |
| Camera step 16 | 76.7 us |
| Camera step 256 | 135 us |

## Experiment 1: remove exact-leaf escape rebuild thrashing

**Status: retained.**

### Theory

TLAS leaf lanes are exact after every transform update so their SIMD cull is
definitive. The old rebuild policy also counted a leaf as "escaped" whenever
its new exact bound was not contained by its previous exact bound. Therefore
every nonzero translation counted as an escape. Moving more than 25% of roots
marked the TLAS dirty, and `applyUpdates()` rebuilt all roots. With 100% of the
population alternating between two positions, every frame rebuilt the TLAS.

Ancestor lanes already use grow-only refit and accumulate the surface area
actually added by motion. That is the correct degradation metric: once an
ancestor contains both positions, further coherent motion adds no area and
does not reduce query quality. Remove the redundant escape counter and rebuild
only when accumulated ancestor-area growth crosses `tlasAreaDrift`.

### Result

Focused pinned A/B measurement used 0.50-second samples and nine repetitions:

| Workload | Baseline | Experiment | Speedup |
|---|---:|---:|---:|
| Move/publish/select 10% of 10k | 156 us | 156 us | neutral |
| Move/publish/select 100% of 10k | 1,197 us | 899 us | 1.33x |

The broader seven-repetition run measured the mass-motion case at 1,167 us
versus 850 us (1.37x) and showed no material change in camera selection,
stable cache hits, raw traversal, or `MotionGroup`. Debug BVH4 and BVH8 both
passed all 180 tests. Raw results: `experiment1-a.json`,
`experiment1-baseline-b.json`, and `experiment1-b.json`.

## Experiment 2: reject unchanged transforms before derived-state work

**Status: retained.**

### Theory

`MotionGroup` is deliberately a persistent caller cohort, so real animation
systems can submit transforms for objects that are paused or unchanged. The
old path translated the world bound, rewrote the instance, bumped its frontier
version, rewrote its exact TLAS leaf, and checked ancestor bounds even when the
new transform was bit-for-bit identical to the stored transform. An exact
position/scale comparison before all derived-state work makes unchanged
submissions true no-ops and, crucially, keeps their cached cuts valid.

### Result

Pinned 0.40-second samples, nine repetitions:

| `MotionGroup` workload | Before | After | Speedup |
|---|---:|---:|---:|
| 128 unchanged roots | 0.718 us | 0.387 us | 1.86x |
| 400 unchanged roots | 2.17 us | 1.18 us | 1.84x |
| 400 changed roots | 2.13 us | 2.28 us | 7.0% slower |

The exact equality gate adds four predictable comparisons to genuinely changed
roots, costing about 0.10 us per 400 submissions, but saves 1.04 us when the
cohort is unchanged and prevents needless cache invalidation. Complete
10%- and 100%-moving frame medians remained within the existing run-to-run
noise (155 us and 869 us). Raw results: `experiment2-baseline.json` and
`experiment2-a.json`.

## Experiment 3: prove full visibility at the TLAS root

**Status: retained.**

### Theory

The dynamic-scene benchmarks view the complete root population. The old TLAS
walk nevertheless descended every internal node and emitted roots one leaf at
a time. If every valid lane of the TLAS root is fully inside all frustum
planes, convexity proves every descendant instance is visible with a zero
active-plane mask. With default all-layer selection and no minimum-pixel cull,
the query can emit the dense live-instance stream directly after one wide root
test. Selective views retain the existing hierarchical walk.

### Result

Pinned 0.25-second samples, seven repetitions, compared with the initial
baseline run:

| Workload | Baseline | Experiment | Speedup |
|---|---:|---:|---:|
| Forest 10k, stable cached | 71.4 us | 64.8 us | 1.10x |
| Camera stationary | 72.3 us | 67.2 us | 1.08x |
| Camera step 0.1 | 72.2 us | 66.4 us | 1.09x |
| Camera step 16 | 76.7 us | 70.6 us | 1.09x |
| Camera step 256 | 135 us | 127 us | 1.06x |
| Move/publish/select 10% of 10k | 156 us | 147 us | 1.06x |

The all-visible shortcut also reduced the flat 10k case to 36.5 us and was
neutral within noise for traversal-dominated raw/forced-miss cases. Raw
result: `experiment3-a.json`.
