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
