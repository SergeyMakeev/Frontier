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
- The final result uses a longer paired baseline/current run from preserved
  executables under the same affinity and build settings.

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

## Experiment 4: retain an unchanged complete cached result

**Status: retained.**

### Theory

On a stable cached frame, the query already owns the exact three contiguous
output buckets from the previous call. The old hit path still probed every
32-byte root record and appended each root's usually two-entry run back into
those same buckets. This made a 100%-reused cut scale with root count and
performed thousands of tiny copies.

Retain the previous output when a conservative whole-result proof succeeds:

- the database generation and selection-policy epoch are unchanged;
- the packed visible-root/mask stream is byte-identical;
- every record was cacheable on the previous completed pass; and
- camera plus projection travel remains below the minimum residual validity
  margin, using the maximum recorded projection slope.

The internal view-returning API can then return its existing spans immediately.
Caller-provided sinks and owning-result copies still receive freshly written
output. A failed proof clears the internal buffers and takes the original
per-root exact path.

### Result

Pinned 0.30-second samples, nine repetitions:

| Workload | Initial baseline | Retained output | Speedup |
|---|---:|---:|---:|
| Forest 10k, stable cached | 71.4 us | 17.2 us | 4.15x |
| Camera stationary | 72.3 us | 17.0 us | 4.25x |
| Camera step 0.1 | 72.2 us | 17.6 us | 4.10x |
| Camera step 16 | 76.7 us | 32.5 us | 2.36x |
| Camera step 256 | 135 us | 98.5 us | 1.37x |

The 10%-moving fallback remained at 156 us, matching the 156-157 us initial
baseline, and mass motion remained far ahead of the original baseline at
925 us versus 1,167-1,197 us. Raw results: `experiment4-a.json` and
`experiment4-fallbacks.json`.

The Debug BVH4/BVH8 suite passes 182/182 tests after adding direct coverage
that the internal view keeps its storage address on a whole-result hit while a
caller-provided fixed sink is still populated on every call.

## Experiment 5: patch changed root runs in place

**Status: retained.**

### Theory

Object motion invalidates the changed roots' frontier versions, so the
whole-result fast path correctly declines the frame. The previous fallback
then cleared all output buckets and copied every unchanged root's cached run
back into them. In the 10%-moving workload, 9,000 valid roots were recopied to
reconstruct bytes the query already owned.

Store each record's three output offsets as cold allocation state. When the
visible root/mask stream and policy epoch are unchanged, leave hit runs in
place and overwrite only walked runs whose bucket counts are unchanged. If a
walk changes any bucket length, finish updating the per-root slab and perform
one exact full assembly pass from that slab. This preserves contiguous result
spans and exact cuts while making the common fixed-topology motion frame
proportional to the changed cohort.

### Result

Pinned 0.30-second samples, nine repetitions:

| Workload | Before in-place patching | Patched | Incremental gain | Initial-baseline speedup |
|---|---:|---:|---:|---:|
| Move/publish/select 10% of 10k | 156 us | 137 us | 1.14x | 1.15x |
| Camera step 16 | 32.5 us | 29.6 us | 1.10x | 2.59x |
| Camera step 256 | 98.5 us | 95.4 us | 1.03x | 1.42x |

Stationary and 0.1-unit camera cases retained their 4.1-4.3x speedups. The
100%-moving case has no hit runs to preserve and remained within 2% of its
pre-change measurement. `RecCold` grows from 4 to 16 bytes, adding 120 KiB at
10,000 record slots; these offsets are never fetched by the hit-validation
path. Raw result: `experiment5-a.json`.

## Experiment 6: charge object translation to cache validity margins

**Status: retained.**

### Theory

The cache invalidated a root's cut on every transform version change. For a
pure translation, that discards information the cache already computed: moving
an object by distance `d` changes its relative distance to the camera by at
most `d`, exactly the same Lipschitz bound used for camera travel. The exact
TLAS leaf is still updated and queried, so frustum visibility remains current.

Maintain a monotonic translation path length per instance and include it in
the record's consumed validity budget. Translation no longer changes the
frontier-content version; a root is re-walked only when its accumulated motion
actually reaches an LOD decision boundary. Scale, deformation, topology, and
readiness changes still invalidate by version. Odometer overflow also bumps
the version and restarts from zero.

### Result

Pinned 0.40-second samples, nine repetitions:

| Workload | Initial baseline | Motion-budget cache | Speedup |
|---|---:|---:|---:|
| Move/publish/select 10% of 10k | 157 us | 70.3 us | 2.23x |
| Move/publish/select 100% of 10k | 1,167 us | 230 us | 5.07x |

Both workloads retain essentially 100% of root cuts; the 10% case averaged
0.084 walked roots per call and the mass-motion median walked none. Replacing
Euclidean travel with its conservative L1 bound improved the initial version
of this experiment from 71.5 to 70.3 us and from 236 to 230 us, while avoiding
one square root per moved root.

The translation odometer costs one four-byte float per root. The standalone
400-root changed `MotionGroup` microbenchmark rises from the pre-campaign
2.13 us to 3.16 us because it measures the new bound accounting but no
selection benefit; unchanged submissions remain 1.28 us versus 2.17 us
initially. In the complete dynamic frame, the avoided BLAS/cache rebuild work
dwarfs that accounting cost. Raw results: `experiment6-a.json` and
`experiment6-l1.json`.

## Experiment 7: validate a cached motion cohort once

**Status: retained.**

### Theory

A persistent `MotionGroup` already resolves, filters, deduplicates, and sorts
its public handles when its physical-order cache is built. Nevertheless, every
later frame revalidated every cached dense id with a bounds check, instance
generation comparison, and reverse-map lookup. Those checks can only change
when an instance is added, removed, reuses a slot, or dense storage is
reordered.

Maintain one database mapping epoch across exactly those operations. A single
epoch comparison before the loop proves the whole cached cohort valid; on a
mismatch the existing refresh path resolves every public handle again. The hot
loop then contains only the dense instance update. A regression test primes a
group, removes one member, reuses its dense slot for a new instance, and proves
the stale group cannot move the replacement.

### Result

Pinned 0.40-second samples, nine repetitions:

| Workload | Per-root validation | Cohort epoch | Speedup |
|---|---:|---:|---:|
| `MotionGroup`, 400 changed roots | 3.16 us | 2.94 us | 1.08x |
| `MotionGroup`, 400 unchanged roots | 1.28 us | 0.894 us | 1.43x |
| Move/publish/select 10% of 10k | 70.3 us | 67.6 us | 1.04x |
| Move/publish/select 100% of 10k | 230 us | 228 us | 1.01x |

The full Debug BVH4/BVH8 matrix passes 184/184 tests. Raw result:
`experiment7-a.json`.

## Experiment 8: stamp a motion batch once

**Status: retained.**

### Theory

Pure translation preserves the per-instance frontier-content version, but it
must still advance the database generation so the whole-result shortcut knows
to examine exact TLAS visibility and per-root motion margins. The first version
of the translation cache advanced that shared counter for every moved root.
Besides doing unnecessary work, 400 dependent read-modify-write operations on
one address serialize the otherwise streaming motion loop.

Advance the generation lazily on the first effective move in a `MotionGroup`
and reuse that stamp for any scale-change invalidations in the same batch.
Per-instance cache correctness only requires the new value to differ from that
instance's recorded value; versions do not need to be unique within a writer
batch. An entirely unchanged batch still leaves the generation untouched.

### Result

Pinned 0.40-second samples, nine repetitions:

| Workload | Per-root stamps | One batch stamp | Speedup |
|---|---:|---:|---:|
| `MotionGroup`, 400 changed roots | 2.94 us | 2.83 us | 1.04x |
| Move/publish/select 100% of 10k | 228 us | 224 us | 1.02x |

The unchanged motion path does not reach the new stamping branch and matched
the prior long audit at 0.958 versus 0.957 us. The 10%-moving frame remained
inside its observed 64-69 us run-to-run band. Debug BVH4/BVH8 passes 184/184
tests. Raw result: `experiment8-a.json`.

## Final paired audit

The preserved `15d8e65` executable and final `484ef41` executable were each
measured with 0.30-second minimum samples and 11 repetitions, pinned to the
same logical processor. Values below are real-time medians from
`final-baseline.json` and `final-current.json`.

| Workload | Baseline | Final | Speedup |
|---|---:|---:|---:|
| `MotionGroup`, 400 changed roots | 2.20 us | 3.00 us | 0.73x |
| `MotionGroup`, 400 unchanged roots | 2.19 us | 0.968 us | 2.27x |
| Move/publish/select 10% of 10k | 159 us | 69.6 us | **2.28x** |
| Move/publish/select 100% of 10k | 1,106 us | 225 us | **4.92x** |
| Forest 10k, uncached | 499 us | 468 us | 1.07x |
| Forest 10k, stable cached | 74.1 us | 17.0 us | **4.37x** |
| Forest 10k, forced cache miss | 750 us | 792 us | 0.95x |
| Camera stationary | 72.6 us | 17.0 us | **4.28x** |
| Camera step 0.1 | 73.9 us | 18.0 us | **4.11x** |
| Camera step 16 | 78.0 us | 29.7 us | **2.62x** |
| Camera step 256 | 135 us | 98.2 us | 1.38x |

The geometric mean across the seven real dynamic/cache-hit workloads (the two
complete object-motion frames, stable cached forest, and four camera steps) is
**3.15x**. The geometric mean across all eleven audit rows, including raw and
deliberately adversarial paths, is **2.18x**.

### Tradeoffs

- The standalone changed-transform loop is 36% slower because it now computes
  and stores the exact conservative translation budget that enables root-cut
  reuse. Once publication and selection are included, that cost buys 2.28x to
  4.92x faster complete moving-object frames. Unchanged batches are 2.27x
  faster.
- The forced-miss cached path is 5.6% slower because in-place patching stores
  output-run offsets that hits use to avoid reconstructing 20,000 entries.
  Callers that know every root must miss should disable reuse; the uncached
  path is 6.7% faster than baseline.
- A 256-unit camera jump re-walks about 928 of 10,000 roots per call and is
  therefore traversal-dominated. Coherent stationary through 16-unit motion,
  which reuses at least 99.4% of roots, improves by 2.62x to 4.28x.

### Retained commits

1. `c4a47e5` - avoid TLAS rebuild thrashing under coherent motion.
2. `24a2684` - skip redundant instance transform updates.
3. `af1eef7` - skip TLAS descent for fully visible scenes.
4. `a0281d5` - retain unchanged cached frontier output.
5. `c55d199` - patch changed cached frontier runs in place.
6. `558bf84` - reuse cached cuts across bounded object motion.
7. `1edd571` - validate cached motion groups once per batch.
8. `484ef41` - stamp batched instance motion once.

### Final validation

- Debug payload64 BVH4/BVH8: 184/184 tests passed.
- Debug payload32/payload64 BVH4/BVH8 matrix: 368/368 tests passed.
- Release benchmark build completed with MSVC AVX2/BVH8 and IPO.
- `git diff --check` passed before every retained commit.
