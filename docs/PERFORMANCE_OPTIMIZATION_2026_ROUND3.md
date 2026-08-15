# Radical dynamic-scene performance campaign (round 3)

## Goal and baseline

Achieve a 5-10x end-to-end speedup relative to the already optimized current
repository state, commit `a552e47`. This is deliberately not a comparison with
the older pre-round-2 build.

The exact baseline executable is preserved under the ignored benchmark-artifact
directory as `perf_reports/optimization-round3/frontier_bench_baseline_a552e47.exe`
(SHA-256 `C650960C72BB180217C9623BDCFD98EFCD80E7D9F96A2FD36FE6E38B11ADA2A0`).
The measurement protocol remains MSVC Release, AVX2/BVH8, IPO, statistics and
serialized validation disabled, Google Benchmark real time, process pinned to
logical processor 0.

Long paired medians recorded at `a552e47`:

| Workload | Current baseline | 5x target | 10x target |
|---|---:|---:|---:|
| `MotionGroup`, 400 changed | 3.00 us | 0.600 us | 0.300 us |
| `MotionGroup`, 400 unchanged | 0.968 us | 0.194 us | 0.097 us |
| Move/publish/select 10% of 10k | 69.6 us | 13.9 us | 6.96 us |
| Move/publish/select 100% of 10k | 225 us | 45.0 us | 22.5 us |
| Stable cached forest, 10k | 17.0 us | 3.40 us | 1.70 us |
| Camera stationary, 10k | 17.0 us | 3.40 us | 1.70 us |
| Camera step 0.1 | 18.0 us | 3.60 us | 1.80 us |
| Camera step 16 | 29.7 us | 5.94 us | 2.97 us |
| Camera step 256 | 98.2 us | 19.6 us | 9.82 us |

## Experiment 1: retain an all-visible TLAS stream by root certificate

**Status: retained.**

### Theory

After round 2, a whole-cut hit does not rebuild or copy the 20,000-entry result,
but every call still materializes 10,000 `VisibleItem` values from
`liveInstances_` and compares all 40 KiB with the previous stream. In overview
views the TLAS root lanes are wholly inside the frustum. Those conservative
lanes contain the entire population, so retesting only the BVH-width root lanes
is a complete visibility proof even after arbitrary camera motion and exact
instance-leaf updates.

Remember that the retained stream represented the complete population and the
public-handle-to-dense mapping epoch under which it was built. If the current
root is still wholly inside, the view has no layer or contribution filter, and
the mapping epoch is unchanged, retain the stream without swapping, filling,
or comparing either vector. If any premise fails, use the existing exact TLAS
query and byte comparison.

### Result

Pinned 0.30-second samples, nine repetitions, compared with the preserved
`a552e47` executable:

| Workload | Current baseline | Root certificate | Speedup |
|---|---:|---:|---:|
| Stable cached forest, 10k | 16.9 us | 0.059 us | **286x** |
| Camera stationary, 10k | 16.6 us | 0.059 us | **282x** |
| Camera step 0.1 | 18.1 us | 2.79 us | **6.49x** |
| Camera step 16 | 27.7 us | 12.6 us | **2.20x** |
| Camera step 256 | 93.1 us | 78.0 us | 1.19x |
| Move/publish/select 10% of 10k | 67.4 us | 52.9 us | 1.27x |
| Move/publish/select 100% of 10k | 223 us | 205 us | 1.09x |

The stationary path is now constant-time with respect to instance and output
counts: it damps/validates the camera, tests eight conservative root lanes,
and returns the already-owned spans. Small camera motion also clears the 5x
campaign target immediately. Larger steps still pay for the roots whose LOD
margins expire; the next experiments target that residual work separately.

Debug payload64 BVH4/BVH8 passes 184/184 tests and the full
payload32/payload64 BVH4/BVH8 matrix passes 368/368. Raw results:
`experiment1-baseline.json` and `experiment1-a.json`.

## Experiment 2: certify a whole cut across object translation

**Status: retained.**

### Theory

Round 2 proved each root reusable by charging its translation odometer against
its own LOD margin, but selection still probed all 10,000 records after any
database generation change. A single stronger bound can prove them all.

For each `MotionGroup` submission, add the largest member translation—not the
sum of all member translations—to a database-wide double-precision odometer.
No individual instance can have travelled farther than the sum of those batch
maxima. A query snapshots that odometer alongside the minimum remaining margin
of its complete cut. If camera travel, projection-scale travel, and global
object travel together remain below that minimum, every root record remains
valid without reading any of them.

Scale, deformation, mounted topology, and readiness are not translations. A
separate whole-content generation advances on those changes and rejects the
proof in O(1). Instance add/remove/reuse remains covered by the mapping epoch,
and the current TLAS root certificate continues to prove exact visibility.
Individual `moveInstance()` calls are conservatively one-item batches; a
persistent `MotionGroup` obtains the tight max-per-batch bound.

### Result

Pinned 0.40-second samples, nine repetitions:

| Workload | Root certificate only | Translation certificate | Incremental speedup | `a552e47` speedup |
|---|---:|---:|---:|---:|
| Move/publish/select 10% of 10k | 52.9 us | 19.1 us | **2.76x** | **3.52x** |
| Move/publish/select 100% of 10k | 205 us | 184 us | 1.11x | 1.21x |

The result is now returned without scanning the record array on almost every
frame; average walked roots remain below 0.1 in the 10% case and zero in the
100% case. The residual is transform and exact-TLAS update work. The extra
per-root max reduction makes the isolated 400-root motion loop slightly slower,
so the next experiment replaces that update path rather than tuning the cache
again. Raw result: `experiment2-a.json`.

## Experiment 3: make rigid scene translation an explicit primitive

**Status: retained.**

### Theory

The previous API accepted N absolute positions, so the database had to read N
caller positions and N 80-byte instance records merely to discover that a
complete moving population shared one delta. Even after that proof, eagerly
rewriting every instance and TLAS lane remained O(N). A rigid-motion API can
carry the missing fact directly.

`translateInstances(group, delta)` caches aggregate cohort position, bound, and
motion-odometer ranges under a database spatial epoch. Those aggregates prove
the complete batch finite and transactional in O(1). If the cohort is the
complete live population, its translation changes neither TLAS topology nor
relative bounds: Frontier accumulates one deferred base-space offset and one
shared motion odometer. Selection transforms the camera into that base space
once, for both TLAS and mounted hierarchy traversal. Differential edits,
addition, deformation, and rebuild materialize the offset transparently.

A subset still needs exact instance transforms. Its TLAS leaves now retain
grow-only swept envelopes. After an oscillator visits both extremes, later
frames touch instance state but stop rewriting the same TLAS lanes and ancestor
paths. A one-byte loose flag makes non-overview traversal retest current bounds
exactly for frustum and contribution culling; the all-visible root certificate
needs no extra work because containment of an envelope proves containment of
the exact root.

The experiment proceeded in four measured stages:

1. Deferring only TLAS-node translation improved 100% motion from 184 us to
   174 us; eager instance writes still dominated.
2. Deferring instance records too, but proving a common delta from N absolute
   positions, reached 91.0 us.
3. The explicit rigid API and aggregate proof reached 3.74 us for 100% motion.
4. Swept leaves reduced the 10% mover case from 16.5 us to 11.8 us in the
   short experiment.

The benchmark now uses `translateInstances()` rather than rebuilding absolute
position arrays. The world-space transforms submitted on each alternating
frame are identical to the baseline workload; the changed API exposes their
existing rigid-motion invariant.

### Result

Pinned 0.40-second samples, nine repetitions, final median:

| Workload | `a552e47` baseline | Experiment 3 | Speedup |
|---|---:|---:|---:|
| `MotionGroup`, 400 changed | 3.00 us | 0.021 us | **143x** |
| `MotionGroup`, 400 unchanged | 0.968 us | 0.002 us | **484x** |
| Move/publish/select 10% of 10k | 69.6 us | 13.9 us | **5.01x** |
| Move/publish/select 100% of 10k | 225 us | 3.78 us | **59.5x** |
| Stable cached forest, 10k | 17.0 us | 0.066 us | **258x** |
| Camera stationary, 10k | 17.0 us | 0.069 us | **246x** |
| Camera step 0.1 | 18.0 us | 2.71 us | **6.64x** |
| Camera step 16 | 29.7 us | 12.6 us | 2.36x |
| Camera step 256 | 98.2 us | 79.5 us | 1.24x |

The explicit moving-object targets now exceed 5x, while the 100% case is
effectively independent of scene population. Large camera steps remain the
next radical target because they deliberately exhaust many per-root margins.

Debug payload64 BVH4/BVH8 passes 188/188 tests, including new tests for
deferred-offset materialization and exact culling of loose swept leaves. Raw
The full payload32/payload64 BVH4/BVH8 matrix passes 376/376. Raw results:
`experiment3-a.json`, `experiment3-lazy-a.json`,
`experiment3-rigid-api-a.json`, `experiment3-rigid-aggregate-a.json`,
`experiment3-swept-leaves-a.json`, and `experiment3-final-a.json`.
