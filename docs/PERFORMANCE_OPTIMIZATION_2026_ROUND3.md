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
