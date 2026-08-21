# Performance optimization 2026 — round 10

## Objective and baseline

This experiment adds a low-cost explicit TLAS topology refresh for simulations
whose continuously moving population measurably degrades traversal quality.
The baseline is repository commit `ff25376`; no PGO, linker placement, CPU-
specific dispatch, or other integration-sensitive technique is involved.

The existing `optimize()` operation always combined three jobs:

1. rebuild TLAS topology at the configured quality tier;
2. compact dead dense instance slots;
3. reorder all per-instance and query-record streams into traversal order.

That combination is valuable at an occasional loading/synchronization point,
but unnecessarily expensive when a live city only needs fresh conservative
topology. The measured application symptom was a recurring ~2 ms repair spike
when `optimize()` was used to recover moving-scene query quality.

At the user's request, the preliminary incremental-local-rotation experiment
was skipped. The implementation went directly to an exact Morton rebuild that
does not compact or permute dense storage.

## Theory

All exact current instance bounds already live in dense instance records. A
fresh TLAS can therefore be produced without first repairing the topology that
will be discarded. The Morton builder is linear apart from radix sorting and
constructs wide leaves and inner levels in streaming passes. It should be much
cheaper than recursive Binned-SAH construction, and it does not require the
dense-stream permutation performed by `optimize()`.

The predicted properties were:

- full topology drift is removed in one explicit operation;
- exact bounds, masks, contribution maxima, TLAS back-pointers, area baseline,
  edit baseline, loose flags, and repair queues are rebuilt/reset;
- public handles and dense IDs remain stable;
- `MotionGroup` and `RigidMotionGroup` cached physical mappings remain valid;
- dead dense slots remain allocated;
- subsequent selection may visit more TLAS nodes than after a Binned-SAH
  `optimize()`.

## Implementation

The public operation is:

```cpp
void SpatialDatabase::refreshTlas();
```

It performs the following sequence:

1. discard pending lane IDs for the old topology;
2. flush queued node-bound edits into exact instance records;
3. materialize any deferred whole-population translation;
4. rebuild TLAS nodes through the Morton path;
5. clear incremental edit, loose-bound, and repair-queue state;
6. establish new population and stored-area drift baselines;
7. preserve dense slots, physical ordering, layout version, and mapping
   version.

The internal rebuild entry now receives its policy explicitly:

```cpp
tlasRebuild(reorderInstances, useConfiguredQuality);
```

This replaces mutable “next build is quality” state. Correctness/recovery
builds and `refreshTlas()` select Morton directly; first spatialization and
`optimize()` select the configured tier. Every exact rebuild establishes a
fresh population baseline.

The advisory API was renamed from `optimizeRecommended` to
`topologyRebuildRecommended`, because either explicit operation can now answer
the same drift signal. Debug state similarly exposes `activeQuality`,
`configuredQuality`, and `rebuildBaselineInstances`.

## Runtime comparison tooling

The dynamic-city sample exposes both methods without restarting:

- manual **Refresh TLAS now** and **Optimize now** buttons;
- periodic or recommendation-gated scheduling;
- a scheduled-method choice between `refreshTlas()` and `optimize()`;
- per-method counts and last-rebuild timing;
- active versus configured TLAS quality in the health panel.

Recommendation-gated scheduling defaults to `refreshTlas()` every two seconds,
so a moving city can recover topology without paying compaction and stream
permutation on every rebuild.

## Correctness experiments

A dedicated test creates 64 instances, removes every third instance to leave
dense holes, moves a surviving instance, triggers topology advice, and calls
`refreshTlas()`. It verifies:

- allocated slot count is unchanged and remains larger than live count;
- dense ID, layout version, and mapping version are unchanged;
- stale removed handles remain stale;
- the moved instance's rebuilt TLAS leaf is exact;
- active topology is Morton while configured quality remains Binned-SAH;
- repair work and topology advice are cleared;
- selection still returns every live instance.

The complete Debug matrix passed: 244/244 tests across BVH4 and BVH8.
The Release city target and both payload-width benchmark targets also built.

## Performance experiment

`BM_TlasTopologyRebuild` creates a Binned-SAH scene, moves a spatially
distributed 10% cohort once before the timed interval, and times only repeated
explicit rebuilds. Method `0` is `refreshTlas()`; method `1` is `optimize()`.
The 1,191-root case matches the live-city TLAS population; 10,000 roots shows
scaling.

Windows x64 Release, BVH4, IPO on, contract/statistics/validation checks off,
9 repetitions, median wall time:

| Payload | Roots | `refreshTlas()` | `optimize()` | Speedup | Wall CV refresh / optimize |
|---|---:|---:|---:|---:|---:|
| 64-bit | 1,191 | 38.3 us | 237 us | **6.19x** | 1.24% / 1.60% |
| 32-bit | 1,191 | 37.2 us | 238 us | **6.40x** | 1.28% / 0.71% |
| 64-bit | 10,000 | 318 us | 2,630 us | **8.27x** | 2.62% / 0.22% |
| 32-bit | 10,000 | 323 us | 2,663 us | **8.24x** | 2.97% / 0.79% |

Wall time is the application-visible safe-point latency and the acceptance
metric for this benchmark.

At 1,191 roots the Morton topology used 172 nodes and the configured
Binned-SAH topology used 258 nodes. At 10,000 roots they used 1,431 and 2,137
nodes respectively.

Raw local results were written to:

- `bench_results/tlas_refresh_payload64.json`
- `bench_results/tlas_refresh_payload32.json`

## Tradeoffs and operating policy

`refreshTlas()` is the ordinary runtime topology reset. It is O(live instance
count), still produces a one-frame safe-point cost, and intentionally does not
promise a hard frame-time cap. In return it removes accumulated topology drift
at roughly one sixth to one eighth of the full optimization cost in this
experiment.

Use `optimize()` when either of these costs has become material:

- dead dense slots should be reclaimed;
- Morton query traversal is measurably worse than the configured Median or
  Binned-SAH tier.

A practical city policy is to answer routine drift advice with
`refreshTlas()`, measure selection and retained dense capacity, and reserve
`optimize()` for infrequent loading screens, large population churn, or an
application-selected low-impact synchronization point.

## Correction: rebuild latency was not sufficient evidence

The first implementation above was rejected after the city TLAS visualization
showed depth-four boxes spanning large fractions of a 4,590-instance scene
immediately after `refreshTlas()`. The health panel reported 99.7% lane
occupancy and zero area *growth*, but neither metric described absolute spatial
overlap. The rebuild was exact with respect to containment while still being a
poor broadphase.

The benchmark defect was that `BM_TlasTopologyRebuild` measured only the
safe-point rebuild. It did not measure a selective query using the topology
that the rebuild produced. `BM_TlasQualitySelection` exposed the transferred
cost on the 10,000-root close-camera grid, with the same 627 results:

| Builder | Selection median |
|---|---:|
| fixed-group Morton | ~26.0 us |
| Median | ~3.52 us |
| BinnedSAH | ~3.73 us |

The apparent 6-8x rebuild win therefore moved about 22 us into every selective
query. High lane occupancy was not a sufficient quality proxy because fixed
groups could be full while joining spatially distant regions.

### Experiments

1. **Median fallback.** `refreshTlas()` was first changed to the existing exact
   Median builder. This restored query quality, but rebuild medians were 137 us
   at 1,191 roots and 1,739 us at 10,000 roots, only about 1.74x and 1.51x
   faster than `optimize()` respectively.
2. **Prefix-aware Morton hierarchy.** Sorted Morton ranges were split at their
   highest differing prefix bit rather than fixed group boundaries. The
   close-camera query regressed further to ~39 us and the tree grew to 2,633
   nodes. Rejected.
3. **Morton leaves plus median upper hierarchy.** Sorting still supplied full
   leaves, while a longest-axis hierarchy organized the leaf bounds. The query
   remained ~28.7 us. Preventing leaves from crossing coarse Morton-cell
   boundaries increased it to ~30.6 us. Both were rejected: even localized
   curve discontinuities created enough broad leaves to dominate selective
   traversal.
4. **Direct spatial bins.** Every large range chooses its longest centroid
   axis, counts instances into `kWide` equal-width bins, scatters 32-bit dense
   ids through a retained scratch stream, and recurses into non-empty bins.
   Ranges no larger than `kWide * kWide`, coincident ranges, and ranges with
   more than seven eighths of their population in one bin use the Median
   builder as a bounded fallback. This passed both rebuild and query gates.

The first accepted version copied each scattered partition back into the input
stream. A follow-up alternates the input and scratch streams at every recursive
level instead. It removed one full range pass per level; the 10,000-root
payload32 median moved from 643 us to 625 us (2.8%), while payload64 moved from
639 us to 636 us (within the run-to-run margin but in the same direction).

The public tier is named `TlasQuality::SpatialBins`; there is no Morton build
path or Morton-key storage in the current implementation. `refreshTlas()` uses
`SpatialBins`, while `optimize()` continues to use the configured tier and
performs compaction and physical reordering.

### Corrected measurements

Windows x64 Release, BVH8, IPO on, contract/statistics/validation checks off,
9 rebuild repetitions and 9 post-rebuild query repetitions. Values are median
wall time; both payload widths return 627 entries in the query gate.

| Payload | Roots | SpatialBins `refreshTlas()` | BinnedSAH `optimize()` | Rebuild speedup |
|---|---:|---:|---:|---:|
| 64-bit | 1,191 | 53.4 us | 232 us | **4.34x** |
| 32-bit | 1,191 | 53.5 us | 238 us | **4.45x** |
| 64-bit | 10,000 | 636 us | 2,615 us | **4.11x** |
| 32-bit | 10,000 | 625 us | 2,611 us | **4.18x** |

`BM_TlasPostRebuildSelection` now starts with a configured BinnedSAH tree,
moves a distributed 10% cohort, performs one explicit rebuild outside the
timed selection loop, and then measures the close-camera query:

| Payload | after `refreshTlas()` | after `optimize()` | Refresh result |
|---|---:|---:|---:|
| 64-bit | **3.43 us** | 3.95 us | 13.2% faster |
| 32-bit | **3.42 us** | 3.95 us | 13.4% faster |

The independent quality-tier gate measured SpatialBins at 3.42 us, Median at
3.57 us, and BinnedSAH at 3.83 us for the same visible set. SpatialBins used
1,919 nodes versus 2,121 for Median and 2,185 for BinnedSAH. The new builder
therefore retains its rebuild advantage without creating the giant overlapping
boxes or charging subsequent selective queries.

The Debug suite also contains a structural regression check on a 100x100 city
grid. After `refreshTlas()`, near-leaf internal bounds must stay below one third
of the world width; this directly catches the city-spanning pattern shown by
the visualization in both BVH4 and BVH8 builds.

### Current tradeoffs

- SpatialBins reuses the retained 32-bit TLAS postorder stream as its scatter
  buffer during a build. Removing the two 12-byte-per-root Morton sort buffers
  reduces retained build-scratch capacity without adding another stream.
- Equal-width bins do not guarantee balanced populations. The seven-eighths
  skew guard and small-range Median path guarantee progress and contain the
  pathological case, at the cost of comparison work for those ranges.
- `refreshTlas()` is still a full O(live roots) safe-point operation and does
  not provide a hard frame-time bound. It avoids compaction and dense stream
  permutation, so `optimize()` remains the explicit choice for reclaiming dead
  slots or restoring configured physical order.
- Area growth remains a drift metric relative to the most recent topology. It
  is not an absolute overlap score. Rebuild acceptance is now gated by actual
  post-rebuild selective traversal rather than inferred from occupancy or
  growth alone.
