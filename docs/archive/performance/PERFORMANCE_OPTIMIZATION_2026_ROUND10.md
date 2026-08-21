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
