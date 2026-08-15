# SBC follow-up: benchmark integrity, raw traversal, and measurement control

## Scope and immutable baseline

The follow-up starts from clean commit `d2f308a`, after the final round-3 audit.
It addresses three findings from the Linux AArch64 format-v3 bundle captured at
`20260815T212125Z`:

1. `BM_InstanceForestSelectionScale` reuse mode 2 no longer forced a cache miss.
2. Fully hierarchical uncached selection measured 5.1% slower than the older
   published SBC snapshot.
3. Linux collection used scheduler-default affinity and the `ondemand`
   frequency governor.

The preserved local baseline executable is
`perf_reports/optimization-round4/frontier_bench_baseline_d2f308a.exe`, SHA-256
`F5365AB5CA62F715CB4D57A3FB9A9F390D68EA4075EA5234011569A02F72F096`.
Local experiments use MSVC Release, AVX2/BVH8, IPO, real time, and nine
0.30-second repetitions pinned to logical processor 0.

Baseline medians:

| Workload | Median | Counters |
|---|---:|---:|
| Mounted forest 10k, uncached | 493.361 us | 10,000 walked |
| Mounted forest 10k, nominal forced miss | 0.0254 us | 10,000 reused, 0 walked |

The latter proves the benchmark defect: alternating two current-cut policies
created exactly two semantic view keys, so the new two-entry exact-view memo
learned both and returned complete snapshots.

## Experiment 1: restore a genuine forced-miss workload

**Theory.** Cycle three finite positive thresholds while the exact-view memo has
two entries. Each threshold change advances the per-record epoch and forces a
miss, while the three-key cycle evicts candidates before any key can be admitted
as a recurring complete-view snapshot. This retains the normal query-owned
view API and avoids adding production configuration solely for a benchmark.

**Result.** Retained. The pinned nine-repetition median is 751.270 us with
exactly 0 reused and 10,000 walked roots, versus the defective baseline's
0.0254 us, 10,000 reused, and 0 walked. The matched uncached control is
507.603 us. The repaired case is therefore a genuine cached-miss workload and
shows its expected 48.0% validation/allocation overhead over uncached traversal.

Raw results: `experiment1-forced-miss.json` and `experiment1-pinned.json`.
