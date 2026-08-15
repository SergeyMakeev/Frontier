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

## Experiment 2: direct contained root-leaf dispatch

**Theory.** The uncached hierarchical benchmark mounts the same fully ready,
one-level definition under every TLAS root. Once the TLAS returns a zero plane
mask, the complete root bound is inside the frustum; the mounted definition is
contractually contained by that bound. The generic path nevertheless creates a
root work item, resolves overlay and mount state, and retests the child bounds.

Dispatch a fully ready, overlay-free, root-leaves-only definition directly when
its TLAS mask is zero. The existing contained-leaf emitter remains exact for
nonzero leaf errors. Add a still cheaper block path when every valid leaf has
zero error, avoiding bounds loads and SIMD distance/error evaluation entirely.

**Result.** Rejected. The pinned uncached median moved from 507.603 us to
503.955 us (0.7%, within run-to-run noise), while the repaired forced-miss
median regressed from 751.270 us to 762.426 us. The generic contained emitter
still computed transformed query bounds before discovering that both leaves had
zero error, so the extra dispatch did not remove enough work. The source change
was reverted. Raw result: `experiment2-pinned.json`.

## Experiment 3: preclassify terminal zero-error definitions

**Theory.** Record at definition registration whether every root child is both
a terminal leaf and zero error. For a fully ready, overlay-free placement with
a zero TLAS plane mask, this stronger classification proves the complete answer:
emit the child handles directly. Unlike experiment 2, this path performs no
camera transform, bounds load, frustum test, SIMD distance calculation, or
error encoding.

**Result.** Rejected. The pinned uncached median was 494.500 us versus
493.361 us for the immutable baseline (0.2% slower), and the forced-miss path
was 772.738 us versus 751.270 us (2.9% slower). Preclassification removed the
intended arithmetic but added another definition-state load and branch to every
refined root; it did not improve the complete loop. All runtime changes were
reverted. Raw result: `experiment3-pinned.json`.

## Raw-selection conclusion

The same pinned host measured the older published-code executable at
499.298 us and final `d2f308a` at 493.361 us. The reported 5.1% SBC difference
therefore does not reproduce as a portable runtime regression. Two plausible
specializations failed exact A/B measurement and were removed. The responsible
next action is a same-core SBC rerun, ideally paired with the historical commit,
rather than retaining architecture-neutral complexity to chase an unpaired
frequency/scheduler delta.

## Experiment 4: deterministic Linux performance collection

**Theory.** Scheduler-default placement can migrate a benchmark between unlike
cores, while a demand governor makes short cases depend on the frequency state
left by previous work. On Linux, choose one allowed CPU deterministically by
capacity and maximum frequency, prefix every performance process with `taskset`,
and give every Google Benchmark case an untimed warmup. Capture selected-core
frequency/governor and thermal/load snapshots around each suite so throttling or
power-state drift is visible rather than inferred from noisy medians.

The collector must not silently mutate a machine-wide governor. It records the
policy and documents performance-governor setup for authoritative small-delta
runs; `FRONTIER_PERF_CPU` and `FRONTIER_PERF_WARMUP_SECONDS` remain explicit
overrides.

**Result.** Retained. `run_all_perf.sh` now defaults to
`FRONTIER_PERF_CPU=auto` on Linux. It tests the process's allowed CPU set and
selects the online core with greatest `cpu_capacity`, then greatest
`cpuinfo_max_freq`; an explicit logical CPU or `none` remains available. Every
performance executable runs through that affinity prefix and every Google
Benchmark suite receives a 0.25-second per-case untimed warmup.

The generated manifest and report record the CPU, capacity, maximum frequency,
governor, control mode, and warmup. `performance_state.txt` snapshots current,
minimum, and maximum frequency, system load, and all exposed thermal zones
before and after each suite. Collection fails if that evidence file is absent.
The script passed Bash syntax validation, rejected a nonnumeric warmup with exit
code 2, and the repository's Google Benchmark build accepted the new warmup
flag. A full Linux execution remains the required next SBC measurement because
the Windows development host has neither Linux cpufreq sysfs nor `taskset`.

## Follow-up outcome

- The benchmark-integrity defect is fixed and its counters prove real misses.
- No portable raw mounted-selection regression was found; two attempted
  specializations lost their paired A/B tests and were reverted.
- Future Linux/SBC bundles control core placement and include enough operating
  state to distinguish code deltas from scheduler, governor, or thermal drift.

## Final validation

- Debug correctness matrix: 380/380 tests passed across BVH8/BVH4 and
  64/32-bit payload builds.
- Fresh Release executions of the repaired 10,000-root, 100%-hierarchical
  forced-miss case reported `reused=0` and `walked=10000` for both payload
  widths.
- `bash -n run_all_perf.sh` passed.
- `FRONTIER_PERF_WARMUP_SECONDS=invalid ./run_all_perf.sh` failed early with
  the intended exit code 2.
- Google Benchmark accepted `--benchmark_min_warmup_time` in the repository's
  pinned dependency build.
- `git diff --check` passed before commit.
