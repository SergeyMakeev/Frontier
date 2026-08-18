# SBC follow-up: benchmark integrity, raw traversal, and measurement control

> Archived engineering journal. Revisions, APIs, measurements, and conclusions
> in this file are historical and are not current product documentation.

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

## Controlled SBC validation

The follow-up AArch64 bundle
`frontier-perf-Linux-aarch64-20260815T231001Z.zip` has SHA-256
`BA1C5F99CF3AE3A3A7DB4EDED45AE3917E64F35E32CA372A206EBC705A343E62`.
It is a complete, clean build of `8320f75`: both payload inventories contain
all 83 medians and all 380 Debug matrix tests pass.

The collector selected logical CPU 4, a Cortex-A72-class core with capacity
1024 and a 2,208,000 kHz maximum. Although the governor remained `ondemand`,
every boundary snapshot from collector start through the architecture suite
reported 2,208,000 kHz. Big-core temperature rose from 40.7 C to 46.2 C and
the hottest reported zone ended at 47.2 C, with no evidence of throttling.

### Defect closure

All four repaired mode-2 cases report zero reused roots and the complete root
population walked. The 10,000-root medians are:

| Payload | Hierarchical roots | Median | CV | Reused / walked |
|---|---:|---:|---:|---:|
| 64-bit | 50% | 2,966.919 us | 0.58% | 0 / 10,000 |
| 64-bit | 100% | 4,769.455 us | 0.41% | 0 / 10,000 |
| 32-bit | 50% | 3,033.783 us | 0.56% | 0 / 10,000 |
| 32-bit | 100% | 4,966.904 us | 0.59% | 0 / 10,000 |

The benchmark defect is therefore closed on the target architecture. For the
fully hierarchical case, cache validation and allocation make a genuine miss
54.9% slower than raw traversal with the 64-bit payload and 61.7% slower with
the 32-bit payload.

### Raw mounted-selection closure

| Eight-byte-payload run | Raw mounted 10k | Change vs controlled rerun |
|---|---:|---:|
| Older published SBC reference | 3,106.796 us | +0.92% |
| `d2f308a`, scheduler default | 3,266.012 us | +6.09% |
| `8320f75`, CPU 4 + warmup | 3,078.618 us | baseline |

The controlled rerun is 5.74% faster than the immediately preceding
scheduler-default bundle and 0.91% faster than the older published reference.
Payload32 independently improves from 3,136.215 us to 3,070.947 us (2.08%).
Root-only selection remains effectively unchanged at 662.071 us versus
663.938 us. These paired results close the suspected portable raw-traversal
regression; the earlier 5.1% delta was a measurement-state artifact.

### Performance-goal confirmation on the SBC

The controlled medians preserve the major architectural improvements against
the earlier published RK3399 results:

| Workload, 64-bit payload | Published | Controlled final | Speedup |
|---|---:|---:|---:|
| Motion group, 400 changed roots | 5.947 us | 0.0207 us | 287x |
| Move/publish/select, 10% of 10k | 1,084.059 us | 38.417 us | 28.2x |
| Move/publish/select, 100% of 10k | 9,243.448 us | 17.213 us | 537x |
| Stable mounted exact view, 10k | 524.382 us | 0.0651 us | 8,058x |
| Recurring camera, 256-unit separation | 937.813 us | 0.0699 us | 13,412x |

The camera row is specifically the two-pose recurring-view workload described
in the benchmark. It demonstrates exact whole-cut memo lookup, not the cost of
an arbitrary stream of unique camera poses.

### Stability and comparison policy

Seventy-eight of 83 payload64 cases and 80 of 83 payload32 cases have CV at or
below 2%. The apparent 32.1% payload64 flat-construction regression has 24.3%
CV and is rejected as noise. Across the 79 unchanged cases per payload, the
median change from the prior bundle is only -0.10% for payload64 and -0.09% for
payload32. Production runtime sources did not change between those bundles;
small opposite-direction deltas in focused cache kernels therefore reflect the
changed execution conditions, not code. `8320f75` is the new controlled SBC
baseline, and subsequent performance claims should compare only against runs
using the same affinity, warmup, and telemetry protocol.
