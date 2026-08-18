# Current performance

Measurements were captured on 2026-08-18. This report describes Frontier
commit `63f2e3f`. All four format-v3 bundles contain the same source and the
complete 85-case
`frontier_bench` registry for both four- and eight-byte payloads.

Cross-machine timings combine different processors, compilers, operating
systems, SIMD widths, schedulers, and power controls. They are absolute
portability and capacity-planning data for the current codebase.

## Machines and measurement protocol

| Machine | OS and compiler | Native configuration | Scheduling |
|---|---|---|---|
| Apple M2 Max | macOS 26.6.1, Apple Clang 21.0 | arm64, NEON, BVH4 | scheduler default |
| Cortex-A72 SBC | Linux 6.1, GCC 13.3 | arm64, NEON, BVH4 | CPU 4 pinned at observed 2.208 GHz |
| Intel Core i9-12900K | Windows 11, MSVC 19.44 | x86-64, AVX2/FMA, BVH8 | scheduler default |
| AMD EPYC 9654 | Windows 11, MSVC 19.51 | x86-64, AVX2/FMA, BVH8 | scheduler default |

Every report completed without a failed stage. Each payload inventory and JSON
contains all 85 medians: 680 end-to-end measurements across four machines and
two payload widths. Every machine also passed all 452 Debug tests across the
payload32/payload64 and BVH4/BVH8 matrix, for 1,808 test executions in total.

Performance builds used Release, IPO, native `AUTO` BVH width, and disabled
statistics, contract checks, and complete serialized-subtree validation. PGO
was not enabled. Each end-to-end case ran for at least 0.5 seconds in five
repetitions; focused kernels used eleven 0.75-second repetitions. Tables use
the aggregate median of Google Benchmark real time.

The ARM collectors applied a 0.25-second per-benchmark warmup. The SBC stayed
at 2.208 GHz and approximately 41-48 C throughout collection. The macOS and
Windows collectors used their default schedulers and do not provide equivalent
per-case frequency or thermal telemetry, so sub-percent differences on those
hosts are not regression evidence.

The M2 source tree reported dirty only because `.DS_Store` and
`profile_results.zip` were untracked. The other three trees were clean.

## Realistic 100,000-leaf city

`BM_LiveCityDrivingFrame` is the representative dynamic workload. It contains
100 moving 50-leaf cars, 1,000 moving 10-leaf pedestrians, an 85,000-leaf
depth-five static world, 1,191 TLAS roots, a continuously changing 40 mph
camera, curved actor trajectories, transform staging, batched rigid motion,
publication, and exact selection. Each repetition executes 8,192 frames, or
136.5 seconds of simulated 60 Hz time.

The exact cut averages 24,072.71 entries and approximately 461.64 ordered
segments per frame, with 16,455/28,076 entry extrema. The payload64 result is
the default comparison because all four payload64 city measurements have CV
below 0.75%.

Payload64 median real time per simulated frame:

| Machine | Complete driving frame | Motion + publication only | Approx. selection remainder | Frames/s | 60 Hz budget |
|---|---:|---:|---:|---:|---:|
| M2 Max | **18.254 us** | 1.953 us | 16.301 us | 54,783 | 0.110% |
| Cortex-A72 SBC | **69.866 us** | 8.140 us | 61.726 us | 14,313 | 0.419% |
| i9-12900K | **38.143 us** | 2.497 us | 35.646 us | 26,217 | 0.229% |
| EPYC 9654 | **23.144 us** | 1.992 us | 21.152 us | 43,208 | 0.139% |

The selection remainder is the complete-frame median minus the independently
measured motion-frame median, not a nested timer. Motion accounts for 6.5-11.7%
of the measured frame. Even the Cortex-A72 result consumes less than half of
one percent of a 60 Hz CPU frame budget.

Both payload widths:

| Machine | Payload32 driving | CV | Payload64 driving | CV | p32 vs p64 |
|---|---:|---:|---:|---:|---:|
| M2 Max | 18.366 us | 1.13% | 18.254 us | 0.51% | +0.61% |
| Cortex-A72 SBC | 69.755 us | 0.43% | 69.866 us | 0.12% | -0.16% |
| i9-12900K | 36.989 us | 6.78% | 38.143 us | 0.73% | -3.03%* |
| EPYC 9654 | 23.578 us | 0.67% | 23.144 us | 0.61% | +1.88% |

`*` The i9 payload32 point is too noisy to support a payload-width conclusion.
The other three machines show no portable payload32 timing advantage.

The two ARM compilers differ by only two emitted entries over 197,203,640
entries across the complete trajectory, about 0.000001%; extrema and total
segment count match. This is consistent with a boundary floating-point
decision. Payload widths produce identical counts on each machine. BVH4 and
BVH8 have the same entry counts but differ by eight segment descriptors over
the full trajectory because their physical grouping differs.

The four comprehensive bundles run the complete primary `frontier_bench`
registry but not the separately linked `frontier_submission_bench`. They
measure motion and exact selection on every platform, but do not provide
cross-platform render-submission or downstream payload-scan values. None is
inferred from selection time.

## General selection controls

These payload64 cases isolate behavior outside the specialized city path. The
mounted forest contains 10,000 roots and emits 20,000 entries; the flat case
emits 10,000 entries. Stable hit is a previously admitted exact view returned
by the two-entry whole-cut memo and is shown in nanoseconds.

| Machine | Flat raw | Mounted raw | Stable exact-view hit | Forced record miss | Root-only raw |
|---|---:|---:|---:|---:|---:|
| M2 Max | 28.646 us | 380.096 us | 9.96 ns | 575.819 us | 99.828 us |
| Cortex-A72 SBC | 109.772 us | 3,003.100 us | 62.86 ns | 4,524.095 us | 655.695 us |
| i9-12900K | 56.560 us | 913.510 us | 25.75 ns | 1,351.087 us | 254.904 us |
| EPYC 9654 | 114.299 us | 899.582 us | 20.89 ns | 1,300.047 us | 186.099 us |

The memo hit returns an existing view; it does not consume its 20,000 entries.
Renderers must still iterate or submit that view. Forced record misses cost
44.5-51.5% more than disabling reuse because they validate dependencies before
performing the same hierarchy walk. Callers that know no record can survive
should disable reuse.

Root-only work represents 20.7-27.9% of the corresponding refined raw call.
The remaining 72.1-79.3% is mounted-definition traversal and additional
output. This explains why terminal ranges, coherent record reuse, and avoiding
per-leaf resolution dominate the current architecture.

### Recurring and continuously unique cameras

The 10,000-root `BM_MovingCameraSelectionScale` deliberately alternates two
exact camera poses. After both poses are admitted, all four tested separations
are whole-cut memo hits:

| Machine | Stationary | 0.1-unit | 16-unit | 256-unit |
|---|---:|---:|---:|---:|
| M2 Max | 10.12 ns | 10.59 ns | 10.57 ns | 10.58 ns |
| Cortex-A72 SBC | 63.85 ns | 67.83 ns | 67.84 ns | 67.82 ns |
| i9-12900K | 25.93 ns | 26.65 ns | 26.60 ns | 26.62 ns |
| EPYC 9654 | 21.27 ns | 21.86 ns | 22.04 ns | 21.84 ns |

These values characterize exact recurrence, not continuous camera motion. The
live-city benchmark above is the continuously unique trajectory and should be
used for frame budgeting.

### Moving root cohorts

`BM_MovingObjectsSelectionScale` translates a stable cohort by 0.25 units,
publishes, and selects the 20,000-entry cut. Conservative placement envelopes
and record certificates allow almost all cuts to survive these translations;
the 100% cohort is also more contiguous than the distributed 10% cohort.

Payload64 median frame time:

| Machine | Move 10% of 10k | Reused roots/call | Move 100% of 10k | Reused roots/call |
|---|---:|---:|---:|---:|
| M2 Max | 7.304 us | 9,999.906 | 4.620 us | 9,999.094 |
| Cortex-A72 SBC | 38.500 us | 9,999.910 | 18.624 us | 9,999.100 |
| i9-12900K | 16.351 us | 9,999.906 | 7.109 us | 9,999.088 |
| EPYC 9654 | 13.499 us | 9,999.906 | 6.396 us | 9,999.100 |

Percentage moved is therefore no longer a proxy for percentage re-walked.
The realistic city case remains the better mixed moving/static workload.

## Reusable assembly

The 400-house scene compares duplicated flattened house detail with one shared
mounted definition. Payload64 selection and complete construction medians:

| Machine | Flat raw select | Assembled raw select | Flat memo hit | Assembled memo hit | Flat construct | Assembled construct | Construction reduction |
|---|---:|---:|---:|---:|---:|---:|---:|
| M2 Max | 8.292 us | 9.499 us | 11.5 ns | 11.5 ns | 66.919 us | 17.464 us | 73.9% |
| Cortex-A72 SBC | 44.471 us | 65.041 us | 77.6 ns | 77.6 ns | 550.080 us | 96.355 us | 82.5% |
| i9-12900K | 14.136 us | 20.157 us | 28.5 ns | 28.8 ns | 136.900 us | 54.506 us | 60.2% |
| EPYC 9654 | 23.556 us | 14.758 us | 28.6 ns | 28.7 ns | 114.047 us | 34.815 us | 69.5% |

Assembly reduces construction latency by 60-83% on every target. Raw mounted
traversal remains architecture-sensitive: it is 37% faster on EPYC and
15-46% slower on the other machines. Once an exact view is memoized, flat and
assembled representations have the same practical lookup cost.

Retained payload64 memory depends on native BVH width rather than CPU model:

| Native layout | Representation | Immutable definitions | Placement state | Total retained | Reduction vs flat |
|---|---|---:|---:|---:|---:|
| BVH4 | Flattened | 200.563 KiB | 7.209 KiB | 207.771 KiB | - |
| BVH4 | Assembled | 23.063 KiB | 54.129 KiB | 77.191 KiB | 62.8% |
| BVH8 | Flattened | 198.813 KiB | 7.209 KiB | 206.021 KiB | - |
| BVH8 | Assembled | 22.875 KiB | 50.418 KiB | 73.293 KiB | 64.4% |

## Mutation and lifecycle reference

Payload64 median real time:

| Machine | Mount + unmount | Toggle shared readiness, 10k | Register 4,096 nodes | Spawn + remove + publish | Override + flush 256 bounds |
|---|---:|---:|---:|---:|---:|
| M2 Max | 33.0 ns | 36.681 us | 3.599 us | 72.095 us | 3.574 us |
| Cortex-A72 SBC | 248.5 ns | 212.794 us | 11.032 us | 499.623 us | 25.649 us |
| i9-12900K | 82.8 ns | 67.080 us | 4.880 us | 215.164 us | 7.331 us |
| EPYC 9654 | 69.5 ns | 62.888 us | 4.447 us | 113.239 us | 5.536 us |

Registration is the trusted zero-copy Release path; source-byte copying and
handle release are outside the timed region. The lifecycle case maintains a
steady 1,024-instance population. The bounds case includes all 256 copy-on-
write updates and `flushBounds()`.

## Payload width and retained city state

Across all 85 cases, the geometric-mean payload32/payload64 time ratio is
1.023 on M2 Max, 1.006 on the SBC, 1.007 on i9, and 1.003 on EPYC. Restricting
the calculation to cases of at least 1 us reduces those ratios to 1.011,
1.000, 0.999, and 1.002 respectively. Payload32 therefore has no portable
execution-time advantage.

It does save retained memory. The live-city counters are:

| Native layout | Payload | Immutable state | Mount state | Query state | Combined reported state |
|---|---:|---:|---:|---:|---:|
| BVH4 | 32 | 5,891.375 KiB | 243.238 KiB | 1,239.477 KiB | 7,374.090 KiB |
| BVH4 | 64 | 6,332.563 KiB | 243.238 KiB | 1,571.711 KiB | 8,147.512 KiB |
| BVH8 | 32 | 9,439.875 KiB | 237.793 KiB | 1,234.480 KiB | 10,912.148 KiB |
| BVH8 | 64 | 9,881.063 KiB | 237.793 KiB | 1,566.715 KiB | 11,685.570 KiB |

Payload32 saves about 773.4 KiB in this scene: 9.5% of the reported BVH4
state and 6.6% of the BVH8 state. Choose payload width for value range and
memory, not an assumed CPU-time win.

## Focused kernel context

Rates below come directly from median real time. Wide-AABB results are
normalized per processed lane; append bandwidth uses the 12-byte
`FrontierEntry` stream.

| Machine | Six-plane wide AABB | Distance/error | Cache-hit validation | Append, 8 entries/range |
|---|---:|---:|---:|---:|
| M2 Max | 450 M lanes/s | 2,401 M lanes/s | 714 M records/s | 46.6 GB/s |
| Cortex-A72 SBC | 42.9 M lanes/s | 182 M lanes/s | 119 M records/s | 3.05 GB/s |
| i9-12900K | 159 M lanes/s | 1,390 M lanes/s | 562 M records/s | 18.7 GB/s |
| EPYC 9654 | 176 M lanes/s | 2,796 M lanes/s | 577 M records/s | 24.6 GB/s |

The M2 is 6.0-15.3x faster than the SBC in the isolated traversal,
validation, and output kernels but only 3.83x faster in the complete city
frame. The current architecture reduces the amount of those brute-force kernels
executed, making frame time less dependent on peak SIMD and bandwidth.

## Variability and interpretation

Of 85 cases per payload, the number at or below 2% CV was 79/82 on M2,
79/81 on the SBC, 75/76 on i9, and 78/79 on EPYC for payload32/payload64.
Most high-CV cases take substantially less than one microsecond, where a small
absolute scheduler disturbance produces a large percentage. The important
exception is the i9 payload32 city frame at 6.78% CV; use its stable payload64
result for cross-machine planning.

One repeatable non-city anomaly remains. Payload32 forced record misses are
11.0-12.6% slower than payload64 on M2 and 2.5-3.7% slower on the SBC, while
the Windows machines are essentially neutral. Because the smaller record
layout is slower, this is more likely a compiler/template-layout or
cache-control-flow interaction than payload bandwidth. It should be evaluated
with a focused paired non-IPO run before changing production code.

## Conclusions

- The current implementation passes the complete 452-test matrix on all four
  devices and produces consistent live-city output across NEON/BVH4 and
  AVX2/BVH8.
- The 100,000-leaf moving city costs 18.254-69.866 us per payload64 database
  frame, or 0.110-0.419% of a 60 Hz frame budget.
- Exact recurring camera poses are 10-68 ns memo lookups; they must not be used
  as a proxy for continuous motion. The city workload supplies that coverage.
- Reusable assembly reduces construction by 60-83% and retained memory by
  63-64%; raw mounted traversal remains target-sensitive.
- Payload32 saves 773 KiB in the measured city but offers no portable timing
  advantage.
- The broad four-device bundles omit the isolated submission executable, so
  this snapshot does not report complete render-plus-payload-scan timing.

The analyzed bundles are:

- `frontier-perf-Darwin-arm64-20260818T041737Z.zip` (M2 Max);
- `frontier-perf-Linux-aarch64-20260818T041504Z.zip` (Cortex-A72 SBC);
- `frontier-perf-Windows-AMD64-20260818T045800Z.zip` (i9-12900K);
- `frontier-perf-Windows-AMD64-20260818T045843Z.zip` (EPYC 9654).
