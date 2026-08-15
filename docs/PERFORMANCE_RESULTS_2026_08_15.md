# Cross-platform performance snapshot (2026-08-15)

This report summarizes the four release-candidate bundles captured from
Frontier commit `d3f12a0` (`Optimize TLAS and subtree traversal`). It describes
the performance of that commit on four machines; it is not a before/after
comparison with the pre-optimization implementation. The controlled,
same-machine result for that comparison remains the
[22.7% optimization result](PERFORMANCE_OPTIMIZATION_2026.md#final-result).

## Test matrix

| Machine | OS and compiler | Native configuration |
|---|---|---|
| Apple M2 Max | macOS 26.6, Apple Clang 21.0 | arm64, NEON, BVH4 |
| RK3399 (4 Cortex-A72 + 4 Cortex-A53) | Linux 6.1, GCC 13.3 | arm64, NEON, BVH4 |
| Intel Core i9-12900K | Windows 11, MSVC 19.44 | x86-64, AVX2/FMA, BVH8 |
| AMD EPYC 9654 | Windows 11, MSVC 19.51 | x86-64, AVX2/FMA, BVH8 |

Every bundle reports `COMPLETE` with no failed stage and contains results for
both four-byte and eight-byte payload builds. Performance builds used Release,
IPO, the native `AUTO` BVH width, and disabled contract checks, statistics, and
complete serialized-subtree validation. Each machine also passed all 180
Debug correctness tests for the native and alternate BVH widths: 720 test
executions across the four reports.

These format-v2 bundles were captured before the cross-machine collector was
corrected: `run_all_perf.sh` and `.bat` hard-coded the
`BM_SubtreeAssembly` filter despite describing their output as the end-to-end
suite. Consequently, the bundles contain that family only; format-v3
collectors run and validate the complete registered suite. The old collector
ran the assembly family five times and the focused production kernels eleven
times. Tables below use the aggregate
median of Google Benchmark real time. No affinity mask was applied. This
matches normal scheduler behavior but makes absolute cross-machine rankings
less controlled, particularly on the heterogeneous i9-12900K and RK3399.

The M2 bundle had only unrelated untracked files, and the i9 bundle had only a
`.gitignore` edit. The Linux and EPYC source trees were clean; all four report
the same source commit.

## Reusable assembly

The benchmark represents a city containing 400 houses and eight ready detail
nodes per house. The flattened representation authors every detail node into
the city definition. The assembled representation authors the house once and
mounts that shared definition below 400 city nodes.

### Selection time

Eight-byte payload, microseconds:

| Machine | Flat, reuse off | Assembled, reuse off | Assembly change | Flat, cache hit | Assembled, cache hit |
|---|---:|---:|---:|---:|---:|
| M2 Max | 8.253 | 9.376 | 13.6% slower | 0.480 | 0.480 |
| RK3399 | 44.412 | 63.782 | 43.6% slower | 3.553 | 3.577 |
| i9-12900K | 13.960 | 19.905 | 42.6% slower | 1.317 | 1.313 |
| EPYC 9654 | 24.021 | 14.678 | 38.9% faster | 0.894 | 0.899 |

Uncached assembly is therefore architecture-sensitive. Mount traversal adds
indirection, while definition sharing reduces the immutable working set; the
balance changes with the memory hierarchy, compiler, BVH width, and backend.
It is a substantial win on the EPYC, a small cost on the M2 Max, and a larger
cost on the RK3399 and i9. These differences are repeatable within the
individual bundles: the relevant 400-house medians have 0.1-5.0% real-time
coefficient of variation.

Once the query's stable-frontier cache hits, the two representations differ by
at most 0.7% on every machine. In practical workloads where the camera and
published scene frequently remain inside the reuse envelope, assembly's extra
topology is removed from the hot selection path.

### Construction and memory

Eight-byte payload, complete scene-construction time:

| Machine | Flat | Assembled | Latency reduction |
|---|---:|---:|---:|
| M2 Max | 59.945 us | 17.588 us | 70.7% |
| RK3399 | 518.897 us | 96.164 us | 81.5% |
| i9-12900K | 119.753 us | 56.176 us | 53.1% |
| EPYC 9654 | 97.587 us | 35.009 us | 64.1% |

Construction benefits from assembly on every target, and the benefit grows
with the number of repeated houses. At 400 houses it is 2.1-5.4 times faster.

Memory is determined by BVH width rather than CPU model:

| Native layout | Representation | Immutable definitions | Placement state | Total retained | Reduction vs flat |
|---|---|---:|---:|---:|---:|
| BVH4 | Flat | 200.563 KiB | 7.209 KiB | 207.771 KiB | - |
| BVH4 | Assembled | 23.063 KiB | 54.129 KiB | 77.191 KiB | 62.8% |
| BVH8 | Flat | 198.813 KiB | 7.209 KiB | 206.021 KiB | - |
| BVH8 | Assembled | 22.875 KiB | 50.418 KiB | 73.293 KiB | 64.4% |

Assembly replaces duplicated immutable node data with placement state. Even
with 400 mounted houses, it retains only about 36-37% of the flattened scene's
memory.

## Four-byte versus eight-byte payloads

Four-byte payloads reduce total retained memory by 6.8% in the flattened
400-house scene and by 2.1-2.2% in the assembled scene. The smaller assembled
gain is expected because placement state, which contains no duplicated payload
word for every shared node, dominates more of its footprint.

There is no general execution-time win from selecting a four-byte payload. In
the two 400-house reuse-off selection cases, four-byte timings were within
1.7% of eight-byte timings on all four machines. Cached sub-microsecond cases
were noisier, and construction results were target-dependent; notably,
assembled construction on the EPYC was 17.5% slower with four-byte payloads
despite low variation in both samples. Choose payload width for value range
and memory footprint, not on the assumption that four bytes will always run
faster.

## Focused kernel context

The following rates are calculated from median real time so Windows CPU-time
accounting cannot distort the comparison. Wide kernels are normalized per
processed lane; append bandwidth uses the 12-byte `FrontierEntry` size.

| Machine | Six-plane wide AABB | Distance/error | Cache-hit validation | Append, 8 entries/range |
|---|---:|---:|---:|---:|
| M2 Max | 450 M lanes/s | 2,401 M lanes/s | 715 M records/s | 46.6 GB/s |
| RK3399 | 42.9 M lanes/s | 174 M lanes/s | 135 M records/s | 3.1 GB/s |
| i9-12900K | 92.9 M lanes/s | 894 M lanes/s | 338 M records/s | 11.4 GB/s |
| EPYC 9654 | 174 M lanes/s | 2,742 M lanes/s | 571 M records/s | 24.3 GB/s |

These probes explain broad target behavior but do not replace the end-to-end
measurements. They combine different processors, compilers, operating systems,
and the native BVH4/BVH8 choice.

## Conclusions

- The optimized commit builds and passes the native/alternate-width Debug
  suite on all four targets.
- Reusable assembly is a clear construction and memory win: 53-82% lower
  construction latency and 63-64% lower retained memory at 400 houses.
- Cached selection makes flattened and assembled layouts effectively
  equivalent. Uncached mount traversal remains target-sensitive and should be
  measured on each shipping architecture.
- Four-byte payloads provide a modest, predictable memory saving but no
  portable speed guarantee. Eight bytes remain the correct general default.
- These bundles validate current cross-platform behavior. They do not contain
  the old implementation and therefore neither replace nor independently
  reproduce the campaign's pinned 22.7% before/after speedup.

The source bundles analyzed for this report are:

- `frontier-perf-Darwin-arm64-20260815T011402Z.zip`;
- `frontier-perf-Linux-aarch64-20260815T011336Z.zip`;
- `frontier-perf-Windows-AMD64-20260815T011304Z.zip` (i9-12900K);
- `frontier-perf-Windows-AMD64-20260815T011426Z.zip` (EPYC 9654).
