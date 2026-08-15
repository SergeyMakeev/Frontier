# Cross-platform performance snapshot (2026-08-15)

This report summarizes four comprehensive release-candidate bundles captured
from Frontier commit `35e7b3f`. The library implementation is the optimized
`d3f12a0` implementation; the intervening commits add documentation, benchmark
coverage, the format-v3 collector, and an ignore-list entry. This is an
absolute cross-platform snapshot, not a before/after comparison with the old
implementation. The controlled same-machine comparison remains the
[22.7% optimization result](PERFORMANCE_OPTIMIZATION_2026.md#final-result).

## Test matrix

| Machine | OS and compiler | Native configuration |
|---|---|---|
| Apple M2 Max | macOS 26.6, Apple Clang 21.0 | arm64, NEON, BVH4 |
| RK3399 (4 Cortex-A72 + 4 Cortex-A53) | Linux 6.1, GCC 13.3 | arm64, NEON, BVH4 |
| Intel Core i9-12900K | Windows 11, MSVC 19.44 | x86-64, AVX2/FMA, BVH8 |
| AMD EPYC 9654 | Windows 11, MSVC 19.51 | x86-64, AVX2/FMA, BVH8 |

Every format-v3 bundle reports `COMPLETE` with no failed stage. Each payload
inventory lists 83 benchmark cases, and each corresponding JSON contains all
83 medians with no missing case: 664 end-to-end case measurements across four
machines and two payload widths. Each machine also passed all 360 Debug tests
covering payload32/payload64 and BVH4/BVH8: 1,440 test executions in total.

Performance builds used Release, IPO, the native `AUTO` BVH width, and
disabled contract checks, statistics, and complete serialized-subtree
validation. Every end-to-end case ran for at least 0.5 seconds in each of five
repetitions. Focused production kernels used eleven 0.75-second repetitions.
Tables use the aggregate median of Google Benchmark real time. No affinity
mask was applied. This matches normal scheduler behavior but makes absolute
cross-machine rankings less controlled, particularly on the heterogeneous
i9-12900K and RK3399.

The M2 source tree contained only unrelated `.DS_Store` and
`profile_results.zip` files. The other three source trees were clean; all four
report the same commit.

## Headline results

For the eight-byte-payload, fully hierarchical 10,000-instance scene that
emits 20,000 frontier entries:

- a stationary cached selection takes 108-524 us and is 3.7-8.5 times faster
  than raw traversal;
- a 16-unit camera step retains about 99.4% of roots and takes 112-575 us;
- moving 10% of roots, publishing, and selecting takes 171-1,084 us while
  preserving exactly 90% root reuse;
- moving and invalidating all 10,000 roots takes 913-9,243 us.

For the separate 400-house assembly scene, sharing the house definition lowers
complete construction latency by 55-81% and retained memory by 63-64%.
Payload32 saves memory but has no portable execution-time advantage. These are
workload-specific medians, not latency guarantees.

## Frontier selection

The primary table uses an eight-byte payload and 10,000 visible instances.
The flat cases emit 10,000 entries. The fully hierarchical mounted forest
refines every instance and emits 20,000 entries.

Microseconds per `selectFrontier()` call:

| Machine | Flat, raw | Flat, reuse enabled | Mounted, raw | Mounted, stable hit | Mounted, forced miss |
|---|---:|---:|---:|---:|---:|
| M2 Max | 47.765 | 47.901 | 404.196 | 108.031 | 576.102 |
| RK3399 | 218.176 | 215.210 | 3,106.796 | 524.382 | 4,685.754 |
| i9-12900K | 60.159 | 61.283 | 939.053 | 124.983 | 1,288.005 |
| EPYC 9654 | 110.148 | 110.571 | 947.064 | 111.682 | 1,341.174 |

Reuse deliberately bypasses the all-flat cache machinery, so enabling it for
a flat TLAS is effectively free. In the fully hierarchical forest, a stable
cut is 3.7-8.5 times faster than a raw walk. The forced-miss case alternates
cut policy on every call: it measures the deliberately adversarial cost of
validating cached records and then re-walking every root, not normal camera
motion. It is 37-51% slower than disabling reuse, so callers that know reuse is
impossible should disable it.

A second raw case views the same mounted forest from far enough away to stop
at its renderable TLAS roots. Its 10,000-entry costs are 115.459 us (M2),
795.378 us (RK3399), 255.936 us (i9), and 193.864 us (EPYC). Together with the
refined case, this separates top-level query/dispatch cost from BLAS traversal.

### Where raw selection time goes

Subtracting the matched root-only case from fully refined raw selection gives
a structural estimate of the work below the TLAS. The cameras and output sizes
differ, so this is not nested timing of one call, but it locates the dominant
cost reliably:

| Machine | TLAS/root-only work | Mounted refinement and additional output |
|---|---:|---:|
| M2 Max | 115.459 us (28.6%) | 288.737 us (71.4%) |
| RK3399 | 795.378 us (25.6%) | 2,311.418 us (74.4%) |
| i9-12900K | 255.936 us (27.3%) | 683.117 us (72.7%) |
| EPYC 9654 | 193.864 us (20.5%) | 753.200 us (79.5%) |

Thus roughly 71-80% of an uncached refined call is below the TLAS: mounted
definition traversal, dependent state loads, readiness decisions, and the
additional output. Stable reuse removes 73-88% of the raw call. Conversely,
when every cached record is forced to miss, validation and cache bookkeeping
add 172 us (M2), 1,579 us (RK3399), 349 us (i9), and 394 us (EPYC) beyond the
corresponding reuse-disabled raw walk.

### Current-cut policy

The mixed-readiness scene has 400 unavailable ideal nodes. Ancestor policy
falls back to 400 ready parents; descendant policy proves and emits a complete
2,000-node ready descendant cover. Both preserve a complete renderable current
cut, but the descendant result is substantially more detailed.

Eight-byte payload, microseconds:

| Machine | Ready ancestors | Ready descendants | Time change | Current entries (ancestor / descendant) | Ideal entries |
|---|---:|---:|---:|---:|---:|
| M2 Max | 24.332 | 31.680 | 30.2% slower | 400 / 2,000 | 800 |
| RK3399 | 130.021 | 179.012 | 37.7% slower | 400 / 2,000 | 800 |
| i9-12900K | 56.391 | 73.059 | 29.6% slower | 400 / 2,000 | 800 |
| EPYC 9654 | 54.309 | 75.904 | 39.8% slower | 400 / 2,000 | 800 |

The 30-40% additional selection time produces five times as many current
entries here; it is not overhead for the same result. It is the cost of proving
and returning the more detailed complete descendant cover.

## Moving cameras and objects

The camera benchmark alternates between two translated cameras over the fully
hierarchical 10,000-instance forest. The timed region contains selection only
and returns 20,000 entries.

Eight-byte payload, microseconds; parentheses show reused roots:

| Machine | Stationary | 0.1-unit step | 16-unit step | 256-unit step |
|---|---:|---:|---:|---:|
| M2 Max | 107.801 (100%) | 107.990 (100%) | 112.229 (99.41%) | 152.594 (90.72%) |
| RK3399 | 541.534 (100%) | 533.895 (100%) | 575.327 (99.45%) | 937.813 (90.81%) |
| i9-12900K | 127.437 (100%) | 128.691 (100%) | 136.552 (99.41%) | 232.947 (90.71%) |
| EPYC 9654 | 111.637 (100%) | 112.069 (100%) | 122.883 (99.41%) | 219.753 (90.74%) |

A 0.1-unit step remains entirely inside every cached validity interval. Even a
256-unit step reuses about 90.7% of roots, limiting selection to 0.153-0.938 ms
on these machines.

The moving-object benchmark includes batched root motion, `applyUpdates()`,
and selection of the resulting 20,000-entry cut. Moving 10% of the roots
invalidates exactly that 10%; moving every root leaves nothing reusable.
`MotionGroup` is shown separately for 400 alternating transforms and measures
the caller's position-array update plus `moveInstances()`, without publication
or selection.

Eight-byte payload, microseconds:

| Machine | MotionGroup, 400 roots | Move/publish/select 10% of 10k | Move/publish/select 100% of 10k |
|---|---:|---:|---:|
| M2 Max | 1.365 | 171.395 | 912.868 |
| RK3399 | 5.947 | 1,084.059 | 9,243.448 |
| i9-12900K | 3.514 | 263.143 | 2,221.829 |
| EPYC 9654 | 3.512 | 245.177 | 1,899.069 |

The 10% case preserves 90% reuse on every architecture. This is why stable
`MotionGroup` ordering and per-root frontier versions matter: moving objects
pay in proportion to the invalidated cohort instead of unconditionally
re-walking the entire scene.

### Where dynamic-frame time goes

The complete moving-object case does not place nested timers around motion,
publication, and selection. The following split is therefore a model, not a
direct measurement: motion scales the measured 400-root `MotionGroup` cost;
the 10% selection proxy is the measured 256-unit camera case with about 90.7%
reuse; the 100% selection proxy is the forced-miss case; publication is the
remainder and absorbs fixed costs and modeling error.

| Machine | Move 10%: motion / publish / select | Move 100%: motion / publish / select |
|---|---:|---:|
| M2 Max | 2% / 9% / 89% | 4% / 33% / 63% |
| RK3399 | 1% / 12% / 87% | 2% / 48% / 51% |
| i9-12900K | 3% / 8% / 89% | 4% / 38% / 58% |
| EPYC 9654 | 4% / 7% / 90% | 5% / 25% / 71% |

With 10% motion, complete selection still dominates because it validates and
emits the entire cut. When every root moves, TLAS publication becomes a second
major cost, especially on the RK3399. Copying transforms itself remains only
about 1-5% in this model. Exact attribution requires separately timed motion,
publication, and post-publication selection cases in a future collection.

### Mutation and lifecycle reference

Eight-byte payload, median real time:

| Machine | Mount + unmount | Toggle readiness, 10k placements | Register 4,096 nodes | Spawn + remove + publish | Override + flush 256 bounds |
|---|---:|---:|---:|---:|---:|
| M2 Max | 32.4 ns | 35.009 us | 1.257 us | 71.758 us | 3.549 us |
| RK3399 | 245.4 ns | 198.857 us | 1.795 us | 491.760 us | 26.358 us |
| i9-12900K | 88.2 ns | 67.489 us | 0.434 us | 211.913 us | 7.255 us |
| EPYC 9654 | 67.6 ns | 61.501 us | 0.636 us | 114.135 us | 5.658 us |

Registration is the zero-copy Release path with complete structural validation
disabled; source-byte copying and handle release are outside the timed region.
The lifecycle case operates in a steady 1,024-instance population. The bounds
case includes 256 copy-on-write updates and `flushBounds()`.

## Reusable assembly

The benchmark represents a city containing 400 houses and eight ready detail
nodes per house. The flattened representation authors every detail node into
the city definition. The assembled representation authors the house once and
mounts that shared definition below 400 city nodes.

### Selection time

Eight-byte payload, microseconds:

| Machine | Flat, reuse off | Assembled, reuse off | Assembly change | Flat, cache hit | Assembled, cache hit |
|---|---:|---:|---:|---:|---:|
| M2 Max | 8.249 | 9.435 | 14.4% slower | 0.555 | 0.713 |
| RK3399 | 44.409 | 63.453 | 42.9% slower | 3.450 | 3.725 |
| i9-12900K | 14.228 | 19.876 | 39.7% slower | 1.312 | 1.315 |
| EPYC 9654 | 23.769 | 14.576 | 38.7% faster | 0.888 | 0.885 |

Uncached assembly is therefore architecture-sensitive. Mount traversal adds
indirection, while definition sharing reduces the immutable working set; the
balance changes with the memory hierarchy, compiler, BVH width, and backend.
It is a substantial win on the EPYC, a small cost on the M2 Max, and a larger
cost on the RK3399 and i9. The uncached results reproduce the earlier bundles
within 2%, despite being a separate comprehensive run.

Once the query's stable-frontier cache hits, the absolute difference is only
0.003-0.275 us on these 400-house scenes. The sub-microsecond M2 cases have
9-12% coefficient of variation, so their large percentage difference should
not be interpreted as a similarly large frame-level cost. In practical
workloads where the camera and published scene remain inside the reuse
envelope, assembly's extra topology is nearly removed from the hot path.

### Construction and memory

Eight-byte payload, complete scene-construction time:

| Machine | Flat | Assembled | Latency reduction |
|---|---:|---:|---:|
| M2 Max | 59.905 us | 18.378 us | 69.3% |
| RK3399 | 502.758 us | 96.312 us | 80.8% |
| i9-12900K | 121.457 us | 54.371 us | 55.2% |
| EPYC 9654 | 96.611 us | 36.333 us | 62.4% |

Construction benefits from assembly on every target, and the benefit grows
with the number of repeated houses. At 400 houses it is 2.2-5.2 times faster.

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

There is no general execution-time win from selecting a four-byte payload.
Across all 83 end-to-end cases, the geometric-mean payload32/payload64 time
ratio is 1.001 on M2, 1.018 on RK3399, 1.015 on i9, and 1.012 on EPYC. Across
the 52 selection cases specifically, payload32 is 1.0% faster on M2 but 3.1%,
2.5%, and 1.5% slower on RK3399, i9, and EPYC respectively. Individual tiny
cases are much noisier than these aggregate ratios. Choose payload width for
value range and memory footprint, not on the assumption that four bytes will
run faster.

## Focused kernel context

The following rates are calculated from median real time so Windows CPU-time
accounting cannot distort the comparison. Wide kernels are normalized per
processed lane; append bandwidth uses the 12-byte `FrontierEntry` size.

| Machine | Six-plane wide AABB | Distance/error | Cache-hit validation | Append, 8 entries/range |
|---|---:|---:|---:|---:|
| M2 Max | 451 M lanes/s | 2,403 M lanes/s | 714 M records/s | 46.5 GB/s |
| RK3399 | 42.8 M lanes/s | 176 M lanes/s | 125 M records/s | 3.1 GB/s |
| i9-12900K | 91.3 M lanes/s | 853 M lanes/s | 338 M records/s | 11.2 GB/s |
| EPYC 9654 | 175 M lanes/s | 2,754 M lanes/s | 569 M records/s | 24.6 GB/s |

These probes explain broad target behavior but do not replace the end-to-end
measurements. They combine different processors, compilers, operating systems,
and the native BVH4/BVH8 choice.

## Conclusions

- The optimized implementation builds and passes the complete payload-width
  and BVH-width Debug matrix on all four targets.
- Stable hierarchical cuts are 3.7-8.5 times faster than raw traversal.
  Real camera motion retains 99.4% of roots at a 16-unit step and about 90.7%
  even at a 256-unit step in the measured scene.
- Moving 10% of roots preserves exactly 90% reuse, keeping the complete
  move/publish/select frame at 0.171-1.084 ms; moving all 10,000 roots costs
  0.913-9.243 ms across these very different processors.
- Reusable assembly is a clear construction and memory win: 55-81% lower
  construction latency and 63-64% lower retained memory at 400 houses.
- Cached assembly adds at most 0.275 us in absolute terms here. Uncached mount
  traversal remains target-sensitive and should be measured on each shipping
  architecture.
- Four-byte payloads provide a modest, predictable memory saving but no
  portable speed guarantee. Eight bytes remain the correct general default.
- These comprehensive bundles validate current cross-platform behavior. They
  do not contain the old implementation and therefore neither replace nor
  independently reproduce the campaign's pinned 22.7% before/after speedup.

The source bundles analyzed for this report are:

- `frontier-perf-Darwin-arm64-20260815T034005Z.zip`;
- `frontier-perf-Linux-aarch64-20260815T033949Z.zip`;
- `frontier-perf-Windows-AMD64-20260815T033937Z.zip` (i9-12900K);
- `frontier-perf-Windows-AMD64-20260815T034013Z.zip` (EPYC 9654).
