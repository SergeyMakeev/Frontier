# Benchmarking Frontier

Use Release builds on an otherwise idle machine in its normal high-performance
power mode. Compare median aggregates and inspect coefficient of variation
before treating a small delta as meaningful.

The repository performance runners use the production-speed profile: Release
with interprocedural optimization, statistics off, contract checks off, and
complete serialized-subtree validation off. Benchmark inputs must therefore be
trusted and valid. Correctness belongs to the separate Debug unit-test build.

The end-to-end suite is compiled twice with otherwise identical settings:
`frontier_bench` uses an eight-byte payload word and
`frontier_bench_payload32` uses a four-byte payload word. The local runners and
CI execute both by default, and every JSON result records
`frontier_payload_bytes` as `8` or `4` in its context. This makes platform-level
cache and bandwidth differences directly comparable without changing the
library's macro-based public payload customization.

## End-to-end subtree benchmark

`frontier_bench` contains the paired city/house experiment:

- `BM_SubtreeAssembly_FrontierCost` compares a flattened city definition with
  a city whose house nodes mount one shared house definition;
- `BM_SubtreeAssembly_ConstructCost` compares complete authoring,
  registration, instantiation, and mounting cost.
- `BM_BranchWidthOccupancy` runs the assembled 400-house scene with exactly
  2, 4, 6, or 8 active shared-definition children per house for direct
  BVH4/BVH8 comparison.
- `BM_SharedNodeReadinessFanout` toggles one house definition node shared by
  32, 128, or 400 mounted houses and measures complete coverage propagation.
- `BM_MixedReadinessFrontier` compares ancestor- and descendant-preferring
  current cuts on a hierarchy where an unavailable ideal node has a complete
  ready descendant cover. Counters report current, ideal, physically stored
  bucket-entry counts, and retained query bytes.
- `BM_SharedNodeReadinessLargeFanout` extends shared-readiness propagation to
  1,024 and 10,000 placements of one definition node.
- `BM_MountUnmountLifecycle` measures steady-state mount/unmount operations in
  an assembled hierarchy.
- `BM_MountUsageConsumption` measures consumption of query-recorded mount use
  for streaming and collapse decisions.
- `BM_MotionGroupSteady` measures repeated rigid translation through a stable
  `MotionGroup` and the explicit `translateInstances()` path.
- `BM_MovingObjectsSelectionScale` moves a distributed 10% or 100% of a
  mounted 1,000/10,000-root forest, publishes the update, and selects the next
  frontier through the same rigid-translation API. Counters separate roots
  reused from roots re-walked.
- `BM_SubtreeBuilder_ConstructCost` isolates serialized definition building
  before registration and instantiation.
- `BM_SubtreeRegistration` isolates validation and zero-copy registration for
  128- and 4,096-node serialized definitions; input copying and release are
  outside the timed region.
- `BM_FlatTlasSelectionScale` covers raw and cached selection at 1,000 and
  10,000 TLAS-owned single-node objects. The reuse-enabled cases verify that
  the automatic all-flat direct path stays at raw-selection cost.
- `BM_InstanceForestSelectionScale` covers raw and cached selection across
  forests of mounted instance hierarchies. Reuse mode 2 cycles three thresholds
  to force deterministic record-cache misses without admitting any of those
  keys to the two-entry exact-view memo.
- `BM_MovingCameraSelectionScale` alternates between two translated cameras
  over a fully hierarchical forest. It covers stationary, 0.1-unit, 16-unit,
  and 256-unit steps and reports average reused/walked roots and the reuse
  rate.
- `BM_InstanceForestRootSelectionScale` uses the same mounted forest but a
  distant camera that stops at renderable TLAS roots, separating top-level
  query/dispatch cost from refined BLAS traversal.
- `BM_TlasQualitySelection` compares Morton, median, and binned-SAH TLAS
  selection with all-visible and close-camera views, and reports entry count,
  node count, and TLAS bytes.
- `BM_FlatInstanceLifecycle` measures steady-state TLAS spawn/remove plus its
  amortized maintenance barrier in a 1,024-object population.
- `BM_BoundsOverrideBatch` measures sparse and promoted-dense copy-on-write
  bound updates and flushes in batches of 1, 32, 64, and 256 definition nodes;
  `overlay_KB` reports the retained COW storage after each batch.

Both representations produce the same fully refined frontier. Cases cover 32,
128, and 400 houses. Frontier selection runs both raw and with a stationary
warm `SpatialQuery`. Counters report immutable definition bytes, mounted state,
total memory, placement count, and frontier size.

```sh
./run_perf_bench.sh \
  --benchmark_filter=BM_SubtreeAssembly \
  --benchmark_repetitions=7 \
  --benchmark_report_aggregates_only=true
```

```bat
run_perf_bench.bat --benchmark_filter=BM_SubtreeAssembly --benchmark_repetitions=7 --benchmark_report_aggregates_only=true
```

These commands run both payload widths. Set `FRONTIER_PERF_PAYLOAD_BITS=32` or
`FRONTIER_PERF_PAYLOAD_BITS=64` for a quick single-width run; the default is
`both`. With no caller arguments, results are written to
`real_world_perf_payload32.json` and `real_world_perf_payload64.json`. In
both-width mode, do not pass `--benchmark_out` because one caller-provided path
cannot hold both results; select one width or run the executables directly when
custom output paths are required.

Run either executable directly after a build when preferred:

```sh
build-perf/bench/frontier_bench \
  --benchmark_filter=BM_SubtreeAssembly \
  --benchmark_out=result.json \
  --benchmark_out_format=json
```

Multi-config generators place the executable under the configuration
directory instead, for example
`build-perf/bench/Release/frontier_bench.exe` with Visual Studio.

Replace `frontier_bench` with `frontier_bench_payload32` for the matched
four-byte build.

The repository retains the keyless serialized-bytes API comparison in:

- `bench_results/bytes_api_before.json`
- `bench_results/bytes_api_after.json`

## Comparing BVH4 and BVH8

Branch width changes both SIMD work and memory layout, so compare complete
workflows rather than only arithmetic throughput. Configure two otherwise
identical builds:

```sh
cmake -S . -B build-bvh4 -DCMAKE_BUILD_TYPE=Release \
  -DFRONTIER_BUILD_TESTS=OFF -DFRONTIER_BUILD_BENCH=ON \
  -DFRONTIER_IPO=ON -DFRONTIER_CONTRACT_CHECKS=OFF \
  -DFRONTIER_VALIDATE_SUBTREES=OFF \
  -DFRONTIER_BVH_WIDTH=4 -DFRONTIER_AVX2=OFF
cmake -S . -B build-bvh8 -DCMAKE_BUILD_TYPE=Release \
  -DFRONTIER_BUILD_TESTS=OFF -DFRONTIER_BUILD_BENCH=ON \
  -DFRONTIER_IPO=ON -DFRONTIER_CONTRACT_CHECKS=OFF \
  -DFRONTIER_VALIDATE_SUBTREES=OFF \
  -DFRONTIER_BVH_WIDTH=8 -DFRONTIER_AVX2=OFF
```

This is the matched 128-bit comparison on x86. Also measure a BVH8 AVX2 build
when that is a supported deployment target. On ARM64, the same width pair
compares one versus two NEON vectors per wide record.

Run `BM_KernelWideAabb` and `BM_KernelDistanceErrorCurrent` to explain SIMD
cost, then use `BM_SubtreeAssembly_FrontierCost`,
`BM_MixedReadinessFrontier`, and `BM_FlatTlasSelectionScale` for the decision.
The flat-TLAS cases report `tlas_nodes` and `tlas_KB`. BVH4 halves one
`WideBlock` from 256 to 128 bytes and reduces combined hot/cold TLAS-node
storage from 320 to 160 bytes, but can require more blocks and a deeper tree.
Favor it only when target-scene lane occupancy, culling, and cache behavior
compensate for that extra traversal.

## Machine characterization

`frontier_machine_bench` is kept separate so synthetic probes do not perturb
end-to-end code layout. It covers scalar dependency/throughput, SIMD, division
and square root, cache and memory bandwidth, hardware prefetch, dependent-load
latency, random-load parallelism, branches, sparse masks, and production wide
bound/error kernels.

```sh
./run_machine_bench.sh
./run_arch_bench.sh
```

Use machine results to explain an end-to-end difference, not as a substitute
for it. Matching source revisions, compiler flags, architecture backend, and
power state matter.

## Cross-machine collector

The unified collectors configure a dedicated Release build, run the complete
BVH4/BVH8 and payload32/payload64 Debug correctness matrix, capture the entire
registered end-to-end benchmark suite for both payload widths, add machine and
focused-kernel characterization, and package the result:

```sh
./run_all_perf.sh m2-max
```

```bat
run_all_perf.bat i9
```

Output is written below `perf_reports/`. `FRONTIER_PERF_LABEL`,
`FRONTIER_ALL_PERF_BUILD_DIR`, and `FRONTIER_PERF_REPORT_ROOT` override the
label and locations. Each report contains `real_world_perf_payload32.json` and
`real_world_perf_payload64.json`. Report format v3 marks
`end_to_end_scope=complete` in `manifest.txt` and verifies that assembly,
readiness, root motion, moving-camera selection, flat and hierarchical
selection, combined moving-object frames, instance lifecycle, and bound-update
families are present before a report can be `COMPLETE`. The collector also
records each executable's `--benchmark_list_tests` inventory and proves that
every listed case appears in the corresponding JSON. Performance uses the
platform's native `AUTO` BVH width; use the explicit configurations above for
a full alternate-width performance comparison.

On Linux, the collector uses `taskset` by default to pin every performance
process to one allowed CPU. `FRONTIER_PERF_CPU=auto` selects the highest CPU
capacity and then the highest advertised maximum frequency, which keeps a
heterogeneous SBC on a deterministic performance core. Set an explicit logical
CPU number to override that choice, or `FRONTIER_PERF_CPU=none` to retain normal
scheduler placement.

Every case receives a 0.25-second untimed warmup before measurement so an
`ondemand` or `schedutil` governor can raise frequency. Override it with
`FRONTIER_PERF_WARMUP_SECONDS`. The collector deliberately does not change the
machine-wide governor; for authoritative small-delta comparisons, select the
platform's performance governor before running the collector. The report
records the chosen CPU, capacity, maximum frequency, governor, warmup, and
before/after frequency, load, and thermal snapshots in `manifest.txt`,
`REPORT.md`, and `performance_state.txt`.

The current registry contains 83 cases per payload build. With five
0.5-second-minimum repetitions plus correctness and machine characterization,
a complete report normally takes roughly 10-20 minutes depending on build and
host speed.

The latest analyzed release snapshot covers M2 Max, RK3399, i9-12900K, and
EPYC 9654 format-v3 results from commit `35e7b3f`; see
[PERFORMANCE_RESULTS_2026_08_15.md](PERFORMANCE_RESULTS_2026_08_15.md). It
uses median real time and keeps cross-machine current-state results separate
from the pinned before/after optimization score.

At a high level, its eight-byte-payload 10,000-root hierarchical workload
emits 20,000 entries in 108-524 us on a stable cached cut, 112-575 us after a
16-unit camera step, and 171-1,084 us for a complete frame that moves 10% of
roots, publishes, and selects. Those ranges span very different processors and
are portability evidence, not a cross-machine ranking or latency guarantee.

## macOS hardware counters

`profile_macos_cpu.sh` records a selected end-to-end case with optimized source
line tables and the available Xcode CPU counter template. The default is the
400-house assembled raw traversal case. It writes the Instruments trace plus a
compact summary under `profile_results/`.

Useful overrides include `FRONTIER_PROFILE_FILTER`,
`FRONTIER_PROFILE_MIN_TIME`, `FRONTIER_PROFILE_TIME_LIMIT`,
`FRONTIER_PROFILE_OUTPUT_DIR`, and `FRONTIER_DEVELOPER_DIR`.

## Build options

| Option | Default | Meaning |
|---|---:|---|
| `FRONTIER_BUILD_TESTS` | standalone `ON`; subdirectory `OFF` | build correctness tests |
| `FRONTIER_BUILD_BENCH` | `OFF` | build benchmark executables; repository performance runners enable it in a dedicated Release build |
| `FRONTIER_BVH_WIDTH` | `AUTO` | select BVH8 for AVX2 and BVH4 for SSE2/NEON/scalar; explicit `4` or `8` overrides it; serialized bytes must match |
| `FRONTIER_AVX2` | `ON` | enable AVX2/FMA for BVH8 on supported x86-64 targets; BVH4 stays 128-bit |
| `FRONTIER_FORCE_SCALAR` | `OFF` | disable intrinsic implementations |
| `FRONTIER_PROFILE_SYMBOLS` | `OFF` | keep optimized Clang line tables |
| `FRONTIER_IPO` | `OFF` | enable supported interprocedural optimization for Frontier and its benchmark executables; performance runners set it to `ON` |
| `FRONTIER_STATS` | `OFF` | retain per-query traversal counters |
| `FRONTIER_CONTRACT_CHECKS` | `ON` | check caller preconditions; performance runners explicitly disable it for trusted workloads |
| `FRONTIER_VALIDATE_SUBTREES` | `ON` | validate complete serialized subtree structure during registration; disable only for trusted compatible builder output |
