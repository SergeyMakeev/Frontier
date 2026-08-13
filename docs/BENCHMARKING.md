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
- `BM_SubtreeRegistration` isolates validation and zero-copy registration for
  128- and 4,096-node serialized definitions; input copying and release are
  outside the timed region.
- `BM_FlatTlasSelectionScale` covers raw and cached selection at 1,000 and
  10,000 TLAS-owned single-node objects. The reuse-enabled cases verify that
  the automatic all-flat direct path stays at raw-selection cost.
- `BM_FlatInstanceLifecycle` measures steady-state TLAS spawn/remove plus its
  amortized maintenance barrier in a 1,024-object population.
- `BM_BoundsOverrideBatch` measures sparse and promoted-dense copy-on-write
  bound updates and flushes in batches of 1, 32, 64, and 256 definition nodes.

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
build/bench/frontier_bench \
  --benchmark_filter=BM_SubtreeAssembly \
  --benchmark_out=result.json \
  --benchmark_out_format=json
```

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
`WideBlock` from 256 to 128 bytes and one `TlasNode` from 320 to 192 bytes, but
can require more blocks and a deeper tree. Favor it only when target-scene lane
occupancy, culling, and cache behavior compensate for that extra traversal.

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

The unified collectors configure a dedicated Release build, run correctness,
capture both payload-width end-to-end suites plus system/toolchain metadata,
and package the result:

```sh
./run_all_perf.sh m2-max
```

```bat
run_all_perf.bat i9
```

Output is written below `perf_reports/`. `FRONTIER_PERF_LABEL`,
`FRONTIER_ALL_PERF_BUILD_DIR`, and `FRONTIER_PERF_REPORT_ROOT` override the
label and locations. Each report contains `real_world_perf_payload32.json` and
`real_world_perf_payload64.json`.

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
