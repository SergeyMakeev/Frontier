# Benchmarking Frontier

Use Release builds on an otherwise idle machine in its normal high-performance
power mode. Compare median aggregates and inspect coefficient of variation
before treating a small delta as meaningful.

## End-to-end subtree benchmark

`frontier_bench` contains the paired city/house experiment:

- `BM_SubtreeAssembly_FrontierCost` compares a flattened city definition with
  a city whose house nodes mount one shared house definition;
- `BM_SubtreeAssembly_ConstructCost` compares complete authoring,
  registration, instantiation, and mounting cost.
- `BM_SharedNodeReadinessFanout` toggles one house definition node shared by
  32, 128, or 400 mounted houses and measures complete coverage propagation.

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

Run the executable directly after a build when preferred:

```sh
build/bench/frontier_bench \
  --benchmark_filter=BM_SubtreeAssembly \
  --benchmark_out=result.json \
  --benchmark_out_format=json
```

The repository retains the keyless serialized-bytes API comparison in:

- `bench_results/bytes_api_before.json`
- `bench_results/bytes_api_after.json`

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
capture benchmarks and system/toolchain metadata, and package the result:

```sh
./run_all_perf.sh m2-max
```

```bat
run_all_perf.bat i9
```

Output is written below `perf_reports/`. `FRONTIER_PERF_LABEL`,
`FRONTIER_ALL_PERF_BUILD_DIR`, and `FRONTIER_PERF_REPORT_ROOT` override the
label and locations.

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
| `FRONTIER_BUILD_TESTS` | `ON` | build correctness tests |
| `FRONTIER_BUILD_BENCH` | `ON` | build benchmark executables |
| `FRONTIER_AVX2` | `ON` | enable AVX2/FMA on supported x86-64 targets |
| `FRONTIER_FORCE_SCALAR` | `OFF` | disable intrinsic implementations |
| `FRONTIER_PROFILE_SYMBOLS` | `OFF` | keep optimized Clang line tables |
