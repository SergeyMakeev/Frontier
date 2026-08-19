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

`frontier_bench` contains the city/house workloads:

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
  reused from roots re-walked. Conservative placement envelopes and validity
  certificates may preserve a record across the small translation, so the
  moved percentage is intentionally not assumed to equal the walked
  percentage.
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
- `BM_MovingCameraSelectionScale` alternates between two exact recurring camera
  poses over a fully hierarchical forest. It covers stationary, 0.1-unit,
  16-unit, and 256-unit separations and measures the two-entry whole-cut memo
  after admission. It is not a continuously unique camera stream; counters
  report average reused/walked roots and the reuse rate.
- `BM_LiveCityDrivingFrame` measures one complete dynamic city frame with a
  continuously changing 40 mph camera, 100 independently moving 50-leaf cars,
  1,000 independently moving 10-leaf pedestrians, and an 85,000-leaf static
  world organized primarily into independent depth-five blocks. The scene has
  exactly 100,000 logical leaves and 1,191 TLAS roots. Every moving actor's
  local forward axis follows the tangent of its curved track, including the
  opposite heading for reverse traffic. Actor roots are authored as
  conservative yaw-swept envelopes, as production vehicle/crowd broadphases
  commonly are. Full actor-transform staging, batched rigid motion,
  publication, and selection are timed. Each repetition runs
  8,192 fixed frames, or 136.5 seconds of simulated 60 Hz time, so every host
  samples two complete deterministic trajectories. `orientation_KB` reports
  the optional cold yaw/local-bounds stream.
- `BM_LiveCityMotionFrame` runs the identical actor trajectories and measures
  only transform staging, the two rigid motion-group submissions, and
  publication. Comparing it with the complete driving-frame case separates
  actor/TLAS update cost from cached traversal and result production.
- `BM_LiveCityRenderSubmissionFrame` extends the complete driving frame by
  iterating the two-span current cut, resolving every immutable payload, and
  writing `{payload, instance, error}` records to a preallocated CPU render
  stream. This covers downstream result consumption without adding allocator
  or graphics-driver variance. Compare it with `BM_LiveCityDrivingFrame` to
  isolate payload resolution and command-generation cost. It is compiled only
  into `frontier_submission_bench` and
  `frontier_submission_bench_payload32`; keeping it out of the primary
  executables prevents the additional function from perturbing the LTO/link
  layout of the established selection guards.
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

## Paired revision gate

On Linux, use `run_paired_perf.sh` when accepting a small change. Give it two
already-configured Release build directories and a report root:

```sh
./run_paired_perf.sh build-baseline build-candidate perf_reports 4
```

Set `FRONTIER_PAIRED_CASES` to a comma-separated label list for a longer
focused follow-up, for example `FRONTIER_PAIRED_CASES=identity_50` with 12
cycles. The unfiltered matrix remains the required first acceptance pass.

The runner pins every benchmark process to one CPU and executes four ABBA
cycles per workload. It covers selection-only and render-submission city
frames, motion-only publication, 50% and 100% uncached hierarchy guards, and
integer/branch/distance/memory machine controls. Each report archives the exact
runner, commits and dirty patches, executable hashes, build paths, compiler,
kernel, governor, per-sample load average, frequency, and temperature.

Summarize a collected report with:

```sh
python3 analyze_paired_perf.py perf_reports/frontier-paired-TIMESTAMP \
  --output perf_reports/frontier-paired-TIMESTAMP/SUMMARY.md
```

The analyzer verifies the ABBA schedule and complete cycle quartets, reports
raw medians/CVs and cycle-paired effects, bootstraps a cycle-level interval,
checks CPU-time versus wall-time agreement, and computes an independent
machine-control geomean. Verdicts require the complete interval to clear a
±0.25% practical-equivalence band; negative candidate changes are
improvements. Do not retain a narrow workload win when the controls or an
unaffected decomposition case move materially in the same run.

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

The current primary registry contains 85 cases per payload build. With five
0.5-second-minimum repetitions plus correctness and machine characterization,
a complete report normally takes roughly 10-20 minutes depending on build and
host speed.

The current analyzed release snapshot covers M2 Max, Cortex-A72 SBC,
i9-12900K, and EPYC 9654 format-v3 results from commit `63f2e3f`; see
[PERFORMANCE.md](PERFORMANCE.md). It
uses median real time. All four primary reports contain 85 medians for both
payload widths and pass all 452 Debug tests.

At a high level, its payload64 continuously moving 100,000-leaf city completes
actor motion, publication, and exact selection in 18.254-69.866 us per frame;
the isolated motion/publication phase takes 1.953-8.140 us. An admitted exact
view in the separate recurring-camera control returns from the two-entry memo
in 10-68 ns, but that lookup neither consumes the 20,000-entry view nor models
continuous camera motion. These ranges span very different processors and are
portability evidence, not a cross-machine ranking or latency guarantee.

The format-v3 comprehensive collector currently inventories `frontier_bench`
and `frontier_bench_payload32`. It does not run the isolated
`frontier_submission_bench` pair, so the current four-machine snapshot does
not claim cross-platform downstream payload-scan timings.

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
| `FRONTIER_DEBUG_TOOLS` | `OFF` | expose on-demand read-only TLAS/query-cache inspection; adds no selection instrumentation |
| `FRONTIER_CONTRACT_CHECKS` | `ON` | check caller preconditions; performance runners explicitly disable it for trusted workloads |
| `FRONTIER_VALIDATE_SUBTREES` | `ON` | validate complete serialized subtree structure during registration; disable only for trusted compatible builder output |
