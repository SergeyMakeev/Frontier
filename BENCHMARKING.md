# Benchmarking HLodTree

This guide describes the current performance executables, repeatable
cross-machine collection, and macOS CPU-counter capture. For headline results,
see [README.md](README.md); for implementation analysis, see
[ARCHITECTURE.md](ARCHITECTURE.md).

Always benchmark a Release build on an otherwise idle machine in its normal
high-performance power mode. The supplied collectors intentionally use the
operating system's default scheduler placement rather than pinning a core, so
the result represents normal application placement.

## Complete cross-machine report

Use the unified collector when comparing machines:

```sh
./run_all_perf.sh m2-max
./run_all_perf.sh epyc
./run_all_perf.sh armbian-sbc
```

```bat
run_all_perf.bat i9
```

`run_all_perf.sh` supports macOS and Linux, including arm64 Linux distributions
such as Armbian. It selects native arm64/NEON on Apple Silicon, AVX2 on x86-64,
and the appropriate non-AVX2 backend on other supported hosts. The Windows
collector builds the AVX2 configuration.

Each invocation:

- configures a dedicated Release build;
- builds and runs the correctness suite;
- runs the real-world, machine-characterization, and focused kernel suites;
- records CPU, memory, topology, OS, power, compiler, build, Git, and command
  information; and
- creates a timestamped directory and archive under `perf_reports/`.

A complete archive contains:

| File | Contents |
|---|---|
| `real_world_perf.json` | end-to-end selection and moving-object workloads |
| `machine_perf.json` | ALU, SIMD, branch, cache, latency, and bandwidth probes |
| `arch_kernel_perf.json` | longer samples of production kernels and output append |
| `REPORT.md` | concise run summary and status |
| `manifest.txt` | machine-readable metadata |
| `hardware.txt` | CPU, memory, topology, OS, and power information |
| `toolchain.txt` | compiler, CMake, target, and build configuration |
| `source.txt` | Git revision and working-tree state |
| `commands.txt` | exact benchmark filters and sampling parameters |
| `tests.log` and benchmark logs | correctness and raw console output |

The archive is `.zip` when a ZIP tool is available; the shell collector falls
back to `.tar.gz`. A failed run still packages partial data and identifies the
failed stage in `REPORT.md`.

The optional label is only for report naming. It can also be supplied through
`HLOD_PERF_LABEL`. Override the build and report directories with:

```sh
HLOD_ALL_PERF_BUILD_DIR=/fast/build \
HLOD_PERF_REPORT_ROOT=/results \
./run_all_perf.sh test-machine
```

On Windows, set the same environment variables before invoking
`run_all_perf.bat`.

## Interpreting reports

Compare matching source revisions and build options. Start with Google
Benchmark's `median` aggregate. Check the coefficient of variation (`cv`) and
the run logs before treating a small difference as meaningful. Thermal state,
background work, power policy, and scheduler placement can easily move a
single sample.

`real_world_perf.json` is the primary end-to-end comparison. Use
`machine_perf.json` to test whether a platform gap correlates with memory
bandwidth, dependent-load latency, SIMD arithmetic, branches, or sparse-mask
iteration. Use `arch_kernel_perf.json` to connect those machine traits to the
wide AABB, distance/error, cache-hit, and output-append kernels used by the
runtime.

Randomized tests and benchmarks use repository-owned xorshift32 generation
with explicit fixed seeds. Float mapping, bounded integers, and shuffling are
also local implementations, so one source revision and seed describe the same
generated workload on MSVC, libc++, and libstdc++. Benchmark cases run in
registration order.

## Focused benchmark runner

For a quick runtime benchmark without collecting a full report:

```sh
./run_perf_bench.sh
./run_perf_bench.sh \
  --benchmark_filter=BM_RootDecisionForest100k \
  --benchmark_repetitions=7
```

```bat
run_perf_bench.bat
run_perf_bench.bat --benchmark_filter=BM_RootDecisionForest100k --benchmark_repetitions=7
```

These scripts configure and verify a Release-only performance build. On Apple
Silicon the shell runner explicitly targets native arm64 even when launched
from a Rosetta terminal. Set `HLOD_PERF_BUILD_DIR` to choose another build
directory.

The executable can also be run directly after a normal build:

```sh
build/bench/hlod_bench --benchmark_list_tests
```

For a multi-config generator, use the Release subdirectory, for example
`build/bench/Release/hlod_bench` or
`build\bench\Release\hlod_bench.exe`.

## Machine and production-kernel probes

`run_machine_bench.sh` builds a separate `hlod_machine_bench` executable and
writes `machine_perf.json` by default:

```sh
./run_machine_bench.sh
```

Pass Google Benchmark arguments to replace the default run. Set
`HLOD_MACHINE_BUILD_DIR` to select another build directory. The synthetic
probes cover scalar dependency and throughput, 128-bit SIMD arithmetic and
compare-to-mask cost, square root/divide, sequential cache and memory
bandwidth, constant-stride hardware prefetch, dependent-load latency,
random-load parallelism, branch prediction, and sparse-mask iteration.

The diagnostic executable is separate so adding a probe cannot perturb the
code layout of `hlod_bench`. Architecture-neutral SIMD probes use one 128-bit
vector on both x86-64 and arm64. The `BM_Kernel*` group instead uses the active
production backend recorded as `hlod_kernel_backend` in the JSON context.

For only the focused production kernels with longer sampling:

```sh
./run_arch_bench.sh
```

This wrapper writes `arch_kernel_perf.json` and uses its own default build
directory.

## macOS hardware CPU counters

On Apple Silicon, `profile_macos_cpu.sh` records the cached hierarchical
benchmark with optimized source line tables and hardware counters:

```sh
./profile_macos_cpu.sh
```

The script discovers a full Xcode installation and selects the available
`CPU Bottlenecks` or `CPU Counters` instrument/template. It writes an
Instruments `.trace`, metadata, an exported table of contents, and a compact
`_summary.zip` under `profile_results/`. The summary contains exported
process/thread bottleneck metrics, counter samples, core placement, and time
profile hotspots. The raw trace remains local because it can be hundreds of
megabytes.

If recording succeeded but export or packaging failed, process an existing
trace without another capture:

```sh
./profile_macos_cpu.sh --process
./profile_macos_cpu.sh --process path/to/capture.trace
```

Useful overrides are:

| Variable | Purpose |
|---|---|
| `HLOD_PROFILE_BUILD_DIR` | profiling build directory |
| `HLOD_PROFILE_OUTPUT_DIR` | trace and summary output directory |
| `HLOD_PROFILE_FILTER` | Google Benchmark case to record |
| `HLOD_PROFILE_MIN_TIME` | benchmark minimum run time |
| `HLOD_PROFILE_TIME_LIMIT` | Instruments recording limit |
| `HLOD_PROFILE_EXPORT_START` | beginning of the focused export window |
| `HLOD_DEVELOPER_DIR` | exact Xcode developer directory |
| `HLOD_XCTRACE_TEMPLATE` | exact template name or `.tracetemplate` path |

Hardware-counter capture requires a full Xcode with `xctrace`; Command Line
Tools alone are insufficient. macOS may also require enabling the terminal
under **Privacy & Security > Developer Tools**. Older `xctrace export`
versions do not support a focused time-start option; in that case the exported
data includes process setup and should be filtered during analysis.

## Build options

| Option | Default | Meaning |
|---|---:|---|
| `HLOD_BUILD_TESTS` | `ON` | build the correctness suite |
| `HLOD_BUILD_BENCH` | `ON` | build the benchmark executables |
| `HLOD_AVX2` | `ON` | use AVX2/FMA on supported x86-64 targets |
| `HLOD_FORCE_SCALAR` | `OFF` | disable intrinsic implementations |
| `HLOD_PROFILE_SYMBOLS` | `OFF` | add optimized Clang line tables for profiling |

When comparing a scalar or fallback build, use a separate build directory so
cached CMake options and stale binaries cannot contaminate the result.
