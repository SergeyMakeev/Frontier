#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${FRONTIER_ALL_PERF_BUILD_DIR:-${ROOT_DIR}/build-perf-report}"
REPORT_ROOT="${FRONTIER_PERF_REPORT_ROOT:-${ROOT_DIR}/perf_reports}"
RNG_POLICY="xorshift32-explicit-seeds-v1"

if (( $# > 1 )); then
    echo "Usage: $0 [machine-label]" >&2
    exit 2
fi

if ! command -v cmake >/dev/null 2>&1; then
    echo "ERROR: CMake was not found in PATH." >&2
    exit 1
fi

host_system="$(uname -s)"
host_machine="$(uname -m)"
raw_label="${FRONTIER_PERF_LABEL:-${1:-${host_system}-${host_machine}}}"
label="$(printf '%s' "${raw_label}" | tr -cs '[:alnum:]. _-' '_' | tr ' ' '_')"
label="${label#_}"
label="${label%_}"
if [[ -z "${label}" ]]; then
    echo "ERROR: The machine label contains no usable characters." >&2
    exit 2
fi

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
start_epoch="$(date +%s)"
report_name="frontier-perf-${label}-${timestamp}"
report_dir="${REPORT_ROOT}/${report_name}"
mkdir -p "${report_dir}"

run_status="FAILED"
failure_stage="setup"
archive_path=""

write_report()
{
    local commit dirty elapsed_seconds
    commit="$(git -c "safe.directory=${ROOT_DIR}" -C "${ROOT_DIR}" rev-parse HEAD 2>/dev/null || printf unknown)"
    if [[ -n "$(git -c "safe.directory=${ROOT_DIR}" -C "${ROOT_DIR}" status --porcelain 2>/dev/null || true)" ]]; then
        dirty=yes
    else
        dirty=no
    fi
    elapsed_seconds=$(( $(date +%s) - start_epoch ))

    cat > "${report_dir}/REPORT.md" <<EOF
# Frontier performance report

- Format: frontier-perf-report-v1
- Status: ${run_status}
- Failed stage: ${failure_stage}
- Machine label: ${raw_label}
- Captured UTC: ${timestamp}
- Elapsed seconds: ${elapsed_seconds}
- Source commit: ${commit}
- Source dirty: ${dirty}
- Host OS: ${host_system}
- Host architecture: ${host_machine}
- CPU affinity: scheduler default
- Workload RNG: ${RNG_POLICY}
- Benchmark order: registration order
- Build: Release

## Result files

- \`real_world_perf.json\`: end-to-end subtree assembly workloads
- \`machine_perf.json\`: ALU, SIMD, branch, cache, latency, and bandwidth probes
- \`arch_kernel_perf.json\`: focused production-kernel probes with longer sampling
- \`tests.log\`: correctness-suite result
- \`hardware.txt\`: CPU, memory, topology, OS, and power information
- \`toolchain.txt\`: compiler, CMake, and build configuration
- \`source.txt\`: exact Git revision and working-tree state
- \`commands.txt\`: benchmark filters and sampling parameters

The benchmarks ran without an affinity mask, matching normal scheduler behavior.
Compare the JSON \`median\` aggregates first; use the accompanying coefficient
of variation to identify unstable cases. Keep the machine plugged in and idle
for the most useful cross-machine comparison.
EOF

    cat > "${report_dir}/manifest.txt" <<EOF
format=frontier-perf-report-v1
status=${run_status}
failure_stage=${failure_stage}
label=${raw_label}
timestamp_utc=${timestamp}
elapsed_seconds=${elapsed_seconds}
git_commit=${commit}
git_dirty=${dirty}
host_os=${host_system}
host_arch=${host_machine}
build_type=Release
affinity=scheduler-default
rng_policy=${RNG_POLICY}
benchmark_order=registration-order
EOF
}

package_report()
{
    set +e
    if command -v zip >/dev/null 2>&1; then
        archive_path="${report_dir}.zip"
        (cd "${REPORT_ROOT}" && zip -qr "$(basename "${archive_path}")" "${report_name}")
    else
        archive_path="${report_dir}.tar.gz"
        tar -C "${REPORT_ROOT}" -czf "${archive_path}" "${report_name}"
    fi
    set -e
}

finish()
{
    local exit_code=$?
    trap - EXIT
    if (( exit_code == 0 )); then
        run_status="COMPLETE"
        failure_stage="none"
    fi
    write_report
    package_report
    echo
    if [[ -n "${archive_path}" && -f "${archive_path}" ]]; then
        echo "Performance report: ${archive_path}"
    else
        echo "Performance report directory: ${report_dir}"
    fi
    if (( exit_code != 0 )); then
        echo "ERROR: Collection failed during '${failure_stage}'; partial data was retained." >&2
    fi
    exit "${exit_code}"
}
trap finish EXIT

{
    echo "Frontier cross-machine performance collection"
    echo "Label: ${raw_label}"
    echo "Started UTC: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "Host: ${host_system} ${host_machine}"
    echo
    uname -a
    echo
    uptime || true

    if [[ "${host_system}" == "Darwin" ]]; then
        echo
        echo "Apple hardware and software"
        system_profiler SPHardwareDataType SPSoftwareDataType 2>&1 || true
        echo
        echo "Selected sysctl values"
        for key in \
            hw.model hw.machine hw.ncpu hw.physicalcpu hw.logicalcpu hw.memsize \
            hw.cachelinesize hw.perflevel0.physicalcpu hw.perflevel1.physicalcpu \
            machdep.cpu.brand_string
        do
            sysctl "${key}" 2>/dev/null || true
        done
        echo
        echo "Power configuration"
        pmset -g custom 2>&1 || true
        pmset -g therm 2>&1 || true
    elif [[ "${host_system}" == "Linux" ]]; then
        echo
        echo "CPU and NUMA topology"
        lscpu 2>&1 || true
        if command -v numactl >/dev/null 2>&1; then
            numactl --hardware 2>&1 || true
        fi
        echo
        echo "Memory"
        free -h 2>&1 || true
        echo
        echo "CPU frequency governors"
        for governor in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
            [[ -r "${governor}" ]] && cat "${governor}"
        done | sort | uniq -c || true
        echo
        echo "SMT and boost"
        [[ -r /sys/devices/system/cpu/smt/active ]] && cat /sys/devices/system/cpu/smt/active
        [[ -r /sys/devices/system/cpu/cpufreq/boost ]] && cat /sys/devices/system/cpu/cpufreq/boost
        [[ -r /sys/devices/system/cpu/intel_pstate/no_turbo ]] && cat /sys/devices/system/cpu/intel_pstate/no_turbo
    fi
} > "${report_dir}/hardware.txt" 2>&1

{
    echo "Commit: $(git -c "safe.directory=${ROOT_DIR}" -C "${ROOT_DIR}" rev-parse HEAD 2>/dev/null || printf unknown)"
    echo "Branch: $(git -c "safe.directory=${ROOT_DIR}" -C "${ROOT_DIR}" branch --show-current 2>/dev/null || printf unknown)"
    echo
    echo "Working tree:"
    git -c "safe.directory=${ROOT_DIR}" -C "${ROOT_DIR}" status --short 2>&1 || true
    echo
    git -c "safe.directory=${ROOT_DIR}" -C "${ROOT_DIR}" diff --stat 2>&1 || true
} > "${report_dir}/source.txt"

cat > "${report_dir}/commands.txt" <<EOF
Correctness:
  ctest --test-dir <build> -C Release --output-on-failure

Real world:
  frontier_bench
    --benchmark_filter=BM_SubtreeAssembly
    --benchmark_repetitions=5
    --benchmark_report_aggregates_only=true

Machine characterization:
  frontier_machine_bench
    --benchmark_min_time=0.15s
    --benchmark_repetitions=5
    --benchmark_report_aggregates_only=true

Focused architecture kernels:
  frontier_machine_bench
    --benchmark_filter=BM_(Kernel(WideAabb|DistanceError|CacheHit)|OutputAppend)
    --benchmark_min_time=0.75s
    --benchmark_repetitions=11
    --benchmark_report_aggregates_only=true

Random workloads:
  ${RNG_POLICY}
  Float mapping and shuffle are repository-owned and standard-library independent.
EOF

avx2=OFF
target_architecture=""
if [[ "${host_system}" == "Darwin" ]]; then
    apple_silicon="$(sysctl -n hw.optional.arm64 2>/dev/null || true)"
    if [[ "${host_machine}" == "arm64" || "${apple_silicon}" == "1" ]]; then
        target_architecture=arm64
        echo "Targeting native Apple Silicon arm64/NEON."
    elif [[ "${host_machine}" == "x86_64" ]]; then
        intel_features="$(sysctl -n machdep.cpu.leaf7_features 2>/dev/null || true)"
        if [[ " ${intel_features} " == *" AVX2 "* ]]; then
            avx2=ON
        fi
    fi
elif [[ "${host_machine}" == "x86_64" ]]; then
    avx2=ON
fi

configure_args=(
    -S "${ROOT_DIR}"
    -B "${BUILD_DIR}"
    -DCMAKE_BUILD_TYPE=Release
    -DFRONTIER_BUILD_TESTS=ON
    -DFRONTIER_BUILD_BENCH=ON
    -DFRONTIER_AVX2="${avx2}"
    -DFRONTIER_FORCE_SCALAR=OFF
)
if [[ ! -f "${BUILD_DIR}/CMakeCache.txt" ]] && command -v ninja >/dev/null 2>&1; then
    configure_args+=(-G Ninja)
fi
if [[ -n "${target_architecture}" ]]; then
    configure_args+=("-DCMAKE_OSX_ARCHITECTURES=${target_architecture}")
fi

failure_stage=configure
echo "Configuring Release build..."
cmake "${configure_args[@]}" 2>&1 | tee "${report_dir}/configure.log"

failure_stage=build
echo "Building tests and performance executables..."
cmake --build "${BUILD_DIR}" --config Release --parallel \
    --target frontier_tests frontier_bench frontier_machine_bench 2>&1 |
    tee "${report_dir}/build.log"

cache_file="${BUILD_DIR}/CMakeCache.txt"
if grep -Eq '^CMAKE_CONFIGURATION_TYPES:[^=]*=.+$' "${cache_file}"; then
    binary_dir="${BUILD_DIR}/bench/Release"
else
    build_type="$(sed -n 's/^CMAKE_BUILD_TYPE:[^=]*=//p' "${cache_file}")"
    if [[ "${build_type}" != "Release" ]]; then
        echo "ERROR: Expected Release, got CMAKE_BUILD_TYPE='${build_type}'." >&2
        exit 1
    fi
    binary_dir="${BUILD_DIR}/bench"
fi

bench_exe="${binary_dir}/frontier_bench"
machine_exe="${binary_dir}/frontier_machine_bench"
if [[ ! -x "${bench_exe}" || ! -x "${machine_exe}" ]]; then
    echo "ERROR: Release benchmark executables were not found under ${binary_dir}." >&2
    exit 1
fi

{
    cmake --version
    echo
    "${CXX:-c++}" --version 2>&1 || true
    echo
    git --version 2>&1 || true
    echo
    grep -E '^(CMAKE_(BUILD_TYPE|CXX_COMPILER|CXX_COMPILER_ID|CXX_COMPILER_VERSION|GENERATOR|OSX_ARCHITECTURES)|FRONTIER_(AVX2|FORCE_SCALAR)):' \
        "${cache_file}" || true
    echo
    file "${bench_exe}" 2>&1 || true
} > "${report_dir}/toolchain.txt"

failure_stage=correctness-tests
echo "Running correctness tests..."
ctest --test-dir "${BUILD_DIR}" -C Release --output-on-failure 2>&1 |
    tee "${report_dir}/tests.log"

failure_stage=real-world-benchmarks
echo "Running real-world performance suite..."
"${bench_exe}" \
    '--benchmark_filter=BM_SubtreeAssembly' \
    --benchmark_repetitions=5 \
    --benchmark_report_aggregates_only=true \
    "--benchmark_out=${report_dir}/real_world_perf.json" \
    --benchmark_out_format=json 2>&1 | tee "${report_dir}/real_world_perf.log"

failure_stage=machine-benchmarks
echo "Running machine characterization suite..."
"${machine_exe}" \
    --benchmark_min_time=0.15s \
    --benchmark_repetitions=5 \
    --benchmark_report_aggregates_only=true \
    "--benchmark_out=${report_dir}/machine_perf.json" \
    --benchmark_out_format=json 2>&1 | tee "${report_dir}/machine_perf.log"

failure_stage=architecture-benchmarks
echo "Running focused production-kernel suite..."
"${machine_exe}" \
    '--benchmark_filter=BM_(Kernel(WideAabb|DistanceError|CacheHit)|OutputAppend)' \
    --benchmark_min_time=0.75s \
    --benchmark_repetitions=11 \
    --benchmark_report_aggregates_only=true \
    "--benchmark_out=${report_dir}/arch_kernel_perf.json" \
    --benchmark_out_format=json 2>&1 | tee "${report_dir}/arch_kernel_perf.log"

failure_stage=validate-results
for result in real_world_perf.json machine_perf.json arch_kernel_perf.json; do
    if [[ ! -s "${report_dir}/${result}" ]] ||
       ! grep -q '"name"' "${report_dir}/${result}"; then
        echo "ERROR: ${result} is missing or contains no benchmark records." >&2
        exit 1
    fi
done

failure_stage=none
