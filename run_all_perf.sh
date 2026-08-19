#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${FRONTIER_ALL_PERF_BUILD_DIR:-${ROOT_DIR}/build-perf-report}"
TEST_BUILD_DIR="${FRONTIER_ALL_TEST_BUILD_DIR:-${BUILD_DIR}-unit-debug}"
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

benchmark_warmup_seconds="${FRONTIER_PERF_WARMUP_SECONDS:-0.25}"
if [[ ! "${benchmark_warmup_seconds}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "ERROR: FRONTIER_PERF_WARMUP_SECONDS must be a non-negative number." >&2
    exit 2
fi

# Linux reports default to one deterministic, allowed CPU. On heterogeneous
# systems, prefer capacity first and advertised maximum frequency second. An
# explicit FRONTIER_PERF_CPU=N overrides that choice; `none` preserves normal
# scheduler placement. We do not change a machine-wide governor automatically.
benchmark_prefix=()
affinity_description="scheduler default"
affinity_manifest="scheduler-default"
affinity_command_display="none"
selected_cpu="none"
selected_governor="not-applicable"
selected_cpu_capacity="unknown"
selected_cpu_max_khz="unknown"
if [[ "${host_system}" == "Linux" ]]; then
    cpu_request="${FRONTIER_PERF_CPU:-auto}"
    if [[ "${cpu_request}" != "none" && "${cpu_request}" != "off" ]]; then
        if ! command -v taskset >/dev/null 2>&1; then
            if [[ "${cpu_request}" == "auto" ]]; then
                echo "WARNING: taskset is unavailable; benchmarks will not be pinned." >&2
            else
                echo "ERROR: FRONTIER_PERF_CPU requires taskset." >&2
                exit 2
            fi
        elif [[ "${cpu_request}" == "auto" ]]; then
            best_capacity=-1
            best_max_khz=-1
            for cpu_dir in /sys/devices/system/cpu/cpu[0-9]*; do
                [[ -d "${cpu_dir}" ]] || continue
                cpu="${cpu_dir##*cpu}"
                [[ "${cpu}" =~ ^[0-9]+$ ]] || continue
                if [[ -r "${cpu_dir}/online" && "$(cat "${cpu_dir}/online")" == "0" ]]; then
                    continue
                fi
                taskset -c "${cpu}" true >/dev/null 2>&1 || continue
                capacity=0
                max_khz=0
                if [[ -r "${cpu_dir}/cpu_capacity" ]]; then
                    capacity="$(cat "${cpu_dir}/cpu_capacity")"
                fi
                if [[ -r "${cpu_dir}/cpufreq/cpuinfo_max_freq" ]]; then
                    max_khz="$(cat "${cpu_dir}/cpufreq/cpuinfo_max_freq")"
                fi
                [[ "${capacity}" =~ ^[0-9]+$ ]] || capacity=0
                [[ "${max_khz}" =~ ^[0-9]+$ ]] || max_khz=0
                if (( capacity > best_capacity ||
                      (capacity == best_capacity && max_khz > best_max_khz) )); then
                    selected_cpu="${cpu}"
                    best_capacity="${capacity}"
                    best_max_khz="${max_khz}"
                fi
            done
            if [[ "${selected_cpu}" == "none" ]]; then
                echo "WARNING: no allowed online CPU was discovered; benchmarks will not be pinned." >&2
            fi
        else
            if [[ ! "${cpu_request}" =~ ^[0-9]+$ ]] ||
               ! taskset -c "${cpu_request}" true >/dev/null 2>&1; then
                echo "ERROR: FRONTIER_PERF_CPU='${cpu_request}' is not an allowed online CPU." >&2
                exit 2
            fi
            selected_cpu="${cpu_request}"
        fi

        if [[ "${selected_cpu}" != "none" ]]; then
            cpu_dir="/sys/devices/system/cpu/cpu${selected_cpu}"
            benchmark_prefix=(taskset -c "${selected_cpu}")
            affinity_description="logical CPU ${selected_cpu}"
            affinity_manifest="cpu-${selected_cpu}"
            affinity_command_display="taskset -c ${selected_cpu}"
            if [[ -r "${cpu_dir}/cpu_capacity" ]]; then
                selected_cpu_capacity="$(cat "${cpu_dir}/cpu_capacity")"
            fi
            if [[ -r "${cpu_dir}/cpufreq/cpuinfo_max_freq" ]]; then
                selected_cpu_max_khz="$(cat "${cpu_dir}/cpufreq/cpuinfo_max_freq")"
            fi
            if [[ -r "${cpu_dir}/cpufreq/scaling_governor" ]]; then
                selected_governor="$(cat "${cpu_dir}/cpufreq/scaling_governor")"
            fi
        fi
    fi
fi

frequency_control="not-applicable"
if [[ "${host_system}" == "Linux" ]]; then
    if [[ "${selected_cpu}" == "none" ]]; then
        frequency_control="uncontrolled-unpinned"
    elif [[ "${selected_governor}" == "performance" ]]; then
        frequency_control="performance-governor"
    elif [[ "${selected_governor}" == "unknown" ||
            "${selected_governor}" == "not-applicable" ]]; then
        frequency_control="pinned-warmup-governor-unknown"
    else
        frequency_control="pinned-warmup-${selected_governor}"
        echo "NOTICE: CPU ${selected_cpu} uses '${selected_governor}'; " \
             "the ${benchmark_warmup_seconds}s per-case warmup will precede measurements." >&2
    fi
fi

run_with_affinity()
{
    if (( ${#benchmark_prefix[@]} > 0 )); then
        "${benchmark_prefix[@]}" "$@"
    else
        "$@"
    fi
}

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

- Format: frontier-perf-report-v3
- Status: ${run_status}
- Failed stage: ${failure_stage}
- Machine label: ${raw_label}
- Captured UTC: ${timestamp}
- Elapsed seconds: ${elapsed_seconds}
- Source commit: ${commit}
- Source dirty: ${dirty}
- Host OS: ${host_system}
- Host architecture: ${host_machine}
- CPU affinity: ${affinity_description}
- Selected CPU capacity: ${selected_cpu_capacity}
- Selected CPU maximum frequency: ${selected_cpu_max_khz} kHz
- Selected CPU governor: ${selected_governor}
- Frequency control: ${frequency_control}
- Per-benchmark frequency warmup: ${benchmark_warmup_seconds} seconds
- Workload RNG: ${RNG_POLICY}
- Benchmark order: registration order
- End-to-end scope: complete registered frontier_bench suite
- Unit-test build: Debug; BVH4/BVH8 and 4-byte/8-byte payloads
- Performance builds: matched 4-byte and 8-byte payloads; Release with IPO;
  contract checks and subtree validation disabled

## Result files

- \`real_world_perf_payload64.json\`: end-to-end workloads with 8-byte payloads
- \`real_world_perf_payload32.json\`: end-to-end workloads with 4-byte payloads
- \`benchmark_inventory_payload64.txt\`: expected 8-byte end-to-end cases
- \`benchmark_inventory_payload32.txt\`: expected 4-byte end-to-end cases
- \`machine_perf.json\`: ALU, SIMD, branch, cache, latency, and bandwidth probes
- \`arch_kernel_perf.json\`: focused production-kernel probes with longer sampling
- \`tests.log\`: Debug BVH4/BVH8 and payload32/payload64 correctness result
- \`hardware.txt\`: CPU, memory, topology, OS, and power information
- \`toolchain.txt\`: compiler, CMake, and build configuration
- \`source.txt\`: exact Git revision and working-tree state
- \`commands.txt\`: benchmark filters and sampling parameters
- \`performance_state.txt\`: selected-core frequency, load, and thermal snapshots
- \`oriented_text_layout.txt\`: linked text bounds, addresses, and sizes for
  both mounted-root walkers and the cached selector when the platform exposes
  them

Performance processes used ${affinity_description}. Each benchmark receives a
${benchmark_warmup_seconds}-second untimed warmup so demand-based governors can
settle before sampling. Compare the JSON \`median\` aggregates first, inspect the
coefficient of variation, and check \`performance_state.txt\` for frequency or
thermal drift before treating a small delta as meaningful.
EOF

    cat > "${report_dir}/manifest.txt" <<EOF
format=frontier-perf-report-v3
status=${run_status}
failure_stage=${failure_stage}
label=${raw_label}
timestamp_utc=${timestamp}
elapsed_seconds=${elapsed_seconds}
git_commit=${commit}
git_dirty=${dirty}
host_os=${host_system}
host_arch=${host_machine}
unit_build_type=Debug
unit_payload_bytes=4,8
perf_build_type=Release
perf_payload_bytes=4,8
end_to_end_scope=complete
affinity=${affinity_manifest}
selected_cpu=${selected_cpu}
selected_cpu_capacity=${selected_cpu_capacity}
selected_cpu_max_khz=${selected_cpu_max_khz}
selected_cpu_governor=${selected_governor}
frequency_control=${frequency_control}
benchmark_warmup_seconds=${benchmark_warmup_seconds}
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

capture_performance_state()
{
    local phase="$1" cpu_dir thermal type temp
    {
        echo "phase=${phase}"
        echo "timestamp_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
        echo "load=$(uptime 2>/dev/null || true)"
        echo "affinity=${affinity_manifest}"
        echo "warmup_seconds=${benchmark_warmup_seconds}"
        if [[ "${host_system}" == "Linux" && "${selected_cpu}" != "none" ]]; then
            cpu_dir="/sys/devices/system/cpu/cpu${selected_cpu}"
            for field in scaling_governor scaling_cur_freq scaling_min_freq \
                         scaling_max_freq cpuinfo_min_freq cpuinfo_max_freq; do
                if [[ -r "${cpu_dir}/cpufreq/${field}" ]]; then
                    echo "cpu${selected_cpu}_${field}=$(cat "${cpu_dir}/cpufreq/${field}")"
                fi
            done
            for thermal in /sys/class/thermal/thermal_zone*; do
                [[ -d "${thermal}" ]] || continue
                type="unknown"
                temp="unknown"
                [[ -r "${thermal}/type" ]] && type="$(cat "${thermal}/type")"
                [[ -r "${thermal}/temp" ]] && temp="$(cat "${thermal}/temp")"
                echo "thermal_$(basename "${thermal}")=${type}:${temp}"
            done
        fi
        echo
    } >> "${report_dir}/performance_state.txt"
}

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
Execution control:
  affinity prefix: ${affinity_command_display}
  --benchmark_min_warmup_time=${benchmark_warmup_seconds}

Correctness:
  ctest --test-dir <unit-build> -C Debug --output-on-failure

Inventory:
  frontier_bench and frontier_bench_payload32 --benchmark_list_tests=true

Real world:
  frontier_bench (8-byte payload) and frontier_bench_payload32 (4-byte payload)
    complete registered benchmark suite (no benchmark filter)
    --benchmark_min_time=0.5s
    --benchmark_min_warmup_time=${benchmark_warmup_seconds}
    --benchmark_repetitions=5
    --benchmark_report_aggregates_only=true

Machine characterization:
  frontier_machine_bench
    --benchmark_min_time=0.15s
    --benchmark_min_warmup_time=${benchmark_warmup_seconds}
    --benchmark_repetitions=5
    --benchmark_report_aggregates_only=true

Focused architecture kernels:
  frontier_machine_bench
    --benchmark_filter=BM_(Kernel(WideAabb|DistanceError|CacheHit)|OutputAppend)
    --benchmark_min_time=0.75s
    --benchmark_min_warmup_time=${benchmark_warmup_seconds}
    --benchmark_repetitions=11
    --benchmark_report_aggregates_only=true

Random workloads:
  ${RNG_POLICY}
  Float mapping and shuffle are repository-owned and standard-library independent.
EOF

capture_performance_state collector-start

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

test_configure_args=(
    -S "${ROOT_DIR}"
    -B "${TEST_BUILD_DIR}"
    -DCMAKE_BUILD_TYPE=Debug
    -DFRONTIER_BUILD_TESTS=ON
    -DFRONTIER_TEST_PAYLOAD32=ON
    -DFRONTIER_BUILD_BENCH=OFF
    -DFRONTIER_AVX2="${avx2}"
    -DFRONTIER_BVH_WIDTH=AUTO
    -DFRONTIER_FORCE_SCALAR=OFF
    -DFRONTIER_STATS=OFF
    -DFRONTIER_CONTRACT_CHECKS=ON
    -DFRONTIER_VALIDATE_SUBTREES=ON
)
configure_args=(
    -S "${ROOT_DIR}"
    -B "${BUILD_DIR}"
    -DCMAKE_BUILD_TYPE=Release
    -DFRONTIER_IPO=ON
    -DFRONTIER_BUILD_TESTS=OFF
    -DFRONTIER_BUILD_BENCH=ON
    -DFRONTIER_AVX2="${avx2}"
    -DFRONTIER_BVH_WIDTH=AUTO
    -DFRONTIER_FORCE_SCALAR=OFF
    -DFRONTIER_STATS=OFF
    -DFRONTIER_CONTRACT_CHECKS=OFF
    -DFRONTIER_VALIDATE_SUBTREES=OFF
)
if [[ ! -f "${TEST_BUILD_DIR}/CMakeCache.txt" ]] && command -v ninja >/dev/null 2>&1; then
    test_configure_args+=(-G Ninja)
fi
if [[ ! -f "${BUILD_DIR}/CMakeCache.txt" ]] && command -v ninja >/dev/null 2>&1; then
    configure_args+=(-G Ninja)
fi
if [[ -n "${target_architecture}" ]]; then
    test_configure_args+=("-DCMAKE_OSX_ARCHITECTURES=${target_architecture}")
    configure_args+=("-DCMAKE_OSX_ARCHITECTURES=${target_architecture}")
fi

failure_stage=configure-unit-tests
echo "Configuring Debug unit tests (BVH4 + BVH8)..."
cmake "${test_configure_args[@]}" 2>&1 | tee "${report_dir}/configure-tests.log"

failure_stage=build-unit-tests
echo "Building Debug unit tests..."
cmake --build "${TEST_BUILD_DIR}" --config Debug --parallel \
    --target frontier_unit_tests 2>&1 | tee "${report_dir}/build-tests.log"

failure_stage=correctness-tests
echo "Running Debug correctness tests..."
ctest --test-dir "${TEST_BUILD_DIR}" -C Debug --output-on-failure 2>&1 |
    tee "${report_dir}/tests.log"

failure_stage=configure-performance
echo "Configuring unchecked Release performance build..."
cmake "${configure_args[@]}" 2>&1 | tee "${report_dir}/configure.log"

failure_stage=build-performance
echo "Building performance executables..."
cmake --build "${BUILD_DIR}" --config Release --parallel \
    --target frontier_bench frontier_bench_payload32 frontier_machine_bench 2>&1 |
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
bench32_exe="${binary_dir}/frontier_bench_payload32"
machine_exe="${binary_dir}/frontier_machine_bench"
if [[ ! -x "${bench_exe}" || ! -x "${bench32_exe}" || ! -x "${machine_exe}" ]]; then
    echo "ERROR: Release benchmark executables were not found under ${binary_dir}." >&2
    exit 1
fi

failure_stage=benchmark-inventory
"${bench_exe}" --benchmark_list_tests=true \
    > "${report_dir}/benchmark_inventory_payload64.txt" \
    2> "${report_dir}/benchmark_inventory_payload64.log"
"${bench32_exe}" --benchmark_list_tests=true \
    > "${report_dir}/benchmark_inventory_payload32.txt" \
    2> "${report_dir}/benchmark_inventory_payload32.log"

{
    echo "payload64=${bench_exe}"
    if command -v readelf >/dev/null 2>&1; then
        readelf -W -S "${bench_exe}" 2>&1 |
            grep -E '[[:space:]]\.text([.[:space:]]|$)' || true
    fi
    if command -v nm >/dev/null 2>&1; then
        nm -n -S -C "${bench_exe}" 2>&1 |
            grep -E 'SpatialDatabase::(selectFrontierCached|run(Oriented)?TlasRootInstance)' || true
    else
        echo "nm unavailable"
    fi
    echo
    echo "payload32=${bench32_exe}"
    if command -v readelf >/dev/null 2>&1; then
        readelf -W -S "${bench32_exe}" 2>&1 |
            grep -E '[[:space:]]\.text([.[:space:]]|$)' || true
    fi
    if command -v nm >/dev/null 2>&1; then
        nm -n -S -C "${bench32_exe}" 2>&1 |
            grep -E 'SpatialDatabase::(selectFrontierCached|run(Oriented)?TlasRootInstance)' || true
    else
        echo "nm unavailable"
    fi
} > "${report_dir}/oriented_text_layout.txt"

{
    cmake --version
    echo
    "${CXX:-c++}" --version 2>&1 || true
    echo
    git --version 2>&1 || true
    echo
    grep -E '^(CMAKE_(BUILD_TYPE|CXX_COMPILER|CXX_COMPILER_ID|CXX_COMPILER_VERSION|GENERATOR|OSX_ARCHITECTURES)|FRONTIER_(AVX2|BVH_WIDTH|CONTRACT_CHECKS|FORCE_SCALAR|IPO|SSE2_ONLY|STATS|VALIDATE_SUBTREES)):' \
        "${cache_file}" || true
    echo
    file "${bench_exe}" 2>&1 || true
} > "${report_dir}/toolchain.txt"

failure_stage=real-world-benchmarks-payload64
echo "Running real-world performance suite with 8-byte payloads..."
capture_performance_state before-payload64
run_with_affinity "${bench_exe}" \
    --benchmark_min_time=0.5s \
    "--benchmark_min_warmup_time=${benchmark_warmup_seconds}" \
    --benchmark_repetitions=5 \
    --benchmark_report_aggregates_only=true \
    "--benchmark_out=${report_dir}/real_world_perf_payload64.json" \
    --benchmark_out_format=json 2>&1 | tee "${report_dir}/real_world_perf_payload64.log"
capture_performance_state after-payload64

failure_stage=real-world-benchmarks-payload32
echo "Running real-world performance suite with 4-byte payloads..."
capture_performance_state before-payload32
run_with_affinity "${bench32_exe}" \
    --benchmark_min_time=0.5s \
    "--benchmark_min_warmup_time=${benchmark_warmup_seconds}" \
    --benchmark_repetitions=5 \
    --benchmark_report_aggregates_only=true \
    "--benchmark_out=${report_dir}/real_world_perf_payload32.json" \
    --benchmark_out_format=json 2>&1 | tee "${report_dir}/real_world_perf_payload32.log"
capture_performance_state after-payload32

failure_stage=machine-benchmarks
echo "Running machine characterization suite..."
capture_performance_state before-machine
run_with_affinity "${machine_exe}" \
    --benchmark_min_time=0.15s \
    "--benchmark_min_warmup_time=${benchmark_warmup_seconds}" \
    --benchmark_repetitions=5 \
    --benchmark_report_aggregates_only=true \
    "--benchmark_out=${report_dir}/machine_perf.json" \
    --benchmark_out_format=json 2>&1 | tee "${report_dir}/machine_perf.log"
capture_performance_state after-machine

failure_stage=architecture-benchmarks
echo "Running focused production-kernel suite..."
capture_performance_state before-architecture
run_with_affinity "${machine_exe}" \
    '--benchmark_filter=BM_(Kernel(WideAabb|DistanceError|CacheHit)|OutputAppend)' \
    --benchmark_min_time=0.75s \
    "--benchmark_min_warmup_time=${benchmark_warmup_seconds}" \
    --benchmark_repetitions=11 \
    --benchmark_report_aggregates_only=true \
    "--benchmark_out=${report_dir}/arch_kernel_perf.json" \
    --benchmark_out_format=json 2>&1 | tee "${report_dir}/arch_kernel_perf.log"
capture_performance_state after-architecture

failure_stage=validate-results
for result in real_world_perf_payload64.json real_world_perf_payload32.json \
              machine_perf.json arch_kernel_perf.json; do
    if [[ ! -s "${report_dir}/${result}" ]] ||
       ! grep -q '"name"' "${report_dir}/${result}"; then
        echo "ERROR: ${result} is missing or contains no benchmark records." >&2
        exit 1
    fi
done
if [[ ! -s "${report_dir}/performance_state.txt" ]]; then
    echo "ERROR: performance_state.txt is missing or empty." >&2
    exit 1
fi

required_families=(
    BM_SubtreeAssembly_
    BM_SharedNodeReadiness
    BM_MotionGroupSteady
    BM_MovingObjectsSelectionScale
    BM_MovingCameraSelectionScale
    BM_LiveCityDrivingFrame
    BM_LiveCityMotionFrame
    BM_FlatTlasSelectionScale
    BM_InstanceForestSelectionScale
    BM_FlatInstanceLifecycle
    BM_BoundsOverrideBatch
)
for result in real_world_perf_payload64.json real_world_perf_payload32.json; do
    for family in "${required_families[@]}"; do
        if ! grep -q "\"name\": \"${family}" "${report_dir}/${result}"; then
            echo "ERROR: ${result} is missing required family ${family}." >&2
            exit 1
        fi
    done
done

for payload in 64 32; do
    result="${report_dir}/real_world_perf_payload${payload}.json"
    inventory="${report_dir}/benchmark_inventory_payload${payload}.txt"
    while IFS= read -r benchmark; do
        [[ -z "${benchmark}" ]] && continue
        if ! grep -Fq "\"run_name\": \"${benchmark}\"" "${result}"; then
            echo "ERROR: payload${payload} result is missing listed case ${benchmark}." >&2
            exit 1
        fi
    done < "${inventory}"
done

failure_stage=none
