#!/usr/bin/env bash
set -euo pipefail

if (( $# < 3 || $# > 4 )); then
    echo "Usage: $0 BASELINE_BUILD CANDIDATE_BUILD REPORT_ROOT [CYCLES]" >&2
    exit 2
fi

baseline_build="$(realpath "$1")"
candidate_build="$(realpath "$2")"
report_root="$(realpath -m "$3")"
cycles="${4:-4}"
cpu="${FRONTIER_PAIRED_CPU:-4}"
warmup="${FRONTIER_PAIRED_WARMUP_SECONDS:-0.25}"
max_temp="${FRONTIER_PAIRED_MAX_TEMP_MILLIC:-50000}"
cooldown_limit="${FRONTIER_PAIRED_COOLDOWN_SECONDS:-300}"
selected_cases="${FRONTIER_PAIRED_CASES:-}"

if [[ ! "${cycles}" =~ ^[1-9][0-9]*$ ]]; then
    echo "ERROR: CYCLES must be a positive integer." >&2
    exit 2
fi
if [[ ! "${cpu}" =~ ^[0-9]+$ ]] || ! taskset -c "${cpu}" true; then
    echo "ERROR: CPU ${cpu} is not available to this process." >&2
    exit 2
fi

declare -A executable
executable[baseline,64]="${baseline_build}/bench/frontier_bench"
executable[baseline,32]="${baseline_build}/bench/frontier_bench_payload32"
executable[candidate,64]="${candidate_build}/bench/frontier_bench"
executable[candidate,32]="${candidate_build}/bench/frontier_bench_payload32"
executable[baseline,submission64]="${baseline_build}/bench/frontier_submission_bench"
executable[baseline,submission32]="${baseline_build}/bench/frontier_submission_bench_payload32"
executable[candidate,submission64]="${candidate_build}/bench/frontier_submission_bench"
executable[candidate,submission32]="${candidate_build}/bench/frontier_submission_bench_payload32"
executable[baseline,machine]="${baseline_build}/bench/frontier_machine_bench"
executable[candidate,machine]="${candidate_build}/bench/frontier_machine_bench"
for path in "${executable[@]}"; do
    if [[ ! -x "${path}" ]]; then
        echo "ERROR: benchmark executable is missing: ${path}" >&2
        exit 1
    fi
done

temperature()
{
    local path="/sys/class/thermal/thermal_zone1/temp"
    if [[ -r "${path}" ]]; then
        cat "${path}"
    else
        echo 0
    fi
}

frequency()
{
    local path="/sys/devices/system/cpu/cpu${cpu}/cpufreq/scaling_cur_freq"
    if [[ -r "${path}" ]]; then
        cat "${path}"
    else
        echo 0
    fi
}

wait_for_temperature()
{
    local elapsed=0 current
    while true; do
        current="$(temperature)"
        if (( current <= max_temp )); then
            return
        fi
        if (( elapsed >= cooldown_limit )); then
            echo "ERROR: CPU temperature stayed above ${max_temp} mC." >&2
            exit 1
        fi
        sleep 2
        elapsed=$((elapsed + 2))
    done
}

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
report_dir="${report_root}/frontier-paired-${timestamp}"
mkdir -p "${report_dir}/raw"

cmake_source_dir()
{
    sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$1/CMakeCache.txt"
}

file_hash()
{
    sha256sum "$1" | awk '{print $1}'
}

baseline_source="$(cmake_source_dir "${baseline_build}")"
candidate_source="$(cmake_source_dir "${candidate_build}")"
baseline_commit="$(git -C "${baseline_source}" rev-parse HEAD)"
candidate_commit="$(git -C "${candidate_source}" rev-parse HEAD)"
git -C "${baseline_source}" status --short > "${report_dir}/baseline_status.txt"
git -C "${candidate_source}" status --short > "${report_dir}/candidate_status.txt"
git -C "${baseline_source}" diff --binary > "${report_dir}/baseline.patch"
git -C "${candidate_source}" diff --binary > "${report_dir}/candidate.patch"
cp "$0" "${report_dir}/runner.sh"

cat > "${report_dir}/manifest.txt" <<EOF
format=frontier-paired-perf-v1
timestamp_utc=${timestamp}
cpu=${cpu}
cycles=${cycles}
schedule=baseline,candidate,candidate,baseline
warmup_seconds=${warmup}
max_temperature_millic=${max_temp}
selected_cases=${selected_cases:-all}
baseline_build=${baseline_build}
candidate_build=${candidate_build}
baseline_source=${baseline_source}
candidate_source=${candidate_source}
baseline_commit=${baseline_commit}
candidate_commit=${candidate_commit}
baseline_payload32_sha256=$(file_hash "${executable[baseline,32]}")
baseline_payload64_sha256=$(file_hash "${executable[baseline,64]}")
baseline_machine_sha256=$(file_hash "${executable[baseline,machine]}")
candidate_payload32_sha256=$(file_hash "${executable[candidate,32]}")
candidate_payload64_sha256=$(file_hash "${executable[candidate,64]}")
baseline_submission32_sha256=$(file_hash "${executable[baseline,submission32]}")
baseline_submission64_sha256=$(file_hash "${executable[baseline,submission64]}")
candidate_submission32_sha256=$(file_hash "${executable[candidate,submission32]}")
candidate_submission64_sha256=$(file_hash "${executable[candidate,submission64]}")
candidate_machine_sha256=$(file_hash "${executable[candidate,machine]}")
compiler=$(c++ --version | head -n 1)
kernel=$(uname -srvmo)
governor=$(cat "/sys/devices/system/cpu/cpu${cpu}/cpufreq/scaling_governor" 2>/dev/null || echo unavailable)
EOF

echo "case,payload,cycle,slot,revision,temperature_before,temperature_after,frequency_before,frequency_after,json" \
    > "${report_dir}/samples.csv"

labels=(
    live_city live_city_render motion identity_50 identity_100
    control_integer control_branch control_distance control_memory
)
filters=(
    'BM_LiveCityDrivingFrame/iterations:8192'
    'BM_LiveCityRenderSubmissionFrame/iterations:8192'
    'BM_LiveCityMotionFrame/iterations:8192'
    'BM_InstanceForestSelectionScale/instances:10000/hierarchical_percent:50/reuse_mode:0'
    'BM_InstanceForestSelectionScale/instances:10000/hierarchical_percent:100/reuse_mode:0'
    'BM_IntegerDependency'
    'BM_BranchDispatch/pattern:2'
    'BM_KernelDistanceErrorCurrent'
    'BM_SequentialRead/bytes:2097152'
)
minimum_times=(0.5s 0.5s 0.5s 0.5s 0.5s 0.5s 0.5s 0.5s 0.5s)
payload_sets=("32 64" "32 64" "32 64" "32 64" "32 64" machine machine machine machine)
binary_kinds=(frontier submission frontier frontier frontier machine machine machine machine)
schedule=(baseline candidate candidate baseline)

if [[ -n "${selected_cases}" ]]; then
    IFS=, read -r -a requested_cases <<< "${selected_cases}"
    for requested in "${requested_cases[@]}"; do
        found=false
        for label in "${labels[@]}"; do
            if [[ "${requested}" == "${label}" ]]; then
                found=true
                break
            fi
        done
        if [[ "${found}" != true ]]; then
            echo "ERROR: unknown FRONTIER_PAIRED_CASES label: ${requested}" >&2
            exit 2
        fi
    done
fi

for index in "${!labels[@]}"; do
    label="${labels[index]}"
    if [[ -n "${selected_cases}" && ",${selected_cases}," != *",${label},"* ]]; then
        continue
    fi
    filter="${filters[index]}"
    minimum_time="${minimum_times[index]}"
    binary_kind="${binary_kinds[index]}"
    for payload in ${payload_sets[index]}; do
        for ((cycle = 1; cycle <= cycles; ++cycle)); do
            for slot in "${!schedule[@]}"; do
                revision="${schedule[slot]}"
                sample="${label}-p${payload}-c${cycle}-s$((slot + 1))-${revision}"
                json="${report_dir}/raw/${sample}.json"
                log="${report_dir}/raw/${sample}.log"
                wait_for_temperature
                before_temp="$(temperature)"
                before_freq="$(frequency)"
                echo "${sample}: temp=${before_temp}mC freq=${before_freq}kHz"
                executable_key="${payload}"
                if [[ "${binary_kind}" == submission ]]; then
                    executable_key="submission${payload}"
                elif [[ "${binary_kind}" == machine ]]; then
                    executable_key=machine
                fi
                taskset -c "${cpu}" "${executable[${revision},${executable_key}]}" \
                    "--benchmark_filter=^${filter}$" \
                    "--benchmark_min_time=${minimum_time}" \
                    "--benchmark_min_warmup_time=${warmup}" \
                    --benchmark_repetitions=1 \
                    "--benchmark_out=${json}" \
                    --benchmark_out_format=json > "${log}" 2>&1
                after_temp="$(temperature)"
                after_freq="$(frequency)"
                printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
                    "${label}" "${payload}" "${cycle}" "$((slot + 1))" \
                    "${revision}" "${before_temp}" "${after_temp}" \
                    "${before_freq}" "${after_freq}" "${json}" \
                    >> "${report_dir}/samples.csv"
            done
        done
    done
done

echo "Paired report: ${report_dir}"
