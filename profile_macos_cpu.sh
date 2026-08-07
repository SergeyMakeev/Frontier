#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${HLOD_PROFILE_BUILD_DIR:-${ROOT_DIR}/build-macos-profile}"
OUTPUT_DIR="${HLOD_PROFILE_OUTPUT_DIR:-${ROOT_DIR}/profile_results}"
BENCHMARK_FILTER="${HLOD_PROFILE_FILTER:-BM_MixedForest100k/flat_pct:0/cached:1$}"
BENCHMARK_MIN_TIME="${HLOD_PROFILE_MIN_TIME:-18s}"
TRACE_TIME_LIMIT="${HLOD_PROFILE_TIME_LIMIT:-25s}"
EXPORT_TIME_START="${HLOD_PROFILE_EXPORT_START:-5s}"

if [[ "$(uname -s)" != "Darwin" ]]; then
    echo "ERROR: CPU Counters profiling is available only on macOS." >&2
    exit 1
fi
if ! command -v cmake >/dev/null 2>&1; then
    echo "ERROR: CMake was not found in PATH." >&2
    exit 1
fi
if ! command -v xcrun >/dev/null 2>&1; then
    echo "ERROR: Xcode command-line tools were not found." >&2
    exit 1
fi

developer_dirs=()
add_developer_dir() {
    local candidate="$1"
    [[ -d "${candidate}" ]] || return
    local existing
    if [[ ${#developer_dirs[@]} -gt 0 ]]; then
        for existing in "${developer_dirs[@]}"; do
            [[ "${existing}" == "${candidate}" ]] && return
        done
    fi
    developer_dirs+=("${candidate}")
}

if [[ -n "${HLOD_DEVELOPER_DIR:-}" ]]; then
    add_developer_dir "${HLOD_DEVELOPER_DIR}"
elif [[ -n "${DEVELOPER_DIR:-}" ]]; then
    add_developer_dir "${DEVELOPER_DIR}"
else
    # Full Xcode contains Instruments/xctrace; Command Line Tools usually does
    # not. Prefer installed Xcode apps before the active developer directory.
    for xcode_app in /Applications/Xcode*.app; do
        add_developer_dir "${xcode_app}/Contents/Developer"
    done
    add_developer_dir "$(xcode-select -p 2>/dev/null || true)"
fi

if [[ ${#developer_dirs[@]} -eq 0 ]]; then
    echo "ERROR: No Xcode developer directory was found." >&2
    exit 1
fi

# Xcode 26 renamed the guided counter template to CPU Bottlenecks. Older
# releases expose the same hardware-PMU workflow as CPU Counters.
if [[ -n "${HLOD_XCTRACE_TEMPLATE:-}" ]]; then
    template_candidates=("${HLOD_XCTRACE_TEMPLATE}")
else
    template_candidates=('CPU Bottlenecks' 'CPU Counters')
fi

selected_developer_dir=""
selected_template=""
selected_instrument=""
if [[ -n "${HLOD_XCTRACE_TEMPLATE:-}" && -f "${HLOD_XCTRACE_TEMPLATE}" ]]; then
    selected_developer_dir="${developer_dirs[0]}"
    selected_template="${HLOD_XCTRACE_TEMPLATE}"
else
    # Prefer a configured template because it carries Apple's recommended
    # counter mode and derived metrics.
    for developer_dir in "${developer_dirs[@]}"; do
        templates="$(DEVELOPER_DIR="${developer_dir}" xcrun xctrace list templates 2>/dev/null || true)"
        for template_candidate in "${template_candidates[@]}"; do
            if grep -Fq "${template_candidate}" <<<"${templates}"; then
                selected_developer_dir="${developer_dir}"
                selected_template="${template_candidate}"
                break 2
            fi
        done
    done

    # Recent xctrace versions can compose a recording from an Instrument even
    # when the corresponding GUI template is not published to the CLI.
    if [[ -z "${selected_template}" && -z "${HLOD_XCTRACE_TEMPLATE:-}" ]]; then
        for developer_dir in "${developer_dirs[@]}"; do
            record_help="$(DEVELOPER_DIR="${developer_dir}" xcrun xctrace help record 2>/dev/null || true)"
            grep -Fq -- '--instrument' <<<"${record_help}" || continue
            instruments="$(DEVELOPER_DIR="${developer_dir}" xcrun xctrace list instruments 2>/dev/null || true)"
            for instrument_candidate in 'CPU Counters' 'Counters'; do
                if grep -Fq "${instrument_candidate}" <<<"${instruments}"; then
                    selected_developer_dir="${developer_dir}"
                    selected_instrument="${instrument_candidate}"
                    break 2
                fi
            done
        done
    fi
fi

if [[ -z "${selected_template}" && -z "${selected_instrument}" ]]; then
    echo "ERROR: No xctrace hardware CPU-counter configuration was found." >&2
    echo "Searched for templates: ${template_candidates[*]}" >&2
    echo "Diagnostics by developer directory:" >&2
    for developer_dir in "${developer_dirs[@]}"; do
        echo "--- ${developer_dir}" >&2
        if xctrace_path="$(DEVELOPER_DIR="${developer_dir}" xcrun --find xctrace 2>&1)"; then
            echo "xctrace: ${xctrace_path}" >&2
            echo "Templates:" >&2
            DEVELOPER_DIR="${developer_dir}" xcrun xctrace list templates >&2 || true
            echo "CPU-related instruments:" >&2
            DEVELOPER_DIR="${developer_dir}" xcrun xctrace list instruments 2>&1 \
                | grep -Ei 'CPU|Counter' >&2 || true
        else
            echo "${xctrace_path}" >&2
        fi
    done
    echo "If full Xcode was just installed, open it once to finish setup, or run:" >&2
    echo "  sudo env DEVELOPER_DIR=/Applications/Xcode.app/Contents/Developer xcodebuild -runFirstLaunch" >&2
    echo "Then rerun this script. HLOD_DEVELOPER_DIR and HLOD_XCTRACE_TEMPLATE can override discovery." >&2
    exit 1
fi

export DEVELOPER_DIR="${selected_developer_dir}"
echo "Using developer directory: ${DEVELOPER_DIR}"
if [[ -n "${selected_template}" ]]; then
    echo "Using Instruments template: ${selected_template}"
else
    echo "Using Instruments directly: ${selected_instrument}"
fi
xcodebuild -version

export_trace_summary() {
    local source_trace="$1"
    local source_base="${source_trace%.trace}"
    local source_dir
    local source_name
    local toc_file="${source_base}_toc.xml"
    local info_file="${source_base}_info.txt"
    local summary_archive="${source_base}_summary.zip"

    if [[ ! -e "${source_trace}" ]]; then
        echo "ERROR: Trace was not found: ${source_trace}" >&2
        return 1
    fi

    echo "Exporting the trace table of contents..."
    xcrun xctrace export --input "${source_trace}" --toc --output "${toc_file}"

    # Keep the summary focused on the target process: the guided bottleneck
    # breakdown, thread/core placement, PMU samples, and symbolized hotspots.
    local measurements_file="${source_base}_measurements.xml"
    local measurements_query
    measurements_query='/trace-toc/run[@number="1"]/data/table['
    measurements_query+='@schema="CounterMetricAggregatedForProcess" or '
    measurements_query+='@schema="CounterMetricByThread" or '
    measurements_query+='@schema="CountingModeSamples" or '
    measurements_query+='@schema="MetricTableForThread" or '
    measurements_query+='@schema="RemarksByThread" or '
    measurements_query+='@schema="CoreTypeByThread" or '
    measurements_query+='@schema="time-profile"]'
    echo "Exporting focused CPU-counter measurements..."
    export_focused_measurements() {
        local export_help
        export_help="$(xcrun xctrace help export 2>/dev/null || true)"
        if grep -Fq -- '--time-start' <<<"${export_help}"; then
            xcrun xctrace export --input "${source_trace}" \
                --time-start "${EXPORT_TIME_START}" \
                --xpath "${measurements_query}" --output "${measurements_file}"
        else
            echo "This xctrace cannot filter exports by time; exporting the full measurement interval."
            xcrun xctrace export --input "${source_trace}" \
                --xpath "${measurements_query}" --output "${measurements_file}"
        fi
    }
    if ! export_focused_measurements; then
        echo "WARNING: xctrace could not export focused measurements; the TOC will still be archived." >&2
        /bin/rm -f "${measurements_file}"
    fi

    source_dir="$(cd "$(dirname "${source_trace}")" && pwd)"
    source_name="$(basename "${source_base}")"
    (
        cd "${source_dir}"
        /bin/rm -f "${source_name}_summary.zip"
        if [[ -f "$(basename "${info_file}")" ]]; then
            /usr/bin/zip -q "${source_name}_summary.zip" \
                "${source_name}"_*.xml "${source_name}_info.txt"
        else
            /usr/bin/zip -q "${source_name}_summary.zip" \
                "${source_name}"_*.xml
        fi
    )

    echo "Wrote ${toc_file}"
    echo "Attach ${summary_archive} to the Codex task for analysis."
}

if [[ "${1:-}" == "--process" ]]; then
    trace_to_process="${2:-}"
    if [[ -z "${trace_to_process}" ]]; then
        for trace_candidate in "${OUTPUT_DIR}"/*.trace; do
            [[ -e "${trace_candidate}" ]] || continue
            if [[ -z "${trace_to_process}" || "${trace_candidate}" -nt "${trace_to_process}" ]]; then
                trace_to_process="${trace_candidate}"
            fi
        done
    fi
    if [[ -z "${trace_to_process}" ]]; then
        echo "ERROR: No trace was provided and none exists under ${OUTPUT_DIR}." >&2
        exit 1
    fi
    export_trace_summary "${trace_to_process}"
    exit 0
fi

echo "Building optimized arm64 benchmark with source line tables..."
configure_benchmark() {
    cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" "$@" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_OSX_ARCHITECTURES=arm64 \
        -DHLOD_BUILD_TESTS=OFF \
        -DHLOD_BUILD_BENCH=ON \
        -DHLOD_AVX2=OFF \
        -DHLOD_FORCE_SCALAR=OFF \
        -DHLOD_PROFILE_SYMBOLS=ON
}
if [[ ! -f "${BUILD_DIR}/CMakeCache.txt" ]] && command -v ninja >/dev/null 2>&1; then
    configure_benchmark -G Ninja
else
    configure_benchmark
fi
cmake --build "${BUILD_DIR}" --config Release --target hlod_bench --parallel

cache_file="${BUILD_DIR}/CMakeCache.txt"
if grep -Eq '^CMAKE_CONFIGURATION_TYPES:[^=]*=.+$' "${cache_file}"; then
    bench_exe="${BUILD_DIR}/bench/Release/hlod_bench"
else
    bench_exe="${BUILD_DIR}/bench/hlod_bench"
fi
if [[ ! -x "${bench_exe}" ]]; then
    echo "ERROR: Benchmark executable was not found at ${bench_exe}." >&2
    exit 1
fi

stamp="$(date +%Y%m%d-%H%M%S)"
mkdir -p "${OUTPUT_DIR}"
trace="${OUTPUT_DIR}/hlod_cpu_counters_${stamp}.trace"
info="${OUTPUT_DIR}/hlod_cpu_counters_${stamp}_info.txt"

{
    echo "developer_dir=${DEVELOPER_DIR}"
    echo "template=${selected_template:-none}"
    echo "instrument=${selected_instrument:-none}"
    echo "benchmark=${bench_exe}"
    echo "benchmark_filter=${BENCHMARK_FILTER}"
    echo "benchmark_min_time=${BENCHMARK_MIN_TIME}"
    echo "trace_time_limit=${TRACE_TIME_LIMIT}"
    echo "export_time_start=${EXPORT_TIME_START}"
    xcodebuild -version
    uname -a
    sysctl -n machdep.cpu.brand_string 2>/dev/null || true
} >"${info}"

echo "Recording hardware CPU counters for the cached hierarchical workload..."
record_cpu_counters() {
    xcrun xctrace record "$@" \
        --output "${trace}" \
        --time-limit "${TRACE_TIME_LIMIT}" \
        --no-prompt \
        --target-stdout - \
        --launch -- "${bench_exe}" \
        "--benchmark_filter=${BENCHMARK_FILTER}" \
        "--benchmark_min_time=${BENCHMARK_MIN_TIME}" \
        --benchmark_repetitions=1
}
if [[ -n "${selected_template}" ]]; then
    record_status=0
    record_cpu_counters --template "${selected_template}" || record_status=$?
else
    record_status=0
    record_cpu_counters --instrument "${selected_instrument}" || record_status=$?
fi

if [[ ! -e "${trace}" ]]; then
    echo "ERROR: xctrace exited with status ${record_status} and did not produce ${trace}." >&2
    [[ ${record_status} -ne 0 ]] && exit "${record_status}"
    exit 1
fi
if [[ ${record_status} -ne 0 ]]; then
    echo "WARNING: xctrace exited with status ${record_status}, but produced a trace; continuing with export." >&2
fi

echo "Wrote ${trace}"
export_trace_summary "${trace}"
echo "Open it with: open '${trace}'"
