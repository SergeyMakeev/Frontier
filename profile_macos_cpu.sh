#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${HLOD_PROFILE_BUILD_DIR:-${ROOT_DIR}/build-macos-profile}"
OUTPUT_DIR="${HLOD_PROFILE_OUTPUT_DIR:-${ROOT_DIR}/profile_results}"
BENCHMARK_FILTER="${HLOD_PROFILE_FILTER:-BM_MixedForest100k/flat_pct:0/cached:1$}"
BENCHMARK_MIN_TIME="${HLOD_PROFILE_MIN_TIME:-18s}"
TRACE_TIME_LIMIT="${HLOD_PROFILE_TIME_LIMIT:-25s}"

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
    add_developer_dir "$(xcode-select -p 2>/dev/null || true)"
    for xcode_app in /Applications/Xcode*.app; do
        add_developer_dir "${xcode_app}/Contents/Developer"
    done
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
if [[ -n "${HLOD_XCTRACE_TEMPLATE:-}" && -f "${HLOD_XCTRACE_TEMPLATE}" ]]; then
    selected_developer_dir="${developer_dirs[0]}"
    selected_template="${HLOD_XCTRACE_TEMPLATE}"
else
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
fi

if [[ -z "${selected_template}" ]]; then
    echo "ERROR: No hardware CPU-counter template is exposed to xctrace." >&2
    echo "Searched for: ${template_candidates[*]}" >&2
    echo "Developer directories checked:" >&2
    printf '  %s\n' "${developer_dirs[@]}" >&2
    echo "Available templates from the selected installation:" >&2
    DEVELOPER_DIR="${developer_dirs[0]}" xcrun xctrace list templates >&2 || true
    echo "Install/select full Xcode 26+ or set HLOD_DEVELOPER_DIR/HLOD_XCTRACE_TEMPLATE." >&2
    exit 1
fi

export DEVELOPER_DIR="${selected_developer_dir}"
echo "Using developer directory: ${DEVELOPER_DIR}"
echo "Using Instruments template: ${selected_template}"
xcodebuild -version

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
toc="${OUTPUT_DIR}/hlod_cpu_counters_${stamp}_toc.xml"
info="${OUTPUT_DIR}/hlod_cpu_counters_${stamp}_info.txt"
archive="${trace}.zip"

{
    echo "developer_dir=${DEVELOPER_DIR}"
    echo "template=${selected_template}"
    echo "benchmark=${bench_exe}"
    echo "benchmark_filter=${BENCHMARK_FILTER}"
    echo "benchmark_min_time=${BENCHMARK_MIN_TIME}"
    echo "trace_time_limit=${TRACE_TIME_LIMIT}"
    xcodebuild -version
    uname -a
    sysctl -n machdep.cpu.brand_string 2>/dev/null || true
} >"${info}"

echo "Recording hardware CPU counters for the cached hierarchical workload..."
xcrun xctrace record \
    --template "${selected_template}" \
    --output "${trace}" \
    --time-limit "${TRACE_TIME_LIMIT}" \
    --no-prompt \
    --target-stdout - \
    --launch -- "${bench_exe}" \
    "--benchmark_filter=${BENCHMARK_FILTER}" \
    "--benchmark_min_time=${BENCHMARK_MIN_TIME}" \
    --benchmark_repetitions=1

echo "Exporting the trace table of contents..."
xcrun xctrace export "${trace}" --toc --output "${toc}"

(
    cd "${OUTPUT_DIR}"
    /usr/bin/zip -qry \
        "$(basename "${archive}")" \
        "$(basename "${trace}")" \
        "$(basename "${toc}")" \
        "$(basename "${info}")"
)

echo "Wrote ${trace}"
echo "Wrote ${toc}"
echo "Attach ${archive} to the Codex task for analysis."
echo "Open it with: open '${trace}'"
