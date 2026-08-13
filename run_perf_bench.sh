#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${FRONTIER_PERF_BUILD_DIR:-${ROOT_DIR}/build-perf}"
payload_mode="${FRONTIER_PERF_PAYLOAD_BITS:-both}"
case "${payload_mode}" in
    both|32|64) ;;
    *)
        echo "ERROR: FRONTIER_PERF_PAYLOAD_BITS must be both, 32, or 64." >&2
        exit 2
        ;;
esac

if ! command -v cmake >/dev/null 2>&1; then
    echo "ERROR: CMake was not found in PATH." >&2
    echo "Install it with 'brew install cmake' or from https://cmake.org/." >&2
    exit 1
fi

# Prefer Ninja for a compact single-config build when it is available. An
# existing build directory keeps the generator recorded in its CMake cache.
generator=""
if [[ ! -f "${BUILD_DIR}/CMakeCache.txt" ]] && command -v ninja >/dev/null 2>&1; then
    generator="Ninja"
fi

# Prefer a native Apple Silicon build even when this script is launched from a
# terminal running under Rosetta. Intel Macs use AVX2 when the host advertises
# it, and otherwise retain the x86 SSE2 baseline.
avx2=OFF
target_architecture=""
host_system="$(uname -s)"
host_machine="$(uname -m)"
if [[ "${host_system}" == "Darwin" ]]; then
    apple_silicon="$(sysctl -n hw.optional.arm64 2>/dev/null || true)"
    if [[ "${host_machine}" == "arm64" || "${apple_silicon}" == "1" ]]; then
        target_architecture="arm64"
        echo "Targeting native Apple Silicon arm64/NEON."
    elif [[ "${host_machine}" == "x86_64" ]]; then
        intel_features="$(sysctl -n machdep.cpu.leaf7_features 2>/dev/null || true)"
        if [[ " ${intel_features} " == *" AVX2 "* ]]; then
            avx2=ON
        else
            echo "AVX2 was not reported by this Intel Mac; using the SSE2 path."
        fi
    fi
elif [[ "${host_machine}" == "x86_64" ]]; then
    avx2=ON
fi

echo "Configuring Release performance build..."
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
if [[ -n "${generator}" ]]; then
    configure_args+=(-G "${generator}")
fi
if [[ -n "${target_architecture}" ]]; then
    configure_args+=("-DCMAKE_OSX_ARCHITECTURES=${target_architecture}")
fi
cmake "${configure_args[@]}"

build_targets=()
if [[ "${payload_mode}" != 32 ]]; then
    build_targets+=(frontier_bench)
fi
if [[ "${payload_mode}" != 64 ]]; then
    build_targets+=(frontier_bench_payload32)
fi
echo "Building payload ${payload_mode} performance benchmark(s)..."
cmake --build "${BUILD_DIR}" --config Release \
    --target "${build_targets[@]}" --parallel

# Multi-config generators (notably Xcode) put each configuration in its own
# directory. Never fall back from Release/ to a top-level executable there: it
# may be a stale Debug binary from another generator or an earlier build.
cache_file="${BUILD_DIR}/CMakeCache.txt"
if grep -Eq '^CMAKE_CONFIGURATION_TYPES:[^=]*=.+$' "${cache_file}"; then
    bench_dir="${BUILD_DIR}/bench/Release"
else
    build_type="$(sed -n 's/^CMAKE_BUILD_TYPE:[^=]*=//p' "${cache_file}")"
    if [[ "${build_type}" != "Release" ]]; then
        echo "ERROR: Expected a Release build, but CMAKE_BUILD_TYPE is '${build_type}'." >&2
        exit 1
    fi
    bench_dir="${BUILD_DIR}/bench"
fi

find_benchmark()
{
    local name="$1" candidate
    for candidate in "${bench_dir}/${name}" "${bench_dir}/${name}.exe"; do
        if [[ -x "${candidate}" || -f "${candidate}" ]]; then
            printf '%s' "${candidate}"
            return 0
        fi
    done
    return 1
}

bench64_exe="$(find_benchmark frontier_bench || true)"
bench32_exe="$(find_benchmark frontier_bench_payload32 || true)"
if [[ "${payload_mode}" != 32 && -z "${bench64_exe}" ]]; then
    echo "ERROR: The 8-byte payload benchmark was not found under ${bench_dir}." >&2
    exit 1
fi
if [[ "${payload_mode}" != 64 && -z "${bench32_exe}" ]]; then
    echo "ERROR: The 4-byte payload benchmark was not found under ${bench_dir}." >&2
    exit 1
fi

if (( $# > 0 )); then
    if [[ "${payload_mode}" == both ]]; then
        for argument in "$@"; do
            if [[ "${argument}" == --benchmark_out ||
                  "${argument}" == --benchmark_out=* ]]; then
                echo "ERROR: --benchmark_out would be overwritten in both-width mode." >&2
                echo "Set FRONTIER_PERF_PAYLOAD_BITS to 32 or 64, or omit --benchmark_out." >&2
                exit 2
            fi
        done
    fi
    echo "Running payload ${payload_mode} benchmark(s) with caller-supplied arguments..."
    if [[ "${payload_mode}" != 32 ]]; then
        "${bench64_exe}" "$@"
    fi
    if [[ "${payload_mode}" != 64 ]]; then
        "${bench32_exe}" "$@"
    fi
    exit 0
fi

echo "Running payload ${payload_mode} documented performance suite with five repetitions..."
echo "Pass Google Benchmark arguments to this script to override the default suite."
if [[ "${payload_mode}" != 32 ]]; then
    echo "Writing ${ROOT_DIR}/real_world_perf_payload64.json"
    "${bench64_exe}" \
        '--benchmark_filter=BM_SubtreeAssembly' \
        --benchmark_repetitions=5 \
        --benchmark_report_aggregates_only=true \
        "--benchmark_out=${ROOT_DIR}/real_world_perf_payload64.json" \
        --benchmark_out_format=json
fi
if [[ "${payload_mode}" != 64 ]]; then
    echo "Writing ${ROOT_DIR}/real_world_perf_payload32.json"
    "${bench32_exe}" \
        '--benchmark_filter=BM_SubtreeAssembly' \
        --benchmark_repetitions=5 \
        --benchmark_report_aggregates_only=true \
        "--benchmark_out=${ROOT_DIR}/real_world_perf_payload32.json" \
        --benchmark_out_format=json
fi
