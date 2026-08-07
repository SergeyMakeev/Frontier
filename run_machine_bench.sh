#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${HLOD_MACHINE_BUILD_DIR:-${ROOT_DIR}/build-machine-perf}"

if ! command -v cmake >/dev/null 2>&1; then
    echo "ERROR: CMake was not found in PATH." >&2
    echo "Install it with 'brew install cmake' or from https://cmake.org/." >&2
    exit 1
fi

generator=""
if [[ ! -f "${BUILD_DIR}/CMakeCache.txt" ]] && command -v ninja >/dev/null 2>&1; then
    generator="Ninja"
fi

target_architecture=""
if [[ "$(uname -s)" == "Darwin" ]]; then
    apple_silicon="$(sysctl -n hw.optional.arm64 2>/dev/null || true)"
    if [[ "$(uname -m)" == "arm64" || "${apple_silicon}" == "1" ]]; then
        target_architecture="arm64"
        echo "Targeting native Apple Silicon arm64/NEON."
    fi
fi

configure_args=(
    -S "${ROOT_DIR}"
    -B "${BUILD_DIR}"
    -DCMAKE_BUILD_TYPE=Release
    -DHLOD_BUILD_TESTS=OFF
    -DHLOD_BUILD_BENCH=ON
    -DHLOD_AVX2=OFF
)
if [[ -n "${generator}" ]]; then
    configure_args+=(-G "${generator}")
fi
if [[ -n "${target_architecture}" ]]; then
    configure_args+=("-DCMAKE_OSX_ARCHITECTURES=${target_architecture}")
fi

cmake "${configure_args[@]}"
cmake --build "${BUILD_DIR}" --config Release \
    --target hlod_machine_bench --parallel

cache_file="${BUILD_DIR}/CMakeCache.txt"
if grep -Eq '^CMAKE_CONFIGURATION_TYPES:[^=]*=.+$' "${cache_file}"; then
    bench_dir="${BUILD_DIR}/bench/Release"
else
    build_type="$(sed -n 's/^CMAKE_BUILD_TYPE:[^=]*=//p' "${cache_file}")"
    if [[ "${build_type}" != "Release" ]]; then
        echo "ERROR: Expected Release, got CMAKE_BUILD_TYPE='${build_type}'." >&2
        exit 1
    fi
    bench_dir="${BUILD_DIR}/bench"
fi

bench_exe="${bench_dir}/hlod_machine_bench"
if [[ ! -x "${bench_exe}" ]]; then
    echo "ERROR: Benchmark executable was not found at ${bench_exe}." >&2
    exit 1
fi

if (( $# > 0 )); then
    exec "${bench_exe}" "$@"
fi

exec "${bench_exe}" \
    --benchmark_min_time=0.15s \
    --benchmark_repetitions=5 \
    --benchmark_enable_random_interleaving=true \
    --benchmark_report_aggregates_only=true \
    "--benchmark_out=${ROOT_DIR}/machine_perf.json" \
    --benchmark_out_format=json
