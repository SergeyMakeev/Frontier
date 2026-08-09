#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${FRONTIER_MACHINE_BUILD_DIR:-${ROOT_DIR}/build-machine-perf}"

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
avx2=OFF
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
        fi
    fi
elif [[ "${host_machine}" == "x86_64" ]]; then
    avx2=ON
fi

configure_args=(
    -S "${ROOT_DIR}"
    -B "${BUILD_DIR}"
    -DCMAKE_BUILD_TYPE=Release
    -DFRONTIER_BUILD_TESTS=OFF
    -DFRONTIER_BUILD_BENCH=ON
    -DFRONTIER_AVX2="${avx2}"
)
if [[ -n "${generator}" ]]; then
    configure_args+=(-G "${generator}")
fi
if [[ -n "${target_architecture}" ]]; then
    configure_args+=("-DCMAKE_OSX_ARCHITECTURES=${target_architecture}")
fi

cmake "${configure_args[@]}"
cmake --build "${BUILD_DIR}" --config Release \
    --target frontier_machine_bench --parallel

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

bench_exe="${bench_dir}/frontier_machine_bench"
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
    --benchmark_report_aggregates_only=true \
    "--benchmark_out=${ROOT_DIR}/machine_perf.json" \
    --benchmark_out_format=json
