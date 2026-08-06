#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${HLOD_PERF_BUILD_DIR:-${ROOT_DIR}/build-perf}"

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
    -DHLOD_BUILD_TESTS=OFF
    -DHLOD_BUILD_BENCH=ON
    -DHLOD_AVX2="${avx2}"
    -DHLOD_FORCE_SCALAR=OFF
)
if [[ -n "${generator}" ]]; then
    configure_args+=(-G "${generator}")
fi
if [[ -n "${target_architecture}" ]]; then
    configure_args+=("-DCMAKE_OSX_ARCHITECTURES=${target_architecture}")
fi
cmake "${configure_args[@]}"

echo "Building hlod_bench..."
cmake --build "${BUILD_DIR}" --config Release --target hlod_bench --parallel

bench_exe=""
for candidate in \
    "${BUILD_DIR}/bench/hlod_bench" \
    "${BUILD_DIR}/bench/Release/hlod_bench" \
    "${BUILD_DIR}/bench/hlod_bench.exe" \
    "${BUILD_DIR}/bench/Release/hlod_bench.exe"
do
    if [[ -x "${candidate}" || -f "${candidate}" ]]; then
        bench_exe="${candidate}"
        break
    fi
done

if [[ -z "${bench_exe}" ]]; then
    echo "ERROR: Built benchmark executable was not found under ${BUILD_DIR}/bench." >&2
    exit 1
fi

if (( $# > 0 )); then
    echo "Running hlod_bench with caller-supplied arguments..."
    exec "${bench_exe}" "$@"
fi

echo "Running the documented performance suite with five repetitions..."
echo "Pass Google Benchmark arguments to this script to override the default suite."
exec "${bench_exe}" \
    '--benchmark_filter=BM_(View_Breakdown|View_MultiView|FlatForest100k|MixedForest100k|RootDecisionForest100k)' \
    --benchmark_repetitions=5 \
    --benchmark_report_aggregates_only=false
