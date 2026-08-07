#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${HLOD_PROFILE_BUILD_DIR:-${ROOT_DIR}/build-macos-profile}"

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

generator=()
if [[ ! -f "${BUILD_DIR}/CMakeCache.txt" ]] && command -v ninja >/dev/null 2>&1; then
    generator=(-G Ninja)
fi

echo "Building optimized arm64 benchmark with source line tables..."
cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" "${generator[@]}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_OSX_ARCHITECTURES=arm64 \
    -DHLOD_BUILD_TESTS=OFF \
    -DHLOD_BUILD_BENCH=ON \
    -DHLOD_AVX2=OFF \
    -DHLOD_FORCE_SCALAR=OFF \
    -DHLOD_PROFILE_SYMBOLS=ON
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

templates="$(xcrun xctrace list templates 2>/dev/null || true)"
if ! grep -Fq 'CPU Counters' <<<"${templates}"; then
    echo "ERROR: This Xcode installation does not expose a CPU Counters template." >&2
    echo "Open Instruments manually, select CPU Counters, and launch:" >&2
    echo "  ${bench_exe}" >&2
    echo "with filter BM_MixedForest100k/flat_pct:0/cached:1" >&2
    exit 1
fi

stamp="$(date +%Y%m%d-%H%M%S)"
trace="${ROOT_DIR}/hlod_cpu_counters_${stamp}.trace"
echo "Recording CPU Counters for the cached hierarchical workload..."
xcrun xctrace record \
    --template 'CPU Counters' \
    --output "${trace}" \
    --time-limit 25s \
    --launch -- "${bench_exe}" \
    '--benchmark_filter=BM_MixedForest100k/flat_pct:0/cached:1$' \
    --benchmark_min_time=18s \
    --benchmark_repetitions=1

archive="${trace}.zip"
/usr/bin/ditto -c -k --sequesterRsrc --keepParent "${trace}" "${archive}"
echo "Wrote ${trace}"
echo "Attach ${archive} to the Codex task for analysis."
echo "Open it with: open '${trace}'"
