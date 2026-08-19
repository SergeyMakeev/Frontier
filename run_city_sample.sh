#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${FRONTIER_CITY_BUILD_DIR:-${ROOT_DIR}/build-city}"

if ! command -v cmake >/dev/null 2>&1; then
    echo "ERROR: CMake was not found in PATH." >&2
    exit 1
fi

configure_args=(
    -S "${ROOT_DIR}"
    -B "${BUILD_DIR}"
    -DCMAKE_BUILD_TYPE=Release
    -DFRONTIER_BUILD_CITY_SAMPLE=ON
    -DFRONTIER_DEBUG_TOOLS=ON
    -DFRONTIER_BUILD_TESTS=OFF
    -DFRONTIER_BUILD_BENCH=OFF
)
if [[ ! -f "${BUILD_DIR}/CMakeCache.txt" ]] && command -v ninja >/dev/null 2>&1; then
    configure_args+=(-G Ninja)
fi

echo "Configuring the Frontier bgfx city sample..."
cmake "${configure_args[@]}"

echo "Building frontier_city..."
cmake --build "${BUILD_DIR}" --config Release \
    --target frontier_city --parallel

sample_candidates=(
    "${BUILD_DIR}/examples/city/frontier_city"
    "${BUILD_DIR}/examples/city/Release/frontier_city"
    "${BUILD_DIR}/examples/city/frontier_city.exe"
    "${BUILD_DIR}/examples/city/Release/frontier_city.exe"
)
sample_executable=""
for candidate in "${sample_candidates[@]}"; do
    if [[ -f "${candidate}" ]]; then
        sample_executable="${candidate}"
        break
    fi
done

if [[ -z "${sample_executable}" ]]; then
    echo "ERROR: Built sample executable was not found under ${BUILD_DIR}/examples/city." >&2
    exit 1
fi

echo "Launching Frontier Dynamic City..."
exec "${sample_executable}" "$@"
