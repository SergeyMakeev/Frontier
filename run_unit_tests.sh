#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${FRONTIER_UNIT_BUILD_DIR:-${ROOT_DIR}/build-unit-debug}"

if ! command -v cmake >/dev/null 2>&1; then
    echo "ERROR: CMake was not found in PATH." >&2
    exit 1
fi

configure_args=(
    -S "${ROOT_DIR}"
    -B "${BUILD_DIR}"
    -DCMAKE_BUILD_TYPE=Debug
    -DFRONTIER_BUILD_TESTS=ON
    -DFRONTIER_BUILD_BENCH=OFF
    -DFRONTIER_BVH_WIDTH=AUTO
    -DFRONTIER_SSE2_ONLY=ON
    -DFRONTIER_STATS=OFF
    -DFRONTIER_CONTRACT_CHECKS=ON
    -DFRONTIER_VALIDATE_SUBTREES=ON
)
if [[ ! -f "${BUILD_DIR}/CMakeCache.txt" ]] && command -v ninja >/dev/null 2>&1; then
    configure_args+=(-G Ninja)
fi

echo "Configuring Debug unit tests (BVH4 + BVH8)..."
cmake "${configure_args[@]}"
cmake --build "${BUILD_DIR}" --config Debug \
    --target frontier_unit_tests --parallel
ctest --test-dir "${BUILD_DIR}" -C Debug --output-on-failure --parallel
