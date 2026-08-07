#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export HLOD_PERF_BUILD_DIR="${ROOT_DIR}/build-perf-layout-on"
export HLOD_SPATIAL_INSTANCE_LAYOUT=ON

exec "${ROOT_DIR}/run_perf_bench.sh" "$@"
