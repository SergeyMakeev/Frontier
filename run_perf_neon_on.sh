#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export HLOD_PERF_BUILD_DIR="${ROOT_DIR}/build-perf-neon-on"
export HLOD_PERF_NEON_RSQRT=ON

exec "${ROOT_DIR}/run_perf_bench.sh" \
    --benchmark_filter='BM_(Adversarial_WideNode|AssetSharing_CutCost|MixedForest100k|View_Breakdown)' \
    --benchmark_repetitions=7 \
    --benchmark_report_aggregates_only=true \
    --benchmark_out="${ROOT_DIR}/neon-on.json" \
    --benchmark_out_format=json \
    "$@"
