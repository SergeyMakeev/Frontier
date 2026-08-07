#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Running isolated NEON mask extraction probes..."
HLOD_MACHINE_BUILD_DIR="${ROOT_DIR}/build-neon-mask-machine" \
    "${ROOT_DIR}/run_machine_bench.sh" \
    '--benchmark_filter=BM_VectorCompareMask(Current|U64).*128' \
    --benchmark_min_time=0.5s \
    --benchmark_repetitions=9 \
    --benchmark_enable_random_interleaving=true \
    --benchmark_report_aggregates_only=true \
    "--benchmark_out=${ROOT_DIR}/neon_mask_machine.json" \
    --benchmark_out_format=json

echo "Running the end-to-end cases most sensitive to wide frustum tests..."
HLOD_PERF_BUILD_DIR="${ROOT_DIR}/build-neon-mask-perf" \
    "${ROOT_DIR}/run_perf_bench.sh" \
    '--benchmark_filter=BM_(MixedForest100k/flat_pct:0|RootDecisionForest100k/far:0)/cached:[01]$' \
    --benchmark_min_time=0.5s \
    --benchmark_repetitions=9 \
    --benchmark_enable_random_interleaving=true \
    --benchmark_report_aggregates_only=true \
    "--benchmark_out=${ROOT_DIR}/neon_mask_hlod.json" \
    --benchmark_out_format=json

echo "Wrote:"
echo "  ${ROOT_DIR}/neon_mask_machine.json"
echo "  ${ROOT_DIR}/neon_mask_hlod.json"
