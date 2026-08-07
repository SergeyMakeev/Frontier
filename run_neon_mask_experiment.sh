#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Running same-process scalarized-vs-vector-resident NEON mask A/B..."
HLOD_MACHINE_BUILD_DIR="${ROOT_DIR}/build-neon-mask-machine" \
    "${ROOT_DIR}/run_machine_bench.sh" \
    '--benchmark_filter=BM_WideAabbMask(Scalarized|VectorResident)' \
    --benchmark_min_time=1s \
    --benchmark_repetitions=15 \
    --benchmark_enable_random_interleaving=true \
    --benchmark_report_aggregates_only=true \
    "--benchmark_out=${ROOT_DIR}/neon_mask_ab.json" \
    --benchmark_out_format=json

echo "Wrote ${ROOT_DIR}/neon_mask_ab.json"
