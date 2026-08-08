#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Running production-kernel, cache-hit, and output-append architecture probes..."
HLOD_MACHINE_BUILD_DIR="${ROOT_DIR}/build-arch-perf" \
    "${ROOT_DIR}/run_machine_bench.sh" \
    '--benchmark_filter=BM_(Kernel(WideAabb|DistanceError|CacheHit)|OutputAppend)' \
    --benchmark_min_time=0.75s \
    --benchmark_repetitions=11 \
    --benchmark_report_aggregates_only=true \
    "--benchmark_out=${ROOT_DIR}/arch_kernel_perf.json" \
    --benchmark_out_format=json

echo "Wrote ${ROOT_DIR}/arch_kernel_perf.json"
