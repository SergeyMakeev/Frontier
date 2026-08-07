#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if (( $# == 0 )); then
    set -- \
        '--benchmark_filter=BM_(MixedForest100k/.*/cached:1|RootDecisionForest100k/.*/cached:1|TlasMortonRebuild/100000/[01])' \
        --benchmark_min_time=0.25s \
        --benchmark_repetitions=7 \
        --benchmark_report_aggregates_only=true \
        "--benchmark_out=${ROOT_DIR}/mac_a.json" \
        --benchmark_out_format=json
fi

echo "Running mac_a: baseline instance layout"
exec "${ROOT_DIR}/run_perf_layout_off.sh" "$@"
