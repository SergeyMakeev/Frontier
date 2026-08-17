#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
build_dir="${1:-${repo_root}/build-arm-pgo}"
cpu="${FRONTIER_PGO_CPU:-0}"
jobs="${FRONTIER_PGO_JOBS:-$(getconf _NPROCESSORS_ONLN)}"
stamp="$(date -u +%Y%m%dT%H%M%SZ)"
profile_dir="${2:-${build_dir}/profile-${stamp}}"

case "$(uname -m)" in
    aarch64|arm64) ;;
    *)
        echo "ERROR: run_arm_pgo.sh is intended for a native ARM64 host." >&2
        exit 2
        ;;
esac

if ! [[ "${cpu}" =~ ^[0-9]+$ ]] || ! taskset -c "${cpu}" true; then
    echo "ERROR: FRONTIER_PGO_CPU=${cpu} is not an available CPU." >&2
    exit 2
fi
if ! [[ "${jobs}" =~ ^[1-9][0-9]*$ ]]; then
    echo "ERROR: FRONTIER_PGO_JOBS must be a positive integer." >&2
    exit 2
fi

common=(
    -G Ninja
    -DCMAKE_BUILD_TYPE=Release
    -DFRONTIER_BUILD_BENCH=ON
    -DFRONTIER_BUILD_TESTS=OFF
    -DFRONTIER_CONTRACT_CHECKS=OFF
    -DFRONTIER_VALIDATE_SUBTREES=OFF
    -DFRONTIER_IPO=ON
    "-DFRONTIER_PGO_DIR=${profile_dir}"
)
training_filter='^BM_LiveCityRenderSubmissionFrame/iterations:8192$'

echo "Configuring GCC profile generation: ${profile_dir}"
cmake -S "${repo_root}" -B "${build_dir}" \
    "${common[@]}" -DFRONTIER_PGO_MODE=GENERATE
cmake --build "${build_dir}" --parallel "${jobs}" --target \
    frontier_pgo_training \
    frontier_submission_bench \
    frontier_submission_bench_payload32

for executable in \
    frontier_pgo_training \
    frontier_submission_bench \
    frontier_submission_bench_payload32
do
    echo "Training ${executable} on CPU ${cpu}"
    taskset -c "${cpu}" "${build_dir}/bench/${executable}" \
        "--benchmark_filter=${training_filter}" \
        --benchmark_min_warmup_time=0.25 \
        --benchmark_repetitions=1
done

echo "Rebuilding with the collected profiles"
cmake -S "${repo_root}" -B "${build_dir}" \
    "${common[@]}" -DFRONTIER_PGO_MODE=USE
cmake --build "${build_dir}" --parallel "${jobs}" --target \
    frontier \
    frontier_pgo_training \
    frontier_submission_bench \
    frontier_submission_bench_payload32

echo "PGO build complete: ${build_dir}"
echo "Profile corpus: ${profile_dir}"
