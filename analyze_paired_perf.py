#!/usr/bin/env python3
"""Summarize an ABBA report produced by run_paired_perf.sh."""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import statistics
from collections import defaultdict
from pathlib import Path


TIME_TO_US = {"ns": 1e-3, "us": 1.0, "ms": 1e3, "s": 1e6}
PRACTICAL_THRESHOLD_PERCENT = 0.25


def percentile(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def geometric_effect(log_ratios: list[float]) -> float:
    return 100.0 * math.expm1(statistics.fmean(log_ratios))


def bootstrap_interval(log_ratios: list[float], iterations: int = 50_000) -> tuple[float, float]:
    rng = random.Random(0xF20A71E2)
    count = len(log_ratios)
    effects = [
        geometric_effect([log_ratios[rng.randrange(count)] for _ in range(count)])
        for _ in range(iterations)
    ]
    return percentile(effects, 0.025), percentile(effects, 0.975)


def coefficient_of_variation(values: list[float]) -> float:
    if len(values) < 2:
        return 0.0
    return 100.0 * statistics.stdev(values) / statistics.fmean(values)


def load_samples(report_dir: Path) -> dict[tuple[str, str], list[dict[str, object]]]:
    grouped: dict[tuple[str, str], list[dict[str, object]]] = defaultdict(list)
    expected_revision = {1: "baseline", 2: "candidate", 3: "candidate", 4: "baseline"}
    seen: set[tuple[str, str, int, int]] = set()
    with (report_dir / "samples.csv").open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            cycle = int(row["cycle"])
            slot = int(row["slot"])
            sample_key = (row["case"], row["payload"], cycle, slot)
            if sample_key in seen:
                raise ValueError(f"duplicate sample row: {sample_key}")
            seen.add(sample_key)
            if expected_revision.get(slot) != row["revision"]:
                raise ValueError(f"invalid ABBA schedule at {sample_key}: {row['revision']}")
            json_path = report_dir / "raw" / Path(row["json"]).name
            with json_path.open(encoding="utf-8") as sample_stream:
                document = json.load(sample_stream)
            benchmarks = document.get("benchmarks", [])
            if len(benchmarks) != 1:
                raise ValueError(f"expected one benchmark in {json_path}, found {len(benchmarks)}")
            benchmark = benchmarks[0]
            context = document.get("context", {})
            load_average = context.get("load_avg", [])
            unit = benchmark["time_unit"]
            if unit not in TIME_TO_US:
                raise ValueError(f"unsupported time unit {unit!r} in {json_path}")
            real_time_us = float(benchmark["real_time"]) * TIME_TO_US[unit]
            cpu_time_us = float(benchmark["cpu_time"]) * TIME_TO_US[unit]
            grouped[(row["case"], row["payload"])].append(
                {
                    "cycle": cycle,
                    "revision": row["revision"],
                    "time_us": real_time_us,
                    "cpu_time_us": cpu_time_us,
                    "temperature_before": int(row["temperature_before"]),
                    "temperature_after": int(row["temperature_after"]),
                    "frequency_before": int(row["frequency_before"]),
                    "frequency_after": int(row["frequency_after"]),
                    "load_1m": float(load_average[0]) if load_average else math.nan,
                }
            )
    return grouped


def summarize(report_dir: Path) -> str:
    grouped = load_samples(report_dir)
    lines = [
        f"# Paired performance summary: `{report_dir.name}`",
        "",
        "Negative changes are improvements. The paired effect is the geometric mean of the",
        "candidate/baseline ratio after averaging each revision's two samples per ABBA cycle.",
        "The interval is a deterministic 50,000-resample percentile bootstrap over cycles.",
        f"Verdicts use a ±{PRACTICAL_THRESHOLD_PERCENT:.2f}% practical-equivalence threshold.",
        "",
        "| Case | Payload | Baseline median | Candidate median | Raw median | Paired effect | 95% interval | Baseline CV | Candidate CV | Verdict |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    all_temperatures: list[int] = []
    all_frequencies: list[int] = []
    all_loads: list[float] = []
    all_cpu_real_deltas: list[float] = []
    paired_effects: dict[tuple[str, str], float] = {}
    table_rows: list[str] = []
    cycle_lines: list[str] = []
    preferred_order = {
        name: index for index, name in enumerate(
            ("live_city", "live_city_render", "motion", "identity_50", "identity_100")
        )
    }
    ordered_groups = sorted(
        grouped.items(),
        key=lambda item: (
            1 if item[0][0].startswith("control_") else 0,
            preferred_order.get(item[0][0], 100),
            item[0][0],
            item[0][1],
        ),
    )
    for (case, payload), samples in ordered_groups:
        revisions = {
            revision: [float(sample["time_us"]) for sample in samples if sample["revision"] == revision]
            for revision in ("baseline", "candidate")
        }
        cycle_values: dict[tuple[int, str], list[float]] = defaultdict(list)
        for sample in samples:
            cycle_values[(int(sample["cycle"]), str(sample["revision"]))].append(float(sample["time_us"]))
            all_temperatures.extend((int(sample["temperature_before"]), int(sample["temperature_after"])))
            all_frequencies.extend((int(sample["frequency_before"]), int(sample["frequency_after"])))
            all_cpu_real_deltas.append(
                100.0 * abs(float(sample["cpu_time_us"]) / float(sample["time_us"]) - 1.0)
            )
            if not math.isnan(float(sample["load_1m"])):
                all_loads.append(float(sample["load_1m"]))
        cycles = sorted({cycle for cycle, _ in cycle_values})
        log_ratios = []
        for cycle in cycles:
            if len(cycle_values[(cycle, "baseline")]) != 2 or len(cycle_values[(cycle, "candidate")]) != 2:
                raise ValueError(f"cycle {cycle} for {case}/payload{payload} is not a complete ABBA quartet")
            baseline = statistics.fmean(cycle_values[(cycle, "baseline")])
            candidate = statistics.fmean(cycle_values[(cycle, "candidate")])
            log_ratios.append(math.log(candidate / baseline))
        baseline_median = statistics.median(revisions["baseline"])
        candidate_median = statistics.median(revisions["candidate"])
        raw_effect = 100.0 * (candidate_median / baseline_median - 1.0)
        effect = geometric_effect(log_ratios)
        paired_effects[(case, payload)] = effect
        low, high = bootstrap_interval(log_ratios)
        if high < -PRACTICAL_THRESHOLD_PERCENT:
            verdict = "improved"
        elif low > PRACTICAL_THRESHOLD_PERCENT:
            verdict = "regressed"
        elif low >= -PRACTICAL_THRESHOLD_PERCENT and high <= PRACTICAL_THRESHOLD_PERCENT:
            verdict = "flat"
        else:
            verdict = "inconclusive"
        table_rows.append(
            f"| `{case}` | {payload} | {baseline_median:.3f} us | {candidate_median:.3f} us | "
            f"{raw_effect:+.2f}% | {effect:+.2f}% | [{low:+.2f}%, {high:+.2f}%] | "
            f"{coefficient_of_variation(revisions['baseline']):.2f}% | "
            f"{coefficient_of_variation(revisions['candidate']):.2f}% | {verdict} |"
        )
        cycle_lines.append(
            f"- `{case}` {payload} cycle effects: "
            + ", ".join(f"{100.0 * math.expm1(value):+.2f}%" for value in log_ratios)
        )
    lines.extend(table_rows)
    lines.extend(["", "## Cycle effects", "", *cycle_lines])
    lines.extend(
        [
            "",
            "## Environment envelope",
            "",
            f"- Temperature: {min(all_temperatures) / 1000.0:.3f}–{max(all_temperatures) / 1000.0:.3f} °C",
            f"- Frequency: {min(all_frequencies)}–{max(all_frequencies)} kHz",
            f"- One-minute load average: {min(all_loads):.3f}–{max(all_loads):.3f}" if all_loads else "- One-minute load average: unavailable",
            f"- Maximum CPU-time/wall-time divergence: {max(all_cpu_real_deltas):.3f}%",
            f"- Samples: {sum(len(samples) for samples in grouped.values())}",
            "",
        ]
    )
    control_effects = [
        effect for (case, _), effect in paired_effects.items() if case.startswith("control_")
    ]
    if control_effects:
        control_geomean = 100.0 * math.expm1(
            statistics.fmean(math.log1p(effect / 100.0) for effect in control_effects)
        )
        lines.extend(
            [
                "## Machine-control gate",
                "",
                f"- Paired geomean across {len(control_effects)} controls: {control_geomean:+.2f}%",
                "- Workload effects are not normalized by this value; the controls are an independent drift gate.",
                "",
            ]
        )
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("report_dir", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    output = summarize(args.report_dir.resolve())
    if args.output:
        args.output.write_text(output, encoding="utf-8")
    else:
        print(output)


if __name__ == "__main__":
    main()
