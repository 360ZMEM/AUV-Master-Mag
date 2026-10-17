#!/usr/bin/env python3
"""Held-out cross-shape publication audit for the v2 FWHM guard."""
from __future__ import annotations

import argparse
import csv
from dataclasses import replace
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tools"))

from auv_mag_tracking.v2 import FwhmConfig, estimate_fwhm, write_experiment_manifest  # noqa: E402
from run_v2_stability_comparison import NOMINAL_SOURCE, SCENARIOS, scenario_parameters  # noqa: E402

EXTRA_SCENARIOS = ("matched_wire", "dipole_as_wire")
ALL_SCENARIOS = (*SCENARIOS, *EXTRA_SCENARIOS)


def profile(scenario: str, seed: int) -> tuple[np.ndarray, np.ndarray, float, float, float, str, str]:
    if scenario in SCENARIOS:
        rng = np.random.default_rng(100_000 * SCENARIOS.index(scenario) + seed)
        source = scenario
    else:
        rng = np.random.default_rng(100_000 * len(SCENARIOS) + seed)
        source = "nominal"
    z = float(rng.uniform(2.5, 5.0))
    params = scenario_parameters(source, rng, z)
    if scenario == "matched_wire":
        params["model"] = "single_wire"
    half_span = float(params["half_span"])
    x = np.linspace(-half_span, half_span, 241)
    center = float(params["center"])
    gain = float(params["gain"])
    baseline = float(params["baseline"])
    data_model = str(params["model"])
    if data_model == "single_wire":
        anomaly = gain * NOMINAL_SOURCE / z / np.sqrt((x - center) ** 2 + z ** 2)
    else:
        anomaly = gain * NOMINAL_SOURCE / ((x - center) ** 2 + z ** 2)
    y = baseline + float(params["slope"]) * x + anomaly
    y += rng.normal(0.0, float(params["noise"]), x.size)
    declared = "physical_single_wire" if scenario in EXTRA_SCENARIOS else "thesis_dipole"
    return x, y, z, baseline, float(params["noise"]), data_model, declared


def run(output_dir: Path, seed_start: int = 200, seeds: int = 200) -> dict[str, object]:
    rows: list[dict[str, object]] = []
    config = FwhmConfig(
        minimum_samples=31, smoothing_window=5, edge_fraction=0.12,
        minimum_prominence_nt=0.25, minimum_snr_db=6.0, minimum_side_samples=3,
        minimum_coverage_factor=0.10, maximum_model_residual=0.35,
        minimum_validity_score=0.50, altitude_m=0.0,
        shape_factor_relative_sigma=0.0, altitude_sigma_m=0.0,
    )
    for scenario in ALL_SCENARIOS:
        for seed in range(seed_start, seed_start + seeds):
            x, y, truth, baseline, noise, data_model, declared = profile(scenario, seed)
            shape_factor = 1.0 / (2.0 * math.sqrt(3.0)) if declared == "physical_single_wire" else 0.5
            candidate = replace(config, shape_model=declared, shape_factor=shape_factor)
            historical = replace(
                candidate, minimum_shape_support_widths=1e-6,
                minimum_competing_shape_advantage=1e6,
            )
            for mode, setting in (("before_guard", historical), ("with_guard", candidate)):
                result = estimate_fwhm(x, y, setting, baseline_nt=baseline, noise_nt=noise, altitude_m=0.0)
                error = None if not result.valid else float(result.vertical_separation_m - truth)
                rows.append({
                    "scenario": scenario, "seed": seed, "mode": mode,
                    "data_model": data_model, "declared_model": declared,
                    "valid": int(result.valid), "truth_m": truth,
                    "estimate_m": result.vertical_separation_m,
                    "abs_error_m": None if error is None else abs(error),
                    "declared_residual": result.model_residual,
                    "competing_residual": result.competing_model_residual,
                    "shape_support_widths": result.shape_support_widths,
                    "reason_codes": "|".join(result.reason_codes),
                })
    summary: dict[str, object] = {
        "seed_start": seed_start, "seeds_per_scenario": seeds,
        "validation_role": "held_out_from_threshold_exploration_seeds_0_to_199",
        "scenarios": {},
    }
    for scenario in ALL_SCENARIOS:
        summary["scenarios"][scenario] = {}
        for mode in ("before_guard", "with_guard"):
            group = [r for r in rows if r["scenario"] == scenario and r["mode"] == mode]
            published = [float(r["abs_error_m"]) for r in group if r["valid"]]
            summary["scenarios"][scenario][mode] = {
                "runs": len(group),
                "valid_rate": sum(int(r["valid"]) for r in group) / len(group),
                "published_mae_m": float(np.mean(published)) if published else None,
                "large_error_publish_rate": sum(
                    r["valid"] and float(r["abs_error_m"]) > 0.5 for r in group
                ) / len(group),
                "competing_shape_rejections": sum(
                    "competing_shape_model_preferred" in str(r["reason_codes"]) for r in group
                ),
                "insufficient_shape_rejections": sum(
                    "shape_evidence_insufficient" in str(r["reason_codes"]) for r in group
                ),
            }
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "per_run.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summary_path = output_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    write_experiment_manifest(
        output_dir, experiment="v2_shape_guard_held_out_validation",
        config={
            "seed_start": seed_start, "seeds_per_scenario": seeds,
            "scenarios": list(ALL_SCENARIOS),
            "minimum_shape_support_widths": config.minimum_shape_support_widths,
            "minimum_competing_shape_advantage": config.minimum_competing_shape_advantage,
            "maximum_competing_shape_residual_ratio": config.maximum_competing_shape_residual_ratio,
        },
        artifact_paths=[csv_path, summary_path], repository_root=ROOT,
    )
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed-start", type=int, default=200)
    parser.add_argument("--seeds", type=int, default=200)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "results/v2/shape_guard_validation")
    args = parser.parse_args()
    if args.seed_start < 0 or args.seeds < 1:
        parser.error("seed start must be non-negative and seeds must be positive")
    print(json.dumps(run(args.output_dir, args.seed_start, args.seeds), ensure_ascii=False, indent=2))
