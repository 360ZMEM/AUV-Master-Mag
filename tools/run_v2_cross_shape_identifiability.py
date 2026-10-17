#!/usr/bin/env python3
"""Audit whether FWHM depth remains identifiable across field-shape families.

All inputs are synthetic. Thresholds are learned only from matched generalized
power-law scans with seeds 0--79, then frozen for seeds 200--299.
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from auv_mag_tracking.v2 import FwhmConfig, estimate_fwhm, write_experiment_manifest  # noqa: E402

SHAPES = ("power_0.5", "power_0.7", "power_0.9", "power_1.0", "double_0.5", "double_1.0", "double_1.5")
HEIGHTS = (0.0, 1.0, 2.0, 4.0, 6.0)
SHORT_HEIGHTS = (0.0, 0.5, 1.0, 1.5)
EXPONENT_GRID = np.linspace(0.4, 1.2, 161)
CONFIG = FwhmConfig(
    shape_model="thesis_dipole", shape_factor=0.5, altitude_m=0.0,
    minimum_prominence_nt=0.1,
)


def _profile(shape: str, seed: int, h: float) -> tuple[np.ndarray, np.ndarray, float, float, float]:
    rng = np.random.default_rng(seed + 100_000 * SHAPES.index(shape) + int(round(100 * h)))
    z0 = 3.0 + 0.35 * np.sin(seed * 0.19)
    z = z0 + h
    x = np.linspace(-50.0, 50.0, 501)
    center = float(rng.uniform(-0.3, 0.3))
    gain = float(rng.uniform(16.0, 24.0))
    baseline = float(rng.uniform(1.5, 2.5))
    noise = 0.08
    if shape.startswith("power_"):
        exponent = float(shape.split("_")[1])
        y = baseline + gain / (1.0 + ((x - center) / z) ** 2) ** exponent
    else:
        conductor_spacing = float(shape.split("_")[1]) * z0
        y = baseline + gain * 0.5 / np.sqrt(1.0 + ((x - center - conductor_spacing / 2) / z) ** 2)
        y += gain * 0.5 / np.sqrt(1.0 + ((x - center + conductor_spacing / 2) / z) ** 2)
    y += rng.normal(0.0, noise, len(x))
    return x, y, z0, baseline, noise


def _fit_exponent(x: np.ndarray, measured: np.ndarray, center: float, width: float) -> tuple[float, float]:
    u2 = ((x - center) / width) ** 2
    candidate = (1.0 + 4.0 * (2.0 ** (1.0 / EXPONENT_GRID[:, None]) - 1.0) * u2[None, :]) ** (-EXPONENT_GRID[:, None])
    residuals = np.sqrt(np.mean((candidate - measured[None, :]) ** 2, axis=1))
    best = int(np.argmin(residuals))
    return float(EXPONENT_GRID[best]), float(residuals[best])


def _one_scan(shape: str, seed: int, height: float) -> dict[str, float | bool | None]:
    x, y, truth, baseline, noise = _profile(shape, seed, height)
    result = estimate_fwhm(x, y, CONFIG, baseline_nt=baseline, noise_nt=noise)
    geometry_reasons = {
        "peak_not_bracketed", "low_prominence", "low_snr", "left_half_crossing_missing",
        "right_half_crossing_missing", "insufficient_half_width_margin", "shape_evidence_insufficient",
        "vertical_separation_below_altitude",
    }
    usable_width = result.width_m is not None and not geometry_reasons.intersection(result.reason_codes)
    exponent = residual = adaptive_depth = None
    if usable_width:
        normalized = (result.smoothed_envelope_nt - result.baseline_nt) / result.prominence_nt
        exponent, residual = _fit_exponent(result.x_m, normalized, result.center_x_m, result.width_m)
        kappa = 1.0 / (2.0 * math.sqrt(2.0 ** (1.0 / exponent) - 1.0))
        adaptive_depth = kappa * result.width_m
    return {
        "height_m": height, "truth_z0_m": truth,
        "width_m": result.width_m if usable_width else None,
        "exponent": exponent, "shape_residual": residual,
        "adaptive_depth_m": adaptive_depth,
        "static_valid": result.valid,
        "static_depth_m": result.vertical_separation_m,
    }


def _features(scans: list[dict[str, float | bool | None]]) -> dict[str, float | bool | None]:
    first = scans[0]
    if any(scan["width_m"] is None for scan in scans):
        return {
            "complete": False, "shape_spread": None, "line_residual_m": None,
            "height_depth_m": None, "first_shape_residual": first["shape_residual"],
        }
    heights = np.asarray([float(scan["height_m"]) for scan in scans])
    widths = np.asarray([float(scan["width_m"]) for scan in scans])
    exponents = np.asarray([float(scan["exponent"]) for scan in scans])
    slope, intercept = np.polyfit(heights, widths, 1)
    return {
        "complete": bool(slope > 0),
        "shape_spread": float(np.ptp(exponents)),
        "line_residual_m": float(np.max(np.abs(widths - (slope * heights + intercept)))),
        "height_depth_m": float(intercept / slope) if slope > 0 else None,
        "first_shape_residual": first["shape_residual"],
    }


def _scenario(shape: str, seed: int, heights: tuple[float, ...]) -> dict[str, object]:
    scans = [_one_scan(shape, seed, height) for height in heights]
    features = _features(scans)
    first = scans[0]
    return {
        "shape": shape, "seed": seed, "height_span_m": max(heights) - min(heights),
        "truth_z0_m": first["truth_z0_m"],
        "static_valid": first["static_valid"], "static_depth_m": first["static_depth_m"],
        "adaptive_depth_m": first["adaptive_depth_m"],
        **features,
    }


def _threshold(rows: list[dict[str, object]], key: str) -> float:
    values = [float(row[key]) for row in rows if row["complete"] and row[key] is not None]
    return float(np.quantile(values, 0.99) * 1.25) if values else math.inf


def _score(rows: list[dict[str, object]], method: str) -> dict[str, float | int | None]:
    n = len(rows)
    valid = [row for row in rows if row[f"{method}_valid"]]
    errors = [abs(float(row[f"{method}_depth_m"]) - float(row["truth_z0_m"])) for row in valid]
    return {
        "runs": n, "release_rate": len(valid) / n,
        "published_mae_m": float(np.mean(errors)) if errors else None,
        "large_error_publish_rate": sum(error > 0.5 for error in errors) / n,
    }


def run(output_dir: Path, exploration_seeds: int = 80, holdout_seeds: int = 100) -> dict[str, object]:
    if exploration_seeds < 10 or holdout_seeds < 10:
        raise ValueError("at least 10 seeds per split are required")
    exploration = [
        _scenario(shape, seed, heights)
        for shape in SHAPES[:4]
        for heights in (HEIGHTS, SHORT_HEIGHTS)
        for seed in range(exploration_seeds)
    ]
    thresholds = {
        "shape_residual": _threshold(exploration, "first_shape_residual"),
        "shape_spread": _threshold(exploration, "shape_spread"),
        "line_residual_m": _threshold(exploration, "line_residual_m"),
    }
    rows = [
        _scenario(shape, seed, heights)
        for shape in SHAPES
        for heights in (HEIGHTS, SHORT_HEIGHTS)
        for seed in range(200, 200 + holdout_seeds)
    ]
    for row in rows:
        first_fit = row["first_shape_residual"]
        adaptive_ok = row["adaptive_depth_m"] is not None and first_fit is not None and float(first_fit) <= thresholds["shape_residual"]
        # A short vertical baseline amplifies width noise in intercept/slope;
        # require substantial excursion relative to the near-scan estimate.
        enough_vertical_baseline = (
            adaptive_ok and float(row["height_span_m"]) >= 1.5 * float(row["adaptive_depth_m"])
        )
        height_ok = bool(row["complete"]) and enough_vertical_baseline and (
            float(row["shape_spread"]) <= thresholds["shape_spread"]
            and float(row["line_residual_m"]) <= thresholds["line_residual_m"]
        )
        row["adaptive_valid"] = adaptive_ok
        row["height_valid"] = height_ok
    summary: dict[str, object] = {
        "source": "simulation_only", "exploration_seeds": exploration_seeds,
        "holdout_seed_start": 200, "holdout_seeds": holdout_seeds,
        "thresholds_from_power_law_only": thresholds,
        "minimum_height_span_over_near_distance": 1.5,
        "heights_m": list(HEIGHTS), "short_heights_m": list(SHORT_HEIGHTS),
        "cases": {},
    }
    for span in (max(HEIGHTS), max(SHORT_HEIGHTS)):
        summary["cases"][str(span)] = {}
        for shape in SHAPES:
            group = [row for row in rows if row["shape"] == shape and row["height_span_m"] == span]
            summary["cases"][str(span)][shape] = {
                name: _score(group, name) for name in ("static", "adaptive", "height")
            }
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "holdout_per_run.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summary_path = output_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    write_experiment_manifest(
        output_dir, experiment="v2_cross_shape_identifiability",
        config={"exploration_seeds": exploration_seeds, "holdout_seeds": holdout_seeds,
                "shapes": list(SHAPES), "heights_m": list(HEIGHTS),
                "short_heights_m": list(SHORT_HEIGHTS), "thresholds": thresholds,
                "minimum_height_span_over_near_distance": 1.5},
        artifact_paths=[csv_path, summary_path], repository_root=ROOT,
    )
    return summary


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exploration-seeds", type=int, default=80)
    parser.add_argument("--holdout-seeds", type=int, default=100)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "results/v2/cross_shape_identifiability")
    args = parser.parse_args()
    print(json.dumps(run(args.output_dir, args.exploration_seeds, args.holdout_seeds), ensure_ascii=False, indent=2))
