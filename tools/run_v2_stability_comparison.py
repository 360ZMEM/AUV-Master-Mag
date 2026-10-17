#!/usr/bin/env python3
"""Compare v2 spatial FWHM stability with three interpretable baselines."""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import curve_fit

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from auv_mag_tracking.v2 import FwhmConfig, estimate_fwhm, write_experiment_manifest  # noqa: E402

METHODS = ("explicit_fwhm", "free_profile_fit", "calibrated_amplitude", "absolute_half")
SCENARIOS = ("nominal", "gain", "background", "noise", "scan_offset", "combined", "model_mismatch")
NOMINAL_SOURCE = 180.0


def median_filter(values: np.ndarray, window: int = 5) -> np.ndarray:
    padded = np.pad(values, (window // 2, window // 2), mode="edge")
    return np.asarray([np.median(padded[i:i + window]) for i in range(values.size)])


def crossing(x: np.ndarray, y: np.ndarray, peak: int, level: float, side: str) -> float | None:
    indices = range(peak - 1, -1, -1) if side == "left" else range(peak, y.size - 1)
    for idx in indices:
        lo, hi = idx, idx + 1
        if (y[lo] - level) * (y[hi] - level) <= 0.0 and y[hi] != y[lo]:
            fraction = (level - y[lo]) / (y[hi] - y[lo])
            return float(x[lo] + fraction * (x[hi] - x[lo]))
    return None


def absolute_half_estimate(x: np.ndarray, y: np.ndarray) -> tuple[bool, float | None]:
    smooth = median_filter(y)
    peak = int(np.argmax(smooth)); level = 0.5 * float(smooth[peak])
    left = crossing(x, smooth, peak, level, "left"); right = crossing(x, smooth, peak, level, "right")
    if left is None or right is None or right <= left:
        return False, None
    return True, 0.5 * (right - left)


def profile_fit_estimate(x: np.ndarray, y: np.ndarray) -> tuple[bool, float | None]:
    edge = max(int(0.12 * x.size), 3)
    background = float(np.median(np.r_[y[:edge], y[-edge:]]))
    peak = int(np.argmax(y)); amplitude = max(float(y[peak] - background), 0.1)

    def model(xx, base, amp, center, separation):
        return base + amp / (1.0 + ((xx - center) / separation) ** 2)

    try:
        params, covariance = curve_fit(
            model, x, y,
            p0=(background, amplitude, float(x[peak]), max(float(np.ptp(x)) / 6.0, 0.5)),
            bounds=([-20.0, 0.0, float(x[0]), 0.15], [40.0, 200.0, float(x[-1]), 20.0]),
            maxfev=3000,
        )
        separation = float(params[3])
        valid = bool(np.isfinite(separation) and np.all(np.isfinite(covariance)) and 0.15 < separation < 19.9)
        return valid, separation if valid else None
    except (RuntimeError, ValueError, FloatingPointError):
        return False, None


def scenario_parameters(name: str, rng: np.random.Generator, z: float) -> dict[str, float | str]:
    params: dict[str, float | str] = {
        "gain": 1.0, "baseline": 2.0, "slope": 0.0, "noise": 0.05,
        "center": 0.0, "half_span": 3.5 * z, "model": "dipole",
    }
    if name == "gain":
        params["gain"] = float(np.exp(rng.uniform(np.log(0.45), np.log(2.2))))
        params["noise"] = 0.08
    elif name == "background":
        params["baseline"] = float(rng.uniform(0.0, 10.0)); params["slope"] = float(rng.uniform(-0.05, 0.05))
        params["noise"] = 0.08
    elif name == "noise":
        params["noise"] = float(np.exp(rng.uniform(np.log(0.03), np.log(1.0))))
    elif name == "scan_offset":
        params["center"] = float(rng.uniform(-2.0 * z, 2.0 * z)); params["half_span"] = 2.5 * z
        params["noise"] = 0.08
    elif name == "combined":
        params.update({
            "gain": float(np.exp(rng.uniform(np.log(0.45), np.log(2.2)))),
            "baseline": float(rng.uniform(0.0, 10.0)), "slope": float(rng.uniform(-0.05, 0.05)),
            "noise": float(np.exp(rng.uniform(np.log(0.03), np.log(0.8)))),
            "center": float(rng.uniform(-1.7 * z, 1.7 * z)), "half_span": float(rng.uniform(2.2, 3.5) * z),
        })
    elif name == "model_mismatch":
        params["model"] = "single_wire"; params["gain"] = float(rng.uniform(0.8, 1.2)); params["noise"] = 0.08
    return params


def one_profile(scenario: str, seed: int) -> list[dict[str, object]]:
    rng = np.random.default_rng(100_000 * SCENARIOS.index(scenario) + seed)
    z_true = float(rng.uniform(2.5, 5.0)); p = scenario_parameters(scenario, rng, z_true)
    half_span = float(p["half_span"]); x = np.linspace(-half_span, half_span, 241)
    center = float(p["center"]); gain = float(p["gain"]); baseline = float(p["baseline"])
    if p["model"] == "single_wire":
        anomaly = gain * NOMINAL_SOURCE / z_true / np.sqrt((x - center) ** 2 + z_true ** 2)
    else:
        anomaly = gain * NOMINAL_SOURCE / ((x - center) ** 2 + z_true ** 2)
    clean = baseline + float(p["slope"]) * x + anomaly
    y = clean + rng.normal(0.0, float(p["noise"]), x.size)

    fwhm = estimate_fwhm(
        x, y,
        FwhmConfig(
            minimum_samples=31, smoothing_window=5, edge_fraction=0.12,
            minimum_prominence_nt=0.25, minimum_snr_db=6.0, minimum_side_samples=3,
            minimum_coverage_factor=0.10, maximum_model_residual=0.35,
            minimum_validity_score=0.50, shape_model="thesis_dipole", shape_factor=0.5,
            altitude_m=0.0, shape_factor_relative_sigma=0.0, altitude_sigma_m=0.0,
        ),
        baseline_nt=baseline, noise_nt=float(p["noise"]), altitude_m=0.0,
    )
    methods: dict[str, tuple[bool, float | None, float | None]] = {
        "explicit_fwhm": (fwhm.valid, fwhm.vertical_separation_m, fwhm.vertical_separation_sigma_m),
        "free_profile_fit": (*profile_fit_estimate(x, y), None),
        "calibrated_amplitude": (True, math.sqrt(NOMINAL_SOURCE / max(float(np.max(median_filter(y)) - baseline), 1e-9)), None),
        "absolute_half": (*absolute_half_estimate(x, y), None),
    }
    records = []
    for method, (valid, estimate, sigma) in methods.items():
        error = None if estimate is None else float(estimate - z_true)
        records.append({
            "scenario": scenario, "method": method, "seed": seed, "valid": int(valid),
            "z_true_m": z_true, "z_estimate_m": estimate, "error_m": error,
            "abs_error_m": None if error is None else abs(error), "reported_sigma_m": sigma,
            "gain_scale": p["gain"], "baseline_nt": p["baseline"], "background_slope_ntpm": p["slope"],
            "noise_nt": p["noise"], "scan_center_offset_m": p["center"], "scan_half_span_m": p["half_span"],
            "data_model": p["model"], "reason_codes": "|".join(fwhm.reason_codes) if method == "explicit_fwhm" else "",
        })
    return records


def summarize(records: list[dict[str, object]], seeds: int) -> dict[str, object]:
    summary: dict[str, object] = {"seeds_per_scenario": seeds, "scenarios": {}}
    for scenario in SCENARIOS:
        methods = {}
        for method in METHODS:
            group = [r for r in records if r["scenario"] == scenario and r["method"] == method]
            published = [r for r in group if int(r["valid"]) and r["abs_error_m"] is not None]
            errors = np.asarray([float(r["error_m"]) for r in published])
            absolute = np.abs(errors)
            methods[method] = {
                "runs": len(group), "valid_rate": float(np.mean([int(r["valid"]) for r in group])),
                "mae_m_published": None if not published else float(np.mean(absolute)),
                "rmse_m_published": None if not published else float(np.sqrt(np.mean(errors ** 2))),
                "q90_abs_error_m_published": None if not published else float(np.percentile(absolute, 90.0)),
                "error_std_m_published": None if not published else float(np.std(errors)),
                "large_error_publish_rate": float(np.mean([int(r["valid"]) and r["abs_error_m"] is not None and float(r["abs_error_m"]) > 0.5 for r in group])),
            }
        summary["scenarios"][scenario] = methods
    return summary


def run(output_dir: Path, seeds: int) -> dict[str, object]:
    records = [record for scenario in SCENARIOS for seed in range(seeds) for record in one_profile(scenario, seed)]
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "per_run.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0])); writer.writeheader(); writer.writerows(records)
    summary = summarize(records, seeds)
    summary_path = output_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    write_experiment_manifest(
        output_dir, experiment="v2_fwhm_stability_comparison",
        config={
            "seeds_per_scenario": seeds, "seed_sequence": list(range(seeds)),
            "methods": list(METHODS), "scenarios": list(SCENARIOS),
            "large_error_threshold_m": 0.5, "nominal_source_nt_m2": NOMINAL_SOURCE,
        },
        artifact_paths=[csv_path, summary_path], repository_root=ROOT,
    )
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, default=200)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "results/v2/stability_comparison")
    args = parser.parse_args()
    if args.seeds < 1: raise SystemExit("--seeds must be positive")
    print(json.dumps(run(args.output_dir, args.seeds), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
