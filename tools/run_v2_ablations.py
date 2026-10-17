#!/usr/bin/env python3
"""Run the predeclared v2 ablations with fixed synthetic seeds.

These experiments isolate implementation choices; they are not PVS or sea-trial
claims.  Every comparison uses the same generated samples within a seed.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))

from auv_mag_tracking.v2 import FwhmConfig, estimate_fwhm, write_experiment_manifest  # noqa: E402


def _profile(x: np.ndarray, z_m: float, *, baseline: float, amplitude: float) -> np.ndarray:
    return baseline + amplitude / (1.0 + (x / z_m) ** 2)


def _config(*, altitude_m: float = 0.0) -> FwhmConfig:
    return FwhmConfig(
        minimum_samples=31,
        smoothing_window=5,
        edge_fraction=0.12,
        minimum_prominence_nt=0.2,
        minimum_snr_db=3.0,
        minimum_side_samples=3,
        minimum_coverage_factor=0.10,
        maximum_model_residual=0.40,
        minimum_validity_score=0.45,
        shape_model="thesis_dipole",
        shape_factor=0.5,
        altitude_m=altitude_m,
    )


def _naive_absolute_half_width(x: np.ndarray, y: np.ndarray) -> float | None:
    order = np.argsort(x)
    x, y = x[order], y[order]
    peak = int(np.argmax(y))
    level = 0.5 * float(y[peak])

    def crossing(start: int, stop: int, step: int) -> float | None:
        for idx in range(start, stop, step):
            left, right = (idx, idx + 1) if step < 0 else (idx, idx + 1)
            if left < 0 or right >= y.size:
                continue
            if (y[left] - level) * (y[right] - level) <= 0.0 and y[right] != y[left]:
                fraction = (level - y[left]) / (y[right] - y[left])
                return float(x[left] + fraction * (x[right] - x[left]))
        return None

    left = crossing(peak - 1, -1, -1)
    right = crossing(peak, y.size - 1, 1)
    return None if left is None or right is None else right - left


def _spatial_time(seed: int) -> list[dict[str, object]]:
    rng = np.random.default_rng(seed)
    t = np.linspace(0.0, 30.0, 301)
    phase = rng.uniform(0.0, 2.0 * math.pi)
    speed = 0.65 * (1.0 + 0.55 * np.sin(2.0 * math.pi * t / 30.0 + phase))
    cumulative = np.r_[0.0, np.cumsum(0.5 * (speed[1:] + speed[:-1]) * np.diff(t))]
    x = -9.0 + 18.0 * cumulative / cumulative[-1]
    z_true = 3.0
    y = _profile(x, z_true, baseline=2.0, amplitude=20.0) + rng.normal(0.0, 0.08, x.size)
    spatial = estimate_fwhm(x, y, _config(), baseline_nt=2.0, noise_nt=0.08)
    temporal = estimate_fwhm(t, y, _config(), baseline_nt=2.0, noise_nt=0.08)
    temporal_z = None if temporal.width_m is None else 0.5 * temporal.width_m * float(np.mean(speed))
    return [
        {"comparison": "spatial_vs_time", "method": "spatial_navigation", "seed": seed,
         "valid": int(spatial.valid), "estimate": spatial.vertical_separation_m,
         "truth": z_true, "abs_error": None if spatial.vertical_separation_m is None else abs(spatial.vertical_separation_m-z_true)},
        {"comparison": "spatial_vs_time", "method": "mean_speed_times_time_width", "seed": seed,
         "valid": int(temporal.valid), "estimate": temporal_z, "truth": z_true,
         "abs_error": None if temporal_z is None else abs(temporal_z-z_true)},
    ]


def _background(seed: int) -> list[dict[str, object]]:
    rng = np.random.default_rng(seed)
    x = np.linspace(-12.0, 12.0, 401)
    z_true = 3.0
    y = _profile(x, z_true, baseline=6.0, amplitude=18.0) + rng.normal(0.0, 0.06, x.size)
    corrected = estimate_fwhm(x, y, _config(), baseline_nt=6.0, noise_nt=0.06)
    naive_width = _naive_absolute_half_width(x, y)
    naive_z = None if naive_width is None else 0.5 * naive_width
    return [
        {"comparison": "background_half_level", "method": "background_corrected", "seed": seed,
         "valid": int(corrected.valid), "estimate": corrected.vertical_separation_m, "truth": z_true,
         "abs_error": None if corrected.vertical_separation_m is None else abs(corrected.vertical_separation_m-z_true)},
        {"comparison": "background_half_level", "method": "absolute_peak_half", "seed": seed,
         "valid": int(naive_z is not None), "estimate": naive_z, "truth": z_true,
         "abs_error": None if naive_z is None else abs(naive_z-z_true)},
    ]


def _trajectory_and_span(seed: int) -> list[dict[str, object]]:
    rng = np.random.default_rng(seed)
    z_true = 3.5
    rows = []
    for method, x in (
        ("segmented_linear", np.linspace(-3.0*z_true, 3.0*z_true, 241)),
        ("sine_half_cycle", 3.0*z_true*np.sin(np.linspace(-math.pi/2, math.pi/2, 241))),
        ("fixed_1p5m", np.linspace(-1.5, 1.5, 241)),
        ("adaptive_3z", np.linspace(-3.0*z_true, 3.0*z_true, 241)),
    ):
        y = _profile(x, z_true, baseline=2.0, amplitude=20.0) + rng.normal(0.0, 0.10, x.size)
        result = estimate_fwhm(x, y, _config(), baseline_nt=2.0, noise_nt=0.10)
        comparison = "cross_scan_trajectory" if method in {"segmented_linear", "sine_half_cycle"} else "fixed_vs_adaptive_span"
        rows.append({"comparison": comparison, "method": method, "seed": seed,
                     "valid": int(result.valid), "estimate": result.vertical_separation_m,
                     "truth": z_true, "abs_error": None if result.vertical_separation_m is None else abs(result.vertical_separation_m-z_true)})
    return rows


def _inversion_fusion(seed: int) -> list[dict[str, object]]:
    rng = np.random.default_rng(seed)
    z_true = 3.0
    scale = rng.uniform(0.45, 2.2)
    x = np.linspace(-10.0, 10.0, 301)
    y = _profile(x, z_true, baseline=2.0, amplitude=20.0*scale) + rng.normal(0.0, 0.08, x.size)
    fwhm = estimate_fwhm(x, y, _config(), baseline_nt=2.0, noise_nt=0.08)
    fwhm_z = fwhm.vertical_separation_m
    amplitude_z = z_true / math.sqrt(scale)
    consistent = fwhm_z is not None and abs(fwhm_z-amplitude_z) <= 0.30
    fused_z = None if fwhm_z is None else (0.8*fwhm_z+0.2*amplitude_z if consistent else fwhm_z)
    return [
        {"comparison": "inversion_fusion", "method": "fwhm_only", "seed": seed, "valid": int(fwhm_z is not None), "estimate": fwhm_z, "truth": z_true, "abs_error": None if fwhm_z is None else abs(fwhm_z-z_true)},
        {"comparison": "inversion_fusion", "method": "amplitude_only", "seed": seed, "valid": 1, "estimate": amplitude_z, "truth": z_true, "abs_error": abs(amplitude_z-z_true)},
        {"comparison": "inversion_fusion", "method": "consistency_gated_fusion", "seed": seed, "valid": int(fused_z is not None), "estimate": fused_z, "truth": z_true, "abs_error": None if fused_z is None else abs(fused_z-z_true)},
    ]


def _position_sources(seed: int) -> list[dict[str, object]]:
    rng = np.random.default_rng(seed)
    truth = 2.0
    prior = 5.0
    observation = truth + rng.normal(0.0, 0.30)
    prior_sigma, observation_sigma = 3.0, 0.30
    fused = (prior/prior_sigma**2 + observation/observation_sigma**2) / (1/prior_sigma**2 + 1/observation_sigma**2)
    return [
        {"comparison": "position_source", "method": "prior_projection", "seed": seed, "valid": 1, "estimate": prior, "truth": truth, "abs_error": abs(prior-truth)},
        {"comparison": "position_source", "method": "observation_only", "seed": seed, "valid": 1, "estimate": observation, "truth": truth, "abs_error": abs(observation-truth)},
        {"comparison": "position_source", "method": "variance_fused", "seed": seed, "valid": 1, "estimate": fused, "truth": truth, "abs_error": abs(fused-truth)},
    ]


def _hysteresis_and_repeat(seed: int) -> list[dict[str, object]]:
    rng = np.random.default_rng(seed)
    score = np.r_[rng.normal(4.8, 0.55, 100), rng.normal(7.2, 0.55, 30), rng.normal(2.0, 0.4, 60)]
    naive = score >= 5.0
    schmitt = np.zeros(score.size, dtype=bool)
    state = False
    for idx, value in enumerate(score):
        state = bool(value >= 6.0) if not state else bool(value >= 3.0)
        schmitt[idx] = state
    naive_transitions = int(np.count_nonzero(naive[1:] != naive[:-1]))
    schmitt_transitions = int(np.count_nonzero(schmitt[1:] != schmitt[:-1]))
    truth_width = 6.0
    widths = truth_width + rng.normal(0.0, 0.10, 3)
    widths[rng.integers(0, 3)] += 1.5
    single = float(widths[0])
    repeated = float(np.median(widths))
    return [
        {"comparison": "detection_hysteresis", "method": "single_threshold", "seed": seed, "valid": 1, "estimate": naive_transitions, "truth": 2.0, "abs_error": abs(naive_transitions-2)},
        {"comparison": "detection_hysteresis", "method": "schmitt_hysteresis", "seed": seed, "valid": 1, "estimate": schmitt_transitions, "truth": 2.0, "abs_error": abs(schmitt_transitions-2)},
        {"comparison": "repeat_consistency", "method": "single_scan", "seed": seed, "valid": 1, "estimate": single, "truth": truth_width, "abs_error": abs(single-truth_width)},
        {"comparison": "repeat_consistency", "method": "three_scan_median", "seed": seed, "valid": 1, "estimate": repeated, "truth": truth_width, "abs_error": abs(repeated-truth_width)},
    ]


def run(output_dir: Path, seeds: int) -> dict[str, object]:
    rows: list[dict[str, object]] = []
    for seed in range(seeds):
        rows.extend(_spatial_time(seed))
        rows.extend(_background(seed))
        rows.extend(_trajectory_and_span(seed))
        rows.extend(_inversion_fusion(seed))
        rows.extend(_position_sources(seed))
        rows.extend(_hysteresis_and_repeat(seed))
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "per_run.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    summary: dict[str, object] = {"seeds": seeds, "comparisons": {}}
    for comparison in sorted({str(row["comparison"]) for row in rows}):
        methods = {}
        selected = [row for row in rows if row["comparison"] == comparison]
        for method in sorted({str(row["method"]) for row in selected}):
            group = [row for row in selected if row["method"] == method]
            errors = np.asarray([float(row["abs_error"]) for row in group if row["abs_error"] is not None])
            methods[method] = {
                "runs": len(group),
                "valid_rate": float(np.mean([int(row["valid"]) for row in group])),
                "mean_abs_error": None if not errors.size else float(np.mean(errors)),
                "q90_abs_error": None if not errors.size else float(np.percentile(errors, 90.0)),
            }
        summary["comparisons"][comparison] = methods
    summary_path = output_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    write_experiment_manifest(
        output_dir,
        experiment="v2_predeclared_ablations",
        config={"seeds": seeds, "seed_sequence": list(range(seeds))},
        artifact_paths=[csv_path, summary_path],
        repository_root=PACKAGE_ROOT,
    )
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, default=50)
    parser.add_argument("--output-dir", type=Path, default=PACKAGE_ROOT/"results/v2/ablations")
    args = parser.parse_args()
    if args.seeds < 1:
        raise SystemExit("--seeds must be positive")
    print(json.dumps(run(args.output_dir, args.seeds), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
