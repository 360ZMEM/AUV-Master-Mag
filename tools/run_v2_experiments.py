"""Run reproducible v2 Cases A-D without touching historical v1 results."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.v2 import (  # noqa: E402
    DetectionConfig,
    FwhmConfig,
    MagneticDetector,
    SearchConfig,
    TrackingConfig,
    build_prior_guided_search,
    compute_tracking_correction,
    estimate_fwhm,
    write_experiment_manifest,
)
from auv_mag_tracking.v2.metrics import error_metrics  # noqa: E402


def _sample_path(waypoints: np.ndarray, step_m: float = 0.5):
    distance = 0.0
    for start, end in zip(waypoints[:-1], waypoints[1:]):
        delta = end - start
        length = float(np.linalg.norm(delta))
        count = max(int(math.ceil(length / step_m)), 1)
        for index in range(1, count + 1):
            point = start + min(index * step_m / max(length, 1e-12), 1.0) * delta
            increment = min(step_m, max(length - (index - 1) * step_m, 0.0))
            distance += increment
            yield point, distance


def _search_run(
    *,
    seed: int,
    true_cable_offset_m: float | None,
    lane_spacing_m: float,
    noise_nt: float,
) -> dict[str, object]:
    rng = np.random.default_rng(seed)
    prior = np.array([[0.0, 0.0], [100.0, 0.0]])
    config = SearchConfig(
        search_width_m=30.0,
        lane_spacing_m=lane_spacing_m,
        velocity_mps=0.7,
        waypoint_spacing_m=5.0,
    )
    plan = build_prior_guided_search(
        prior,
        config,
        initial_position_xy_m=np.array([0.0, -15.0]),
        initial_position_uncertainty_m=15.0,
    )
    detector = MagneticDetector(
        DetectionConfig(
            window_samples=80,
            min_ready_samples=20,
            threshold_on=5.0,
            threshold_off=2.5,
            consecutive_on=3,
            consecutive_off=6,
            minimum_noise_nt=max(noise_nt, 0.05),
        )
    )
    time_s = 0.0
    for _ in range(30):
        detector.update(time_s, 1.0 + rng.normal(0.0, noise_nt))
        time_s += 0.1
    detected = False
    detection_distance = plan.estimated_distance_m
    detection_time = None
    for point, distance in _sample_path(plan.waypoints_xy_m):
        if true_cable_offset_m is None:
            envelope = 1.0 + rng.normal(0.0, noise_nt)
        else:
            lateral = float(point[1] - true_cable_offset_m)
            envelope = 1.0 + 12.0 / (1.0 + (lateral / 2.5) ** 2) + rng.normal(0.0, noise_nt)
        time_s = distance / config.velocity_mps
        result = detector.update(time_s, envelope)
        if result.detected:
            detected = True
            detection_distance = distance
            detection_time = time_s
            break
    return {
        "seed": seed,
        "true_cable_offset_m": true_cable_offset_m,
        "lane_spacing_m": lane_spacing_m,
        "noise_nt": noise_nt,
        "detected": int(detected),
        "detection_time_s": detection_time,
        "travelled_distance_m": detection_distance,
        "planned_distance_m": plan.estimated_distance_m,
    }


def _fwhm_run(*, seed: int, burial_m: float, noise_nt: float, half_span_m: float) -> dict[str, object]:
    rng = np.random.default_rng(seed)
    altitude_m = 2.0
    vertical_m = altitude_m + burial_m
    x_m = np.linspace(-half_span_m, half_span_m, 241)
    baseline_nt = 2.0
    amplitude_nt = 20.0
    clean = baseline_nt + amplitude_nt / (1.0 + (x_m / vertical_m) ** 2)
    measured = clean + rng.normal(0.0, noise_nt, size=x_m.size)
    result = estimate_fwhm(
        x_m,
        measured,
        FwhmConfig(
            minimum_samples=31,
            smoothing_window=5,
            minimum_prominence_nt=0.25,
            minimum_snr_db=6.0,
            shape_factor=0.5,
            altitude_m=altitude_m,
        ),
        baseline_nt=baseline_nt,
        noise_nt=noise_nt,
    )
    return {
        "seed": seed,
        "burial_true_m": burial_m,
        "vertical_true_m": vertical_m,
        "noise_nt": noise_nt,
        "half_span_m": half_span_m,
        "fwhm_valid": int(result.valid),
        "fwhm_width_m": result.width_m,
        "burial_estimate_m": result.burial_depth_m,
        "burial_error_m": None if result.burial_depth_m is None else result.burial_depth_m - burial_m,
        "validity_score": result.validity_score,
        "confidence": result.confidence,
        "reason_codes": "|".join(result.reason_codes),
    }


def _tracking_run(seed: int, initial_offset_m: float, measurement_noise_m: float) -> dict[str, object]:
    rng = np.random.default_rng(seed)
    config = TrackingConfig()
    offset = float(initial_offset_m)
    errors = []
    for _ in range(200):
        measured = offset + rng.normal(0.0, measurement_noise_m)
        correction = compute_tracking_correction(measured, 0.85, config)
        offset += 0.15 * correction.lateral_correction_m
        errors.append(offset)
    tail = np.asarray(errors[50:], dtype=float)
    return {
        "seed": seed,
        "initial_offset_m": initial_offset_m,
        "measurement_noise_m": measurement_noise_m,
        "tracking_rmse_m": float(np.sqrt(np.mean(tail ** 2))),
        "tracking_q95_abs_m": float(np.percentile(np.abs(tail), 95.0)),
        "tracking_final_abs_m": abs(float(offset)),
    }


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def run(output_dir: Path, seeds: int = 20) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    case_a = [
        _search_run(seed=seed, true_cable_offset_m=offset, lane_spacing_m=6.0, noise_nt=0.08)
        for offset in (-12.0, -8.0, 0.0, 8.0, 12.0)
        for seed in range(seeds)
    ]
    case_b = [
        _fwhm_run(
            seed=seed,
            burial_m=burial,
            noise_nt=0.08,
            half_span_m=3.5 * (2.0 + burial),
        )
        for burial in (0.5, 1.0, 1.5, 2.0, 3.0)
        for seed in range(seeds)
    ]
    case_c = [
        _fwhm_run(seed=seed, burial_m=1.5, noise_nt=noise, half_span_m=12.25)
        for noise in (0.02, 0.05, 0.1, 0.25, 0.5, 1.0)
        for seed in range(seeds)
    ]
    case_d = [
        _search_run(seed=seed, true_cable_offset_m=8.0, lane_spacing_m=spacing, noise_nt=0.08)
        for spacing in (2.0, 4.0, 6.0, 8.0, 12.0)
        for seed in range(seeds)
    ]
    no_target = [
        _search_run(seed=seed, true_cable_offset_m=None, lane_spacing_m=6.0, noise_nt=0.08)
        for seed in range(seeds)
    ]
    tracking = [
        _tracking_run(seed, initial_offset_m=offset, measurement_noise_m=0.15)
        for offset in (-5.0, -2.0, 2.0, 5.0)
        for seed in range(seeds)
    ]
    _write_csv(output_dir / "case_a_initial_uncertainty.csv", case_a)
    _write_csv(output_dir / "case_b_burial_depth.csv", case_b)
    _write_csv(output_dir / "case_c_noise.csv", case_c)
    _write_csv(output_dir / "case_d_lane_spacing.csv", case_d)
    _write_csv(output_dir / "no_target_false_alarm.csv", no_target)
    _write_csv(output_dir / "tracking.csv", tracking)

    def search_summary(rows: list[dict[str, object]]) -> dict[str, float]:
        detected = np.asarray([int(row["detected"]) for row in rows], dtype=float)
        times = np.asarray(
            [float(row["detection_time_s"]) for row in rows if row["detection_time_s"] is not None]
        )
        distances = np.asarray([float(row["travelled_distance_m"]) for row in rows])
        return {
            "runs": len(rows),
            "detection_probability": float(np.mean(detected)),
            "mean_detection_time_s_detected_only": float(np.mean(times)) if times.size else float("nan"),
            "mean_travelled_distance_m": float(np.mean(distances)),
        }

    def fwhm_summary(rows: list[dict[str, object]]) -> dict[str, float]:
        valid = np.asarray([int(row["fwhm_valid"]) for row in rows], dtype=float)
        estimate = np.asarray(
            [float("nan") if row["burial_estimate_m"] is None else float(row["burial_estimate_m"]) for row in rows]
        )
        truth = np.asarray([float(row["burial_true_m"]) for row in rows])
        metrics = error_metrics(estimate, truth)
        return {
            "runs": len(rows),
            "valid_rate": float(np.mean(valid)),
            "burial_bias_m": metrics.bias,
            "burial_mae_m": metrics.mae,
            "burial_rmse_m": metrics.rmse,
            "burial_q90_abs_m": metrics.q90_abs,
        }

    def grouped(rows: list[dict[str, object]], key: str, summarizer) -> dict[str, dict[str, float]]:
        values = sorted({row[key] for row in rows}, key=lambda value: float(value))
        return {
            str(value): summarizer([row for row in rows if row[key] == value])
            for value in values
        }

    tracking_rmse = np.asarray([float(row["tracking_rmse_m"]) for row in tracking])
    summary = {
        "seeds_per_level": seeds,
        "case_a": search_summary(case_a),
        "case_a_by_offset_m": grouped(case_a, "true_cable_offset_m", search_summary),
        "case_b": fwhm_summary(case_b),
        "case_b_by_burial_m": grouped(case_b, "burial_true_m", fwhm_summary),
        "case_c": fwhm_summary(case_c),
        "case_c_by_noise_nt": grouped(case_c, "noise_nt", fwhm_summary),
        "case_d": search_summary(case_d),
        "case_d_by_lane_spacing_m": grouped(case_d, "lane_spacing_m", search_summary),
        "no_target": {
            "runs": len(no_target),
            "false_alarm_probability": float(np.mean([int(row["detected"]) for row in no_target])),
        },
        "tracking": {
            "runs": len(tracking),
            "mean_rmse_m": float(np.mean(tracking_rmse)),
            "q90_rmse_m": float(np.percentile(tracking_rmse, 90.0)),
        },
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=True) + "\n",
        encoding="utf-8",
    )
    write_experiment_manifest(
        output_dir,
        experiment="v2_cases_a_d",
        config={
            "seeds_per_level": seeds,
            "case_a_offsets_m": [-12.0, -8.0, 0.0, 8.0, 12.0],
            "case_b_burial_m": [0.5, 1.0, 1.5, 2.0, 3.0],
            "case_c_noise_nt": [0.02, 0.05, 0.1, 0.25, 0.5, 1.0],
            "case_d_lane_spacing_m": [2.0, 4.0, 6.0, 8.0, 12.0],
        },
        artifact_paths=[*output_dir.glob("*.csv"), output_dir / "summary.json"],
        repository_root=PROJECT_ROOT,
    )
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=PROJECT_ROOT / "results" / "v2" / "cases_a_d",
    )
    parser.add_argument("--seeds", type=int, default=20)
    args = parser.parse_args()
    if args.seeds < 1:
        raise SystemExit("--seeds must be positive")
    summary = run(args.output_dir, args.seeds)
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=True))


if __name__ == "__main__":
    main()
