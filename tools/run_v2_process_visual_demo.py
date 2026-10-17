#!/usr/bin/env python3
"""Generate a deterministic, process-level v2 inspection demonstration dataset."""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from auv_mag_tracking.v2 import (  # noqa: E402
    CrossScanConfig,
    DetectionConfig,
    FwhmConfig,
    MagneticDetector,
    SearchConfig,
    TrackingConfig,
    build_cross_scan,
    build_prior_guided_search,
    compute_tracking_correction,
    estimate_fwhm,
    write_experiment_manifest,
)


def sample_polyline(points: np.ndarray, step_m: float) -> np.ndarray:
    samples: list[np.ndarray] = []
    for start, end in zip(points[:-1], points[1:]):
        delta = end - start
        length = float(np.linalg.norm(delta))
        count = max(int(math.ceil(length / step_m)), 1)
        samples.extend(start + min(i * step_m / max(length, 1e-12), 1.0) * delta for i in range(1, count + 1))
    return np.asarray(samples, dtype=float)


def burial_truth(progress_m: np.ndarray | float) -> np.ndarray:
    s = np.asarray(progress_m, dtype=float)
    return 1.35 + 0.22 * np.sin(2.0 * np.pi * s / 70.0) + 0.18 * np.exp(-((s - 68.0) / 9.0) ** 2)


def altitude_truth(progress_m: np.ndarray | float) -> np.ndarray:
    s = np.asarray(progress_m, dtype=float)
    return 2.0 + 0.12 * np.cos(2.0 * np.pi * s / 55.0)


def write_csv(path: Path, records: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader(); writer.writerows(records)


def run(output_dir: Path, seed: int = 24) -> dict[str, object]:
    rng = np.random.default_rng(seed)
    output_dir.mkdir(parents=True, exist_ok=True)
    prior = np.array([[0.0, 0.0], [100.0, 0.0]])
    true_cable_y = 8.0
    search_cfg = SearchConfig(search_width_m=30.0, lane_spacing_m=6.0, velocity_mps=0.7, waypoint_spacing_m=5.0)
    search = build_prior_guided_search(
        prior, search_cfg,
        initial_position_xy_m=np.array([0.0, -15.0]),
        initial_position_uncertainty_m=15.0,
    )
    detector = MagneticDetector(DetectionConfig(
        window_samples=80, min_ready_samples=20, threshold_on=20.0, threshold_off=10.0,
        consecutive_on=3, consecutive_off=6, minimum_noise_nt=0.08,
    ))
    for index in range(30):
        detector.update(index * 0.1, 2.0 + rng.normal(0.0, 0.08))

    search_points = sample_polyline(search.waypoints_xy_m, 0.5)
    search_records: list[dict[str, object]] = []
    detection_point = None
    distance = 0.0
    previous = np.array([0.0, -15.0])
    for point in search_points:
        distance += float(np.linalg.norm(point - previous)); previous = point
        lateral = float(point[1] - true_cable_y)
        finite_support = 1.0 / (1.0 + math.exp(-(float(point[0]) - 12.0) / 3.0))
        envelope = 2.0 + finite_support * 20.0 / (1.0 + (lateral / 3.4) ** 2) + rng.normal(0.0, 0.08)
        detection = detector.update(distance / search_cfg.velocity_mps, envelope)
        search_records.append({
            "sequence": len(search_records), "x_m": point[0], "y_m": point[1],
            "distance_m": distance, "envelope_nt": envelope, "detection_score": detection.score,
            "detected": int(detection.detected),
        })
        if detection.detected:
            detection_point = point.copy(); break
    if detection_point is None:
        raise RuntimeError("deterministic process demonstration did not detect the cable")

    cross_cfg = CrossScanConfig(half_span_levels_m=(4.0, 8.0, 14.0, 20.0), early_complete_when_valid=False)
    profile_cfg = FwhmConfig(
        minimum_samples=31, smoothing_window=5, minimum_prominence_nt=0.25,
        minimum_snr_db=6.0, shape_model="thesis_dipole", shape_factor=0.5,
        altitude_m=float(altitude_truth(detection_point[0])),
        shape_factor_relative_sigma=0.04, altitude_sigma_m=0.08,
    )
    profile_records: list[dict[str, object]] = []
    attempt_summaries: list[dict[str, object]] = []
    final_result = None
    final_plan = None
    current = detection_point.copy()
    for attempt, span in enumerate(cross_cfg.half_span_levels_m):
        plan = build_cross_scan(
            scan_id=attempt + 1, center_xy_m=detection_point, tangent_xy=np.array([1.0, 0.0]),
            current_position_xy_m=current, config=cross_cfg, attempt_index=attempt,
        )
        traverse = np.linspace(plan.waypoints_xy_m[0], plan.waypoints_xy_m[1], 161)
        local_x = (traverse - plan.center_xy_m) @ plan.normal_xy
        progress = float(detection_point[0])
        vertical = float(burial_truth(progress) + altitude_truth(progress))
        lateral_true = traverse[:, 1] - true_cable_y
        measured = 2.0 + 20.0 / (1.0 + (lateral_true / vertical) ** 2) + rng.normal(0.0, 0.08, traverse.shape[0])
        result = estimate_fwhm(
            local_x, measured, profile_cfg, baseline_nt=2.0, noise_nt=0.08,
            altitude_m=float(altitude_truth(progress)),
        )
        for order, (point, x_local, value) in enumerate(zip(traverse, local_x, measured)):
            profile_records.append({
                "attempt": attempt + 1, "half_span_m": span, "sample_order": order,
                "map_x_m": point[0], "map_y_m": point[1], "cross_coordinate_m": x_local,
                "envelope_nt": value, "valid_attempt": int(result.valid),
            })
        attempt_summaries.append({
            "attempt": attempt + 1, "half_span_m": span, "valid": int(result.valid),
            "left_crossing_m": result.left_crossing_x_m, "right_crossing_m": result.right_crossing_x_m,
            "fwhm_m": result.width_m, "center_x_m": result.center_x_m,
            "burial_estimate_m": result.burial_depth_m, "confidence": result.confidence,
            "reason_codes": "|".join(result.reason_codes),
        })
        current = plan.waypoints_xy_m[1]
        if result.valid:
            final_result, final_plan = result, plan
            break
    if final_result is None or final_plan is None or final_result.center_x_m is None:
        raise RuntimeError(
            f"no valid confirmation profile at {detection_point.tolist()}: {attempt_summaries}"
        )
    observed_y = float(detection_point[1] + final_result.center_x_m * final_plan.normal_xy[1])

    station_records: list[dict[str, object]] = []
    observation_records: list[dict[str, object]] = []
    truth_progress = np.linspace(0.0, 100.0, 501)
    truth_records = [
        {
            "progress_m": progress,
            "burial_true_m": float(burial_truth(progress)),
            "altitude_m": float(altitude_truth(progress)),
        }
        for progress in truth_progress
    ]
    for station_x in np.arange(15.0, 96.0, 8.0):
        burial = float(burial_truth(station_x)); altitude = float(altitude_truth(station_x)); vertical = burial + altitude
        scan_center_y = observed_y + rng.normal(0.0, 0.10)
        cross = np.linspace(-3.5 * vertical, 3.5 * vertical, 241)
        map_y = scan_center_y + cross
        measured = 2.0 + 20.0 / (1.0 + ((map_y - true_cable_y) / vertical) ** 2) + rng.normal(0.0, 0.10, cross.size)
        cfg = FwhmConfig(
            minimum_samples=31, smoothing_window=5, minimum_prominence_nt=0.25,
            minimum_snr_db=6.0, shape_model="thesis_dipole", shape_factor=0.5,
            altitude_m=altitude, shape_factor_relative_sigma=0.04, altitude_sigma_m=0.08,
        )
        result = estimate_fwhm(cross, measured, cfg, baseline_nt=2.0, noise_nt=0.10, altitude_m=altitude)
        estimated_y = scan_center_y + (result.center_x_m or 0.0)
        observation_records.append({
            "progress_m": station_x, "estimated_cable_y_m": estimated_y,
            "lateral_sigma_m": max((result.width_sigma_m or 0.2) * 0.5, 0.05),
            "confidence": result.confidence,
        })
        station_records.append({
            "progress_m": station_x, "burial_true_m": burial,
            "burial_estimate_m": result.burial_depth_m,
            "burial_sigma_m": result.burial_depth_sigma_m,
            "altitude_m": altitude, "confidence": result.confidence,
            "valid": int(result.valid), "estimated_cable_y_m": estimated_y,
        })

    estimated_route_y = float(np.average(
        [r["estimated_cable_y_m"] for r in observation_records],
        weights=[max(float(r["confidence"]), 0.05) for r in observation_records],
    ))
    target_x = 96.0 if detection_point[0] < 50.0 else 4.0
    track_x = np.linspace(float(detection_point[0]), target_x, 180)
    vehicle_y = float(detection_point[1])
    track_records: list[dict[str, object]] = []
    tracking_cfg = TrackingConfig()
    for index, x_m in enumerate(track_x):
        measured_offset = vehicle_y - estimated_route_y + rng.normal(0.0, 0.08)
        correction = compute_tracking_correction(measured_offset, 0.93, tracking_cfg)
        vehicle_y += 0.16 * correction.lateral_correction_m + rng.normal(0.0, 0.012)
        track_records.append({
            "sequence": index, "x_m": x_m, "auv_y_m": vehicle_y,
            "prior_y_m": 0.0, "true_cable_y_m": true_cable_y,
            "estimated_route_y_m": estimated_route_y,
            "true_cross_track_error_m": vehicle_y - true_cable_y,
            "estimated_cross_track_error_m": vehicle_y - estimated_route_y,
            "heading_correction_deg": correction.heading_correction_deg,
        })

    paths = {
        "search_trace.csv": search_records,
        "cross_scan_profiles.csv": profile_records,
        "cross_scan_attempts.csv": attempt_summaries,
        "depth_reconstruction.csv": station_records,
        "depth_truth_curve.csv": truth_records,
        "cable_observations.csv": observation_records,
        "tracking_trace.csv": track_records,
    }
    for name, records in paths.items(): write_csv(output_dir / name, records)
    summary = {
        "seed": seed, "prior_cable_y_m": 0.0, "true_cable_y_m": true_cable_y,
        "detection_point_xy_m": detection_point.tolist(),
        "search_distance_to_detection_m": distance,
        "successful_cross_scan_attempt": int(attempt_summaries[-1]["attempt"]),
        "successful_half_span_m": float(attempt_summaries[-1]["half_span_m"]),
        "fwhm_m": final_result.width_m, "left_crossing_m": final_result.left_crossing_x_m,
        "right_crossing_m": final_result.right_crossing_x_m, "half_level_nt": final_result.half_level_nt,
        "baseline_nt": final_result.baseline_nt, "peak_value_nt": final_result.peak_value_nt,
        "burial_true_at_detection_m": float(burial_truth(detection_point[0])),
        "burial_estimate_at_detection_m": final_result.burial_depth_m,
        "burial_sigma_at_detection_m": final_result.burial_depth_sigma_m,
        "observed_cable_y_m": observed_y, "estimated_route_y_m": estimated_route_y,
        "mean_abs_depth_error_m": float(np.mean([abs(float(r["burial_estimate_m"])-float(r["burial_true_m"])) for r in station_records])),
        "final_true_cross_track_error_m": float(track_records[-1]["true_cross_track_error_m"]),
        "claim_boundary": "Deterministic process visualization using the production v2 primitives and a declared synthetic dipole-envelope field. It explains algorithm flow; it is not field accuracy evidence.",
    }
    summary_path = output_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    write_experiment_manifest(
        output_dir, experiment="v2_process_visual_demo",
        config={"seed": seed, "search_width_m": 30.0, "lane_spacing_m": 6.0, "cross_scan_half_spans_m": [4.0, 8.0, 14.0, 20.0]},
        artifact_paths=[*(output_dir / name for name in paths), summary_path], repository_root=ROOT,
    )
    return summary


def main() -> None:
    output = ROOT / "results/v2/process_visual_demo"
    print(json.dumps(run(output), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
