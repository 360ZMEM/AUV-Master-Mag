#!/usr/bin/env python3
"""Controlled inspection workflow audit with observation-only inputs and turn dynamics."""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from auv_mag_tracking.api import CableMap, MagneticInput, NavigationInput  # noqa: E402
from auv_mag_tracking.v2 import (  # noqa: E402
    AuvMagTrackingPipelineV2, CrossScanConfig, DetectionConfig, FwhmConfig,
    InspectionConfig, SearchConfig, TrackingConfig, V2Config, write_experiment_manifest,
)

CASES = ("dipole", "wire", "intermediate", "map_slope", "magnetic_dropout", "navigation_dropout",
         "no_source", "persistent_magnetic_fault")


def config() -> V2Config:
    return V2Config(
        search=SearchConfig(search_width_m=30, lane_spacing_m=12, velocity_mps=0.7, pattern="transverse"),
        detection=DetectionConfig(window_samples=80, min_ready_samples=20, minimum_noise_nt=0.08),
        cross_scan=CrossScanConfig(half_span_levels_m=(5., 9., 14.), max_attempts=3,
                                  maximum_along_track_range_m=5., return_tolerance_m=0.8),
        fwhm=FwhmConfig(shape_model="thesis_dipole", shape_factor=0.5, altitude_m=3.),
        tracking=TrackingConfig(route_lookahead_m=6., rejoin_tolerance_m=1., endpoint_tolerance_m=1.5),
        inspection=InspectionConfig(enabled=True, maximum_observation_distance_m=18., maximum_observation_age_s=120.),
    )


def run_case(case: str, seed: int, output_dir: Path, maximum_time_s: float = 1500.) -> dict:
    rng = np.random.default_rng(seed)
    setting = config()
    boundary = np.array([[-30., -30.], [100., -30.], [100., 36.], [-30., 36.]])
    pipeline = AuvMagTrackingPipelineV2(
        object(), CableMap(np.array([[0., 0.], [60., 0.]])),
        v2_config=setting, search_boundary_xy_m=boundary,
    )
    slope = 0.05 if case == "map_slope" else 0.0
    exponent = 0.5 if case == "wire" else (0.7 if case == "intermediate" else 1.0)
    position = np.array([0., -10.])
    heading, speed = 0.0, 0.0
    dt = 0.25
    records = []
    first_track = None
    fault_seen = False
    recovered = False
    completed = False
    track_errors = []
    scan_ids = set()
    step_times_ms = []
    started = time.perf_counter()
    for index in range(int(maximum_time_s / dt)):
        t = index * dt
        true_cable_y = 6.0 + slope * position[0]
        lateral = (position[1] - true_cable_y) / math.sqrt(1 + slope ** 2)
        value = 1.0 + 20.0 / (1 + (lateral / 3.0) ** 2) ** exponent + rng.normal(0, 0.03)
        if case == "no_source":
            value = 1.0 + rng.normal(0, 0.03)
        if case == "persistent_magnetic_fault":
            value = float("nan")
        reported = np.array([position[0], position[1], -10.])
        fault = first_track is not None and first_track + 8 <= t < first_track + 23
        if fault and case == "magnetic_dropout":
            value = float("nan")
            fault_seen = True
        if fault and case == "navigation_dropout":
            reported[:] = float("nan")
            fault_seen = True
        navigation = NavigationInput(t, reported, heading, speed_mps=speed, position_std_m=0.1)
        magnetic = MagneticInput(t, np.tile([value, 0, 0], (4, 1)), 50.)
        tick_start = time.perf_counter()
        tracking, guidance = pipeline.step_with_guidance(navigation, magnetic, target_depth_m=10.)
        step_times_ms.append(1000 * (time.perf_counter() - tick_start))
        finite_command = np.isfinite([guidance.desired_heading_deg, guidance.target_depth_m, guidance.speed_mps]).all()
        if not finite_command or not 0 <= guidance.speed_mps <= setting.inspection.maximum_guidance_speed_mps:
            raise AssertionError(f"unsafe guidance at {case}/{seed}/{t}")
        if tracking.burial_depth_m is not None or tracking.diagnostics.get("depth_valid"):
            raise AssertionError("uncalibrated depth escaped the publication gate")
        json.dumps(tracking.diagnostics, allow_nan=False)
        if tracking.mode == "track" and first_track is None:
            first_track = t
        if fault_seen and not fault and tracking.mode == "track":
            recovered = True
        if tracking.mode == "track" and tracking.diagnostics.get("tracking_stage") == "inspect_along_cable":
            track_errors.append(abs(lateral))
        scan_id = tracking.diagnostics.get("cross_scan_id")
        if scan_id is not None:
            scan_ids.add(scan_id)
        records.append({
            "time_s": t, "x_m": position[0], "y_m": position[1], "true_cable_y_m": true_cable_y,
            "mode": tracking.mode, "stage": tracking.diagnostics.get("tracking_stage", "hold"),
            "lateral_valid": bool(tracking.diagnostics.get("lateral_valid")),
            "depth_valid": bool(tracking.diagnostics.get("depth_valid")),
            "cable_estimate_y_m": tracking.estimated_cable_xy_m[1],
            "speed_command_mps": guidance.speed_mps, "actual_speed_mps": speed,
            "heading_deg": heading, "cross_track_truth_m": lateral,
            "scan_id": scan_id, "reason": tracking.diagnostics.get("continuity_reason", ""),
            "fwhm_reasons": "|".join(tracking.diagnostics.get("fwhm_reason_codes", [])),
            "scan_center_strategy": tracking.diagnostics.get("cross_scan_center_strategy", ""),
            "scan_sample_count": tracking.diagnostics.get("cross_scan_sample_count", 0),
        })
        if tracking.diagnostics.get("endpoint_complete"):
            completed = True
            break
        error_heading = (guidance.desired_heading_deg - heading + 180) % 360 - 180
        heading += float(np.clip(error_heading, -12 * dt, 12 * dt))
        speed += float(np.clip(guidance.speed_mps - speed, -0.25 * dt, 0.20 * dt))
        position += dt * speed * np.array([math.cos(math.radians(heading)), math.sin(math.radians(heading))])
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / f"{case}_seed_{seed}.csv"
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    summary = {
        "case": case, "seed": seed, "completed": completed, "duration_s": records[-1]["time_s"],
        "first_track_s": first_track, "scan_count": len(scan_ids),
        "fault_seen": fault_seen, "recovered": recovered,
        "tracking_mae_m": float(np.mean(track_errors)) if track_errors else None,
        "tracking_p95_m": float(np.percentile(track_errors, 95)) if track_errors else None,
        "finite_commands": True, "uncalibrated_depth_publications": 0,
        "final_position_xy_m": position.tolist(),
        "step_latency_p99_ms": float(np.percentile(step_times_ms, 99)),
        "max_scan_sample_count": max(row["scan_sample_count"] for row in records),
        "wall_time_s": time.perf_counter() - started,
    }
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", nargs="+", choices=CASES, default=list(CASES))
    parser.add_argument("--seed-start", type=int, default=100)
    parser.add_argument("--seeds", type=int, default=1)
    parser.add_argument("--max-time-s", type=float, default=1500.)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "results/v2/inspection_closed_loop")
    args = parser.parse_args()
    results = []
    for case in args.cases:
        for seed in range(args.seed_start, args.seed_start + args.seeds):
            result = run_case(case, seed, args.output_dir, args.max_time_s)
            results.append(result)
            print(json.dumps(result), flush=True)
    summary_path = args.output_dir / "summary.json"
    summary_path.write_text(json.dumps({"config": asdict(config()), "runs": results}, indent=2) + "\n")
    write_experiment_manifest(
        args.output_dir, experiment="inspection_contract_closed_loop",
        config={"seed_start": args.seed_start, "seeds": args.seeds, "cases": args.cases, "v2": asdict(config())},
        artifact_paths=[summary_path, *args.output_dir.glob("*.csv")], repository_root=ROOT,
    )


if __name__ == "__main__":
    main()
