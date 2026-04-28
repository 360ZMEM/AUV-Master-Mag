#!/usr/bin/env python3
"""Quick deployment-mode smoke test with optional visualization."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parent
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.config import get_scenario
from auv_mag_tracking.main_viz import AuvCableTrackingSimulation


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Deployment-mode smoke test for the AUV cable tracking demo")
    parser.add_argument("--case", default="case1", help="Scenario name to run, e.g. case1/case2/case3/case4/case5/case_hf_phone/case_hf_industrial")
    parser.add_argument("--duration-s", type=float, default=100.0, help="Override the scenario duration for faster smoke testing")
    parser.add_argument("--sonar-mode", choices=["auto", "off", "degraded", "reliable_absence"], default="off", help="Override the sonar behavior")
    parser.add_argument("--viz", dest="enable_visualization", action="store_true", help="Enable live matplotlib visualization")
    parser.add_argument("--no-viz", dest="enable_visualization", action="store_false", help="Disable live matplotlib visualization")
    parser.set_defaults(enable_visualization=False)
    deployment_group = parser.add_mutually_exclusive_group()
    deployment_group.add_argument("--deployment-mode", dest="deployment_mode", action="store_true", help="Run in deployment mode with nominal-route priors disabled")
    deployment_group.add_argument("--no-deployment-mode", dest="deployment_mode", action="store_false", help="Run with nominal-route priors enabled")
    parser.set_defaults(deployment_mode=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    scenario = get_scenario(args.case)
    if scenario is None:
        print(f"ERROR: {args.case} not found")
        return 1

    if args.deployment_mode:
        scenario.tracking.use_nominal_route_prior = False
    scenario.sonar.mode = args.sonar_mode
    scenario.duration_s = args.duration_s

    print(f"Running deployment mode test for {scenario.name}")
    print(f"Duration: {scenario.duration_s}s")
    print(f"Visualization: {'on' if args.enable_visualization else 'off'}")
    print(f"Initial heading: {scenario.vehicle.initial_heading_deg}°")
    print(f"Cable waypoints: {scenario.environment.cable_waypoints_xy_m}")
    print(f"Min peak strength: {scenario.tracking.min_peak_strength_nt} nT")
    print(f"Peak cooldown: {scenario.tracking.peak_cooldown_s}s")
    print(f"Sonar mode: {scenario.sonar.mode}")
    print()

    simulation = AuvCableTrackingSimulation(scenario)
    report = simulation.run(enable_visualization=args.enable_visualization)

    print()
    print("=" * 60)
    print("Final Results:")
    print(f"Peaks: {report.peak_count}")
    print(f"Confidence: {report.final_confidence:.2f}")
    print(f"Mode: {report.final_mode}")
    print(f"Tracked: {report.tracked_distance_m:.1f}m")
    if report.hold_duration_s is not None:
        print(f"Hold duration: {report.hold_duration_s:.1f}s")
    if report.cable_heading_error_deg is not None:
        print(f"Final heading error: {report.cable_heading_error_deg:.1f}°")
    if report.displayed_centerline_heading_error_deg is not None:
        print(f"Displayed centerline error: {report.displayed_centerline_heading_error_deg:.1f}°")
    if report.mean_cable_heading_error_deg is not None:
        print(f"Mean heading error: {report.mean_cable_heading_error_deg:.1f}°")
    if report.hold_entry_cable_heading_error_deg is not None:
        print(f"Hold-entry heading error: {report.hold_entry_cable_heading_error_deg:.1f}°")
    if report.mean_lateral_deviation_m is not None:
        print(f"Lateral dev: {report.mean_lateral_deviation_m:.1f}m")
    if report.along_track_coverage_ratio is not None:
        print(f"Coverage: {report.along_track_coverage_ratio:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
