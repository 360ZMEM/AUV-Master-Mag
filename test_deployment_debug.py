#!/usr/bin/env python3
"""Quick deployment mode test with debug output."""

import sys
sys.path.insert(0, '/media/guanwen/E46F83AD460FE3C7/毕设分身')

from src.auv_mag_tracking.config import get_scenario
from src.auv_mag_tracking.main_viz import AuvCableTrackingSimulation

# Get case1 and enable deployment mode
scenario = get_scenario('case1')
if scenario is None:
    print("ERROR: case1 not found")
    sys.exit(1)

scenario.tracking.use_nominal_route_prior = False  # Enable deployment mode
scenario.sonar.mode = 'off'  # Disable sonar
scenario.duration_s = 100.0  # Shorter test for faster iteration

print(f"Running deployment mode test for {scenario.name}")
print(f"Duration: {scenario.duration_s}s")
print(f"Initial heading: {scenario.vehicle.initial_heading_deg}°")
print(f"Cable waypoints: {scenario.environment.cable_waypoints_xy_m}")
print(f"Min peak strength: {scenario.tracking.min_peak_strength_nt} nT")
print(f"Peak cooldown: {scenario.tracking.peak_cooldown_s}s")
print()

sim = AuvCableTrackingSimulation(scenario)
report = sim.run(enable_visualization=False)

print()
print("="*60)
print(f"Final Results:")
print(f"Peaks: {report.peak_count}")
print(f"Confidence: {report.final_confidence:.2f}")
print(f"Mode: {report.final_mode}")
print(f"Tracked: {report.tracked_distance_m:.1f}m")
if report.cable_heading_error_deg is not None:
    print(f"Heading error: {report.cable_heading_error_deg:.1f}°")
if report.mean_lateral_deviation_m is not None:
    print(f"Lateral dev: {report.mean_lateral_deviation_m:.1f}m")
if report.along_track_coverage_ratio is not None:
    print(f"Coverage: {report.along_track_coverage_ratio:.3f}")
