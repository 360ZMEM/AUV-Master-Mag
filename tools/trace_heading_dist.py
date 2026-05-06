"""Quick diagnostic trace to see heading estimates vs truth."""
import sys
from pathlib import Path
WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

import copy
import numpy as np
from auv_mag_tracking.config import build_default_scenarios
from auv_mag_tracking.main_viz import AuvCableTrackingSimulation
from auv_mag_tracking.math_utils import smallest_angle_error_deg

def run_trace():
    scenario = build_default_scenarios()["case1"]
    scenario = copy.deepcopy(scenario)
    scenario.tracking.use_nominal_route_prior = False
    sim = AuvCableTrackingSimulation(scenario)

    errors = []
    deploy_headings = []
    true_headings = []
    modes = []

    for step_idx in range(400):
        time_s = step_idx * scenario.dt_s
        from auv_mag_tracking.controller import apply_attitude_profile, propagate_vehicle
        apply_attitude_profile(sim.pose, scenario, time_s)
        active_sample_rate_hz = 1.0 / max(sim.magnetometer.sample_period_s, 1e-9)
        sample_count = max(1, int(round(scenario.dt_s * active_sample_rate_hz)))
        sample_times_s = time_s + (np.arange(sample_count, dtype=float) + 1.0) * sim.magnetometer.sample_period_s
        current_block_a = scenario.signal.current_for_times(sample_times_s)
        cable_field_gain_ned_nt = sim.environment.field_model.cable_field_gain_ned_nt(sim.pose.position_ned_m)
        cable_field_block_ned_nt = current_block_a[:, None] * cable_field_gain_ned_nt[None, :]
        true_field_block_ned_nt = cable_field_block_ned_nt + sim.environment.background_field_ned_nt
        magnetometer_reading = sim.magnetometer.sample_block(true_field_block_ned_nt, sim.pose, sample_times_s, cable_fields_ned_nt=cable_field_block_ned_nt)
        signal_frame = sim.signal_driver.update(magnetometer_reading)
        pose_measurement = sim.imu.observe(sim.pose, time_s)
        cable_truth = sim.environment.cable_truth_at_xy(sim.pose.position_ned_m[:2])
        sonar_reading = sim.sonar.sample(sim.pose, cable_truth, time_s)
        burial_measurement = sim.burial_observer.observe(cable_truth.burial_depth_m, time_s)
        perception_state = sim.perception.update(
            reading=magnetometer_reading,
            pose_measurement=pose_measurement,
            vehicle_position_xy_m=sim.pose.position_ned_m[:2],
            burial_measurement=burial_measurement,
            true_burial_depth_m=cable_truth.burial_depth_m,
            sonar_reading=sonar_reading,
            signal_features=signal_frame.features,
        )

        deploy_hdg = getattr(sim.perception, 'deployment_estimated_cable_heading_deg', None)
        fused_hdg = getattr(perception_state, 'fused_heading_deg', None)
        mode = getattr(perception_state, 'mode', '?')
        n = len(getattr(sim.perception, 'crossing_headings', []))
        
        if deploy_hdg is not None and n >= 4:
            # True heading should be ~270° for case1
            true_hdg = 270.0  # Assuming case1 cable runs along x-axis
            err = abs(smallest_angle_error_deg(deploy_hdg, true_hdg))
            errors.append(err)
            deploy_headings.append(deploy_hdg)
            true_headings.append(true_hdg)
            modes.append(mode)

    if errors:
        print(f"\n=== HEADING DIAGNOSTIC (n>={4}) ===")
        print(f"Frames: {len(errors)}")
        print(f"Mean error: {np.mean(errors):.1f}°")
        print(f"Median error: {np.median(errors):.1f}°")
        print(f"Bad > 120°: {sum(1 for e in errors if e > 120)}")
        print(f"Bad > 150°: {sum(1 for e in errors if e > 150)}")
        
        # Show distribution
        bins_30 = sum(1 for e in errors if e <= 30)
        bins_60 = sum(1 for e in errors if 30 < e <= 60)
        bins_90 = sum(1 for e in errors if 60 < e <= 90)
        bins_120 = sum(1 for e in errors if 90 < e <= 120)
        bins_150 = sum(1 for e in errors if 120 < e <= 150)
        bins_180 = sum(1 for e in errors if e > 150)
        print(f"\nError Distribution:")
        print(f"  0-30°:  {bins_30} ({bins_30/len(errors)*100:.0f}%)")
        print(f"  30-60°: {bins_60} ({bins_60/len(errors)*100:.0f}%)")
        print(f"  60-90°: {bins_90} ({bins_90/len(errors)*100:.0f}%)")
        print(f"  90-120°:{bins_120} ({bins_120/len(errors)*100:.0f}%)")
        print(f"  120-150°:{bins_150} ({bins_150/len(errors)*100:.0f}%)")
        print(f"  >150°:  {bins_180} ({bins_180/len(errors)*100:.0f}%)")
        
        # Show mean heading estimate
        print(f"\nMean deploy heading: {np.mean(deploy_headings):.1f}°")
        print(f"Expected: ~270°")
        print(f"Offset from expected: {smallest_angle_error_deg(np.mean(deploy_headings), 270.0):.1f}°")
        
        # Check for bimodal distribution
        low_errors = [e for e in errors if e <= 90]
        high_errors = [e for e in errors if e > 90]
        print(f"\nBimodal Check:")
        print(f"  Low group (<=90°): {len(low_errors)} frames, mean={np.mean(low_errors):.1f}°")
        print(f"  High group (>90°): {len(high_errors)} frames, mean={np.mean(high_errors):.1f}°")

if __name__ == "__main__":
    run_trace()
