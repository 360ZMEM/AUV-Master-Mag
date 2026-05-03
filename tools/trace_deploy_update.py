"""Quick trace to see what deployment_estimated_cable_heading_deg values are actually set."""
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

    import auv_mag_tracking.perception as perception
    original_deploy_update = perception.MagneticCablePerception._update_deployment_cable_heading

    call_count = [0]

    _DEBUG_FULL = True

    def patched_deploy_update(self, heading_deg, position_xy_m, velocity_xy):
        before = self.deployment_estimated_cable_heading_deg
        before_flag = getattr(self, '_deployment_heading_self_corrected', False)
        result = original_deploy_update(self, heading_deg, position_xy_m, velocity_xy)
        after = self.deployment_estimated_cable_heading_deg
        after_flag = getattr(self, '_deployment_heading_self_corrected', False)
        n = len(self.crossing_headings)
        call_count[0] += 1
        if n >= 4 and after is not None and _DEBUG_FULL:
            b_str = f"{before:.1f}" if before is not None else "None"
            print(f"  [{call_count[0]}] cross{n-1}: {b_str} -> {after:.1f} | flag: {before_flag}->{after_flag}")
        return result

    perception.MagneticCablePerception._update_deployment_cable_heading = patched_deploy_update

    # Also patch update() to trace fusion overrides
    original_update = perception.MagneticCablePerception.update
    fusion_override_count = [0]

    def patched_update(self, reading, pose_measurement, vehicle_position_xy_m,
                       burial_measurement, true_burial_depth_m,
                       sonar_reading=None, signal_features=None):
        result = original_update(self, reading, pose_measurement, vehicle_position_xy_m,
                                burial_measurement, true_burial_depth_m,
                                sonar_reading, signal_features)
        n = len(getattr(self, 'crossing_headings', []))
        if n >= 4:
            deploy_hdg = getattr(self, 'deployment_estimated_cable_heading_deg', None)
            deploy_flag = getattr(self, '_deployment_heading_self_corrected', False)
            fused_hdg = getattr(result, 'fused_heading_deg', None)
            src = getattr(result, 'guidance_source', '?')
            fusion_override_count[0] += 1
            d_str = f"{deploy_hdg:.1f}" if deploy_hdg is not None else "None"
            f_str = f"{fused_hdg:.1f}" if fused_hdg is not None else "None"
            print(f"  FUSION[{fusion_override_count[0]}]: deploy={d_str} flag={deploy_flag} fused={f_str} src={src}")
        return result

    perception.MagneticCablePerception.update = patched_update

    try:
        step_count = 0
        max_steps = 600
        for step_idx in range(max_steps):
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
            command = sim.controller.update(sim.pose, perception_state)
            seabed_depth_m = sim.environment.seabed_depth_m(sim.pose.position_ned_m[:2])
            sim.pose = propagate_vehicle(sim.pose, command, scenario, seabed_depth_m, scenario.dt_s)
            step_count += 1
    finally:
        perception.MagneticCablePerception._update_deployment_cable_heading = original_deploy_update
        perception.MagneticCablePerception.update = original_update

    print(f"\nTotal steps: {step_count}, deploy_updates: {call_count[0]}, fusion_overrides: {fusion_override_count[0]}")

if __name__ == "__main__":
    run_trace()