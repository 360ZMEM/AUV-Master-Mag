import sys
import unittest
from pathlib import Path
from typing import Optional

import numpy as np


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.config import build_default_scenarios
from auv_mag_tracking.controller import ZigZagController
from auv_mag_tracking.math_utils import Pose, smallest_angle_error_deg
from auv_mag_tracking.mission_manager import MissionState
from auv_mag_tracking.perception import FitResult, PerceptionState


def _perception_state(
    *,
    time_s: float = 0.0,
    confidence: float = 0.8,
    fused_heading_deg: float = 0.0,
    local_path_heading_deg: float = 0.0,
    local_path_tracking_state: str = "line_track",
    deployment_reacquire_required: bool = False,
    estimated_cable_point_xy_m: Optional[np.ndarray] = None,
    magnetic_cross_track_offset_m: Optional[float] = None,
    magnetic_path_observation_valid: bool = False,
    magnetic_path_heading_deg: Optional[float] = None,
    magnetic_path_cross_track_offset_m: Optional[float] = None,
) -> PerceptionState:
    fit_direction = np.array([1.0, 0.0], dtype=float)
    return PerceptionState(
        time_s=time_s,
        sensor_field_nt=np.zeros(3, dtype=float),
        body_field_nt=np.zeros(3, dtype=float),
        ned_field_nt=np.zeros(3, dtype=float),
        anomaly_ned_nt=np.zeros(3, dtype=float),
        ac_component_ned_nt=np.zeros(3, dtype=float),
        filtered_strength_nt=80.0,
        rms_strength_nt=80.0,
        tracking_strength_nt=80.0,
        noise_floor_nt=1.0,
        snr=80.0,
        snr_db=38.0,
        magnetic_confidence=confidence,
        sonar_confidence=confidence,
        confidence=confidence,
        weak_signal_flag=False,
        signal_reliable=True,
        is_ac_detected=True,
        dominant_frequency_hz=50.0,
        peak_detected=True,
        fit_result=FitResult(
            origin_xy_m=np.array([0.0, 0.0], dtype=float),
            direction_xy=fit_direction,
            residual_m=0.5,
            covariance_xy_m2=np.eye(2, dtype=float) * 0.2,
        ),
        line_heading_deg=0.0,
        fused_heading_deg=fused_heading_deg,
        blind_heading_deg=None,
        guidance_source="LOCAL_PATH",
        safe_lock_active=False,
        zigzag_width_m=4.0,
        sonar_status="TRACKING",
        sonar_relative_position_body_m=None,
        sonar_heading_deg=fused_heading_deg,
        estimated_cable_point_xy_m=estimated_cable_point_xy_m,
        estimated_path_points_xy_m=np.array([[0.0, 0.0], [5.0, 0.0], [10.0, 0.0]], dtype=float),
        estimated_path_covariance_xy_m2=np.eye(2, dtype=float) * 0.2,
        fit_update_rejected=False,
        estimated_burial_depth_m=None,
        true_burial_depth_m=1.5,
        burial_measurement_valid=False,
        last_detection_age_s=0.1,
        deployment_reacquire_required=deployment_reacquire_required,
        local_path_heading_deg=local_path_heading_deg,
        local_path_confidence=confidence,
        local_path_residual_m=0.5,
        local_path_radius_m=60.0,
        local_path_tracking_state=local_path_tracking_state,
        magnetic_cross_track_offset_m=magnetic_cross_track_offset_m,
        magnetic_path_observation_valid=magnetic_path_observation_valid,
        magnetic_path_heading_deg=magnetic_path_heading_deg,
        magnetic_path_cross_track_offset_m=magnetic_path_cross_track_offset_m,
    )


class ZigZagControllerReacquireTest(unittest.TestCase):
    def setUp(self) -> None:
        self.scenario = build_default_scenarios()["case_maze_sonar"]
        self.controller = ZigZagController(self.scenario)

    def test_reacquire_heading_targets_anchor_sector_instead_of_stale_heading(self) -> None:
        pose = Pose(
            position_ned_m=np.array([-10.0, 0.0, 0.0], dtype=float),
            heading_deg=0.0,
            pitch_deg=0.0,
            roll_deg=0.0,
            speed_mps=1.0,
        )
        self.controller.last_trusted_cable_point_xy_m = np.array([0.0, 0.0], dtype=float)
        self.controller.last_trusted_cable_heading_deg = 0.0
        self.controller.leg_sign = 1.0

        heading_deg = self.controller._reacquire_heading_deg(
            pose,
            _perception_state(
                deployment_reacquire_required=True,
                estimated_cable_point_xy_m=None,
                local_path_heading_deg=None,
                fused_heading_deg=None,
            ),
            fallback_heading_deg=0.0,
        )

        self.assertIsNotNone(heading_deg)
        self.assertGreater(abs(smallest_angle_error_deg(heading_deg, 0.0)), 20.0)
        self.assertLess(abs(smallest_angle_error_deg(heading_deg, 47.0)), 8.0)

    def test_reacquire_zigzag_flips_at_anchor_half_band(self) -> None:
        pose = Pose(
            position_ned_m=np.array([5.0, 25.0, 0.0], dtype=float),
            heading_deg=0.0,
            pitch_deg=0.0,
            roll_deg=0.0,
            speed_mps=1.0,
        )
        self.controller.reacquire_anchor_xy_m = np.array([0.0, 0.0], dtype=float)
        self.controller.reacquire_anchor_heading_deg = 0.0
        self.controller.leg_sign = 1.0

        heading_deg = self.controller._reacquire_zigzag_heading_deg(pose)

        self.assertIsNotNone(heading_deg)
        self.assertLess(self.controller.leg_sign, 0.0)
        self.assertEqual(self.controller.reacquire_leg_index, 1)
        self.assertLess(abs(smallest_angle_error_deg(heading_deg, -90.0)), 35.0)

    def test_curve_track_uses_dedicated_crossing_angle_and_speed_factor(self) -> None:
        pose = Pose(
            position_ned_m=np.array([0.0, 0.0, 0.0], dtype=float),
            heading_deg=0.0,
            pitch_deg=0.0,
            roll_deg=0.0,
            speed_mps=1.0,
        )
        perception = _perception_state(
            time_s=1.0,
            fused_heading_deg=45.0,
            local_path_heading_deg=45.0,
            local_path_tracking_state="curve_track",
            estimated_cable_point_xy_m=np.array([0.0, 0.0], dtype=float),
        )

        crossing_angle_deg = self.controller._crossing_angle_for_state(
            MissionState.SEARCH_ZIGZAG,
            perception.zigzag_width_m,
            magnetic_fit_ready=True,
            local_path_tracking_state=perception.local_path_tracking_state,
        )
        command = self.controller.update(pose, perception)

        self.assertAlmostEqual(crossing_angle_deg, self.scenario.tracking.curve_track_crossing_angle_deg)
        self.assertAlmostEqual(command.speed_mps, self.scenario.vehicle.cruise_speed_mps * self.scenario.tracking.curve_track_speed_factor)
        self.assertLess(abs(smallest_angle_error_deg(command.desired_heading_deg, 45.0 + crossing_angle_deg)), 1.0)

    def test_magnetic_crossing_probe_flips_on_magnetic_sign_change(self) -> None:
        self.scenario.tracking.use_nominal_route_prior = False
        self.scenario.tracking.magnetic_crossing_probe_control_enabled = True
        self.scenario.tracking.magnetic_crossing_probe_min_flip_interval_s = 0.0
        self.scenario.tracking.track_active_zigzag_angle_deg = 10.0
        self.controller = ZigZagController(self.scenario)
        pose = Pose(
            position_ned_m=np.array([0.0, 0.0, 0.0], dtype=float),
            heading_deg=0.0,
            pitch_deg=0.0,
            roll_deg=0.0,
            speed_mps=1.0,
        )

        self.controller.update(
            pose,
            _perception_state(
                time_s=1.0,
                magnetic_path_observation_valid=True,
                magnetic_path_heading_deg=0.0,
                magnetic_path_cross_track_offset_m=1.0,
            ),
        )
        self.assertLess(self.controller.leg_sign, 0.0)

        self.controller.update(
            pose,
            _perception_state(
                time_s=2.0,
                magnetic_path_observation_valid=True,
                magnetic_path_heading_deg=0.0,
                magnetic_path_cross_track_offset_m=-1.0,
            ),
        )

        self.assertGreater(self.controller.leg_sign, 0.0)

    def test_magnetic_crossing_probe_normalizes_offset_sign_to_base_heading(self) -> None:
        self.scenario.tracking.use_nominal_route_prior = False
        self.scenario.tracking.magnetic_crossing_probe_control_enabled = True
        self.scenario.tracking.magnetic_crossing_probe_min_flip_interval_s = 0.0
        self.controller = ZigZagController(self.scenario)
        pose = Pose(
            position_ned_m=np.array([0.0, 0.0, 0.0], dtype=float),
            heading_deg=0.0,
            pitch_deg=0.0,
            roll_deg=0.0,
            speed_mps=1.0,
        )

        self.controller.update(
            pose,
            _perception_state(
                time_s=1.0,
                magnetic_path_observation_valid=True,
                magnetic_path_heading_deg=180.0,
                magnetic_path_cross_track_offset_m=1.0,
            ),
        )

        self.assertGreater(self.controller.leg_sign, 0.0)

    def test_magnetic_crossing_probe_prefers_continuous_ratio_offset(self) -> None:
        self.scenario.tracking.use_nominal_route_prior = False
        self.scenario.tracking.magnetic_crossing_probe_control_enabled = True
        self.scenario.tracking.magnetic_crossing_probe_min_flip_interval_s = 0.0
        self.controller = ZigZagController(self.scenario)
        pose = Pose(
            position_ned_m=np.array([0.0, 0.0, 0.0], dtype=float),
            heading_deg=0.0,
            pitch_deg=0.0,
            roll_deg=0.0,
            speed_mps=1.0,
        )

        self.controller.update(
            pose,
            _perception_state(
                time_s=1.0,
                magnetic_cross_track_offset_m=-1.0,
                magnetic_path_observation_valid=True,
                magnetic_path_heading_deg=0.0,
                magnetic_path_cross_track_offset_m=1.0,
            ),
        )

        self.assertGreater(self.controller.leg_sign, 0.0)

    def test_magnetic_crossing_probe_forces_flip_after_missed_crossing(self) -> None:
        self.scenario.tracking.use_nominal_route_prior = False
        self.scenario.tracking.magnetic_crossing_probe_control_enabled = True
        self.scenario.tracking.magnetic_crossing_probe_forced_flip_multiplier = 1.0
        self.scenario.tracking.magnetic_crossing_probe_max_wait_s = 2.0
        self.controller = ZigZagController(self.scenario)
        pose = Pose(
            position_ned_m=np.array([0.0, 0.0, 0.0], dtype=float),
            heading_deg=0.0,
            pitch_deg=0.0,
            roll_deg=0.0,
            speed_mps=1.0,
        )

        self.controller.update(
            pose,
            _perception_state(
                time_s=0.0,
                magnetic_path_observation_valid=True,
                magnetic_path_heading_deg=0.0,
                magnetic_path_cross_track_offset_m=1.0,
            ),
        )
        perception = _perception_state(
            time_s=3.0,
            magnetic_path_observation_valid=True,
            magnetic_path_heading_deg=0.0,
            magnetic_path_cross_track_offset_m=1.0,
        )
        self.controller.update(pose, perception)

        self.assertTrue(perception.magnetic_crossing_probe_forced_flip)
        self.assertEqual(perception.magnetic_crossing_probe_missed_count, 1)


if __name__ == "__main__":
    unittest.main()
