import copy
import sys
import unittest
from pathlib import Path

import numpy as np


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.config import SonarConfig, build_default_scenarios
from auv_mag_tracking.controller import GuidanceCommand, propagate_vehicle
from auv_mag_tracking.environment import CableEnvironment, CableFitTruth, CableRoute
from auv_mag_tracking.math_utils import Pose, project_point_to_line, smallest_angle_error_deg
from auv_mag_tracking.main_viz import _initial_vehicle_position_ned_m
from auv_mag_tracking.mission_manager import MissionInput, MissionManager, MissionState, MissionThresholds
from auv_mag_tracking.perception import ConfidenceEstimator, MagneticCablePerception, PeakDetector, PerceptionState, WeightedSlidingWindowFitter, FitResult, MagneticVectorAnalyzer, StreamingVectorPCAFitter
from auv_mag_tracking.sensor_model import MagnetometerModel, PoseMeasurement, SonarModel


class FusionFeatureTest(unittest.TestCase):
    def test_magnetic_guidance_confidence_is_not_penalized_by_offline_sonar(self) -> None:
        estimator = ConfidenceEstimator(lost_timeout_s=4.0)
        confidence = estimator.fused_confidence(0.52, 0.0, "MAGNETIC")
        self.assertAlmostEqual(confidence, 0.52)

    def test_memory_guidance_uses_fit_quality_to_raise_confidence(self) -> None:
        estimator = ConfidenceEstimator(lost_timeout_s=4.0)
        confidence = estimator.fused_confidence(
            0.16,
            0.0,
            "MEMORY",
            fit_residual_m=1.2,
            fit_covariance_xy_m2=np.array([[1.5, 0.0], [0.0, 0.4]], dtype=float),
        )
        self.assertGreater(confidence, 0.35)
        self.assertGreater(confidence, 0.16)

    def test_magnetometer_block_strength_uses_rms_not_last_sample(self) -> None:
        model = MagnetometerModel(build_default_scenarios()["case1"].sensor)
        sample_times_s = np.arange(1, 11, dtype=float) * model.sample_period_s
        cable_fields_ned_nt = np.zeros((sample_times_s.size, 3), dtype=float)
        cable_fields_ned_nt[:, 0] = [100.0, 0.0, -100.0, 0.0, 100.0, 0.0, -100.0, 0.0, 100.0, 0.0]

        reading = model.sample_block(
            true_fields_ned_nt=cable_fields_ned_nt,
            pose=Pose(position_ned_m=np.zeros(3, dtype=float), heading_deg=0.0, pitch_deg=0.0, roll_deg=0.0),
            sample_times_s=sample_times_s,
            cable_fields_ned_nt=cable_fields_ned_nt,
        )

        self.assertGreater(reading.cable_strength_nt, 60.0)
        self.assertFalse(reading.weak_signal_flag)

    def test_spline_route_has_nonzero_curvature(self) -> None:
        scenario = build_default_scenarios()["case6"]
        route = CableRoute(scenario.environment)
        curvature = route.curvature_at_xy(np.array([0.0, 0.0], dtype=float))
        self.assertGreater(curvature, 0.0)

    def test_sonar_dropouts_when_probability_zero(self) -> None:
        sonar = SonarModel(SonarConfig(prob_detection=0.0, max_range_m=15.0, horizontal_fov_deg=120.0))
        pose = Pose(position_ned_m=np.array([0.0, 0.0, 25.0]), heading_deg=0.0, pitch_deg=0.0, roll_deg=0.0)
        truth = CableFitTruth(
            nearest_point_xy_m=np.array([5.0, 0.0], dtype=float),
            tangent_xy=np.array([1.0, 0.0], dtype=float),
            heading_deg=0.0,
            burial_depth_m=1.5,
            cable_depth_m=31.5,
            curvature_1pm=0.02,
            progress_m=5.0,
        )
        reading = sonar.sample(pose, truth, time_s=1.0)
        self.assertFalse(reading.valid)
        self.assertEqual(reading.status, "OFFLINE")

    def test_sonar_detects_when_probability_one(self) -> None:
        sonar = SonarModel(SonarConfig(prob_detection=1.0, max_range_m=15.0, horizontal_fov_deg=120.0))
        pose = Pose(position_ned_m=np.array([0.0, 0.0, 25.0]), heading_deg=0.0, pitch_deg=0.0, roll_deg=0.0)
        truth = CableFitTruth(
            nearest_point_xy_m=np.array([6.0, 2.0], dtype=float),
            tangent_xy=np.array([1.0, 0.0], dtype=float),
            heading_deg=0.0,
            burial_depth_m=1.5,
            cable_depth_m=31.5,
            curvature_1pm=0.02,
            progress_m=6.0,
        )
        reading = sonar.sample(pose, truth, time_s=1.0)
        self.assertTrue(reading.valid)
        self.assertEqual(reading.status, "ONLINE")
        self.assertIsNotNone(reading.relative_position_body_m)

    def test_sonar_reliable_absence_returns_no_cable_far_away(self) -> None:
        sonar = SonarModel(
            SonarConfig(
                mode="reliable_absence",
                prob_detection=1.0,
                max_range_m=15.0,
                horizontal_fov_deg=120.0,
                absence_range_m=18.0,
            )
        )
        pose = Pose(position_ned_m=np.array([0.0, 0.0, 25.0]), heading_deg=0.0, pitch_deg=0.0, roll_deg=0.0)
        truth = CableFitTruth(
            nearest_point_xy_m=np.array([25.0, 0.0], dtype=float),
            tangent_xy=np.array([1.0, 0.0], dtype=float),
            heading_deg=0.0,
            burial_depth_m=1.5,
            cable_depth_m=31.5,
            curvature_1pm=0.02,
            progress_m=25.0,
        )
        reading = sonar.sample(pose, truth, time_s=1.0)
        self.assertFalse(reading.valid)
        self.assertEqual(reading.status, "NO_CABLE")
        self.assertIsNone(reading.relative_position_body_m)

    def test_sonar_degraded_advantage_hit_raises_confidence(self) -> None:
        sonar = SonarModel(
            SonarConfig(
                mode="degraded",
                prob_detection=1.0,
                max_range_m=15.0,
                horizontal_fov_deg=120.0,
                position_noise_std_m=0.8,
                heading_noise_deg=8.0,
                advantage_probability=1.0,
                advantage_position_noise_scale=0.1,
                advantage_heading_noise_scale=0.1,
                advantage_confidence_floor=0.92,
            )
        )
        pose = Pose(position_ned_m=np.array([0.0, 0.0, 25.0]), heading_deg=0.0, pitch_deg=0.0, roll_deg=0.0)
        truth = CableFitTruth(
            nearest_point_xy_m=np.array([6.0, 2.0], dtype=float),
            tangent_xy=np.array([1.0, 0.0], dtype=float),
            heading_deg=0.0,
            burial_depth_m=1.5,
            cable_depth_m=31.5,
            curvature_1pm=0.02,
            progress_m=6.0,
        )
        reading = sonar.sample(pose, truth, time_s=1.0)
        self.assertTrue(reading.valid)
        self.assertEqual(reading.status, "ONLINE")
        self.assertGreaterEqual(reading.confidence, 0.92)

    def test_toy_sonar_mode_pulls_initial_position_closer_to_cable(self) -> None:
        scenario = copy.deepcopy(build_default_scenarios()["case1"])
        scenario.sonar.mode = "off"
        environment = CableRoute(scenario.environment)
        original_xy = np.asarray(scenario.vehicle.initial_position_ned_m[:2], dtype=float)
        original_nearest_xy, _, original_distance_m = environment.nearest_point_and_tangent(original_xy)

        adjusted_position_ned_m = _initial_vehicle_position_ned_m(scenario, CableEnvironment(scenario))
        adjusted_xy = adjusted_position_ned_m[:2]
        adjusted_nearest_xy, _, adjusted_distance_m = environment.nearest_point_and_tangent(adjusted_xy)

        self.assertLess(adjusted_distance_m, original_distance_m)
        self.assertLessEqual(adjusted_distance_m, max(4.0, 0.35 * scenario.sonar.absence_range_m) + 1e-9)
        self.assertTrue(np.allclose(original_nearest_xy, adjusted_nearest_xy))

    def test_deployment_mode_keeps_configured_initial_position(self) -> None:
        scenario = copy.deepcopy(build_default_scenarios()["case1"])
        scenario.tracking.use_nominal_route_prior = False
        scenario.sonar.mode = "off"
        environment = CableEnvironment(scenario)

        adjusted_position_ned_m = _initial_vehicle_position_ned_m(scenario, environment)
        np.testing.assert_allclose(adjusted_position_ned_m, np.asarray(scenario.vehicle.initial_position_ned_m, dtype=float))

    def test_deployment_peak_observations_project_to_reliable_fit(self) -> None:
        scenario = copy.deepcopy(build_default_scenarios()["case1"])
        scenario.tracking.use_nominal_route_prior = False
        perception = MagneticCablePerception(scenario)
        perception.last_accepted_fit_result = FitResult(
            origin_xy_m=np.zeros(2, dtype=float),
            direction_xy=np.array([1.0, 0.0], dtype=float),
            residual_m=0.5,
            covariance_xy_m2=np.eye(2, dtype=float),
        )

        projected = perception._peak_cable_observation_xy_m(  # type: ignore[attr-defined]
            np.array([10.0, 10.0], dtype=float),
            sonar_reading=None,
        )

        self.assertIsNotNone(projected)
        np.testing.assert_allclose(projected, np.array([10.0, 0.0], dtype=float))

    def test_deployment_blind_heading_waits_for_enough_points(self) -> None:
        scenario = copy.deepcopy(build_default_scenarios()["case1"])
        scenario.tracking.use_nominal_route_prior = False
        perception = MagneticCablePerception(scenario)
        perception.valid_points_xy.clear()
        perception.valid_points_xy.append(np.array([0.0, 0.0], dtype=float))
        perception.valid_points_xy.append(np.array([1.0, 0.0], dtype=float))

        self.assertIsNone(perception._blind_heading())

    def test_weighted_sliding_window_fitter_prefers_high_snr_points(self) -> None:
        fitter = WeightedSlidingWindowFitter(capacity=8, snr_floor=1.05)
        for point_xy in [(-2.0, -0.3), (-1.0, -0.1), (1.0, 0.1), (2.0, 0.2)]:
            fitter.add_peak(np.array(point_xy, dtype=float), snr_linear=100.0, confidence=0.9, time_s=1.0)
        for point_xy in [(-0.2, -2.0), (0.0, -1.0), (0.1, 1.0), (0.2, 2.0)]:
            fitter.add_peak(np.array(point_xy, dtype=float), snr_linear=1.2, confidence=0.3, time_s=1.0)

        fit_result = fitter.fit()
        heading_deg = float(np.rad2deg(np.arctan2(fit_result.direction_xy[1], fit_result.direction_xy[0])))
        axis_error_deg = min(abs(heading_deg), abs(abs(heading_deg) - 180.0))
        self.assertLess(axis_error_deg, 25.0)
        self.assertIsNotNone(fit_result.covariance_xy_m2)
        self.assertEqual(fit_result.covariance_xy_m2.shape, (2, 2))

    def test_peak_detector_returns_peak_position(self) -> None:
        detector = PeakDetector(min_peak_strength_nt=5.0, turn_trigger_ratio=0.8, hysteresis_fraction=0.1, cooldown_s=0.1)
        detector.update(4.0, 0.0, position_xy_m=np.array([0.0, 0.0], dtype=float))
        detector.update(8.0, 0.1, position_xy_m=np.array([1.0, 0.5], dtype=float))
        detector.update(12.0, 0.2, position_xy_m=np.array([2.0, 1.0], dtype=float))
        detector.update(9.0, 0.3, position_xy_m=np.array([2.6, 1.3], dtype=float))
        event = detector.update(6.5, 0.4, position_xy_m=np.array([3.0, 1.5], dtype=float))
        self.assertTrue(event.detected)
        self.assertIsNotNone(event.peak_position_xy_m)
        self.assertGreater(event.peak_position_xy_m[0], 1.5)
        self.assertLess(event.peak_position_xy_m[0], 2.6)

    def test_project_point_to_line_returns_local_cable_point(self) -> None:
        projected = project_point_to_line(
            point_xy=np.array([10.0, 4.0], dtype=float),
            origin_xy=np.array([0.0, 0.0], dtype=float),
            direction_xy=np.array([1.0, 0.0], dtype=float),
        )
        np.testing.assert_allclose(projected, np.array([10.0, 0.0], dtype=float))

    def test_projected_peak_points_preserve_route_heading(self) -> None:
        fitter = WeightedSlidingWindowFitter(capacity=8, snr_floor=1.05)
        raw_peak_points = [
            np.array([-91.206, -17.546], dtype=float),
            np.array([-94.798, -8.071], dtype=float),
            np.array([-98.500, 0.596], dtype=float),
            np.array([-99.656, 4.084], dtype=float),
            np.array([-98.334, 7.429], dtype=float),
        ]
        anchor_xy = raw_peak_points[0]
        route_direction_xy = np.array([1.0, 0.0], dtype=float)
        for index, peak_point_xy in enumerate(raw_peak_points):
            projected_point_xy = project_point_to_line(peak_point_xy, anchor_xy, route_direction_xy)
            fitter.add_peak(projected_point_xy, snr_linear=50.0 + index, confidence=0.8, time_s=float(index))

        fit_result = fitter.fit()
        heading_deg = float(np.rad2deg(np.arctan2(fit_result.direction_xy[1], fit_result.direction_xy[0])))
        axis_error_deg = min(abs(heading_deg), abs(abs(heading_deg) - 180.0))
        self.assertLess(axis_error_deg, 5.0)

    def test_deployment_mode_uses_raw_peaks_before_fit_exists(self) -> None:
        scenario = build_default_scenarios()["case1"]
        scenario.tracking.use_nominal_route_prior = False
        perception = MagneticCablePerception(scenario)

        observed_point = perception._peak_cable_observation_xy_m(
            peak_position_xy_m=np.array([12.0, -8.0], dtype=float),
            sonar_reading=None,
        )

        np.testing.assert_allclose(observed_point, np.array([12.0, -8.0], dtype=float))

    def test_propagate_vehicle_respects_turning_radius_limit(self) -> None:
        scenario = build_default_scenarios()["case6"]
        pose = Pose(position_ned_m=np.array([0.0, 0.0, 25.0]), heading_deg=0.0, pitch_deg=0.0, roll_deg=0.0, speed_mps=0.5)
        command = GuidanceCommand(desired_heading_deg=90.0, speed_mps=0.5, mode=MissionState.LOCK_ALIGN, yaw_rate_deg_s=100.0)
        updated = propagate_vehicle(pose, command, scenario, seabed_depth_m=30.0, dt_s=1.0)
        max_expected_heading_step = min(
            scenario.vehicle.max_yaw_rate_deg_s,
            np.rad2deg(command.speed_mps / scenario.vehicle.min_turning_radius_m),
        )
        self.assertLessEqual(updated.heading_deg, max_expected_heading_step + 1e-6)

    def test_magnetic_vector_analyzer_pca_extraction(self) -> None:
        """PCA extracts principal axis from consistent AC vector samples."""
        analyzer = MagneticVectorAnalyzer(buffer_capacity=8, pca_buffer_capacity=20)
        # Simulate vectors oscillating along ~45° axis
        expected_angle_deg = 45.0
        expected_rad = np.deg2rad(expected_angle_deg)
        base_vec = np.array([np.cos(expected_rad), np.sin(expected_rad)], dtype=float)
        orthogonal = np.array([-base_vec[1], base_vec[0]], dtype=float)

        for i in range(15):
            # Strong signal along principal axis + small orthogonal noise
            magnitude = 50.0 + 30.0 * np.sin(2.0 * np.pi * i / 10.0)
            noise = 5.0 * np.sin(2.0 * np.pi * i / 7.0)
            sample = magnitude * base_vec + noise * orthogonal
            analyzer.update(
                np.array([sample[0], sample[1], 0.0], dtype=float),
                tracking_strength_nt=80.0,
                snr_db=20.0,
                signal_mode="ac_50hz",
            )

        self.assertIsNotNone(analyzer.magnetic_vector_heading_deg)
        heading_error = abs(smallest_angle_error_deg(analyzer.magnetic_vector_heading_deg, expected_angle_deg))
        self.assertLess(min(heading_error, 180.0 - heading_error), 15.0)
        self.assertGreater(analyzer.vector_consistency_score, 0.5)

    def test_magnetic_vector_analyzer_snr_gating(self) -> None:
        """Low SNR should prevent vector heading updates."""
        analyzer = MagneticVectorAnalyzer(buffer_capacity=8, pca_buffer_capacity=20)

        # Fill with varying high-SNR samples along a consistent direction
        expected_angle_deg = 45.0
        expected_rad = np.deg2rad(expected_angle_deg)
        base_vec = np.array([np.cos(expected_rad), np.sin(expected_rad)], dtype=float)
        for i in range(15):
            magnitude = 50.0 + 30.0 * np.sin(2.0 * np.pi * i / 8.0)
            sample = magnitude * base_vec + 3.0 * np.array([np.sin(i * 0.7), np.cos(i * 0.7)])
            analyzer.update(
                np.array([sample[0], sample[1], 0.0], dtype=float),
                tracking_strength_nt=80.0,
                snr_db=20.0,
                signal_mode="ac_50hz",
            )
        heading_after_snr_ok = analyzer.magnetic_vector_heading_deg
        self.assertIsNotNone(heading_after_snr_ok)

        # Now try low-SNR updates
        prev_heading = analyzer.magnetic_vector_heading_deg
        prev_consistency = analyzer.vector_consistency_score
        analyzer.update(
            np.array([0.0, 200.0, 0.0], dtype=float),
            tracking_strength_nt=80.0,
            snr_db=5.0,
            signal_mode="ac_50hz",
        )
        # Heading should not have changed (low-SNR sample rejected)
        self.assertEqual(analyzer.magnetic_vector_heading_deg, prev_heading)
        self.assertEqual(analyzer.vector_consistency_score, prev_consistency)

    def test_magnetic_vector_analyzer_attitude_gating(self) -> None:
        """Large roll/pitch should trigger leakage risk and reject updates."""
        analyzer = MagneticVectorAnalyzer(buffer_capacity=8, pca_buffer_capacity=20)

        # Baseline with good attitude
        good_pose = PoseMeasurement(time_s=1.0, heading_deg=0.0, pitch_deg=0.5, roll_deg=0.5, speed_mps=1.0)
        for _ in range(10):
            analyzer.update(
                np.array([100.0, 50.0, 0.0], dtype=float),
                tracking_strength_nt=80.0,
                pose_measurement=good_pose,
                snr_db=20.0,
                signal_mode="ac_50hz",
            )
        self.assertFalse(analyzer.attitude_leakage_risk)

        # Now with large roll
        bad_pose = PoseMeasurement(time_s=2.0, heading_deg=0.0, pitch_deg=0.5, roll_deg=5.0, speed_mps=1.0)
        prev_heading = analyzer.magnetic_vector_heading_deg
        analyzer.update(
            np.array([0.0, 200.0, 0.0], dtype=float),
            tracking_strength_nt=80.0,
            pose_measurement=bad_pose,
            snr_db=20.0,
            signal_mode="ac_50hz",
        )
        self.assertTrue(analyzer.attitude_leakage_risk)
        self.assertEqual(analyzer.magnetic_vector_heading_deg, prev_heading)

        # Large pitch should also gate
        bad_pitch_pose = PoseMeasurement(time_s=3.0, heading_deg=0.0, pitch_deg=4.0, roll_deg=0.5, speed_mps=1.0)
        analyzer.attitude_leakage_risk = False
        analyzer.update(
            np.array([0.0, 200.0, 0.0], dtype=float),
            tracking_strength_nt=80.0,
            pose_measurement=bad_pitch_pose,
            snr_db=20.0,
            signal_mode="ac_50hz",
        )
        self.assertTrue(analyzer.attitude_leakage_risk)

    def test_magnetic_vector_analyzer_sign_alignment(self) -> None:
        """Consecutive frames should not flip 180° in vector heading."""
        analyzer = MagneticVectorAnalyzer(buffer_capacity=8, pca_buffer_capacity=20)

        # Feed consistent samples along ~30° direction
        expected_angle_deg = 30.0
        expected_rad = np.deg2rad(expected_angle_deg)
        base_vec = np.array([np.cos(expected_rad), np.sin(expected_rad)], dtype=float)

        # First batch: build up the PCA buffer
        for i in range(15):
            magnitude = 50.0 + 20.0 * np.sin(2.0 * np.pi * i / 8.0)
            sample = magnitude * base_vec + 3.0 * np.random.randn(2)
            analyzer.update(
                np.array([sample[0], sample[1], 0.0], dtype=float),
                tracking_strength_nt=70.0,
                snr_db=18.0,
                signal_mode="ac_50hz",
            )

        first_heading = analyzer.magnetic_vector_heading_deg
        self.assertIsNotNone(first_heading)

        # Continue feeding samples: heading should stay near the same direction
        for i in range(10):
            magnitude = 50.0 + 20.0 * np.sin(2.0 * np.pi * i / 8.0)
            sample = magnitude * base_vec + 3.0 * np.random.randn(2)
            analyzer.update(
                np.array([sample[0], sample[1], 0.0], dtype=float),
                tracking_strength_nt=70.0,
                snr_db=18.0,
                signal_mode="ac_50hz",
            )

        final_heading = analyzer.magnetic_vector_heading_deg
        self.assertIsNotNone(final_heading)
        heading_error = abs(smallest_angle_error_deg(final_heading, first_heading))
        self.assertLess(heading_error, 30.0)

    def test_streaming_vector_pca_fitter_basic(self) -> None:
        """PCA fitter extracts dominant direction from synthetic data."""
        rng = np.random.default_rng(seed=42)
        fitter = StreamingVectorPCAFitter(buffer_capacity=20)
        angle_rad = np.deg2rad(60.0)
        direction = np.array([np.cos(angle_rad), np.sin(angle_rad)], dtype=float)
        orthogonal = np.array([-direction[1], direction[0]], dtype=float)

        for i in range(15):
            amplitude = 40.0 + 15.0 * np.sin(2.0 * np.pi * i / 13.0)
            noise_along = rng.normal(0, 2.0)
            noise_ortho = rng.normal(0, 1.5)
            sample = amplitude * direction + noise_along * direction + noise_ortho * orthogonal
            fitter.add_sample(sample)

        principal_vec, consistency = fitter.compute_principal_vector()
        self.assertGreater(consistency, 0.5)
        angle_error = abs(smallest_angle_error_deg(
            float(np.rad2deg(np.arctan2(principal_vec[1], principal_vec[0]))),
            60.0,
        ))
        self.assertLess(min(angle_error, 180.0 - angle_error), 15.0)

    def test_streaming_vector_pca_fitter_insufficient_samples(self) -> None:
        """PCA fitter returns default when buffer has < 3 samples."""
        fitter = StreamingVectorPCAFitter(buffer_capacity=20)
        fitter.add_sample(np.array([10.0, 5.0], dtype=float))
        fitter.add_sample(np.array([12.0, 6.0], dtype=float))
        vec, consistency = fitter.compute_principal_vector()
        self.assertAlmostEqual(consistency, 0.0)
        np.testing.assert_array_equal(vec, np.array([1.0, 0.0], dtype=float))

    def test_perception_state_has_vector_diagnostics(self) -> None:
        """PerceptionState includes vector_consistency_score and attitude_leakage_risk."""
        state = PerceptionState(
            time_s=1.0,
            sensor_field_nt=np.zeros(3, dtype=float),
            body_field_nt=np.zeros(3, dtype=float),
            ned_field_nt=np.zeros(3, dtype=float),
            anomaly_ned_nt=np.zeros(3, dtype=float),
            ac_component_ned_nt=np.zeros(3, dtype=float),
            filtered_strength_nt=0.0,
            rms_strength_nt=0.0,
            tracking_strength_nt=0.0,
            noise_floor_nt=1.0,
            snr=0.0,
            snr_db=-120.0,
            magnetic_confidence=0.0,
            sonar_confidence=0.0,
            confidence=0.0,
            weak_signal_flag=True,
            signal_reliable=False,
            is_ac_detected=False,
            dominant_frequency_hz=0.0,
            peak_detected=False,
            fit_result=FitResult(origin_xy_m=None, direction_xy=None, residual_m=float("inf"), covariance_xy_m2=None),
            line_heading_deg=None,
            fused_heading_deg=None,
            blind_heading_deg=None,
            guidance_source="SEARCH",
            safe_lock_active=False,
            zigzag_width_m=3.0,
            sonar_status="OFFLINE",
            sonar_relative_position_body_m=None,
            sonar_heading_deg=None,
            estimated_cable_point_xy_m=None,
            estimated_path_points_xy_m=np.empty((0, 2), dtype=float),
            estimated_path_covariance_xy_m2=None,
            fit_update_rejected=False,
            estimated_burial_depth_m=None,
            true_burial_depth_m=1.5,
            burial_measurement_valid=False,
            last_detection_age_s=1e9,
            vector_consistency_score=0.75,
            attitude_leakage_risk=False,
        )
        self.assertAlmostEqual(state.vector_consistency_score, 0.75)
        self.assertFalse(state.attitude_leakage_risk)


class MissionFsmTest(unittest.TestCase):
    """Covers the three-state mission FSM that replaced behavior_tree."""

    def setUp(self) -> None:
        self.thresholds = MissionThresholds()
        self.manager = MissionManager(self.thresholds)

    def _input(
        self,
        time_s: float,
        *,
        signal: bool = False,
        converged: bool = False,
        confidence: float = 0.0,
        peak: bool = False,
    ) -> MissionInput:
        return MissionInput(
            time_s=time_s,
            mag_strength_nT=self.thresholds.mag_lock_threshold_nT + 10.0 if signal else 0.0,
            sonar_confidence=0.0,
            confidence=confidence,
            fused_heading_deg=0.0,
            yaw_error_deg=2.0 if converged else 30.0,
            fit_covariance_xy_m2=(
                np.array([[5.0, 0.0], [0.0, 0.4]], dtype=float) if converged else None
            ),
            peak_detected=peak,
        )

    def test_starts_in_search(self) -> None:
        self.assertEqual(self.manager.state, MissionState.SEARCH_ZIGZAG)

    def test_search_to_lock_after_signal_streak(self) -> None:
        for step in range(self.thresholds.lock_streak_required):
            decision = self.manager.update(self._input(float(step), signal=True))
        self.assertEqual(decision.state, MissionState.LOCK_ALIGN)
        self.assertEqual(decision.guidance_source, "MAGNETIC")
        self.assertAlmostEqual(decision.speed_factor, self.thresholds.align_speed_factor)

    def test_lock_to_track_after_converged_streak(self) -> None:
        for step in range(self.thresholds.lock_streak_required):
            self.manager.update(self._input(float(step), signal=True))
        self.assertEqual(self.manager.state, MissionState.LOCK_ALIGN)
        for step in range(self.thresholds.track_streak_required):
            decision = self.manager.update(
                self._input(10.0 + step, signal=True, converged=True, confidence=0.8)
            )
        self.assertEqual(decision.state, MissionState.TRACK_ACTIVE)

    def test_lock_falls_back_to_search_on_signal_loss(self) -> None:
        for step in range(self.thresholds.lock_streak_required):
            self.manager.update(self._input(float(step), signal=True))
        self.assertEqual(self.manager.state, MissionState.LOCK_ALIGN)
        last_signal_s = float(self.thresholds.lock_streak_required - 1)
        # One trough frame is not enough; loss is declared only after the signal
        # has been absent longer than the time-based hold window.
        self.manager.update(self._input(last_signal_s + 1.0, signal=False))
        decision = self.manager.update(
            self._input(last_signal_s + self.thresholds.signal_hold_s + 1.0, signal=False)
        )
        self.assertEqual(decision.state, MissionState.SEARCH_ZIGZAG)

    def test_track_to_emergency_on_sustained_low_confidence(self) -> None:
        for step in range(self.thresholds.lock_streak_required):
            self.manager.update(self._input(float(step), signal=True))
        for step in range(self.thresholds.track_streak_required):
            self.manager.update(self._input(10.0 + step, signal=True, converged=True, confidence=0.8))
        self.assertEqual(self.manager.state, MissionState.TRACK_ACTIVE)

        start_s = 100.0
        self.manager.update(self._input(start_s, signal=True, confidence=0.0))
        decision = self.manager.update(
            self._input(start_s + self.thresholds.emergency_hold_s, signal=True, confidence=0.0)
        )
        self.assertEqual(decision.state, MissionState.EMERGENCY_SURFACE)
        self.assertTrue(decision.emergency_flag)
        self.assertEqual(decision.speed_factor, 0.0)

    def test_emergency_is_terminal(self) -> None:
        for step in range(self.thresholds.lock_streak_required):
            self.manager.update(self._input(float(step), signal=True))
        for step in range(self.thresholds.track_streak_required):
            self.manager.update(self._input(10.0 + step, signal=True, converged=True, confidence=0.8))
        self.manager.update(self._input(100.0, signal=True, confidence=0.0))
        self.manager.update(self._input(100.0 + self.thresholds.emergency_hold_s, confidence=0.0))
        self.assertEqual(self.manager.state, MissionState.EMERGENCY_SURFACE)
        # Even a perfect signal cannot leave the terminal state.
        decision = self.manager.update(self._input(200.0, signal=True, converged=True, confidence=1.0))
        self.assertEqual(decision.state, MissionState.EMERGENCY_SURFACE)

    def test_peak_promotes_guidance_source(self) -> None:
        for step in range(self.thresholds.lock_streak_required):
            decision = self.manager.update(self._input(float(step), signal=True, peak=True))
        self.assertEqual(decision.guidance_source, "MAGNETIC_PEAK")


if __name__ == "__main__":
    unittest.main()