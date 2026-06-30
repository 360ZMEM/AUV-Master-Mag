import sys
import unittest
from pathlib import Path

import numpy as np


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.viz import (
    MilestoneMetrics,
    PRE_2G,
    compare_to_baseline,
    health_score,
)
from auv_mag_tracking.config import build_default_scenarios
from auv_mag_tracking.viz.metrics import HealthMetrics, compute_health_metrics
from auv_mag_tracking.viz.recorder import RunRecorder, simulate_case, simulate_run


def _metrics(case_name: str, *, mean_err: float, track: float, switches: int,
             cross_track: float = 2.0, good_ratio: float = 0.9,
             track_vehicle_err: float = 4.0, endpoint_goal: float = 0.0,
             route: float = 0.0) -> HealthMetrics:
    """Build a HealthMetrics with only the fields the progress view reads set."""
    return HealthMetrics(
        case_name=case_name,
        deployment_mode=False,
        duration_s=200.0,
        total_steps=4000,
        mean_heading_error_deg=mean_err,
        median_heading_error_deg=mean_err,
        final_heading_error_deg=mean_err,
        good_ratio=good_ratio,
        flip_count=0,
        heading_oscillations=0,
        mode_fraction={"track": track},
        track_active_fraction=track,
        mode_switches=switches,
        source_fraction={},
        sonar_contribution=0.0,
        magnetic_contribution=1.0,
        mean_snr_db=10.0,
        total_peaks=5,
        peak_rate_hz=0.1,
        mean_fit_residual_m=0.5,
        lock_grade_fraction=1.0,
        mean_cross_track_m=cross_track,
        max_cross_track_m=cross_track,
        mean_confidence=0.7,
        safe_lock_fraction=0.0,
        mean_vector_consistency=0.8,
        burial_inversion_mae_m=float("nan"),
        mean_vehicle_heading_error_deg=track_vehicle_err,
        track_mean_heading_error_deg=mean_err,
        track_mean_vehicle_heading_error_deg=track_vehicle_err,
        track_mean_cross_track_m=cross_track,
        median_cross_track_m=cross_track,
        p90_cross_track_m=cross_track,
        final_cross_track_m=cross_track,
        route_completion_ratio=route,
        final_route_progress_m=route * 100.0,
        route_length_m=100.0,
        final_route_distance_m=cross_track,
        p99_cross_track_m=cross_track,
        maze_geometry_passed=1.0,
        endpoint_goal_enabled=endpoint_goal,
        endpoint_completed=1.0 if endpoint_goal and route >= 0.95 else 0.0,
        heading_errors_deg=np.array([mean_err]),
    )


class ProgressComparisonTest(unittest.TestCase):
    def test_baseline_constants_cover_all_default_cases(self) -> None:
        self.assertEqual(set(PRE_2G), {f"case{i}" for i in range(1, 6)})
        self.assertEqual(PRE_2G["case2"].mode_switches, 164)

    def test_switch_storm_collapse_reads_as_improved(self) -> None:
        current = _metrics("case2", mean_err=7.2, track=0.47, switches=2)
        delta = compare_to_baseline(current, PRE_2G["case2"])
        before, after, change, higher_is_better, _, target = delta.fields["switches"]
        self.assertEqual(before, 164.0)
        self.assertEqual(after, 2.0)
        self.assertEqual(change, -162.0)
        self.assertFalse(higher_is_better)
        self.assertTrue(delta.improved("switches"))

    def test_higher_is_better_direction_for_track(self) -> None:
        current = _metrics("case3", mean_err=0.0, track=0.365, switches=2)
        delta = compare_to_baseline(current, PRE_2G["case3"])
        before, after, change, higher_is_better, _, _ = delta.fields["track_pct"]
        self.assertAlmostEqual(before, 4.0)
        self.assertAlmostEqual(after, 36.5)
        self.assertTrue(higher_is_better)
        self.assertTrue(delta.improved("track_pct"))

    def test_health_field_uses_live_health_score(self) -> None:
        current = _metrics("case4", mean_err=0.7, track=0.53, switches=2)
        delta = compare_to_baseline(current, PRE_2G["case4"])
        _, after, _, _, _, _ = delta.fields["health"]
        self.assertAlmostEqual(after, health_score(current))

    def test_health_score_uses_task_metrics_not_only_fused_heading(self) -> None:
        current = _metrics(
            "case6_like",
            mean_err=28.0,
            track=0.50,
            switches=2,
            cross_track=3.0,
            good_ratio=0.35,
            track_vehicle_err=6.0,
        )
        self.assertGreater(health_score(current), 70.0)

    def test_endpoint_goal_route_failure_is_penalized(self) -> None:
        current = _metrics(
            "maze_dropout_like",
            mean_err=4.0,
            track=0.52,
            switches=22,
            cross_track=24.0,
            good_ratio=0.9,
            track_vehicle_err=90.0,
            endpoint_goal=1.0,
            route=0.015,
        )
        self.assertLess(health_score(current), 35.0)

    def test_maze_route_jump_marks_geometry_failure(self) -> None:
        recorder = RunRecorder(
            "case_maze_sonar",
            deployment_mode=False,
            dt_s=1.0,
            cable_route_xy_m=np.array([[0.0, 0.0], [100.0, 0.0]], dtype=float),
        )
        for index, progress_m in enumerate([0.0, 1.0, 2.0, 80.0, 81.0]):
            recorder.append(
                time_s=float(index),
                pos_x_m=float(index),
                pos_y_m=0.0,
                heading_deg=0.0,
                true_heading_deg=0.0,
                true_nearest_x_m=float(index),
                true_nearest_y_m=0.0,
                true_burial_depth_m=1.0,
                route_progress_m=progress_m,
                route_distance_m=0.0,
                confidence=1.0,
                snr_db=20.0,
                fit_residual_m=0.0,
                fit_perp_eig_m2=0.0,
                peak_detected=0.0,
                safe_lock_active=0.0,
                vector_consistency=1.0,
                mode="track",
                source="SONAR",
            )
        record = recorder.finalize()
        record.metadata.update(
            {
                "route_length_m": 81.0,
                "final_route_progress_m": 81.0,
                "final_route_distance_m": 0.0,
                "route_completion_ratio": 1.0,
                "endpoint_goal_enabled": 1.0,
                "endpoint_completed": 1.0,
            }
        )

        metrics = compute_health_metrics(record)

        self.assertEqual(metrics.route_progress_large_jump_count, 1)
        self.assertGreater(metrics.route_progress_max_jump_m, 25.0)
        self.assertEqual(metrics.lane_shortcut_indicator, 1.0)
        self.assertEqual(metrics.maze_geometry_passed, 0.0)

    def test_regression_detected_when_error_grows(self) -> None:
        baseline = MilestoneMetrics("caseX", health=80.0, mean_heading_error_deg=5.0,
                                    track_active_fraction=0.5, mode_switches=2)
        current = _metrics("caseX", mean_err=12.0, track=0.5, switches=2)
        delta = compare_to_baseline(current, baseline)
        self.assertFalse(delta.improved("mean_err"))

    def test_local_path_side_channel_is_recorded(self) -> None:
        record = simulate_case("case1", max_steps=200)
        self.assertIn("local_path_heading_deg", record.channels)
        self.assertIn("local_path_model_code", record.channels)
        self.assertGreater(np.count_nonzero(record["local_path_model_code"] > 0.0), 0)

    def test_maze_sonar_baseline_is_isolated_from_dropout_exploration(self) -> None:
        scenarios = build_default_scenarios()

        self.assertTrue(scenarios["case_maze_sonar"].tracking.use_nominal_route_prior)
        self.assertTrue(scenarios["case_maze_sparse_sonar"].tracking.use_nominal_route_prior)
        self.assertFalse(scenarios["case_maze_sonar_dropout"].tracking.use_nominal_route_prior)

        # The clean maze baselines must carry no prior-pose bias and no online
        # correction; all stress is isolated in the tiered prior scenarios.
        for name in ("case_maze_sonar", "case_maze_sparse_sonar"):
            self.assertEqual(
                scenarios[name].tracking.nominal_route_prior_translation_xy_m,
                (0.0, 0.0),
            )
            self.assertFalse(
                scenarios[name].tracking.nominal_route_prior_observation_correction_enabled
            )

        # Continuous-sonar tiers run the milder static-bias envelope and keep the
        # progress guard disabled (no guard window when sonar is always-on).
        for tier, translation, rotation, scale in (
            ("light", (0.0, 1.0), 0.5, (0.998, 1.0)),
            ("mid", (0.0, 2.0), 1.0, (0.997, 1.0)),
            ("heavy", (0.0, 3.0), 1.5, (0.995, 1.0)),
        ):
            name = f"case_maze_sonar_prior_{tier}"
            tracking = scenarios[name].tracking
            self.assertTrue(scenarios[name].navigation.enabled)
            self.assertTrue(tracking.use_nominal_route_prior)
            self.assertEqual(tracking.nominal_route_prior_translation_xy_m, translation)
            self.assertAlmostEqual(tracking.nominal_route_prior_rotation_deg, rotation)
            self.assertEqual(tracking.nominal_route_prior_scale_xy, scale)
            self.assertTrue(tracking.nominal_route_prior_observation_correction_enabled)
            self.assertFalse(tracking.nominal_route_progress_guard_enabled)

        # Sparse / dropout tiers carry the stronger static-bias envelope plus the
        # EKF correction and the progress guard window.
        for sonar_kind in ("sparse_sonar", "sonar_dropout"):
            for tier, translation, rotation, scale in (
                ("light", (0.0, 3.0), 1.5, (0.995, 1.0)),
                ("mid", (0.0, 7.5), 3.0, (0.99, 1.0)),
                ("heavy", (0.0, 10.0), 5.0, (0.98, 1.0)),
            ):
                name = f"case_maze_{sonar_kind}_prior_{tier}"
                tracking = scenarios[name].tracking
                self.assertTrue(scenarios[name].navigation.enabled)
                self.assertTrue(tracking.use_nominal_route_prior)
                self.assertEqual(tracking.nominal_route_prior_translation_xy_m, translation)
                self.assertAlmostEqual(tracking.nominal_route_prior_rotation_deg, rotation)
                self.assertEqual(tracking.nominal_route_prior_scale_xy, scale)
                self.assertTrue(tracking.nominal_route_prior_observation_correction_enabled)
                self.assertTrue(tracking.nominal_route_prior_correction_ekf_enabled)
                self.assertTrue(tracking.nominal_route_progress_guard_enabled)

        # Sparse + prob=0.15 tuning variant tightens the EKF anchor weighting.
        prob015 = scenarios["case_maze_sparse_sonar_prior_mid_prob015"].tracking
        self.assertEqual(prob015.nominal_route_prior_translation_xy_m, (0.0, 7.5))
        self.assertLessEqual(prob015.nominal_route_prior_correction_max_step_m, 0.20)

        self.assertFalse(
            scenarios["case_maze_sonar_dropout"].tracking.reacquire_region_route_guard_enabled
        )
        self.assertFalse(scenarios["case_maze_sonar_dropout"].tracking.probe_burst_manager_enabled)
        self.assertTrue(scenarios["case_maze_sonar_dropout"].tracking.adaptive_track_zigzag_angle_enabled)
        self.assertAlmostEqual(
            scenarios["case_maze_sonar_dropout"].tracking.adaptive_track_zigzag_effective_distance_m,
            3.0,
        )
        self.assertAlmostEqual(
            scenarios["case_maze_sonar_dropout"].tracking.adaptive_track_zigzag_angle_adjustment_deg,
            5.0,
        )
        self.assertTrue(scenarios["case_maze_sonar_dropout"].tracking.magnetic_path_observation_enabled)
        self.assertAlmostEqual(
            scenarios["case_maze_sonar_dropout"].tracking.magnetic_path_max_cross_track_m,
            30.0,
        )

    def test_zigzag_probe_cycle_channels_are_recorded_when_probe_enabled(self) -> None:
        scenario = build_default_scenarios()["case1"]
        scenario.tracking.track_active_zigzag_angle_deg = 10.0
        scenario.tracking.curve_track_crossing_angle_deg = 10.0

        record = simulate_run(scenario, max_steps=800)

        self.assertIn("zigzag_probe_active", record.channels)
        self.assertIn("zigzag_probe_cycle_id", record.channels)
        self.assertIn("zigzag_probe_leg_sign", record.channels)
        self.assertIn("zigzag_probe_magnetic_crossing_event", record.channels)
        self.assertIn("zigzag_probe_leg_route_delta_m", record.channels)
        self.assertIn("zigzag_probe_completed_leg_route_delta_m", record.channels)
        self.assertIn("zigzag_probe_forward_leg_event", record.channels)
        self.assertIn("zigzag_probe_backward_leg_event", record.channels)
        self.assertIn("zigzag_probe_magnetic_crossing_forward_leg_event", record.channels)
        self.assertIn("zigzag_probe_magnetic_crossing_backward_leg_event", record.channels)
        self.assertIn("zigzag_probe_forward_phase_active", record.channels)
        self.assertIn("zigzag_probe_forward_phase_magnetic_crossing_event", record.channels)
        self.assertIn("zigzag_probe_forward_phase_magnetic_path_valid", record.channels)
        self.assertIn("zigzag_probe_forward_phase_candidate_valid", record.channels)
        self.assertIn("shadow_forward_zigzag_valid", record.channels)
        self.assertIn("shadow_forward_zigzag_forward_dot", record.channels)
        self.assertIn("shadow_forward_zigzag_completed_leg_route_delta_m", record.channels)
        self.assertIn("shadow_forward_zigzag_completed_leg_lateral_sweep_m", record.channels)
        self.assertIn("shadow_decoupled_lateral_valid", record.channels)
        self.assertIn("shadow_decoupled_lateral_forward_dot", record.channels)
        self.assertIn("shadow_decoupled_lateral_targeting_dot", record.channels)
        self.assertIn("shadow_decoupled_lateral_completed_leg_sweep_m", record.channels)
        self.assertIn("probe_burst_manager_state_code", record.channels)
        self.assertIn("probe_burst_manager_reason_code", record.channels)
        self.assertIn("probe_burst_manager_route_delta_m", record.channels)
        self.assertIn("probe_burst_manager_control_allowed", record.channels)
        self.assertIn("probe_burst_manager_reacquire_safe_control_allowed", record.channels)
        self.assertIn("probe_burst_manager_entry_abs_cross_track_m", record.channels)
        self.assertIn("zigzag_probe_field_ratio", record.channels)
        self.assertIn("zigzag_probe_burial_valid", record.channels)
        self.assertIn("zigzag_probe_cycle_burial_valid", record.channels)
        self.assertIn("zigzag_probe_cycle_burial_depth_m", record.channels)
        self.assertIn("shadow_hypothesis_readiness_score", record.channels)
        self.assertIn("shadow_hypothesis_bottleneck_code", record.channels)
        self.assertIn("shadow_axis_hypothesis_valid", record.channels)
        self.assertIn("shadow_axis_score_margin", record.channels)
        self.assertIn("shadow_axis_validation_reason_code", record.channels)
        self.assertIn("shadow_axis_validation_margin_deficit", record.channels)
        self.assertIn("magnetic_phase_detector_reason_code", record.channels)
        self.assertIn("magnetic_phase_detector_candidate_duration_s", record.channels)
        self.assertIn("magnetic_phase_detector_axis_delta_deg", record.channels)
        self.assertGreater(np.count_nonzero(record["zigzag_probe_active"] > 0.5), 0)
        self.assertGreater(np.count_nonzero(np.isfinite(record["zigzag_probe_cycle_age_s"])), 0)

    def test_forward_biased_probe_metrics_classify_legs_and_crossings(self) -> None:
        recorder = RunRecorder(
            "synthetic_probe",
            deployment_mode=False,
            dt_s=1.0,
            cable_route_xy_m=np.array([[0.0, 0.0], [10.0, 0.0]], dtype=float),
        )
        rows = [
            (2.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0),
            (-3.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        ]
        for index, (
            completed_delta_m,
            forward_leg,
            backward_leg,
            stall_leg,
            crossing_forward,
            crossing_backward,
            crossing_stall,
            forward_phase,
            forward_phase_crossing,
            forward_phase_path,
            forward_phase_phase,
            forward_phase_lookahead,
            forward_phase_candidate,
        ) in enumerate(rows):
            recorder.append(
                time_s=float(index),
                pos_x_m=0.0,
                pos_y_m=0.0,
                heading_deg=0.0,
                true_heading_deg=0.0,
                true_nearest_x_m=0.0,
                true_nearest_y_m=0.0,
                true_burial_depth_m=1.0,
                route_progress_m=float(index),
                route_distance_m=0.0,
                confidence=1.0,
                snr_db=20.0,
                fit_residual_m=0.0,
                fit_perp_eig_m2=0.0,
                speed_mps=4.0,
                zigzag_probe_active=1.0,
                zigzag_probe_leg_flip_event=1.0,
                zigzag_probe_magnetic_crossing_event=1.0,
                zigzag_probe_completed_leg_route_delta_m=completed_delta_m,
                zigzag_probe_forward_leg_event=forward_leg,
                zigzag_probe_backward_leg_event=backward_leg,
                zigzag_probe_stall_leg_event=stall_leg,
                zigzag_probe_magnetic_crossing_forward_leg_event=crossing_forward,
                zigzag_probe_magnetic_crossing_backward_leg_event=crossing_backward,
                zigzag_probe_magnetic_crossing_stall_leg_event=crossing_stall,
                zigzag_probe_forward_phase_active=forward_phase,
                zigzag_probe_forward_phase_magnetic_crossing_event=forward_phase_crossing,
                zigzag_probe_forward_phase_magnetic_path_valid=forward_phase_path,
                zigzag_probe_forward_phase_magnetic_phase_valid=forward_phase_phase,
                zigzag_probe_forward_phase_lookahead_valid=forward_phase_lookahead,
                zigzag_probe_forward_phase_candidate_valid=forward_phase_candidate,
                shadow_forward_zigzag_valid=1.0,
                shadow_forward_zigzag_feasible=1.0 if index < 2 else 0.0,
                shadow_forward_zigzag_forward_dot=0.9,
                shadow_forward_zigzag_lateral_dot_abs=0.2,
                shadow_forward_zigzag_forward_rate_mps=1.8,
                shadow_forward_zigzag_lateral_rate_mps=0.4,
                shadow_forward_zigzag_completed_leg_route_delta_m=(
                    4.0 if index == 0 else 5.0 if index == 1 else np.nan
                ),
                shadow_forward_zigzag_completed_leg_lateral_sweep_m=(
                    2.5 if index == 0 else 3.5 if index == 1 else np.nan
                ),
                shadow_forward_zigzag_completed_leg_feasible_event=1.0 if index == 0 else 0.0,
                shadow_decoupled_lateral_valid=1.0,
                shadow_decoupled_lateral_feasible=1.0 if index < 2 else 0.0,
                shadow_decoupled_lateral_forward_dot=0.8,
                shadow_decoupled_lateral_targeting_dot=0.3,
                shadow_decoupled_lateral_error_m=2.0 if index == 0 else -1.0 if index == 1 else 0.5,
                shadow_decoupled_lateral_forward_rate_mps=1.6,
                shadow_decoupled_lateral_targeting_rate_mps=0.6,
                shadow_decoupled_lateral_completed_leg_route_delta_m=(
                    6.0 if index == 0 else 4.0 if index == 1 else np.nan
                ),
                shadow_decoupled_lateral_completed_leg_sweep_m=(
                    2.2 if index == 0 else 1.8 if index == 1 else np.nan
                ),
                shadow_decoupled_lateral_completed_leg_feasible_event=1.0 if index == 0 else 0.0,
                probe_burst_manager_state_code=(
                    1.0 if index == 0 else 2.0 if index == 1 else 3.0
                ),
                probe_burst_manager_burst_active=1.0 if index == 1 else 0.0,
                probe_burst_manager_recovery_active=1.0 if index == 2 else 0.0,
                probe_burst_manager_reason_code=(
                    3.0 if index == 1 else 7.0 if index == 2 else 2.0
                ),
                probe_burst_manager_state_elapsed_s=float(index + 1),
                probe_burst_manager_route_delta_m=float(index * 2),
                probe_burst_manager_evidence_count=float(index),
                probe_burst_manager_control_allowed=1.0 if index < 2 else 0.0,
                probe_burst_manager_reacquire_safe_control_allowed=1.0 if index == 1 else 0.0,
                probe_burst_manager_entry_abs_cross_track_m=2.0 if index == 0 else 10.0 if index == 1 else 30.0,
                peak_detected=0.0,
                safe_lock_active=0.0,
                vector_consistency=1.0,
                mode="track",
                source="SONAR",
            )
        record = recorder.finalize()

        metrics = compute_health_metrics(record)

        self.assertAlmostEqual(metrics.zigzag_probe_forward_leg_fraction, 1.0 / 3.0)
        self.assertAlmostEqual(metrics.zigzag_probe_backward_leg_fraction, 1.0 / 3.0)
        self.assertAlmostEqual(metrics.zigzag_probe_stall_leg_fraction, 1.0 / 3.0)
        self.assertAlmostEqual(metrics.zigzag_probe_crossing_forward_leg_fraction, 1.0 / 3.0)
        self.assertAlmostEqual(metrics.zigzag_probe_crossing_backward_leg_fraction, 1.0 / 3.0)
        self.assertAlmostEqual(metrics.zigzag_probe_crossing_stall_leg_fraction, 1.0 / 3.0)
        self.assertAlmostEqual(metrics.zigzag_probe_mean_forward_leg_delta_m, 2.0)
        self.assertAlmostEqual(metrics.zigzag_probe_mean_backward_leg_delta_m, -3.0)
        self.assertAlmostEqual(metrics.zigzag_probe_forward_phase_fraction, 2.0 / 3.0)
        self.assertEqual(metrics.zigzag_probe_forward_phase_crossing_count, 1)
        self.assertAlmostEqual(metrics.zigzag_probe_forward_phase_crossing_fraction, 1.0 / 3.0)
        self.assertAlmostEqual(metrics.zigzag_probe_forward_phase_magnetic_path_fraction, 0.5)
        self.assertAlmostEqual(metrics.zigzag_probe_forward_phase_magnetic_phase_fraction, 0.5)
        self.assertAlmostEqual(metrics.zigzag_probe_forward_phase_lookahead_fraction, 0.5)
        self.assertAlmostEqual(metrics.zigzag_probe_forward_phase_candidate_fraction, 0.5)
        self.assertAlmostEqual(metrics.shadow_forward_zigzag_valid_fraction, 1.0)
        self.assertAlmostEqual(metrics.shadow_forward_zigzag_feasible_fraction, 2.0 / 3.0)
        self.assertAlmostEqual(metrics.shadow_forward_zigzag_mean_forward_dot, 0.9)
        self.assertAlmostEqual(metrics.shadow_forward_zigzag_mean_lateral_dot_abs, 0.2)
        self.assertAlmostEqual(metrics.shadow_forward_zigzag_mean_forward_rate_mps, 1.8)
        self.assertAlmostEqual(metrics.shadow_forward_zigzag_mean_lateral_rate_mps, 0.4)
        self.assertAlmostEqual(metrics.shadow_forward_zigzag_completed_leg_feasible_fraction, 0.5)
        self.assertAlmostEqual(metrics.shadow_forward_zigzag_mean_leg_route_delta_m, 4.5)
        self.assertAlmostEqual(metrics.shadow_forward_zigzag_mean_leg_lateral_sweep_m, 3.0)
        self.assertAlmostEqual(metrics.shadow_forward_sweep_best_angle_deg, 22.0)
        self.assertAlmostEqual(metrics.shadow_forward_sweep_best_leg_duration_multiplier, 2.0)
        self.assertAlmostEqual(metrics.shadow_forward_sweep_best_feasible_fraction, 1.0)
        self.assertAlmostEqual(
            metrics.shadow_forward_sweep_best_mean_leg_route_delta_m,
            4.0 * np.cos(np.deg2rad(22.0)) * 2.0,
        )
        self.assertAlmostEqual(
            metrics.shadow_forward_sweep_best_mean_leg_lateral_sweep_m,
            4.0 * np.sin(np.deg2rad(22.0)) * 2.0,
        )
        self.assertAlmostEqual(metrics.shadow_forward_sweep_best_forward_dot, np.cos(np.deg2rad(22.0)))
        self.assertAlmostEqual(metrics.shadow_forward_sweep_best_lateral_dot_abs, np.sin(np.deg2rad(22.0)))
        self.assertAlmostEqual(metrics.shadow_decoupled_lateral_valid_fraction, 1.0)
        self.assertAlmostEqual(metrics.shadow_decoupled_lateral_feasible_fraction, 2.0 / 3.0)
        self.assertAlmostEqual(metrics.shadow_decoupled_lateral_mean_forward_dot, 0.8)
        self.assertAlmostEqual(metrics.shadow_decoupled_lateral_mean_targeting_dot, 0.3)
        self.assertAlmostEqual(metrics.shadow_decoupled_lateral_mean_abs_error_m, (2.0 + 1.0 + 0.5) / 3.0)
        self.assertAlmostEqual(metrics.shadow_decoupled_lateral_mean_forward_rate_mps, 1.6)
        self.assertAlmostEqual(metrics.shadow_decoupled_lateral_mean_targeting_rate_mps, 0.6)
        self.assertAlmostEqual(metrics.shadow_decoupled_lateral_completed_leg_feasible_fraction, 0.5)
        self.assertAlmostEqual(metrics.shadow_decoupled_lateral_mean_leg_route_delta_m, 5.0)
        self.assertAlmostEqual(metrics.shadow_decoupled_lateral_mean_leg_sweep_m, 2.0)
        self.assertAlmostEqual(metrics.probe_burst_manager_active_fraction, 1.0)
        self.assertAlmostEqual(metrics.probe_burst_manager_idle_fraction, 1.0 / 3.0)
        self.assertAlmostEqual(metrics.probe_burst_manager_burst_fraction, 1.0 / 3.0)
        self.assertAlmostEqual(metrics.probe_burst_manager_recovery_fraction, 1.0 / 3.0)
        self.assertEqual(metrics.probe_burst_manager_transition_count, 2)
        self.assertEqual(metrics.probe_burst_manager_recovery_timeout_count, 1)
        self.assertAlmostEqual(metrics.probe_burst_manager_mean_state_elapsed_s, 2.0)
        self.assertAlmostEqual(metrics.probe_burst_manager_mean_route_delta_m, 2.0)
        self.assertEqual(metrics.probe_burst_manager_max_evidence_count, 2)
        self.assertAlmostEqual(metrics.probe_burst_manager_control_allowed_fraction, 2.0 / 3.0)
        self.assertAlmostEqual(metrics.probe_burst_manager_reacquire_safe_control_allowed_fraction, 1.0 / 3.0)
        self.assertAlmostEqual(metrics.probe_burst_manager_mean_entry_abs_cross_track_m, 14.0)
        self.assertAlmostEqual(metrics.probe_burst_manager_entry_xt_le4_fraction, 1.0 / 3.0)
        self.assertAlmostEqual(metrics.probe_burst_manager_entry_xt_le20_fraction, 2.0 / 3.0)


if __name__ == "__main__":
    unittest.main()
