import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.config import build_default_scenarios
from auv_mag_tracking.environment import CableEnvironment
from auv_mag_tracking.math_utils import smallest_angle_error_deg
from auv_mag_tracking.perception import (
    LocalCableState,
    LocalCableStateEstimator,
    MagneticCablePerception,
    MagneticLookaheadHypothesis,
    MagneticLookaheadTargetBuilder,
    MagneticPathObservation,
    MagneticPathObservationBuilder,
    MagneticShadowHypothesisSelector,
    MagneticZigzagPhaseDetector,
    MagneticZigzagPhaseObservation,
)


class MagneticTurnObservabilityTest(unittest.TestCase):
    def _case6_magnetic_observations(self, *, lateral_amplitude_m: float) -> tuple[list, list]:
        scenario = build_default_scenarios()["case6"]
        environment = CableEnvironment(scenario)
        route_points = environment.route.sample_xy(step_m=7.5)[14:50]
        builder = MagneticPathObservationBuilder(
            vertical_separation_m=scenario.vehicle.altitude_above_seabed_m + scenario.environment.burial_depth_m,
            min_horizontal_field_nt=0.01,
            max_cross_track_m=25.0,
            max_step_heading_change_deg=90.0,
        )

        observations = []
        truths = []
        previous_vehicle_xy = None
        for index, cable_xy in enumerate(route_points):
            truth = environment.cable_truth_at_xy(cable_xy)
            normal_xy = np.array([-truth.tangent_xy[1], truth.tangent_xy[0]], dtype=float)
            lateral_offset_m = lateral_amplitude_m * np.sin(index * 0.85)
            vehicle_xy = cable_xy + lateral_offset_m * normal_xy
            vehicle_z_m = environment.seabed_depth_m(vehicle_xy) - scenario.vehicle.altitude_above_seabed_m
            field_gain = environment.field_model.cable_field_gain_ned_nt(
                np.array([vehicle_xy[0], vehicle_xy[1], vehicle_z_m], dtype=float)
            )
            anomaly_ned_nt = field_gain * scenario.signal.ac_current_amplitude_a
            movement_heading_deg = None
            if previous_vehicle_xy is not None:
                movement = vehicle_xy - previous_vehicle_xy
                if np.linalg.norm(movement) > 1e-6:
                    movement_heading_deg = float(np.rad2deg(np.arctan2(movement[1], movement[0])))
            previous_vehicle_xy = vehicle_xy.copy()
            observation = builder.build(vehicle_xy, anomaly_ned_nt, movement_heading_deg=movement_heading_deg)
            if observation is None:
                continue
            observations.append(observation)
            truths.append(truth)
        return observations, truths

    def test_pure_magnetic_history_can_reconstruct_case6_curve_when_laterally_excited(self) -> None:
        observations, truths = self._case6_magnetic_observations(lateral_amplitude_m=5.0)

        self.assertGreaterEqual(len(observations), 24)
        point_errors_m = [
            float(np.linalg.norm(observation.position_xy_m - truth.nearest_point_xy_m))
            for observation, truth in zip(observations, truths)
        ]
        heading_errors_deg = [
            abs(smallest_angle_error_deg(observation.heading_deg, truth.heading_deg))
            for observation, truth in zip(observations, truths)
        ]
        self.assertLess(float(np.median(point_errors_m)), 3.0)
        self.assertLess(float(np.percentile(point_errors_m, 90.0)), 8.0)
        self.assertLess(float(np.median(heading_errors_deg)), 20.0)

        estimator = LocalCableStateEstimator(
            capacity=22,
            local_line_window=5,
            min_arc_radius_m=30.0,
            min_arc_angle_span_deg=180.0,
            heading_blend=0.55,
            curve_heading_delta_deg=10.0,
            min_observation_spacing_m=1.0,
        )
        fit_errors_deg = []
        curve_states = 0
        for index, (observation, truth) in enumerate(zip(observations, truths)):
            estimator.add_observation(
                observation.position_xy_m,
                time_s=float(index),
                confidence=observation.confidence,
                heading_deg=observation.heading_deg,
            )
            state = estimator.estimate()
            if state is None or index < 8:
                continue
            fit_errors_deg.append(abs(smallest_angle_error_deg(state.heading_deg, truth.heading_deg)))
            if state.tracking_state.value == "curve_track":
                curve_states += 1

        self.assertGreater(curve_states, 8)
        self.assertLess(float(np.median(fit_errors_deg)), 25.0)

    def test_pure_magnetic_turn_observation_requires_lateral_excitation(self) -> None:
        observations, _ = self._case6_magnetic_observations(lateral_amplitude_m=0.0)

        offsets = np.array([abs(observation.cross_track_offset_m) for observation in observations], dtype=float)
        self.assertGreater(len(observations), 20)
        self.assertLess(float(np.percentile(offsets, 90.0)), 0.5)

    def test_zigzag_phase_detector_requires_both_sides_before_accepting(self) -> None:
        detector = MagneticZigzagPhaseDetector(
            min_offset_m=1.0,
            min_duration_s=1.0,
            max_duration_s=20.0,
            max_axis_delta_deg=20.0,
        )

        def observation(offset_m: float, x_m: float) -> MagneticPathObservation:
            return MagneticPathObservation(
                position_xy_m=np.array([x_m, 0.0], dtype=float),
                heading_deg=0.0,
                cross_track_offset_m=offset_m,
                confidence=0.8,
            )

        self.assertIsNone(detector.update(observation(1.5, 0.0), time_s=0.0))
        self.assertIsNone(detector.update(observation(2.0, 1.0), time_s=1.0))
        self.assertIsNone(detector.update(observation(-0.4, 2.0), time_s=2.0))

        phase_observation = detector.update(observation(-1.8, 3.0), time_s=4.0)

        self.assertIsNotNone(phase_observation)
        assert phase_observation is not None
        self.assertAlmostEqual(float(phase_observation.observation.position_xy_m[0]), 2.0)
        self.assertAlmostEqual(phase_observation.amplitude_m, 1.9)
        self.assertAlmostEqual(phase_observation.duration_s, 3.0)

    def test_magnetic_lookahead_target_persists_between_phase_events(self) -> None:
        builder = MagneticLookaheadTargetBuilder(
            max_age_s=10.0,
            lookahead_distance_m=5.0,
            heading_blend=1.0,
        )
        phase_observation = MagneticZigzagPhaseObservation(
            observation=MagneticPathObservation(
                position_xy_m=np.array([0.0, 1.0], dtype=float),
                heading_deg=0.0,
                cross_track_offset_m=0.0,
                confidence=0.8,
            ),
            amplitude_m=2.0,
            duration_s=4.0,
        )

        target = builder.update(
            vehicle_position_xy_m=np.array([3.0, 4.0], dtype=float),
            time_s=0.0,
            phase_observation=phase_observation,
        )

        self.assertIsNotNone(target)
        assert target is not None
        np.testing.assert_allclose(target.cable_point_xy_m, np.array([3.0, 1.0]), atol=1e-6)
        np.testing.assert_allclose(target.lookahead_xy_m, np.array([8.0, 1.0]), atol=1e-6)
        self.assertAlmostEqual(target.heading_deg, 0.0)

        persisted = builder.update(
            vehicle_position_xy_m=np.array([6.0, -2.0], dtype=float),
            time_s=5.0,
        )

        self.assertIsNotNone(persisted)
        assert persisted is not None
        np.testing.assert_allclose(persisted.cable_point_xy_m, np.array([6.0, 1.0]), atol=1e-6)
        self.assertLess(persisted.confidence, target.confidence)

    def test_magnetic_lookahead_axis_selection_uses_phase_progress(self) -> None:
        builder = MagneticLookaheadTargetBuilder(
            max_age_s=10.0,
            lookahead_distance_m=5.0,
            heading_blend=1.0,
            axis_selection_enabled=True,
            axis_selection_min_progress_m=2.0,
        )

        def phase(x_m: float, heading_deg: float) -> MagneticZigzagPhaseObservation:
            return MagneticZigzagPhaseObservation(
                observation=MagneticPathObservation(
                    position_xy_m=np.array([x_m, 0.0], dtype=float),
                    heading_deg=heading_deg,
                    cross_track_offset_m=0.0,
                    confidence=0.8,
                ),
                amplitude_m=2.0,
                duration_s=4.0,
            )

        builder.update(
            vehicle_position_xy_m=np.array([0.0, 0.0], dtype=float),
            time_s=0.0,
            phase_observation=phase(0.0, 0.0),
        )
        target = builder.update(
            vehicle_position_xy_m=np.array([6.0, 1.0], dtype=float),
            time_s=4.0,
            phase_observation=phase(5.0, 180.0),
        )

        self.assertIsNotNone(target)
        assert target is not None
        self.assertAlmostEqual(target.heading_deg, 0.0, delta=1e-6)
        np.testing.assert_allclose(target.lookahead_xy_m, np.array([11.0, 0.0]), atol=1e-6)

    def test_magnetic_lookahead_axis_hysteresis_requires_repeated_evidence(self) -> None:
        builder = MagneticLookaheadTargetBuilder(
            max_age_s=10.0,
            lookahead_distance_m=5.0,
            heading_blend=1.0,
            axis_selection_enabled=True,
            axis_selection_min_progress_m=2.0,
            axis_hysteresis_enabled=True,
            axis_hysteresis_threshold=1.5,
            axis_score_decay=0.0,
        )

        def phase(x_m: float, heading_deg: float) -> MagneticZigzagPhaseObservation:
            return MagneticZigzagPhaseObservation(
                observation=MagneticPathObservation(
                    position_xy_m=np.array([x_m, 0.0], dtype=float),
                    heading_deg=heading_deg,
                    cross_track_offset_m=0.0,
                    confidence=0.8,
                ),
                amplitude_m=2.0,
                duration_s=4.0,
            )

        builder.update(
            vehicle_position_xy_m=np.array([0.0, 0.0], dtype=float),
            time_s=0.0,
            phase_observation=phase(0.0, 0.0),
        )
        target = builder.update(
            vehicle_position_xy_m=np.array([6.0, 0.0], dtype=float),
            time_s=4.0,
            phase_observation=phase(5.0, 180.0),
        )

        self.assertIsNotNone(target)
        assert target is not None
        self.assertAlmostEqual(target.heading_deg, 0.0, delta=1e-6)

    def test_shadow_hypothesis_selector_keeps_both_axes_and_selects_progress(self) -> None:
        selector = MagneticShadowHypothesisSelector(
            max_age_s=10.0,
            lookahead_distance_m=5.0,
            min_progress_m=2.0,
        )
        phase_observation = MagneticZigzagPhaseObservation(
            observation=MagneticPathObservation(
                position_xy_m=np.array([0.0, 0.0], dtype=float),
                heading_deg=0.0,
                cross_track_offset_m=0.0,
                confidence=0.8,
            ),
            amplitude_m=2.0,
            duration_s=4.0,
        )

        selected = selector.update(
            vehicle_position_xy_m=np.array([3.0, 0.0], dtype=float),
            vehicle_heading_deg=0.0,
            time_s=0.0,
            phase_observation=phase_observation,
        )

        self.assertIsNotNone(selected)
        assert selected is not None
        self.assertEqual(selected.candidate_count, 2)
        self.assertEqual(len(selected.candidates), 2)
        self.assertTrue(all(isinstance(candidate, MagneticLookaheadHypothesis) for candidate in selected.candidates))
        self.assertGreater(selected.selected_sign, 0.0)
        self.assertGreater(selected.score_margin, 0.1)
        self.assertGreater(selected.positive_score, selected.negative_score)
        np.testing.assert_allclose(selected.target_xy_m, np.array([8.0, 0.0]), atol=1e-6)

    def test_shadow_hypothesis_selector_can_choose_negative_axis(self) -> None:
        selector = MagneticShadowHypothesisSelector(
            max_age_s=10.0,
            lookahead_distance_m=5.0,
            min_progress_m=2.0,
        )
        phase_observation = MagneticZigzagPhaseObservation(
            observation=MagneticPathObservation(
                position_xy_m=np.array([0.0, 0.0], dtype=float),
                heading_deg=0.0,
                cross_track_offset_m=0.0,
                confidence=0.8,
            ),
            amplitude_m=2.0,
            duration_s=4.0,
        )

        selected = selector.update(
            vehicle_position_xy_m=np.array([-3.0, 0.0], dtype=float),
            vehicle_heading_deg=180.0,
            time_s=0.0,
            phase_observation=phase_observation,
        )

        self.assertIsNotNone(selected)
        assert selected is not None
        self.assertLess(selected.selected_sign, 0.0)
        self.assertLess(selected.negative_score, 1.01)
        self.assertGreater(selected.negative_score, selected.positive_score)
        self.assertAlmostEqual(abs(selected.heading_deg), 180.0)

    def test_shadow_axis_validation_reports_pass_and_reject_reasons(self) -> None:
        scenario = build_default_scenarios()["case1"]
        scenario.tracking.magnetic_shadow_hypothesis_enabled = True
        scenario.tracking.magnetic_shadow_validation_min_score = 0.70
        scenario.tracking.magnetic_shadow_validation_min_margin = 0.25
        scenario.tracking.magnetic_shadow_validation_max_age_s = 10.0
        perception = MagneticCablePerception(scenario)

        no_hypothesis = perception._shadow_axis_validation_diagnostics(None, time_s=0.0)
        self.assertEqual(no_hypothesis["reason_code"], 2.0)

        perception.last_magnetic_phase_time_s = 0.0
        expired_selector = perception._shadow_axis_validation_diagnostics(None, time_s=80.0)
        self.assertEqual(expired_selector["reason_code"], 7.0)
        self.assertAlmostEqual(expired_selector["age_over_s"], 20.0)

        low_score = perception._shadow_axis_validation_diagnostics(SimpleNamespace(
            candidate_count=2,
            selected_score=0.60,
            score_margin=0.50,
            age_s=1.0,
        ))
        self.assertEqual(low_score["reason_code"], 4.0)
        self.assertAlmostEqual(low_score["score_deficit"], 0.10)

        low_margin = perception._shadow_axis_validation_diagnostics(SimpleNamespace(
            candidate_count=2,
            selected_score=0.80,
            score_margin=0.10,
            age_s=1.0,
        ))
        self.assertEqual(low_margin["reason_code"], 5.0)
        self.assertAlmostEqual(low_margin["margin_deficit"], 0.15)

        stale = perception._shadow_axis_validation_diagnostics(SimpleNamespace(
            candidate_count=2,
            selected_score=0.80,
            score_margin=0.50,
            age_s=12.0,
        ))
        self.assertEqual(stale["reason_code"], 6.0)
        self.assertAlmostEqual(stale["age_over_s"], 2.0)

        passed = perception._shadow_axis_validation_diagnostics(SimpleNamespace(
            candidate_count=2,
            selected_score=0.80,
            score_margin=0.50,
            age_s=1.0,
        ))
        self.assertEqual(passed["reason_code"], 1.0)
        self.assertEqual(passed["passed"], 1.0)

    def test_shadow_axis_dual_gate_combines_validation_and_feed(self) -> None:
        scenario = build_default_scenarios()["case1"]
        scenario.tracking.magnetic_shadow_dual_gate_shadow_enabled = True
        perception = MagneticCablePerception(scenario)

        disabled = perception._shadow_axis_dual_gate_diagnostics(
            {"passed": 1.0, "reason_code": 1.0},
            {"allowed": 1.0, "reason_code": 1.0},
        )
        self.assertEqual(disabled["enabled"], 1.0)
        self.assertEqual(disabled["reason_code"], 1.0)
        self.assertEqual(disabled["passed"], 1.0)

        scenario.tracking.magnetic_shadow_dual_gate_shadow_enabled = False
        off = perception._shadow_axis_dual_gate_diagnostics(
            {"passed": 1.0, "reason_code": 1.0},
            {"allowed": 1.0, "reason_code": 1.0},
        )
        self.assertEqual(off["enabled"], 0.0)
        self.assertEqual(off["reason_code"], 0.0)

        scenario.tracking.magnetic_shadow_dual_gate_shadow_enabled = True
        validation_reject = perception._shadow_axis_dual_gate_diagnostics(
            {"passed": 0.0, "reason_code": 4.0},
            {"allowed": 1.0, "reason_code": 1.0},
        )
        self.assertEqual(validation_reject["reason_code"], 2.0)
        self.assertEqual(validation_reject["passed"], 0.0)

        feed_reject = perception._shadow_axis_dual_gate_diagnostics(
            {"passed": 1.0, "reason_code": 1.0},
            {"allowed": 0.0, "reason_code": 5.0},
        )
        self.assertEqual(feed_reject["reason_code"], 3.0)
        self.assertEqual(feed_reject["passed"], 0.0)

    def test_shadow_axis_progress_alignment_rejects_reverse_candidate(self) -> None:
        scenario = build_default_scenarios()["case1"]
        scenario.tracking.magnetic_shadow_progress_alignment_enabled = True
        perception = MagneticCablePerception(scenario)
        reverse_candidate = MagneticLookaheadHypothesis(
            hypothesis_id="reverse",
            axis_sign=-1.0,
            anchor_xy_m=np.array([0.0, 0.0], dtype=float),
            direction_xy=np.array([-1.0, 0.0], dtype=float),
            cable_point_xy_m=np.array([0.0, 0.0], dtype=float),
            lookahead_xy_m=np.array([-20.0, 0.0], dtype=float),
            heading_deg=180.0,
            confidence=0.9,
            age_s=1.0,
            score=0.9,
            progress_score=1.0,
            heading_score=1.0,
            freshness_score=1.0,
        )
        selection = SimpleNamespace(selected_candidate=reverse_candidate)
        local_state = LocalCableState(
            model="line",
            anchor_xy_m=np.array([0.0, 0.0], dtype=float),
            tangent_xy=np.array([1.0, 0.0], dtype=float),
            heading_deg=0.0,
            residual_m=0.1,
            confidence=0.8,
            latest_time_s=10.0,
        )
        proxy = perception._update_shadow_progress_proxy_diagnostics(10.0, None, local_state)
        self.assertEqual(proxy["valid"], 1.0)
        self.assertEqual(proxy["source_code"], 2.0)

        rejected = perception._shadow_axis_progress_alignment_diagnostics(selection, 10.0, proxy)
        self.assertEqual(rejected["enabled"], 1.0)
        self.assertEqual(rejected["reason_code"], 6.0)
        self.assertLess(rejected["alignment_dot"], 0.0)

        forward_candidate = MagneticLookaheadHypothesis(
            hypothesis_id="forward",
            axis_sign=1.0,
            anchor_xy_m=np.array([0.0, 0.0], dtype=float),
            direction_xy=np.array([1.0, 0.0], dtype=float),
            cable_point_xy_m=np.array([0.0, 0.0], dtype=float),
            lookahead_xy_m=np.array([20.0, 0.0], dtype=float),
            heading_deg=0.0,
            confidence=0.9,
            age_s=1.0,
            score=0.9,
            progress_score=1.0,
            heading_score=1.0,
            freshness_score=1.0,
        )
        passed = perception._shadow_axis_progress_alignment_diagnostics(
            SimpleNamespace(selected_candidate=forward_candidate),
            10.0,
              proxy,
        )
        self.assertEqual(passed["reason_code"], 1.0)
        self.assertEqual(passed["passed"], 1.0)

        selection = SimpleNamespace(candidates=(reverse_candidate, forward_candidate))
        selected = perception._shadow_axis_progress_aligned_candidate_selection_diagnostics(
            selection,
            10.0,
              proxy,
        )
        self.assertEqual(selected["reason_code"], 1.0)
        self.assertEqual(selected["valid"], 1.0)
        self.assertGreater(selected["selected_sign"], 0.0)
        self.assertAlmostEqual(selected["selected_score"], forward_candidate.score)
        self.assertAlmostEqual(selected["task_progress_dot"], 1.0)
        self.assertAlmostEqual(selected["task_score"], 1.0)
        self.assertGreater(selected["combined_score"], selected["selected_score"])

    def test_shadow_progress_aligned_selection_can_prioritize_task_score(self) -> None:
        scenario = build_default_scenarios()["case1"]
        scenario.tracking.magnetic_shadow_progress_alignment_enabled = True
        scenario.tracking.magnetic_shadow_progress_alignment_min_dot = -1.0
        scenario.tracking.magnetic_shadow_task_score_motion_weight = 0.2
        scenario.tracking.magnetic_shadow_task_score_progress_weight = 0.8
        perception = MagneticCablePerception(scenario)
        local_state = LocalCableState(
            model="line",
            anchor_xy_m=np.array([0.0, 0.0], dtype=float),
            tangent_xy=np.array([1.0, 0.0], dtype=float),
            heading_deg=0.0,
            residual_m=0.1,
            confidence=0.8,
            latest_time_s=10.0,
        )
        proxy = perception._update_shadow_progress_proxy_diagnostics(10.0, None, local_state)
        high_motion_reverse = MagneticLookaheadHypothesis(
            hypothesis_id="reverse",
            axis_sign=-1.0,
            anchor_xy_m=np.array([0.0, 0.0], dtype=float),
            direction_xy=np.array([-1.0, 0.0], dtype=float),
            cable_point_xy_m=np.array([0.0, 0.0], dtype=float),
            lookahead_xy_m=np.array([-20.0, 0.0], dtype=float),
            heading_deg=180.0,
            confidence=0.9,
            age_s=1.0,
            score=0.95,
            progress_score=1.0,
            heading_score=1.0,
            freshness_score=1.0,
        )
        low_motion_forward = MagneticLookaheadHypothesis(
            hypothesis_id="forward",
            axis_sign=1.0,
            anchor_xy_m=np.array([0.0, 0.0], dtype=float),
            direction_xy=np.array([1.0, 0.0], dtype=float),
            cable_point_xy_m=np.array([0.0, 0.0], dtype=float),
            lookahead_xy_m=np.array([20.0, 0.0], dtype=float),
            heading_deg=0.0,
            confidence=0.9,
            age_s=1.0,
            score=0.30,
            progress_score=0.2,
            heading_score=0.2,
            freshness_score=1.0,
        )

        selected = perception._shadow_axis_progress_aligned_candidate_selection_diagnostics(
            SimpleNamespace(candidates=(high_motion_reverse, low_motion_forward)),
            10.0,
            proxy,
        )
        self.assertGreater(selected["selected_sign"], 0.0)
        self.assertAlmostEqual(selected["selected_score"], 0.30)
        self.assertAlmostEqual(selected["task_score"], 1.0)
        self.assertGreater(selected["combined_score"], 0.80)

    def test_shadow_progress_proxy_holds_last_direction(self) -> None:
        scenario = build_default_scenarios()["case1"]
        scenario.tracking.magnetic_shadow_progress_alignment_enabled = True
        scenario.tracking.magnetic_shadow_progress_proxy_hold_enabled = True
        scenario.tracking.magnetic_shadow_progress_alignment_max_age_s = 20.0
        perception = MagneticCablePerception(scenario)
        local_state = LocalCableState(
            model="line",
            anchor_xy_m=np.array([0.0, 0.0], dtype=float),
            tangent_xy=np.array([1.0, 0.0], dtype=float),
            heading_deg=0.0,
            residual_m=0.1,
            confidence=0.8,
            latest_time_s=10.0,
        )

        fresh = perception._update_shadow_progress_proxy_diagnostics(10.0, None, local_state)
        self.assertEqual(fresh["valid"], 1.0)
        self.assertEqual(fresh["source_code"], 2.0)

        low_conf_state = LocalCableState(
            model="line",
            anchor_xy_m=np.array([0.0, 0.0], dtype=float),
            tangent_xy=np.array([0.0, 1.0], dtype=float),
            heading_deg=90.0,
            residual_m=0.1,
            confidence=0.1,
            latest_time_s=11.0,
        )
        held = perception._update_shadow_progress_proxy_diagnostics(15.0, None, low_conf_state)
        self.assertEqual(held["valid"], 1.0)
        self.assertEqual(held["source_code"], 1.0)
        np.testing.assert_allclose(held["direction_xy"], np.array([1.0, 0.0]), atol=1e-9)

    def test_route_bound_progress_proxy_uses_nominal_route_direction(self) -> None:
        scenario = build_default_scenarios()["case1"]
        scenario.tracking.magnetic_shadow_progress_alignment_enabled = True
        scenario.tracking.magnetic_shadow_route_bound_progress_proxy_enabled = True
        perception = MagneticCablePerception(scenario)
        route_proxy = perception._shadow_route_bound_progress_proxy_diagnostics(
            np.array([5.0, 3.0], dtype=float),
            None,
            None,
        )
        self.assertEqual(route_proxy["valid"], 1.0)
        self.assertEqual(route_proxy["source_code"], 2.0)
        self.assertGreater(route_proxy["progress_m"], 0.0)
        self.assertGreater(route_proxy["direction_xy"][0], 0.0)

        forward_candidate = MagneticLookaheadHypothesis(
            hypothesis_id="forward",
            axis_sign=1.0,
            anchor_xy_m=np.array([0.0, 0.0], dtype=float),
            direction_xy=np.array([1.0, 0.0], dtype=float),
            cable_point_xy_m=np.array([0.0, 0.0], dtype=float),
            lookahead_xy_m=np.array([20.0, 0.0], dtype=float),
            heading_deg=0.0,
            confidence=0.9,
            age_s=1.0,
            score=0.4,
            progress_score=0.2,
            heading_score=0.2,
            freshness_score=1.0,
        )
        reverse_candidate = MagneticLookaheadHypothesis(
            hypothesis_id="reverse",
            axis_sign=-1.0,
            anchor_xy_m=np.array([0.0, 0.0], dtype=float),
            direction_xy=np.array([-1.0, 0.0], dtype=float),
            cable_point_xy_m=np.array([0.0, 0.0], dtype=float),
            lookahead_xy_m=np.array([-20.0, 0.0], dtype=float),
            heading_deg=180.0,
            confidence=0.9,
            age_s=1.0,
            score=0.9,
            progress_score=1.0,
            heading_score=1.0,
            freshness_score=1.0,
        )
        selected = perception._shadow_axis_progress_aligned_candidate_selection_diagnostics(
            SimpleNamespace(candidates=(reverse_candidate, forward_candidate)),
            10.0,
            route_proxy,
        )
        self.assertEqual(selected["valid"], 1.0)
        self.assertGreater(selected["selected_sign"], 0.0)
        self.assertAlmostEqual(selected["task_progress_dot"], 1.0)

    def test_zigzag_phase_detector_reports_reject_reasons(self) -> None:
        detector = MagneticZigzagPhaseDetector(
            min_offset_m=1.0,
            min_duration_s=2.0,
            max_duration_s=5.0,
            max_axis_delta_deg=20.0,
        )

        def obs(offset_m: float, heading_deg: float = 0.0) -> MagneticPathObservation:
            return MagneticPathObservation(
                position_xy_m=np.array([0.0, offset_m], dtype=float),
                heading_deg=heading_deg,
                cross_track_offset_m=offset_m,
                confidence=0.8,
            )

        self.assertIsNone(detector.update(obs(0.0), 0.0))
        self.assertEqual(detector.last_reason_code, 2.0)
        self.assertIsNone(detector.update(obs(1.2), 0.0))
        self.assertEqual(detector.last_reason_code, 3.0)
        self.assertIsNone(detector.update(obs(-1.2), 1.0))
        self.assertEqual(detector.last_reason_code, 4.0)
        self.assertIsNone(detector.update(obs(-1.3), 1.5))
        self.assertEqual(detector.last_reason_code, 7.0)
        self.assertAlmostEqual(detector.last_duration_s, 1.5)
        self.assertIsNone(detector.update(obs(-1.4, heading_deg=35.0), 3.0))
        self.assertEqual(detector.last_reason_code, 9.0)
        self.assertAlmostEqual(detector.last_axis_delta_deg, 35.0)
        self.assertIsNotNone(detector.update(obs(-1.5, heading_deg=0.0), 3.2))
        self.assertEqual(detector.last_reason_code, 1.0)


if __name__ == "__main__":
    unittest.main()
