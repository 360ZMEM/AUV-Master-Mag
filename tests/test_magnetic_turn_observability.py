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
    LocalCableStateEstimator,
    MagneticCablePerception,
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
        self.assertGreater(selected.selected_sign, 0.0)
        self.assertGreater(selected.score_margin, 0.1)
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


if __name__ == "__main__":
    unittest.main()
