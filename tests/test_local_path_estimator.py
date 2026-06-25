import sys
import unittest
from pathlib import Path

import numpy as np


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.math_utils import smallest_angle_error_deg
from auv_mag_tracking.perception import LocalCableStateEstimator, LocalPathTrackingState


def _arc_points(
    center_xy: np.ndarray,
    radius_m: float,
    start_deg: float,
    sweep_deg: float,
    count: int,
    noise_scale_m: float = 0.0,
) -> tuple:
    angles_deg = np.linspace(start_deg, start_deg + sweep_deg, count)
    angles_rad = np.deg2rad(angles_deg)
    points = center_xy[None, :] + radius_m * np.column_stack([np.cos(angles_rad), np.sin(angles_rad)])
    if noise_scale_m > 0.0:
        noise = noise_scale_m * np.column_stack([
            np.sin(np.arange(count) * 1.7),
            np.cos(np.arange(count) * 1.3),
        ])
        points = points + noise
    return points, angles_deg


def _expected_tangent_heading_deg(angle_deg: float, sweep_deg: float) -> float:
    angle_rad = np.deg2rad(angle_deg)
    if sweep_deg >= 0.0:
        tangent = np.array([-np.sin(angle_rad), np.cos(angle_rad)], dtype=float)
    else:
        tangent = np.array([np.sin(angle_rad), -np.cos(angle_rad)], dtype=float)
    return float(np.rad2deg(np.arctan2(tangent[1], tangent[0])))


def _maze_u_turn_points(radius_m: float = 60.0) -> tuple:
    line1_x = np.linspace(-140.0, 0.0, 18)
    line1 = np.column_stack([line1_x, np.zeros_like(line1_x)])
    line1_heading = np.zeros(line1.shape[0], dtype=float)

    arc_angles_deg = np.linspace(-90.0, 90.0, 28)
    arc_angles_rad = np.deg2rad(arc_angles_deg)
    center = np.array([0.0, radius_m], dtype=float)
    arc = center[None, :] + radius_m * np.column_stack([np.cos(arc_angles_rad), np.sin(arc_angles_rad)])
    arc_heading = arc_angles_deg + 90.0

    line2_x = np.linspace(0.0, -140.0, 18)
    line2 = np.column_stack([line2_x, np.full_like(line2_x, 2.0 * radius_m)])
    line2_heading = np.full(line2.shape[0], 180.0, dtype=float)

    points = np.vstack([line1[:-1], arc, line2[1:]])
    headings = np.concatenate([line1_heading[:-1], arc_heading, line2_heading[1:]])
    return points, headings


def _s_curve_points(radius_m: float = 66.0) -> tuple:
    first_angles_deg = np.linspace(-90.0, -10.0, 20)
    first_center = np.array([0.0, radius_m], dtype=float)
    first = first_center[None, :] + radius_m * np.column_stack([
        np.cos(np.deg2rad(first_angles_deg)),
        np.sin(np.deg2rad(first_angles_deg)),
    ])
    first_heading = first_angles_deg + 90.0

    start_second = first[-1]
    start_heading_deg = first_heading[-1]
    heading_rad = np.deg2rad(start_heading_deg)
    right_normal = np.array([np.sin(heading_rad), -np.cos(heading_rad)], dtype=float)
    second_center = start_second + radius_m * right_normal
    start_radial = start_second - second_center
    start_angle_deg = np.rad2deg(np.arctan2(start_radial[1], start_radial[0]))
    second_angles_deg = np.linspace(start_angle_deg, start_angle_deg - 95.0, 24)
    second = second_center[None, :] + radius_m * np.column_stack([
        np.cos(np.deg2rad(second_angles_deg)),
        np.sin(np.deg2rad(second_angles_deg)),
    ])
    second_heading = second_angles_deg - 90.0

    points = np.vstack([first, second[1:]])
    headings = np.concatenate([first_heading, second_heading[1:]])
    return points, headings


class LocalCableStateEstimatorTest(unittest.TestCase):
    def test_prefers_line_for_straight_observations(self) -> None:
        estimator = LocalCableStateEstimator(capacity=16, min_arc_radius_m=30.0)
        for index, x_m in enumerate(np.linspace(0.0, 30.0, 10)):
            point = np.array([x_m, 0.08 * np.sin(index)], dtype=float)
            estimator.add_observation(point, time_s=float(index), confidence=0.9)

        state = estimator.estimate()

        self.assertIsNotNone(state)
        self.assertEqual(state.model, "line")
        self.assertLess(abs(smallest_angle_error_deg(state.heading_deg, 0.0)), 2.0)
        self.assertLess(state.residual_m, 0.1)

    def test_fits_short_large_angle_arc_with_radius_above_30m(self) -> None:
        estimator = LocalCableStateEstimator(
            capacity=24,
            min_arc_radius_m=30.0,
            min_arc_angle_span_deg=45.0,
        )
        center = np.array([12.0, -7.0], dtype=float)
        radius_m = 38.0
        sweep_deg = 120.0
        points, angles_deg = _arc_points(
            center,
            radius_m=radius_m,
            start_deg=-65.0,
            sweep_deg=sweep_deg,
            count=18,
            noise_scale_m=0.12,
        )
        for index, point in enumerate(points):
            estimator.add_observation(point, time_s=float(index), confidence=0.85)

        state = estimator.estimate()

        self.assertIsNotNone(state)
        self.assertEqual(state.model, "arc")
        self.assertGreater(state.radius_m, 30.0)
        self.assertAlmostEqual(state.radius_m, radius_m, delta=1.2)
        self.assertGreater(state.arc_angle_span_deg, 110.0)
        self.assertLess(state.residual_m, 0.25)

        expected_heading = _expected_tangent_heading_deg(angles_deg[-1], sweep_deg)
        self.assertLess(abs(smallest_angle_error_deg(state.heading_deg, expected_heading)), 3.0)
        self.assertGreater(state.curvature_1pm, 0.0)

    def test_local_line_heading_updates_along_curve_when_arc_is_disabled(self) -> None:
        estimator = LocalCableStateEstimator(
            capacity=10,
            local_line_window=5,
            min_arc_radius_m=30.0,
            min_arc_angle_span_deg=180.0,
            heading_blend=0.65,
        )
        points, angles_deg = _arc_points(
            np.array([0.0, 0.0], dtype=float),
            radius_m=40.0,
            start_deg=-70.0,
            sweep_deg=120.0,
            count=15,
            noise_scale_m=0.04,
        )
        sampled_headings = []
        expected_headings = []
        for index, (point, angle_deg) in enumerate(zip(points, angles_deg)):
            heading_deg = _expected_tangent_heading_deg(angle_deg, 120.0)
            estimator.add_observation(point, time_s=float(index), confidence=0.9, heading_deg=heading_deg)
            if index >= 5:
                state = estimator.estimate()
                self.assertIsNotNone(state)
                self.assertEqual(state.model, "local_line")
                sampled_headings.append(state.heading_deg)
                expected_headings.append(heading_deg)

        self.assertGreater(abs(smallest_angle_error_deg(sampled_headings[-1], sampled_headings[0])), 55.0)
        self.assertLess(abs(smallest_angle_error_deg(sampled_headings[-1], expected_headings[-1])), 8.0)
        self.assertLess(abs(smallest_angle_error_deg(sampled_headings[0], expected_headings[0])), 12.0)

    def test_noisy_local_line_remains_usable_when_arc_is_disabled(self) -> None:
        rng = np.random.default_rng(20260624)
        estimator = LocalCableStateEstimator(
            capacity=14,
            local_line_window=5,
            min_arc_radius_m=30.0,
            min_arc_angle_span_deg=180.0,
            heading_blend=0.65,
        )
        radius_m = 40.0
        points, angles_deg = _arc_points(
            np.array([5.0, -3.0], dtype=float),
            radius_m=radius_m,
            start_deg=-70.0,
            sweep_deg=120.0,
            count=18,
        )
        heading_errors = []
        for index, (point, angle_deg) in enumerate(zip(points, angles_deg)):
            noisy_point = point + rng.normal(0.0, 0.45, size=2)
            true_heading_deg = _expected_tangent_heading_deg(angle_deg, 120.0)
            noisy_heading_deg = true_heading_deg + rng.normal(0.0, 5.0)
            estimator.add_observation(noisy_point, time_s=float(index), confidence=0.85, heading_deg=noisy_heading_deg)
            if index >= 6:
                state = estimator.estimate()
                self.assertIsNotNone(state)
                self.assertEqual(state.model, "local_line")
                heading_errors.append(abs(smallest_angle_error_deg(state.heading_deg, true_heading_deg)))

        self.assertLess(float(np.mean(heading_errors)), 7.0)
        self.assertLess(float(np.percentile(heading_errors, 90.0)), 11.0)

    def test_state_machine_tracks_maze_scale_u_turn_with_local_tangent(self) -> None:
        rng = np.random.default_rng(20260625)
        estimator = LocalCableStateEstimator(
            capacity=20,
            local_line_window=5,
            min_arc_radius_m=30.0,
            heading_blend=0.65,
            curve_heading_delta_deg=12.0,
        )
        points, headings = _maze_u_turn_points(radius_m=60.0)
        heading_errors = []
        global_line_errors = []
        step_changes = []
        curve_states = 0
        previous_heading = None
        for index, (point, heading_deg) in enumerate(zip(points, headings)):
            noisy_point = point + rng.normal(0.0, 0.35, size=2)
            noisy_heading = heading_deg + rng.normal(0.0, 3.0)
            estimator.add_observation(noisy_point, time_s=float(index), confidence=0.9, heading_deg=noisy_heading)
            state = estimator.estimate()
            if state is None or index < 8:
                continue
            heading_error = abs(smallest_angle_error_deg(state.heading_deg, heading_deg))
            heading_errors.append(heading_error)
            if state.tracking_state == LocalPathTrackingState.CURVE_TRACK:
                curve_states += 1
            if previous_heading is not None:
                step_changes.append(abs(smallest_angle_error_deg(state.heading_deg, previous_heading)))
            previous_heading = state.heading_deg

            global_direction = points[index] - points[max(0, index - estimator.capacity + 1)]
            global_heading = float(np.rad2deg(np.arctan2(global_direction[1], global_direction[0])))
            global_line_errors.append(abs(smallest_angle_error_deg(global_heading, heading_deg)))

        self.assertGreater(curve_states, 12)
        self.assertLess(float(np.mean(heading_errors)), 9.0)
        self.assertLess(float(np.percentile(heading_errors, 90.0)), 16.0)
        self.assertLess(float(np.max(step_changes)), 35.0)
        self.assertGreater(float(np.mean(global_line_errors)), float(np.mean(heading_errors)) + 8.0)

    def test_state_machine_tracks_case6_style_s_curve_without_heading_flip(self) -> None:
        rng = np.random.default_rng(20260626)
        estimator = LocalCableStateEstimator(
            capacity=22,
            local_line_window=5,
            min_arc_radius_m=30.0,
            min_arc_angle_span_deg=180.0,
            heading_blend=0.65,
            curve_heading_delta_deg=10.0,
        )
        points, headings = _s_curve_points(radius_m=66.0)
        heading_errors = []
        step_changes = []
        curve_states = 0
        previous_heading = None
        for index, (point, heading_deg) in enumerate(zip(points, headings)):
            noisy_point = point + rng.normal(0.0, 0.45, size=2)
            noisy_heading = heading_deg + rng.normal(0.0, 5.0)
            estimator.add_observation(noisy_point, time_s=float(index), confidence=0.85, heading_deg=noisy_heading)
            state = estimator.estimate()
            if state is None or index < 8:
                continue
            heading_errors.append(abs(smallest_angle_error_deg(state.heading_deg, heading_deg)))
            if state.tracking_state == LocalPathTrackingState.CURVE_TRACK:
                curve_states += 1
            if previous_heading is not None:
                step_changes.append(abs(smallest_angle_error_deg(state.heading_deg, previous_heading)))
            previous_heading = state.heading_deg

        self.assertGreater(curve_states, 20)
        self.assertLess(float(np.mean(heading_errors)), 10.0)
        self.assertLess(float(np.percentile(heading_errors, 90.0)), 18.0)
        self.assertLess(float(np.max(step_changes)), 40.0)

    def test_state_machine_reacquires_after_observation_gap(self) -> None:
        estimator = LocalCableStateEstimator(
            capacity=12,
            local_line_window=5,
            reacquire_gap_s=4.0,
        )
        first_points, first_headings = _maze_u_turn_points(radius_m=60.0)
        for index, (point, heading_deg) in enumerate(zip(first_points[:14], first_headings[:14])):
            estimator.add_observation(point, time_s=float(index), confidence=0.9, heading_deg=heading_deg)
            estimator.estimate()

        estimator.add_observation(np.array([200.0, 0.0], dtype=float), time_s=30.0, confidence=0.9, heading_deg=0.0)
        self.assertEqual(estimator.tracking_state, LocalPathTrackingState.REACQUIRE)
        self.assertIsNone(estimator.estimate())

        estimator.add_observation(np.array([210.0, 0.0], dtype=float), time_s=31.0, confidence=0.9, heading_deg=0.0)
        state = estimator.estimate()

        self.assertIsNotNone(state)
        self.assertIn(state.tracking_state, {LocalPathTrackingState.COLLECTING, LocalPathTrackingState.LINE_TRACK})
        self.assertLess(abs(smallest_angle_error_deg(state.heading_deg, 0.0)), 5.0)

    def test_state_machine_restores_curve_track_after_reacquire_gap(self) -> None:
        estimator = LocalCableStateEstimator(
            capacity=18,
            local_line_window=5,
            heading_blend=0.65,
            curve_heading_delta_deg=10.0,
            reacquire_gap_s=4.0,
        )
        first_points, first_headings = _maze_u_turn_points(radius_m=60.0)
        for index, (point, heading_deg) in enumerate(zip(first_points[:16], first_headings[:16])):
            estimator.add_observation(point, time_s=float(index), confidence=0.9, heading_deg=heading_deg)
            estimator.estimate()

        second_points, second_headings = _maze_u_turn_points(radius_m=60.0)
        restored_states = []
        start_time_s = 40.0
        for index, (point, heading_deg) in enumerate(zip(second_points[16:30], second_headings[16:30])):
            estimator.add_observation(point, time_s=start_time_s + float(index), confidence=0.9, heading_deg=heading_deg)
            state = estimator.estimate()
            if state is not None:
                restored_states.append((state, heading_deg))

        self.assertGreater(len(restored_states), 4)
        self.assertEqual(restored_states[-1][0].tracking_state, LocalPathTrackingState.CURVE_TRACK)
        self.assertLess(
            abs(smallest_angle_error_deg(restored_states[-1][0].heading_deg, restored_states[-1][1])),
            12.0,
        )

    def test_clockwise_arc_reports_negative_curvature(self) -> None:
        estimator = LocalCableStateEstimator(capacity=18, min_arc_radius_m=30.0)
        points, angles_deg = _arc_points(
            np.array([-4.0, 3.0], dtype=float),
            radius_m=42.0,
            start_deg=80.0,
            sweep_deg=-105.0,
            count=16,
            noise_scale_m=0.08,
        )
        for index, point in enumerate(points):
            estimator.add_observation(point, time_s=float(index), confidence=0.9)

        state = estimator.estimate()

        self.assertIsNotNone(state)
        self.assertEqual(state.model, "arc")
        self.assertLess(state.curvature_1pm, 0.0)
        expected_heading = _expected_tangent_heading_deg(angles_deg[-1], -105.0)
        self.assertLess(abs(smallest_angle_error_deg(state.heading_deg, expected_heading)), 3.0)

    def test_exports_fit_result_compatible_tangent(self) -> None:
        estimator = LocalCableStateEstimator(capacity=12)
        points, _ = _arc_points(
            np.array([0.0, 0.0], dtype=float),
            radius_m=36.0,
            start_deg=-30.0,
            sweep_deg=90.0,
            count=12,
        )
        for index, point in enumerate(points):
            estimator.add_observation(point, time_s=float(index), confidence=1.0)

        state = estimator.estimate()
        fit_result = state.as_fit_result()

        self.assertIsNotNone(fit_result.origin_xy_m)
        self.assertIsNotNone(fit_result.direction_xy)
        self.assertEqual(fit_result.covariance_xy_m2.shape, (2, 2))
        np.testing.assert_allclose(fit_result.direction_xy, state.tangent_xy)


if __name__ == "__main__":
    unittest.main()
