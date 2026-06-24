import sys
import unittest
from pathlib import Path

import numpy as np


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.math_utils import smallest_angle_error_deg
from auv_mag_tracking.perception import LocalCableStateEstimator


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
            heading_blend=0.50,
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
