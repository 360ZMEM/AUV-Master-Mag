import sys
import unittest
from pathlib import Path

import numpy as np


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.math_utils import (
    body_to_ned,
    estimate_polyline_curvature,
    finite_wire_field_nT,
    ned_to_body,
    sample_tightening_arc_path,
)


class MathUtilsTest(unittest.TestCase):
    def test_body_ned_roundtrip(self) -> None:
        vector_body = np.array([120.0, -50.0, 35.0])
        vector_ned = body_to_ned(vector_body, roll_deg=12.0, pitch_deg=-8.0, yaw_deg=35.0)
        restored = ned_to_body(vector_ned, roll_deg=12.0, pitch_deg=-8.0, yaw_deg=35.0)
        self.assertTrue(np.allclose(vector_body, restored, atol=1e-8))

    def test_finite_wire_field_decreases_with_distance(self) -> None:
        start = np.array([-100.0, 0.0, 30.0])
        end = np.array([100.0, 0.0, 30.0])
        near_point = np.array([0.0, 8.0, 25.0])
        far_point = np.array([0.0, 20.0, 25.0])
        near_field = np.linalg.norm(finite_wire_field_nT(near_point, start, end, 600.0))
        far_field = np.linalg.norm(finite_wire_field_nT(far_point, start, end, 600.0))
        self.assertGreater(near_field, far_field)

    def test_tightening_arc_straight_then_constant_radius(self) -> None:
        radius_m = 40.0
        route = sample_tightening_arc_path(
            initial_straight_length_m=120.0,
            turn_angle_deg=90.0,
            radius_m=radius_m,
            step_m=2.0,
        )
        # First segment is a straight run along +x at y≈0.
        head = route[:30]
        self.assertLess(float(np.max(np.abs(head[:, 1]))), 1e-6)
        self.assertTrue(np.all(np.diff(head[:, 0]) > 0.0))
        # Arc minimum curvature radius equals radius_m (within discretisation).
        radii = []
        for i in range(1, len(route) - 1):
            kappa = estimate_polyline_curvature(route, i)
            if abs(kappa) > 1e-6:
                radii.append(1.0 / abs(kappa))
        self.assertTrue(radii)
        self.assertAlmostEqual(min(radii), radius_m, delta=2.0)
        # Total heading change ≈ 90 degrees (start east, end north).
        end_tangent = route[-1] - route[-2]
        end_heading = np.rad2deg(np.arctan2(end_tangent[1], end_tangent[0]))
        self.assertAlmostEqual(end_heading, 90.0, delta=3.0)


if __name__ == "__main__":
    unittest.main()
