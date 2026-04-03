import sys
import unittest
from pathlib import Path

import numpy as np


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.math_utils import body_to_ned, finite_wire_field_nT, ned_to_body


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


if __name__ == "__main__":
    unittest.main()
