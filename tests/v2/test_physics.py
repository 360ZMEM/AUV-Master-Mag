from __future__ import annotations

import math
import unittest

import numpy as np

from auv_mag_tracking.v2 import physical_finite_wire_field_nT


class PhysicalFiniteWireTests(unittest.TestCase):
    def test_long_wire_approaches_infinite_wire_limit(self) -> None:
        current_a = 10.0
        radius_m = 2.0
        field = physical_finite_wire_field_nT(
            np.array([0.0, radius_m, 0.0]),
            np.array([-10000.0, 0.0, 0.0]),
            np.array([10000.0, 0.0, 0.0]),
            current_a,
        )
        expected_nt = 4.0 * math.pi * 1e-7 * current_a / (2.0 * math.pi * radius_m) * 1e9
        self.assertAlmostEqual(float(np.linalg.norm(field)), expected_nt, delta=expected_nt * 1e-6)

    def test_field_has_right_hand_rule_direction(self) -> None:
        field = physical_finite_wire_field_nT(
            np.array([0.0, 1.0, 0.0]),
            np.array([-10.0, 0.0, 0.0]),
            np.array([10.0, 0.0, 0.0]),
            1.0,
        )
        self.assertGreater(field[2], 0.0)
        self.assertAlmostEqual(field[0], 0.0)
        self.assertAlmostEqual(field[1], 0.0)


if __name__ == "__main__":
    unittest.main()
