from __future__ import annotations

import unittest

import numpy as np

from auv_mag_tracking.v2 import (
    CrossScanConfig,
    MissionPhase,
    TrackingConfig,
    V2StateMachine,
    build_cross_scan,
    compute_tracking_correction,
)


class CrossScanTests(unittest.TestCase):
    def test_cross_scan_uses_cable_normal_and_expands_by_attempt(self) -> None:
        config = CrossScanConfig(half_span_levels_m=(2.0, 4.0, 6.0))
        plan = build_cross_scan(
            scan_id=7,
            center_xy_m=np.array([10.0, 5.0]),
            tangent_xy=np.array([1.0, 0.0]),
            current_position_xy_m=np.array([10.0, 4.0]),
            config=config,
            attempt_index=1,
        )
        self.assertTrue(plan.valid)
        self.assertEqual(plan.scan_id, 7)
        self.assertAlmostEqual(plan.half_span_m, 4.0)
        np.testing.assert_allclose(plan.waypoints_xy_m[0], [10.0, 1.0])
        np.testing.assert_allclose(plan.waypoints_xy_m[1], [10.0, 9.0])


class TrackingTests(unittest.TestCase):
    def test_tracking_is_bounded_and_confidence_schedules_speed(self) -> None:
        config = TrackingConfig(max_heading_correction_deg=15.0, max_lateral_correction_m=2.0)
        low = compute_tracking_correction(20.0, 0.0, config)
        high = compute_tracking_correction(20.0, 1.0, config)
        self.assertEqual(low.heading_correction_deg, -15.0)
        self.assertEqual(low.lateral_correction_m, -2.0)
        self.assertTrue(low.limited)
        self.assertLess(low.speed_mps, high.speed_mps)


class StateMachineTests(unittest.TestCase):
    def test_nominal_and_reacquire_transitions(self) -> None:
        machine = V2StateMachine()
        self.assertEqual(machine.phase, MissionPhase.SEARCH)
        self.assertEqual(machine.update(detected=True).phase, MissionPhase.CONFIRM)
        self.assertEqual(
            machine.update(detected=True, scan_complete=True, fwhm_valid=True).phase,
            MissionPhase.TRACK,
        )
        self.assertEqual(machine.update(detected=False, tracking_confidence=0.1).phase, MissionPhase.REACQUIRE)
        self.assertEqual(machine.update(detected=True).phase, MissionPhase.CONFIRM)


if __name__ == "__main__":
    unittest.main()
