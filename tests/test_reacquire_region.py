import sys
import unittest
from pathlib import Path

import numpy as np


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.perception.reacquire_region import ObservableRegionSelector


class ObservableRegionSelectorTest(unittest.TestCase):
    def test_forward_gate_uses_last_trusted_heading_without_truth_route(self) -> None:
        selector = ObservableRegionSelector(forward_distance_m=40.0, half_length_m=20.0, half_width_m=10.0)
        selector.update_trusted_state(
            anchor_xy_m=np.array([10.0, 5.0], dtype=float),
            heading_deg=0.0,
            confidence=0.8,
            time_s=10.0,
        )

        region = selector.select(
            time_s=15.0,
            vehicle_position_xy_m=np.array([0.0, 0.0], dtype=float),
            reacquire_required=True,
        )

        self.assertIsNotNone(region)
        self.assertEqual(region.reason, "forward_gate")
        np.testing.assert_allclose(region.center_xy_m, np.array([50.0, 5.0]), atol=1e-6)
        self.assertAlmostEqual(region.heading_deg, 0.0)
        self.assertGreater(region.score, 0.0)

    def test_turn_side_gate_wins_when_curvature_is_observable(self) -> None:
        selector = ObservableRegionSelector(
            forward_distance_m=40.0,
            turn_lateral_offset_m=60.0,
            half_length_m=20.0,
            half_width_m=10.0,
            min_turn_curvature_1pm=1.0 / 200.0,
        )
        selector.update_trusted_state(
            anchor_xy_m=np.array([0.0, 0.0], dtype=float),
            heading_deg=0.0,
            confidence=0.9,
            time_s=0.0,
            curvature_1pm=1.0 / 60.0,
        )

        region = selector.select(
            time_s=5.0,
            vehicle_position_xy_m=np.array([0.0, 0.0], dtype=float),
            reacquire_required=True,
        )

        self.assertIsNotNone(region)
        self.assertEqual(region.reason, "turn_side_gate")
        self.assertGreater(region.center_xy_m[1], 40.0)
        self.assertGreater(region.heading_deg, 0.0)

    def test_returns_none_when_not_reacquiring_or_anchor_expired(self) -> None:
        selector = ObservableRegionSelector(max_anchor_age_s=5.0)
        selector.update_trusted_state(
            anchor_xy_m=np.array([0.0, 0.0], dtype=float),
            heading_deg=0.0,
            confidence=1.0,
            time_s=0.0,
        )

        self.assertIsNone(
            selector.select(
                time_s=1.0,
                vehicle_position_xy_m=np.array([0.0, 0.0], dtype=float),
                reacquire_required=False,
            )
        )
        self.assertIsNone(
            selector.select(
                time_s=10.0,
                vehicle_position_xy_m=np.array([0.0, 0.0], dtype=float),
                reacquire_required=True,
            )
        )

    def test_progressive_forward_gate_advances_with_vehicle(self) -> None:
        selector = ObservableRegionSelector(
            forward_distance_m=40.0,
            half_length_m=20.0,
            half_width_m=10.0,
            progressive_forward_enabled=True,
            progressive_margin_m=12.0,
        )
        selector.update_trusted_state(
            anchor_xy_m=np.array([0.0, 0.0], dtype=float),
            heading_deg=0.0,
            confidence=0.8,
            time_s=0.0,
        )

        region = selector.select(
            time_s=5.0,
            vehicle_position_xy_m=np.array([70.0, 0.0], dtype=float),
            reacquire_required=True,
        )

        self.assertIsNotNone(region)
        self.assertEqual(region.reason, "local_tangent_forward_gate")
        np.testing.assert_allclose(region.center_xy_m, np.array([82.0, 0.0]), atol=1e-6)


if __name__ == "__main__":
    unittest.main()
