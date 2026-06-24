import sys
import unittest
from pathlib import Path

import numpy as np


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.routes import CableMapBuilder, build_cable_map_from_record
from auv_mag_tracking.viz import RunRecord


class CableMapTest(unittest.TestCase):
    def test_builder_compacts_ordered_observations(self) -> None:
        builder = CableMapBuilder(min_confidence=0.5, min_spacing_m=1.0, simplify_tolerance_m=0.2)
        for idx, x_m in enumerate(np.linspace(0.0, 20.0, 21)):
            y_m = 0.0 if x_m < 10.0 else 0.4 * (x_m - 10.0)
            builder.add_observation((x_m, y_m), confidence=0.8, time_s=float(idx), source="SONAR")
        cable_map = builder.build(metadata={"case_name": "unit"})
        self.assertGreaterEqual(cable_map.waypoints_xy_m.shape[0], 3)
        self.assertLess(cable_map.waypoints_xy_m.shape[0], builder.observation_count)
        route = cable_map.to_prior_route()
        self.assertTrue(route.within_tolerance(np.array([5.0, 0.5], dtype=float)))

    def test_build_map_from_record_uses_estimated_channels(self) -> None:
        n = 12
        channels = {
            "time_s": np.arange(n, dtype=float),
            "estimated_cable_x_m": np.linspace(0.0, 22.0, n),
            "estimated_cable_y_m": np.zeros(n, dtype=float),
            "true_nearest_x_m": np.full(n, 999.0, dtype=float),
            "true_nearest_y_m": np.full(n, 999.0, dtype=float),
            "confidence": np.linspace(0.2, 0.9, n),
        }
        record = RunRecord(
            case_name="unit",
            deployment_mode=False,
            dt_s=0.1,
            channels=channels,
            modes=["TRACK_ACTIVE"] * n,
            sources=["SONAR"] * n,
            cable_route_xy_m=np.zeros((2, 2), dtype=float),
        )
        cable_map = build_cable_map_from_record(record, min_confidence=0.5, min_spacing_m=2.0, simplify_tolerance_m=0.5)
        self.assertEqual(cable_map.metadata["case_name"], "unit")
        self.assertGreater(cable_map.metadata["raw_observation_count"], 0)
        self.assertLess(float(np.max(cable_map.waypoints_xy_m[:, 0])), 999.0)


if __name__ == "__main__":
    unittest.main()
