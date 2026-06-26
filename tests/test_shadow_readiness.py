import sys
import unittest
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.viz.readiness import score_shadow_hypothesis_readiness


class ShadowReadinessTest(unittest.TestCase):
    def _score(self, **overrides):
        params = dict(
            magnetic_path_valid=True,
            magnetic_phase_valid=True,
            magnetic_lookahead_valid=True,
            magnetic_lookahead_confidence=0.6,
            lookahead_feed_allowed=True,
            cycle_burial_valid=True,
            cycle_burial_quality=0.5,
            local_path_confidence=0.7,
            local_path_residual_m=1.0,
            local_path_max_residual_m=5.0,
            guidance_source="LOCAL_PATH",
            route_progress_rate_mps=0.8,
            yaw_rate_abs_fraction=0.2,
        )
        params.update(overrides)
        return score_shadow_hypothesis_readiness(**params)

    def test_ready_pipeline_scores_high(self) -> None:
        score = self._score()

        self.assertGreater(score.supply, 0.8)
        self.assertGreater(score.selection, 0.8)
        self.assertGreater(score.consumption, 0.8)
        self.assertGreater(score.total, 0.6)
        self.assertEqual(score.bottleneck_code, 1.0)

    def test_supply_bottleneck_when_no_magnetic_hypothesis(self) -> None:
        score = self._score(
            magnetic_path_valid=False,
            magnetic_phase_valid=False,
            magnetic_lookahead_valid=False,
            lookahead_feed_allowed=False,
            cycle_burial_valid=False,
        )

        self.assertEqual(score.bottleneck_code, 2.0)
        self.assertLess(score.supply, 0.1)

    def test_selection_bottleneck_when_feed_and_local_path_are_poor(self) -> None:
        score = self._score(
            lookahead_feed_allowed=False,
            local_path_confidence=0.1,
            local_path_residual_m=20.0,
        )

        self.assertEqual(score.bottleneck_code, 3.0)
        self.assertLess(score.selection, score.supply)

    def test_consumption_bottleneck_when_controller_does_not_progress(self) -> None:
        score = self._score(
            guidance_source="SEARCH",
            route_progress_rate_mps=-0.1,
            yaw_rate_abs_fraction=1.0,
        )

        self.assertEqual(score.bottleneck_code, 4.0)
        self.assertLess(score.consumption, 0.1)


if __name__ == "__main__":
    unittest.main()
