import sys
import unittest
from pathlib import Path

import numpy as np


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.viz import (
    MilestoneMetrics,
    PRE_2G,
    compare_to_baseline,
    health_score,
)
from auv_mag_tracking.viz.metrics import HealthMetrics
from auv_mag_tracking.viz.recorder import simulate_case


def _metrics(case_name: str, *, mean_err: float, track: float, switches: int,
             cross_track: float = 2.0, good_ratio: float = 0.9) -> HealthMetrics:
    """Build a HealthMetrics with only the fields the progress view reads set."""
    return HealthMetrics(
        case_name=case_name,
        deployment_mode=False,
        duration_s=200.0,
        total_steps=4000,
        mean_heading_error_deg=mean_err,
        median_heading_error_deg=mean_err,
        final_heading_error_deg=mean_err,
        good_ratio=good_ratio,
        flip_count=0,
        heading_oscillations=0,
        mode_fraction={"track": track},
        track_active_fraction=track,
        mode_switches=switches,
        source_fraction={},
        sonar_contribution=0.0,
        magnetic_contribution=1.0,
        mean_snr_db=10.0,
        total_peaks=5,
        peak_rate_hz=0.1,
        mean_fit_residual_m=0.5,
        lock_grade_fraction=1.0,
        mean_cross_track_m=cross_track,
        max_cross_track_m=cross_track,
        mean_confidence=0.7,
        safe_lock_fraction=0.0,
        mean_vector_consistency=0.8,
        burial_inversion_mae_m=float("nan"),
        heading_errors_deg=np.array([mean_err]),
    )


class ProgressComparisonTest(unittest.TestCase):
    def test_baseline_constants_cover_all_default_cases(self) -> None:
        self.assertEqual(set(PRE_2G), {f"case{i}" for i in range(1, 6)})
        self.assertEqual(PRE_2G["case2"].mode_switches, 164)

    def test_switch_storm_collapse_reads_as_improved(self) -> None:
        current = _metrics("case2", mean_err=7.2, track=0.47, switches=2)
        delta = compare_to_baseline(current, PRE_2G["case2"])
        before, after, change, higher_is_better, _, target = delta.fields["switches"]
        self.assertEqual(before, 164.0)
        self.assertEqual(after, 2.0)
        self.assertEqual(change, -162.0)
        self.assertFalse(higher_is_better)
        self.assertTrue(delta.improved("switches"))

    def test_higher_is_better_direction_for_track(self) -> None:
        current = _metrics("case3", mean_err=0.0, track=0.365, switches=2)
        delta = compare_to_baseline(current, PRE_2G["case3"])
        before, after, change, higher_is_better, _, _ = delta.fields["track_pct"]
        self.assertAlmostEqual(before, 4.0)
        self.assertAlmostEqual(after, 36.5)
        self.assertTrue(higher_is_better)
        self.assertTrue(delta.improved("track_pct"))

    def test_health_field_uses_live_health_score(self) -> None:
        current = _metrics("case4", mean_err=0.7, track=0.53, switches=2)
        delta = compare_to_baseline(current, PRE_2G["case4"])
        _, after, _, _, _, _ = delta.fields["health"]
        self.assertAlmostEqual(after, health_score(current))

    def test_regression_detected_when_error_grows(self) -> None:
        baseline = MilestoneMetrics("caseX", health=80.0, mean_heading_error_deg=5.0,
                                    track_active_fraction=0.5, mode_switches=2)
        current = _metrics("caseX", mean_err=12.0, track=0.5, switches=2)
        delta = compare_to_baseline(current, baseline)
        self.assertFalse(delta.improved("mean_err"))

    def test_local_path_side_channel_is_recorded(self) -> None:
        record = simulate_case("case1", max_steps=200)
        self.assertIn("local_path_heading_deg", record.channels)
        self.assertIn("local_path_model_code", record.channels)
        self.assertGreater(np.count_nonzero(record["local_path_model_code"] > 0.0), 0)


if __name__ == "__main__":
    unittest.main()
