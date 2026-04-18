import sys
import unittest
from pathlib import Path

import numpy as np


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.config import build_default_scenarios
from auv_mag_tracking.perception import CableRouteFitter, FitResult, MagneticCablePerception, PeakDetector, RMSExtractor


class PerceptionTest(unittest.TestCase):
    def test_rms_window_covers_two_periods(self) -> None:
        extractor = RMSExtractor(sample_rate_hz=100.0, minimum_frequency_hz=20.0)
        self.assertGreaterEqual(extractor.window_size_samples, 10)

    def test_peak_detector_cooldown(self) -> None:
        detector = PeakDetector(min_peak_strength_nt=10.0, turn_trigger_ratio=0.6, hysteresis_fraction=0.05, cooldown_s=0.5)
        samples = [4.0, 8.0, 14.0, 20.0, 19.0, 16.0, 13.0, 22.0, 20.0, 14.0]
        times = [index * 0.1 for index in range(len(samples))]
        detections = [detector.update(sample, time_s).detected for sample, time_s in zip(samples, times)]
        self.assertEqual(sum(detections), 1)

    def test_peak_detector_uses_weighted_centroid_position(self) -> None:
        detector = PeakDetector(
            min_peak_strength_nt=5.0,
            turn_trigger_ratio=0.6,
            hysteresis_fraction=0.05,
            cooldown_s=0.1,
            ascending_min_samples=2,
            descending_min_samples=2,
            peak_zone_window_size=20,
        )
        strengths_nt = [2.0, 4.0, 8.0, 12.0, 10.0, 7.0, 5.0]
        positions_xy = [
            np.array([0.0, 0.0], dtype=float),
            np.array([0.4, 0.0], dtype=float),
            np.array([0.9, 0.0], dtype=float),
            np.array([1.1, 0.0], dtype=float),
            np.array([1.3, 0.0], dtype=float),
            np.array([1.5, 0.0], dtype=float),
            np.array([1.7, 0.0], dtype=float),
        ]
        detected_event = None
        for index, (strength_nt, position_xy) in enumerate(zip(strengths_nt, positions_xy)):
            event = detector.update(strength_nt, index * 0.1, position_xy_m=position_xy)
            if event.detected:
                detected_event = event

        self.assertIsNotNone(detected_event)
        self.assertIsNotNone(detected_event.peak_position_xy_m)
        self.assertGreater(detected_event.peak_position_xy_m[0], 1.0)
        self.assertLess(detected_event.peak_position_xy_m[0], 1.4)

    def test_peak_outlier_rejection_uses_last_fit_residual(self) -> None:
        scenario = build_default_scenarios()["case1"]
        perception = MagneticCablePerception(scenario)
        perception.last_accepted_fit_result = FitResult(
            origin_xy_m=np.array([0.0, 0.0], dtype=float),
            direction_xy=np.array([1.0, 0.0], dtype=float),
            residual_m=0.2,
            covariance_xy_m2=np.eye(2, dtype=float),
        )

        self.assertFalse(perception._is_peak_outlier(np.array([5.0, 1.5], dtype=float)))
        self.assertTrue(perception._is_peak_outlier(np.array([5.0, 3.2], dtype=float)))

    def test_route_fitter_tracks_linear_points(self) -> None:
        fitter = CableRouteFitter(history_size=5, forgetting_factor=0.7)
        for index in range(5):
            fitter.add_peak(np.array([float(index), float(index) * 0.1]), time_s=float(index))
        fit = fitter.fit()
        self.assertIsNotNone(fit.direction_xy)
        self.assertLess(fit.residual_m, 0.2)


if __name__ == "__main__":
    unittest.main()
