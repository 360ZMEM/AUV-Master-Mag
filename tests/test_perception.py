import sys
import unittest
from pathlib import Path

import numpy as np


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.perception import CableRouteFitter, PeakDetector, RMSExtractor


class PerceptionTest(unittest.TestCase):
    def test_rms_window_covers_two_periods(self) -> None:
        extractor = RMSExtractor(sample_rate_hz=100.0, minimum_frequency_hz=20.0)
        self.assertGreaterEqual(extractor.window_size_samples, 10)

    def test_peak_detector_cooldown(self) -> None:
        detector = PeakDetector(min_peak_strength_nt=10.0, turn_trigger_ratio=0.6, hysteresis_fraction=0.05, cooldown_s=0.5)
        samples = [10.0, 18.0, 30.0, 28.0, 24.0, 16.0, 14.0, 30.0, 14.0]
        times = [index * 0.1 for index in range(len(samples))]
        detections = [detector.update(sample, time_s).detected for sample, time_s in zip(samples, times)]
        self.assertEqual(sum(detections), 1)

    def test_route_fitter_tracks_linear_points(self) -> None:
        fitter = CableRouteFitter(history_size=5, forgetting_factor=0.7)
        for index in range(5):
            fitter.add_peak(np.array([float(index), float(index) * 0.1]), time_s=float(index))
        fit = fitter.fit()
        self.assertIsNotNone(fit.direction_xy)
        self.assertLess(fit.residual_m, 0.2)


if __name__ == "__main__":
    unittest.main()
