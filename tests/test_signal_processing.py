import sys
import unittest
from pathlib import Path

import numpy as np


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.perception import MedianWindowFilter, StreamingBandpassFilter


class SignalProcessingTest(unittest.TestCase):
    def test_median_window_filter_suppresses_single_outlier(self) -> None:
        median_filter = MedianWindowFilter(window_size=5)
        values = [10.0, 11.0, 9.0, 200.0, 10.0]
        filtered = [median_filter.update(value) for value in values]
        self.assertLess(filtered[-1], 20.0)

    def test_bandpass_filter_rejects_dc_and_keeps_target_frequency(self) -> None:
        sample_rate_hz = 200.0
        center_frequency_hz = 50.0
        bandpass_filter = StreamingBandpassFilter(
            sample_rate_hz=sample_rate_hz,
            center_frequency_hz=center_frequency_hz,
            half_width_hz=8.0,
        )

        time_axis = np.arange(0.0, 2.0, 1.0 / sample_rate_hz)
        input_signal = 200.0 + 20.0 * np.sin(2.0 * np.pi * center_frequency_hz * time_axis)
        output_signal = []
        for sample in input_signal:
            filtered = bandpass_filter.update(np.array([sample, 0.0, 0.0], dtype=float))
            output_signal.append(filtered[0])

        steady_state = np.asarray(output_signal[int(0.5 * sample_rate_hz):], dtype=float)
        self.assertLess(abs(np.mean(steady_state)), 2.0)
        self.assertGreater(np.std(steady_state), 5.0)


if __name__ == "__main__":
    unittest.main()
