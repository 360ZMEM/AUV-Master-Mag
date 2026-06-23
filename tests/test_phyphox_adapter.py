import sys
import unittest
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.experimental.phyphox_adapter import MagnetometerReading, PhyphoxSample, SignalProcessor


class PhyphoxAdapterTest(unittest.TestCase):
    def test_magnetometer_reading_reports_magnitude(self) -> None:
        reading = MagnetometerReading(timestamp_s=1.0, bx_uT=3.0, by_uT=4.0, bz_uT=12.0, confidence=0.9)
        self.assertAlmostEqual(reading.magnitude_uT, 13.0)

    def test_signal_processor_removes_dc_bias_and_smooths(self) -> None:
        processor = SignalProcessor(sample_rate_hz=20.0, calibration_seconds=3.0, lowpass_window_seconds=0.35)

        last_reading = None
        for index in range(80):
            sample = PhyphoxSample(
                timestamp_s=index * 0.05,
                timestamp_source="phone",
                mag_x_uT=100.0,
                mag_y_uT=0.0,
                mag_z_uT=0.0,
            )
            last_reading = processor.process(sample)

        self.assertIsNotNone(last_reading)
        self.assertTrue(processor.calibrated)
        self.assertLess(abs(last_reading.bx_uT), 1.0)
        self.assertAlmostEqual(last_reading.by_uT, 0.0, delta=1.0)
        self.assertAlmostEqual(last_reading.bz_uT, 0.0, delta=1.0)
        self.assertGreaterEqual(last_reading.confidence, 0.75)


if __name__ == "__main__":
    unittest.main()