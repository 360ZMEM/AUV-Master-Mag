import sys
import unittest
from pathlib import Path

import numpy as np


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.config import build_default_scenarios
from auv_mag_tracking.math_utils import Pose
from auv_mag_tracking.perception_driver import PerceptionDriver
from auv_mag_tracking.sensor_model import HighFidelityMagnetometer, MagnetometerReading


class PerceptionDriverTest(unittest.TestCase):
    def test_high_fidelity_scenarios_are_registered(self) -> None:
        scenarios = build_default_scenarios()
        self.assertIn("case_hf_phone", scenarios)
        self.assertIn("case_hf_industrial", scenarios)
        self.assertTrue(scenarios["case_hf_phone"].sensor.high_fidelity.enabled)
        self.assertTrue(scenarios["case_hf_industrial"].sensor.high_fidelity.enabled)

    def test_high_fidelity_magnetometer_returns_quantized_waveform_block(self) -> None:
        scenario = build_default_scenarios()["case1"]
        scenario.sensor.high_fidelity.enabled = True
        scenario.sensor.high_fidelity.sampling_rate_hz = 400.0
        sensor = HighFidelityMagnetometer(scenario.sensor)

        sample_times_s = np.arange(1, 21, dtype=float) * sensor.sample_period_s
        true_fields_ned_nt = np.tile(np.array([[25000.0, 0.0, 42000.0]], dtype=float), (sample_times_s.size, 1))
        cable_fields_ned_nt = np.zeros_like(true_fields_ned_nt)
        reading = sensor.sample_block(
            true_fields_ned_nt=true_fields_ned_nt,
            pose=Pose(position_ned_m=np.zeros(3, dtype=float), heading_deg=0.0, pitch_deg=0.0, roll_deg=0.0),
            sample_times_s=sample_times_s,
            cable_fields_ned_nt=cable_fields_ned_nt,
        )

        self.assertEqual(reading.sample_block_sensor_nt.shape, (20, 3))
        self.assertEqual(reading.quantized_sensor_block_nt.shape, (20, 3))
        self.assertEqual(reading.raw_sensor_block_nt.shape, (20, 3))
        self.assertEqual(reading.bit_depth, 24)
        self.assertAlmostEqual(reading.sample_rate_hz, 400.0)

    def test_perception_driver_detects_ac_peak(self) -> None:
        scenario = build_default_scenarios()["case1"]
        scenario.signal_processing.window_size = 256
        driver = PerceptionDriver(scenario)
        sample_rate_hz = scenario.sensor.magnetometer_sample_rate_hz
        sample_times_s = np.arange(256, dtype=float) / sample_rate_hz
        signal_axis_nt = 40.0 * np.sin(2.0 * np.pi * 49.3 * sample_times_s)
        signal_block_nt = np.column_stack((signal_axis_nt, np.zeros(256, dtype=float), np.zeros(256, dtype=float)))
        reading = MagnetometerReading(
            time_s=float(sample_times_s[-1]),
            sensor_field_nt=signal_block_nt[-1].copy(),
            sample_times_s=sample_times_s,
            sample_block_sensor_nt=signal_block_nt.copy(),
            cable_strength_nt=80.0,
            weak_signal_flag=False,
            raw_sensor_block_nt=signal_block_nt.copy(),
            quantized_sensor_block_nt=signal_block_nt.copy(),
            dc_reference_sensor_nt=np.zeros(3, dtype=float),
            clipping_ratio=0.0,
            sample_rate_hz=sample_rate_hz,
            bit_depth=24,
        )

        frame = driver.update(reading)
        self.assertTrue(frame.features.is_ac_detected)
        self.assertTrue(frame.features.signal_reliable if hasattr(frame.features, "signal_reliable") else frame.features.reliability_flag)
        self.assertAlmostEqual(frame.features.dominant_frequency_hz, 49.3, delta=1.0)
        self.assertAlmostEqual(frame.diagnostics.detected_peak_frequency_hz, 49.3, delta=1.0)
        self.assertAlmostEqual(frame.features.frequency_error_hz, -0.7, delta=1.0)
        self.assertGreater(frame.features.processed_intensity_nt, 10.0)
        self.assertEqual(frame.features.signal_method, "lockin")
        self.assertEqual(frame.diagnostics.processed_amplitude_window_nt.shape, frame.diagnostics.relative_time_s.shape)

    def test_perception_driver_removes_dc_bias_and_high_frequency_noise(self) -> None:
        scenario = build_default_scenarios()["case1"]
        driver = PerceptionDriver(scenario)
        sample_rate_hz = scenario.sensor.magnetometer_sample_rate_hz
        sample_times_s = np.arange(256, dtype=float) / sample_rate_hz
        signal_axis_nt = (
            35.0 * np.sin(2.0 * np.pi * 50.0 * sample_times_s)
            + 120.0
            + 6.0 * np.sin(2.0 * np.pi * 95.0 * sample_times_s)
        )
        signal_block_nt = np.column_stack((signal_axis_nt, np.zeros(256, dtype=float), np.zeros(256, dtype=float)))
        reading = MagnetometerReading(
            time_s=float(sample_times_s[-1]),
            sensor_field_nt=signal_block_nt[-1].copy(),
            sample_times_s=sample_times_s,
            sample_block_sensor_nt=signal_block_nt.copy(),
            cable_strength_nt=80.0,
            weak_signal_flag=False,
            raw_sensor_block_nt=signal_block_nt.copy(),
            quantized_sensor_block_nt=signal_block_nt.copy(),
            dc_reference_sensor_nt=np.array([120.0, 0.0, 0.0], dtype=float),
            clipping_ratio=0.0,
            sample_rate_hz=sample_rate_hz,
            bit_depth=24,
        )

        frame = driver.update(reading)
        self.assertTrue(frame.features.is_ac_detected)
        self.assertGreater(frame.features.processed_intensity_nt, 15.0)
        self.assertLess(abs(np.mean(frame.diagnostics.filtered_time_window_nt)), 2.0)

    def test_perception_driver_interpolates_200hz_input_to_1khz_processing_rate(self) -> None:
        scenario = build_default_scenarios()["case1"]
        scenario.signal_processing.window_size = 64
        scenario.sensor.magnetometer_sample_rate_hz = 200.0
        driver = PerceptionDriver(scenario)
        sample_rate_hz = scenario.sensor.magnetometer_sample_rate_hz
        sample_times_s = np.arange(10, dtype=float) / sample_rate_hz
        signal_axis_nt = 20.0 * np.sin(2.0 * np.pi * 50.0 * sample_times_s)
        signal_block_nt = np.column_stack((signal_axis_nt, np.zeros(10, dtype=float), np.zeros(10, dtype=float)))
        reading = MagnetometerReading(
            time_s=float(sample_times_s[-1]),
            sensor_field_nt=signal_block_nt[-1].copy(),
            sample_times_s=sample_times_s,
            sample_block_sensor_nt=signal_block_nt.copy(),
            cable_strength_nt=40.0,
            weak_signal_flag=False,
            raw_sensor_block_nt=signal_block_nt.copy(),
            quantized_sensor_block_nt=signal_block_nt.copy(),
            dc_reference_sensor_nt=np.zeros(3, dtype=float),
            clipping_ratio=0.0,
            sample_rate_hz=sample_rate_hz,
            bit_depth=24,
        )

        frame = driver.update(reading)
        self.assertAlmostEqual(frame.features.processing_sample_rate_hz, 1000.0)
        self.assertGreater(frame.diagnostics.relative_time_s.size, sample_times_s.size)

    def test_high_fidelity_industrial_tuning_is_tighter(self) -> None:
        scenario = build_default_scenarios()["case_hf_industrial"]
        self.assertEqual(scenario.signal_processing.window_size, 512)
        self.assertLessEqual(scenario.signal_processing.peak_search_half_width_hz, 1.5)
        self.assertLessEqual(scenario.visualization.psd_max_frequency_hz, 80.0)

    def test_perception_driver_detects_dc_excursion(self) -> None:
        scenario = build_default_scenarios()["case1"]
        scenario.signal.mode = "dc"
        scenario.signal_processing.window_size = 128
        driver = PerceptionDriver(scenario)
        sample_rate_hz = scenario.sensor.magnetometer_sample_rate_hz
        sample_times_s = np.arange(128, dtype=float) / sample_rate_hz
        signal_block_nt = np.column_stack((np.full(128, 22.0, dtype=float), np.zeros(128, dtype=float), np.zeros(128, dtype=float)))
        reading = MagnetometerReading(
            time_s=float(sample_times_s[-1]),
            sensor_field_nt=signal_block_nt[-1].copy(),
            sample_times_s=sample_times_s,
            sample_block_sensor_nt=signal_block_nt.copy(),
            cable_strength_nt=22.0,
            weak_signal_flag=False,
            raw_sensor_block_nt=signal_block_nt.copy(),
            quantized_sensor_block_nt=signal_block_nt.copy(),
            dc_reference_sensor_nt=np.zeros(3, dtype=float),
            clipping_ratio=0.0,
            sample_rate_hz=sample_rate_hz,
            bit_depth=24,
        )

        frame = driver.update(reading)
        self.assertFalse(frame.features.is_ac_detected)
        self.assertGreater(frame.features.processed_intensity_nt, 15.0)
        self.assertLess(frame.features.dominant_frequency_hz, 8.0)


if __name__ == "__main__":
    unittest.main()