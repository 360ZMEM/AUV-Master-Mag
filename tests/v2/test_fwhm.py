from __future__ import annotations

import unittest

import numpy as np

from auv_mag_tracking.v2 import FwhmConfig, estimate_fwhm


class FwhmEstimatorTests(unittest.TestCase):
    def test_lorentzian_recovers_width_and_depth(self) -> None:
        z_m = 3.0
        x_m = np.linspace(-12.0, 12.0, 481)
        baseline_nt = 7.0
        envelope_nt = baseline_nt + 20.0 / (1.0 + (x_m / z_m) ** 2)
        result = estimate_fwhm(
            x_m,
            envelope_nt,
            FwhmConfig(altitude_m=1.0, minimum_prominence_nt=0.1),
            noise_nt=0.02,
            baseline_nt=baseline_nt,
        )
        self.assertTrue(result.valid, result.reason_codes)
        self.assertAlmostEqual(result.width_m, 2.0 * z_m, delta=0.08)
        self.assertAlmostEqual(result.center_x_m, 0.0, delta=0.04)
        self.assertAlmostEqual(result.vertical_separation_m, z_m, delta=0.04)
        self.assertAlmostEqual(result.burial_depth_m, 2.0, delta=0.04)
        self.assertGreater(result.vertical_separation_sigma_m, 0.14)
        self.assertGreater(result.burial_depth_sigma_m, result.vertical_separation_sigma_m)

    def test_amplitude_scale_does_not_change_width(self) -> None:
        x_m = np.linspace(-10.0, 10.0, 401)
        config = FwhmConfig(altitude_m=0.0, minimum_prominence_nt=0.01)
        widths = []
        for amplitude in (1.0, 10.0, 100.0):
            y = 4.0 + amplitude / (1.0 + (x_m / 2.0) ** 2)
            result = estimate_fwhm(x_m, y, config, noise_nt=0.001, baseline_nt=4.0)
            self.assertTrue(result.valid, result.reason_codes)
            widths.append(result.width_m)
        self.assertLess(max(widths) - min(widths), 1e-9)

    def test_missing_half_crossing_is_invalid(self) -> None:
        x_m = np.linspace(-0.4, 4.0, 100)
        y = 1.0 + 10.0 / (1.0 + (x_m / 2.0) ** 2)
        result = estimate_fwhm(
            x_m,
            y,
            FwhmConfig(minimum_prominence_nt=0.01),
            noise_nt=0.01,
        )
        self.assertFalse(result.valid)
        self.assertIn("left_half_crossing_missing", result.reason_codes)
        self.assertIsNone(result.burial_depth_m)

    def test_background_corrected_half_level(self) -> None:
        x_m = np.linspace(-8.0, 8.0, 321)
        y = 100.0 + 8.0 / (1.0 + (x_m / 1.5) ** 2)
        result = estimate_fwhm(
            x_m,
            y,
            FwhmConfig(altitude_m=0.0, minimum_prominence_nt=0.01),
            noise_nt=0.01,
            baseline_nt=100.0,
        )
        self.assertTrue(result.valid, result.reason_codes)
        self.assertGreater(result.half_level_nt, 100.0)
        self.assertAlmostEqual(result.width_m, 3.0, delta=0.08)

    def test_physical_wire_profile_uses_physical_shape_model(self) -> None:
        z_m = 4.0
        x_m = np.linspace(-30.0, 30.0, 1201)
        y = 2.0 + 12.0 / np.sqrt(1.0 + (x_m / z_m) ** 2)
        result = estimate_fwhm(
            x_m,
            y,
            FwhmConfig(
                altitude_m=1.0,
                minimum_prominence_nt=0.01,
                shape_model="physical_single_wire",
                shape_factor=1.0 / (2.0 * np.sqrt(3.0)),
            ),
            baseline_nt=2.0,
            noise_nt=0.01,
        )
        self.assertTrue(result.valid, result.reason_codes)
        self.assertAlmostEqual(result.vertical_separation_m, z_m, delta=0.03)
        self.assertAlmostEqual(result.burial_depth_m, 3.0, delta=0.03)

    def test_depth_below_altitude_is_invalid_not_clamped(self) -> None:
        x_m = np.linspace(-4.0, 4.0, 161)
        y = 1.0 + 10.0 / (1.0 + (x_m / 1.0) ** 2)
        result = estimate_fwhm(
            x_m,
            y,
            FwhmConfig(altitude_m=2.0, minimum_prominence_nt=0.01),
            baseline_nt=1.0,
            noise_nt=0.01,
        )
        self.assertFalse(result.valid)
        self.assertIsNone(result.burial_depth_m)
        self.assertIn("vertical_separation_below_altitude", result.reason_codes)

    def test_contaminated_external_background_is_recovered_by_profile(self) -> None:
        z_m = 5.0
        x_m = np.linspace(-25.0, 25.0, 1001)
        y = 2.0 + 15.0 / np.sqrt(1.0 + ((x_m - 2.5) / z_m) ** 2)
        result = estimate_fwhm(
            x_m,
            y,
            FwhmConfig(
                altitude_m=2.0,
                minimum_prominence_nt=0.01,
                shape_model="physical_single_wire",
                shape_factor=1.0 / (2.0 * np.sqrt(3.0)),
            ),
            baseline_nt=8.0,
            noise_nt=0.01,
        )
        self.assertTrue(result.valid, result.reason_codes)
        self.assertAlmostEqual(result.baseline_nt, 2.0, delta=0.03)
        self.assertAlmostEqual(result.vertical_separation_m, z_m, delta=0.04)

    def test_dynamic_altitude_overrides_static_configuration(self) -> None:
        x_m = np.linspace(-12.0, 12.0, 481)
        y = 1.0 + 10.0 / (1.0 + (x_m / 3.0) ** 2)
        result = estimate_fwhm(
            x_m,
            y,
            FwhmConfig(altitude_m=1.0, minimum_prominence_nt=0.01),
            altitude_m=2.0,
            baseline_nt=1.0,
            noise_nt=0.01,
        )
        self.assertTrue(result.valid, result.reason_codes)
        self.assertAlmostEqual(result.burial_depth_m, 1.0, delta=0.04)

    def test_depth_calibration_factor_does_not_change_profile_shape_check(self) -> None:
        x_m = np.linspace(-12.0, 12.0, 481)
        y = 3.0 + 20.0 / (1.0 + (x_m / 3.0) ** 2)
        result = estimate_fwhm(
            x_m,
            y,
            FwhmConfig(
                altitude_m=0.0,
                minimum_prominence_nt=0.01,
                maximum_model_residual=0.05,
                shape_model="legacy_segment",
                shape_factor=0.47203575,
            ),
            baseline_nt=3.0,
            noise_nt=0.01,
        )
        self.assertTrue(result.valid, result.reason_codes)
        self.assertAlmostEqual(result.width_m, 6.0, delta=0.08)
        self.assertAlmostEqual(result.vertical_separation_m, 2.8322145, delta=0.05)

    def test_wrong_declared_shape_rejects_depth_even_with_complete_crossings(self) -> None:
        x_m = np.linspace(-14.0, 14.0, 241)
        y = 2.0 + 20.0 / np.sqrt(1.0 + (x_m / 4.0) ** 2)
        result = estimate_fwhm(
            x_m, y,
            FwhmConfig(shape_model="thesis_dipole", shape_factor=0.5, altitude_m=0.0),
            baseline_nt=2.0, noise_nt=0.05,
        )
        self.assertIsNotNone(result.left_crossing_x_m)
        self.assertIsNotNone(result.right_crossing_x_m)
        self.assertFalse(result.valid)
        self.assertIn("competing_shape_model_preferred", result.reason_codes)
        self.assertIsNone(result.vertical_separation_m)
        self.assertIsNone(result.burial_depth_m)
        self.assertEqual(result.confidence, 0.0)
        self.assertLess(result.competing_model_residual, result.model_residual)

    def test_correct_wire_model_remains_publishable(self) -> None:
        x_m = np.linspace(-14.0, 14.0, 241)
        y = 2.0 + 20.0 / np.sqrt(1.0 + (x_m / 4.0) ** 2)
        result = estimate_fwhm(
            x_m, y,
            FwhmConfig(
                shape_model="physical_single_wire",
                shape_factor=1.0 / (2.0 * np.sqrt(3.0)), altitude_m=0.0,
            ),
            baseline_nt=2.0, noise_nt=0.05,
        )
        self.assertTrue(result.valid, result.reason_codes)
        self.assertAlmostEqual(result.vertical_separation_m, 4.0, delta=0.08)

    def test_short_wings_are_rejected_before_depth_release(self) -> None:
        x_m = np.linspace(-3.8, 3.8, 241)
        y = 1.0 + 20.0 / (1.0 + (x_m / 3.0) ** 2)
        result = estimate_fwhm(
            x_m, y, FwhmConfig(altitude_m=0.0), baseline_nt=1.0, noise_nt=0.05,
        )
        self.assertIsNotNone(result.width_m)
        self.assertFalse(result.valid)
        self.assertIn("shape_evidence_insufficient", result.reason_codes)
        self.assertIsNone(result.burial_depth_m)


if __name__ == "__main__":
    unittest.main()
