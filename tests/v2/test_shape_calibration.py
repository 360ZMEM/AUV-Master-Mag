from __future__ import annotations

from dataclasses import replace
import math
import unittest

import numpy as np

from auv_mag_tracking.v2 import (
    CalibrationScan, FwhmConfig, ShapeCalibration, calibrate_shape_factor,
    calibrate_affine_width, estimate_with_affine_calibration,
    estimate_with_calibrated_models,
)


def scan(scan_id: str, z: float, *, split_shape: str = "wire", noise: float = 0.02) -> CalibrationScan:
    rng = np.random.default_rng(int(scan_id.split("_")[-1]))
    x = np.linspace(-24.0, 24.0, 401)
    anomaly = 20.0 / np.sqrt(1.0 + (x / z) ** 2) if split_shape == "wire" else 20.0 / (1.0 + (x / z) ** 2)
    return CalibrationScan(scan_id, x, 2.0 + anomaly + rng.normal(0, noise, x.size), z, 2.0, noise)


class ShapeCalibrationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.wire = FwhmConfig(
            shape_model="physical_single_wire", shape_factor=0.5,
            altitude_m=0.0, minimum_prominence_nt=0.05,
        )

    def test_independent_holdout_recovers_factor_and_rejects_wrong_default(self) -> None:
        train = [scan(f"train_{i}", 2.5 + 0.3 * i) for i in range(10)]
        holdout = [scan(f"hold_{i + 100}", 2.7 + 0.25 * i) for i in range(10)]
        result = calibrate_shape_factor(train, holdout, self.wire)
        self.assertTrue(result.accepted, result.reason_codes)
        self.assertAlmostEqual(result.shape_factor, 1.0 / (2 * math.sqrt(3)), delta=0.003)
        self.assertLess(result.holdout_mae_m, 0.05)
        self.assertGreaterEqual(result.factor_relative_sigma, 0.01)

    def test_wrong_shape_fails_before_factor_fit(self) -> None:
        train = [scan(f"train_{i}", 2.5 + 0.3 * i) for i in range(10)]
        holdout = [scan(f"hold_{i + 100}", 2.7 + 0.25 * i) for i in range(10)]
        result = calibrate_shape_factor(train, holdout, replace(self.wire, shape_model="thesis_dipole"))
        self.assertFalse(result.accepted)
        self.assertIn("insufficient_valid_training_scans", result.reason_codes)

    def test_group_leakage_is_rejected(self) -> None:
        item = scan("same_1", 3.0)
        with self.assertRaisesRegex(ValueError, "disjoint"):
            calibrate_shape_factor([item], [item], self.wire)

    def test_unknown_calibration_fails_closed(self) -> None:
        item = scan("demo_1", 4.0)
        result = estimate_with_calibrated_models(
            item.x_m, item.envelope_nt,
            [(self.wire, ShapeCalibration("physical_single_wire", None, None, 0, 0, 0, None, None, False, ()))],
            baseline_nt=2.0, noise_nt=0.02, altitude_m=0.0,
        )
        self.assertFalse(result.valid)
        self.assertIn("no_accepted_shape_calibration", result.reason_codes)
        self.assertIsNone(result.vertical_separation_m)

    def test_simulation_calibration_cannot_publish_by_default(self) -> None:
        item = scan("demo_1", 4.0)
        simulated = ShapeCalibration(
            "physical_single_wire", 1 / (2 * math.sqrt(3)), 0.02,
            10, 10, 1.0, 0.02, 0.04, True, (), "simulation",
        )
        result = estimate_with_calibrated_models(
            item.x_m, item.envelope_nt, [(self.wire, simulated)],
            baseline_nt=2.0, noise_nt=0.02, altitude_m=0.0,
        )
        self.assertFalse(result.valid)
        self.assertIn("no_accepted_shape_calibration", result.reason_codes)

    def test_multimodel_selects_distinct_calibrated_wire(self) -> None:
        item = scan("demo_1", 4.0)
        wire = ShapeCalibration("physical_single_wire", 1 / (2 * math.sqrt(3)), 0.02, 10, 10, 1, 0.02, 0.04, True, (), "controlled_measurement")
        dipole = ShapeCalibration("thesis_dipole", 0.5, 0.02, 10, 10, 1, 0.02, 0.04, True, (), "controlled_measurement")
        result = estimate_with_calibrated_models(
            item.x_m, item.envelope_nt,
            [(self.wire, wire), (replace(self.wire, shape_model="thesis_dipole"), dipole)],
            baseline_nt=2.0, noise_nt=0.02, altitude_m=0.0,
        )
        self.assertTrue(result.valid, result.reason_codes)
        self.assertEqual(result.shape_model, "physical_single_wire")
        self.assertAlmostEqual(result.vertical_separation_m, 4.0, delta=0.05)

    def test_multimodel_disagreement_rejects_even_if_each_model_passes(self) -> None:
        item = scan("demo_1", 4.0)
        wire = ShapeCalibration("physical_single_wire", 1 / (2 * math.sqrt(3)), 0.02, 10, 10, 1, 0.02, 0.04, True, (), "controlled_measurement")
        dipole = ShapeCalibration("thesis_dipole", 0.5, 0.02, 10, 10, 1, 0.02, 0.04, True, (), "controlled_measurement")
        permissive = replace(self.wire, minimum_competing_shape_advantage=1e6)
        result = estimate_with_calibrated_models(
            item.x_m, item.envelope_nt,
            [(permissive, wire), (replace(permissive, shape_model="thesis_dipole"), dipole)],
            baseline_nt=2.0, noise_nt=0.02, altitude_m=0.0,
        )
        self.assertFalse(result.valid)
        self.assertIn("model_depth_disagreement", result.reason_codes)
        self.assertIsNone(result.burial_depth_m)

    def test_affine_mapping_corrects_stable_nonzero_width_intercept(self) -> None:
        x = np.linspace(-35, 35, 701)

        def one(scan_id: str, truth: float) -> CalibrationScan:
            # An effective source scale offset produces W = 2 * (z + 0.35).
            y = 2.0 + 20.0 / (1.0 + (x / (truth + 0.35)) ** 2)
            return CalibrationScan(scan_id, x, y, truth, 2.0, 0.01)

        train = [one(f"train_{i}", z) for i, z in enumerate((2, 3, 4, 5, 6, 7, 8, 9))]
        holdout = [one(f"hold_{i}", z) for i, z in enumerate((2.5, 3.5, 5.5, 7.5, 8.5))]
        config = FwhmConfig(altitude_m=0, minimum_prominence_nt=0.1)
        constant = calibrate_shape_factor(train, holdout, config)
        affine = calibrate_affine_width(train, holdout, config)
        self.assertTrue(affine.accepted, affine.reason_codes)
        self.assertAlmostEqual(affine.slope, 0.5, delta=0.005)
        self.assertAlmostEqual(affine.intercept_m, -0.35, delta=0.05)
        self.assertLess(affine.holdout_mae_m, constant.holdout_mae_m / 3)
        target = holdout[2]
        released = estimate_with_affine_calibration(
            target.x_m, target.envelope_nt, config,
            replace(affine, source="controlled_measurement"),
            baseline_nt=2.0, noise_nt=0.01, altitude_m=0.0,
        )
        self.assertTrue(released.valid, released.reason_codes)
        self.assertAlmostEqual(released.vertical_separation_m, target.true_separation_m, delta=0.05)
        self.assertAlmostEqual(released.shape_intercept_m, -0.35, delta=0.05)
        # The old coefficient may imply z < altitude; affine validation must
        # apply altitude only after the calibrated width conversion.
        corrected = estimate_with_affine_calibration(
            target.x_m, target.envelope_nt,
            replace(config, shape_factor=0.1, altitude_m=2.0),
            replace(affine, source="controlled_measurement"),
            baseline_nt=2.0, noise_nt=0.01, altitude_m=2.0,
        )
        self.assertTrue(corrected.valid, corrected.reason_codes)
        self.assertAlmostEqual(corrected.burial_depth_m, target.true_separation_m - 2.0, delta=0.05)

    def test_affine_holdout_outside_training_range_is_rejected(self) -> None:
        x = np.linspace(-40, 40, 801)

        def one(scan_id: str, truth: float) -> CalibrationScan:
            y = 2.0 + 20.0 / (1.0 + (x / truth) ** 2)
            return CalibrationScan(scan_id, x, y, truth, 2.0, 0.01)

        train = [one(f"train_{i}", float(i + 2)) for i in range(5)]
        holdout = [one(f"hold_{i}", 8.0 + i) for i in range(5)]
        result = calibrate_affine_width(train, holdout, FwhmConfig(altitude_m=0))
        self.assertFalse(result.accepted)
        self.assertEqual(result.holdout_valid_rate, 0.0)
        self.assertIn("holdout_valid_rate_below_limit", result.reason_codes)

    def test_affine_runtime_rejects_unverified_source(self) -> None:
        item = scan("demo_1", 4.0)
        from auv_mag_tracking.v2 import AffineWidthCalibration
        calibration = AffineWidthCalibration(
            "physical_single_wire", 0.288675, 0.0, 5.0, 25.0,
            10, 10, 1.0, 0.02, 0.04, True, (), "simulation",
        )
        result = estimate_with_affine_calibration(
            item.x_m, item.envelope_nt, self.wire, calibration,
            baseline_nt=2.0, noise_nt=0.02, altitude_m=0.0,
        )
        self.assertFalse(result.valid)
        self.assertIsNone(result.burial_depth_m)
        self.assertIn("no_accepted_affine_calibration", result.reason_codes)


if __name__ == "__main__":
    unittest.main()
