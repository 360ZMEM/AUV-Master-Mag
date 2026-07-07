import math
import sys
import unittest
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.perception import (
    BurialCycleEstimate,
    BurialEstimate,
    MagneticBurialCycleEstimator,
    MagneticBurialInverter,
)


def _strength_for(burial_m: float, lateral_m: float, k: float, i_rms: float, altitude_m: float) -> float:
    """Inverse of the calibrated model: given a geometry, the strength that yields it."""
    slant_range_m = math.sqrt((altitude_m + burial_m) ** 2 + lateral_m ** 2)
    return k * i_rms / slant_range_m


class MagneticBurialInverterTest(unittest.TestCase):
    K = 11.4329
    I_RMS = 800.0 / math.sqrt(2.0)
    ALT = 6.0

    def _inverter(self, **overrides) -> MagneticBurialInverter:
        params = dict(
            coupling_constant_nt_m_per_a_rms=self.K,
            current_rms_a=self.I_RMS,
            altitude_m=self.ALT,
            snr_gate_db=6.0,
            min_strength_nt=1.0,
            min_samples=5,
            max_lateral_offset_m=1.0,
        )
        params.update(overrides)
        return MagneticBurialInverter(**params)

    def test_converges_to_true_burial_at_crossing(self) -> None:
        inv = self._inverter()
        true_burial = 1.5
        b = _strength_for(true_burial, lateral_m=0.0, k=self.K, i_rms=self.I_RMS, altitude_m=self.ALT)
        estimate = None
        for _ in range(20):
            estimate = inv.update(strength_nt=b, lateral_offset_m=0.0, snr_db=20.0)
        self.assertIsInstance(estimate, BurialEstimate)
        self.assertAlmostEqual(estimate.depth_m, true_burial, places=6)
        self.assertGreaterEqual(estimate.fit_quality, 0.0)
        self.assertLessEqual(estimate.fit_quality, 1.0)

    def test_warmup_gate_returns_none(self) -> None:
        inv = self._inverter(min_samples=5)
        b = _strength_for(1.5, 0.0, self.K, self.I_RMS, self.ALT)
        for _ in range(4):
            self.assertIsNone(inv.update(strength_nt=b, lateral_offset_m=0.0, snr_db=20.0))
        self.assertIsNotNone(inv.update(strength_nt=b, lateral_offset_m=0.0, snr_db=20.0))

    def test_low_snr_frames_are_rejected(self) -> None:
        inv = self._inverter(min_samples=3)
        b = _strength_for(1.5, 0.0, self.K, self.I_RMS, self.ALT)
        for _ in range(10):
            self.assertIsNone(inv.update(strength_nt=b, lateral_offset_m=0.0, snr_db=2.0))

    def test_far_lateral_frames_are_rejected(self) -> None:
        inv = self._inverter(min_samples=3, max_lateral_offset_m=1.0)
        b = _strength_for(1.5, lateral_m=5.0, k=self.K, i_rms=self.I_RMS, altitude_m=self.ALT)
        for _ in range(10):
            self.assertIsNone(inv.update(strength_nt=b, lateral_offset_m=5.0, snr_db=20.0))

    def test_implausibly_deep_frames_are_rejected(self) -> None:
        inv = self._inverter(min_samples=2, max_depth_m=10.0)
        implausibly_deep = _strength_for(80.0, lateral_m=0.2, k=self.K, i_rms=self.I_RMS, altitude_m=self.ALT)
        plausible = _strength_for(1.5, lateral_m=0.2, k=self.K, i_rms=self.I_RMS, altitude_m=self.ALT)

        for _ in range(5):
            self.assertIsNone(inv.update(strength_nt=implausibly_deep, lateral_offset_m=0.2, snr_db=20.0))
        self.assertEqual(inv.sample_count, 0)

        self.assertIsNone(inv.update(strength_nt=plausible, lateral_offset_m=0.2, snr_db=20.0))
        self.assertIsNotNone(inv.update(strength_nt=plausible, lateral_offset_m=0.2, snr_db=20.0))

    def test_lateral_correction_recovers_burial(self) -> None:
        inv = self._inverter(min_samples=3, max_lateral_offset_m=1.0)
        true_burial = 1.5
        lateral = 0.8
        b = _strength_for(true_burial, lateral, self.K, self.I_RMS, self.ALT)
        estimate = None
        for _ in range(10):
            estimate = inv.update(strength_nt=b, lateral_offset_m=lateral, snr_db=20.0)
        self.assertAlmostEqual(estimate.depth_m, true_burial, places=6)

    def test_reset_clears_samples(self) -> None:
        inv = self._inverter(min_samples=2)
        b = _strength_for(1.5, 0.0, self.K, self.I_RMS, self.ALT)
        inv.update(strength_nt=b, lateral_offset_m=0.0, snr_db=20.0)
        self.assertIsNotNone(inv.update(strength_nt=b, lateral_offset_m=0.0, snr_db=20.0))
        inv.reset()
        self.assertIsNone(inv.update(strength_nt=b, lateral_offset_m=0.0, snr_db=20.0))

    def test_sliding_window_forgets_old_burial_distribution(self) -> None:
        inv = self._inverter(min_samples=3, max_samples=5)
        shallow = _strength_for(1.0, 0.0, self.K, self.I_RMS, self.ALT)
        deep = _strength_for(3.0, 0.0, self.K, self.I_RMS, self.ALT)

        for _ in range(5):
            estimate = inv.update(strength_nt=shallow, lateral_offset_m=0.0, snr_db=20.0)
        assert estimate is not None
        self.assertAlmostEqual(estimate.depth_m, 1.0, places=6)

        for _ in range(5):
            estimate = inv.update(strength_nt=deep, lateral_offset_m=0.0, snr_db=20.0)
        assert estimate is not None
        self.assertAlmostEqual(estimate.depth_m, 3.0, places=6)
        self.assertEqual(inv.sample_count, 5)


class MagneticBurialCycleEstimatorTest(unittest.TestCase):
    K = 11.4329
    I_RMS = 800.0 / math.sqrt(2.0)
    ALT = 6.0

    def _cycle_estimator(self, **overrides) -> MagneticBurialCycleEstimator:
        params = dict(
            coupling_constant_nt_m_per_a_rms=self.K,
            current_rms_a=self.I_RMS,
            altitude_m=self.ALT,
            snr_gate_db=6.0,
            min_strength_nt=1.0,
            min_samples=3,
            max_lateral_offset_m=2.0,
        )
        params.update(overrides)
        return MagneticBurialCycleEstimator(**params)

    def test_cycle_estimator_converges_within_probe_cycle(self) -> None:
        estimator = self._cycle_estimator(min_samples=3)
        true_burial = 1.5
        estimate = None
        for lateral in (0.15, 0.25, 0.35, 0.45):
            strength = _strength_for(true_burial, lateral, self.K, self.I_RMS, self.ALT)
            estimate = estimator.update(strength_nt=strength, lateral_offset_m=lateral, snr_db=24.0)

        self.assertIsInstance(estimate, BurialCycleEstimate)
        assert estimate is not None
        self.assertAlmostEqual(estimate.depth_m, true_burial, delta=0.05)
        self.assertGreater(estimate.fit_quality, 0.0)
        self.assertEqual(estimate.sample_count, 4)

    def test_cycle_estimator_downweights_edge_outliers(self) -> None:
        estimator = self._cycle_estimator(min_samples=3)
        true_burial = 1.5
        for lateral in (0.10, 0.15, 0.20, 0.25):
            strength = _strength_for(true_burial, lateral, self.K, self.I_RMS, self.ALT)
            estimate = estimator.update(strength_nt=strength, lateral_offset_m=lateral, snr_db=24.0)
        for lateral in (1.85, 1.90, 1.95):
            strength = _strength_for(6.0, lateral, self.K, self.I_RMS, self.ALT)
            estimate = estimator.update(strength_nt=strength, lateral_offset_m=lateral, snr_db=7.0)

        self.assertIsNotNone(estimate)
        assert estimate is not None
        self.assertLess(abs(estimate.depth_m - true_burial), 0.20)

    def test_cycle_reset_starts_new_posterior(self) -> None:
        estimator = self._cycle_estimator(min_samples=2)
        strength = _strength_for(1.5, 0.1, self.K, self.I_RMS, self.ALT)
        estimator.update(strength_nt=strength, lateral_offset_m=0.1, snr_db=20.0)
        self.assertIsNotNone(estimator.update(strength_nt=strength, lateral_offset_m=0.1, snr_db=20.0))

        estimator.reset()

        self.assertIsNone(estimator.update(strength_nt=strength, lateral_offset_m=0.1, snr_db=20.0))


if __name__ == "__main__":
    unittest.main()
