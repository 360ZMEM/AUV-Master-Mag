"""Lightweight deployment quality estimation for the public API."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Optional

import numpy as np

from ..perception.burial_inversion import BurialEstimate, MagneticBurialInverter
from ..perception.confidence import ConfidenceEstimator
from .types import DeploymentPerceptionConfig, MagneticInput, NavigationInput, SonarInput


@dataclass
class DeploymentQualityOutput:
    magnetic_strength_nt: float
    magnetic_std_nt: float
    snr_db: float
    magnetic_confidence: float
    fit_residual_m: float
    prior_alignment_residual_m: float
    confidence: float
    magnetic_used: bool
    industrial_ready: bool
    quality_flags: list[str] = field(default_factory=list)
    burial_estimate: Optional[BurialEstimate] = None
    burial_sample_count: int = 0
    burial_status: str = "disabled"


class DeploymentQualityEstimator:
    """Conservative quality shell for deployment API outputs.

    It intentionally consumes only public API inputs and cable-map projection
    results.  The richer simulation/orchestrator stack remains independent.
    """

    def __init__(self, config: DeploymentPerceptionConfig) -> None:
        self.config = config
        self.confidence_estimator = ConfidenceEstimator(lost_timeout_s=2.0)
        self.burial_inverter: Optional[MagneticBurialInverter] = None
        if bool(config.enable_burial_inversion):
            self.burial_inverter = MagneticBurialInverter(
                coupling_constant_nt_m_per_a_rms=config.burial_coupling_constant_nt_m_per_a_rms,
                current_rms_a=config.burial_current_rms_a,
                altitude_m=config.burial_altitude_m,
                snr_gate_db=config.burial_snr_gate_db,
                min_strength_nt=config.burial_min_strength_nt,
                min_samples=config.burial_min_samples,
                max_lateral_offset_m=config.burial_max_lateral_offset_m,
                max_depth_m=config.burial_max_depth_m,
                max_samples=config.burial_window_samples,
            )

    def reset(self) -> None:
        if self.burial_inverter is not None:
            self.burial_inverter.reset()

    def evaluate(
        self,
        *,
        navigation: NavigationInput,
        magnetic: MagneticInput,
        route_distance_m: float,
        signed_cross_track_m: float,
        sonar: Optional[SonarInput] = None,
    ) -> DeploymentQualityOutput:
        flags: list[str] = []
        external_flags = self._external_quality_flags(magnetic)
        flags.extend(external_flags)
        samples = self._magnetic_samples(magnetic)
        if samples.size == 0 or not bool(self.config.enable_magnetic_quality):
            flags.append("missing_magnetic_block")
            return self._fallback(route_distance_m, sonar, flags)

        norms = np.linalg.norm(samples, axis=1)
        finite_norms = norms[np.isfinite(norms)]
        if finite_norms.size == 0:
            flags.append("invalid_magnetic_block")
            return self._fallback(route_distance_m, sonar, flags)

        strength_nt = float(np.mean(finite_norms))
        std_nt = float(np.std(finite_norms))
        noise_floor = max(float(self.config.magnetic_noise_floor_nt), 1.0e-9)
        snr_linear = strength_nt / noise_floor
        snr_db = 20.0 * math.log10(max(snr_linear, 1.0e-12))
        weak_signal = strength_nt < float(self.config.burial_min_strength_nt) or bool(external_flags)
        if weak_signal:
            flags.append("weak_magnetic_signal")
        if abs(float(route_distance_m)) > float(self.config.route_offset_ready_m):
            flags.append("route_offset_large")

        magnetic_confidence = self.confidence_estimator.magnetic_confidence(
            snr=snr_linear,
            fit_residual_m=abs(float(route_distance_m)),
            detection_age_s=0.0,
            weak_signal_flag=weak_signal,
            speed_mps=max(float(navigation.speed_mps), 0.1),
        )
        if external_flags:
            magnetic_confidence *= self._external_quality_scale(external_flags)
        sonar_confidence = float(sonar.confidence) if sonar is not None and sonar.valid else 0.0
        confidence = self.confidence_estimator.fused_confidence(
            magnetic_confidence=magnetic_confidence,
            sonar_confidence=sonar_confidence,
            guidance_source="MAGNETIC" if not weak_signal else "BLIND",
            fit_residual_m=abs(float(route_distance_m)),
        )
        if confidence < float(self.config.confidence_min_ready):
            flags.append("confidence_low")

        burial_estimate = None
        burial_status = "disabled"
        burial_sample_count = 0
        if self.burial_inverter is not None:
            burial_estimate = self.burial_inverter.update(
                strength_nt=strength_nt,
                lateral_offset_m=signed_cross_track_m,
                snr_db=snr_db,
            )
            burial_sample_count = self.burial_inverter.sample_count
            burial_status = "ready" if burial_estimate is not None else "warming_up"
            if burial_estimate is None:
                flags.append("burial_warming_up")

        industrial_ready = not flags
        return DeploymentQualityOutput(
            magnetic_strength_nt=strength_nt,
            magnetic_std_nt=std_nt,
            snr_db=snr_db,
            magnetic_confidence=magnetic_confidence,
            fit_residual_m=abs(float(route_distance_m)),
            prior_alignment_residual_m=abs(float(route_distance_m)),
            confidence=confidence,
            magnetic_used=not weak_signal,
            industrial_ready=industrial_ready,
            quality_flags=flags,
            burial_estimate=burial_estimate,
            burial_sample_count=burial_sample_count,
            burial_status=burial_status,
        )

    @staticmethod
    def _magnetic_samples(magnetic: MagneticInput) -> np.ndarray:
        try:
            samples = np.asarray(magnetic.sample_block_nt, dtype=float).reshape(-1, 3)
        except Exception:
            return np.empty((0, 3), dtype=float)
        return samples

    @staticmethod
    def _external_quality_flags(magnetic: MagneticInput) -> list[str]:
        raw = magnetic.quality_flags or {}
        if isinstance(raw, dict):
            return [
                f"sensor_{str(key)}"
                for key, value in raw.items()
                if bool(value)
            ]
        if isinstance(raw, (list, tuple, set)):
            return [f"sensor_{str(item)}" for item in raw if str(item)]
        return [f"sensor_{str(raw)}"] if str(raw) else []

    @staticmethod
    def _external_quality_scale(flags: list[str]) -> float:
        severe_tokens = ("saturat", "invalid", "calib", "missing", "stale")
        if any(any(token in flag.lower() for token in severe_tokens) for flag in flags):
            return 0.35
        return 0.65

    @staticmethod
    def _fallback(
        route_distance_m: float,
        sonar: Optional[SonarInput],
        flags: list[str],
    ) -> DeploymentQualityOutput:
        sonar_confidence = float(sonar.confidence) if sonar is not None and sonar.valid else 0.0
        confidence = min(max(sonar_confidence, 0.0), 0.4)
        if confidence <= 0.0:
            flags.append("confidence_low")
        return DeploymentQualityOutput(
            magnetic_strength_nt=0.0,
            magnetic_std_nt=0.0,
            snr_db=float("-inf"),
            magnetic_confidence=0.0,
            fit_residual_m=abs(float(route_distance_m)),
            prior_alignment_residual_m=abs(float(route_distance_m)),
            confidence=confidence,
            magnetic_used=False,
            industrial_ready=False,
            quality_flags=flags,
        )
