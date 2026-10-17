"""Online alignment state from route prior to observed cable geometry."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from ..math_utils import heading_from_direction_xy, smallest_angle_error_deg


@dataclass
class PriorAlignmentState:
    """Estimated correction that maps the route prior toward observed cable points."""

    translation_xy_m: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=float))
    rotation_deg: float = 0.0
    covariance_diag: np.ndarray = field(default_factory=lambda: np.ones(3, dtype=float))
    residual_xy_m: np.ndarray = field(default_factory=lambda: np.full(2, np.nan, dtype=float))
    residual_norm_m: float = float("nan")
    heading_residual_deg: float = float("nan")
    applied_step_xy_m: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=float))
    applied_step_norm_m: float = 0.0
    applied_rotation_step_deg: float = 0.0
    confidence: float = 0.0
    progress_m: float = float("nan")
    accepted: bool = False
    reason_code: int = 0


class PriorAlignmentEstimator:
    """Gated online estimator for prior-to-real route correction."""

    REASON_NONE = 0
    REASON_ACCEPTED = 1
    REASON_RESIDUAL_TOO_LARGE = 2
    REASON_HEADING_TOO_LARGE = 3
    REASON_NONFINITE = 4

    def __init__(
        self,
        initial_translation_xy_m: np.ndarray,
        initial_rotation_deg: float,
        initial_covariance_diag: np.ndarray,
    ) -> None:
        self.state = PriorAlignmentState(
            translation_xy_m=np.asarray(initial_translation_xy_m, dtype=float).copy(),
            rotation_deg=float(initial_rotation_deg),
            covariance_diag=np.asarray(initial_covariance_diag, dtype=float).copy(),
        )

    def predict(self, tracking, dt_s: float) -> None:
        """Diffuse covariance when the EKF-style update is enabled."""
        if not tracking.nominal_route_prior_correction_ekf_enabled or dt_s <= 0.0:
            return
        translation_process_var = (
            tracking.nominal_route_prior_correction_ekf_translation_process_std_m_per_sqrt_s ** 2
        ) * dt_s
        rotation_process_var = (
            tracking.nominal_route_prior_correction_ekf_rotation_process_std_deg_per_sqrt_s ** 2
        ) * dt_s
        self.state.covariance_diag[0] += translation_process_var
        self.state.covariance_diag[1] += translation_process_var
        self.state.covariance_diag[2] += rotation_process_var

    def clear_observation_diagnostics(self) -> None:
        """Reset per-frame observation diagnostics without changing the accumulated pose."""
        self.state.residual_xy_m = np.full(2, np.nan, dtype=float)
        self.state.residual_norm_m = float("nan")
        self.state.heading_residual_deg = float("nan")
        self.state.applied_step_xy_m = np.zeros(2, dtype=float)
        self.state.applied_step_norm_m = 0.0
        self.state.applied_rotation_step_deg = 0.0
        self.state.confidence = 0.0
        self.state.progress_m = float("nan")
        self.state.accepted = False
        self.state.reason_code = self.REASON_NONE

    def update(
        self,
        tracking,
        observed_point_xy: np.ndarray,
        prior_point_xy: np.ndarray,
        prior_tangent_xy: np.ndarray,
        observed_heading_deg: Optional[float],
        confidence: float,
        progress_m: float,
    ) -> PriorAlignmentState:
        """Apply one gated residual update and return the latest state."""
        observed_point_xy = np.asarray(observed_point_xy, dtype=float)
        prior_point_xy = np.asarray(prior_point_xy, dtype=float)
        residual_xy_m = observed_point_xy - prior_point_xy
        residual_norm_m = float(np.linalg.norm(residual_xy_m))
        heading_residual_deg = float("nan")
        if observed_heading_deg is not None:
            prior_heading_deg = heading_from_direction_xy(np.asarray(prior_tangent_xy, dtype=float))
            heading_residual_deg = smallest_angle_error_deg(float(observed_heading_deg), prior_heading_deg)

        self.clear_observation_diagnostics()
        self.state.residual_xy_m = residual_xy_m.copy()
        self.state.residual_norm_m = residual_norm_m
        self.state.heading_residual_deg = heading_residual_deg
        self.state.confidence = float(confidence)
        self.state.progress_m = float(progress_m)

        if not np.all(np.isfinite(residual_xy_m)) or not np.isfinite(residual_norm_m):
            self.state.reason_code = self.REASON_NONFINITE
            return self.state

        max_residual_m = max(float(tracking.nominal_route_prior_correction_max_residual_m), 0.0)
        if max_residual_m > 0.0 and residual_norm_m > max_residual_m:
            self.state.reason_code = self.REASON_RESIDUAL_TOO_LARGE
            return self.state

        max_heading_error_deg = max(float(tracking.nominal_route_prior_correction_max_heading_error_deg), 0.0)
        if (
            observed_heading_deg is not None
            and max_heading_error_deg > 0.0
            and abs(float(heading_residual_deg)) > max_heading_error_deg
        ):
            self.state.reason_code = self.REASON_HEADING_TOO_LARGE
            return self.state

        if tracking.nominal_route_prior_correction_ekf_enabled:
            translation_step_xy_m, rotation_step_deg = self._ekf_step(
                tracking,
                residual_xy_m,
                heading_residual_deg if observed_heading_deg is not None else None,
                confidence,
            )
        else:
            gain = float(np.clip(tracking.nominal_route_prior_correction_gain, 0.0, 1.0))
            translation_step_xy_m = gain * residual_xy_m
            rotation_step_deg = (
                gain * float(heading_residual_deg)
                if observed_heading_deg is not None and np.isfinite(heading_residual_deg)
                else 0.0
            )

        translation_step_xy_m = self._limit_translation_step(
            translation_step_xy_m,
            max(float(tracking.nominal_route_prior_correction_max_step_m), 0.0),
        )
        self.state.translation_xy_m = self.state.translation_xy_m + translation_step_xy_m
        self.state.rotation_deg = self.state.rotation_deg + float(rotation_step_deg)
        self._clip_total_correction(tracking)

        self.state.applied_step_xy_m = translation_step_xy_m.copy()
        self.state.applied_step_norm_m = float(np.linalg.norm(translation_step_xy_m))
        self.state.applied_rotation_step_deg = float(rotation_step_deg)
        self.state.accepted = True
        self.state.reason_code = self.REASON_ACCEPTED
        return self.state

    def _ekf_step(
        self,
        tracking,
        residual_xy_m: np.ndarray,
        heading_residual_deg: Optional[float],
        confidence: float,
    ) -> tuple[np.ndarray, float]:
        confidence = max(float(confidence), 1e-3)
        translation_meas_var = (
            tracking.nominal_route_prior_correction_ekf_translation_meas_std_m / confidence
        ) ** 2
        rotation_meas_var = (
            tracking.nominal_route_prior_correction_ekf_rotation_meas_std_deg / confidence
        ) ** 2
        translation_step_xy_m = np.zeros(2, dtype=float)
        for axis in (0, 1):
            prior_var = self.state.covariance_diag[axis]
            kalman_gain = prior_var / (prior_var + translation_meas_var)
            translation_step_xy_m[axis] = kalman_gain * residual_xy_m[axis]
            self.state.covariance_diag[axis] = (1.0 - kalman_gain) * prior_var

        rotation_step_deg = 0.0
        if heading_residual_deg is not None:
            prior_var = self.state.covariance_diag[2]
            kalman_gain = prior_var / (prior_var + rotation_meas_var)
            rotation_step_deg = kalman_gain * float(heading_residual_deg)
            self.state.covariance_diag[2] = (1.0 - kalman_gain) * prior_var
        return translation_step_xy_m, float(rotation_step_deg)

    @staticmethod
    def _limit_translation_step(step_xy_m: np.ndarray, max_step_m: float) -> np.ndarray:
        if max_step_m <= 0.0:
            return np.asarray(step_xy_m, dtype=float)
        step_xy_m = np.asarray(step_xy_m, dtype=float)
        step_norm_m = float(np.linalg.norm(step_xy_m))
        if step_norm_m <= max_step_m or step_norm_m <= 1e-12:
            return step_xy_m
        return step_xy_m * (max_step_m / step_norm_m)

    def _clip_total_correction(self, tracking) -> None:
        max_translation_m = max(float(tracking.nominal_route_prior_correction_max_translation_m), 0.0)
        correction_norm_m = float(np.linalg.norm(self.state.translation_xy_m))
        if correction_norm_m > max_translation_m > 0.0:
            self.state.translation_xy_m *= max_translation_m / correction_norm_m
        max_rotation_deg = max(float(tracking.nominal_route_prior_correction_max_rotation_deg), 0.0)
        self.state.rotation_deg = float(np.clip(
            self.state.rotation_deg,
            -max_rotation_deg,
            max_rotation_deg,
        ))
