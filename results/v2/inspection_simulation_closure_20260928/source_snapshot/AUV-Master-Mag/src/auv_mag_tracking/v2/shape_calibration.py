"""Controlled cross-scan calibration and conservative multi-model release.

Calibration needs an independently measured sensor-to-conductor separation.
Neither a field-model simulation nor the FWHM estimate is a truth source.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Sequence

import numpy as np

from .config import FwhmConfig
from .fwhm import estimate_fwhm
from .types import FwhmResult


@dataclass(frozen=True)
class CalibrationScan:
    scan_id: str
    x_m: np.ndarray
    envelope_nt: np.ndarray
    true_separation_m: float
    baseline_nt: float
    noise_nt: float


@dataclass(frozen=True)
class ShapeCalibration:
    shape_model: str
    shape_factor: float | None
    factor_relative_sigma: float | None
    train_scans: int
    holdout_scans: int
    holdout_valid_rate: float
    holdout_mae_m: float | None
    holdout_p95_m: float | None
    accepted: bool
    reason_codes: tuple[str, ...]
    source: str = "simulation"


@dataclass(frozen=True)
class AffineWidthCalibration:
    """Empirical z = slope * FWHM + intercept, valid only inside train range."""

    shape_model: str
    slope: float | None
    intercept_m: float | None
    minimum_width_m: float | None
    maximum_width_m: float | None
    train_scans: int
    holdout_scans: int
    holdout_valid_rate: float
    holdout_mae_m: float | None
    holdout_p95_m: float | None
    accepted: bool
    reason_codes: tuple[str, ...]
    source: str = "simulation"


def _measured_width(scan: CalibrationScan, config: FwhmConfig) -> FwhmResult:
    if not scan.scan_id or not math.isfinite(scan.true_separation_m) or scan.true_separation_m <= 0:
        raise ValueError("scan ID and independent positive separation truth are required")
    if not math.isfinite(scan.baseline_nt) or not math.isfinite(scan.noise_nt) or scan.noise_nt <= 0:
        raise ValueError("finite baseline and positive noise are required")
    # A zero altitude is intentional: validation concerns sensor-to-conductor
    # separation; burial depth needs a second, independent height measurement.
    return estimate_fwhm(
        scan.x_m, scan.envelope_nt, replace(config, altitude_m=0.0),
        baseline_nt=scan.baseline_nt, noise_nt=scan.noise_nt, altitude_m=0.0,
    )


def calibrate_shape_factor(
    train: Sequence[CalibrationScan],
    holdout: Sequence[CalibrationScan],
    config: FwhmConfig,
    *,
    minimum_train_scans: int = 5,
    minimum_holdout_scans: int = 5,
    minimum_holdout_valid_rate: float = 0.9,
    maximum_holdout_mae_m: float = 0.2,
    maximum_holdout_p95_m: float = 0.5,
    source: str = "simulation",
) -> ShapeCalibration:
    """Fit only the width-to-distance coefficient; audit on disjoint scans.

    The acceptance limits are engineering choices, not a field certification.
    Invalid train scans are excluded; invalid holdout scans reduce valid rate.
    """
    config.validate()
    if source not in ("simulation", "controlled_measurement"):
        raise ValueError("calibration source must be simulation or controlled_measurement")
    if minimum_train_scans < 2 or minimum_holdout_scans < 1:
        raise ValueError("minimum scan counts must be positive")
    if not 0 < minimum_holdout_valid_rate <= 1 or maximum_holdout_mae_m <= 0 or maximum_holdout_p95_m <= 0:
        raise ValueError("invalid holdout acceptance limits")
    train_ids = [scan.scan_id for scan in train]
    holdout_ids = [scan.scan_id for scan in holdout]
    if len(set(train_ids)) != len(train_ids) or len(set(holdout_ids)) != len(holdout_ids):
        raise ValueError("scan IDs must be unique within each split")
    if set(train_ids) & set(holdout_ids):
        raise ValueError("train and holdout must have disjoint scan IDs")
    reasons: list[str] = []
    train_ratios: list[float] = []
    train_truth: list[float] = []
    for scan in train:
        result = _measured_width(scan, config)
        if result.valid and result.width_m is not None:
            train_ratios.append(scan.true_separation_m / result.width_m)
            train_truth.append(scan.true_separation_m)
    if len(train_ratios) < minimum_train_scans:
        reasons.append("insufficient_valid_training_scans")
    if len(train_truth) >= 2 and max(train_truth) - min(train_truth) < 0.5:
        reasons.append("insufficient_depth_diversity")
    if not train_ratios:
        return ShapeCalibration(config.shape_model, None, None, 0, len(holdout), 0.0, None, None, False, tuple(reasons), source)
    factor = float(np.median(train_ratios))
    # MAD measures between-scan variation. A small floor avoids declaring
    # zero uncertainty on noiseless or duplicated calibration scans.
    relative_sigma = max(
        float(1.4826 * np.median(np.abs(np.asarray(train_ratios) - factor)) / factor),
        0.01,
    )
    if len(holdout) < minimum_holdout_scans:
        reasons.append("insufficient_holdout_scans")
    errors: list[float] = []
    for scan in holdout:
        result = _measured_width(scan, config)
        if result.valid and result.width_m is not None:
            errors.append(abs(factor * result.width_m - scan.true_separation_m))
    valid_rate = len(errors) / len(holdout) if holdout else 0.0
    mae = float(np.mean(errors)) if errors else None
    p95 = float(np.percentile(errors, 95)) if errors else None
    if valid_rate < minimum_holdout_valid_rate:
        reasons.append("holdout_valid_rate_below_limit")
    if mae is None or mae > maximum_holdout_mae_m:
        reasons.append("holdout_mae_above_limit")
    if p95 is None or p95 > maximum_holdout_p95_m:
        reasons.append("holdout_p95_above_limit")
    return ShapeCalibration(
        config.shape_model, factor, relative_sigma, len(train_ratios),
        len(holdout), valid_rate, mae, p95, not reasons, tuple(reasons), source,
    )


def calibrate_affine_width(
    train: Sequence[CalibrationScan],
    holdout: Sequence[CalibrationScan],
    config: FwhmConfig,
    *,
    minimum_train_scans: int = 5,
    minimum_holdout_scans: int = 5,
    minimum_holdout_valid_rate: float = 0.9,
    maximum_holdout_mae_m: float = 0.2,
    maximum_holdout_p95_m: float = 0.5,
    source: str = "simulation",
) -> AffineWidthCalibration:
    """Fit a two-parameter width map when a constant factor is insufficient.

    The holdout is used solely for acceptance, never to fit either parameter.
    No extrapolation beyond training widths is allowed.
    """
    config.validate()
    if source not in ("simulation", "controlled_measurement"):
        raise ValueError("calibration source must be simulation or controlled_measurement")
    if minimum_train_scans < 3 or minimum_holdout_scans < 1:
        raise ValueError("affine calibration needs at least three training and one holdout scan")
    if not 0 < minimum_holdout_valid_rate <= 1 or maximum_holdout_mae_m <= 0 or maximum_holdout_p95_m <= 0:
        raise ValueError("invalid holdout acceptance limits")
    train_ids = [scan.scan_id for scan in train]
    holdout_ids = [scan.scan_id for scan in holdout]
    if len(set(train_ids)) != len(train_ids) or len(set(holdout_ids)) != len(holdout_ids):
        raise ValueError("scan IDs must be unique within each split")
    if set(train_ids) & set(holdout_ids):
        raise ValueError("train and holdout must have disjoint scan IDs")
    widths: list[float] = []
    truths: list[float] = []
    for scan in train:
        result = _measured_width(scan, config)
        if result.valid and result.width_m is not None:
            widths.append(result.width_m)
            truths.append(scan.true_separation_m)
    reasons: list[str] = []
    if len(widths) < minimum_train_scans:
        reasons.append("insufficient_valid_training_scans")
    if len(set(round(z, 6) for z in truths)) < 3 or (truths and max(truths) - min(truths) < 0.5):
        reasons.append("insufficient_depth_diversity")
    if len(widths) < 2 or max(widths) - min(widths) < 1e-9:
        reasons.append("insufficient_width_diversity")
    if "insufficient_width_diversity" in reasons:
        return AffineWidthCalibration(config.shape_model, None, None, None, None, len(widths), len(holdout), 0.0, None, None, False, tuple(reasons), source)
    slope, intercept = np.polyfit(widths, truths, 1)
    slope, intercept = float(slope), float(intercept)
    if not math.isfinite(slope) or slope <= 0 or not math.isfinite(intercept):
        reasons.append("nonpositive_or_nonfinite_slope")
    if len(holdout) < minimum_holdout_scans:
        reasons.append("insufficient_holdout_scans")
    minimum_width, maximum_width = min(widths), max(widths)
    errors: list[float] = []
    for scan in holdout:
        result = _measured_width(scan, config)
        if result.valid and result.width_m is not None and minimum_width <= result.width_m <= maximum_width:
            errors.append(abs(slope * result.width_m + intercept - scan.true_separation_m))
    valid_rate = len(errors) / len(holdout) if holdout else 0.0
    mae = float(np.mean(errors)) if errors else None
    p95 = float(np.percentile(errors, 95)) if errors else None
    if valid_rate < minimum_holdout_valid_rate:
        reasons.append("holdout_valid_rate_below_limit")
    if mae is None or mae > maximum_holdout_mae_m:
        reasons.append("holdout_mae_above_limit")
    if p95 is None or p95 > maximum_holdout_p95_m:
        reasons.append("holdout_p95_above_limit")
    return AffineWidthCalibration(
        config.shape_model, slope, intercept, minimum_width, maximum_width,
        len(widths), len(holdout), valid_rate, mae, p95, not reasons,
        tuple(reasons), source,
    )


def estimate_with_affine_calibration(
    x_m: np.ndarray,
    envelope_nt: np.ndarray,
    config: FwhmConfig,
    calibration: AffineWidthCalibration,
    *,
    baseline_nt: float | None = None,
    noise_nt: float | None = None,
    altitude_m: float | None = None,
    allow_simulation_calibration: bool = False,
) -> FwhmResult:
    """Apply a range-limited, held-out width map after the normal shape gate."""
    if config.shape_model != calibration.shape_model:
        raise ValueError("calibration model does not match configuration")
    result = estimate_fwhm(
        x_m, envelope_nt, replace(config, altitude_m=0.0), baseline_nt=baseline_nt,
        noise_nt=noise_nt, altitude_m=0.0,
    )
    if (
        not calibration.accepted or calibration.slope is None
        or calibration.intercept_m is None
        or calibration.minimum_width_m is None or calibration.maximum_width_m is None
        or (calibration.source != "controlled_measurement" and not allow_simulation_calibration)
    ):
        return _reject(result, "no_accepted_affine_calibration")
    if not result.valid or result.width_m is None:
        return result
    if not calibration.minimum_width_m <= result.width_m <= calibration.maximum_width_m:
        return _reject(result, "width_outside_calibrated_range")
    vertical = calibration.slope * result.width_m + calibration.intercept_m
    altitude = config.altitude_m if altitude_m is None else float(altitude_m)
    if not math.isfinite(altitude) or vertical < altitude:
        return _reject(result, "calibrated_separation_below_altitude")
    # Treat the observed holdout P95 as a conservative uncertainty floor.
    width_sigma = result.width_sigma_m or 0.0
    calibration_floor = calibration.holdout_p95_m or 0.0
    vertical_sigma = math.hypot(calibration.slope * width_sigma, calibration_floor)
    return replace(
        result, vertical_separation_m=vertical, burial_depth_m=vertical - altitude,
        vertical_separation_sigma_m=vertical_sigma,
        burial_depth_sigma_m=math.hypot(vertical_sigma, config.altitude_sigma_m),
        shape_factor=calibration.slope, shape_intercept_m=calibration.intercept_m,
    )


def estimate_with_calibrated_models(
    x_m: np.ndarray,
    envelope_nt: np.ndarray,
    calibrations: Sequence[tuple[FwhmConfig, ShapeCalibration]],
    *,
    baseline_nt: float | None = None,
    noise_nt: float | None = None,
    altitude_m: float | None = None,
    maximum_disagreement_m: float = 0.5,
    allow_simulation_calibration: bool = False,
) -> FwhmResult:
    """Release depth only when accepted model calibrations identify a safe result.

    All configured models are evaluated. A sole winner must be positively
    distinguished by the competing-shape guard. Multiple winners must agree
    in distance. This cannot detect a shared wrong shape factor.
    """
    if not calibrations:
        raise ValueError("at least one model is required")
    if maximum_disagreement_m <= 0:
        raise ValueError("maximum_disagreement_m must be positive")
    evaluated: list[FwhmResult] = []
    for config, calibration in calibrations:
        if config.shape_model != calibration.shape_model:
            raise ValueError("calibration model does not match configuration")
        if not calibration.accepted or calibration.shape_factor is None:
            continue
        if calibration.source != "controlled_measurement" and not allow_simulation_calibration:
            continue
        setting = replace(
            config, shape_factor=calibration.shape_factor,
            shape_factor_relative_sigma=max(
                config.shape_factor_relative_sigma,
                calibration.factor_relative_sigma or 0.0,
            ),
        )
        evaluated.append(estimate_fwhm(
            x_m, envelope_nt, setting, baseline_nt=baseline_nt,
            noise_nt=noise_nt, altitude_m=altitude_m,
        ))
    if not evaluated:
        config = calibrations[0][0]
        candidate = estimate_fwhm(
            x_m, envelope_nt, config, baseline_nt=baseline_nt,
            noise_nt=noise_nt, altitude_m=altitude_m,
        )
        return _reject(candidate, "no_accepted_shape_calibration")
    valid = [result for result in evaluated if result.valid]
    if not valid:
        return _reject(evaluated[0], "no_qualified_shape_model")
    if len(valid) == 1 and len(evaluated) > 1:
        if not all(
            "competing_shape_model_preferred" in result.reason_codes
            for result in evaluated if result is not valid[0]
        ):
            return _reject(valid[0], "shape_model_not_distinguishable")
    if len(valid) > 1:
        depths = [float(result.vertical_separation_m) for result in valid]
        if max(depths) - min(depths) > maximum_disagreement_m:
            return _reject(min(valid, key=lambda result: result.model_residual or math.inf), "model_depth_disagreement")
    return min(valid, key=lambda result: result.model_residual if result.model_residual is not None else math.inf)


def _reject(candidate: FwhmResult, reason: str) -> FwhmResult:
    return replace(
        candidate, valid=False, vertical_separation_m=None, burial_depth_m=None,
        vertical_separation_sigma_m=None, burial_depth_sigma_m=None,
        confidence=0.0, reason_codes=tuple(dict.fromkeys((*candidate.reason_codes, reason))),
    )
