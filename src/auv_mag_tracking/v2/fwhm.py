"""Explicit background-corrected FWHM estimation in spatial coordinates."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
try:
    from scipy.optimize import curve_fit
except ModuleNotFoundError:  # pragma: no cover - repository requirements include scipy
    curve_fit = None

from .config import FwhmConfig
from .types import FwhmResult


def _median_filter(values: np.ndarray, window: int) -> np.ndarray:
    if window <= 1 or values.size < window:
        return values.copy()
    radius = window // 2
    padded = np.pad(values, (radius, radius), mode="edge")
    return np.asarray([np.median(padded[i : i + window]) for i in range(values.size)], dtype=float)


def _crossing(x0: float, y0: float, x1: float, y1: float, level: float) -> Optional[float]:
    delta = y1 - y0
    if abs(delta) <= 1e-12:
        return None
    fraction = (level - y0) / delta
    if fraction < -1e-9 or fraction > 1.0 + 1e-9:
        return None
    return float(x0 + np.clip(fraction, 0.0, 1.0) * (x1 - x0))


def _fit_background(x: np.ndarray, y: np.ndarray, shape_model: str) -> Optional[float]:
    """Fit only to recover a contaminated background candidate.

    Crossings and width remain data-derived.  The fitted centre/scale are not
    returned and cannot silently replace explicit FWHM geometry.
    """
    if curve_fit is None or x.size < 9:
        return None
    span = float(x[-1] - x[0])
    resolution = max(float(np.median(np.diff(x))), 1e-6)

    def profile(x_value, background, amplitude, center, scale):
        q = ((x_value - center) / scale) ** 2
        if shape_model == "physical_single_wire":
            return background + amplitude / np.sqrt(1.0 + q)
        return background + amplitude / (1.0 + q)

    minimum = float(np.min(y))
    maximum = float(np.max(y))
    amplitude = max(maximum - minimum, 1e-9)
    try:
        params, _ = curve_fit(
            profile,
            x,
            y,
            p0=(minimum, amplitude, float(x[int(np.argmax(y))]), max(span / 6.0, resolution)),
            bounds=(
                (minimum - 2.0 * amplitude, 0.0, float(x[0]), resolution),
                (minimum, 4.0 * max(amplitude, 1e-9), float(x[-1]), 2.0 * span),
            ),
            maxfev=10000,
        )
    except (RuntimeError, ValueError, FloatingPointError):
        return None
    fitted = float(params[0])
    return fitted if math.isfinite(fitted) else None


def _normalized_profile(x: np.ndarray, center: float, width: float, shape_model: str) -> np.ndarray:
    """Compare shapes at measured peak, centre and width, independent of depth calibration."""
    u2 = ((x - center) / max(width, 1e-12)) ** 2
    if shape_model == "physical_single_wire":
        return 1.0 / np.sqrt(1.0 + 12.0 * u2)
    return 1.0 / (1.0 + 4.0 * u2)


def equal_dof_shape_residuals(result: FwhmResult) -> Optional[tuple[float, float]]:
    """Compare two field families after fitting one background for each.

    The ordinary FWHM background can be estimated with the declared family.
    Reusing that baseline for both candidates rewards the declared family by
    construction. This comparison instead gives each family the same one
    scalar nuisance parameter and normalizes both residuals by the same
    measured peak-to-edge scale. It is only an identifiability check; it does
    not recalibrate the FWHM width-to-distance factor.
    """
    if (result.center_x_m is None or result.width_m is None or result.peak_value_nt is None
            or result.shape_model not in {"thesis_dipole", "physical_single_wire"}):
        return None
    x = np.asarray(result.x_m, dtype=float)
    y = np.asarray(result.smoothed_envelope_nt, dtype=float)
    if x.size < 9 or x.size != y.size or not np.isfinite(y).all():
        return None
    scale = float(result.peak_value_nt - np.min(y))
    if scale <= 1e-9:
        return None

    def residual(model: str) -> float:
        shape = _normalized_profile(x, result.center_x_m, result.width_m, model)
        tail = 1. - shape
        background = float(np.dot(tail, y - result.peak_value_nt * shape) /
                           max(np.dot(tail, tail), 1e-12))
        predicted = background + (result.peak_value_nt - background) * shape
        return float(np.sqrt(np.mean((y - predicted) ** 2)) / scale)

    competing = ("physical_single_wire" if result.shape_model == "thesis_dipole"
                 else "thesis_dipole")
    return residual(result.shape_model), residual(competing)


def _empty(reasons: list[str], sample_count: int, shape_model: str, shape_factor: float) -> FwhmResult:
    return FwhmResult(
        valid=False, peak_x_m=None, peak_value_nt=None, baseline_nt=None,
        half_level_nt=None, left_crossing_x_m=None, right_crossing_x_m=None,
        width_m=None, center_x_m=None, vertical_separation_m=None,
        burial_depth_m=None, width_sigma_m=None, validity_score=0.0,
        confidence=0.0, prominence_nt=0.0, snr_db=float("-inf"),
        symmetry_score=0.0, coverage_score=0.0, model_residual=None,
        sample_count=sample_count, shape_model=shape_model, shape_factor=shape_factor,
        reason_codes=tuple(reasons),
    )


def estimate_fwhm(
    x_m: np.ndarray,
    envelope_nt: np.ndarray,
    config: FwhmConfig,
    *,
    noise_nt: Optional[float] = None,
    baseline_nt: Optional[float] = None,
    altitude_m: Optional[float] = None,
) -> FwhmResult:
    """Estimate peak, half-height crossings, spatial width and model-qualified depth."""
    config.validate()
    x = np.asarray(x_m, dtype=float).reshape(-1)
    y = np.asarray(envelope_nt, dtype=float).reshape(-1)
    if x.size != y.size:
        raise ValueError("x_m and envelope_nt must have equal lengths")
    finite = np.isfinite(x) & np.isfinite(y)
    x, y = x[finite], y[finite]
    if x.size < config.minimum_samples:
        return _empty(["insufficient_samples"], int(x.size), config.shape_model, config.shape_factor)
    order = np.argsort(x)
    x, y = x[order], y[order]
    unique_x, inverse = np.unique(x, return_inverse=True)
    if unique_x.size != x.size:
        sums = np.bincount(inverse, weights=y)
        counts = np.bincount(inverse)
        x, y = unique_x, sums / np.maximum(counts, 1)
    if x.size < config.minimum_samples or float(x[-1] - x[0]) <= 1e-9:
        return _empty(
            ["insufficient_spatial_coverage"], int(x.size), config.shape_model, config.shape_factor
        )

    smooth = _median_filter(y, config.smoothing_window)
    edge_count = max(int(math.ceil(config.edge_fraction * x.size)), 2)
    edge_values = np.r_[smooth[:edge_count], smooth[-edge_count:]]
    edge_baseline = float(np.median(edge_values))
    if baseline_nt is not None and math.isfinite(float(baseline_nt)):
        baseline = float(baseline_nt)
        # A positive anomaly background cannot sit above the observed edge.
        # This indicates Search began inside the cable tail; recover background
        # from the declared profile while retaining measured crossings.
        if baseline > float(np.min(edge_values)):
            fitted_background = _fit_background(x, smooth, config.shape_model)
            baseline = edge_baseline if fitted_background is None else fitted_background
    else:
        fitted_background = _fit_background(x, smooth, config.shape_model)
        baseline = edge_baseline if fitted_background is None else fitted_background
    estimated_noise = float(1.4826 * np.median(np.abs(edge_values - edge_baseline)))
    resolved_noise = max(
        estimated_noise if noise_nt is None or not math.isfinite(float(noise_nt)) else float(noise_nt),
        1e-9,
    )
    peak_index = int(np.argmax(smooth))
    peak_value = float(smooth[peak_index])
    prominence = peak_value - baseline
    snr_db = 20.0 * math.log10(max(prominence, 1e-12) / resolved_noise)
    reasons: list[str] = []
    if peak_index < config.minimum_side_samples or x.size - peak_index - 1 < config.minimum_side_samples:
        reasons.append("peak_not_bracketed")
    if prominence < config.minimum_prominence_nt:
        reasons.append("low_prominence")
    if snr_db < config.minimum_snr_db:
        reasons.append("low_snr")
    half_level = baseline + 0.5 * prominence

    left = None
    for idx in range(peak_index - 1, -1, -1):
        if (smooth[idx] - half_level) * (smooth[idx + 1] - half_level) <= 0.0:
            left = _crossing(x[idx], smooth[idx], x[idx + 1], smooth[idx + 1], half_level)
            if left is not None:
                break
    right = None
    for idx in range(peak_index, x.size - 1):
        if (smooth[idx] - half_level) * (smooth[idx + 1] - half_level) <= 0.0:
            right = _crossing(x[idx], smooth[idx], x[idx + 1], smooth[idx + 1], half_level)
            if right is not None:
                break
    if left is None:
        reasons.append("left_half_crossing_missing")
    if right is None:
        reasons.append("right_half_crossing_missing")
    if left is None or right is None or right <= left:
        return FwhmResult(
            valid=False, peak_x_m=float(x[peak_index]), peak_value_nt=peak_value,
            baseline_nt=baseline, half_level_nt=half_level,
            left_crossing_x_m=left, right_crossing_x_m=right, width_m=None,
            center_x_m=None, vertical_separation_m=None, burial_depth_m=None,
            width_sigma_m=None, validity_score=0.0, confidence=0.0,
            prominence_nt=prominence, snr_db=snr_db, symmetry_score=0.0,
            coverage_score=0.0, model_residual=None, sample_count=int(x.size),
            shape_model=config.shape_model, shape_factor=config.shape_factor,
            reason_codes=tuple(dict.fromkeys(reasons)),
            x_m=x, smoothed_envelope_nt=smooth,
        )

    width = float(right - left)
    center = 0.5 * (left + right)
    left_width, right_width = float(x[peak_index] - left), float(right - x[peak_index])
    symmetry = min(left_width, right_width) / max(left_width, right_width, 1e-12)
    margin = min(left - x[0], x[-1] - right)
    coverage = float(np.clip(margin / max(config.minimum_coverage_factor * width, 1e-12), 0.0, 1.0))
    if coverage < 1.0:
        reasons.append("insufficient_half_width_margin")

    z = config.shape_factor * width
    model = baseline + prominence * _normalized_profile(x, center, width, config.shape_model)
    residual = float(np.sqrt(np.mean((smooth - model) ** 2)) / max(prominence, 1e-12))
    if residual > config.maximum_model_residual:
        reasons.append("profile_model_mismatch")
    competing_shape = "thesis_dipole" if config.shape_model == "physical_single_wire" else "physical_single_wire"
    competing_model = baseline + prominence * _normalized_profile(x, center, width, competing_shape)
    competing_residual = float(
        np.sqrt(np.mean((smooth - competing_model) ** 2)) / max(prominence, 1e-12)
    )
    shape_support = min(center - x[0], x[-1] - center) / max(width, 1e-12)
    if shape_support < config.minimum_shape_support_widths:
        reasons.append("shape_evidence_insufficient")
    elif (
        residual - competing_residual > config.minimum_competing_shape_advantage
        and competing_residual < config.maximum_competing_shape_residual_ratio * residual
    ):
        reasons.append("competing_shape_model_preferred")
    resolution = float(np.median(np.diff(x)))
    width_sigma = float(math.sqrt(2.0) * max(resolution, resolved_noise * width / max(prominence, 1e-12)))

    prominence_score = float(np.clip(prominence / max(config.minimum_prominence_nt * 2.0, 1e-12), 0.0, 1.0))
    snr_score = float(np.clip((snr_db - config.minimum_snr_db) / 12.0, 0.0, 1.0))
    model_score = float(np.clip(1.0 - residual / max(config.maximum_model_residual, 1e-12), 0.0, 1.0))
    validity = float(
        0.20 * prominence_score + 0.20 * snr_score + 0.20 * symmetry
        + 0.20 * coverage + 0.20 * model_score
    )
    resolved_altitude = config.altitude_m if altitude_m is None else float(altitude_m)
    if z < resolved_altitude:
        reasons.append("vertical_separation_below_altitude")
    hard_invalid = any(
        reason in reasons
        for reason in (
            "peak_not_bracketed",
            "low_prominence",
            "low_snr",
            "profile_model_mismatch",
            "shape_evidence_insufficient",
            "competing_shape_model_preferred",
            "vertical_separation_below_altitude",
        )
    )
    valid = not hard_invalid and coverage > 0.0 and validity >= config.minimum_validity_score
    if validity < config.minimum_validity_score:
        reasons.append("validity_below_threshold")
    confidence = (
        float(validity * np.clip(1.0 - width_sigma / max(width, 1e-12), 0.0, 1.0))
        if valid else 0.0
    )
    vertical = z if valid else None
    burial = z - resolved_altitude if valid else None
    model_sigma = config.shape_factor_relative_sigma * z
    vertical_sigma = (
        math.hypot(config.shape_factor * width_sigma, model_sigma) if valid else None
    )
    burial_sigma = (
        math.hypot(vertical_sigma, config.altitude_sigma_m)
        if vertical_sigma is not None
        else None
    )
    return FwhmResult(
        valid=valid, peak_x_m=float(x[peak_index]), peak_value_nt=peak_value,
        baseline_nt=baseline, half_level_nt=half_level, left_crossing_x_m=left,
        right_crossing_x_m=right, width_m=width, center_x_m=center,
        vertical_separation_m=vertical, burial_depth_m=burial,
        width_sigma_m=width_sigma, validity_score=validity, confidence=confidence,
        prominence_nt=prominence, snr_db=snr_db, symmetry_score=float(symmetry),
        coverage_score=coverage, model_residual=residual, sample_count=int(x.size),
        shape_model=config.shape_model, shape_factor=config.shape_factor,
        reason_codes=tuple(dict.fromkeys(reasons)),
        x_m=x, smoothed_envelope_nt=smooth,
        vertical_separation_sigma_m=vertical_sigma,
        burial_depth_sigma_m=burial_sigma,
        competing_model_residual=competing_residual,
        shape_support_widths=float(shape_support),
    )
