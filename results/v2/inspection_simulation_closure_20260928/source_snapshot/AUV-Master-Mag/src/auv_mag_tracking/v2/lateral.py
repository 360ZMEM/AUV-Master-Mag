"""Qualify the centre of a single symmetric magnetic anomaly independently of depth."""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from scipy.signal import find_peaks

from .config import FwhmConfig, InspectionConfig
from .types import FwhmResult


@dataclass(frozen=True)
class LateralConfirmation:
    valid: bool
    center_x_m: float | None
    sigma_m: float | None
    confidence: float
    reason_codes: tuple[str, ...]


def qualify_lateral_center(
    result: FwhmResult, fwhm: FwhmConfig, policy: InspectionConfig,
) -> LateralConfirmation:
    """A geometric anomaly centre is not a claim of cable identity or burial depth.

    Deployment scope: one dominant, approximately symmetric cable anomaly in
    the declared corridor. Model mismatch does not invalidate its centre.
    """
    reasons: list[str] = []
    geometry_failures = {
        "peak_not_bracketed", "low_prominence", "low_snr",
        "left_half_crossing_missing", "right_half_crossing_missing",
        "along_track_excursion_excessive", "insufficient_samples",
        "insufficient_spatial_coverage",
        "scan_spatial_gap",
    }
    reasons.extend(sorted(geometry_failures.intersection(result.reason_codes)))
    needed = (result.center_x_m, result.width_m, result.left_crossing_x_m, result.right_crossing_x_m)
    if any(value is None or not math.isfinite(value) for value in needed):
        reasons.append("center_geometry_missing")
    if result.width_m is not None and result.width_m <= 0:
        reasons.append("nonpositive_width")
    if not math.isfinite(result.snr_db) or result.snr_db < fwhm.minimum_snr_db:
        reasons.append("low_snr")
    if result.prominence_nt < fwhm.minimum_prominence_nt:
        reasons.append("low_prominence")
    if result.symmetry_score < policy.minimum_center_symmetry:
        reasons.append("asymmetric_anomaly")
    if result.coverage_score < policy.minimum_center_coverage:
        reasons.append("center_coverage_insufficient")
    if len(result.smoothed_envelope_nt):
        peaks, _ = find_peaks(
            result.smoothed_envelope_nt,
            prominence=max(0.25 * result.prominence_nt, fwhm.minimum_prominence_nt),
        )
        # A sampled, nearly flat summit can be split into two equal-height
        # peaks by tiny noise after median filtering. Treat only adjacent
        # summits with both a short separation and a sub-noise saddle as one.
        distinct_peaks = 0
        previous_peak = None
        noise_nt = result.prominence_nt / 10. ** (result.snr_db / 20.)
        for peak in peaks:
            if previous_peak is None:
                distinct_peaks += 1
            else:
                separation_m = result.x_m[peak] - result.x_m[previous_peak]
                saddle_nt = min(result.smoothed_envelope_nt[previous_peak],
                                result.smoothed_envelope_nt[peak]) - min(
                                    result.smoothed_envelope_nt[previous_peak:peak + 1])
                if (result.width_m is None or separation_m > 0.05 * result.width_m
                        or saddle_nt > max(3. * noise_nt, 0.01 * result.prominence_nt)):
                    distinct_peaks += 1
            previous_peak = peak
        if distinct_peaks != 1:
            reasons.append("ambiguous_anomaly_peaks")
    else:
        reasons.append("profile_missing")
    if reasons:
        return LateralConfirmation(False, None, None, 0.0, tuple(dict.fromkeys(reasons)))
    sigma = max(
        (result.width_sigma_m or 0.5) * 0.5,
        (1 - result.symmetry_score) * result.width_m * 0.5,
        0.05,
    )
    snr_score = float(np.clip((result.snr_db - fwhm.minimum_snr_db) / 12, 0, 1))
    confidence = float(np.clip(0.5 * result.symmetry_score + 0.3 * result.coverage_score + 0.2 * snr_score, 0, 1))
    return LateralConfirmation(True, result.center_x_m, sigma, confidence, ())
