"""Data contracts for the independent v2 cable-search and FWHM pipeline."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np


class MissionPhase(str, Enum):
    SEARCH = "search"
    CONFIRM = "confirm"
    TRACK = "track"
    REACQUIRE = "reacquire"


@dataclass(frozen=True)
class SearchPlan:
    waypoints_xy_m: np.ndarray
    lane_offsets_m: np.ndarray
    velocity_mps: float
    estimated_distance_m: float
    estimated_time_s: float
    valid: bool = True
    reason_codes: tuple[str, ...] = ()


@dataclass(frozen=True)
class DetectionResult:
    time_s: float
    envelope_nt: float
    baseline_nt: float
    noise_nt: float
    score: float
    detected: bool
    peak_evidence: bool
    sample_count: int
    reason_codes: tuple[str, ...] = ()


@dataclass(frozen=True)
class CrossScanPlan:
    scan_id: int
    center_xy_m: np.ndarray
    tangent_xy: np.ndarray
    normal_xy: np.ndarray
    waypoints_xy_m: np.ndarray
    half_span_m: float
    velocity_mps: float
    valid: bool = True
    reason_codes: tuple[str, ...] = ()


@dataclass(frozen=True)
class FwhmResult:
    valid: bool
    peak_x_m: Optional[float]
    peak_value_nt: Optional[float]
    baseline_nt: Optional[float]
    half_level_nt: Optional[float]
    left_crossing_x_m: Optional[float]
    right_crossing_x_m: Optional[float]
    width_m: Optional[float]
    center_x_m: Optional[float]
    vertical_separation_m: Optional[float]
    burial_depth_m: Optional[float]
    width_sigma_m: Optional[float]
    validity_score: float
    confidence: float
    prominence_nt: float
    snr_db: float
    symmetry_score: float
    coverage_score: float
    model_residual: Optional[float]
    sample_count: int
    shape_model: str
    shape_factor: float
    reason_codes: tuple[str, ...] = ()
    x_m: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=float), repr=False)
    smoothed_envelope_nt: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=float), repr=False
    )
    vertical_separation_sigma_m: Optional[float] = None
    burial_depth_sigma_m: Optional[float] = None
    competing_model_residual: Optional[float] = None
    shape_support_widths: Optional[float] = None
    shape_intercept_m: Optional[float] = None
    inspection_declared_shape_residual: Optional[float] = None
    inspection_competing_shape_residual: Optional[float] = None


@dataclass(frozen=True)
class CableObservation:
    time_s: float
    position_xy_m: np.ndarray
    heading_deg: float
    lateral_sigma_m: float
    confidence: float
    source: str = "fwhm_cross_scan"


@dataclass(frozen=True)
class TrackingCorrection:
    signed_offset_m: float
    heading_correction_deg: float
    lateral_correction_m: float
    speed_mps: float
    confidence: float
    limited: bool
