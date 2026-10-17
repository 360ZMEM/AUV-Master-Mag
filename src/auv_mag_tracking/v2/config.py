"""Validated configuration for the independent v2 pipeline."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class SearchConfig:
    search_width_m: float = 30.0
    lane_spacing_m: float = 6.0
    velocity_mps: float = 0.7
    waypoint_spacing_m: float = 5.0
    arrival_tolerance_m: float = 1.5
    guidance_lookahead_m: float = 12.0
    pattern: str = "longitudinal"

    def validate(self) -> None:
        if self.pattern not in {"longitudinal", "transverse"}:
            raise ValueError("search pattern must be longitudinal or transverse")
        if self.search_width_m <= 0.0:
            raise ValueError("search_width_m must be positive")
        if self.lane_spacing_m <= 0.0 or self.lane_spacing_m > self.search_width_m:
            raise ValueError("lane_spacing_m must be in (0, search_width_m]")
        if self.velocity_mps <= 0.0 or self.waypoint_spacing_m <= 0.0:
            raise ValueError("search velocity and waypoint spacing must be positive")
        if self.guidance_lookahead_m < self.waypoint_spacing_m:
            raise ValueError("guidance_lookahead_m must be at least waypoint_spacing_m")


@dataclass(frozen=True)
class DetectionConfig:
    window_samples: int = 80
    min_ready_samples: int = 20
    threshold_on: float = 6.0
    threshold_off: float = 3.0
    consecutive_on: int = 3
    consecutive_off: int = 6
    minimum_noise_nt: float = 0.05
    peak_prominence_score: float = 2.0

    def validate(self) -> None:
        if self.window_samples < 5 or self.min_ready_samples < 5:
            raise ValueError("detection windows must contain at least five samples")
        if self.min_ready_samples > self.window_samples:
            raise ValueError("min_ready_samples cannot exceed window_samples")
        if self.threshold_on <= self.threshold_off or self.threshold_off < 0.0:
            raise ValueError("detection thresholds must satisfy on > off >= 0")
        if self.consecutive_on < 1 or self.consecutive_off < 1:
            raise ValueError("detection persistence must be positive")


@dataclass(frozen=True)
class CrossScanConfig:
    half_span_levels_m: tuple[float, ...] = (4.0, 8.0, 14.0, 20.0)
    velocity_mps: float = 0.45
    arrival_tolerance_m: float = 0.6
    boundary_margin_m: float = 0.2
    max_attempts: int = 4
    alignment_tolerance_deg: float = 8.0
    alignment_speed_mps: float = 0.3
    maximum_along_track_range_m: float = 5.0
    recenter_on_partial_peak: bool = True
    early_complete_when_valid: bool = True
    early_completion_coverage_score: float = 0.8
    return_tolerance_m: float = 2.0

    def validate(self) -> None:
        if not self.half_span_levels_m or any(value <= 0.0 for value in self.half_span_levels_m):
            raise ValueError("half_span_levels_m must contain positive values")
        if tuple(sorted(self.half_span_levels_m)) != self.half_span_levels_m:
            raise ValueError("half_span_levels_m must be sorted")
        if self.velocity_mps <= 0.0 or self.max_attempts < 1:
            raise ValueError("cross-scan velocity and attempts must be positive")
        if not 0.0 < self.alignment_tolerance_deg <= 90.0:
            raise ValueError("alignment_tolerance_deg must be in (0, 90]")
        if self.alignment_speed_mps < 0.0 or self.alignment_speed_mps > self.velocity_mps:
            raise ValueError("alignment_speed_mps must be in [0, velocity_mps]")
        if self.maximum_along_track_range_m <= 0.0:
            raise ValueError("maximum_along_track_range_m must be positive")
        if not 0.0 <= self.early_completion_coverage_score <= 1.0:
            raise ValueError("early_completion_coverage_score must be in [0, 1]")
        if self.return_tolerance_m <= 0.0:
            raise ValueError("return_tolerance_m must be positive")


@dataclass(frozen=True)
class FwhmConfig:
    minimum_samples: int = 21
    smoothing_window: int = 5
    edge_fraction: float = 0.15
    minimum_prominence_nt: float = 0.25
    minimum_snr_db: float = 6.0
    minimum_side_samples: int = 3
    minimum_coverage_factor: float = 0.15
    maximum_model_residual: float = 0.35
    minimum_shape_support_widths: float = 0.65
    minimum_competing_shape_advantage: float = 0.02
    maximum_competing_shape_residual_ratio: float = 0.75
    minimum_validity_score: float = 0.55
    shape_model: str = "thesis_dipole"
    shape_factor: float = 0.5
    altitude_m: float = 6.0
    shape_factor_relative_sigma: float = 0.05
    altitude_sigma_m: float = 0.25

    def validate(self) -> None:
        if self.minimum_samples < 7:
            raise ValueError("minimum_samples must be at least seven")
        if self.smoothing_window < 1 or self.smoothing_window % 2 == 0:
            raise ValueError("smoothing_window must be a positive odd integer")
        if not 0.0 < self.edge_fraction < 0.5:
            raise ValueError("edge_fraction must be in (0, 0.5)")
        if self.shape_factor <= 0.0 or self.altitude_m < 0.0:
            raise ValueError("shape_factor must be positive and altitude non-negative")
        if self.shape_factor_relative_sigma < 0.0 or self.altitude_sigma_m < 0.0:
            raise ValueError("FWHM model and altitude uncertainty must be non-negative")
        if not self.shape_model.strip():
            raise ValueError("shape_model must identify the field profile used for depth conversion")
        if self.minimum_shape_support_widths <= 0.0:
            raise ValueError("minimum_shape_support_widths must be positive")
        if self.minimum_competing_shape_advantage < 0.0:
            raise ValueError("minimum_competing_shape_advantage must be non-negative")
        if not 0.0 < self.maximum_competing_shape_residual_ratio < 1.0:
            raise ValueError("maximum_competing_shape_residual_ratio must be in (0, 1)")


@dataclass(frozen=True)
class TrackingConfig:
    heading_gain_deg_per_m: float = 3.0
    lateral_gain: float = 0.8
    max_heading_correction_deg: float = 20.0
    max_lateral_correction_m: float = 3.0
    nominal_speed_mps: float = 0.8
    minimum_speed_mps: float = 0.3
    offset_deadband_m: float = 0.1
    route_lookahead_m: float = 8.0
    rejoin_tolerance_m: float = 1.0
    endpoint_tolerance_m: float = 2.0
    stop_at_endpoint: bool = True

    def validate(self) -> None:
        if self.heading_gain_deg_per_m < 0.0 or self.lateral_gain < 0.0:
            raise ValueError("tracking gains must be non-negative")
        if self.minimum_speed_mps <= 0.0 or self.nominal_speed_mps < self.minimum_speed_mps:
            raise ValueError("tracking speeds are inconsistent")
        if (
            self.route_lookahead_m <= 0.0
            or self.rejoin_tolerance_m <= 0.0
            or self.endpoint_tolerance_m <= 0.0
        ):
            raise ValueError("tracking lookahead, rejoin tolerance and endpoint tolerance must be positive")


@dataclass(frozen=True)
class InspectionConfig:
    """Opt-in operational contract; legacy experiment semantics remain available."""

    enabled: bool = False
    allow_uncalibrated_depth: bool = False
    minimum_center_symmetry: float = 0.70
    minimum_center_coverage: float = 0.8
    maximum_observation_age_s: float = 120.0
    maximum_observation_distance_m: float = 30.0
    maximum_scan_duration_s: float = 240.0
    maximum_input_age_s: float = 1.5
    maximum_navigation_sigma_m: float = 2.0
    maximum_guidance_speed_mps: float = 0.8
    maximum_scan_gap_m: float = 1.0
    maximum_scan_samples: int = 5000
    boundary_stop_margin_m: float = 0.0
    minimum_depth_shape_support_widths: float = 2.0
    minimum_declared_shape_advantage: float = 0.02

    def validate(self) -> None:
        import math
        if not 0 < self.minimum_center_symmetry <= 1 or not 0 < self.minimum_center_coverage <= 1:
            raise ValueError("center quality gates must be in (0, 1]")
        values = (self.maximum_observation_age_s, self.maximum_observation_distance_m,
                  self.maximum_scan_duration_s, self.maximum_input_age_s,
                  self.maximum_navigation_sigma_m, self.maximum_guidance_speed_mps)
        if any(not math.isfinite(value) or value <= 0 for value in values):
            raise ValueError("inspection limits must be finite and positive")
        if not math.isfinite(self.maximum_scan_gap_m) or self.maximum_scan_gap_m <= 0 or self.maximum_scan_samples < 21:
            raise ValueError("invalid scan spatial gap or sample budget")
        if not math.isfinite(self.boundary_stop_margin_m) or self.boundary_stop_margin_m < 0:
            raise ValueError("boundary stop margin must be finite and nonnegative")
        if not math.isfinite(self.minimum_depth_shape_support_widths) or self.minimum_depth_shape_support_widths <= 0:
            raise ValueError("depth shape support must be finite and positive")
        if not math.isfinite(self.minimum_declared_shape_advantage) or self.minimum_declared_shape_advantage < 0:
            raise ValueError("declared shape advantage must be finite and nonnegative")


@dataclass(frozen=True)
class V2Config:
    search: SearchConfig = field(default_factory=SearchConfig)
    detection: DetectionConfig = field(default_factory=DetectionConfig)
    cross_scan: CrossScanConfig = field(default_factory=CrossScanConfig)
    fwhm: FwhmConfig = field(default_factory=FwhmConfig)
    tracking: TrackingConfig = field(default_factory=TrackingConfig)
    inspection: InspectionConfig = field(default_factory=InspectionConfig)

    def validate(self) -> None:
        self.search.validate()
        self.detection.validate()
        self.cross_scan.validate()
        self.fwhm.validate()
        self.tracking.validate()
        self.inspection.validate()
