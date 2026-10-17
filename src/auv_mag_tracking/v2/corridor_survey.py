"""Opt-in multi-crossing survey of one isolated magnetic corridor.

This is a geometric candidate, not an automatic target-identity or depth
release. Three separated transverse profiles estimate both the lateral
position and local heading before longitudinal tracking. A reviewed corridor
isolation claim must come from outside magnetic peak fitting.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .config import FwhmConfig, InspectionConfig
from .fwhm import estimate_fwhm
from .lateral import qualify_lateral_center
from .search import _point_in_polygon


@dataclass(frozen=True)
class CorridorSurveyConfig:
    half_width_m: float = 15.0
    station_spacing_m: float = 20.0
    station_count: int = 3
    velocity_mps: float = 0.3
    boundary_margin_m: float = 5.0
    maximum_scan_gap_m: float = 1.0
    minimum_crossing_span_fraction: float = 0.7
    maximum_along_excursion_m: float = 2.0
    maximum_center_sigma_m: float = 0.75
    maximum_line_residual_m: float = 0.75
    maximum_heading_deviation_deg: float = 15.0
    maximum_prior_bend_deg: float = 5.0

    def validate(self) -> None:
        values = (self.half_width_m, self.station_spacing_m, self.velocity_mps,
                  self.maximum_scan_gap_m, self.maximum_along_excursion_m,
                  self.maximum_center_sigma_m, self.maximum_line_residual_m,
                  self.maximum_heading_deviation_deg, self.maximum_prior_bend_deg)
        if not all(math.isfinite(value) and value > 0 for value in values):
            raise ValueError("survey limits must be finite and positive")
        if not math.isfinite(self.boundary_margin_m) or self.boundary_margin_m < 0:
            raise ValueError("survey boundary margin must be finite and nonnegative")
        if self.station_count < 3:
            raise ValueError("at least three separated crossings are required")
        if self.maximum_heading_deviation_deg >= 45 or self.maximum_prior_bend_deg >= 45:
            raise ValueError("survey heading bounds must be below 45 degrees")
        if not 0 < self.minimum_crossing_span_fraction <= 1:
            raise ValueError("survey crossing span fraction must be in (0, 1]")


@dataclass(frozen=True)
class SurveyPlan:
    station_centers_xy_m: np.ndarray
    station_along_m: np.ndarray
    waypoints_xy_m: np.ndarray
    prior_tangent_xy: np.ndarray
    prior_normal_xy: np.ndarray
    velocity_mps: float
    estimated_distance_m: float
    profile_index_ranges: tuple[tuple[int, int], ...] = ()
    path_kind: str = "transverse_geometry"


@dataclass(frozen=True)
class ShallowZigzagConfig:
    amplitude_m: float = 24.0
    half_wavelength_m: float = 135.0
    station_count: int = 3
    waypoint_spacing_m: float = 2.0
    velocity_mps: float = 0.3
    minimum_turn_radius_m: float = 50.0
    boundary_margin_m: float = 5.0
    prior_offset_bound_m: float = 6.0
    prior_heading_error_bound_deg: float = 1.15
    half_crossing_bound_m: float = 5.2
    tail_margin_m: float = 1.5
    navigation_lateral_margin_m: float = 3.0
    initial_lateral_sign: int = -1

    def validate(self) -> None:
        values = (self.amplitude_m, self.half_wavelength_m,
                  self.waypoint_spacing_m, self.velocity_mps,
                  self.minimum_turn_radius_m)
        if not all(math.isfinite(value) and value > 0 for value in values):
            raise ValueError("shallow zig-zag limits must be finite and positive")
        if self.station_count < 3 or self.waypoint_spacing_m >= self.half_wavelength_m:
            raise ValueError("shallow zig-zag needs three stations and dense waypoints")
        if not math.isfinite(self.boundary_margin_m) or self.boundary_margin_m < 0:
            raise ValueError("shallow zig-zag boundary margin must be nonnegative")
        coverage = (self.prior_offset_bound_m, self.prior_heading_error_bound_deg,
                    self.half_crossing_bound_m, self.tail_margin_m,
                    self.navigation_lateral_margin_m)
        if not all(math.isfinite(value) and value >= 0 for value in coverage):
            raise ValueError("shallow zig-zag coverage budgets must be finite and nonnegative")
        if self.prior_heading_error_bound_deg >= 45:
            raise ValueError("shallow zig-zag prior heading bound must be below 45 degrees")
        if self.initial_lateral_sign not in (-1, 1):
            raise ValueError("shallow zig-zag initial side must be -1 or +1")
        needed = (self.prior_offset_bound_m
                  + math.tan(math.radians(self.prior_heading_error_bound_deg))
                  * self.station_count * self.half_wavelength_m
                  + self.half_crossing_bound_m + self.tail_margin_m
                  + self.navigation_lateral_margin_m)
        if needed > self.amplitude_m:
            raise ValueError("shallow zig-zag amplitude cannot cover declared prior/profile envelope")


def build_shallow_zigzag_survey(
    prior_path_xy_m: np.ndarray, boundary_xy_m: np.ndarray,
    config: ShallowZigzagConfig,
) -> SurveyPlan:
    """Continuous cosine half-waves with an analytic curvature lower bound.

    This proves a reference curve has no sharp corner and lies in the convex
    survey envelope; a PVS/real vehicle still needs tracking validation.
    """
    config.validate()
    prior = np.asarray(prior_path_xy_m, dtype=float)
    boundary = np.asarray(boundary_xy_m, dtype=float)
    if prior.ndim != 2 or prior.shape[1] != 2 or len(prior) < 2 or not np.isfinite(prior).all():
        raise ValueError("shallow zig-zag prior must be finite (N, 2) points")
    if not _convex_polygon(boundary):
        raise ValueError("shallow zig-zag boundary must be a convex polygon")
    chord = prior[-1] - prior[0]
    total_prior_m = float(np.linalg.norm(chord))
    if total_prior_m <= 1e-9:
        raise ValueError("shallow zig-zag prior has no forward chord")
    tangent = chord / total_prior_m
    segments = np.diff(prior, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    if np.any(lengths <= 1e-9):
        raise ValueError("shallow zig-zag prior has a zero-length segment")
    if np.any(np.degrees(np.arccos(np.clip(segments @ tangent / lengths, -1., 1.))) > 5.):
        raise ValueError("shallow zig-zag needs an approximately straight prior")
    normal = np.array([-tangent[1], tangent[0]])
    total_s = config.station_count * config.half_wavelength_m
    if total_s > total_prior_m + 1e-9:
        raise ValueError("shallow zig-zag exceeds the prior route")
    minimum_curve_radius = config.half_wavelength_m ** 2 / (config.amplitude_m * math.pi ** 2)
    if minimum_curve_radius < config.minimum_turn_radius_m:
        raise ValueError("shallow zig-zag curvature exceeds turn-radius bound")
    steps_per_halfwave = int(math.ceil(config.half_wavelength_m / config.waypoint_spacing_m))
    s = np.linspace(0., total_s, config.station_count * steps_per_halfwave + 1)
    lateral = config.initial_lateral_sign * config.amplitude_m * np.cos(
        math.pi * s / config.half_wavelength_m)
    waypoints = prior[0] + s[:, None] * tangent + lateral[:, None] * normal
    for point in waypoints:
        if not _point_in_polygon(point, boundary) or _clearance(point, boundary) < config.boundary_margin_m:
            raise ValueError("shallow zig-zag lacks boundary/stop margin")
    station_along = (np.arange(config.station_count, dtype=float) + .5) * config.half_wavelength_m
    station_centers = prior[0] + station_along[:, None] * tangent
    ranges = tuple((i * steps_per_halfwave, (i + 1) * steps_per_halfwave + 1)
                   for i in range(config.station_count))
    distance = float(np.sum(np.linalg.norm(np.diff(waypoints, axis=0), axis=1)))
    return SurveyPlan(station_centers, station_along, waypoints, tangent, normal,
                      config.velocity_mps, distance, ranges, "shallow_cosine_zigzag")


@dataclass(frozen=True)
class SurveyProfile:
    station_index: int
    positions_xy_m: np.ndarray
    envelope_nt: np.ndarray
    noise_nt: float | None = None


@dataclass(frozen=True)
class SurveyEstimate:
    geometry_valid: bool
    line_start_xy_m: np.ndarray | None
    line_end_xy_m: np.ndarray | None
    station_centers_xy_m: np.ndarray
    center_sigmas_m: np.ndarray
    heading_deviation_deg: float | None
    fit_rmse_m: float | None
    burial_depth_m: None
    reason_codes: tuple[str, ...]


@dataclass(frozen=True)
class TurnFeasibility:
    straight_run_sufficient: bool
    required_tangent_m: np.ndarray
    available_segment_m: np.ndarray
    reason_codes: tuple[str, ...]


@dataclass(frozen=True)
class StartSideDecision:
    initial_lateral_sign: int | None
    measured_crosscurrent_mps: float | None
    reason_codes: tuple[str, ...]


def select_shallow_start_side(
    prior_tangent_xy: np.ndarray, estimated_current_xy_mps: np.ndarray,
    current_sigma_mps: float, *, maximum_crosscurrent_mps: float = 0.05,
    quiet_current_mps: float = 0.02,
) -> StartSideDecision:
    """Choose the upstream flank from a supplied, uncertainty-bounded current.

    This does not measure current or authorize staging. An ambiguous sign in
    a meaningful flow rejects automatic side selection.
    """
    tangent = np.asarray(prior_tangent_xy, dtype=float)
    current = np.asarray(estimated_current_xy_mps, dtype=float)
    if (tangent.shape != (2,) or current.shape != (2,)
            or not np.isfinite(tangent).all() or not np.isfinite(current).all()
            or np.linalg.norm(tangent) <= 1e-9
            or not math.isfinite(current_sigma_mps) or current_sigma_mps < 0
            or not math.isfinite(maximum_crosscurrent_mps) or maximum_crosscurrent_mps <= 0
            or not math.isfinite(quiet_current_mps) or quiet_current_mps < 0):
        return StartSideDecision(None, None, ("survey_current_evidence_invalid",))
    tangent = tangent / np.linalg.norm(tangent)
    normal = np.array([-tangent[1], tangent[0]])
    lateral = float(np.dot(current, normal))
    bound = abs(lateral) + 2. * current_sigma_mps
    if bound > maximum_crosscurrent_mps + 1e-12:
        return StartSideDecision(None, lateral, ("survey_crosscurrent_outside_envelope",))
    if abs(lateral) <= 2. * current_sigma_mps:
        if bound <= quiet_current_mps:
            return StartSideDecision(-1, lateral, ("survey_quiet_current_default_side",))
        return StartSideDecision(None, lateral, ("survey_crosscurrent_direction_ambiguous",))
    return StartSideDecision(-1 if lateral > 0 else 1, lateral, ())


def assess_turn_feasibility(plan: SurveyPlan, minimum_turn_radius_m: float) -> TurnFeasibility:
    """Necessary fillet-length check, not a dynamic or obstacle clearance proof."""
    if not math.isfinite(minimum_turn_radius_m) or minimum_turn_radius_m <= 0:
        raise ValueError("minimum turn radius must be finite and positive")
    segments = np.diff(plan.waypoints_xy_m, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    if len(lengths) < 2 or np.any(lengths <= 1e-9):
        raise ValueError("turn check requires distinct consecutive waypoints")
    headings = segments / lengths[:, None]
    corner_angles = np.arccos(np.clip(np.sum(headings[:-1] * headings[1:], axis=1), -1., 1.))
    tangents = minimum_turn_radius_m * np.tan(corner_angles / 2.)
    required = np.zeros_like(lengths)
    required[:-1] += tangents
    required[1:] += tangents
    feasible = bool(np.all(lengths + 1e-9 >= required))
    return TurnFeasibility(feasible, tangents, lengths,
                           () if feasible else ("survey_waypoint_turns_exceed_straight_runs",))


def _convex_polygon(boundary: np.ndarray) -> bool:
    if boundary.ndim != 2 or boundary.shape[1] != 2 or len(boundary) < 3:
        return False
    if not np.isfinite(boundary).all():
        return False
    edges = np.roll(boundary, -1, axis=0) - boundary
    cross = np.cross(edges, np.roll(edges, -1, axis=0))
    nonzero = cross[np.abs(cross) > 1e-9]
    return bool(len(nonzero) and (np.all(nonzero > 0) or np.all(nonzero < 0)))


def _clearance(point: np.ndarray, boundary: np.ndarray) -> float:
    a = boundary
    b = np.roll(a, -1, axis=0)
    edges = b - a
    t = np.clip(np.sum((point - a) * edges, axis=1) /
                np.maximum(np.sum(edges * edges, axis=1), 1e-12), 0., 1.)
    return float(np.min(np.linalg.norm(point - (a + t[:, None] * edges), axis=1)))


def build_corridor_survey(
    prior_path_xy_m: np.ndarray, boundary_xy_m: np.ndarray,
    config: CorridorSurveyConfig,
) -> SurveyPlan:
    """Plan alternating transverse crossings inside a convex work envelope.

    Convexity is deliberate: it proves each straight connector between
    admitted waypoints remains in the polygon. Turn and stop footprints still
    need separate vehicle-specific acceptance before deployment.
    """
    config.validate()
    prior = np.asarray(prior_path_xy_m, dtype=float)
    boundary = np.asarray(boundary_xy_m, dtype=float)
    if prior.ndim != 2 or prior.shape[1] != 2 or len(prior) < 2 or not np.isfinite(prior).all():
        raise ValueError("survey prior must be finite (N, 2) points")
    if not _convex_polygon(boundary):
        raise ValueError("survey boundary must be a finite convex polygon")
    segments = np.diff(prior, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    if np.any(lengths <= 1e-9):
        raise ValueError("survey prior has a zero-length segment")
    chord = prior[-1] - prior[0]
    if np.linalg.norm(chord) <= 1e-9:
        raise ValueError("survey prior has no forward chord")
    tangent = chord / np.linalg.norm(chord)
    deviations = np.degrees(np.arccos(np.clip(segments @ tangent / lengths, -1., 1.)))
    if np.max(deviations) > config.maximum_prior_bend_deg:
        raise ValueError("survey prior bends beyond the declared envelope")
    normal = np.array([-tangent[1], tangent[0]])
    station_along = np.arange(config.station_count, dtype=float) * config.station_spacing_m
    if station_along[-1] > float(np.dot(prior[-1] - prior[0], tangent)) + 1e-9:
        raise ValueError("survey stations exceed the prior path")
    centers = prior[0] + station_along[:, None] * tangent
    endpoints = []
    for idx, center in enumerate(centers):
        left, right = center - config.half_width_m * normal, center + config.half_width_m * normal
        endpoints.extend((left, right) if idx % 2 == 0 else (right, left))
    waypoints = np.asarray(endpoints)
    for point in waypoints:
        if not _point_in_polygon(point, boundary) or _clearance(point, boundary) < config.boundary_margin_m:
            raise ValueError("survey crossing lacks boundary/stop margin")
    distance = float(np.sum(np.linalg.norm(np.diff(waypoints, axis=0), axis=1)))
    return SurveyPlan(centers, station_along, waypoints, tangent, normal,
                      config.velocity_mps, distance)


def estimate_survey_line(
    plan: SurveyPlan, profiles: list[SurveyProfile], config: CorridorSurveyConfig,
    fwhm_config: FwhmConfig, inspection_config: InspectionConfig,
) -> SurveyEstimate:
    """Fit a local line only when every separated crossing has one sound peak.

    The FWHM geometry is reused for centre localization. Its depth conversion
    is intentionally discarded even if the profile happens to fit a model.
    """
    config.validate()
    count = len(plan.station_along_m)
    reasons: list[str] = []
    centers = np.full((count, 2), np.nan)
    sigmas = np.full(count, np.nan)
    indices = [profile.station_index for profile in profiles]
    if (any(not isinstance(index, (int, np.integer)) for index in indices)
            or sorted(indices) != list(range(count))):
        reasons.append("survey_station_set_incomplete_or_duplicated")
    else:
        for profile in profiles:
            idx = profile.station_index
            positions = np.asarray(profile.positions_xy_m, dtype=float)
            envelope = np.asarray(profile.envelope_nt, dtype=float)
            if (positions.ndim != 2 or positions.shape[1] != 2 or envelope.ndim != 1
                    or len(positions) != len(envelope) or not np.isfinite(positions).all()
                    or not np.isfinite(envelope).all()):
                reasons.append(f"station_{idx}_invalid_samples")
                continue
            relative = positions - plan.station_centers_xy_m[idx]
            x = relative @ plan.prior_normal_xy
            along = relative @ plan.prior_tangent_xy
            # The executed crossing may drift relative to the reference
            # centreline. Require substantial measured span; let the actual
            # half-height crossings and margins decide whether this source
            # was covered. Demanding both nominal endpoints would reject a
            # complete shifted profile without increasing correctness.
            if (len(x) < fwhm_config.minimum_samples
                    or np.ptp(along) > config.maximum_along_excursion_m
                    or np.ptp(x) < 2 * config.half_width_m * config.minimum_crossing_span_fraction):
                reasons.append(f"station_{idx}_incomplete_crossing")
                continue
            if np.max(np.diff(np.unique(np.sort(x)))) > config.maximum_scan_gap_m:
                reasons.append(f"station_{idx}_spatial_gap")
                continue
            fwhm = estimate_fwhm(x, envelope, fwhm_config,
                                 noise_nt=profile.noise_nt, altitude_m=0.)
            lateral = qualify_lateral_center(fwhm, fwhm_config, inspection_config)
            if not lateral.valid:
                reasons.extend(f"station_{idx}_{reason}" for reason in lateral.reason_codes)
                continue
            if lateral.sigma_m > config.maximum_center_sigma_m:
                reasons.append(f"station_{idx}_center_uncertain")
                continue
            # On an oblique transect the lateral peak is reached at a distinct
            # along-track position. Retain its measured 2-D navigation point;
            # projecting it onto the nominal station would bias heading.
            order = np.argsort(x)
            peak_along = float(np.interp(lateral.center_x_m, x[order], along[order]))
            centers[idx] = (plan.station_centers_xy_m[idx]
                            + peak_along * plan.prior_tangent_xy
                            + lateral.center_x_m * plan.prior_normal_xy)
            sigmas[idx] = lateral.sigma_m
    if reasons:
        return SurveyEstimate(False, None, None, centers, sigmas, None, None, None,
                              tuple(dict.fromkeys(reasons)))
    origin = plan.station_centers_xy_m[0] - plan.station_along_m[0] * plan.prior_tangent_xy
    measured_s = (centers - origin) @ plan.prior_tangent_xy
    offset = (centers - origin) @ plan.prior_normal_xy
    reference_s = float(np.mean(measured_s))
    centered_s = measured_s - reference_s
    weights = 1. / np.maximum(sigmas, 0.05) ** 2
    design = np.column_stack((np.ones(count), centered_s))
    params = np.linalg.lstsq(design * np.sqrt(weights[:, None]),
                             offset * np.sqrt(weights), rcond=None)[0]
    fitted = design @ params
    rmse = float(np.sqrt(np.mean((offset - fitted) ** 2)))
    heading = math.degrees(math.atan(float(params[1])))
    if rmse > config.maximum_line_residual_m:
        reasons.append("survey_line_inconsistent")
    if abs(heading) > config.maximum_heading_deviation_deg:
        reasons.append("survey_heading_outside_prior_envelope")
    if reasons:
        return SurveyEstimate(False, None, None, centers, sigmas, heading, rmse, None,
                              tuple(reasons))
    end_s = measured_s[[0, -1]]
    end_offsets = params[0] + params[1] * (end_s - reference_s)
    endpoints = (origin + end_s[:, None] * plan.prior_tangent_xy
                 + end_offsets[:, None] * plan.prior_normal_xy)
    return SurveyEstimate(True, endpoints[0], endpoints[1], centers, sigmas,
                          heading, rmse, None, ())
