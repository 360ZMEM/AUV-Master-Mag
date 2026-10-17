"""Interface-compatible v2 search, confirmation, FWHM and tracking pipeline."""

from __future__ import annotations

import math
from dataclasses import replace
from typing import Optional

import numpy as np

from ..api.cable_map import CableMap
from ..api.types import (
    CableGuidanceOutput,
    CableTrackingOutput,
    DeploymentPerceptionConfig,
    MagneticInput,
    NavigationInput,
    SonarInput,
)
from ..math_utils import nearest_point_on_polyline
from .cable_estimator import CableLineEstimator
from .config import FwhmConfig, V2Config
from .cross_scan import build_cross_scan
from .detection import MagneticDetector
from .fwhm import estimate_fwhm, equal_dof_shape_residuals
from .lateral import LateralConfirmation, qualify_lateral_center
from .shape_calibration import (
    AffineWidthCalibration, ShapeCalibration,
    estimate_with_affine_calibration, estimate_with_calibrated_models,
)
from .search import build_prior_guided_search, _point_in_polygon
from .state_machine import V2StateMachine
from .tracking import compute_tracking_correction
from .types import CableObservation, CrossScanPlan, DetectionResult, FwhmResult, MissionPhase


def _finite_diagnostics(value):
    """Diagnostics use JSON null for unavailable numbers, never NaN/Infinity."""
    if isinstance(value, dict):
        return {key: _finite_diagnostics(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_finite_diagnostics(item) for item in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    return value


def _boundary_clearance_m(point_xy: np.ndarray, polygon_xy: np.ndarray) -> float:
    """Shortest in-plane distance to a polygon edge, for a stop buffer."""
    vertices = np.asarray(polygon_xy, dtype=float)
    point = np.asarray(point_xy, dtype=float)
    a = vertices
    b = np.roll(vertices, -1, axis=0)
    edges = b - a
    length2 = np.sum(edges * edges, axis=1)
    fraction = np.clip(np.sum((point - a) * edges, axis=1) / np.maximum(length2, 1e-12), 0., 1.)
    nearest = a + fraction[:, None] * edges
    return float(np.min(np.linalg.norm(nearest - point, axis=1)))


class AuvMagTrackingPipelineV2:
    """A deterministic v2 facade with the same step contracts as v1.

    Navigation positions provide the spatial coordinate used by FWHM.  Truth is
    never consumed.  The cable prior selects a search corridor and an initial
    tangent; confirmed cross-scan centres are maintained separately as
    observation-only cable points.
    """

    def __init__(
        self,
        config,
        cable_map: CableMap,
        deployment_config: Optional[DeploymentPerceptionConfig] = None,
        *,
        v2_config: Optional[V2Config] = None,
        search_boundary_xy_m: Optional[np.ndarray] = None,
        initial_position_uncertainty_m: float = 0.0,
        depth_calibrations: Optional[tuple[tuple[FwhmConfig, ShapeCalibration], ...]] = None,
        affine_depth_calibration: Optional[tuple[FwhmConfig, AffineWidthCalibration]] = None,
    ) -> None:
        self.config = config
        self.cable_map = cable_map
        self.deployment_config = deployment_config
        self.v2_config = v2_config or V2Config()
        self.v2_config.validate()
        self.depth_calibrations = depth_calibrations
        self.affine_depth_calibration = affine_depth_calibration
        if depth_calibrations is not None and affine_depth_calibration is not None:
            raise ValueError("choose either constant multi-model or affine depth calibration")
        self.search_boundary_xy_m = (
            None if search_boundary_xy_m is None else np.asarray(search_boundary_xy_m, dtype=float)
        )
        self.initial_position_uncertainty_m = float(initial_position_uncertainty_m)
        self._cache = cable_map.projection_cache()
        self.detector = MagneticDetector(self.v2_config.detection)
        self.state_machine = V2StateMachine()
        self.line_estimator = CableLineEstimator()
        self.last_output: Optional[CableTrackingOutput] = None
        self._search_plan = None
        self._search_waypoint_index = 0
        self._scan_plan: Optional[CrossScanPlan] = None
        self._scan_waypoint_index = 0
        self._scan_x_m: list[float] = []
        self._scan_envelope_nt: list[float] = []
        self._scan_noise_nt: list[float] = []
        self._scan_along_track_m: list[float] = []
        self._scan_aligned = False
        self._scan_attempt = 0
        self._scan_id = 0
        self._confirmation_exhausted = False
        self._retry_pending = False
        self._detection_at_confirm: Optional[DetectionResult] = None
        self._confirmation_center_xy: Optional[np.ndarray] = None
        self._confirmation_tangent_xy: Optional[np.ndarray] = None
        self._scan_center_strategy = "detection_position"
        self._scan_completion_reason = "not_complete"
        self._last_fwhm: Optional[FwhmResult] = None
        self._last_observation: Optional[CableObservation] = None
        self._observation_prior_offset_m: Optional[float] = None
        self._tracking_target_xy: Optional[np.ndarray] = None
        self._tracking_stage = "unavailable"
        self._endpoint_complete = False
        self._last_lateral: Optional[LateralConfirmation] = None
        self._scan_started_s = 0.0
        self._last_step_s: Optional[float] = None
        self._last_navigation: Optional[NavigationInput] = None
        self._continuity_reason = "initial_search"
        self._refresh_scan_pending = False
        self._scan_measurement_time_s = 0.0
        self._inspection_started = False
        self._mission_complete = False

    def reset(self) -> None:
        self.detector.reset()
        self.state_machine.reset()
        self.line_estimator.reset()
        self.last_output = None
        self._search_plan = None
        self._search_waypoint_index = 0
        self._scan_plan = None
        self._scan_waypoint_index = 0
        self._scan_x_m.clear()
        self._scan_envelope_nt.clear()
        self._scan_noise_nt.clear()
        self._scan_along_track_m.clear()
        self._scan_aligned = False
        self._scan_attempt = 0
        self._scan_id = 0
        self._confirmation_exhausted = False
        self._retry_pending = False
        self._detection_at_confirm = None
        self._confirmation_center_xy = None
        self._confirmation_tangent_xy = None
        self._scan_center_strategy = "detection_position"
        self._scan_completion_reason = "not_complete"
        self._last_fwhm = None
        self._last_observation = None
        self._observation_prior_offset_m = None
        self._tracking_target_xy = None
        self._tracking_stage = "unavailable"
        self._endpoint_complete = False
        self._last_lateral = None
        self._scan_started_s = 0.0
        self._last_step_s = None
        self._last_navigation = None
        self._continuity_reason = "initial_search"
        self._refresh_scan_pending = False
        self._scan_measurement_time_s = 0.0
        self._inspection_started = False
        self._mission_complete = False

    def export_state(self) -> dict[str, object]:
        return {
            "initialized": True,
            "has_output": self.last_output is not None,
            "algorithm_version": "v2",
            "mission_phase": self.state_machine.phase.value,
            "search_waypoint_index": self._search_waypoint_index,
            "scan_attempt": self._scan_attempt,
            "fwhm_valid": bool(self._last_fwhm is not None and self._last_fwhm.valid),
        }

    @staticmethod
    def _envelope_nt(magnetic: MagneticInput) -> float:
        try:
            samples = np.asarray(magnetic.sample_block_nt, dtype=float).reshape(-1, 3)
        except Exception:
            return float("nan")
        finite = samples[np.all(np.isfinite(samples), axis=1)]
        if finite.size == 0:
            return float("nan")
        return float(np.median(np.linalg.norm(finite, axis=1)))

    def _ensure_search_plan(self, nav_xy: np.ndarray) -> None:
        if self._search_plan is None:
            self._search_plan = build_prior_guided_search(
                self.cable_map.points_xy_m,
                self.v2_config.search,
                initial_position_xy_m=nav_xy,
                initial_position_uncertainty_m=self.initial_position_uncertainty_m,
                search_boundary_xy_m=self.search_boundary_xy_m,
            )

    def _advance_search(self, nav_xy: np.ndarray) -> None:
        if self._search_plan is None or not self._search_plan.valid:
            return
        while self._search_waypoint_index < len(self._search_plan.waypoints_xy_m):
            target = self._search_plan.waypoints_xy_m[self._search_waypoint_index]
            reached = float(np.linalg.norm(nav_xy - target)) <= self.v2_config.search.arrival_tolerance_m
            passed = False
            if self.v2_config.inspection.enabled and self._search_waypoint_index > 0:
                previous = self._search_plan.waypoints_xy_m[self._search_waypoint_index - 1]
                direction = target - previous
                length = float(np.linalg.norm(direction))
                lateral = abs(float(np.dot(nav_xy - target, [-direction[1], direction[0]]))) / max(length, 1e-9)
                passed = bool(np.dot(nav_xy - target, direction) >= 0 and lateral <= self.v2_config.search.arrival_tolerance_m)
            elif not self.v2_config.inspection.enabled and self._search_waypoint_index + 1 < len(self._search_plan.waypoints_xy_m):
                following = self._search_plan.waypoints_xy_m[self._search_waypoint_index + 1]
                direction = following - target
                passed = float(np.dot(nav_xy - target, direction)) >= 0.0
            if not reached and not passed:
                break
            self._search_waypoint_index += 1

    def _search_guidance_waypoint(self, nav_xy: np.ndarray) -> Optional[np.ndarray]:
        if self._search_plan is None or not self._search_plan.valid:
            return None
        points = self._search_plan.waypoints_xy_m
        if self._search_waypoint_index >= len(points):
            return None
        if self.v2_config.inspection.enabled and self._search_waypoint_index == 0:
            return points[0]
        target_index = self._search_waypoint_index
        while target_index + 1 < len(points):
            if float(np.linalg.norm(points[target_index] - nav_xy)) >= self.v2_config.search.guidance_lookahead_m:
                break
            if self.v2_config.inspection.enabled and target_index > 0:
                incoming = points[target_index] - points[target_index - 1]
                outgoing = points[target_index + 1] - points[target_index]
                cosine = float(np.dot(incoming, outgoing)) / max(float(np.linalg.norm(incoming) * np.linalg.norm(outgoing)), 1e-9)
                if cosine < math.cos(math.radians(45)):
                    break
            target_index += 1
        return points[target_index]

    def _start_scan(self, nav_xy: np.ndarray, point_xy: np.ndarray, tangent_xy: np.ndarray) -> None:
        self._scan_started_s = self._last_step_s or 0.0
        if self.v2_config.inspection.enabled:
            self._last_lateral = None
            self._last_fwhm = None
        self._scan_id += 1
        self._scan_plan = build_cross_scan(
            scan_id=self._scan_id,
            center_xy_m=point_xy,
            tangent_xy=tangent_xy,
            current_position_xy_m=nav_xy,
            config=self.v2_config.cross_scan,
            attempt_index=self._scan_attempt,
            search_boundary_xy_m=self.search_boundary_xy_m,
        )
        self._scan_waypoint_index = 0
        self._scan_x_m.clear()
        self._scan_envelope_nt.clear()
        self._scan_noise_nt.clear()
        self._scan_along_track_m.clear()
        self._scan_aligned = False
        self._scan_completion_reason = "not_complete"

    def _estimate_current_scan(self, altitude_m: Optional[float]) -> FwhmResult:
        result = self._estimate_current_scan_unqualified(altitude_m)
        if not self.v2_config.inspection.enabled:
            return result
        reasons = []
        equal_dof = equal_dof_shape_residuals(result)
        if equal_dof is None:
            reasons.append("depth_shape_comparison_unavailable")
        else:
            result = replace(
                result, inspection_declared_shape_residual=equal_dof[0],
                inspection_competing_shape_residual=equal_dof[1],
            )
            if equal_dof[1] - equal_dof[0] < self.v2_config.inspection.minimum_declared_shape_advantage:
                reasons.append("depth_shape_ambiguous")
        # A short outer tail can let an incorrect field family fit better than
        # the true one after background adjustment. Withhold depth unless both
        # sides span enough measured width to test the model's decay; lateral
        # peak geometry is qualified separately below.
        if (
            result.width_m is not None and
            (result.shape_support_widths is None or
             result.shape_support_widths < self.v2_config.inspection.minimum_depth_shape_support_widths)
        ):
            reasons.append("depth_shape_support_insufficient")
        if (
            not self.v2_config.inspection.allow_uncalibrated_depth
            and self.depth_calibrations is None and self.affine_depth_calibration is None
        ):
            reasons.append("depth_calibration_required")
        if altitude_m is None:
            reasons.append("fresh_altitude_required")
        if reasons:
            return replace(
                result, valid=False, confidence=0.0,
                vertical_separation_m=None, burial_depth_m=None,
                vertical_separation_sigma_m=None, burial_depth_sigma_m=None,
                reason_codes=tuple(dict.fromkeys((*result.reason_codes, *reasons))),
            )
        return result

    def _estimate_current_scan_unqualified(self, altitude_m: Optional[float]) -> FwhmResult:
        baseline = None if self._detection_at_confirm is None else self._detection_at_confirm.baseline_nt
        finite_noise = np.asarray(self._scan_noise_nt, dtype=float)
        finite_noise = finite_noise[np.isfinite(finite_noise)]
        noise = float(np.median(finite_noise)) if finite_noise.size else None
        if self.affine_depth_calibration is not None:
            config, calibration = self.affine_depth_calibration
            return estimate_with_affine_calibration(
                np.asarray(self._scan_x_m, dtype=float),
                np.asarray(self._scan_envelope_nt, dtype=float),
                config, calibration, baseline_nt=baseline, noise_nt=noise,
                altitude_m=altitude_m,
            )
        if self.depth_calibrations is not None:
            if not self.depth_calibrations:
                raise ValueError("depth_calibrations must contain at least one candidate")
            return estimate_with_calibrated_models(
                np.asarray(self._scan_x_m, dtype=float),
                np.asarray(self._scan_envelope_nt, dtype=float),
                self.depth_calibrations,
                baseline_nt=baseline, noise_nt=noise, altitude_m=altitude_m,
            )
        return estimate_fwhm(
            np.asarray(self._scan_x_m, dtype=float),
            np.asarray(self._scan_envelope_nt, dtype=float),
            self.v2_config.fwhm,
            baseline_nt=baseline,
            noise_nt=noise,
            altitude_m=altitude_m,
        )

    def _point_at_progress(self, progress_m: float) -> tuple[np.ndarray, np.ndarray]:
        """Interpolate a finite prior route at clamped arc length."""
        total_m = float(self._cache.cumulative_length_m[-1])
        progress = float(np.clip(progress_m, 0.0, total_m))
        index = int(np.searchsorted(self._cache.cumulative_length_m[1:], progress, side="right"))
        index = min(index, len(self._cache.segment_lengths_m) - 1)
        length_m = float(self._cache.segment_lengths_m[index])
        if length_m <= 1e-12:
            return self._cache.segment_starts_xy[index].copy(), self._cache.segment_tangents_xy[index].copy()
        alpha = (progress - float(self._cache.cumulative_length_m[index])) / length_m
        point = self._cache.segment_starts_xy[index] + np.clip(alpha, 0.0, 1.0) * self._cache.segment_vectors_xy[index]
        return point.copy(), self._cache.segment_tangents_xy[index].copy()

    def _update_scan(
        self,
        navigation: NavigationInput,
        nav_xy: np.ndarray,
        envelope_nt: float,
        detection: DetectionResult,
        prior_heading_deg: float,
        altitude_m: Optional[float],
    ) -> None:
        plan = self._scan_plan
        if self.v2_config.inspection.enabled and (
            navigation.time_s - self._scan_started_s > self.v2_config.inspection.maximum_scan_duration_s
            or len(self._scan_x_m) >= self.v2_config.inspection.maximum_scan_samples
        ):
            self._expire_observation("scan_timeout")
            return
        if plan is None or not plan.valid:
            self.state_machine.update(detected=False, scan_complete=True, fwhm_valid=False)
            if self.v2_config.inspection.enabled:
                self._expire_observation("scan_boundary_unavailable")
            return

        if self._scan_waypoint_index == 2 and self._confirmation_valid():
            return_target = plan.waypoints_xy_m[2]
            return_distance_m = float(np.linalg.norm(nav_xy - return_target))
            if (
                return_distance_m
                > self.v2_config.cross_scan.return_tolerance_m
            ):
                return
            self._scan_waypoint_index = 3
            self._scan_completion_reason += "+center_returned"
            self.state_machine.update(
                detected=detection.detected,
                scan_complete=True,
                fwhm_valid=bool(self._last_fwhm.valid),
                lateral_valid=bool(self._last_lateral is not None and self._last_lateral.valid),
            )
            if self._last_fwhm.center_x_m is not None:
                observed_xy = plan.center_xy_m + self._last_fwhm.center_x_m * plan.normal_xy
                sigma = max((self._last_fwhm.width_sigma_m or 0.5) * 0.5, 0.05)
                observation_confidence = self._last_fwhm.confidence
                if self.v2_config.inspection.enabled and self._last_lateral is not None:
                    sigma = self._last_lateral.sigma_m
                    observation_confidence = self._last_lateral.confidence
                observation = CableObservation(
                    time_s=float(navigation.time_s),
                    position_xy_m=observed_xy,
                    heading_deg=prior_heading_deg,
                    lateral_sigma_m=sigma,
                    confidence=observation_confidence,
                    source="magnetic_anomaly_center" if self.v2_config.inspection.enabled else "fwhm_cross_scan",
                )
                self.line_estimator.add(observation)
                self._last_observation = observation
                self._scan_attempt = 0
                self._confirmation_exhausted = False
                self._continuity_reason = "lateral_observation_confirmed"
            return

        if self._scan_waypoint_index == 0:
            first = plan.waypoints_xy_m[0]
            first_x = float(np.dot(first - plan.center_xy_m, plan.normal_xy))
            current_x = float(np.dot(nav_xy - plan.center_xy_m, plan.normal_xy))
            reached_first_plane = math.copysign(1.0, first_x) * current_x >= (
                abs(first_x) - self.v2_config.cross_scan.arrival_tolerance_m
            )
            if reached_first_plane or float(np.linalg.norm(nav_xy - first)) <= self.v2_config.cross_scan.arrival_tolerance_m:
                self._scan_waypoint_index = 1
                self._scan_x_m.clear()
                self._scan_envelope_nt.clear()
                self._scan_noise_nt.clear()
                self._scan_along_track_m.clear()
                self._scan_aligned = False
            return

        if self._scan_waypoint_index == 1:
            x_m = float(np.dot(nav_xy - plan.center_xy_m, plan.normal_xy))
            if not self._scan_aligned:
                desired_heading_deg = math.degrees(
                    math.atan2(plan.waypoints_xy_m[1, 1] - nav_xy[1], plan.waypoints_xy_m[1, 0] - nav_xy[0])
                )
                heading_error_deg = (desired_heading_deg - navigation.heading_deg + 180.0) % 360.0 - 180.0
                if abs(heading_error_deg) <= self.v2_config.cross_scan.alignment_tolerance_deg:
                    self._scan_aligned = True
                    self._scan_x_m.clear()
                    self._scan_envelope_nt.clear()
                    self._scan_noise_nt.clear()
                    self._scan_along_track_m.clear()
                return
            if math.isfinite(envelope_nt):
                self._scan_x_m.append(x_m)
                self._scan_envelope_nt.append(envelope_nt)
                self._scan_noise_nt.append(detection.noise_nt)
                self._scan_along_track_m.append(
                    float(np.dot(nav_xy - plan.center_xy_m, plan.tangent_xy))
                )
            second = plan.waypoints_xy_m[1]
            second_x = float(np.dot(second - plan.center_xy_m, plan.normal_xy))
            reached_second_plane = math.copysign(1.0, second_x) * x_m >= (
                abs(second_x) - self.v2_config.cross_scan.arrival_tolerance_m
            )
            reached_endpoint = reached_second_plane or (
                float(np.linalg.norm(nav_xy - second))
                <= self.v2_config.cross_scan.arrival_tolerance_m
            )
            early_result = None
            enough_samples = len(self._scan_x_m) >= self.v2_config.fwhm.minimum_samples
            if (
                self.v2_config.cross_scan.early_complete_when_valid
                and not self.v2_config.inspection.enabled
                and enough_samples
                and len(self._scan_x_m) % 5 == 0
            ):
                candidate = self._estimate_current_scan(altitude_m)
                if (
                    candidate.valid
                    and candidate.coverage_score
                    >= self.v2_config.cross_scan.early_completion_coverage_score
                ):
                    early_result = candidate
            if not reached_endpoint and early_result is None:
                return

            self._last_fwhm = early_result or self._estimate_current_scan(altitude_m)
            self._scan_measurement_time_s = float(navigation.time_s)
            self._scan_completion_reason = (
                "fwhm_evidence_complete" if early_result is not None else "endpoint_reached"
            )
            along_track_range_m = (
                float(np.ptp(np.asarray(self._scan_along_track_m, dtype=float)))
                if self._scan_along_track_m
                else float("inf")
            )
            if along_track_range_m > self.v2_config.cross_scan.maximum_along_track_range_m:
                reasons = tuple(dict.fromkeys((*self._last_fwhm.reason_codes, "along_track_excursion_excessive")))
                self._last_fwhm = replace(
                    self._last_fwhm,
                    valid=False,
                    confidence=0.0,
                    vertical_separation_m=None, burial_depth_m=None,
                    vertical_separation_sigma_m=None, burial_depth_sigma_m=None,
                    validity_score=min(self._last_fwhm.validity_score, 0.25),
                    reason_codes=reasons,
                )
            self._scan_waypoint_index = 2
            if self.v2_config.inspection.enabled:
                positions = np.unique(np.asarray(self._scan_x_m))
                if len(positions) > 1 and np.max(np.diff(positions)) > self.v2_config.inspection.maximum_scan_gap_m:
                    self._last_fwhm = replace(
                        self._last_fwhm, valid=False, confidence=0.0,
                        vertical_separation_m=None, burial_depth_m=None,
                        vertical_separation_sigma_m=None, burial_depth_sigma_m=None,
                        reason_codes=(*self._last_fwhm.reason_codes, "scan_spatial_gap"),
                    )
                self._last_lateral = qualify_lateral_center(
                    self._last_fwhm, self.v2_config.fwhm, self.v2_config.inspection,
                )
                if self._last_lateral.valid:
                    waypoints = plan.waypoints_xy_m.copy()
                    waypoints[2] = plan.center_xy_m + self._last_lateral.center_x_m * plan.normal_xy
                    self._scan_plan = replace(plan, waypoints_xy_m=waypoints)
            if self._confirmation_valid():
                # Keep CONFIRM active while executing the third cross-scan
                # waypoint.  TRACK begins only after returning close to the
                # measured cable centre.
                return

            decision = self.state_machine.update(
                detected=detection.detected,
                scan_complete=True,
                fwhm_valid=False,
            )
            if decision.phase == MissionPhase.REACQUIRE:
                missing_crossing = any(
                    reason in self._last_fwhm.reason_codes
                    for reason in ("left_half_crossing_missing", "right_half_crossing_missing")
                )
                peak_is_usable = bool(
                    self.v2_config.cross_scan.recenter_on_partial_peak
                    and missing_crossing
                    and self._last_fwhm.peak_x_m is not None
                    and math.isfinite(float(self._last_fwhm.peak_x_m))
                    and self._last_fwhm.prominence_nt >= self.v2_config.fwhm.minimum_prominence_nt
                    and "peak_not_bracketed" not in self._last_fwhm.reason_codes
                    and "low_snr" not in self._last_fwhm.reason_codes
                )
                # An edge maximum supplies a direction for the next bounded
                # scan, never a localization observation. Keeping the original
                # detection centre can strand all retries on the same flank.
                flank_follow = False
                if self.v2_config.inspection.enabled and len(self._scan_x_m) >= self.v2_config.fwhm.minimum_samples:
                    result = self._last_fwhm
                    flank_follow = bool(
                        result.peak_x_m is not None
                        and math.isfinite(result.peak_x_m)
                        and result.prominence_nt >= self.v2_config.fwhm.minimum_prominence_nt
                        and result.snr_db >= self.v2_config.fwhm.minimum_snr_db
                        and not {"along_track_excursion_excessive", "scan_spatial_gap"}.intersection(result.reason_codes)
                        and self._last_lateral is not None
                        and "ambiguous_anomaly_peaks" not in self._last_lateral.reason_codes
                    )
                    # A monotonic flank has no interior peak. Permit it only
                    # as a search hint with a consistently signed gradient.
                    if "peak_not_bracketed" in result.reason_codes:
                        order = np.argsort(self._scan_x_m)
                        bins = np.array_split(np.asarray(self._scan_envelope_nt)[order], 5)
                        levels = np.asarray([np.median(part) for part in bins])
                        differences = np.diff(levels)
                        monotonic = bool(np.all(differences > 0) or np.all(differences < 0))
                        flank_follow = bool(
                            monotonic and result.peak_x_m is not None
                            and math.isfinite(result.peak_x_m)
                            and result.prominence_nt >= self.v2_config.fwhm.minimum_prominence_nt
                            and result.snr_db >= self.v2_config.fwhm.minimum_snr_db
                            and not {"along_track_excursion_excessive", "scan_spatial_gap"}.intersection(result.reason_codes)
                        )
                    peak_is_usable = peak_is_usable or flank_follow
                if peak_is_usable and self._confirmation_center_xy is not None:
                    # The peak is an observation of the lateral cable centre,
                    # even when one half-height tail is still outside the
                    # current scan.  Centre the expanded retry on that measured
                    # peak instead of repeating about the original detection.
                    peak_offset_m = float(np.clip(
                        self._last_fwhm.peak_x_m,
                        -plan.half_span_m,
                        plan.half_span_m,
                    ))
                    self._confirmation_center_xy = (
                        plan.center_xy_m + peak_offset_m * plan.normal_xy
                    )
                    self._scan_center_strategy = "bounded_flank_recenter" if flank_follow else "partial_peak_recenter"
                self._scan_attempt += 1
                self._scan_plan = None
                self._confirmation_exhausted = self._scan_attempt >= self.v2_config.cross_scan.max_attempts
                self._retry_pending = not self._confirmation_exhausted

    def _confirmation_valid(self) -> bool:
        if self.v2_config.inspection.enabled:
            return self._last_lateral is not None and self._last_lateral.valid
        return self._last_fwhm is not None and self._last_fwhm.valid

    def _expire_observation(self, reason: str) -> None:
        """A bounded estimate must never remain a perpetual tracking authority."""
        self.line_estimator.reset()
        self._last_observation = None
        self._last_lateral = None
        self._last_fwhm = None
        self._observation_prior_offset_m = None
        self._tracking_target_xy = None
        self._scan_plan = None
        self._scan_attempt = 0
        self._retry_pending = False
        self._confirmation_exhausted = False
        self._confirmation_center_xy = None
        self._confirmation_tangent_xy = None
        self.state_machine.phase = MissionPhase.REACQUIRE
        self.detector.reset()
        self._continuity_reason = reason
        self._refresh_scan_pending = False

    def _input_problem(self, navigation: NavigationInput, magnetic: MagneticInput) -> Optional[str]:
        try:
            position = np.asarray(navigation.position_ned_m, dtype=float)
            if position.shape != (3,) or not np.all(np.isfinite(position)):
                return "navigation_invalid"
            values = (navigation.time_s, navigation.heading_deg, navigation.speed_mps,
                      navigation.position_std_m, navigation.heading_std_deg)
            if not all(isinstance(value, (int, float, np.integer, np.floating))
                       and math.isfinite(float(value)) for value in values):
                return "navigation_invalid"
            if navigation.speed_mps < 0 or navigation.heading_std_deg < 0:
                return "navigation_invalid"
            if navigation.position_std_m < 0 or navigation.position_std_m > self.v2_config.inspection.maximum_navigation_sigma_m:
                return "navigation_uncertain"
            ground_velocity = getattr(navigation, "ground_velocity_ned_mps", None)
            if ground_velocity is not None:
                ground_velocity = np.asarray(ground_velocity, dtype=float)
                if ground_velocity.shape != (3,) or not np.isfinite(ground_velocity).all():
                    return "navigation_invalid"
            if self._last_step_s is not None and navigation.time_s <= self._last_step_s:
                return "navigation_time_not_advancing"
            if self.search_boundary_xy_m is not None and not _point_in_polygon(position[:2], self.search_boundary_xy_m):
                return "outside_search_boundary"
            margin = self.v2_config.inspection.boundary_stop_margin_m
            if (
                self.search_boundary_xy_m is not None and margin > 0
                and _boundary_clearance_m(position[:2], self.search_boundary_xy_m) <= margin
            ):
                return "boundary_stop_buffer"
            samples = np.asarray(magnetic.sample_block_nt, dtype=float)
            if samples.size == 0 or samples.size % 3 or not np.all(np.isfinite(samples)):
                return "magnetic_invalid"
            if not math.isfinite(magnetic.sample_rate_hz) or magnetic.sample_rate_hz <= 0:
                return "magnetic_rate_invalid"
            age = navigation.time_s - float(magnetic.time_s)
            if not math.isfinite(age) or age < -0.05 or age > self.v2_config.inspection.maximum_input_age_s:
                return "magnetic_stale_or_future"
        except (TypeError, ValueError, AttributeError, OverflowError):
            return "malformed_input"
        return None

    def _hold_output(self, reason: str) -> CableTrackingOutput:
        nav = self._last_navigation
        output = CableTrackingOutput(
            time_s=0.0 if nav is None else float(nav.time_s),
            estimated_cable_xy_m=np.zeros(2) if nav is None else np.asarray(nav.position_ned_m)[:2].copy(),
            cross_track_m=0.0, route_progress_m=0.0,
            cable_heading_deg=0.0 if nav is None else float(nav.heading_deg),
            burial_depth_m=None, burial_sigma_m=None, confidence=0.0,
            mode="hold", diagnostics={
                "algorithm_version": "v2", "mission_phase": "hold",
                "source": "unavailable", "continuity_reason": reason,
                "lateral_valid": False, "depth_valid": False,
                "inspection_enabled": True,
                "observation_only_available": False,
            },
        )
        self.last_output = output
        return output

    def step(
        self, navigation: NavigationInput, magnetic: MagneticInput,
        sonar: Optional[SonarInput] = None,
    ) -> CableTrackingOutput:
        if not self.v2_config.inspection.enabled:
            return self._step_impl(navigation, magnetic, sonar)
        problem = self._input_problem(navigation, magnetic)
        if problem is not None:
            return self._hold_output(problem)
        self._last_step_s = float(navigation.time_s)
        self._last_navigation = replace(navigation, position_ned_m=np.asarray(navigation.position_ned_m).copy())
        if self._mission_complete:
            output = self._hold_output("mission_complete")
            output.mode = "complete"
            output.diagnostics.update(mission_phase="complete", endpoint_complete=True)
            return output
        if self._last_observation is not None:
            age = navigation.time_s - self._last_observation.time_s
            distance = np.linalg.norm(np.asarray(navigation.position_ned_m)[:2] - self._last_observation.position_xy_m)
            if age > self.v2_config.inspection.maximum_observation_age_s or distance > self.v2_config.inspection.maximum_observation_distance_m:
                nav_xy = np.asarray(navigation.position_ned_m)[:2]
                prior_xy, tangent, _, _, _ = nearest_point_on_polyline(nav_xy, self._cache)
                normal = np.array([-tangent[1], tangent[0]])
                predicted_center = prior_xy + (self._observation_prior_offset_m or 0.0) * normal
                background_reference = self._detection_at_confirm
                previous_width = self._last_fwhm.width_m if self._last_fwhm is not None else None
                self._expire_observation("lateral_observation_expired")
                if previous_width is not None:
                    needed_span = 0.7 * previous_width + 1.0
                    levels = self.v2_config.cross_scan.half_span_levels_m
                    self._scan_attempt = next((i for i, span in enumerate(levels) if span >= needed_span), len(levels) - 1)
                self._confirmation_center_xy = predicted_center
                self._confirmation_tangent_xy = tangent
                self._detection_at_confirm = background_reference
                self._retry_pending = True
                self._refresh_scan_pending = True
        output = self._step_impl(navigation, magnetic, sonar)
        output.diagnostics = _finite_diagnostics(output.diagnostics)
        return output

    def _step_impl(
        self,
        navigation: NavigationInput,
        magnetic: MagneticInput,
        sonar: Optional[SonarInput] = None,
    ) -> CableTrackingOutput:
        dynamic_altitude_m = None
        sonar_altitude_m = None if sonar is None else getattr(sonar, "altitude_m", None)
        if sonar is not None and getattr(sonar, "valid", False) and sonar_altitude_m is not None:
            try:
                candidate_altitude_m = float(sonar_altitude_m)
                fresh = not self.v2_config.inspection.enabled or (
                    math.isfinite(float(sonar.time_s))
                    and 0 <= navigation.time_s - float(sonar.time_s) <= self.v2_config.inspection.maximum_input_age_s
                )
                if fresh and math.isfinite(candidate_altitude_m) and candidate_altitude_m >= 0.0:
                    dynamic_altitude_m = candidate_altitude_m
            except (TypeError, ValueError, AttributeError, OverflowError):
                if not self.v2_config.inspection.enabled:
                    raise
        nav_xy = np.asarray(navigation.position_ned_m, dtype=float)[:2]
        point_xy, tangent_xy, prior_distance_m, progress_m, segment_index = nearest_point_on_polyline(
            nav_xy, self._cache
        )
        normal_xy = np.array([-tangent_xy[1], tangent_xy[0]], dtype=float)
        prior_signed_offset_m = float(np.dot(nav_xy - point_xy, normal_xy))
        prior_heading_deg = float(math.degrees(math.atan2(tangent_xy[1], tangent_xy[0])))
        envelope_nt = self._envelope_nt(magnetic)
        detection = self.detector.update(navigation.time_s, envelope_nt)
        self._ensure_search_plan(nav_xy)

        if (
            not detection.detected
            and self.state_machine.phase == MissionPhase.REACQUIRE
            and self._confirmation_exhausted
        ):
            self._confirmation_exhausted = False
            if not self._retry_pending:
                self._scan_attempt = 0
                self._confirmation_center_xy = None
                self._confirmation_tangent_xy = None
                self._detection_at_confirm = None
        effective_detected = (detection.detected or self._retry_pending) and not self._confirmation_exhausted
        if self.state_machine.phase in {MissionPhase.SEARCH, MissionPhase.REACQUIRE}:
            decision = self.state_machine.update(detected=effective_detected)
            if decision.changed and decision.phase == MissionPhase.CONFIRM:
                if (self._scan_attempt == 0 and not self._refresh_scan_pending) or self._confirmation_center_xy is None:
                    self._detection_at_confirm = detection
                    # The prior supplies only the local tangent.  Centring the
                    # scan on the detected vehicle position preserves large
                    # prior-offset recovery and avoids circular prior forcing.
                    self._confirmation_center_xy = nav_xy.copy()
                    self._confirmation_tangent_xy = tangent_xy.copy()
                    self._scan_center_strategy = "detection_position"
                self._start_scan(
                    nav_xy,
                    self._confirmation_center_xy,
                    self._confirmation_tangent_xy,
                )
                self._retry_pending = False
                self._refresh_scan_pending = False

        if self.state_machine.phase == MissionPhase.SEARCH or (
            self.v2_config.inspection.enabled and self.state_machine.phase == MissionPhase.REACQUIRE
        ):
            self._advance_search(nav_xy)
        elif self.state_machine.phase == MissionPhase.CONFIRM:
            self._update_scan(
                navigation,
                nav_xy,
                envelope_nt,
                detection,
                prior_heading_deg,
                dynamic_altitude_m,
            )

        observation_center_xy, observation_heading_deg, line_sigma_m = self.line_estimator.estimate(prior_heading_deg)
        observation_only = observation_center_xy is not None
        self._tracking_target_xy = None
        self._tracking_stage = "unavailable"
        self._endpoint_complete = False
        if observation_center_xy is None:
            estimate_xy = point_xy
            estimated_heading_deg = prior_heading_deg
            signed_offset_m = prior_signed_offset_m
            distance_m = float(prior_distance_m)
        else:
            # A single cross-scan constrains lateral position well but cannot
            # justify extending a straight line without bound.  Transfer its
            # measured lateral correction onto the finite local prior route.
            obs_prior_xy, obs_prior_tangent, _, _, _ = nearest_point_on_polyline(
                observation_center_xy, self._cache
            )
            obs_prior_normal = np.array([-obs_prior_tangent[1], obs_prior_tangent[0]], dtype=float)
            self._observation_prior_offset_m = float(
                np.dot(observation_center_xy - obs_prior_xy, obs_prior_normal)
            )
            estimate_xy = point_xy + self._observation_prior_offset_m * normal_xy
            estimated_heading_deg = prior_heading_deg
            signed_offset_m = float(np.dot(nav_xy - estimate_xy, normal_xy))
            distance_m = abs(signed_offset_m)

            if self.state_machine.phase == MissionPhase.TRACK:
                if self.v2_config.inspection.enabled and not self._inspection_started:
                    start_xy, start_tangent = self._point_at_progress(0.0)
                    start_normal = np.array([-start_tangent[1], start_tangent[0]])
                    start_xy = start_xy + self._observation_prior_offset_m * start_normal
                    if np.linalg.norm(nav_xy - start_xy) <= self.v2_config.tracking.endpoint_tolerance_m:
                        self._inspection_started = True
                # First close the lateral error at the nearest corrected-route
                # projection.  Once rejoined, advance along the cable with the
                # configured lookahead.  This makes the transition from a
                # cross-scan to longitudinal inspection explicit and avoids a
                # long diagonal shortcut that is hard to interpret in replay.
                if distance_m > self.v2_config.tracking.rejoin_tolerance_m:
                    self._tracking_target_xy = estimate_xy.copy()
                    self._tracking_stage = "rejoin_estimated_cable"
                else:
                    total_route_m = float(self._cache.cumulative_length_m[-1])
                    returning_to_start = self.v2_config.inspection.enabled and not self._inspection_started
                    target_progress_m = (
                        max(0.0, progress_m - self.v2_config.tracking.route_lookahead_m)
                        if returning_to_start else min(progress_m + self.v2_config.tracking.route_lookahead_m, total_route_m)
                    )
                    target_prior_xy, target_tangent = self._point_at_progress(target_progress_m)
                    target_normal = np.array([-target_tangent[1], target_tangent[0]], dtype=float)
                    self._tracking_target_xy = (
                        target_prior_xy + self._observation_prior_offset_m * target_normal
                    )
                    self._tracking_stage = "return_to_inspection_start" if returning_to_start else "inspect_along_cable"
                total_route_m = float(self._cache.cumulative_length_m[-1])
                endpoint_target_xy, endpoint_tangent = self._point_at_progress(total_route_m)
                endpoint_normal = np.array([-endpoint_tangent[1], endpoint_tangent[0]], dtype=float)
                endpoint_target_xy = (
                    endpoint_target_xy + self._observation_prior_offset_m * endpoint_normal
                )
                self._endpoint_complete = bool(
                    self.v2_config.tracking.stop_at_endpoint
                    and (not self.v2_config.inspection.enabled or self._inspection_started)
                    and progress_m >= total_route_m - self.v2_config.tracking.endpoint_tolerance_m
                    and float(np.linalg.norm(nav_xy - endpoint_target_xy))
                    <= self.v2_config.tracking.endpoint_tolerance_m
                )
                if self.v2_config.inspection.enabled and self._endpoint_complete:
                    self._mission_complete = True

        fwhm = self._last_fwhm
        if self.state_machine.phase == MissionPhase.TRACK and fwhm is not None and fwhm.valid:
            confidence = fwhm.confidence
            burial_depth = fwhm.burial_depth_m
            burial_sigma = fwhm.burial_depth_sigma_m
        else:
            confidence = min(0.49, detection.score / max(self.v2_config.detection.threshold_on, 1e-9) * 0.49)
            burial_depth = None
            burial_sigma = None
        if self.v2_config.inspection.enabled and self.state_machine.phase == MissionPhase.TRACK and self._last_observation is not None:
            confidence = self._last_observation.confidence
            # A vertical result belongs to its scan; it is not a depth map of
            # all subsequent route points. Publish only while fresh and local.
            depth_local = (
                navigation.time_s - self._scan_measurement_time_s <= self.v2_config.inspection.maximum_input_age_s
                and np.linalg.norm(nav_xy - self._last_observation.position_xy_m) <= 1.0
                and dynamic_altitude_m is not None
            )
            if not depth_local:
                burial_depth = burial_sigma = None

        diagnostics = {
            "algorithm_version": "v2",
            "inspection_contract_enabled": self.v2_config.inspection.enabled,
            "continuity_reason": self._continuity_reason,
            "lateral_valid": bool(observation_only and self.state_machine.phase == MissionPhase.TRACK),
            "inspection_enabled": self.v2_config.inspection.enabled,
            "depth_valid": burial_depth is not None,
            "lateral_reason_codes": [] if self._last_lateral is None else list(self._last_lateral.reason_codes),
            "mission_phase": self.state_machine.phase.value,
            "source": ("magnetic_anomaly_center" if self.v2_config.inspection.enabled else "fwhm_observation") if observation_only else "prior_projection",
            "map_frame": self.cable_map.frame,
            "map_segment_index": int(segment_index),
            "signed_cross_track_m": signed_offset_m,
            "prior_signed_cross_track_m": prior_signed_offset_m,
            "observation_only_available": observation_only,
            "observation_heading_deg": None if not observation_only else observation_heading_deg,
            "observation_prior_offset_m": self._observation_prior_offset_m,
            "tracking_target_xy_m": None if self._tracking_target_xy is None else self._tracking_target_xy.tolist(),
            "tracking_stage": self._tracking_stage,
            "endpoint_complete": self._endpoint_complete,
            "observation_line_sigma_m": line_sigma_m,
            "fwhm_altitude_source": "sonar" if dynamic_altitude_m is not None else "config",
            "fwhm_altitude_m": self.v2_config.fwhm.altitude_m if dynamic_altitude_m is None else dynamic_altitude_m,
            "magnetic_envelope_nt": envelope_nt,
            "detection_score": detection.score,
            "detected": detection.detected,
            "detection_baseline_nt": detection.baseline_nt,
            "detection_noise_nt": detection.noise_nt,
            "detection_peak_evidence": detection.peak_evidence,
            "detection_reason_codes": list(detection.reason_codes),
            "search_plan_valid": bool(self._search_plan is not None and self._search_plan.valid),
            "search_waypoint_index": self._search_waypoint_index,
            "search_waypoint_count": 0 if self._search_plan is None else len(self._search_plan.waypoints_xy_m),
            "cross_scan_id": None if self._scan_plan is None else self._scan_plan.scan_id,
            "cross_scan_attempt": self._scan_attempt,
            "cross_scan_waypoint_index": self._scan_waypoint_index,
            "cross_scan_sample_count": len(self._scan_x_m),
            "cross_scan_aligned": self._scan_aligned,
            "cross_scan_along_track_range_m": None if not self._scan_along_track_m else float(np.ptp(np.asarray(self._scan_along_track_m, dtype=float))),
            "confirmation_exhausted": self._confirmation_exhausted,
            "cross_scan_retry_pending": self._retry_pending,
            "cross_scan_center_strategy": self._scan_center_strategy,
            "cross_scan_center_xy_m": None if self._confirmation_center_xy is None else self._confirmation_center_xy.tolist(),
            "cross_scan_completion_reason": self._scan_completion_reason,
            "cross_scan_return_distance_m": (
                None
                if self._scan_plan is None or self._scan_waypoint_index != 2
                else float(np.linalg.norm(nav_xy - self._scan_plan.waypoints_xy_m[2]))
            ),
        }
        if self._search_plan is not None and self._search_plan.valid:
            diagnostics["search_waypoints_xy_m"] = self._search_plan.waypoints_xy_m.tolist()
        if self._scan_plan is not None and self._scan_plan.valid:
            diagnostics["cross_scan_waypoints_xy_m"] = self._scan_plan.waypoints_xy_m.tolist()
        if self.state_machine.phase == MissionPhase.SEARCH and self._search_plan is not None and self._search_plan.valid:
            diagnostics["active_plan_kind"] = "prior_guided_search"
            diagnostics["active_plan_waypoints_xy_m"] = self._search_plan.waypoints_xy_m.tolist()
        elif self.state_machine.phase == MissionPhase.CONFIRM and self._scan_plan is not None and self._scan_plan.valid:
            diagnostics["active_plan_kind"] = "fwhm_cross_scan"
            diagnostics["active_plan_waypoints_xy_m"] = self._scan_plan.waypoints_xy_m.tolist()
        elif self.state_machine.phase == MissionPhase.TRACK and self._tracking_target_xy is not None:
            diagnostics["active_plan_kind"] = "observation_track_target"
            diagnostics["active_plan_waypoints_xy_m"] = [nav_xy.tolist(), self._tracking_target_xy.tolist()]
        if fwhm is not None:
            diagnostics.update(
                {
                    "fwhm_valid": fwhm.valid,
                    "fwhm_peak_x_m": fwhm.peak_x_m,
                    "fwhm_peak_value_nt": fwhm.peak_value_nt,
                    "fwhm_baseline_nt": fwhm.baseline_nt,
                    "fwhm_half_level_nt": fwhm.half_level_nt,
                    "fwhm_left_x_m": fwhm.left_crossing_x_m,
                    "fwhm_right_x_m": fwhm.right_crossing_x_m,
                    "fwhm_width_m": fwhm.width_m,
                    "fwhm_vertical_separation_m": fwhm.vertical_separation_m,
                    "fwhm_vertical_separation_sigma_m": fwhm.vertical_separation_sigma_m,
                    "fwhm_burial_depth_m": fwhm.burial_depth_m,
                    "fwhm_burial_depth_sigma_m": fwhm.burial_depth_sigma_m,
                    "fwhm_validity_score": fwhm.validity_score,
                    "fwhm_confidence": fwhm.confidence,
                    "fwhm_model_residual": fwhm.model_residual,
                    "fwhm_competing_model_residual": fwhm.competing_model_residual,
                    "fwhm_inspection_declared_shape_residual": fwhm.inspection_declared_shape_residual,
                    "fwhm_inspection_competing_shape_residual": fwhm.inspection_competing_shape_residual,
                    "fwhm_shape_support_widths": fwhm.shape_support_widths,
                    "fwhm_prominence_nt": fwhm.prominence_nt,
                    "fwhm_snr_db": fwhm.snr_db,
                    "fwhm_symmetry_score": fwhm.symmetry_score,
                    "fwhm_coverage_score": fwhm.coverage_score,
                    "fwhm_shape_model": fwhm.shape_model,
                    "fwhm_shape_factor": fwhm.shape_factor,
                    "fwhm_shape_intercept_m": fwhm.shape_intercept_m,
                    "fwhm_reason_codes": list(fwhm.reason_codes),
                }
            )

        output = CableTrackingOutput(
            time_s=float(navigation.time_s),
            estimated_cable_xy_m=np.asarray(estimate_xy, dtype=float),
            cross_track_m=float(distance_m),
            route_progress_m=float(progress_m),
            cable_heading_deg=float(estimated_heading_deg),
            burial_depth_m=burial_depth,
            burial_sigma_m=burial_sigma,
            confidence=float(np.clip(confidence, 0.0, 1.0)),
            mode=self.state_machine.phase.value,
            diagnostics=diagnostics,
        )
        self.last_output = output
        return output

    def _active_waypoint(self, nav_xy: np.ndarray) -> Optional[np.ndarray]:
        if (self.state_machine.phase == MissionPhase.SEARCH or (
            self.v2_config.inspection.enabled and self.state_machine.phase == MissionPhase.REACQUIRE
        )) and self._search_plan is not None:
            return self._search_guidance_waypoint(nav_xy)
        if self.state_machine.phase == MissionPhase.CONFIRM and self._scan_plan is not None:
            index = min(self._scan_waypoint_index, len(self._scan_plan.waypoints_xy_m) - 1)
            return self._scan_plan.waypoints_xy_m[index]
        if self.state_machine.phase == MissionPhase.TRACK and self._tracking_target_xy is not None:
            return self._tracking_target_xy
        return None

    def step_with_guidance(
        self,
        navigation: NavigationInput,
        magnetic: MagneticInput,
        sonar: Optional[SonarInput] = None,
        *,
        target_depth_m: Optional[float] = None,
        speed_mps: Optional[float] = None,
    ) -> tuple[CableTrackingOutput, CableGuidanceOutput]:
        output = self.step(navigation, magnetic, sonar)
        if output.mode in {"hold", "complete"}:
            nav = self._last_navigation
            return output, CableGuidanceOutput(
                desired_heading_deg=output.cable_heading_deg,
                target_depth_m=0.0 if nav is None else abs(float(nav.position_ned_m[2])),
                speed_mps=0.0, mode=output.mode,
                guidance_source="v2_endpoint_hold" if output.mode == "complete" else "v2_input_hold",
                safe_lock_active=True,
                diagnostics={"continuity_reason": output.diagnostics["continuity_reason"]},
            )
        nav_xy = np.asarray(navigation.position_ned_m, dtype=float)[:2]
        target = self._active_waypoint(nav_xy)
        if target is not None:
            delta = target - nav_xy
            desired_heading = math.degrees(math.atan2(delta[1], delta[0])) if np.linalg.norm(delta) > 1e-9 else navigation.heading_deg
            if self.state_machine.phase == MissionPhase.CONFIRM:
                resolved_speed = (
                    self.v2_config.cross_scan.velocity_mps
                    if self._scan_waypoint_index != 1 or self._scan_aligned
                    else self.v2_config.cross_scan.alignment_speed_mps
                )
                source = "v2_cross_scan_waypoint"
            elif self.state_machine.phase == MissionPhase.TRACK:
                correction = compute_tracking_correction(
                    float(output.diagnostics["signed_cross_track_m"]),
                    output.confidence,
                    self.v2_config.tracking,
                )
                resolved_speed = 0.0 if self._endpoint_complete else correction.speed_mps
                source = "v2_endpoint_hold" if self._endpoint_complete else "v2_observation_corrected_route"
            else:
                resolved_speed = self.v2_config.search.velocity_mps
                source = "v2_search_waypoint"
            correction_deg = 0.0
        else:
            correction = compute_tracking_correction(
                float(output.diagnostics["signed_cross_track_m"]), output.confidence, self.v2_config.tracking
            )
            desired_heading = output.cable_heading_deg + correction.heading_correction_deg
            resolved_speed = correction.speed_mps
            source = "v2_observation_track" if output.diagnostics["observation_only_available"] else "v2_prior_fallback"
            correction_deg = correction.heading_correction_deg
        try:
            if speed_mps is not None:
                resolved_speed = float(speed_mps)
            depth = abs(float(np.asarray(navigation.position_ned_m, dtype=float)[2])) if target_depth_m is None else float(target_depth_m)
        except (TypeError, ValueError, OverflowError):
            if not self.v2_config.inspection.enabled:
                raise
            resolved_speed, depth = float("nan"), abs(float(navigation.position_ned_m[2]))
        desired_heading = (float(desired_heading) + 180.0) % 360.0 - 180.0
        safe_lock = False
        if self.v2_config.inspection.enabled:
            if not math.isfinite(depth) or depth < 0 or not math.isfinite(resolved_speed):
                depth = abs(float(navigation.position_ned_m[2]))
                resolved_speed, safe_lock, source = 0.0, True, "v2_invalid_command_hold"
            resolved_speed = float(np.clip(resolved_speed, 0.0, self.v2_config.inspection.maximum_guidance_speed_mps))
            turn_error = abs((desired_heading - navigation.heading_deg + 180) % 360 - 180)
            if turn_error > 45:
                resolved_speed = min(resolved_speed, self.v2_config.cross_scan.alignment_speed_mps)
            outside_target = target is not None and self.search_boundary_xy_m is not None and not _point_in_polygon(target, self.search_boundary_xy_m)
            if target is None or outside_target or self._endpoint_complete:
                resolved_speed, safe_lock = 0.0, True
                source = "v2_endpoint_hold" if self._endpoint_complete else "v2_plan_unavailable_hold"
        guidance = CableGuidanceOutput(
            desired_heading_deg=desired_heading,
            target_depth_m=depth,
            speed_mps=float(resolved_speed),
            safe_lock_active=safe_lock,
            mode=output.mode,
            guidance_source=source,
            zigzag_width_m=0.0 if self._scan_plan is None else self._scan_plan.half_span_m,
            diagnostics={
                "algorithm_version": "v2",
                "mission_phase": self.state_machine.phase.value,
                "active_waypoint_xy_m": None if target is None else target.tolist(),
                "heading_correction_deg": correction_deg,
            },
        )
        output.diagnostics["guidance_source"] = source
        return output, guidance
