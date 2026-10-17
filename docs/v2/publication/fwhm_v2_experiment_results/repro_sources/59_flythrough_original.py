"""Opt-in straight, shallow-angle magnetic crossing for a prealigned vehicle.

The search leg itself supplies the two sides of a magnetic profile. This mode
has a deliberately narrow operating envelope: a straight prior, one dominant
anomaly, known staging pose and room for a long run-in. It does not change the
ordinary v2 search or cross-scan planners.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from ..api.types import MagneticInput, NavigationInput, SonarInput
from .pipeline import AuvMagTrackingPipelineV2, _boundary_clearance_m
from .search import _point_in_polygon
from .types import CableObservation, CrossScanPlan, MissionPhase, SearchPlan
from .lateral import qualify_lateral_center


@dataclass(frozen=True)
class FlythroughConfig:
    start_along_m: float = -250.
    end_along_m: float = -80.
    start_lateral_m: float = -15.
    end_lateral_m: float = 15.
    staging_tolerance_m: float = 2.
    heading_tolerance_deg: float = 15.
    maximum_prior_bend_deg: float = 2.
    path_lookahead_m: float = 40.
    current_filter_alpha: float = 0.05
    maximum_estimated_current_mps: float = 0.08
    rejoin_overshoot_m: float = 5.
    side_hint_required: bool = False
    side_hint_min_m: float | None = None
    side_hint_max_m: float | None = None
    side_hint_source: str = ""
    allow_synthetic_side_hint: bool = True
    maximum_side_hint_width_m: float | None = None
    require_runtime_identity_cue: bool = False
    runtime_identity_supersedes_side_hint_center: bool = False
    maximum_runtime_identity_age_s: float = 0.5
    maximum_runtime_identity_gap_s: float = 2.0
    maximum_runtime_identity_confirm_half_width_m: float = 0.5
    maximum_runtime_identity_track_half_width_m: float = 0.6
    runtime_target_anchor_min_m: float | None = None
    runtime_target_anchor_max_m: float | None = None
    runtime_target_exclusive_min_m: float | None = None
    runtime_target_exclusive_max_m: float | None = None
    minimum_halfmax_tail_m: float = 0.
    side_hint_observation_tolerance_m: float = 0.5
    rejoin_runin_m: float = 0.
    rejoin_opposed_runin_m: float | None = None
    rejoin_current_direction_threshold_mps: float = 0.02
    maximum_heading_sigma_deg: float = 2.
    maximum_ground_velocity_sigma_mps: float = 0.03
    route_quality_mean_limit_m: float | None = None
    route_quality_max_limit_m: float | None = None
    route_quality_min_samples: int = 100

    def validate(self) -> None:
        values = (self.start_along_m, self.end_along_m, self.start_lateral_m,
                  self.end_lateral_m, self.staging_tolerance_m,
                  self.heading_tolerance_deg, self.maximum_prior_bend_deg,
                  self.path_lookahead_m, self.current_filter_alpha,
                  self.maximum_estimated_current_mps, self.rejoin_overshoot_m,
                  self.minimum_halfmax_tail_m, self.side_hint_observation_tolerance_m,
                  self.maximum_runtime_identity_age_s,
                  self.maximum_runtime_identity_gap_s,
                  self.maximum_runtime_identity_confirm_half_width_m,
                  self.maximum_runtime_identity_track_half_width_m,
                  self.rejoin_runin_m, self.rejoin_current_direction_threshold_mps,
                  self.maximum_heading_sigma_deg, self.maximum_ground_velocity_sigma_mps)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("flythrough parameters must be finite")
        if not (self.start_along_m < self.end_along_m <= 0.):
            raise ValueError("flythrough staging must precede the route start")
        if not (self.start_lateral_m * self.end_lateral_m < 0.):
            raise ValueError("flythrough must cross the prior from opposite sides")
        if self.staging_tolerance_m <= 0 or not 0 < self.heading_tolerance_deg < 45:
            raise ValueError("invalid flythrough staging tolerances")
        if self.path_lookahead_m <= 0 or not 0 < self.current_filter_alpha <= 1:
            raise ValueError("invalid flythrough path/current filter settings")
        if self.maximum_estimated_current_mps <= 0:
            raise ValueError("maximum estimated current must be positive")
        if self.rejoin_overshoot_m <= 0:
            raise ValueError("invalid flythrough rejoin envelope")
        if self.minimum_halfmax_tail_m < 0:
            raise ValueError("minimum half-maximum tail support must be nonnegative")
        if self.side_hint_observation_tolerance_m < 0:
            raise ValueError("side hint observation tolerance must be nonnegative")
        if self.runtime_identity_supersedes_side_hint_center and not self.require_runtime_identity_cue:
            raise ValueError("runtime identity center authority requires a runtime identity cue")
        if self.maximum_runtime_identity_age_s <= 0:
            raise ValueError("runtime identity age must be positive")
        if self.maximum_runtime_identity_gap_s < self.maximum_runtime_identity_age_s:
            raise ValueError("runtime identity gap must include the maximum cue age")
        if (self.maximum_runtime_identity_confirm_half_width_m <= 0.
                or self.maximum_runtime_identity_track_half_width_m
                < self.maximum_runtime_identity_confirm_half_width_m):
            raise ValueError("runtime identity track width must cover the confirm width")
        anchor = (self.runtime_target_anchor_min_m, self.runtime_target_anchor_max_m)
        if (anchor[0] is None) != (anchor[1] is None):
            raise ValueError("runtime target anchor interval must be complete")
        if anchor[0] is not None and (
                not self.require_runtime_identity_cue
                or any(isinstance(value, bool) or not isinstance(value, (int, float))
                       or not math.isfinite(value) for value in anchor)
                or anchor[0] >= anchor[1]):
            raise ValueError("runtime target anchor requires a valid runtime cue and interval")
        exclusive = (self.runtime_target_exclusive_min_m,
                     self.runtime_target_exclusive_max_m)
        if (exclusive[0] is None) != (exclusive[1] is None):
            raise ValueError("runtime target exclusive corridor must be complete")
        if exclusive[0] is not None and (
                anchor[0] is None
                or any(isinstance(value, bool) or not isinstance(value, (int, float))
                       or not math.isfinite(value) for value in exclusive)
                or not exclusive[0] < anchor[0] < anchor[1] < exclusive[1]):
            raise ValueError("runtime target exclusive corridor must enclose the anchor")
        if (self.maximum_side_hint_width_m is not None and
                (not math.isfinite(self.maximum_side_hint_width_m)
                 or self.maximum_side_hint_width_m <= 0.)):
            raise ValueError("maximum side hint width must be finite and positive")
        if not 0. <= self.rejoin_runin_m < -self.end_along_m:
            raise ValueError("rejoin run-in must fit upstream of the route start")
        if self.rejoin_opposed_runin_m is not None and (
                not math.isfinite(self.rejoin_opposed_runin_m)
                or not 0. <= self.rejoin_opposed_runin_m < -self.end_along_m):
            raise ValueError("opposed rejoin run-in must fit upstream of the route start")
        if self.rejoin_current_direction_threshold_mps < 0.:
            raise ValueError("rejoin current direction threshold must be nonnegative")
        if self.maximum_heading_sigma_deg <= 0. or self.maximum_ground_velocity_sigma_mps <= 0.:
            raise ValueError("navigation uncertainty limits must be positive")
        quality_limits = (self.route_quality_mean_limit_m, self.route_quality_max_limit_m)
        if (quality_limits[0] is None) != (quality_limits[1] is None):
            raise ValueError("route quality limits must be configured together")
        if quality_limits[0] is not None and (
                not all(math.isfinite(value) and value > 0. for value in quality_limits)
                or self.route_quality_min_samples < 1):
            raise ValueError("invalid route quality limits")


class FlythroughInspectionPipelineV2(AuvMagTrackingPipelineV2):
    """Use one oblique transit as Search and Confirm; refuse unstaged starts."""

    def __init__(self, *args, flythrough_config: FlythroughConfig | None = None, **kwargs):
        super().__init__(*args, **kwargs)
        if not self.v2_config.inspection.enabled:
            raise ValueError("flythrough requires the inspection contract")
        self.flythrough_config = flythrough_config or FlythroughConfig()
        self.flythrough_config.validate()
        points = np.asarray(self.cable_map.points_xy_m, dtype=float)
        vectors = np.diff(points, axis=0)
        if len(vectors) == 0 or np.any(np.linalg.norm(vectors, axis=1) <= 1e-9):
            raise ValueError("flythrough requires nonzero straight prior segments")
        tangent = vectors[0] / np.linalg.norm(vectors[0])
        angles = np.degrees(np.arccos(np.clip(vectors @ tangent / np.linalg.norm(vectors, axis=1), -1., 1.)))
        if np.any(angles > self.flythrough_config.maximum_prior_bend_deg):
            raise ValueError("flythrough requires an approximately straight cable prior")
        self._fly_tangent = tangent
        self._fly_normal = np.array([-tangent[1], tangent[0]])
        self._fly_origin = points[0].copy()
        policy = self.flythrough_config
        self._fly_start = self._fly_origin + policy.start_along_m * tangent + policy.start_lateral_m * self._fly_normal
        self._fly_end = self._fly_origin + policy.end_along_m * tangent + policy.end_lateral_m * self._fly_normal
        self._fly_profile: list[tuple[float, float, float]] = []
        self._fly_failed = False
        self._fly_failure_reason = "flythrough_staging_invalid"
        self._fly_previous_nav: tuple[float, np.ndarray] | None = None
        self._fly_current_xy = np.zeros(2, dtype=float)
        self._fly_runin_reached = False
        self._fly_active_runin_m: float | None = None
        self._fly_route_quality_sum_m = 0.
        self._fly_route_quality_max_m = 0.
        self._fly_route_quality_samples = 0
        self._runtime_identity_cue = None
        self._runtime_identity_last_seen_s: float | None = None

    def reset(self) -> None:
        super().reset()
        self._fly_profile.clear()
        self._fly_failed = False
        self._fly_failure_reason = "flythrough_staging_invalid"
        self._fly_previous_nav = None
        self._fly_current_xy[:] = 0.
        self._fly_runin_reached = False
        self._fly_active_runin_m = None
        self._fly_route_quality_sum_m = 0.
        self._fly_route_quality_max_m = 0.
        self._fly_route_quality_samples = 0
        self._runtime_identity_cue = None
        self._runtime_identity_last_seen_s = None

    def set_runtime_identity_cue(self, cue) -> None:
        """Accept a separately qualified, timestamped target-line cue."""
        self._runtime_identity_cue = cue

    def _ensure_search_plan(self, nav_xy: np.ndarray) -> None:
        if self._search_plan is not None:
            return
        side_hint_reason = self._side_hint_problem()
        direction = self._fly_end - self._fly_start
        length = float(np.linalg.norm(direction))
        heading = math.degrees(math.atan2(direction[1], direction[0]))
        nav_heading = self._last_navigation.heading_deg if self._last_navigation is not None else heading
        heading_error = abs((nav_heading - heading + 180.) % 360. - 180.)
        valid = (
            np.linalg.norm(nav_xy - self._fly_start) <= self.flythrough_config.staging_tolerance_m
            and heading_error <= self.flythrough_config.heading_tolerance_deg
            and self.search_boundary_xy_m is not None
            and _point_in_polygon(self._fly_start, self.search_boundary_xy_m)
            and _point_in_polygon(self._fly_end, self.search_boundary_xy_m)
            and side_hint_reason is None
        )
        self._fly_failed = not valid
        if not valid:
            self._fly_failure_reason = side_hint_reason or "flythrough_staging_invalid"
        self._search_plan = SearchPlan(
            waypoints_xy_m=np.array([self._fly_end]), lane_offsets_m=np.array([self.flythrough_config.end_lateral_m]),
            velocity_mps=self.v2_config.search.velocity_mps,
            estimated_distance_m=length, estimated_time_s=length / self.v2_config.search.velocity_mps,
            valid=valid, reason_codes=() if valid else (self._fly_failure_reason,),
        )

    def _side_hint_problem(self) -> str | None:
        policy = self.flythrough_config
        if not policy.side_hint_required:
            return None
        lower, upper = policy.side_hint_min_m, policy.side_hint_max_m
        if lower is None or upper is None or policy.side_hint_source is None or policy.side_hint_source == "":
            return "flythrough_side_hint_unavailable"
        if not isinstance(policy.side_hint_source, str) or not policy.side_hint_source.strip():
            return "flythrough_side_hint_invalid"
        if policy.side_hint_source == "synthetic_test_fixture" and not policy.allow_synthetic_side_hint:
            return "flythrough_side_hint_simulation_only"
        if isinstance(lower, bool) or isinstance(upper, bool):
            return "flythrough_side_hint_invalid"
        try:
            lower, upper = float(lower), float(upper)
        except (TypeError, ValueError):
            return "flythrough_side_hint_invalid"
        if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
            return "flythrough_side_hint_invalid"
        if (policy.maximum_side_hint_width_m is not None
                and upper - lower > policy.maximum_side_hint_width_m):
            return "flythrough_side_hint_too_wide"
        if lower <= 0. <= upper:
            return "flythrough_side_hint_ambiguous"
        if (upper < 0.) != (policy.end_lateral_m < 0.):
            return "flythrough_side_hint_direction_mismatch"
        left, right = sorted((policy.start_lateral_m, policy.end_lateral_m))
        if lower <= left or upper >= right:
            return "flythrough_side_hint_outside_scan"
        if policy.minimum_halfmax_tail_m <= 0.:
            return "flythrough_side_hint_support_unconfigured"
        if (lower - left < policy.minimum_halfmax_tail_m or
                right - upper < policy.minimum_halfmax_tail_m):
            return "flythrough_side_hint_support_insufficient"
        return None

    def _choose_rejoin_runin_m(self) -> float:
        policy = self.flythrough_config
        if policy.rejoin_opposed_runin_m is None:
            return policy.rejoin_runin_m
        offset_m = float(self._observation_prior_offset_m or 0.)
        current_normal_mps = float(np.dot(self._fly_current_xy, self._fly_normal))
        current_toward_cable_mps = current_normal_mps * (1. if offset_m >= 0. else -1.)
        return (policy.rejoin_runin_m
                if current_toward_cable_mps >= policy.rejoin_current_direction_threshold_mps
                else policy.rejoin_opposed_runin_m)

    def _input_problem(self, navigation: NavigationInput, magnetic: MagneticInput) -> str | None:
        reason = super()._input_problem(navigation, magnetic)
        if reason is not None:
            return reason
        if getattr(navigation, "ground_velocity_frame_valid", True) is False:
            return "navigation_velocity_frame_mismatch"
        if navigation.heading_std_deg > self.flythrough_config.maximum_heading_sigma_deg:
            return "navigation_heading_uncertain"
        velocity_sigma = getattr(navigation, "ground_velocity_std_mps", None)
        if velocity_sigma is not None:
            if (not isinstance(velocity_sigma, (int, float, np.integer, np.floating))
                    or not math.isfinite(float(velocity_sigma)) or velocity_sigma < 0.):
                return "navigation_velocity_invalid"
            if velocity_sigma > self.flythrough_config.maximum_ground_velocity_sigma_mps:
                return "navigation_velocity_uncertain"
        return None

    def step(self, navigation: NavigationInput, magnetic: MagneticInput,
             sonar: SonarInput | None = None):
        if self._fly_failed:
            return self._hold_output(self._fly_failure_reason)
        output = super().step(navigation, magnetic, sonar)
        if self._fly_failed:
            if (output.mode == "hold" and
                    output.diagnostics.get("continuity_reason") == self._fly_failure_reason):
                return output
            return self._hold_output(self._fly_failure_reason)
        return output

    def step_with_guidance(self, navigation: NavigationInput, magnetic: MagneticInput,
                           sonar: SonarInput | None = None, *, target_depth_m: float | None = None,
                           speed_mps: float | None = None):
        output, guidance = super().step_with_guidance(
            navigation, magnetic, sonar, target_depth_m=target_depth_m, speed_mps=speed_mps,
        )
        if output.mode not in {"search", "confirm", "track"} or guidance.safe_lock_active:
            return output, guidance
        position = np.asarray(navigation.position_ned_m, dtype=float)[:2]
        if np.isfinite(position).all() and math.isfinite(navigation.heading_deg) and math.isfinite(navigation.speed_mps):
            ground_velocity = getattr(navigation, "ground_velocity_ned_mps", None)
            ground_velocity = None if ground_velocity is None else np.asarray(ground_velocity, dtype=float)
            if ground_velocity is not None and ground_velocity.shape == (3,) and np.isfinite(ground_velocity).all():
                ground_xy = ground_velocity[:2]
                heading_rad = math.radians(navigation.heading_deg)
                forward = navigation.speed_mps * np.array([math.cos(heading_rad), math.sin(heading_rad)])
                disturbance = ground_xy - forward
                alpha = self.flythrough_config.current_filter_alpha
                self._fly_current_xy = (1. - alpha) * self._fly_current_xy + alpha * disturbance
            elif self._fly_previous_nav is not None:
                previous_time, previous_xy = self._fly_previous_nav
                dt = float(navigation.time_s - previous_time)
                if 0.05 <= dt <= 2.:
                    heading_rad = math.radians(navigation.heading_deg)
                    body_forward = navigation.speed_mps * np.array([math.cos(heading_rad), math.sin(heading_rad)])
                    disturbance = (position - previous_xy) / dt - body_forward
                    alpha = self.flythrough_config.current_filter_alpha
                    self._fly_current_xy = (1. - alpha) * self._fly_current_xy + alpha * disturbance
            self._fly_previous_nav = float(navigation.time_s), position.copy()
        # The velocity-minus-heading proxy includes vehicle sway during
        # turning. Its magnitude is an environmental-current bound only on
        # the straight crossing, not while the vehicle rejoins the route.
        if (output.mode in {"search", "confirm"}
                and np.linalg.norm(self._fly_current_xy) > self.flythrough_config.maximum_estimated_current_mps):
            self._fly_failed = True
            self._fly_failure_reason = "flythrough_current_exceeds_envelope"
            output = self._hold_output(self._fly_failure_reason)
            guidance.speed_mps = 0.
            guidance.safe_lock_active = True
            guidance.mode = "hold"
            guidance.guidance_source = "v2_flythrough_current_limit_hold"
            return output, guidance
        if output.mode in {"search", "confirm"} and not guidance.safe_lock_active:
            # Hold a straight ground-track line, not a bearing to the fixed
            # endpoint. The latter becomes singular near the endpoint and
            # reacts too late to lateral current on a slow-turning vehicle.
            direction = self._fly_end - self._fly_start
            direction /= np.linalg.norm(direction)
            normal = np.array([-direction[1], direction[0]])
            cross_track = float(np.dot(position - self._fly_start, normal))
            correction = float(np.clip(
                math.atan2(-cross_track, self.flythrough_config.path_lookahead_m),
                -math.radians(15.), math.radians(15.),
            ))
            desired_ground = np.array([
                math.cos(math.atan2(direction[1], direction[0]) + correction),
                math.sin(math.atan2(direction[1], direction[0]) + correction),
            ])
            desired_throughwater = max(guidance.speed_mps, .1) * desired_ground - self._fly_current_xy
            guidance.desired_heading_deg = (math.degrees(math.atan2(
                desired_throughwater[1], desired_throughwater[0])) + 180.) % 360. - 180.
            guidance.guidance_source = "v2_flythrough_disturbance_adaptive_line"
            guidance.diagnostics["flythrough_path_cross_track_m"] = cross_track
            output.diagnostics["guidance_source"] = guidance.guidance_source
        if output.mode == "track" and not self._inspection_started and not guidance.safe_lock_active:
            start_target = self._fly_origin + (self._observation_prior_offset_m or 0.) * self._fly_normal
            if self._fly_active_runin_m is None:
                self._fly_active_runin_m = self._choose_rejoin_runin_m()
            runin_m = self._fly_active_runin_m
            if runin_m > 0.:
                runin_target = start_target - runin_m * self._fly_tangent
                margin_m = self.v2_config.inspection.boundary_stop_margin_m
                if (self.search_boundary_xy_m is None
                        or not _point_in_polygon(runin_target, self.search_boundary_xy_m)
                        or _boundary_clearance_m(runin_target, self.search_boundary_xy_m) < margin_m):
                    self._fly_failed = True
                    self._fly_failure_reason = "flythrough_runin_boundary_insufficient"
                    output = self._hold_output(self._fly_failure_reason)
                    guidance.speed_mps = 0.
                    guidance.safe_lock_active = True
                    guidance.mode = "hold"
                    guidance.guidance_source = "v2_flythrough_runin_boundary_hold"
                    return output, guidance
                approach = runin_target - self._fly_end
                approach /= np.linalg.norm(approach)
                if (np.linalg.norm(position - runin_target) <= 2.
                        or np.dot(position - runin_target, approach) >= 0.):
                    self._fly_runin_reached = True
                line_start = runin_target if self._fly_runin_reached else self._fly_end
                direction = self._fly_tangent if self._fly_runin_reached else approach
                lookahead_m = (0.5 * self.flythrough_config.path_lookahead_m
                               if self._fly_runin_reached else self.flythrough_config.path_lookahead_m)
                overshoot_m = float(np.dot(position - start_target, self._fly_tangent))
            else:
                line_start = self._fly_end
                direction = start_target - self._fly_end
                direction /= np.linalg.norm(direction)
                lookahead_m = self.flythrough_config.path_lookahead_m
                overshoot_m = float(np.dot(position - start_target, direction))
            if overshoot_m > self.flythrough_config.rejoin_overshoot_m:
                self._fly_failed = True
                self._fly_failure_reason = "flythrough_rejoin_overshoot"
                output = self._hold_output(self._fly_failure_reason)
                guidance.speed_mps = 0.
                guidance.safe_lock_active = True
                guidance.mode = "hold"
                guidance.guidance_source = "v2_flythrough_rejoin_overshoot_hold"
                return output, guidance
            normal = np.array([-direction[1], direction[0]])
            cross_track = float(np.dot(position - line_start, normal))
            correction = float(np.clip(
                math.atan2(-cross_track, lookahead_m),
                -math.radians(15.), math.radians(15.),
            ))
            desired_ground_heading = math.atan2(direction[1], direction[0]) + correction
            desired_ground = np.array([math.cos(desired_ground_heading), math.sin(desired_ground_heading)])
            desired_throughwater = max(guidance.speed_mps, .1) * desired_ground - self._fly_current_xy
            guidance.desired_heading_deg = (math.degrees(math.atan2(
                desired_throughwater[1], desired_throughwater[0])) + 180.) % 360. - 180.
            guidance.guidance_source = "v2_flythrough_disturbance_adaptive_rejoin"
            guidance.diagnostics["flythrough_rejoin_cross_track_m"] = cross_track
            guidance.diagnostics["flythrough_rejoin_runin_m"] = runin_m
            output.diagnostics["guidance_source"] = guidance.guidance_source
        if output.mode == "track" and self._inspection_started and not self._endpoint_complete:
            # A perpendicular rejoin target creates a 90-degree command for
            # metre-scale lateral errors. Native PVS cannot follow it at the
            # inspection speed; use a bounded route-tangent correction.
            point = np.asarray(navigation.position_ned_m, dtype=float)[:2]
            signed_error = float(np.dot(point - output.estimated_cable_xy_m, self._fly_normal))
            correction_deg = float(np.clip(math.degrees(math.atan2(-signed_error, 20.)), -10., 10.))
            route_heading = math.degrees(math.atan2(self._fly_tangent[1], self._fly_tangent[0]))
            desired_ground_heading = math.radians(route_heading + correction_deg)
            desired_ground = np.array([math.cos(desired_ground_heading), math.sin(desired_ground_heading)])
            desired_throughwater = max(guidance.speed_mps, .1) * desired_ground - self._fly_current_xy
            guidance.desired_heading_deg = (math.degrees(math.atan2(
                desired_throughwater[1], desired_throughwater[0])) + 180.) % 360. - 180.
            guidance.guidance_source = "v2_flythrough_bounded_track"
            guidance.diagnostics["heading_correction_deg"] = correction_deg
            output.diagnostics["guidance_source"] = guidance.guidance_source
        output.diagnostics["flythrough_estimated_current_xy_mps"] = self._fly_current_xy.tolist()
        output.diagnostics["flythrough_lateral_disturbance_proxy_xy_mps"] = self._fly_current_xy.tolist()
        return output, guidance

    def _step_impl(self, navigation: NavigationInput, magnetic: MagneticInput,
                   sonar: SonarInput | None = None):
        if self.state_machine.phase == MissionPhase.SEARCH:
            pos = np.asarray(navigation.position_ned_m, dtype=float)[:2]
            envelope = self._envelope_nt(magnetic)
            if np.isfinite(pos).all() and math.isfinite(envelope):
                self._fly_profile.append((float(np.dot(pos - self._fly_origin, self._fly_normal)),
                                          envelope, float(navigation.time_s)))
        output = super()._step_impl(navigation, magnetic, sonar)
        if output.mode == "track" and self.flythrough_config.require_runtime_identity_cue:
            identity_reason = self._track_identity_problem(navigation, output)
            if identity_reason is not None:
                self._fly_failed = True
                self._fly_failure_reason = identity_reason
                held = self._hold_output(identity_reason)
                held.diagnostics["runtime_identity_last_seen_s"] = self._runtime_identity_last_seen_s
                return held
            if self.flythrough_config.runtime_identity_supersedes_side_hint_center:
                output.diagnostics["target_identity_authority"] = (
                    "runtime_sonar_with_independent_anchor"
                    if self.flythrough_config.runtime_target_anchor_min_m is not None
                    else "runtime_identity_cue"
                )
        if output.mode == "track" and self._inspection_started:
            nav_xy = np.asarray(navigation.position_ned_m, dtype=float)[:2]
            start_xy = self._fly_origin + float(self._observation_prior_offset_m or 0.) * self._fly_normal
            along_m = float(np.dot(nav_xy - start_xy, self._fly_tangent))
            route_length_m = float(self._cache.cumulative_length_m[-1])
            if 0. <= along_m <= route_length_m + self.v2_config.tracking.endpoint_tolerance_m:
                observed_error_m = abs(float(np.dot(
                    nav_xy - output.estimated_cable_xy_m, self._fly_normal)))
                self._fly_route_quality_sum_m += observed_error_m
                self._fly_route_quality_max_m = max(self._fly_route_quality_max_m, observed_error_m)
                self._fly_route_quality_samples += 1
            count = self._fly_route_quality_samples
            mean_m = self._fly_route_quality_sum_m / count if count else None
            output.diagnostics["route_observed_lateral_mae_m"] = mean_m
            output.diagnostics["route_observed_lateral_max_m"] = self._fly_route_quality_max_m if count else None
            output.diagnostics["route_observed_lateral_samples"] = count
            policy = self.flythrough_config
            if output.diagnostics.get("endpoint_complete") and policy.route_quality_mean_limit_m is not None:
                quality_valid = (count >= policy.route_quality_min_samples
                                 and mean_m <= policy.route_quality_mean_limit_m
                                 and self._fly_route_quality_max_m <= policy.route_quality_max_limit_m)
                output.diagnostics["route_quality_valid"] = quality_valid
                if not quality_valid:
                    self._fly_failed = True
                    self._fly_failure_reason = "flythrough_route_quality_rejected"
                    held = self._hold_output(self._fly_failure_reason)
                    held.diagnostics.update({
                        "route_observed_lateral_mae_m": mean_m,
                        "route_observed_lateral_max_m": self._fly_route_quality_max_m,
                        "route_observed_lateral_samples": count,
                        "route_quality_valid": False,
                    })
                    return held
        if self.state_machine.phase == MissionPhase.SEARCH and self._search_plan is not None and self._search_plan.valid:
            direction = self._fly_end - self._fly_start
            if np.dot(np.asarray(navigation.position_ned_m)[:2] - self._fly_end, direction) >= 0:
                self._fly_failed = True
                self._fly_failure_reason = "flythrough_no_detection"
                self._continuity_reason = self._fly_failure_reason
        return output

    def _track_identity_problem(self, navigation: NavigationInput, output) -> str | None:
        """Keep target association bounded after the initial Confirm permit."""
        cue = self._runtime_identity_cue
        if cue is not None:
            try:
                age_s = float(navigation.time_s) - float(cue.time_s)
                center_m = float(cue.lateral_center_m)
                half_width_m = float(cue.half_width_m)
                valid = (all(math.isfinite(value) for value in (age_s, center_m, half_width_m))
                         and 0. <= age_s <= self.flythrough_config.maximum_runtime_identity_age_s
                         and 0. <= half_width_m
                         <= self.flythrough_config.maximum_runtime_identity_track_half_width_m)
            except (AttributeError, TypeError, ValueError):
                valid = False
            if valid:
                predicted_m = float(np.dot(
                    np.asarray(output.estimated_cable_xy_m, dtype=float) - self._fly_origin,
                    self._fly_normal,
                ))
                if not math.isfinite(predicted_m):
                    return "flythrough_runtime_identity_prediction_invalid"
                if abs(predicted_m - center_m) > half_width_m:
                    return "flythrough_runtime_identity_diverged"
                if (self.flythrough_config.runtime_target_exclusive_min_m is not None
                        and (center_m - half_width_m
                             < self.flythrough_config.runtime_target_exclusive_min_m
                             or center_m + half_width_m
                             > self.flythrough_config.runtime_target_exclusive_max_m)):
                    return "flythrough_runtime_identity_exclusive_corridor_inconsistent"
                self._runtime_identity_last_seen_s = float(cue.time_s)
                output.diagnostics["runtime_identity_track_residual_m"] = abs(predicted_m - center_m)
        if (self._runtime_identity_last_seen_s is None
                or float(navigation.time_s) - self._runtime_identity_last_seen_s
                > self.flythrough_config.maximum_runtime_identity_gap_s):
            return "flythrough_runtime_identity_lost"
        output.diagnostics["runtime_identity_last_seen_s"] = self._runtime_identity_last_seen_s
        return None

    def _start_scan(self, nav_xy: np.ndarray, point_xy: np.ndarray, tangent_xy: np.ndarray) -> None:
        self._scan_started_s = self._last_step_s or 0.
        self._last_lateral = self._last_fwhm = None
        self._scan_id += 1
        self._scan_plan = CrossScanPlan(
            scan_id=self._scan_id, center_xy_m=self._fly_origin.copy(),
            tangent_xy=self._fly_tangent.copy(), normal_xy=self._fly_normal.copy(),
            waypoints_xy_m=np.array([self._fly_end]),
            half_span_m=max(abs(self.flythrough_config.start_lateral_m), self.flythrough_config.end_lateral_m),
            velocity_mps=self.v2_config.cross_scan.velocity_mps,
        )
        self._scan_waypoint_index = 0
        self._scan_x_m = [row[0] for row in self._fly_profile[:-1]]
        self._scan_envelope_nt = [row[1] for row in self._fly_profile[:-1]]
        self._scan_noise_nt = []
        self._scan_along_track_m = []
        self._scan_aligned = True
        self._scan_completion_reason = "not_complete"

    def _update_scan(self, navigation: NavigationInput, nav_xy: np.ndarray,
                     envelope_nt: float, detection, prior_heading_deg: float,
                     altitude_m: float | None) -> None:
        plan = self._scan_plan
        if plan is None or not plan.valid:
            self._fly_failed = True
            self._fly_failure_reason = "flythrough_plan_invalid"
            self._continuity_reason = self._fly_failure_reason
            return
        if navigation.time_s - self._scan_started_s > self.v2_config.inspection.maximum_scan_duration_s:
            self._fly_failed = True
            self._fly_failure_reason = "flythrough_scan_timeout"
            self._continuity_reason = self._fly_failure_reason
            return
        if math.isfinite(envelope_nt):
            self._scan_x_m.append(float(np.dot(nav_xy - self._fly_origin, self._fly_normal)))
            self._scan_envelope_nt.append(envelope_nt)
            self._scan_noise_nt.append(detection.noise_nt)
        direction = self._fly_end - self._fly_start
        passed = np.dot(nav_xy - self._fly_end, direction) >= 0
        reached = np.linalg.norm(nav_xy - self._fly_end) <= self.v2_config.cross_scan.arrival_tolerance_m
        if not (passed or reached):
            return
        self._last_fwhm = self._estimate_current_scan(altitude_m)
        self._scan_measurement_time_s = float(navigation.time_s)
        positions = np.unique(np.asarray(self._scan_x_m))
        if len(positions) > 1 and np.max(np.diff(positions)) > self.v2_config.inspection.maximum_scan_gap_m:
            from dataclasses import replace
            self._last_fwhm = replace(self._last_fwhm, valid=False, confidence=0.,
                                      vertical_separation_m=None, burial_depth_m=None,
                                      reason_codes=(*self._last_fwhm.reason_codes, "scan_spatial_gap"))
        self._last_lateral = qualify_lateral_center(self._last_fwhm, self.v2_config.fwhm, self.v2_config.inspection)
        if (self._last_lateral.valid and self.flythrough_config.side_hint_required
                and not self.flythrough_config.runtime_identity_supersedes_side_hint_center):
            lower = float(self.flythrough_config.side_hint_min_m)
            upper = float(self.flythrough_config.side_hint_max_m)
            tolerance = self.flythrough_config.side_hint_observation_tolerance_m
            center_x = float(self._last_lateral.center_x_m)
            if not lower - tolerance <= center_x <= upper + tolerance:
                from dataclasses import replace
                self._last_lateral = replace(
                    self._last_lateral, valid=False, confidence=0.,
                    reason_codes=(*self._last_lateral.reason_codes,
                                  "side_hint_observation_inconsistent"),
                )
        if self._last_lateral.valid and self.flythrough_config.require_runtime_identity_cue:
            from dataclasses import replace
            cue = self._runtime_identity_cue
            reason = None
            if cue is None:
                reason = "runtime_identity_cue_unavailable"
            else:
                try:
                    age_s = float(navigation.time_s) - float(cue.time_s)
                    center_m = float(cue.lateral_center_m)
                    half_width_m = float(cue.half_width_m)
                    if (not all(math.isfinite(v) for v in (age_s, center_m, half_width_m))
                            or not 0. <= age_s <= self.flythrough_config.maximum_runtime_identity_age_s
                            or half_width_m < 0.):
                        reason = "runtime_identity_cue_stale_or_invalid"
                    elif half_width_m > self.flythrough_config.maximum_runtime_identity_confirm_half_width_m:
                        reason = "runtime_identity_cue_uncertain"
                    elif abs(float(self._last_lateral.center_x_m) - center_m) > half_width_m:
                        reason = "runtime_identity_cue_inconsistent"
                    elif (self.flythrough_config.runtime_target_anchor_min_m is not None
                          and (center_m + half_width_m
                               < self.flythrough_config.runtime_target_anchor_min_m
                               or center_m - half_width_m
                               > self.flythrough_config.runtime_target_anchor_max_m)):
                        reason = "runtime_target_anchor_inconsistent"
                    elif (self.flythrough_config.runtime_target_exclusive_min_m is not None
                          and (center_m - half_width_m
                               < self.flythrough_config.runtime_target_exclusive_min_m
                               or center_m + half_width_m
                               > self.flythrough_config.runtime_target_exclusive_max_m)):
                        reason = "runtime_target_exclusive_corridor_inconsistent"
                except (AttributeError, TypeError, ValueError):
                    reason = "runtime_identity_cue_stale_or_invalid"
            if reason is not None:
                self._last_lateral = replace(
                    self._last_lateral, valid=False, confidence=0.,
                    reason_codes=(*self._last_lateral.reason_codes, reason),
                )
            else:
                self._runtime_identity_last_seen_s = float(cue.time_s)
        self._scan_completion_reason = "flythrough_endpoint_reached"
        self._scan_waypoint_index = 1
        if not self._last_lateral.valid:
            self.state_machine.update(detected=detection.detected, scan_complete=True, fwhm_valid=False)
            self._fly_failed = True
            self._fly_failure_reason = "flythrough_lateral_rejected"
            self._continuity_reason = self._fly_failure_reason
            return
        center = self._fly_origin + self._last_lateral.center_x_m * self._fly_normal
        observation = CableObservation(
            time_s=float(navigation.time_s), position_xy_m=center,
            heading_deg=prior_heading_deg, lateral_sigma_m=self._last_lateral.sigma_m,
            confidence=self._last_lateral.confidence, source="flythrough_magnetic_center",
        )
        self.line_estimator.add(observation)
        self._last_observation = observation
        self._continuity_reason = "flythrough_lateral_confirmed"
        self.state_machine.update(detected=detection.detected, scan_complete=True,
                                  fwhm_valid=bool(self._last_fwhm.valid), lateral_valid=True)

    def _active_waypoint(self, nav_xy: np.ndarray):
        if self.state_machine.phase == MissionPhase.CONFIRM:
            return self._fly_end
        return super()._active_waypoint(nav_xy)
