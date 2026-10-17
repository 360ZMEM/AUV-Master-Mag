"""Opt-in, simulation-gated multi-crossing inspection workflow.

The geometry is magnetic; target identity and current direction are separate
timestamped inputs.  This class intentionally never publishes burial depth.
It shares the v2 public input/output contract without changing the v1 or
single-flythrough state machines.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math

import numpy as np

from ..api.types import CableGuidanceOutput, CableTrackingOutput, MagneticInput, NavigationInput, SonarInput
from .corridor_survey import (
    CorridorSurveyConfig, ShallowZigzagConfig, SurveyProfile,
    build_shallow_zigzag_survey, estimate_survey_line, select_shallow_start_side,
)
from .pipeline import AuvMagTrackingPipelineV2, _boundary_clearance_m, _finite_diagnostics
from .search import _point_in_polygon


@dataclass(frozen=True)
class SurveyCurrentEvidence:
    time_s: float
    velocity_xy_mps: tuple[float, float]
    sigma_mps: float
    source: str
    frame: str = "world"


@dataclass(frozen=True)
class ShallowSurveyMissionConfig:
    half_wavelength_m: float = 135.
    waypoint_spacing_m: float = .5
    three_station_amplitude_m: float = 24.
    four_station_amplitude_m: float = 27.
    velocity_mps: float = .3
    staging_tolerance_m: float = 2.
    maximum_current_age_s: float = 2.
    maximum_identity_age_s: float = .5
    maximum_identity_gap_s: float = 2.
    maximum_identity_half_width_m: float = .5
    maximum_identity_residual_m: float = .5
    maximum_crosscurrent_mps: float = .05
    current_sigma_mps: float = .005
    lookahead_m: float = 25.
    maximum_heading_correction_deg: float = 10.
    rejoin_limit_m: float = 120.
    rejoin_tolerance_m: float = 1.
    track_length_m: float = 80.
    maximum_line_extrapolation_m: float = 220.
    maximum_reference_deviation_m: float = 3.
    maximum_survey_duration_s: float = 2300.
    simulation_noise_floor_nt: float = .02
    target_side_deadband_m: float = .5
    simulated_evidence_only: bool = True

    def validate(self) -> None:
        values = [getattr(self, name) for name in self.__dataclass_fields__
                  if name != "simulated_evidence_only"]
        if not all(isinstance(value, (int, float)) and math.isfinite(value) and value > 0
                   for value in values):
            raise ValueError("shallow survey limits must be finite and positive")
        if self.maximum_identity_gap_s < self.maximum_identity_age_s:
            raise ValueError("identity gap must include cue age")
        if self.maximum_heading_correction_deg >= 45.:
            raise ValueError("survey heading correction must remain below 45 degrees")
        if not self.simulated_evidence_only:
            raise ValueError("shallow survey has no approved physical evidence contract")


class ShallowSurveyInspectionPipelineV2(AuvMagTrackingPipelineV2):
    """Survey -> qualified magnetic line -> identity -> rejoin -> finite Track.

    The caller must supply independent current and target-line cues.  A cue
    made from the magnetic estimate itself would defeat the identity check.
    """

    def __init__(self, *args, survey_config: ShallowSurveyMissionConfig | None = None, **kwargs):
        super().__init__(*args, **kwargs)
        if not self.v2_config.inspection.enabled or self.search_boundary_xy_m is None:
            raise ValueError("shallow survey requires inspection and a search boundary")
        self.survey_config = survey_config or ShallowSurveyMissionConfig()
        self.survey_config.validate()
        self._survey_origin = np.asarray(self.cable_map.points_xy_m[0], dtype=float).copy()
        chord = np.asarray(self.cable_map.points_xy_m[-1], dtype=float) - self._survey_origin
        length = float(np.linalg.norm(chord))
        if length <= 1e-9:
            raise ValueError("shallow survey needs a nonzero prior")
        self._survey_tangent = chord / length
        self._survey_normal = np.array([-self._survey_tangent[1], self._survey_tangent[0]])
        self._survey_current_evidence: SurveyCurrentEvidence | None = None
        self._runtime_identity_cue = None
        self._survey_plan = None
        self._survey_policy = None
        self._survey_quality = None
        self._survey_samples: list[list[tuple[np.ndarray, float]]] = []
        self._survey_start_s: float | None = None
        self._survey_estimate = None
        self._survey_line_offset_m: float | None = None
        self._survey_line_slope: float | None = None
        self._survey_last_center_s: float | None = None
        self._survey_track_start_s: float | None = None
        self._survey_identity_last_seen_s: float | None = None
        self._survey_phase = "preflight"
        self._survey_failed = False
        self._survey_failure_reason = "survey_preflight_unavailable"

    def set_current_evidence(self, evidence: SurveyCurrentEvidence | None) -> None:
        self._survey_current_evidence = evidence

    def set_runtime_identity_cue(self, cue) -> None:
        self._runtime_identity_cue = cue

    def reset(self) -> None:
        super().reset()
        self._survey_current_evidence = None
        self._runtime_identity_cue = None
        self._survey_plan = self._survey_policy = self._survey_quality = None
        self._survey_samples = []
        self._survey_start_s = self._survey_estimate = None
        self._survey_line_offset_m = self._survey_line_slope = None
        self._survey_last_center_s = self._survey_track_start_s = None
        self._survey_identity_last_seen_s = None
        self._survey_phase = "preflight"
        self._survey_failed = False
        self._survey_failure_reason = "survey_preflight_unavailable"

    def export_state(self) -> dict[str, object]:
        return {**super().export_state(), "mission_phase": self._survey_phase,
                "survey_phase": self._survey_phase,
                "survey_failed": self._survey_failed,
                "survey_failure_reason": self._survey_failure_reason}

    def _fail(self, reason: str) -> CableTrackingOutput:
        self._survey_failed = True
        self._survey_failure_reason = reason
        self._survey_phase = "hold"
        return self._hold_output(reason)

    def _cue_lateral(self, time_s: float) -> tuple[float, float] | None:
        cue = self._runtime_identity_cue
        if cue is None:
            return None
        try:
            age = time_s - float(cue.time_s)
            lateral = float(cue.lateral_center_m)
            width = float(cue.half_width_m)
            if (not all(math.isfinite(value) for value in (age, lateral, width))
                    or not 0 <= age <= self.survey_config.maximum_identity_age_s
                    or not 0 <= width <= self.survey_config.maximum_identity_half_width_m
                    or not str(cue.source).strip() or not str(cue.calibration_id).strip()):
                return None
            return lateral, width
        except (AttributeError, TypeError, ValueError, OverflowError):
            return None

    def _line_lateral(self, along_m: float) -> float:
        return float(self._survey_line_offset_m + self._survey_line_slope * along_m)

    def _make_plan(self, navigation: NavigationInput, nav_xy: np.ndarray) -> str | None:
        policy = self.survey_config
        current = self._survey_current_evidence
        if current is None:
            return "survey_current_evidence_unavailable"
        try:
            age = navigation.time_s - float(current.time_s)
            velocity = np.asarray(current.velocity_xy_mps, dtype=float)
            sigma = float(current.sigma_mps)
            if (not 0. <= age <= policy.maximum_current_age_s
                    or velocity.shape != (2,) or not np.isfinite(velocity).all()
                    or not math.isfinite(sigma) or sigma < 0.
                    or current.frame != "world"
                    or not str(current.source).strip()):
                return "survey_current_evidence_invalid_or_stale"
        except (TypeError, ValueError, AttributeError, OverflowError):
            return "survey_current_evidence_invalid_or_stale"
        decision = select_shallow_start_side(
            self._survey_tangent, velocity, sigma,
            maximum_crosscurrent_mps=policy.maximum_crosscurrent_mps,
        )
        if decision.initial_lateral_sign is None:
            return decision.reason_codes[0]
        cue = self._cue_lateral(navigation.time_s)
        if cue is None:
            return "survey_target_side_unavailable"
        center, half_width = cue
        if abs(center) <= half_width + policy.target_side_deadband_m:
            return "survey_target_side_ambiguous"
        target_side = 1 if center > 0 else -1
        start_side = decision.initial_lateral_sign
        count = 3 if target_side != start_side else 4
        amplitude = (policy.three_station_amplitude_m if count == 3
                     else policy.four_station_amplitude_m)
        recipe = ShallowZigzagConfig(
            amplitude_m=amplitude, half_wavelength_m=policy.half_wavelength_m,
            station_count=count, waypoint_spacing_m=policy.waypoint_spacing_m,
            velocity_mps=policy.velocity_mps,
            # The observed synthetic line fit can differ by ~0.002 degrees
            # from the nominal 1.15-degree slope. 1.16 remains inside the
            # analytic A/L coverage test for both 3- and 4-station recipes.
            prior_heading_error_bound_deg=1.16,
            initial_lateral_sign=start_side,
        )
        if abs(center) + half_width > recipe.prior_offset_bound_m + 1e-9:
            return "survey_target_outside_prior_envelope"
        try:
            plan = build_shallow_zigzag_survey(
                self.cable_map.points_xy_m, self.search_boundary_xy_m, recipe)
        except ValueError:
            return "survey_plan_outside_declared_envelope"
        start = plan.waypoints_xy_m[0]
        heading = math.degrees(math.atan2(self._survey_tangent[1], self._survey_tangent[0]))
        heading_error = abs((navigation.heading_deg - heading + 180.) % 360. - 180.)
        if (np.linalg.norm(nav_xy - start) > policy.staging_tolerance_m
                or heading_error > 15.):
            return "survey_staging_invalid"
        self._survey_plan = plan
        self._survey_policy = recipe
        self._survey_quality = CorridorSurveyConfig(
            half_width_m=amplitude, station_spacing_m=policy.half_wavelength_m,
            station_count=count, maximum_along_excursion_m=policy.half_wavelength_m + 2.,
            maximum_heading_deviation_deg=recipe.prior_heading_error_bound_deg,
        )
        self._survey_samples = [[] for _ in range(count)]
        self._survey_start_s = float(navigation.time_s)
        self._survey_identity_last_seen_s = float(self._runtime_identity_cue.time_s)
        self._survey_phase = "survey"
        return None

    def declared_route_envelope_points_xy_m(self) -> np.ndarray | None:
        """Vertices for a convex-boundary Hold budget, including future Track.

        The line estimate is later checked against the same origin-offset and
        heading bounds. This is a necessary route-envelope admission, not a
        real-vehicle turn or coast proof.
        """
        if self._survey_plan is None or self._survey_policy is None:
            return None
        survey_end = self._survey_policy.station_count * self.survey_config.half_wavelength_m
        final_s = survey_end + self.survey_config.rejoin_limit_m + self.survey_config.track_length_m
        lateral_bound = (self._survey_policy.prior_offset_bound_m
                         + math.tan(math.radians(self._survey_policy.prior_heading_error_bound_deg)) * final_s)
        end_points = np.array([
            self._survey_origin + final_s * self._survey_tangent + sign * lateral_bound * self._survey_normal
            for sign in (-1., 1.)])
        return np.vstack((self._survey_plan.waypoints_xy_m, end_points))

    def step(self, navigation: NavigationInput, magnetic: MagneticInput,
             sonar: SonarInput | None = None) -> CableTrackingOutput:
        if self._survey_failed:
            return self._hold_output(self._survey_failure_reason)
        problem = self._input_problem(navigation, magnetic)
        if problem is not None:
            return self._fail(problem)
        self._last_step_s = float(navigation.time_s)
        self._last_navigation = replace(navigation, position_ned_m=np.asarray(navigation.position_ned_m).copy())
        if self._mission_complete:
            output = self._hold_output("mission_complete")
            output.mode = "complete"
            output.diagnostics.update(mission_phase="complete", endpoint_complete=True)
            return output
        nav_xy = np.asarray(navigation.position_ned_m, dtype=float)[:2]
        along = float(np.dot(nav_xy - self._survey_origin, self._survey_tangent))
        if self._survey_plan is None:
            reason = self._make_plan(navigation, nav_xy)
            if reason is not None:
                return self._fail(reason)
        policy = self.survey_config
        survey_end_s = self._survey_policy.station_count * policy.half_wavelength_m
        envelope = self._envelope_nt(magnetic)
        if not math.isfinite(envelope):
            return self._fail("survey_magnetic_envelope_invalid")
        if self._survey_phase == "survey":
            if navigation.time_s - self._survey_start_s > policy.maximum_survey_duration_s:
                return self._fail("survey_duration_exhausted")
            reference_lateral = float(np.interp(
                np.clip(along, 0., survey_end_s),
                (self._survey_plan.waypoints_xy_m - self._survey_origin) @ self._survey_tangent,
                (self._survey_plan.waypoints_xy_m - self._survey_origin) @ self._survey_normal,
            ))
            actual_lateral = float(np.dot(nav_xy - self._survey_origin, self._survey_normal))
            if abs(actual_lateral - reference_lateral) > policy.maximum_reference_deviation_m:
                return self._fail("survey_reference_deviation_exceeded")
            if 0. <= along <= survey_end_s:
                station = min(int(along // policy.half_wavelength_m),
                              self._survey_policy.station_count - 1)
                self._survey_samples[station].append((nav_xy.copy(), envelope))
            if along >= survey_end_s:
                profiles = [SurveyProfile(i,
                                          np.asarray([point for point, _ in samples], dtype=float).reshape(-1, 2),
                                          np.asarray([value for _, value in samples], dtype=float),
                                          policy.simulation_noise_floor_nt)
                            for i, samples in enumerate(self._survey_samples)]
                estimate = estimate_survey_line(
                    self._survey_plan, profiles, self._survey_quality,
                    self.v2_config.fwhm, self.v2_config.inspection)
                self._survey_estimate = estimate
                if not estimate.geometry_valid:
                    return self._fail("survey_geometry_rejected:" + ",".join(estimate.reason_codes))
                first, last = estimate.line_start_xy_m, estimate.line_end_xy_m
                s0 = float(np.dot(first - self._survey_origin, self._survey_tangent))
                s1 = float(np.dot(last - self._survey_origin, self._survey_tangent))
                if s1 - s0 <= 1e-6:
                    return self._fail("survey_line_progress_invalid")
                l0 = float(np.dot(first - self._survey_origin, self._survey_normal))
                l1 = float(np.dot(last - self._survey_origin, self._survey_normal))
                self._survey_line_slope = (l1 - l0) / (s1 - s0)
                self._survey_line_offset_m = l0 - self._survey_line_slope * s0
                if abs(self._survey_line_offset_m) > self._survey_policy.prior_offset_bound_m:
                    return self._fail("survey_line_offset_outside_prior_envelope")
                self._survey_last_center_s = float(np.max(
                    (estimate.station_centers_xy_m - self._survey_origin) @ self._survey_tangent))
                self._survey_phase = "rejoin"
        if self._survey_phase in {"rejoin", "track"}:
            cue = self._cue_lateral(navigation.time_s)
            if cue is not None:
                residual = abs(self._line_lateral(along) - cue[0])
                if residual > min(cue[1], policy.maximum_identity_residual_m):
                    return self._fail("survey_identity_rejected")
                self._survey_identity_last_seen_s = float(self._runtime_identity_cue.time_s)
            if (self._survey_identity_last_seen_s is None
                    or navigation.time_s - self._survey_identity_last_seen_s > policy.maximum_identity_gap_s):
                return self._fail("survey_identity_lost")
            if along - self._survey_last_center_s > policy.maximum_line_extrapolation_m:
                return self._fail("survey_line_extrapolation_exhausted")
            lateral_error = abs(float(np.dot(nav_xy - self._survey_origin, self._survey_normal))
                                - self._line_lateral(along))
            if self._survey_phase == "rejoin":
                if along >= survey_end_s + policy.rejoin_limit_m:
                    return self._fail("survey_rejoin_length_exhausted")
                if along >= survey_end_s + 20. and lateral_error <= policy.rejoin_tolerance_m:
                    self._survey_phase = "track"
                    self._survey_track_start_s = along
            elif along >= self._survey_track_start_s + policy.track_length_m:
                self._survey_phase = "complete"
                self._mission_complete = True
        line_lateral = (float(np.dot(nav_xy - self._survey_origin, self._survey_normal))
                        if self._survey_line_offset_m is None else self._line_lateral(along))
        line_xy = self._survey_origin + along * self._survey_tangent + line_lateral * self._survey_normal
        heading = (math.degrees(math.atan2(self._survey_tangent[1], self._survey_tangent[0]))
                   if self._survey_line_slope is None else
                   math.degrees(math.atan2(
                       (self._survey_tangent + self._survey_line_slope * self._survey_normal)[1],
                       (self._survey_tangent + self._survey_line_slope * self._survey_normal)[0])))
        mode = ("search" if self._survey_phase == "survey" else
                "complete" if self._survey_phase == "complete" else "track")
        diagnostics = {
            "algorithm_version": "v2", "mission_geometry": "shallow_survey",
            "mission_phase": self._survey_phase, "tracking_stage": self._survey_phase,
            "continuity_reason": self._survey_phase, "source": (
                "magnetic_survey_line" if self._survey_estimate is not None else "prior_projection"),
            "lateral_valid": self._survey_phase in {"rejoin", "track", "complete"},
            "depth_valid": False, "burial_depth_m": None,
            "inspection_enabled": True, "observation_only_available": self._survey_estimate is not None,
            "signed_cross_track_m": float(np.dot(nav_xy - line_xy, self._survey_normal)),
            "survey_station_count": self._survey_policy.station_count,
            "survey_profile_sample_counts": [len(samples) for samples in self._survey_samples],
            "survey_geometry_valid": None if self._survey_estimate is None else self._survey_estimate.geometry_valid,
            "survey_heading_deviation_deg": None if self._survey_estimate is None else self._survey_estimate.heading_deviation_deg,
            "survey_fit_rmse_m": None if self._survey_estimate is None else self._survey_estimate.fit_rmse_m,
            "survey_identity_last_seen_s": self._survey_identity_last_seen_s,
            "survey_line_extrapolation_m": None if self._survey_last_center_s is None else along - self._survey_last_center_s,
            "endpoint_complete": self._mission_complete,
        }
        output = CableTrackingOutput(
            time_s=float(navigation.time_s), estimated_cable_xy_m=line_xy,
            cross_track_m=abs(diagnostics["signed_cross_track_m"]),
            route_progress_m=along, cable_heading_deg=heading,
            burial_depth_m=None, burial_sigma_m=None,
            confidence=.8 if self._survey_estimate is not None else 0.,
            mode=mode, diagnostics=_finite_diagnostics(diagnostics))
        self.last_output = output
        return output

    def step_with_guidance(self, navigation: NavigationInput, magnetic: MagneticInput,
                           sonar: SonarInput | None = None, *, target_depth_m: float | None = None,
                           speed_mps: float | None = None) -> tuple[CableTrackingOutput, CableGuidanceOutput]:
        output = self.step(navigation, magnetic, sonar)
        try:
            depth = (abs(float(np.asarray(navigation.position_ned_m)[2]))
                     if target_depth_m is None else float(target_depth_m))
        except (TypeError, ValueError, OverflowError, IndexError):
            depth = float("nan")
        if output.mode in {"hold", "complete"}:
            return output, CableGuidanceOutput(
                desired_heading_deg=float(navigation.heading_deg), target_depth_m=depth,
                speed_mps=0., mode=output.mode, guidance_source="v2_shallow_survey_hold",
                safe_lock_active=True)
        nav_xy = np.asarray(navigation.position_ned_m, dtype=float)[:2]
        along = float(np.dot(nav_xy - self._survey_origin, self._survey_tangent))
        policy = self.survey_config
        if self._survey_phase == "survey":
            aim_s = min(along + policy.lookahead_m,
                        self._survey_policy.station_count * policy.half_wavelength_m)
            positions = self._survey_plan.waypoints_xy_m
            s = (positions - self._survey_origin) @ self._survey_tangent
            lateral = float(np.interp(aim_s, s,
                                      (positions - self._survey_origin) @ self._survey_normal))
            target = self._survey_origin + aim_s * self._survey_tangent + lateral * self._survey_normal
            direction = target - nav_xy
            direction /= max(float(np.linalg.norm(direction)), 1e-9)
        else:
            error = float(np.dot(nav_xy - self._survey_origin, self._survey_normal)
                          - self._line_lateral(along))
            correction = float(np.clip(math.atan2(-error, policy.lookahead_m),
                                       -math.radians(policy.maximum_heading_correction_deg),
                                       math.radians(policy.maximum_heading_correction_deg)))
            line_heading = math.atan2(
                (self._survey_tangent + self._survey_line_slope * self._survey_normal)[1],
                (self._survey_tangent + self._survey_line_slope * self._survey_normal)[0])
            direction = np.array([math.cos(line_heading + correction),
                                  math.sin(line_heading + correction)])
        current = self._survey_current_evidence
        current_valid = False
        if current is not None:
            try:
                velocity = np.asarray(current.velocity_xy_mps, dtype=float)
                sigma = float(current.sigma_mps)
                current_valid = (0. <= navigation.time_s - float(current.time_s) <= policy.maximum_current_age_s
                                 and velocity.shape == (2,) and np.isfinite(velocity).all()
                                 and math.isfinite(sigma) and sigma >= 0.
                                 and abs(float(np.dot(velocity, self._survey_normal))) + 2. * sigma
                                 <= policy.maximum_crosscurrent_mps
                                 and current.frame == "world" and bool(str(current.source).strip()))
            except (TypeError, ValueError, AttributeError, OverflowError):
                pass
        if not current_valid:
            output = self._fail("survey_current_evidence_lost")
            return output, CableGuidanceOutput(
                desired_heading_deg=float(navigation.heading_deg), target_depth_m=depth,
                speed_mps=0., mode="hold", guidance_source="v2_shallow_survey_current_hold",
                safe_lock_active=True)
        throughwater = policy.velocity_mps * direction - np.asarray(current.velocity_xy_mps)
        desired = (math.degrees(math.atan2(throughwater[1], throughwater[0])) + 180.) % 360. - 180.
        speed = policy.velocity_mps if speed_mps is None else min(policy.velocity_mps, max(0., float(speed_mps)))
        if (not math.isfinite(depth) or depth < 0. or not math.isfinite(speed)
                or not math.isfinite(desired)):
            output = self._fail("survey_command_invalid")
            return output, CableGuidanceOutput(
                desired_heading_deg=float(navigation.heading_deg), target_depth_m=abs(float(navigation.position_ned_m[2])),
                speed_mps=0., mode="hold", guidance_source="v2_shallow_survey_command_hold",
                safe_lock_active=True)
        return output, CableGuidanceOutput(
            desired_heading_deg=desired, target_depth_m=depth, speed_mps=speed,
            mode=output.mode, guidance_source="v2_shallow_survey_" + self._survey_phase,
            safe_lock_active=False,
            diagnostics={"survey_phase": self._survey_phase})
