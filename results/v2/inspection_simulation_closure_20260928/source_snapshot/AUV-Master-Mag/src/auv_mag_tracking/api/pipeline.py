"""Deployment-facing lightweight tracking pipeline facade."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np

from ..config import ScenarioConfig
from ..math_utils import (
    apply_route_prior_pose_error,
    build_polyline_projection_cache,
    nearest_point_on_polyline,
)
from ..perception.cross_track import MagneticCrossTrackEstimator
from ..perception.prior_alignment import PriorAlignmentEstimator
from .cable_map import CableMap
from .deployment_quality import DeploymentQualityEstimator
from .types import (
    CableGuidanceOutput,
    CableTrackingOutput,
    DeploymentPerceptionConfig,
    MagneticInput,
    NavigationInput,
    SonarInput,
)


class AuvMagTrackingPipeline:
    """Minimal plug-and-play facade around cable-map projection contracts.

    This facade is intentionally free of GUI and offline simulation dependencies.
    It provides a stable I/O shell for external AUV managers while deeper
    perception modules continue to evolve behind it.
    """

    def __init__(
        self,
        config: ScenarioConfig,
        cable_map: CableMap,
        deployment_config: Optional[DeploymentPerceptionConfig] = None,
    ) -> None:
        self.config = config
        self.cable_map = cable_map
        self._base_route_xy = np.asarray(cable_map.points_xy_m, dtype=float).copy()
        self._cache = cable_map.projection_cache()
        self.last_output: Optional[CableTrackingOutput] = None
        self.deployment_config = deployment_config
        self.quality_estimator = (
            DeploymentQualityEstimator(deployment_config)
            if deployment_config is not None
            else None
        )

        self._online_prior_alignment_enabled = bool(
            deployment_config is not None
            and getattr(deployment_config, "enable_online_prior_alignment", False)
        )
        self._prior_alignment: Optional[PriorAlignmentEstimator] = None
        self._cross_track_estimator: Optional[MagneticCrossTrackEstimator] = None
        self._vertical_separation_m = 0.0
        self._last_alignment_time_s: Optional[float] = None
        if self._online_prior_alignment_enabled:
            self._init_online_prior_alignment()

    def _init_online_prior_alignment(self) -> None:
        tracking_cfg = getattr(self.config, "tracking", None)
        self._cross_track_estimator = MagneticCrossTrackEstimator(
            window=int(getattr(tracking_cfg, "mag_cross_track_window", 40)),
            min_perp_amplitude_nt=float(
                getattr(tracking_cfg, "mag_cross_track_min_perp_amplitude_nt", 20.0)
            ),
            quality_gate=float(getattr(tracking_cfg, "mag_cross_track_quality_gate", 0.985)),
        )
        translation_var = float(
            getattr(tracking_cfg, "nominal_route_prior_correction_ekf_initial_translation_var_m2", 4.0)
        )
        rotation_var = float(
            getattr(tracking_cfg, "nominal_route_prior_correction_ekf_initial_rotation_var_deg2", 4.0)
        )
        self._prior_alignment = PriorAlignmentEstimator(
            initial_translation_xy_m=np.zeros(2, dtype=float),
            initial_rotation_deg=0.0,
            initial_covariance_diag=np.array(
                [translation_var, translation_var, rotation_var], dtype=float
            ),
        )
        vehicle_cfg = getattr(self.config, "vehicle", None)
        environment_cfg = getattr(self.config, "environment", None)
        altitude_m = float(getattr(vehicle_cfg, "altitude_above_seabed_m", 6.0))
        burial_m = self._nominal_burial_depth_m(environment_cfg)
        self._vertical_separation_m = max(altitude_m + burial_m, 1e-3)

    def _nominal_burial_depth_m(self, environment_cfg) -> float:
        depth = self.cable_map.burial_depth_m
        if isinstance(depth, (float, int)):
            return float(depth)
        if isinstance(depth, np.ndarray) and depth.size:
            return float(np.mean(depth))
        return float(getattr(environment_cfg, "burial_depth_m", 1.5))

    def reset(self) -> None:
        self.last_output = None
        if self.quality_estimator is not None:
            self.quality_estimator.reset()
        self._last_alignment_time_s = None
        self._cache = build_polyline_projection_cache(self._base_route_xy)
        if self._online_prior_alignment_enabled:
            self._init_online_prior_alignment()

    def export_state(self) -> dict[str, object]:
        if self.last_output is None:
            return {"initialized": True, "has_output": False}
        return {
            "initialized": True,
            "has_output": True,
            "route_progress_m": self.last_output.route_progress_m,
            "cross_track_m": self.last_output.cross_track_m,
            "confidence": self.last_output.confidence,
            "mode": self.last_output.mode,
        }

    def step(
        self,
        navigation: NavigationInput,
        magnetic: MagneticInput,
        sonar: Optional[SonarInput] = None,
    ) -> CableTrackingOutput:
        nav_xy = np.asarray(navigation.position_ned_m, dtype=float)[:2]
        point_xy, tangent_xy, distance_m, progress_m, segment_index = nearest_point_on_polyline(
            nav_xy,
            self._cache,
        )
        normal_xy = np.array([-tangent_xy[1], tangent_xy[0]], dtype=float)
        signed_cross_track_m = float(np.dot(nav_xy - point_xy, normal_xy))

        alignment_diag: Optional[dict[str, object]] = None
        if self._online_prior_alignment_enabled and self._prior_alignment is not None:
            alignment_diag = self._update_prior_alignment(
                navigation=navigation,
                magnetic=magnetic,
                nav_xy=nav_xy,
                tangent_xy=tangent_xy,
                normal_xy=normal_xy,
                prior_signed_cross_track_m=signed_cross_track_m,
            )
            point_xy, tangent_xy, distance_m, progress_m, segment_index = nearest_point_on_polyline(
                nav_xy,
                self._cache,
            )
            normal_xy = np.array([-tangent_xy[1], tangent_xy[0]], dtype=float)
            signed_cross_track_m = float(np.dot(nav_xy - point_xy, normal_xy))

        estimate_xy = point_xy
        source = "map_projection"
        confidence = 0.5
        if sonar is not None and sonar.valid and sonar.relative_position_body_m is not None:
            sonar_xy = self._sonar_to_ned(nav_xy, navigation.heading_deg, sonar.relative_position_body_m)
            estimate_xy = sonar_xy
            source = "sonar"
            confidence = max(0.0, min(float(sonar.confidence), 1.0))

        cable_heading_deg = float(math.degrees(math.atan2(tangent_xy[1], tangent_xy[0])))
        burial_depth = None
        if isinstance(self.cable_map.burial_depth_m, (float, int)):
            burial_depth = float(self.cable_map.burial_depth_m)
        elif isinstance(self.cable_map.burial_depth_m, np.ndarray) and self.cable_map.burial_depth_m.size:
            idx = min(max(int(segment_index), 0), self.cable_map.burial_depth_m.size - 1)
            burial_depth = float(self.cable_map.burial_depth_m[idx])

        burial_sigma = None
        quality = None
        if self.quality_estimator is not None:
            quality = self.quality_estimator.evaluate(
                navigation=navigation,
                magnetic=magnetic,
                route_distance_m=float(distance_m),
                signed_cross_track_m=signed_cross_track_m,
                sonar=sonar,
            )
            confidence = float(quality.confidence)
            if quality.burial_estimate is not None:
                burial_depth = float(quality.burial_estimate.depth_m)
                burial_sigma = float(quality.burial_estimate.sigma_m)
            if not quality.magnetic_used:
                source = f"{source}+quality_limited"

        mode = "track" if confidence >= 0.5 else "map_fallback"
        if quality is not None and not quality.industrial_ready:
            mode = "quality_limited"

        diagnostics = {
            "source": source,
            "map_frame": self.cable_map.frame,
            "map_segment_index": int(segment_index),
            "magnetic_used": False if quality is None else bool(quality.magnetic_used),
            "magnetic_sample_count": self._magnetic_sample_count(magnetic),
            "navigation_source": navigation.source,
            "signed_cross_track_m": signed_cross_track_m,
        }
        if quality is not None:
            diagnostics.update(
                {
                    "deployment_quality_connected": True,
                    "magnetic_strength_nt": quality.magnetic_strength_nt,
                    "magnetic_std_nt": quality.magnetic_std_nt,
                    "magnetic_snr_db": quality.snr_db,
                    "magnetic_confidence": quality.magnetic_confidence,
                    "fit_residual_m": quality.fit_residual_m,
                    "prior_alignment_residual_m": quality.prior_alignment_residual_m,
                    "quality_flags": quality.quality_flags,
                    "industrial_ready": quality.industrial_ready,
                    "burial_fit_quality": (
                        None if quality.burial_estimate is None else quality.burial_estimate.fit_quality
                    ),
                    "burial_sample_count": quality.burial_sample_count,
                    "burial_status": quality.burial_status,
                }
            )
        else:
            diagnostics["deployment_quality_connected"] = False

        if alignment_diag is not None:
            diagnostics["source"] = (
                f"{diagnostics['source']}+online_prior_alignment"
                if diagnostics["source"] == "map_projection"
                else diagnostics["source"]
            )
            diagnostics.update(alignment_diag)

        output = CableTrackingOutput(
            time_s=float(navigation.time_s),
            estimated_cable_xy_m=np.asarray(estimate_xy, dtype=float),
            cross_track_m=float(distance_m),
            route_progress_m=float(progress_m),
            cable_heading_deg=cable_heading_deg,
            burial_depth_m=burial_depth,
            burial_sigma_m=burial_sigma,
            confidence=confidence,
            mode=mode,
            diagnostics=diagnostics,
        )
        self.last_output = output
        return output

    def _update_prior_alignment(
        self,
        *,
        navigation: NavigationInput,
        magnetic: MagneticInput,
        nav_xy: np.ndarray,
        tangent_xy: np.ndarray,
        normal_xy: np.ndarray,
        prior_signed_cross_track_m: float,
    ) -> dict[str, object]:
        """Derive an independent magnetic cross-track observation and correct the prior.

        The observation is derived from the anomaly ratio ``y = (B_down/B_perp)*d``
        (line current cancels), giving a signed cross-track offset that is
        independent of the route prior.  This is the only prior-independent cable
        observation available to the deployment facade (the ROS node feeds no
        sonar), so it is what lets the online estimator pull the distorted prior
        back toward the true cable in closed loop.
        """
        assert self._prior_alignment is not None and self._cross_track_estimator is not None
        tracking = getattr(self.config, "tracking", None)
        time_s = float(navigation.time_s)
        dt_s = 0.0
        if self._last_alignment_time_s is not None:
            dt_s = max(time_s - self._last_alignment_time_s, 0.0)
        self._last_alignment_time_s = time_s

        self._prior_alignment.predict(tracking, dt_s)
        self._prior_alignment.clear_observation_diagnostics()

        anomaly_nt = self._mean_anomaly_ned_nt(magnetic)
        tangent_norm = float(np.linalg.norm(tangent_xy))
        observed_offset_m = None
        cross_track_quality = 0.0
        if anomaly_nt is not None and tangent_norm > 1e-9:
            b_perp = float(np.dot(anomaly_nt[:2], normal_xy))
            b_down = float(anomaly_nt[2])
            self._cross_track_estimator.update(b_perp, b_down)
            cross_track_quality = float(self._cross_track_estimator.quality)
            observed_offset_m = self._cross_track_estimator.cross_track_offset_m(
                self._vertical_separation_m
            )

        diag: dict[str, object] = {
            "prior_alignment_connected": True,
            "prior_alignment_online": True,
            "prior_alignment_dt_s": dt_s,
            "prior_alignment_vertical_separation_m": float(self._vertical_separation_m),
            "prior_alignment_cross_track_quality": cross_track_quality,
            "prior_alignment_prior_cross_track_m": float(prior_signed_cross_track_m),
        }

        if observed_offset_m is None:
            diag.update(
                {
                    "prior_alignment_observed": False,
                    "prior_alignment_translation_norm_m": float(
                        np.linalg.norm(self._prior_alignment.state.translation_xy_m)
                    ),
                    "prior_alignment_rotation_deg": float(self._prior_alignment.state.rotation_deg),
                    "prior_alignment_accepted": False,
                    "prior_alignment_reason_code": int(self._prior_alignment.state.reason_code),
                }
            )
            return diag

        observed_point_xy = nav_xy - float(observed_offset_m) * normal_xy
        prior_point_xy, prior_tangent_xy, _, progress_m, _ = nearest_point_on_polyline(
            observed_point_xy,
            self._cache,
        )
        min_confidence = float(getattr(tracking, "nominal_route_prior_correction_min_confidence", 0.35))
        if cross_track_quality < min_confidence:
            diag.update(
                {
                    "prior_alignment_observed": True,
                    "prior_alignment_translation_norm_m": float(
                        np.linalg.norm(self._prior_alignment.state.translation_xy_m)
                    ),
                    "prior_alignment_rotation_deg": float(self._prior_alignment.state.rotation_deg),
                    "prior_alignment_accepted": False,
                    "prior_alignment_reason_code": int(self._prior_alignment.state.reason_code),
                    "prior_alignment_observed_offset_m": float(observed_offset_m),
                }
            )
            return diag

        state = self._prior_alignment.update(
            tracking=tracking,
            observed_point_xy=observed_point_xy,
            prior_point_xy=prior_point_xy,
            prior_tangent_xy=prior_tangent_xy,
            observed_heading_deg=None,
            confidence=cross_track_quality,
            progress_m=progress_m,
        )
        if state.accepted:
            self._rebuild_corrected_cache(state.translation_xy_m, state.rotation_deg)

        diag.update(
            {
                "prior_alignment_observed": True,
                "prior_alignment_observed_offset_m": float(observed_offset_m),
                "prior_alignment_residual_norm_m": float(state.residual_norm_m),
                "prior_alignment_applied_step_norm_m": float(state.applied_step_norm_m),
                "prior_alignment_translation_norm_m": float(np.linalg.norm(state.translation_xy_m)),
                "prior_alignment_rotation_deg": float(state.rotation_deg),
                "prior_alignment_accepted": bool(state.accepted),
                "prior_alignment_reason_code": int(state.reason_code),
            }
        )
        return diag

    def _rebuild_corrected_cache(self, translation_xy_m: np.ndarray, rotation_deg: float) -> None:
        corrected_route_xy = apply_route_prior_pose_error(
            self._base_route_xy,
            translation_xy_m,
            float(rotation_deg),
            (1.0, 1.0),
        )
        self._cache = build_polyline_projection_cache(corrected_route_xy)

    @staticmethod
    def _mean_anomaly_ned_nt(magnetic: MagneticInput) -> Optional[np.ndarray]:
        try:
            samples = np.asarray(magnetic.sample_block_nt, dtype=float).reshape(-1, 3)
        except Exception:
            return None
        if samples.size == 0:
            return None
        finite = samples[np.all(np.isfinite(samples), axis=1)]
        if finite.size == 0:
            return None
        return np.mean(finite, axis=0)

    def step_with_guidance(
        self,
        navigation: NavigationInput,
        magnetic: MagneticInput,
        sonar: Optional[SonarInput] = None,
        *,
        target_depth_m: Optional[float] = None,
        speed_mps: Optional[float] = None,
    ) -> tuple[CableTrackingOutput, CableGuidanceOutput]:
        """Return deployment tracking plus a controller-facing guidance command.

        The public API stays independent from ``main_viz.py`` and simulation
        truth.  It uses the stable cable-map projection contract as the fallback
        authority and exposes diagnostics so downstream systems can tell whether
        the heavier perception stack has been wired behind the facade.
        """
        output = self.step(navigation, magnetic, sonar)
        nav_xy = np.asarray(navigation.position_ned_m, dtype=float)[:2]
        point_xy, tangent_xy, _, _, _ = nearest_point_on_polyline(nav_xy, self._cache)
        cable_heading_deg = float(math.degrees(math.atan2(tangent_xy[1], tangent_xy[0])))
        normal_xy = np.array([-tangent_xy[1], tangent_xy[0]], dtype=float)
        signed_cross_track_m = float(np.dot(nav_xy - point_xy, normal_xy))

        tracking_cfg = getattr(self.config, "tracking", None)
        vehicle_cfg = getattr(self.config, "vehicle", None)
        gain_deg_per_m = float(getattr(tracking_cfg, "track_cross_track_gain_deg_per_m", 2.0))
        max_correction_deg = float(getattr(tracking_cfg, "track_cross_track_max_correction_deg", 20.0))
        correction_deg = float(np.clip(-gain_deg_per_m * signed_cross_track_m, -max_correction_deg, max_correction_deg))
        desired_heading_deg = self._wrap_deg(cable_heading_deg + correction_deg)

        cruise_speed = float(getattr(vehicle_cfg, "cruise_speed_mps", 0.8))
        min_turn_radius = float(getattr(vehicle_cfg, "min_turning_radius_m", float("inf")))
        max_yaw_rate = float(getattr(vehicle_cfg, "max_yaw_rate_deg_s", 0.0))
        resolved_speed = float(speed_mps if speed_mps is not None else cruise_speed)
        heading_error_deg = self._smallest_angle_error_deg(desired_heading_deg, navigation.heading_deg)
        yaw_rate_deg_s = float(np.clip(heading_error_deg, -max_yaw_rate, max_yaw_rate)) if max_yaw_rate > 0.0 else 0.0
        turn_radius = float("inf")
        if abs(yaw_rate_deg_s) > 1e-9:
            turn_radius = resolved_speed / max(math.radians(abs(yaw_rate_deg_s)), 1e-9)
        if math.isfinite(min_turn_radius) and turn_radius < min_turn_radius and min_turn_radius > 0.0:
            max_radius_yaw_rate = math.degrees(resolved_speed / min_turn_radius)
            yaw_rate_deg_s = float(np.clip(yaw_rate_deg_s, -max_radius_yaw_rate, max_radius_yaw_rate))
            turn_radius = min_turn_radius

        guidance = CableGuidanceOutput(
            desired_heading_deg=desired_heading_deg,
            target_depth_m=float(
                target_depth_m
                if target_depth_m is not None
                else abs(float(np.asarray(navigation.position_ned_m, dtype=float)[2]))
            ),
            speed_mps=resolved_speed,
            mode=output.mode,
            guidance_source="api_route_projection",
            zigzag_width_m=float(getattr(tracking_cfg, "max_zigzag_width_m", 0.0)),
            commanded_turn_radius_m=turn_radius,
            yaw_rate_deg_s=yaw_rate_deg_s,
            safe_lock_active=False,
            emergency_flag=False,
            diagnostics={
                "facade_mode": "route_projection_guidance",
                "full_perception_stack_connected": bool(output.diagnostics.get("deployment_quality_connected", False)),
                "deployment_quality_connected": bool(output.diagnostics.get("deployment_quality_connected", False)),
                "signed_cross_track_m": signed_cross_track_m,
                "heading_correction_deg": correction_deg,
                "magnetic_used": bool(output.diagnostics.get("magnetic_used", False)),
                "quality_flags": list(output.diagnostics.get("quality_flags", [])),
            },
        )
        output.diagnostics["guidance_source"] = guidance.guidance_source
        output.diagnostics["signed_cross_track_m"] = signed_cross_track_m
        return output, guidance

    @staticmethod
    def _sonar_to_ned(position_xy: np.ndarray, heading_deg: float, relative_body_m: np.ndarray) -> np.ndarray:
        rel = np.asarray(relative_body_m, dtype=float)
        if rel.size < 2:
            raise ValueError("sonar relative_position_body_m must contain at least x/y")
        yaw = math.radians(float(heading_deg))
        rot = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]], dtype=float)
        return np.asarray(position_xy, dtype=float) + rot @ rel[:2]

    @staticmethod
    def _wrap_deg(angle_deg: float) -> float:
        return (float(angle_deg) + 180.0) % 360.0 - 180.0

    @staticmethod
    def _smallest_angle_error_deg(target_deg: float, current_deg: float) -> float:
        return AuvMagTrackingPipeline._wrap_deg(float(target_deg) - float(current_deg))

    @staticmethod
    def _magnetic_sample_count(magnetic: MagneticInput) -> int:
        try:
            return int(np.asarray(magnetic.sample_block_nt).reshape(-1, 3).shape[0])
        except Exception:
            return 0
