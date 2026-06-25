"""Mission-FSM-backed zig-zag controller and constrained AUV kinematics.

The controller is the kinematic layer: it asks :class:`MissionManager` *what* to do
(sweep / align / track / surface) and translates that into a heading + speed command
under the vehicle's turning-radius and yaw-rate limits.  All strategic decisions live
in the mission manager; the controller only owns geometry.
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .config import ScenarioConfig
from .math_utils import (
    Pose,
    build_nominal_route_xy,
    build_polyline_projection_cache,
    heading_from_direction_xy,
    nearest_point_on_polyline,
    smallest_angle_error_deg,
    wrap_angle_deg,
)
from .mission_manager import MissionDecision, MissionInput, MissionManager, MissionState, MissionThresholds
from .perception import PerceptionState


@dataclass
class GuidanceCommand:
    """表示控制器输出给车辆运动学的单步航向与速度指令。"""

    desired_heading_deg: float
    speed_mps: float
    mode: MissionState
    guidance_source: str = "SEARCH"
    commanded_turn_radius_m: float = float("inf")
    yaw_rate_deg_s: float = 0.0
    safe_lock_active: bool = False
    zigzag_width_m: float = 0.0
    emergency_flag: bool = False


class ZigZagController:
    def __init__(self, scenario: ScenarioConfig) -> None:
        """根据场景配置初始化之字形控制器、任务 FSM 与名义路线缓存。"""
        self.scenario = scenario
        self.leg_sign = 1.0
        self.last_leg_flip_time_s = 0.0
        self.mission_manager = MissionManager(self._build_mission_thresholds())
        self.nominal_route_xy = build_nominal_route_xy(self.scenario.environment)
        self.nominal_route_lookup = build_polyline_projection_cache(self.nominal_route_xy)
        self.smoothed_base_heading_deg: Optional[float] = None
        self.last_trusted_cable_point_xy_m: Optional[np.ndarray] = None
        self.last_trusted_cable_heading_deg: Optional[float] = None
        self.reacquire_anchor_xy_m: Optional[np.ndarray] = None
        self.reacquire_anchor_heading_deg: Optional[float] = None
        self.reacquire_leg_index: int = 0

    def _build_mission_thresholds(self) -> MissionThresholds:
        """从场景配置组装任务层阈值（暂用默认 + 复用已有置信度阈值）。"""
        tracking = self.scenario.tracking
        vehicle = self.scenario.vehicle
        # The loss hold must outlast one zig-zag crossing period so a sweep
        # trough is never mistaken for cable loss.
        sweep_period_s = tracking.max_zigzag_width_m * tracking.crossing_width_periods / max(vehicle.cruise_speed_mps, 0.5)
        return MissionThresholds(
            track_confidence_required=tracking.high_confidence_threshold,
            signal_hold_s=max(tracking.lost_timeout_s, sweep_period_s),
            cov_perp_converged_m2=tracking.fsm_cov_perp_converged_m2,
            yaw_err_converged_deg=tracking.fsm_yaw_err_converged_deg,
            reacquire_region_min_confidence=tracking.reacquire_region_min_confidence,
            reacquire_region_entry_streak_required=tracking.reacquire_region_entry_streak_required,
            reacquire_region_recovery_streak_required=tracking.reacquire_region_recovery_streak_required,
            reacquire_region_unavailable_hold_s=tracking.reacquire_region_unavailable_hold_s,
            reacquire_region_max_duration_s=tracking.reacquire_region_max_duration_s,
        )

    def _nominal_route_reference(self, position_xy_m: np.ndarray) -> tuple:
        """返回当前位置在名义路线上的最近点、切向与距离。"""
        best_point, best_tangent, best_distance, _, _ = nearest_point_on_polyline(position_xy_m, self.nominal_route_lookup)
        return best_point, best_tangent, best_distance

    def _crossing_angle_deg(self, zigzag_width_m: float) -> float:
        """根据当前之字形宽度估计交叉入射角。"""
        tracking = self.scenario.tracking
        lookahead_distance_m = max(
            tracking.lookahead_turn_radius_factor * self.scenario.vehicle.min_turning_radius_m,
            tracking.lookahead_min_distance_m,
        )
        nominal_angle_deg = float(np.rad2deg(np.arctan2(max(zigzag_width_m, 1e-6), lookahead_distance_m)))
        return float(np.clip(
            nominal_angle_deg,
            self.scenario.tracking.approach_angle_min_deg,
            self.scenario.tracking.approach_angle_max_deg,
        ))

    def _limited_yaw_rate_deg_s(self, speed_mps: float, desired_heading_deg: float, current_heading_deg: float) -> float:
        """在转弯半径和最大角速度约束下计算可执行的偏航角速度。"""
        heading_error_deg = smallest_angle_error_deg(desired_heading_deg, current_heading_deg)
        max_radius_rate_deg_s = float(np.rad2deg(max(speed_mps, 1e-6) / max(self.scenario.vehicle.min_turning_radius_m, 1e-6)))
        max_yaw_rate_deg_s = min(self.scenario.vehicle.max_yaw_rate_deg_s, max_radius_rate_deg_s)
        requested_yaw_rate_deg_s = heading_error_deg / max(self.scenario.dt_s, 1e-6)
        return float(np.clip(requested_yaw_rate_deg_s, -max_yaw_rate_deg_s, max_yaw_rate_deg_s))

    def _base_heading_deg(self, pose: Pose, perception: PerceptionState) -> float:
        """选择本步之字形扫描的基准（沿电缆纵向）航向。

        纵向基准与横向压线分属两个轴：名义模式下纵向参考权威地来自先验路线切向，
        磁/声呐只负责横向回缆（见 :meth:`_cross_track_offset_m` /
        :meth:`_track_cross_track_offset_m`），不参与纵向基准。否则在急弯处滑窗
        【直线】拟合会结构性滞后真实切向（拟合窗内电缆非直线），又被 TRACK 压线
        饿死峰值而冻结，反过来污染基准航向、把车辆甩出弯道。

        部署模式没有先验路线，此时纵向基准只能退回采信融合航向（置信度足够时）。
        """
        if self.scenario.tracking.use_nominal_route_prior:
            _, tangent_xy, _ = self._nominal_route_reference(pose.position_ned_m[:2])
            return heading_from_direction_xy(tangent_xy)

        along_cable_heading_deg = pose.heading_deg
        if (
            perception.local_path_tracking_state == "curve_track"
            and perception.fused_heading_deg is not None
        ):
            return perception.fused_heading_deg
        if (
            perception.fused_heading_deg is not None
            and perception.confidence >= self.scenario.tracking.low_confidence_threshold
        ):
            candidate_heading_deg = perception.fused_heading_deg
            if abs(smallest_angle_error_deg(candidate_heading_deg, pose.heading_deg)) > 90.0:
                candidate_heading_deg = wrap_angle_deg(candidate_heading_deg + 180.0)
            along_cable_heading_deg = candidate_heading_deg
        return along_cable_heading_deg

    def _cross_track_offset_m(self, pose: Pose, perception: PerceptionState, base_heading_deg: float) -> Optional[float]:
        """返回车辆相对电缆的带符号横向偏移（沿基准航向左正右负）。

        名义模式用先验路线最近点，部署模式用感知估计的电缆点；无可用电缆点时返回
        ``None``，此时退回看门狗按时间翻腿。
        """
        position_xy_m = pose.position_ned_m[:2]
        if self.scenario.tracking.use_nominal_route_prior:
            cable_point_xy_m, _, _ = self._nominal_route_reference(position_xy_m)
        elif perception.estimated_cable_point_xy_m is not None:
            cable_point_xy_m = np.asarray(perception.estimated_cable_point_xy_m, dtype=float)
        else:
            return None
        heading_rad = np.deg2rad(base_heading_deg)
        perpendicular_xy = np.array([-np.sin(heading_rad), np.cos(heading_rad)], dtype=float)
        return float(np.dot(position_xy_m - cable_point_xy_m, perpendicular_xy))

    def _track_cross_track_offset_m(self, pose: Pose, perception: PerceptionState, base_heading_deg: float) -> Optional[float]:
        """返回 TRACK_ACTIVE 压线用的带符号横向偏移，按 声呐 > 磁 > 路由 优先级取信号。

        声呐在线时直接用其电缆点（声呐优先）；声呐失效后改用磁比值偏移实现稳定
        独立持续跟踪；二者都不可用时退回先验路线。返回值与 :meth:`_cross_track_offset_m`
        同符号约定（沿基准航向左正右负），供控制器驱动偏移趋零。
        """
        position_xy_m = pose.position_ned_m[:2]
        heading_rad = np.deg2rad(base_heading_deg)
        perpendicular_xy = np.array([-np.sin(heading_rad), np.cos(heading_rad)], dtype=float)
        sonar_fresh = perception.last_detection_age_s <= self.scenario.tracking.lost_timeout_s
        if sonar_fresh and perception.estimated_cable_point_xy_m is not None:
            cable_point_xy_m = np.asarray(perception.estimated_cable_point_xy_m, dtype=float)
            return float(np.dot(position_xy_m - cable_point_xy_m, perpendicular_xy))
        if perception.magnetic_lookahead_valid and perception.magnetic_lookahead_cable_point_xy_m is not None:
            cable_point_xy_m = np.asarray(perception.magnetic_lookahead_cable_point_xy_m, dtype=float)
            return float(np.dot(position_xy_m - cable_point_xy_m, perpendicular_xy))
        if perception.magnetic_cross_track_offset_m is not None:
            return float(perception.magnetic_cross_track_offset_m)
        return self._cross_track_offset_m(pose, perception, base_heading_deg)

    def _mission_input(self, pose: Pose, perception: PerceptionState) -> MissionInput:
        """将感知状态与位姿投影为任务 FSM 的输入契约。"""
        yaw_error_deg = (
            smallest_angle_error_deg(perception.fused_heading_deg, pose.heading_deg)
            if perception.fused_heading_deg is not None
            else None
        )
        return MissionInput(
            time_s=perception.time_s,
            mag_strength_nT=perception.tracking_strength_nt,
            sonar_confidence=perception.sonar_confidence,
            confidence=perception.confidence,
            fused_heading_deg=perception.fused_heading_deg,
            yaw_error_deg=yaw_error_deg,
            fit_covariance_xy_m2=perception.fit_result.covariance_xy_m2,
            peak_detected=perception.peak_detected,
            reacquire_required=perception.deployment_reacquire_required,
            reacquire_region_available=perception.reacquire_region_center_xy_m is not None,
            reacquire_region_confidence=perception.reacquire_region_confidence,
            reacquire_region_control_enabled=self.scenario.tracking.reacquire_region_control_enabled,
        )

    def _crossing_angle_for_state(
        self,
        state: MissionState,
        zigzag_width_m: float,
        magnetic_fit_ready: bool,
        local_path_tracking_state: str = "collecting",
    ) -> float:
        """按任务状态决定本步穿越角。

        SEARCH 与 LOCK_ALIGN 都需主动横摆穿越电缆以持续产生磁峰值（拟合收敛的前提）；
        二者唯一区别是 LOCK_ALIGN 降速取更密采样。TRACK_ACTIVE 默认压线；
        若场景显式配置 ``track_active_zigzag_angle_deg``，则按国标/测线需求保留低幅 zig-zag。
        """
        if local_path_tracking_state == "curve_track":
            return max(0.0, self.scenario.tracking.curve_track_crossing_angle_deg)
        if state in (MissionState.SEARCH_ZIGZAG, MissionState.LOCK_ALIGN):
            angle_deg = self._crossing_angle_deg(zigzag_width_m)
            if not magnetic_fit_ready:
                angle_deg = max(angle_deg, self.scenario.tracking.probing_crossing_angle_deg)
            return angle_deg
        if state == MissionState.TRACK_ACTIVE:
            return max(0.0, self.scenario.tracking.track_active_zigzag_angle_deg)
        return 0.0  # EMERGENCY_SURFACE: hold centerline

    def _remember_trusted_cable_state(self, perception: PerceptionState) -> None:
        """Store the last usable cable anchor for turn-aware reacquisition."""
        if perception.deployment_reacquire_required:
            return
        if perception.estimated_cable_point_xy_m is None:
            return
        heading_deg = (
            perception.local_path_heading_deg
            if perception.local_path_heading_deg is not None
            else perception.fused_heading_deg
        )
        if heading_deg is None:
            return
        if perception.confidence < self.scenario.tracking.low_confidence_threshold:
            return
        self.last_trusted_cable_point_xy_m = np.asarray(perception.estimated_cable_point_xy_m, dtype=float).copy()
        self.last_trusted_cable_heading_deg = float(heading_deg)
        self.reacquire_anchor_xy_m = None
        self.reacquire_anchor_heading_deg = None
        self.reacquire_leg_index = 0

    def _reacquire_heading_deg(
        self,
        pose: Pose,
        perception: PerceptionState,
        fallback_heading_deg: float,
    ) -> Optional[float]:
        """Search around the last cable anchor instead of sweeping along a stale heading."""
        anchor_xy = self.last_trusted_cable_point_xy_m
        heading_deg = self.last_trusted_cable_heading_deg
        if not perception.deployment_reacquire_required and perception.estimated_cable_point_xy_m is not None:
            anchor_xy = np.asarray(perception.estimated_cable_point_xy_m, dtype=float)
        if not perception.deployment_reacquire_required and perception.local_path_heading_deg is not None:
            heading_deg = perception.local_path_heading_deg
        elif not perception.deployment_reacquire_required and perception.fused_heading_deg is not None:
            heading_deg = perception.fused_heading_deg

        if anchor_xy is None:
            return None
        if heading_deg is None:
            heading_deg = fallback_heading_deg
        if perception.deployment_reacquire_required and self.scenario.tracking.reacquire_zigzag_enabled:
            if self.reacquire_anchor_xy_m is None:
                self.reacquire_anchor_xy_m = anchor_xy.copy()
                self.reacquire_anchor_heading_deg = float(heading_deg)
                self.reacquire_leg_index = 0
            return self._reacquire_zigzag_heading_deg(pose)

        heading_rad = np.deg2rad(heading_deg)
        tangent_xy = np.array([np.cos(heading_rad), np.sin(heading_rad)], dtype=float)
        perpendicular_xy = np.array([-tangent_xy[1], tangent_xy[0]], dtype=float)
        radius_m = max(self.scenario.tracking.reacquire_search_radius_m, self.scenario.tracking.min_zigzag_half_band_width_m)
        anchor_delta_xy = anchor_xy - pose.position_ned_m[:2]
        if np.linalg.norm(anchor_delta_xy) > 1.5 * radius_m:
            target_xy = anchor_xy
        else:
            target_xy = anchor_xy + 0.5 * radius_m * tangent_xy + self.leg_sign * radius_m * perpendicular_xy
        delta_xy = target_xy - pose.position_ned_m[:2]
        if np.linalg.norm(delta_xy) < 1e-6:
            return None
        return heading_from_direction_xy(delta_xy)

    def _reacquire_zigzag_heading_deg(self, pose: Pose) -> Optional[float]:
        """Run a bounded zig-zag around the last trusted cable anchor."""
        if self.reacquire_anchor_xy_m is None or self.reacquire_anchor_heading_deg is None:
            return None

        heading_rad = np.deg2rad(self.reacquire_anchor_heading_deg)
        tangent_xy = np.array([np.cos(heading_rad), np.sin(heading_rad)], dtype=float)
        perpendicular_xy = np.array([-tangent_xy[1], tangent_xy[0]], dtype=float)
        half_band_m = max(self.scenario.tracking.reacquire_search_radius_m, self.scenario.tracking.min_zigzag_half_band_width_m)
        along_step_m = max(self.scenario.tracking.reacquire_zigzag_along_step_m, 1.0)
        max_along_m = max(self.scenario.tracking.reacquire_zigzag_max_along_m, along_step_m)

        relative_xy = pose.position_ned_m[:2] - self.reacquire_anchor_xy_m
        along_m = float(np.dot(relative_xy, tangent_xy))
        lateral_m = float(np.dot(relative_xy, perpendicular_xy))
        if self.leg_sign > 0.0 and lateral_m >= half_band_m:
            self.leg_sign = -1.0
            self.reacquire_leg_index += 1
        elif self.leg_sign < 0.0 and lateral_m <= -half_band_m:
            self.leg_sign = 1.0
            self.reacquire_leg_index += 1

        target_along_m = min(max_along_m, max(along_step_m, self.reacquire_leg_index * along_step_m))
        target_xy = self.reacquire_anchor_xy_m + target_along_m * tangent_xy + self.leg_sign * half_band_m * perpendicular_xy
        delta_xy = target_xy - pose.position_ned_m[:2]
        if np.linalg.norm(delta_xy) < 1e-6:
            return None
        return heading_from_direction_xy(delta_xy)

    def _region_reacquire_heading_deg(self, pose: Pose, perception: PerceptionState) -> Optional[float]:
        """Execute the observable region selected by perception/mission."""
        if perception.reacquire_region_center_xy_m is None or perception.reacquire_region_heading_deg is None:
            return None

        center_xy = np.asarray(perception.reacquire_region_center_xy_m, dtype=float)
        heading_rad = np.deg2rad(perception.reacquire_region_heading_deg)
        tangent_xy = np.array([np.cos(heading_rad), np.sin(heading_rad)], dtype=float)
        normal_xy = np.array([-tangent_xy[1], tangent_xy[0]], dtype=float)
        half_length_m = max(perception.reacquire_region_half_length_m, 1.0)
        half_width_m = max(perception.reacquire_region_half_width_m, self.scenario.tracking.min_zigzag_half_band_width_m)

        position_xy = pose.position_ned_m[:2]
        relative_xy = position_xy - center_xy
        along_m = float(np.dot(relative_xy, tangent_xy))
        lateral_m = float(np.dot(relative_xy, normal_xy))

        if abs(along_m) > half_length_m or abs(lateral_m) > half_width_m:
            entry_xy = center_xy - half_length_m * tangent_xy
            if np.linalg.norm(entry_xy - position_xy) > np.linalg.norm(center_xy - position_xy):
                entry_xy = center_xy
            target_xy = entry_xy
        else:
            if self.leg_sign > 0.0 and lateral_m >= half_width_m:
                self.leg_sign = -1.0
                self.last_leg_flip_time_s = perception.time_s
            elif self.leg_sign < 0.0 and lateral_m <= -half_width_m:
                self.leg_sign = 1.0
                self.last_leg_flip_time_s = perception.time_s
            target_along_m = float(np.clip(along_m + 0.5 * half_length_m, -half_length_m, half_length_m))
            target_xy = center_xy + target_along_m * tangent_xy + self.leg_sign * half_width_m * normal_xy

        delta_xy = target_xy - position_xy
        if np.linalg.norm(delta_xy) < 1e-6:
            return None
        return heading_from_direction_xy(delta_xy)

    def update(self, pose: Pose, perception: PerceptionState) -> GuidanceCommand:
        """结合感知状态与任务 FSM 输出当前控制指令。"""
        decision: MissionDecision = self.mission_manager.update(self._mission_input(pose, perception))
        state = decision.state

        base_heading_deg = self._base_heading_deg(pose, perception)
        zigzag_width_m = perception.zigzag_width_m
        self._remember_trusted_cable_state(perception)
        reacquire_zigzag_active = (
            perception.deployment_reacquire_required
            and self.scenario.tracking.reacquire_zigzag_enabled
        )

        # --- Cross-track-band leg flip ---
        # The sweep flips when the vehicle reaches the half-band edge on the side
        # it is currently steering toward.  Because the base heading is the cable
        # tangent, every leg advances along-track while weaving ±half_band across
        # the line, so cable crossings recur at a fixed along-track spacing and the
        # field reliably rises-peaks-falls (the peak detector's prerequisite).
        half_band_m = 0.5 * max(zigzag_width_m, self.scenario.tracking.min_zigzag_half_band_width_m)
        cross_track_offset_m = self._cross_track_offset_m(pose, perception, base_heading_deg)
        if cross_track_offset_m is not None and not perception.safe_lock_active and not reacquire_zigzag_active:
            if self.leg_sign > 0.0 and cross_track_offset_m >= half_band_m:
                self.leg_sign = -1.0
                self.last_leg_flip_time_s = perception.time_s
            elif self.leg_sign < 0.0 and cross_track_offset_m <= -half_band_m:
                self.leg_sign = 1.0
                self.last_leg_flip_time_s = perception.time_s

        # --- Watchdog leg flip (anti-escape fallback) ---
        # Guards the case where no cable point is available (deployment cold start)
        # so the band rule cannot fire; forces a turn after a generous sweep time.
        expected_cross_time_s = max(
            zigzag_width_m * self.scenario.tracking.crossing_width_periods / max(pose.speed_mps, 0.5),
            self.scenario.tracking.watchdog_min_cross_time_s,
        )
        if not reacquire_zigzag_active and perception.time_s - self.last_leg_flip_time_s > expected_cross_time_s:
            self.leg_sign *= -1.0
            self.last_leg_flip_time_s = perception.time_s

        # --- Base-heading low-pass ---
        if perception.local_path_tracking_state == "curve_track":
            self.smoothed_base_heading_deg = base_heading_deg
        elif self.smoothed_base_heading_deg is None:
            self.smoothed_base_heading_deg = base_heading_deg
        else:
            heading_diff = smallest_angle_error_deg(base_heading_deg, self.smoothed_base_heading_deg)
            self.smoothed_base_heading_deg = wrap_angle_deg(self.smoothed_base_heading_deg + heading_diff * self.scenario.tracking.base_heading_smoothing)
        effective_base_heading_deg = self.smoothed_base_heading_deg

        magnetic_fit_ready = perception.fit_result.origin_xy_m is not None and len(perception.estimated_path_points_xy_m) >= 3
        crossing_angle_deg = self._crossing_angle_for_state(
            state,
            zigzag_width_m,
            magnetic_fit_ready,
            perception.local_path_tracking_state,
        )
        desired_heading_deg = wrap_angle_deg(effective_base_heading_deg + self.leg_sign * crossing_angle_deg)
        if perception.deployment_reacquire_required:
            reacquire_heading_deg = self._reacquire_heading_deg(pose, perception, effective_base_heading_deg)
            if reacquire_heading_deg is not None:
                desired_heading_deg = reacquire_heading_deg
        if state == MissionState.REACQUIRE_REGION:
            region_heading_deg = self._region_reacquire_heading_deg(pose, perception)
            if region_heading_deg is not None:
                desired_heading_deg = region_heading_deg

        # --- TRACK_ACTIVE centerline hold ---
        # Holding the cable tangent alone lets residual cross-track offset persist
        # (and drift during sonar outages).  A proportional correction steers the
        # signed offset to zero: a positive (left) offset subtracts heading to bank
        # right.  Source priority is sonar > magnetic ratio > prior route, so the
        # vehicle keeps tracking the line from the magnetometer alone when sonar
        # drops out, without the coarse offset ever touching the line fit.
        if state == MissionState.TRACK_ACTIVE:
            if (
                self.scenario.tracking.magnetic_lookahead_pursuit_enabled
                and perception.magnetic_lookahead_valid
                and perception.magnetic_lookahead_target_xy_m is not None
            ):
                target_xy_m = np.asarray(perception.magnetic_lookahead_target_xy_m, dtype=float)
                to_target_xy = target_xy_m - pose.position_ned_m[:2]
                target_heading_deg = heading_from_direction_xy(to_target_xy)
                if target_heading_deg is not None:
                    pursuit_error_deg = smallest_angle_error_deg(target_heading_deg, desired_heading_deg)
                    pursuit_correction_deg = float(np.clip(
                        self.scenario.tracking.magnetic_lookahead_pursuit_gain * pursuit_error_deg,
                        -self.scenario.tracking.magnetic_lookahead_pursuit_max_correction_deg,
                        self.scenario.tracking.magnetic_lookahead_pursuit_max_correction_deg,
                    ))
                    desired_heading_deg = wrap_angle_deg(desired_heading_deg + pursuit_correction_deg)
            track_offset_m = self._track_cross_track_offset_m(pose, perception, effective_base_heading_deg)
            if track_offset_m is not None:
                correction_deg = float(np.clip(
                    -self.scenario.tracking.track_cross_track_gain_deg_per_m * track_offset_m,
                    -self.scenario.tracking.track_cross_track_max_correction_deg,
                    self.scenario.tracking.track_cross_track_max_correction_deg,
                ))
                desired_heading_deg = wrap_angle_deg(desired_heading_deg + correction_deg)

        # The align slow-down only helps once a fit exists to refine; while still
        # bootstrapping, keep cruise speed so the zig-zag legs are long enough to
        # produce clean magnetic peaks (the fit's only input).
        speed_factor = decision.speed_factor
        if state == MissionState.LOCK_ALIGN and not magnetic_fit_ready:
            speed_factor = 1.0
        if perception.local_path_tracking_state == "curve_track":
            speed_factor = min(speed_factor, self.scenario.tracking.curve_track_speed_factor)
        speed_mps = self.scenario.vehicle.cruise_speed_mps * speed_factor

        yaw_rate_deg_s = self._limited_yaw_rate_deg_s(speed_mps, desired_heading_deg, pose.heading_deg)
        turning_radius_m = float("inf")
        if abs(yaw_rate_deg_s) > 1e-6:
            turning_radius_m = max(speed_mps, 1e-6) / max(np.deg2rad(abs(yaw_rate_deg_s)), 1e-6)

        return GuidanceCommand(
            desired_heading_deg=desired_heading_deg,
            speed_mps=speed_mps,
            mode=state,
            guidance_source=decision.guidance_source,
            commanded_turn_radius_m=turning_radius_m,
            yaw_rate_deg_s=yaw_rate_deg_s,
            safe_lock_active=perception.safe_lock_active,
            zigzag_width_m=zigzag_width_m,
            emergency_flag=decision.emergency_flag,
        )


def apply_attitude_profile(pose: Pose, scenario: ScenarioConfig, time_s: float) -> None:
    """根据场景配置为当前姿态叠加周期性俯仰和横滚扰动。"""
    vehicle = scenario.vehicle
    pose.pitch_deg = vehicle.pitch_amplitude_deg * np.sin(2.0 * np.pi * vehicle.pitch_frequency_hz * time_s)
    pose.roll_deg = vehicle.roll_amplitude_deg * np.sin(2.0 * np.pi * vehicle.roll_frequency_hz * time_s)


def propagate_vehicle(pose: Pose, command: GuidanceCommand, scenario: ScenarioConfig, seabed_depth_m: float, dt_s: float) -> Pose:
    """根据控制指令和简化运动学模型推进车辆状态。"""
    updated_pose = pose.copy()
    speed_mps = max(command.speed_mps, 1e-6)
    radius_limited_yaw_rate_deg_s = float(np.rad2deg(speed_mps / max(scenario.vehicle.min_turning_radius_m, 1e-6)))
    max_yaw_rate_deg_s = min(scenario.vehicle.max_yaw_rate_deg_s, radius_limited_yaw_rate_deg_s)
    commanded_yaw_rate_deg_s = float(np.clip(command.yaw_rate_deg_s, -max_yaw_rate_deg_s, max_yaw_rate_deg_s))
    if abs(commanded_yaw_rate_deg_s) <= 1e-9:
        heading_error_deg = smallest_angle_error_deg(command.desired_heading_deg, updated_pose.heading_deg)
        commanded_yaw_rate_deg_s = float(np.clip(heading_error_deg / max(dt_s, 1e-6), -max_yaw_rate_deg_s, max_yaw_rate_deg_s))
    heading_step_deg = commanded_yaw_rate_deg_s * dt_s
    updated_pose.heading_deg = wrap_angle_deg(updated_pose.heading_deg + heading_step_deg)
    updated_pose.speed_mps = command.speed_mps

    heading_rad = np.deg2rad(updated_pose.heading_deg)
    north_rate = updated_pose.speed_mps * np.cos(heading_rad)
    east_rate = updated_pose.speed_mps * np.sin(heading_rad)
    updated_pose.position_ned_m[0] += north_rate * dt_s
    updated_pose.position_ned_m[1] += east_rate * dt_s
    updated_pose.position_ned_m[2] = seabed_depth_m - scenario.vehicle.altitude_above_seabed_m
    return updated_pose
