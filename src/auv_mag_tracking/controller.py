"""Behavior-tree-backed zig-zag controller and constrained AUV kinematics."""

from dataclasses import dataclass
from enum import Enum
from typing import Optional

import numpy as np

from .behavior_tree import BehaviorContext, BehaviorMode, BehaviorTree
from .config import ScenarioConfig
from .math_utils import (
    Pose,
    build_polyline_projection_cache,
    heading_from_direction_xy,
    nearest_point_on_polyline,
    sample_sine_overlay_path,
    sample_spline_path,
    smallest_angle_error_deg,
    wrap_angle_deg,
)
from .perception import PerceptionState


class TrackingMode(str, Enum):
    """定义控制层当前采用的跟踪工作模式。"""

    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    TURN = "TURN"
    HOLD = "HOLD"
    LOST = "LOST"
    SPIRAL_SEARCH = "SPIRAL_SEARCH"


@dataclass
class GuidanceCommand:
    """表示控制器输出给车辆运动学的单步航向与速度指令。"""

    desired_heading_deg: float
    speed_mps: float
    mode: TrackingMode
    guidance_source: str = "SEARCH"
    commanded_turn_radius_m: float = float("inf")
    yaw_rate_deg_s: float = 0.0
    safe_lock_active: bool = False
    zigzag_width_m: float = 0.0


class ZigZagController:
    def __init__(self, scenario: ScenarioConfig) -> None:
        """根据场景配置初始化之字形控制器与名义路线缓存。"""
        self.scenario = scenario
        self.leg_sign = 1.0
        self.last_leg_flip_time_s = 0.0
        self.behavior_tree = BehaviorTree()
        self.spiral_start_time_s: Optional[float] = None
        self.last_mode: Optional[TrackingMode] = None
        self.nominal_route_xy = self._build_nominal_route_xy()
        self.nominal_route_lookup = build_polyline_projection_cache(self.nominal_route_xy)

    def _build_nominal_route_xy(self) -> np.ndarray:
        """根据场景中的路线模式生成控制器使用的名义电缆路径。"""
        waypoints_xy = np.asarray(self.scenario.environment.cable_waypoints_xy_m, dtype=float)
        step_m = max(self.scenario.environment.field_segment_length_m * 0.5, 1.0)
        if self.scenario.environment.cable_route_mode == "spline":
            return sample_spline_path(waypoints_xy, step_m)
        if self.scenario.environment.cable_route_mode == "sine":
            return sample_sine_overlay_path(
                waypoints_xy,
                step_m,
                amplitudes_m=self.scenario.environment.sine_amplitudes_m,
                wavelengths_m=self.scenario.environment.sine_wavelengths_m,
            )
        return waypoints_xy.copy()

    def _nominal_route_reference(self, position_xy_m: np.ndarray) -> tuple:
        """返回当前位置在名义路线上的最近点、切向与距离。"""
        best_point, best_tangent, best_distance, _, _ = nearest_point_on_polyline(position_xy_m, self.nominal_route_lookup)
        return best_point, best_tangent, best_distance

    def _crossing_angle_deg(self, zigzag_width_m: float) -> float:
        """根据当前之字形宽度估计交叉入射角。"""
        lookahead_distance_m = max(2.0 * self.scenario.vehicle.min_turning_radius_m, 10.0)
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

    def _spiral_guidance(self, pose: Pose, behavior, time_s: float) -> tuple:
        """在失锁恢复时生成逐步扩大的螺旋搜索航向。"""
        if self.last_mode != TrackingMode.SPIRAL_SEARCH or self.spiral_start_time_s is None:
            self.spiral_start_time_s = time_s
        elapsed_s = max(time_s - self.spiral_start_time_s, 0.0)
        commanded_turn_radius_m = min(
            self.scenario.vehicle.min_turning_radius_m + self.scenario.tracking.spiral_radius_growth_mps * elapsed_s,
            self.scenario.tracking.spiral_max_radius_m,
        )
        yaw_rate_deg_s = float(np.rad2deg(max(behavior.speed_mps, 1e-6) / max(commanded_turn_radius_m, 1e-6)))
        yaw_rate_deg_s *= self.leg_sign
        desired_heading_deg = wrap_angle_deg(pose.heading_deg + yaw_rate_deg_s * self.scenario.dt_s)
        return desired_heading_deg, yaw_rate_deg_s, commanded_turn_radius_m

    def update(self, pose: Pose, perception: PerceptionState) -> GuidanceCommand:
        """结合感知状态与行为树输出当前控制指令。"""
        if self.scenario.tracking.use_nominal_route_prior:
            nominal_point_xy, nominal_tangent_xy, nominal_distance_m = self._nominal_route_reference(pose.position_ned_m[:2])
            nominal_route_heading_deg = heading_from_direction_xy(nominal_tangent_xy)
            fused_heading_deg = perception.fused_heading_deg if perception.fused_heading_deg is not None else nominal_route_heading_deg
        else:
            nominal_point_xy = pose.position_ned_m[:2].copy()
            nominal_distance_m = 0.0
            # Deployment mode must not use any cable-angle prior.  Start from
            # the vehicle's current heading and let the perception stack build
            # a measurement-derived estimate from peak geometry.
            nominal_route_heading_deg = pose.heading_deg
            fused_heading_deg = perception.fused_heading_deg if perception.fused_heading_deg is not None else pose.heading_deg

        # --- Peak-triggered leg flip ---
        if perception.peak_detected and not perception.safe_lock_active:
            self.leg_sign *= -1.0
            self.last_leg_flip_time_s = perception.time_s

        intercept_vector_xy = nominal_point_xy - pose.position_ned_m[:2]
        intercept_heading_deg = heading_from_direction_xy(intercept_vector_xy) if np.linalg.norm(intercept_vector_xy) > 1e-6 else nominal_route_heading_deg
        behavior = self.behavior_tree.evaluate(
            BehaviorContext(
                time_s=perception.time_s,
                nominal_heading_deg=nominal_route_heading_deg,
                intercept_heading_deg=intercept_heading_deg,
                nominal_distance_m=nominal_distance_m,
                confidence=perception.confidence,
                has_detection_history=perception.last_detection_age_s < 1e8,
                last_detection_age_s=perception.last_detection_age_s,
                fused_heading_deg=fused_heading_deg,
                blind_heading_deg=None if not self.scenario.tracking.use_nominal_route_prior else perception.blind_heading_deg,
                sonar_status=perception.sonar_status,
                weak_signal_flag=perception.weak_signal_flag,
                safe_lock_active=perception.safe_lock_active,
                peak_detected=perception.peak_detected,
                zigzag_width_m=perception.zigzag_width_m,
                high_confidence_threshold=self.scenario.tracking.high_confidence_threshold,
                low_confidence_threshold=self.scenario.tracking.low_confidence_threshold,
                lost_timeout_s=self.scenario.tracking.lost_timeout_s,
                guidance_memory_timeout_s=self.scenario.tracking.guidance_memory_timeout_s,
                consecutive_miss_threshold=self.scenario.tracking.consecutive_miss_threshold,
                spiral_entry_window_s=self.scenario.tracking.spiral_entry_window_s,
                search_speed_mps=self.scenario.vehicle.search_speed_mps,
                cruise_speed_mps=self.scenario.vehicle.cruise_speed_mps,
                guidance_source=perception.guidance_source,
                fit_residual_m=perception.fit_result.residual_m,
                deployment_heading_confidence=perception.deployment_heading_confidence,
                tracking_maturity=perception.tracking_maturity,
                safe_lock_criterion_b_active=perception.safe_lock_criterion_b_active,
                deployment_hold_maturity_threshold=self.scenario.tracking.deployment_hold_maturity_threshold,
                deployment_lost_timeout_high_maturity_multiplier=self.scenario.tracking.deployment_lost_timeout_high_maturity_multiplier,
                deployment_mode=not self.scenario.tracking.use_nominal_route_prior,
                deployment_reacquire_required=perception.deployment_reacquire_required,
            )
        )

        # --- Time-based leg flip for SEARCH/LOST/SPIRAL modes ---
        if behavior.mode in {BehaviorMode.SEARCH, BehaviorMode.APPROACH, BehaviorMode.LOST, BehaviorMode.SPIRAL_SEARCH} and perception.time_s - self.last_leg_flip_time_s >= self.scenario.tracking.search_leg_time_s:
            self.leg_sign *= -1.0
            self.last_leg_flip_time_s = perception.time_s

        # --- Task 4: Watchdog Leg Flip (Anti-Escape Safety Lock) ---
        # If no leg flip for too long, force a turn to prevent AUV from flying away.
        leg_timeout_s = max(
            behavior.zigzag_width_m / max(behavior.speed_mps, 0.3) * 1.5,
            5.0
        )
        if perception.time_s - self.last_leg_flip_time_s >= leg_timeout_s:
            self.leg_sign *= -1.0
            self.last_leg_flip_time_s = perception.time_s

        # --- Task 5: Inverse Confidence Zigzag Width Mapping ---
        # Low confidence -> large width (wide sweeping search)
        # High confidence -> small width (tight cable following)
        confidence = perception.confidence
        max_width = self.scenario.tracking.max_zigzag_width_m
        min_width = self.scenario.tracking.min_zigzag_width_m
        inverse_width = max_width - (max_width - min_width) * confidence
        adjusted_zigzag_width = float(np.clip(inverse_width, min_width, max_width))

        # --- Task 6: Force minimum crossing angle during initialization ---
        # Before magnetic fit is mature, force a probing crossing angle even if force_centerline is True.
        effective_force_centerline = behavior.force_centerline
        magnetic_fit_ready = perception.fit_result.origin_xy_m is not None and len(perception.estimated_path_points_xy_m) >= 3
        if not effective_force_centerline:
            crossing_angle_deg = self._crossing_angle_deg(adjusted_zigzag_width)
        elif not magnetic_fit_ready:
            # Initialization phase: force at least 15° probing angle
            crossing_angle_deg = max(self._crossing_angle_deg(adjusted_zigzag_width), 15.0)
            effective_force_centerline = False
        else:
            crossing_angle_deg = 0.0

        desired_heading_deg = wrap_angle_deg(behavior.base_heading_deg + self.leg_sign * crossing_angle_deg)
        if perception.safe_lock_active:
            desired_heading_deg = wrap_angle_deg(behavior.base_heading_deg)

        # --- Task 2: Startup Swing (Large Amplitude) ---
        # If bootstrap not ready (< 3 fit points), force large crossing angle
        if not magnetic_fit_ready and behavior.mode in {BehaviorMode.APPROACH, BehaviorMode.SEARCH}:
            crossing_angle_deg = max(crossing_angle_deg, 45.0)
            desired_heading_deg = wrap_angle_deg(behavior.base_heading_deg + self.leg_sign * crossing_angle_deg)

        mode = TrackingMode(behavior.mode.value)
        if mode == TrackingMode.HOLD:
            # Don't lock to base_heading if bootstrap not ready
            if not magnetic_fit_ready:
                crossing_angle_deg = max(crossing_angle_deg, 15.0)
                desired_heading_deg = wrap_angle_deg(behavior.base_heading_deg + self.leg_sign * crossing_angle_deg)
            else:
                desired_heading_deg = wrap_angle_deg(behavior.base_heading_deg)

        if mode == TrackingMode.SPIRAL_SEARCH:
            desired_heading_deg, yaw_rate_deg_s, turning_radius_from_command = self._spiral_guidance(pose, behavior, perception.time_s)
        else:
            if mode == TrackingMode.TURN:
                desired_heading_deg = wrap_angle_deg(behavior.base_heading_deg + self.leg_sign * crossing_angle_deg)
            yaw_rate_deg_s = self._limited_yaw_rate_deg_s(behavior.speed_mps, desired_heading_deg, pose.heading_deg)
            turning_radius_from_command = float("inf")
            if abs(yaw_rate_deg_s) > 1e-6:
                turning_radius_from_command = max(behavior.speed_mps, 1e-6) / max(np.deg2rad(abs(yaw_rate_deg_s)), 1e-6)

        if mode != TrackingMode.SPIRAL_SEARCH:
            self.spiral_start_time_s = None

        self.last_mode = mode

        return GuidanceCommand(
            desired_heading_deg=desired_heading_deg,
            speed_mps=behavior.speed_mps,
            mode=mode,
            guidance_source=behavior.guidance_source,
            commanded_turn_radius_m=turning_radius_from_command,
            yaw_rate_deg_s=yaw_rate_deg_s,
            safe_lock_active=perception.safe_lock_active,
            zigzag_width_m=behavior.zigzag_width_m,
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