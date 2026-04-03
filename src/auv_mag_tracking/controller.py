"""Zig-zag controller and simple AUV kinematics."""

from dataclasses import dataclass
from enum import Enum
from typing import Optional

import numpy as np

from .config import ScenarioConfig
from .math_utils import Pose, closest_point_on_segment, heading_from_direction_xy, smallest_angle_error_deg, wrap_angle_deg
from .perception import PerceptionState


class TrackingMode(str, Enum):
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    TURN = "TURN"
    HOLD = "HOLD"
    LOST = "LOST"


@dataclass
class GuidanceCommand:
    desired_heading_deg: float
    speed_mps: float
    mode: TrackingMode


class ZigZagController:
    def __init__(self, scenario: ScenarioConfig) -> None:
        self.scenario = scenario
        self.leg_sign = 1.0
        self.last_leg_flip_time_s = 0.0

    def _nominal_route_reference(self, position_xy_m: np.ndarray) -> tuple:
        waypoints = np.asarray(self.scenario.environment.cable_waypoints_xy_m, dtype=float)
        best_point = waypoints[0]
        best_tangent = waypoints[1] - waypoints[0]
        best_distance = float("inf")
        for start_xy, end_xy in zip(waypoints[:-1], waypoints[1:]):
            nearest_point_xy, _ = closest_point_on_segment(position_xy_m, start_xy, end_xy)
            distance = float(np.linalg.norm(position_xy_m - nearest_point_xy))
            if distance < best_distance:
                best_distance = distance
                best_point = nearest_point_xy
                best_tangent = end_xy - start_xy
        tangent_norm = np.linalg.norm(best_tangent)
        if tangent_norm < 1e-9:
            best_tangent = np.array([1.0, 0.0], dtype=float)
        else:
            best_tangent = best_tangent / tangent_norm
        return best_point, best_tangent, best_distance

    def update(self, pose: Pose, perception: PerceptionState) -> GuidanceCommand:
        route_heading_deg = self.scenario.environment.nominal_route_heading_deg
        if perception.line_heading_deg is not None and perception.confidence >= self.scenario.tracking.low_confidence_threshold:
            route_heading_deg = perception.line_heading_deg

        has_detection_history = perception.last_detection_age_s < 1e8
        nominal_point_xy, nominal_tangent_xy, nominal_distance_m = self._nominal_route_reference(
            pose.position_ned_m[:2]
        )
        nominal_route_heading_deg = heading_from_direction_xy(nominal_tangent_xy)

        if perception.peak_detected:
            self.leg_sign *= -1.0
            self.last_leg_flip_time_s = perception.time_s
            desired_heading_deg = wrap_angle_deg(
                route_heading_deg + self.leg_sign * self.scenario.tracking.approach_angle_deg
            )
            return GuidanceCommand(
                desired_heading_deg=desired_heading_deg,
                speed_mps=self.scenario.vehicle.cruise_speed_mps,
                mode=TrackingMode.TURN,
            )

        if not has_detection_history:
            acquisition_band_m = 3.0
            if nominal_distance_m > acquisition_band_m:
                intercept_vector_xy = nominal_point_xy - pose.position_ned_m[:2]
                intercept_heading_deg = heading_from_direction_xy(intercept_vector_xy)
                return GuidanceCommand(
                    desired_heading_deg=wrap_angle_deg(intercept_heading_deg),
                    speed_mps=self.scenario.vehicle.search_speed_mps,
                    mode=TrackingMode.SEARCH,
                )
            route_heading_deg = nominal_route_heading_deg

        hold_ready = (
            perception.line_heading_deg is not None
            and np.isfinite(perception.fit_result.residual_m)
            and perception.fit_result.residual_m <= 15.0
            and perception.last_detection_age_s <= 0.75 * self.scenario.tracking.lost_timeout_s
        )

        if perception.confidence >= self.scenario.tracking.high_confidence_threshold and hold_ready:
            mode = TrackingMode.HOLD
            speed_mps = self.scenario.vehicle.cruise_speed_mps
        elif perception.confidence >= self.scenario.tracking.low_confidence_threshold:
            mode = TrackingMode.APPROACH
            speed_mps = 0.95 * self.scenario.vehicle.cruise_speed_mps
        else:
            if perception.last_detection_age_s > self.scenario.tracking.lost_timeout_s:
                mode = TrackingMode.LOST
            else:
                mode = TrackingMode.SEARCH
            speed_mps = self.scenario.vehicle.search_speed_mps
            if perception.time_s - self.last_leg_flip_time_s >= self.scenario.tracking.search_leg_time_s:
                self.leg_sign *= -1.0
                self.last_leg_flip_time_s = perception.time_s

        search_bias_deg = 15.0 if mode in {TrackingMode.SEARCH, TrackingMode.LOST} else 0.0
        desired_heading_deg = wrap_angle_deg(
            route_heading_deg + self.leg_sign * (self.scenario.tracking.approach_angle_deg + search_bias_deg)
        )
        return GuidanceCommand(desired_heading_deg=desired_heading_deg, speed_mps=speed_mps, mode=mode)


def apply_attitude_profile(pose: Pose, scenario: ScenarioConfig, time_s: float) -> None:
    vehicle = scenario.vehicle
    pose.pitch_deg = vehicle.pitch_amplitude_deg * np.sin(2.0 * np.pi * vehicle.pitch_frequency_hz * time_s)
    pose.roll_deg = vehicle.roll_amplitude_deg * np.sin(2.0 * np.pi * vehicle.roll_frequency_hz * time_s)


def propagate_vehicle(pose: Pose, command: GuidanceCommand, scenario: ScenarioConfig, seabed_depth_m: float, dt_s: float) -> Pose:
    updated_pose = pose.copy()
    heading_error_deg = smallest_angle_error_deg(command.desired_heading_deg, updated_pose.heading_deg)
    max_heading_step_deg = scenario.vehicle.max_yaw_rate_deg_s * dt_s
    heading_step_deg = np.clip(heading_error_deg, -max_heading_step_deg, max_heading_step_deg)
    updated_pose.heading_deg = wrap_angle_deg(updated_pose.heading_deg + heading_step_deg)
    updated_pose.speed_mps = command.speed_mps

    heading_rad = np.deg2rad(updated_pose.heading_deg)
    north_rate = updated_pose.speed_mps * np.cos(heading_rad)
    east_rate = updated_pose.speed_mps * np.sin(heading_rad)
    updated_pose.position_ned_m[0] += north_rate * dt_s
    updated_pose.position_ned_m[1] += east_rate * dt_s
    updated_pose.position_ned_m[2] = seabed_depth_m - scenario.vehicle.altitude_above_seabed_m
    return updated_pose
