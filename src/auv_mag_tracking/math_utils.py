"""Core math utilities for frame transforms, path geometry and magnetic field evaluation."""

from dataclasses import dataclass
from typing import Iterable, Tuple, Union

import numpy as np
try:
    from scipy.interpolate import CubicSpline
except ModuleNotFoundError:
    CubicSpline = None


EPSILON = 1e-12
MU0_OVER_4PI = 1e-7


@dataclass
class Pose:
    """表示 AUV 在 NED 坐标系下的姿态与运动状态。"""

    position_ned_m: np.ndarray
    heading_deg: float
    pitch_deg: float
    roll_deg: float
    speed_mps: float = 0.0

    def copy(self) -> "Pose":
        return Pose(
            position_ned_m=self.position_ned_m.copy(),
            heading_deg=float(self.heading_deg),
            pitch_deg=float(self.pitch_deg),
            roll_deg=float(self.roll_deg),
            speed_mps=float(self.speed_mps),
        )


@dataclass
class PolylineProjectionCache:
    """缓存折线路径的几何信息，便于快速执行投影与最近点查询。"""

    polyline_xy: np.ndarray
    segment_starts_xy: np.ndarray
    segment_vectors_xy: np.ndarray
    segment_lengths_m: np.ndarray
    segment_lengths_sq_m2: np.ndarray
    cumulative_length_m: np.ndarray
    segment_tangents_xy: np.ndarray


def as_vector(values: Iterable[float]) -> np.ndarray:
    """将任意可迭代数值转换为一维浮点向量。"""
    return np.asarray(list(values), dtype=float)


def wrap_angle_deg(angle_deg: float) -> float:
    """将角度规范化到 [−180, 180) 区间。"""
    return (angle_deg + 180.0) % 360.0 - 180.0


def smallest_angle_error_deg(target_deg: float, current_deg: float) -> float:
    """计算目标角与当前角之间的最小有符号误差。"""
    return wrap_angle_deg(target_deg - current_deg)


def rotation_matrix_body_to_ned(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Return the aerospace ZYX body-to-NED direction cosine matrix."""
    roll = np.deg2rad(roll_deg)
    pitch = np.deg2rad(pitch_deg)
    yaw = np.deg2rad(yaw_deg)

    cr = np.cos(roll)
    sr = np.sin(roll)
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cy = np.cos(yaw)
    sy = np.sin(yaw)

    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def rotation_matrix_sensor_to_body(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """构造传感器坐标系到机体系的静态安装旋转矩阵。"""
    return rotation_matrix_body_to_ned(roll_deg, pitch_deg, yaw_deg)


def body_to_ned(vector_body: Iterable[float], roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """将机体系向量旋转到 NED 坐标系。"""
    return rotation_matrix_body_to_ned(roll_deg, pitch_deg, yaw_deg) @ as_vector(vector_body)


def ned_to_body(vector_ned: Iterable[float], roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """将 NED 向量旋转回机体系。"""
    rotation = rotation_matrix_body_to_ned(roll_deg, pitch_deg, yaw_deg)
    return rotation.T @ as_vector(vector_ned)


def sensor_to_body(vector_sensor: Iterable[float], sensor_to_body_matrix: np.ndarray) -> np.ndarray:
    """将传感器坐标系向量映射到机体系。"""
    return sensor_to_body_matrix @ as_vector(vector_sensor)


def body_to_sensor(vector_body: Iterable[float], sensor_to_body_matrix: np.ndarray) -> np.ndarray:
    """将机体系向量映射到传感器坐标系。"""
    return sensor_to_body_matrix.T @ as_vector(vector_body)


def norm(vector: Iterable[float]) -> float:
    """返回向量的欧几里得范数。"""
    return float(np.linalg.norm(as_vector(vector)))


def unit(vector: Iterable[float]) -> np.ndarray:
    """返回单位化后的向量；若长度过小则返回零向量。"""
    vector_array = as_vector(vector)
    magnitude = np.linalg.norm(vector_array)
    if magnitude < EPSILON:
        return np.zeros_like(vector_array)
    return vector_array / magnitude


def heading_from_direction_xy(direction_xy: Iterable[float]) -> float:
    """根据平面方向向量计算航向角。"""
    direction = as_vector(direction_xy)
    return float(np.rad2deg(np.arctan2(direction[1], direction[0])))


def body_xy_to_ned(relative_xy_m: Iterable[float], heading_deg: float) -> np.ndarray:
    """将机体系平面位移旋转到 NED 平面。"""
    relative_xy = as_vector(relative_xy_m)
    heading_rad = np.deg2rad(heading_deg)
    rotation = np.array(
        [
            [np.cos(heading_rad), -np.sin(heading_rad)],
            [np.sin(heading_rad), np.cos(heading_rad)],
        ],
        dtype=float,
    )
    return rotation @ relative_xy


def ned_xy_to_body(relative_xy_m: Iterable[float], heading_deg: float) -> np.ndarray:
    """将 NED 平面位移旋转到机体系。"""
    relative_xy = as_vector(relative_xy_m)
    heading_rad = np.deg2rad(heading_deg)
    rotation = np.array(
        [
            [np.cos(heading_rad), np.sin(heading_rad)],
            [-np.sin(heading_rad), np.cos(heading_rad)],
        ],
        dtype=float,
    )
    return rotation @ relative_xy


def finite_wire_field_nT(
    point_ned_m: Iterable[float],
    segment_start_ned_m: Iterable[float],
    segment_end_ned_m: Iterable[float],
    current_a: float,
) -> np.ndarray:
    """计算有限长度直导线在观测点处产生的磁场强度。"""
    """Compute the magnetic field of a finite straight wire segment in nT.

    The formulation is the compact Biot-Savart vector expression for a finite
    conductor. Phase 1 uses this as the magnetic core model and keeps absolute
    current magnitude configurable because the standards constrain detector
    sensitivity and survey semantics more strongly than a single canonical load.
    """
    point = as_vector(point_ned_m)
    start = as_vector(segment_start_ned_m)
    end = as_vector(segment_end_ned_m)

    r1 = point - start
    r2 = point - end
    cross_term = np.cross(r1, r2)
    cross_norm = float(np.sqrt(np.dot(cross_term, cross_term)))
    if cross_norm < EPSILON:
        return np.zeros(3, dtype=float)

    r1_norm = float(np.sqrt(np.dot(r1, r1)))
    r2_norm = float(np.sqrt(np.dot(r2, r2)))
    denominator = r1_norm * r2_norm * (r1_norm * r2_norm + np.dot(r1, r2))
    if abs(denominator) < EPSILON:
        return np.zeros(3, dtype=float)

    field_tesla = MU0_OVER_4PI * current_a * cross_term / denominator
    return field_tesla * 1e9


def batch_finite_wire_field_nT(
    point_ned_m: Iterable[float],
    segment_starts_ned_m: np.ndarray,
    segment_ends_ned_m: np.ndarray,
    current_a: float,
) -> np.ndarray:
    """批量计算多个观测点对应的有限导线磁场。"""
    point = as_vector(point_ned_m)
    starts = np.asarray(segment_starts_ned_m, dtype=float)
    ends = np.asarray(segment_ends_ned_m, dtype=float)
    if starts.size == 0 or ends.size == 0 or abs(current_a) < EPSILON:
        return np.zeros(3, dtype=float)

    r1 = point[None, :] - starts
    r2 = point[None, :] - ends
    cross_terms = np.cross(r1, r2)
    cross_norm_sq = np.einsum("ij,ij->i", cross_terms, cross_terms)
    r1_norm_sq = np.einsum("ij,ij->i", r1, r1)
    r2_norm_sq = np.einsum("ij,ij->i", r2, r2)
    r1_norm = np.sqrt(np.maximum(r1_norm_sq, 0.0))
    r2_norm = np.sqrt(np.maximum(r2_norm_sq, 0.0))
    dot_products = np.einsum("ij,ij->i", r1, r2)
    denominator = r1_norm * r2_norm * (r1_norm * r2_norm + dot_products)

    valid_mask = (cross_norm_sq > EPSILON * EPSILON) & (np.abs(denominator) > EPSILON)
    if not np.any(valid_mask):
        return np.zeros(3, dtype=float)

    field_terms = np.zeros_like(cross_terms)
    field_terms[valid_mask] = (MU0_OVER_4PI * current_a) * cross_terms[valid_mask] / denominator[valid_mask, None]
    return np.sum(field_terms, axis=0) * 1e9


def project_point_to_line(point_xy: Iterable[float], origin_xy: Iterable[float], direction_xy: Iterable[float]) -> np.ndarray:
    """将点投影到给定直线上。"""
    point = as_vector(point_xy)
    origin = as_vector(origin_xy)
    direction = unit(direction_xy)
    if np.linalg.norm(direction) < EPSILON:
        return origin.copy()
    projection_length = float(np.dot(point - origin, direction))
    return origin + projection_length * direction


def polyline_length(points_xy: np.ndarray) -> float:
    """计算折线总长度。"""
    if len(points_xy) < 2:
        return 0.0
    deltas = np.diff(points_xy, axis=0)
    return float(np.sum(np.linalg.norm(deltas, axis=1)))


def closest_point_on_segment(point_xy: np.ndarray, start_xy: np.ndarray, end_xy: np.ndarray) -> Tuple[np.ndarray, float]:
    """求点到线段的最近点及参数位置。"""
    segment = end_xy - start_xy
    length_sq = float(np.dot(segment, segment))
    if length_sq < EPSILON:
        return start_xy.copy(), 0.0
    parameter = float(np.dot(point_xy - start_xy, segment) / length_sq)
    parameter = max(0.0, min(1.0, parameter))
    return start_xy + parameter * segment, parameter


def cumulative_arc_length(points_xy: np.ndarray) -> np.ndarray:
    """计算折线上各顶点的累计弧长。"""
    if len(points_xy) == 0:
        return np.zeros(0, dtype=float)
    arc_length = np.zeros(len(points_xy), dtype=float)
    if len(points_xy) > 1:
        arc_length[1:] = np.cumsum(np.linalg.norm(np.diff(points_xy, axis=0), axis=1))
    return arc_length


def build_polyline_projection_cache(polyline_xy: np.ndarray) -> PolylineProjectionCache:
    """为折线路径构造投影缓存。"""
    polyline_xy = np.asarray(polyline_xy, dtype=float)
    if len(polyline_xy) < 2:
        raise ValueError("Polyline requires at least two points")

    segment_starts_xy = polyline_xy[:-1]
    segment_vectors_xy = polyline_xy[1:] - polyline_xy[:-1]
    segment_lengths_sq_m2 = np.einsum("ij,ij->i", segment_vectors_xy, segment_vectors_xy)
    segment_lengths_m = np.sqrt(np.maximum(segment_lengths_sq_m2, 0.0))
    cumulative_length_m = np.zeros(len(polyline_xy), dtype=float)
    cumulative_length_m[1:] = np.cumsum(segment_lengths_m)

    segment_tangents_xy = np.zeros_like(segment_vectors_xy)
    valid_mask = segment_lengths_m > EPSILON
    if np.any(valid_mask):
        segment_tangents_xy[valid_mask] = segment_vectors_xy[valid_mask] / segment_lengths_m[valid_mask, None]
        first_valid_tangent_xy = segment_tangents_xy[int(np.argmax(valid_mask))].copy()
        segment_tangents_xy[~valid_mask] = first_valid_tangent_xy
    else:
        segment_tangents_xy[:] = np.array([1.0, 0.0], dtype=float)

    return PolylineProjectionCache(
        polyline_xy=polyline_xy,
        segment_starts_xy=segment_starts_xy,
        segment_vectors_xy=segment_vectors_xy,
        segment_lengths_m=segment_lengths_m,
        segment_lengths_sq_m2=segment_lengths_sq_m2,
        cumulative_length_m=cumulative_length_m,
        segment_tangents_xy=segment_tangents_xy,
    )


def sample_spline_path(waypoints_xy: np.ndarray, step_m: float) -> np.ndarray:
    """基于样条插值对航点序列进行平滑采样。"""
    arc_length = cumulative_arc_length(waypoints_xy)
    if arc_length[-1] < EPSILON:
        return waypoints_xy.copy()
    sample_count = max(8, int(np.ceil(arc_length[-1] / max(step_m, 0.5))) + 1)
    sample_s = np.linspace(0.0, arc_length[-1], sample_count)
    if CubicSpline is None:
        return np.column_stack(
            (
                np.interp(sample_s, arc_length, waypoints_xy[:, 0]),
                np.interp(sample_s, arc_length, waypoints_xy[:, 1]),
            )
        )
    spline_x = CubicSpline(arc_length, waypoints_xy[:, 0], bc_type="natural")
    spline_y = CubicSpline(arc_length, waypoints_xy[:, 1], bc_type="natural")
    return np.column_stack((spline_x(sample_s), spline_y(sample_s)))


def sample_serpentine_path(
    turn_count: int,
    straight_length_m: float,
    lane_spacing_m: float,
    turn_radius_m: float,
    step_m: float,
) -> np.ndarray:
    """Sample a smooth back-and-forth cable with constant-radius U-turns.

    When lane spacing exceeds ``2 * turn_radius_m``, each turnaround is built
    from two quarter-circle arcs and one straight connector.  This preserves C1
    continuity and keeps the minimum curvature radius equal to ``turn_radius_m``
    instead of introducing a hidden corner between lanes.
    """
    turn_count = max(1, int(turn_count))
    straight_length_m = max(float(straight_length_m), 2.0 * float(turn_radius_m))
    lane_spacing_m = max(float(lane_spacing_m), 2.0 * float(turn_radius_m))
    turn_radius_m = max(float(turn_radius_m), EPSILON)
    step_m = max(float(step_m), 0.5)
    half_length_m = 0.5 * straight_length_m

    points = []

    def append_line(start_xy: np.ndarray, end_xy: np.ndarray) -> None:
        distance_m = float(np.linalg.norm(end_xy - start_xy))
        count = max(2, int(np.ceil(distance_m / step_m)) + 1)
        for idx, alpha in enumerate(np.linspace(0.0, 1.0, count)):
            if points and idx == 0:
                continue
            points.append(start_xy + alpha * (end_xy - start_xy))

    def append_arc(center_xy: np.ndarray, start_rad: float, end_rad: float) -> None:
        arc_length_m = abs(end_rad - start_rad) * turn_radius_m
        count = max(8, int(np.ceil(arc_length_m / step_m)) + 1)
        for idx, theta in enumerate(np.linspace(start_rad, end_rad, count)):
            if points and idx == 0:
                continue
            points.append(center_xy + turn_radius_m * np.array([np.cos(theta), np.sin(theta)], dtype=float))

    lane_count = turn_count + 1
    for lane_idx in range(lane_count):
        lane_y_m = -lane_idx * lane_spacing_m
        if lane_idx % 2 == 0:
            line_start = np.array([-half_length_m, lane_y_m], dtype=float)
            line_end = np.array([half_length_m, lane_y_m], dtype=float)
        else:
            line_start = np.array([half_length_m, lane_y_m], dtype=float)
            line_end = np.array([-half_length_m, lane_y_m], dtype=float)
        append_line(line_start, line_end)

        if lane_idx == turn_count:
            break
        connector_m = max(0.0, lane_spacing_m - 2.0 * turn_radius_m)
        if lane_idx % 2 == 0:
            upper_center = np.array([half_length_m, lane_y_m - turn_radius_m], dtype=float)
            lower_center = np.array([half_length_m, lane_y_m - lane_spacing_m + turn_radius_m], dtype=float)
            append_arc(upper_center, 0.5 * np.pi, 0.0)
            if connector_m > EPSILON:
                append_line(
                    np.array([half_length_m + turn_radius_m, lane_y_m - turn_radius_m], dtype=float),
                    np.array([half_length_m + turn_radius_m, lane_y_m - lane_spacing_m + turn_radius_m], dtype=float),
                )
            append_arc(lower_center, 0.0, -0.5 * np.pi)
        else:
            upper_center = np.array([-half_length_m, lane_y_m - turn_radius_m], dtype=float)
            lower_center = np.array([-half_length_m, lane_y_m - lane_spacing_m + turn_radius_m], dtype=float)
            append_arc(upper_center, 0.5 * np.pi, np.pi)
            if connector_m > EPSILON:
                append_line(
                    np.array([-half_length_m - turn_radius_m, lane_y_m - turn_radius_m], dtype=float),
                    np.array([-half_length_m - turn_radius_m, lane_y_m - lane_spacing_m + turn_radius_m], dtype=float),
                )
            append_arc(lower_center, np.pi, 1.5 * np.pi)

    return np.asarray(points, dtype=float)


def sample_tightening_arc_path(
    initial_straight_length_m: float,
    turn_angle_deg: float,
    radius_m: float,
    step_m: float,
) -> np.ndarray:
    """Sample an initial straight run followed by one constant-radius left turn.

    The cable starts at ``(-initial_straight_length_m, 0)``, runs east to the
    origin, then sweeps a single circular arc of radius ``radius_m`` through a
    total heading change of ``turn_angle_deg`` (turning left, centre at
    ``(0, radius_m)``).  The minimum curvature radius of the whole route equals
    ``radius_m`` exactly, so shrinking ``radius_m`` is a clean knob for probing
    the curvature a tracker can sustain.  Geometry is C1 continuous at the
    straight/arc junction.
    """
    initial_straight_length_m = max(float(initial_straight_length_m), 0.0)
    radius_m = max(float(radius_m), EPSILON)
    turn_angle_rad = np.deg2rad(float(turn_angle_deg))
    step_m = max(float(step_m), 0.5)

    points = []

    def append_line(start_xy: np.ndarray, end_xy: np.ndarray) -> None:
        distance_m = float(np.linalg.norm(end_xy - start_xy))
        count = max(2, int(np.ceil(distance_m / step_m)) + 1)
        for idx, alpha in enumerate(np.linspace(0.0, 1.0, count)):
            if points and idx == 0:
                continue
            points.append(start_xy + alpha * (end_xy - start_xy))

    straight_start = np.array([-initial_straight_length_m, 0.0], dtype=float)
    arc_origin = np.array([0.0, 0.0], dtype=float)
    if initial_straight_length_m > EPSILON:
        append_line(straight_start, arc_origin)
    else:
        points.append(arc_origin.copy())

    center_xy = np.array([0.0, radius_m], dtype=float)
    arc_length_m = abs(turn_angle_rad) * radius_m
    count = max(8, int(np.ceil(arc_length_m / step_m)) + 1)
    start_theta = -0.5 * np.pi
    for idx, theta in enumerate(np.linspace(start_theta, start_theta + turn_angle_rad, count)):
        if points and idx == 0:
            continue
        points.append(center_xy + radius_m * np.array([np.cos(theta), np.sin(theta)], dtype=float))

    return np.asarray(points, dtype=float)


def sample_sine_overlay_path(
    waypoints_xy: np.ndarray,
    step_m: float,
    amplitudes_m: Tuple[float, ...],
    wavelengths_m: Tuple[float, ...],
) -> np.ndarray:
    """在平滑路径上叠加正弦横向扰动。"""
    base_path = sample_spline_path(waypoints_xy, step_m)
    arc_length = cumulative_arc_length(base_path)
    if len(base_path) < 2:
        return base_path
    tangents = np.gradient(base_path, axis=0)
    tangent_norms = np.linalg.norm(tangents, axis=1, keepdims=True)
    tangent_norms = np.maximum(tangent_norms, EPSILON)
    tangents = tangents / tangent_norms
    normals = np.column_stack((-tangents[:, 1], tangents[:, 0]))
    lateral_offset = np.zeros(len(base_path), dtype=float)
    for amplitude_m, wavelength_m in zip(amplitudes_m, wavelengths_m):
        effective_wavelength = max(wavelength_m, 1.0)
        lateral_offset += amplitude_m * np.sin(2.0 * np.pi * arc_length / effective_wavelength)
    return base_path + normals * lateral_offset[:, None]


def build_nominal_route_xy(environment_config) -> np.ndarray:
    """按环境配置的路线模式生成名义电缆折线（先验参考的单一来源）。

    供 controller 与 perception 共享，避免两处各自复制采样逻辑而漂移。线性模式
    返回原始 waypoints（与控制器 / 感知历史行为字节一致），spline/sine/serpentine 模式按
    ``field_segment_length_m`` 半步采样。
    """
    waypoints_xy = np.asarray(environment_config.cable_waypoints_xy_m, dtype=float)
    step_m = max(environment_config.field_segment_length_m * 0.5, 1.0)
    if environment_config.cable_route_mode == "spline":
        return sample_spline_path(waypoints_xy, step_m)
    if environment_config.cable_route_mode == "serpentine":
        return sample_serpentine_path(
            turn_count=environment_config.maze_turn_count,
            straight_length_m=environment_config.maze_straight_length_m,
            lane_spacing_m=environment_config.maze_lane_spacing_m,
            turn_radius_m=environment_config.maze_turn_radius_m,
            step_m=step_m,
        )
    if environment_config.cable_route_mode == "tightening_arc":
        return sample_tightening_arc_path(
            initial_straight_length_m=environment_config.arc_initial_straight_length_m,
            turn_angle_deg=environment_config.arc_turn_angle_deg,
            radius_m=environment_config.arc_radius_m,
            step_m=step_m,
        )
    if environment_config.cable_route_mode == "sine":
        return sample_sine_overlay_path(
            waypoints_xy,
            step_m,
            amplitudes_m=environment_config.sine_amplitudes_m,
            wavelengths_m=environment_config.sine_wavelengths_m,
        )
    return waypoints_xy.copy()


def apply_route_prior_pose_error(
    route_xy: np.ndarray,
    translation_xy_m: Iterable[float],
    rotation_deg: float = 0.0,
    scale_xy: Iterable[float] = (1.0, 1.0),
) -> np.ndarray:
    """Apply a pose/shape error to a nominal route prior without changing truth geometry."""
    route = np.asarray(route_xy, dtype=float).copy()
    if route.size == 0:
        return route
    pivot_xy = route[0].copy()
    scale = np.asarray(scale_xy, dtype=float)
    transformed = (route - pivot_xy) * scale[None, :] + pivot_xy
    angle_rad = np.deg2rad(float(rotation_deg))
    rotation = np.array(
        [
            [np.cos(angle_rad), -np.sin(angle_rad)],
            [np.sin(angle_rad), np.cos(angle_rad)],
        ],
        dtype=float,
    )
    transformed = (transformed - pivot_xy) @ rotation.T + pivot_xy
    return transformed + np.asarray(translation_xy_m, dtype=float)


def nearest_point_on_polyline(
    point_xy: np.ndarray,
    polyline_xy: Union[np.ndarray, PolylineProjectionCache],
) -> Tuple[np.ndarray, np.ndarray, float, float, int]:
    """返回点到折线的最近点、切向、距离和进度信息。"""
    cache = polyline_xy if isinstance(polyline_xy, PolylineProjectionCache) else build_polyline_projection_cache(polyline_xy)
    point_xy = np.asarray(point_xy, dtype=float)

    safe_lengths_sq_m2 = np.maximum(cache.segment_lengths_sq_m2, EPSILON)
    parameters = np.einsum(
        "ij,ij->i",
        point_xy[None, :] - cache.segment_starts_xy,
        cache.segment_vectors_xy,
    ) / safe_lengths_sq_m2
    parameters = np.clip(parameters, 0.0, 1.0)
    nearest_points_xy = cache.segment_starts_xy + parameters[:, None] * cache.segment_vectors_xy
    deltas_xy = point_xy[None, :] - nearest_points_xy
    distance_sq_m2 = np.einsum("ij,ij->i", deltas_xy, deltas_xy)
    distance_sq_m2 = np.where(cache.segment_lengths_m > EPSILON, distance_sq_m2, np.inf)

    best_index = int(np.argmin(distance_sq_m2))
    best_point_xy = nearest_points_xy[best_index]
    best_tangent_xy = cache.segment_tangents_xy[best_index]
    best_distance_m = float(np.sqrt(max(float(distance_sq_m2[best_index]), 0.0)))
    best_progress_m = float(cache.cumulative_length_m[best_index] + parameters[best_index] * cache.segment_lengths_m[best_index])
    return best_point_xy.copy(), best_tangent_xy.copy(), best_distance_m, best_progress_m, best_index


def nearest_point_on_polyline_within_progress(
    point_xy: np.ndarray,
    polyline_xy: Union[np.ndarray, PolylineProjectionCache],
    progress_min_m: float,
    progress_max_m: float,
) -> Tuple[np.ndarray, np.ndarray, float, float, int]:
    """Project ``point_xy`` onto the polyline but restrict the search to the
    arc-length window ``[progress_min_m, progress_max_m]``.

    Segments fully outside the window are ignored; the candidate parameter on
    each surviving segment is clipped so the returned point lies inside the
    window. Returns ``(point_xy, tangent_xy, distance_m, progress_m, index)``
    with the same shape as :func:`nearest_point_on_polyline`. Falls back to a
    global projection when the window does not intersect any segment.
    """
    cache = polyline_xy if isinstance(polyline_xy, PolylineProjectionCache) else build_polyline_projection_cache(polyline_xy)
    if progress_max_m <= progress_min_m:
        return nearest_point_on_polyline(point_xy, cache)

    segment_start_progress_m = cache.cumulative_length_m[:-1]
    segment_end_progress_m = cache.cumulative_length_m[1:]
    overlap_mask = (segment_end_progress_m >= progress_min_m) & (segment_start_progress_m <= progress_max_m)
    overlap_mask = overlap_mask & (cache.segment_lengths_m > EPSILON)
    if not np.any(overlap_mask):
        return nearest_point_on_polyline(point_xy, cache)

    point_xy = np.asarray(point_xy, dtype=float)
    candidate_indices = np.where(overlap_mask)[0]
    starts_xy = cache.segment_starts_xy[candidate_indices]
    vectors_xy = cache.segment_vectors_xy[candidate_indices]
    lengths_m = cache.segment_lengths_m[candidate_indices]
    lengths_sq_m2 = np.maximum(cache.segment_lengths_sq_m2[candidate_indices], EPSILON)

    raw_parameters = np.einsum("ij,ij->i", point_xy[None, :] - starts_xy, vectors_xy) / lengths_sq_m2
    raw_parameters = np.clip(raw_parameters, 0.0, 1.0)

    segment_start_window = np.clip(
        (progress_min_m - cache.cumulative_length_m[candidate_indices]) / lengths_m,
        0.0,
        1.0,
    )
    segment_end_window = np.clip(
        (progress_max_m - cache.cumulative_length_m[candidate_indices]) / lengths_m,
        0.0,
        1.0,
    )
    parameters = np.clip(raw_parameters, segment_start_window, segment_end_window)

    nearest_points_xy = starts_xy + parameters[:, None] * vectors_xy
    deltas_xy = point_xy[None, :] - nearest_points_xy
    distance_sq_m2 = np.einsum("ij,ij->i", deltas_xy, deltas_xy)

    local_best = int(np.argmin(distance_sq_m2))
    best_index = int(candidate_indices[local_best])
    best_parameter = float(parameters[local_best])
    best_point_xy = nearest_points_xy[local_best]
    best_tangent_xy = cache.segment_tangents_xy[best_index]
    best_distance_m = float(np.sqrt(max(float(distance_sq_m2[local_best]), 0.0)))
    best_progress_m = float(
        cache.cumulative_length_m[best_index]
        + best_parameter * cache.segment_lengths_m[best_index]
    )
    return best_point_xy.copy(), best_tangent_xy.copy(), best_distance_m, best_progress_m, best_index


def point_on_polyline_at_progress(
    polyline_xy: Union[np.ndarray, PolylineProjectionCache],
    progress_m: float,
) -> np.ndarray:
    """Return the (x, y) point on the polyline at the given arc-length progress.

    The query is clamped to ``[0, total_length]``. Useful for tests and for
    seeding the progress-guard window from a known anchor.
    """
    cache = polyline_xy if isinstance(polyline_xy, PolylineProjectionCache) else build_polyline_projection_cache(polyline_xy)
    total_length_m = float(cache.cumulative_length_m[-1])
    clamped_progress_m = float(np.clip(progress_m, 0.0, total_length_m))
    segment_index = int(np.searchsorted(cache.cumulative_length_m[1:], clamped_progress_m, side="left"))
    segment_index = max(0, min(segment_index, len(cache.segment_lengths_m) - 1))
    segment_length_m = float(cache.segment_lengths_m[segment_index])
    if segment_length_m <= EPSILON:
        return cache.segment_starts_xy[segment_index].copy()
    alpha = (clamped_progress_m - float(cache.cumulative_length_m[segment_index])) / segment_length_m
    alpha = float(np.clip(alpha, 0.0, 1.0))
    return (
        cache.segment_starts_xy[segment_index]
        + alpha * cache.segment_vectors_xy[segment_index]
    ).copy()


def estimate_polyline_curvature(polyline_xy: np.ndarray, index: int) -> float:
    """估计折线在指定顶点处的离散曲率。"""
    if len(polyline_xy) < 3:
        return 0.0
    center_index = int(np.clip(index + 1, 1, len(polyline_xy) - 2))
    previous_xy = polyline_xy[center_index - 1]
    current_xy = polyline_xy[center_index]
    next_xy = polyline_xy[center_index + 1]
    segment_a = current_xy - previous_xy
    segment_b = next_xy - current_xy
    length_a = norm(segment_a)
    length_b = norm(segment_b)
    chord_length = norm(next_xy - previous_xy)
    if min(length_a, length_b, chord_length) < EPSILON:
        return 0.0
    cross_value = abs(segment_a[0] * segment_b[1] - segment_a[1] * segment_b[0])
    return float(2.0 * cross_value / (length_a * length_b * chord_length))
