"""Core math utilities for frame transforms, path geometry and magnetic field evaluation."""

from dataclasses import dataclass
from typing import Iterable, Tuple, Union

import numpy as np
from scipy.interpolate import CubicSpline


EPSILON = 1e-12
MU0_OVER_4PI = 1e-7


@dataclass
class Pose:
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
    polyline_xy: np.ndarray
    segment_starts_xy: np.ndarray
    segment_vectors_xy: np.ndarray
    segment_lengths_m: np.ndarray
    segment_lengths_sq_m2: np.ndarray
    cumulative_length_m: np.ndarray
    segment_tangents_xy: np.ndarray


def as_vector(values: Iterable[float]) -> np.ndarray:
    return np.asarray(list(values), dtype=float)


def wrap_angle_deg(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


def smallest_angle_error_deg(target_deg: float, current_deg: float) -> float:
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
    return rotation_matrix_body_to_ned(roll_deg, pitch_deg, yaw_deg)


def body_to_ned(vector_body: Iterable[float], roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    return rotation_matrix_body_to_ned(roll_deg, pitch_deg, yaw_deg) @ as_vector(vector_body)


def ned_to_body(vector_ned: Iterable[float], roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    rotation = rotation_matrix_body_to_ned(roll_deg, pitch_deg, yaw_deg)
    return rotation.T @ as_vector(vector_ned)


def sensor_to_body(vector_sensor: Iterable[float], sensor_to_body_matrix: np.ndarray) -> np.ndarray:
    return sensor_to_body_matrix @ as_vector(vector_sensor)


def body_to_sensor(vector_body: Iterable[float], sensor_to_body_matrix: np.ndarray) -> np.ndarray:
    return sensor_to_body_matrix.T @ as_vector(vector_body)


def norm(vector: Iterable[float]) -> float:
    return float(np.linalg.norm(as_vector(vector)))


def unit(vector: Iterable[float]) -> np.ndarray:
    vector_array = as_vector(vector)
    magnitude = np.linalg.norm(vector_array)
    if magnitude < EPSILON:
        return np.zeros_like(vector_array)
    return vector_array / magnitude


def heading_from_direction_xy(direction_xy: Iterable[float]) -> float:
    direction = as_vector(direction_xy)
    return float(np.rad2deg(np.arctan2(direction[1], direction[0])))


def body_xy_to_ned(relative_xy_m: Iterable[float], heading_deg: float) -> np.ndarray:
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
    point = as_vector(point_xy)
    origin = as_vector(origin_xy)
    direction = unit(direction_xy)
    if np.linalg.norm(direction) < EPSILON:
        return origin.copy()
    projection_length = float(np.dot(point - origin, direction))
    return origin + projection_length * direction


def polyline_length(points_xy: np.ndarray) -> float:
    if len(points_xy) < 2:
        return 0.0
    deltas = np.diff(points_xy, axis=0)
    return float(np.sum(np.linalg.norm(deltas, axis=1)))


def closest_point_on_segment(point_xy: np.ndarray, start_xy: np.ndarray, end_xy: np.ndarray) -> Tuple[np.ndarray, float]:
    segment = end_xy - start_xy
    length_sq = float(np.dot(segment, segment))
    if length_sq < EPSILON:
        return start_xy.copy(), 0.0
    parameter = float(np.dot(point_xy - start_xy, segment) / length_sq)
    parameter = max(0.0, min(1.0, parameter))
    return start_xy + parameter * segment, parameter


def cumulative_arc_length(points_xy: np.ndarray) -> np.ndarray:
    if len(points_xy) == 0:
        return np.zeros(0, dtype=float)
    arc_length = np.zeros(len(points_xy), dtype=float)
    if len(points_xy) > 1:
        arc_length[1:] = np.cumsum(np.linalg.norm(np.diff(points_xy, axis=0), axis=1))
    return arc_length


def build_polyline_projection_cache(polyline_xy: np.ndarray) -> PolylineProjectionCache:
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
    arc_length = cumulative_arc_length(waypoints_xy)
    if arc_length[-1] < EPSILON:
        return waypoints_xy.copy()
    sample_count = max(8, int(np.ceil(arc_length[-1] / max(step_m, 0.5))) + 1)
    sample_s = np.linspace(0.0, arc_length[-1], sample_count)
    spline_x = CubicSpline(arc_length, waypoints_xy[:, 0], bc_type="natural")
    spline_y = CubicSpline(arc_length, waypoints_xy[:, 1], bc_type="natural")
    return np.column_stack((spline_x(sample_s), spline_y(sample_s)))


def sample_sine_overlay_path(
    waypoints_xy: np.ndarray,
    step_m: float,
    amplitudes_m: Tuple[float, ...],
    wavelengths_m: Tuple[float, ...],
) -> np.ndarray:
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


def nearest_point_on_polyline(
    point_xy: np.ndarray,
    polyline_xy: Union[np.ndarray, PolylineProjectionCache],
) -> Tuple[np.ndarray, np.ndarray, float, float, int]:
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


def estimate_polyline_curvature(polyline_xy: np.ndarray, index: int) -> float:
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
