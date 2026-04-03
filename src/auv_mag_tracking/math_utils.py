"""Core math utilities for frame transforms and magnetic field evaluation."""

from dataclasses import dataclass
from typing import Iterable, Tuple

import numpy as np


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
    cross_norm = np.linalg.norm(cross_term)
    if cross_norm < EPSILON:
        return np.zeros(3, dtype=float)

    r1_norm = np.linalg.norm(r1)
    r2_norm = np.linalg.norm(r2)
    denominator = r1_norm * r2_norm * (r1_norm * r2_norm + np.dot(r1, r2))
    if abs(denominator) < EPSILON:
        return np.zeros(3, dtype=float)

    field_tesla = MU0_OVER_4PI * current_a * cross_term / denominator
    return field_tesla * 1e9


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
