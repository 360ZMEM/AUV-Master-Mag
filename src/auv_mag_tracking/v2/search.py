"""Deterministic prior-corridor coverage search planning."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np

from .config import SearchConfig
from .types import SearchPlan


def _resample_polyline(points_xy_m: np.ndarray, spacing_m: float) -> np.ndarray:
    points = np.asarray(points_xy_m, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or points.shape[0] < 2:
        raise ValueError("prior path must have shape (N, 2), N >= 2")
    segments = np.diff(points, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    if np.any(lengths <= 1e-9):
        keep = np.r_[True, lengths > 1e-9]
        points = points[keep]
        if points.shape[0] < 2:
            raise ValueError("prior path has zero length")
        segments = np.diff(points, axis=0)
        lengths = np.linalg.norm(segments, axis=1)
    cumulative = np.r_[0.0, np.cumsum(lengths)]
    samples = np.arange(0.0, cumulative[-1], spacing_m, dtype=float)
    if samples.size == 0 or samples[-1] < cumulative[-1]:
        samples = np.r_[samples, cumulative[-1]]
    result = np.empty((samples.size, 2), dtype=float)
    for idx, distance in enumerate(samples):
        seg_idx = min(int(np.searchsorted(cumulative, distance, side="right") - 1), lengths.size - 1)
        fraction = (distance - cumulative[seg_idx]) / lengths[seg_idx]
        result[idx] = points[seg_idx] + fraction * segments[seg_idx]
    return result


def _point_in_polygon(point_xy: np.ndarray, polygon_xy: np.ndarray) -> bool:
    x, y = map(float, point_xy)
    polygon = np.asarray(polygon_xy, dtype=float)
    inside = False
    j = polygon.shape[0] - 1
    for i in range(polygon.shape[0]):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        on_edge = abs((xj - xi) * (y - yi) - (yj - yi) * (x - xi)) <= 1e-9
        if on_edge and min(xi, xj) - 1e-9 <= x <= max(xi, xj) + 1e-9 and min(yi, yj) - 1e-9 <= y <= max(yi, yj) + 1e-9:
            return True
        intersects = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / (yj - yi + 1e-15) + xi
        )
        if intersects:
            inside = not inside
        j = i
    return inside


def _offsets(search_width_m: float, lane_spacing_m: float) -> np.ndarray:
    half = 0.5 * float(search_width_m)
    count = max(int(math.ceil(search_width_m / lane_spacing_m)), 1)
    return np.linspace(-half, half, count + 1, dtype=float)


def _normals(points_xy_m: np.ndarray) -> np.ndarray:
    tangent = np.gradient(points_xy_m, axis=0)
    norm = np.linalg.norm(tangent, axis=1)
    tangent = tangent / np.maximum(norm[:, None], 1e-12)
    return np.column_stack((-tangent[:, 1], tangent[:, 0]))


def build_prior_guided_search(
    prior_path_xy_m: np.ndarray,
    config: SearchConfig,
    *,
    initial_position_xy_m: Optional[np.ndarray] = None,
    initial_position_uncertainty_m: float = 0.0,
    search_boundary_xy_m: Optional[np.ndarray] = None,
) -> SearchPlan:
    """Create alternating lanes along a prior cable corridor.

    The planner is deliberately deterministic.  It rejects a boundary that does
    not contain the requested corridor instead of silently creating connectors
    outside the authorized search polygon.
    """
    config.validate()
    if initial_position_uncertainty_m < 0.0:
        raise ValueError("initial_position_uncertainty_m must be non-negative")
    effective_width = max(config.search_width_m, 2.0 * float(initial_position_uncertainty_m))
    centerline = _resample_polyline(
        prior_path_xy_m, config.lane_spacing_m if config.pattern == "transverse" else config.waypoint_spacing_m,
    )
    normals = _normals(centerline)
    offsets = _offsets(effective_width, config.lane_spacing_m)
    lanes = [centerline + offset * normals for offset in offsets]
    if config.pattern == "transverse":
        # Cross the uncertain cable corridor early instead of first travelling
        # its entire length on a parallel lane that may never see an anomaly.
        lanes = [
            _resample_polyline(np.vstack((point - effective_width * 0.5 * normal,
                                         point + effective_width * 0.5 * normal)), config.waypoint_spacing_m)
            for point, normal in zip(centerline, normals)
        ]

    if search_boundary_xy_m is not None:
        boundary = np.asarray(search_boundary_xy_m, dtype=float)
        if boundary.ndim != 2 or boundary.shape[1] != 2 or boundary.shape[0] < 3:
            raise ValueError("search boundary must have shape (N, 2), N >= 3")
        if any(not _point_in_polygon(point, boundary) for lane in lanes for point in lane):
            return SearchPlan(
                waypoints_xy_m=np.empty((0, 2), dtype=float),
                lane_offsets_m=offsets,
                velocity_mps=config.velocity_mps,
                estimated_distance_m=0.0,
                estimated_time_s=0.0,
                valid=False,
                reason_codes=("corridor_outside_search_boundary",),
            )

    if initial_position_xy_m is not None and config.pattern != "transverse":
        start = np.asarray(initial_position_xy_m, dtype=float)[:2]
        first_cost = [min(np.linalg.norm(start - lane[0]), np.linalg.norm(start - lane[-1])) for lane in lanes]
        if first_cost and int(np.argmin(first_cost)) > len(lanes) // 2:
            lanes.reverse()
            offsets = offsets[::-1]

    ordered = []
    for lane_index, lane in enumerate(lanes):
        ordered.append(lane if lane_index % 2 == 0 else lane[::-1])
    waypoints = np.vstack(ordered)
    if initial_position_xy_m is not None:
        first = waypoints[0]
        last = waypoints[-1]
        start = np.asarray(initial_position_xy_m, dtype=float)[:2]
        if np.linalg.norm(start - last) < np.linalg.norm(start - first):
            waypoints = waypoints[::-1]
            offsets = offsets[::-1]
    distance = float(np.sum(np.linalg.norm(np.diff(waypoints, axis=0), axis=1)))
    return SearchPlan(
        waypoints_xy_m=waypoints,
        lane_offsets_m=offsets,
        velocity_mps=config.velocity_mps,
        estimated_distance_m=distance,
        estimated_time_s=distance / config.velocity_mps,
    )
