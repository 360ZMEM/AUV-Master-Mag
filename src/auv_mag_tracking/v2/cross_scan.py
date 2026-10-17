"""Bounded local cross-scan trajectory generation."""

from __future__ import annotations

from typing import Optional

import numpy as np

from .config import CrossScanConfig
from .search import _point_in_polygon
from .types import CrossScanPlan


def build_cross_scan(
    *,
    scan_id: int,
    center_xy_m: np.ndarray,
    tangent_xy: np.ndarray,
    current_position_xy_m: np.ndarray,
    config: CrossScanConfig,
    attempt_index: int = 0,
    search_boundary_xy_m: Optional[np.ndarray] = None,
) -> CrossScanPlan:
    """Build a side-to-side-to-centre scan with a nearly constant-speed core leg."""
    config.validate()
    center = np.asarray(center_xy_m, dtype=float)[:2]
    tangent = np.asarray(tangent_xy, dtype=float)[:2]
    tangent_norm = float(np.linalg.norm(tangent))
    if tangent_norm <= 1e-9:
        raise ValueError("tangent_xy must be non-zero")
    tangent = tangent / tangent_norm
    normal = np.array([-tangent[1], tangent[0]], dtype=float)
    level = min(max(int(attempt_index), 0), len(config.half_span_levels_m) - 1)
    half_span = float(config.half_span_levels_m[level])
    negative = center - half_span * normal
    positive = center + half_span * normal
    current = np.asarray(current_position_xy_m, dtype=float)[:2]
    first, second = (negative, positive) if np.linalg.norm(current - negative) <= np.linalg.norm(current - positive) else (positive, negative)
    waypoints = np.vstack((first, second, center))

    reasons: list[str] = []
    if search_boundary_xy_m is not None:
        boundary = np.asarray(search_boundary_xy_m, dtype=float)
        if any(not _point_in_polygon(point, boundary) for point in waypoints):
            reasons.append("cross_scan_outside_search_boundary")
    return CrossScanPlan(
        scan_id=int(scan_id), center_xy_m=center, tangent_xy=tangent, normal_xy=normal,
        waypoints_xy_m=waypoints, half_span_m=half_span, velocity_mps=config.velocity_mps,
        valid=not reasons, reason_codes=tuple(reasons),
    )

