"""Sparse industrial prior route: a 3–5 point poly-line plus a tolerance corridor.

Replaces the legacy dense ``nominal_route`` and the deployment heuristics.  The route
carries only the way-points and a ±band; projection helpers reuse the shared
poly-line cache so the mission layer can bound its lateral sweep without re-deriving
geometry.  Pure data — no perception/controller coupling.
"""

from dataclasses import dataclass, field
from typing import Tuple

import numpy as np

from ..math_utils import (
    PolylineProjectionCache,
    build_polyline_projection_cache,
    heading_from_direction_xy,
    nearest_point_on_polyline,
)


@dataclass
class PriorWaypointsRoute:
    """工业先验航路：3–5 个 waypoints + ±公差带。"""

    waypoints_xy_m: np.ndarray
    tolerance_band_m: float = 30.0
    _cache: PolylineProjectionCache = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        """校验 waypoints 形状并构建投影缓存。"""
        self.waypoints_xy_m = np.asarray(self.waypoints_xy_m, dtype=float)
        if self.waypoints_xy_m.ndim != 2 or self.waypoints_xy_m.shape[1] != 2:
            raise ValueError("waypoints_xy_m must have shape (N, 2)")
        if len(self.waypoints_xy_m) < 2:
            raise ValueError("PriorWaypointsRoute requires at least two way-points")
        self._cache = build_polyline_projection_cache(self.waypoints_xy_m)

    def reference(self, position_xy_m: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
        """返回给定位置在先验航路上的最近点、切向与横向距离。"""
        nearest_xy, tangent_xy, distance_m, _, _ = nearest_point_on_polyline(
            np.asarray(position_xy_m, dtype=float)[:2], self._cache
        )
        return nearest_xy, tangent_xy, distance_m

    def heading_deg(self, position_xy_m: np.ndarray) -> float:
        """返回先验航路在最近点处的切向航向（度）。"""
        _, tangent_xy, _ = self.reference(position_xy_m)
        return heading_from_direction_xy(tangent_xy)

    def within_tolerance(self, position_xy_m: np.ndarray) -> bool:
        """判断给定位置是否仍在 ±tolerance_band_m 的公差走廊内。"""
        _, _, distance_m = self.reference(position_xy_m)
        return distance_m <= self.tolerance_band_m
