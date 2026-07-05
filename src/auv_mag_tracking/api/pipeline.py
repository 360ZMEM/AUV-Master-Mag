"""Deployment-facing lightweight tracking pipeline facade."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np

from ..config import ScenarioConfig
from ..math_utils import nearest_point_on_polyline
from .cable_map import CableMap
from .types import CableTrackingOutput, MagneticInput, NavigationInput, SonarInput


class AuvMagTrackingPipeline:
    """Minimal plug-and-play facade around cable-map projection contracts.

    This facade is intentionally free of GUI and offline simulation dependencies.
    It provides a stable I/O shell for external AUV managers while deeper
    perception modules continue to evolve behind it.
    """

    def __init__(self, config: ScenarioConfig, cable_map: CableMap) -> None:
        self.config = config
        self.cable_map = cable_map
        self._cache = cable_map.projection_cache()
        self.last_output: Optional[CableTrackingOutput] = None

    def reset(self) -> None:
        self.last_output = None

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

        output = CableTrackingOutput(
            time_s=float(navigation.time_s),
            estimated_cable_xy_m=np.asarray(estimate_xy, dtype=float),
            cross_track_m=float(distance_m),
            route_progress_m=float(progress_m),
            cable_heading_deg=cable_heading_deg,
            burial_depth_m=burial_depth,
            burial_sigma_m=None,
            confidence=confidence,
            mode="track" if confidence >= 0.5 else "map_fallback",
            diagnostics={
                "source": source,
                "map_frame": self.cable_map.frame,
                "map_segment_index": int(segment_index),
                "magnetic_used": False,
                "magnetic_sample_count": int(np.asarray(magnetic.sample_block_nt).reshape(-1, 3).shape[0]),
                "navigation_source": navigation.source,
            },
        )
        self.last_output = output
        return output

    @staticmethod
    def _sonar_to_ned(position_xy: np.ndarray, heading_deg: float, relative_body_m: np.ndarray) -> np.ndarray:
        rel = np.asarray(relative_body_m, dtype=float)
        if rel.size < 2:
            raise ValueError("sonar relative_position_body_m must contain at least x/y")
        yaw = math.radians(float(heading_deg))
        rot = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]], dtype=float)
        return np.asarray(position_xy, dtype=float) + rot @ rel[:2]
