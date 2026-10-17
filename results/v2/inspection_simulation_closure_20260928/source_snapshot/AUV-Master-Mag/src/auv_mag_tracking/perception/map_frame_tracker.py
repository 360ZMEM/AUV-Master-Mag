"""Cable-map-frame shadow tracker.

The tracker maintains an AUV state in cable-map coordinates
(``progress_m``, ``lateral_m``).  It is deliberately independent from the
controller: callers can run it as a shadow diagnostic before deciding whether a
map-frame projection strategy is mature enough for closed-loop use.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from ..math_utils import (
    build_polyline_projection_cache,
    nearest_point_on_polyline,
    nearest_point_on_polyline_within_progress,
)


@dataclass
class CableMapFrameState:
    """AUV state expressed in the cable map frame."""

    progress_m: float
    lateral_m: float
    projection_distance_m: float
    consistency_score: float
    tangent_xy: np.ndarray
    point_xy: np.ndarray
    segment_index: int


class CableMapFrameTracker:
    """Shadow progress tracker constrained by local cable-map continuity."""

    def __init__(
        self,
        route_xy: np.ndarray,
        initial_position_xy: np.ndarray,
        lookback_m: float = 8.0,
        lookahead_m: float = 30.0,
        max_forward_step_m: float = 2.5,
        max_lateral_step_m: float = 4.0,
        correction_gain: float = 0.15,
    ) -> None:
        route_xy = np.asarray(route_xy, dtype=float)
        if route_xy.ndim != 2 or route_xy.shape[1] != 2 or route_xy.shape[0] < 2:
            raise ValueError("route_xy must have shape (N, 2), N>=2")
        self.route_cache = build_polyline_projection_cache(route_xy)
        self.lookback_m = max(float(lookback_m), 0.0)
        self.lookahead_m = max(float(lookahead_m), 0.0)
        self.max_forward_step_m = max(float(max_forward_step_m), 1e-6)
        self.max_lateral_step_m = max(float(max_lateral_step_m), 1e-6)
        self.correction_gain = min(max(float(correction_gain), 0.0), 1.0)
        self.last_navigation_xy = np.asarray(initial_position_xy, dtype=float)

        point_xy, tangent_xy, distance_m, progress_m, segment_index = nearest_point_on_polyline(
            self.last_navigation_xy,
            self.route_cache,
        )
        self.state = self._state(self.last_navigation_xy, progress_m, point_xy, tangent_xy, distance_m, segment_index)

    def rebase_route(self, route_xy: np.ndarray) -> CableMapFrameState:
        """Rebuild the route cache without allowing a global lane snap."""
        route_xy = np.asarray(route_xy, dtype=float)
        if route_xy.ndim != 2 or route_xy.shape[1] != 2 or route_xy.shape[0] < 2:
            raise ValueError("route_xy must have shape (N, 2), N>=2")
        previous_progress_m = float(self.state.progress_m)
        self.route_cache = build_polyline_projection_cache(route_xy)
        point_xy, tangent_xy, distance_m, progress_m, segment_index = nearest_point_on_polyline_within_progress(
            self.last_navigation_xy,
            self.route_cache,
            previous_progress_m - self.lookback_m,
            previous_progress_m + self.lookahead_m,
        )
        self.state = self._state(
            self.last_navigation_xy,
            progress_m,
            point_xy,
            tangent_xy,
            distance_m,
            segment_index,
        )
        return self.state

    def update(
        self,
        navigation_position_xy: np.ndarray,
        observation_xy: Optional[np.ndarray] = None,
        observation_confidence: float = 0.0,
    ) -> CableMapFrameState:
        """Advance the map-frame state from navigation delta and optional cable observation."""
        navigation_xy = np.asarray(navigation_position_xy, dtype=float)
        delta_xy = navigation_xy - self.last_navigation_xy
        tangent_xy = self.state.tangent_xy
        normal_xy = np.array([-tangent_xy[1], tangent_xy[0]], dtype=float)

        delta_progress_m = float(np.dot(delta_xy, tangent_xy))
        delta_lateral_m = float(np.dot(delta_xy, normal_xy))
        delta_progress_m = float(np.clip(delta_progress_m, -self.max_forward_step_m, self.max_forward_step_m))
        delta_lateral_m = float(np.clip(delta_lateral_m, -self.max_lateral_step_m, self.max_lateral_step_m))
        predicted_progress_m = self.state.progress_m + delta_progress_m
        predicted_lateral_m = self.state.lateral_m + delta_lateral_m

        if observation_xy is not None and observation_confidence > 0.0:
            _, _, _, obs_progress_m, _ = nearest_point_on_polyline_within_progress(
                np.asarray(observation_xy, dtype=float),
                self.route_cache,
                predicted_progress_m - self.lookback_m,
                predicted_progress_m + self.lookahead_m,
            )
            gain = self.correction_gain * min(max(float(observation_confidence), 0.0), 1.0)
            predicted_progress_m = (1.0 - gain) * predicted_progress_m + gain * obs_progress_m

        point_xy, tangent_xy, distance_m, progress_m, segment_index = nearest_point_on_polyline_within_progress(
            navigation_xy,
            self.route_cache,
            predicted_progress_m - self.lookback_m,
            predicted_progress_m + self.lookahead_m,
        )
        normal_xy = np.array([-tangent_xy[1], tangent_xy[0]], dtype=float)
        signed_lateral_m = float(np.dot(navigation_xy - point_xy, normal_xy))
        if np.isfinite(predicted_lateral_m):
            signed_lateral_m = 0.5 * signed_lateral_m + 0.5 * predicted_lateral_m

        self.state = self._state(
            navigation_xy,
            progress_m,
            point_xy,
            tangent_xy,
            distance_m,
            segment_index,
            lateral_m=signed_lateral_m,
        )
        self.last_navigation_xy = navigation_xy.copy()
        return self.state

    def _state(
        self,
        navigation_xy: np.ndarray,
        progress_m: float,
        point_xy: np.ndarray,
        tangent_xy: np.ndarray,
        projection_distance_m: float,
        segment_index: int,
        lateral_m: Optional[float] = None,
    ) -> CableMapFrameState:
        distance_m = abs(float(projection_distance_m))
        consistency_score = 1.0 / (1.0 + distance_m)
        normal_xy = np.array([-tangent_xy[1], tangent_xy[0]], dtype=float)
        if lateral_m is None:
            lateral_m = float(np.dot(np.asarray(navigation_xy, dtype=float) - point_xy, normal_xy))
        return CableMapFrameState(
            progress_m=float(progress_m),
            lateral_m=float(lateral_m),
            projection_distance_m=distance_m,
            consistency_score=consistency_score,
            tangent_xy=np.asarray(tangent_xy, dtype=float).copy(),
            point_xy=np.asarray(point_xy, dtype=float).copy(),
            segment_index=int(segment_index),
        )
