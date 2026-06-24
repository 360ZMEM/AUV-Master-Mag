"""Local cable path state estimator for short curved segments.

This module is intentionally independent from the mission loop.  It estimates a
local cable state from recent cable-point observations and supports a line model
plus a circular-arc model.  The first integration target is unit-tested short
curves with radius above the physical lower bound, not the full maze scenario.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, Iterable, Optional

import numpy as np

from ..math_utils import heading_from_direction_xy, smallest_angle_error_deg, unit
from .state import FitResult


@dataclass
class LocalPathObservation:
    """One local cable observation in NED XY coordinates."""

    position_xy_m: np.ndarray
    time_s: float
    confidence: float = 1.0
    heading_deg: Optional[float] = None


@dataclass
class LocalCableState:
    """Estimated local cable geometry at the latest observation."""

    model: str
    anchor_xy_m: np.ndarray
    tangent_xy: np.ndarray
    heading_deg: float
    residual_m: float
    confidence: float
    curvature_1pm: float = 0.0
    radius_m: float = float("inf")
    center_xy_m: Optional[np.ndarray] = None
    arc_angle_span_deg: float = 0.0

    def as_fit_result(self) -> FitResult:
        """Expose the local tangent through the legacy FitResult contract."""
        covariance = np.array(
            [
                [self.residual_m**2, 0.0],
                [0.0, self.residual_m**2],
            ],
            dtype=float,
        )
        return FitResult(
            origin_xy_m=self.anchor_xy_m.copy(),
            direction_xy=self.tangent_xy.copy(),
            residual_m=float(self.residual_m),
            covariance_xy_m2=covariance,
        )


class LocalCableStateEstimator:
    """Fit a local line or circular arc from recent cable observations."""

    def __init__(
        self,
        capacity: int = 24,
        min_arc_radius_m: float = 30.0,
        min_arc_angle_span_deg: float = 35.0,
        arc_residual_ratio: float = 0.85,
        local_line_window: int = 6,
        heading_blend: float = 0.50,
    ) -> None:
        self.capacity = max(3, int(capacity))
        self.min_arc_radius_m = max(1e-6, float(min_arc_radius_m))
        self.min_arc_angle_span_deg = max(0.0, float(min_arc_angle_span_deg))
        self.arc_residual_ratio = float(np.clip(arc_residual_ratio, 0.2, 1.5))
        self.local_line_window = max(3, int(local_line_window))
        self.heading_blend = float(np.clip(heading_blend, 0.0, 0.9))
        self.observations: Deque[LocalPathObservation] = deque(maxlen=self.capacity)

    def reset(self) -> None:
        self.observations.clear()

    def add_observation(
        self,
        position_xy_m: Iterable[float],
        time_s: float,
        confidence: float = 1.0,
        heading_deg: Optional[float] = None,
    ) -> None:
        self.observations.append(
            LocalPathObservation(
                position_xy_m=np.asarray(position_xy_m, dtype=float),
                time_s=float(time_s),
                confidence=float(np.clip(confidence, 1e-3, 1.0)),
                heading_deg=None if heading_deg is None else float(heading_deg),
            )
        )

    def estimate(self) -> Optional[LocalCableState]:
        if len(self.observations) < 2:
            return None

        points = np.vstack([observation.position_xy_m for observation in self.observations])
        weights = np.asarray([observation.confidence for observation in self.observations], dtype=float)
        weights = np.maximum(weights, 1e-3)
        weights = weights / np.sum(weights)

        line_state = self._fit_line(points, weights, model_name="line")
        local_line_state = self._fit_local_line()
        circle_state = self._fit_circle(points, weights)
        if circle_state is None:
            return self._select_line_fallback(line_state, local_line_state)

        arc_is_valid = (
            circle_state.radius_m >= self.min_arc_radius_m
            and circle_state.arc_angle_span_deg >= self.min_arc_angle_span_deg
            and circle_state.residual_m <= max(line_state.residual_m * self.arc_residual_ratio, 0.25)
        )
        if not arc_is_valid:
            return self._select_line_fallback(line_state, local_line_state)

        return self._apply_heading_observations(circle_state)

    @staticmethod
    def _select_line_fallback(line_state: LocalCableState, local_line_state: LocalCableState) -> LocalCableState:
        if line_state.residual_m <= max(local_line_state.residual_m * 1.25, 0.10):
            return line_state
        return local_line_state

    def _fit_line(self, points: np.ndarray, weights: np.ndarray, model_name: str = "line") -> LocalCableState:
        centroid = np.sum(points * weights[:, None], axis=0)
        centered = points - centroid
        covariance = np.zeros((2, 2), dtype=float)
        for weight, point in zip(weights, centered):
            covariance += weight * np.outer(point, point)

        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        direction = eigenvectors[:, int(np.argmax(eigenvalues))]
        direction = unit(direction)
        macro_vec = points[-1] - points[0]
        if np.dot(direction, macro_vec) < 0.0:
            direction = -direction

        orthogonal = np.array([-direction[1], direction[0]], dtype=float)
        residual = float(np.sqrt(np.sum(weights * (centered @ orthogonal) ** 2)))
        anchor = points[-1].copy()
        confidence = self._state_confidence(residual, len(points))
        state = LocalCableState(
            model=model_name,
            anchor_xy_m=anchor,
            tangent_xy=direction,
            heading_deg=heading_from_direction_xy(direction),
            residual_m=residual,
            confidence=confidence,
        )
        return self._apply_heading_observations(state, blend_heading=True)

    def _fit_local_line(self) -> LocalCableState:
        recent_observations = list(self.observations)[-self.local_line_window :]
        points = np.vstack([observation.position_xy_m for observation in recent_observations])
        weights = np.asarray([observation.confidence for observation in recent_observations], dtype=float)

        # Favor the newest samples so a curved cable produces a moving local
        # tangent instead of a stale global chord.
        age_index = np.arange(points.shape[0], dtype=float)
        recency = np.exp(-(points.shape[0] - 1.0 - age_index) / max(points.shape[0] * 0.45, 1.0))
        weights = np.maximum(weights * recency, 1e-3)
        weights = weights / np.sum(weights)
        return self._fit_line(points, weights, model_name="local_line")

    def _fit_circle(self, points: np.ndarray, weights: np.ndarray) -> Optional[LocalCableState]:
        if points.shape[0] < 3:
            return None

        x = points[:, 0]
        y = points[:, 1]
        design = np.column_stack([x, y, np.ones_like(x)])
        target = -(x * x + y * y)
        weighted_design = design * np.sqrt(weights)[:, None]
        weighted_target = target * np.sqrt(weights)
        try:
            coeffs, _, _, _ = np.linalg.lstsq(weighted_design, weighted_target, rcond=None)
        except np.linalg.LinAlgError:
            return None

        d_coeff, e_coeff, f_coeff = coeffs
        center = np.array([-0.5 * d_coeff, -0.5 * e_coeff], dtype=float)
        radius_sq = float(np.dot(center, center) - f_coeff)
        if not np.isfinite(radius_sq) or radius_sq <= 1e-9:
            return None
        radius = float(np.sqrt(radius_sq))

        radial_vectors = points - center
        radial_distances = np.linalg.norm(radial_vectors, axis=1)
        if np.any(radial_distances < 1e-9):
            return None
        residual = float(np.sqrt(np.sum(weights * (radial_distances - radius) ** 2)))

        angles = np.unwrap(np.arctan2(radial_vectors[:, 1], radial_vectors[:, 0]))
        arc_span_deg = float(np.rad2deg(np.max(angles) - np.min(angles)))

        latest_radial = radial_vectors[-1] / radial_distances[-1]
        tangent = np.array([-latest_radial[1], latest_radial[0]], dtype=float)
        macro_vec = points[-1] - points[0]
        if np.dot(tangent, macro_vec) < 0.0:
            tangent = -tangent
        tangent = unit(tangent)

        # Positive curvature means counter-clockwise motion along the estimated arc.
        ccw_tangent = np.array([-latest_radial[1], latest_radial[0]], dtype=float)
        turn_sign = 1.0 if np.dot(ccw_tangent, tangent) >= 0.0 else -1.0

        anchor = center + latest_radial * radius
        confidence = self._state_confidence(residual, len(points))
        return LocalCableState(
            model="arc",
            anchor_xy_m=anchor,
            tangent_xy=tangent,
            heading_deg=heading_from_direction_xy(tangent),
            residual_m=residual,
            confidence=confidence,
            curvature_1pm=turn_sign / radius,
            radius_m=radius,
            center_xy_m=center,
            arc_angle_span_deg=arc_span_deg,
        )

    def _apply_heading_observations(self, state: LocalCableState, blend_heading: bool = False) -> LocalCableState:
        headings = [observation.heading_deg for observation in self.observations if observation.heading_deg is not None]
        if not headings:
            return state

        latest_heading_deg = float(headings[-1])
        heading_error_deg = abs(smallest_angle_error_deg(state.heading_deg, latest_heading_deg))
        if blend_heading and heading_error_deg <= 90.0 and self.heading_blend > 0.0:
            heading_rad = np.deg2rad(latest_heading_deg)
            heading_vector = np.array([np.cos(heading_rad), np.sin(heading_rad)], dtype=float)
            if np.dot(heading_vector, state.tangent_xy) < 0.0:
                heading_vector = -heading_vector
            tangent = unit((1.0 - self.heading_blend) * state.tangent_xy + self.heading_blend * heading_vector)
            if np.linalg.norm(tangent) > 0.0:
                state.tangent_xy = tangent
                state.heading_deg = heading_from_direction_xy(tangent)
                heading_error_deg = abs(smallest_angle_error_deg(state.heading_deg, latest_heading_deg))
        if heading_error_deg <= 90.0:
            state.confidence *= float(np.exp(-heading_error_deg / 45.0))
        else:
            state.confidence *= 0.2
        return state

    @staticmethod
    def _state_confidence(residual_m: float, observation_count: int) -> float:
        residual_score = float(np.exp(-max(residual_m, 0.0) / 1.5))
        count_score = float(np.clip((observation_count - 2) / 6.0, 0.0, 1.0))
        return float(np.clip(0.65 * residual_score + 0.35 * count_score, 0.0, 1.0))
