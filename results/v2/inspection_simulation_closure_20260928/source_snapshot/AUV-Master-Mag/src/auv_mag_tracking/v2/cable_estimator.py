"""Small, explainable fusion of cable-centre observations."""

from __future__ import annotations

from collections import deque
import math
from typing import Optional

import numpy as np

from .types import CableObservation


class CableLineEstimator:
    def __init__(self, max_observations: int = 20) -> None:
        if max_observations < 2:
            raise ValueError("max_observations must be at least two")
        self._observations: deque[CableObservation] = deque(maxlen=max_observations)

    def reset(self) -> None:
        self._observations.clear()

    def add(self, observation: CableObservation) -> None:
        if np.all(np.isfinite(observation.position_xy_m)) and observation.confidence > 0.0:
            self._observations.append(observation)

    @property
    def count(self) -> int:
        return len(self._observations)

    def estimate(self, fallback_heading_deg: float) -> tuple[Optional[np.ndarray], float, float]:
        if not self._observations:
            return None, float(fallback_heading_deg), float("inf")
        points = np.asarray([item.position_xy_m for item in self._observations], dtype=float)
        weights = np.asarray(
            [item.confidence / max(item.lateral_sigma_m ** 2, 1e-4) for item in self._observations],
            dtype=float,
        )
        weights /= np.sum(weights)
        center = np.sum(points * weights[:, None], axis=0)
        if points.shape[0] < 2 or float(np.linalg.norm(points[-1] - points[0])) < 1e-6:
            return center, float(fallback_heading_deg), float(np.mean([item.lateral_sigma_m for item in self._observations]))
        centered = points - center
        covariance = (centered * weights[:, None]).T @ centered
        values, vectors = np.linalg.eigh(covariance)
        tangent = vectors[:, int(np.argmax(values))]
        fallback = np.array([math.cos(math.radians(fallback_heading_deg)), math.sin(math.radians(fallback_heading_deg))])
        if float(np.dot(tangent, fallback)) < 0.0:
            tangent = -tangent
        heading = math.degrees(math.atan2(tangent[1], tangent[0]))
        residual = np.abs(centered @ np.array([-tangent[1], tangent[0]]))
        sigma = float(math.sqrt(np.sum(weights * residual ** 2)))
        return center, heading, sigma

