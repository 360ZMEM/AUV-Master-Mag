"""Compact cable-map reconstruction from localized cable observations."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, List, Optional

import numpy as np

from .prior_waypoints import PriorWaypointsRoute


@dataclass
class CableMapObservation:
    """One localized cable observation in the deployment NED frame."""

    position_xy_m: np.ndarray
    confidence: float
    time_s: Optional[float] = None
    source: str = "UNKNOWN"


@dataclass
class CableMap:
    """A compact polyline representation of the identified cable shape."""

    waypoints_xy_m: np.ndarray
    tolerance_band_m: float = 30.0
    metadata: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.waypoints_xy_m = np.asarray(self.waypoints_xy_m, dtype=float)
        if self.waypoints_xy_m.ndim != 2 or self.waypoints_xy_m.shape[1] != 2:
            raise ValueError("waypoints_xy_m must have shape (N, 2)")

    def to_prior_route(self) -> PriorWaypointsRoute:
        """Expose the map as a route-prior corridor for mission logic."""
        return PriorWaypointsRoute(self.waypoints_xy_m, tolerance_band_m=self.tolerance_band_m)

    def to_dict(self) -> dict:
        """Return a JSON-serializable map payload."""
        return {
            "schema": "auv_mag_tracking.cable_map.v1",
            "tolerance_band_m": float(self.tolerance_band_m),
            "waypoints_xy_m": self.waypoints_xy_m.tolist(),
            "metadata": self.metadata,
        }

    def save_json(self, path: str | Path) -> None:
        """Write the compact map to JSON."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.to_dict(), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    @classmethod
    def from_dict(cls, payload: dict) -> "CableMap":
        """Build a map from a JSON payload."""
        return cls(
            waypoints_xy_m=np.asarray(payload["waypoints_xy_m"], dtype=float),
            tolerance_band_m=float(payload.get("tolerance_band_m", 30.0)),
            metadata=dict(payload.get("metadata", {})),
        )

    @classmethod
    def load_json(cls, path: str | Path) -> "CableMap":
        """Read a compact map JSON file."""
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls.from_dict(payload)


class CableMapBuilder:
    """Incrementally build a compact cable map from ordered observations."""

    def __init__(
        self,
        *,
        min_confidence: float = 0.45,
        min_spacing_m: float = 2.0,
        simplify_tolerance_m: float = 3.0,
        tolerance_band_m: float = 30.0,
    ) -> None:
        self.min_confidence = float(min_confidence)
        self.min_spacing_m = float(min_spacing_m)
        self.simplify_tolerance_m = float(simplify_tolerance_m)
        self.tolerance_band_m = float(tolerance_band_m)
        self._observations: List[CableMapObservation] = []

    def add_observation(
        self,
        position_xy_m: Iterable[float],
        *,
        confidence: float,
        time_s: Optional[float] = None,
        source: str = "UNKNOWN",
    ) -> None:
        """Add one localized cable point if it passes confidence and finite checks."""
        position = np.asarray(position_xy_m, dtype=float)[:2]
        if position.shape != (2,) or not np.all(np.isfinite(position)):
            return
        if not np.isfinite(confidence) or confidence < self.min_confidence:
            return
        self._observations.append(
            CableMapObservation(position_xy_m=position, confidence=float(confidence), time_s=time_s, source=source)
        )

    @property
    def observation_count(self) -> int:
        """Number of accepted observations before spatial compaction."""
        return len(self._observations)

    def build(self, *, metadata: Optional[dict] = None) -> CableMap:
        """Return a spatially thinned and simplified cable map."""
        if not self._observations:
            raise ValueError("no valid cable observations to build a map")

        observations = sorted(
            self._observations,
            key=lambda obs: float("inf") if obs.time_s is None else float(obs.time_s),
        )
        points = np.asarray([obs.position_xy_m for obs in observations], dtype=float)
        thinned = _thin_by_spacing(points, self.min_spacing_m)
        simplified = _ramer_douglas_peucker(thinned, self.simplify_tolerance_m)
        meta = {
            "raw_observation_count": len(self._observations),
            "thinned_point_count": int(thinned.shape[0]),
            "waypoint_count": int(simplified.shape[0]),
            "min_confidence": self.min_confidence,
            "min_spacing_m": self.min_spacing_m,
            "simplify_tolerance_m": self.simplify_tolerance_m,
        }
        if metadata:
            meta.update(metadata)
        return CableMap(simplified, tolerance_band_m=self.tolerance_band_m, metadata=meta)


def build_cable_map_from_record(
    record,
    *,
    min_confidence: float = 0.45,
    min_spacing_m: float = 2.0,
    simplify_tolerance_m: float = 3.0,
    tolerance_band_m: float = 30.0,
    truth_fallback: bool = False,
) -> CableMap:
    """Build a compact map from a ``viz.RunRecord``.

    The preferred inputs are ``estimated_cable_x_m`` / ``estimated_cable_y_m``.
    ``truth_fallback`` is only for synthetic smoke tests and should stay disabled
    for deployment logs.
    """
    builder = CableMapBuilder(
        min_confidence=min_confidence,
        min_spacing_m=min_spacing_m,
        simplify_tolerance_m=simplify_tolerance_m,
        tolerance_band_m=tolerance_band_m,
    )
    channels = record.channels
    if "estimated_cable_x_m" in channels and "estimated_cable_y_m" in channels:
        xs = channels["estimated_cable_x_m"]
        ys = channels["estimated_cable_y_m"]
    elif truth_fallback:
        xs = channels["true_nearest_x_m"]
        ys = channels["true_nearest_y_m"]
    else:
        raise ValueError("record has no estimated cable point channels; rerun visualization with current code")

    times = channels.get("time_s", np.arange(len(xs), dtype=float))
    confidence = channels.get("confidence", np.ones_like(xs, dtype=float))
    sources = getattr(record, "sources", ["UNKNOWN"] * len(xs))
    for idx, (x_m, y_m) in enumerate(zip(xs, ys)):
        builder.add_observation(
            (x_m, y_m),
            confidence=float(confidence[idx]),
            time_s=float(times[idx]),
            source=str(sources[idx]) if idx < len(sources) else "UNKNOWN",
        )
    return builder.build(
        metadata={
            "case_name": getattr(record, "case_name", "unknown"),
            "deployment_mode": bool(getattr(record, "deployment_mode", False)),
            "truth_fallback": bool(truth_fallback),
        }
    )


def _thin_by_spacing(points_xy_m: np.ndarray, min_spacing_m: float) -> np.ndarray:
    if points_xy_m.shape[0] <= 1 or min_spacing_m <= 0.0:
        return points_xy_m.copy()
    kept = [points_xy_m[0]]
    for point in points_xy_m[1:]:
        if float(np.linalg.norm(point - kept[-1])) >= min_spacing_m:
            kept.append(point)
    if not np.allclose(kept[-1], points_xy_m[-1]):
        kept.append(points_xy_m[-1])
    return np.asarray(kept, dtype=float)


def _ramer_douglas_peucker(points_xy_m: np.ndarray, tolerance_m: float) -> np.ndarray:
    if points_xy_m.shape[0] <= 2 or tolerance_m <= 0.0:
        return points_xy_m.copy()
    start = points_xy_m[0]
    end = points_xy_m[-1]
    segment = end - start
    segment_norm = float(np.linalg.norm(segment))
    if segment_norm <= 1e-9:
        distances = np.linalg.norm(points_xy_m - start, axis=1)
    else:
        rel = points_xy_m - start
        distances = np.abs(segment[0] * rel[:, 1] - segment[1] * rel[:, 0]) / segment_norm
    split_index = int(np.argmax(distances))
    max_distance = float(distances[split_index])
    if max_distance <= tolerance_m:
        return np.vstack([start, end])
    left = _ramer_douglas_peucker(points_xy_m[: split_index + 1], tolerance_m)
    right = _ramer_douglas_peucker(points_xy_m[split_index:], tolerance_m)
    return np.vstack([left[:-1], right])
