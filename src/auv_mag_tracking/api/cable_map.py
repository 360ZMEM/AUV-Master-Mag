"""Cable-map import and projection helpers for public API use."""

from __future__ import annotations

import csv
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

import numpy as np

from ..math_utils import build_polyline_projection_cache


@dataclass
class CableMap:
    points_xy_m: np.ndarray
    frame: str = "local_ned"
    burial_depth_m: Optional[float | np.ndarray] = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.points_xy_m = np.asarray(self.points_xy_m, dtype=float)
        if self.points_xy_m.ndim != 2 or self.points_xy_m.shape[1] != 2:
            raise ValueError("points_xy_m must have shape (N, 2)")
        if self.points_xy_m.shape[0] < 2:
            raise ValueError("CableMap requires at least two points")

    @classmethod
    def from_csv(cls, path: str | Path, frame: str = "local_ned") -> "CableMap":
        points = []
        burial_values: list[float | None] = []
        with Path(path).open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            required = {"x_m", "y_m"}
            if not reader.fieldnames or not required.issubset(reader.fieldnames):
                raise ValueError("Cable map CSV must contain x_m and y_m columns")
            has_burial_column = "burial_depth_m" in reader.fieldnames
            for row in reader:
                points.append((float(row["x_m"]), float(row["y_m"])))
                if has_burial_column:
                    value = row.get("burial_depth_m")
                    burial_values.append(None if value in (None, "") else float(value))
        burial = None
        if burial_values:
            if any(value is None for value in burial_values):
                raise ValueError("burial_depth_m must be present for every cable-map row or omitted entirely")
            burial = np.asarray(burial_values, dtype=float)
        return cls(points_xy_m=np.asarray(points, dtype=float), frame=frame, burial_depth_m=burial)

    @classmethod
    def from_geojson(cls, path: str | Path, frame: str = "local_ned") -> "CableMap":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        geometry = data.get("geometry", data)
        gtype = geometry.get("type")
        coords = geometry.get("coordinates")
        if gtype == "LineString":
            points = [(float(x), float(y)) for x, y, *_ in coords]
        elif gtype == "MultiLineString":
            points = [(float(x), float(y)) for line in coords for x, y, *_ in line]
        else:
            raise ValueError("GeoJSON cable map must be LineString or MultiLineString")
        return cls(points_xy_m=np.asarray(points, dtype=float), frame=frame)

    def to_polyline(self) -> np.ndarray:
        return self.points_xy_m.copy()

    def projection_cache(self):
        return build_polyline_projection_cache(self.points_xy_m)
