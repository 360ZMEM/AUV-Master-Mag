"""Export helpers for cable-operation reports."""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Iterable

import numpy as np

from .types import CableTrackingOutput


def export_tracking_outputs(outputs: Iterable[CableTrackingOutput], output_dir: str | Path) -> None:
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    rows = []
    for idx, item in enumerate(outputs):
        rows.append({
            "inspection_point_id": idx,
            "time_s": f"{item.time_s:.3f}",
            "estimated_cable_x_m": f"{float(item.estimated_cable_xy_m[0]):.3f}",
            "estimated_cable_y_m": f"{float(item.estimated_cable_xy_m[1]):.3f}",
            "cross_track_m": f"{item.cross_track_m:.3f}",
            "route_progress_m": f"{item.route_progress_m:.3f}",
            "cable_heading_deg": f"{item.cable_heading_deg:.3f}",
            "burial_depth_m": "" if item.burial_depth_m is None else f"{item.burial_depth_m:.3f}",
            "burial_sigma_m": "" if item.burial_sigma_m is None else f"{item.burial_sigma_m:.3f}",
            "confidence": f"{item.confidence:.3f}",
            "mode": item.mode,
        })
    with (output_path / "cable_ops_points.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()) if rows else ["inspection_point_id"])
        writer.writeheader()
        writer.writerows(rows)
    burial_rows = [row for row in rows if row.get("burial_depth_m")]
    with (output_path / "burial_profile.csv").open("w", newline="", encoding="utf-8") as handle:
        fieldnames = ["inspection_point_id", "time_s", "route_progress_m", "burial_depth_m", "burial_sigma_m"]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in burial_rows:
            writer.writerow({field: row[field] for field in fieldnames})
    diagnostics = {
        "point_count": len(rows),
        "burial_point_count": len(burial_rows),
        "mean_confidence": float(np.mean([float(row["confidence"]) for row in rows])) if rows else 0.0,
    }
    (output_path / "diagnostics.json").write_text(
        json.dumps(diagnostics, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
