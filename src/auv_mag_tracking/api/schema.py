"""Input-schema validation helpers for deployment files."""

from __future__ import annotations

import csv
from pathlib import Path


def _require_columns(path: str | Path, required: set[str]) -> list[str]:
    with Path(path).open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fieldnames = set(reader.fieldnames or [])
    missing = sorted(required - fieldnames)
    if missing:
        raise ValueError(f"{path}: missing required columns: {', '.join(missing)}")
    return sorted(fieldnames)


def validate_cable_map_csv(path: str | Path) -> list[str]:
    return _require_columns(path, {"x_m", "y_m"})


def validate_navigation_csv(path: str | Path) -> list[str]:
    return _require_columns(path, {"time_s", "position_x_m", "position_y_m", "heading_deg"})


def validate_magnetometer_csv(path: str | Path) -> list[str]:
    return _require_columns(path, {"time_s", "b_x_nt", "b_y_nt", "b_z_nt"})


def validate_sonar_csv(path: str | Path) -> list[str]:
    return _require_columns(path, {"time_s", "rel_x_m", "rel_y_m", "confidence", "valid"})
