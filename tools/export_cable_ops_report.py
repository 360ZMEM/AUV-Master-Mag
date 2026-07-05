"""Export deployment/API tracking outputs into cable-operations report files."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any, Iterable

import numpy as np

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.api import CableTrackingOutput, export_tracking_outputs  # noqa: E402


def _optional_float(value: Any) -> float | None:
    if value in (None, ""):
        return None
    return float(value)


def _xy_from_row(row: dict[str, Any]) -> np.ndarray:
    if "estimated_cable_xy_m" in row:
        return np.asarray(row["estimated_cable_xy_m"], dtype=float)[:2]
    return np.array([float(row["estimated_cable_x_m"]), float(row["estimated_cable_y_m"])], dtype=float)


def _output_from_mapping(row: dict[str, Any]) -> CableTrackingOutput:
    return CableTrackingOutput(
        time_s=float(row["time_s"]),
        estimated_cable_xy_m=_xy_from_row(row),
        cross_track_m=float(row.get("cross_track_m", 0.0)),
        route_progress_m=float(row.get("route_progress_m", 0.0)),
        cable_heading_deg=float(row.get("cable_heading_deg", 0.0)),
        burial_depth_m=_optional_float(row.get("burial_depth_m")),
        burial_sigma_m=_optional_float(row.get("burial_sigma_m")),
        confidence=float(row.get("confidence", 0.0)),
        mode=str(row.get("mode", "unknown")),
        diagnostics=dict(row.get("diagnostics", {})) if isinstance(row.get("diagnostics", {}), dict) else {},
    )


def _read_json_or_jsonl(path: Path) -> list[CableTrackingOutput]:
    text = path.read_text(encoding="utf-8").strip()
    if not text:
        return []
    if text.startswith("["):
        payload = json.loads(text)
    else:
        payload = [json.loads(line) for line in text.splitlines() if line.strip()]
    return [_output_from_mapping(item) for item in payload]


def _read_csv(path: Path) -> list[CableTrackingOutput]:
    with path.open(newline="", encoding="utf-8") as handle:
        return [_output_from_mapping(row) for row in csv.DictReader(handle)]


def _read_outputs(path: Path, input_format: str) -> list[CableTrackingOutput]:
    if input_format == "auto":
        input_format = "csv" if path.suffix.lower() == ".csv" else "jsonl"
    if input_format == "csv":
        return _read_csv(path)
    if input_format in {"json", "jsonl"}:
        return _read_json_or_jsonl(path)
    raise ValueError(f"unsupported input format: {input_format}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True, help="API output CSV, JSON, or JSONL.")
    parser.add_argument("--output-dir", type=Path, required=True, help="Directory for cable-operations report files.")
    parser.add_argument("--format", choices=("auto", "csv", "json", "jsonl"), default="auto")
    args = parser.parse_args()

    outputs = _read_outputs(args.input, args.format)
    export_tracking_outputs(outputs, args.output_dir)
    print(f"[export] wrote {len(outputs)} tracking rows to {args.output_dir}")


if __name__ == "__main__":
    main()
