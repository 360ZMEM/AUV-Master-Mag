"""Validate deployment input files for the public tracking API."""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.api import (  # noqa: E402
    validate_cable_map_csv,
    validate_magnetometer_csv,
    validate_navigation_csv,
    validate_sonar_csv,
)


def _validate_monotonic_time(path: Path, column: str = "time_s") -> None:
    previous = None
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if column not in (reader.fieldnames or []):
            raise ValueError(f"{path}: missing required column: {column}")
        for line_number, row in enumerate(reader, start=2):
            value = float(row[column])
            if previous is not None and value < previous:
                raise ValueError(f"{path}: {column} is not monotonic at line {line_number}")
            previous = value


def _check(label: str, path: Path, validator, check_time: bool) -> tuple[str, bool, str]:
    try:
        columns = validator(path)
        if check_time:
            _validate_monotonic_time(path)
        return label, True, f"{path} ({len(columns)} columns)"
    except Exception as exc:  # noqa: BLE001 - CLI should report all validation failures.
        return label, False, str(exc)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cable-map", type=Path, help="Cable-map CSV with at least x_m,y_m.")
    parser.add_argument("--navigation", type=Path, help="Navigation CSV with time_s, position_x_m, position_y_m, heading_deg.")
    parser.add_argument("--magnetometer", type=Path, help="Magnetometer CSV with time_s, b_x_nt, b_y_nt, b_z_nt.")
    parser.add_argument("--sonar", type=Path, help="Sonar CSV with time_s, rel_x_m, rel_y_m, confidence, valid.")
    args = parser.parse_args()

    checks = []
    if args.cable_map:
        checks.append(_check("cable_map", args.cable_map, validate_cable_map_csv, False))
    if args.navigation:
        checks.append(_check("navigation", args.navigation, validate_navigation_csv, True))
    if args.magnetometer:
        checks.append(_check("magnetometer", args.magnetometer, validate_magnetometer_csv, True))
    if args.sonar:
        checks.append(_check("sonar", args.sonar, validate_sonar_csv, True))

    if not checks:
        parser.print_help()
        return

    failed = False
    for label, ok, message in checks:
        status = "OK" if ok else "FAIL"
        print(f"[{status}] {label}: {message}")
        failed = failed or not ok
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
