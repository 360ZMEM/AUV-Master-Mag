#!/usr/bin/env python3
"""Calibrate FWHM width-to-distance factor from controlled, split cross scans."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from auv_mag_tracking.v2 import (  # noqa: E402
    CalibrationScan, FwhmConfig, calibrate_affine_width, calibrate_shape_factor,
)


def load_scans(path: Path) -> dict[str, list[CalibrationScan]]:
    required = {
        "scan_id", "split", "x_m", "envelope_nt", "true_separation_m",
        "baseline_nt", "noise_nt",
    }
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"CSV missing columns: {sorted(required - set(reader.fieldnames or []))}")
        grouped: dict[str, list[dict[str, str]]] = {}
        for row in reader:
            grouped.setdefault(row["scan_id"], []).append(row)
    scans: dict[str, list[CalibrationScan]] = {"train": [], "holdout": []}
    for scan_id, rows in grouped.items():
        split = rows[0]["split"]
        if split not in scans or any(row["split"] != split for row in rows):
            raise ValueError(f"scan {scan_id}: split must be consistently train or holdout")
        for key in ("true_separation_m", "baseline_nt", "noise_nt"):
            if any(row[key] != rows[0][key] for row in rows):
                raise ValueError(f"scan {scan_id}: {key} must be constant")
        scans[split].append(CalibrationScan(
            scan_id=scan_id,
            x_m=np.asarray([float(row["x_m"]) for row in rows]),
            envelope_nt=np.asarray([float(row["envelope_nt"]) for row in rows]),
            true_separation_m=float(rows[0]["true_separation_m"]),
            baseline_nt=float(rows[0]["baseline_nt"]),
            noise_nt=float(rows[0]["noise_nt"]),
        ))
    return scans


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--shape-model", choices=("thesis_dipole", "physical_single_wire"), required=True)
    parser.add_argument("--mapping", choices=("constant", "affine"), default="constant")
    parser.add_argument("--source", choices=("controlled_measurement", "simulation"), required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--max-mae-m", type=float, default=0.2)
    parser.add_argument("--max-p95-m", type=float, default=0.5)
    args = parser.parse_args()
    scans = load_scans(args.input_csv)
    config = FwhmConfig(
        shape_model=args.shape_model,
        shape_factor=0.5 if args.shape_model == "thesis_dipole" else 1.0 / (2.0 * np.sqrt(3.0)),
        altitude_m=0.0,
    )
    calibrate = calibrate_shape_factor if args.mapping == "constant" else calibrate_affine_width
    result = calibrate(
        scans["train"], scans["holdout"], config,
        maximum_holdout_mae_m=args.max_mae_m,
        maximum_holdout_p95_m=args.max_p95_m,
        source=args.source,
    )
    payload = {
        "source": args.source,
        "mapping": args.mapping,
        "input_csv": str(args.input_csv.resolve()),
        "input_sha256": hashlib.sha256(args.input_csv.read_bytes()).hexdigest(),
        "result": asdict(result),
        "statistical_holdout_accepted": bool(result.accepted),
        # A command-line source label and a held-out CSV cannot authenticate
        # the physical origin of magnetic samples or independent geometry.
        "deployment_qualified": False,
        "physical_provenance_review_required": True,
        "note": "A held-out fit is not field approval. Physical use requires raw magnetic and independent geometry evidence, synchronized altitude, source hashes, and a reviewed deployment package.",
    }
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    if not result.accepted:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
