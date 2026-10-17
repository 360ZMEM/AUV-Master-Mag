#!/usr/bin/env python3
"""Preflight the physical evidence behind a held-out FWHM calibration.

This checks traceability, hashes and split independence. It cannot determine
whether a file was really recorded on the vehicle or approve field release.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any


ROLES = ("raw_magnetic", "geometry_truth", "altitude_navigation")


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def resolve_artifact(base: Path, value: Any, label: str, errors: list[str]) -> Path | None:
    if not isinstance(value, dict):
        errors.append(f"{label}: artifact record missing")
        return None
    name = value.get("path")
    expected = value.get("sha256")
    if not isinstance(name, str) or not name.strip():
        errors.append(f"{label}: path missing")
        return None
    if not isinstance(expected, str) or len(expected) != 64 or any(c not in "0123456789abcdef" for c in expected.lower()):
        errors.append(f"{label}: SHA-256 missing or invalid")
        return None
    path = Path(name).expanduser()
    if not path.is_absolute():
        path = base / path
    path = path.resolve()
    if not path.is_file() or path.stat().st_size == 0:
        errors.append(f"{label}: file missing or empty: {path}")
        return None
    if file_sha256(path) != expected.lower():
        errors.append(f"{label}: SHA-256 mismatch: {path}")
        return None
    return path


def audit(manifest_path: Path) -> dict[str, Any]:
    errors: list[str] = []
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        return {"preflight_passed": False, "field_release_authorized": False,
                "blockers": [f"manifest unreadable: {exc}"]}
    if not isinstance(manifest, dict) or manifest.get("schema_version") != 1:
        return {"preflight_passed": False, "field_release_authorized": False,
                "blockers": ["manifest schema_version must be 1"]}
    base = manifest_path.resolve().parent
    if manifest.get("data_domain") != "physical":
        errors.append("data_domain must be physical; synthetic evidence cannot qualify depth")

    calibration_path = resolve_artifact(base, manifest.get("calibration_json"), "calibration_json", errors)
    calibration: dict[str, Any] = {}
    csv_path: Path | None = None
    if calibration_path is not None:
        try:
            calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
            if not isinstance(calibration, dict):
                raise ValueError("calibration JSON must be an object")
            if calibration.get("source") != "controlled_measurement":
                errors.append("calibration source is not controlled_measurement")
            if calibration.get("result", {}).get("accepted") is not True:
                errors.append("calibration held-out result was not accepted")
            if calibration.get("deployment_qualified") is True:
                errors.append("legacy calibration claims deployment approval from a source label alone")
            raw_input = calibration.get("input_csv")
            if not isinstance(raw_input, str) or not raw_input:
                errors.append("calibration input_csv missing")
            else:
                csv_path = Path(raw_input).expanduser()
                if not csv_path.is_absolute():
                    csv_path = calibration_path.parent / csv_path
                csv_path = csv_path.resolve()
                if not csv_path.is_file() or file_sha256(csv_path) != calibration.get("input_sha256"):
                    errors.append("calibration input CSV missing or hash mismatch")
                    csv_path = None
        except (OSError, UnicodeDecodeError, json.JSONDecodeError, ValueError, AttributeError) as exc:
            errors.append(f"calibration JSON invalid: {exc}")

    csv_scans: dict[str, str] = {}
    sample_counts: dict[str, int] = {}
    scan_constants: dict[str, tuple[float, float, float]] = {}
    if csv_path is not None:
        try:
            with csv_path.open(newline="", encoding="utf-8") as stream:
                reader = csv.DictReader(stream)
                required = {"scan_id", "split", "x_m", "envelope_nt", "true_separation_m", "baseline_nt", "noise_nt"}
                if not required.issubset(reader.fieldnames or []):
                    raise ValueError(f"input CSV lacks calibration columns: {sorted(required - set(reader.fieldnames or []))}")
                for row in reader:
                    sid, split = row["scan_id"].strip(), row["split"].strip()
                    if not sid or split not in ("train", "holdout"):
                        raise ValueError("input CSV has empty scan_id or invalid split")
                    if sid in csv_scans and csv_scans[sid] != split:
                        raise ValueError(f"scan {sid} appears in both splits")
                    for key in ("x_m", "envelope_nt", "true_separation_m", "baseline_nt", "noise_nt"):
                        value = float(row[key])
                        if not math.isfinite(value) or (key in ("true_separation_m", "noise_nt") and value <= 0):
                            raise ValueError(f"scan {sid} has invalid {key}")
                    constants = tuple(float(row[key]) for key in ("true_separation_m", "baseline_nt", "noise_nt"))
                    if sid in scan_constants and scan_constants[sid] != constants:
                        raise ValueError(f"scan {sid} changes geometry truth, baseline or noise within one scan")
                    scan_constants[sid] = constants
                    csv_scans[sid] = split
                    sample_counts[sid] = sample_counts.get(sid, 0) + 1
                if any(count < 20 for count in sample_counts.values()):
                    raise ValueError("each complete cross scan needs at least 20 samples")
        except (OSError, UnicodeDecodeError, csv.Error, TypeError, ValueError) as exc:
            errors.append(f"calibration input CSV invalid: {exc}")

    records = manifest.get("scans")
    if not isinstance(records, list):
        errors.append("scans must be a list")
        records = []
    manifest_scans: dict[str, str] = {}
    session_splits: dict[str, str] = {}
    artifact_roles: dict[Path, str] = {}
    artifact_splits: dict[Path, str] = {}
    for index, record in enumerate(records):
        label = f"scans[{index}]"
        if not isinstance(record, dict):
            errors.append(f"{label}: object required")
            continue
        sid, split, session = record.get("scan_id"), record.get("split"), record.get("session_id")
        if not isinstance(sid, str) or not sid.strip() or sid in manifest_scans:
            errors.append(f"{label}: scan_id missing or duplicate")
            continue
        if split not in ("train", "holdout"):
            errors.append(f"{label}: split must be train or holdout")
            continue
        if not isinstance(session, str) or not session.strip():
            errors.append(f"{label}: physical session_id missing")
        elif session in session_splits and session_splits[session] != split:
            errors.append(f"{label}: session {session} crosses train/holdout boundary")
        else:
            session_splits[session] = split
        manifest_scans[sid] = split
        method = str(record.get("geometry_truth_method", "")).strip().lower()
        if not method or any(token in method for token in ("magnetic", "fwhm", "simulation", "pvs")):
            errors.append(f"{label}: independent geometry truth method missing or circular")
        for role in ROLES:
            path = resolve_artifact(base, record.get(role), f"{label}.{role}", errors)
            if path is not None:
                if path in artifact_roles and artifact_roles[path] != role:
                    errors.append(f"{label}.{role}: file reused as {artifact_roles[path]}")
                if path in artifact_splits and artifact_splits[path] != split:
                    errors.append(f"{label}.{role}: file crosses train/holdout boundary")
                artifact_roles[path] = role
                artifact_splits[path] = split
    if manifest_scans != csv_scans:
        errors.append("manifest scan IDs/splits do not exactly match calibration CSV")
    counts = {split: sum(value == split for value in manifest_scans.values()) for split in ("train", "holdout")}
    if counts["train"] < 5 or counts["holdout"] < 5:
        errors.append("at least five complete train and five complete holdout scans are required")
    if len({sid for sid, split in session_splits.items() if split == "train"}) < 1 or len({sid for sid, split in session_splits.items() if split == "holdout"}) < 1:
        errors.append("both splits require distinct physical sessions")

    return {
        "preflight_passed": not errors,
        "field_release_authorized": False,
        "manifest_sha256": file_sha256(manifest_path),
        "audit_script_sha256": file_sha256(Path(__file__)),
        "calibration_tool_sha256": file_sha256(Path(__file__).with_name("calibrate_v2_fwhm_shape.py")),
        "calibration_sha256": file_sha256(calibration_path) if calibration_path is not None else None,
        "scan_counts": counts,
        "blockers": errors,
        "manual_review_required": [
            "verify actual physical acquisition and independent geometry truth",
            "verify sensor extrinsics, time synchronization, altitude and deployment envelope",
            "approve the exact release configuration and physical execution package",
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output-json", type=Path)
    args = parser.parse_args()
    result = audit(args.manifest)
    rendered = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    if not result["preflight_passed"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
