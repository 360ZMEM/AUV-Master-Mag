from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from audit_v2_depth_field_provenance import audit  # noqa: E402


def artifact(path: Path, contents: str) -> dict[str, str]:
    path.write_text(contents, encoding="utf-8")
    return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def fixture(tmp_path: Path) -> Path:
    csv_path = tmp_path / "scans.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=(
            "scan_id", "split", "x_m", "envelope_nt", "true_separation_m",
            "baseline_nt", "noise_nt"))
        writer.writeheader()
        for split in ("train", "holdout"):
            for i in range(5):
                for step in range(20):
                    writer.writerow({"scan_id": f"{split}_{i}", "split": split,
                                     "x_m": step - 10, "envelope_nt": 2.,
                                     "true_separation_m": 3., "baseline_nt": 1.,
                                     "noise_nt": .08})
    calibration = {
        "source": "controlled_measurement", "result": {"accepted": True},
        "deployment_qualified": False, "input_csv": str(csv_path),
        "input_sha256": hashlib.sha256(csv_path.read_bytes()).hexdigest(),
    }
    calibration_artifact = artifact(tmp_path / "calibration.json", json.dumps(calibration))
    scans = []
    for split in ("train", "holdout"):
        for i in range(5):
            sid = f"{split}_{i}"
            scans.append({
                "scan_id": sid, "split": split, "session_id": split,
                "geometry_truth_method": "surveyed conductor and measured sensor pose",
                **{role: artifact(tmp_path / f"{sid}_{role}.csv", f"{sid},{role}\n")
                   for role in ("raw_magnetic", "geometry_truth", "altitude_navigation")},
            })
    manifest_path = tmp_path / "manifest.json"
    manifest_path.write_text(json.dumps({
        "schema_version": 1, "data_domain": "physical",
        "calibration_json": calibration_artifact, "scans": scans,
    }), encoding="utf-8")
    return manifest_path


def update_manifest(path: Path, fn) -> None:
    payload = json.loads(path.read_text(encoding="utf-8"))
    fn(payload)
    path.write_text(json.dumps(payload), encoding="utf-8")


def test_preflight_can_pass_but_never_authorizes_field_release(tmp_path):
    result = audit(fixture(tmp_path))
    assert result["preflight_passed"]
    assert result["scan_counts"] == {"train": 5, "holdout": 5}
    assert result["field_release_authorized"] is False
    assert result["manual_review_required"]


def test_changed_raw_data_or_missing_altitude_is_rejected(tmp_path):
    manifest = fixture(tmp_path)
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    Path(payload["scans"][0]["raw_magnetic"]["path"]).write_text("tampered", encoding="utf-8")
    update_manifest(manifest, lambda doc: doc["scans"][1].pop("altitude_navigation"))
    result = audit(manifest)
    assert not result["preflight_passed"]
    assert any("SHA-256 mismatch" in reason for reason in result["blockers"])
    assert any("altitude_navigation: artifact record missing" in reason for reason in result["blockers"])


def test_same_session_or_raw_file_cannot_cross_train_holdout(tmp_path):
    manifest = fixture(tmp_path)
    def cross(doc):
        doc["scans"][5]["session_id"] = "train"
        doc["scans"][5]["raw_magnetic"] = doc["scans"][0]["raw_magnetic"]
    update_manifest(manifest, cross)
    result = audit(manifest)
    assert not result["preflight_passed"]
    assert any("session train crosses" in reason for reason in result["blockers"])
    assert any("file crosses train/holdout" in reason for reason in result["blockers"])


def test_legacy_self_declared_approval_is_rejected(tmp_path):
    manifest = fixture(tmp_path)
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    calibration_path = Path(payload["calibration_json"]["path"])
    calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
    calibration["deployment_qualified"] = True
    payload["calibration_json"] = artifact(calibration_path, json.dumps(calibration))
    manifest.write_text(json.dumps(payload), encoding="utf-8")
    result = audit(manifest)
    assert not result["preflight_passed"]
    assert any("legacy calibration claims deployment approval" in reason for reason in result["blockers"])


def test_controlled_measurement_label_alone_never_grants_deployment(tmp_path):
    source = tmp_path / "synthetic_claimed_physical.csv"
    output = tmp_path / "result.json"
    with source.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("scan_id", "split", "x_m", "envelope_nt",
                         "true_separation_m", "baseline_nt", "noise_nt"))
        for split in ("train", "holdout"):
            for index, vertical in enumerate((2., 3., 4., 5., 6.)):
                for step in range(-250, 251):
                    x = step / 5.
                    envelope = 1. + 20. / (1. + (x / vertical) ** 2)
                    writer.writerow((f"{split}_{index}", split, x, envelope,
                                     vertical, 1., .08))
    tool = Path(__file__).resolve().parents[2] / "tools/calibrate_v2_fwhm_shape.py"
    run = subprocess.run((sys.executable, str(tool), "--input-csv", str(source),
                          "--shape-model", "thesis_dipole", "--source",
                          "controlled_measurement", "--output-json", str(output)),
                         capture_output=True, text=True, check=False)
    assert run.returncode == 0, run.stderr + run.stdout
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["statistical_holdout_accepted"] is True
    assert payload["deployment_qualified"] is False
    assert payload["physical_provenance_review_required"] is True
