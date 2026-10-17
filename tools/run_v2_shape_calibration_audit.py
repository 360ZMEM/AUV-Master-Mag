#!/usr/bin/env python3
"""Reproducible synthetic procedure audit; does not qualify physical deployment."""

from __future__ import annotations

import csv
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tools"))

from auv_mag_tracking.v2 import FwhmConfig, calibrate_shape_factor, write_experiment_manifest  # noqa: E402
from calibrate_v2_fwhm_shape import load_scans  # noqa: E402


def make_csv(path: Path, model: str, *, drift_holdout: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("scan_id", "split", "x_m", "envelope_nt", "true_separation_m", "baseline_nt", "noise_nt"))
        for split, seeds in (("train", range(50)), ("holdout", range(200, 250))):
            for seed in seeds:
                rng = np.random.default_rng(seed + (1000 if model == "physical_single_wire" else 0))
                truth = float(rng.uniform(2.5, 5.0))
                center = float(rng.uniform(-0.5, 0.5))
                baseline = float(rng.uniform(1.0, 3.0))
                gain = float(rng.uniform(12.0, 25.0))
                noise = float(rng.uniform(0.01, 0.06))
                # Deliberately shift apparent width while keeping the same
                # normalized shape. Only independent geometry truth detects it.
                apparent_z = truth * (1.5 if drift_holdout and split == "holdout" else 1.0)
                x = np.linspace(-30.0, 30.0, 401)
                q = ((x - center) / apparent_z) ** 2
                y = baseline + (gain / np.sqrt(1 + q) if model == "physical_single_wire" else gain / (1 + q))
                y += rng.normal(0.0, noise, x.size)
                writer.writerows((f"{split}_{seed}", split, float(xi), float(yi), truth, baseline, noise) for xi, yi in zip(x, y))


def run(output_dir: Path) -> dict[str, object]:
    cases = {
        "matched_wire": ("physical_single_wire", False),
        "matched_dipole": ("thesis_dipole", False),
        "same_shape_factor_drift": ("physical_single_wire", True),
    }
    summary: dict[str, object] = {"source": "simulation_only", "cases": {}}
    artifacts: list[Path] = []
    for name, (model, drift) in cases.items():
        path = output_dir / f"{name}.csv"
        make_csv(path, model, drift_holdout=drift)
        artifacts.append(path)
        scans = load_scans(path)
        config = FwhmConfig(
            shape_model=model,
            shape_factor=0.5 if model == "thesis_dipole" else 1 / (2 * math.sqrt(3)),
            altitude_m=0.0,
        )
        summary["cases"][name] = asdict(calibrate_shape_factor(scans["train"], scans["holdout"], config))
    output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = output_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    artifacts.append(summary_path)
    write_experiment_manifest(
        output_dir, experiment="v2_shape_calibration_procedure_audit",
        config={"seed_train": "0:49", "seed_holdout": "200:249", "source": "simulation_only", "cases": list(cases)},
        artifact_paths=artifacts, repository_root=ROOT,
    )
    return summary


if __name__ == "__main__":
    print(json.dumps(run(ROOT / "results/v2/shape_calibration_audit"), ensure_ascii=False, indent=2))
