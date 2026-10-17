#!/usr/bin/env python3
"""Audit constant versus affine FWHM calibration on frozen field-model profiles.

This is a deterministic model-transfer check, not an independent physical trial.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from auv_mag_tracking.v2 import FwhmConfig, estimate_fwhm, write_experiment_manifest  # noqa: E402
from run_v2_cross_shape_identifiability import _fit_exponent  # noqa: E402


def run(output_dir: Path) -> dict[str, object]:
    source = ROOT / "results/v2/fwhm_model_consistency/model_consistency.csv"
    with source.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    models = sorted({row["model"] for row in rows})
    train_z = np.asarray([2.0, 4.0, 8.0])
    holdout_z = np.asarray([3.0, 6.0])
    summary: dict[str, object] = {
        "source": "frozen_deterministic_field_model_profiles",
        "train_separations_m": train_z.tolist(),
        "holdout_separations_m": holdout_z.tolist(),
        "models": {},
    }
    for model in models:
        width_by_z = {
            float(row["vertical_separation_m"]): float(row["fwhm_width_m"])
            for row in rows if row["model"] == model
        }
        train_width = np.asarray([width_by_z[z] for z in train_z])
        holdout_width = np.asarray([width_by_z[z] for z in holdout_z])
        constant_factor = float(np.median(train_z / train_width))
        affine_slope, affine_intercept = np.polyfit(train_width, train_z, 1)
        constant_errors = constant_factor * holdout_width - holdout_z
        affine_errors = affine_slope * holdout_width + affine_intercept - holdout_z
        column = {
            "legacy_segment_formula": "legacy_gain_nt_per_a",
            "physical_single_wire": "physical_single_wire_gain_nt_per_a",
            "thesis_dipole_envelope": "thesis_dipole_normalized",
        }[model]
        shape_fit_by_separation: dict[str, dict[str, float]] = {}
        for z in sorted(width_by_z):
            profile_path = ROOT / f"results/v2/fwhm_model_consistency/profiles/profile_z_{z:g}m.csv"
            profile = np.genfromtxt(profile_path, delimiter=",", names=True)
            x, y = profile["cross_track_m"], profile[column]
            result = estimate_fwhm(
                x, y,
                FwhmConfig(
                    shape_model="thesis_dipole", altitude_m=0.0,
                    smoothing_window=1, minimum_samples=101,
                    minimum_prominence_nt=1e-12, minimum_snr_db=-100.0,
                    maximum_model_residual=1.0, minimum_validity_score=0.0,
                ),
                baseline_nt=0.0, noise_nt=max(float(np.max(y)) * 1e-9, 1e-15),
            )
            normalized = (result.smoothed_envelope_nt - result.baseline_nt) / result.prominence_nt
            exponent, residual = _fit_exponent(result.x_m, normalized, result.center_x_m, result.width_m)
            shape_fit_by_separation[str(z)] = {"fitted_exponent": exponent, "normalized_residual": residual}
        summary["models"][model] = {
            "constant_factor": constant_factor,
            "affine_slope": float(affine_slope),
            "affine_intercept_m": float(affine_intercept),
            "constant_holdout_errors_m": constant_errors.tolist(),
            "affine_holdout_errors_m": affine_errors.tolist(),
            "constant_holdout_mae_m": float(np.mean(np.abs(constant_errors))),
            "affine_holdout_mae_m": float(np.mean(np.abs(affine_errors))),
            "shape_fit_by_separation": shape_fit_by_separation,
        }
    output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = output_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    write_experiment_manifest(
        output_dir, experiment="v2_legacy_width_transfer",
        config={"train_separations_m": train_z.tolist(), "holdout_separations_m": holdout_z.tolist(),
                "source": str(source.relative_to(ROOT))},
        artifact_paths=[source, summary_path], repository_root=ROOT,
    )
    return summary


if __name__ == "__main__":
    print(json.dumps(run(ROOT / "results/v2/legacy_width_transfer"), ensure_ascii=False, indent=2))
