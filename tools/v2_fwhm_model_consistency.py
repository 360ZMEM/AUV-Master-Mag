"""Measure FWHM/depth coefficients in the existing finite-wire field model."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.config import build_default_scenarios  # noqa: E402
from auv_mag_tracking.environment import CableEnvironment  # noqa: E402
from auv_mag_tracking.v2 import (  # noqa: E402
    FwhmConfig,
    estimate_fwhm,
    physical_polyline_field_nT,
    write_experiment_manifest,
)


def run(output_dir: Path, separations_m: tuple[float, ...]) -> dict[str, object]:
    scenario = build_default_scenarios()["case1"]
    environment = CableEnvironment(scenario)
    route = environment.field_model.route_points_ned_m
    midpoint_index = max(1, min(len(route) // 2, len(route) - 2))
    cable_point = route[midpoint_index]
    tangent = route[midpoint_index + 1, :2] - route[midpoint_index - 1, :2]
    tangent /= np.linalg.norm(tangent)
    normal = np.array([-tangent[1], tangent[0]], dtype=float)

    rows: list[dict[str, float | int | str]] = []
    profile_dir = output_dir / "profiles"
    profile_dir.mkdir(parents=True, exist_ok=True)
    for separation_m in separations_m:
        offsets = np.linspace(-6.0 * separation_m, 6.0 * separation_m, 1201)
        legacy_gain = []
        physical_gain = []
        for offset_m in offsets:
            position = np.array(
                [
                    cable_point[0] + offset_m * normal[0],
                    cable_point[1] + offset_m * normal[1],
                    cable_point[2] - separation_m,
                ],
                dtype=float,
            )
            legacy_gain.append(float(np.linalg.norm(environment.field_model.cable_field_gain_ned_nt(position))))
            physical_gain.append(
                float(
                    np.linalg.norm(
                        physical_polyline_field_nT(
                            position,
                            environment.field_model.route_segment_starts_ned_m,
                            environment.field_model.route_segment_ends_ned_m,
                            1.0,
                        )
                    )
                )
            )
        legacy_gain_array = np.asarray(legacy_gain, dtype=float)
        physical_gain_array = np.asarray(physical_gain, dtype=float)
        dipole_gain_array = 1.0 / (offsets ** 2 + separation_m ** 2)
        model_profiles = {
            "legacy_segment_formula": legacy_gain_array,
            "physical_single_wire": physical_gain_array,
            "thesis_dipole_envelope": dipole_gain_array,
        }
        for model_name, gain_array in model_profiles.items():
            shape_model = (
                "physical_single_wire"
                if model_name == "physical_single_wire"
                else "thesis_dipole"
            )
            result = estimate_fwhm(
                offsets,
                gain_array,
                FwhmConfig(
                    minimum_samples=101,
                    smoothing_window=1,
                    minimum_prominence_nt=1e-12,
                    minimum_snr_db=-100.0,
                    maximum_model_residual=1.0,
                    minimum_validity_score=0.0,
                    shape_model=shape_model,
                    shape_factor=0.5,
                    altitude_m=0.0,
                ),
                baseline_nt=0.0,
                noise_nt=max(float(np.max(gain_array)) * 1e-9, 1e-15),
            )
            if result.width_m is None:
                raise RuntimeError(
                    f"FWHM missing for {model_name} at {separation_m} m: {result.reason_codes}"
                )
            implied_shape_factor = separation_m / result.width_m
            rows.append(
                {
                    "model": model_name,
                    "vertical_separation_m": separation_m,
                    "fwhm_width_m": result.width_m,
                    "width_over_separation": result.width_m / separation_m,
                    "implied_shape_factor": implied_shape_factor,
                    "thesis_shape_factor": 0.5,
                    "infinite_wire_shape_factor": 1.0 / (2.0 * math.sqrt(3.0)),
                    "thesis_depth_relative_error": (0.5 * result.width_m - separation_m) / separation_m,
                    "profile_model_residual": result.model_residual,
                    "fwhm_valid": int(result.valid),
                    "reason_codes": "|".join(result.reason_codes),
                }
            )
        with (profile_dir / f"profile_z_{separation_m:g}m.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                (
                    "cross_track_m",
                    "legacy_gain_nt_per_a",
                    "physical_single_wire_gain_nt_per_a",
                    "thesis_dipole_normalized",
                )
            )
            writer.writerows(zip(offsets, legacy_gain_array, physical_gain_array, dipole_gain_array))

    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "model_consistency.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    factor_summary = {}
    for model_name in ("legacy_segment_formula", "physical_single_wire", "thesis_dipole_envelope"):
        factors = np.asarray(
            [float(row["implied_shape_factor"]) for row in rows if row["model"] == model_name]
        )
        factor_summary[model_name] = {
            "mean": float(np.mean(factors)),
            "std": float(np.std(factors)),
            "min": float(np.min(factors)),
            "max": float(np.max(factors)),
        }
    summary = {
        "scenario": "case1",
        "field_model": "finite_wire_biot_savart",
        "sample_location_ned_m": cable_point.tolist(),
        "separations_m": list(separations_m),
        "implied_shape_factor_by_model": factor_summary,
        "thesis_shape_factor": 0.5,
        "infinite_wire_shape_factor": 1.0 / (2.0 * math.sqrt(3.0)),
        "csv": str(csv_path),
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    write_experiment_manifest(
        output_dir,
        experiment="v2_fwhm_model_consistency",
        config={"scenario": "case1", "separations_m": list(separations_m)},
        artifact_paths=[
            output_dir / "summary.json",
            csv_path,
            *profile_dir.glob("*.csv"),
        ],
        repository_root=PROJECT_ROOT,
    )
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=PROJECT_ROOT / "results" / "v2" / "fwhm_model_consistency",
    )
    parser.add_argument("--separations", default="2,3,4,6,8")
    args = parser.parse_args()
    separations = tuple(float(item) for item in args.separations.split(",") if item.strip())
    summary = run(args.output_dir, separations)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
