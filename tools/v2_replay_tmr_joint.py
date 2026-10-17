#!/usr/bin/env python3
"""Audit whether the synchronized TMR--ArUco recording supports v2 FWHM.

The source recording is read only.  The tool projects the measured sensor
trajectory into coordinates parallel and normal to the fitted wire, splits the
dominant cross-wire coordinate into monotonic legs, and runs the production v2
FWHM estimator on every sufficiently long leg.  It deliberately does not turn
the fitted wire distance into burial-depth truth: the handheld trajectory is
three-dimensional and has no synchronized seabed-altitude reference.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

from auv_mag_tracking.v2 import FwhmConfig, estimate_fwhm, write_experiment_manifest


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_INPUT = (
    PROJECT_ROOT
    / "results/magnetic_analysis/20260809_r07_45hz_reanalysis/"
    "joint_geometry_rerun/1783235358_205719/aligned_timeseries.npz"
)
DEFAULT_OUTPUT = PROJECT_ROOT / "AUV-Master-Mag/results/v2/tmr_joint_replay"


def _monotonic_legs(values: np.ndarray, smoothing_window: int) -> list[tuple[int, int]]:
    radius = smoothing_window // 2
    kernel = np.ones(smoothing_window, dtype=float) / smoothing_window
    smooth = np.convolve(np.pad(values, (radius, radius), mode="edge"), kernel, mode="valid")
    direction = np.sign(np.diff(smooth))
    direction[direction == 0.0] = 1.0
    changes = np.flatnonzero(direction[1:] != direction[:-1]) + 1
    edges = np.r_[0, changes, values.size]
    return [(int(start), int(stop)) for start, stop in zip(edges[:-1], edges[1:])]


def _json_value(value: Any) -> Any:
    if isinstance(value, (np.floating, float)):
        return float(value) if math.isfinite(float(value)) else None
    if isinstance(value, (np.integer, int)):
        return int(value)
    if isinstance(value, np.ndarray):
        return [_json_value(item) for item in value.tolist()]
    if isinstance(value, (tuple, list)):
        return [_json_value(item) for item in value]
    return value


def audit_recording(
    input_path: Path,
    *,
    minimum_samples: int = 21,
    minimum_cross_span_m: float = 0.15,
    maximum_along_ratio: float = 0.25,
    maximum_secondary_ratio: float = 0.20,
) -> tuple[dict[str, Any], list[dict[str, Any]], dict[str, np.ndarray]]:
    fit_path = input_path.with_name("fit_summary.json")
    arrays = np.load(input_path)
    fit = json.loads(fit_path.read_text(encoding="utf-8"))["complex_iq_free_scale"]
    position = np.asarray(arrays["tmr_position_m"], dtype=float)
    timestamp = np.asarray(arrays["timestamp"], dtype=float)
    envelope_ut = np.asarray(arrays["vector_amplitude_ut"], dtype=float)

    wire_point = np.asarray(fit["p0_m"], dtype=float)
    wire_axis = np.asarray(fit["axis_unit"], dtype=float)
    wire_axis /= np.linalg.norm(wire_axis)
    displacement = position - wire_point
    along_wire_m = displacement @ wire_axis
    normal_vector = displacement - along_wire_m[:, None] * wire_axis
    centered_normal = normal_vector - np.mean(normal_vector, axis=0)
    _, _, basis = np.linalg.svd(centered_normal, full_matrices=False)
    cross_m = centered_normal @ basis[0]
    secondary_m = centered_normal @ basis[1]

    low_values = envelope_ut[envelope_ut <= np.percentile(envelope_ut, 20.0)]
    baseline_ut = float(np.percentile(envelope_ut, 5.0))
    noise_ut = float(1.4826 * np.median(np.abs(low_values - np.median(low_values))))
    config = FwhmConfig(
        minimum_samples=minimum_samples,
        smoothing_window=5,
        edge_fraction=0.15,
        minimum_prominence_nt=0.05,
        minimum_snr_db=3.0,
        minimum_side_samples=3,
        minimum_coverage_factor=0.10,
        maximum_model_residual=0.35,
        minimum_validity_score=0.45,
        shape_model="physical_single_wire",
        shape_factor=1.0 / (2.0 * math.sqrt(3.0)),
        altitude_m=0.0,
        shape_factor_relative_sigma=0.10,
        altitude_sigma_m=0.0,
    )

    rows: list[dict[str, Any]] = []
    for leg_id, (start, stop) in enumerate(_monotonic_legs(cross_m, 11)):
        count = stop - start
        cross_range = float(np.ptp(cross_m[start:stop])) if count else 0.0
        if count < minimum_samples or cross_range < minimum_cross_span_m:
            continue
        secondary_range = float(np.ptp(secondary_m[start:stop]))
        along_range = float(np.ptp(along_wire_m[start:stop]))
        result = estimate_fwhm(
            cross_m[start:stop],
            envelope_ut[start:stop],
            config,
            noise_nt=noise_ut,
            baseline_nt=baseline_ut,
        )
        along_ratio = along_range / cross_range
        secondary_ratio = secondary_range / cross_range
        geometry_reasons: list[str] = []
        if along_ratio > maximum_along_ratio:
            geometry_reasons.append("along_wire_excursion_excessive")
        if secondary_ratio > maximum_secondary_ratio:
            geometry_reasons.append("nonplanar_cross_scan")
        eligible = bool(result.valid and not geometry_reasons)
        rows.append(
            {
                "leg_id": leg_id,
                "start_index": start,
                "stop_index_exclusive": stop,
                "sample_count": count,
                "duration_s": float(timestamp[stop - 1] - timestamp[start]),
                "cross_span_m": cross_range,
                "secondary_span_m": secondary_range,
                "along_wire_span_m": along_range,
                "secondary_to_cross_ratio": secondary_ratio,
                "along_to_cross_ratio": along_ratio,
                "fwhm_valid": bool(result.valid),
                "geometry_valid": not geometry_reasons,
                "eligible_for_depth": eligible,
                "fwhm_m": result.width_m,
                "implied_wire_distance_m": result.vertical_separation_m,
                "profile_residual": result.model_residual,
                "validity_score": result.validity_score,
                "confidence": result.confidence,
                "reason_codes": list(dict.fromkeys((*result.reason_codes, *geometry_reasons))),
            }
        )

    summary = {
        "source": str(input_path),
        "source_used_read_only": True,
        "sample_count": int(timestamp.size),
        "duration_s": float(timestamp[-1] - timestamp[0]),
        "wire_fit_complex_r2": float(fit["complex_r2"]),
        "wire_axis_unit": wire_axis.tolist(),
        "normal_plane_primary_axis": basis[0].tolist(),
        "normal_plane_secondary_axis": basis[1].tolist(),
        "full_along_wire_span_m": float(np.ptp(along_wire_m)),
        "full_cross_span_m": float(np.ptp(cross_m)),
        "full_secondary_span_m": float(np.ptp(secondary_m)),
        "baseline_ut": baseline_ut,
        "noise_ut": noise_ut,
        "candidate_leg_count": len(rows),
        "fwhm_valid_leg_count": sum(bool(row["fwhm_valid"]) for row in rows),
        "geometry_valid_leg_count": sum(bool(row["geometry_valid"]) for row in rows),
        "eligible_depth_leg_count": sum(bool(row["eligible_for_depth"]) for row in rows),
        "eligibility_requirements": {
            "minimum_samples": minimum_samples,
            "minimum_cross_span_m": minimum_cross_span_m,
            "maximum_along_to_cross_ratio": maximum_along_ratio,
            "maximum_secondary_to_cross_ratio": maximum_secondary_ratio,
            "both_half_height_crossings": True,
        },
        "claim_boundary": (
            "The recording can exercise v2 spatial-profile rejection and quantify trajectory quality. "
            "It cannot validate absolute burial depth because it is a free-hand 3-D trajectory without "
            "a controlled planar two-sided cross-scan or synchronized seabed altitude."
        ),
    }
    coordinates = {
        "timestamp": timestamp,
        "cross_m": cross_m,
        "secondary_m": secondary_m,
        "along_wire_m": along_wire_m,
        "envelope_ut": envelope_ut,
    }
    return summary, rows, coordinates


def _write_outputs(
    output_dir: Path,
    summary: dict[str, Any],
    rows: list[dict[str, Any]],
    coordinates: dict[str, np.ndarray],
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "summary.json").write_text(
        json.dumps(_json_value(summary), ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    fieldnames = list(rows[0]) if rows else ["leg_id"]
    with (output_dir / "legs.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            encoded = dict(row)
            encoded["reason_codes"] = ";".join(row["reason_codes"])
            writer.writerow(_json_value(encoded))

    time_s = coordinates["timestamp"] - coordinates["timestamp"][0]
    fig, axes = plt.subplots(2, 1, figsize=(9.2, 7.2), constrained_layout=True)
    points = axes[0].scatter(
        coordinates["cross_m"], coordinates["envelope_ut"],
        c=time_s, s=15, cmap="viridis", alpha=0.8,
    )
    axes[0].set_xlabel("dominant cross-wire coordinate (m)")
    axes[0].set_ylabel("45 Hz vector envelope (µT)")
    axes[0].set_title("Measured TMR profile: repeated 3-D free-hand passes")
    fig.colorbar(points, ax=axes[0], label="elapsed time (s)")
    if rows:
        leg_ids = [str(row["leg_id"]) for row in rows]
        axes[1].bar(leg_ids, [row["along_to_cross_ratio"] for row in rows], label="along/cross")
        axes[1].bar(
            leg_ids,
            [row["secondary_to_cross_ratio"] for row in rows],
            bottom=[row["along_to_cross_ratio"] for row in rows],
            label="secondary/cross",
            alpha=0.75,
        )
    axes[1].axhline(0.25, color="tab:red", linestyle="--", linewidth=1, label="along limit")
    axes[1].set_xlabel("candidate monotonic leg id")
    axes[1].set_ylabel("trajectory excursion ratio")
    axes[1].set_title("Cross-scan geometry audit")
    axes[1].legend(loc="upper right")
    fig.savefig(output_dir / "tmr_joint_fwhm_audit.png", dpi=180)
    fig.savefig(output_dir / "tmr_joint_fwhm_audit.pdf")
    plt.close(fig)

    report = [
        "# TMR--ArUco v2 FWHM eligibility audit",
        "",
        f"- Aligned samples: {summary['sample_count']}",
        f"- Duration: {summary['duration_s']:.3f} s",
        f"- Candidate monotonic legs: {summary['candidate_leg_count']}",
        f"- Legs with complete estimator FWHM: {summary['fwhm_valid_leg_count']}",
        f"- Legs passing trajectory geometry: {summary['geometry_valid_leg_count']}",
        f"- Legs eligible for depth publication: {summary['eligible_depth_leg_count']}",
        "",
        "The recording is useful as a real-signal rejection test. It is not a controlled burial-depth "
        "ground-truth experiment: motion changes in two directions normal to the wire, has measurable "
        "along-wire drift, and lacks synchronized seabed altitude.",
        "",
        "No invalid leg is converted into a depth estimate.",
    ]
    (output_dir / "README.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    write_experiment_manifest(
        output_dir,
        experiment="v2_tmr_joint_fwhm_eligibility",
        config=dict(summary["eligibility_requirements"]),
        artifact_paths=[
            output_dir / "summary.json",
            output_dir / "legs.csv",
            output_dir / "README.md",
            output_dir / "tmr_joint_fwhm_audit.png",
            output_dir / "tmr_joint_fwhm_audit.pdf",
        ],
        repository_root=PROJECT_ROOT / "AUV-Master-Mag",
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    summary, rows, coordinates = audit_recording(args.input.resolve())
    _write_outputs(args.output_dir.resolve(), summary, rows, coordinates)
    print(json.dumps(_json_value(summary), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
