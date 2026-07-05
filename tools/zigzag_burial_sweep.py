"""Independent zig-zag burial-depth estimation sweep.

The sweep isolates whether TRACK-stage zig-zag motion provides useful burial
depth estimates.  It mutates the calibrated straight baseline (case1) across a
small zig-zag amplitude grid and a burial-depth grid, then records global and
cycle-local burial errors.  The pass flag follows the paper-facing requirement:
cycle MAE <= 0.15 m and cycle coverage >= 0.30.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from copy import deepcopy
from pathlib import Path

import numpy as np

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.config import build_default_scenarios  # noqa: E402
from auv_mag_tracking.perception import MagneticBurialCycleEstimator  # noqa: E402
from auv_mag_tracking.viz.metrics import compute_health_metrics  # noqa: E402
from auv_mag_tracking.viz.recorder import simulate_run  # noqa: E402

ANGLE_GRID_DEG = (0.0, 5.0, 10.0, 15.0, 20.0)
BURIAL_DEPTH_GRID_M = (1.0, 1.5, 2.0)
DL_T_1278_TARGET_M = 0.15
MIN_CYCLE_COVERAGE = 0.30

FIELDNAMES = (
    "case",
    "zigzag_angle_deg",
    "burial_depth_true_m",
    "cycle_min_samples",
    "cycle_max_lateral_offset_m",
    "global_burial_mae_m",
    "global_burial_coverage",
    "cycle_burial_mae_m",
    "cycle_burial_p50_abs_error_m",
    "cycle_burial_p90_abs_error_m",
    "cycle_burial_coverage",
    "cycle_burial_mean_sigma_m",
    "cycle_burial_mean_quality",
    "track_mean_cross_track_m",
    "route_completion_ratio",
    "passed_dl_t_1278",
)


def _finite_abs(values: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    values = np.abs(values[np.isfinite(values)])
    return values


def _fmt(value: float, digits: int = 3) -> str:
    return "nan" if not math.isfinite(float(value)) else f"{float(value):.{digits}f}"


def _current_rms_a(scenario) -> float:
    if scenario.signal.mode == "dc":
        return abs(float(scenario.signal.dc_current_a))
    return abs(float(scenario.signal.ac_current_amplitude_a)) / math.sqrt(2.0)


def _offline_cycle_burial_stats(
    record,
    scenario,
    true_burial_m: float,
    cycle_min_samples: int,
    cycle_max_lateral_offset_m: float,
) -> dict[str, float]:
    """Estimate burial per zig-zag cycle from recorded strength/SNR/lateral samples.

    The online recorder's cycle-burial channel is gated by magnetic crossing
    events.  For this independent experiment we deliberately evaluate the
    estimator itself: each recorded zig-zag cycle gets a fresh
    ``MagneticBurialCycleEstimator`` and consumes near-crossing samples from
    that cycle.
    """
    cycle_id = np.asarray(record["zigzag_probe_cycle_id"], dtype=float)
    valid_cycle_ids = sorted({int(value) for value in cycle_id[np.isfinite(cycle_id)]})
    if not valid_cycle_ids:
        return {
            "mae": float("nan"),
            "p50": float("nan"),
            "p90": float("nan"),
            "coverage": 0.0,
            "sigma": float("nan"),
            "quality": float("nan"),
        }

    errors: list[float] = []
    sigmas: list[float] = []
    qualities: list[float] = []
    mature_cycles = 0
    for cid in valid_cycle_ids:
        estimator = MagneticBurialCycleEstimator(
            coupling_constant_nt_m_per_a_rms=(
                scenario.burial_inversion.coupling_constant_nt_m_per_a_rms
            ),
            current_rms_a=_current_rms_a(scenario),
            altitude_m=scenario.vehicle.altitude_above_seabed_m,
            snr_gate_db=scenario.burial_inversion.snr_gate_db,
            min_strength_nt=scenario.burial_inversion.min_strength_nt,
            min_samples=cycle_min_samples,
            max_lateral_offset_m=cycle_max_lateral_offset_m,
        )
        estimate = None
        mask = cycle_id == float(cid)
        for strength_nt, lateral_m, snr_db in zip(
            record["tracking_strength_nt"][mask],
            np.abs(record["zigzag_probe_signed_cross_track_m"][mask]),
            record["snr_db"][mask],
        ):
            estimate = estimator.update(strength_nt, lateral_m, snr_db)
        if estimate is None:
            continue
        mature_cycles += 1
        errors.append(abs(float(estimate.depth_m) - float(true_burial_m)))
        sigmas.append(float(estimate.sigma_m))
        qualities.append(float(estimate.fit_quality))

    error_array = np.asarray(errors, dtype=float)
    return {
        "mae": float(np.mean(error_array)) if error_array.size else float("nan"),
        "p50": float(np.percentile(error_array, 50.0)) if error_array.size else float("nan"),
        "p90": float(np.percentile(error_array, 90.0)) if error_array.size else float("nan"),
        "coverage": mature_cycles / float(len(valid_cycle_ids)),
        "sigma": float(np.mean(sigmas)) if sigmas else float("nan"),
        "quality": float(np.mean(qualities)) if qualities else float("nan"),
    }


def _row_for(
    angle_deg: float,
    burial_depth_m: float,
    cycle_min_samples: int,
    cycle_max_lateral_offset_m: float,
    duration_s: float,
) -> dict[str, object]:
    scenarios = build_default_scenarios()
    scenario = deepcopy(scenarios["case1"])
    scenario.name = (
        f"case_burial_zigzag_amp_{int(angle_deg)}_depth_{str(burial_depth_m).replace('.', 'p')}"
        f"_lat_{str(cycle_max_lateral_offset_m).replace('.', 'p')}_n{cycle_min_samples}"
    )
    scenario.description = (
        f"Independent burial-depth sweep: TRACK zig-zag {angle_deg:.0f} deg, "
        f"burial depth {burial_depth_m:.1f} m."
    )
    scenario.environment.burial_depth_m = float(burial_depth_m)
    scenario.tracking.track_active_zigzag_angle_deg = float(angle_deg)
    scenario.tracking.curve_track_crossing_angle_deg = 0.0
    scenario.tracking.adaptive_track_zigzag_angle_enabled = False
    scenario.duration_s = float(duration_s)
    scenario.burial_inversion.enabled = True
    if scenario.burial_inversion.coupling_constant_nt_m_per_a_rms <= 0.0:
        scenario.burial_inversion.coupling_constant_nt_m_per_a_rms = 11.4329

    record = simulate_run(scenario)
    metrics = compute_health_metrics(record)

    cycle_stats = _offline_cycle_burial_stats(
        record,
        scenario,
        burial_depth_m,
        cycle_min_samples=cycle_min_samples,
        cycle_max_lateral_offset_m=cycle_max_lateral_offset_m,
    )
    cycle_mae = cycle_stats["mae"]
    cycle_p50 = cycle_stats["p50"]
    cycle_p90 = cycle_stats["p90"]
    cycle_coverage = cycle_stats["coverage"]
    cycle_sigma = cycle_stats["sigma"]
    cycle_mean_quality = cycle_stats["quality"]

    global_errors = _finite_abs(
        record["estimated_burial_depth_m"] - record["true_burial_depth_m"]
    )
    global_valid = np.isfinite(record["estimated_burial_depth_m"])
    global_mae = float(np.mean(global_errors)) if global_errors.size else float("nan")
    global_coverage = float(np.mean(global_valid)) if global_valid.size else 0.0

    passed = (
        math.isfinite(cycle_mae)
        and cycle_mae <= DL_T_1278_TARGET_M
        and cycle_coverage >= MIN_CYCLE_COVERAGE
    )

    return {
        "case": scenario.name,
        "zigzag_angle_deg": f"{angle_deg:.0f}",
        "burial_depth_true_m": f"{burial_depth_m:.1f}",
        "cycle_min_samples": int(cycle_min_samples),
        "cycle_max_lateral_offset_m": _fmt(cycle_max_lateral_offset_m),
        "global_burial_mae_m": _fmt(global_mae),
        "global_burial_coverage": _fmt(global_coverage),
        "cycle_burial_mae_m": _fmt(cycle_mae),
        "cycle_burial_p50_abs_error_m": _fmt(cycle_p50),
        "cycle_burial_p90_abs_error_m": _fmt(cycle_p90),
        "cycle_burial_coverage": _fmt(cycle_coverage),
        "cycle_burial_mean_sigma_m": _fmt(cycle_sigma),
        "cycle_burial_mean_quality": _fmt(cycle_mean_quality),
        "track_mean_cross_track_m": _fmt(metrics.track_mean_cross_track_m),
        "route_completion_ratio": _fmt(metrics.route_completion_ratio),
        "passed_dl_t_1278": int(passed),
    }


def _parse_float_list(value: str | None, default_values: tuple[float, ...]) -> tuple[float, ...]:
    if value is None:
        return default_values
    return tuple(float(item.strip()) for item in value.split(",") if item.strip())


def _parse_int_list(value: str | None, default_values: tuple[int, ...]) -> tuple[int, ...]:
    if value is None:
        return default_values
    return tuple(int(item.strip()) for item in value.split(",") if item.strip())


def run_sweep(
    output_csv: Path,
    angles_deg: tuple[float, ...] = ANGLE_GRID_DEG,
    burial_depths_m: tuple[float, ...] = BURIAL_DEPTH_GRID_M,
    cycle_min_samples_grid: tuple[int, ...] = (3,),
    cycle_max_lateral_offsets_m: tuple[float, ...] | None = None,
    duration_s: float = 220.0,
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    if cycle_max_lateral_offsets_m is None:
        scenarios = build_default_scenarios()
        default_offset = float(scenarios["case1"].burial_inversion.max_lateral_offset_m)
        cycle_max_lateral_offsets_m = (default_offset,)

    for burial_depth_m in burial_depths_m:
        for angle_deg in angles_deg:
            for cycle_max_lateral_offset_m in cycle_max_lateral_offsets_m:
                for cycle_min_samples in cycle_min_samples_grid:
                    row = _row_for(
                        angle_deg,
                        burial_depth_m,
                        cycle_min_samples=cycle_min_samples,
                        cycle_max_lateral_offset_m=cycle_max_lateral_offset_m,
                        duration_s=duration_s,
                    )
                    rows.append(row)
                    print(
                        f"[burial] depth={burial_depth_m:.1f} angle={angle_deg:.0f} "
                        f"lat={cycle_max_lateral_offset_m:.1f} n={cycle_min_samples} "
                        f"cycle_mae={row['cycle_burial_mae_m']} "
                        f"coverage={row['cycle_burial_coverage']} "
                        f"passed={row['passed_dl_t_1278']}",
                        flush=True,
                    )

    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=WORKSPACE_ROOT / "results" / "20260705_zigzag_burial" / "zigzag_burial_sweep.csv",
        help="CSV path for zig-zag burial-depth sweep metrics.",
    )
    parser.add_argument("--angles", help="Comma-separated zig-zag angles in deg.")
    parser.add_argument("--depths", help="Comma-separated true burial depths in m.")
    parser.add_argument("--cycle-min-samples", help="Comma-separated cycle estimator min_samples grid.")
    parser.add_argument("--cycle-max-lateral-offsets", help="Comma-separated cycle estimator lateral gates in m.")
    parser.add_argument("--duration", type=float, default=220.0, help="Scenario duration in seconds.")
    args = parser.parse_args()
    rows = run_sweep(
        args.output,
        angles_deg=_parse_float_list(args.angles, ANGLE_GRID_DEG),
        burial_depths_m=_parse_float_list(args.depths, BURIAL_DEPTH_GRID_M),
        cycle_min_samples_grid=_parse_int_list(args.cycle_min_samples, (3,)),
        cycle_max_lateral_offsets_m=(
            None if args.cycle_max_lateral_offsets is None
            else _parse_float_list(args.cycle_max_lateral_offsets, ())
        ),
        duration_s=args.duration,
    )
    print(f"[burial] wrote {len(rows)} rows to {args.output}")


if __name__ == "__main__":
    main()
