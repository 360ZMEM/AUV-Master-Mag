"""Run DR/INS and sparse-sonar robustness boundary sweeps.

The sweep operates on the tiered prior scenarios in
``auv_mag_tracking.config`` (``case_maze_{sonar,sparse_sonar,sonar_dropout}_prior_{light,mid,heavy}``)
plus the ``_prob015`` sparse variant. Each scenario already bundles its
DR/INS drift, prior pose offsets, walking rotation drift and slight scale
distortion, so the sweep only needs to override the sonar prob_detection
when exploring sparsity boundaries.
"""

from __future__ import annotations

import argparse
import csv
import sys
from copy import deepcopy
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.config import build_default_scenarios  # noqa: E402
from auv_mag_tracking.viz.metrics import compute_health_metrics, health_score  # noqa: E402
from auv_mag_tracking.viz.recorder import simulate_run  # noqa: E402


# Tuple shape: (case_name, sonar_prob_override_or_None)
# Compact set of 6 anchors: covers sparse @ 0.15/default, dropout mid, and
# continuous-sonar mid/heavy so the boundary is bracketed on all three sonar
# regimes without paying for the full grid.
CRITICAL_RUNS = (
    ("case_maze_sparse_sonar_prior_mid_prob015", None),
    ("case_maze_sparse_sonar_prior_mid", None),
    ("case_maze_sparse_sonar_prior_heavy", None),
    ("case_maze_sonar_dropout_prior_mid", None),
    ("case_maze_sonar_prior_mid", None),
    ("case_maze_sonar_prior_heavy", None),
)

SPARSE_PROB_GRID = (0.10, 0.15, 0.20, 0.30)
QUICK_PROB_GRID = (0.15, 0.20, 0.30)

FIELDNAMES = (
    "case",
    "sonar_prob",
    "health",
    "route",
    "endpoint",
    "geometry",
    "max_jump_m",
    "large_jumps",
    "track_xt_m",
    "vehicle_err_deg",
    "final_xt_m",
    "mag_path_fraction",
    "passed",
)


def _metrics_row(case_name: str, scenario, sonar_prob: float | None) -> dict[str, object]:
    metrics = compute_health_metrics(simulate_run(scenario))
    passed = (
        metrics.endpoint_completed >= 0.5
        and metrics.maze_geometry_passed >= 0.5
        and metrics.route_progress_large_jump_count == 0
    )
    return {
        "case": case_name,
        "sonar_prob": "" if sonar_prob is None else f"{sonar_prob:.2f}",
        "health": f"{health_score(metrics):.1f}",
        "route": f"{metrics.route_completion_ratio:.3f}",
        "endpoint": int(metrics.endpoint_completed >= 0.5),
        "geometry": int(metrics.maze_geometry_passed >= 0.5),
        "max_jump_m": f"{metrics.route_progress_max_jump_m:.1f}",
        "large_jumps": metrics.route_progress_large_jump_count,
        "track_xt_m": f"{metrics.track_mean_cross_track_m:.1f}",
        "vehicle_err_deg": f"{metrics.track_mean_vehicle_heading_error_deg:.1f}",
        "final_xt_m": f"{metrics.final_cross_track_m:.1f}",
        "mag_path_fraction": f"{metrics.magnetic_path_observation_fraction:.2f}",
        "passed": int(passed),
    }


def run_boundary_sweep(
    output_csv: Path,
    full: bool = False,
    critical: bool = False,
    critical_indices: tuple[int, ...] = (),
) -> None:
    scenarios = build_default_scenarios()
    rows: list[dict[str, object]] = []
    if critical:
        indexed_runs = tuple(enumerate(CRITICAL_RUNS))
        if critical_indices:
            requested_indices = set(critical_indices)
            indexed_runs = tuple(
                (idx, run) for idx, run in indexed_runs if idx in requested_indices
            )
        for idx, (case_name, sonar_prob) in indexed_runs:
            scenario = deepcopy(scenarios[case_name])
            if sonar_prob is not None:
                scenario.sonar.prob_detection = sonar_prob
            rows.append(_metrics_row(scenario.name, scenario, sonar_prob))
            print(f"[boundary] finished critical[{idx}] {case_name}", flush=True)
    else:
        sparse_probs = SPARSE_PROB_GRID if full else QUICK_PROB_GRID
        sparse_tiers = ("light", "mid", "heavy") if full else ("mid", "heavy")
        for tier in sparse_tiers:
            base = scenarios[f"case_maze_sparse_sonar_prior_{tier}"]
            for sonar_prob in sparse_probs:
                scenario = deepcopy(base)
                scenario.sonar.prob_detection = sonar_prob
                rows.append(_metrics_row(scenario.name, scenario, sonar_prob))
                print(
                    f"[boundary] finished sparse {tier} prob={sonar_prob:.2f}", flush=True
                )
        for sonar_kind in ("sonar", "sonar_dropout"):
            for tier in sparse_tiers:
                base = scenarios[f"case_maze_{sonar_kind}_prior_{tier}"]
                scenario = deepcopy(base)
                rows.append(_metrics_row(scenario.name, scenario, None))
                print(f"[boundary] finished {sonar_kind} {tier}", flush=True)

    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)
    for row in rows:
        print(",".join(str(row[field_name]) for field_name in FIELDNAMES))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=WORKSPACE_ROOT / "results" / "20260629_prior_tiers" / "critical_sweep.csv",
        help="CSV path for boundary sweep metrics.",
    )
    parser.add_argument(
        "--full",
        action="store_true",
        help="Run the full sparse-prob × tier grid. Quick mode runs a reduced grid.",
    )
    parser.add_argument(
        "--critical",
        action="store_true",
        help="Run only the current 6-anchor boundary set (default for boundary checks).",
    )
    parser.add_argument(
        "--critical-index",
        type=int,
        nargs="*",
        default=(),
        help="Optional zero-based CRITICAL_RUNS indices to execute with --critical.",
    )
    args = parser.parse_args()
    run_boundary_sweep(
        args.output,
        full=args.full,
        critical=args.critical,
        critical_indices=tuple(args.critical_index),
    )
    print(f"[boundary] csv written to {args.output}")


if __name__ == "__main__":
    main()
