"""Sweep the pure-magnetic minimum curvature-radius boundary.

The sweep walks the ``case_radius_{R}_dropout_prior_mid`` family (initial
straight + single 90 deg constant-radius left turn, post-lock sonar dropout,
mid prior tier) from the largest bend radius down to 30 m. For each radius it
runs one deterministic simulation and records the same task-level health
metrics and pass criterion as :mod:`tools.dr_ins_boundary_sweep`. The smallest
radius whose run still passes is the pure-magnetic curvature boundary.
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

RADIUS_GRID_M = (120.0, 100.0, 80.0, 60.0, 50.0, 40.0, 35.0, 30.0)
PRIOR_TIERS = ("mid", "heavy")

FIELDNAMES = (
    "case",
    "radius_m",
    "prior_tier",
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


def _metrics_row(case_name: str, scenario, radius_m: float, tier: str) -> dict[str, object]:
    metrics = compute_health_metrics(simulate_run(scenario))
    passed = (
        metrics.endpoint_completed >= 0.5
        and metrics.maze_geometry_passed >= 0.5
        and metrics.route_progress_large_jump_count == 0
    )
    return {
        "case": case_name,
        "radius_m": f"{radius_m:.0f}",
        "prior_tier": tier,
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


def run_radius_sweep(output_csv: Path) -> None:
    scenarios = build_default_scenarios()
    rows: list[dict[str, object]] = []
    for tier in PRIOR_TIERS:
        for radius_m in RADIUS_GRID_M:
            case_name = f"case_radius_{int(round(radius_m))}_dropout_prior_{tier}"
            scenario = deepcopy(scenarios[case_name])
            row = _metrics_row(case_name, scenario, radius_m, tier)
            rows.append(row)
            print(
                f"[radius] finished {case_name} passed={row['passed']} "
                f"health={row['health']} max_jump={row['max_jump_m']}",
                flush=True,
            )

    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)
    for row in rows:
        print(",".join(str(row[field_name]) for field_name in FIELDNAMES))

    for tier in PRIOR_TIERS:
        passing = [float(r["radius_m"]) for r in rows if r["prior_tier"] == tier and r["passed"]]
        if passing:
            print(f"[radius] {tier}: minimum passing radius = {min(passing):.0f} m")
        else:
            print(f"[radius] {tier}: no radius in the grid passed")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=WORKSPACE_ROOT / "results" / "20260630_radius_boundary" / "radius_sweep.csv",
        help="CSV path for the radius boundary sweep metrics.",
    )
    args = parser.parse_args()
    run_radius_sweep(args.output)
    print(f"[radius] csv written to {args.output}")


if __name__ == "__main__":
    main()
