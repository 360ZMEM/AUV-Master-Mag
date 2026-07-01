"""Leave-one-out ablation sweep for the pure-magnetic dropout tracker.

Starting from the fully-equipped ``case_maze_sonar_dropout_prior_{tier}``
baseline (post-lock sonar dropout, distorted prior), each run turns *exactly
one* mechanism off and keeps the rest on. Comparing each ablation against the
all-on baseline isolates the marginal contribution of every mechanism and
shows *why* the method works -- not merely that it works.

The four mechanisms (per the approved plan) are:

  1. progress window projection / guard   (nominal_route_progress_guard_enabled)
  2. online prior correction              (nominal_route_prior_observation_correction_enabled)
  3. magnetic path observation            (magnetic_path_observation_enabled)
  4. adaptive zig-zag active probing       (track_active_zigzag_angle_deg / adaptive flag)

Each variant runs one deterministic simulation and records the same task-level
health metrics and pass criterion as :mod:`tools.radius_boundary_sweep`.
"""

from __future__ import annotations

import argparse
import csv
import sys
from copy import deepcopy
from pathlib import Path
from typing import Callable

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.config import ScenarioConfig, build_default_scenarios  # noqa: E402
from auv_mag_tracking.viz.metrics import compute_health_metrics, health_score  # noqa: E402
from auv_mag_tracking.viz.recorder import simulate_run  # noqa: E402

PRIOR_TIERS = ("mid", "heavy")

FIELDNAMES = (
    "tier",
    "variant",
    "ablated",
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


def _disable_progress_guard(scenario: ScenarioConfig) -> None:
    scenario.tracking.nominal_route_progress_guard_enabled = False


def _disable_prior_correction(scenario: ScenarioConfig) -> None:
    scenario.tracking.nominal_route_prior_observation_correction_enabled = False


def _disable_magnetic_path(scenario: ScenarioConfig) -> None:
    # Sonar drops out after lock; without the magnetic path observation the
    # tracker has no live cable measurement -- this is the "sonar-only" regime.
    scenario.tracking.magnetic_path_observation_enabled = False


def _disable_zigzag(scenario: ScenarioConfig) -> None:
    scenario.tracking.track_active_zigzag_angle_deg = 0.0
    scenario.tracking.curve_track_crossing_angle_deg = 0.0
    scenario.tracking.adaptive_track_zigzag_angle_enabled = False


# variant label -> (human-readable ablated mechanism, mutator)
ABLATIONS: dict[str, tuple[str, Callable[[ScenarioConfig], None]]] = {
    "baseline_all_on": ("none", lambda s: None),
    "no_progress_guard": ("progress_window_projection", _disable_progress_guard),
    "no_prior_correction": ("online_prior_correction", _disable_prior_correction),
    "no_magnetic_path": ("magnetic_path_observation", _disable_magnetic_path),
    "no_zigzag": ("adaptive_zigzag_probing", _disable_zigzag),
}


def _metrics_row(tier: str, variant: str, ablated: str, scenario: ScenarioConfig) -> dict[str, object]:
    metrics = compute_health_metrics(simulate_run(scenario))
    passed = (
        metrics.endpoint_completed >= 0.5
        and metrics.maze_geometry_passed >= 0.5
        and metrics.route_progress_large_jump_count == 0
    )
    return {
        "tier": tier,
        "variant": variant,
        "ablated": ablated,
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


def run_ablation_sweep(output_csv: Path) -> None:
    scenarios = build_default_scenarios()
    rows: list[dict[str, object]] = []
    for tier in PRIOR_TIERS:
        base_name = f"case_maze_sonar_dropout_prior_{tier}"
        base = scenarios[base_name]
        for variant, (ablated, mutate) in ABLATIONS.items():
            scenario = deepcopy(base)
            scenario.name = f"{base_name}__{variant}"
            mutate(scenario)
            row = _metrics_row(tier, variant, ablated, scenario)
            rows.append(row)
            print(
                f"[ablation] {tier}/{variant} passed={row['passed']} "
                f"health={row['health']} route={row['route']} "
                f"max_jump={row['max_jump_m']} large_jumps={row['large_jumps']}",
                flush=True,
            )

    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)
    print(f"[ablation] csv written to {output_csv}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=WORKSPACE_ROOT / "results" / "20260630_ablation" / "ablation_sweep.csv",
        help="CSV path for the leave-one-out ablation metrics.",
    )
    args = parser.parse_args()
    run_ablation_sweep(args.output)


if __name__ == "__main__":
    main()
