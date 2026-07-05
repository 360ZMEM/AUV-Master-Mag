"""Counterfactual checks for the 30 m pure-magnetic radius result."""

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
    "case",
    "route_mode",
    "radius_m",
    "prior_translation_y_m",
    "prior_rotation_deg",
    "prior_scale_x",
    "prior_correction_enabled",
    "zigzag_angle_deg",
    "sonar_dropout_enabled",
    "health",
    "route",
    "endpoint",
    "geometry",
    "max_jump_m",
    "large_jumps",
    "track_xt_m",
    "mag_path_fraction",
    "passed",
)


def _no_prior_distortion(scenario: ScenarioConfig) -> None:
    scenario.tracking.nominal_route_prior_translation_xy_m = (0.0, 0.0)
    scenario.tracking.nominal_route_prior_rotation_deg = 0.0
    scenario.tracking.nominal_route_prior_scale_xy = (1.0, 1.0)


def _no_prior_correction(scenario: ScenarioConfig) -> None:
    scenario.tracking.nominal_route_prior_observation_correction_enabled = False


def _no_zigzag(scenario: ScenarioConfig) -> None:
    scenario.tracking.track_active_zigzag_angle_deg = 0.0
    scenario.tracking.curve_track_crossing_angle_deg = 0.0
    scenario.tracking.adaptive_track_zigzag_angle_enabled = False


VARIANTS: tuple[tuple[str, Callable[[ScenarioConfig], None]], ...] = (
    ("baseline", lambda scenario: None),
    ("no_prior_distortion", _no_prior_distortion),
    ("no_prior_correction", _no_prior_correction),
    ("no_zigzag", _no_zigzag),
)


def _row(tier: str, variant: str, scenario: ScenarioConfig) -> dict[str, object]:
    metrics = compute_health_metrics(simulate_run(scenario))
    passed = (
        metrics.endpoint_completed >= 0.5
        and metrics.maze_geometry_passed >= 0.5
        and metrics.route_progress_large_jump_count == 0
    )
    radius = (
        float(scenario.environment.arc_radius_m)
        if scenario.environment.cable_route_mode == "tightening_arc"
        else float("nan")
    )
    return {
        "tier": tier,
        "variant": variant,
        "case": scenario.name,
        "route_mode": scenario.environment.cable_route_mode,
        "radius_m": "nan" if radius != radius else f"{radius:.0f}",
        "prior_translation_y_m": f"{float(scenario.tracking.nominal_route_prior_translation_xy_m[1]):.1f}",
        "prior_rotation_deg": f"{float(scenario.tracking.nominal_route_prior_rotation_deg):.1f}",
        "prior_scale_x": f"{float(scenario.tracking.nominal_route_prior_scale_xy[0]):.3f}",
        "prior_correction_enabled": int(
            scenario.tracking.nominal_route_prior_observation_correction_enabled
        ),
        "zigzag_angle_deg": f"{float(scenario.tracking.track_active_zigzag_angle_deg):.1f}",
        "sonar_dropout_enabled": int(
            scenario.sonar.fail_after_track_active
            and float(scenario.sonar.fail_after_track_delay_s) == 0.0
        ),
        "health": f"{health_score(metrics):.1f}",
        "route": f"{metrics.route_completion_ratio:.3f}",
        "endpoint": int(metrics.endpoint_completed >= 0.5),
        "geometry": int(metrics.maze_geometry_passed >= 0.5),
        "max_jump_m": f"{metrics.route_progress_max_jump_m:.1f}",
        "large_jumps": metrics.route_progress_large_jump_count,
        "track_xt_m": f"{metrics.track_mean_cross_track_m:.1f}",
        "mag_path_fraction": f"{metrics.magnetic_path_observation_fraction:.2f}",
        "passed": int(passed),
    }


def run_sweep(output_csv: Path) -> list[dict[str, object]]:
    scenarios = build_default_scenarios()
    rows: list[dict[str, object]] = []
    for tier in PRIOR_TIERS:
        base_radius = scenarios[f"case_radius_30_dropout_prior_{tier}"]
        for variant, mutate in VARIANTS:
            scenario = deepcopy(base_radius)
            scenario.name = f"{base_radius.name}__{variant}"
            mutate(scenario)
            row = _row(tier, variant, scenario)
            rows.append(row)
            print(
                f"[radius-causality] {tier}/{variant} passed={row['passed']} "
                f"health={row['health']} max_jump={row['max_jump_m']}",
                flush=True,
            )

        maze = deepcopy(scenarios[f"case_maze_sonar_dropout_prior_{tier}"])
        maze.name = f"{maze.name}__maze_reference"
        row = _row(tier, "maze_dropout_reference", maze)
        rows.append(row)
        print(
            f"[radius-causality] {tier}/maze_reference passed={row['passed']} "
            f"health={row['health']} max_jump={row['max_jump_m']}",
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
        default=WORKSPACE_ROOT / "results" / "20260705_radius_causality" / "radius_causality.csv",
    )
    args = parser.parse_args()
    rows = run_sweep(args.output)
    print(f"[radius-causality] wrote {len(rows)} rows to {args.output}")


if __name__ == "__main__":
    main()
