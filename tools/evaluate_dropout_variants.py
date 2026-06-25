"""Evaluate representative case_maze_sonar_dropout tuning variants.

This script is intentionally small and deterministic: it runs a curated set of
D0-D2 representative points without doing a parameter grid search.
"""

from __future__ import annotations

import argparse
import copy
import sys
from pathlib import Path
from typing import Callable, List, Tuple

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.config import ScenarioConfig, build_default_scenarios  # noqa: E402
from auv_mag_tracking.viz import compute_health_metrics, health_score  # noqa: E402
from auv_mag_tracking.viz.recorder import simulate_run  # noqa: E402


VariantBuilder = Callable[[ScenarioConfig], ScenarioConfig]


def _variant(name: str, apply: Callable[[ScenarioConfig], None]) -> Tuple[str, VariantBuilder]:
    def build(base: ScenarioConfig) -> ScenarioConfig:
        scenario = copy.deepcopy(base)
        scenario.name = name
        apply(scenario)
        return scenario

    return name, build


def _variants() -> List[Tuple[str, VariantBuilder]]:
    return [
        _variant("d0_baseline", lambda s: None),
        _variant("d1_delay60", lambda s: setattr(s.sonar, "fail_after_track_delay_s", 60.0)),
        _variant("d1_delay180", lambda s: setattr(s.sonar, "fail_after_track_delay_s", 180.0)),
        _variant("d1_sparse_sonar", _sparse_sonar),
        _variant("d2_local_age180", lambda s: setattr(s.tracking, "local_path_max_age_s", 180.0)),
        _variant("d2_local_capacity36", lambda s: setattr(s.tracking, "local_path_capacity", 36)),
        _variant("d2_spacing2m", lambda s: setattr(s.tracking, "local_path_min_observation_spacing_m", 2.0)),
        _variant("d2_forgetting060", lambda s: setattr(s.tracking, "forgetting_factor", 0.60)),
        _variant("d3_progressive_gate", _progressive_gate),
        _variant("d4_sparse035", lambda s: _sparse_sonar_prob(s, 0.35)),
        _variant("d4_sparse050", lambda s: _sparse_sonar_prob(s, 0.50)),
    ]


def _sparse_sonar(scenario: ScenarioConfig) -> None:
    _sparse_sonar_prob(scenario, 0.20)


def _sparse_sonar_prob(scenario: ScenarioConfig, probability: float) -> None:
    scenario.sonar.fail_after_track_active = False
    scenario.sonar.prob_detection = probability


def _progressive_gate(scenario: ScenarioConfig) -> None:
    scenario.tracking.reacquire_region_progressive_forward_enabled = True


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--phase",
        choices=("all", "d1", "d2", "d3", "d4"),
        default="all",
        help="subset of representative variants to run",
    )
    parser.add_argument("--max-steps", type=int, default=None, help="optional simulation step cap")
    args = parser.parse_args()

    base = build_default_scenarios()["case_maze_sonar_dropout"]
    print(
        "name,health,mean_err,track_xt,track_vehicle_err,route,final_dist,"
        "track_pct,switches,stop"
    )
    for name, build in _variants():
        if args.phase == "d1" and not name.startswith(("d0_", "d1_")):
            continue
        if args.phase == "d2" and not name.startswith(("d0_", "d2_")):
            continue
        if args.phase == "d3" and not name.startswith(("d0_", "d3_")):
            continue
        if args.phase == "d4" and not name.startswith(("d0_", "d4_")):
            continue
        scenario = build(base)
        record = simulate_run(scenario, max_steps=args.max_steps)
        metrics = compute_health_metrics(record)
        print(
            f"{name},{health_score(metrics):.1f},{metrics.mean_heading_error_deg:.1f},"
            f"{metrics.track_mean_cross_track_m:.1f},"
            f"{metrics.track_mean_vehicle_heading_error_deg:.1f},"
            f"{metrics.route_completion_ratio * 100.0:.1f},"
            f"{metrics.final_route_distance_m:.1f},"
            f"{metrics.track_active_fraction * 100.0:.1f},"
            f"{metrics.mode_switches},{record.metadata.get('stop_reason')}"
        )


if __name__ == "__main__":
    main()
