"""Evaluate representative case_maze_sonar_dropout tuning variants.

This script is intentionally small and deterministic: it runs curated D0-D4 and
zig-zag probe representative points without doing a parameter grid search.
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
        _variant("p1_probe3_nomag", lambda s: _zigzag_probe(s, angle_deg=3.0, magnetic_path=False)),
        _variant("p2_probe6_nomag", lambda s: _zigzag_probe(s, angle_deg=6.0, magnetic_path=False)),
        _variant("p3_probe10_nomag", lambda s: _zigzag_probe(s, angle_deg=10.0, magnetic_path=False)),
        _variant("p4_probe3_mag", lambda s: _zigzag_probe(s, angle_deg=3.0, magnetic_path=True)),
        _variant("p5_probe6_mag", lambda s: _zigzag_probe(s, angle_deg=6.0, magnetic_path=True)),
        _variant("p6_probe10_mag", lambda s: _zigzag_probe(s, angle_deg=10.0, magnetic_path=True)),
        _variant("p6d_probe10_mag_diag", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            feed_local_path=False,
        )),
        _variant("p7_probe6_mag_age180", lambda s: _zigzag_probe(s, angle_deg=6.0, magnetic_path=True, local_age_s=180.0)),
        _variant("p8_probe10_mag_age180", lambda s: _zigzag_probe(s, angle_deg=10.0, magnetic_path=True, local_age_s=180.0)),
        _variant("p9_probe10_mag_gate20", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p10_probe10_mag_gate10", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            feed_max_innovation_m=10.0,
            feed_max_axis_delta_deg=35.0,
        )),
    ]


def _sparse_sonar(scenario: ScenarioConfig) -> None:
    _sparse_sonar_prob(scenario, 0.20)


def _sparse_sonar_prob(scenario: ScenarioConfig, probability: float) -> None:
    scenario.sonar.fail_after_track_active = False
    scenario.sonar.prob_detection = probability


def _progressive_gate(scenario: ScenarioConfig) -> None:
    scenario.tracking.reacquire_region_progressive_forward_enabled = True


def _zigzag_probe(
    scenario: ScenarioConfig,
    *,
    angle_deg: float,
    magnetic_path: bool,
    feed_local_path: bool = True,
    local_age_s: float | None = None,
    min_width_m: float | None = None,
    feed_max_innovation_m: float | None = None,
    feed_max_axis_delta_deg: float | None = None,
) -> None:
    scenario.tracking.track_active_zigzag_angle_deg = angle_deg
    scenario.tracking.curve_track_crossing_angle_deg = angle_deg
    scenario.tracking.magnetic_path_observation_enabled = magnetic_path
    scenario.tracking.magnetic_path_feed_local_path = feed_local_path
    scenario.tracking.magnetic_path_min_horizontal_field_nt = 5.0
    scenario.tracking.magnetic_path_max_cross_track_m = 25.0
    if local_age_s is not None:
        scenario.tracking.local_path_max_age_s = local_age_s
    if min_width_m is not None:
        scenario.tracking.min_zigzag_width_m = min_width_m
        scenario.tracking.max_zigzag_width_m = max(scenario.tracking.max_zigzag_width_m, min_width_m)
    if feed_max_innovation_m is not None:
        scenario.tracking.magnetic_path_feed_max_innovation_m = feed_max_innovation_m
    if feed_max_axis_delta_deg is not None:
        scenario.tracking.magnetic_path_feed_max_heading_delta_deg = feed_max_axis_delta_deg


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--phase",
        choices=("all", "d1", "d2", "d3", "d4", "probe"),
        default="all",
        help="subset of representative variants to run",
    )
    parser.add_argument("--max-steps", type=int, default=None, help="optional simulation step cap")
    parser.add_argument(
        "--name",
        action="append",
        default=[],
        help="run only the named variant; may be passed multiple times",
    )
    args = parser.parse_args()

    base = build_default_scenarios()["case_maze_sonar_dropout"]
    print(
        "name,health,mean_err,track_xt,track_vehicle_err,route,final_dist,"
        "track_pct,switches,mag_probe_pct,mag_axis_err,mag_pos_err,mag_offset,burial_cov,burial_mae,stop",
        flush=True,
    )
    for name, build in _variants():
        if args.name and name not in set(args.name):
            continue
        if args.phase == "d1" and not name.startswith(("d0_", "d1_")):
            continue
        if args.phase == "d2" and not name.startswith(("d0_", "d2_")):
            continue
        if args.phase == "d3" and not name.startswith(("d0_", "d3_")):
            continue
        if args.phase == "d4" and not name.startswith(("d0_", "d4_")):
            continue
        if args.phase == "probe" and not name.startswith(("d0_", "p")):
            continue
        if args.phase == "probe" and not args.name and name.startswith(("p9_", "p10_")):
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
            f"{metrics.mode_switches},"
            f"{metrics.magnetic_path_observation_fraction * 100.0:.1f},"
            f"{metrics.magnetic_path_mean_axis_error_deg:.1f},"
            f"{metrics.magnetic_path_mean_position_error_m:.1f},"
            f"{metrics.magnetic_path_mean_cross_track_offset_m:.1f},"
            f"{metrics.burial_inversion_coverage * 100.0:.1f},"
            f"{metrics.burial_inversion_mae_m:.3f},"
            f"{record.metadata.get('stop_reason')}",
            flush=True,
        )


if __name__ == "__main__":
    main()
