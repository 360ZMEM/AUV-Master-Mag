"""Unified visualization CLI for AUV cable tracking.

Single entry point for the offline visualization system (Phase 2V).  All output
is written under ``results/<timestamp>/``.

Examples
--------
    python tools/visualize.py --case case1            # one full report
    python tools/visualize.py --all                   # case1..6 + showcase
    python tools/visualize.py --variants              # case1v..6v + showcase
    python tools/visualize.py --maze                  # maze stress cases + showcase
    python tools/visualize.py --all --deployment      # deployment mode
    python tools/visualize.py --live --case case1     # real-time dashboard
"""

from __future__ import annotations

import argparse
import sys
from datetime import datetime
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.viz import (  # noqa: E402
    PRE_2G,
    compare_to_baseline,
    compute_health_metrics,
    health_score,
    render_paper_progress_panels,
    render_paper_showcase_panels,
    render_progress,
    render_run,
    render_showcase,
    save_progress_report,
    save_run_report,
    save_showcase_report,
    simulate_case,
    simulate_run,
)
from auv_mag_tracking.config import ScenarioConfig, build_default_scenarios  # noqa: E402

DEFAULT_CASES = ["case1", "case2", "case3", "case4", "case5", "case6"]
VARIANT_CASES = ["case1v", "case2v", "case3v", "case4v", "case5v", "case6v"]
MAZE_CASES = ["case_maze_sonar", "case_maze_sonar_dropout", "case_maze_sparse_sonar", "case_maze_no_sonar"]
RESULTS_ROOT = WORKSPACE_ROOT / "results"
_DURATION_OVERRIDE_S = None


def _apply_zigzag_probe(scenario: ScenarioConfig) -> None:
    """Enable the small-amplitude TRACK zig-zag magnetic probe."""
    scenario.name = f"{scenario.name}_zigzag_probe"
    scenario.tracking.track_active_zigzag_angle_deg = max(
        scenario.tracking.track_active_zigzag_angle_deg,
        3.0,
    )
    scenario.tracking.curve_track_crossing_angle_deg = max(
        scenario.tracking.curve_track_crossing_angle_deg,
        3.0,
    )
    scenario.tracking.magnetic_path_observation_enabled = scenario.tracking.use_nominal_route_prior
    scenario.tracking.magnetic_path_min_horizontal_field_nt = 5.0
    scenario.tracking.magnetic_path_max_cross_track_m = 25.0


def _process_case(case_name: str, run_dir: Path, deployment: bool, max_steps,
                  zigzag_probe: bool = False, paper_panels: bool = True):
    """跑一例仿真，落盘 record/figures/report，返回其健康指标。"""
    mode_name = "deployment" if deployment else "nominal"
    if zigzag_probe:
        mode_name += "+zigzag_probe"
    print(f"[viz] simulating {case_name} ({mode_name}) ...")
    if zigzag_probe:
        scenario = build_default_scenarios()[case_name]
        _apply_zigzag_probe(scenario)
        if deployment:
            scenario.tracking.use_nominal_route_prior = False
        record = simulate_run(
            scenario,
            deployment_mode=deployment,
            max_steps=max_steps,
            duration_override_s=_DURATION_OVERRIDE_S,
        )
    else:
        record = simulate_case(
            case_name,
            deployment_mode=deployment,
            max_steps=max_steps,
            duration_override_s=_DURATION_OVERRIDE_S,
        )
    metrics = compute_health_metrics(record)

    case_dir = run_dir / case_name
    fig_dir = case_dir / "figures"
    fig_paths = render_run(record, metrics, fig_dir, paper_panels=paper_panels)
    record.save_npz(case_dir / "record.npz")
    save_run_report(metrics, fig_paths, case_dir / "report.md")

    print(f"       health {health_score(metrics):.0f}/100  "
          f"mean_err {metrics.mean_heading_error_deg:.1f}deg  "
          f"TRACK {metrics.track_active_fraction*100:.0f}%  "
          f"switches {metrics.mode_switches}")
    print(f"       track_xt {metrics.track_mean_cross_track_m:.1f}m  "
          f"track_vehicle_err {metrics.track_mean_vehicle_heading_error_deg:.1f}deg  "
          f"final_xt {metrics.final_cross_track_m:.1f}m")
    if metrics.magnetic_path_observation_fraction > 0.0:
        print(f"       mag_probe {metrics.magnetic_path_observation_fraction*100:.0f}%  "
              f"axis_err {metrics.magnetic_path_mean_axis_error_deg:.1f}deg  "
              f"pos_err {metrics.magnetic_path_mean_position_error_m:.1f}m")
    if record.metadata:
        completion = record.metadata.get("route_completion_ratio")
        stop_reason = record.metadata.get("stop_reason")
        if completion is not None and stop_reason is not None:
            print(f"       route {completion*100:.1f}%  stop={stop_reason}")
    return metrics


def main() -> None:
    parser = argparse.ArgumentParser(description="Unified AUV cable-tracking visualization")
    parser.add_argument("--case", default="case1", help="scenario name (default: case1)")
    parser.add_argument("--all", action="store_true", help="run case1..6 + showcase")
    parser.add_argument("--variants", action="store_true", help="run case1v..6v downstream-turn variants + showcase")
    parser.add_argument("--maze", action="store_true", help="run smooth serpentine maze stress cases + showcase")
    parser.add_argument("--progress", action="store_true",
                        help="run case1..5 + before/after progress report vs committed baseline")
    parser.add_argument("--deployment", action="store_true", help="disable nominal route prior")
    parser.add_argument("--zigzag-probe", action="store_true", help="enable small TRACK zig-zag magnetic probe")
    parser.add_argument("--live", action="store_true", help="real-time dashboard via main_viz")
    parser.add_argument("--no-paper-figures", action="store_true",
                        help="disable IEEE single-column PNG+PDF paper panels under figures/paper/")
    parser.add_argument("--max-steps", type=int, default=None, help="cap simulation steps")
    parser.add_argument("--duration-s", type=float, default=None, help="override scenario duration for stress tests")
    parser.add_argument("--outdir", default=None, help="override results directory")
    args = parser.parse_args()
    global _DURATION_OVERRIDE_S
    _DURATION_OVERRIDE_S = args.duration_s

    if args.live:
        from auv_mag_tracking.main_viz import AuvCableTrackingSimulation
        scenario = build_default_scenarios()[args.case]
        if args.zigzag_probe:
            _apply_zigzag_probe(scenario)
        if args.deployment:
            scenario.tracking.use_nominal_route_prior = False
        if args.duration_s is not None:
            scenario.duration_s = float(args.duration_s)
        AuvCableTrackingSimulation(scenario).run(enable_visualization=True)
        return

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path(args.outdir) if args.outdir else RESULTS_ROOT / timestamp
    run_dir.mkdir(parents=True, exist_ok=True)

    if args.variants:
        cases = VARIANT_CASES
    elif args.maze:
        cases = MAZE_CASES
    elif args.all or args.progress:
        cases = DEFAULT_CASES
    else:
        cases = [args.case]
    paper_panels = not args.no_paper_figures
    metrics_list = [
        _process_case(c, run_dir, args.deployment, args.max_steps,
                      args.zigzag_probe, paper_panels=paper_panels)
        for c in cases
    ]

    if args.all or args.variants or args.maze:
        showcase_fig = render_showcase(metrics_list, run_dir / "showcase.png")
        save_showcase_report(metrics_list, showcase_fig, run_dir / "showcase.md")
        print(f"[viz] showcase written to {run_dir / 'showcase.png'}")
        if paper_panels:
            paper_dir = run_dir / "paper"
            paper_paths = render_paper_showcase_panels(metrics_list, paper_dir)
            print(f"[viz] showcase paper panels ({len(paper_paths)}) under {paper_dir}")

    if args.progress:
        deltas = [
            compare_to_baseline(m, PRE_2G[m.case_name])
            for m in metrics_list if m.case_name in PRE_2G
        ]
        progress_fig = render_progress(deltas, run_dir / "progress.png")
        save_progress_report(deltas, progress_fig, run_dir / "progress.md")
        total_before = sum(d.fields["switches"][0] for d in deltas)
        total_after = sum(d.fields["switches"][1] for d in deltas)
        print(f"[viz] progress written to {run_dir / 'progress.png'}  "
              f"(FSM switches {total_before:.0f} -> {total_after:.0f})")
        if paper_panels:
            paper_dir = run_dir / "paper"
            paper_paths = render_paper_progress_panels(deltas, paper_dir)
            print(f"[viz] progress paper panels ({len(paper_paths)}) under {paper_dir}")

    print(f"[viz] all artifacts under: {run_dir}")


if __name__ == "__main__":
    main()
