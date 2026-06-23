"""Unified visualization CLI for AUV cable tracking.

Single entry point for the offline visualization system (Phase 2V).  All output
is written under ``results/<timestamp>/``.

Examples
--------
    python tools/visualize.py --case case1            # one full report
    python tools/visualize.py --all                   # case1..5 + showcase
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
    compute_health_metrics,
    health_score,
    render_run,
    render_showcase,
    save_run_report,
    save_showcase_report,
    simulate_case,
)

DEFAULT_CASES = ["case1", "case2", "case3", "case4", "case5"]
RESULTS_ROOT = WORKSPACE_ROOT / "results"


def _process_case(case_name: str, run_dir: Path, deployment: bool, max_steps):
    """跑一例仿真，落盘 record/figures/report，返回其健康指标。"""
    print(f"[viz] simulating {case_name} ({'deployment' if deployment else 'nominal'}) ...")
    record = simulate_case(case_name, deployment_mode=deployment, max_steps=max_steps)
    metrics = compute_health_metrics(record)

    case_dir = run_dir / case_name
    fig_dir = case_dir / "figures"
    fig_paths = render_run(record, metrics, fig_dir)
    record.save_npz(case_dir / "record.npz")
    save_run_report(metrics, fig_paths, case_dir / "report.md")

    print(f"       health {health_score(metrics):.0f}/100  "
          f"mean_err {metrics.mean_heading_error_deg:.1f}deg  "
          f"TRACK {metrics.track_active_fraction*100:.0f}%  "
          f"switches {metrics.mode_switches}")
    return metrics


def main() -> None:
    parser = argparse.ArgumentParser(description="Unified AUV cable-tracking visualization")
    parser.add_argument("--case", default="case1", help="scenario name (default: case1)")
    parser.add_argument("--all", action="store_true", help="run case1..5 + showcase")
    parser.add_argument("--deployment", action="store_true", help="disable nominal route prior")
    parser.add_argument("--live", action="store_true", help="real-time dashboard via main_viz")
    parser.add_argument("--max-steps", type=int, default=None, help="cap simulation steps")
    parser.add_argument("--outdir", default=None, help="override results directory")
    args = parser.parse_args()

    if args.live:
        from auv_mag_tracking.config import build_default_scenarios
        from auv_mag_tracking.main_viz import AuvCableTrackingSimulation
        scenario = build_default_scenarios()[args.case]
        if args.deployment:
            scenario.tracking.use_nominal_route_prior = False
        AuvCableTrackingSimulation(scenario).run(enable_visualization=True)
        return

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path(args.outdir) if args.outdir else RESULTS_ROOT / timestamp
    run_dir.mkdir(parents=True, exist_ok=True)

    cases = DEFAULT_CASES if args.all else [args.case]
    metrics_list = [_process_case(c, run_dir, args.deployment, args.max_steps) for c in cases]

    if args.all:
        showcase_fig = render_showcase(metrics_list, run_dir / "showcase.png")
        save_showcase_report(metrics_list, showcase_fig, run_dir / "showcase.md")
        print(f"[viz] showcase written to {run_dir / 'showcase.png'}")

    print(f"[viz] all artifacts under: {run_dir}")


if __name__ == "__main__":
    main()
