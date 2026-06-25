"""Thin wrapper kept for backward compatibility — superseded by ``tools/visualize.py``.

The historical 800-line diagnostic re-implemented its own simulation loop and health
metrics; both now live in the single-source ``auv_mag_tracking.viz`` package (Phase 2V).
This shim simply runs one case through that package so the long-standing command
``python tools/diagnose_heading_error.py --case caseN`` still produces a health report.

For batch runs + cross-case showcase use ``python tools/visualize.py --all``.
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
    save_run_report,
    simulate_case,
)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Single-case health report (thin wrapper over tools/visualize.py)"
    )
    parser.add_argument("--case", default="case1", help="scenario name (default: case1)")
    parser.add_argument("--deployment", action="store_true", help="disable nominal route prior")
    parser.add_argument("--max-steps", type=int, default=None, help="cap simulation steps")
    args = parser.parse_args()

    run_dir = WORKSPACE_ROOT / "results" / datetime.now().strftime("%Y%m%d_%H%M%S")
    case_dir = run_dir / args.case
    case_dir.mkdir(parents=True, exist_ok=True)

    print(f"Running diagnostic for {args.case} ...")
    record = simulate_case(args.case, deployment_mode=args.deployment, max_steps=args.max_steps)
    metrics = compute_health_metrics(record)
    fig_paths = render_run(record, metrics, case_dir / "figures")
    record.save_npz(case_dir / "record.npz")
    save_run_report(metrics, fig_paths, case_dir / "report.md")

    print(
        f"health {health_score(metrics):.0f}/100  "
        f"mean_err {metrics.mean_heading_error_deg:.1f}deg  "
        f"TRACK {metrics.track_active_fraction*100:.0f}%  "
        f"switches {metrics.mode_switches}"
    )
    print(
        f"track_xt {metrics.track_mean_cross_track_m:.1f}m  "
        f"track_vehicle_err {metrics.track_mean_vehicle_heading_error_deg:.1f}deg  "
        f"final_xt {metrics.final_cross_track_m:.1f}m"
    )
    print(f"report: {case_dir / 'report.md'}")


if __name__ == "__main__":
    main()
