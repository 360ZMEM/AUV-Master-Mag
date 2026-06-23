"""Unified visualization & results-showcase system.

GUI/Logic separation: the simulation produces a :class:`RunRecord`, metrics are
pure functions over it, and figures/reports are the only consumers that touch the
plotting backend / the file system.  Real-time dashboard (``main_viz``) and offline
reporting share the same metric functions, so numbers never drift.
"""

from .figures import render_detail, render_overview, render_run, render_showcase
from .metrics import HealthMetrics, compute_health_metrics, health_score, metrics_to_dict
from .recorder import RunRecord, RunRecorder, simulate_case, simulate_run
from .report import save_run_report, save_showcase_report

__all__ = [
    "RunRecord",
    "RunRecorder",
    "simulate_run",
    "simulate_case",
    "HealthMetrics",
    "compute_health_metrics",
    "health_score",
    "metrics_to_dict",
    "render_run",
    "render_overview",
    "render_detail",
    "render_showcase",
    "save_run_report",
    "save_showcase_report",
]
