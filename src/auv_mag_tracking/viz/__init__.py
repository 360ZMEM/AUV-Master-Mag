"""Unified visualization & results-showcase system.

GUI/Logic separation: the simulation produces a :class:`RunRecord`, metrics are
pure functions over it, and figures/reports are the only consumers that touch the
plotting backend / the file system.  Real-time dashboard (``main_viz``) and offline
reporting share the same metric functions, so numbers never drift.
"""

from .baseline import PRE_2G, PRE_REFACTOR, MilestoneMetrics
from .figures import (
    render_detail,
    render_overview,
    render_paper_progress_panels,
    render_paper_run_panels,
    render_paper_showcase_panels,
    render_progress,
    render_run,
    render_selector_sync,
    render_showcase,
)
from .metrics import (
    HealthMetrics,
    ProgressDelta,
    compare_to_baseline,
    compute_health_metrics,
    health_score,
    metrics_to_dict,
)
from .recorder import RunRecord, RunRecorder, simulate_case, simulate_run
from .report import save_progress_report, save_run_report, save_showcase_report

__all__ = [
    "RunRecord",
    "RunRecorder",
    "simulate_run",
    "simulate_case",
    "HealthMetrics",
    "compute_health_metrics",
    "health_score",
    "metrics_to_dict",
    "ProgressDelta",
    "compare_to_baseline",
    "MilestoneMetrics",
    "PRE_2G",
    "PRE_REFACTOR",
    "render_run",
    "render_overview",
    "render_detail",
    "render_selector_sync",
    "render_showcase",
    "render_progress",
    "render_paper_run_panels",
    "render_paper_showcase_panels",
    "render_paper_progress_panels",
    "save_run_report",
    "save_showcase_report",
    "save_progress_report",
]
