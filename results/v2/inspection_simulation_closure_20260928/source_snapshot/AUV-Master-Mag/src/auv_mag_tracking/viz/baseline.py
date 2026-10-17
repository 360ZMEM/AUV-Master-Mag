"""Committed milestone baselines for the progress-showcase view.

``results/`` is git-ignored, so the pre-fix run archives cannot serve as a
durable reference for "how much did the refactor improve things?".  This module
freezes the milestone numbers as plain, committed constants so the progress
comparison is reproducible from a clean checkout — no dependency on local
``results/`` directories.

GUI/Logic separation: pure data only (no I/O, no plotting).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict


@dataclass(frozen=True)
class MilestoneMetrics:
    """单个里程碑下某场景的关键指标快照（与 HealthMetrics 关键子集对齐）。"""

    case_name: str
    health: float
    mean_heading_error_deg: float
    track_active_fraction: float
    mode_switches: int


# --- PRE_2G: the switch-storm cross-section just before Phase 2G ---
# Source: results/20260623_171744/showcase.md (Phase 2V baseline run, measured).
# This is the dominant "before" the progress view compares against, because the
# FSM hysteresis + time-hold fix (Phase 2G) is the headline structural gain.
PRE_2G: Dict[str, MilestoneMetrics] = {
    "case1": MilestoneMetrics("case1", health=93.0, mean_heading_error_deg=4.2,
                              track_active_fraction=0.73, mode_switches=2),
    "case2": MilestoneMetrics("case2", health=54.0, mean_heading_error_deg=6.5,
                              track_active_fraction=0.11, mode_switches=164),
    "case3": MilestoneMetrics("case3", health=59.0, mean_heading_error_deg=0.3,
                              track_active_fraction=0.04, mode_switches=134),
    "case4": MilestoneMetrics("case4", health=69.0, mean_heading_error_deg=0.6,
                              track_active_fraction=0.17, mode_switches=83),
    "case5": MilestoneMetrics("case5", health=48.0, mean_heading_error_deg=21.7,
                              track_active_fraction=0.52, mode_switches=3),
}


# --- PRE_REFACTOR: the case1 health-report fact sheet before Phase 0 ---
# Source: tools/health_report_case1.md (legacy behavior_tree build).  TRACK_ACTIVE
# did not exist yet; its closest equivalent (safe_lock_frames) was 0, so the
# track fraction is recorded as 0.0.  Kept for the long-horizon narrative.
PRE_REFACTOR: Dict[str, MilestoneMetrics] = {
    "case1": MilestoneMetrics("case1", health=float("nan"), mean_heading_error_deg=2.91,
                              track_active_fraction=0.0, mode_switches=4),
}
