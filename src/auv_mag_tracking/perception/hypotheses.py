"""DTOs for magnetic lookahead hypotheses and probe-cycle summaries."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Tuple

import numpy as np


@dataclass(frozen=True)
class ZigzagProbeCycleSummary:
    """Cycle-level magnetic probe summary used as hypothesis supply."""

    cycle_id: int
    start_time_s: float
    end_time_s: float
    amplitude_m: float
    magnetic_crossing_count: int
    phase_event_count: int
    axis_delta_deg: float
    confidence: float
    burial_depth_m: float = float("nan")
    burial_sigma_m: float = float("nan")
    burial_quality: float = 0.0
    burial_sample_count: int = 0

    @property
    def duration_s(self) -> float:
        return max(0.0, float(self.end_time_s) - float(self.start_time_s))


@dataclass(frozen=True)
class MagneticLookaheadHypothesis:
    """One candidate local cable target for a specific +/- axis sign."""

    hypothesis_id: str
    axis_sign: float
    anchor_xy_m: np.ndarray
    direction_xy: np.ndarray
    cable_point_xy_m: np.ndarray
    lookahead_xy_m: np.ndarray
    heading_deg: float
    confidence: float
    age_s: float
    score: float
    progress_score: float
    heading_score: float
    freshness_score: float
    innovation_m: float = float("nan")
    local_residual_m: float = float("nan")
    burial_quality: float = 0.0


@dataclass(frozen=True)
class MagneticShadowHypothesisSelection:
    """Shadow-only selection result while preserving all axis candidates."""

    candidate_count: int
    selected_sign: float
    selected_score: float
    score_margin: float
    target_xy_m: np.ndarray
    heading_deg: float
    age_s: float
    candidates: Tuple[MagneticLookaheadHypothesis, ...] = field(default_factory=tuple)

    @property
    def selected_candidate(self) -> MagneticLookaheadHypothesis | None:
        for candidate in self.candidates:
            if candidate.axis_sign == self.selected_sign:
                return candidate
        return None

    @property
    def positive_score(self) -> float:
        for candidate in self.candidates:
            if candidate.axis_sign > 0.0:
                return candidate.score
        return float("nan")

    @property
    def negative_score(self) -> float:
        for candidate in self.candidates:
            if candidate.axis_sign < 0.0:
                return candidate.score
        return float("nan")
