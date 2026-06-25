"""Observable-region selection for deployment reacquisition.

This module keeps the strategic question out of the controller: when tracking
goes stale, it proposes a bounded region where the sensors are likely to recover
the cable.  It uses only observed cable states and vehicle pose, never the
simulated route truth.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from ..math_utils import heading_from_direction_xy, smallest_angle_error_deg


@dataclass
class ObservableRegion:
    """Bounded local region selected for reacquisition."""

    center_xy_m: np.ndarray
    heading_deg: float
    half_length_m: float
    half_width_m: float
    confidence: float
    score: float
    reason: str


@dataclass
class _TrustedCableState:
    anchor_xy_m: np.ndarray
    heading_deg: float
    confidence: float
    time_s: float
    curvature_1pm: float = 0.0


class ObservableRegionSelector:
    """Select the next bounded observable region from recent cable observations."""

    def __init__(
        self,
        forward_distance_m: float = 48.0,
        turn_lateral_offset_m: float = 60.0,
        half_length_m: float = 36.0,
        half_width_m: float = 24.0,
        max_anchor_age_s: float = 120.0,
        min_turn_curvature_1pm: float = 1.0 / 160.0,
        progressive_forward_enabled: bool = False,
        progressive_margin_m: float = 12.0,
    ) -> None:
        self.forward_distance_m = max(1.0, float(forward_distance_m))
        self.turn_lateral_offset_m = max(1.0, float(turn_lateral_offset_m))
        self.half_length_m = max(1.0, float(half_length_m))
        self.half_width_m = max(1.0, float(half_width_m))
        self.max_anchor_age_s = max(1.0, float(max_anchor_age_s))
        self.min_turn_curvature_1pm = max(0.0, float(min_turn_curvature_1pm))
        self.progressive_forward_enabled = bool(progressive_forward_enabled)
        self.progressive_margin_m = max(0.0, float(progressive_margin_m))
        self._trusted_state: Optional[_TrustedCableState] = None
        self._previous_trusted_state: Optional[_TrustedCableState] = None

    def reset(self) -> None:
        self._trusted_state = None
        self._previous_trusted_state = None

    def update_trusted_state(
        self,
        *,
        anchor_xy_m: Optional[np.ndarray],
        heading_deg: Optional[float],
        confidence: float,
        time_s: float,
        curvature_1pm: float = 0.0,
    ) -> None:
        """Store the latest cable state that is good enough to seed a region."""
        if anchor_xy_m is None or heading_deg is None:
            return
        if confidence <= 0.0:
            return
        next_state = _TrustedCableState(
            anchor_xy_m=np.asarray(anchor_xy_m, dtype=float).copy(),
            heading_deg=float(heading_deg),
            confidence=float(np.clip(confidence, 0.0, 1.0)),
            time_s=float(time_s),
            curvature_1pm=float(curvature_1pm),
        )
        if self._trusted_state is not None:
            self._previous_trusted_state = self._trusted_state
        self._trusted_state = next_state

    def select(
        self,
        *,
        time_s: float,
        vehicle_position_xy_m: np.ndarray,
        reacquire_required: bool,
    ) -> Optional[ObservableRegion]:
        """Return the highest-scoring region when reacquisition is active."""
        if not reacquire_required or self._trusted_state is None:
            return None

        state = self._trusted_state
        anchor_age_s = max(0.0, float(time_s) - state.time_s)
        if anchor_age_s > self.max_anchor_age_s:
            return None

        heading_rad = np.deg2rad(state.heading_deg)
        tangent_xy = np.array([np.cos(heading_rad), np.sin(heading_rad)], dtype=float)
        normal_xy = np.array([-tangent_xy[1], tangent_xy[0]], dtype=float)
        vehicle_xy = np.asarray(vehicle_position_xy_m, dtype=float)

        candidates = [
            self._candidate(
                reason="forward_gate",
                center_xy_m=state.anchor_xy_m + self.forward_distance_m * tangent_xy,
                heading_deg=state.heading_deg,
                base_confidence=state.confidence,
                anchor_age_s=anchor_age_s,
                vehicle_xy=vehicle_xy,
            )
        ]
        vehicle_along_m = float(np.dot(vehicle_xy - state.anchor_xy_m, tangent_xy))
        progressive_along_m = max(
            self.forward_distance_m,
            vehicle_along_m + self.progressive_margin_m,
        )
        if self.progressive_forward_enabled and progressive_along_m > self.forward_distance_m:
            candidates.append(
                self._candidate(
                    reason="local_tangent_forward_gate",
                    center_xy_m=state.anchor_xy_m + progressive_along_m * tangent_xy,
                    heading_deg=state.heading_deg,
                    base_confidence=state.confidence,
                    anchor_age_s=anchor_age_s,
                    vehicle_xy=vehicle_xy,
                    exploration_bonus=0.08,
                )
            )

        curvature_1pm = self._effective_curvature_1pm(state)
        curvature_abs = abs(curvature_1pm)
        if curvature_abs >= self.min_turn_curvature_1pm:
            side_sign = 1.0 if curvature_1pm >= 0.0 else -1.0
            candidates.append(
                self._candidate(
                    reason="turn_side_gate",
                    center_xy_m=(
                        state.anchor_xy_m
                        + 0.5 * self.forward_distance_m * tangent_xy
                        + side_sign * self.turn_lateral_offset_m * normal_xy
                    ),
                    heading_deg=heading_from_direction_xy(tangent_xy + side_sign * normal_xy),
                    base_confidence=min(1.0, state.confidence + 0.15),
                    anchor_age_s=anchor_age_s,
                    vehicle_xy=vehicle_xy,
                    exploration_bonus=0.10,
                )
            )

        return max(candidates, key=lambda region: region.score)

    def _effective_curvature_1pm(self, state: _TrustedCableState) -> float:
        if abs(state.curvature_1pm) >= self.min_turn_curvature_1pm:
            return state.curvature_1pm
        if self._previous_trusted_state is None:
            return 0.0
        displacement_m = float(np.linalg.norm(state.anchor_xy_m - self._previous_trusted_state.anchor_xy_m))
        if displacement_m < 1e-6:
            return 0.0
        heading_delta_deg = smallest_angle_error_deg(state.heading_deg, self._previous_trusted_state.heading_deg)
        return float(np.deg2rad(heading_delta_deg) / displacement_m)

    def _candidate(
        self,
        *,
        reason: str,
        center_xy_m: np.ndarray,
        heading_deg: float,
        base_confidence: float,
        anchor_age_s: float,
        vehicle_xy: np.ndarray,
        exploration_bonus: float = 0.0,
    ) -> ObservableRegion:
        recency_score = float(np.exp(-anchor_age_s / max(self.max_anchor_age_s, 1e-6)))
        reachability_score = float(np.exp(-np.linalg.norm(center_xy_m - vehicle_xy) / 160.0))
        confidence = float(np.clip(base_confidence * recency_score, 0.0, 1.0))
        score = float(
            0.35 * recency_score
            + 0.30 * confidence
            + 0.20 * reachability_score
            + exploration_bonus
        )
        return ObservableRegion(
            center_xy_m=np.asarray(center_xy_m, dtype=float),
            heading_deg=float(heading_deg),
            half_length_m=self.half_length_m,
            half_width_m=self.half_width_m,
            confidence=confidence,
            score=score,
            reason=reason,
        )
