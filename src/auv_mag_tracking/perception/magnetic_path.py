"""Pure-magnetic local path observation helpers.

The helper in this module deliberately stays below the main workflow.  It turns
one magnetic anomaly vector plus vehicle pose into an implicit cable-point
observation only when the local straight-wire ratio is well conditioned.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from ..math_utils import heading_from_direction_xy, smallest_angle_error_deg, unit


@dataclass
class MagneticPathObservation:
    """Implicit cable point reconstructed from magnetic vector geometry."""

    position_xy_m: np.ndarray
    heading_deg: float
    cross_track_offset_m: float
    confidence: float


@dataclass
class MagneticZigzagPhaseObservation:
    """Pure-magnetic observation accepted after a completed zig-zag crossing."""

    observation: MagneticPathObservation
    amplitude_m: float
    duration_s: float


class MagneticPathObservationBuilder:
    """Build local cable observations from magnetic anomaly history.

    For a locally straight buried cable, the horizontal magnetic anomaly is
    perpendicular to the cable and ``B_down / B_perp == y / d``.  The builder
    uses the horizontal anomaly to infer a tangent direction up to 180 degrees,
    aligns that tangent with heading history, and projects the vehicle position
    back to the cable centreline.

    This is only reliable with lateral excitation (e.g. zig-zag) and should be
    treated as a local observation source, not a global route prior.
    """

    def __init__(
        self,
        vertical_separation_m: float,
        min_horizontal_field_nt: float = 5.0,
        max_cross_track_m: float = 30.0,
        max_step_heading_change_deg: float = 80.0,
    ) -> None:
        self.vertical_separation_m = max(0.1, float(vertical_separation_m))
        self.min_horizontal_field_nt = max(1e-6, float(min_horizontal_field_nt))
        self.max_cross_track_m = max(0.1, float(max_cross_track_m))
        self.max_step_heading_change_deg = max(1.0, float(max_step_heading_change_deg))
        self._last_heading_deg: Optional[float] = None

    def reset(self) -> None:
        self._last_heading_deg = None

    def build(
        self,
        vehicle_position_xy_m: np.ndarray,
        anomaly_ned_nt: np.ndarray,
        movement_heading_deg: Optional[float] = None,
    ) -> Optional[MagneticPathObservation]:
        vehicle_xy = np.asarray(vehicle_position_xy_m, dtype=float)
        anomaly = np.asarray(anomaly_ned_nt, dtype=float)
        horizontal = anomaly[:2]
        horizontal_norm = float(np.linalg.norm(horizontal))
        if horizontal_norm < self.min_horizontal_field_nt:
            return None

        # Horizontal anomaly is cable-normal; cable tangent is +/-90 degrees.
        normal_xy = horizontal / horizontal_norm
        tangent_xy = unit(np.array([-normal_xy[1], normal_xy[0]], dtype=float))
        if np.linalg.norm(tangent_xy) < 1e-9:
            return None

        heading_deg = heading_from_direction_xy(tangent_xy)
        reference_heading_deg = self._last_heading_deg if self._last_heading_deg is not None else movement_heading_deg
        has_direction_reference = reference_heading_deg is not None
        if reference_heading_deg is not None:
            flipped_heading_deg = (heading_deg + 180.0) % 360.0
            if abs(smallest_angle_error_deg(flipped_heading_deg, reference_heading_deg)) < abs(
                smallest_angle_error_deg(heading_deg, reference_heading_deg)
            ):
                tangent_xy = -tangent_xy
                heading_deg = flipped_heading_deg
        if self._last_heading_deg is not None:
            if abs(smallest_angle_error_deg(heading_deg, self._last_heading_deg)) > self.max_step_heading_change_deg:
                return None

        perpendicular_xy = np.array([-tangent_xy[1], tangent_xy[0]], dtype=float)
        b_perp = float(np.dot(horizontal, perpendicular_xy))
        if abs(b_perp) < self.min_horizontal_field_nt:
            return None
        offset_m = float(anomaly[2] / b_perp * self.vertical_separation_m)
        if not np.isfinite(offset_m) or abs(offset_m) > self.max_cross_track_m:
            return None

        cable_point_xy = vehicle_xy - offset_m * perpendicular_xy
        offset_quality = max(0.0, 1.0 - abs(offset_m) / self.max_cross_track_m)
        field_quality = min(1.0, horizontal_norm / (4.0 * self.min_horizontal_field_nt))
        confidence = float(np.clip(0.35 + 0.45 * offset_quality + 0.20 * field_quality, 0.05, 0.95))
        if has_direction_reference:
            self._last_heading_deg = heading_deg
        return MagneticPathObservation(
            position_xy_m=cable_point_xy,
            heading_deg=heading_deg,
            cross_track_offset_m=offset_m,
            confidence=confidence,
        )


class MagneticZigzagPhaseDetector:
    """Accept pure-magnetic path observations only after a full lateral phase.

    The raw pure-magnetic projection is noisy in maze dropout because each
    single frame can be geometrically plausible while still pointing to the
    wrong local branch.  This detector waits for a zig-zag leg to visit both
    sides of the inferred cable axis, then emits one averaged observation from
    the two extrema.  It is intentionally local and stateful; it does not create
    a route prior.
    """

    def __init__(
        self,
        min_offset_m: float = 1.0,
        min_duration_s: float = 2.0,
        max_duration_s: float = 45.0,
        max_axis_delta_deg: float = 35.0,
    ) -> None:
        self.min_offset_m = max(0.05, float(min_offset_m))
        self.min_duration_s = max(0.0, float(min_duration_s))
        self.max_duration_s = max(self.min_duration_s + 1e-3, float(max_duration_s))
        self.max_axis_delta_deg = max(1.0, float(max_axis_delta_deg))
        self._previous_extreme: Optional[tuple[float, MagneticPathObservation]] = None
        self._current_sign: int = 0
        self._current_extreme: Optional[tuple[float, MagneticPathObservation]] = None

    def reset(self) -> None:
        self._previous_extreme = None
        self._current_sign = 0
        self._current_extreme = None

    def update(
        self,
        observation: MagneticPathObservation,
        time_s: float,
    ) -> Optional[MagneticZigzagPhaseObservation]:
        offset_m = float(observation.cross_track_offset_m)
        sign = 1 if offset_m > 0.0 else -1 if offset_m < 0.0 else 0
        if sign == 0:
            return None

        if self._current_sign == 0:
            self._current_sign = sign
            self._current_extreme = (float(time_s), observation)
            return None

        if sign != self._current_sign:
            if self._current_extreme is not None and abs(self._current_extreme[1].cross_track_offset_m) >= self.min_offset_m:
                self._previous_extreme = self._current_extreme
            self._current_sign = sign
            self._current_extreme = (float(time_s), observation)
            return None

        if (
            self._current_extreme is None
            or abs(offset_m) > abs(self._current_extreme[1].cross_track_offset_m)
        ):
            self._current_extreme = (float(time_s), observation)

        if self._previous_extreme is None or self._current_extreme is None:
            return None
        previous_time_s, previous_observation = self._previous_extreme
        current_time_s, current_observation = self._current_extreme
        if abs(current_observation.cross_track_offset_m) < self.min_offset_m:
            return None
        duration_s = current_time_s - previous_time_s
        if duration_s < self.min_duration_s or duration_s > self.max_duration_s:
            return None

        axis_delta_deg = abs(smallest_angle_error_deg(
            current_observation.heading_deg,
            previous_observation.heading_deg,
        ))
        axis_delta_deg = min(axis_delta_deg, abs(180.0 - axis_delta_deg))
        if axis_delta_deg > self.max_axis_delta_deg:
            return None

        position_xy = 0.5 * (previous_observation.position_xy_m + current_observation.position_xy_m)
        heading_deg = self._average_axis_heading_deg(previous_observation.heading_deg, current_observation.heading_deg)
        amplitude_m = 0.5 * (
            abs(previous_observation.cross_track_offset_m) + abs(current_observation.cross_track_offset_m)
        )
        confidence = float(np.clip(
            min(previous_observation.confidence, current_observation.confidence)
            * min(1.0, amplitude_m / max(self.min_offset_m, 1e-6)),
            0.05,
            0.95,
        ))
        self._previous_extreme = self._current_extreme
        return MagneticZigzagPhaseObservation(
            observation=MagneticPathObservation(
                position_xy_m=position_xy,
                heading_deg=heading_deg,
                cross_track_offset_m=0.0,
                confidence=confidence,
            ),
            amplitude_m=amplitude_m,
            duration_s=duration_s,
        )

    @staticmethod
    def _average_axis_heading_deg(first_heading_deg: float, second_heading_deg: float) -> float:
        first_rad = np.deg2rad(first_heading_deg)
        second_heading_deg = second_heading_deg % 360.0
        if abs(smallest_angle_error_deg(second_heading_deg, first_heading_deg)) > 90.0:
            second_heading_deg = (second_heading_deg + 180.0) % 360.0
        second_rad = np.deg2rad(second_heading_deg)
        vector = np.array([
            np.cos(first_rad) + np.cos(second_rad),
            np.sin(first_rad) + np.sin(second_rad),
        ])
        heading_deg = heading_from_direction_xy(vector)
        return 0.0 if heading_deg is None else heading_deg
