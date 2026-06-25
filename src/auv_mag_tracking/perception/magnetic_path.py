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
