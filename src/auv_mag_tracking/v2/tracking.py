"""Stable bounded tracking corrections for v2."""

from __future__ import annotations

import numpy as np

from .config import TrackingConfig
from .types import TrackingCorrection


def compute_tracking_correction(
    signed_offset_m: float,
    confidence: float,
    config: TrackingConfig,
) -> TrackingCorrection:
    config.validate()
    offset = float(signed_offset_m)
    conf = float(np.clip(confidence, 0.0, 1.0))
    effective = 0.0 if abs(offset) <= config.offset_deadband_m else offset
    raw_heading = -config.heading_gain_deg_per_m * effective
    raw_lateral = -config.lateral_gain * effective
    heading = float(np.clip(raw_heading, -config.max_heading_correction_deg, config.max_heading_correction_deg))
    lateral = float(np.clip(raw_lateral, -config.max_lateral_correction_m, config.max_lateral_correction_m))
    speed = config.minimum_speed_mps + conf * (config.nominal_speed_mps - config.minimum_speed_mps)
    return TrackingCorrection(
        signed_offset_m=offset, heading_correction_deg=heading,
        lateral_correction_m=lateral, speed_mps=float(speed), confidence=conf,
        limited=not np.isclose(raw_heading, heading) or not np.isclose(raw_lateral, lateral),
    )

