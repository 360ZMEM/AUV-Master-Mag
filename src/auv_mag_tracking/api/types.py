"""Stable public data contracts for deployment-facing AUV magnetic tracking."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Optional

import numpy as np


@dataclass
class NavigationInput:
    time_s: float
    position_ned_m: np.ndarray
    heading_deg: float
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    speed_mps: float = 0.0
    position_std_m: float = 0.0
    heading_std_deg: float = 0.0
    source: str = "dr_ins"


@dataclass
class MagneticInput:
    time_s: float
    sample_block_nt: np.ndarray
    sample_rate_hz: float
    sensor_frame: str = "sensor"
    quality_flags: dict[str, Any] = field(default_factory=dict)


@dataclass
class SonarInput:
    time_s: float
    relative_position_body_m: Optional[np.ndarray] = None
    heading_deg: Optional[float] = None
    confidence: float = 0.0
    valid: bool = False


@dataclass
class CableTrackingOutput:
    time_s: float
    estimated_cable_xy_m: np.ndarray
    cross_track_m: float
    route_progress_m: float
    cable_heading_deg: float
    burial_depth_m: Optional[float]
    burial_sigma_m: Optional[float]
    confidence: float
    mode: str
    diagnostics: dict[str, Any] = field(default_factory=dict)
