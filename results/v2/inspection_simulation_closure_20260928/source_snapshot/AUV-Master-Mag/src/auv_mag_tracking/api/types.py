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
class DeploymentPerceptionConfig:
    enable_magnetic_quality: bool = True
    enable_burial_inversion: bool = True
    enable_online_prior_alignment: bool = False
    magnetic_noise_floor_nt: float = 0.05
    burial_coupling_constant_nt_m_per_a_rms: float = 11.4329
    burial_current_rms_a: float = 100.0
    burial_altitude_m: float = 6.0
    burial_snr_gate_db: float = 6.0
    burial_min_strength_nt: float = 1.0
    burial_min_samples: int = 20
    burial_window_samples: int = 240
    burial_max_lateral_offset_m: float = 1.0
    burial_max_depth_m: Optional[float] = None
    confidence_min_ready: float = 0.65
    route_offset_ready_m: float = 2.0


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


@dataclass
class CableGuidanceOutput:
    desired_heading_deg: float
    target_depth_m: float
    speed_mps: float
    mode: str
    guidance_source: str
    zigzag_width_m: float = 0.0
    commanded_turn_radius_m: float = float("inf")
    yaw_rate_deg_s: float = 0.0
    safe_lock_active: bool = False
    emergency_flag: bool = False
    diagnostics: dict[str, Any] = field(default_factory=dict)
