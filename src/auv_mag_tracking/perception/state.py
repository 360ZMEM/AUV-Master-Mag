"""Perception data contracts: fit result, peak events and the fused state."""

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class FitResult:
    """表示磁性峰值历史拟合得到的线路模型结果。

    该结果用于描述当前观测到的峰值点集是否能够稳定拟合为一条
    近似直线，并给出拟合中心、方向向量、残差以及协方差矩阵，
    供后续的航迹融合、置信度评估和失锁恢复逻辑使用。
    """

    origin_xy_m: Optional[np.ndarray]
    direction_xy: Optional[np.ndarray]
    residual_m: float
    covariance_xy_m2: Optional[np.ndarray] = None


@dataclass
class PeakEvent:
    """表示一次可用于线路更新的磁场峰值事件。

    峰值事件是峰值检测器的最终输出，携带峰值是否成立、峰值强度、
    峰值发生时间、估计位置以及估计电缆航向等信息，供感知层与控制层
    共同消费。
    """

    detected: bool
    peak_strength_nt: float = 0.0
    peak_time_s: float = 0.0
    peak_position_xy_m: Optional[np.ndarray] = None
    estimated_cable_heading_deg: Optional[float] = None


@dataclass
class PeakObservation:
    """表示用于滑动拟合的单个峰值观测样本。"""

    position_xy_m: np.ndarray
    snr_linear: float
    confidence: float
    time_s: float


@dataclass
class PeakZoneSample:
    """表示峰区内部的原始采样点及其空间位置。"""

    time_s: float
    strength_nt: float
    position_xy_m: Optional[np.ndarray]


@dataclass
class PerceptionState:
    """表示感知层对当前时刻的完整融合状态。

    该结构汇总了坐标系变换后的磁场分量、滤波强度、信噪比、峰值检出、
    拟合结果、声呐辅助信息、部署模式估计量以及安全锁诊断信息，作为
    控制层唯一的感知输入。
    """

    time_s: float
    sensor_field_nt: np.ndarray
    body_field_nt: np.ndarray
    ned_field_nt: np.ndarray
    anomaly_ned_nt: np.ndarray
    ac_component_ned_nt: np.ndarray
    filtered_strength_nt: float
    rms_strength_nt: float
    tracking_strength_nt: float
    noise_floor_nt: float
    snr: float
    snr_db: float
    magnetic_confidence: float
    sonar_confidence: float
    confidence: float
    weak_signal_flag: bool
    signal_reliable: bool
    is_ac_detected: bool
    dominant_frequency_hz: float
    peak_detected: bool
    fit_result: FitResult
    line_heading_deg: Optional[float]
    fused_heading_deg: Optional[float]
    blind_heading_deg: Optional[float]
    guidance_source: str
    safe_lock_active: bool
    zigzag_width_m: float
    sonar_status: str
    sonar_relative_position_body_m: Optional[np.ndarray]
    sonar_heading_deg: Optional[float]
    estimated_cable_point_xy_m: Optional[np.ndarray]
    estimated_path_points_xy_m: np.ndarray
    estimated_path_covariance_xy_m2: Optional[np.ndarray]
    fit_update_rejected: bool
    estimated_burial_depth_m: Optional[float]
    true_burial_depth_m: float
    burial_measurement_valid: bool
    last_detection_age_s: float
    detected_peak_xy_m: Optional[np.ndarray] = None
    deployment_estimated_cable_heading_deg: Optional[float] = None
    deployment_heading_confidence: float = 0.0
    deployment_reacquire_required: bool = False
    tracking_maturity: float = 0.0
    # --- Signal enhancement layer outputs ---
    envelope_gradient_nT_per_m: float = 0.0
    envelope_gradient_heading_deg: Optional[float] = None
    magnetic_vector_heading_deg: Optional[float] = None
    vector_cable_heading_deg: Optional[float] = None
    # --- Vector extraction diagnostics ---
    vector_consistency_score: float = 0.0
    attitude_leakage_risk: bool = False
    # --- Safe-lock diagnostics ---
    safe_lock_criterion_a_active: bool = False
    safe_lock_criterion_b_active: bool = False
    safe_lock_fit_invalidated: bool = False
    last_valid_peak_strength_nt: float = 0.0
    displacement_since_last_peak_m: float = 0.0
    # --- Magnetic cross-track steering signal (peak-free ratio estimator) ---
    magnetic_cross_track_offset_m: Optional[float] = None
    magnetic_cross_track_quality: float = 0.0
    # --- Magnetic burial-depth inversion diagnostics ---
    burial_inversion_uncertainty_m: Optional[float] = None
