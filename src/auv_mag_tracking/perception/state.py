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
    true_burial_depth_m: Optional[float]
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
    # --- Pure-magnetic implicit path observation diagnostics ---
    magnetic_path_observation_valid: bool = False
    magnetic_path_x_m: Optional[float] = None
    magnetic_path_y_m: Optional[float] = None
    magnetic_path_heading_deg: Optional[float] = None
    magnetic_path_cross_track_offset_m: Optional[float] = None
    magnetic_path_confidence: float = 0.0
    # --- Magnetic-crossing-aligned probe control diagnostics ---
    magnetic_crossing_probe_wait_s: float = 0.0
    magnetic_crossing_probe_missed_count: int = 0
    magnetic_crossing_probe_forced_flip: bool = False
    # --- Zig-zag phase-confirmed pure-magnetic observation diagnostics ---
    magnetic_phase_observation_valid: bool = False
    magnetic_phase_x_m: Optional[float] = None
    magnetic_phase_y_m: Optional[float] = None
    magnetic_phase_heading_deg: Optional[float] = None
    magnetic_phase_amplitude_m: float = 0.0
    magnetic_phase_duration_s: float = 0.0
    magnetic_phase_confidence: float = 0.0
    magnetic_phase_detector_reason_code: float = 0.0
    magnetic_phase_detector_candidate_duration_s: float = float("nan")
    magnetic_phase_detector_axis_delta_deg: float = float("nan")
    # --- Magnetic lookahead target diagnostics ---
    magnetic_lookahead_valid: bool = False
    magnetic_lookahead_cable_point_xy_m: Optional[np.ndarray] = None
    magnetic_lookahead_target_xy_m: Optional[np.ndarray] = None
    magnetic_lookahead_heading_deg: Optional[float] = None
    magnetic_lookahead_confidence: float = 0.0
    magnetic_lookahead_age_s: float = float("inf")
    magnetic_lookahead_feed_allowed: bool = False
    magnetic_lookahead_feed_reason_code: float = 0.0
    magnetic_lookahead_feed_phase_age_s: float = float("inf")
    magnetic_lookahead_feed_innovation_m: float = float("nan")
    magnetic_lookahead_feed_axis_delta_deg: float = float("nan")
    magnetic_lookahead_feed_local_residual_m: float = float("nan")
    # --- D2 shadow +/- axis hypothesis selector diagnostics ---
    shadow_axis_hypothesis_valid: bool = False
    shadow_axis_hypothesis_count: int = 0
    shadow_axis_selected_sign: float = 0.0
    shadow_axis_selected_score: float = 0.0
    shadow_axis_score_margin: float = 0.0
    shadow_axis_positive_score: float = float("nan")
    shadow_axis_negative_score: float = float("nan")
    shadow_axis_target_xy_m: Optional[np.ndarray] = None
    shadow_axis_heading_deg: Optional[float] = None
    shadow_axis_age_s: float = float("inf")
    shadow_axis_validation_passed: bool = False
    shadow_axis_validation_reason_code: float = 0.0
    shadow_axis_validation_score_deficit: float = 0.0
    shadow_axis_validation_margin_deficit: float = 0.0
    shadow_axis_validation_age_over_s: float = 0.0
    # --- D3 shadow dual gate (selector validation + lookahead feed) diagnostics ---
    shadow_axis_dual_gate_enabled: bool = False
    shadow_axis_dual_gate_passed: bool = False
    shadow_axis_dual_gate_reason_code: float = 0.0
    # --- D4 progress-aligned shadow gate diagnostics ---
    shadow_axis_progress_alignment_enabled: bool = False
    shadow_axis_progress_alignment_passed: bool = False
    shadow_axis_progress_alignment_reason_code: float = 0.0
    shadow_axis_progress_alignment_dot: float = float("nan")
    shadow_axis_progress_alignment_reference_age_s: float = float("inf")
    shadow_axis_progress_aligned_dual_gate_passed: bool = False
    shadow_axis_progress_aligned_dual_gate_reason_code: float = 0.0
    shadow_axis_progress_aligned_candidate_valid: bool = False
    shadow_axis_progress_aligned_candidate_reason_code: float = 0.0
    shadow_axis_progress_aligned_candidate_sign: float = 0.0
    shadow_axis_progress_aligned_candidate_score: float = 0.0
    shadow_axis_progress_aligned_candidate_task_score: float = 0.0
    shadow_axis_progress_aligned_candidate_combined_score: float = 0.0
    shadow_axis_progress_aligned_candidate_margin: float = 0.0
    shadow_axis_progress_aligned_candidate_dot: float = float("nan")
    shadow_axis_progress_aligned_candidate_count: int = 0
    shadow_axis_progress_proxy_valid: bool = False
    shadow_axis_progress_proxy_source_code: float = 0.0
    shadow_axis_progress_proxy_age_s: float = float("inf")
    shadow_axis_progress_proxy_confidence: float = 0.0
    shadow_axis_progress_proxy_heading_deg: Optional[float] = None
    shadow_axis_route_bound_proxy_valid: bool = False
    shadow_axis_route_bound_proxy_source_code: float = 0.0
    shadow_axis_route_bound_proxy_progress_m: float = float("nan")
    shadow_axis_route_bound_proxy_distance_m: float = float("nan")
    shadow_axis_route_bound_proxy_heading_deg: Optional[float] = None
    shadow_axis_route_bound_candidate_dot: float = float("nan")
    # --- D4 map-frame closed-loop route projection diagnostics ---
    map_frame_projection_enabled: bool = False
    map_frame_progress_m: float = float("nan")
    map_frame_lateral_m: float = float("nan")
    map_frame_projection_distance_m: float = float("nan")
    map_frame_consistency_score: float = float("nan")
    map_frame_projection_untrusted: bool = False
    # --- Online prior-to-real cable alignment diagnostics ---
    prior_alignment_enabled: bool = False
    prior_alignment_accepted: bool = False
    prior_alignment_reason_code: float = 0.0
    prior_alignment_residual_m: float = float("nan")
    prior_alignment_residual_x_m: float = float("nan")
    prior_alignment_residual_y_m: float = float("nan")
    prior_alignment_step_m: float = 0.0
    prior_alignment_step_x_m: float = 0.0
    prior_alignment_step_y_m: float = 0.0
    prior_alignment_translation_x_m: float = 0.0
    prior_alignment_translation_y_m: float = 0.0
    prior_alignment_rotation_deg: float = 0.0
    prior_alignment_heading_residual_deg: float = float("nan")
    prior_alignment_confidence: float = 0.0
    prior_alignment_progress_m: float = float("nan")
    # --- Magnetic burial-depth inversion diagnostics ---
    burial_inversion_uncertainty_m: Optional[float] = None
    # --- Local path estimator side-channel diagnostics ---
    local_path_model_code: float = 0.0
    local_path_heading_deg: Optional[float] = None
    local_path_confidence: float = 0.0
    local_path_residual_m: float = float("inf")
    local_path_radius_m: float = float("inf")
    local_path_tracking_state: str = "collecting"
    # --- Reacquisition observable-region diagnostics ---
    reacquire_region_center_xy_m: Optional[np.ndarray] = None
    reacquire_region_heading_deg: Optional[float] = None
    reacquire_region_half_length_m: float = 0.0
    reacquire_region_half_width_m: float = 0.0
    reacquire_region_confidence: float = 0.0
    reacquire_region_score: float = 0.0
    reacquire_region_reason: str = "none"
