"""Perception orchestrator: drives preprocessing, peak detection, fitting and state fusion."""

from collections import deque
from typing import Deque, Optional

import numpy as np

from ..config import ScenarioConfig
from ..math_utils import (
    body_to_ned,
    build_nominal_route_xy,
    build_polyline_projection_cache,
    heading_from_direction_xy,
    nearest_point_on_polyline,
    norm,
    project_point_to_line,
    rotation_matrix_sensor_to_body,
    sensor_to_body,
    smallest_angle_error_deg,
    wrap_angle_deg,
)
from ..perception_driver import ProcessedSignalFeatures
from ..sensor_model import BurialDepthMeasurement, MagnetometerReading, PoseMeasurement, SonarReading
from .burial_inversion import MagneticBurialInverter
from .confidence import ConfidenceEstimator
from .cross_track import MagneticCrossTrackEstimator
from .filters import LowPassFilter, MedianWindowFilter, RMSExtractor, StreamingBandpassFilter
from .fitter import WeightedSlidingWindowFitter
from .local_path import LocalCableStateEstimator, LocalPathTrackingState
from .magnetic_path import (
    MagneticLookaheadTargetBuilder,
    MagneticPathObservationBuilder,
    MagneticShadowHypothesisSelector,
    MagneticZigzagPhaseDetector,
)
from .peaks import PeakDetector
from .reacquire_region import ObservableRegionSelector
from .state import FitResult, PerceptionState
from .vector import EnvelopeGradientTracker, MagneticVectorAnalyzer


class MagneticCablePerception:
    """磁感知主流程：完成预处理、峰值检测、拟合与状态汇总。"""

    def __init__(self, scenario: ScenarioConfig) -> None:
        """根据场景配置初始化全部感知链路组件。"""
        self.scenario = scenario
        self.background_field_ned_nt = np.asarray(scenario.environment.background_field_ned_nt, dtype=float)
        self.sensor_to_body_matrix = rotation_matrix_sensor_to_body(*scenario.sensor.static_rotation_euler_deg)
        self.median_filter = MedianWindowFilter(scenario.tracking.median_window_samples)
        self.conditioner = LowPassFilter(scenario.tracking.smoothing_time_constant_s)
        self.envelope_filter = LowPassFilter(scenario.tracking.envelope_time_constant_s)
        self.noise_floor_filter = LowPassFilter(scenario.tracking.noise_floor_time_constant_s)
        minimum_frequency_hz = 50.0 if scenario.signal.mode == "ac_50hz" else max(10.0, scenario.signal.frequency_hz)
        self.rms_extractor = RMSExtractor(scenario.sensor.magnetometer_sample_rate_hz, minimum_frequency_hz)
        self.bandpass_filter = None
        if scenario.signal.mode != "dc":
            self.bandpass_filter = StreamingBandpassFilter(
                scenario.sensor.magnetometer_sample_rate_hz,
                scenario.signal.frequency_hz,
                scenario.signal.bandpass_half_width_hz,
            )
        self.peak_detector = PeakDetector(
            scenario.tracking.min_peak_strength_nt,
            scenario.tracking.turn_trigger_ratio,
            scenario.tracking.hysteresis_fraction,
            scenario.tracking.peak_cooldown_s,
            ascending_min_samples=scenario.tracking.peak_ascending_min_samples,
            descending_min_samples=scenario.tracking.peak_descending_min_samples,
            peak_zone_window_size=scenario.tracking.peak_zone_window_size,
        )
        self.fitter = WeightedSlidingWindowFitter(
            capacity=scenario.tracking.weighted_fitter_capacity,
            snr_floor=scenario.tracking.weighted_fitter_snr_floor,
            washout_residual_m=scenario.tracking.deployment_washout_residual_m,
            washout_snr_linear_threshold=scenario.tracking.deployment_washout_snr_linear_threshold,
            washout_retention_count=scenario.tracking.deployment_washout_retention_count,
        )
        self.local_path_estimator = LocalCableStateEstimator(
            capacity=scenario.tracking.local_path_capacity,
            local_line_window=scenario.tracking.local_path_local_line_window,
            heading_blend=scenario.tracking.local_path_heading_blend,
            min_observation_spacing_m=0.0,
            state_machine_enabled=False,
        )
        self.local_path_tracking_estimator = LocalCableStateEstimator(
            capacity=scenario.tracking.local_path_capacity,
            local_line_window=scenario.tracking.local_path_local_line_window,
            heading_blend=scenario.tracking.local_path_heading_blend,
            min_observation_spacing_m=scenario.tracking.local_path_min_observation_spacing_m,
            state_machine_enabled=(
                scenario.tracking.local_path_guidance_enabled
                and not scenario.tracking.use_nominal_route_prior
            ),
        )
        self.reacquire_region_selector = ObservableRegionSelector(
            forward_distance_m=scenario.tracking.reacquire_region_forward_distance_m,
            turn_lateral_offset_m=scenario.tracking.reacquire_region_turn_lateral_offset_m,
            half_length_m=scenario.tracking.reacquire_region_half_length_m,
            half_width_m=scenario.tracking.reacquire_region_half_width_m,
            max_anchor_age_s=scenario.tracking.local_path_max_age_s,
            progressive_forward_enabled=scenario.tracking.reacquire_region_progressive_forward_enabled,
            progressive_margin_m=scenario.tracking.reacquire_region_progressive_margin_m,
        )
        self.confidence_estimator = ConfidenceEstimator(scenario.tracking.lost_timeout_s)
        self.valid_points_xy: Deque[np.ndarray] = deque(maxlen=max(3, scenario.tracking.blind_follow_memory_size))
        self.last_confirmed_peak_strength_nt = 0.0
        self.last_accepted_fit_result = FitResult(origin_xy_m=None, direction_xy=None, residual_m=float("inf"), covariance_xy_m2=None)
        self.last_output_confidence = 0.0
        self.safe_lock_until_s = -1e9
        self.last_time_s = 0.0
        # Deployment-mode heading estimate, mirrored from the sonar-fed line fit
        # (Part A) for the deployment visualiser / harness contract.  All the old
        # ±90° crossing-disambiguation machinery is gone: the sonar-fed fitter is
        # the single heading source, so the prior-route corridor replaces it.
        self.deployment_estimated_cable_heading_deg: Optional[float] = None
        self.deployment_heading_confidence: float = 0.0
        # --- Signal enhancement layer ---
        self.envelope_tracker = EnvelopeGradientTracker(
            window_size=scenario.tracking.envelope_savgol_window,
            polyorder=scenario.tracking.envelope_savgol_polyorder,
            min_speed_mps=scenario.tracking.spatial_gradient_min_speed_mps,
        )
        self.vector_analyzer = MagneticVectorAnalyzer() if scenario.tracking.vector_heading_enabled else None
        self.cross_track_estimator = MagneticCrossTrackEstimator(
            window=scenario.tracking.mag_cross_track_window,
            min_perp_amplitude_nt=scenario.tracking.mag_cross_track_min_perp_amplitude_nt,
            quality_gate=scenario.tracking.mag_cross_track_quality_gate,
        )
        self.magnetic_path_builder = (
            MagneticPathObservationBuilder(
                vertical_separation_m=scenario.vehicle.altitude_above_seabed_m + scenario.environment.burial_depth_m,
                min_horizontal_field_nt=scenario.tracking.magnetic_path_min_horizontal_field_nt,
                max_cross_track_m=scenario.tracking.magnetic_path_max_cross_track_m,
            )
            if scenario.tracking.magnetic_path_observation_enabled
            else None
        )
        self.magnetic_phase_detector = (
            MagneticZigzagPhaseDetector(
                min_offset_m=scenario.tracking.magnetic_path_phase_min_offset_m,
                min_duration_s=scenario.tracking.magnetic_path_phase_min_duration_s,
                max_duration_s=scenario.tracking.magnetic_path_phase_max_duration_s,
                max_axis_delta_deg=scenario.tracking.magnetic_path_phase_max_axis_delta_deg,
            )
            if scenario.tracking.magnetic_path_phase_gate_enabled
            else None
        )
        self.last_magnetic_phase_time_s = -1e9
        self.magnetic_lookahead_builder = (
            MagneticLookaheadTargetBuilder(
                max_age_s=scenario.tracking.magnetic_lookahead_max_age_s,
                lookahead_distance_m=scenario.tracking.magnetic_lookahead_distance_m,
                heading_blend=scenario.tracking.magnetic_lookahead_heading_blend,
                axis_selection_enabled=scenario.tracking.magnetic_lookahead_axis_selection_enabled,
                axis_selection_min_progress_m=scenario.tracking.magnetic_lookahead_axis_selection_min_progress_m,
                axis_hysteresis_enabled=scenario.tracking.magnetic_lookahead_axis_hysteresis_enabled,
                axis_hysteresis_threshold=scenario.tracking.magnetic_lookahead_axis_hysteresis_threshold,
                axis_score_decay=scenario.tracking.magnetic_lookahead_axis_score_decay,
            )
            if scenario.tracking.magnetic_lookahead_enabled
            else None
        )
        self.magnetic_shadow_hypothesis_selector = (
            MagneticShadowHypothesisSelector(
                max_age_s=scenario.tracking.magnetic_lookahead_max_age_s,
                lookahead_distance_m=scenario.tracking.magnetic_lookahead_distance_m,
                min_progress_m=scenario.tracking.magnetic_shadow_hypothesis_min_progress_m,
            )
            if scenario.tracking.magnetic_shadow_hypothesis_enabled
            else None
        )
        self.last_magnetic_lookahead_feed_heading_deg: Optional[float] = None
        # --- Calibrated-amplitude burial-depth inverter (peak-free) ---
        burial_cfg = scenario.burial_inversion
        self.burial_inverter = (
            MagneticBurialInverter(
                coupling_constant_nt_m_per_a_rms=burial_cfg.coupling_constant_nt_m_per_a_rms,
                current_rms_a=self._signal_current_rms_a(),
                altitude_m=scenario.vehicle.altitude_above_seabed_m,
                snr_gate_db=burial_cfg.snr_gate_db,
                min_strength_nt=burial_cfg.min_strength_nt,
                min_samples=burial_cfg.min_samples,
                max_lateral_offset_m=burial_cfg.max_lateral_offset_m,
            )
            if burial_cfg.enabled
            else None
        )
        # --- Safe-lock state ---
        self.last_valid_peak_strength_nt: float = 0.0
        self.last_peak_position_xy_m: Optional[np.ndarray] = None
        self.displacement_since_last_peak_m: float = 0.0
        self.safe_lock_criterion_a_active: bool = False
        self.safe_lock_criterion_b_active: bool = False
        self.safe_lock_fit_invalidated: bool = False
        self.nominal_route_xy = build_nominal_route_xy(self.scenario.environment)
        self.nominal_route_lookup = build_polyline_projection_cache(self.nominal_route_xy)

    def _blind_heading(self) -> Optional[float]:
        """在没有可用航向先验时，根据历史有效点估计盲航向。"""
        minimum_points = 2 if self.scenario.tracking.use_nominal_route_prior else max(2, self.scenario.tracking.blind_follow_memory_size)
        if len(self.valid_points_xy) < minimum_points:
            return None
        delta_xy = self.valid_points_xy[-1] - self.valid_points_xy[0]
        if norm(delta_xy) < 1e-6:
            return None
        return heading_from_direction_xy(delta_xy)

    def _reference_heading_deg(self) -> Optional[float]:
        """返回用于局部投影和异常判断的参考航向。"""
        if not self.scenario.tracking.use_nominal_route_prior:
            if len(self.valid_points_xy) >= 2:
                delta_xy = self.valid_points_xy[-1] - self.valid_points_xy[0]
                if norm(delta_xy) >= 1e-6:
                    return heading_from_direction_xy(delta_xy)
            return None
        if self.last_accepted_fit_result.direction_xy is not None:
            return heading_from_direction_xy(self.last_accepted_fit_result.direction_xy)
        if len(self.valid_points_xy) >= 2:
            delta_xy = self.valid_points_xy[-1] - self.valid_points_xy[0]
            if norm(delta_xy) >= 1e-6:
                return heading_from_direction_xy(delta_xy)
        if self.scenario.tracking.use_nominal_route_prior:
            return float(self.scenario.environment.nominal_route_heading_deg)
        return None

    def _local_line_point(self, vehicle_position_xy_m: np.ndarray, fit_result: FitResult) -> Optional[np.ndarray]:
        """将车体位置投影到拟合直线上，得到局部线路参考点。"""
        if fit_result.origin_xy_m is None or fit_result.direction_xy is None:
            return None
        return project_point_to_line(vehicle_position_xy_m, fit_result.origin_xy_m, fit_result.direction_xy)

    def _peak_cable_observation_xy_m(
        self,
        peak_position_xy_m: np.ndarray,
        sonar_reading: Optional[SonarReading],
    ) -> Optional[np.ndarray]:
        """将峰值位置与声呐/名义路线先验融合，输出用于拟合的观测点。"""
        if sonar_reading is not None and sonar_reading.valid and sonar_reading.estimated_position_ned_m is not None:
            return sonar_reading.estimated_position_ned_m.copy()

        peak_position_xy_m = np.asarray(peak_position_xy_m, dtype=float)
        if not self.scenario.tracking.use_nominal_route_prior:
            if (
                self.last_accepted_fit_result.origin_xy_m is not None
                and self.last_accepted_fit_result.direction_xy is not None
                and np.isfinite(self.last_accepted_fit_result.residual_m)
                and self.last_accepted_fit_result.residual_m <= self.scenario.tracking.fit_acceptance_residual_m
            ):
                return project_point_to_line(
                    peak_position_xy_m,
                    self.last_accepted_fit_result.origin_xy_m,
                    self.last_accepted_fit_result.direction_xy,
                )
            return peak_position_xy_m.copy()
        if self.last_accepted_fit_result.origin_xy_m is None or self.last_accepted_fit_result.direction_xy is None:
            nearest_point_xy, _, _, _, _ = nearest_point_on_polyline(peak_position_xy_m, self.nominal_route_lookup)
            return nearest_point_xy

        reference_heading_deg = self._reference_heading_deg()
        if reference_heading_deg is None:
            nearest_point_xy, _, _, _, _ = nearest_point_on_polyline(peak_position_xy_m, self.nominal_route_lookup)
            return nearest_point_xy

        reference_direction_xy = np.array(
            [
                np.cos(np.deg2rad(reference_heading_deg)),
                np.sin(np.deg2rad(reference_heading_deg)),
            ],
            dtype=float,
        )
        anchor_xy = self.last_accepted_fit_result.origin_xy_m
        projected_peak_xy_m = project_point_to_line(peak_position_xy_m, anchor_xy, reference_direction_xy)
        if self.scenario.tracking.use_nominal_route_prior:
            nearest_point_xy, _, _, _, _ = nearest_point_on_polyline(projected_peak_xy_m, self.nominal_route_lookup)
            return nearest_point_xy
        return projected_peak_xy_m

    def _is_peak_outlier(self, peak_position_xy_m: np.ndarray) -> bool:
        """判断当前峰值是否相对既有中心线偏离过大。"""
        if not self.scenario.tracking.use_nominal_route_prior:
            return False
        if self.last_accepted_fit_result.origin_xy_m is None or self.last_accepted_fit_result.direction_xy is None:
            return False
        direction_xy = np.asarray(self.last_accepted_fit_result.direction_xy, dtype=float)
        direction_norm = np.linalg.norm(direction_xy)
        if direction_norm < 1e-6:
            return False
        direction_xy = direction_xy / direction_norm
        orthogonal_xy = np.array([-direction_xy[1], direction_xy[0]], dtype=float)
        residual_m = abs(float(np.dot(np.asarray(peak_position_xy_m, dtype=float) - self.last_accepted_fit_result.origin_xy_m, orthogonal_xy)))
        return residual_m > self.scenario.tracking.peak_outlier_rejection_distance_m

    def _bootstrap_fit_ready(self, fit_result_candidate: FitResult) -> bool:
        """判断部署模式下的拟合结果是否已满足启动共识所需条件。"""
        if self.scenario.tracking.use_nominal_route_prior:
            return True
        observation_count = len(self.fitter.peak_observations)
        if observation_count < self.scenario.tracking.deployment_bootstrap_min_peak_count:
            return False
        points_xy = np.vstack([observation.position_xy_m for observation in self.fitter.peak_observations])
        point_span_m = float(np.max(np.linalg.norm(points_xy - points_xy[0], axis=1))) if len(points_xy) > 1 else 0.0
        if point_span_m < self.scenario.tracking.deployment_bootstrap_min_span_m:
            return False
        return fit_result_candidate.direction_xy is not None and np.isfinite(fit_result_candidate.residual_m)

    @staticmethod
    def _perpendicular_spread_m2(covariance_xy_m2: Optional[np.ndarray]) -> Optional[float]:
        """从加权拟合协方差中提取垂直方向散布（较小特征值）。"""
        if covariance_xy_m2 is None:
            return None
        covariance = np.asarray(covariance_xy_m2, dtype=float)
        if covariance.shape != (2, 2) or not np.all(np.isfinite(covariance)):
            return None
        return float(np.min(np.linalg.eigvalsh(covariance)))

    def _magnetic_cross_track_offset(
        self,
        anomaly_ned_nt: np.ndarray,
        burial_measurement: BurialDepthMeasurement,
    ) -> Optional[float]:
        """从异常比值估计车辆相对电缆的带符号横向偏移（不触碰拟合器）。

        以当前拟合方向为电缆走向参考，把异常向量分解为电缆垂直水平分量与
        竖直分量；同一线电流驱动两者，故比值消去电流并满足无限长直线模型
        ``y = (B_down / B_perp) * d``，其中 ``d`` 为车体到电缆的垂直分隔。
        这是一个【转向】信号（左正右负），仅供控制器在 TRACK_ACTIVE 压线使用，
        绝不喂入中心线拟合器——其 ~1m 精度会污染拟合协方差（即 LOCK->TRACK 门限）。

        仅在拟合已稳定（声呐优先建立中心线）且比值质量达标时返回数值，从而
        避免在弯段/远离段输出错误偏移。
        """
        direction_xy = self.last_accepted_fit_result.direction_xy
        if direction_xy is None:
            return None
        direction_xy = np.asarray(direction_xy, dtype=float)
        direction_norm = float(np.linalg.norm(direction_xy))
        if direction_norm < 1e-6:
            return None
        direction_xy = direction_xy / direction_norm
        perpendicular_xy = np.array([-direction_xy[1], direction_xy[0]], dtype=float)

        b_perp = float(np.dot(anomaly_ned_nt[:2], perpendicular_xy))
        b_down = float(anomaly_ned_nt[2])
        self.cross_track_estimator.update(b_perp, b_down)

        stabilised_spread_m2 = self._perpendicular_spread_m2(self.last_accepted_fit_result.covariance_xy_m2)
        if stabilised_spread_m2 is None or stabilised_spread_m2 >= self.scenario.tracking.mag_cross_track_stabilized_cov_m2:
            return None

        burial_depth_m = burial_measurement.depth_m if burial_measurement.valid else self.scenario.environment.burial_depth_m
        vertical_separation_m = self.scenario.vehicle.altitude_above_seabed_m + burial_depth_m
        offset_m = self.cross_track_estimator.cross_track_offset_m(vertical_separation_m)
        if offset_m is None or abs(offset_m) > self.scenario.tracking.mag_cross_track_max_offset_m:
            return None
        return offset_m

    def _signal_current_rms_a(self) -> float:
        """返回电缆激励电流的 RMS 幅值（埋深反演标定单位）。"""
        signal = self.scenario.signal
        if signal.mode == "dc":
            return abs(float(signal.dc_current_a))
        return abs(float(signal.ac_current_amplitude_a)) / np.sqrt(2.0)

    def _burial_lateral_offset_m(
        self,
        vehicle_position_xy_m: np.ndarray,
        sonar_reading: Optional[SonarReading],
        fit_result: FitResult,
    ) -> Optional[float]:
        """估计车辆到电缆中心线的水平横距，用于从斜距分离埋深。

        优先用声呐确认的电缆位置（直接横距）；否则退回已接受拟合中心线的
        垂距投影。两者都不可用时返回 None，让反演器跳过本帧。
        """
        if sonar_reading is not None and sonar_reading.valid and sonar_reading.estimated_position_ned_m is not None:
            delta_xy = np.asarray(vehicle_position_xy_m, dtype=float) - np.asarray(sonar_reading.estimated_position_ned_m, dtype=float)[:2]
            return float(norm(delta_xy))
        if fit_result.origin_xy_m is not None and fit_result.direction_xy is not None:
            line_point_xy = self._local_line_point(vehicle_position_xy_m, fit_result)
            if line_point_xy is not None:
                return float(norm(np.asarray(vehicle_position_xy_m, dtype=float) - line_point_xy))
        return None

    def update(
        self,
        reading: MagnetometerReading,
        pose_measurement: PoseMeasurement,
        vehicle_position_xy_m: np.ndarray,
        burial_measurement: BurialDepthMeasurement,
        true_burial_depth_m: Optional[float] = None,
        sonar_reading: Optional[SonarReading] = None,
        signal_features: Optional[ProcessedSignalFeatures] = None,
    ) -> PerceptionState:
        """处理一帧磁测量、姿态和声呐输入，并输出完整感知状态。"""
        dt_s = max(reading.time_s - self.last_time_s, self.scenario.dt_s)
        self.last_time_s = reading.time_s
        sample_times_s = np.asarray(reading.sample_times_s, dtype=float)
        sample_block_sensor_nt = np.asarray(reading.sample_block_sensor_nt, dtype=float)
        sample_count = max(1, sample_times_s.size)
        sample_dt_s = max(dt_s / sample_count, 1.0 / max(self.scenario.sensor.magnetometer_sample_rate_hz, 1e-6))

        body_field_nt = np.zeros(3, dtype=float)
        ned_field_nt = np.zeros(3, dtype=float)
        anomaly_ned_nt = np.zeros(3, dtype=float)
        ac_component_ned_nt = np.zeros(3, dtype=float)
        filtered_strength_nt = 0.0
        rms_strength_nt = 0.0
        tracking_strength_nt = 0.0
        noise_floor_nt = max(self.scenario.sensor.noise_std_nt, 1e-6)
        snr = 0.0
        snr_db = -120.0
        signal_reliable = False
        is_ac_detected = False
        dominant_frequency_hz = 0.0

        if signal_features is not None:
            body_field_nt = sensor_to_body(reading.sensor_field_nt, self.sensor_to_body_matrix)
            ned_field_nt = body_to_ned(body_field_nt, pose_measurement.roll_deg, pose_measurement.pitch_deg, pose_measurement.heading_deg)
            anomaly_ned_nt = ned_field_nt - self.background_field_ned_nt
            ac_component_ned_nt = anomaly_ned_nt.copy() if signal_features.is_ac_detected else np.zeros(3, dtype=float)
            filtered_strength_nt = signal_features.filtered_intensity_nt
            rms_strength_nt = signal_features.target_magnitude_nt if signal_features.is_ac_detected else signal_features.processed_intensity_nt
            tracking_strength_nt = signal_features.processed_intensity_nt
            noise_floor_nt = max(signal_features.noise_floor_nt, self.scenario.sensor.noise_std_nt)
            snr = signal_features.snr_linear
            snr_db = signal_features.snr_db
            signal_reliable = signal_features.reliability_flag
            is_ac_detected = signal_features.is_ac_detected
            dominant_frequency_hz = signal_features.dominant_frequency_hz
        else:
            for sensor_sample_nt in sample_block_sensor_nt:
                body_field_nt = sensor_to_body(sensor_sample_nt, self.sensor_to_body_matrix)
                ned_field_nt = body_to_ned(body_field_nt, pose_measurement.roll_deg, pose_measurement.pitch_deg, pose_measurement.heading_deg)
                anomaly_ned_nt = ned_field_nt - self.background_field_ned_nt
                ac_component_ned_nt = anomaly_ned_nt.copy()
                if self.bandpass_filter is not None:
                    ac_component_ned_nt = self.bandpass_filter.update(anomaly_ned_nt)

                instantaneous_strength_nt = norm(ac_component_ned_nt)
                median_strength_nt = self.median_filter.update(instantaneous_strength_nt)
                filtered_strength_nt = self.conditioner.update(median_strength_nt, sample_dt_s)
                rms_strength_nt = self.rms_extractor.update(instantaneous_strength_nt)
                raw_tracking_strength_nt = rms_strength_nt if self.scenario.signal.mode != "dc" else filtered_strength_nt
                tracking_strength_nt = self.envelope_filter.update(raw_tracking_strength_nt, sample_dt_s)
                noise_proxy_nt = abs(instantaneous_strength_nt - filtered_strength_nt)
                noise_floor_nt = max(self.noise_floor_filter.update(noise_proxy_nt, sample_dt_s), self.scenario.sensor.noise_std_nt)

            snr = tracking_strength_nt / max(noise_floor_nt, 1e-6)
            snr_db = 20.0 * np.log10(max(snr, 1e-6))
            signal_reliable = snr_db >= 6.0
            is_ac_detected = self.scenario.signal.mode != "dc"
            dominant_frequency_hz = self.scenario.signal.frequency_hz if is_ac_detected else 0.0

        weak_signal_threshold_nt = self.scenario.sensor.weak_signal_threshold_nt
        if signal_features is not None:
            weak_signal_flag = signal_features.weak_signal_flag
        else:
            weak_signal_flag = tracking_strength_nt < weak_signal_threshold_nt or not signal_reliable

        # --- Signal enhancement layer updates ---
        # 1. Envelope gradient tracking
        self.envelope_tracker.update(
            strength_nt=tracking_strength_nt,
            time_s=reading.time_s,
            position_xy_m=vehicle_position_xy_m,
            speed_mps=pose_measurement.speed_mps if hasattr(pose_measurement, 'speed_mps') else self.scenario.vehicle.cruise_speed_mps,
        )

        # 2. Magnetic vector analysis
        if self.vector_analyzer is not None and is_ac_detected:
            self.vector_analyzer.update(
                anomaly_ned_nt,
                tracking_strength_nt,
                pose_measurement=pose_measurement,
                snr_db=snr_db,
                signal_mode=self.scenario.signal.mode,
            )

        # 3. Displacement tracking for safe-lock criterion A
        if self.last_peak_position_xy_m is not None:
            delta_since_peak = np.asarray(vehicle_position_xy_m, dtype=float) - self.last_peak_position_xy_m
            self.displacement_since_last_peak_m = float(np.linalg.norm(delta_since_peak))
        else:
            self.displacement_since_last_peak_m = 0.0

        peak_event = self.peak_detector.update(
            tracking_strength_nt, reading.time_s,
            position_xy_m=vehicle_position_xy_m,
            vehicle_heading_deg=pose_measurement.heading_deg,
            use_nominal_route_prior=self.scenario.tracking.use_nominal_route_prior,
        )
        detected_peak_xy_m = None if peak_event.peak_position_xy_m is None else peak_event.peak_position_xy_m.copy()
        detection_age_s = reading.time_s - self.fitter.last_detection_time_s
        recent_detection_window_s = max(self.scenario.tracking.peak_cooldown_s * 2.0, 1.0)
        if peak_event.detected:
            peak_position_xy_m = peak_event.peak_position_xy_m if peak_event.peak_position_xy_m is not None else np.asarray(vehicle_position_xy_m, dtype=float)

            # --- Posterior position compensation ---
            if self.scenario.tracking.parabolic_interpolation_enabled and self.scenario.tracking.peak_position_delay_s > 0:
                speed_mps = pose_measurement.speed_mps if hasattr(pose_measurement, 'speed_mps') else self.scenario.vehicle.cruise_speed_mps
                heading_rad = np.deg2rad(pose_measurement.heading_deg)
                velocity_xy = np.array([np.cos(heading_rad), np.sin(heading_rad)], dtype=float) * speed_mps
                compensation_xy = velocity_xy * self.scenario.tracking.peak_position_delay_s
                peak_position_xy_m = peak_position_xy_m - compensation_xy

            peak_position_xy_m = self._peak_cable_observation_xy_m(peak_position_xy_m, sonar_reading)
            if peak_position_xy_m is not None and not self._is_peak_outlier(peak_position_xy_m):
                self.fitter.add_peak(
                    peak_position_xy_m,
                    snr_linear=max(snr, self.scenario.tracking.weighted_fitter_snr_floor),
                    confidence=max(self.last_output_confidence, 0.05),
                    time_s=reading.time_s,
                )
                detection_age_s = reading.time_s - self.fitter.last_detection_time_s
                detected_peak_xy_m = peak_position_xy_m.copy()
            else:
                detected_peak_xy_m = None
            if self.last_confirmed_peak_strength_nt > 0.0 and peak_event.peak_strength_nt < self.last_confirmed_peak_strength_nt - self.scenario.tracking.safe_lock_peak_drop_nt:
                self.safe_lock_until_s = reading.time_s + 1.0
            self.last_confirmed_peak_strength_nt = peak_event.peak_strength_nt
            # Track last valid peak for safe-lock criterion A
            self.last_valid_peak_strength_nt = peak_event.peak_strength_nt
            self.last_peak_position_xy_m = detected_peak_xy_m.copy() if detected_peak_xy_m is not None else None
            self.displacement_since_last_peak_m = 0.0
        elif (
            self.last_confirmed_peak_strength_nt > 0.0
            and self.peak_detector.current_peak_strength_nt > 0.0
            and detection_age_s <= recent_detection_window_s
        ):
            if self.peak_detector.current_peak_strength_nt < self.last_confirmed_peak_strength_nt - self.scenario.tracking.safe_lock_peak_drop_nt:
                self.safe_lock_until_s = reading.time_s + 0.5

        # --- Sonar positioning feed (sonar is the dedicated positioning sensor) ---
        # The buried-cable field is too gentle for the magnetic peak detector to
        # fire during steady-state zig-zag, so a magnetic-peak-only fitter never
        # converges.  Sonar delivers direct cable-position fixes; routing them into
        # the same SNR-weighted fitter lets the centreline — and its PCA covariance,
        # the ``LOCK_ALIGN -> TRACK_ACTIVE`` gate — converge.  The fitter's spatial
        # exclusion keeps the fixes spread along-track instead of clustering.
        if sonar_reading is not None and sonar_reading.valid and sonar_reading.estimated_position_ned_m is not None:
            self.fitter.add_peak(
                np.asarray(sonar_reading.estimated_position_ned_m, dtype=float)[:2],
                snr_linear=max(sonar_reading.confidence * 20.0, self.scenario.tracking.weighted_fitter_snr_floor),
                confidence=max(sonar_reading.confidence, 0.05),
                time_s=reading.time_s,
            )
            self.local_path_estimator.add_observation(
                np.asarray(sonar_reading.estimated_position_ned_m, dtype=float)[:2],
                time_s=reading.time_s,
                confidence=max(sonar_reading.confidence, 0.05),
                heading_deg=sonar_reading.estimated_heading_ned_deg,
            )
            self.local_path_tracking_estimator.add_observation(
                np.asarray(sonar_reading.estimated_position_ned_m, dtype=float)[:2],
                time_s=reading.time_s,
                confidence=max(sonar_reading.confidence, 0.05),
                heading_deg=sonar_reading.estimated_heading_ned_deg,
            )
            detection_age_s = reading.time_s - self.fitter.last_detection_time_s
        elif detected_peak_xy_m is not None:
            self.local_path_estimator.add_observation(
                detected_peak_xy_m,
                time_s=reading.time_s,
                confidence=max(self.last_output_confidence, 0.05),
            )
            self.local_path_tracking_estimator.add_observation(
                detected_peak_xy_m,
                time_s=reading.time_s,
                confidence=max(self.last_output_confidence, 0.05),
            )

        # --- Magnetic cross-track steering signal (peak-free) ---
        # The ratio ``B_down / B_perp == y / d`` yields a signed cross-track offset
        # (the line current cancels), no peak detection needed.  This is a STEERING
        # signal only: it is surfaced in the state for the controller's TRACK_ACTIVE
        # centreline hold, and is deliberately NOT fed into the line fitter — its
        # ~1 m precision would inflate the fit covariance that gates LOCK -> TRACK.
        # Sonar remains the sole positioning sensor for the centreline itself.
        magnetic_cross_track_offset_m = None
        if self.scenario.tracking.mag_cross_track_enabled:
            magnetic_cross_track_offset_m = self._magnetic_cross_track_offset(
                anomaly_ned_nt=anomaly_ned_nt,
                burial_measurement=burial_measurement,
            )

        magnetic_path_observation = None
        magnetic_phase_observation = None
        magnetic_lookahead_target = None
        magnetic_shadow_hypothesis_selection = None
        shadow_axis_validation_diag = self._shadow_axis_validation_diagnostics(None, reading.time_s)
        magnetic_lookahead_feed_diag = self._magnetic_lookahead_feed_diagnostics(
            None,
            reading.time_s,
            None,
        )
        if (
            self.magnetic_path_builder is not None
            and (sonar_reading is None or not sonar_reading.valid)
            and signal_reliable
            and self.last_accepted_fit_result.direction_xy is not None
            and self.last_output_confidence >= self.scenario.tracking.low_confidence_threshold
        ):
            magnetic_path_observation = self.magnetic_path_builder.build(
                vehicle_position_xy_m=np.asarray(vehicle_position_xy_m, dtype=float),
                anomaly_ned_nt=anomaly_ned_nt,
                movement_heading_deg=(
                    heading_from_direction_xy(self.last_accepted_fit_result.direction_xy)
                    if self.last_accepted_fit_result.direction_xy is not None
                    else pose_measurement.heading_deg
                ),
            )
            if magnetic_path_observation is not None and self.magnetic_phase_detector is not None:
                magnetic_phase_observation = self.magnetic_phase_detector.update(
                    magnetic_path_observation,
                    reading.time_s,
                )
                if magnetic_phase_observation is not None:
                    self.last_magnetic_phase_time_s = reading.time_s
            phase_latched = (
                self.scenario.tracking.magnetic_path_phase_gate_enabled
                and reading.time_s - self.last_magnetic_phase_time_s
                <= self.scenario.tracking.magnetic_path_phase_latch_duration_s
            )
            feed_observation = (
                magnetic_phase_observation.observation
                if magnetic_phase_observation is not None
                else magnetic_path_observation
            )
            if (
                feed_observation is not None
                and self.scenario.tracking.magnetic_path_feed_local_path
                and (
                    not self.scenario.tracking.magnetic_path_phase_gate_enabled
                    or magnetic_phase_observation is not None
                    or phase_latched
                )
                and self._magnetic_path_feed_allowed(
                    vehicle_position_xy_m,
                    feed_observation.position_xy_m,
                    feed_observation.heading_deg,
                )
            ):
                self.local_path_estimator.add_observation(
                    feed_observation.position_xy_m,
                    time_s=reading.time_s,
                    confidence=feed_observation.confidence,
                    heading_deg=feed_observation.heading_deg,
                )
                self.local_path_tracking_estimator.add_observation(
                    feed_observation.position_xy_m,
                    time_s=reading.time_s,
                    confidence=feed_observation.confidence,
                    heading_deg=feed_observation.heading_deg,
                )
        if self.magnetic_lookahead_builder is not None:
            magnetic_lookahead_target = self.magnetic_lookahead_builder.update(
                vehicle_position_xy_m=np.asarray(vehicle_position_xy_m, dtype=float),
                time_s=reading.time_s,
                phase_observation=magnetic_phase_observation,
            )
            magnetic_lookahead_feed_diag = self._magnetic_lookahead_feed_diagnostics(
                magnetic_lookahead_target,
                reading.time_s,
                self.local_path_estimator.estimate(),
            )
            phase_anchor_fed = False
            if (
                magnetic_phase_observation is not None
                and self.scenario.tracking.magnetic_lookahead_feed_local_path
                and self.scenario.tracking.magnetic_lookahead_feed_phase_anchor_enabled
            ):
                anchor_observation = magnetic_phase_observation.observation
                anchor_confidence = max(
                    anchor_observation.confidence,
                    self.scenario.tracking.magnetic_lookahead_feed_phase_anchor_confidence,
                )
                self.local_path_estimator.add_observation(
                    anchor_observation.position_xy_m,
                    time_s=reading.time_s,
                    confidence=anchor_confidence,
                    heading_deg=anchor_observation.heading_deg,
                )
                self.local_path_tracking_estimator.add_observation(
                    anchor_observation.position_xy_m,
                    time_s=reading.time_s,
                    confidence=anchor_confidence,
                    heading_deg=anchor_observation.heading_deg,
                )
                phase_anchor_fed = True
            if (
                magnetic_lookahead_target is not None
                and not phase_anchor_fed
                and magnetic_lookahead_feed_diag["allowed"] > 0.5
            ):
                extrapolated_confidence = magnetic_lookahead_target.confidence * (
                    self.scenario.tracking.magnetic_lookahead_feed_extrapolated_confidence_scale
                )
                feed_heading_deg = self._smooth_magnetic_lookahead_feed_heading(
                    magnetic_lookahead_target.heading_deg
                )
                self.local_path_estimator.add_observation(
                    magnetic_lookahead_target.cable_point_xy_m,
                    time_s=reading.time_s,
                    confidence=extrapolated_confidence,
                    heading_deg=feed_heading_deg,
                )
                self.local_path_tracking_estimator.add_observation(
                    magnetic_lookahead_target.cable_point_xy_m,
                    time_s=reading.time_s,
                    confidence=extrapolated_confidence,
                    heading_deg=feed_heading_deg,
                )
        if self.magnetic_shadow_hypothesis_selector is not None:
            magnetic_shadow_hypothesis_selection = self.magnetic_shadow_hypothesis_selector.update(
                vehicle_position_xy_m=np.asarray(vehicle_position_xy_m, dtype=float),
                vehicle_heading_deg=pose_measurement.heading_deg,
                time_s=reading.time_s,
                phase_observation=magnetic_phase_observation,
            )
            shadow_axis_validation_diag = self._shadow_axis_validation_diagnostics(
                magnetic_shadow_hypothesis_selection,
                reading.time_s,
            )

        if detection_age_s > self.scenario.tracking.lost_timeout_s:
            self.safe_lock_until_s = -1e9
            self.last_confirmed_peak_strength_nt = 0.0

        local_path_state = self.local_path_estimator.estimate()
        local_path_tracking_state_estimate = self.local_path_tracking_estimator.estimate()
        fit_result_candidate = self.fitter.fit()
        fit_update_rejected = False
        bootstrap_fit_ready = self._bootstrap_fit_ready(fit_result_candidate)
        if fit_result_candidate.direction_xy is not None:
            candidate_heading_deg = heading_from_direction_xy(fit_result_candidate.direction_xy)
            if (
                not self.scenario.tracking.use_nominal_route_prior
                and (
                    not bootstrap_fit_ready
                    or not np.isfinite(fit_result_candidate.residual_m)
                    or fit_result_candidate.residual_m > self.scenario.tracking.fit_acceptance_residual_m
                )
            ):
                fit_result = self.last_accepted_fit_result if self.last_accepted_fit_result.direction_xy is not None else FitResult(origin_xy_m=None, direction_xy=None, residual_m=float("inf"), covariance_xy_m2=None)
                fit_update_rejected = True
            elif (
                self.scenario.tracking.use_nominal_route_prior
                and candidate_heading_deg is not None
                and self._reference_heading_deg() is not None
                and abs(smallest_angle_error_deg(candidate_heading_deg, self._reference_heading_deg())) > self.scenario.tracking.fit_reject_heading_delta_deg
                and self.last_output_confidence < self.scenario.tracking.fit_reject_confidence_threshold
                and self.last_accepted_fit_result.direction_xy is not None
            ):
                fit_result = self.last_accepted_fit_result
                fit_update_rejected = True
            else:
                fit_result = fit_result_candidate
                if bootstrap_fit_ready or not self.scenario.tracking.use_nominal_route_prior:
                    self.last_accepted_fit_result = fit_result_candidate
        elif self.last_accepted_fit_result.direction_xy is not None and detection_age_s <= self.scenario.tracking.lost_timeout_s:
            fit_result = self.last_accepted_fit_result
        else:
            fit_result = fit_result_candidate

        line_heading_deg = heading_from_direction_xy(fit_result.direction_xy) if fit_result.direction_xy is not None else None

        # Retrieve previous zigzag width, assuming it was stored or calculated.
        # Since it's not stored in state directly, let's use max_zigzag_width_m as a safe upper bound if not available,
        # but wait, self.last_output_confidence was used above. I should add self.last_zigzag_width_m.
        # Actually I can just use self.scenario.tracking.max_zigzag_width_m or the dynamic value.
        current_zigzag_width = getattr(self, "last_zigzag_width_m", self.scenario.tracking.max_zigzag_width_m)
        magnetic_confidence = self.confidence_estimator.magnetic_confidence(
            snr,
            fit_result.residual_m,
            detection_age_s,
            weak_signal_flag,
            zigzag_width_m=current_zigzag_width,
            speed_mps=pose_measurement.speed_mps if hasattr(pose_measurement, 'speed_mps') else self.scenario.vehicle.cruise_speed_mps,
        )

        sonar_status = "OFFLINE"
        sonar_confidence = 0.0
        sonar_heading_deg = None
        sonar_relative_position_body_m = None
        estimated_cable_point_xy_m = self._local_line_point(vehicle_position_xy_m, fit_result)
        if sonar_reading is not None:
            sonar_status = sonar_reading.status
            sonar_confidence = sonar_reading.confidence if sonar_reading.valid else 0.0
            sonar_heading_deg = sonar_reading.estimated_heading_ned_deg
            if sonar_reading.relative_position_body_m is not None:
                sonar_relative_position_body_m = sonar_reading.relative_position_body_m.copy()

        guidance_source = "SEARCH"
        fused_heading_deg = None
        local_path_age_s = (
            reading.time_s - local_path_state.latest_time_s
            if local_path_state is not None
            else float("inf")
        )
        local_path_tracking_state = (
            local_path_tracking_state_estimate.tracking_state
            if local_path_tracking_state_estimate is not None
            else self.local_path_tracking_estimator.tracking_state
        )
        magnetic_lookahead_ready = (
            magnetic_lookahead_target is not None
            and magnetic_lookahead_target.confidence >= self.scenario.tracking.magnetic_lookahead_min_confidence
        )
        local_path_stale_for_reacquire = (
            self.scenario.tracking.local_path_guidance_enabled
            and not self.scenario.tracking.use_nominal_route_prior
            and local_path_state is not None
            and not magnetic_lookahead_ready
            and local_path_age_s > self.scenario.tracking.reacquire_stale_timeout_s
            and reading.time_s >= self.scenario.tracking.reacquire_min_elapsed_s
        )
        if local_path_stale_for_reacquire:
            local_path_tracking_state = LocalPathTrackingState.REACQUIRE
        local_path_max_residual_m = self.scenario.tracking.local_path_max_residual_m
        if local_path_tracking_state == LocalPathTrackingState.CURVE_TRACK:
            local_path_max_residual_m *= self.scenario.tracking.local_path_curve_residual_relax
        local_path_guidance_ready = (
            self.scenario.tracking.local_path_guidance_enabled
            and not self.scenario.tracking.use_nominal_route_prior
            and local_path_state is not None
            and local_path_state.heading_deg is not None
            and local_path_state.confidence >= self.scenario.tracking.local_path_min_confidence
            and np.isfinite(local_path_state.residual_m)
            and local_path_state.residual_m <= local_path_max_residual_m
            and local_path_age_s <= self.scenario.tracking.local_path_max_age_s
            and not local_path_stale_for_reacquire
        )
        deployment_reacquire_required = (
            local_path_tracking_state == LocalPathTrackingState.REACQUIRE
            or local_path_stale_for_reacquire
        )
        if (
            self.scenario.tracking.reacquire_region_enabled
            and local_path_state is not None
            and not deployment_reacquire_required
            and local_path_state.confidence >= self.scenario.tracking.local_path_min_confidence
        ):
            self.reacquire_region_selector.update_trusted_state(
                anchor_xy_m=local_path_state.anchor_xy_m,
                heading_deg=local_path_state.heading_deg,
                confidence=local_path_state.confidence,
                time_s=local_path_state.latest_time_s,
                curvature_1pm=local_path_state.curvature_1pm,
            )
        reacquire_region = None
        if self.scenario.tracking.reacquire_region_enabled:
            reacquire_region = self.reacquire_region_selector.select(
                time_s=reading.time_s,
                vehicle_position_xy_m=vehicle_position_xy_m,
                reacquire_required=deployment_reacquire_required,
            )

        # --- Task 1: Sonar Seed Injection (Highest Priority) ---
        # When sonar is online and valid, and magnetic fit is not yet mature,
        # force use sonar heading as the primary navigation command.
        # This breaks the SPIRAL_SEARCH deadlock in deployment mode.
        _sonar_seed_confidence = None
        if (
            sonar_reading is not None
            and sonar_reading.valid
            and sonar_reading.estimated_heading_ned_deg is not None
            and (not bootstrap_fit_ready or detection_age_s > 10.0)
        ):
            fused_heading_deg = sonar_reading.estimated_heading_ned_deg
            if sonar_reading.estimated_position_ned_m is not None:
                estimated_cable_point_xy_m = sonar_reading.estimated_position_ned_m.copy()
            guidance_source = "SONAR_SEED"
            _sonar_seed_confidence = sonar_confidence * 0.8
        elif sonar_reading is not None and sonar_reading.valid and ((sonar_reading.distance_m is not None and sonar_reading.distance_m >= self.scenario.tracking.sonar_preferred_distance_m) or weak_signal_flag):
            fused_heading_deg = sonar_reading.estimated_heading_ned_deg
            if sonar_reading.estimated_position_ned_m is not None:
                estimated_cable_point_xy_m = sonar_reading.estimated_position_ned_m.copy()
            guidance_source = "SONAR"
        elif local_path_guidance_ready:
            fused_heading_deg = local_path_state.heading_deg
            estimated_cable_point_xy_m = local_path_state.anchor_xy_m.copy()
            guidance_source = "LOCAL_PATH"
        elif (
            magnetic_lookahead_ready
            and magnetic_lookahead_target is not None
        ):
            fused_heading_deg = magnetic_lookahead_target.heading_deg
            estimated_cable_point_xy_m = magnetic_lookahead_target.cable_point_xy_m.copy()
            guidance_source = "MAGNETIC_LOOKAHEAD"
        elif line_heading_deg is not None and not weak_signal_flag and bootstrap_fit_ready:
            fused_heading_deg = line_heading_deg
            guidance_source = "MAGNETIC"
        elif (
            line_heading_deg is not None
            and bootstrap_fit_ready
            and detection_age_s <= self.scenario.tracking.guidance_memory_timeout_s
            and np.isfinite(fit_result.residual_m)
            and fit_result.residual_m <= max(self.scenario.tracking.fit_acceptance_residual_m, 15.0)
        ):
            fused_heading_deg = line_heading_deg
            guidance_source = "MEMORY"

        # In deployment (no-prior) mode, prefer raw peak position over line
        # projection to avoid feeding stale fit geometry into blind_heading.
        point_to_record = estimated_cable_point_xy_m
        if (
            not self.scenario.tracking.use_nominal_route_prior
            and detected_peak_xy_m is not None
        ):
            point_to_record = detected_peak_xy_m
        if point_to_record is not None:
            self.valid_points_xy.append(point_to_record.copy())

        blind_heading_deg = None
        if fused_heading_deg is None:
            blind_heading_deg = self._blind_heading()
            if blind_heading_deg is None and line_heading_deg is not None and (bootstrap_fit_ready or not self.scenario.tracking.use_nominal_route_prior) and np.isfinite(fit_result.residual_m):
                fit_acceptance_residual_m = max(self.scenario.tracking.fit_acceptance_residual_m, 15.0)
                if fit_result.residual_m <= fit_acceptance_residual_m:
                    blind_heading_deg = line_heading_deg
            if fused_heading_deg is None:
                fused_heading_deg = blind_heading_deg
            if blind_heading_deg is not None and guidance_source != "MEMORY":
                guidance_source = "BLIND"

        if not self.scenario.tracking.use_nominal_route_prior:
            # Deployment mode: the sonar-fed line fitter (Part A) is the single,
            # unambiguous heading source, so the deployment estimate just mirrors
            # the fitted centreline heading for the deployment visualiser/harness.
            # All the old ±90° crossing-disambiguation rules are gone.
            if line_heading_deg is not None:
                self.deployment_estimated_cable_heading_deg = line_heading_deg
                fit_confidence = float(np.clip(np.exp(-fit_result.residual_m / 6.0), 0.0, 1.0)) if np.isfinite(fit_result.residual_m) else 0.0
                self.deployment_heading_confidence = max(self.deployment_heading_confidence, fit_confidence)

        # Safe-Lock Criterion A/B are disabled (kept as inert diagnostics until the
        # mission-FSM refactor removes them from the perception contract).
        self.safe_lock_criterion_a_active = False
        self.safe_lock_criterion_b_active = False

        confidence_fit_residual_m = local_path_state.residual_m if guidance_source == "LOCAL_PATH" and local_path_state is not None else fit_result.residual_m
        confidence = self.confidence_estimator.fused_confidence(
            magnetic_confidence,
            sonar_confidence,
            guidance_source,
            confidence_fit_residual_m,
            fit_result.covariance_xy_m2,
        )
        if guidance_source == "SEARCH":
            confidence = min(confidence, 0.25)
        elif guidance_source == "MEMORY":
            confidence = max(confidence, self.scenario.tracking.memory_guidance_confidence_floor)
        elif guidance_source == "SONAR_SEED" and _sonar_seed_confidence is not None:
            # Cap SONAR_SEED confidence at 0.6 to prevent premature HOLD entry
            # before magnetic peaks are captured
            confidence = max(min(confidence, 0.6), max(_sonar_seed_confidence, 0.4))
        elif guidance_source == "LOCAL_PATH" and local_path_state is not None:
            confidence = max(confidence, min(0.82, local_path_state.confidence))
        elif guidance_source == "MAGNETIC_LOOKAHEAD" and magnetic_lookahead_target is not None:
            confidence = max(confidence, min(0.72, magnetic_lookahead_target.confidence))
        self.last_output_confidence = confidence

        # --- Task 3: Inverse Confidence Zigzag Width Mapping ---
        # High confidence -> tight tracking (min_width)
        # Low confidence -> wide sweeping search (max_width)
        max_z_width = self.scenario.tracking.max_zigzag_width_m
        min_z_width = self.scenario.tracking.min_zigzag_width_m
        
        # Override: During Bootstrap phase, force maximum width to ensure cable crossing
        magnetic_fit_ready = fit_result.origin_xy_m is not None and (len(self.valid_points_xy) >= 3 if self.valid_points_xy else False)
        if not magnetic_fit_ready:
            zigzag_width_m = max_z_width
        else:
            zigzag_width_m = float(np.clip(
                max_z_width - (max_z_width - min_z_width) * confidence,
                min_z_width,
                max_z_width
            ))

        # Task 2: Residual-driven width reduction
        valid_fit = (len(self.fitter.peak_observations) >= 3) and (fit_result.residual_m <= 3.0)
        if valid_fit:
            zigzag_width_m = min(zigzag_width_m, min_z_width * 1.5)

        self.last_zigzag_width_m = zigzag_width_m

        # Original safe-lock logic (peak drop detection)
        safe_lock_active = (
            detection_age_s <= recent_detection_window_s
            and reading.time_s <= self.safe_lock_until_s
        ) or self.safe_lock_criterion_a_active
        if safe_lock_active:
            zigzag_width_m = min_z_width

        estimated_path_points_xy_m = np.vstack(self.valid_points_xy) if self.valid_points_xy else np.empty((0, 2), dtype=float)
        estimated_path_covariance_xy_m2 = fit_result.covariance_xy_m2
        if estimated_path_covariance_xy_m2 is None and estimated_path_points_xy_m.shape[0] >= 2:
            estimated_path_covariance_xy_m2 = np.cov(estimated_path_points_xy_m.T)

        # --- Magnetic burial-depth inversion (calibrated amplitude, peak-free) ---
        # The GT BurialDepthObserver is kept only as the evaluation truth channel;
        # the published estimate now comes from the inverter (or None when it is
        # disabled / not yet warmed up / lateral unknown).
        estimated_burial_depth_m: Optional[float] = None
        burial_inversion_uncertainty_m: Optional[float] = None
        if self.burial_inverter is not None:
            lateral_offset_m = self._burial_lateral_offset_m(vehicle_position_xy_m, sonar_reading, fit_result)
            if lateral_offset_m is not None:
                burial_estimate = self.burial_inverter.update(tracking_strength_nt, lateral_offset_m, snr_db)
                if burial_estimate is not None:
                    estimated_burial_depth_m = burial_estimate.depth_m
                    burial_inversion_uncertainty_m = burial_estimate.sigma_m

        local_path_model_codes = {"line": 1.0, "local_line": 2.0, "arc": 3.0}
        local_path_model_code = 0.0
        local_path_heading_deg = None
        local_path_confidence = 0.0
        local_path_residual_m = float("inf")
        local_path_radius_m = float("inf")
        local_path_tracking_state_value = local_path_tracking_state.value
        if local_path_state is not None:
            local_path_model_code = local_path_model_codes.get(local_path_state.model, 0.0)
            local_path_heading_deg = local_path_state.heading_deg
            local_path_confidence = local_path_state.confidence
            local_path_residual_m = local_path_state.residual_m
            local_path_radius_m = local_path_state.radius_m

        return PerceptionState(
            time_s=reading.time_s,
            sensor_field_nt=reading.sensor_field_nt,
            body_field_nt=body_field_nt,
            ned_field_nt=ned_field_nt,
            anomaly_ned_nt=anomaly_ned_nt,
            ac_component_ned_nt=ac_component_ned_nt,
            filtered_strength_nt=filtered_strength_nt,
            rms_strength_nt=rms_strength_nt,
            tracking_strength_nt=tracking_strength_nt,
            noise_floor_nt=noise_floor_nt,
            snr=snr,
            snr_db=snr_db,
            magnetic_confidence=magnetic_confidence,
            sonar_confidence=sonar_confidence,
            confidence=confidence,
            weak_signal_flag=weak_signal_flag,
            signal_reliable=signal_reliable,
            is_ac_detected=is_ac_detected,
            dominant_frequency_hz=dominant_frequency_hz,
            peak_detected=peak_event.detected,
            fit_result=fit_result,
            line_heading_deg=line_heading_deg,
            fused_heading_deg=fused_heading_deg,
            blind_heading_deg=blind_heading_deg,
            guidance_source=guidance_source,
            safe_lock_active=safe_lock_active,
            zigzag_width_m=zigzag_width_m,
            sonar_status=sonar_status,
            sonar_relative_position_body_m=sonar_relative_position_body_m,
            sonar_heading_deg=sonar_heading_deg,
            estimated_cable_point_xy_m=estimated_cable_point_xy_m,
            estimated_path_points_xy_m=estimated_path_points_xy_m,
            estimated_path_covariance_xy_m2=estimated_path_covariance_xy_m2,
            fit_update_rejected=fit_update_rejected,
            estimated_burial_depth_m=estimated_burial_depth_m,
            true_burial_depth_m=true_burial_depth_m,
            burial_measurement_valid=burial_measurement.valid,
            last_detection_age_s=detection_age_s,
            detected_peak_xy_m=detected_peak_xy_m,
            deployment_estimated_cable_heading_deg=self.deployment_estimated_cable_heading_deg,
            deployment_heading_confidence=self.deployment_heading_confidence,
            deployment_reacquire_required=deployment_reacquire_required,
            # Signal enhancement outputs
            envelope_gradient_nT_per_m=self.envelope_tracker.gradient_nT_per_m,
            envelope_gradient_heading_deg=self.envelope_tracker.gradient_heading_deg,
            magnetic_vector_heading_deg=(
                self.vector_analyzer.magnetic_vector_heading_deg
                if self.vector_analyzer is not None else None
            ),
            vector_cable_heading_deg=(
                self.vector_analyzer.vector_cable_heading_deg
                if self.vector_analyzer is not None else None
            ),
            vector_consistency_score=(
                self.vector_analyzer.vector_consistency_score
                if self.vector_analyzer is not None else 0.0
            ),
            attitude_leakage_risk=(
                self.vector_analyzer.attitude_leakage_risk
                if self.vector_analyzer is not None else False
            ),
            # Safe-lock diagnostics
            safe_lock_criterion_a_active=self.safe_lock_criterion_a_active,
            safe_lock_criterion_b_active=self.safe_lock_criterion_b_active,
            safe_lock_fit_invalidated=self.safe_lock_fit_invalidated,
            last_valid_peak_strength_nt=self.last_valid_peak_strength_nt,
            displacement_since_last_peak_m=self.displacement_since_last_peak_m,
            magnetic_cross_track_offset_m=magnetic_cross_track_offset_m,
            magnetic_cross_track_quality=float(self.cross_track_estimator.quality),
            magnetic_path_observation_valid=magnetic_path_observation is not None,
            magnetic_path_x_m=None if magnetic_path_observation is None else float(magnetic_path_observation.position_xy_m[0]),
            magnetic_path_y_m=None if magnetic_path_observation is None else float(magnetic_path_observation.position_xy_m[1]),
            magnetic_path_heading_deg=None if magnetic_path_observation is None else magnetic_path_observation.heading_deg,
            magnetic_path_cross_track_offset_m=(
                None if magnetic_path_observation is None else magnetic_path_observation.cross_track_offset_m
            ),
            magnetic_path_confidence=0.0 if magnetic_path_observation is None else magnetic_path_observation.confidence,
            magnetic_phase_observation_valid=magnetic_phase_observation is not None,
            magnetic_phase_x_m=(
                None if magnetic_phase_observation is None else float(magnetic_phase_observation.observation.position_xy_m[0])
            ),
            magnetic_phase_y_m=(
                None if magnetic_phase_observation is None else float(magnetic_phase_observation.observation.position_xy_m[1])
            ),
            magnetic_phase_heading_deg=(
                None if magnetic_phase_observation is None else magnetic_phase_observation.observation.heading_deg
            ),
            magnetic_phase_amplitude_m=0.0 if magnetic_phase_observation is None else magnetic_phase_observation.amplitude_m,
            magnetic_phase_duration_s=0.0 if magnetic_phase_observation is None else magnetic_phase_observation.duration_s,
            magnetic_phase_confidence=(
                0.0 if magnetic_phase_observation is None else magnetic_phase_observation.observation.confidence
            ),
            magnetic_lookahead_valid=magnetic_lookahead_target is not None,
            magnetic_lookahead_cable_point_xy_m=(
                None if magnetic_lookahead_target is None else magnetic_lookahead_target.cable_point_xy_m.copy()
            ),
            magnetic_lookahead_target_xy_m=(
                None if magnetic_lookahead_target is None else magnetic_lookahead_target.lookahead_xy_m.copy()
            ),
            magnetic_lookahead_heading_deg=(
                None if magnetic_lookahead_target is None else magnetic_lookahead_target.heading_deg
            ),
            magnetic_lookahead_confidence=(
                0.0 if magnetic_lookahead_target is None else magnetic_lookahead_target.confidence
            ),
            magnetic_lookahead_age_s=float("inf") if magnetic_lookahead_target is None else magnetic_lookahead_target.age_s,
            magnetic_lookahead_feed_allowed=magnetic_lookahead_feed_diag["allowed"] > 0.5,
            magnetic_lookahead_feed_reason_code=magnetic_lookahead_feed_diag["reason_code"],
            magnetic_lookahead_feed_phase_age_s=magnetic_lookahead_feed_diag["phase_age_s"],
            magnetic_lookahead_feed_innovation_m=magnetic_lookahead_feed_diag["innovation_m"],
            magnetic_lookahead_feed_axis_delta_deg=magnetic_lookahead_feed_diag["axis_delta_deg"],
            magnetic_lookahead_feed_local_residual_m=magnetic_lookahead_feed_diag["local_residual_m"],
            shadow_axis_hypothesis_valid=magnetic_shadow_hypothesis_selection is not None,
            shadow_axis_hypothesis_count=(
                0 if magnetic_shadow_hypothesis_selection is None else magnetic_shadow_hypothesis_selection.candidate_count
            ),
            shadow_axis_selected_sign=(
                0.0 if magnetic_shadow_hypothesis_selection is None else magnetic_shadow_hypothesis_selection.selected_sign
            ),
            shadow_axis_selected_score=(
                0.0 if magnetic_shadow_hypothesis_selection is None else magnetic_shadow_hypothesis_selection.selected_score
            ),
            shadow_axis_score_margin=(
                0.0 if magnetic_shadow_hypothesis_selection is None else magnetic_shadow_hypothesis_selection.score_margin
            ),
            shadow_axis_target_xy_m=(
                None if magnetic_shadow_hypothesis_selection is None else magnetic_shadow_hypothesis_selection.target_xy_m.copy()
            ),
            shadow_axis_heading_deg=(
                None if magnetic_shadow_hypothesis_selection is None else magnetic_shadow_hypothesis_selection.heading_deg
            ),
            shadow_axis_age_s=(
                float("inf") if magnetic_shadow_hypothesis_selection is None else magnetic_shadow_hypothesis_selection.age_s
            ),
            shadow_axis_validation_passed=shadow_axis_validation_diag["passed"] > 0.5,
            shadow_axis_validation_reason_code=shadow_axis_validation_diag["reason_code"],
            shadow_axis_validation_score_deficit=shadow_axis_validation_diag["score_deficit"],
            shadow_axis_validation_margin_deficit=shadow_axis_validation_diag["margin_deficit"],
            shadow_axis_validation_age_over_s=shadow_axis_validation_diag["age_over_s"],
            burial_inversion_uncertainty_m=burial_inversion_uncertainty_m,
            local_path_model_code=local_path_model_code,
            local_path_heading_deg=local_path_heading_deg,
            local_path_confidence=local_path_confidence,
            local_path_residual_m=local_path_residual_m,
            local_path_radius_m=local_path_radius_m,
            local_path_tracking_state=local_path_tracking_state_value,
            reacquire_region_center_xy_m=None if reacquire_region is None else reacquire_region.center_xy_m.copy(),
            reacquire_region_heading_deg=None if reacquire_region is None else reacquire_region.heading_deg,
            reacquire_region_half_length_m=0.0 if reacquire_region is None else reacquire_region.half_length_m,
            reacquire_region_half_width_m=0.0 if reacquire_region is None else reacquire_region.half_width_m,
            reacquire_region_confidence=0.0 if reacquire_region is None else reacquire_region.confidence,
            reacquire_region_score=0.0 if reacquire_region is None else reacquire_region.score,
            reacquire_region_reason="none" if reacquire_region is None else reacquire_region.reason,
        )

    def _magnetic_path_feed_allowed(
        self,
        vehicle_position_xy_m: np.ndarray,
        observation_xy_m: np.ndarray,
        observation_heading_deg: float,
    ) -> bool:
        """Gate pure-magnetic observations before they can affect local path."""
        if self.last_accepted_fit_result.direction_xy is None:
            return False
        reference_xy = self._local_line_point(vehicle_position_xy_m, self.last_accepted_fit_result)
        if reference_xy is None:
            return False
        innovation_m = float(np.linalg.norm(np.asarray(observation_xy_m, dtype=float) - reference_xy))
        if innovation_m > self.scenario.tracking.magnetic_path_feed_max_innovation_m:
            return False
        reference_heading_deg = heading_from_direction_xy(self.last_accepted_fit_result.direction_xy)
        if reference_heading_deg is None:
            return False
        axis_error_deg = abs(smallest_angle_error_deg(observation_heading_deg, reference_heading_deg))
        axis_error_deg = min(axis_error_deg, abs(180.0 - axis_error_deg))
        return axis_error_deg <= self.scenario.tracking.magnetic_path_feed_max_heading_delta_deg

    def _smooth_magnetic_lookahead_feed_heading(self, heading_deg: float) -> float:
        """Limit heading jumps before lookahead observations enter local path."""
        if not self.scenario.tracking.magnetic_lookahead_feed_heading_smoothing_enabled:
            self.last_magnetic_lookahead_feed_heading_deg = float(heading_deg)
            return float(heading_deg)
        if self.last_magnetic_lookahead_feed_heading_deg is None:
            self.last_magnetic_lookahead_feed_heading_deg = float(heading_deg)
            return float(heading_deg)
        delta_deg = smallest_angle_error_deg(heading_deg, self.last_magnetic_lookahead_feed_heading_deg)
        limited_delta_deg = float(np.clip(
            delta_deg,
            -self.scenario.tracking.magnetic_lookahead_feed_heading_max_step_deg,
            self.scenario.tracking.magnetic_lookahead_feed_heading_max_step_deg,
        ))
        smoothed_heading_deg = wrap_angle_deg(
            self.last_magnetic_lookahead_feed_heading_deg + limited_delta_deg
        )
        self.last_magnetic_lookahead_feed_heading_deg = smoothed_heading_deg
        return smoothed_heading_deg

    def _shadow_axis_validation_diagnostics(self, shadow_axis_selection, time_s: float = 0.0) -> dict:
        """Diagnose the D3 shadow selector gate without changing control state.

        Reason code:
        0 none/disabled, 1 passed, 2 no hypothesis, 3 insufficient candidates,
        4 low score, 5 low margin, 6 stale age, 7 selector anchor expired.
        """
        diagnostics = {
            "passed": 0.0,
            "reason_code": 0.0,
            "score_deficit": 0.0,
            "margin_deficit": 0.0,
            "age_over_s": 0.0,
        }
        if not self.scenario.tracking.magnetic_shadow_hypothesis_enabled:
            return diagnostics
        if shadow_axis_selection is None:
            phase_age_s = float(time_s) - self.last_magnetic_phase_time_s
            if (
                self.last_magnetic_phase_time_s > -1e8
                and phase_age_s > self.scenario.tracking.magnetic_lookahead_max_age_s
            ):
                diagnostics["reason_code"] = 7.0
                diagnostics["age_over_s"] = phase_age_s - self.scenario.tracking.magnetic_lookahead_max_age_s
                return diagnostics
            diagnostics["reason_code"] = 2.0
            return diagnostics
        if shadow_axis_selection.candidate_count < 2:
            diagnostics["reason_code"] = 3.0
            return diagnostics

        min_score = self.scenario.tracking.magnetic_shadow_validation_min_score
        score_deficit = max(0.0, min_score - shadow_axis_selection.selected_score)
        diagnostics["score_deficit"] = score_deficit
        if score_deficit > 0.0:
            diagnostics["reason_code"] = 4.0
            return diagnostics

        min_margin = self.scenario.tracking.magnetic_shadow_validation_min_margin
        margin_deficit = max(0.0, min_margin - shadow_axis_selection.score_margin)
        diagnostics["margin_deficit"] = margin_deficit
        if margin_deficit > 0.0:
            diagnostics["reason_code"] = 5.0
            return diagnostics

        max_age_s = self.scenario.tracking.magnetic_shadow_validation_max_age_s
        age_over_s = max(0.0, shadow_axis_selection.age_s - max_age_s)
        diagnostics["age_over_s"] = age_over_s
        if age_over_s > 0.0:
            diagnostics["reason_code"] = 6.0
            return diagnostics

        diagnostics["passed"] = 1.0
        diagnostics["reason_code"] = 1.0
        return diagnostics

    def _magnetic_lookahead_feed_diagnostics(
        self,
        magnetic_lookahead_target,
        time_s: float,
        local_path_state,
    ) -> dict:
        """Diagnose and gate lookahead-derived observations before local path feed."""
        diagnostics = {
            "allowed": 0.0,
            "reason_code": 0.0,  # 0 none, 1 allowed, 2 no target, 3 disabled, 4 low confidence
            "phase_age_s": float("inf"),
            "innovation_m": float("nan"),
            "axis_delta_deg": float("nan"),
            "local_residual_m": float("nan"),
        }
        if magnetic_lookahead_target is None:
            diagnostics["reason_code"] = 2.0
            return diagnostics
        if not self.scenario.tracking.magnetic_lookahead_feed_local_path:
            diagnostics["reason_code"] = 3.0
            return diagnostics
        if magnetic_lookahead_target.confidence < self.scenario.tracking.magnetic_lookahead_min_confidence:
            diagnostics["reason_code"] = 4.0
            return diagnostics
        if magnetic_lookahead_target.age_s > self.scenario.tracking.magnetic_lookahead_feed_max_age_s:
            diagnostics["reason_code"] = 5.0
            return diagnostics
        phase_age_s = float(time_s) - self.last_magnetic_phase_time_s
        diagnostics["phase_age_s"] = phase_age_s
        if phase_age_s > self.scenario.tracking.magnetic_lookahead_feed_max_phase_age_s:
            diagnostics["reason_code"] = 6.0
            return diagnostics

        if local_path_state is None:
            diagnostics["allowed"] = 1.0
            diagnostics["reason_code"] = 1.0
            return diagnostics

        diagnostics["local_residual_m"] = float(local_path_state.residual_m)

        if (
            np.isfinite(local_path_state.residual_m)
            and local_path_state.residual_m > self.scenario.tracking.magnetic_lookahead_feed_max_local_residual_m
        ):
            diagnostics["reason_code"] = 7.0
            return diagnostics

        reference_heading_deg = local_path_state.heading_deg
        axis_error_deg = abs(smallest_angle_error_deg(magnetic_lookahead_target.heading_deg, reference_heading_deg))
        axis_error_deg = min(axis_error_deg, abs(180.0 - axis_error_deg))
        diagnostics["axis_delta_deg"] = axis_error_deg
        if axis_error_deg > self.scenario.tracking.magnetic_lookahead_feed_max_heading_delta_deg:
            diagnostics["reason_code"] = 8.0
            return diagnostics

        projected_xy = project_point_to_line(
            magnetic_lookahead_target.cable_point_xy_m,
            local_path_state.anchor_xy_m,
            local_path_state.tangent_xy,
        )
        innovation_m = float(np.linalg.norm(magnetic_lookahead_target.cable_point_xy_m - projected_xy))
        diagnostics["innovation_m"] = innovation_m
        if innovation_m > self.scenario.tracking.magnetic_lookahead_feed_max_innovation_m:
            diagnostics["reason_code"] = 9.0
            return diagnostics
        diagnostics["allowed"] = 1.0
        diagnostics["reason_code"] = 1.0
        return diagnostics
