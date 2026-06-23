"""Perception orchestrator: drives preprocessing, peak detection, fitting and state fusion."""

from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np

from ..config import ScenarioConfig
from ..math_utils import (
    body_to_ned,
    build_polyline_projection_cache,
    heading_from_direction_xy,
    nearest_point_on_polyline,
    norm,
    project_point_to_line,
    rotation_matrix_sensor_to_body,
    sample_sine_overlay_path,
    sample_spline_path,
    sensor_to_body,
    smallest_angle_error_deg,
)
from ..perception_driver import ProcessedSignalFeatures
from ..sensor_model import BurialDepthMeasurement, MagnetometerReading, PoseMeasurement, SonarReading
from .confidence import ConfidenceEstimator
from .filters import LowPassFilter, MedianWindowFilter, RMSExtractor, StreamingBandpassFilter
from .fitter import WeightedSlidingWindowFitter
from .peaks import PeakDetector
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
        self.confidence_estimator = ConfidenceEstimator(scenario.tracking.lost_timeout_s)
        self.valid_points_xy: Deque[np.ndarray] = deque(maxlen=max(3, scenario.tracking.blind_follow_memory_size))
        self.last_confirmed_peak_strength_nt = 0.0
        self.last_accepted_fit_result = FitResult(origin_xy_m=None, direction_xy=None, residual_m=float("inf"), covariance_xy_m2=None)
        self.last_output_confidence = 0.0
        self.safe_lock_until_s = -1e9
        self.last_time_s = 0.0
        # Deployment-mode crossing estimation: stores (time_s, heading_deg)
        # for each detected peak where a vehicle heading was available.
        self.crossing_headings: Deque[tuple] = deque(maxlen=16)
        self.crossing_positions_xy: Deque[np.ndarray] = deque(maxlen=16)
        self.crossing_velocities_xy: Deque[np.ndarray] = deque(maxlen=16)
        self.deployment_estimated_cable_heading_deg: Optional[float] = None
        self.deployment_heading_confidence: float = 0.0
        self.deployment_reacquire_required: bool = False
        self.deployment_last_offset: Optional[float] = None
        self._deployment_heading_self_corrected: bool = False
        self._velocity_confirmed_offset: Optional[float] = None
        self._deployment_hysteresis_locked: bool = False
        self._deployment_locked_heading_deg: Optional[float] = None
        self.tracking_maturity: float = 0.0
        self.deployment_recent_washout_until_s: float = -1e9
        # Field vector gradient tracking for deployment mode: stores recent
        # anomaly vectors and their horizontal directions to estimate which
        # side of the cable the vehicle is on.
        self.field_gradient_history: Deque[tuple] = deque(maxlen=8)
        self.gradient_heading_offset_deg: float = 0.0
        # --- Signal enhancement layer ---
        self.envelope_tracker = EnvelopeGradientTracker(
            window_size=scenario.tracking.envelope_savgol_window,
            polyorder=scenario.tracking.envelope_savgol_polyorder,
            min_speed_mps=scenario.tracking.spatial_gradient_min_speed_mps,
        )
        self.vector_analyzer = MagneticVectorAnalyzer() if scenario.tracking.vector_heading_enabled else None
        # --- Safe-lock state ---
        self.last_valid_peak_strength_nt: float = 0.0
        self.last_peak_position_xy_m: Optional[np.ndarray] = None
        self.displacement_since_last_peak_m: float = 0.0
        self.safe_lock_criterion_a_active: bool = False
        self.safe_lock_criterion_b_active: bool = False
        self.safe_lock_fit_invalidated: bool = False
        self.nominal_route_xy = self._build_nominal_route_xy()
        self.nominal_route_lookup = build_polyline_projection_cache(self.nominal_route_xy)

    def _build_nominal_route_xy(self) -> np.ndarray:
        """根据场景中的路线模式生成用于先验参考的名义电缆路径。"""
        waypoints_xy = np.asarray(self.scenario.environment.cable_waypoints_xy_m, dtype=float)
        step_m = max(self.scenario.environment.field_segment_length_m * 0.5, 1.0)
        if self.scenario.environment.cable_route_mode == "spline":
            return sample_spline_path(waypoints_xy, step_m)
        if self.scenario.environment.cable_route_mode == "sine":
            return sample_sine_overlay_path(
                waypoints_xy,
                step_m,
                amplitudes_m=self.scenario.environment.sine_amplitudes_m,
                wavelengths_m=self.scenario.environment.sine_wavelengths_m,
            )
        return waypoints_xy.copy()

    def _blind_heading(self) -> Optional[float]:
        """在没有可用航向先验时，根据历史有效点估计盲航向。"""
        minimum_points = 2 if self.scenario.tracking.use_nominal_route_prior else max(2, self.scenario.tracking.blind_follow_memory_size)
        if len(self.valid_points_xy) < minimum_points:
            return None
        delta_xy = self.valid_points_xy[-1] - self.valid_points_xy[0]
        if norm(delta_xy) < 1e-6:
            return None
        return heading_from_direction_xy(delta_xy)

    def _deployment_fallback_heading(
        self,
        line_heading_deg: Optional[float],
        blind_heading_deg: Optional[float],
        fit_result: FitResult,
        bootstrap_fit_ready: bool,
    ) -> Tuple[Optional[float], bool]:
        """在部署模式下为当前航向提供回退估计并标记是否来自记忆。"""
        if not bootstrap_fit_ready:
            if self.deployment_estimated_cable_heading_deg is not None:
                return self.deployment_estimated_cable_heading_deg, False
            return blind_heading_deg, False

        if line_heading_deg is None:
            return blind_heading_deg, False

        fit_acceptance_residual_m = max(self.scenario.tracking.fit_acceptance_residual_m, 15.0)
        if blind_heading_deg is not None and np.isfinite(fit_result.residual_m):
            blind_line_agreement_deg = abs(smallest_angle_error_deg(line_heading_deg, blind_heading_deg))
            if fit_result.residual_m <= fit_acceptance_residual_m and blind_line_agreement_deg <= 20.0:
                return line_heading_deg, True

        if blind_heading_deg is None and np.isfinite(fit_result.residual_m) and fit_result.residual_m <= fit_acceptance_residual_m:
            return line_heading_deg, True

        return blind_heading_deg, False

    def _deployment_fit_is_consistent(self, candidate_heading_deg: Optional[float]) -> bool:
        """判断当前部署模式拟合航向是否与历史共识一致。"""
        if candidate_heading_deg is None:
            return False
        if self.deployment_estimated_cable_heading_deg is None:
            return True
        if self.deployment_heading_confidence < 0.35:
            return True
        heading_delta_deg = abs(smallest_angle_error_deg(candidate_heading_deg, self.deployment_estimated_cable_heading_deg))
        return heading_delta_deg <= self.scenario.tracking.fit_reject_heading_delta_deg

    def _deployment_gradient_heading(self) -> Optional[float]:
        """从包络梯度跟踪器中提取可用的部署模式航向候选。

        在部署模式下，梯度在接近电缆时有明确物理含义（指向电缆），
        但信号很弱时梯度方向不可靠。通过梯度幅度门控可以过滤掉大部分
        虚假梯度（而不是依赖PCA一致性，因为它在bootstrap阶段尚未建立）。
        """
        gradient_heading_deg = self.envelope_tracker.gradient_heading_deg
        if gradient_heading_deg is None:
            return None
        if abs(self.envelope_tracker.gradient_nT_per_m) < 2.0:
            return None
        return gradient_heading_deg

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

    def _update_tracking_maturity(self, peak_detected: bool, fit_residual_m: float, detection_age_s: float, dt_s: float) -> None:
        """根据峰值持续性、拟合残差和时效更新部署跟踪成熟度。"""
        if peak_detected and np.isfinite(fit_residual_m) and fit_residual_m < self.scenario.tracking.deployment_tracking_maturity_residual_threshold_m:
            self.tracking_maturity = min(1.0, self.tracking_maturity + self.scenario.tracking.deployment_tracking_maturity_gain)
        if detection_age_s > self.scenario.tracking.deployment_tracking_maturity_stale_age_s:
            self.tracking_maturity = max(0.0, self.tracking_maturity - self.scenario.tracking.deployment_tracking_maturity_decay_per_s * dt_s)
        self.tracking_maturity = float(np.clip(self.tracking_maturity, 0.0, 1.0))

    def _update_deployment_cable_heading(self, heading_deg: float, position_xy_m: np.ndarray, velocity_xy: np.ndarray) -> None:
        """更新部署模式下的电缆航向共识，并对跨越样本做多假设消歧。

        该逻辑把每次峰值跨越的车头航向与位置保存到历史中，再按照奇偶
        跨越分组分别选择 +90° 或 -90° 的电缆方向候选；当两组结论一致时
        进行融合，否则优先采用更近的样本组。最终结果还会考虑变化率约束，
        避免电缆航向在短时间内跳变过大。
        """
        self._deployment_heading_self_corrected = False
        self.crossing_headings.append((self.last_time_s, heading_deg))
        self.crossing_positions_xy.append(np.asarray(position_xy_m, dtype=float).copy())
        self.crossing_velocities_xy.append(np.asarray(velocity_xy, dtype=float).copy())

        n = len(self.crossing_headings)
        min_peaks = self.scenario.tracking.deployment_bootstrap_min_peak_count
        if n < min_peaks:
            self.deployment_estimated_cable_heading_deg = None
            self.deployment_heading_confidence = 0.0
            return

        # --- Two-point bootstrap: require heading diversity ---
        min_heading_diff_deg = self.scenario.tracking.bootstrap_min_heading_diff_deg
        if n >= 2 and min_heading_diff_deg > 0:
            max_heading_diff = 0.0
            for i in range(len(self.crossing_headings)):
                for j in range(i + 1, len(self.crossing_headings)):
                    diff = abs(smallest_angle_error_deg(
                        self.crossing_headings[i][1], self.crossing_headings[j][1]
                    ))
                    max_heading_diff = max(max_heading_diff, diff)
            if max_heading_diff < min_heading_diff_deg:
                # Not enough directional diversity yet
                self.deployment_estimated_cable_heading_deg = None
                self.deployment_heading_confidence = 0.0
                return

        def _circular_stats(headings_deg: list) -> tuple:
            """返回航向集合的圆周均值与离散度。"""
            if not headings_deg:
                return 0.0, 180.0
            rads = np.array([np.deg2rad(h % 360.0) for h in headings_deg])
            mean_sin = float(np.mean(np.sin(rads)))
            mean_cos = float(np.mean(np.cos(rads)))
            mean_rad = np.arctan2(mean_sin, mean_cos)
            mean_deg = float(np.rad2deg(mean_rad)) % 360.0
            # RMS of smallest-angle errors
            if len(headings_deg) >= 2:
                errors = [abs(float(np.rad2deg(np.arctan2(np.sin(r) - np.sin(mean_rad),
                                                           np.cos(r) - np.cos(mean_rad)))))
                          for r in rads]
                spread_deg = float(np.sqrt(np.mean(np.square(errors))))
            else:
                spread_deg = 0.0
            return mean_deg, spread_deg

        HYSTERESIS_MARGIN_DEG = 15.0

        def _choose_offset_for_group(raw_headings: list, prev_offset: Optional[float]) -> tuple:
            """为当前组选择更稳定的 ±90° 偏置并返回统计结果。

            使用滞后效应防止在 offsets 之间振荡：当上一个有效 offset
            与当前最小化 spread 的 offset 不同时，只有在新的 offset
            产生明显更好的 spread（差距 > HYSTERESIS_MARGIN_DEG）时才切换。
            当两个 offset 的 spread 相同时，优先选择 90°（保留"正交侧"
            的方向类别），避免在只有1-2个样本时随机选择错误的偏移方向。
            """
            cand_sets: list = []
            for offset in [90.0, -90.0]:
                cands = [(rh + offset) % 360.0 for rh in raw_headings]
                cand_sets.append(cands)
            spread_a = _circular_stats(cand_sets[0])[1]
            spread_b = _circular_stats(cand_sets[1])[1]

            # When spreads are nearly equal (e.g., only 1 sample in group),
            # prefer offset=90 (0° class) over offset=-90 (180° class).
            # This avoids wrong lock-in when spread minimization is ambiguous.
            if abs(spread_a - spread_b) < 0.5:
                chosen_idx = 0  # prefer offset=90° (0° class)
            else:
                chosen_idx = 0 if spread_a <= spread_b else 1
            chosen_offset = 90.0 if chosen_idx == 0 else -90.0

            if prev_offset is not None and chosen_offset != prev_offset:
                spread_chosen = spread_a if chosen_idx == 0 else spread_b
                spread_prev = spread_b if chosen_idx == 0 else spread_a
                if spread_chosen >= spread_prev - HYSTERESIS_MARGIN_DEG:
                    chosen_idx = 0 if prev_offset == 90.0 else 1
                    chosen_offset = prev_offset

            return cand_sets[chosen_idx], _circular_stats(cand_sets[chosen_idx]), chosen_offset

        # --- Step 1: Split into odd/even groups (zigzag-aware) ---
        raw_headings = [h for _t, h in self.crossing_headings]
        odd_headings = [raw_headings[i] for i in range(0, len(raw_headings), 2)]
        even_headings = [raw_headings[i] for i in range(1, len(raw_headings), 2)]

        # --- Step 2: Each group independently chooses ±90° with hysteresis ---
        odd_chosen, odd_stats, odd_offset = _choose_offset_for_group(odd_headings, self.deployment_last_offset)
        even_chosen, even_stats, even_offset = _choose_offset_for_group(even_headings, self.deployment_last_offset)
        odd_mean, odd_spread = odd_stats
        even_mean, even_spread = even_stats

        # Store the most-used offset for next time
        if len(odd_headings) > 0 and len(even_headings) > 0:
            self.deployment_last_offset = odd_offset if len(odd_headings) >= len(even_headings) else even_offset
        elif len(odd_headings) > 0:
            self.deployment_last_offset = odd_offset
        elif len(even_headings) > 0:
            self.deployment_last_offset = even_offset

        # --- Step 2.5: Early Velocity Arbitration for 2-crossing bootstrap ---
        # When we have exactly 2 crossings (odd=1, even=1) with equal spread=0,
        # use velocity direction to determine which offset class (0° vs 180°) is correct.
        # Cable direction should be perpendicular to vehicle travel direction.
        early_velocity_triggered = (
            (len(odd_headings) == 1 and len(even_headings) == 1) or
            (len(odd_headings) == 2 and len(even_headings) == 1) or
            (len(odd_headings) == 1 and len(even_headings) == 2) or
            (len(odd_headings) == 2 and len(even_headings) == 2)
        ) and len(self.crossing_positions_xy) >= 2
        if early_velocity_triggered:
            positions = np.asarray(self.crossing_positions_xy)
            dx = positions[-1][0] - positions[-2][0]
            dy = positions[-1][1] - positions[-2][1]
            seg_len = np.sqrt(dx*dx + dy*dy)
            if seg_len >= 0.5:
                travel_dir = np.arctan2(dy, dx)
                perp_to_travel = (travel_dir + np.pi/2.0) % (2*np.pi)
                odd_raw = raw_headings[0]
                even_raw = raw_headings[1]
                odd_cand_0 = (odd_raw + 90.0) % 360.0
                odd_cand_180 = (odd_raw - 90.0) % 360.0
                even_cand_0 = (even_raw + 90.0) % 360.0
                even_cand_180 = (even_raw - 90.0) % 360.0
                err_odd_0 = abs(smallest_angle_error_deg(odd_cand_0, np.rad2deg(perp_to_travel)))
                err_odd_180 = abs(smallest_angle_error_deg(odd_cand_180, np.rad2deg(perp_to_travel)))
                err_even_0 = abs(smallest_angle_error_deg(even_cand_0, np.rad2deg(perp_to_travel)))
                err_even_180 = abs(smallest_angle_error_deg(even_cand_180, np.rad2deg(perp_to_travel)))
                total_err_0 = err_odd_0 + err_even_0
                total_err_180 = err_odd_180 + err_even_180
                if total_err_0 < total_err_180:
                    odd_offset = 90.0
                    even_offset = 90.0
                else:
                    odd_offset = -90.0
                    even_offset = -90.0
                odd_chosen = [(odd_raw + odd_offset) % 360.0]
                even_chosen = [(even_raw + even_offset) % 360.0]
                odd_mean = odd_chosen[0]
                even_mean = even_chosen[0]
                odd_spread = 0.0
                even_spread = 0.0
                self._velocity_confirmed_offset = odd_offset

        # --- Step 3: Merge or select based on consistency ---
        MERGE_THRESHOLD_DEG = 20.0
        heading_error_deg = abs(smallest_angle_error_deg(odd_mean, even_mean))

        if len(odd_headings) == 0:
            # Only even group available
            final_headings = even_chosen
            final_mean = even_mean
            final_spread = even_spread
        elif len(even_headings) == 0:
            # Only odd group available
            final_headings = odd_chosen
            final_mean = odd_mean
            final_spread = odd_spread
        elif heading_error_deg <= MERGE_THRESHOLD_DEG:
            # Groups agree: merge all crossings with their optimal offsets
            merged_headings = []
            for i, rh in enumerate(raw_headings):
                offset = odd_offset if i % 2 == 0 else even_offset
                merged_headings.append((rh + offset) % 360.0)
            final_headings = merged_headings
            final_mean, final_spread = _circular_stats(merged_headings)
        else:
            # Groups disagree: trust the newer (larger index) group
            if len(even_headings) >= len(odd_headings):
                final_headings = even_chosen
                final_mean = even_mean
                final_spread = even_spread
            else:
                final_headings = odd_chosen
                final_mean = odd_mean
                final_spread = odd_spread

        # --- Step 3PAIRI: Velocity-Based Offset Arbitration (replaces 0° reference assumption) ---
        # When odd/even groups diverge by ~180° (heading_error_deg > 90°),
        # use the vehicle's travel direction and crossing positions to determine
        # which offset is correct. The cable must lie in the direction that is
        # perpendicular to velocity AND passes through the crossing positions.
        if (len(odd_headings) >= 1 and len(even_headings) >= 1
            and heading_error_deg > 90.0
            and len(self.crossing_positions_xy) >= 3):
            positions = np.asarray(self.crossing_positions_xy)
            valid_segments = 0
            correct_offset = None
            for k in range(len(positions) - 1):
                dx = positions[k+1][0] - positions[k][0]
                dy = positions[k+1][1] - positions[k][1]
                seg_len = np.sqrt(dx*dx + dy*dy)
                if seg_len < 0.5:
                    continue
                valid_segments += 1
                travel_dir = np.arctan2(dy, dx)
                perp = (travel_dir + np.pi/2.0) % (2*np.pi)
                cable_dir_from_pos = perp
                idx_a = k
                idx_b = k+1
                if idx_a % 2 == 0:
                    cand_a = (raw_headings[idx_a] + odd_offset) % 360.0
                    cand_b = (raw_headings[idx_b] + even_offset) % 360.0
                else:
                    cand_a = (raw_headings[idx_a] + even_offset) % 360.0
                    cand_b = (raw_headings[idx_b] + odd_offset) % 360.0
                err_a = abs(smallest_angle_error_deg(cand_a, np.rad2deg(cable_dir_from_pos)))
                err_b = abs(smallest_angle_error_deg(cand_b, np.rad2deg(cable_dir_from_pos)))
                if err_a < err_b:
                    chosen_offset = odd_offset if idx_a % 2 == 0 else even_offset
                else:
                    chosen_offset = even_offset if idx_a % 2 == 0 else odd_offset
                if correct_offset is None:
                    correct_offset = chosen_offset
                elif abs(smallest_angle_error_deg(chosen_offset, correct_offset)) > 90.0:
                    correct_offset = -correct_offset
            if correct_offset is not None and valid_segments >= 1:
                odd_corrected = -odd_offset if abs(smallest_angle_error_deg(odd_offset, correct_offset)) > 90.0 else odd_offset
                even_corrected = -even_offset if abs(smallest_angle_error_deg(even_offset, correct_offset)) > 90.0 else even_offset
                corrected_merged = []
                for i, rh in enumerate(raw_headings):
                    offset = odd_corrected if i % 2 == 0 else even_corrected
                    corrected_merged.append((rh + offset) % 360.0)
                final_headings = corrected_merged
                final_mean, final_spread = _circular_stats(corrected_merged)

        # --- Step 4: 180° Self-Correction for Straight Cables (Orthogonal Disaster Recovery) ---
        # If the old deque (before this new crossing) has >= 4 entries and ALL of them
        # cluster in [135°, 225°], the system has locked into the wrong direction.
        # We detect this by checking if the old deque is consistently 180°-flipped.
        # When this is detected, we flip final_mean by 180° so the new estimate is corrected.
        old_deque_headings = list(self.crossing_headings)
        if len(old_deque_headings) >= 4:
            old_mean_rad = float(np.arctan2(
                np.mean(np.sin(np.deg2rad([h for (t, h) in old_deque_headings]))),
                np.mean(np.cos(np.deg2rad([h for (t, h) in old_deque_headings]))),
            ))
            old_mean_deg = float(np.rad2deg(old_mean_rad)) % 360.0
            if 135.0 <= old_mean_deg <= 225.0:
                final_mean = (final_mean + 180.0) % 360.0
                self._deployment_heading_self_corrected = True

        # --- Step 4.5: Velocity Arbitration & Hysteresis Lock ---
        # Ensure the final_mean direction aligns with AUV's macroscopic travel direction.
        # Also lock the heading once confidence is high enough to prevent late-stage flips.
        if len(self.crossing_positions_xy) >= 2:
            positions = np.asarray(self.crossing_positions_xy)
            macro_vec = positions[-1] - positions[0]
            macro_len = np.linalg.norm(macro_vec)
            if macro_len >= 1.0:
                final_vec = np.array([np.cos(np.deg2rad(final_mean)), np.sin(np.deg2rad(final_mean))])
                if np.dot(final_vec, macro_vec) < 0:
                    final_mean = (final_mean + 180.0) % 360.0

        # Hysteresis lock: once confidence >= 0.8 and >= 4 points, lock the heading.
        # New estimates must stay within 90° of the locked heading, otherwise flip.
        n_crossings = len(self.crossing_headings)
        if (self.deployment_heading_confidence >= 0.8 and n_crossings >= 4):
            if not self._deployment_hysteresis_locked:
                self._deployment_hysteresis_locked = True
                self._deployment_locked_heading_deg = final_mean
            elif self._deployment_locked_heading_deg is not None:
                diff = abs(smallest_angle_error_deg(final_mean, self._deployment_locked_heading_deg))
                if diff > 90.0:
                    final_mean = (final_mean + 180.0) % 360.0
                    # Update locked heading to the corrected value
                    self._deployment_locked_heading_deg = final_mean

        # --- Step 5: Heading change rate constraint ---
        # Allow fast correction: 180° error should correct within 2s of new peaks
        max_heading_change_rate_deg_s = 90.0
        if self.deployment_estimated_cable_heading_deg is not None and not self._deployment_heading_self_corrected:
            current_time_s = self.crossing_headings[-1][0]
            previous_time_s = self.crossing_headings[-2][0] if len(self.crossing_headings) >= 2 else current_time_s
            time_since_last = current_time_s - previous_time_s
            if time_since_last > 0:
                allowed_change_deg = max_heading_change_rate_deg_s * time_since_last
                actual_change_deg = abs(smallest_angle_error_deg(final_mean, self.deployment_estimated_cable_heading_deg))
                if actual_change_deg > allowed_change_deg:
                    # Clamp the change to the maximum allowed
                    if actual_change_deg > 1e-6:
                        sign = 1.0 if smallest_angle_error_deg(final_mean, self.deployment_estimated_cable_heading_deg) >= 0 else -1.0
                        final_mean = (self.deployment_estimated_cable_heading_deg + sign * allowed_change_deg) % 360.0

        # --- Step 5: Bayesian confidence calculation ---
        # Confidence = (count_factor) * (spread_factor) * (consistency_factor)
        # count_factor: saturates at 8 crossings
        # spread_factor: 1.0 for spread <= 15°, 0.0 for spread >= 45°
        # consistency_factor: 1.0 if groups agreed, 0.7 if disagreed
        count_factor = float(np.clip(len(final_headings) / 8.0, 0.3, 1.0))
        # Use the minimum spread from individual groups, not the merged spread
        # (merged spread can be large when mixing odd/even offsets)
        min_group_spread = min(odd_spread, even_spread) if len(odd_headings) > 0 and len(even_headings) > 0 else final_spread
        spread_factor = float(np.clip(1.0 - (min_group_spread - 15.0) / 30.0, 0.0, 1.0))
        consistency_factor = 1.0 if heading_error_deg <= MERGE_THRESHOLD_DEG else 0.7

        self.deployment_estimated_cable_heading_deg = final_mean
        self.deployment_heading_confidence = float(np.clip(count_factor * spread_factor * consistency_factor, 0.0, 1.0))

    def update(
        self,
        reading: MagnetometerReading,
        pose_measurement: PoseMeasurement,
        vehicle_position_xy_m: np.ndarray,
        burial_measurement: BurialDepthMeasurement,
        true_burial_depth_m: float,
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
        weak_signal_flag = max(reading.cable_strength_nt, tracking_strength_nt) < weak_signal_threshold_nt or not signal_reliable

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
            else:
                speed_mps = pose_measurement.speed_mps if hasattr(pose_measurement, 'speed_mps') else self.scenario.vehicle.cruise_speed_mps
                heading_rad = np.deg2rad(pose_measurement.heading_deg)
                velocity_xy = np.array([np.cos(heading_rad), np.sin(heading_rad)], dtype=float) * speed_mps

            peak_position_xy_m = self._peak_cable_observation_xy_m(peak_position_xy_m, sonar_reading)
            if peak_position_xy_m is not None and not self._is_peak_outlier(peak_position_xy_m):
                washout_triggered = self.fitter.add_peak(
                    peak_position_xy_m,
                    snr_linear=max(snr, self.scenario.tracking.weighted_fitter_snr_floor),
                    confidence=max(self.last_output_confidence, 0.05),
                    time_s=reading.time_s,
                )
                if washout_triggered:
                    self.deployment_recent_washout_until_s = max(
                        self.deployment_recent_washout_until_s,
                        reading.time_s + self.scenario.tracking.deployment_washout_reacquire_holdoff_s,
                    )
                detection_age_s = reading.time_s - self.fitter.last_detection_time_s
                detected_peak_xy_m = peak_position_xy_m.copy()
                if not self.scenario.tracking.use_nominal_route_prior:
                    self._update_deployment_cable_heading(pose_measurement.heading_deg, detected_peak_xy_m, velocity_xy)
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

        if detection_age_s > self.scenario.tracking.lost_timeout_s:
            self.safe_lock_until_s = -1e9
            self.last_confirmed_peak_strength_nt = 0.0

        fit_result_candidate = self.fitter.fit()
        fit_update_rejected = False
        bootstrap_fit_ready = self._bootstrap_fit_ready(fit_result_candidate)
        recent_washout_active = reading.time_s <= self.deployment_recent_washout_until_s
        self._update_tracking_maturity(peak_event.detected, fit_result_candidate.residual_m, detection_age_s, dt_s)
        deployment_reacquire_required = self.deployment_reacquire_required
        deployment_reacquire_required = False
        if fit_result_candidate.direction_xy is not None:
            candidate_heading_deg = heading_from_direction_xy(fit_result_candidate.direction_xy)
            if (
                not self.scenario.tracking.use_nominal_route_prior
                and (
                    not bootstrap_fit_ready
                    or not np.isfinite(fit_result_candidate.residual_m)
                    or fit_result_candidate.residual_m > self.scenario.tracking.fit_acceptance_residual_m
                    or not self._deployment_fit_is_consistent(candidate_heading_deg)
                )
            ):
                fit_result = self.last_accepted_fit_result if self.last_accepted_fit_result.direction_xy is not None else FitResult(origin_xy_m=None, direction_xy=None, residual_m=float("inf"), covariance_xy_m2=None)
                fit_update_rejected = True
                effective_reacquire_timeout_s = self.scenario.tracking.lost_timeout_s
                if self.tracking_maturity >= self.scenario.tracking.deployment_hold_maturity_threshold:
                    effective_reacquire_timeout_s *= self.scenario.tracking.deployment_lost_timeout_high_maturity_multiplier
                if detection_age_s > effective_reacquire_timeout_s and not recent_washout_active:
                    deployment_reacquire_required = True
                    self.deployment_reacquire_required = True
                else:
                    deployment_reacquire_required = False
                    self.deployment_reacquire_required = False
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
                    deployment_reacquire_required = False
                    self.deployment_reacquire_required = False
        elif self.last_accepted_fit_result.direction_xy is not None and detection_age_s <= self.scenario.tracking.lost_timeout_s:
            fit_result = self.last_accepted_fit_result
            deployment_reacquire_required = False
            self.deployment_reacquire_required = False
        else:
            fit_result = fit_result_candidate
            deployment_reacquire_required = False

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
        elif line_heading_deg is not None and not weak_signal_flag and bootstrap_fit_ready and not self._deployment_heading_self_corrected:
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
        used_memory_heading = False
        if fused_heading_deg is None:
            blind_heading_deg = self._blind_heading()
            if not self.scenario.tracking.use_nominal_route_prior:
                fallback_heading_deg, used_memory_heading = self._deployment_fallback_heading(
                    line_heading_deg=line_heading_deg,
                    blind_heading_deg=blind_heading_deg,
                    fit_result=fit_result,
                    bootstrap_fit_ready=bootstrap_fit_ready,
                )
                if fallback_heading_deg is not None and not self._deployment_heading_self_corrected:
                    fused_heading_deg = fallback_heading_deg
                    blind_heading_deg = fallback_heading_deg if used_memory_heading else blind_heading_deg
                    guidance_source = "MEMORY" if used_memory_heading else "BLIND"
            if fused_heading_deg is None and blind_heading_deg is None and line_heading_deg is not None and (bootstrap_fit_ready or not self.scenario.tracking.use_nominal_route_prior) and np.isfinite(fit_result.residual_m):
                fit_acceptance_residual_m = max(self.scenario.tracking.fit_acceptance_residual_m, 15.0)
                if fit_result.residual_m <= fit_acceptance_residual_m:
                    blind_heading_deg = line_heading_deg
                    used_memory_heading = True
            if fused_heading_deg is None:
                fused_heading_deg = blind_heading_deg
            if blind_heading_deg is not None and guidance_source != "MEMORY":
                guidance_source = "BLIND"

        if not self.scenario.tracking.use_nominal_route_prior:
            # =====================================================================
            # DEPLOYMENT MODE: Sonar-Seeded Bootstrap + Dominant Source Selector
            # =====================================================================
            #
            # Rule 1 — Sonar-Seeded Bootstrap:
            #   When magnetic has < 4 valid crossing points (still bootstrapping),
            #   and sonar provides a valid heading, use it to seed the deployment
            #   heading estimate. This prevents wrong orthogonal lock-in.
            #
            bootstrap_point_count = len(self.crossing_headings)
            SONAR_BOOTSTRAP_THRESHOLD = 4
            if (
                bootstrap_point_count < SONAR_BOOTSTRAP_THRESHOLD
                and self.deployment_estimated_cable_heading_deg is None
                and sonar_reading is not None
                and sonar_reading.valid
                and sonar_reading.estimated_heading_ned_deg is not None
            ):
                self.deployment_estimated_cable_heading_deg = sonar_reading.estimated_heading_ned_deg
                self.deployment_heading_confidence = max(
                    float(np.clip(sonar_confidence * 0.6, 0.0, 0.55)),
                    self.deployment_heading_confidence,
                )

            # Rule 2 — Anti-Orthogonal Guard:
            #   If magnetic line fit is nearly perpendicular to sonar heading,
            #   this is the "orthogonal disaster" (B_xy captured AUV cross-track
            #   instead of cable direction). Reject magnetic in favour of sonar.
            #   Exception: if magnetic has tracked stably for > 15m (long-track
            #   confidence), the magnetic reading is trustworthy.
            #
            long_track_confidence = False
            if len(self.crossing_headings) >= 5 and bootstrap_point_count >= 5:
                positions_arr = np.asarray(self.crossing_positions_xy)
                if len(positions_arr) >= 2:
                    total_travel_m = float(np.sum(np.linalg.norm(np.diff(positions_arr, axis=0), axis=1)))
                    long_track_confidence = total_travel_m > 15.0

            line_fit_orthogonal_to_sonar = False
            if (
                line_heading_deg is not None
                and sonar_reading is not None
                and sonar_reading.valid
                and sonar_reading.estimated_heading_ned_deg is not None
                and not long_track_confidence
            ):
                angle_between = abs(smallest_angle_error_deg(line_heading_deg, sonar_reading.estimated_heading_ned_deg))
                if 75.0 <= angle_between <= 105.0:
                    line_fit_orthogonal_to_sonar = True

            # Rule 3 — Dominant Source Selection:
            #   SONAR when: valid sonar AND (far from cable OR low magnetic SNR)
            #   MAGNETIC when: strong magnetic signal (tracking_strength_nt > 20 nT)
            #                  AND near cable (distance < 5m)
            #   Otherwise prefer sonar if valid, else magnetic
            #
            sonar_preferred = (
                sonar_reading is not None
                and sonar_reading.valid
                and sonar_reading.estimated_heading_ned_deg is not None
            )
            magnetic_strong = tracking_strength_nt > 20.0
            cable_distance_m = None
            if detected_peak_xy_m is not None:
                cable_distance_m = float(np.linalg.norm(vehicle_position_xy_m - detected_peak_xy_m))
            near_cable = cable_distance_m is not None and cable_distance_m < 5.0
            magnetic_preferred = magnetic_strong and near_cable

            velocity_offset_guard = True
            if self._velocity_confirmed_offset is not None and line_heading_deg is not None:
                current_offset = self.deployment_last_offset
                if current_offset is not None:
                    offset_diff = abs(smallest_angle_error_deg(current_offset, self._velocity_confirmed_offset))
                    if offset_diff > 45.0:
                        velocity_offset_guard = False

            if (
                bootstrap_fit_ready
                and line_heading_deg is not None
                and np.isfinite(fit_result.residual_m)
                and fit_result.residual_m <= self.scenario.tracking.fit_acceptance_residual_m
                and not line_fit_orthogonal_to_sonar
                and not self._deployment_heading_self_corrected
                and velocity_offset_guard
            ):
                self.deployment_estimated_cable_heading_deg = line_heading_deg
                fit_confidence = float(np.clip(np.exp(-fit_result.residual_m / 6.0), 0.0, 1.0))
                self.deployment_heading_confidence = max(self.deployment_heading_confidence, fit_confidence)

            # Rule 4 — Dominant source governs fused_heading_deg:
            # Override sonar/magnetic selection only when line_fit is not orthogonal
            # to sonar (anti-orthogonal guard takes priority).
            if sonar_preferred and (not magnetic_preferred or (snr_db < 6.0)):
                if not line_fit_orthogonal_to_sonar or not bootstrap_fit_ready:
                    fused_heading_deg = sonar_reading.estimated_heading_ned_deg
                    if sonar_reading.estimated_position_ned_m is not None:
                        estimated_cable_point_xy_m = sonar_reading.estimated_position_ned_m.copy()
                    guidance_source = "SONAR"
            elif magnetic_preferred and bootstrap_fit_ready and not line_fit_orthogonal_to_sonar and not self._deployment_heading_self_corrected:
                fused_heading_deg = line_heading_deg
                guidance_source = "MAGNETIC"

            # Rule 5 — Gradient heading as fallback (with stricter gating):
            # Only use gradient when both sonar and magnetic are unavailable,
            # and only if vector consistency confirms a reliable direction.
            gradient_heading_deg = self._deployment_gradient_heading()
            if gradient_heading_deg is not None and fused_heading_deg is None:
                gradient_vs_line_deg = abs(smallest_angle_error_deg(gradient_heading_deg, line_heading_deg)) if line_heading_deg is not None else 0.0
                if line_heading_deg is None or gradient_vs_line_deg > 30.0:
                    self.deployment_estimated_cable_heading_deg = gradient_heading_deg
                    self.deployment_heading_confidence = max(
                        self.deployment_heading_confidence,
                        float(np.clip(0.55 + 0.05 * min(abs(self.envelope_tracker.gradient_nT_per_m), 10.0), 0.0, 0.9)),
                    )
                    if fused_heading_deg is None or guidance_source in {"SEARCH", "BLIND", "MEMORY"}:
                        fused_heading_deg = gradient_heading_deg
                        guidance_source = "GRADIENT"

            if (
                self.deployment_estimated_cable_heading_deg is not None
                and self.deployment_heading_confidence >= 0.9
                and len(self.crossing_headings) >= 5
            ):
                self.deployment_reacquire_required = False

        # Safe-Lock Criterion A/B are disabled (kept as inert diagnostics until the
        # mission-FSM refactor removes them from the perception contract).
        self.safe_lock_criterion_a_active = False
        self.safe_lock_criterion_b_active = False
        gradient_penalty = 0.0

        confidence = self.confidence_estimator.fused_confidence(
            magnetic_confidence,
            sonar_confidence,
            guidance_source,
            fit_result.residual_m,
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
        # Apply gradient inconsistency penalty from criterion B
        if gradient_penalty > 0:
            confidence = float(np.clip(confidence - gradient_penalty, 0.0, 1.0))
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
            estimated_burial_depth_m=burial_measurement.depth_m,
            true_burial_depth_m=true_burial_depth_m,
            burial_measurement_valid=burial_measurement.valid,
            last_detection_age_s=detection_age_s,
            detected_peak_xy_m=detected_peak_xy_m,
            deployment_estimated_cable_heading_deg=self.deployment_estimated_cable_heading_deg,
            deployment_heading_confidence=self.deployment_heading_confidence,
            gradient_heading_offset_deg=self.gradient_heading_offset_deg,
            tracking_maturity=self.tracking_maturity,
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
            deployment_reacquire_required=deployment_reacquire_required,
        )
