"""Perception layer: numeric-only sonar-magnetic fusion, filtering and path estimation."""

from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import numpy as np
from scipy.signal import butter, sosfilt

from .config import ScenarioConfig
from .math_utils import (
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
from .perception_driver import ProcessedSignalFeatures
from .sensor_model import BurialDepthMeasurement, MagnetometerReading, PoseMeasurement, SonarReading


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
    gradient_heading_offset_deg: float = 0.0
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


class LowPassFilter:
    """实现一阶离散低通滤波器，用于平滑瞬时测量值。"""

    def __init__(self, time_constant_s: float) -> None:
        """初始化低通滤波器并设置时间常数。"""
        self.time_constant_s = max(time_constant_s, 1e-3)
        self.value = 0.0
        self.initialized = False

    def update(self, measurement: float, dt_s: float) -> float:
        """根据新测量值更新滤波输出。"""
        alpha = dt_s / (self.time_constant_s + dt_s)
        if not self.initialized:
            self.value = measurement
            self.initialized = True
        else:
            self.value = (1.0 - alpha) * self.value + alpha * measurement
        return self.value


class MedianWindowFilter:
    """实现固定窗口中值滤波器，用于抑制瞬态离群点。"""

    def __init__(self, window_size: int) -> None:
        """初始化中值滤波窗口大小。"""
        self.buffer: Deque[float] = deque(maxlen=max(1, window_size))

    def update(self, measurement: float) -> float:
        """写入新样本并返回窗口中值。"""
        self.buffer.append(measurement)
        return float(np.median(np.asarray(self.buffer, dtype=float)))


class StreamingBandpassFilter:
    """实现流式带通滤波器，用于提取目标工频磁信号分量。"""

    def __init__(self, sample_rate_hz: float, center_frequency_hz: float, half_width_hz: float, order: int = 2) -> None:
        """根据采样率和目标频率构建带通 SOS 滤波器。"""
        nyquist_hz = 0.5 * max(sample_rate_hz, 1e-6)
        low_hz = max(0.5, center_frequency_hz - half_width_hz)
        high_hz = min(nyquist_hz * 0.95, center_frequency_hz + half_width_hz)
        if low_hz >= high_hz:
            high_hz = min(nyquist_hz * 0.95, low_hz + max(1.0, 0.1 * center_frequency_hz))
        low_normalized = max(1e-4, low_hz / nyquist_hz)
        high_normalized = min(0.999, high_hz / nyquist_hz)
        self.sos = butter(order, [low_normalized, high_normalized], btype="bandpass", output="sos")
        self.zi = np.zeros((self.sos.shape[0], 2, 3), dtype=float)

    def update(self, vector_nt: np.ndarray) -> np.ndarray:
        """对三轴磁场向量执行逐轴流式滤波。"""
        filtered = np.zeros(3, dtype=float)
        for axis_index in range(3):
            result, self.zi[:, :, axis_index] = sosfilt(
                self.sos,
                [float(vector_nt[axis_index])],
                zi=self.zi[:, :, axis_index],
            )
            filtered[axis_index] = float(result[0])
        return filtered


class RMSExtractor:
    """在滑动窗口内提取均方根幅值，用于形成跟踪强度。"""

    def __init__(self, sample_rate_hz: float, minimum_frequency_hz: float) -> None:
        """初始化 RMS 窗口大小，保证至少覆盖一个完整周期。"""
        min_window_s = 2.0 / max(minimum_frequency_hz, 1e-6)
        self.window_size_samples = max(3, int(np.ceil(sample_rate_hz * min_window_s)))
        self.buffer: Deque[float] = deque(maxlen=self.window_size_samples)

    def update(self, sample_value: float) -> float:
        """写入新样本并返回当前窗口 RMS 值。"""
        self.buffer.append(sample_value)
        if not self.buffer:
            return 0.0
        values = np.asarray(self.buffer, dtype=float)
        return float(np.sqrt(np.mean(values**2)))


class PeakDetector:
    """基于上升-峰区-下降状态机检测磁场峰值事件。"""

    def __init__(
        self,
        min_peak_strength_nt: float,
        turn_trigger_ratio: float,
        hysteresis_fraction: float,
        cooldown_s: float,
        ascending_min_samples: int = 2,
        descending_min_samples: int = 2,
        peak_zone_window_size: int = 20,
    ) -> None:
        """初始化峰值检测器的阈值、滞回和冷却参数。"""
        self.min_peak_strength_nt = min_peak_strength_nt
        self.turn_trigger_ratio = turn_trigger_ratio
        self.hysteresis_fraction = hysteresis_fraction
        self.cooldown_s = cooldown_s
        self.ascending_min_samples = max(1, ascending_min_samples)
        self.descending_min_samples = max(1, descending_min_samples)
        self.peak_zone_window_size = max(3, peak_zone_window_size)
        self.current_peak_strength_nt = 0.0
        self.current_peak_time_s = -1e9
        self.current_peak_position_xy_m: Optional[np.ndarray] = None
        self.cooldown_until_s = -1e9
        self.state = "IDLE"
        self.previous_strength_nt: Optional[float] = None
        self.previous_time_s: Optional[float] = None
        self.ascending_count = 0
        self.descending_count = 0
        self.recent_samples: Deque[PeakZoneSample] = deque(maxlen=self.peak_zone_window_size)
        self.peak_zone_samples: Deque[PeakZoneSample] = deque(maxlen=self.peak_zone_window_size)
        # Deep reset mechanism
        self.armed = True
        # Morphology-driven detection window (size 7 for trend analysis)
        self.morphology_window: Deque[PeakZoneSample] = deque(maxlen=7)

    def _reset_tracking_state(self) -> None:
        """重置峰区追踪状态，回到空闲等待下一次峰值。"""
        self.state = "IDLE"
        self.ascending_count = 0
        self.descending_count = 0
        self.current_peak_strength_nt = 0.0
        self.current_peak_time_s = self.previous_time_s if self.previous_time_s is not None else -1e9
        self.current_peak_position_xy_m = None
        self.peak_zone_samples.clear()

    def _update_current_peak(self, sample: PeakZoneSample) -> None:
        """使用当前样本刷新峰区中的最大峰值记录。"""
        if sample.strength_nt >= self.current_peak_strength_nt:
            self.current_peak_strength_nt = sample.strength_nt
            self.current_peak_time_s = sample.time_s
            self.current_peak_position_xy_m = None if sample.position_xy_m is None else sample.position_xy_m.copy()

    def _emit_peak_event(
        self,
        time_s: float,
        vehicle_heading_deg: Optional[float] = None,
        use_nominal_route_prior: bool = True,
    ) -> PeakEvent:
        """从峰区样本中构造最终峰值事件并清空内部状态。"""
        if not self.peak_zone_samples:
            self._reset_tracking_state()
            self.cooldown_until_s = time_s + self.cooldown_s
            return PeakEvent(detected=False)

        strengths_nt = np.array([sample.strength_nt for sample in self.peak_zone_samples], dtype=float)
        times_s = np.array([sample.time_s for sample in self.peak_zone_samples], dtype=float)
        if np.max(strengths_nt) < self.min_peak_strength_nt:
            self._reset_tracking_state()
            self.cooldown_until_s = time_s + self.cooldown_s
            return PeakEvent(detected=False)

        # --- Parabolic interpolation for sub-sample peak time ---
        peak_idx = int(np.argmax(strengths_nt))
        y_peak = float(strengths_nt[peak_idx])
        interpolated_peak_time_s = times_s[peak_idx]
        if len(strengths_nt) >= 3 and peak_idx > 0 and peak_idx < len(strengths_nt) - 1:
            y_prev = strengths_nt[peak_idx - 1]
            y_next = strengths_nt[peak_idx + 1]
            denominator = 2.0 * (2.0 * y_peak - y_prev - y_next)
            if abs(denominator) > 1e-12:
                offset = (y_prev - y_next) / denominator
                offset = float(np.clip(offset, -0.5, 0.5))
                dt_samples = times_s[peak_idx + 1] - times_s[peak_idx]
                interpolated_peak_time_s = times_s[peak_idx] + offset * dt_samples

        # Interpolated peak strength via parabolic estimate
        interpolated_strength = y_peak
        if len(strengths_nt) >= 3 and peak_idx > 0 and peak_idx < len(strengths_nt) - 1:
            y_prev = strengths_nt[peak_idx - 1]
            y_next = strengths_nt[peak_idx + 1]
            curvature = y_prev - 2.0 * y_peak + y_next
            if abs(curvature) > 1e-12:
                interpolated_strength = y_peak - ((y_prev - y_next) ** 2) / (8.0 * curvature)
                if not np.isfinite(interpolated_strength):
                    interpolated_strength = y_peak

        weights = np.maximum(strengths_nt, 1e-6)
        peak_time_s = interpolated_peak_time_s

        centroid_xy_m = None
        interpolated_position_xy_m = None
        weighted_positions = [
            (sample.position_xy_m, weight)
            for sample, weight in zip(self.peak_zone_samples, weights)
            if sample.position_xy_m is not None
        ]
        if weighted_positions:
            positions_xy = np.vstack([position_xy for position_xy, _ in weighted_positions])
            position_weights = np.array([weight for _, weight in weighted_positions], dtype=float)
            centroid_xy_m = np.sum(positions_xy * position_weights[:, None], axis=0) / np.sum(position_weights)

            if len(self.peak_zone_samples) >= 2:
                sample_times = np.array([sample.time_s for sample in self.peak_zone_samples], dtype=float)
                sample_positions = np.array([
                    np.asarray(sample.position_xy_m, dtype=float)
                    for sample in self.peak_zone_samples
                    if sample.position_xy_m is not None
                ], dtype=float)
                valid_time_mask = np.array([sample.position_xy_m is not None for sample in self.peak_zone_samples], dtype=bool)
                sample_times = sample_times[valid_time_mask]
                if sample_times.size >= 2 and sample_positions.shape[0] >= 2:
                    order = np.argsort(sample_times)
                    sample_times = sample_times[order]
                    sample_positions = sample_positions[order]
                    if peak_time_s <= sample_times[0]:
                        interpolated_position_xy_m = sample_positions[0].copy()
                    elif peak_time_s >= sample_times[-1]:
                        interpolated_position_xy_m = sample_positions[-1].copy()
                    else:
                        upper_index = int(np.searchsorted(sample_times, peak_time_s, side="right"))
                        lower_index = max(0, upper_index - 1)
                        upper_index = min(upper_index, sample_times.size - 1)
                        t0 = float(sample_times[lower_index])
                        t1 = float(sample_times[upper_index])
                        p0 = sample_positions[lower_index]
                        p1 = sample_positions[upper_index]
                        if t1 > t0:
                            alpha = float(np.clip((peak_time_s - t0) / (t1 - t0), 0.0, 1.0))
                            interpolated_position_xy_m = (1.0 - alpha) * p0 + alpha * p1
                        else:
                            interpolated_position_xy_m = p0.copy()

        # When the vehicle heading is known, estimate the cable heading from
        # the crossing direction.  An infinite straight wire produces maximum
        # field strength when the vehicle passes perpendicular to it, so the
        # cable direction is approximately ±90° from the vehicle heading at
        # peak.  We record both candidates; disambiguation happens later when
        # multiple peaks are available.
        estimated_cable_heading_deg = None
        if vehicle_heading_deg is not None:
            estimated_cable_heading_deg = vehicle_heading_deg + 90.0

        event = PeakEvent(
            detected=True,
            peak_strength_nt=interpolated_strength,
            peak_time_s=peak_time_s,
            peak_position_xy_m=(
                interpolated_position_xy_m.copy()
                if (interpolated_position_xy_m is not None and not use_nominal_route_prior)
                else (centroid_xy_m.copy() if centroid_xy_m is not None else None)
            ),
            estimated_cable_heading_deg=estimated_cable_heading_deg,
        )
        self._reset_tracking_state()
        self.cooldown_until_s = time_s + self.cooldown_s
        return event

    def update(
        self,
        strength_nt: float,
        time_s: float,
        position_xy_m: Optional[np.ndarray] = None,
        vehicle_heading_deg: Optional[float] = None,
        use_nominal_route_prior: bool = True,
    ) -> PeakEvent:
        """输入新的强度样本，推动峰值状态机并在必要时输出事件。"""
        sample = PeakZoneSample(
            time_s=float(time_s),
            strength_nt=float(strength_nt),
            position_xy_m=None if position_xy_m is None else np.asarray(position_xy_m, dtype=float).copy(),
        )
        self.recent_samples.append(sample)
        self.morphology_window.append(sample)

        if self.previous_strength_nt is None or self.previous_time_s is None:
            self.previous_strength_nt = sample.strength_nt
            self.previous_time_s = sample.time_s
            self.current_peak_strength_nt = sample.strength_nt
            self.current_peak_time_s = sample.time_s
            self.current_peak_position_xy_m = None if sample.position_xy_m is None else sample.position_xy_m.copy()
            return PeakEvent(detected=False)

        # --- Deep Reset / Armed Check ---
        if not self.armed:
            recovery_threshold = max(0.35 * self.current_peak_strength_nt, self.min_peak_strength_nt)
            if sample.strength_nt < recovery_threshold:
                self.armed = True
            else:
                self.previous_strength_nt = sample.strength_nt
                self.previous_time_s = sample.time_s
                return PeakEvent(detected=False)

        if time_s < self.cooldown_until_s:
            self.previous_strength_nt = sample.strength_nt
            self.previous_time_s = sample.time_s
            return PeakEvent(detected=False)

        # --- Morphology-driven detection (replaces slope-based logic) ---
        morphology_trend = self._get_morphology_trend()

        if morphology_trend == "rising":
            self.ascending_count += 1
            self.descending_count = 0
        elif morphology_trend == "falling":
            self.descending_count += 1
            self.ascending_count = 0
        else:
            self.ascending_count = 0
            self.descending_count = 0

        if self.state == "IDLE" and self.ascending_count >= self.ascending_min_samples:
            self.state = "ASCENDING"
            self.peak_zone_samples = deque(self.recent_samples, maxlen=self.peak_zone_window_size)
            self._update_current_peak(sample)
        elif self.state == "ASCENDING":
            self.peak_zone_samples.append(sample)
            self._update_current_peak(sample)
            if morphology_trend != "rising":
                self.state = "PEAK_ZONE"
                self.descending_count = 1 if morphology_trend == "falling" else 0
        elif self.state == "PEAK_ZONE":
            self.peak_zone_samples.append(sample)
            self._update_current_peak(sample)
            if morphology_trend == "falling" and self.descending_count >= self.descending_min_samples:
                event = self._emit_peak_event(
                    sample.time_s,
                    vehicle_heading_deg=vehicle_heading_deg,
                    use_nominal_route_prior=use_nominal_route_prior,
                )
                if event.detected:
                    self.armed = False
                self.previous_strength_nt = sample.strength_nt
                self.previous_time_s = sample.time_s
                return event
            if morphology_trend == "rising":
                self.descending_count = 0

        self.previous_strength_nt = sample.strength_nt
        self.previous_time_s = sample.time_s
        return PeakEvent(detected=False)

    def _get_morphology_trend(self) -> str:
        """Analyze morphology of recent samples to determine trend.

        Uses a hybrid approach: majority vote for rising trend +
        relative drop for falling trend. Relaxed to allow small noise.
        """
        if len(self.morphology_window) < 3:
            return "flat"

        values = [s.strength_nt for s in self.morphology_window]
        n = len(values)

        # Rising detection: count how many recent steps are increasing
        # Allow up to 1 noisy point in the window
        steps_increasing = sum(1 for i in range(1, n) if values[i] > values[i-1])
        total_steps = n - 1
        rising_ratio = steps_increasing / total_steps if total_steps > 0 else 0.0

        # Falling detection: current value dropped below max * turn_trigger_ratio
        max_val = max(values)
        current_val = values[-1]
        drop_ratio = current_val / max_val if max_val > 1e-6 else 1.0

        if rising_ratio >= 0.6:
            return "rising"
        elif drop_ratio < self.turn_trigger_ratio:
            return "falling"
        else:
            return "flat"


class EnvelopeGradientTracker:
    """Compute spatial gradient on the RMS envelope using Savitzky-Golay
    filtering.

    Gradient is computed on the *smoothed* envelope to avoid noise from
    raw 50 Hz residual ripple.  The temporal gradient is then converted to
    a spatial gradient (nT / m) using the current vehicle speed so that the
    feature is invariant to velocity changes.
    """

    def __init__(
        self,
        window_size: int = 7,
        polyorder: int = 2,
        buffer_capacity: int = 40,
        min_speed_mps: float = 0.3,
    ) -> None:
        """初始化包络梯度跟踪器的窗口、缓存与速度约束。"""
        self.window_size = max(3, window_size if window_size % 2 == 1 else window_size + 1)
        self.polyorder = min(polyorder, self.window_size - 1)
        self.buffer_capacity = max(4, buffer_capacity)
        self.min_speed_mps = max(min_speed_mps, 0.05)
        self.time_buffer: Deque[float] = deque(maxlen=self.buffer_capacity)
        self.strength_buffer: Deque[float] = deque(maxlen=self.buffer_capacity)
        self.position_buffer: Deque[np.ndarray] = deque(maxlen=self.buffer_capacity)
        self.gradient_nT_per_m: float = 0.0
        self.gradient_heading_deg: Optional[float] = None
        self.gradient_sign: int = 0  # +1 ascending, -1 descending, 0 flat

    def update(
        self,
        strength_nt: float,
        time_s: float,
        position_xy_m: np.ndarray,
        speed_mps: float,
    ) -> None:
        """更新梯度估计并同步计算信号上升/下降方向。"""
        self.time_buffer.append(time_s)
        self.strength_buffer.append(strength_nt)
        self.position_buffer.append(np.asarray(position_xy_m, dtype=float).copy())

        n = len(self.strength_buffer)
        if n < self.window_size:
            self.gradient_nT_per_m = 0.0
            self.gradient_heading_deg = None
            self.gradient_sign = 0
            return

        # Apply Savitzky-Golay filter and compute derivative
        strengths = np.asarray(list(self.strength_buffer), dtype=float)
        times = np.asarray(list(self.time_buffer), dtype=float)
        positions = np.vstack(list(self.position_buffer))

        try:
            from scipy.signal import savgol_filter
            # Derivative order 1 gives us dRMS/dt in units of index-space
            deriv = savgol_filter(strengths, self.window_size, self.polyorder, deriv=1, delta=1.0)
            # The delta=1.0 means deriv is in per-sample units.
            # Convert to temporal gradient using average dt
            avg_dt = float(np.mean(np.diff(times)))
            temporal_gradient = deriv[-1] / max(avg_dt, 1e-6)

            # Convert to spatial gradient using speed
            effective_speed = max(speed_mps, self.min_speed_mps)
            self.gradient_nT_per_m = temporal_gradient / effective_speed
        except Exception:
            self.gradient_nT_per_m = 0.0

        # Gradient sign: positive = signal ascending, negative = descending
        if abs(self.gradient_nT_per_m) > 0.5:
            self.gradient_sign = 1 if self.gradient_nT_per_m > 0 else -1
        else:
            self.gradient_sign = 0

        # Gradient heading: direction of maximum signal increase
        if n >= 2:
            delta_xy = positions[-1] - positions[-2]
            dist_m = float(np.linalg.norm(delta_xy))
            if dist_m > 1e-3:
                movement_heading = float(np.rad2deg(np.arctan2(delta_xy[1], delta_xy[0]))) % 360.0
                if self.gradient_sign < 0:
                    # Signal decreasing → cable is behind us
                    self.gradient_heading_deg = (movement_heading + 180.0) % 360.0
                elif self.gradient_sign > 0:
                    self.gradient_heading_deg = movement_heading
                else:
                    self.gradient_heading_deg = None
            else:
                self.gradient_heading_deg = None
        else:
            self.gradient_heading_deg = None


class StreamingVectorPCAFitter:
    """Extract the principal component direction from a block of AC magnetic
    vector samples using covariance eigen-analysis.

    For 50 Hz AC cables, the magnetic field oscillates rapidly. Instead of
    using a single instantaneous snapshot (which may land on any phase of the
    sine wave), we accumulate XY vector samples in a sliding window and
    compute the dominant oscillation axis via PCA on the 2x2 covariance matrix.
    """

    def __init__(self, buffer_capacity: int = 20) -> None:
        """Initialize the PCA fitter with a fixed-capacity vector buffer."""
        self.buffer_capacity = max(3, buffer_capacity)
        self._buffer_x: Deque[float] = deque(maxlen=self.buffer_capacity)
        self._buffer_y: Deque[float] = deque(maxlen=self.buffer_capacity)

    def add_sample(self, vector_xy: np.ndarray) -> None:
        """Append a single [Bx, By] sample to the sliding window."""
        self._buffer_x.append(float(vector_xy[0]))
        self._buffer_y.append(float(vector_xy[1]))

    def compute_principal_vector(self) -> Tuple[np.ndarray, float]:
        """Return the principal eigenvector and a consistency score.

        Returns
        -------
        principal_vector : np.ndarray of shape (2,)
            The dominant oscillation axis in the XY plane.
        consistency : float
            A value in [0, 1] reflecting how concentrated the samples are
            along the principal axis (based on eigenvalue ratio and circular
            mean resultant length).
        """
        n = len(self._buffer_x)
        if n < 3:
            return np.array([1.0, 0.0], dtype=float), 0.0

        xs = np.asarray(list(self._buffer_x), dtype=float)
        ys = np.asarray(list(self._buffer_y), dtype=float)

        # Build the 2x2 covariance matrix
        data = np.stack([xs, ys], axis=1)  # (n, 2)
        cov_matrix = np.cov(data, rowvar=False)  # (2, 2)

        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
        # eigenvalues are sorted ascending; take the largest
        principal_idx = int(np.argmax(eigenvalues))
        principal_vec = eigenvectors[:, principal_idx]
        principal_vec = principal_vec / max(np.linalg.norm(principal_vec), 1e-12)

        # Consistency: ratio of largest eigenvalue to total variance
        total_var = float(np.sum(eigenvalues))
        if total_var < 1e-12:
            return principal_vec, 0.0
        eigen_ratio = float(eigenvalues[principal_idx] / total_var)

        # Also compute circular mean resultant length of the vector angles
        angles = np.arctan2(ys, xs)
        mean_r = float(np.sqrt(np.mean(np.cos(angles)) ** 2 + np.mean(np.sin(angles)) ** 2))
        # Combine eigen_ratio and circular consistency
        consistency = float(np.clip(0.6 * eigen_ratio + 0.4 * mean_r, 0.0, 1.0))

        return principal_vec, consistency

    def clear(self) -> None:
        """Reset the internal buffer."""
        self._buffer_x.clear()
        self._buffer_y.clear()


class MagneticVectorAnalyzer:
    """Extract horizontal magnetic vector direction and infer cable heading.

    Physics constraint: at the cable crossing (peak), the horizontal
    magnetic vector B_xy = [Bx, By] is perpendicular to the cable
    direction.  Therefore cable_heading ≈ vector_heading ± 90°.

    For AC mode (e.g. 50 Hz), instead of using an instantaneous snapshot,
    we accumulate vector samples in a sliding window and extract the
    principal oscillation axis via PCA/SVD on the 2×2 covariance matrix.
    This eliminates aliasing caused by sampling at arbitrary phases of the
    AC waveform.

    A dynamic gating mechanism rejects updates when SNR is too low or when
    AUV attitude (roll/pitch) is unstable, which would cause earth-field
    leakage to dominate the anomaly vector.
    """

    def __init__(
        self,
        buffer_capacity: int = 8,
        pca_buffer_capacity: int = 40,
    ) -> None:
        """Initialize the magnetic vector analyzer with PCA support."""
        self.buffer_capacity = max(1, buffer_capacity)
        self.vector_headings: Deque[float] = deque(maxlen=self.buffer_capacity)
        self.vector_magnitudes: Deque[float] = deque(maxlen=self.buffer_capacity)
        self.magnetic_vector_heading_deg: Optional[float] = None
        self.vector_cable_heading_deg: Optional[float] = None
        self.vector_confidence: float = 0.0

        # PCA fitter for AC mode vector extraction
        self.pca_fitter = StreamingVectorPCAFitter(buffer_capacity=pca_buffer_capacity)
        self._previous_vector_xy: Optional[np.ndarray] = None

        # Diagnostic state
        self.vector_consistency_score: float = 0.0
        self.attitude_leakage_risk: bool = False

    def update(
        self,
        anomaly_ned_nt: np.ndarray,
        tracking_strength_nt: float,
        pose_measurement: Optional["PoseMeasurement"] = None,
        snr_db: float = -120.0,
        signal_mode: str = "dc",
    ) -> None:
        """Estimate cable heading from the NED magnetic anomaly vector.

        Parameters
        ----------
        anomaly_ned_nt : np.ndarray
            3-element magnetic anomaly vector in NED coordinates.
        tracking_strength_nt : float
            Current tracking field strength (RMS or filtered).
        pose_measurement : PoseMeasurement, optional
            IMU-derived pose for attitude-based gating.
        snr_db : float
            Current signal-to-noise ratio in dB.
        signal_mode : str
            Signal mode identifier ("dc", "ac_50hz", etc.).
        """
        bx, by = float(anomaly_ned_nt[0]), float(anomaly_ned_nt[1])
        magnitude_xy = float(np.sqrt(bx * bx + by * by))

        # --- Dynamic gating: reject low-SNR updates ---
        if snr_db < 10.0:
            self.attitude_leakage_risk = False
            return

        # --- Dynamic gating: reject high-attitude-risk updates ---
        if pose_measurement is not None:
            roll_ok = abs(float(pose_measurement.roll_deg)) <= 3.0
            pitch_ok = abs(float(pose_measurement.pitch_deg)) <= 3.0
            if not (roll_ok and pitch_ok):
                self.attitude_leakage_risk = True
                return
        self.attitude_leakage_risk = False

        # --- AC mode: use PCA to extract principal oscillation axis ---
        if signal_mode != "dc":
            self.pca_fitter.add_sample(anomaly_ned_nt[:2])
            principal_vec, pca_consistency = self.pca_fitter.compute_principal_vector()

            if pca_consistency < 0.1:
                return

            # Sign alignment: prevent 180° flip between consecutive frames
            if self._previous_vector_xy is not None:
                if float(np.dot(principal_vec, self._previous_vector_xy)) < 0:
                    principal_vec = -principal_vec
            self._previous_vector_xy = principal_vec.copy()

            vector_heading = float(np.rad2deg(np.arctan2(principal_vec[1], principal_vec[0]))) % 360.0
            vector_magnitude = magnitude_xy
            self.vector_consistency_score = pca_consistency
        else:
            # --- DC mode: use instantaneous vector directly ---
            if magnitude_xy < 1e-3 or tracking_strength_nt < 10.0:
                return
            vector_heading = float(np.rad2deg(np.arctan2(by, bx))) % 360.0
            vector_magnitude = magnitude_xy
            # For DC mode, consistency is not PCA-based
            self.vector_consistency_score = 0.0

        self.vector_headings.append(vector_heading)
        self.vector_magnitudes.append(vector_magnitude)

        n = len(self.vector_headings)
        if n < 1:
            return

        # Circular mean of recent vector headings
        rads = np.array([np.deg2rad(h) for h in self.vector_headings])
        mean_sin = float(np.mean(np.sin(rads)))
        mean_cos = float(np.mean(np.cos(rads)))
        mean_rad = np.arctan2(mean_sin, mean_cos)
        self.magnetic_vector_heading_deg = float(np.rad2deg(mean_rad)) % 360.0

        # Cable heading is perpendicular to B_xy
        self.vector_cable_heading_deg = (self.magnetic_vector_heading_deg + 90.0) % 360.0

        # Confidence based on magnitude consistency (R < 1 → spread)
        resultant_length = float(np.sqrt(mean_sin ** 2 + mean_cos ** 2))
        self.vector_confidence = float(np.clip(resultant_length, 0.0, 1.0))


class WeightedSlidingWindowFitter:
    """结合 SNR 权重的滑动窗口拟合器，用于部署模式下的稳健中心线估计。"""

    def __init__(
        self,
        capacity: int,
        snr_floor: float,
        washout_residual_m: float = 5.0,
        washout_snr_linear_threshold: float = 10.0,
        washout_retention_count: int = 2,
    ) -> None:
        """初始化滑动窗口容量、权重下限与洗出阈值。"""
        self.capacity = max(2, capacity)
        self.snr_floor = max(snr_floor, 1.0001)
        self.washout_residual_m = max(washout_residual_m, 0.5)
        self.washout_snr_linear_threshold = max(washout_snr_linear_threshold, self.snr_floor)
        self.washout_retention_count = max(1, washout_retention_count)
        self.peak_observations: Deque[PeakObservation] = deque(maxlen=self.capacity)
        self.last_detection_time_s = -1e9

    def _fit_observations(self, observations: Deque[PeakObservation]) -> FitResult:
        """对给定观测序列执行加权直线拟合。"""
        if len(observations) < 2:
            return FitResult(origin_xy_m=None, direction_xy=None, residual_m=float("inf"), covariance_xy_m2=None)

        points = np.vstack([observation.position_xy_m for observation in observations])
        weights = np.array([np.log10(max(observation.snr_linear, self.snr_floor)) for observation in observations], dtype=float)
        weights = np.maximum(weights, 1e-3)
        weights = weights / np.sum(weights)
        centroid = np.sum(points * weights[:, None], axis=0)
        centered = points - centroid
        covariance = np.zeros((2, 2), dtype=float)
        for weight, point in zip(weights, centered):
            covariance += weight * np.outer(point, point)

        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        direction = eigenvectors[:, int(np.argmax(eigenvalues))]
        direction = direction / max(np.linalg.norm(direction), 1e-9)

        # Chronological Sign Correction: 确保特征向量与宏观时间流向一致
        oldest_point = observations[0].position_xy_m
        latest_point = observations[-1].position_xy_m
        macro_vec = latest_point - oldest_point
        if np.dot(direction, macro_vec) < 0:
            direction = -direction

        orthogonal = np.array([-direction[1], direction[0]], dtype=float)
        residual = float(np.sqrt(np.sum(weights * (centered @ orthogonal) ** 2)))
        return FitResult(origin_xy_m=centroid, direction_xy=direction, residual_m=residual, covariance_xy_m2=covariance)

    def add_peak(self, position_xy_m: np.ndarray, snr_linear: float, confidence: float, time_s: float) -> bool:
        """添加峰值观测，并在异常偏离时触发洗出处理。"""
        washout_triggered = False
        position_xy_m = np.asarray(position_xy_m, dtype=float)

        # Spatial mutual exclusion filter: prevent dense clusters from dominating PCA fit
        SPATIAL_EXCLUSION_M = 8.0
        for i, obs in enumerate(self.peak_observations):
            dist = float(np.linalg.norm(position_xy_m - obs.position_xy_m))
            if dist < SPATIAL_EXCLUSION_M:
                # New point is too close to an existing point
                if snr_linear > obs.snr_linear:
                    # Replace old point with better SNR new point
                    self.peak_observations[i] = PeakObservation(
                        position_xy_m=position_xy_m,
                        snr_linear=float(max(snr_linear, self.snr_floor)),
                        confidence=float(confidence),
                        time_s=float(time_s),
                    )
                    self.last_detection_time_s = time_s
                    return washout_triggered
                else:
                    # Old point is better, discard new point
                    self.last_detection_time_s = time_s
                    return washout_triggered

        if len(self.peak_observations) >= 2 and snr_linear >= self.washout_snr_linear_threshold:
            current_fit = self._fit_observations(self.peak_observations)
            if current_fit.direction_xy is not None and np.isfinite(current_fit.residual_m) and current_fit.origin_xy_m is not None:
                orthogonal_xy = np.array([-current_fit.direction_xy[1], current_fit.direction_xy[0]], dtype=float)
                residual_m = abs(float(np.dot(position_xy_m - current_fit.origin_xy_m, orthogonal_xy)))
                if residual_m > self.washout_residual_m:
                    retained = list(self.peak_observations)[-self.washout_retention_count :]
                    self.peak_observations = deque(retained, maxlen=self.capacity)
                    washout_triggered = True
        self.peak_observations.append(
            PeakObservation(
                position_xy_m=position_xy_m,
                snr_linear=float(max(snr_linear, self.snr_floor)),
                confidence=float(confidence),
                time_s=float(time_s),
            )
        )
        self.last_detection_time_s = time_s
        return washout_triggered

    def fit(self) -> FitResult:
        """返回当前滑动窗口中的稳健拟合结果。"""
        return self._fit_observations(self.peak_observations)


class ConfidenceEstimator:
    """将磁信号、拟合质量与声呐信息融合为统一置信度。"""

    def __init__(self, lost_timeout_s: float) -> None:
        """初始化丢失超时参数。"""
        self.lost_timeout_s = max(lost_timeout_s, 0.1)

    def magnetic_confidence(
        self,
        snr: float,
        fit_residual_m: float,
        detection_age_s: float,
        weak_signal_flag: bool,
        zigzag_width_m: float = 0.0,
        speed_mps: float = 1.0,
    ) -> float:
        """根据信噪比、拟合残差和动态检测时效评估磁感知置信度。"""
        snr_score = np.clip((snr - 1.0) / 8.0, 0.0, 1.0)
        fit_score = float(np.exp(-fit_residual_m / 10.0)) if np.isfinite(fit_residual_m) else 0.0
        
        # 动态容忍时间：横切一整个宽度所需时间
        dynamic_timeout_s = max(self.lost_timeout_s, (zigzag_width_m * 2.0) / max(speed_mps, 0.5))
        age_score = float(np.exp(-detection_age_s / dynamic_timeout_s))
        
        weak_penalty = 0.35 if weak_signal_flag else 1.0
        return float(np.clip((0.45 * snr_score + 0.35 * fit_score + 0.20 * age_score) * weak_penalty, 0.0, 1.0))

    def fused_confidence(
        self,
        magnetic_confidence: float,
        sonar_confidence: float,
        guidance_source: str,
        fit_residual_m: float = float("inf"),
        fit_covariance_xy_m2: Optional[np.ndarray] = None,
    ) -> float:
        """根据当前引导来源融合磁与声呐置信度。"""
        fit_quality = 0.0
        if np.isfinite(fit_residual_m):
            fit_quality = float(np.exp(-fit_residual_m / 8.0))
        if fit_covariance_xy_m2 is not None:
            covariance_xy_m2 = np.asarray(fit_covariance_xy_m2, dtype=float)
            if covariance_xy_m2.shape == (2, 2):
                eigenvalues = np.linalg.eigvalsh(covariance_xy_m2)
                major_axis_m = float(np.sqrt(max(float(np.max(eigenvalues)), 0.0)))
                minor_axis_m = float(np.sqrt(max(float(np.min(eigenvalues)), 0.0)))
                fit_quality = 0.5 * fit_quality + 0.5 * float(np.exp(-(major_axis_m + 0.5 * minor_axis_m) / 18.0))

        if guidance_source == "SONAR":
            confidence = sonar_confidence if magnetic_confidence <= 0.0 else 0.8 * sonar_confidence + 0.2 * magnetic_confidence
        elif guidance_source == "MAGNETIC":
            confidence = magnetic_confidence if sonar_confidence <= 0.0 else 0.8 * magnetic_confidence + 0.2 * sonar_confidence
        elif guidance_source == "MEMORY":
            confidence = 0.24 + 0.52 * magnetic_confidence + 0.10 * max(sonar_confidence, magnetic_confidence) + 0.14 * fit_quality
            confidence = min(0.82, confidence)
        elif guidance_source == "BLIND":
            confidence = min(0.4, 0.6 * magnetic_confidence + 0.4 * sonar_confidence)
        elif guidance_source == "SONAR_SEED":
            confidence = min(0.6, max(sonar_confidence * 0.8, magnetic_confidence))
        else:
            confidence = max(magnetic_confidence, sonar_confidence * 0.75)
        return float(np.clip(confidence, 0.0, 1.0))


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
