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
    origin_xy_m: Optional[np.ndarray]
    direction_xy: Optional[np.ndarray]
    residual_m: float
    covariance_xy_m2: Optional[np.ndarray] = None


@dataclass
class PeakEvent:
    detected: bool
    peak_strength_nt: float = 0.0
    peak_time_s: float = 0.0
    peak_position_xy_m: Optional[np.ndarray] = None
    estimated_cable_heading_deg: Optional[float] = None


@dataclass
class PeakObservation:
    position_xy_m: np.ndarray
    snr_linear: float
    confidence: float
    time_s: float


@dataclass
class PeakZoneSample:
    time_s: float
    strength_nt: float
    position_xy_m: Optional[np.ndarray]


@dataclass
class PerceptionState:
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
    gradient_heading_offset_deg: float = 0.0
    # --- Signal enhancement layer outputs ---
    envelope_gradient_nT_per_m: float = 0.0
    envelope_gradient_heading_deg: Optional[float] = None
    magnetic_vector_heading_deg: Optional[float] = None
    vector_cable_heading_deg: Optional[float] = None
    # --- Safe-lock diagnostics ---
    safe_lock_criterion_a_active: bool = False
    safe_lock_criterion_b_active: bool = False
    safe_lock_fit_invalidated: bool = False
    last_valid_peak_strength_nt: float = 0.0
    displacement_since_last_peak_m: float = 0.0


class LowPassFilter:
    def __init__(self, time_constant_s: float) -> None:
        self.time_constant_s = max(time_constant_s, 1e-3)
        self.value = 0.0
        self.initialized = False

    def update(self, measurement: float, dt_s: float) -> float:
        alpha = dt_s / (self.time_constant_s + dt_s)
        if not self.initialized:
            self.value = measurement
            self.initialized = True
        else:
            self.value = (1.0 - alpha) * self.value + alpha * measurement
        return self.value


class MedianWindowFilter:
    def __init__(self, window_size: int) -> None:
        self.buffer: Deque[float] = deque(maxlen=max(1, window_size))

    def update(self, measurement: float) -> float:
        self.buffer.append(measurement)
        return float(np.median(np.asarray(self.buffer, dtype=float)))


class StreamingBandpassFilter:
    def __init__(self, sample_rate_hz: float, center_frequency_hz: float, half_width_hz: float, order: int = 2) -> None:
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
    def __init__(self, sample_rate_hz: float, minimum_frequency_hz: float) -> None:
        min_window_s = 2.0 / max(minimum_frequency_hz, 1e-6)
        self.window_size_samples = max(3, int(np.ceil(sample_rate_hz * min_window_s)))
        self.buffer: Deque[float] = deque(maxlen=self.window_size_samples)

    def update(self, sample_value: float) -> float:
        self.buffer.append(sample_value)
        if not self.buffer:
            return 0.0
        values = np.asarray(self.buffer, dtype=float)
        return float(np.sqrt(np.mean(values**2)))


class PeakDetector:
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

    def _reset_tracking_state(self) -> None:
        self.state = "IDLE"
        self.ascending_count = 0
        self.descending_count = 0
        self.current_peak_strength_nt = 0.0
        self.current_peak_time_s = self.previous_time_s if self.previous_time_s is not None else -1e9
        self.current_peak_position_xy_m = None
        self.peak_zone_samples.clear()

    def _update_current_peak(self, sample: PeakZoneSample) -> None:
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
        sample = PeakZoneSample(
            time_s=float(time_s),
            strength_nt=float(strength_nt),
            position_xy_m=None if position_xy_m is None else np.asarray(position_xy_m, dtype=float).copy(),
        )
        self.recent_samples.append(sample)

        if self.previous_strength_nt is None or self.previous_time_s is None:
            self.previous_strength_nt = sample.strength_nt
            self.previous_time_s = sample.time_s
            self.current_peak_strength_nt = sample.strength_nt
            self.current_peak_time_s = sample.time_s
            self.current_peak_position_xy_m = None if sample.position_xy_m is None else sample.position_xy_m.copy()
            return PeakEvent(detected=False)

        delta_time_s = max(sample.time_s - self.previous_time_s, 1e-6)
        previous_db = 20.0 * np.log10(max(self.previous_strength_nt, 1e-6))
        current_db = 20.0 * np.log10(max(sample.strength_nt, 1e-6))
        slope_db_per_s = (current_db - previous_db) / delta_time_s
        is_ascending = slope_db_per_s > 0.1
        is_descending = slope_db_per_s < -0.1

        if time_s < self.cooldown_until_s:
            self.previous_strength_nt = sample.strength_nt
            self.previous_time_s = sample.time_s
            return PeakEvent(detected=False)

        if is_ascending:
            self.ascending_count += 1
            self.descending_count = 0
        elif is_descending:
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
            if not is_ascending:
                self.state = "PEAK_ZONE"
                self.descending_count = 1 if is_descending else 0
        elif self.state == "PEAK_ZONE":
            self.peak_zone_samples.append(sample)
            self._update_current_peak(sample)
            if is_descending and self.descending_count >= self.descending_min_samples:
                event = self._emit_peak_event(
                    sample.time_s,
                    vehicle_heading_deg=vehicle_heading_deg,
                    use_nominal_route_prior=use_nominal_route_prior,
                )
                self.previous_strength_nt = sample.strength_nt
                self.previous_time_s = sample.time_s
                return event
            if is_ascending:
                self.descending_count = 0

        self.previous_strength_nt = sample.strength_nt
        self.previous_time_s = sample.time_s
        return PeakEvent(detected=False)


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


class MagneticVectorAnalyzer:
    """Extract horizontal magnetic vector direction and infer cable heading.

    Physics constraint: at the cable crossing (peak), the horizontal
    magnetic vector B_xy = [Bx, By] is perpendicular to the cable
    direction.  Therefore cable_heading ≈ vector_heading ± 90°.
    """

    def __init__(self, buffer_capacity: int = 8) -> None:
        self.buffer_capacity = max(1, buffer_capacity)
        self.vector_headings: Deque[float] = deque(maxlen=self.buffer_capacity)
        self.vector_magnitudes: Deque[float] = deque(maxlen=self.buffer_capacity)
        self.magnetic_vector_heading_deg: Optional[float] = None
        self.vector_cable_heading_deg: Optional[float] = None
        self.vector_confidence: float = 0.0

    def update(
        self,
        anomaly_ned_nt: np.ndarray,
        tracking_strength_nt: float,
    ) -> None:
        bx, by = float(anomaly_ned_nt[0]), float(anomaly_ned_nt[1])
        magnitude_xy = float(np.sqrt(bx * bx + by * by))

        if magnitude_xy < 1e-3 or tracking_strength_nt < 10.0:
            return

        vector_heading = float(np.rad2deg(np.arctan2(by, bx))) % 360.0
        self.vector_headings.append(vector_heading)
        self.vector_magnitudes.append(magnitude_xy)

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


class CableRouteFitter:
    def __init__(self, history_size: int, forgetting_factor: float) -> None:
        self.history_size = history_size
        self.forgetting_factor = np.clip(forgetting_factor, 0.1, 0.99)
        self.peak_positions_xy: Deque[np.ndarray] = deque(maxlen=history_size)
        self.last_detection_time_s = -1e9

    def add_peak(self, position_xy_m: np.ndarray, time_s: float) -> None:
        self.peak_positions_xy.append(np.asarray(position_xy_m, dtype=float))
        self.last_detection_time_s = time_s

    def fit(self) -> FitResult:
        if len(self.peak_positions_xy) < 2:
            return FitResult(origin_xy_m=None, direction_xy=None, residual_m=float("inf"), covariance_xy_m2=None)

        points = np.vstack(self.peak_positions_xy)
        sample_count = len(points)
        weights = self.forgetting_factor ** np.arange(sample_count - 1, -1, -1)
        weights = weights / np.sum(weights)
        centroid = np.sum(points * weights[:, None], axis=0)
        centered = points - centroid
        covariance = np.zeros((2, 2), dtype=float)
        for weight, point in zip(weights, centered):
            covariance += weight * np.outer(point, point)

        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        direction = eigenvectors[:, int(np.argmax(eigenvalues))]
        direction = direction / max(np.linalg.norm(direction), 1e-9)
        orthogonal = np.array([-direction[1], direction[0]], dtype=float)
        residual = float(np.sqrt(np.sum(weights * (centered @ orthogonal) ** 2)))
        return FitResult(origin_xy_m=centroid, direction_xy=direction, residual_m=residual, covariance_xy_m2=covariance)


class WeightedSlidingWindowFitter:
    def __init__(self, capacity: int, snr_floor: float) -> None:
        self.capacity = max(2, capacity)
        self.snr_floor = max(snr_floor, 1.0001)
        self.peak_observations: Deque[PeakObservation] = deque(maxlen=self.capacity)
        self.last_detection_time_s = -1e9

    def add_peak(self, position_xy_m: np.ndarray, snr_linear: float, confidence: float, time_s: float) -> None:
        self.peak_observations.append(
            PeakObservation(
                position_xy_m=np.asarray(position_xy_m, dtype=float),
                snr_linear=float(max(snr_linear, self.snr_floor)),
                confidence=float(confidence),
                time_s=float(time_s),
            )
        )
        self.last_detection_time_s = time_s

    def fit(self) -> FitResult:
        if len(self.peak_observations) < 2:
            return FitResult(origin_xy_m=None, direction_xy=None, residual_m=float("inf"), covariance_xy_m2=None)

        points = np.vstack([observation.position_xy_m for observation in self.peak_observations])
        weights = np.array([np.log10(max(observation.snr_linear, self.snr_floor)) for observation in self.peak_observations], dtype=float)
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
        orthogonal = np.array([-direction[1], direction[0]], dtype=float)
        residual = float(np.sqrt(np.sum(weights * (centered @ orthogonal) ** 2)))
        return FitResult(origin_xy_m=centroid, direction_xy=direction, residual_m=residual, covariance_xy_m2=covariance)


class ConfidenceEstimator:
    def __init__(self, lost_timeout_s: float) -> None:
        self.lost_timeout_s = max(lost_timeout_s, 0.1)

    def magnetic_confidence(self, snr: float, fit_residual_m: float, detection_age_s: float, weak_signal_flag: bool) -> float:
        snr_score = np.clip((snr - 1.0) / 8.0, 0.0, 1.0)
        fit_score = float(np.exp(-fit_residual_m / 10.0)) if np.isfinite(fit_residual_m) else 0.0
        age_score = float(np.exp(-detection_age_s / self.lost_timeout_s))
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
        else:
            confidence = max(magnetic_confidence, sonar_confidence * 0.75)
        return float(np.clip(confidence, 0.0, 1.0))


class MagneticCablePerception:
    def __init__(self, scenario: ScenarioConfig) -> None:
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
        self.deployment_estimated_cable_heading_deg: Optional[float] = None
        self.deployment_heading_confidence: float = 0.0
        self.deployment_reacquire_required: bool = False
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
        if line_heading_deg is None:
            return blind_heading_deg, False

        fit_acceptance_residual_m = max(self.scenario.tracking.fit_acceptance_residual_m, 15.0)
        if blind_heading_deg is not None and np.isfinite(fit_result.residual_m):
            blind_line_agreement_deg = abs(smallest_angle_error_deg(line_heading_deg, blind_heading_deg))
            if fit_result.residual_m <= fit_acceptance_residual_m and blind_line_agreement_deg <= 20.0:
                return line_heading_deg, True

        if (
            blind_heading_deg is None
            and (bootstrap_fit_ready or not self.scenario.tracking.use_nominal_route_prior)
            and np.isfinite(fit_result.residual_m)
            and fit_result.residual_m <= fit_acceptance_residual_m
        ):
            return line_heading_deg, True

        return blind_heading_deg, False

    def _deployment_fit_is_consistent(self, candidate_heading_deg: Optional[float]) -> bool:
        if candidate_heading_deg is None:
            return False
        if self.deployment_estimated_cable_heading_deg is None:
            return True
        if self.deployment_heading_confidence < 0.35:
            return True
        heading_delta_deg = abs(smallest_angle_error_deg(candidate_heading_deg, self.deployment_estimated_cable_heading_deg))
        return heading_delta_deg <= self.scenario.tracking.fit_reject_heading_delta_deg

    def _deployment_gradient_heading(self) -> Optional[float]:
        gradient_heading_deg = self.envelope_tracker.gradient_heading_deg
        if gradient_heading_deg is None:
            return None
        if abs(self.envelope_tracker.gradient_nT_per_m) < 0.5:
            return None
        return gradient_heading_deg

    def _reference_heading_deg(self) -> Optional[float]:
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
        if fit_result.origin_xy_m is None or fit_result.direction_xy is None:
            return None
        return project_point_to_line(vehicle_position_xy_m, fit_result.origin_xy_m, fit_result.direction_xy)

    def _peak_cable_observation_xy_m(
        self,
        peak_position_xy_m: np.ndarray,
        sonar_reading: Optional[SonarReading],
    ) -> Optional[np.ndarray]:
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

    def _update_deployment_cable_heading(self, heading_deg: float, position_xy_m: np.ndarray) -> None:
        """Record a single crossing observation and update the consensus cable
        heading estimate for deployment (no-prior) mode.

        Strategy (Multi-Hypothesis Disambiguation)
        -----------------------------------------
        Each crossing yields *two* candidate cable headings: heading+90° and
        heading−90°.  In zigzag mode, odd-indexed crossings (right-to-left)
        and even-indexed crossings (left-to-right) have opposite vehicle
        headings, so the optimal ±90° choice also differs.

        The method stores both the raw heading deg and the crossing position.
        It then:
        1. Splits crossings into odd/even groups by sequence index.
        2. Each group independently selects +90° or -90° via spread minimization.
        3. If both groups agree on cable direction (error < 20°), merge them.
           Otherwise, trust only the newer group.
        4. Computes Bayesian confidence from count and angular consistency.
        5. Enforces a heading change rate constraint (max 10°/s).
        """
        self.crossing_headings.append((self.last_time_s, heading_deg))
        self.crossing_positions_xy.append(np.asarray(position_xy_m, dtype=float).copy())

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

        # --- Helper: circular mean and spread ---
        def _circular_stats(headings_deg: list) -> tuple:
            """Return (mean_deg, spread_deg) for a list of headings."""
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

        # --- Helper: choose best offset (+90 or -90) for a group ---
        def _choose_offset_for_group(raw_headings: list) -> tuple:
            """Return (chosen_headings, mean_deg, spread_deg, offset_used)."""
            cand_sets: list = []
            for offset in [90.0, -90.0]:
                cands = [(rh + offset) % 360.0 for rh in raw_headings]
                cand_sets.append(cands)
            spread_a = _circular_stats(cand_sets[0])[1]
            spread_b = _circular_stats(cand_sets[1])[1]
            if spread_a <= spread_b:
                return cand_sets[0], _circular_stats(cand_sets[0]), 90.0
            else:
                return cand_sets[1], _circular_stats(cand_sets[1]), -90.0

        # --- Step 1: Split into odd/even groups (zigzag-aware) ---
        raw_headings = [h for _t, h in self.crossing_headings]
        odd_headings = [raw_headings[i] for i in range(0, len(raw_headings), 2)]  # 0, 2, 4, ...
        even_headings = [raw_headings[i] for i in range(1, len(raw_headings), 2)]  # 1, 3, 5, ...

        # --- Step 2: Each group independently chooses ±90° ---
        odd_chosen, odd_stats, odd_offset = _choose_offset_for_group(odd_headings)
        even_chosen, even_stats, even_offset = _choose_offset_for_group(even_headings)
        odd_mean, odd_spread = odd_stats
        even_mean, even_spread = even_stats

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

        # --- Step 4: Heading change rate constraint ---
        # Cable heading should not change faster than 10°/s (configurable)
        max_heading_change_rate_deg_s = 10.0
        if self.deployment_estimated_cable_heading_deg is not None:
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
            self.vector_analyzer.update(anomaly_ned_nt, tracking_strength_nt)

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
                if not self.scenario.tracking.use_nominal_route_prior:
                    self._update_deployment_cable_heading(pose_measurement.heading_deg, detected_peak_xy_m)
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
                deployment_reacquire_required = True
                self.deployment_reacquire_required = True
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

        magnetic_confidence = self.confidence_estimator.magnetic_confidence(
            snr,
            fit_result.residual_m,
            detection_age_s,
            weak_signal_flag,
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
        vector_cable_heading_deg = (
            self.vector_analyzer.vector_cable_heading_deg
            if self.vector_analyzer is not None else None
        )
        if sonar_reading is not None and sonar_reading.valid and ((sonar_reading.distance_m is not None and sonar_reading.distance_m >= self.scenario.tracking.sonar_preferred_distance_m) or weak_signal_flag):
            fused_heading_deg = sonar_reading.estimated_heading_ned_deg
            if sonar_reading.estimated_position_ned_m is not None:
                estimated_cable_point_xy_m = sonar_reading.estimated_position_ned_m.copy()
            guidance_source = "SONAR"
        elif line_heading_deg is not None and not weak_signal_flag and (bootstrap_fit_ready or not self.scenario.tracking.use_nominal_route_prior):
            fused_heading_deg = line_heading_deg
            guidance_source = "MAGNETIC"
        elif (
            line_heading_deg is not None
            and (bootstrap_fit_ready or not self.scenario.tracking.use_nominal_route_prior)
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
                if fallback_heading_deg is not None:
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
            if (
                line_heading_deg is not None
                and np.isfinite(fit_result.residual_m)
                and fit_result.residual_m <= self.scenario.tracking.fit_acceptance_residual_m
            ):
                self.deployment_estimated_cable_heading_deg = line_heading_deg
                fit_confidence = float(np.clip(np.exp(-fit_result.residual_m / 6.0), 0.0, 1.0))
                self.deployment_heading_confidence = max(self.deployment_heading_confidence, fit_confidence)
            elif vector_cable_heading_deg is not None and signal_reliable and line_heading_deg is None:
                self.deployment_estimated_cable_heading_deg = vector_cable_heading_deg
                vector_confidence = self.vector_analyzer.vector_confidence if self.vector_analyzer is not None else 0.0
                self.deployment_heading_confidence = max(
                    self.deployment_heading_confidence,
                    float(np.clip(0.35 + 0.40 * vector_confidence, 0.0, 0.85)),
                )

            gradient_heading_deg = self._deployment_gradient_heading()
            if gradient_heading_deg is not None:
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

        zigzag_width_m = float(np.clip(
            self.scenario.tracking.min_zigzag_width_m + self.scenario.tracking.zigzag_width_gain_m_per_nt * tracking_strength_nt,
            self.scenario.tracking.min_zigzag_width_m,
            self.scenario.tracking.max_zigzag_width_m,
        ))
        # --- Perception-Anchored Safe-Lock (Criterion A & B) ---
        # Criterion A: signal strength collapse + excessive displacement
        self.safe_lock_criterion_a_active = False
        if (
            self.last_valid_peak_strength_nt > 0.0
            and tracking_strength_nt < self.scenario.tracking.safe_lock_strength_ratio_threshold * self.last_valid_peak_strength_nt
            and self.displacement_since_last_peak_m > self.scenario.tracking.safe_lock_displacement_factor * self.scenario.tracking.safe_lock_ideal_field_width_m
        ):
            self.safe_lock_criterion_a_active = True
            self.safe_lock_fit_invalidated = True
            # Invalidate the old fit to prevent following a stale line
            self.last_accepted_fit_result = FitResult(
                origin_xy_m=None, direction_xy=None,
                residual_m=float("inf"), covariance_xy_m2=None,
            )
            self.safe_lock_until_s = reading.time_s + 2.0

        # Criterion B: gradient direction inconsistency with fitted line normal
        self.safe_lock_criterion_b_active = False
        gradient_penalty = 0.0
        if (
            self.envelope_tracker.gradient_heading_deg is not None
            and fit_result.direction_xy is not None
            and not self.scenario.tracking.use_nominal_route_prior
        ):
            line_normal_deg = (
                float(np.rad2deg(np.arctan2(fit_result.direction_xy[0], -fit_result.direction_xy[1]))) % 360.0
            )
            gradient_angle_error = abs(smallest_angle_error_deg(
                self.envelope_tracker.gradient_heading_deg, line_normal_deg
            ))
            if gradient_angle_error > self.scenario.tracking.safe_lock_gradient_angle_threshold_deg:
                self.safe_lock_criterion_b_active = True
                gradient_penalty = self.scenario.tracking.safe_lock_gradient_confidence_penalty

        # Original safe-lock logic (peak drop detection)
        safe_lock_active = (
            detection_age_s <= recent_detection_window_s
            and reading.time_s <= self.safe_lock_until_s
        ) or self.safe_lock_criterion_a_active
        if safe_lock_active:
            zigzag_width_m = self.scenario.tracking.min_zigzag_width_m

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
        # Apply gradient inconsistency penalty from criterion B
        if gradient_penalty > 0:
            confidence = float(np.clip(confidence - gradient_penalty, 0.0, 1.0))
        self.last_output_confidence = confidence

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
            # Safe-lock diagnostics
            safe_lock_criterion_a_active=self.safe_lock_criterion_a_active,
            safe_lock_criterion_b_active=self.safe_lock_criterion_b_active,
            safe_lock_fit_invalidated=self.safe_lock_fit_invalidated,
            last_valid_peak_strength_nt=self.last_valid_peak_strength_nt,
            displacement_since_last_peak_m=self.displacement_since_last_peak_m,
            deployment_reacquire_required=deployment_reacquire_required,
        )
