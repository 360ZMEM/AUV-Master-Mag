"""Perception layer: filtering, RMS extraction, peak detection and fitting."""

from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import numpy as np
from scipy.signal import butter, sosfilt

from .config import ScenarioConfig
from .math_utils import heading_from_direction_xy, norm, sensor_to_body, body_to_ned, rotation_matrix_sensor_to_body
from .sensor_model import BurialDepthMeasurement, MagnetometerReading, PoseMeasurement


@dataclass
class FitResult:
    origin_xy_m: Optional[np.ndarray]
    direction_xy: Optional[np.ndarray]
    residual_m: float


@dataclass
class PeakEvent:
    detected: bool
    peak_strength_nt: float = 0.0
    peak_time_s: float = 0.0


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
    confidence: float
    peak_detected: bool
    fit_result: FitResult
    line_heading_deg: Optional[float]
    estimated_burial_depth_m: Optional[float]
    true_burial_depth_m: float
    burial_measurement_valid: bool
    last_detection_age_s: float


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
    def __init__(self, min_peak_strength_nt: float, turn_trigger_ratio: float, hysteresis_fraction: float, cooldown_s: float) -> None:
        self.min_peak_strength_nt = min_peak_strength_nt
        self.turn_trigger_ratio = turn_trigger_ratio
        self.hysteresis_fraction = hysteresis_fraction
        self.cooldown_s = cooldown_s
        self.current_peak_strength_nt = 0.0
        self.current_peak_time_s = -1e9
        self.cooldown_until_s = -1e9

    def update(self, strength_nt: float, time_s: float) -> PeakEvent:
        if time_s < self.cooldown_until_s:
            return PeakEvent(detected=False)

        if strength_nt >= self.current_peak_strength_nt:
            self.current_peak_strength_nt = strength_nt
            self.current_peak_time_s = time_s
            return PeakEvent(detected=False)

        drop_nt = self.current_peak_strength_nt - strength_nt
        ratio_triggered = strength_nt <= self.current_peak_strength_nt * self.turn_trigger_ratio
        hysteresis_triggered = drop_nt >= self.current_peak_strength_nt * self.hysteresis_fraction
        if (
            self.current_peak_strength_nt >= self.min_peak_strength_nt
            and ratio_triggered
            and hysteresis_triggered
        ):
            event = PeakEvent(
                detected=True,
                peak_strength_nt=self.current_peak_strength_nt,
                peak_time_s=self.current_peak_time_s,
            )
            self.current_peak_strength_nt = 0.0
            self.current_peak_time_s = time_s
            self.cooldown_until_s = time_s + self.cooldown_s
            return event
        return PeakEvent(detected=False)


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
            return FitResult(origin_xy_m=None, direction_xy=None, residual_m=float("inf"))

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
        return FitResult(origin_xy_m=centroid, direction_xy=direction, residual_m=residual)


class ConfidenceEstimator:
    def __init__(self, lost_timeout_s: float) -> None:
        self.lost_timeout_s = max(lost_timeout_s, 0.1)

    def update(
        self,
        snr: float,
        fit_residual_m: float,
        detection_age_s: float,
        burial_valid: bool,
        has_detection_history: bool,
    ) -> float:
        snr_score = np.clip((snr - 1.0) / 8.0, 0.0, 1.0)
        fit_score = float(np.exp(-fit_residual_m / 10.0)) if np.isfinite(fit_residual_m) else 0.0
        age_score = float(np.exp(-detection_age_s / self.lost_timeout_s))
        burial_score = 1.0 if burial_valid else 0.35
        confidence = 0.35 * snr_score + 0.30 * fit_score + 0.20 * age_score + 0.15 * burial_score
        if not has_detection_history:
            confidence = min(confidence, 0.25)
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
                sample_rate_hz=scenario.sensor.magnetometer_sample_rate_hz,
                center_frequency_hz=scenario.signal.frequency_hz,
                half_width_hz=scenario.signal.bandpass_half_width_hz,
            )
        self.peak_detector = PeakDetector(
            min_peak_strength_nt=scenario.tracking.min_peak_strength_nt,
            turn_trigger_ratio=scenario.tracking.turn_trigger_ratio,
            hysteresis_fraction=scenario.tracking.hysteresis_fraction,
            cooldown_s=scenario.tracking.peak_cooldown_s,
        )
        self.fitter = CableRouteFitter(
            history_size=scenario.tracking.fit_history_size,
            forgetting_factor=scenario.tracking.forgetting_factor,
        )
        self.confidence_estimator = ConfidenceEstimator(scenario.tracking.lost_timeout_s)
        self.last_time_s = 0.0

    def update(
        self,
        reading: MagnetometerReading,
        pose_measurement: PoseMeasurement,
        vehicle_position_xy_m: np.ndarray,
        burial_measurement: BurialDepthMeasurement,
        true_burial_depth_m: float,
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

        for sensor_sample_nt in sample_block_sensor_nt:
            body_field_nt = sensor_to_body(sensor_sample_nt, self.sensor_to_body_matrix)
            ned_field_nt = body_to_ned(
                body_field_nt,
                pose_measurement.roll_deg,
                pose_measurement.pitch_deg,
                pose_measurement.heading_deg,
            )
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

        peak_event = self.peak_detector.update(tracking_strength_nt, reading.time_s)
        if peak_event.detected:
            self.fitter.add_peak(vehicle_position_xy_m, reading.time_s)

        fit_result = self.fitter.fit()
        line_heading_deg = None
        if fit_result.direction_xy is not None:
            line_heading_deg = heading_from_direction_xy(fit_result.direction_xy)

        snr = tracking_strength_nt / max(noise_floor_nt, 1e-6)
        detection_age_s = reading.time_s - self.fitter.last_detection_time_s
        has_detection_history = self.fitter.last_detection_time_s > -1e8
        confidence = self.confidence_estimator.update(
            snr=snr,
            fit_residual_m=fit_result.residual_m,
            detection_age_s=detection_age_s,
            burial_valid=burial_measurement.valid,
            has_detection_history=has_detection_history,
        )

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
            confidence=confidence,
            peak_detected=peak_event.detected,
            fit_result=fit_result,
            line_heading_deg=line_heading_deg,
            estimated_burial_depth_m=burial_measurement.depth_m,
            true_burial_depth_m=true_burial_depth_m,
            burial_measurement_valid=burial_measurement.valid,
            last_detection_age_s=detection_age_s,
        )
