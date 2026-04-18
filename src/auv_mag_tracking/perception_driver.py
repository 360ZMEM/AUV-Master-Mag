"""Signal-processing adapter that converts raw magnetometer waveforms into semantic features."""

from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import numpy as np
from scipy.signal import butter, sosfilt

from .config import ScenarioConfig
from .sensor_model import MagnetometerReading


@dataclass
class ProcessedSignalFeatures:
    time_s: float
    processed_intensity_nt: float
    filtered_intensity_nt: float
    target_magnitude_nt: float
    noise_floor_nt: float
    snr_linear: float
    snr_db: float
    is_ac_detected: bool
    dominant_frequency_hz: float
    target_frequency_hz: float
    reliability_flag: bool
    weak_signal_flag: bool
    frequency_error_hz: float
    peak_prominence_db: float
    sample_rate_hz: float
    bit_depth: int
    clipping_ratio: float
    representative_field_nt: np.ndarray
    processing_sample_rate_hz: float
    signal_method: str


@dataclass
class SignalDiagnostics:
    time_s: float
    relative_time_s: np.ndarray
    raw_time_window_nt: np.ndarray
    filtered_time_window_nt: np.ndarray
    processed_amplitude_window_nt: np.ndarray
    dc_component_nt: float
    frequency_axis_hz: np.ndarray
    psd_nt2: np.ndarray
    detected_peak_frequency_hz: Optional[float]
    detected_peak_magnitude_nt: float


@dataclass
class PerceptionDriverFrame:
    features: ProcessedSignalFeatures
    diagnostics: SignalDiagnostics


class ScalarStreamingBandpassFilter:
    def __init__(self, sample_rate_hz: float, center_frequency_hz: float, half_width_hz: float, order: int = 2) -> None:
        nyquist_hz = 0.5 * max(sample_rate_hz, 1e-6)
        low_hz = max(0.5, center_frequency_hz - half_width_hz)
        high_hz = min(nyquist_hz * 0.95, center_frequency_hz + half_width_hz)
        if low_hz >= high_hz:
            high_hz = min(nyquist_hz * 0.95, low_hz + max(1.0, 0.1 * center_frequency_hz))
        low_normalized = max(1e-4, low_hz / nyquist_hz)
        high_normalized = min(0.999, high_hz / nyquist_hz)
        self.sos = butter(order, [low_normalized, high_normalized], btype="bandpass", output="sos")
        self.zi = np.zeros((self.sos.shape[0], 2), dtype=float)

    def process_block(self, samples_nt: np.ndarray) -> np.ndarray:
        if samples_nt.size == 0:
            return np.zeros(0, dtype=float)
        filtered_nt, self.zi = sosfilt(self.sos, np.asarray(samples_nt, dtype=float), zi=self.zi)
        return filtered_nt


class SlidingWindowRMS:
    def __init__(self, window_size_samples: int) -> None:
        self.window_size_samples = max(1, window_size_samples)
        self.buffer: Deque[float] = deque(maxlen=self.window_size_samples)
        self.sum_squares = 0.0

    def update(self, sample_value: float) -> float:
        if len(self.buffer) == self.buffer.maxlen:
            oldest_value = self.buffer[0]
            self.sum_squares -= oldest_value * oldest_value
        self.buffer.append(float(sample_value))
        self.sum_squares += float(sample_value) * float(sample_value)
        sample_count = max(len(self.buffer), 1)
        return float(np.sqrt(max(self.sum_squares, 0.0) / sample_count))


class SlidingLockInDemodulator:
    def __init__(self, frequency_hz: float, window_size_samples: int) -> None:
        self.frequency_hz = float(max(frequency_hz, 1e-6))
        self.window_size_samples = max(1, window_size_samples)
        self.i_buffer: Deque[float] = deque(maxlen=self.window_size_samples)
        self.q_buffer: Deque[float] = deque(maxlen=self.window_size_samples)
        self.i_sum = 0.0
        self.q_sum = 0.0

    def update(self, sample_value: float, time_s: float) -> float:
        phase_rad = 2.0 * np.pi * self.frequency_hz * float(time_s)
        i_sample = float(sample_value) * np.sin(phase_rad)
        q_sample = float(sample_value) * np.cos(phase_rad)
        if len(self.i_buffer) == self.i_buffer.maxlen:
            self.i_sum -= self.i_buffer[0]
            self.q_sum -= self.q_buffer[0]
        self.i_buffer.append(i_sample)
        self.q_buffer.append(q_sample)
        self.i_sum += i_sample
        self.q_sum += q_sample
        sample_count = max(len(self.i_buffer), 1)
        mean_i = self.i_sum / sample_count
        mean_q = self.q_sum / sample_count
        return float(2.0 * np.sqrt(mean_i * mean_i + mean_q * mean_q))


class PerceptionDriver:
    def __init__(self, scenario: ScenarioConfig) -> None:
        self.scenario = scenario
        self.config = scenario.signal_processing
        sensor_rate_hz = scenario.sensor.high_fidelity.sampling_rate_hz if scenario.sensor.high_fidelity.enabled else scenario.sensor.magnetometer_sample_rate_hz
        self.input_sample_rate_hz = float(sensor_rate_hz)
        self.processing_sample_rate_hz = float(self.input_sample_rate_hz)
        if (
            self.scenario.signal.mode != "dc"
            and self.config.enable_interpolation
            and self.input_sample_rate_hz <= self.config.interpolation_input_rate_threshold_hz
        ):
            self.processing_sample_rate_hz = float(max(self.config.interpolation_target_rate_hz, self.input_sample_rate_hz))
        self.processing_dt_s = 1.0 / max(self.processing_sample_rate_hz, 1e-6)
        self.window_size = max(16, int(round(self.config.window_size * self.processing_sample_rate_hz / max(self.input_sample_rate_hz, 1e-6))))
        self.max_buffer_samples = max(self.window_size * 4, int(np.ceil(self.processing_sample_rate_hz * 2.0)))
        self.raw_signal_buffer: Deque[float] = deque(maxlen=self.max_buffer_samples)
        self.filtered_signal_buffer: Deque[float] = deque(maxlen=self.max_buffer_samples)
        self.processed_amplitude_buffer: Deque[float] = deque(maxlen=self.max_buffer_samples)
        self.time_buffer: Deque[float] = deque(maxlen=self.max_buffer_samples)
        self.last_frame: Optional[PerceptionDriverFrame] = None
        self.window_cache = {}
        self.frequency_axis_cache = {}
        self.spectral_constant_cache = {}
        self.last_scalar_sample: Optional[Tuple[float, float]] = None
        self.bandpass_filter = None
        if self.scenario.signal.mode != "dc":
            self.bandpass_filter = ScalarStreamingBandpassFilter(
                sample_rate_hz=self.processing_sample_rate_hz,
                center_frequency_hz=self.scenario.signal.frequency_hz,
                half_width_hz=self.scenario.signal.bandpass_half_width_hz,
                order=self.config.bandpass_order,
            )
        rms_window_size = self._cycle_window_size(self.config.rms_cycle_count)
        self.rms_extractor = SlidingWindowRMS(rms_window_size)
        self.lockin_demodulator = None
        if self.scenario.signal.mode != "dc" and self.config.lockin_enabled:
            self.lockin_demodulator = SlidingLockInDemodulator(
                frequency_hz=self.scenario.signal.frequency_hz,
                window_size_samples=self._cycle_window_size(self.config.lockin_cycle_count),
            )

    def _cycle_window_size(self, cycle_count: int) -> int:
        period_s = 1.0 / max(self.scenario.signal.frequency_hz, 1e-6)
        return max(1, int(round(max(cycle_count, 1) * period_s * self.processing_sample_rate_hz)))

    def _window_values(self, sample_count: int) -> np.ndarray:
        if sample_count in self.window_cache:
            return self.window_cache[sample_count]
        if sample_count <= 1:
            window = np.ones(sample_count, dtype=float)
        elif self.config.window_function.lower() in {"hanning", "hann"}:
            window = np.hanning(sample_count)
        elif self.config.window_function.lower() == "hamming":
            window = np.hamming(sample_count)
        else:
            window = np.ones(sample_count, dtype=float)
        self.window_cache[sample_count] = window
        return window

    def _frequency_axis_hz(self, sample_count: int) -> np.ndarray:
        if sample_count not in self.frequency_axis_cache:
            self.frequency_axis_cache[sample_count] = np.fft.rfftfreq(sample_count, d=1.0 / max(self.processing_sample_rate_hz, 1e-6))
        return self.frequency_axis_cache[sample_count]

    def _spectral_constants(self, sample_count: int) -> Tuple[float, float]:
        if sample_count not in self.spectral_constant_cache:
            window_values = self._window_values(sample_count)
            self.spectral_constant_cache[sample_count] = (
                float(np.sum(window_values)),
                float(np.sum(window_values**2)),
            )
        return self.spectral_constant_cache[sample_count]

    def _extract_scalar_waveform(self, reading: MagnetometerReading) -> Tuple[np.ndarray, float]:
        raw_block_nt = reading.quantized_sensor_block_nt if reading.quantized_sensor_block_nt is not None else reading.sample_block_sensor_nt
        raw_block_nt = np.asarray(raw_block_nt, dtype=float)
        dc_reference_sensor_nt = reading.dc_reference_sensor_nt
        if dc_reference_sensor_nt is None:
            dc_reference_sensor_nt = np.mean(raw_block_nt, axis=0)
        dc_reference_sensor_nt = np.asarray(dc_reference_sensor_nt, dtype=float)
        anomaly_block_nt = raw_block_nt - dc_reference_sensor_nt.reshape(1, 3)

        axis_mode = self.config.axis_combination_mode.lower()
        if axis_mode == "vector_norm":
            scalar_signal_nt = np.linalg.norm(anomaly_block_nt, axis=1)
        else:
            dominant_axis = int(np.argmax(np.var(anomaly_block_nt, axis=0)))
            scalar_signal_nt = anomaly_block_nt[:, dominant_axis]
        dc_scalar_nt = float(np.mean(dc_reference_sensor_nt))
        return scalar_signal_nt.astype(float), dc_scalar_nt

    def _maybe_resample_block(self, sample_times_s: np.ndarray, scalar_signal_nt: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        sample_times_s = np.asarray(sample_times_s, dtype=float)
        scalar_signal_nt = np.asarray(scalar_signal_nt, dtype=float)
        if sample_times_s.size == 0:
            return sample_times_s, scalar_signal_nt
        if self.processing_sample_rate_hz <= self.input_sample_rate_hz + 1e-9:
            self.last_scalar_sample = (float(sample_times_s[-1]), float(scalar_signal_nt[-1]))
            return sample_times_s, scalar_signal_nt

        if self.last_scalar_sample is not None:
            previous_time_s, previous_value_nt = self.last_scalar_sample
            combined_times_s = np.concatenate([[previous_time_s], sample_times_s])
            combined_values_nt = np.concatenate([[previous_value_nt], scalar_signal_nt])
            start_time_s = previous_time_s + self.processing_dt_s
        else:
            combined_times_s = sample_times_s
            combined_values_nt = scalar_signal_nt
            start_time_s = float(sample_times_s[0])

        end_time_s = float(sample_times_s[-1])
        if end_time_s < start_time_s - 1e-12:
            self.last_scalar_sample = (float(sample_times_s[-1]), float(scalar_signal_nt[-1]))
            return np.zeros(0, dtype=float), np.zeros(0, dtype=float)
        resampled_times_s = np.arange(start_time_s, end_time_s + 0.5 * self.processing_dt_s, self.processing_dt_s)
        resampled_values_nt = np.interp(resampled_times_s, combined_times_s, combined_values_nt)
        self.last_scalar_sample = (float(sample_times_s[-1]), float(scalar_signal_nt[-1]))
        return resampled_times_s, resampled_values_nt

    def _append_processing_buffers(self, time_block_s: np.ndarray, raw_block_nt: np.ndarray, filtered_block_nt: np.ndarray, amplitude_block_nt: np.ndarray) -> None:
        for time_s, raw_nt, filtered_nt, amplitude_nt in zip(time_block_s, raw_block_nt, filtered_block_nt, amplitude_block_nt):
            self.time_buffer.append(float(time_s))
            self.raw_signal_buffer.append(float(raw_nt))
            self.filtered_signal_buffer.append(float(filtered_nt))
            self.processed_amplitude_buffer.append(float(amplitude_nt))

    def _window_arrays(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        window_size = min(self.window_size, len(self.time_buffer))
        time_window_s = np.asarray(list(self.time_buffer)[-window_size:], dtype=float)
        raw_window_nt = np.asarray(list(self.raw_signal_buffer)[-window_size:], dtype=float)
        filtered_window_nt = np.asarray(list(self.filtered_signal_buffer)[-window_size:], dtype=float)
        amplitude_window_nt = np.asarray(list(self.processed_amplitude_buffer)[-window_size:], dtype=float)
        return time_window_s, raw_window_nt, filtered_window_nt, amplitude_window_nt

    def _fft_diagnostics(
        self,
        raw_window_nt: np.ndarray,
        target_frequency_hz: float,
    ) -> Tuple[np.ndarray, np.ndarray, Optional[float], float, float, float, float, float]:
        if raw_window_nt.size == 0 or not self.config.diagnostics_use_fft:
            return (
                np.zeros(1, dtype=float),
                np.zeros(1, dtype=float),
                None,
                0.0,
                max(self.scenario.sensor.noise_std_nt, 1e-6),
                0.0,
                -120.0,
                -120.0,
            )

        sample_count = raw_window_nt.size
        centered_window_nt = raw_window_nt - np.mean(raw_window_nt)
        window_values = self._window_values(sample_count)
        window_sum, window_energy = self._spectral_constants(sample_count)
        spectrum = np.fft.rfft(centered_window_nt * window_values)
        amplitude_spectrum_nt = 2.0 * np.abs(spectrum) / max(window_sum, 1e-6)
        psd_nt2 = (np.abs(spectrum) ** 2) / max(window_energy, 1e-6)
        frequency_axis_hz = self._frequency_axis_hz(sample_count)
        non_dc_mask = frequency_axis_hz >= self.config.min_ac_frequency_hz
        if not np.any(non_dc_mask):
            return (
                frequency_axis_hz,
                psd_nt2,
                None,
                0.0,
                max(self.scenario.sensor.noise_std_nt, 1e-6),
                0.0,
                -120.0,
                -120.0,
            )

        target_band_mask = np.abs(frequency_axis_hz - target_frequency_hz) <= self.config.target_frequency_tolerance_hz
        target_band_mask &= non_dc_mask
        dominant_index = np.flatnonzero(non_dc_mask)[int(np.argmax(amplitude_spectrum_nt[non_dc_mask]))]
        dominant_frequency_hz = float(frequency_axis_hz[dominant_index])
        detected_peak_frequency_hz = dominant_frequency_hz
        if np.any(target_band_mask) and self.config.use_centroid_frequency_estimation:
            centroid_weights = np.maximum(psd_nt2[target_band_mask], 1e-12)
            detected_peak_frequency_hz = float(np.sum(frequency_axis_hz[target_band_mask] * centroid_weights) / np.sum(centroid_weights))
            dominant_frequency_hz = detected_peak_frequency_hz
        detected_peak_magnitude_nt = float(np.max(amplitude_spectrum_nt[target_band_mask])) if np.any(target_band_mask) else float(amplitude_spectrum_nt[dominant_index])
        noise_mask = non_dc_mask & ~target_band_mask
        noise_floor_nt = float(np.median(amplitude_spectrum_nt[noise_mask])) if np.any(noise_mask) else max(self.scenario.sensor.noise_std_nt, 1e-6)
        snr_linear = detected_peak_magnitude_nt / max(noise_floor_nt, 1e-6)
        snr_db = 20.0 * np.log10(max(snr_linear, 1e-6))
        peak_prominence_db = 20.0 * np.log10(max(detected_peak_magnitude_nt, 1e-6) / max(noise_floor_nt, 1e-6))
        return (
            frequency_axis_hz,
            psd_nt2,
            detected_peak_frequency_hz,
            detected_peak_magnitude_nt,
            noise_floor_nt,
            snr_linear,
            snr_db,
            peak_prominence_db,
        )

    def _update_dc(self, reading: MagnetometerReading, scalar_signal_nt: np.ndarray, dc_scalar_nt: float) -> PerceptionDriverFrame:
        sample_times_s = np.asarray(reading.sample_times_s, dtype=float)
        for time_s, sample_nt in zip(sample_times_s, scalar_signal_nt):
            self.time_buffer.append(float(time_s))
            self.raw_signal_buffer.append(float(sample_nt))
            self.filtered_signal_buffer.append(float(sample_nt))
            self.processed_amplitude_buffer.append(float(abs(sample_nt)))

        time_window_s, raw_window_nt, filtered_window_nt, amplitude_window_nt = self._window_arrays()
        relative_time_s = time_window_s - time_window_s[-1] if time_window_s.size else np.zeros(0, dtype=float)
        processed_intensity_nt = float(np.mean(amplitude_window_nt)) if amplitude_window_nt.size else 0.0
        features = ProcessedSignalFeatures(
            time_s=reading.time_s,
            processed_intensity_nt=processed_intensity_nt,
            filtered_intensity_nt=processed_intensity_nt,
            target_magnitude_nt=processed_intensity_nt,
            noise_floor_nt=max(self.scenario.sensor.noise_std_nt, 1e-6),
            snr_linear=0.0,
            snr_db=-120.0,
            is_ac_detected=False,
            dominant_frequency_hz=0.0,
            target_frequency_hz=0.0,
            reliability_flag=processed_intensity_nt >= 0.5 * self.scenario.sensor.weak_signal_threshold_nt,
            weak_signal_flag=processed_intensity_nt < self.scenario.sensor.weak_signal_threshold_nt,
            frequency_error_hz=0.0,
            peak_prominence_db=-120.0,
            sample_rate_hz=reading.sample_rate_hz,
            bit_depth=reading.bit_depth,
            clipping_ratio=reading.clipping_ratio,
            representative_field_nt=reading.sensor_field_nt.copy(),
            processing_sample_rate_hz=self.processing_sample_rate_hz,
            signal_method="dc_envelope",
        )
        diagnostics = SignalDiagnostics(
            time_s=reading.time_s,
            relative_time_s=relative_time_s,
            raw_time_window_nt=raw_window_nt,
            filtered_time_window_nt=filtered_window_nt,
            processed_amplitude_window_nt=amplitude_window_nt,
            dc_component_nt=dc_scalar_nt,
            frequency_axis_hz=np.zeros(1, dtype=float),
            psd_nt2=np.zeros(1, dtype=float),
            detected_peak_frequency_hz=None,
            detected_peak_magnitude_nt=processed_intensity_nt,
        )
        frame = PerceptionDriverFrame(features=features, diagnostics=diagnostics)
        self.last_frame = frame
        return frame

    def update(self, reading: MagnetometerReading) -> PerceptionDriverFrame:
        scalar_signal_nt, dc_scalar_nt = self._extract_scalar_waveform(reading)
        if self.scenario.signal.mode == "dc":
            return self._update_dc(reading, scalar_signal_nt, dc_scalar_nt)

        sample_times_s = np.asarray(reading.sample_times_s, dtype=float)
        processing_times_s, processing_signal_nt = self._maybe_resample_block(sample_times_s, scalar_signal_nt)
        if processing_signal_nt.size == 0:
            processing_times_s = sample_times_s
            processing_signal_nt = scalar_signal_nt
        filtered_block_nt = self.bandpass_filter.process_block(processing_signal_nt) if self.bandpass_filter is not None else processing_signal_nt.copy()
        amplitude_block_nt = np.zeros_like(filtered_block_nt)
        for sample_index, (time_s, filtered_nt) in enumerate(zip(processing_times_s, filtered_block_nt)):
            rms_value_nt = self.rms_extractor.update(filtered_nt)
            if self.lockin_demodulator is not None:
                lockin_value_nt = self.lockin_demodulator.update(filtered_nt, time_s)
                amplitude_block_nt[sample_index] = lockin_value_nt if lockin_value_nt > 0.0 else rms_value_nt
            else:
                amplitude_block_nt[sample_index] = rms_value_nt

        self._append_processing_buffers(processing_times_s, processing_signal_nt, filtered_block_nt, amplitude_block_nt)
        time_window_s, raw_window_nt, filtered_window_nt, amplitude_window_nt = self._window_arrays()
        relative_time_s = time_window_s - time_window_s[-1] if time_window_s.size else np.zeros(0, dtype=float)

        (
            frequency_axis_hz,
            psd_nt2,
            detected_peak_frequency_hz,
            detected_peak_magnitude_nt,
            diagnostic_noise_floor_nt,
            diagnostic_snr_linear,
            diagnostic_snr_db,
            peak_prominence_db,
        ) = self._fft_diagnostics(raw_window_nt, self.scenario.signal.frequency_hz)

        processed_intensity_nt = float(amplitude_window_nt[-1]) if amplitude_window_nt.size else 0.0
        filtered_intensity_nt = float(np.sqrt(np.mean(filtered_window_nt**2))) if filtered_window_nt.size else 0.0
        target_magnitude_nt = processed_intensity_nt
        dominant_frequency_hz = detected_peak_frequency_hz if detected_peak_frequency_hz is not None else 0.0
        noise_floor_nt = diagnostic_noise_floor_nt
        snr_linear = target_magnitude_nt / max(noise_floor_nt, 1e-6)
        snr_db = 20.0 * np.log10(max(snr_linear, 1e-6))
        is_ac_detected = (
            dominant_frequency_hz >= self.config.min_ac_frequency_hz
            and abs(dominant_frequency_hz - self.scenario.signal.frequency_hz) <= self.config.target_frequency_tolerance_hz
            and target_magnitude_nt >= self.scenario.sensor.weak_signal_threshold_nt * 0.35
        )
        reliability_flag = snr_db >= self.config.snr_detection_threshold_db
        weak_signal_flag = target_magnitude_nt < self.scenario.sensor.weak_signal_threshold_nt or not reliability_flag
        frequency_error_hz = dominant_frequency_hz - self.scenario.signal.frequency_hz if dominant_frequency_hz > 0.0 else 0.0

        features = ProcessedSignalFeatures(
            time_s=reading.time_s,
            processed_intensity_nt=processed_intensity_nt,
            filtered_intensity_nt=filtered_intensity_nt,
            target_magnitude_nt=target_magnitude_nt,
            noise_floor_nt=noise_floor_nt,
            snr_linear=snr_linear,
            snr_db=snr_db,
            is_ac_detected=is_ac_detected,
            dominant_frequency_hz=dominant_frequency_hz,
            target_frequency_hz=self.scenario.signal.frequency_hz,
            reliability_flag=reliability_flag,
            weak_signal_flag=weak_signal_flag,
            frequency_error_hz=frequency_error_hz,
            peak_prominence_db=peak_prominence_db if np.isfinite(peak_prominence_db) else diagnostic_snr_db,
            sample_rate_hz=reading.sample_rate_hz,
            bit_depth=reading.bit_depth,
            clipping_ratio=reading.clipping_ratio,
            representative_field_nt=reading.sensor_field_nt.copy(),
            processing_sample_rate_hz=self.processing_sample_rate_hz,
            signal_method="lockin" if self.lockin_demodulator is not None else "sliding_rms",
        )
        diagnostics = SignalDiagnostics(
            time_s=reading.time_s,
            relative_time_s=relative_time_s,
            raw_time_window_nt=raw_window_nt,
            filtered_time_window_nt=filtered_window_nt,
            processed_amplitude_window_nt=amplitude_window_nt,
            dc_component_nt=dc_scalar_nt,
            frequency_axis_hz=frequency_axis_hz,
            psd_nt2=psd_nt2,
            detected_peak_frequency_hz=detected_peak_frequency_hz,
            detected_peak_magnitude_nt=detected_peak_magnitude_nt,
        )
        frame = PerceptionDriverFrame(features=features, diagnostics=diagnostics)
        self.last_frame = frame
        return frame