"""Sensor abstractions for the sonar-magnetic cable tracking demo."""

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .config import SensorConfig, SonarConfig, SurveyConfig
from .environment import CableFitTruth
from .math_utils import Pose, body_to_sensor, ned_to_body, ned_xy_to_body, rotation_matrix_sensor_to_body, wrap_angle_deg


@dataclass
class MagnetometerReading:
    time_s: float
    sensor_field_nt: np.ndarray
    sample_times_s: np.ndarray
    sample_block_sensor_nt: np.ndarray
    cable_strength_nt: float
    weak_signal_flag: bool
    raw_sensor_block_nt: Optional[np.ndarray] = None
    quantized_sensor_block_nt: Optional[np.ndarray] = None
    dc_reference_sensor_nt: Optional[np.ndarray] = None
    clipping_ratio: float = 0.0
    sample_rate_hz: float = 0.0
    bit_depth: int = 0


@dataclass
class PoseMeasurement:
    time_s: float
    heading_deg: float
    pitch_deg: float
    roll_deg: float
    speed_mps: float = 0.0


@dataclass
class BurialDepthMeasurement:
    time_s: float
    depth_m: Optional[float]
    valid: bool


@dataclass
class SonarReading:
    time_s: float
    valid: bool
    status: str
    relative_position_body_m: Optional[np.ndarray]
    relative_heading_body_deg: Optional[float]
    estimated_position_ned_m: Optional[np.ndarray]
    estimated_heading_ned_deg: Optional[float]
    confidence: float
    distance_m: Optional[float]


class MagnetometerModel:
    def __init__(self, config: SensorConfig, random_seed: int = 7) -> None:
        self.config = config
        self.rng = np.random.default_rng(random_seed)
        self.sample_period_s = 1.0 / max(config.magnetometer_sample_rate_hz, 1e-9)
        self.sensor_to_body_matrix = rotation_matrix_sensor_to_body(*config.static_rotation_euler_deg)
        self.nonorthogonality_matrix = self._build_nonorthogonality_matrix(config.nonorthogonality_deg)
        self.bias_sensor_nt = np.zeros(3, dtype=float)
        self.last_sample_time_s = -1e9
        self.last_reading = MagnetometerReading(
            time_s=0.0,
            sensor_field_nt=np.zeros(3, dtype=float),
            sample_times_s=np.zeros(1, dtype=float),
            sample_block_sensor_nt=np.zeros((1, 3), dtype=float),
            cable_strength_nt=0.0,
            weak_signal_flag=True,
            raw_sensor_block_nt=np.zeros((1, 3), dtype=float),
            quantized_sensor_block_nt=np.zeros((1, 3), dtype=float),
            dc_reference_sensor_nt=np.zeros(3, dtype=float),
            clipping_ratio=0.0,
            sample_rate_hz=config.magnetometer_sample_rate_hz,
            bit_depth=0,
        )

    @staticmethod
    def _build_nonorthogonality_matrix(nonorthogonality_deg: float) -> np.ndarray:
        skew = np.deg2rad(nonorthogonality_deg)
        return np.array(
            [
                [1.0, np.sin(skew), -0.5 * np.sin(skew)],
                [0.0, 1.0, np.sin(skew)],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )

    def sample(
        self,
        true_field_ned_nt: np.ndarray,
        pose: Pose,
        time_s: float,
        cable_field_ned_nt: Optional[np.ndarray] = None,
    ) -> MagnetometerReading:
        if time_s - self.last_sample_time_s < self.sample_period_s:
            return self.last_reading

        delta_t = max(time_s - self.last_sample_time_s, self.sample_period_s)
        field_body_nt = ned_to_body(true_field_ned_nt, pose.roll_deg, pose.pitch_deg, pose.heading_deg)
        field_sensor_nt = body_to_sensor(field_body_nt, self.sensor_to_body_matrix)
        drift_increment = self.rng.normal(0.0, self.config.bias_drift_std_nt_per_s * np.sqrt(delta_t), size=3)
        self.bias_sensor_nt += drift_increment
        gaussian_noise = self.rng.normal(0.0, self.config.noise_std_nt, size=3)
        measured_sensor_nt = self.nonorthogonality_matrix @ field_sensor_nt
        measured_sensor_nt = measured_sensor_nt + self.bias_sensor_nt + gaussian_noise
        measured_sensor_nt = np.clip(measured_sensor_nt, -self.config.dynamic_range_nt, self.config.dynamic_range_nt)

        cable_strength_nt = 0.0 if cable_field_ned_nt is None else float(np.linalg.norm(cable_field_ned_nt))
        self.last_sample_time_s = time_s
        self.last_reading = MagnetometerReading(
            time_s=time_s,
            sensor_field_nt=measured_sensor_nt,
            sample_times_s=np.asarray([time_s], dtype=float),
            sample_block_sensor_nt=measured_sensor_nt.reshape(1, 3),
            cable_strength_nt=cable_strength_nt,
            weak_signal_flag=cable_strength_nt < self.config.weak_signal_threshold_nt,
            raw_sensor_block_nt=measured_sensor_nt.reshape(1, 3),
            quantized_sensor_block_nt=measured_sensor_nt.reshape(1, 3),
            dc_reference_sensor_nt=measured_sensor_nt.copy(),
            clipping_ratio=0.0,
            sample_rate_hz=self.config.magnetometer_sample_rate_hz,
            bit_depth=0,
        )
        return self.last_reading

    def sample_block(
        self,
        true_fields_ned_nt: np.ndarray,
        pose: Pose,
        sample_times_s: np.ndarray,
        cable_fields_ned_nt: Optional[np.ndarray] = None,
    ) -> MagnetometerReading:
        sample_times_s = np.asarray(sample_times_s, dtype=float)
        true_fields_ned_nt = np.asarray(true_fields_ned_nt, dtype=float)
        if sample_times_s.ndim != 1 or true_fields_ned_nt.shape != (sample_times_s.size, 3):
            raise ValueError("sample_times_s and true_fields_ned_nt must form an (N, 3) sample block")

        if cable_fields_ned_nt is None:
            cable_fields_ned_nt = np.zeros_like(true_fields_ned_nt)
        else:
            cable_fields_ned_nt = np.asarray(cable_fields_ned_nt, dtype=float)
            if cable_fields_ned_nt.shape != true_fields_ned_nt.shape:
                raise ValueError("cable_fields_ned_nt must match true_fields_ned_nt shape")

        block_measurements = []
        for time_s, true_field_ned_nt, cable_field_ned_nt in zip(sample_times_s, true_fields_ned_nt, cable_fields_ned_nt):
            reading = self.sample(true_field_ned_nt, pose, float(time_s), cable_field_ned_nt=cable_field_ned_nt)
            block_measurements.append(reading.sensor_field_nt.copy())

        sensor_block_nt = np.asarray(block_measurements, dtype=float)
        cable_strength_samples_nt = np.linalg.norm(cable_fields_ned_nt, axis=1)
        block_cable_strength_nt = float(np.sqrt(np.mean(cable_strength_samples_nt**2)))
        self.last_reading = MagnetometerReading(
            time_s=float(sample_times_s[-1]),
            sensor_field_nt=sensor_block_nt[-1].copy(),
            sample_times_s=sample_times_s.copy(),
            sample_block_sensor_nt=sensor_block_nt,
            cable_strength_nt=block_cable_strength_nt,
            weak_signal_flag=block_cable_strength_nt < self.config.weak_signal_threshold_nt,
            raw_sensor_block_nt=sensor_block_nt.copy(),
            quantized_sensor_block_nt=sensor_block_nt.copy(),
            dc_reference_sensor_nt=np.mean(sensor_block_nt, axis=0),
            clipping_ratio=0.0,
            sample_rate_hz=self.config.magnetometer_sample_rate_hz,
            bit_depth=0,
        )
        return self.last_reading


class HighFidelityMagnetometer(MagnetometerModel):
    def __init__(self, config: SensorConfig, random_seed: int = 31) -> None:
        super().__init__(config, random_seed=random_seed)
        self.hf_config = config.high_fidelity
        self.sample_period_s = 1.0 / max(self.hf_config.sampling_rate_hz, 1e-9)
        self.impulse_state_sensor_nt = np.zeros(3, dtype=float)

    def _generate_colored_noise_block(self, sample_count: int) -> np.ndarray:
        if sample_count <= 1 or self.hf_config.pink_noise_std_nt <= 0.0:
            return np.zeros((sample_count, 3), dtype=float)

        white_block = self.rng.normal(0.0, 1.0, size=(sample_count, 3))
        freqs_hz = np.fft.rfftfreq(sample_count, d=self.sample_period_s)
        scale = np.ones_like(freqs_hz)
        if freqs_hz.size > 1:
            scale[0] = 0.0
            exponent = max(self.hf_config.pink_noise_exponent, 0.0) * 0.5
            scale[1:] = 1.0 / np.maximum(freqs_hz[1:], 1e-6) ** exponent

        colored_block = np.zeros((sample_count, 3), dtype=float)
        for axis_index in range(3):
            spectrum = np.fft.rfft(white_block[:, axis_index])
            colored = np.fft.irfft(spectrum * scale, n=sample_count)
            colored = colored - np.mean(colored)
            colored_std = float(np.std(colored))
            if colored_std > 1e-9:
                colored = colored * (self.hf_config.pink_noise_std_nt / colored_std)
            colored_block[:, axis_index] = colored
        return colored_block

    def _generate_impulse_block(self, sample_count: int) -> np.ndarray:
        impulse_block = np.zeros((sample_count, 3), dtype=float)
        decay = float(np.exp(-1.0 / max(self.hf_config.impulse_decay_samples, 1)))
        for sample_index in range(sample_count):
            self.impulse_state_sensor_nt *= decay
            if self.rng.uniform() < self.hf_config.impulse_probability:
                direction = self.rng.normal(0.0, 1.0, size=3)
                direction_norm = float(np.linalg.norm(direction))
                if direction_norm > 1e-9:
                    direction = direction / direction_norm
                amplitude_nt = self.hf_config.impulse_amplitude_nt * (0.5 + self.rng.uniform())
                self.impulse_state_sensor_nt += amplitude_nt * direction
            impulse_block[sample_index] = self.impulse_state_sensor_nt.copy()
        return impulse_block

    def _quantize_block(self, raw_block_nt: np.ndarray) -> tuple:
        full_scale_nt = max(self.hf_config.full_scale_nt, 1.0)
        quantization_levels = max((1 << max(self.hf_config.bit_depth, 1)) - 1, 1)
        step_nt = 2.0 * full_scale_nt / quantization_levels
        clipped_block_nt = np.clip(raw_block_nt, -full_scale_nt, full_scale_nt)
        quantized_block_nt = np.round(clipped_block_nt / step_nt) * step_nt
        clipping_ratio = float(np.mean(np.any(np.abs(raw_block_nt) >= full_scale_nt, axis=1))) if raw_block_nt.size else 0.0
        return quantized_block_nt, clipping_ratio

    def sample(
        self,
        true_field_ned_nt: np.ndarray,
        pose: Pose,
        time_s: float,
        cable_field_ned_nt: Optional[np.ndarray] = None,
    ) -> MagnetometerReading:
        reading = self.sample_block(
            true_fields_ned_nt=np.asarray(true_field_ned_nt, dtype=float).reshape(1, 3),
            pose=pose,
            sample_times_s=np.asarray([time_s], dtype=float),
            cable_fields_ned_nt=None if cable_field_ned_nt is None else np.asarray(cable_field_ned_nt, dtype=float).reshape(1, 3),
        )
        return reading

    def sample_block(
        self,
        true_fields_ned_nt: np.ndarray,
        pose: Pose,
        sample_times_s: np.ndarray,
        cable_fields_ned_nt: Optional[np.ndarray] = None,
    ) -> MagnetometerReading:
        sample_times_s = np.asarray(sample_times_s, dtype=float)
        true_fields_ned_nt = np.asarray(true_fields_ned_nt, dtype=float)
        if sample_times_s.ndim != 1 or true_fields_ned_nt.shape != (sample_times_s.size, 3):
            raise ValueError("sample_times_s and true_fields_ned_nt must form an (N, 3) sample block")

        if cable_fields_ned_nt is None:
            cable_fields_ned_nt = np.zeros_like(true_fields_ned_nt)
        else:
            cable_fields_ned_nt = np.asarray(cable_fields_ned_nt, dtype=float)
            if cable_fields_ned_nt.shape != true_fields_ned_nt.shape:
                raise ValueError("cable_fields_ned_nt must match true_fields_ned_nt shape")

        sample_count = sample_times_s.size
        colored_noise_block_nt = self._generate_colored_noise_block(sample_count)
        impulse_block_nt = self._generate_impulse_block(sample_count)
        white_noise_block_nt = self.rng.normal(0.0, self.hf_config.white_noise_std_nt, size=(sample_count, 3))
        platform_interference_body_nt = np.asarray(self.hf_config.auv_static_interference_body_nt, dtype=float)

        raw_sensor_block_nt = np.zeros((sample_count, 3), dtype=float)
        dc_reference_block_nt = np.zeros((sample_count, 3), dtype=float)
        for sample_index, (time_s, true_field_ned_nt, cable_field_ned_nt) in enumerate(zip(sample_times_s, true_fields_ned_nt, cable_fields_ned_nt)):
            delta_t = max(float(time_s) - self.last_sample_time_s, self.sample_period_s)
            drift_increment = self.rng.normal(0.0, self.config.bias_drift_std_nt_per_s * np.sqrt(delta_t), size=3)
            self.bias_sensor_nt += drift_increment

            dc_field_ned_nt = true_field_ned_nt - cable_field_ned_nt
            dc_field_body_nt = ned_to_body(dc_field_ned_nt, pose.roll_deg, pose.pitch_deg, pose.heading_deg) + platform_interference_body_nt
            ac_field_body_nt = ned_to_body(cable_field_ned_nt, pose.roll_deg, pose.pitch_deg, pose.heading_deg)
            dc_sensor_nt = body_to_sensor(dc_field_body_nt, self.sensor_to_body_matrix)
            ac_sensor_nt = body_to_sensor(ac_field_body_nt, self.sensor_to_body_matrix)

            ideal_sensor_nt = self.nonorthogonality_matrix @ (dc_sensor_nt + ac_sensor_nt)
            raw_sensor_nt = ideal_sensor_nt + self.bias_sensor_nt + white_noise_block_nt[sample_index] + colored_noise_block_nt[sample_index] + impulse_block_nt[sample_index]
            raw_sensor_block_nt[sample_index] = raw_sensor_nt
            dc_reference_block_nt[sample_index] = dc_sensor_nt
            self.last_sample_time_s = float(time_s)

        quantized_block_nt, clipping_ratio = self._quantize_block(raw_sensor_block_nt)
        cable_strength_samples_nt = np.linalg.norm(cable_fields_ned_nt, axis=1)
        block_cable_strength_nt = float(np.sqrt(np.mean(cable_strength_samples_nt**2)))
        dc_reference_sensor_nt = np.mean(dc_reference_block_nt, axis=0)
        self.last_reading = MagnetometerReading(
            time_s=float(sample_times_s[-1]),
            sensor_field_nt=quantized_block_nt[-1].copy(),
            sample_times_s=sample_times_s.copy(),
            sample_block_sensor_nt=quantized_block_nt.copy(),
            cable_strength_nt=block_cable_strength_nt,
            weak_signal_flag=block_cable_strength_nt < self.config.weak_signal_threshold_nt,
            raw_sensor_block_nt=raw_sensor_block_nt.copy(),
            quantized_sensor_block_nt=quantized_block_nt.copy(),
            dc_reference_sensor_nt=dc_reference_sensor_nt,
            clipping_ratio=clipping_ratio,
            sample_rate_hz=self.hf_config.sampling_rate_hz,
            bit_depth=self.hf_config.bit_depth,
        )
        return self.last_reading


class IMUSimulator:
    def __init__(self, config: SensorConfig, random_seed: int = 13) -> None:
        self.config = config
        self.rng = np.random.default_rng(random_seed)

    def observe(self, pose: Pose, time_s: float) -> PoseMeasurement:
        heading_deg = pose.heading_deg + self.rng.normal(0.0, self.config.imu_heading_noise_deg)
        pitch_deg = pose.pitch_deg + self.rng.normal(0.0, self.config.imu_tilt_noise_deg)
        roll_deg = pose.roll_deg + self.rng.normal(0.0, self.config.imu_tilt_noise_deg)
        return PoseMeasurement(
            time_s=time_s, heading_deg=heading_deg, pitch_deg=pitch_deg,
            roll_deg=roll_deg, speed_mps=pose.speed_mps,
        )


class BurialDepthObserver:
    def __init__(self, config: SurveyConfig, random_seed: int = 19) -> None:
        self.config = config
        self.rng = np.random.default_rng(random_seed)
        self.sample_period_s = 1.0 / max(config.burial_depth_update_rate_hz, 1e-9)
        self.last_sample_time_s = -1e9
        self.last_measurement = BurialDepthMeasurement(time_s=0.0, depth_m=None, valid=False)

    def observe(self, true_burial_depth_m: float, time_s: float) -> BurialDepthMeasurement:
        if time_s - self.last_sample_time_s < self.sample_period_s:
            return self.last_measurement

        self.last_sample_time_s = time_s
        if self.rng.uniform() < self.config.burial_depth_dropout_probability:
            self.last_measurement = BurialDepthMeasurement(time_s=time_s, depth_m=None, valid=False)
            return self.last_measurement

        measured_depth_m = true_burial_depth_m + self.rng.normal(0.0, self.config.burial_depth_noise_std_m)
        self.last_measurement = BurialDepthMeasurement(time_s=time_s, depth_m=float(measured_depth_m), valid=True)
        return self.last_measurement


class SonarModel:
    def __init__(self, config: SonarConfig, random_seed: int = 23) -> None:
        self.config = config
        self.rng = np.random.default_rng(random_seed)
        self.sample_period_s = 1.0 / max(config.update_rate_hz, 1e-9)
        self.last_sample_time_s = -1e9
        self.last_reading = SonarReading(
            time_s=0.0,
            valid=False,
            status="OFFLINE",
            relative_position_body_m=None,
            relative_heading_body_deg=None,
            estimated_position_ned_m=None,
            estimated_heading_ned_deg=None,
            confidence=0.0,
            distance_m=None,
        )

    def sample(self, pose: Pose, cable_truth: CableFitTruth, time_s: float) -> SonarReading:
        if time_s - self.last_sample_time_s < self.sample_period_s:
            return self.last_reading

        self.last_sample_time_s = time_s
        relative_ned_xy_m = cable_truth.nearest_point_xy_m - pose.position_ned_m[:2]
        relative_body_xy_m = ned_xy_to_body(relative_ned_xy_m, pose.heading_deg)
        distance_m = float(np.linalg.norm(relative_body_xy_m))
        bearing_deg = wrap_angle_deg(float(np.rad2deg(np.arctan2(relative_body_xy_m[1], relative_body_xy_m[0]))))
        in_sector = distance_m <= self.config.max_range_m and abs(bearing_deg) <= 0.5 * self.config.horizontal_fov_deg

        sonar_mode = self.config.mode.lower()
        if sonar_mode == "off":
            self.last_reading = SonarReading(
                time_s=time_s,
                valid=False,
                status="OFFLINE",
                relative_position_body_m=None,
                relative_heading_body_deg=None,
                estimated_position_ned_m=None,
                estimated_heading_ned_deg=None,
                confidence=0.0,
                distance_m=distance_m if in_sector else None,
            )
            return self.last_reading

        if sonar_mode == "reliable_absence" and distance_m >= self.config.absence_range_m:
            self.last_reading = SonarReading(
                time_s=time_s,
                valid=False,
                status="NO_CABLE",
                relative_position_body_m=None,
                relative_heading_body_deg=None,
                estimated_position_ned_m=None,
                estimated_heading_ned_deg=None,
                confidence=0.0,
                distance_m=distance_m,
            )
            return self.last_reading

        detection_probability = self.config.prob_detection - self.config.buried_loss_factor * max(cable_truth.burial_depth_m - 1.0, 0.0)
        detection_probability = float(np.clip(detection_probability, 0.0, 1.0))
        if not in_sector or self.rng.uniform() > detection_probability:
            self.last_reading = SonarReading(
                time_s=time_s,
                valid=False,
                status="OFFLINE",
                relative_position_body_m=None,
                relative_heading_body_deg=None,
                estimated_position_ned_m=None,
                estimated_heading_ned_deg=None,
                confidence=0.0,
                distance_m=distance_m if in_sector else None,
            )
            return self.last_reading

        advantage_hit = sonar_mode == "degraded" and self.rng.uniform() < self.config.advantage_probability
        position_noise_std_m = self.config.position_noise_std_m * (self.config.advantage_position_noise_scale if advantage_hit else 1.0)
        heading_noise_deg = self.config.heading_noise_deg * (self.config.advantage_heading_noise_scale if advantage_hit else 1.0)

        noisy_body_xy_m = relative_body_xy_m + self.rng.normal(0.0, position_noise_std_m, size=2)
        noisy_heading_ned_deg = wrap_angle_deg(cable_truth.heading_deg + self.rng.normal(0.0, heading_noise_deg))
        noisy_heading_body_deg = wrap_angle_deg(noisy_heading_ned_deg - pose.heading_deg)
        heading_rad = np.deg2rad(pose.heading_deg)
        noisy_position_ned_m = np.array(
            [
                pose.position_ned_m[0] + np.cos(heading_rad) * noisy_body_xy_m[0] - np.sin(heading_rad) * noisy_body_xy_m[1],
                pose.position_ned_m[1] + np.sin(heading_rad) * noisy_body_xy_m[0] + np.cos(heading_rad) * noisy_body_xy_m[1],
            ],
            dtype=float,
        )
        confidence = float(np.clip(1.0 - distance_m / max(self.config.max_range_m, 1e-6), 0.15, 0.95))
        if advantage_hit:
            confidence = max(confidence, self.config.advantage_confidence_floor)
        self.last_reading = SonarReading(
            time_s=time_s,
            valid=True,
            status="ONLINE",
            relative_position_body_m=noisy_body_xy_m,
            relative_heading_body_deg=noisy_heading_body_deg,
            estimated_position_ned_m=noisy_position_ned_m,
            estimated_heading_ned_deg=noisy_heading_ned_deg,
            confidence=confidence,
            distance_m=float(np.linalg.norm(noisy_body_xy_m)),
        )
        return self.last_reading
