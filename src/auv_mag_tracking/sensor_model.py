"""Sensor abstractions for the magnetic cable tracking demo."""

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .config import ScenarioConfig, SensorConfig, SurveyConfig
from .math_utils import Pose, body_to_sensor, ned_to_body, rotation_matrix_sensor_to_body


@dataclass
class MagnetometerReading:
    time_s: float
    sensor_field_nt: np.ndarray
    sample_times_s: np.ndarray
    sample_block_sensor_nt: np.ndarray


@dataclass
class PoseMeasurement:
    time_s: float
    heading_deg: float
    pitch_deg: float
    roll_deg: float


@dataclass
class BurialDepthMeasurement:
    time_s: float
    depth_m: Optional[float]
    valid: bool


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

    def sample(self, true_field_ned_nt: np.ndarray, pose: Pose, time_s: float) -> MagnetometerReading:
        if time_s - self.last_sample_time_s < self.sample_period_s:
            return self.last_reading

        delta_t = max(time_s - self.last_sample_time_s, self.sample_period_s)
        field_body_nt = ned_to_body(true_field_ned_nt, pose.roll_deg, pose.pitch_deg, pose.heading_deg)
        field_sensor_nt = body_to_sensor(field_body_nt, self.sensor_to_body_matrix)

        drift_increment = self.rng.normal(
            loc=0.0,
            scale=self.config.bias_drift_std_nt_per_s * np.sqrt(delta_t),
            size=3,
        )
        self.bias_sensor_nt += drift_increment

        gaussian_noise = self.rng.normal(loc=0.0, scale=self.config.noise_std_nt, size=3)
        measured_sensor_nt = self.nonorthogonality_matrix @ field_sensor_nt
        measured_sensor_nt = measured_sensor_nt + self.bias_sensor_nt + gaussian_noise
        measured_sensor_nt = np.clip(
            measured_sensor_nt,
            -self.config.dynamic_range_nt,
            self.config.dynamic_range_nt,
        )

        self.last_sample_time_s = time_s
        self.last_reading = MagnetometerReading(
            time_s=time_s,
            sensor_field_nt=measured_sensor_nt,
            sample_times_s=np.asarray([time_s], dtype=float),
            sample_block_sensor_nt=measured_sensor_nt.reshape(1, 3),
        )
        return self.last_reading

    def sample_block(self, true_fields_ned_nt: np.ndarray, pose: Pose, sample_times_s: np.ndarray) -> MagnetometerReading:
        sample_times_s = np.asarray(sample_times_s, dtype=float)
        true_fields_ned_nt = np.asarray(true_fields_ned_nt, dtype=float)
        if sample_times_s.ndim != 1 or true_fields_ned_nt.shape != (sample_times_s.size, 3):
            raise ValueError("sample_times_s and true_fields_ned_nt must form an (N, 3) sample block")

        block_measurements = []
        for time_s, true_field_ned_nt in zip(sample_times_s, true_fields_ned_nt):
            reading = self.sample(true_field_ned_nt, pose, float(time_s))
            block_measurements.append(reading.sensor_field_nt.copy())

        sensor_block_nt = np.asarray(block_measurements, dtype=float)
        self.last_reading = MagnetometerReading(
            time_s=float(sample_times_s[-1]),
            sensor_field_nt=sensor_block_nt[-1].copy(),
            sample_times_s=sample_times_s.copy(),
            sample_block_sensor_nt=sensor_block_nt,
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
        return PoseMeasurement(time_s=time_s, heading_deg=heading_deg, pitch_deg=pitch_deg, roll_deg=roll_deg)


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
