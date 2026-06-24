"""Deployment sensor source adapter.

This module is intentionally opt-in: importing ``auv_mag_tracking.experimental``
does not load it, so the default simulation path stays lightweight.
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np

from ..config import ScenarioConfig
from ..main_viz import SensorFrame
from ..math_utils import Pose, wrap_angle_deg
from ..sensor_model import (
    BurialDepthMeasurement,
    MagnetometerReading,
    PoseMeasurement,
    SonarReading,
)
from .simulator_connector import HoloOceanConnector, RawSensorBundle


@dataclass
class DeploymentSensorSource:
    """Adapt :class:`RawSensorBundle` into the runner's ``SensorFrame`` contract."""

    connector: HoloOceanConnector
    scenario: ScenarioConfig

    def poll(self, time_s: float, pose: Pose) -> SensorFrame:
        """Return one sensor frame using connector data and pose fallbacks."""
        bundle = self.connector.recv_sensor_updates()
        frame_time_s = float(bundle.time_s if bundle.time_s > 0.0 else time_s)
        magnetometer_reading = self._magnetometer_reading(bundle, frame_time_s)
        pose_measurement = self._pose_measurement(bundle, frame_time_s, pose)
        vehicle_position_xy_m = self._vehicle_position_xy(bundle, pose)
        sonar_reading = self._sonar_reading(bundle, frame_time_s, pose)
        burial_measurement = BurialDepthMeasurement(
            time_s=frame_time_s,
            depth_m=None if bundle.burial_depth_m is None else float(bundle.burial_depth_m),
            valid=bundle.burial_depth_m is not None,
        )
        return SensorFrame(
            magnetometer_reading=magnetometer_reading,
            pose_measurement=pose_measurement,
            vehicle_position_xy_m=vehicle_position_xy_m,
            burial_measurement=burial_measurement,
            sonar_reading=sonar_reading,
            true_burial_depth_m=None,
            true_cable_heading_deg=None,
        )

    def _magnetometer_reading(self, bundle: RawSensorBundle, time_s: float) -> MagnetometerReading:
        sample_block_nt = bundle.magnetometer_block_nt
        if sample_block_nt is None:
            sample_block_nt = np.zeros((1, 3), dtype=float)
        sample_block_nt = np.asarray(sample_block_nt, dtype=float)
        if sample_block_nt.ndim == 1:
            sample_block_nt = sample_block_nt.reshape(1, 3)
        if sample_block_nt.ndim != 2 or sample_block_nt.shape[1] != 3:
            raise ValueError("magnetometer_block_nt must have shape (N, 3)")

        sample_rate_hz = max(self.scenario.sensor.magnetometer_sample_rate_hz, 1e-9)
        sample_count = sample_block_nt.shape[0]
        sample_times_s = time_s - (sample_count - 1 - np.arange(sample_count, dtype=float)) / sample_rate_hz
        return MagnetometerReading(
            time_s=time_s,
            sensor_field_nt=sample_block_nt[-1].copy(),
            sample_times_s=sample_times_s,
            sample_block_sensor_nt=sample_block_nt,
            # Real deployment cannot know true cable-only field strength here.
            # Perception derives weak-signal status from ProcessedSignalFeatures.
            cable_strength_nt=0.0,
            weak_signal_flag=True,
            raw_sensor_block_nt=sample_block_nt.copy(),
            quantized_sensor_block_nt=sample_block_nt.copy(),
            dc_reference_sensor_nt=np.mean(sample_block_nt, axis=0),
            clipping_ratio=0.0,
            sample_rate_hz=sample_rate_hz,
            bit_depth=0,
        )

    @staticmethod
    def _pose_measurement(bundle: RawSensorBundle, time_s: float, pose: Pose) -> PoseMeasurement:
        return PoseMeasurement(
            time_s=time_s,
            heading_deg=pose.heading_deg if bundle.imu_heading_deg is None else float(bundle.imu_heading_deg),
            pitch_deg=pose.pitch_deg if bundle.imu_pitch_deg is None else float(bundle.imu_pitch_deg),
            roll_deg=pose.roll_deg if bundle.imu_roll_deg is None else float(bundle.imu_roll_deg),
            speed_mps=pose.speed_mps if bundle.vehicle_speed_mps is None else float(bundle.vehicle_speed_mps),
        )

    @staticmethod
    def _vehicle_position_xy(bundle: RawSensorBundle, pose: Pose) -> np.ndarray:
        if bundle.vehicle_position_ned_m is None:
            return pose.position_ned_m[:2].copy()
        position_ned_m = np.asarray(bundle.vehicle_position_ned_m, dtype=float)
        if position_ned_m.size < 2:
            raise ValueError("vehicle_position_ned_m must contain at least x/y")
        return position_ned_m[:2].copy()

    @staticmethod
    def _sonar_reading(bundle: RawSensorBundle, time_s: float, pose: Pose) -> Optional[SonarReading]:
        if bundle.sonar_relative_position_body_m is None:
            return None
        relative_body_xy_m = np.asarray(bundle.sonar_relative_position_body_m, dtype=float)[:2]
        if relative_body_xy_m.size != 2:
            raise ValueError("sonar_relative_position_body_m must contain x/y")

        heading_rad = np.deg2rad(pose.heading_deg)
        estimated_position_ned_m = np.array(
            [
                pose.position_ned_m[0] + np.cos(heading_rad) * relative_body_xy_m[0] - np.sin(heading_rad) * relative_body_xy_m[1],
                pose.position_ned_m[1] + np.sin(heading_rad) * relative_body_xy_m[0] + np.cos(heading_rad) * relative_body_xy_m[1],
            ],
            dtype=float,
        )
        estimated_heading_ned_deg = None
        relative_heading_body_deg = None
        if bundle.sonar_heading_deg is not None:
            estimated_heading_ned_deg = wrap_angle_deg(float(bundle.sonar_heading_deg))
            relative_heading_body_deg = wrap_angle_deg(estimated_heading_ned_deg - pose.heading_deg)
        confidence = float(np.clip(0.5 if bundle.sonar_confidence is None else bundle.sonar_confidence, 0.0, 1.0))
        return SonarReading(
            time_s=time_s,
            valid=True,
            status="ONLINE",
            relative_position_body_m=relative_body_xy_m.copy(),
            relative_heading_body_deg=relative_heading_body_deg,
            estimated_position_ned_m=estimated_position_ned_m,
            estimated_heading_ned_deg=estimated_heading_ned_deg,
            confidence=confidence,
            distance_m=float(np.linalg.norm(relative_body_xy_m)),
        )
