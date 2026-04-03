"""Configuration objects and canned scenarios for the Phase 1 demo."""

from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple


Vector3Tuple = Tuple[float, float, float]
Vector2Tuple = Tuple[float, float]


@dataclass
class SignalConfig:
    mode: str = "ac_50hz"
    frequency_hz: float = 50.0
    dc_current_a: float = 0.0
    ac_current_amplitude_a: float = 600.0
    phase_rad: float = 0.0
    bandpass_half_width_hz: float = 8.0

    def current_at_time(self, time_s: float) -> float:
        if self.mode == "dc":
            return self.dc_current_a
        if self.mode in {"ac_50hz", "ac_demo", "ac_60hz"}:
            import math

            return self.ac_current_amplitude_a * math.sin(
                2.0 * math.pi * self.frequency_hz * time_s + self.phase_rad
            )
        return self.dc_current_a


@dataclass
class SensorConfig:
    magnetometer_sample_rate_hz: float = 200.0
    noise_std_nt: float = 0.05
    bias_drift_std_nt_per_s: float = 0.01
    nonorthogonality_deg: float = 0.2
    static_rotation_euler_deg: Vector3Tuple = (0.0, 0.0, 0.0)
    dynamic_range_nt: float = 100000.0
    imu_heading_noise_deg: float = 0.1
    imu_tilt_noise_deg: float = 0.05


@dataclass
class TrackingConfig:
    approach_angle_deg: float = 45.0
    turn_trigger_ratio: float = 0.60
    hysteresis_fraction: float = 0.08
    smoothing_time_constant_s: float = 0.20
    envelope_time_constant_s: float = 0.25
    noise_floor_time_constant_s: float = 1.50
    peak_cooldown_s: float = 0.80
    min_peak_strength_nt: float = 80.0
    fit_history_size: int = 5
    forgetting_factor: float = 0.72
    median_window_samples: int = 5
    lost_timeout_s: float = 4.0
    high_confidence_threshold: float = 0.65
    low_confidence_threshold: float = 0.35
    search_leg_time_s: float = 4.0


@dataclass
class VehicleConfig:
    cruise_speed_mps: float = 1.2
    search_speed_mps: float = 0.85
    max_yaw_rate_deg_s: float = 18.0
    altitude_above_seabed_m: float = 6.0
    initial_position_ned_m: Vector3Tuple = (-90.0, -45.0, 0.0)
    initial_heading_deg: float = 30.0
    pitch_amplitude_deg: float = 2.0
    roll_amplitude_deg: float = 3.0
    pitch_frequency_hz: float = 0.04
    roll_frequency_hz: float = 0.05


@dataclass
class EnvironmentConfig:
    cable_waypoints_xy_m: Tuple[Vector2Tuple, ...] = ((-220.0, 0.0), (220.0, 0.0))
    seabed_depth_m: float = 30.0
    seabed_undulation_m: float = 0.8
    seabed_wavelength_m: float = 220.0
    burial_depth_m: float = 1.5
    suspended_height_m: float = 0.0
    background_field_ned_nt: Vector3Tuple = (25000.0, 0.0, 42000.0)
    nominal_route_heading_deg: float = 0.0
    field_segment_length_m: float = 4.0


@dataclass
class SurveyConfig:
    burial_depth_update_rate_hz: float = 2.0
    burial_depth_noise_std_m: float = 0.12
    burial_depth_dropout_probability: float = 0.04


@dataclass
class VisualizationConfig:
    history_seconds: float = 20.0
    frame_interval_ms: int = 40
    figure_title: str = "AUV Magnetic Cable Tracking Demo"


@dataclass
class ScenarioConfig:
    name: str
    description: str
    duration_s: float
    dt_s: float
    signal: SignalConfig = field(default_factory=SignalConfig)
    sensor: SensorConfig = field(default_factory=SensorConfig)
    tracking: TrackingConfig = field(default_factory=TrackingConfig)
    vehicle: VehicleConfig = field(default_factory=VehicleConfig)
    environment: EnvironmentConfig = field(default_factory=EnvironmentConfig)
    survey: SurveyConfig = field(default_factory=SurveyConfig)
    visualization: VisualizationConfig = field(default_factory=VisualizationConfig)


def build_default_scenarios() -> Dict[str, ScenarioConfig]:
    standard = ScenarioConfig(
        name="case1",
        description="Straight cable tracking baseline using the default 50 Hz AC mode.",
        duration_s=80.0,
        dt_s=0.05,
        signal=SignalConfig(mode="ac_50hz", frequency_hz=50.0, ac_current_amplitude_a=620.0),
        sensor=SensorConfig(
            magnetometer_sample_rate_hz=200.0,
            noise_std_nt=0.05,
            bias_drift_std_nt_per_s=0.01,
            nonorthogonality_deg=0.18,
            static_rotation_euler_deg=(0.0, 0.0, 0.0),
            dynamic_range_nt=100000.0,
        ),
        tracking=TrackingConfig(
            approach_angle_deg=45.0,
            turn_trigger_ratio=0.85,
            smoothing_time_constant_s=0.18,
            envelope_time_constant_s=0.22,
            peak_cooldown_s=0.75,
            min_peak_strength_nt=120.0,
        ),
        vehicle=VehicleConfig(initial_position_ned_m=(-90.0, -45.0, 0.0), initial_heading_deg=35.0),
        environment=EnvironmentConfig(
            cable_waypoints_xy_m=((-220.0, 0.0), (220.0, 0.0)),
            nominal_route_heading_deg=0.0,
            burial_depth_m=1.5,
        ),
    )

    turning = ScenarioConfig(
        name="case2",
        description="Polyline cable turn scenario to verify forgetting-factor and centerline fitting updates.",
        duration_s=95.0,
        dt_s=0.05,
        signal=SignalConfig(mode="ac_50hz", frequency_hz=50.0, ac_current_amplitude_a=650.0, bandpass_half_width_hz=7.0),
        tracking=TrackingConfig(
            approach_angle_deg=42.0,
            turn_trigger_ratio=0.82,
            envelope_time_constant_s=0.20,
            forgetting_factor=0.68,
            fit_history_size=5,
            min_peak_strength_nt=130.0,
        ),
        vehicle=VehicleConfig(initial_position_ned_m=(-100.0, -50.0, 0.0), initial_heading_deg=35.0),
        environment=EnvironmentConfig(
            cable_waypoints_xy_m=((-220.0, 0.0), (20.0, 0.0), (180.0, 85.0)),
            nominal_route_heading_deg=0.0,
            burial_depth_m=1.3,
        ),
    )

    noisy = ScenarioConfig(
        name="case3",
        description="High-noise smartphone-like scenario to verify low-pass, hysteresis, and confidence degradation.",
        duration_s=90.0,
        dt_s=0.05,
        signal=SignalConfig(mode="ac_50hz", frequency_hz=50.0, ac_current_amplitude_a=700.0, bandpass_half_width_hz=6.0),
        sensor=SensorConfig(
            magnetometer_sample_rate_hz=200.0,
            noise_std_nt=150.0,
            bias_drift_std_nt_per_s=0.15,
            nonorthogonality_deg=1.5,
            static_rotation_euler_deg=(8.0, -5.0, 18.0),
            dynamic_range_nt=100000.0,
        ),
        tracking=TrackingConfig(
            approach_angle_deg=45.0,
            turn_trigger_ratio=0.86,
            smoothing_time_constant_s=0.35,
            envelope_time_constant_s=0.50,
            noise_floor_time_constant_s=2.5,
            hysteresis_fraction=0.14,
            peak_cooldown_s=1.2,
            min_peak_strength_nt=160.0,
            forgetting_factor=0.75,
            median_window_samples=9,
            search_leg_time_s=5.0,
        ),
        survey=SurveyConfig(burial_depth_noise_std_m=0.18, burial_depth_dropout_probability=0.10),
        vehicle=VehicleConfig(initial_position_ned_m=(-90.0, -42.0, 0.0), initial_heading_deg=28.0),
    )

    tilt = ScenarioConfig(
        name="case4",
        description="Large attitude disturbance scenario to verify static installation matrix and body-to-NED compensation.",
        duration_s=85.0,
        dt_s=0.05,
        signal=SignalConfig(mode="ac_50hz", frequency_hz=50.0, ac_current_amplitude_a=620.0, bandpass_half_width_hz=7.0),
        sensor=SensorConfig(
            magnetometer_sample_rate_hz=200.0,
            noise_std_nt=0.10,
            bias_drift_std_nt_per_s=0.02,
            nonorthogonality_deg=0.45,
            static_rotation_euler_deg=(20.0, -12.0, 35.0),
            dynamic_range_nt=100000.0,
            imu_heading_noise_deg=0.15,
            imu_tilt_noise_deg=0.08,
        ),
        vehicle=VehicleConfig(
            initial_position_ned_m=(-90.0, -44.0, 0.0),
            initial_heading_deg=35.0,
            pitch_amplitude_deg=12.0,
            roll_amplitude_deg=18.0,
            pitch_frequency_hz=0.06,
            roll_frequency_hz=0.08,
        ),
        tracking=TrackingConfig(
            approach_angle_deg=45.0,
            turn_trigger_ratio=0.84,
            smoothing_time_constant_s=0.22,
            envelope_time_constant_s=0.30,
            peak_cooldown_s=0.90,
            min_peak_strength_nt=100.0,
            median_window_samples=7,
            forgetting_factor=0.74,
        ),
    )

    demo = ScenarioConfig(
        name="case5",
        description="10-20 Hz demo mode for comparison with the original experimental concept.",
        duration_s=75.0,
        dt_s=0.05,
        signal=SignalConfig(mode="ac_demo", frequency_hz=15.0, ac_current_amplitude_a=520.0, bandpass_half_width_hz=5.0),
        tracking=TrackingConfig(
            approach_angle_deg=45.0,
            turn_trigger_ratio=0.84,
            smoothing_time_constant_s=0.22,
            envelope_time_constant_s=0.26,
            peak_cooldown_s=0.85,
            min_peak_strength_nt=90.0,
        ),
        vehicle=VehicleConfig(initial_position_ned_m=(-85.0, -40.0, 0.0), initial_heading_deg=30.0),
    )

    return {
        standard.name: standard,
        turning.name: turning,
        noisy.name: noisy,
        tilt.name: tilt,
        demo.name: demo,
    }


def get_scenario(case_name: str) -> Optional[ScenarioConfig]:
    return build_default_scenarios().get(case_name)
