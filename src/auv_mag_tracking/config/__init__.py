"""Configuration objects and canned scenarios for the sonar-magnetic fusion demo."""

import copy
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import numpy as np


Vector3Tuple = Tuple[float, float, float]
Vector2Tuple = Tuple[float, float]


@dataclass
class HighFidelityMagnetometerConfig:
    enabled: bool = False
    sampling_rate_hz: float = 1000.0
    bit_depth: int = 24
    full_scale_nt: float = 100000.0
    auv_static_interference_body_nt: Vector3Tuple = (35.0, -20.0, 15.0)
    white_noise_std_nt: float = 0.03
    pink_noise_std_nt: float = 0.08
    pink_noise_exponent: float = 1.0
    impulse_probability: float = 0.002
    impulse_amplitude_nt: float = 60.0
    impulse_decay_samples: int = 8


@dataclass
class SignalProcessingConfig:
    window_size: int = 256
    overlap: float = 0.75
    window_function: str = "hann"
    target_frequency_tolerance_hz: float = 2.0
    peak_search_half_width_hz: float = 2.0
    use_centroid_frequency_estimation: bool = True
    min_ac_frequency_hz: float = 8.0
    ac_energy_ratio_threshold: float = 0.18
    snr_detection_threshold_db: float = 6.0
    axis_combination_mode: str = "dominant_axis"
    enable_interpolation: bool = True
    interpolation_target_rate_hz: float = 1000.0
    interpolation_input_rate_threshold_hz: float = 250.0
    bandpass_order: int = 2
    rms_cycle_count: int = 1
    lockin_enabled: bool = True
    lockin_cycle_count: int = 1
    diagnostics_use_fft: bool = True


@dataclass
class SignalConfig:
    mode: str = "ac_50hz"
    frequency_hz: float = 50.0
    dc_current_a: float = 0.0
    ac_current_amplitude_a: float = 600.0
    phase_rad: float = 0.0
    bandpass_half_width_hz: float = 8.0

    def current_for_times(self, time_s: np.ndarray) -> np.ndarray:
        time_s = np.asarray(time_s, dtype=float)
        if self.mode == "dc":
            return np.full_like(time_s, fill_value=self.dc_current_a, dtype=float)
        if self.mode in {"ac_50hz", "ac_demo", "ac_60hz"}:
            return self.ac_current_amplitude_a * np.sin(2.0 * np.pi * self.frequency_hz * time_s + self.phase_rad)
        return np.full_like(time_s, fill_value=self.dc_current_a, dtype=float)

    def current_at_time(self, time_s: float) -> float:
        return float(self.current_for_times(np.asarray([time_s], dtype=float))[0])


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
    weak_signal_threshold_nt: float = 18.0
    high_fidelity: HighFidelityMagnetometerConfig = field(default_factory=HighFidelityMagnetometerConfig)


@dataclass
class SonarConfig:
    mode: str = "degraded"
    max_range_m: float = 15.0
    horizontal_fov_deg: float = 120.0
    prob_detection: float = 0.70
    position_noise_std_m: float = 0.45
    heading_noise_deg: float = 5.0
    buried_loss_factor: float = 0.08
    update_rate_hz: float = 8.0
    absence_range_m: float = 18.0
    advantage_probability: float = 0.15
    advantage_position_noise_scale: float = 0.25
    advantage_heading_noise_scale: float = 0.35
    advantage_confidence_floor: float = 0.90


@dataclass
class TrackingConfig:
    approach_angle_deg: float = 45.0
    approach_angle_min_deg: float = 30.0
    approach_angle_max_deg: float = 45.0
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
    sonar_preferred_distance_m: float = 7.5
    magnetic_takeover_strength_nt: float = 55.0
    min_zigzag_width_m: float = 2.5
    max_zigzag_width_m: float = 9.0
    zigzag_width_gain_m_per_nt: float = 0.02
    safe_lock_peak_drop_nt: float = 15.0
    blind_follow_memory_size: int = 3
    fit_acceptance_residual_m: float = 12.0
    weighted_fitter_capacity: int = 8
    weighted_fitter_snr_floor: float = 1.05
    fit_reject_heading_delta_deg: float = 30.0
    fit_reject_confidence_threshold: float = 0.60
    consecutive_miss_threshold: int = 3
    spiral_radius_growth_mps: float = 0.55
    spiral_max_radius_m: float = 20.0
    spiral_entry_window_s: float = 2.0
    guidance_memory_timeout_s: float = 7.5
    memory_guidance_confidence_floor: float = 0.38
    use_nominal_route_prior: bool = True
    peak_ascending_min_samples: int = 2
    peak_descending_min_samples: int = 2
    peak_zone_window_size: int = 20
    peak_outlier_rejection_distance_m: float = 2.0
    deployment_bootstrap_min_peak_count: int = 3
    deployment_bootstrap_min_span_m: float = 6.0
    # --- Signal enhancement & gradient parameters ---
    envelope_savgol_window: int = 7
    envelope_savgol_polyorder: int = 2
    spatial_gradient_min_speed_mps: float = 0.3
    # --- Robust peak finding parameters ---
    parabolic_interpolation_enabled: bool = True
    peak_position_delay_s: float = 0.04
    # --- Global estimation parameters ---
    bootstrap_min_heading_diff_deg: float = 30.0
    weighted_ransac_iterations: int = 0
    weighted_ransac_inlier_threshold_m: float = 5.0
    weighted_ransac_min_inlier_ratio: float = 0.5
    # --- Perception safe-lock parameters ---
    safe_lock_strength_ratio_threshold: float = 0.20
    safe_lock_ideal_field_width_m: float = 12.0
    safe_lock_displacement_factor: float = 1.5
    safe_lock_gradient_angle_threshold_deg: float = 45.0
    safe_lock_gradient_confidence_penalty: float = 0.3
    # --- Vector heading analysis ---
    vector_heading_enabled: bool = True
    vector_heading_confidence_weight: float = 0.15


@dataclass
class VehicleConfig:
    cruise_speed_mps: float = 1.0
    search_speed_mps: float = 1.0
    max_yaw_rate_deg_s: float = 36.0
    min_turning_radius_m: float = 2.5
    altitude_above_seabed_m: float = 6.0
    initial_position_ned_m: Vector3Tuple = (-90.0, -45.0, 0.0)
    initial_heading_deg: float = 30.0
    pitch_amplitude_deg: float = 2.0
    roll_amplitude_deg: float = 3.0
    pitch_frequency_hz: float = 0.04
    roll_frequency_hz: float = 0.05


@dataclass
class EnvironmentConfig:
    cable_waypoints_xy_m: Tuple[Vector2Tuple, ...] = ((-320.0, 0.0), (360.0, 0.0))
    cable_route_mode: str = "spline"
    spline_tension: float = 0.0
    sine_amplitudes_m: Tuple[float, ...] = (6.0, 2.5)
    sine_wavelengths_m: Tuple[float, ...] = (140.0, 55.0)
    seabed_depth_m: float = 30.0
    seabed_undulation_m: float = 0.8
    seabed_wavelength_m: float = 220.0
    burial_depth_m: float = 1.5
    suspended_height_m: float = 0.0
    background_field_ned_nt: Vector3Tuple = (25000.0, 0.0, 42000.0)
    nominal_route_heading_deg: float = 0.0
    field_segment_length_m: float = 4.0
    min_cable_curvature_radius_m: float = 50.0
    validate_curvature_on_build: bool = True


@dataclass
class SurveyConfig:
    burial_depth_update_rate_hz: float = 2.0
    burial_depth_noise_std_m: float = 0.12
    burial_depth_dropout_probability: float = 0.04


@dataclass
class VisualizationConfig:
    history_seconds: float = 20.0
    frame_interval_ms: int = 40
    figure_title: str = "AUV Sonar-Magnetic Cable Tracking Demo"
    psd_max_frequency_hz: float = 120.0
    update_stride_steps: int = 6
    uncertainty_smoothing_alpha: float = 0.22


@dataclass
class ScenarioConfig:
    name: str
    description: str
    duration_s: float
    dt_s: float
    signal: SignalConfig = field(default_factory=SignalConfig)
    sensor: SensorConfig = field(default_factory=SensorConfig)
    signal_processing: SignalProcessingConfig = field(default_factory=SignalProcessingConfig)
    sonar: SonarConfig = field(default_factory=SonarConfig)
    tracking: TrackingConfig = field(default_factory=TrackingConfig)
    vehicle: VehicleConfig = field(default_factory=VehicleConfig)
    environment: EnvironmentConfig = field(default_factory=EnvironmentConfig)
    survey: SurveyConfig = field(default_factory=SurveyConfig)
    visualization: VisualizationConfig = field(default_factory=VisualizationConfig)


def build_default_scenarios() -> Dict[str, ScenarioConfig]:
    standard = ScenarioConfig(
        name="case1",
        description="Straight cable tracking baseline using the default 50 Hz AC mode.",
        duration_s=200.0,
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
            weighted_fitter_capacity=8,
        ),
        vehicle=VehicleConfig(initial_position_ned_m=(-90.0, -45.0, 0.0), initial_heading_deg=35.0),
        environment=EnvironmentConfig(
            cable_waypoints_xy_m=((-320.0, 0.0), (360.0, 0.0)),
            cable_route_mode="spline",
            nominal_route_heading_deg=0.0,
            burial_depth_m=1.5,
        ),
    )

    turning = ScenarioConfig(
        name="case2",
        description="Polyline cable turn scenario to verify forgetting-factor and centerline fitting updates.",
        duration_s=200.0,
        dt_s=0.05,
        signal=SignalConfig(mode="ac_50hz", frequency_hz=50.0, ac_current_amplitude_a=690.0, bandpass_half_width_hz=7.0),
        sonar=SonarConfig(prob_detection=0.82, position_noise_std_m=0.28, heading_noise_deg=3.5, update_rate_hz=10.0),
        tracking=TrackingConfig(
            approach_angle_deg=40.0,
            approach_angle_min_deg=32.0,
            approach_angle_max_deg=42.0,
            turn_trigger_ratio=0.80,
            envelope_time_constant_s=0.18,
            forgetting_factor=0.68,
            fit_history_size=8,
            min_peak_strength_nt=130.0,
            weighted_fitter_capacity=8,
            lost_timeout_s=6.0,
            sonar_preferred_distance_m=8.5,
            spiral_entry_window_s=3.5,
            spiral_radius_growth_mps=0.40,
            safe_lock_peak_drop_nt=14.0,
            guidance_memory_timeout_s=10.0,
            memory_guidance_confidence_floor=0.40,
        ),
        vehicle=VehicleConfig(
            cruise_speed_mps=1.05,
            search_speed_mps=1.05,
            max_yaw_rate_deg_s=38.0,
            min_turning_radius_m=2.5,
            initial_position_ned_m=(-110.0, -46.0, 0.0),
            initial_heading_deg=33.0,
        ),
        environment=EnvironmentConfig(
            cable_waypoints_xy_m=((-320.0, 0.0), (-40.0, 0.0), (120.0, 26.0), (260.0, 96.0), (420.0, 168.0)),
            cable_route_mode="spline",
            nominal_route_heading_deg=0.0,
            burial_depth_m=1.3,
        ),
    )

    noisy = ScenarioConfig(
        name="case3",
        description="High-noise smartphone-like scenario to verify low-pass, hysteresis, and confidence degradation.",
        duration_s=200.0,
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
            safe_lock_peak_drop_nt=22.0,
            magnetic_takeover_strength_nt=85.0,
            sonar_preferred_distance_m=9.0,
            weighted_fitter_capacity=8,
        ),
        sonar=SonarConfig(prob_detection=0.55, position_noise_std_m=0.9, heading_noise_deg=8.0),
        survey=SurveyConfig(burial_depth_noise_std_m=0.18, burial_depth_dropout_probability=0.10),
        vehicle=VehicleConfig(initial_position_ned_m=(-90.0, -42.0, 0.0), initial_heading_deg=28.0),
    )

    tilt = ScenarioConfig(
        name="case4",
        description="Large attitude disturbance scenario to verify static installation matrix and body-to-NED compensation.",
        duration_s=200.0,
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
            safe_lock_peak_drop_nt=18.0,
            weighted_fitter_capacity=8,
        ),
        sonar=SonarConfig(prob_detection=0.62, position_noise_std_m=0.55, heading_noise_deg=6.0),
    )

    demo = ScenarioConfig(
        name="case5",
        description="10-20 Hz demo mode for comparison with the original experimental concept.",
        duration_s=200.0,
        dt_s=0.05,
        signal=SignalConfig(mode="ac_demo", frequency_hz=15.0, ac_current_amplitude_a=520.0, bandpass_half_width_hz=5.0),
        tracking=TrackingConfig(
            approach_angle_deg=45.0,
            turn_trigger_ratio=0.84,
            smoothing_time_constant_s=0.22,
            envelope_time_constant_s=0.26,
            peak_cooldown_s=0.85,
            min_peak_strength_nt=90.0,
            weighted_fitter_capacity=8,
        ),
        vehicle=VehicleConfig(initial_position_ned_m=(-85.0, -40.0, 0.0), initial_heading_deg=30.0),
        environment=EnvironmentConfig(
            cable_waypoints_xy_m=((-200.0, 0.0), (-60.0, 14.0), (60.0, -12.0), (200.0, 6.0)),
            cable_route_mode="sine",
            nominal_route_heading_deg=0.0,
            burial_depth_m=1.4,
        ),
    )

    fusion = ScenarioConfig(
        name="case6",
        description="Sonar-magnetic fusion scenario with intermittent sonar and curved cable tracking.",
        duration_s=200.0,
        dt_s=0.05,
        signal=SignalConfig(mode="ac_50hz", frequency_hz=50.0, ac_current_amplitude_a=680.0, bandpass_half_width_hz=7.0),
        sensor=SensorConfig(
            magnetometer_sample_rate_hz=200.0,
            noise_std_nt=0.12,
            bias_drift_std_nt_per_s=0.03,
            nonorthogonality_deg=0.35,
            static_rotation_euler_deg=(5.0, -3.0, 10.0),
            dynamic_range_nt=100000.0,
            weak_signal_threshold_nt=20.0,
        ),
        sonar=SonarConfig(prob_detection=0.70, position_noise_std_m=0.35, heading_noise_deg=4.0),
        tracking=TrackingConfig(
            approach_angle_deg=40.0,
            turn_trigger_ratio=0.84,
            smoothing_time_constant_s=0.24,
            envelope_time_constant_s=0.24,
            peak_cooldown_s=0.85,
            min_peak_strength_nt=100.0,
            magnetic_takeover_strength_nt=65.0,
            safe_lock_peak_drop_nt=16.0,
            fit_acceptance_residual_m=10.0,
            weighted_fitter_capacity=8,
        ),
        vehicle=VehicleConfig(initial_position_ned_m=(-95.0, -48.0, 0.0), initial_heading_deg=32.0),
        environment=EnvironmentConfig(
            cable_waypoints_xy_m=((-220.0, -4.0), (-120.0, 22.0), (0.0, -18.0), (120.0, 28.0), (220.0, -2.0)),
            cable_route_mode="spline",
            nominal_route_heading_deg=0.0,
            burial_depth_m=1.5,
        ),
    )

    hf_phone = copy.deepcopy(standard)
    hf_phone.name = "case_hf_phone"
    hf_phone.description = "High-fidelity phone-grade magnetometer scenario with 15 Hz AC and 100 Hz sampling."
    hf_phone.sensor.magnetometer_sample_rate_hz = 100.0
    hf_phone.sensor.high_fidelity = HighFidelityMagnetometerConfig(
        enabled=True,
        sampling_rate_hz=100.0,
        bit_depth=24,
        full_scale_nt=100000.0,
        auv_static_interference_body_nt=(26.0, -12.0, 8.0),
        white_noise_std_nt=0.04,
        pink_noise_std_nt=0.12,
        impulse_probability=0.004,
        impulse_amplitude_nt=42.0,
        impulse_decay_samples=6,
    )
    hf_phone.signal = SignalConfig(mode="ac_demo", frequency_hz=15.0, ac_current_amplitude_a=540.0, bandpass_half_width_hz=5.0)
    hf_phone.signal_processing = SignalProcessingConfig(
        window_size=96,
        overlap=0.8,
        window_function="hann",
        target_frequency_tolerance_hz=2.5,
        peak_search_half_width_hz=2.5,
        min_ac_frequency_hz=8.0,
        ac_energy_ratio_threshold=0.15,
        snr_detection_threshold_db=5.0,
        axis_combination_mode="dominant_axis",
    )
    hf_phone.visualization.psd_max_frequency_hz = 40.0
    hf_phone.visualization.update_stride_steps = 5

    hf_industrial = copy.deepcopy(fusion)
    hf_industrial.name = "case_hf_industrial"
    hf_industrial.description = "High-fidelity industrial magnetometer scenario with 50 Hz AC and 1000 Hz sampling."
    hf_industrial.sensor.magnetometer_sample_rate_hz = 1000.0
    hf_industrial.sensor.high_fidelity = HighFidelityMagnetometerConfig(
        enabled=True,
        sampling_rate_hz=1000.0,
        bit_depth=24,
        full_scale_nt=100000.0,
        auv_static_interference_body_nt=(35.0, -20.0, 15.0),
        white_noise_std_nt=0.025,
        pink_noise_std_nt=0.06,
        impulse_probability=0.002,
        impulse_amplitude_nt=60.0,
        impulse_decay_samples=8,
    )
    hf_industrial.signal_processing = SignalProcessingConfig(
        window_size=512,
        overlap=0.85,
        window_function="hann",
        target_frequency_tolerance_hz=1.5,
        peak_search_half_width_hz=1.5,
        min_ac_frequency_hz=8.0,
        ac_energy_ratio_threshold=0.18,
        snr_detection_threshold_db=6.0,
        axis_combination_mode="dominant_axis",
    )
    hf_industrial.visualization.psd_max_frequency_hz = 80.0
    hf_industrial.visualization.update_stride_steps = 4

    # --- case8: Tight curve scenario (minimum curvature radius ~50 m) ---
    # Design a smooth S-bend where the tightest curvature approaches the
    # 50 m minimum radius constraint.  Waypoints are chosen so that the
    # cubic spline stays within physical plausibility.
    tight_bend = ScenarioConfig(
        name="case8",
        description="Tight-curve scenario with curvature approaching 50 m minimum radius. Tests safe-lock and fit recovery at bends.",
        duration_s=300.0,
        dt_s=0.05,
        signal=SignalConfig(mode="ac_50hz", frequency_hz=50.0, ac_current_amplitude_a=680.0, bandpass_half_width_hz=7.0),
        sensor=SensorConfig(
            magnetometer_sample_rate_hz=200.0,
            noise_std_nt=0.08,
            bias_drift_std_nt_per_s=0.015,
        ),
        tracking=TrackingConfig(
            approach_angle_deg=40.0,
            turn_trigger_ratio=0.82,
            envelope_time_constant_s=0.20,
            peak_cooldown_s=0.80,
            min_peak_strength_nt=110.0,
            forgetting_factor=0.65,
            weighted_fitter_capacity=10,
            lost_timeout_s=5.0,
            safe_lock_peak_drop_nt=18.0,
            safe_lock_strength_ratio_threshold=0.20,
            safe_lock_ideal_field_width_m=12.0,
            safe_lock_displacement_factor=1.5,
            safe_lock_gradient_angle_threshold_deg=45.0,
            bootstrap_min_heading_diff_deg=30.0,
        ),
        vehicle=VehicleConfig(
            cruise_speed_mps=0.9,
            min_turning_radius_m=2.5,
            initial_position_ned_m=(-200.0, -50.0, 0.0),
            initial_heading_deg=25.0,
        ),
        environment=EnvironmentConfig(
            cable_waypoints_xy_m=(
                (-300.0, 0.0),
                (-100.0, 0.0),
                (-30.0, 40.0),
                (50.0, 50.0),
                (120.0, 10.0),
                (300.0, 10.0),
            ),
            cable_route_mode="spline",
            nominal_route_heading_deg=0.0,
            burial_depth_m=1.4,
            min_cable_curvature_radius_m=50.0,
            validate_curvature_on_build=True,
        ),
    )

    return {
        standard.name: standard,
        turning.name: turning,
        noisy.name: noisy,
        tilt.name: tilt,
        demo.name: demo,
        fusion.name: fusion,
        hf_phone.name: hf_phone,
        hf_industrial.name: hf_industrial,
        tight_bend.name: tight_bend,
    }


def get_scenario(case_name: str) -> Optional[ScenarioConfig]:
    return build_default_scenarios().get(case_name)
