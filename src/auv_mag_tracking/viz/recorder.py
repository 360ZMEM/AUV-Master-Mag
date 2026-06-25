"""Single offline simulation-loop contract for the visualization system.

Every offline figure / report / showcase consumes a :class:`RunRecord` produced
here.  This removes the duplicate simulation loop that used to live inside
``tools/diagnose_heading_error.py``: there is now exactly one offline driver.

GUI/Logic separation: this module only *produces data* (column arrays); it never
imports any plotting backend and never touches the file system except for the
optional ``.npz`` archive helpers.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from ..config import ScenarioConfig, build_default_scenarios
from ..controller import apply_attitude_profile, propagate_vehicle
from ..main_viz import AuvCableTrackingSimulation

# --- Numeric channels recorded each frame (column-store, binary friendly) ---
_NUMERIC_CHANNELS = (
    "time_s",
    "pos_x_m",
    "pos_y_m",
    "heading_deg",
    "speed_mps",
    "true_heading_deg",
    "true_nearest_x_m",
    "true_nearest_y_m",
    "true_burial_depth_m",
    "route_progress_m",
    "route_distance_m",
    "estimated_cable_x_m",
    "estimated_cable_y_m",
    "confidence",
    "sonar_confidence",
    "snr_db",
    "tracking_strength_nt",
    "fused_heading_deg",
    "line_heading_deg",
    "deployment_heading_deg",
    "estimated_burial_depth_m",
    "burial_inversion_uncertainty_m",
    "fit_residual_m",
    "fit_perp_eig_m2",
    "local_path_model_code",
    "local_path_heading_deg",
    "local_path_confidence",
    "local_path_residual_m",
    "local_path_radius_m",
    "local_path_tracking_state_code",
    "deployment_reacquire_required",
    "reacquire_region_center_x_m",
    "reacquire_region_center_y_m",
    "reacquire_region_heading_deg",
    "reacquire_region_half_length_m",
    "reacquire_region_half_width_m",
    "reacquire_region_confidence",
    "reacquire_region_score",
    "reacquire_region_reason_code",
    "magnetic_cross_track_offset_m",
    "magnetic_path_observation_valid",
    "magnetic_path_x_m",
    "magnetic_path_y_m",
    "magnetic_path_heading_deg",
    "magnetic_path_cross_track_offset_m",
    "magnetic_path_confidence",
    "magnetic_phase_observation_valid",
    "magnetic_phase_x_m",
    "magnetic_phase_y_m",
    "magnetic_phase_heading_deg",
    "magnetic_phase_amplitude_m",
    "magnetic_phase_duration_s",
    "magnetic_phase_confidence",
    "vector_consistency",
    "peak_detected",
    "safe_lock_active",
    "desired_heading_deg",
    "yaw_rate_deg_s",
)


@dataclass
class RunRecord:
    """逐帧仿真采集结果：可视化体系的唯一数据契约。"""

    case_name: str
    deployment_mode: bool
    dt_s: float
    channels: Dict[str, np.ndarray]
    modes: List[str]
    sources: List[str]
    cable_route_xy_m: np.ndarray
    metadata: Dict[str, float] = field(default_factory=dict)

    def __getitem__(self, key: str) -> np.ndarray:
        return self.channels[key]

    @property
    def n_steps(self) -> int:
        return int(self.channels["time_s"].size)

    def save_npz(self, path) -> None:
        """把数值通道与元数据归档为二进制 ``.npz``。"""
        np.savez_compressed(
            path,
            __case_name__=self.case_name,
            __deployment_mode__=self.deployment_mode,
            __dt_s__=self.dt_s,
            __modes__=np.array(self.modes, dtype=object),
            __sources__=np.array(self.sources, dtype=object),
            __cable_route_xy_m__=self.cable_route_xy_m,
            **self.channels,
        )


class RunRecorder:
    """累积逐帧通道，最终冻结为 :class:`RunRecord`。"""

    def __init__(self, case_name: str, deployment_mode: bool, dt_s: float,
                 cable_route_xy_m: np.ndarray) -> None:
        self.case_name = case_name
        self.deployment_mode = deployment_mode
        self.dt_s = dt_s
        self.cable_route_xy_m = np.asarray(cable_route_xy_m, dtype=float)
        self._numeric: Dict[str, List[float]] = {name: [] for name in _NUMERIC_CHANNELS}
        self.modes: List[str] = []
        self.sources: List[str] = []

    def append(self, **values: float) -> None:
        """记录一帧；缺省通道以 NaN 填充以保持列对齐。"""
        for name in _NUMERIC_CHANNELS:
            self._numeric[name].append(float(values.get(name, np.nan)))
        self.modes.append(str(values["mode"]))
        self.sources.append(str(values["source"]))

    def finalize(self) -> RunRecord:
        channels = {name: np.asarray(col, dtype=float) for name, col in self._numeric.items()}
        return RunRecord(
            case_name=self.case_name,
            deployment_mode=self.deployment_mode,
            dt_s=self.dt_s,
            channels=channels,
            modes=self.modes,
            sources=self.sources,
            cable_route_xy_m=self.cable_route_xy_m,
        )


def _perp_eigenvalue_m2(covariance_xy_m2: Optional[np.ndarray]) -> float:
    """拟合协方差的最小特征值，即 LOCK→TRACK 门限所用的垂直散布代理。"""
    if covariance_xy_m2 is None:
        return np.nan
    covariance = np.asarray(covariance_xy_m2, dtype=float)
    if covariance.shape != (2, 2) or not np.all(np.isfinite(covariance)):
        return np.nan
    return float(np.min(np.linalg.eigvalsh(covariance)))


def _optional(value: Optional[float]) -> float:
    return float(value) if value is not None else np.nan


def simulate_run(
    scenario: ScenarioConfig,
    deployment_mode: bool = False,
    max_steps: Optional[int] = None,
    duration_override_s: Optional[float] = None,
) -> RunRecord:
    """运行一次确定性仿真，返回逐帧 :class:`RunRecord`。

    这是离线可视化体系的唯一仿真循环；其物理步进与 ``main_viz`` 的实时循环
    使用完全相同的传感器/感知/控制组件，区别仅在于本函数只产出数据、不绘图。
    """
    scenario = copy.deepcopy(scenario)
    if deployment_mode:
        scenario.tracking.use_nominal_route_prior = False
    if duration_override_s is not None:
        scenario.duration_s = float(duration_override_s)

    sim = AuvCableTrackingSimulation(scenario)
    route_xy_m = sim.environment.sampled_cable_route_ned_m()[:, :2]
    route_length_m = sim.environment.route.total_length_m
    recorder = RunRecorder(scenario.name, deployment_mode, scenario.dt_s, route_xy_m)

    total_steps = int(np.ceil(scenario.duration_s / scenario.dt_s))
    if max_steps is not None:
        total_steps = min(total_steps, max_steps)

    track_entry_time_s: Optional[float] = None
    endpoint_completed = False
    for step_index in range(total_steps):
        time_s = step_index * scenario.dt_s
        apply_attitude_profile(sim.pose, scenario, time_s)

        sample_rate_hz = 1.0 / max(sim.magnetometer.sample_period_s, 1e-9)
        sample_count = max(1, int(round(scenario.dt_s * sample_rate_hz)))
        sample_times_s = time_s + (np.arange(sample_count, dtype=float) + 1.0) * sim.magnetometer.sample_period_s
        current_block_a = scenario.signal.current_for_times(sample_times_s)
        gain_ned_nt = sim.environment.field_model.cable_field_gain_ned_nt(sim.pose.position_ned_m)
        cable_block_ned_nt = current_block_a[:, None] * gain_ned_nt[None, :]
        true_block_ned_nt = cable_block_ned_nt + sim.environment.background_field_ned_nt
        reading = sim.magnetometer.sample_block(
            true_block_ned_nt, sim.pose, sample_times_s, cable_fields_ned_nt=cable_block_ned_nt
        )
        signal_frame = sim.signal_driver.update(reading)
        pose_measurement = sim.imu.observe(sim.pose, time_s)
        truth = sim.environment.cable_truth_at_xy(sim.pose.position_ned_m[:2])
        sonar_failure_active = (
            scenario.sonar.fail_after_track_active
            and track_entry_time_s is not None
            and time_s >= track_entry_time_s + scenario.sonar.fail_after_track_delay_s
        )
        if sonar_failure_active:
            sonar_reading = sim.sonar.force_offline(sim.pose, truth, time_s, status="FORCED_OFFLINE")
        else:
            sonar_reading = sim.sonar.sample(sim.pose, truth, time_s)
        burial_measurement = sim.burial_observer.observe(truth.burial_depth_m, time_s)
        perception = sim.perception.update(
            reading=reading,
            pose_measurement=pose_measurement,
            vehicle_position_xy_m=sim.pose.position_ned_m[:2],
            burial_measurement=burial_measurement,
            true_burial_depth_m=truth.burial_depth_m,
            sonar_reading=sonar_reading,
            signal_features=signal_frame.features,
        )
        command = sim.controller.update(sim.pose, perception)
        if command.mode.value == "track" and track_entry_time_s is None:
            track_entry_time_s = time_s

        nearest_xy, _, route_distance_m = sim.environment.route.nearest_point_and_tangent(sim.pose.position_ned_m[:2])
        estimated_cable_xy = perception.estimated_cable_point_xy_m
        recorder.append(
            time_s=time_s,
            pos_x_m=sim.pose.position_ned_m[0],
            pos_y_m=sim.pose.position_ned_m[1],
            heading_deg=sim.pose.heading_deg,
            speed_mps=command.speed_mps,
            true_heading_deg=truth.heading_deg,
            true_nearest_x_m=nearest_xy[0],
            true_nearest_y_m=nearest_xy[1],
            true_burial_depth_m=truth.burial_depth_m,
            route_progress_m=truth.progress_m,
            route_distance_m=route_distance_m,
            estimated_cable_x_m=np.nan if estimated_cable_xy is None else estimated_cable_xy[0],
            estimated_cable_y_m=np.nan if estimated_cable_xy is None else estimated_cable_xy[1],
            confidence=perception.confidence,
            sonar_confidence=perception.sonar_confidence,
            snr_db=perception.snr_db,
            tracking_strength_nt=perception.tracking_strength_nt,
            fused_heading_deg=_optional(perception.fused_heading_deg),
            line_heading_deg=_optional(perception.line_heading_deg),
            deployment_heading_deg=_optional(perception.deployment_estimated_cable_heading_deg),
            estimated_burial_depth_m=_optional(perception.estimated_burial_depth_m),
            burial_inversion_uncertainty_m=_optional(perception.burial_inversion_uncertainty_m),
            fit_residual_m=perception.fit_result.residual_m,
            fit_perp_eig_m2=_perp_eigenvalue_m2(perception.fit_result.covariance_xy_m2),
            local_path_model_code=perception.local_path_model_code,
            local_path_heading_deg=_optional(perception.local_path_heading_deg),
            local_path_confidence=perception.local_path_confidence,
            local_path_residual_m=perception.local_path_residual_m,
            local_path_radius_m=perception.local_path_radius_m,
            local_path_tracking_state_code={
                "collecting": 0.0,
                "line_track": 1.0,
                "curve_track": 2.0,
                "reacquire": 3.0,
            }.get(perception.local_path_tracking_state, 0.0),
            deployment_reacquire_required=1.0 if perception.deployment_reacquire_required else 0.0,
            reacquire_region_center_x_m=(
                np.nan if perception.reacquire_region_center_xy_m is None else perception.reacquire_region_center_xy_m[0]
            ),
            reacquire_region_center_y_m=(
                np.nan if perception.reacquire_region_center_xy_m is None else perception.reacquire_region_center_xy_m[1]
            ),
            reacquire_region_heading_deg=_optional(perception.reacquire_region_heading_deg),
            reacquire_region_half_length_m=perception.reacquire_region_half_length_m,
            reacquire_region_half_width_m=perception.reacquire_region_half_width_m,
            reacquire_region_confidence=perception.reacquire_region_confidence,
            reacquire_region_score=perception.reacquire_region_score,
            reacquire_region_reason_code={
                "none": 0.0,
                "forward_gate": 1.0,
                "turn_side_gate": 2.0,
                "last_crossing_gate": 3.0,
                "expanding_box": 4.0,
                "local_tangent_forward_gate": 5.0,
            }.get(perception.reacquire_region_reason, 0.0),
            magnetic_cross_track_offset_m=_optional(perception.magnetic_cross_track_offset_m),
            magnetic_path_observation_valid=1.0 if perception.magnetic_path_observation_valid else 0.0,
            magnetic_path_x_m=_optional(perception.magnetic_path_x_m),
            magnetic_path_y_m=_optional(perception.magnetic_path_y_m),
            magnetic_path_heading_deg=_optional(perception.magnetic_path_heading_deg),
            magnetic_path_cross_track_offset_m=_optional(perception.magnetic_path_cross_track_offset_m),
            magnetic_path_confidence=perception.magnetic_path_confidence,
            magnetic_phase_observation_valid=1.0 if perception.magnetic_phase_observation_valid else 0.0,
            magnetic_phase_x_m=_optional(perception.magnetic_phase_x_m),
            magnetic_phase_y_m=_optional(perception.magnetic_phase_y_m),
            magnetic_phase_heading_deg=_optional(perception.magnetic_phase_heading_deg),
            magnetic_phase_amplitude_m=perception.magnetic_phase_amplitude_m,
            magnetic_phase_duration_s=perception.magnetic_phase_duration_s,
            magnetic_phase_confidence=perception.magnetic_phase_confidence,
            vector_consistency=perception.vector_consistency_score,
            peak_detected=1.0 if perception.peak_detected else 0.0,
            safe_lock_active=1.0 if perception.safe_lock_active else 0.0,
            desired_heading_deg=command.desired_heading_deg,
            yaw_rate_deg_s=command.yaw_rate_deg_s,
            mode=command.mode.value,
            source=command.guidance_source,
        )

        if (
            scenario.stop_at_cable_endpoint
            and truth.progress_m >= route_length_m - scenario.endpoint_progress_margin_m
            and route_distance_m <= scenario.endpoint_lateral_tolerance_m
        ):
            endpoint_completed = True
            break

        seabed_depth_m = sim.environment.seabed_depth_m(sim.pose.position_ned_m[:2])
        sim.pose = propagate_vehicle(sim.pose, command, scenario, seabed_depth_m, scenario.dt_s)

    record = recorder.finalize()
    final_progress_m = float(record["route_progress_m"][-1]) if record.n_steps else 0.0
    final_distance_m = float(record["route_distance_m"][-1]) if record.n_steps else float("nan")
    record.metadata.update(
        {
            "route_length_m": float(route_length_m),
            "final_route_progress_m": final_progress_m,
            "final_route_distance_m": final_distance_m,
            "route_completion_ratio": final_progress_m / max(float(route_length_m), 1e-9),
            "endpoint_goal_enabled": float(scenario.stop_at_cable_endpoint),
            "endpoint_completed": float(endpoint_completed),
            "stop_reason": "endpoint" if endpoint_completed else "duration",
        }
    )
    return record


def simulate_case(case_name: str, deployment_mode: bool = False,
                  max_steps: Optional[int] = None,
                  duration_override_s: Optional[float] = None) -> RunRecord:
    """按场景名运行仿真的便捷封装。"""
    scenario = build_default_scenarios()[case_name]
    return simulate_run(
        scenario,
        deployment_mode=deployment_mode,
        max_steps=max_steps,
        duration_override_s=duration_override_s,
    )
