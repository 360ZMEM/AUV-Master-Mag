"""Simulation runner and real-time visualization."""

from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional

import matplotlib.pyplot as plt
import numpy as np
from tqdm.auto import tqdm

from .config import ScenarioConfig
from .controller import GuidanceCommand, ZigZagController, TrackingMode, apply_attitude_profile, propagate_vehicle
from .environment import CableEnvironment
from .math_utils import Pose
from .perception import MagneticCablePerception, PerceptionState
from .sensor_model import BurialDepthObserver, IMUSimulator, MagnetometerModel


@dataclass
class SimulationReport:
    case_name: str
    duration_s: float
    peak_count: int
    final_confidence: float
    final_mode: str
    tracked_distance_m: float


class SimulationVisualizer:
    def __init__(self, scenario: ScenarioConfig, cable_route_ned_m: np.ndarray) -> None:
        self.scenario = scenario
        self.cable_route_ned_m = cable_route_ned_m
        plt.ion()
        self.figure = plt.figure(figsize=(14, 8))
        self.figure.suptitle(scenario.visualization.figure_title)
        grid = self.figure.add_gridspec(2, 2, height_ratios=[2.0, 1.0])
        self.ax_top = self.figure.add_subplot(grid[:, 0])
        self.ax_signal = self.figure.add_subplot(grid[0, 1])
        self.ax_status = self.figure.add_subplot(grid[1, 1])
        self.ax_status.axis("off")

        self.ax_top.set_title("Top-down View")
        self.ax_top.set_xlabel("North [m]")
        self.ax_top.set_ylabel("East [m]")
        self.ax_top.grid(True, alpha=0.3)
        self.ax_top.plot(cable_route_ned_m[:, 0], cable_route_ned_m[:, 1], color="black", lw=2.0, label="True cable")
        (self.vehicle_line,) = self.ax_top.plot([], [], color="tab:blue", lw=1.8, label="AUV track")
        (self.estimate_line,) = self.ax_top.plot([], [], color="tab:red", lw=1.5, ls="--", label="Estimated centerline")
        self.peak_scatter = self.ax_top.scatter([], [], color="tab:orange", s=24, label="Peak points")
        self.current_marker = self.ax_top.scatter([], [], color="tab:green", s=70, label="AUV")
        self.ax_top.legend(loc="upper right")

        self.ax_signal.set_title("Signal View")
        self.ax_signal.set_xlabel("Time [s]")
        self.ax_signal.set_ylabel("Field [nT]")
        self.ax_signal.grid(True, alpha=0.3)
        (self.signal_x_line,) = self.ax_signal.plot([], [], label="Sensor X")
        (self.signal_y_line,) = self.ax_signal.plot([], [], label="Sensor Y")
        (self.signal_z_line,) = self.ax_signal.plot([], [], label="Sensor Z")
        (self.rms_line,) = self.ax_signal.plot([], [], color="tab:red", lw=2.0, label="RMS / tracking strength")
        self.ax_signal.legend(loc="upper right")

    def update(
        self,
        time_history_s: List[float],
        vehicle_history_ned_m: np.ndarray,
        signal_history_nt: np.ndarray,
        rms_history_nt: List[float],
        peak_points_xy_m: np.ndarray,
        line_points_xy_m: Optional[np.ndarray],
        perception: PerceptionState,
        command: GuidanceCommand,
        pose: Pose,
    ) -> None:
        self.vehicle_line.set_data(vehicle_history_ned_m[:, 0], vehicle_history_ned_m[:, 1])
        latest_position = vehicle_history_ned_m[-1, :2]
        self.current_marker.set_offsets(latest_position.reshape(1, 2))

        if peak_points_xy_m.size > 0:
            self.peak_scatter.set_offsets(peak_points_xy_m)
        else:
            self.peak_scatter.set_offsets(np.empty((0, 2)))

        if line_points_xy_m is not None:
            self.estimate_line.set_data(line_points_xy_m[:, 0], line_points_xy_m[:, 1])
        else:
            self.estimate_line.set_data([], [])

        if signal_history_nt.size > 0:
            self.signal_x_line.set_data(time_history_s, signal_history_nt[:, 0])
            self.signal_y_line.set_data(time_history_s, signal_history_nt[:, 1])
            self.signal_z_line.set_data(time_history_s, signal_history_nt[:, 2])
            self.rms_line.set_data(time_history_s, rms_history_nt)
            self.ax_signal.set_xlim(min(time_history_s), max(time_history_s) + 1e-9)
            all_values = np.concatenate([signal_history_nt.flatten(), np.asarray(rms_history_nt, dtype=float)])
            spread = max(np.max(np.abs(all_values)), 10.0)
            self.ax_signal.set_ylim(-1.1 * spread, 1.1 * spread)

        self.ax_status.clear()
        self.ax_status.axis("off")
        burial_estimate = "N/A"
        if perception.estimated_burial_depth_m is not None:
            burial_estimate = f"{perception.estimated_burial_depth_m:.2f} m"
        status_text = "\n".join(
            [
                f"Case: {self.scenario.name} | {self.scenario.description}",
                f"Mode: {command.mode.value} | Signal: {self.scenario.signal.mode} @ {self.scenario.signal.frequency_hz:.1f} Hz",
                f"Confidence: {perception.confidence:.2f} | SNR: {perception.snr:.1f}",
                f"Speed: {command.speed_mps:.2f} m/s | Heading cmd: {command.desired_heading_deg:.1f} deg",
                f"Pitch/Roll: {pose.pitch_deg:.1f} deg / {pose.roll_deg:.1f} deg",
                f"Tracking strength: {perception.tracking_strength_nt:.1f} nT | RMS: {perception.rms_strength_nt:.1f} nT",
                f"Noise floor: {perception.noise_floor_nt:.1f} nT | Line heading: {perception.line_heading_deg if perception.line_heading_deg is not None else 'N/A'}",
                f"Burial true: {perception.true_burial_depth_m:.2f} m | Burial est: {burial_estimate}",
                f"Peak detected: {perception.peak_detected} | Detection age: {perception.last_detection_age_s:.2f} s",
                f"Fit residual: {perception.fit_result.residual_m:.2f} m | Burial valid: {perception.burial_measurement_valid}",
            ]
        )
        self.ax_status.text(0.02, 0.98, status_text, va="top", ha="left", family="monospace", fontsize=10)

        self.ax_top.relim()
        self.ax_top.autoscale_view()
        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()
        plt.pause(0.001)


class AuvCableTrackingSimulation:
    def __init__(self, scenario: ScenarioConfig) -> None:
        self.scenario = scenario
        self.environment = CableEnvironment(scenario)
        initial_xy = np.asarray(scenario.vehicle.initial_position_ned_m[:2], dtype=float)
        initial_seabed_depth_m = self.environment.seabed_depth_m(initial_xy)
        initial_pose = Pose(
            position_ned_m=np.asarray(
                [
                    scenario.vehicle.initial_position_ned_m[0],
                    scenario.vehicle.initial_position_ned_m[1],
                    initial_seabed_depth_m - scenario.vehicle.altitude_above_seabed_m,
                ],
                dtype=float,
            ),
            heading_deg=scenario.vehicle.initial_heading_deg,
            pitch_deg=0.0,
            roll_deg=0.0,
            speed_mps=scenario.vehicle.search_speed_mps,
        )
        self.pose = initial_pose
        self.magnetometer = MagnetometerModel(scenario.sensor)
        self.imu = IMUSimulator(scenario.sensor)
        self.burial_observer = BurialDepthObserver(scenario.survey)
        self.perception = MagneticCablePerception(scenario)
        self.controller = ZigZagController(scenario)

        history_length = max(10, int(np.ceil(scenario.visualization.history_seconds / scenario.dt_s)))
        self.time_history_s: Deque[float] = deque(maxlen=history_length)
        self.vehicle_history_ned_m: Deque[np.ndarray] = deque(maxlen=history_length)
        self.signal_history_nt: Deque[np.ndarray] = deque(maxlen=history_length)
        self.rms_history_nt: Deque[float] = deque(maxlen=history_length)
        self.peak_positions_xy_m: List[np.ndarray] = []
        self.latest_command = GuidanceCommand(
            desired_heading_deg=self.pose.heading_deg,
            speed_mps=self.pose.speed_mps,
            mode=TrackingMode.SEARCH,
        )
        self.latest_perception: Optional[PerceptionState] = None

    def _estimated_line_points(self) -> Optional[np.ndarray]:
        if self.latest_perception is None:
            return None
        fit_result = self.latest_perception.fit_result
        if fit_result.origin_xy_m is None or fit_result.direction_xy is None:
            return None
        direction = fit_result.direction_xy
        origin = fit_result.origin_xy_m
        line_length_m = 140.0
        start = origin - direction * line_length_m
        end = origin + direction * line_length_m
        return np.vstack([start, end])

    def run(self, enable_visualization: bool = True) -> SimulationReport:
        visualizer = None
        if enable_visualization:
            visualizer = SimulationVisualizer(self.scenario, self.environment.sampled_cable_route_ned_m())

        peak_count = 0
        tracked_distance_m = 0.0
        previous_position = self.pose.position_ned_m.copy()
        total_steps = int(np.ceil(self.scenario.duration_s / self.scenario.dt_s))
        progress = tqdm(
            range(total_steps),
            desc=f"{self.scenario.name} simulation",
            unit="step",
            dynamic_ncols=True,
            leave=False,
        )

        for step_index in progress:
            time_s = step_index * self.scenario.dt_s
            apply_attitude_profile(self.pose, self.scenario, time_s)

            sample_count = max(1, int(round(self.scenario.dt_s * self.scenario.sensor.magnetometer_sample_rate_hz)))
            sample_times_s = time_s + (np.arange(sample_count, dtype=float) + 1.0) * self.magnetometer.sample_period_s
            true_field_block_ned_nt = np.vstack(
                [self.environment.full_field_ned_nt(self.pose.position_ned_m, sample_time_s) for sample_time_s in sample_times_s]
            )
            magnetometer_reading = self.magnetometer.sample_block(true_field_block_ned_nt, self.pose, sample_times_s)
            pose_measurement = self.imu.observe(self.pose, time_s)
            cable_truth = self.environment.cable_truth_at_xy(self.pose.position_ned_m[:2])
            burial_measurement = self.burial_observer.observe(cable_truth.burial_depth_m, time_s)
            perception_state = self.perception.update(
                reading=magnetometer_reading,
                pose_measurement=pose_measurement,
                vehicle_position_xy_m=self.pose.position_ned_m[:2],
                burial_measurement=burial_measurement,
                true_burial_depth_m=cable_truth.burial_depth_m,
            )
            command = self.controller.update(self.pose, perception_state)

            seabed_depth_m = self.environment.seabed_depth_m(self.pose.position_ned_m[:2])
            self.pose = propagate_vehicle(self.pose, command, self.scenario, seabed_depth_m, self.scenario.dt_s)

            tracked_distance_m += float(np.linalg.norm(self.pose.position_ned_m[:2] - previous_position[:2]))
            previous_position = self.pose.position_ned_m.copy()

            self.time_history_s.append(magnetometer_reading.time_s)
            self.vehicle_history_ned_m.append(self.pose.position_ned_m.copy())
            self.signal_history_nt.append(magnetometer_reading.sensor_field_nt.copy())
            self.rms_history_nt.append(perception_state.tracking_strength_nt)
            if perception_state.peak_detected:
                peak_count += 1
                self.peak_positions_xy_m.append(self.pose.position_ned_m[:2].copy())

            progress.set_postfix(
                mode=command.mode.value,
                peaks=peak_count,
                confidence=f"{perception_state.confidence:.2f}",
            )

            self.latest_command = command
            self.latest_perception = perception_state

            if visualizer and step_index % 2 == 0:
                visualizer.update(
                    time_history_s=list(self.time_history_s),
                    vehicle_history_ned_m=np.asarray(self.vehicle_history_ned_m, dtype=float),
                    signal_history_nt=np.asarray(self.signal_history_nt, dtype=float),
                    rms_history_nt=list(self.rms_history_nt),
                    peak_points_xy_m=np.asarray(self.peak_positions_xy_m, dtype=float) if self.peak_positions_xy_m else np.empty((0, 2)),
                    line_points_xy_m=self._estimated_line_points(),
                    perception=perception_state,
                    command=command,
                    pose=self.pose,
                )

        progress.close()

        if enable_visualization:
            plt.ioff()
            plt.show()

        final_confidence = 0.0 if self.latest_perception is None else self.latest_perception.confidence
        final_mode = self.latest_command.mode.value
        return SimulationReport(
            case_name=self.scenario.name,
            duration_s=self.scenario.duration_s,
            peak_count=peak_count,
            final_confidence=final_confidence,
            final_mode=final_mode,
            tracked_distance_m=tracked_distance_m,
        )
