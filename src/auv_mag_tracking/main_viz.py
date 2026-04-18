"""Simulation runner and real-time visualization."""

from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Ellipse
import numpy as np
from tqdm.auto import tqdm

from .config import ScenarioConfig
from .controller import GuidanceCommand, ZigZagController, TrackingMode, apply_attitude_profile, propagate_vehicle
from .environment import CableEnvironment
from .math_utils import Pose, smallest_angle_error_deg, wrap_angle_deg
from .perception import MagneticCablePerception, PerceptionState
from .perception_driver import PerceptionDriver, PerceptionDriverFrame
from .sensor_model import BurialDepthObserver, HighFidelityMagnetometer, IMUSimulator, MagnetometerModel, SonarModel


@dataclass
class SimulationReport:
    case_name: str
    duration_s: float
    peak_count: int
    final_confidence: float
    final_mode: str
    tracked_distance_m: float
    # Deployment-mode performance metrics
    cable_heading_error_deg: Optional[float] = None
    mean_lateral_deviation_m: Optional[float] = None
    along_track_coverage_ratio: Optional[float] = None
    heading_estimate_history_deg: Optional[List[float]] = None


@dataclass
class SignalTrendHistory:
    time_s: List[float]
    confidence: List[float]
    snr_db: List[float]
    processed_intensity_nt: List[float]
    signal_reliable: List[float]
    is_ac_detected: List[float]
    safe_lock_active: List[float]


class DeploymentPerformanceEvaluator:
    """Collects real-time cable-tracking performance metrics for deployment
    (no-prior) mode.

    Metrics
    -------
    * **Cable heading error**: angle between the estimated cable heading and
      the true cable tangent at the vehicle position.
    * **Mean lateral deviation**: average perpendicular distance from the
      vehicle track to the true cable path.
    * **Along-track coverage ratio**: fraction of total distance travelled
      that is projected onto the cable direction (1.0 = perfect following).
    """

    def __init__(self, environment: CableEnvironment) -> None:
        self.environment = environment
        self.vehicle_positions_xy: List[np.ndarray] = []
        self.heading_estimates_deg: List[Optional[float]] = []
        self.heading_confidences: List[float] = []
        self.true_headings_deg: List[float] = []

    def record(self, vehicle_xy_m: np.ndarray, estimated_heading_deg: Optional[float],
               heading_confidence: float) -> None:
        self.vehicle_positions_xy.append(np.asarray(vehicle_xy_m[:2], dtype=float))
        self.heading_estimates_deg.append(estimated_heading_deg)
        self.heading_confidences.append(heading_confidence)
        _, tangent_xy, _ = self.environment.route.nearest_point_and_tangent(vehicle_xy_m[:2])
        true_h = float(np.rad2deg(np.arctan2(tangent_xy[1], tangent_xy[0]))) % 360.0
        self.true_headings_deg.append(true_h)

    def compute(self) -> dict:
        result: dict = {
            "cable_heading_error_deg": None,
            "mean_lateral_deviation_m": None,
            "along_track_coverage_ratio": None,
            "final_heading_error_deg": None,
        }
        n = len(self.vehicle_positions_xy)
        if n < 2:
            return result

        positions = np.vstack(self.vehicle_positions_xy)

        # --- Mean lateral deviation ---
        lateral_deviations = []
        for pos in positions:
            nearest_pt, tangent, dist = self.environment.route.nearest_point_and_tangent(pos)
            lateral_deviations.append(dist)
        result["mean_lateral_deviation_m"] = float(np.mean(lateral_deviations))

        # --- Along-track coverage ---
        total_distance = float(np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1)))
        # Project each step onto the local cable tangent and sum the positive
        # components (moving along cable).
        along_track_distance = 0.0
        for i in range(1, n):
            step = positions[i] - positions[i - 1]
            _, tangent, _ = self.environment.route.nearest_point_and_tangent(positions[i])
            projection = float(np.dot(step, tangent))
            along_track_distance += max(projection, 0.0)
        result["along_track_coverage_ratio"] = (
            along_track_distance / max(total_distance, 1e-6)
            if total_distance > 1e-6
            else None
        )

        # --- Cable heading error ---
        valid_errors = []
        for est_h, true_h, conf in zip(
            self.heading_estimates_deg, self.true_headings_deg, self.heading_confidences
        ):
            # Lowered threshold from 0.25 to 0.10 for early convergence testing
            if est_h is not None and conf >= 0.10:
                valid_errors.append(abs(smallest_angle_error_deg(est_h, true_h)))
        if valid_errors:
            result["cable_heading_error_deg"] = float(np.mean(valid_errors))
            result["final_heading_error_deg"] = valid_errors[-1]
        return result


def _initial_vehicle_position_ned_m(scenario: ScenarioConfig, environment: CableEnvironment) -> np.ndarray:
    initial_position_ned_m = np.asarray(scenario.vehicle.initial_position_ned_m, dtype=float)
    if (not scenario.tracking.use_nominal_route_prior) or scenario.sonar.mode.lower() != "off":
        return initial_position_ned_m.copy()

    initial_xy_m = initial_position_ned_m[:2]
    nearest_xy_m, tangent_xy, _ = environment.route.nearest_point_and_tangent(initial_xy_m)
    offset_xy_m = initial_xy_m - nearest_xy_m
    offset_distance_m = float(np.linalg.norm(offset_xy_m))
    target_distance_m = min(offset_distance_m, max(4.0, 0.35 * scenario.sonar.absence_range_m))
    if offset_distance_m <= target_distance_m or offset_distance_m <= 1e-9:
        return initial_position_ned_m.copy()

    normal_xy_m = np.array([-tangent_xy[1], tangent_xy[0]], dtype=float)
    normal_norm = float(np.linalg.norm(normal_xy_m))
    if normal_norm <= 1e-9:
        return initial_position_ned_m.copy()
    normal_xy_m /= normal_norm
    normal_sign = float(np.sign(np.dot(offset_xy_m, normal_xy_m)))
    if normal_sign == 0.0:
        normal_sign = 1.0

    adjusted_position_ned_m = initial_position_ned_m.copy()
    adjusted_position_ned_m[:2] = nearest_xy_m + normal_sign * normal_xy_m * target_distance_m
    return adjusted_position_ned_m


class SimulationVisualizer:
    def __init__(self, scenario: ScenarioConfig, cable_route_ned_m: np.ndarray) -> None:
        self.scenario = scenario
        self.cable_route_ned_m = cable_route_ned_m
        plt.ion()
        self.figure = plt.figure(figsize=(17, 12.5))
        self.figure.suptitle(scenario.visualization.figure_title)
        self.figure.subplots_adjust(top=0.93, bottom=0.05, left=0.06, right=0.98)
        grid = self.figure.add_gridspec(4, 2, width_ratios=[1.4, 1.0], height_ratios=[1.0, 1.0, 0.9, 1.15], wspace=0.22, hspace=0.40)
        self.ax_top = self.figure.add_subplot(grid[:, 0])
        self.ax_signal = self.figure.add_subplot(grid[0, 1])
        self.ax_psd = self.figure.add_subplot(grid[1, 1])
        self.ax_quality = self.figure.add_subplot(grid[2, 1])
        self.ax_status = self.figure.add_subplot(grid[3, 1])
        self.ax_status.axis("off")
        self.ellipse_major_axis_m = 2.0
        self.ellipse_minor_axis_m = 1.0
        self.ellipse_angle_deg = 0.0
        self.ellipse_alpha = 0.2
        self.path_alpha = 0.85
        route_min_xy = np.min(cable_route_ned_m[:, :2], axis=0)
        route_max_xy = np.max(cable_route_ned_m[:, :2], axis=0)
        route_span_xy = np.maximum(route_max_xy - route_min_xy, np.array([40.0, 40.0], dtype=float))
        route_margin_xy = np.maximum(0.12 * route_span_xy, np.array([20.0, 20.0], dtype=float))

        self.ax_top.set_title("Top-down View", pad=10)
        self.ax_top.set_xlabel("North [m]")
        self.ax_top.set_ylabel("East [m]")
        self.ax_top.grid(True, alpha=0.3)
        self.ax_top.set_xlim(route_min_xy[0] - route_margin_xy[0], route_max_xy[0] + route_margin_xy[0])
        self.ax_top.set_ylim(route_min_xy[1] - route_margin_xy[1], route_max_xy[1] + route_margin_xy[1])
        self.ax_top.plot(cable_route_ned_m[:, 0], cable_route_ned_m[:, 1], color="black", lw=2.0, label="True cable")
        (self.vehicle_line,) = self.ax_top.plot([], [], color="tab:blue", lw=1.8, label="AUV track")
        (self.estimate_line,) = self.ax_top.plot([], [], color="tab:red", lw=1.5, ls="--", label="Estimated centerline")
        (self.estimated_path_line,) = self.ax_top.plot([], [], color="tab:purple", lw=1.2, alpha=0.85, label="Estimated path")
        self.peak_scatter = self.ax_top.scatter([], [], color="tab:orange", s=24, label="Peak points")
        self.current_marker = self.ax_top.scatter([], [], color="tab:green", s=70, label="AUV")
        self.turn_radius_ring = Circle((0.0, 0.0), radius=scenario.vehicle.min_turning_radius_m, fill=False, ec="tab:cyan", ls=":", lw=1.2)
        self.uncertainty_ellipse = Ellipse((0.0, 0.0), width=0.0, height=0.0, angle=0.0, fill=True, fc="#b8e6b8", ec="#6ba36b", alpha=0.2)
        self.ax_top.add_patch(self.turn_radius_ring)
        self.ax_top.add_patch(self.uncertainty_ellipse)
        self.ax_top.legend(loc="upper right", fontsize=8)
        # Magnetic vector direction arrow (quiver)
        self.vector_quiver = self.ax_top.quiver(
            0, 0, 0, 0,
            color="tab:cyan", scale=1.0, scale_units="xy",
            width=0.006, headwidth=3.5, headlength=4.5,
            alpha=0.9, label="B_xy vector", zorder=5,
        )
        # Cable direction estimate arrow from vector analysis
        self.cable_vector_quiver = self.ax_top.quiver(
            0, 0, 0, 0,
            color="tab:pink", scale=1.0, scale_units="xy",
            width=0.006, headwidth=3.5, headlength=4.5,
            alpha=0.9, label="Vector cable est.", zorder=5,
        )

        self.ax_signal.set_title("Time-domain Signal", pad=9)
        self.ax_signal.set_xlabel("Time relative to now [s]")
        self.ax_signal.set_ylabel("Field [nT]")
        self.ax_signal.grid(True, alpha=0.3)
        self.ax_signal.axhline(0.0, color="#999999", lw=0.8, alpha=0.5)
        (self.signal_raw_line,) = self.ax_signal.plot([], [], color="tab:gray", lw=1.0, alpha=0.85, label="Raw waveform")
        (self.signal_filtered_line,) = self.ax_signal.plot([], [], color="tab:red", lw=2.0, label="Filtered / extracted")
        (self.signal_amplitude_line,) = self.ax_signal.plot([], [], color="tab:green", lw=1.6, label="Processed amplitude")
        self.ax_signal.legend(loc="upper right", fontsize=8)

        self.ax_psd.set_title("Power Spectral Density", pad=9)
        self.ax_psd.set_xlabel("Frequency [Hz]")
        self.ax_psd.set_ylabel("PSD [nT^2]")
        self.ax_psd.grid(True, alpha=0.3)
        (self.psd_line,) = self.ax_psd.plot([], [], color="tab:blue", lw=1.6, label="PSD")
        self.psd_peak_marker = self.ax_psd.scatter([], [], color="tab:orange", s=32, label="Detected peak")
        self.target_frequency_line = self.ax_psd.axvline(self.scenario.signal.frequency_hz, color="tab:green", lw=1.1, ls="--", alpha=0.85, label="Target freq")
        self.detected_frequency_line = self.ax_psd.axvline(0.0, color="tab:orange", lw=1.1, ls=":", alpha=0.85, label="Detected freq")
        self.ax_psd.legend(loc="upper right", fontsize=8)

        self.ax_quality.set_title("Semantic Quality Trend", pad=9)
        self.ax_quality.set_xlabel("Time [s]")
        self.ax_quality.set_ylabel("State / confidence")
        self.ax_quality.set_ylim(-0.05, 1.1)
        self.ax_quality.set_yticks([0.0, 0.5, 1.0])
        self.ax_quality.grid(True, alpha=0.3)
        (self.quality_confidence_line,) = self.ax_quality.plot([], [], color="tab:blue", lw=1.8, label="Confidence")
        (self.quality_reliable_line,) = self.ax_quality.plot([], [], color="tab:green", lw=1.2, drawstyle="steps-post", label="DSP reliable")
        (self.quality_ac_line,) = self.ax_quality.plot([], [], color="tab:orange", lw=1.2, drawstyle="steps-post", label="AC detected")
        (self.quality_safe_lock_line,) = self.ax_quality.plot([], [], color="tab:red", lw=1.2, drawstyle="steps-post", label="Safe lock")
        self.ax_quality_snr = self.ax_quality.twinx()
        self.ax_quality_snr.set_ylabel("SNR [dB]")
        self.quality_snr_line, = self.ax_quality_snr.plot([], [], color="tab:purple", lw=1.5, alpha=0.8, label="SNR [dB]")
        self.ax_quality.legend(loc="upper left", fontsize=8)
        self.intensity_text = self.ax_quality.text(
            0.99,
            0.07,
            "",
            transform=self.ax_quality.transAxes,
            ha="right",
            va="bottom",
            fontsize=8.5,
            bbox=dict(boxstyle="round,pad=0.22", fc="white", ec="#bbbbbb", alpha=0.85),
        )
        self.status_text_artist = self.ax_status.text(
            0.02,
            0.98,
            "",
            va="top",
            ha="left",
            family="monospace",
            fontsize=8.7,
            linespacing=1.12,
        )

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
        signal_frame: Optional[PerceptionDriverFrame],
        trend_history: SignalTrendHistory,
    ) -> None:
        self.vehicle_line.set_data(vehicle_history_ned_m[:, 0], vehicle_history_ned_m[:, 1])
        latest_position = vehicle_history_ned_m[-1, :2]
        self.current_marker.set_offsets(latest_position.reshape(1, 2))

        # Magnetic vector direction arrows
        arrow_length_m = 8.0
        if perception.magnetic_vector_heading_deg is not None:
            b_rad = np.deg2rad(perception.magnetic_vector_heading_deg)
            self.vector_quiver.set_offsets(latest_position.reshape(1, 2))
            self.vector_quiver.set_UVC(
                np.array([arrow_length_m * np.cos(b_rad)]),
                np.array([arrow_length_m * np.sin(b_rad)]),
            )
        if perception.vector_cable_heading_deg is not None:
            c_rad = np.deg2rad(perception.vector_cable_heading_deg)
            self.cable_vector_quiver.set_offsets(latest_position.reshape(1, 2))
            self.cable_vector_quiver.set_UVC(
                np.array([arrow_length_m * np.cos(c_rad)]),
                np.array([arrow_length_m * np.sin(c_rad)]),
            )

        if peak_points_xy_m.size > 0:
            self.peak_scatter.set_offsets(peak_points_xy_m)
        else:
            self.peak_scatter.set_offsets(np.empty((0, 2)))

        if line_points_xy_m is not None:
            self.estimate_line.set_data(line_points_xy_m[:, 0], line_points_xy_m[:, 1])
        else:
            self.estimate_line.set_data([], [])

        if perception.estimated_path_points_xy_m.size > 0:
            self.estimated_path_line.set_data(perception.estimated_path_points_xy_m[:, 0], perception.estimated_path_points_xy_m[:, 1])
            target_path_alpha = 0.15 + 0.85 * float(np.clip(perception.confidence, 0.0, 1.0))
            smoothing_alpha = float(np.clip(self.scenario.visualization.uncertainty_smoothing_alpha, 0.0, 1.0))
            self.path_alpha = (1.0 - smoothing_alpha) * self.path_alpha + smoothing_alpha * target_path_alpha
            self.estimated_path_line.set_alpha(self.path_alpha)
        else:
            self.estimated_path_line.set_data([], [])
        self.turn_radius_ring.center = (latest_position[0], latest_position[1])
        self.turn_radius_ring.set_radius(self.scenario.vehicle.min_turning_radius_m)
        uncertainty_scale = float(np.clip(1.0 - perception.confidence, 0.0, 1.0))
        covariance_xy_m2 = perception.estimated_path_covariance_xy_m2
        major_axis_m = 2.0 + 10.0 * uncertainty_scale
        minor_axis_m = 1.0 + 6.0 * uncertainty_scale
        ellipse_heading_deg = perception.fused_heading_deg if perception.fused_heading_deg is not None else pose.heading_deg
        if covariance_xy_m2 is not None:
            covariance_xy_m2 = np.asarray(covariance_xy_m2, dtype=float)
            if covariance_xy_m2.shape == (2, 2) and np.all(np.isfinite(covariance_xy_m2)):
                eigenvalues, eigenvectors = np.linalg.eigh(covariance_xy_m2)
                eigenvalues = np.clip(eigenvalues, 0.0, None)
                major_sigma_m = float(np.sqrt(eigenvalues[1]))
                minor_sigma_m = float(np.sqrt(eigenvalues[0]))
                principal_axis_xy = eigenvectors[:, 1]
                ellipse_heading_deg = float(np.rad2deg(np.arctan2(principal_axis_xy[1], principal_axis_xy[0])))
                covariance_gain = 1.3 + 2.8 * uncertainty_scale
                residual_inflation_m = 0.45 * max(perception.fit_result.residual_m, 0.0)
                major_axis_m = max(major_axis_m, 2.0 * covariance_gain * major_sigma_m + residual_inflation_m + 1.0)
                minor_axis_m = max(minor_axis_m, 2.0 * covariance_gain * minor_sigma_m + 0.35 * residual_inflation_m + 0.8)
        ellipse_alpha = 0.20
        ellipse_facecolor = "#b8e6b8"
        ellipse_edgecolor = "#6ba36b"
        if perception.confidence < 0.4:
            ellipse_facecolor = "#ff7b7b"
            ellipse_edgecolor = "#cb4242"
            ellipse_alpha = 0.10 if int(perception.time_s * 3.0) % 2 == 0 else 0.28
        elif perception.confidence < 0.8:
            ellipse_facecolor = "#f6d08f"
            ellipse_edgecolor = "#cb9640"
            ellipse_alpha = 0.16 + 0.10 * uncertainty_scale
        smoothing_alpha = float(np.clip(self.scenario.visualization.uncertainty_smoothing_alpha, 0.0, 1.0))
        self.ellipse_major_axis_m = (1.0 - smoothing_alpha) * self.ellipse_major_axis_m + smoothing_alpha * major_axis_m
        self.ellipse_minor_axis_m = (1.0 - smoothing_alpha) * self.ellipse_minor_axis_m + smoothing_alpha * minor_axis_m
        angle_step_deg = smallest_angle_error_deg(ellipse_heading_deg, self.ellipse_angle_deg)
        self.ellipse_angle_deg = wrap_angle_deg(self.ellipse_angle_deg + smoothing_alpha * angle_step_deg)
        self.ellipse_alpha = (1.0 - smoothing_alpha) * self.ellipse_alpha + smoothing_alpha * ellipse_alpha
        self.uncertainty_ellipse.center = (latest_position[0], latest_position[1])
        self.uncertainty_ellipse.width = self.ellipse_major_axis_m
        self.uncertainty_ellipse.height = self.ellipse_minor_axis_m
        self.uncertainty_ellipse.angle = self.ellipse_angle_deg
        self.uncertainty_ellipse.set_facecolor(ellipse_facecolor)
        self.uncertainty_ellipse.set_edgecolor(ellipse_edgecolor)
        self.uncertainty_ellipse.set_alpha(self.ellipse_alpha)

        if signal_frame is not None:
            diagnostics = signal_frame.diagnostics
            if diagnostics.relative_time_s.size > 0:
                self.signal_raw_line.set_data(diagnostics.relative_time_s, diagnostics.raw_time_window_nt)
                self.signal_filtered_line.set_data(diagnostics.relative_time_s, diagnostics.filtered_time_window_nt)
                self.signal_amplitude_line.set_data(diagnostics.relative_time_s, diagnostics.processed_amplitude_window_nt)
                self.ax_signal.set_xlim(np.min(diagnostics.relative_time_s), 0.0)
                time_values = np.concatenate([
                    diagnostics.raw_time_window_nt,
                    diagnostics.filtered_time_window_nt,
                    diagnostics.processed_amplitude_window_nt,
                ])
                spread = max(float(np.max(np.abs(time_values))), 10.0)
                self.ax_signal.set_ylim(-1.1 * spread, 1.1 * spread)
            if diagnostics.frequency_axis_hz.size > 0:
                frequency_mask = diagnostics.frequency_axis_hz <= self.scenario.visualization.psd_max_frequency_hz
                freq_axis_hz = diagnostics.frequency_axis_hz[frequency_mask]
                psd_nt2 = diagnostics.psd_nt2[frequency_mask]
                self.psd_line.set_data(freq_axis_hz, psd_nt2)
                self.ax_psd.set_xlim(0.0, self.scenario.visualization.psd_max_frequency_hz)
                positive_psd = psd_nt2[psd_nt2 > 0.0]
                upper_psd = max(float(np.max(positive_psd)) if positive_psd.size else 1.0, 1.0)
                self.ax_psd.set_ylim(0.0, 1.1 * upper_psd)
                self.target_frequency_line.set_xdata([signal_frame.features.target_frequency_hz, signal_frame.features.target_frequency_hz])
                if diagnostics.detected_peak_frequency_hz is not None:
                    peak_frequency_hz = diagnostics.detected_peak_frequency_hz
                    peak_psd = diagnostics.detected_peak_magnitude_nt ** 2
                    self.psd_peak_marker.set_offsets(np.array([[peak_frequency_hz, peak_psd]], dtype=float))
                    self.detected_frequency_line.set_xdata([peak_frequency_hz, peak_frequency_hz])
                    self.detected_frequency_line.set_visible(True)
                else:
                    self.psd_peak_marker.set_offsets(np.empty((0, 2)))
                    self.detected_frequency_line.set_visible(False)
        elif signal_history_nt.size > 0:
            self.signal_raw_line.set_data(time_history_s, signal_history_nt[:, 0])
            self.signal_filtered_line.set_data(time_history_s, np.asarray(rms_history_nt, dtype=float))
            self.signal_amplitude_line.set_data(time_history_s, np.asarray(rms_history_nt, dtype=float))
            self.ax_signal.set_xlim(min(time_history_s), max(time_history_s) + 1e-9)
            all_values = np.concatenate([signal_history_nt[:, 0], np.asarray(rms_history_nt, dtype=float)])
            spread = max(np.max(np.abs(all_values)), 10.0)
            self.ax_signal.set_ylim(-1.1 * spread, 1.1 * spread)
            self.psd_line.set_data([], [])
            self.psd_peak_marker.set_offsets(np.empty((0, 2)))
            self.detected_frequency_line.set_visible(False)

        if trend_history.time_s:
            self.quality_confidence_line.set_data(trend_history.time_s, trend_history.confidence)
            self.quality_reliable_line.set_data(trend_history.time_s, trend_history.signal_reliable)
            self.quality_ac_line.set_data(trend_history.time_s, trend_history.is_ac_detected)
            self.quality_safe_lock_line.set_data(trend_history.time_s, trend_history.safe_lock_active)
            self.quality_snr_line.set_data(trend_history.time_s, trend_history.snr_db)
            self.ax_quality.set_xlim(min(trend_history.time_s), max(trend_history.time_s) + 1e-9)
            snr_min = min(min(trend_history.snr_db), -5.0)
            snr_max = max(max(trend_history.snr_db), 10.0)
            self.ax_quality_snr.set_ylim(snr_min - 2.0, snr_max + 2.0)
            intensity_ratio = trend_history.processed_intensity_nt[-1] / max(self.scenario.sensor.weak_signal_threshold_nt, 1e-6)
            self.intensity_text.set_text(f"Intensity/threshold: {intensity_ratio:.2f}x")

        burial_estimate = "N/A"
        if perception.estimated_burial_depth_m is not None:
            burial_estimate = f"{perception.estimated_burial_depth_m:.2f} m"
        hf_summary = f"{perception.signal_reliable} | {perception.is_ac_detected} | {perception.dominant_frequency_hz:.1f} Hz"
        if signal_frame is not None:
            hf_summary = (
                f"{signal_frame.features.sample_rate_hz:.0f} Hz / {signal_frame.features.bit_depth}-bit | "
                f"clip {signal_frame.features.clipping_ratio * 100.0:.1f}% | "
                f"Δf {signal_frame.features.frequency_error_hz:+.2f} Hz"
            )
        def fmt_optional(value: Optional[float], precision: str = ".1f", suffix: str = "") -> str:
            if value is None:
                return "N/A"
            return f"{value:{precision}}{suffix}"

        status_text = "\n".join(
            [
                f"Case: {self.scenario.name} | {self.scenario.description}",
                f"Mode: {command.mode.value} | Signal: {self.scenario.signal.mode} @ {self.scenario.signal.frequency_hz:.1f} Hz | DSP: {hf_summary}",
                f"Confidence: {perception.confidence:.2f} | SNR: {perception.snr:.1f} ({perception.snr_db:.1f} dB) | Source: {command.guidance_source}",
                f"Speed: {command.speed_mps:.2f} m/s | Heading cmd: {command.desired_heading_deg:.1f} deg | Yaw rate: {command.yaw_rate_deg_s:.1f} deg/s",
                f"Pitch/Roll: {pose.pitch_deg:.1f} deg / {pose.roll_deg:.1f} deg",
                f"Tracking strength: {perception.tracking_strength_nt:.1f} nT | RMS: {perception.rms_strength_nt:.1f} nT | Weak: {perception.weak_signal_flag}",
                f"DSP reliable: {perception.signal_reliable} | AC detected: {perception.is_ac_detected} | Dominant freq: {perception.dominant_frequency_hz:.1f} Hz | Peak prom: {signal_frame.features.peak_prominence_db:.1f} dB | DC ref: {signal_frame.diagnostics.dc_component_nt:.1f} nT" if signal_frame is not None else f"DSP reliable: {perception.signal_reliable} | AC detected: {perception.is_ac_detected} | Dominant freq: {perception.dominant_frequency_hz:.1f} Hz",
                f"Signal method: {signal_frame.features.signal_method} | Proc fs: {signal_frame.features.processing_sample_rate_hz:.0f} Hz" if signal_frame is not None else "Signal method: legacy",
                f"Noise floor: {perception.noise_floor_nt:.1f} nT | Line heading: {perception.line_heading_deg if perception.line_heading_deg is not None else 'N/A'}",
                f"Sonar_Status: {perception.sonar_status} | Safe_Lock: {command.safe_lock_active} | CritA: {perception.safe_lock_criterion_a_active} | CritB: {perception.safe_lock_criterion_b_active} | FitInv: {perception.safe_lock_fit_invalidated}",
                f"EnvGrad: {fmt_optional(perception.envelope_gradient_nT_per_m, '.2f', ' nT/m')} | GradHdg: {fmt_optional(perception.envelope_gradient_heading_deg, '.1f', '°')} | VecHdg: {fmt_optional(perception.magnetic_vector_heading_deg, '.1f', '°')} | VecCableHdg: {fmt_optional(perception.vector_cable_heading_deg, '.1f', '°')}",
                f"Turn radius limit: {self.scenario.vehicle.min_turning_radius_m:.2f} m | Commanded radius: {command.commanded_turn_radius_m if np.isfinite(command.commanded_turn_radius_m) else 'N/A'}",
                f"Burial true: {perception.true_burial_depth_m:.2f} m | Burial est: {burial_estimate}",
                f"Peak detected: {perception.peak_detected} | Detection age: {perception.last_detection_age_s:.2f} s",
                f"Fit residual: {perception.fit_result.residual_m:.2f} m | Fit rejected: {perception.fit_update_rejected} | Burial valid: {perception.burial_measurement_valid}",
            ]
        )
        self.status_text_artist.set_text(status_text)
        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()
        plt.pause(0.001)


class AuvCableTrackingSimulation:
    def __init__(self, scenario: ScenarioConfig) -> None:
        self.scenario = scenario
        self.environment = CableEnvironment(scenario)
        initial_position_ned_m = _initial_vehicle_position_ned_m(scenario, self.environment)
        initial_xy = np.asarray(initial_position_ned_m[:2], dtype=float)
        initial_seabed_depth_m = self.environment.seabed_depth_m(initial_xy)
        initial_pose = Pose(
            position_ned_m=np.asarray(
                [
                    initial_position_ned_m[0],
                    initial_position_ned_m[1],
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
        if scenario.sensor.high_fidelity.enabled:
            self.magnetometer = HighFidelityMagnetometer(scenario.sensor)
        else:
            self.magnetometer = MagnetometerModel(scenario.sensor)
        self.sonar = SonarModel(scenario.sonar)
        self.imu = IMUSimulator(scenario.sensor)
        self.burial_observer = BurialDepthObserver(scenario.survey)
        self.signal_driver = PerceptionDriver(scenario)
        self.perception = MagneticCablePerception(scenario)
        self.controller = ZigZagController(scenario)

        history_length = max(10, int(np.ceil(scenario.visualization.history_seconds / scenario.dt_s)))
        self.time_history_s: Deque[float] = deque(maxlen=history_length)
        self.vehicle_history_ned_m: Deque[np.ndarray] = deque(maxlen=history_length)
        self.signal_history_nt: Deque[np.ndarray] = deque(maxlen=history_length)
        self.rms_history_nt: Deque[float] = deque(maxlen=history_length)
        self.trend_time_s: Deque[float] = deque(maxlen=history_length)
        self.trend_confidence: Deque[float] = deque(maxlen=history_length)
        self.trend_snr_db: Deque[float] = deque(maxlen=history_length)
        self.trend_processed_intensity_nt: Deque[float] = deque(maxlen=history_length)
        self.trend_signal_reliable: Deque[float] = deque(maxlen=history_length)
        self.trend_is_ac_detected: Deque[float] = deque(maxlen=history_length)
        self.trend_safe_lock_active: Deque[float] = deque(maxlen=history_length)
        self.peak_positions_xy_m: List[np.ndarray] = []
        self.latest_command = GuidanceCommand(
            desired_heading_deg=self.pose.heading_deg,
            speed_mps=self.pose.speed_mps,
            mode=TrackingMode.SEARCH,
        )
        self.latest_perception: Optional[PerceptionState] = None
        self.latest_signal_frame: Optional[PerceptionDriverFrame] = None
        self.performance_evaluator: Optional[DeploymentPerformanceEvaluator] = None
        if not scenario.tracking.use_nominal_route_prior:
            self.performance_evaluator = DeploymentPerformanceEvaluator(self.environment)

    def _build_trend_history(self) -> SignalTrendHistory:
        return SignalTrendHistory(
            time_s=list(self.trend_time_s),
            confidence=list(self.trend_confidence),
            snr_db=list(self.trend_snr_db),
            processed_intensity_nt=list(self.trend_processed_intensity_nt),
            signal_reliable=list(self.trend_signal_reliable),
            is_ac_detected=list(self.trend_is_ac_detected),
            safe_lock_active=list(self.trend_safe_lock_active),
        )

    def _estimated_line_points(self) -> Optional[np.ndarray]:
        if self.latest_perception is None:
            return None
        fit_result = self.latest_perception.fit_result
        if fit_result.origin_xy_m is None or fit_result.direction_xy is None:
            return None
        direction = fit_result.direction_xy
        origin = self.latest_perception.estimated_cable_point_xy_m
        if origin is None:
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

            active_sample_rate_hz = 1.0 / max(self.magnetometer.sample_period_s, 1e-9)
            sample_count = max(1, int(round(self.scenario.dt_s * active_sample_rate_hz)))
            sample_times_s = time_s + (np.arange(sample_count, dtype=float) + 1.0) * self.magnetometer.sample_period_s
            current_block_a = self.scenario.signal.current_for_times(sample_times_s)
            cable_field_gain_ned_nt = self.environment.field_model.cable_field_gain_ned_nt(self.pose.position_ned_m)
            cable_field_block_ned_nt = current_block_a[:, None] * cable_field_gain_ned_nt[None, :]
            true_field_block_ned_nt = cable_field_block_ned_nt + self.environment.background_field_ned_nt
            magnetometer_reading = self.magnetometer.sample_block(
                true_field_block_ned_nt,
                self.pose,
                sample_times_s,
                cable_fields_ned_nt=cable_field_block_ned_nt,
            )
            signal_frame = self.signal_driver.update(magnetometer_reading)
            pose_measurement = self.imu.observe(self.pose, time_s)
            cable_truth = self.environment.cable_truth_at_xy(self.pose.position_ned_m[:2])
            sonar_reading = self.sonar.sample(self.pose, cable_truth, time_s)
            burial_measurement = self.burial_observer.observe(cable_truth.burial_depth_m, time_s)
            perception_state = self.perception.update(
                reading=magnetometer_reading,
                pose_measurement=pose_measurement,
                vehicle_position_xy_m=self.pose.position_ned_m[:2],
                burial_measurement=burial_measurement,
                true_burial_depth_m=cable_truth.burial_depth_m,
                sonar_reading=sonar_reading,
                signal_features=signal_frame.features,
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
            self.trend_time_s.append(perception_state.time_s)
            self.trend_confidence.append(perception_state.confidence)
            self.trend_snr_db.append(perception_state.snr_db)
            self.trend_processed_intensity_nt.append(signal_frame.features.processed_intensity_nt)
            self.trend_signal_reliable.append(1.0 if perception_state.signal_reliable else 0.0)
            self.trend_is_ac_detected.append(1.0 if perception_state.is_ac_detected else 0.0)
            self.trend_safe_lock_active.append(1.0 if perception_state.safe_lock_active else 0.0)
            if perception_state.peak_detected and perception_state.detected_peak_xy_m is not None:
                peak_count += 1
                self.peak_positions_xy_m.append(perception_state.detected_peak_xy_m.copy())

            if self.performance_evaluator is not None:
                self.performance_evaluator.record(
                    self.pose.position_ned_m[:2],
                    perception_state.deployment_estimated_cable_heading_deg,
                    perception_state.deployment_heading_confidence,
                )

            if step_index % 20 == 0 or step_index == total_steps - 1:
                progress.set_postfix(
                    mode=command.mode.value,
                    peaks=peak_count,
                    confidence=f"{perception_state.confidence:.2f}",
                )

            self.latest_command = command
            self.latest_perception = perception_state
            self.latest_signal_frame = signal_frame

            if visualizer and step_index % max(1, self.scenario.visualization.update_stride_steps) == 0:
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
                    signal_frame=signal_frame,
                    trend_history=self._build_trend_history(),
                )

        progress.close()

        if enable_visualization:
            plt.ioff()
            plt.show()

        final_confidence = 0.0 if self.latest_perception is None else self.latest_perception.confidence
        final_mode = self.latest_command.mode.value

        # Compute deployment performance metrics
        cable_heading_error = None
        mean_lateral_dev = None
        along_track_ratio = None
        heading_history = None
        if self.performance_evaluator is not None:
            metrics = self.performance_evaluator.compute()
            mean_lateral_dev = metrics["mean_lateral_deviation_m"]
            along_track_ratio = metrics["along_track_coverage_ratio"]
            heading_history = [
                h for h, c in zip(
                    self.performance_evaluator.heading_estimates_deg,
                    self.performance_evaluator.heading_confidences,
                )
                if h is not None and c >= 0.10
            ]

            if self.latest_perception is not None:
                estimated_heading_deg = self.latest_perception.deployment_estimated_cable_heading_deg
                if estimated_heading_deg is None:
                    estimated_heading_deg = self.latest_perception.line_heading_deg
                if estimated_heading_deg is not None:
                    true_heading_deg = self.environment.cable_truth_at_xy(self.pose.position_ned_m[:2]).heading_deg
                    directed_error_deg = abs(smallest_angle_error_deg(estimated_heading_deg, true_heading_deg))
                    cable_heading_error = min(directed_error_deg, 180.0 - directed_error_deg)
                else:
                    cable_heading_error = metrics["cable_heading_error_deg"]

        return SimulationReport(
            case_name=self.scenario.name,
            duration_s=self.scenario.duration_s,
            peak_count=peak_count,
            final_confidence=final_confidence,
            final_mode=final_mode,
            tracked_distance_m=tracked_distance_m,
            cable_heading_error_deg=cable_heading_error,
            mean_lateral_deviation_m=mean_lateral_dev,
            along_track_coverage_ratio=along_track_ratio,
            heading_estimate_history_deg=heading_history,
        )
