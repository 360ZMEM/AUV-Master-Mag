"""Pure metric functions for the visualization system.

All functions here are side-effect free: they consume a :class:`RunRecord` and
return plain dataclasses / dicts.  The real-time dashboard and the offline report
share these same functions, so there is a single source of truth for every
number (no "two metric implementations" drift).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List

import numpy as np

from ..math_utils import smallest_angle_error_deg
from .baseline import MilestoneMetrics
from .recorder import RunRecord

# Heading-quality gates (deg)
_GOOD_HEADING_DEG = 15.0
_DECENT_HEADING_DEG = 30.0
_FLIP_HEADING_DEG = 135.0
# PCA covariance proxy that the FSM uses for LOCK_ALIGN -> TRACK_ACTIVE (m^2)
_LOCK_PERP_EIG_M2 = 1.0


@dataclass
class HealthMetrics:
    """单次运行的健康指标汇总（纯数据，无 I/O）。"""

    case_name: str
    deployment_mode: bool
    duration_s: float
    total_steps: int
    # --- Heading quality ---
    mean_heading_error_deg: float
    median_heading_error_deg: float
    final_heading_error_deg: float
    good_ratio: float
    flip_count: int
    heading_oscillations: int
    # --- FSM / mode ---
    mode_fraction: Dict[str, float]
    track_active_fraction: float
    mode_switches: int
    # --- Guidance source contribution ---
    source_fraction: Dict[str, float]
    sonar_contribution: float
    magnetic_contribution: float
    # --- Signal & fit ---
    mean_snr_db: float
    total_peaks: int
    peak_rate_hz: float
    mean_fit_residual_m: float
    lock_grade_fraction: float
    # --- Navigation ---
    mean_cross_track_m: float
    max_cross_track_m: float
    # --- Confidence / robustness ---
    mean_confidence: float
    safe_lock_fraction: float
    mean_vector_consistency: float
    # --- Burial inversion (Phase 4 placeholder; NaN until implemented) ---
    burial_inversion_mae_m: float
    # --- Task-level health indicators ---
    mean_vehicle_heading_error_deg: float = float("nan")
    track_mean_heading_error_deg: float = float("nan")
    track_mean_vehicle_heading_error_deg: float = float("nan")
    track_mean_cross_track_m: float = float("nan")
    median_cross_track_m: float = float("nan")
    p90_cross_track_m: float = float("nan")
    final_cross_track_m: float = float("nan")
    route_completion_ratio: float = float("nan")
    final_route_progress_m: float = float("nan")
    route_length_m: float = float("nan")
    final_route_distance_m: float = float("nan")
    endpoint_goal_enabled: float = 0.0
    endpoint_completed: float = 0.0
    magnetic_path_observation_fraction: float = 0.0
    magnetic_path_mean_axis_error_deg: float = float("nan")
    magnetic_path_mean_position_error_m: float = float("nan")
    magnetic_path_mean_cross_track_offset_m: float = float("nan")
    magnetic_phase_observation_fraction: float = 0.0
    magnetic_phase_mean_axis_error_deg: float = float("nan")
    magnetic_phase_mean_position_error_m: float = float("nan")
    magnetic_phase_mean_amplitude_m: float = float("nan")
    magnetic_phase_detector_emit_fraction: float = 0.0
    magnetic_phase_detector_reject_no_pair_fraction: float = 0.0
    magnetic_phase_detector_reject_offset_fraction: float = 0.0
    magnetic_phase_detector_reject_duration_fraction: float = 0.0
    magnetic_phase_detector_reject_axis_fraction: float = 0.0
    magnetic_phase_detector_waiting_fraction: float = 0.0
    magnetic_phase_detector_mean_candidate_duration_s: float = float("nan")
    magnetic_phase_detector_mean_axis_delta_deg: float = float("nan")
    magnetic_lookahead_fraction: float = 0.0
    magnetic_lookahead_mean_axis_error_deg: float = float("nan")
    magnetic_lookahead_mean_position_error_m: float = float("nan")
    magnetic_lookahead_mean_age_s: float = float("nan")
    magnetic_lookahead_feed_allowed_fraction: float = 0.0
    magnetic_lookahead_feed_reject_age_fraction: float = 0.0
    magnetic_lookahead_feed_reject_phase_age_fraction: float = 0.0
    magnetic_lookahead_feed_reject_residual_fraction: float = 0.0
    magnetic_lookahead_feed_reject_heading_fraction: float = 0.0
    magnetic_lookahead_feed_reject_innovation_fraction: float = 0.0
    magnetic_lookahead_feed_mean_phase_age_s: float = float("nan")
    magnetic_lookahead_feed_mean_innovation_m: float = float("nan")
    magnetic_lookahead_feed_mean_axis_delta_deg: float = float("nan")
    magnetic_lookahead_feed_mean_local_residual_m: float = float("nan")
    shadow_axis_hypothesis_fraction: float = 0.0
    shadow_axis_mean_score: float = float("nan")
    shadow_axis_mean_margin: float = float("nan")
    shadow_axis_positive_fraction: float = 0.0
    shadow_axis_mean_age_s: float = float("nan")
    shadow_axis_validation_pass_fraction: float = 0.0
    shadow_axis_validation_reject_no_hypothesis_fraction: float = 0.0
    shadow_axis_validation_reject_insufficient_candidates_fraction: float = 0.0
    shadow_axis_validation_reject_low_score_fraction: float = 0.0
    shadow_axis_validation_reject_low_margin_fraction: float = 0.0
    shadow_axis_validation_reject_stale_age_fraction: float = 0.0
    shadow_axis_validation_reject_selector_expired_fraction: float = 0.0
    shadow_axis_validation_mean_score_deficit: float = float("nan")
    shadow_axis_validation_mean_margin_deficit: float = float("nan")
    shadow_axis_validation_mean_age_over_s: float = float("nan")
    shadow_axis_supply_fraction: float = 0.0
    shadow_axis_validation_fraction: float = 0.0
    shadow_axis_selection_fraction: float = 0.0
    shadow_axis_consumption_fraction: float = 0.0
    shadow_axis_dual_gate_active_fraction: float = 0.0
    shadow_axis_dual_gate_pass_fraction: float = 0.0
    shadow_axis_dual_gate_reject_validation_fraction: float = 0.0
    shadow_axis_dual_gate_reject_feed_fraction: float = 0.0
    zigzag_probe_active_fraction: float = 0.0
    zigzag_probe_cycle_count: int = 0
    zigzag_probe_leg_flip_count: int = 0
    zigzag_probe_magnetic_crossing_count: int = 0
    zigzag_probe_magnetic_crossings_per_cycle: float = 0.0
    magnetic_crossing_probe_forced_flip_count: int = 0
    magnetic_crossing_probe_missed_count: int = 0
    magnetic_crossing_probe_mean_wait_s: float = float("nan")
    zigzag_probe_mean_cycle_duration_s: float = float("nan")
    zigzag_probe_mean_peak_abs_cross_track_m: float = float("nan")
    zigzag_probe_phase_events_per_cycle: float = 0.0
    zigzag_probe_mean_abs_field_ratio: float = float("nan")
    zigzag_probe_mean_abs_b_perp_nt: float = float("nan")
    zigzag_probe_burial_coverage: float = 0.0
    zigzag_probe_burial_mae_m: float = float("nan")
    zigzag_probe_cycle_burial_coverage: float = 0.0
    zigzag_probe_cycle_burial_mae_m: float = float("nan")
    zigzag_probe_cycle_burial_mean_sigma_m: float = float("nan")
    zigzag_probe_cycle_burial_mean_quality: float = float("nan")
    shadow_hypothesis_mean_supply_score: float = float("nan")
    shadow_hypothesis_mean_selection_score: float = float("nan")
    shadow_hypothesis_mean_consumption_score: float = float("nan")
    shadow_hypothesis_mean_readiness_score: float = float("nan")
    shadow_hypothesis_bottleneck_supply_fraction: float = 0.0
    shadow_hypothesis_bottleneck_selection_fraction: float = 0.0
    shadow_hypothesis_bottleneck_consumption_fraction: float = 0.0
    burial_inversion_coverage: float = 0.0
    # Per-frame heading error array (kept for plotting; excluded from JSON)
    heading_errors_deg: np.ndarray = field(default_factory=lambda: np.empty(0))


def _heading_error_series(record: RunRecord) -> np.ndarray:
    """每帧 fused-heading 相对真值的绝对误差（无估计处为 NaN）。"""
    fused = record["fused_heading_deg"]
    true = record["true_heading_deg"]
    errors = np.full(fused.shape, np.nan)
    for i in range(fused.size):
        if np.isnan(fused[i]):
            continue
        errors[i] = abs(smallest_angle_error_deg(fused[i], true[i]))
    return errors


def _fraction_table(labels: List[str]) -> Dict[str, float]:
    """把离散标签序列折算成占比字典。"""
    total = max(len(labels), 1)
    out: Dict[str, float] = {}
    for label in labels:
        out[label] = out.get(label, 0.0) + 1.0 / total
    return out


def compute_health_metrics(record: RunRecord) -> HealthMetrics:
    """从 :class:`RunRecord` 计算全部健康指标（纯函数）。"""
    t = record["time_s"]
    duration_s = float(t[-1]) if t.size else 0.0
    n = record.n_steps

    heading_errors = _heading_error_series(record)
    valid_errors = heading_errors[~np.isnan(heading_errors)]
    mean_err = float(np.mean(valid_errors)) if valid_errors.size else float("nan")
    median_err = float(np.median(valid_errors)) if valid_errors.size else float("nan")
    final_err = float(valid_errors[-1]) if valid_errors.size else float("nan")
    good_ratio = float(np.mean(valid_errors < _GOOD_HEADING_DEG)) if valid_errors.size else 0.0
    flip_count = int(np.sum(valid_errors > _FLIP_HEADING_DEG)) if valid_errors.size else 0

    vehicle_heading_errors = np.array([
        abs(smallest_angle_error_deg(heading, true_heading))
        for heading, true_heading in zip(record["heading_deg"], record["true_heading_deg"])
    ], dtype=float)
    mean_vehicle_err = float(np.mean(vehicle_heading_errors)) if vehicle_heading_errors.size else float("nan")

    # Oscillations: consecutive valid fused-heading samples jumping > 30 deg.
    valid_idx = np.where(~np.isnan(record["fused_heading_deg"]))[0]
    oscillations = 0
    fused = record["fused_heading_deg"]
    for k in range(1, valid_idx.size):
        i0, i1 = valid_idx[k - 1], valid_idx[k]
        if abs(smallest_angle_error_deg(fused[i0], fused[i1])) > _DECENT_HEADING_DEG:
            oscillations += 1

    mode_fraction = _fraction_table(record.modes)
    track_fraction = mode_fraction.get("track", 0.0)
    track_mask = np.asarray([mode == "track" for mode in record.modes], dtype=bool)
    mode_switches = sum(1 for i in range(1, len(record.modes)) if record.modes[i] != record.modes[i - 1])

    source_fraction = _fraction_table(record.sources)
    sonar_contribution = sum(v for k, v in source_fraction.items() if k.startswith("SONAR"))
    magnetic_contribution = sum(v for k, v in source_fraction.items() if k.startswith("MAGNETIC"))

    snr = record["snr_db"]
    valid_snr = snr[~np.isnan(snr)]
    mean_snr = float(np.mean(valid_snr)) if valid_snr.size else float("nan")

    total_peaks = int(np.nansum(record["peak_detected"]))
    peak_rate = total_peaks / duration_s if duration_s > 0 else 0.0

    residual = record["fit_residual_m"]
    finite_residual = residual[np.isfinite(residual)]
    mean_residual = float(np.mean(finite_residual)) if finite_residual.size else float("nan")

    perp_eig = record["fit_perp_eig_m2"]
    lock_grade_fraction = float(np.mean(perp_eig[~np.isnan(perp_eig)] < _LOCK_PERP_EIG_M2)) if np.any(~np.isnan(perp_eig)) else 0.0

    cross_track = np.hypot(record["pos_x_m"] - record["true_nearest_x_m"],
                           record["pos_y_m"] - record["true_nearest_y_m"])
    mean_cross_track = float(np.mean(cross_track)) if cross_track.size else float("nan")
    max_cross_track = float(np.max(cross_track)) if cross_track.size else float("nan")
    median_cross_track = float(np.median(cross_track)) if cross_track.size else float("nan")
    p90_cross_track = float(np.percentile(cross_track, 90.0)) if cross_track.size else float("nan")
    final_cross_track = float(cross_track[-1]) if cross_track.size else float("nan")

    track_heading_errors = heading_errors[track_mask] if track_mask.size == heading_errors.size else np.empty(0)
    valid_track_heading_errors = track_heading_errors[np.isfinite(track_heading_errors)]
    track_mean_heading_error = (
        float(np.mean(valid_track_heading_errors)) if valid_track_heading_errors.size else float("nan")
    )
    track_mean_vehicle_heading_error = (
        float(np.mean(vehicle_heading_errors[track_mask]))
        if track_mask.size == vehicle_heading_errors.size and np.any(track_mask)
        else float("nan")
    )
    track_mean_cross_track = (
        float(np.mean(cross_track[track_mask]))
        if track_mask.size == cross_track.size and np.any(track_mask)
        else float("nan")
    )

    mean_conf = float(np.mean(record["confidence"])) if n else float("nan")
    safe_lock_fraction = float(np.mean(record["safe_lock_active"])) if n else 0.0
    vec = record["vector_consistency"]
    valid_vec = vec[~np.isnan(vec)]
    mean_vec = float(np.mean(valid_vec)) if valid_vec.size else 0.0

    # Burial inversion error: only over frames that carry an estimate (the
    # inverter emits NaN until it has warmed up on near-crossing samples).
    est_burial = record["estimated_burial_depth_m"]
    true_burial = record["true_burial_depth_m"]
    burial_valid = ~np.isnan(est_burial)
    burial_mae = (
        float(np.mean(np.abs(est_burial[burial_valid] - true_burial[burial_valid])))
        if np.any(burial_valid)
        else float("nan")
    )
    burial_coverage = float(np.mean(burial_valid)) if est_burial.size else 0.0

    magnetic_path_valid = record["magnetic_path_observation_valid"] > 0.5
    magnetic_path_fraction = float(np.mean(magnetic_path_valid)) if n else 0.0
    magnetic_path_heading = record["magnetic_path_heading_deg"]
    magnetic_path_axis_errors = []
    for estimated_heading, true_heading in zip(magnetic_path_heading[magnetic_path_valid], record["true_heading_deg"][magnetic_path_valid]):
        directional_error = abs(smallest_angle_error_deg(estimated_heading, true_heading))
        magnetic_path_axis_errors.append(min(directional_error, abs(180.0 - directional_error)))
    magnetic_path_mean_axis_error = (
        float(np.mean(magnetic_path_axis_errors)) if magnetic_path_axis_errors else float("nan")
    )
    magnetic_path_position_error = np.hypot(
        record["magnetic_path_x_m"][magnetic_path_valid] - record["true_nearest_x_m"][magnetic_path_valid],
        record["magnetic_path_y_m"][magnetic_path_valid] - record["true_nearest_y_m"][magnetic_path_valid],
    )
    magnetic_path_mean_position_error = (
        float(np.mean(magnetic_path_position_error)) if magnetic_path_position_error.size else float("nan")
    )
    magnetic_path_offsets = np.abs(record["magnetic_path_cross_track_offset_m"][magnetic_path_valid])
    magnetic_path_mean_offset = float(np.mean(magnetic_path_offsets)) if magnetic_path_offsets.size else float("nan")

    magnetic_phase_valid = record["magnetic_phase_observation_valid"] > 0.5
    magnetic_phase_fraction = float(np.mean(magnetic_phase_valid)) if n else 0.0
    magnetic_phase_heading = record["magnetic_phase_heading_deg"]
    magnetic_phase_axis_errors = []
    for estimated_heading, true_heading in zip(magnetic_phase_heading[magnetic_phase_valid], record["true_heading_deg"][magnetic_phase_valid]):
        directional_error = abs(smallest_angle_error_deg(estimated_heading, true_heading))
        magnetic_phase_axis_errors.append(min(directional_error, abs(180.0 - directional_error)))
    magnetic_phase_mean_axis_error = (
        float(np.mean(magnetic_phase_axis_errors)) if magnetic_phase_axis_errors else float("nan")
    )
    magnetic_phase_position_error = np.hypot(
        record["magnetic_phase_x_m"][magnetic_phase_valid] - record["true_nearest_x_m"][magnetic_phase_valid],
        record["magnetic_phase_y_m"][magnetic_phase_valid] - record["true_nearest_y_m"][magnetic_phase_valid],
    )
    magnetic_phase_mean_position_error = (
        float(np.mean(magnetic_phase_position_error)) if magnetic_phase_position_error.size else float("nan")
    )
    magnetic_phase_amplitude = record["magnetic_phase_amplitude_m"][magnetic_phase_valid]
    magnetic_phase_mean_amplitude = (
        float(np.mean(magnetic_phase_amplitude)) if magnetic_phase_amplitude.size else float("nan")
    )

    magnetic_lookahead_valid = record["magnetic_lookahead_valid"] > 0.5
    magnetic_lookahead_fraction = float(np.mean(magnetic_lookahead_valid)) if n else 0.0
    magnetic_lookahead_heading = record["magnetic_lookahead_heading_deg"]
    magnetic_lookahead_axis_errors = []
    for estimated_heading, true_heading in zip(magnetic_lookahead_heading[magnetic_lookahead_valid], record["true_heading_deg"][magnetic_lookahead_valid]):
        directional_error = abs(smallest_angle_error_deg(estimated_heading, true_heading))
        magnetic_lookahead_axis_errors.append(min(directional_error, abs(180.0 - directional_error)))
    magnetic_lookahead_mean_axis_error = (
        float(np.mean(magnetic_lookahead_axis_errors)) if magnetic_lookahead_axis_errors else float("nan")
    )
    magnetic_lookahead_position_error = np.hypot(
        record["magnetic_lookahead_cable_x_m"][magnetic_lookahead_valid] - record["true_nearest_x_m"][magnetic_lookahead_valid],
        record["magnetic_lookahead_cable_y_m"][magnetic_lookahead_valid] - record["true_nearest_y_m"][magnetic_lookahead_valid],
    )
    magnetic_lookahead_mean_position_error = (
        float(np.mean(magnetic_lookahead_position_error)) if magnetic_lookahead_position_error.size else float("nan")
    )
    magnetic_lookahead_age = record["magnetic_lookahead_age_s"][magnetic_lookahead_valid]
    magnetic_lookahead_mean_age = (
        float(np.mean(magnetic_lookahead_age)) if magnetic_lookahead_age.size else float("nan")
    )
    lookahead_feed_reason = record["magnetic_lookahead_feed_reason_code"]
    lookahead_feed_allowed = record["magnetic_lookahead_feed_allowed"] > 0.5
    lookahead_feed_denominator = int(np.sum(magnetic_lookahead_valid))
    if lookahead_feed_denominator:
        magnetic_lookahead_feed_allowed_fraction = float(np.sum(lookahead_feed_allowed & magnetic_lookahead_valid) / lookahead_feed_denominator)
        magnetic_lookahead_feed_reject_age_fraction = float(np.sum((lookahead_feed_reason == 5.0) & magnetic_lookahead_valid) / lookahead_feed_denominator)
        magnetic_lookahead_feed_reject_phase_age_fraction = float(np.sum((lookahead_feed_reason == 6.0) & magnetic_lookahead_valid) / lookahead_feed_denominator)
        magnetic_lookahead_feed_reject_residual_fraction = float(np.sum((lookahead_feed_reason == 7.0) & magnetic_lookahead_valid) / lookahead_feed_denominator)
        magnetic_lookahead_feed_reject_heading_fraction = float(np.sum((lookahead_feed_reason == 8.0) & magnetic_lookahead_valid) / lookahead_feed_denominator)
        magnetic_lookahead_feed_reject_innovation_fraction = float(np.sum((lookahead_feed_reason == 9.0) & magnetic_lookahead_valid) / lookahead_feed_denominator)
    else:
        magnetic_lookahead_feed_allowed_fraction = 0.0
        magnetic_lookahead_feed_reject_age_fraction = 0.0
        magnetic_lookahead_feed_reject_phase_age_fraction = 0.0
        magnetic_lookahead_feed_reject_residual_fraction = 0.0
        magnetic_lookahead_feed_reject_heading_fraction = 0.0
        magnetic_lookahead_feed_reject_innovation_fraction = 0.0

    def _finite_mean(name: str) -> float:
        values = record[name][magnetic_lookahead_valid]
        values = values[np.isfinite(values)]
        return float(np.mean(values)) if values.size else float("nan")

    def _finite_mean_for_mask(name: str, mask: np.ndarray) -> float:
        values = record[name][mask]
        values = values[np.isfinite(values)]
        return float(np.mean(values)) if values.size else float("nan")

    magnetic_lookahead_feed_mean_phase_age = _finite_mean("magnetic_lookahead_feed_phase_age_s")
    magnetic_lookahead_feed_mean_innovation = _finite_mean("magnetic_lookahead_feed_innovation_m")
    magnetic_lookahead_feed_mean_axis_delta = _finite_mean("magnetic_lookahead_feed_axis_delta_deg")
    magnetic_lookahead_feed_mean_local_residual = _finite_mean("magnetic_lookahead_feed_local_residual_m")

    zigzag_probe_active = record["zigzag_probe_active"] > 0.5
    zigzag_probe_active_fraction = float(np.mean(zigzag_probe_active)) if n else 0.0
    zigzag_probe_leg_flip_count = int(np.nansum(record["zigzag_probe_leg_flip_event"]))
    zigzag_probe_magnetic_crossing_count = int(np.nansum(record["zigzag_probe_magnetic_crossing_event"]))
    magnetic_crossing_probe_forced_flip_count = int(np.nansum(record["magnetic_crossing_probe_forced_flip"]))
    missed_values = record["magnetic_crossing_probe_missed_count"]
    missed_values = missed_values[np.isfinite(missed_values)]
    magnetic_crossing_probe_missed_count = int(np.max(missed_values)) if missed_values.size else 0
    magnetic_crossing_wait = record["magnetic_crossing_probe_wait_s"][zigzag_probe_active]
    magnetic_crossing_wait = magnetic_crossing_wait[np.isfinite(magnetic_crossing_wait)]
    magnetic_crossing_probe_mean_wait_s = (
        float(np.mean(magnetic_crossing_wait)) if magnetic_crossing_wait.size else float("nan")
    )
    cycle_ids = record["zigzag_probe_cycle_id"][zigzag_probe_active]
    finite_cycle_ids = cycle_ids[np.isfinite(cycle_ids)]
    zigzag_probe_cycle_count = int(np.max(finite_cycle_ids) + 1) if finite_cycle_ids.size else 0
    zigzag_probe_magnetic_crossings_per_cycle = (
        float(zigzag_probe_magnetic_crossing_count / max(zigzag_probe_cycle_count, 1))
        if zigzag_probe_cycle_count > 0
        else 0.0
    )
    cycle_duration_at_flip = record["zigzag_probe_last_cycle_duration_s"][
        record["zigzag_probe_leg_flip_event"] > 0.5
    ]
    cycle_duration_at_flip = cycle_duration_at_flip[np.isfinite(cycle_duration_at_flip)]
    zigzag_probe_mean_cycle_duration = (
        float(np.mean(cycle_duration_at_flip)) if cycle_duration_at_flip.size else float("nan")
    )
    peak_abs_xt = record["zigzag_probe_cycle_peak_abs_cross_track_m"][zigzag_probe_active]
    peak_abs_xt = peak_abs_xt[np.isfinite(peak_abs_xt)]
    zigzag_probe_mean_peak_abs_cross_track = (
        float(np.mean(peak_abs_xt)) if peak_abs_xt.size else float("nan")
    )
    phase_events_per_cycle = (
        float(np.sum(record["magnetic_phase_observation_valid"][zigzag_probe_active] > 0.5)
              / max(zigzag_probe_cycle_count, 1))
        if np.any(zigzag_probe_active)
        else 0.0
    )
    phase_detector_codes = record["magnetic_phase_detector_reason_code"]
    phase_detector_active = phase_detector_codes > 0.5
    phase_detector_denominator = max(int(np.sum(phase_detector_active)), 1)
    magnetic_phase_detector_emit_fraction = float(
        np.sum(phase_detector_codes == 1.0) / phase_detector_denominator
    )
    magnetic_phase_detector_reject_no_pair_fraction = float(
        np.sum(np.isin(phase_detector_codes, [3.0, 4.0, 5.0])) / phase_detector_denominator
    )
    magnetic_phase_detector_reject_offset_fraction = float(
        np.sum(phase_detector_codes == 6.0) / phase_detector_denominator
    )
    magnetic_phase_detector_reject_duration_fraction = float(
        np.sum(np.isin(phase_detector_codes, [7.0, 8.0])) / phase_detector_denominator
    )
    magnetic_phase_detector_reject_axis_fraction = float(
        np.sum(phase_detector_codes == 9.0) / phase_detector_denominator
    )
    magnetic_phase_detector_waiting_fraction = float(
        np.sum(phase_detector_codes == 10.0) / phase_detector_denominator
    )
    magnetic_phase_detector_mean_candidate_duration = _finite_mean_for_mask(
        "magnetic_phase_detector_candidate_duration_s",
        phase_detector_active,
    )
    magnetic_phase_detector_mean_axis_delta = _finite_mean_for_mask(
        "magnetic_phase_detector_axis_delta_deg",
        phase_detector_active,
    )
    field_ratio = record["zigzag_probe_field_ratio"][zigzag_probe_active]
    field_ratio = field_ratio[np.isfinite(field_ratio)]
    zigzag_probe_mean_abs_field_ratio = (
        float(np.mean(np.abs(field_ratio))) if field_ratio.size else float("nan")
    )
    b_perp = record["zigzag_probe_b_perp_nt"][zigzag_probe_active]
    b_perp = b_perp[np.isfinite(b_perp)]
    zigzag_probe_mean_abs_b_perp = float(np.mean(np.abs(b_perp))) if b_perp.size else float("nan")
    burial_probe_valid = (record["zigzag_probe_burial_valid"] > 0.5) & zigzag_probe_active
    zigzag_probe_burial_coverage = (
        float(np.sum(burial_probe_valid) / max(np.sum(zigzag_probe_active), 1))
        if np.any(zigzag_probe_active)
        else 0.0
    )
    probe_burial_errors = np.abs(record["zigzag_probe_burial_error_m"][burial_probe_valid])
    probe_burial_errors = probe_burial_errors[np.isfinite(probe_burial_errors)]
    zigzag_probe_burial_mae = (
        float(np.mean(probe_burial_errors)) if probe_burial_errors.size else float("nan")
    )
    cycle_burial_valid = (record["zigzag_probe_cycle_burial_valid"] > 0.5) & zigzag_probe_active
    zigzag_probe_cycle_burial_coverage = (
        float(np.sum(cycle_burial_valid) / max(np.sum(zigzag_probe_active), 1))
        if np.any(zigzag_probe_active)
        else 0.0
    )
    cycle_burial_errors = np.abs(record["zigzag_probe_cycle_burial_error_m"][cycle_burial_valid])
    cycle_burial_errors = cycle_burial_errors[np.isfinite(cycle_burial_errors)]
    zigzag_probe_cycle_burial_mae = (
        float(np.mean(cycle_burial_errors)) if cycle_burial_errors.size else float("nan")
    )
    cycle_burial_sigma = record["zigzag_probe_cycle_burial_sigma_m"][cycle_burial_valid]
    cycle_burial_sigma = cycle_burial_sigma[np.isfinite(cycle_burial_sigma)]
    zigzag_probe_cycle_burial_mean_sigma = (
        float(np.mean(cycle_burial_sigma)) if cycle_burial_sigma.size else float("nan")
    )
    cycle_burial_quality = record["zigzag_probe_cycle_burial_quality"][cycle_burial_valid]
    cycle_burial_quality = cycle_burial_quality[np.isfinite(cycle_burial_quality)]
    zigzag_probe_cycle_burial_mean_quality = (
        float(np.mean(cycle_burial_quality)) if cycle_burial_quality.size else float("nan")
    )
    shadow_mask = track_mask if np.any(track_mask) else np.ones(n, dtype=bool)
    shadow_mean_supply = _finite_mean_for_mask("shadow_hypothesis_supply_score", shadow_mask)
    shadow_mean_selection = _finite_mean_for_mask("shadow_hypothesis_selection_score", shadow_mask)
    shadow_mean_consumption = _finite_mean_for_mask("shadow_hypothesis_consumption_score", shadow_mask)
    shadow_mean_readiness = _finite_mean_for_mask("shadow_hypothesis_readiness_score", shadow_mask)
    bottleneck_codes = record["shadow_hypothesis_bottleneck_code"][shadow_mask]
    bottleneck_codes = bottleneck_codes[np.isfinite(bottleneck_codes)]
    bottleneck_denominator = max(bottleneck_codes.size, 1)
    bottleneck_supply_fraction = float(np.sum(bottleneck_codes == 2.0) / bottleneck_denominator)
    bottleneck_selection_fraction = float(np.sum(bottleneck_codes == 3.0) / bottleneck_denominator)
    bottleneck_consumption_fraction = float(np.sum(bottleneck_codes == 4.0) / bottleneck_denominator)
    shadow_axis_valid = record["shadow_axis_hypothesis_valid"] > 0.5
    shadow_axis_hypothesis_fraction = float(np.mean(shadow_axis_valid)) if n else 0.0
    shadow_axis_scores = record["shadow_axis_selected_score"][shadow_axis_valid]
    shadow_axis_scores = shadow_axis_scores[np.isfinite(shadow_axis_scores)]
    shadow_axis_mean_score = float(np.mean(shadow_axis_scores)) if shadow_axis_scores.size else float("nan")
    shadow_axis_margins = record["shadow_axis_score_margin"][shadow_axis_valid]
    shadow_axis_margins = shadow_axis_margins[np.isfinite(shadow_axis_margins)]
    shadow_axis_mean_margin = float(np.mean(shadow_axis_margins)) if shadow_axis_margins.size else float("nan")
    shadow_axis_signs = record["shadow_axis_selected_sign"][shadow_axis_valid]
    shadow_axis_signs = shadow_axis_signs[np.isfinite(shadow_axis_signs)]
    shadow_axis_positive_fraction = (
        float(np.mean(shadow_axis_signs > 0.0)) if shadow_axis_signs.size else 0.0
    )
    shadow_axis_ages = record["shadow_axis_age_s"][shadow_axis_valid]
    shadow_axis_ages = shadow_axis_ages[np.isfinite(shadow_axis_ages)]
    shadow_axis_mean_age = float(np.mean(shadow_axis_ages)) if shadow_axis_ages.size else float("nan")
    shadow_axis_validation_codes = record["shadow_axis_validation_reason_code"]
    validation_active = shadow_axis_validation_codes > 0.5
    validation_denominator = max(int(np.sum(validation_active)), 1)
    shadow_axis_validation_pass_fraction = float(
        np.sum(shadow_axis_validation_codes == 1.0) / validation_denominator
    )
    shadow_axis_validation_reject_no_hypothesis_fraction = float(
        np.sum(shadow_axis_validation_codes == 2.0) / validation_denominator
    )
    shadow_axis_validation_reject_insufficient_candidates_fraction = float(
        np.sum(shadow_axis_validation_codes == 3.0) / validation_denominator
    )
    shadow_axis_validation_reject_low_score_fraction = float(
        np.sum(shadow_axis_validation_codes == 4.0) / validation_denominator
    )
    shadow_axis_validation_reject_low_margin_fraction = float(
        np.sum(shadow_axis_validation_codes == 5.0) / validation_denominator
    )
    shadow_axis_validation_reject_stale_age_fraction = float(
        np.sum(shadow_axis_validation_codes == 6.0) / validation_denominator
    )
    shadow_axis_validation_reject_selector_expired_fraction = float(
        np.sum(shadow_axis_validation_codes == 7.0) / validation_denominator
    )

    def _finite_mean_for_validation(name: str) -> float:
        values = record[name][validation_active]
        values = values[np.isfinite(values)]
        return float(np.mean(values)) if values.size else float("nan")

    shadow_axis_validation_mean_score_deficit = _finite_mean_for_validation("shadow_axis_validation_score_deficit")
    shadow_axis_validation_mean_margin_deficit = _finite_mean_for_validation("shadow_axis_validation_margin_deficit")
    shadow_axis_validation_mean_age_over = _finite_mean_for_validation("shadow_axis_validation_age_over_s")

    shadow_axis_supply_fraction = (
        shadow_axis_validation_reject_no_hypothesis_fraction
        + shadow_axis_validation_reject_insufficient_candidates_fraction
    )
    shadow_axis_validation_fraction = (
        shadow_axis_validation_reject_stale_age_fraction
        + shadow_axis_validation_reject_selector_expired_fraction
    )
    shadow_axis_selection_fraction = (
        shadow_axis_validation_reject_low_score_fraction
        + shadow_axis_validation_reject_low_margin_fraction
    )
    dual_gate_active_mask = record["shadow_axis_dual_gate_enabled"] > 0.5
    dual_gate_active_count = int(np.sum(dual_gate_active_mask))
    if dual_gate_active_count > 0:
        dual_gate_reason = record["shadow_axis_dual_gate_reason_code"][dual_gate_active_mask]
        shadow_axis_dual_gate_active_fraction = (
            dual_gate_active_count / n if n else 0.0
        )
        shadow_axis_dual_gate_pass_fraction = float(np.sum(dual_gate_reason == 1.0) / dual_gate_active_count)
        shadow_axis_dual_gate_reject_validation_fraction = float(np.sum(dual_gate_reason == 2.0) / dual_gate_active_count)
        shadow_axis_dual_gate_reject_feed_fraction = float(np.sum(dual_gate_reason == 3.0) / dual_gate_active_count)
        shadow_axis_consumption_fraction = (
            shadow_axis_validation_pass_fraction * shadow_axis_dual_gate_reject_feed_fraction
        )
    else:
        shadow_axis_dual_gate_active_fraction = 0.0
        shadow_axis_dual_gate_pass_fraction = 0.0
        shadow_axis_dual_gate_reject_validation_fraction = 0.0
        shadow_axis_dual_gate_reject_feed_fraction = 0.0
        shadow_axis_consumption_fraction = 0.0

    return HealthMetrics(
        case_name=record.case_name,
        deployment_mode=record.deployment_mode,
        duration_s=duration_s,
        total_steps=n,
        mean_heading_error_deg=mean_err,
        median_heading_error_deg=median_err,
        final_heading_error_deg=final_err,
        good_ratio=good_ratio,
        flip_count=flip_count,
        heading_oscillations=oscillations,
        mode_fraction=mode_fraction,
        track_active_fraction=track_fraction,
        mode_switches=mode_switches,
        source_fraction=source_fraction,
        sonar_contribution=sonar_contribution,
        magnetic_contribution=magnetic_contribution,
        mean_snr_db=mean_snr,
        total_peaks=total_peaks,
        peak_rate_hz=peak_rate,
        mean_fit_residual_m=mean_residual,
        lock_grade_fraction=lock_grade_fraction,
        mean_cross_track_m=mean_cross_track,
        max_cross_track_m=max_cross_track,
        mean_confidence=mean_conf,
        safe_lock_fraction=safe_lock_fraction,
        mean_vector_consistency=mean_vec,
        burial_inversion_mae_m=burial_mae,
        mean_vehicle_heading_error_deg=mean_vehicle_err,
        track_mean_heading_error_deg=track_mean_heading_error,
        track_mean_vehicle_heading_error_deg=track_mean_vehicle_heading_error,
        track_mean_cross_track_m=track_mean_cross_track,
        median_cross_track_m=median_cross_track,
        p90_cross_track_m=p90_cross_track,
        final_cross_track_m=final_cross_track,
        route_completion_ratio=float(record.metadata.get("route_completion_ratio", float("nan"))),
        final_route_progress_m=float(record.metadata.get("final_route_progress_m", float("nan"))),
        route_length_m=float(record.metadata.get("route_length_m", float("nan"))),
        final_route_distance_m=float(record.metadata.get("final_route_distance_m", float("nan"))),
        endpoint_goal_enabled=float(record.metadata.get("endpoint_goal_enabled", 0.0)),
        endpoint_completed=float(record.metadata.get("endpoint_completed", 0.0)),
        magnetic_path_observation_fraction=magnetic_path_fraction,
        magnetic_path_mean_axis_error_deg=magnetic_path_mean_axis_error,
        magnetic_path_mean_position_error_m=magnetic_path_mean_position_error,
        magnetic_path_mean_cross_track_offset_m=magnetic_path_mean_offset,
        magnetic_phase_observation_fraction=magnetic_phase_fraction,
        magnetic_phase_mean_axis_error_deg=magnetic_phase_mean_axis_error,
        magnetic_phase_mean_position_error_m=magnetic_phase_mean_position_error,
        magnetic_phase_mean_amplitude_m=magnetic_phase_mean_amplitude,
        magnetic_phase_detector_emit_fraction=magnetic_phase_detector_emit_fraction,
        magnetic_phase_detector_reject_no_pair_fraction=magnetic_phase_detector_reject_no_pair_fraction,
        magnetic_phase_detector_reject_offset_fraction=magnetic_phase_detector_reject_offset_fraction,
        magnetic_phase_detector_reject_duration_fraction=magnetic_phase_detector_reject_duration_fraction,
        magnetic_phase_detector_reject_axis_fraction=magnetic_phase_detector_reject_axis_fraction,
        magnetic_phase_detector_waiting_fraction=magnetic_phase_detector_waiting_fraction,
        magnetic_phase_detector_mean_candidate_duration_s=magnetic_phase_detector_mean_candidate_duration,
        magnetic_phase_detector_mean_axis_delta_deg=magnetic_phase_detector_mean_axis_delta,
        magnetic_lookahead_fraction=magnetic_lookahead_fraction,
        magnetic_lookahead_mean_axis_error_deg=magnetic_lookahead_mean_axis_error,
        magnetic_lookahead_mean_position_error_m=magnetic_lookahead_mean_position_error,
        magnetic_lookahead_mean_age_s=magnetic_lookahead_mean_age,
        magnetic_lookahead_feed_allowed_fraction=magnetic_lookahead_feed_allowed_fraction,
        magnetic_lookahead_feed_reject_age_fraction=magnetic_lookahead_feed_reject_age_fraction,
        magnetic_lookahead_feed_reject_phase_age_fraction=magnetic_lookahead_feed_reject_phase_age_fraction,
        magnetic_lookahead_feed_reject_residual_fraction=magnetic_lookahead_feed_reject_residual_fraction,
        magnetic_lookahead_feed_reject_heading_fraction=magnetic_lookahead_feed_reject_heading_fraction,
        magnetic_lookahead_feed_reject_innovation_fraction=magnetic_lookahead_feed_reject_innovation_fraction,
        magnetic_lookahead_feed_mean_phase_age_s=magnetic_lookahead_feed_mean_phase_age,
        magnetic_lookahead_feed_mean_innovation_m=magnetic_lookahead_feed_mean_innovation,
        magnetic_lookahead_feed_mean_axis_delta_deg=magnetic_lookahead_feed_mean_axis_delta,
        magnetic_lookahead_feed_mean_local_residual_m=magnetic_lookahead_feed_mean_local_residual,
        shadow_axis_hypothesis_fraction=shadow_axis_hypothesis_fraction,
        shadow_axis_mean_score=shadow_axis_mean_score,
        shadow_axis_mean_margin=shadow_axis_mean_margin,
        shadow_axis_positive_fraction=shadow_axis_positive_fraction,
        shadow_axis_mean_age_s=shadow_axis_mean_age,
        shadow_axis_validation_pass_fraction=shadow_axis_validation_pass_fraction,
        shadow_axis_validation_reject_no_hypothesis_fraction=shadow_axis_validation_reject_no_hypothesis_fraction,
        shadow_axis_validation_reject_insufficient_candidates_fraction=(
            shadow_axis_validation_reject_insufficient_candidates_fraction
        ),
        shadow_axis_validation_reject_low_score_fraction=shadow_axis_validation_reject_low_score_fraction,
        shadow_axis_validation_reject_low_margin_fraction=shadow_axis_validation_reject_low_margin_fraction,
        shadow_axis_validation_reject_stale_age_fraction=shadow_axis_validation_reject_stale_age_fraction,
        shadow_axis_validation_reject_selector_expired_fraction=(
            shadow_axis_validation_reject_selector_expired_fraction
        ),
        shadow_axis_validation_mean_score_deficit=shadow_axis_validation_mean_score_deficit,
        shadow_axis_validation_mean_margin_deficit=shadow_axis_validation_mean_margin_deficit,
        shadow_axis_validation_mean_age_over_s=shadow_axis_validation_mean_age_over,
        shadow_axis_supply_fraction=shadow_axis_supply_fraction,
        shadow_axis_validation_fraction=shadow_axis_validation_fraction,
        shadow_axis_selection_fraction=shadow_axis_selection_fraction,
        shadow_axis_consumption_fraction=shadow_axis_consumption_fraction,
        shadow_axis_dual_gate_active_fraction=shadow_axis_dual_gate_active_fraction,
        shadow_axis_dual_gate_pass_fraction=shadow_axis_dual_gate_pass_fraction,
        shadow_axis_dual_gate_reject_validation_fraction=shadow_axis_dual_gate_reject_validation_fraction,
        shadow_axis_dual_gate_reject_feed_fraction=shadow_axis_dual_gate_reject_feed_fraction,
        zigzag_probe_active_fraction=zigzag_probe_active_fraction,
        zigzag_probe_cycle_count=zigzag_probe_cycle_count,
        zigzag_probe_leg_flip_count=zigzag_probe_leg_flip_count,
        zigzag_probe_magnetic_crossing_count=zigzag_probe_magnetic_crossing_count,
        zigzag_probe_magnetic_crossings_per_cycle=zigzag_probe_magnetic_crossings_per_cycle,
        magnetic_crossing_probe_forced_flip_count=magnetic_crossing_probe_forced_flip_count,
        magnetic_crossing_probe_missed_count=magnetic_crossing_probe_missed_count,
        magnetic_crossing_probe_mean_wait_s=magnetic_crossing_probe_mean_wait_s,
        zigzag_probe_mean_cycle_duration_s=zigzag_probe_mean_cycle_duration,
        zigzag_probe_mean_peak_abs_cross_track_m=zigzag_probe_mean_peak_abs_cross_track,
        zigzag_probe_phase_events_per_cycle=phase_events_per_cycle,
        zigzag_probe_mean_abs_field_ratio=zigzag_probe_mean_abs_field_ratio,
        zigzag_probe_mean_abs_b_perp_nt=zigzag_probe_mean_abs_b_perp,
        zigzag_probe_burial_coverage=zigzag_probe_burial_coverage,
        zigzag_probe_burial_mae_m=zigzag_probe_burial_mae,
        zigzag_probe_cycle_burial_coverage=zigzag_probe_cycle_burial_coverage,
        zigzag_probe_cycle_burial_mae_m=zigzag_probe_cycle_burial_mae,
        zigzag_probe_cycle_burial_mean_sigma_m=zigzag_probe_cycle_burial_mean_sigma,
        zigzag_probe_cycle_burial_mean_quality=zigzag_probe_cycle_burial_mean_quality,
        shadow_hypothesis_mean_supply_score=shadow_mean_supply,
        shadow_hypothesis_mean_selection_score=shadow_mean_selection,
        shadow_hypothesis_mean_consumption_score=shadow_mean_consumption,
        shadow_hypothesis_mean_readiness_score=shadow_mean_readiness,
        shadow_hypothesis_bottleneck_supply_fraction=bottleneck_supply_fraction,
        shadow_hypothesis_bottleneck_selection_fraction=bottleneck_selection_fraction,
        shadow_hypothesis_bottleneck_consumption_fraction=bottleneck_consumption_fraction,
        burial_inversion_coverage=burial_coverage,
        heading_errors_deg=heading_errors,
    )


def health_score(metrics: HealthMetrics) -> float:
    """把若干关键指标加权成 0–100 的总分（越高越好）。"""
    score = 0.0
    # Fused-heading accuracy: diagnostic only; strong curves can still be
    # controllable even when the perception heading lags the local tangent.
    if not np.isnan(metrics.mean_heading_error_deg):
        score += max(0.0, 35.0 - metrics.mean_heading_error_deg) / 35.0 * 10.0
    score += metrics.good_ratio * 5.0
    # Closed-loop task quality: prioritize TRACK-phase vehicle behavior over
    # all-run perception angle means.
    if not np.isnan(metrics.track_mean_vehicle_heading_error_deg):
        score += max(0.0, 25.0 - metrics.track_mean_vehicle_heading_error_deg) / 25.0 * 10.0
    if not np.isnan(metrics.track_mean_cross_track_m):
        score += max(0.0, 12.0 - metrics.track_mean_cross_track_m) / 12.0 * 20.0
    score += min(metrics.track_active_fraction / 0.30, 1.0) * 10.0
    score += max(0.0, (30.0 - metrics.mode_switches)) / 24.0 * 10.0
    if not np.isnan(metrics.final_cross_track_m):
        score += max(0.0, 18.0 - metrics.final_cross_track_m) / 18.0 * 10.0
    if metrics.endpoint_goal_enabled >= 0.5 and not np.isnan(metrics.route_completion_ratio):
        route_score = max(0.0, min(metrics.route_completion_ratio, 1.0)) * 20.0
        endpoint_bonus = 5.0 if metrics.endpoint_completed >= 0.5 else 0.0
        score += min(25.0, route_score + endpoint_bonus)
    elif metrics.endpoint_goal_enabled < 0.5:
        score += 25.0
    return float(max(0.0, min(100.0, score)))


def metrics_to_dict(metrics: HealthMetrics) -> Dict[str, object]:
    """把指标序列化为 JSON 友好的字典（剔除大数组）。"""
    out: Dict[str, object] = {}
    for key, value in metrics.__dict__.items():
        if key == "heading_errors_deg":
            continue
        if isinstance(value, np.floating):
            out[key] = float(value)
        elif isinstance(value, np.integer):
            out[key] = int(value)
        elif isinstance(value, dict):
            out[key] = {k: float(v) for k, v in value.items()}
        else:
            out[key] = value
    out["health_score"] = health_score(metrics)
    return out


# Progress fields: (label, getter, higher_is_better, unit, accept_target)
_PROGRESS_FIELDS = (
    ("health", lambda h, b: (health_score(h), b.health), True, "/100", 90.0),
    ("mean_err", lambda h, b: (h.mean_heading_error_deg, b.mean_heading_error_deg),
     False, "deg", 15.0),
    ("track_pct", lambda h, b: (h.track_active_fraction * 100.0, b.track_active_fraction * 100.0),
     True, "%", 30.0),
    ("switches", lambda h, b: (float(h.mode_switches), float(b.mode_switches)),
     False, "", 6.0),
)


@dataclass
class ProgressDelta:
    """单场景 before→after 进度对照（每个关键指标的修复前/后值与改善量）。"""

    case_name: str
    # field -> (before, after, delta, higher_is_better, unit, accept_target)
    fields: Dict[str, tuple]

    def improved(self, field: str) -> bool:
        before, after, delta, higher_is_better, _, _ = self.fields[field]
        return delta > 0 if higher_is_better else delta < 0


def compare_to_baseline(current: HealthMetrics, baseline: MilestoneMetrics) -> ProgressDelta:
    """把当前运行指标与固化基线逐字段对照（纯函数，供图/报告共用）。"""
    fields: Dict[str, tuple] = {}
    for name, getter, higher_is_better, unit, target in _PROGRESS_FIELDS:
        after, before = getter(current, baseline)
        delta = after - before
        fields[name] = (before, after, delta, higher_is_better, unit, target)
    return ProgressDelta(case_name=current.case_name, fields=fields)
