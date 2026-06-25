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
