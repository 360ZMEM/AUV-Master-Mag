"""Comprehensive diagnostic + health metrics report for AUV cable tracking.

Generates:
1. Terminal health metrics summary
2. Multi-panel matplotlib figure saved to PNG
3. Markdown report saved to tools/health_report_<case>.md
4. Auto-analysis of root causes and recommended actions

Run with: python tools/diagnose_heading_error.py
"""

import copy
import sys
import json
from datetime import datetime
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

import numpy as np
from auv_mag_tracking.config import build_default_scenarios
from auv_mag_tracking.main_viz import AuvCableTrackingSimulation
from auv_mag_tracking.math_utils import smallest_angle_error_deg


def run_diagnostic(case_name: str = "case1", max_steps: int = 4000) -> dict:
    scenario = build_default_scenarios()[case_name]
    scenario = copy.deepcopy(scenario)
    scenario.tracking.use_nominal_route_prior = False

    sim = AuvCableTrackingSimulation(scenario)

    initial_xy = np.asarray(scenario.vehicle.initial_position_ned_m[:2], dtype=float)
    nearest_xy, tangent_xy, _ = sim.environment.route.nearest_point_and_tangent(initial_xy)
    offset_distance = float(np.linalg.norm(initial_xy - nearest_xy))

    history = {
        "time_s": [],
        "true_heading_deg": [],
        "deployment_estimated_cable_heading_deg": [],
        "vector_heading_deg": [],
        "vector_cable_heading_deg": [],
        "line_heading_deg": [],
        "gradient_heading_deg": [],
        "fused_heading_deg": [],
        "guidance_source": [],
        "confidence": [],
        "snr_db": [],
        "peak_detected": [],
        "vector_consistency": [],
        "attitude_leakage_risk": [],
        "position_x": [],
        "position_y": [],
        "tracking_strength_nt": [],
        "envelope_gradient_nT_per_m": [],
        "safe_lock_active": [],
        "mode": [],
        "crossing_heading_values": [],
    }

    # Also collect true cable position for lateral deviation
    true_cable_x = []
    true_cable_y = []

    for step_idx in range(max_steps):
        time_s = step_idx * scenario.dt_s
        from auv_mag_tracking.controller import apply_attitude_profile, propagate_vehicle

        apply_attitude_profile(sim.pose, scenario, time_s)

        active_sample_rate_hz = 1.0 / max(sim.magnetometer.sample_period_s, 1e-9)
        sample_count = max(1, int(round(scenario.dt_s * active_sample_rate_hz)))
        sample_times_s = time_s + (np.arange(sample_count, dtype=float) + 1.0) * sim.magnetometer.sample_period_s
        current_block_a = scenario.signal.current_for_times(sample_times_s)
        cable_field_gain_ned_nt = sim.environment.field_model.cable_field_gain_ned_nt(sim.pose.position_ned_m)
        cable_field_block_ned_nt = current_block_a[:, None] * cable_field_gain_ned_nt[None, :]
        true_field_block_ned_nt = cable_field_block_ned_nt + sim.environment.background_field_ned_nt
        magnetometer_reading = sim.magnetometer.sample_block(
            true_field_block_ned_nt, sim.pose, sample_times_s, cable_fields_ned_nt=cable_field_block_ned_nt
        )
        signal_frame = sim.signal_driver.update(magnetometer_reading)
        pose_measurement = sim.imu.observe(sim.pose, time_s)
        cable_truth = sim.environment.cable_truth_at_xy(sim.pose.position_ned_m[:2])
        sonar_reading = sim.sonar.sample(sim.pose, cable_truth, time_s)
        burial_measurement = sim.burial_observer.observe(cable_truth.burial_depth_m, time_s)
        perception_state = sim.perception.update(
            reading=magnetometer_reading,
            pose_measurement=pose_measurement,
            vehicle_position_xy_m=sim.pose.position_ned_m[:2],
            burial_measurement=burial_measurement,
            true_burial_depth_m=cable_truth.burial_depth_m,
            sonar_reading=sonar_reading,
            signal_features=signal_frame.features,
        )
        command = sim.controller.update(sim.pose, perception_state)

        seabed_depth_m = sim.environment.seabed_depth_m(sim.pose.position_ned_m[:2])
        sim.pose = propagate_vehicle(sim.pose, command, scenario, seabed_depth_m, scenario.dt_s)

        true_heading = cable_truth.heading_deg
        true_cable_point = sim.environment.route.nearest_point_and_tangent(sim.pose.position_ned_m[:2])[0]

        history["time_s"].append(time_s)
        history["true_heading_deg"].append(true_heading)
        history["deployment_estimated_cable_heading_deg"].append(
            perception_state.deployment_estimated_cable_heading_deg
            if perception_state.deployment_estimated_cable_heading_deg is not None
            else np.nan
        )
        history["vector_heading_deg"].append(
            perception_state.magnetic_vector_heading_deg
            if perception_state.magnetic_vector_heading_deg is not None
            else np.nan
        )
        history["vector_cable_heading_deg"].append(
            perception_state.vector_cable_heading_deg
            if perception_state.vector_cable_heading_deg is not None
            else np.nan
        )
        history["line_heading_deg"].append(
            perception_state.line_heading_deg if perception_state.line_heading_deg is not None else np.nan
        )
        history["gradient_heading_deg"].append(
            perception_state.envelope_gradient_heading_deg
            if perception_state.envelope_gradient_heading_deg is not None
            else np.nan
        )
        history["fused_heading_deg"].append(
            perception_state.fused_heading_deg if perception_state.fused_heading_deg is not None else np.nan
        )
        history["guidance_source"].append(command.guidance_source)
        history["confidence"].append(perception_state.confidence)
        history["snr_db"].append(perception_state.snr_db)
        history["peak_detected"].append(perception_state.peak_detected)
        history["vector_consistency"].append(perception_state.vector_consistency_score)
        history["attitude_leakage_risk"].append(perception_state.attitude_leakage_risk)
        history["position_x"].append(sim.pose.position_ned_m[0])
        history["position_y"].append(sim.pose.position_ned_m[1])
        history["tracking_strength_nt"].append(perception_state.tracking_strength_nt)
        history["envelope_gradient_nT_per_m"].append(perception_state.envelope_gradient_nT_per_m or 0.0)
        history["safe_lock_active"].append(perception_state.safe_lock_active)
        history["mode"].append(command.mode.value if hasattr(command.mode, 'value') else str(command.mode))
        history["crossing_heading_values"].append(
            sim.perception.crossing_headings[-1][1] if sim.perception.crossing_headings else np.nan
        )
        true_cable_x.append(true_cable_point[0])
        true_cable_y.append(true_cable_point[1])

        sim.latest_command = command
        sim.latest_perception = perception_state
        sim.latest_signal_frame = signal_frame

    for key in history:
        if key not in ("guidance_source", "mode"):
            history[key] = np.array(history[key], dtype=float)

    true_cable_x = np.array(true_cable_x, dtype=float)
    true_cable_y = np.array(true_cable_y, dtype=float)
    positions = np.stack([history["position_x"], history["position_y"]], axis=1)

    return scenario, history, positions, true_cable_x, true_cable_y, offset_distance


def compute_health_metrics(scenario, history, positions, true_cable_x, true_cable_y, offset_distance):
    t = history["time_s"]
    deploy_hdg = history["deployment_estimated_cable_heading_deg"]
    true_hdg = history["true_heading_deg"]
    vec_consist = history["vector_consistency"]
    snr_db = history["snr_db"]
    src = history["guidance_source"]
    mode = history["mode"]
    conf = history["confidence"]
    safe_lock = history["safe_lock_active"]
    vec_cable_hdg = history["vector_cable_heading_deg"]
    line_hdg = history["line_heading_deg"]
    grad_hdg = history["gradient_heading_deg"]
    tracking_strength = history["tracking_strength_nt"]
    peak_det = history["peak_detected"]
    grad_mag = np.abs(history["envelope_gradient_nT_per_m"])
    attitude_leakage = history["attitude_leakage_risk"]

    # === HEALTH METRICS ===

    # 1. Heading errors
    valid_deploy = ~np.isnan(deploy_hdg)
    heading_errors = []
    heading_errors_windows = []
    for i in range(len(deploy_hdg)):
        if not np.isnan(deploy_hdg[i]):
            err = abs(smallest_angle_error_deg(deploy_hdg[i], true_hdg[i]))
            heading_errors.append(err)
            heading_errors_windows.append(err)
        else:
            heading_errors.append(np.nan)
            heading_errors_windows.append(np.nan)
    heading_errors = np.array(heading_errors, dtype=float)

    # Per-source stats
    src_stats = {}
    for i in range(len(src)):
        s = src[i]
        if s not in src_stats:
            src_stats[s] = {"errors": [], "counts": 0}
        if not np.isnan(deploy_hdg[i]):
            err = abs(smallest_angle_error_deg(deploy_hdg[i], true_hdg[i]))
            src_stats[s]["errors"].append(err)
        src_stats[s]["counts"] += 1

    # 2. Vector consistency
    valid_vc = vec_consist[~np.isnan(vec_consist)]
    mean_vc = float(np.mean(valid_vc)) if len(valid_vc) > 0 else 0.0
    min_vc = float(np.min(valid_vc)) if len(valid_vc) > 0 else 0.0
    vc_below_04 = int(np.sum(vec_consist < 0.4))
    vc_below_06 = int(np.sum(vec_consist < 0.6))
    vc_above_08 = int(np.sum(vec_consist >= 0.8))

    # 3. Heading stability (oscillation detection)
    valid_hdg_idx = np.where(~np.isnan(deploy_hdg))[0]
    heading_oscillations = 0
    if len(valid_hdg_idx) > 5:
        for i in range(5, len(valid_hdg_idx)):
            prev_err = abs(smallest_angle_error_deg(deploy_hdg[valid_hdg_idx[i-1]], deploy_hdg[valid_hdg_idx[i]]))
            if prev_err > 30.0:
                heading_oscillations += 1

    # 4. Peak detection quality
    total_peaks = int(np.sum(peak_det))
    peak_rate = total_peaks / t[-1] if t[-1] > 0 else 0.0

    # 5. Mode confusion: count rapid mode switches
    mode_switches = 0
    for i in range(1, len(mode)):
        if mode[i] != mode[i-1]:
            mode_switches += 1
    mode_switch_rate = mode_switches / t[-1] if t[-1] > 0 else 0.0

    # 6. Attitude leakage risk
    leakage_frames = int(np.sum(attitude_leakage))
    leakage_ratio = leakage_frames / len(t) if len(t) > 0 else 0.0

    # 7. SNR quality
    valid_snr = snr_db[~np.isnan(snr_db)]
    mean_snr = float(np.mean(valid_snr)) if len(valid_snr) > 0 else -120.0
    snr_below_6 = int(np.sum(snr_db < 6.0))

    # 8. Gradient magnitude (signal strength proxy)
    mean_grad = float(np.mean(grad_mag))
    grad_below_2 = int(np.sum(grad_mag < 2.0))

    # 9. Line fit quality
    valid_line = ~np.isnan(line_hdg)
    line_jumps = 0
    line_valid_idx = np.where(valid_line)[0]
    if len(line_valid_idx) > 3:
        for i in range(1, len(line_valid_idx)):
            j0, j1 = line_valid_idx[i-1], line_valid_idx[i]
            delta_t = t[j1] - t[j0]
            if delta_t < 5.0:
                jump = abs(smallest_angle_error_deg(line_hdg[j0], line_hdg[j1]))
                if jump > 45.0:
                    line_jumps += 1

    # 10. Vector cable heading vs line heading divergence
    vec_vs_line_divergence = []
    for i in range(len(vec_cable_hdg)):
        if not np.isnan(vec_cable_hdg[i]) and not np.isnan(line_hdg[i]):
            d = abs(smallest_angle_error_deg(vec_cable_hdg[i], line_hdg[i]))
            vec_vs_line_divergence.append(min(d, 180.0 - d))
    mean_vv_div = float(np.mean(vec_vs_line_divergence)) if vec_vs_line_divergence else 0.0
    vv_div_above_45 = int(np.sum(np.array(vec_vs_line_divergence) > 45.0)) if vec_vs_line_divergence else 0

    # 11. Lateral deviation
    lateral_devs = []
    for i in range(len(positions)):
        dx = positions[i, 0] - true_cable_x[i]
        dy = positions[i, 1] - true_cable_y[i]
        lateral_devs.append(float(np.sqrt(dx*dx + dy*dy)))
    lateral_devs = np.array(lateral_devs, dtype=float)
    mean_lateral_dev = float(np.mean(lateral_devs))
    max_lateral_dev = float(np.max(lateral_devs))

    # 12. Confidence level
    mean_conf = float(np.mean(conf))

    # 13. Safe-lock frequency
    safe_lock_frames = int(np.sum(safe_lock))
    safe_lock_ratio = safe_lock_frames / len(t) if len(t) > 0 else 0.0

    # 14. Deployment heading quality
    good_est = sum(1 for err in heading_errors if not np.isnan(err) and err < 15.0)
    decent_est = sum(1 for err in heading_errors if not np.isnan(err) and err < 30.0)
    bad_est_180 = sum(1 for err in heading_errors if not np.isnan(err) and err > 135.0)
    total_valid = sum(1 for err in heading_errors if not np.isnan(err))
    good_ratio = good_est / total_valid if total_valid > 0 else 0.0

    return {
        "initial_lateral_offset_m": offset_distance,
        "total_duration_s": t[-1],
        "total_steps": len(t),
        "total_peaks": total_peaks,
        "peak_rate_hz": peak_rate,
        "total_mode_switches": mode_switches,
        "mode_switch_rate_hz": mode_switch_rate,
        "mean_confidence": mean_conf,
        "good_est_count": good_est,
        "decent_est_count": decent_est,
        "bad_est_180_count": bad_est_180,
        "total_valid_est": total_valid,
        "good_ratio": good_ratio,
        "mean_heading_error_deg": float(np.nanmean(heading_errors)),
        "median_heading_error_deg": float(np.nanmedian(heading_errors)),
        "final_heading_error_deg": heading_errors[~np.isnan(heading_errors)][-1] if any(not np.isnan(e) for e in heading_errors) else np.nan,
        "heading_oscillations": heading_oscillations,
        "mean_vector_consistency": mean_vc,
        "min_vector_consistency": min_vc,
        "vc_below_04_count": vc_below_04,
        "vc_below_06_count": vc_below_06,
        "vc_above_08_count": vc_above_08,
        "mean_snr_db": mean_snr,
        "snr_below_6_count": snr_below_6,
        "leakage_frames": leakage_frames,
        "leakage_ratio": leakage_ratio,
        "mean_gradient_nT_per_m": mean_grad,
        "grad_below_2_count": grad_below_2,
        "line_jumps": line_jumps,
        "vec_vs_line_divergence_deg": mean_vv_div,
        "vv_div_above_45_count": vv_div_above_45,
        "mean_lateral_dev_m": mean_lateral_dev,
        "max_lateral_dev_m": max_lateral_dev,
        "safe_lock_frames": safe_lock_frames,
        "safe_lock_ratio": safe_lock_ratio,
        "heading_errors": heading_errors,
        "src_stats": src_stats,
    }


def print_health_report(metrics: dict) -> None:
    print("\n" + "=" * 70)
    print("  COMPREHENSIVE HEALTH METRICS REPORT")
    print("=" * 70)

    print(f"\n## Scenario & Setup")
    print(f"  Initial lateral offset: {metrics['initial_lateral_offset_m']:.1f} m")
    print(f"  Duration: {metrics['total_duration_s']:.1f} s  |  Steps: {metrics['total_steps']}")

    print(f"\n## Heading Quality")
    print(f"  Mean error:   {metrics['mean_heading_error_deg']:.1f} deg")
    print(f"  Median error: {metrics['median_heading_error_deg']:.1f} deg")
    print(f"  Final error:  {metrics['final_heading_error_deg']:.1f} deg")
    print(f"  Good (<15°):  {metrics['good_est_count']}/{metrics['total_valid_est']} ({100*metrics['good_ratio']:.0f}%)")
    print(f"  Bad ~180°:    {metrics['bad_est_180_count']} occurrences")
    print(f"  Oscillations (>30° jumps): {metrics['heading_oscillations']}")

    print(f"\n## Vector Consistency (PCA quality)")
    print(f"  Mean: {metrics['mean_vector_consistency']:.3f}  |  Min: {metrics['min_vector_consistency']:.3f}")
    print(f"  <0.4: {metrics['vc_below_04_count']} frames  |  <0.6: {metrics['vc_below_06_count']}  |  >=0.8: {metrics['vc_above_08_count']}")

    print(f"\n## Signal Quality")
    print(f"  Mean SNR: {metrics['mean_snr_db']:.1f} dB")
    print(f"  SNR < 6dB: {metrics['snr_below_6_count']} frames")
    print(f"  Mean gradient: {metrics['mean_gradient_nT_per_m']:.2f} nT/m")
    print(f"  Gradient < 2: {metrics['grad_below_2_count']} frames")

    print(f"\n## Heading Source Breakdown")
    for src, stat in sorted(metrics["src_stats"].items(), key=lambda x: np.mean(x[1]["errors"]) if x[1]["errors"] else 999):
        mean_err = np.mean(stat["errors"]) if stat["errors"] else float("nan")
        cnt = len(stat["errors"])
        print(f"  {src:25s}: mean_err={mean_err:6.1f}deg  count={cnt:4d}")

    print(f"\n## Mode & Behavior")
    print(f"  Mode switches: {metrics['total_mode_switches']}  (rate: {metrics['mode_switch_rate_hz']:.2f}/s)")
    print(f"  Peaks captured: {metrics['total_peaks']}  (rate: {metrics['peak_rate_hz']:.2f}/s)")
    print(f"  Mean confidence: {metrics['mean_confidence']:.2f}")

    print(f"\n## Vector vs Line Divergence (Orthogonal risk)")
    print(f"  Mean divergence: {metrics['vec_vs_line_divergence_deg']:.1f} deg")
    print(f"  Divergence > 45°: {metrics['vv_div_above_45_count']} frames")

    print(f"\n## Navigation Quality")
    print(f"  Mean lateral deviation: {metrics['mean_lateral_dev_m']:.1f} m")
    print(f"  Max lateral deviation: {metrics['max_lateral_dev_m']:.1f} m")

    print(f"\n## Leakage & Safe-Lock")
    print(f"  Attitude leakage frames: {metrics['leakage_frames']} ({100*metrics['leakage_ratio']:.0f}%)")
    print(f"  Safe-lock frames: {metrics['safe_lock_frames']} ({100*metrics['safe_lock_ratio']:.0f}%)")

    print(f"\n## HEALTH SCORE")
    score = 0.0
    score += max(0, 25 - metrics["mean_heading_error_deg"]) / 25.0 * 25
    score += max(0, metrics["mean_vector_consistency"] - 0.5) * 2 * 20 if metrics["mean_vector_consistency"] >= 0.5 else metrics["mean_vector_consistency"] * 20
    score += max(0, 20 - metrics["heading_oscillations"]) / 20.0 * 15
    score += max(0, metrics["good_ratio"] - 0.5) * 2 * 15 if metrics["good_ratio"] >= 0.5 else metrics["good_ratio"] * 15
    score += max(0, 0.5 - metrics["leakage_ratio"]) * 2 * 10
    score += max(0, metrics["vv_div_above_45_count"] / max(metrics["total_steps"], 1) * 100 - 10) / 10.0 * 15
    score = max(0.0, min(100.0, score))
    print(f"  Overall: {score:.1f}/100")
    print("=" * 70)
    # Also return score for use in markdown report
    return score


def auto_analyze_issues(metrics: dict) -> str:
    """Automatically analyze health metrics and produce diagnostic summary."""
    issues = []
    recommendations = []

    # Issue 1: Mean heading error too high
    mean_err = metrics["mean_heading_error_deg"]
    if mean_err > 120:
        issues.append(f"🔴 **CRITICAL**: Mean heading error is {mean_err:.1f}° (>120°). The system is fundamentally misaligned.")
        # Check for 180° flip pattern
        if metrics["bad_est_180_count"] > 0:
            ratio = metrics["bad_est_180_count"] / max(metrics["total_valid_est"], 1)
            issues.append(f"  ↳ {metrics['bad_est_180_count']} frames (~{ratio*100:.0f}%) have ~180° errors. The ±90° offset selection is choosing the wrong branch.")
            recommendations.append("✅ Fix Step 2.5 Early Velocity Arbitration to trigger on first 2-4 crossings")
            recommendations.append("✅ Add spread=0 tiebreaker using vehicle travel direction")
        else:
            issues.append("  ↳ No 180° flips detected, but heading is still wrong. Likely a systematic offset error in line fit or vector analyzer.")
            recommendations.append("✅ Verify CableRouteFitter direction_xy calculation (eigenvalue order, arctan2 usage)")
            recommendations.append("✅ Check if vector_cable_heading_deg has ±180° sign ambiguity")

    elif mean_err > 60:
        issues.append(f"🟡 **WARNING**: Mean heading error is {mean_err:.1f}° (>60°). Significant but not catastrophic.")
        recommendations.append("✅ Improve initial crossing offset selection with velocity arbitration")
        recommendations.append("✅ Add hysteresis to prevent late-stage offset switching")

    elif mean_err > 30:
        issues.append(f"🟢 **MODERATE**: Mean heading error is {mean_err:.1f}°. Within acceptable range for early deployment.")
    else:
        issues.append(f"✅ **GOOD**: Mean heading error is {mean_err:.1f}°.")

    # Issue 2: Oscillation
    osc = metrics["heading_oscillations"]
    if osc > 100:
        issues.append(f"🔴 **CRITICAL**: {osc} heading oscillations (>30° jumps). System is unstable.")
        recommendations.append("✅ Increase heading rate constraint (max_heading_change_rate_deg_s)")
        recommendations.append("✅ Add low-pass filter on fused_heading_deg")
        recommendations.append("✅ Check if safe-lock washout is triggering repeatedly")
    elif osc > 20:
        issues.append(f"🟡 **WARNING**: {osc} heading oscillations. System has moderate instability.")
        recommendations.append("✅ Review fusion rule transitions for sudden heading changes")
    else:
        issues.append(f"✅ **GOOD**: Only {osc} oscillations. System is stable.")

    # Issue 3: Mode switching
    mode_rate = metrics["mode_switch_rate_hz"]
    if mode_rate > 0.5:
        issues.append(f"🔴 **CRITICAL**: Mode switching rate is {mode_rate:.2f}/s. System cannot settle into any mode.")
        recommendations.append("✅ Increase mode hysteresis thresholds")
        recommendations.append("✅ Extend SPIRAL_RECOVERY duration before transitioning to HOLD")
    elif mode_rate > 0.2:
        issues.append(f"🟡 **WARNING**: Mode switching rate is {mode_rate:.2f}/s. Frequent transitions reduce tracking stability.")
    else:
        issues.append(f"✅ **GOOD**: Mode switching rate is {mode_rate:.2f}/s.")

    # Issue 4: Peak capture rate
    peak_rate = metrics["peak_rate_hz"]
    if peak_rate < 0.05:
        issues.append(f"🟡 **WARNING**: Peak capture rate is only {peak_rate:.2f}/s ({metrics['total_peaks']} total). Insufficient data for robust fitting.")
        recommendations.append("✅ Lower peak detection threshold (SNR floor)")
        recommendations.append("✅ Check if safe-lock washout is discarding valid peaks")
    elif peak_rate > 0.2:
        issues.append(f"🟡 **WARNING**: Peak capture rate is {peak_rate:.2f}/s. May include false positives.")
    else:
        issues.append(f"✅ **GOOD**: Peak capture rate is {peak_rate:.2f}/s ({metrics['total_peaks']} total).")

    # Issue 5: Vector consistency
    vc = metrics["mean_vector_consistency"]
    if vc < 0.5:
        issues.append(f"🔴 **CRITICAL**: Mean vector consistency is {vc:.3f} (<0.5). PCA extraction is failing.")
        recommendations.append("✅ Increase PCA buffer capacity for more samples")
        recommendations.append("✅ Check if attitude jitter is causing earth-field leakage into AC signal")
    elif vc < 0.7:
        issues.append(f"🟡 **WARNING**: Mean vector consistency is {vc:.3f}. PCA extraction has moderate quality.")
    else:
        issues.append(f"✅ **GOOD**: Mean vector consistency is {vc:.3f}.")

    # Issue 6: Vector vs Line divergence
    div = metrics["vec_vs_line_divergence_deg"]
    div_above = metrics["vv_div_above_45_count"]
    if div_above > metrics["total_steps"] * 0.3:
        ratio = div_above / max(metrics["total_steps"], 1)
        issues.append(f"🔴 **CRITICAL**: Vector vs Line divergence >45° in {div_above} frames ({ratio*100:.0f}%). Orthogonal conflict detected.")
        recommendations.append("✅ Disable vector_cable_heading_deg fusion until sign ambiguity is resolved")
        recommendations.append("✅ Use line_heading_deg as primary source when divergence > 45°")
    else:
        issues.append(f"✅ **GOOD**: Vector vs Line divergence is acceptable ({div:.1f}° mean, {div_above} frames >45°).")

    # Issue 7: SNR
    snr = metrics["mean_snr_db"]
    if snr < 10:
        issues.append(f"🟡 **WARNING**: Mean SNR is {snr:.1f} dB (<10dB). Signal is weak.")
    else:
        issues.append(f"✅ **GOOD**: Mean SNR is {snr:.1f} dB.")

    # Issue 8: Lateral deviation
    lat_dev = metrics["max_lateral_dev_m"]
    if lat_dev > 50:
        issues.append(f"🟡 **WARNING**: Max lateral deviation is {lat_dev:.1f} m. AUV is far from cable.")
        recommendations.append("✅ Increase zigzag width during spiral recovery")
        recommendations.append("✅ Add lateral deviation-based confidence penalty")

    # Issue 9: Safe-lock
    sl_ratio = metrics["safe_lock_ratio"]
    if sl_ratio > 0.1:
        issues.append(f"🟡 **WARNING**: Safe-lock active for {sl_ratio*100:.0f}% of frames. May be too aggressive.")
        recommendations.append("✅ Review safe-lock criterion A & B thresholds")
        recommendations.append("✅ Consider temporary disable for ablation testing")

    # Issue 10: Source breakdown analysis
    src_stats = metrics["src_stats"]
    for src, stat in src_stats.items():
        if stat["errors"]:
            mean_src_err = np.mean(stat["errors"])
            if mean_src_err > 100 and len(stat["errors"]) > 50:
                issues.append(f"🔴 **SOURCE ISSUE**: {src} has {len(stat['errors'])} frames with mean error {mean_src_err:.1f}°.")
                if src == "MAGNETIC":
                    recommendations.append(f"✅ {src} source is dominant but wrong. Check line fit direction_xy sign")
                elif src == "SONAR":
                    recommendations.append(f"✅ {src} source may have orthogonal ambiguity")

    # Build output
    output = []
    output.append("## AUTO-ANALYSIS: Root Causes & Recommendations")
    output.append("")
    output.append("### Issues Detected")
    output.append("")
    for issue in issues:
        output.append(f"- {issue}")
    output.append("")
    output.append("### Recommended Actions")
    output.append("")
    for i, rec in enumerate(recommendations, 1):
        output.append(f"{i}. {rec}")
    output.append("")

    # Summary verdict
    if metrics["mean_heading_error_deg"] < 15 and metrics["bad_est_180_count"] == 0:
        output.append("### ✅ VERDICT: System is performing within specification.")
        output.append(f"   - Mean error {metrics['mean_heading_error_deg']:.1f}° < 15° target")
        output.append(f"   - No 180° flips detected")
    elif metrics["bad_est_180_count"] > metrics["total_valid_est"] * 0.5:
        output.append("### 🔴 VERDICT: System is fundamentally broken. >50% of estimates are ~180° wrong.")
        output.append("   Priority: Fix ±90° offset selection immediately.")
    elif metrics["mean_heading_error_deg"] > 60:
        output.append("### 🟡 VERDICT: System is partially working but has systematic heading errors.")
        output.append(f"   - Mean error {metrics['mean_heading_error_deg']:.1f}° is {metrics['mean_heading_error_deg'] - 15:.0f}° above target")
        output.append("   - Priority: Improve initial offset selection and fusion rule protection.")
    else:
        output.append("### 🟢 VERDICT: System is approaching target. Minor tuning needed.")
        output.append(f"   - Mean error {metrics['mean_heading_error_deg']:.1f}° is {max(0, metrics['mean_heading_error_deg'] - 15):.0f}° above target")

    return "\n".join(output)


def save_markdown_report(metrics: dict, score: float, auto_analysis: str, output_path: Path, png_path: Path = None) -> None:
    """Save a comprehensive markdown report to file."""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    lines = []
    lines.append(f"# AUV Cable Tracking Health Report — {metrics.get('scenario_name', 'case1')}")
    lines.append(f"")
    lines.append(f"**Generated**: {timestamp}")
    lines.append(f"**Health Score**: {score:.1f}/100")
    lines.append(f"")
    lines.append("---")
    lines.append("")

    # Embed PNG image if available
    if png_path is not None and png_path.exists():
        lines.append("## Health Dashboard Visualization")
        lines.append("")
        lines.append(f"![Health Dashboard]({png_path.name})")
        lines.append("")
        lines.append("---")
        lines.append("")

    lines.append("## Summary Metrics")
    lines.append("")
    lines.append(f"| Metric | Value |")
    lines.append(f"|--------|-------|")
    lines.append(f"| Mean heading error | {metrics['mean_heading_error_deg']:.1f}° |")
    lines.append(f"| Median heading error | {metrics['median_heading_error_deg']:.1f}° |")
    lines.append(f"| Final heading error | {metrics['final_heading_error_deg']:.1f}° |")
    lines.append(f"| Good estimates (<15°) | {metrics['good_est_count']}/{metrics['total_valid_est']} ({100*metrics['good_ratio']:.0f}%) |")
    lines.append(f"| Bad ~180° estimates | {metrics['bad_est_180_count']} |")
    lines.append(f"| Heading oscillations | {metrics['heading_oscillations']} |")
    lines.append(f"| Mode switch rate | {metrics['mode_switch_rate_hz']:.2f}/s |")
    lines.append(f"| Peak capture rate | {metrics['peak_rate_hz']:.2f}/s |")
    lines.append(f"| Mean SNR | {metrics['mean_snr_db']:.1f} dB |")
    lines.append(f"| Mean vector consistency | {metrics['mean_vector_consistency']:.3f} |")
    lines.append(f"| Max lateral deviation | {metrics['max_lateral_dev_m']:.1f} m |")
    lines.append(f"| Safe-lock frames | {metrics['safe_lock_frames']} ({100*metrics['safe_lock_ratio']:.0f}%) |")
    lines.append("")
    lines.append("---")
    lines.append("")
    lines.append("## Heading Source Breakdown")
    lines.append("")
    lines.append(f"| Source | Mean Error | Count |")
    lines.append(f"|--------|-----------|-------|")
    for src, stat in sorted(metrics["src_stats"].items(), key=lambda x: np.mean(x[1]["errors"]) if x[1]["errors"] else 999):
        mean_err = np.mean(stat["errors"]) if stat["errors"] else float("nan")
        cnt = len(stat["errors"])
        lines.append(f"| {src} | {mean_err:.1f}° | {cnt} |")
    lines.append("")
    lines.append("---")
    lines.append("")
    lines.append(auto_analysis)
    lines.append("")
    lines.append("---")
    lines.append("")
    lines.append("## Raw Metrics (JSON)")
    lines.append("")
    # Serialize metrics (exclude numpy arrays)
    serializable = {}
    for k, v in metrics.items():
        if k == "heading_errors":
            continue  # Too large
        if isinstance(v, (int, float, str, bool, list, dict)):
            serializable[k] = v
        elif isinstance(v, np.floating):
            serializable[k] = float(v)
        elif isinstance(v, np.integer):
            serializable[k] = int(v)
        elif isinstance(v, np.ndarray):
            serializable[k] = v.tolist()
    lines.append("```json")
    lines.append(json.dumps(serializable, indent=2))
    lines.append("```")
    lines.append("")

    output_path.write_text("\n".join(lines), encoding="utf-8")
    print(f"\n📄 Markdown report saved to: {output_path}")


def generate_plots(scenario, history, positions, true_cable_x, true_cable_y, metrics: dict, output_path: Path, score: float = 0.0):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
    except ImportError:
        print("matplotlib not available, skipping plot generation")
        return

    t = history["time_s"]
    deploy_hdg = history["deployment_estimated_cable_heading_deg"]
    true_hdg = history["true_heading_deg"]
    vec_consist = history["vector_consistency"]
    snr_db = history["snr_db"]
    src = history["guidance_source"]
    conf = history["confidence"]
    safe_lock = history["safe_lock_active"]
    vec_cable_hdg = history["vector_cable_heading_deg"]
    line_hdg = history["line_heading_deg"]
    grad_hdg = history["gradient_heading_deg"]
    tracking_strength = history["tracking_strength_nt"]
    peak_det = history["peak_detected"]
    grad_mag = np.abs(history["envelope_gradient_nT_per_m"])
    attitude_leakage = history["attitude_leakage_risk"]
    mode = history["mode"]
    heading_errors = metrics["heading_errors"]

    fig = plt.figure(figsize=(20, 24))
    fig.suptitle(f"AUV Cable Tracking Health Report — {scenario.name}\n"
                 f"Final Error: {metrics['final_heading_error_deg']:.1f}° | "
                 f"Mean Error: {metrics['mean_heading_error_deg']:.1f}° | "
                 f"Health Score: {score:.1f}/100",
                 fontsize=14, fontweight="bold")

    # Color map for sources
    src_colors = {
        "NOMINAL": "green",
        "MAGNETIC": "blue",
        "MAGNETIC_PEAK": "cyan",
        "SONAR": "orange",
        "SONAR_SEED": "lime",
        "SONAR_AVOID_SPIRAL": "yellowgreen",
        "GRADIENT": "purple",
        "MEMORY": "gray",
        "BLIND": "brown",
        "SEARCH": "red",
        "SPIRAL_RECOVERY": "pink",
        "REACQUIRE_SPIRAL": "coral",
        "DEPLOYMENT_SPIRAL": "salmon",
        "SAFE_LOCK": "darkred",
        "FUSION_HOLD": "gold",
        "BOOTSTRAP_SPIRAL": "lightpink",
        "BLIND_RECOVERY": "tan",
        "BLIND_INERTIA": "khaki",
    }
    src_color_arr = [src_colors.get(s, "lightgray") for s in src]

    gs = fig.add_gridspec(6, 2, hspace=0.5, wspace=0.3)

    # Panel 1: Heading over time
    ax1 = fig.add_subplot(gs[0, :])
    ax1.plot(t, true_hdg, "k-", linewidth=2, label="True Cable", alpha=0.8)
    valid_idx = ~np.isnan(deploy_hdg)
    if np.any(valid_idx):
        src_color_arr_valid = np.array(src_color_arr)[valid_idx]
        ax1.scatter(t[valid_idx], deploy_hdg[valid_idx], c=src_color_arr_valid, s=3, alpha=0.6, label="Deployment Heading")
    ax1.set_ylabel("Heading (deg)")
    ax1.set_title("Cable Heading Estimation vs True (colored by source)")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.set_ylim(-10, 370)
    ax1.grid(True, alpha=0.3)

    # Panel 2: Heading error over time
    ax2 = fig.add_subplot(gs[1, :])
    ax2.plot(t, heading_errors, "r-", linewidth=1, alpha=0.7)
    ax2.axhline(15, color="green", linestyle="--", linewidth=1, label="15° threshold")
    ax2.axhline(45, color="orange", linestyle="--", linewidth=1, label="45° threshold")
    ax2.axhline(135, color="red", linestyle="--", linewidth=1, label="135° (180° flip)")
    ax2.set_ylabel("Abs Heading Error (deg)")
    ax2.set_title("Heading Error Over Time")
    ax2.legend(loc="upper right", fontsize=8)
    ax2.set_ylim(0, 200)
    ax2.grid(True, alpha=0.3)

    # Panel 3: Vector consistency + attitude leakage
    ax3 = fig.add_subplot(gs[2, 0])
    ax3.plot(t, vec_consist, "b-", linewidth=1, alpha=0.7, label="Vector Consistency")
    ax3.axhline(0.5, color="orange", linestyle="--", linewidth=1, label="0.5 threshold")
    ax3.axhline(0.8, color="green", linestyle="--", linewidth=1, label="0.8 (good)")
    leak_idx = np.where(attitude_leakage > 0)[0]
    if len(leak_idx) > 0:
        ax3.scatter(t[leak_idx], vec_consist[leak_idx], c="red", s=5, alpha=0.5, label=f"Leak Risk ({len(leak_idx)} frames)")
    ax3.set_ylabel("Vector Consistency")
    ax3.set_title("PCA Vector Consistency Score")
    ax3.legend(loc="lower right", fontsize=8)
    ax3.set_ylim(0, 1.05)
    ax3.grid(True, alpha=0.3)

    # Panel 4: SNR over time
    ax4 = fig.add_subplot(gs[2, 1])
    ax4.plot(t, snr_db, "g-", linewidth=1, alpha=0.7)
    ax4.axhline(6, color="red", linestyle="--", linewidth=1, label="6 dB threshold")
    ax4.axhline(10, color="orange", linestyle="--", linewidth=1, label="10 dB threshold")
    ax4.set_ylabel("SNR (dB)")
    ax4.set_title("Signal-to-Noise Ratio")
    ax4.legend(loc="lower right", fontsize=8)
    ax4.grid(True, alpha=0.3)

    # Panel 5: Vector cable heading vs line heading
    ax5 = fig.add_subplot(gs[3, 0])
    valid_vec = ~np.isnan(vec_cable_hdg)
    valid_line = ~np.isnan(line_hdg)
    if np.any(valid_vec):
        ax5.plot(t, vec_cable_hdg, "b-", linewidth=1, alpha=0.7, label="Vector Cable Hdg")
    if np.any(valid_line):
        ax5.plot(t, line_hdg, "r-", linewidth=1, alpha=0.7, label="Line Hdg")
    ax5.set_ylabel("Heading (deg)")
    ax5.set_title("Vector Cable Heading vs Line Heading")
    ax5.legend(loc="upper right", fontsize=8)
    ax5.set_ylim(-10, 370)
    ax5.grid(True, alpha=0.3)

    # Panel 6: Gradient magnitude
    ax6 = fig.add_subplot(gs[3, 1])
    ax6.plot(t, grad_mag, "purple", linewidth=1, alpha=0.7)
    ax6.axhline(2.0, color="orange", linestyle="--", linewidth=1, label="2.0 threshold")
    ax6.axhline(5.0, color="green", linestyle="--", linewidth=1, label="5.0 (strong)")
    peak_idx = np.where(peak_det > 0)[0]
    if len(peak_idx) > 0:
        ax6.scatter(t[peak_idx], grad_mag[peak_idx], c="red", s=8, alpha=0.5, label=f"Peaks ({len(peak_idx)})")
    ax6.set_ylabel("|Gradient| (nT/m)")
    ax6.set_title("Envelope Gradient Magnitude")
    ax6.legend(loc="upper right", fontsize=8)
    ax6.grid(True, alpha=0.3)

    # Panel 7: Mode / guidance source over time
    ax7 = fig.add_subplot(gs[4, :])
    src_num = {s: i for i, s in enumerate(sorted(set(src)))}
    src_num_arr = np.array([src_num.get(s, -1) for s in src], dtype=float)
    cmap = plt.cm.get_cmap("tab20", len(src_num))
    sc = ax7.scatter(t, src_num_arr, c=src_num_arr, cmap=cmap, s=3, alpha=0.8)
    ax7.set_yticks(list(src_num.values()))
    ax7.set_yticklabels(list(src_num.keys()), fontsize=7)
    ax7.set_ylabel("Guidance Source")
    ax7.set_title("Guidance Source Over Time")
    ax7.grid(True, alpha=0.3, axis="x")

    # Panel 8: 2D trajectory
    ax8 = fig.add_subplot(gs[5, 0])
    ax8.plot(positions[:, 0], positions[:, 1], "b-", linewidth=0.8, alpha=0.6, label="AUV Path")
    ax8.plot(true_cable_x, true_cable_y, "k-", linewidth=2, alpha=0.8, label="True Cable")
    # Mark peak positions
    peak_positions = []
    for i in range(len(t)):
        if peak_det[i] > 0:
            peak_positions.append([positions[i, 0], positions[i, 1]])
    if peak_positions:
        peak_arr = np.array(peak_positions)
        ax8.scatter(peak_arr[:, 0], peak_arr[:, 1], c="red", s=5, alpha=0.5, label=f"Peaks ({len(peak_arr)})")
    ax8.set_xlabel("X (m)")
    ax8.set_ylabel("Y (m)")
    ax8.set_title("AUV Trajectory vs True Cable")
    ax8.legend(loc="upper right", fontsize=8)
    ax8.grid(True, alpha=0.3)
    ax8.set_aspect("equal")

    # Panel 9: Confidence + safe-lock
    ax9 = fig.add_subplot(gs[5, 1])
    ax9.plot(t, conf, "b-", linewidth=1, alpha=0.7, label="Confidence")
    ax9.plot(t, safe_lock.astype(float), "r-", linewidth=1, alpha=0.5, label="Safe Lock")
    ax9.set_ylabel("Confidence / SafeLock")
    ax9.set_title("Controller Confidence & Safe Lock")
    ax9.legend(loc="upper right", fontsize=8)
    ax9.set_ylim(-0.1, 1.1)
    ax9.grid(True, alpha=0.3)

    plt.savefig(output_path, dpi=150, bbox_inches="tight", facecolor="white")
    print(f"\nFigure saved to: {output_path}")
    plt.close(fig)


def main():
    case_name = "case1"
    output_png = Path(f"/Users/bytedance/coding/AUV-Master-Mag/tools/health_report_{case_name}.png")
    output_md = Path(f"/Users/bytedance/coding/AUV-Master-Mag/tools/health_report_{case_name}.md")

    print(f"Running diagnostic for {case_name}...")
    scenario, history, positions, true_cable_x, true_cable_y, offset_distance = run_diagnostic(case_name)

    metrics = compute_health_metrics(scenario, history, positions, true_cable_x, true_cable_y, offset_distance)
    # Add scenario name for report
    metrics["scenario_name"] = case_name

    # Generate plot first (PNG)
    generate_plots(scenario, history, positions, true_cable_x, true_cable_y, metrics, output_png)

    # Print terminal report
    score = print_health_report(metrics)

    # Auto-analyze issues
    auto_analysis = auto_analyze_issues(metrics)
    print(f"\n{auto_analysis}")

    # Save markdown report (with PNG embedded)
    save_markdown_report(metrics, score, auto_analysis, output_md, png_path=output_png)

    return metrics


if __name__ == "__main__":
    main()
