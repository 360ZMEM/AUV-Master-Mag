"""Markdown report generation for the visualization system.

Pure text assembly (plus file writes): consumes :class:`HealthMetrics` and figure
paths, emits a self-contained Markdown report with an embedded health score and
automatic issue analysis.
"""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import Dict, List

from .metrics import HealthMetrics, ProgressDelta, health_score, metrics_to_dict


def _auto_analysis(metrics: HealthMetrics) -> List[str]:
    """根据指标自动产出问题诊断与结论（无副作用）。"""
    lines = ["## Auto-analysis", ""]
    mean_err = metrics.mean_heading_error_deg

    if mean_err <= 15.0:
        lines.append(f"- GOOD: mean heading error {mean_err:.1f} deg within 15 deg target.")
    elif mean_err <= 30.0:
        lines.append(
            f"- MODERATE: mean fused-heading error {mean_err:.1f} deg, above 15 deg target; "
            "check task-level metrics before judging tracking failure."
        )
    else:
        lines.append(f"- WARNING: mean fused-heading error {mean_err:.1f} deg is high; check line-fit direction / FSM transitions.")

    if metrics.track_active_fraction > 0.0:
        lines.append(
            f"- TRACK quality: vehicle heading error {metrics.track_mean_vehicle_heading_error_deg:.1f} deg, "
            f"cross-track {metrics.track_mean_cross_track_m:.1f} m during TRACK_ACTIVE."
        )

    if metrics.flip_count > 0:
        lines.append(f"- WARNING: {metrics.flip_count} frames with ~180 deg error (heading-flip residue).")

    if metrics.mode_switches <= 6:
        lines.append(f"- GOOD: {metrics.mode_switches} FSM switches (stable).")
    else:
        lines.append(f"- WARNING: {metrics.mode_switches} FSM switches; consider stronger hysteresis.")

    if metrics.track_active_fraction >= 0.30:
        lines.append(f"- GOOD: TRACK_ACTIVE occupies {metrics.track_active_fraction*100:.0f}% of the run.")
    else:
        lines.append(f"- INFO: TRACK_ACTIVE occupies only {metrics.track_active_fraction*100:.0f}%; lock convergence may be slow.")

    if metrics.max_cross_track_m > 12.0:
        lines.append(f"- WARNING: max cross-track {metrics.max_cross_track_m:.1f} m exceeds 12 m band.")
    else:
        lines.append(f"- GOOD: max cross-track {metrics.max_cross_track_m:.1f} m within 12 m band.")

    if metrics.route_completion_ratio == metrics.route_completion_ratio:
        lines.append(
            f"- Route progress: {metrics.route_completion_ratio*100:.1f}% "
            f"(final route distance {metrics.final_route_distance_m:.1f} m, "
            f"endpoint={'yes' if metrics.endpoint_completed >= 0.5 else 'no'})."
        )

    lines.append(
        f"- Guidance contribution: sonar {metrics.sonar_contribution*100:.0f}% / "
        f"magnetic {metrics.magnetic_contribution*100:.0f}% "
        f"(peaks={metrics.total_peaks}, rate={metrics.peak_rate_hz:.2f}/s)."
    )
    if metrics.magnetic_path_observation_fraction > 0.0:
        lines.append(
            f"- Magnetic probe: observations {metrics.magnetic_path_observation_fraction*100:.0f}% of frames, "
            f"axis error {metrics.magnetic_path_mean_axis_error_deg:.1f} deg, "
            f"position error {metrics.magnetic_path_mean_position_error_m:.1f} m."
        )
    lines.append("")
    return lines


def save_run_report(metrics: HealthMetrics, fig_paths: Dict[str, Path], out_path: Path) -> Path:
    """为单次运行写出 Markdown 报告（含图、指标表、自动分析、JSON）。"""
    score = health_score(metrics)
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    mode_tag = "deployment" if metrics.deployment_mode else "nominal"

    lines: List[str] = [
        f"# AUV Cable Tracking Report — {metrics.case_name} ({mode_tag})",
        "",
        f"**Generated**: {timestamp}",
        f"**Health Score**: {score:.1f}/100",
        f"**Duration**: {metrics.duration_s:.1f} s ({metrics.total_steps} steps)",
        "",
        "---",
        "",
    ]

    for tier in ("overview", "detail"):
        path = fig_paths.get(tier)
        if path is not None:
            lines += [f"## {tier.capitalize()} figure", "", f"![{tier}]({Path(path).name})", "", "---", ""]

    lines += [
        "## Summary metrics",
        "",
        "| Metric | Value |",
        "|--------|-------|",
        f"| Mean heading error | {metrics.mean_heading_error_deg:.1f} deg |",
        f"| Median heading error | {metrics.median_heading_error_deg:.1f} deg |",
        f"| Final heading error | {metrics.final_heading_error_deg:.1f} deg |",
        f"| Mean vehicle heading error | {metrics.mean_vehicle_heading_error_deg:.1f} deg |",
        f"| TRACK fused-heading error | {metrics.track_mean_heading_error_deg:.1f} deg |",
        f"| TRACK vehicle heading error | {metrics.track_mean_vehicle_heading_error_deg:.1f} deg |",
        f"| Good estimates (<15 deg) | {metrics.good_ratio*100:.0f}% |",
        f"| Heading flips (~180 deg) | {metrics.flip_count} |",
        f"| Heading oscillations | {metrics.heading_oscillations} |",
        f"| TRACK_ACTIVE fraction | {metrics.track_active_fraction*100:.0f}% |",
        f"| FSM switches | {metrics.mode_switches} |",
        f"| Total magnetic peaks | {metrics.total_peaks} |",
        f"| Peak rate | {metrics.peak_rate_hz:.2f}/s |",
        f"| Mean SNR | {metrics.mean_snr_db:.1f} dB |",
        f"| Mean fit residual | {metrics.mean_fit_residual_m:.2f} m |",
        f"| Lock-grade fraction (lambda_perp<1) | {metrics.lock_grade_fraction*100:.0f}% |",
        f"| Mean cross-track | {metrics.mean_cross_track_m:.1f} m |",
        f"| Median cross-track | {metrics.median_cross_track_m:.1f} m |",
        f"| P90 cross-track | {metrics.p90_cross_track_m:.1f} m |",
        f"| TRACK mean cross-track | {metrics.track_mean_cross_track_m:.1f} m |",
        f"| Max cross-track | {metrics.max_cross_track_m:.1f} m |",
        f"| Final cross-track | {metrics.final_cross_track_m:.1f} m |",
        f"| Route completion | {metrics.route_completion_ratio*100:.1f}% |",
        f"| Final route distance | {metrics.final_route_distance_m:.1f} m |",
        f"| Endpoint goal enabled | {'yes' if metrics.endpoint_goal_enabled >= 0.5 else 'no'} |",
        f"| Endpoint completed | {'yes' if metrics.endpoint_completed >= 0.5 else 'no'} |",
        f"| Mean confidence | {metrics.mean_confidence:.2f} |",
        f"| Sonar contribution | {metrics.sonar_contribution*100:.0f}% |",
        f"| Magnetic contribution | {metrics.magnetic_contribution*100:.0f}% |",
        f"| Magnetic path observation fraction | {metrics.magnetic_path_observation_fraction*100:.0f}% |",
        f"| Magnetic path axis error | {metrics.magnetic_path_mean_axis_error_deg:.1f} deg |",
        f"| Magnetic path position error | {metrics.magnetic_path_mean_position_error_m:.1f} m |",
        f"| Magnetic path mean abs offset | {metrics.magnetic_path_mean_cross_track_offset_m:.1f} m |",
        f"| Magnetic phase observation fraction | {metrics.magnetic_phase_observation_fraction*100:.0f}% |",
        f"| Magnetic phase axis error | {metrics.magnetic_phase_mean_axis_error_deg:.1f} deg |",
        f"| Magnetic phase position error | {metrics.magnetic_phase_mean_position_error_m:.1f} m |",
        f"| Magnetic phase mean amplitude | {metrics.magnetic_phase_mean_amplitude_m:.1f} m |",
        f"| Shadow axis hypothesis valid | {metrics.shadow_axis_hypothesis_fraction*100:.0f}% |",
        f"| Shadow axis selected score | {metrics.shadow_axis_mean_score:.2f} |",
        f"| Shadow axis score margin | {metrics.shadow_axis_mean_margin:.2f} |",
        f"| Shadow axis positive fraction | {metrics.shadow_axis_positive_fraction*100:.0f}% |",
        f"| Zig-zag probe active | {metrics.zigzag_probe_active_fraction*100:.0f}% |",
        f"| Zig-zag probe cycles | {metrics.zigzag_probe_cycle_count} |",
        f"| Zig-zag probe leg flips | {metrics.zigzag_probe_leg_flip_count} |",
        f"| Zig-zag probe magnetic crossings | {metrics.zigzag_probe_magnetic_crossing_count} |",
        f"| Zig-zag probe magnetic crossings / cycle | {metrics.zigzag_probe_magnetic_crossings_per_cycle:.2f} |",
        f"| Magnetic-crossing probe forced flips | {metrics.magnetic_crossing_probe_forced_flip_count} |",
        f"| Magnetic-crossing probe missed count | {metrics.magnetic_crossing_probe_missed_count} |",
        f"| Magnetic-crossing probe mean wait | {metrics.magnetic_crossing_probe_mean_wait_s:.1f} s |",
        f"| Zig-zag probe mean cycle duration | {metrics.zigzag_probe_mean_cycle_duration_s:.1f} s |",
        f"| Zig-zag probe mean peak abs XT | {metrics.zigzag_probe_mean_peak_abs_cross_track_m:.1f} m |",
        f"| Zig-zag probe phase events / cycle | {metrics.zigzag_probe_phase_events_per_cycle:.2f} |",
        f"| Zig-zag probe mean abs B_down/B_perp | {metrics.zigzag_probe_mean_abs_field_ratio:.2f} |",
        f"| Zig-zag probe mean abs B_perp | {metrics.zigzag_probe_mean_abs_b_perp_nt:.1f} nT |",
        f"| Zig-zag probe burial coverage | {metrics.zigzag_probe_burial_coverage*100:.0f}% |",
        f"| Zig-zag probe burial MAE | {metrics.zigzag_probe_burial_mae_m:.3f} m |",
        f"| Zig-zag probe cycle burial coverage | {metrics.zigzag_probe_cycle_burial_coverage*100:.0f}% |",
        f"| Zig-zag probe cycle burial MAE | {metrics.zigzag_probe_cycle_burial_mae_m:.3f} m |",
        f"| Zig-zag probe cycle burial sigma | {metrics.zigzag_probe_cycle_burial_mean_sigma_m:.3f} m |",
        f"| Zig-zag probe cycle burial quality | {metrics.zigzag_probe_cycle_burial_mean_quality:.2f} |",
        f"| Shadow hypothesis supply score | {metrics.shadow_hypothesis_mean_supply_score:.2f} |",
        f"| Shadow hypothesis selection score | {metrics.shadow_hypothesis_mean_selection_score:.2f} |",
        f"| Shadow hypothesis consumption score | {metrics.shadow_hypothesis_mean_consumption_score:.2f} |",
        f"| Shadow hypothesis readiness score | {metrics.shadow_hypothesis_mean_readiness_score:.2f} |",
        f"| Shadow bottleneck supply | {metrics.shadow_hypothesis_bottleneck_supply_fraction*100:.0f}% |",
        f"| Shadow bottleneck selection | {metrics.shadow_hypothesis_bottleneck_selection_fraction*100:.0f}% |",
        f"| Shadow bottleneck consumption | {metrics.shadow_hypothesis_bottleneck_consumption_fraction*100:.0f}% |",
        f"| Magnetic lookahead fraction | {metrics.magnetic_lookahead_fraction*100:.0f}% |",
        f"| Magnetic lookahead axis error | {metrics.magnetic_lookahead_mean_axis_error_deg:.1f} deg |",
        f"| Magnetic lookahead position error | {metrics.magnetic_lookahead_mean_position_error_m:.1f} m |",
        f"| Magnetic lookahead mean age | {metrics.magnetic_lookahead_mean_age_s:.1f} s |",
        f"| Lookahead feed allowed | {metrics.magnetic_lookahead_feed_allowed_fraction*100:.0f}% |",
        f"| Lookahead feed reject age | {metrics.magnetic_lookahead_feed_reject_age_fraction*100:.0f}% |",
        f"| Lookahead feed reject phase age | {metrics.magnetic_lookahead_feed_reject_phase_age_fraction*100:.0f}% |",
        f"| Lookahead feed reject residual | {metrics.magnetic_lookahead_feed_reject_residual_fraction*100:.0f}% |",
        f"| Lookahead feed reject heading | {metrics.magnetic_lookahead_feed_reject_heading_fraction*100:.0f}% |",
        f"| Lookahead feed reject innovation | {metrics.magnetic_lookahead_feed_reject_innovation_fraction*100:.0f}% |",
        f"| Lookahead feed mean phase age | {metrics.magnetic_lookahead_feed_mean_phase_age_s:.1f} s |",
        f"| Lookahead feed mean innovation | {metrics.magnetic_lookahead_feed_mean_innovation_m:.1f} m |",
        f"| Lookahead feed mean axis delta | {metrics.magnetic_lookahead_feed_mean_axis_delta_deg:.1f} deg |",
        f"| Lookahead feed mean local residual | {metrics.magnetic_lookahead_feed_mean_local_residual_m:.1f} m |",
        f"| Burial inversion coverage | {metrics.burial_inversion_coverage*100:.0f}% |",
        f"| Burial inversion MAE | {metrics.burial_inversion_mae_m:.3f} m |",
        "",
        "---",
        "",
        "## FSM occupancy",
        "",
        "| State | Fraction |",
        "|-------|----------|",
    ]
    for mode, frac in sorted(metrics.mode_fraction.items(), key=lambda kv: -kv[1]):
        lines.append(f"| {mode} | {frac*100:.0f}% |")
    lines += ["", "---", ""]

    lines += _auto_analysis(metrics)
    lines += ["---", "", "## Raw metrics (JSON)", "", "```json",
              json.dumps(metrics_to_dict(metrics), indent=2), "```", ""]

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(lines), encoding="utf-8")
    return out_path


def save_showcase_report(metrics_list: List[HealthMetrics], showcase_fig: Path, out_path: Path) -> Path:
    """写出跨 case 成果汇总报告（系统展示前序重构成果）。"""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    lines: List[str] = [
        "# Refactor Showcase — Phase 0–2 Results",
        "",
        f"**Generated**: {timestamp}",
        "",
        "Demonstrates the structural fixes delivered so far: dead-code removal,"
        " perception package split, three-state FSM, sonar-fed line fitting and the"
        " magnetic cross-track steering signal — verified across all scenarios.",
        "",
        "---",
        "",
        "## Cross-case comparison figure",
        "",
        f"![showcase]({Path(showcase_fig).name})",
        "",
        "---",
        "",
        "## Scenario matrix",
        "",
        "| Case | Health | Fused err [deg] | TRACK veh err [deg] | TRACK XT [m] | Route % | TRACK % | Switches |",
        "|------|--------|------------------|---------------------|--------------|---------|---------|----------|",
    ]
    for m in metrics_list:
        lines.append(
            f"| {m.case_name} | {health_score(m):.0f} | {m.mean_heading_error_deg:.1f} | "
            f"{m.track_mean_vehicle_heading_error_deg:.1f} | {m.track_mean_cross_track_m:.1f} | "
            f"{m.route_completion_ratio*100:.1f} | {m.track_active_fraction*100:.0f} | {m.mode_switches} |"
        )
    lines.append("")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(lines), encoding="utf-8")
    return out_path


_PROGRESS_FIELD_LABELS = {
    "switches": "FSM switches",
    "health": "Health /100",
    "mean_err": "Mean err [deg]",
    "track_pct": "TRACK [%]",
}

# Sub-noise tolerance per field: the committed baselines are rounded, so changes
# within these bands are reported as "flat" rather than spurious improve/regress.
_PROGRESS_TOLERANCE = {
    "switches": 0.5,
    "health": 1.5,
    "mean_err": 0.5,
    "track_pct": 1.5,
}


def _progress_verdict(delta: ProgressDelta, field: str) -> str:
    """单字段进度结论：改善/持平/回退，并标注是否达标。"""
    before, after, change, higher_is_better, _, target = delta.fields[field]
    meets = (after >= target) if higher_is_better else (after <= target)
    if abs(change) <= _PROGRESS_TOLERANCE.get(field, 0.0):
        trend = "flat"
    elif delta.improved(field):
        trend = "improved"
    else:
        trend = "REGRESSED"
    return f"{trend}{' ✓' if meets else ''}"


def save_progress_report(deltas: List[ProgressDelta], progress_fig: Path, out_path: Path,
                         subtitle: str = "Phase 0–2G refactor gains") -> Path:
    """写出 before→after 进度对照报告（系统展示前序修复成果）。"""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    lines: List[str] = [
        "# Refactor Progress Report — before → after",
        "",
        f"**Generated**: {timestamp}",
        f"**Scope**: {subtitle}",
        "",
        "Quantifies the gain delivered by the prior structural fixes (dead-code"
        " removal, perception package split, three-state FSM, sonar-fed line"
        " fitting, magnetic cross-track steering, and the Phase 2G Schmitt"
        " hysteresis + time-hold that collapsed the FSM switch storm). Baselines"
        " are committed constants (see `viz/baseline.py`), so this report"
        " reproduces from a clean checkout without the git-ignored `results/`.",
        "",
        "---",
        "",
        "## Progress figure",
        "",
        f"![progress]({Path(progress_fig).name})",
        "",
        "---",
        "",
        "## Per-case progress matrix",
        "",
    ]

    for field, label in _PROGRESS_FIELD_LABELS.items():
        lines += [f"### {label}", "", "| Case | before | after | Δ | verdict |",
                  "|------|--------|-------|---|---------|"]
        for delta in deltas:
            before, after, change, _, _, _ = delta.fields[field]
            lines.append(
                f"| {delta.case_name} | {before:.1f} | {after:.1f} | {change:+.1f} | "
                f"{_progress_verdict(delta, field)} |"
            )
        lines.append("")

    # Aggregate headline numbers for the switch storm collapse.
    sw_before = sum(d.fields["switches"][0] for d in deltas)
    sw_after = sum(d.fields["switches"][1] for d in deltas)
    reduction = (1.0 - sw_after / sw_before) * 100.0 if sw_before else 0.0
    err_ok = sum(1 for d in deltas if d.fields["mean_err"][1] <= 15.0)
    lines += [
        "---",
        "",
        "## Headline",
        "",
        f"- FSM switch storm: total {sw_before:.0f} → {sw_after:.0f} switches "
        f"across {len(deltas)} cases ({reduction:.1f}% reduction).",
        f"- Heading accuracy: {err_ok}/{len(deltas)} cases now within the 15° hard limit.",
        "",
    ]

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(lines), encoding="utf-8")
    return out_path
