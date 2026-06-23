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

from .metrics import HealthMetrics, health_score, metrics_to_dict


def _auto_analysis(metrics: HealthMetrics) -> List[str]:
    """根据指标自动产出问题诊断与结论（无副作用）。"""
    lines = ["## Auto-analysis", ""]
    mean_err = metrics.mean_heading_error_deg

    if mean_err <= 15.0:
        lines.append(f"- GOOD: mean heading error {mean_err:.1f} deg within 15 deg target.")
    elif mean_err <= 30.0:
        lines.append(f"- MODERATE: mean heading error {mean_err:.1f} deg, above 15 deg target.")
    else:
        lines.append(f"- WARNING: mean heading error {mean_err:.1f} deg is high; check line-fit direction / FSM transitions.")

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

    lines.append(
        f"- Guidance contribution: sonar {metrics.sonar_contribution*100:.0f}% / "
        f"magnetic {metrics.magnetic_contribution*100:.0f}% "
        f"(peaks={metrics.total_peaks}, rate={metrics.peak_rate_hz:.2f}/s)."
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
        f"| Max cross-track | {metrics.max_cross_track_m:.1f} m |",
        f"| Mean confidence | {metrics.mean_confidence:.2f} |",
        f"| Sonar contribution | {metrics.sonar_contribution*100:.0f}% |",
        f"| Magnetic contribution | {metrics.magnetic_contribution*100:.0f}% |",
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
        "| Case | Health | Mean err [deg] | TRACK % | Switches | Peaks | Sonar % | Mag % | Max XT [m] |",
        "|------|--------|----------------|---------|----------|-------|---------|-------|------------|",
    ]
    for m in metrics_list:
        lines.append(
            f"| {m.case_name} | {health_score(m):.0f} | {m.mean_heading_error_deg:.1f} | "
            f"{m.track_active_fraction*100:.0f} | {m.mode_switches} | {m.total_peaks} | "
            f"{m.sonar_contribution*100:.0f} | {m.magnetic_contribution*100:.0f} | {m.max_cross_track_m:.1f} |"
        )
    lines.append("")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(lines), encoding="utf-8")
    return out_path
