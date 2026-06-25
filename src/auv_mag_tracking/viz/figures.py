"""Static academic-style figure rendering for the visualization system.

This is the *only* module in :mod:`auv_mag_tracking.viz` that imports matplotlib
(drawing single-point rule).  Style targets IEEE/HKU publication figures: serif
fonts, semantic cold/warm colour coding (sonar domain = cold, magnetic domain =
warm), mathtext formulae, thick borders and 1.5:1–2:1 aspect ratios so panels
stay legible when scaled down.

Two tiers are produced per run:
  * ``overview``  — 4-panel digest for slides / paper (thick borders).
  * ``detail``    — 9-panel diagnostic dashboard.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .metrics import HealthMetrics, ProgressDelta, health_score
from .recorder import RunRecord

# --- Semantic palette (cold = sonar / geometry, warm = magnetic) ---
_C_TRUTH = "#1a1a1a"
_C_AUV = "#1f6fb2"        # cold blue  — vehicle / sonar domain
_C_MAGNETIC = "#d1601a"   # warm orange — magnetic domain
_C_FIT = "#7b1fa2"        # purple — fused estimate
_C_GOOD = "#2e7d32"
_C_WARN = "#ef9a00"
_C_BAD = "#c62828"

_MODE_COLORS = {
    "search": "#9ecae1",     # light cold
    "align": "#fdae6b",      # light warm
    "track": "#31a354",      # green = converged
    "emergency": "#c62828",
}
_MODE_ORDER = ["search", "align", "track", "emergency"]


def _apply_style() -> None:
    """全局套用学术出版风格（衬线字体、粗边框、mathtext）。"""
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman", "DejaVu Serif"],
        "mathtext.fontset": "stix",
        "axes.linewidth": 1.4,
        "axes.titlesize": 12,
        "axes.labelsize": 11,
        "axes.grid": True,
        "grid.alpha": 0.3,
        "legend.fontsize": 8,
        "figure.facecolor": "white",
        "savefig.facecolor": "white",
    })


def _mode_spans(modes: List[str]) -> List[tuple]:
    """把模式序列折算为连续区间 ``(start_idx, end_idx, mode)``。"""
    spans: List[tuple] = []
    if not modes:
        return spans
    start = 0
    for i in range(1, len(modes)):
        if modes[i] != modes[start]:
            spans.append((start, i, modes[start]))
            start = i
    spans.append((start, len(modes), modes[start]))
    return spans


def _shade_modes(ax, t: np.ndarray, modes: List[str]) -> None:
    """在时间轴背景上用半透明色带标注三态 FSM 区间。"""
    for start, end, mode in _mode_spans(modes):
        ax.axvspan(t[start], t[min(end, len(t) - 1)],
                   color=_MODE_COLORS.get(mode, "#dddddd"), alpha=0.25, lw=0)


def _mode_legend_handles():
    import matplotlib.patches as mpatches
    return [mpatches.Patch(color=_MODE_COLORS[m], alpha=0.45, label=m) for m in _MODE_ORDER]


def render_overview(record: RunRecord, metrics: HealthMetrics, out_path: Path) -> Path:
    """渲染 4 面板总览版（投影/汇报用，1.8:1，粗边框）。"""
    _apply_style()
    t = record["time_s"]
    fig = plt.figure(figsize=(15, 8.5))
    score = health_score(metrics)
    mode_tag = "Deployment" if record.deployment_mode else "Nominal"
    fig.suptitle(
        f"AUV Cable Tracking — {record.case_name} ({mode_tag})   "
        f"Health {score:.0f}/100   "
        rf"fused $\bar{{e}}_\psi={metrics.mean_heading_error_deg:.1f}^\circ$   "
        rf"TRACK XT={metrics.track_mean_cross_track_m:.1f} m   "
        f"TRACK {metrics.track_active_fraction*100:.0f}%",
        fontsize=14, fontweight="bold",
    )
    gs = fig.add_gridspec(2, 2, hspace=0.32, wspace=0.22,
                          left=0.07, right=0.97, top=0.9, bottom=0.08)

    # Panel A: trajectory vs true cable
    ax = fig.add_subplot(gs[0, 0])
    ax.plot(record.cable_route_xy_m[:, 0], record.cable_route_xy_m[:, 1],
            color=_C_TRUTH, lw=2.2, label="True cable")
    ax.plot(record["pos_x_m"], record["pos_y_m"], color=_C_AUV, lw=1.4, label="AUV track")
    peak_idx = np.where(record["peak_detected"] > 0)[0]
    if peak_idx.size:
        ax.scatter(record["pos_x_m"][peak_idx], record["pos_y_m"][peak_idx],
                   color=_C_MAGNETIC, s=14, zorder=5, label="Magnetic peaks")
    ax.scatter([record["pos_x_m"][0]], [record["pos_y_m"][0]], color=_C_GOOD,
               s=55, marker="o", zorder=6, label="Start")
    ax.set_xlabel("North [m]"); ax.set_ylabel("East [m]")
    ax.set_title("(a) Trajectory vs. true cable")
    ax.legend(loc="best"); ax.set_aspect("equal", adjustable="datalim")

    # Panel B: heading error with FSM shading
    ax = fig.add_subplot(gs[0, 1])
    _shade_modes(ax, t, record.modes)
    ax.plot(t, metrics.heading_errors_deg, color=_C_FIT, lw=1.1)
    ax.axhline(15.0, color=_C_GOOD, ls="--", lw=1.2, label=r"$15^\circ$ target")
    ax.set_xlabel("Time [s]"); ax.set_ylabel(r"$|e_\psi|$ [deg]")
    ax.set_ylim(0, max(40.0, np.nanmax(metrics.heading_errors_deg) * 1.1 if np.any(~np.isnan(metrics.heading_errors_deg)) else 40.0))
    ax.set_title("(b) Heading error (shaded = FSM state)")
    ax.legend(handles=ax.get_legend_handles_labels()[0] + _mode_legend_handles(), loc="upper right", ncol=2)

    # Panel C: cross-track containment
    ax = fig.add_subplot(gs[1, 0])
    cross_track = np.hypot(record["pos_x_m"] - record["true_nearest_x_m"],
                           record["pos_y_m"] - record["true_nearest_y_m"])
    _shade_modes(ax, t, record.modes)
    ax.plot(t, cross_track, color=_C_AUV, lw=1.2)
    ax.axhline(metrics.mean_cross_track_m, color=_C_WARN, ls=":", lw=1.2,
               label=rf"mean ${metrics.mean_cross_track_m:.1f}\,$m")
    ax.set_xlabel("Time [s]"); ax.set_ylabel("Cross-track [m]")
    ax.set_title("(c) Distance to true cable")
    ax.legend(loc="upper right")

    # Panel D: source contribution + confidence
    ax = fig.add_subplot(gs[1, 1])
    ax.plot(t, record["confidence"], color=_C_FIT, lw=1.3, label="Confidence")
    ax.fill_between(t, 0, record["safe_lock_active"], color=_C_BAD, alpha=0.15, label="Safe-lock")
    ax.set_ylim(-0.05, 1.1); ax.set_xlabel("Time [s]"); ax.set_ylabel("Confidence")
    sonar_pct = metrics.sonar_contribution * 100.0
    mag_pct = metrics.magnetic_contribution * 100.0
    ax.set_title(f"(d) Confidence  |  sonar {sonar_pct:.0f}% / magnetic {mag_pct:.0f}%")
    ax.legend(loc="lower right")

    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return out_path


def render_detail(record: RunRecord, metrics: HealthMetrics, out_path: Path) -> Path:
    """渲染 9 面板详细诊断版（工程排障用）。"""
    _apply_style()
    t = record["time_s"]
    fig = plt.figure(figsize=(16, 20))
    fig.suptitle(f"AUV Cable Tracking Diagnostic — {record.case_name}",
                 fontsize=15, fontweight="bold")
    gs = fig.add_gridspec(5, 2, hspace=0.45, wspace=0.24,
                          left=0.07, right=0.97, top=0.95, bottom=0.04)

    # 1: heading vs true
    ax = fig.add_subplot(gs[0, :])
    _shade_modes(ax, t, record.modes)
    ax.plot(t, record["true_heading_deg"], color=_C_TRUTH, lw=2.0, label="True")
    ax.plot(t, record["fused_heading_deg"], color=_C_FIT, lw=1.0, label="Fused est.")
    ax.plot(t, record["line_heading_deg"], color=_C_MAGNETIC, lw=0.8, alpha=0.7, label="Line fit")
    ax.set_ylabel("Heading [deg]"); ax.set_title("(1) Heading estimate vs. truth")
    ax.legend(loc="upper right", ncol=3)

    # 2: heading error
    ax = fig.add_subplot(gs[1, :])
    ax.plot(t, metrics.heading_errors_deg, color=_C_FIT, lw=1.0)
    for thr, c in ((15.0, _C_GOOD), (45.0, _C_WARN), (135.0, _C_BAD)):
        ax.axhline(thr, color=c, ls="--", lw=1.0)
    ax.set_ylabel(r"$|e_\psi|$ [deg]"); ax.set_ylim(0, 200)
    ax.set_title("(2) Heading error over time")

    # 3: SNR
    ax = fig.add_subplot(gs[2, 0])
    ax.plot(t, record["snr_db"], color=_C_MAGNETIC, lw=1.0)
    ax.axhline(6.0, color=_C_BAD, ls="--", lw=1.0, label=r"$6\,$dB")
    ax.set_ylabel("SNR [dB]"); ax.set_title("(3) Signal-to-noise ratio"); ax.legend(loc="lower right")

    # 4: fit perpendicular eigenvalue (LOCK->TRACK gate)
    ax = fig.add_subplot(gs[2, 1])
    ax.plot(t, record["fit_perp_eig_m2"], color=_C_FIT, lw=1.0)
    ax.axhline(1.0, color=_C_GOOD, ls="--", lw=1.0, label=r"$\lambda_\perp=1\,\mathrm{m}^2$ gate")
    ax.set_ylabel(r"$\lambda_\perp$ [m$^2$]"); ax.set_ylim(0, 5)
    ax.set_title("(4) Fit covariance proxy (LOCK→TRACK)"); ax.legend(loc="upper right")

    # 5: magnetic cross-track steering signal
    ax = fig.add_subplot(gs[3, 0])
    ax.plot(t, record["magnetic_cross_track_offset_m"], color=_C_MAGNETIC, lw=1.0)
    ax.axhline(0.0, color=_C_TRUTH, lw=0.8)
    ax.set_ylabel("Mag offset [m]"); ax.set_title(r"(5) Magnetic cross-track $y=(B_\downarrow/B_\perp)\,d$")

    # 6: tracking strength + peaks
    ax = fig.add_subplot(gs[3, 1])
    ax.plot(t, record["tracking_strength_nt"], color=_C_MAGNETIC, lw=1.0, label="Tracking strength")
    peak_idx = np.where(record["peak_detected"] > 0)[0]
    if peak_idx.size:
        ax.scatter(t[peak_idx], record["tracking_strength_nt"][peak_idx],
                   color=_C_BAD, s=10, zorder=5, label=f"Peaks ({peak_idx.size})")
    ax.set_ylabel("Field [nT]"); ax.set_title("(6) Tracking strength & peaks"); ax.legend(loc="upper right")

    # 7: FSM state timeline
    ax = fig.add_subplot(gs[4, 0])
    mode_num = {m: i for i, m in enumerate(_MODE_ORDER)}
    state_series = np.array([mode_num.get(m, -1) for m in record.modes], dtype=float)
    ax.plot(t, state_series, color=_C_AUV, lw=1.2, drawstyle="steps-post")
    ax.set_yticks(list(mode_num.values())); ax.set_yticklabels(list(mode_num.keys()))
    ax.set_xlabel("Time [s]"); ax.set_title(f"(7) FSM state ({metrics.mode_switches} switches)")

    # 8: trajectory
    ax = fig.add_subplot(gs[4, 1])
    ax.plot(record.cable_route_xy_m[:, 0], record.cable_route_xy_m[:, 1], color=_C_TRUTH, lw=2.0, label="True cable")
    ax.plot(record["pos_x_m"], record["pos_y_m"], color=_C_AUV, lw=1.0, label="AUV")
    ax.set_xlabel("North [m]"); ax.set_ylabel("East [m]")
    ax.set_title("(8) Trajectory"); ax.legend(loc="best"); ax.set_aspect("equal", adjustable="datalim")

    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out_path


def render_run(record: RunRecord, metrics: HealthMetrics, fig_dir: Path) -> Dict[str, Path]:
    """渲染单次运行的 overview + detail 两版图，返回路径字典。"""
    fig_dir.mkdir(parents=True, exist_ok=True)
    return {
        "overview": render_overview(record, metrics, fig_dir / f"{record.case_name}_overview.png"),
        "detail": render_detail(record, metrics, fig_dir / f"{record.case_name}_detail.png"),
    }


def render_showcase(metrics_list: List[HealthMetrics], out_path: Path) -> Path:
    """渲染跨 case 成果对照图（重构成果系统展示）。"""
    _apply_style()
    cases = [m.case_name for m in metrics_list]
    x = np.arange(len(cases))
    fig = plt.figure(figsize=(16, 9))
    fig.suptitle("Refactor Showcase — Phase 0–2 Results Across Scenarios",
                 fontsize=15, fontweight="bold")
    gs = fig.add_gridspec(2, 2, hspace=0.4, wspace=0.22,
                          left=0.07, right=0.97, top=0.9, bottom=0.1)

    # (a) fused-heading error vs task-level TRACK vehicle heading error
    ax = fig.add_subplot(gs[0, 0])
    width = 0.38
    ax.bar(x - width / 2, [m.mean_heading_error_deg for m in metrics_list], color=_C_FIT, width=width,
           label="Fused heading")
    ax.bar(x + width / 2, [m.track_mean_vehicle_heading_error_deg for m in metrics_list], color=_C_AUV, width=width,
           label="TRACK vehicle")
    ax.axhline(15.0, color=_C_GOOD, ls="--", lw=1.3, label=r"$15^\circ$ target")
    ax.set_xticks(x); ax.set_xticklabels(cases, rotation=20)
    ax.set_ylabel(r"$\bar{e}_\psi$ [deg]"); ax.set_title("(a) Perception vs. closed-loop heading"); ax.legend()

    # (b) FSM occupancy stacked
    ax = fig.add_subplot(gs[0, 1])
    bottom = np.zeros(len(cases))
    for mode in _MODE_ORDER:
        vals = np.array([m.mode_fraction.get(mode, 0.0) * 100.0 for m in metrics_list])
        ax.bar(x, vals, bottom=bottom, color=_MODE_COLORS[mode], label=mode, width=0.6)
        bottom += vals
    ax.set_xticks(x); ax.set_xticklabels(cases, rotation=20)
    ax.set_ylabel("FSM occupancy [%]"); ax.set_title("(b) Three-state FSM occupancy"); ax.legend(ncol=2)

    # (c) sonar vs magnetic contribution
    ax = fig.add_subplot(gs[1, 0])
    ax.bar(x - width / 2, [m.sonar_contribution * 100.0 for m in metrics_list], width,
           color=_C_AUV, label="Sonar")
    ax.bar(x + width / 2, [m.magnetic_contribution * 100.0 for m in metrics_list], width,
           color=_C_MAGNETIC, label="Magnetic")
    ax.set_xticks(x); ax.set_xticklabels(cases, rotation=20)
    ax.set_ylabel("Guidance contribution [%]"); ax.set_title("(c) Sonar + magnetic co-operation"); ax.legend()

    # (d) health score + mode switches
    ax = fig.add_subplot(gs[1, 1])
    ax.bar(x, [health_score(m) for m in metrics_list], color=_C_GOOD, width=0.6, label="Health score")
    ax2 = ax.twinx()
    ax2.plot(x, [m.mode_switches for m in metrics_list], color=_C_BAD, marker="o", lw=1.5, label="Mode switches")
    ax2.set_ylabel("Mode switches", color=_C_BAD)
    ax.set_xticks(x); ax.set_xticklabels(cases, rotation=20)
    ax.set_ylabel("Health score / 100"); ax.set_ylim(0, 100)
    ax.set_title("(d) Health score & FSM stability")
    ax.legend(loc="upper left"); ax2.legend(loc="upper right")

    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return out_path


# Progress-view semantic colours: warm-grey = before fix, cold-green = after fix.
_C_BEFORE = "#9e8e7a"
_C_AFTER = "#2e7d32"

_PROGRESS_PANELS = (
    ("switches", "(a) FSM mode switches", r"switches", r"$\leq 6$ target"),
    ("health", "(b) Health score", r"score / 100", r"$\geq 90$ target"),
    ("mean_err", "(c) Mean heading error", r"$\bar{e}_\psi$ [deg]", r"$15^\circ$ target"),
    ("track_pct", "(d) TRACK_ACTIVE occupancy", r"TRACK [%]", r"$30\%$ target"),
)


def render_progress(deltas: List[ProgressDelta], out_path: Path,
                    subtitle: str = "Phase 0–2G refactor gains") -> Path:
    """渲染 before→after 进度对照图（系统展示前序修复成果，本包唯一另一绘图入口）。"""
    _apply_style()
    cases = [d.case_name for d in deltas]
    x = np.arange(len(cases))
    width = 0.36
    fig = plt.figure(figsize=(16, 9))
    fig.suptitle(f"Refactor Progress — before → after  ({subtitle})",
                 fontsize=15, fontweight="bold")
    gs = fig.add_gridspec(2, 2, hspace=0.42, wspace=0.22,
                          left=0.07, right=0.97, top=0.9, bottom=0.1)

    for panel_idx, (field, title, ylabel, target_label) in enumerate(_PROGRESS_PANELS):
        ax = fig.add_subplot(gs[panel_idx // 2, panel_idx % 2])
        before = np.array([d.fields[field][0] for d in deltas], dtype=float)
        after = np.array([d.fields[field][1] for d in deltas], dtype=float)
        target = deltas[0].fields[field][5]

        ax.bar(x - width / 2, before, width, color=_C_BEFORE, label="before")
        ax.bar(x + width / 2, after, width, color=_C_AFTER, label="after")
        ax.axhline(target, color=_C_BAD, ls="--", lw=1.2, label=target_label)

        # Annotate the per-case improvement on top of the "after" bar.
        for xi, (b, a) in enumerate(zip(before, after)):
            if not np.isfinite(b) or not np.isfinite(a):
                continue
            delta = a - b
            ax.annotate(f"{delta:+.0f}" if abs(delta) >= 1 else f"{delta:+.1f}",
                        xy=(xi + width / 2, a), xytext=(0, 3),
                        textcoords="offset points", ha="center", fontsize=8,
                        color=_C_AFTER if (delta != 0) else "#666")

        ax.set_xticks(x); ax.set_xticklabels(cases, rotation=20)
        ax.set_ylabel(ylabel); ax.set_title(title)
        ax.legend(loc="best", ncol=2)

    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return out_path
