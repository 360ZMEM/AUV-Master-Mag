"""Static academic-style figure rendering for the visualization system.

This is the *only* module in :mod:`auv_mag_tracking.viz` that imports matplotlib
(drawing single-point rule).  Style targets IEEE/HKU publication figures: serif
fonts, semantic cold/warm colour coding (sonar domain = cold, magnetic domain =
warm), mathtext formulae, thick borders and 1.5:1-2:1 aspect ratios so panels
stay legible when scaled down.

Two integrated tiers per run:
  * ``overview``      - 4-panel digest for slides / paper (thick borders).
  * ``detail``        - 12-panel diagnostic dashboard.
  * ``selector_sync`` - 4-panel shadow-selector timing.

In addition, every panel is also rendered as a single-panel paper figure
(IEEE single-column ~3.5 in) with PNG and PDF dual format under
``figures/paper/``.  This keeps backward compatibility of the integrated
figures while providing publication-ready cropped panels.
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .metrics import HealthMetrics, ProgressDelta, health_score
from .recorder import RunRecord

# --- Semantic palette (cold = sonar / geometry, warm = magnetic) ---
_C_TRUTH = "#1a1a1a"
_C_AUV = "#1f6fb2"        # cold blue  - vehicle / sonar domain
_C_MAGNETIC = "#d1601a"   # warm orange - magnetic domain
_C_FIT = "#7b1fa2"        # purple - fused estimate
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


# Paper geometry: IEEE single-column ~3.5 in figure width.
_PAPER_FIGSIZE_SQUARE = (3.5, 3.0)
_PAPER_FIGSIZE_WIDE = (3.5, 2.4)
_PAPER_FIGSIZE_TIME = (3.5, 2.2)


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


def _apply_paper_style() -> None:
    """IEEE single-column 论文小图样式：8-9pt 字号、1.0 线粗、紧凑布局。"""
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman", "DejaVu Serif"],
        "mathtext.fontset": "stix",
        "font.size": 8.5,
        "axes.linewidth": 0.8,
        "axes.titlesize": 9.0,
        "axes.labelsize": 8.5,
        "xtick.labelsize": 7.5,
        "ytick.labelsize": 7.5,
        "xtick.major.width": 0.6,
        "ytick.major.width": 0.6,
        "lines.linewidth": 1.0,
        "lines.markersize": 3.0,
        "axes.grid": True,
        "grid.alpha": 0.25,
        "grid.linewidth": 0.4,
        "legend.fontsize": 6.8,
        "legend.frameon": True,
        "legend.framealpha": 0.85,
        "legend.handlelength": 1.6,
        "legend.borderpad": 0.3,
        "legend.labelspacing": 0.25,
        "figure.facecolor": "white",
        "savefig.facecolor": "white",
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.02,
    })


def _save_dual_format(fig: plt.Figure, base_path: Path, dpi: int = 300) -> Dict[str, Path]:
    """Save the figure both as PNG and PDF and return the two paths."""
    base_path.parent.mkdir(parents=True, exist_ok=True)
    png_path = base_path.with_suffix(".png")
    pdf_path = base_path.with_suffix(".pdf")
    fig.savefig(png_path, dpi=dpi)
    fig.savefig(pdf_path)
    plt.close(fig)
    return {"png": png_path, "pdf": pdf_path}


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


# ---------------------------------------------------------------------------
# Per-panel painters (no figure creation / save).  Both the integrated render_*
# functions and the per-panel paper renderers call these.  Painters never set
# titles with an "(a)/(1)" prefix; the caller decides the panel label.
# ---------------------------------------------------------------------------


def _draw_trajectory(ax, record: RunRecord) -> None:
    ax.plot(record.cable_route_xy_m[:, 0], record.cable_route_xy_m[:, 1],
            color=_C_TRUTH, lw=1.6, label="True cable")
    ax.plot(record["pos_x_m"], record["pos_y_m"], color=_C_AUV, lw=1.0, label="AUV track")
    peak_idx = np.where(record["peak_detected"] > 0)[0]
    if peak_idx.size:
        ax.scatter(record["pos_x_m"][peak_idx], record["pos_y_m"][peak_idx],
                   color=_C_MAGNETIC, s=10, zorder=5, label="Magnetic peaks")
    ax.scatter([record["pos_x_m"][0]], [record["pos_y_m"][0]], color=_C_GOOD,
               s=30, marker="o", zorder=6, label="Start")
    ax.set_xlabel("North [m]")
    ax.set_ylabel("East [m]")
    ax.legend(loc="best")
    ax.set_aspect("equal", adjustable="datalim")


def _draw_heading_error(ax, record: RunRecord, metrics: HealthMetrics) -> None:
    t = record["time_s"]
    _shade_modes(ax, t, record.modes)
    ax.plot(t, metrics.heading_errors_deg, color=_C_FIT, lw=1.0)
    ax.axhline(15.0, color=_C_GOOD, ls="--", lw=1.0, label=r"$15^\circ$ target")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"$|e_\psi|$ [deg]")
    ymax = 40.0
    if np.any(~np.isnan(metrics.heading_errors_deg)):
        ymax = max(40.0, float(np.nanmax(metrics.heading_errors_deg)) * 1.1)
    ax.set_ylim(0, ymax)
    handles, _ = ax.get_legend_handles_labels()
    ax.legend(handles=handles + _mode_legend_handles(), loc="upper right", ncol=2)


def _draw_cross_track(ax, record: RunRecord, metrics: HealthMetrics) -> None:
    t = record["time_s"]
    cross_track = np.hypot(record["pos_x_m"] - record["true_nearest_x_m"],
                           record["pos_y_m"] - record["true_nearest_y_m"])
    _shade_modes(ax, t, record.modes)
    ax.plot(t, cross_track, color=_C_AUV, lw=1.0)
    ax.axhline(metrics.mean_cross_track_m, color=_C_WARN, ls=":", lw=1.0,
               label=rf"mean ${metrics.mean_cross_track_m:.1f}\,$m")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Cross-track [m]")
    ax.legend(loc="upper right")


def _draw_confidence(ax, record: RunRecord, metrics: HealthMetrics) -> None:
    t = record["time_s"]
    ax.plot(t, record["confidence"], color=_C_FIT, lw=1.0, label="Confidence")
    ax.fill_between(t, 0, record["safe_lock_active"], color=_C_BAD, alpha=0.15,
                    label="Safe-lock")
    ax.set_ylim(-0.05, 1.1)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Confidence")
    ax.legend(loc="lower right")


def _draw_heading_vs_truth(ax, record: RunRecord) -> None:
    t = record["time_s"]
    _shade_modes(ax, t, record.modes)
    ax.plot(t, record["true_heading_deg"], color=_C_TRUTH, lw=1.4, label="True")
    ax.plot(t, record["fused_heading_deg"], color=_C_FIT, lw=0.9, label="Fused est.")
    ax.plot(t, record["line_heading_deg"], color=_C_MAGNETIC, lw=0.7, alpha=0.7,
            label="Line fit")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Heading [deg]")
    ax.legend(loc="upper right", ncol=3)


def _draw_heading_error_thresholds(ax, record: RunRecord, metrics: HealthMetrics) -> None:
    t = record["time_s"]
    ax.plot(t, metrics.heading_errors_deg, color=_C_FIT, lw=0.9)
    for thr, c in ((15.0, _C_GOOD), (45.0, _C_WARN), (135.0, _C_BAD)):
        ax.axhline(thr, color=c, ls="--", lw=0.8)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"$|e_\psi|$ [deg]")
    ax.set_ylim(0, 200)


def _draw_snr(ax, record: RunRecord) -> None:
    t = record["time_s"]
    ax.plot(t, record["snr_db"], color=_C_MAGNETIC, lw=0.9)
    ax.axhline(6.0, color=_C_BAD, ls="--", lw=0.8, label=r"$6\,$dB")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("SNR [dB]")
    ax.legend(loc="lower right")


def _draw_fit_eig(ax, record: RunRecord) -> None:
    t = record["time_s"]
    ax.plot(t, record["fit_perp_eig_m2"], color=_C_FIT, lw=0.9)
    ax.axhline(1.0, color=_C_GOOD, ls="--", lw=0.8,
               label=r"$\lambda_\perp=1\,\mathrm{m}^2$ gate")
    ax.set_ylim(0, 5)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"$\lambda_\perp$ [m$^2$]")
    ax.legend(loc="upper right")


def _draw_mag_offset(ax, record: RunRecord) -> None:
    t = record["time_s"]
    ax.plot(t, record["magnetic_cross_track_offset_m"], color=_C_MAGNETIC, lw=0.9)
    ax.axhline(0.0, color=_C_TRUTH, lw=0.6)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Mag offset [m]")


def _draw_tracking_strength(ax, record: RunRecord) -> None:
    t = record["time_s"]
    ax.plot(t, record["tracking_strength_nt"], color=_C_MAGNETIC, lw=0.9,
            label="Tracking strength")
    peak_idx = np.where(record["peak_detected"] > 0)[0]
    if peak_idx.size:
        ax.scatter(t[peak_idx], record["tracking_strength_nt"][peak_idx],
                   color=_C_BAD, s=8, zorder=5, label=f"Peaks ({peak_idx.size})")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Field [nT]")
    ax.legend(loc="upper right")


def _draw_fsm_timeline(ax, record: RunRecord) -> None:
    t = record["time_s"]
    mode_num = {m: i for i, m in enumerate(_MODE_ORDER)}
    state_series = np.array([mode_num.get(m, -1) for m in record.modes], dtype=float)
    ax.plot(t, state_series, color=_C_AUV, lw=1.0, drawstyle="steps-post")
    ax.set_yticks(list(mode_num.values()))
    ax.set_yticklabels(list(mode_num.keys()))
    ax.set_xlabel("Time [s]")


def _draw_feed_reason(ax, record: RunRecord) -> None:
    t = record["time_s"]
    reason_code = record["magnetic_lookahead_feed_reason_code"]
    ax.plot(t, reason_code, color=_C_WARN, lw=0.9, drawstyle="steps-post")
    ax.plot(t, record["magnetic_lookahead_feed_allowed"], color=_C_GOOD, lw=0.8,
            alpha=0.8, label="Allowed")
    ax.set_yticks([1, 2, 3, 4, 5, 6, 7, 8, 9])
    ax.set_yticklabels(["ok", "no tgt", "off", "conf", "age", "phase", "resid",
                        "head", "innov"])
    ax.set_xlabel("Time [s]")
    ax.legend(loc="upper right")


def _draw_feed_margins(ax, record: RunRecord) -> None:
    t = record["time_s"]
    ax.plot(t, record["magnetic_lookahead_feed_axis_delta_deg"], color=_C_FIT,
            lw=0.9, label="Axis delta [deg]")
    ax.plot(t, record["magnetic_lookahead_feed_innovation_m"], color=_C_MAGNETIC,
            lw=0.9, label="Innovation [m]")
    ax.plot(t, record["magnetic_lookahead_feed_phase_age_s"], color=_C_WARN,
            lw=0.8, alpha=0.7, label="Phase age [s]")
    ax.set_xlabel("Time [s]")
    ax.legend(loc="upper right")


def _draw_probe_cycle(ax, record: RunRecord) -> None:
    t = record["time_s"]
    ax.plot(t, record["zigzag_probe_signed_cross_track_m"], color=_C_AUV,
            lw=0.8, label="Signed XT [m]")
    ax.plot(t, record["zigzag_probe_leg_sign"], color=_C_WARN, lw=0.8,
            drawstyle="steps-post", label="Leg sign")
    ax.plot(t, record["zigzag_probe_leg_route_delta_m"], color=_C_GOOD,
            lw=0.7, alpha=0.8, label="Leg route delta [m]")
    ax.plot(t, record["zigzag_probe_forward_phase_active"], color=_C_MAGNETIC,
            lw=0.7, drawstyle="steps-post", label="Forward phase")
    ax.plot(t, record["shadow_forward_zigzag_forward_dot"], color=_C_FIT,
            lw=0.7, alpha=0.8, label="Shadow forward dot")
    flip_idx = np.where(record["zigzag_probe_leg_flip_event"] > 0.5)[0]
    if flip_idx.size:
        ax.scatter(t[flip_idx], record["zigzag_probe_signed_cross_track_m"][flip_idx],
                   color=_C_BAD, s=10, zorder=5, label="Leg flip")
    forward_idx = np.where(record["zigzag_probe_magnetic_crossing_forward_leg_event"] > 0.5)[0]
    backward_idx = np.where(record["zigzag_probe_magnetic_crossing_backward_leg_event"] > 0.5)[0]
    if forward_idx.size:
        ax.scatter(t[forward_idx],
                   record["zigzag_probe_signed_cross_track_m"][forward_idx],
                   color=_C_GOOD, s=14, marker="^", zorder=6, label="X-fwd")
    if backward_idx.size:
        ax.scatter(t[backward_idx],
                   record["zigzag_probe_signed_cross_track_m"][backward_idx],
                   color=_C_BAD, s=14, marker="v", zorder=6, label="X-bwd")
    ax.set_xlabel("Time [s]")
    ax.legend(loc="upper right", ncol=2)


def _draw_route_progress(ax, record: RunRecord) -> None:
    t = record["time_s"]
    ax.plot(t, record["route_progress_m"], color=_C_GOOD, lw=1.0,
            label="Route progress")
    ax.plot(t, record["route_distance_m"], color=_C_WARN, lw=0.8,
            label="Route distance")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("[m]")
    ax.legend(loc="best")


def _draw_trajectory_compact(ax, record: RunRecord) -> None:
    ax.plot(record.cable_route_xy_m[:, 0], record.cable_route_xy_m[:, 1],
            color=_C_TRUTH, lw=1.4, label="True cable")
    ax.plot(record["pos_x_m"], record["pos_y_m"], color=_C_AUV, lw=0.8,
            label="AUV")
    ax.set_xlabel("North [m]")
    ax.set_ylabel("East [m]")
    ax.legend(loc="best")
    ax.set_aspect("equal", adjustable="datalim")


# --- selector_sync painters ---------------------------------------------------


def _draw_sel_route_progress(ax, record: RunRecord) -> None:
    t = record["time_s"]
    _shade_modes(ax, t, record.modes)
    ax.plot(t, record["route_progress_m"], color=_C_GOOD, lw=1.2,
            label="Route progress")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Progress [m]")
    ax.legend(loc="upper left")


def _draw_sel_progress_rate(ax, record: RunRecord) -> None:
    t = record["time_s"]
    _shade_modes(ax, t, record.modes)
    ax.plot(t, record["route_progress_rate_mps"], color=_C_AUV, lw=0.9,
            label="Progress rate")
    ax.axhline(0.0, color=_C_TRUTH, lw=0.6)
    ax.axhline(0.6, color=_C_GOOD, lw=0.7, ls="--", label="Readiness full-credit")
    ax.set_ylim(-1.5, 1.5)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Rate [m/s]")
    ax.legend(loc="upper right")


def _draw_sel_gate_overlap(ax, record: RunRecord, metrics: HealthMetrics) -> None:
    t = record["time_s"]
    _shade_modes(ax, t, record.modes)
    ax.plot(t, record["shadow_axis_hypothesis_valid"], color=_C_FIT, lw=0.8,
            drawstyle="steps-post", label="Hyp valid")
    ax.plot(t, record["shadow_axis_validation_passed"], color=_C_GOOD, lw=0.9,
            drawstyle="steps-post", label="Val pass")
    ax.plot(t, record["magnetic_lookahead_feed_allowed"], color=_C_WARN, lw=0.8,
            drawstyle="steps-post", label="Feed allowed")
    ax.plot(t, record["shadow_axis_dual_gate_passed"], color=_C_BAD, lw=0.9,
            drawstyle="steps-post", label="Dual pass")
    ax.plot(t, record["shadow_axis_progress_aligned_dual_gate_passed"],
            color=_C_TRUTH, lw=0.9, drawstyle="steps-post", label="Aligned dual")
    ax.plot(t, record["shadow_axis_progress_aligned_candidate_valid"],
            color=_C_MAGNETIC, lw=0.7, drawstyle="steps-post", label="Aligned cand")
    ax.set_ylim(-0.1, 1.25)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Gate")
    ax.legend(loc="upper right", ncol=3)


def _draw_sel_reasons(ax, record: RunRecord) -> None:
    t = record["time_s"]
    _shade_modes(ax, t, record.modes)
    ax.plot(t, record["shadow_axis_validation_reason_code"], color=_C_FIT,
            lw=0.8, drawstyle="steps-post", label="Val reason")
    ax.plot(t, record["shadow_axis_dual_gate_reason_code"], color=_C_BAD,
            lw=0.8, drawstyle="steps-post", label="Dual reason")
    ax.plot(t, record["shadow_axis_progress_alignment_reason_code"], color=_C_GOOD,
            lw=0.7, drawstyle="steps-post", label="Progress reason")
    ax.plot(t, record["shadow_axis_progress_aligned_candidate_reason_code"],
            color=_C_MAGNETIC, lw=0.7, drawstyle="steps-post", label="Aligned reason")
    ax.plot(t, record["shadow_axis_progress_proxy_valid"], color=_C_AUV, lw=0.7,
            drawstyle="steps-post", label="Proxy valid")
    ax.plot(t, record["shadow_axis_route_bound_proxy_valid"], color=_C_WARN,
            lw=0.7, drawstyle="steps-post", label="Route-bound valid")
    ax.plot(t, record["magnetic_lookahead_feed_reason_code"], color=_C_WARN,
            lw=0.6, alpha=0.8, drawstyle="steps-post", label="Feed reason")
    ax.set_yticks([1, 2, 3, 4, 5, 6, 7, 8, 9])
    ax.set_yticklabels(["pass/ok", "no hyp/val", "cands/feed", "score",
                        "margin/age", "stale/phase", "expired/resid",
                        "head", "innov"])
    ax.set_xlabel("Time [s]")
    ax.legend(loc="upper right", ncol=3)


# --- showcase painters --------------------------------------------------------


def _draw_showcase_heading_bar(ax, metrics_list: List[HealthMetrics]) -> None:
    cases = [m.case_name for m in metrics_list]
    x = np.arange(len(cases))
    width = 0.38
    ax.bar(x - width / 2, [m.mean_heading_error_deg for m in metrics_list],
           color=_C_FIT, width=width, label="Fused heading")
    ax.bar(x + width / 2, [m.track_mean_vehicle_heading_error_deg for m in metrics_list],
           color=_C_AUV, width=width, label="TRACK vehicle")
    ax.axhline(15.0, color=_C_GOOD, ls="--", lw=1.0, label=r"$15^\circ$ target")
    ax.set_xticks(x)
    ax.set_xticklabels(cases, rotation=20, ha="right")
    ax.set_ylabel(r"$\bar{e}_\psi$ [deg]")
    ax.legend()


def _draw_showcase_fsm_stack(ax, metrics_list: List[HealthMetrics]) -> None:
    cases = [m.case_name for m in metrics_list]
    x = np.arange(len(cases))
    bottom = np.zeros(len(cases))
    for mode in _MODE_ORDER:
        vals = np.array([m.mode_fraction.get(mode, 0.0) * 100.0 for m in metrics_list])
        ax.bar(x, vals, bottom=bottom, color=_MODE_COLORS[mode], label=mode,
               width=0.6)
        bottom += vals
    ax.set_xticks(x)
    ax.set_xticklabels(cases, rotation=20, ha="right")
    ax.set_ylabel("FSM occupancy [%]")
    ax.legend(ncol=2)


def _draw_showcase_contribution(ax, metrics_list: List[HealthMetrics]) -> None:
    cases = [m.case_name for m in metrics_list]
    x = np.arange(len(cases))
    width = 0.38
    ax.bar(x - width / 2, [m.sonar_contribution * 100.0 for m in metrics_list],
           width, color=_C_AUV, label="Sonar")
    ax.bar(x + width / 2, [m.magnetic_contribution * 100.0 for m in metrics_list],
           width, color=_C_MAGNETIC, label="Magnetic")
    ax.set_xticks(x)
    ax.set_xticklabels(cases, rotation=20, ha="right")
    ax.set_ylabel("Guidance contribution [%]")
    ax.legend()


def _draw_showcase_health(ax, metrics_list: List[HealthMetrics]) -> None:
    cases = [m.case_name for m in metrics_list]
    x = np.arange(len(cases))
    ax.bar(x, [health_score(m) for m in metrics_list], color=_C_GOOD, width=0.6,
           label="Health score")
    ax2 = ax.twinx()
    ax2.plot(x, [m.mode_switches for m in metrics_list], color=_C_BAD,
             marker="o", lw=1.2, label="Mode switches")
    ax2.set_ylabel("Mode switches", color=_C_BAD)
    ax.set_xticks(x)
    ax.set_xticklabels(cases, rotation=20, ha="right")
    ax.set_ylabel("Health score / 100")
    ax.set_ylim(0, 100)
    ax.legend(loc="upper left")
    ax2.legend(loc="upper right")


# --- progress painters --------------------------------------------------------


_PROGRESS_PANELS = (
    ("switches", "FSM mode switches", r"switches", r"$\leq 6$ target"),
    ("health", "Health score", r"score / 100", r"$\geq 90$ target"),
    ("mean_err", "Mean heading error", r"$\bar{e}_\psi$ [deg]", r"$15^\circ$ target"),
    ("track_pct", "TRACK_ACTIVE occupancy", r"TRACK [%]", r"$30\%$ target"),
)

_C_BEFORE = "#9e8e7a"
_C_AFTER = "#2e7d32"


def _draw_progress_panel(ax, deltas: List[ProgressDelta], field: str,
                         ylabel: str, target_label: str) -> None:
    cases = [d.case_name for d in deltas]
    x = np.arange(len(cases))
    width = 0.36
    before = np.array([d.fields[field][0] for d in deltas], dtype=float)
    after = np.array([d.fields[field][1] for d in deltas], dtype=float)
    target = deltas[0].fields[field][5]
    ax.bar(x - width / 2, before, width, color=_C_BEFORE, label="before")
    ax.bar(x + width / 2, after, width, color=_C_AFTER, label="after")
    ax.axhline(target, color=_C_BAD, ls="--", lw=1.0, label=target_label)
    for xi, (b, a) in enumerate(zip(before, after)):
        if not np.isfinite(b) or not np.isfinite(a):
            continue
        delta = a - b
        ax.annotate(f"{delta:+.0f}" if abs(delta) >= 1 else f"{delta:+.1f}",
                    xy=(xi + width / 2, a), xytext=(0, 3),
                    textcoords="offset points", ha="center", fontsize=7,
                    color=_C_AFTER if (delta != 0) else "#666")
    ax.set_xticks(x)
    ax.set_xticklabels(cases, rotation=20, ha="right")
    ax.set_ylabel(ylabel)
    ax.legend(loc="best", ncol=2)


# ---------------------------------------------------------------------------
# Integrated big-figure renderers (legacy outputs kept intact).
# ---------------------------------------------------------------------------


def render_overview(record: RunRecord, metrics: HealthMetrics, out_path: Path) -> Path:
    """渲染 4 面板总览版（投影/汇报用，1.8:1，粗边框）。"""
    _apply_style()
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

    ax = fig.add_subplot(gs[0, 0]); _draw_trajectory(ax, record)
    ax.set_title("(a) Trajectory vs. true cable")

    ax = fig.add_subplot(gs[0, 1]); _draw_heading_error(ax, record, metrics)
    ax.set_title("(b) Heading error (shaded = FSM state)")

    ax = fig.add_subplot(gs[1, 0]); _draw_cross_track(ax, record, metrics)
    ax.set_title("(c) Distance to true cable")

    ax = fig.add_subplot(gs[1, 1]); _draw_confidence(ax, record, metrics)
    sonar_pct = metrics.sonar_contribution * 100.0
    mag_pct = metrics.magnetic_contribution * 100.0
    ax.set_title(f"(d) Confidence  |  sonar {sonar_pct:.0f}% / magnetic {mag_pct:.0f}%")

    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return out_path


def render_detail(record: RunRecord, metrics: HealthMetrics, out_path: Path) -> Path:
    """渲染详细诊断版（工程排障用）。"""
    _apply_style()
    fig = plt.figure(figsize=(16, 28))
    fig.suptitle(f"AUV Cable Tracking Diagnostic — {record.case_name}",
                 fontsize=15, fontweight="bold")
    gs = fig.add_gridspec(7, 2, hspace=0.45, wspace=0.24,
                          left=0.07, right=0.97, top=0.95, bottom=0.04)

    ax = fig.add_subplot(gs[0, :]); _draw_heading_vs_truth(ax, record)
    ax.set_title("(1) Heading estimate vs. truth")

    ax = fig.add_subplot(gs[1, :]); _draw_heading_error_thresholds(ax, record, metrics)
    ax.set_title("(2) Heading error over time")

    ax = fig.add_subplot(gs[2, 0]); _draw_snr(ax, record)
    ax.set_title("(3) Signal-to-noise ratio")

    ax = fig.add_subplot(gs[2, 1]); _draw_fit_eig(ax, record)
    ax.set_title("(4) Fit covariance proxy (LOCK→TRACK)")

    ax = fig.add_subplot(gs[3, 0]); _draw_mag_offset(ax, record)
    ax.set_title(r"(5) Magnetic cross-track $y=(B_\downarrow/B_\perp)\,d$")

    ax = fig.add_subplot(gs[3, 1]); _draw_tracking_strength(ax, record)
    ax.set_title("(6) Tracking strength & peaks")

    ax = fig.add_subplot(gs[4, 0]); _draw_fsm_timeline(ax, record)
    ax.set_title(f"(7) FSM state ({metrics.mode_switches} switches)")

    ax = fig.add_subplot(gs[4, 1]); _draw_feed_reason(ax, record)
    ax.set_title("(8) Lookahead feed gate reason")

    ax = fig.add_subplot(gs[5, 0]); _draw_feed_margins(ax, record)
    ax.set_title("(9) Lookahead feed gate margins")

    ax = fig.add_subplot(gs[5, 1]); _draw_probe_cycle(ax, record)
    ax.set_title("(10) Zig-zag probe cycle")

    ax = fig.add_subplot(gs[6, 0]); _draw_route_progress(ax, record)
    ax.set_title("(11) Route progress and distance")

    ax = fig.add_subplot(gs[6, 1]); _draw_trajectory_compact(ax, record)
    ax.set_title("(12) Trajectory")

    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out_path


def render_selector_sync(record: RunRecord, metrics: HealthMetrics,
                         out_path: Path) -> Path:
    """Render shadow-selector timing against route progress and feed gates."""
    _apply_style()
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle(
        f"Shadow Selector Timing — {record.case_name}",
        fontsize=15, fontweight="bold",
    )
    gs = fig.add_gridspec(4, 1, hspace=0.34, left=0.07, right=0.97,
                          top=0.92, bottom=0.07)

    ax0 = fig.add_subplot(gs[0, 0]); _draw_sel_route_progress(ax0, record)
    ax0.set_title("(1) Route progress")

    ax1 = fig.add_subplot(gs[1, 0], sharex=ax0); _draw_sel_progress_rate(ax1, record)
    ax1.set_title("(2) Route progress rate")

    ax2 = fig.add_subplot(gs[2, 0], sharex=ax0); _draw_sel_gate_overlap(ax2, record, metrics)
    ax2.set_title(
        "(3) Selector/feed overlap  |  "
        f"dual pass {metrics.shadow_axis_dual_gate_pass_fraction*100:.1f}%, "
        f"validation reject {metrics.shadow_axis_dual_gate_reject_validation_fraction*100:.1f}%, "
        f"feed reject {metrics.shadow_axis_dual_gate_reject_feed_fraction*100:.1f}%"
    )

    ax3 = fig.add_subplot(gs[3, 0], sharex=ax0); _draw_sel_reasons(ax3, record)
    ax3.set_title("(4) Reject reason codes")

    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out_path


def render_showcase(metrics_list: List[HealthMetrics], out_path: Path) -> Path:
    """渲染跨 case 成果对照图（重构成果系统展示）。"""
    _apply_style()
    fig = plt.figure(figsize=(16, 9))
    fig.suptitle("Refactor Showcase — Phase 0–2 Results Across Scenarios",
                 fontsize=15, fontweight="bold")
    gs = fig.add_gridspec(2, 2, hspace=0.4, wspace=0.22,
                          left=0.07, right=0.97, top=0.9, bottom=0.1)

    ax = fig.add_subplot(gs[0, 0]); _draw_showcase_heading_bar(ax, metrics_list)
    ax.set_title("(a) Perception vs. closed-loop heading")

    ax = fig.add_subplot(gs[0, 1]); _draw_showcase_fsm_stack(ax, metrics_list)
    ax.set_title("(b) Three-state FSM occupancy")

    ax = fig.add_subplot(gs[1, 0]); _draw_showcase_contribution(ax, metrics_list)
    ax.set_title("(c) Sonar + magnetic co-operation")

    ax = fig.add_subplot(gs[1, 1]); _draw_showcase_health(ax, metrics_list)
    ax.set_title("(d) Health score & FSM stability")

    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return out_path


def render_progress(deltas: List[ProgressDelta], out_path: Path,
                    subtitle: str = "Phase 0–2G refactor gains") -> Path:
    """渲染 before→after 进度对照图（系统展示前序修复成果，本包唯一另一绘图入口）。"""
    _apply_style()
    fig = plt.figure(figsize=(16, 9))
    fig.suptitle(f"Refactor Progress — before → after  ({subtitle})",
                 fontsize=15, fontweight="bold")
    gs = fig.add_gridspec(2, 2, hspace=0.42, wspace=0.22,
                          left=0.07, right=0.97, top=0.9, bottom=0.1)
    for panel_idx, (field, title, ylabel, target_label) in enumerate(_PROGRESS_PANELS):
        ax = fig.add_subplot(gs[panel_idx // 2, panel_idx % 2])
        _draw_progress_panel(ax, deltas, field, ylabel, target_label)
        ax.set_title(f"({'abcd'[panel_idx]}) {title}")
    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return out_path


# ---------------------------------------------------------------------------
# Paper-figure renderers: single-panel, IEEE-column sized, PNG + PDF.
# ---------------------------------------------------------------------------


# Registry: (panel slug, painter callable, figsize, needs metrics flag).
# Each painter signature: (ax, record, metrics) for run panels.
_PAPER_RUN_PANELS: Sequence[Tuple[str, Callable, Tuple[float, float], str]] = (
    # overview tier ----------------------------------------------------------
    ("overview_trajectory",
     lambda ax, r, m: (_draw_trajectory(ax, r), ax.set_title("Trajectory vs. true cable"))[0],
     _PAPER_FIGSIZE_SQUARE, "overview"),
    ("overview_heading_error",
     lambda ax, r, m: (_draw_heading_error(ax, r, m), ax.set_title("Heading error"))[0],
     _PAPER_FIGSIZE_TIME, "overview"),
    ("overview_cross_track",
     lambda ax, r, m: (_draw_cross_track(ax, r, m), ax.set_title("Distance to true cable"))[0],
     _PAPER_FIGSIZE_TIME, "overview"),
    ("overview_confidence",
     lambda ax, r, m: (_draw_confidence(ax, r, m), ax.set_title("Confidence & safe-lock"))[0],
     _PAPER_FIGSIZE_TIME, "overview"),
    # detail tier ------------------------------------------------------------
    ("detail_heading_vs_truth",
     lambda ax, r, m: (_draw_heading_vs_truth(ax, r), ax.set_title("Heading estimate vs. truth"))[0],
     _PAPER_FIGSIZE_TIME, "detail"),
    ("detail_heading_error_thresholds",
     lambda ax, r, m: (_draw_heading_error_thresholds(ax, r, m),
                        ax.set_title("Heading error vs. thresholds"))[0],
     _PAPER_FIGSIZE_TIME, "detail"),
    ("detail_snr",
     lambda ax, r, m: (_draw_snr(ax, r), ax.set_title("Signal-to-noise ratio"))[0],
     _PAPER_FIGSIZE_TIME, "detail"),
    ("detail_fit_eig",
     lambda ax, r, m: (_draw_fit_eig(ax, r), ax.set_title(r"Fit covariance $\lambda_\perp$"))[0],
     _PAPER_FIGSIZE_TIME, "detail"),
    ("detail_mag_offset",
     lambda ax, r, m: (_draw_mag_offset(ax, r), ax.set_title("Magnetic cross-track offset"))[0],
     _PAPER_FIGSIZE_TIME, "detail"),
    ("detail_tracking_strength",
     lambda ax, r, m: (_draw_tracking_strength(ax, r), ax.set_title("Tracking strength & peaks"))[0],
     _PAPER_FIGSIZE_TIME, "detail"),
    ("detail_fsm_timeline",
     lambda ax, r, m: (_draw_fsm_timeline(ax, r),
                        ax.set_title(f"FSM state ({m.mode_switches} switches)"))[0],
     _PAPER_FIGSIZE_TIME, "detail"),
    ("detail_feed_reason",
     lambda ax, r, m: (_draw_feed_reason(ax, r),
                        ax.set_title("Lookahead feed gate reason"))[0],
     _PAPER_FIGSIZE_TIME, "detail"),
    ("detail_feed_margins",
     lambda ax, r, m: (_draw_feed_margins(ax, r),
                        ax.set_title("Lookahead feed gate margins"))[0],
     _PAPER_FIGSIZE_TIME, "detail"),
    ("detail_probe_cycle",
     lambda ax, r, m: (_draw_probe_cycle(ax, r), ax.set_title("Zig-zag probe cycle"))[0],
     _PAPER_FIGSIZE_TIME, "detail"),
    ("detail_route_progress",
     lambda ax, r, m: (_draw_route_progress(ax, r),
                        ax.set_title("Route progress and distance"))[0],
     _PAPER_FIGSIZE_TIME, "detail"),
    ("detail_trajectory_compact",
     lambda ax, r, m: (_draw_trajectory_compact(ax, r), ax.set_title("Trajectory"))[0],
     _PAPER_FIGSIZE_SQUARE, "detail"),
    # selector_sync tier -----------------------------------------------------
    ("selector_route_progress",
     lambda ax, r, m: (_draw_sel_route_progress(ax, r), ax.set_title("Route progress"))[0],
     _PAPER_FIGSIZE_TIME, "selector"),
    ("selector_progress_rate",
     lambda ax, r, m: (_draw_sel_progress_rate(ax, r), ax.set_title("Route progress rate"))[0],
     _PAPER_FIGSIZE_TIME, "selector"),
    ("selector_gate_overlap",
     lambda ax, r, m: (_draw_sel_gate_overlap(ax, r, m),
                        ax.set_title("Selector/feed gate overlap"))[0],
     _PAPER_FIGSIZE_TIME, "selector"),
    ("selector_reasons",
     lambda ax, r, m: (_draw_sel_reasons(ax, r), ax.set_title("Reject reason codes"))[0],
     _PAPER_FIGSIZE_TIME, "selector"),
)


def render_paper_run_panels(record: RunRecord, metrics: HealthMetrics,
                            paper_dir: Path) -> Dict[str, Dict[str, Path]]:
    """Emit per-panel single-figure PNG+PDF files under ``paper_dir``.

    Returns a mapping ``slug -> {"png": Path, "pdf": Path}``.
    """
    paper_dir.mkdir(parents=True, exist_ok=True)
    out: Dict[str, Dict[str, Path]] = {}
    for slug, painter, figsize, _tier in _PAPER_RUN_PANELS:
        _apply_paper_style()
        fig, ax = plt.subplots(figsize=figsize)
        painter(ax, record, metrics)
        fig.tight_layout()
        out[slug] = _save_dual_format(
            fig, paper_dir / f"{record.case_name}_{slug}",
        )
    return out


# Each showcase panel signature: (ax, metrics_list)
_PAPER_SHOWCASE_PANELS: Sequence[Tuple[str, Callable, Tuple[float, float]]] = (
    ("heading_bar",
     lambda ax, ml: (_draw_showcase_heading_bar(ax, ml),
                      ax.set_title("Perception vs. closed-loop heading"))[0],
     (3.5, 2.6)),
    ("fsm_stack",
     lambda ax, ml: (_draw_showcase_fsm_stack(ax, ml),
                      ax.set_title("FSM state occupancy"))[0],
     (3.5, 2.6)),
    ("contribution",
     lambda ax, ml: (_draw_showcase_contribution(ax, ml),
                      ax.set_title("Sonar / magnetic contribution"))[0],
     (3.5, 2.6)),
    ("health",
     lambda ax, ml: (_draw_showcase_health(ax, ml),
                      ax.set_title("Health score & FSM stability"))[0],
     (3.5, 2.6)),
)


def render_paper_showcase_panels(metrics_list: List[HealthMetrics],
                                 paper_dir: Path,
                                 slug_prefix: str = "showcase",
                                 ) -> Dict[str, Dict[str, Path]]:
    """Emit per-panel showcase PNG+PDF figures."""
    paper_dir.mkdir(parents=True, exist_ok=True)
    out: Dict[str, Dict[str, Path]] = {}
    for slug, painter, figsize in _PAPER_SHOWCASE_PANELS:
        _apply_paper_style()
        fig, ax = plt.subplots(figsize=figsize)
        painter(ax, metrics_list)
        fig.tight_layout()
        out[f"{slug_prefix}_{slug}"] = _save_dual_format(
            fig, paper_dir / f"{slug_prefix}_{slug}",
        )
    return out


def render_paper_progress_panels(deltas: List[ProgressDelta], paper_dir: Path,
                                 slug_prefix: str = "progress",
                                 ) -> Dict[str, Dict[str, Path]]:
    """Emit per-panel progress PNG+PDF figures."""
    paper_dir.mkdir(parents=True, exist_ok=True)
    out: Dict[str, Dict[str, Path]] = {}
    for field, title, ylabel, target_label in _PROGRESS_PANELS:
        _apply_paper_style()
        fig, ax = plt.subplots(figsize=(3.5, 2.6))
        _draw_progress_panel(ax, deltas, field, ylabel, target_label)
        ax.set_title(title)
        fig.tight_layout()
        out[f"{slug_prefix}_{field}"] = _save_dual_format(
            fig, paper_dir / f"{slug_prefix}_{field}",
        )
    return out


# ---------------------------------------------------------------------------
# Run-level orchestrator: emits integrated big figures *and* paper panels.
# ---------------------------------------------------------------------------


def render_run(record: RunRecord, metrics: HealthMetrics, fig_dir: Path,
               *, paper_panels: bool = True) -> Dict[str, object]:
    """渲染单次运行的整体大图（overview + detail + selector_sync），并可选输出
    论文风格的单面板拆分图（PNG+PDF 双格式）。

    Returns a dict with the integrated PNG paths plus, when ``paper_panels`` is
    enabled, a nested ``"paper"`` mapping of slug -> {"png", "pdf"} paths.
    """
    fig_dir.mkdir(parents=True, exist_ok=True)
    out: Dict[str, object] = {
        "overview": render_overview(record, metrics,
                                    fig_dir / f"{record.case_name}_overview.png"),
        "detail": render_detail(record, metrics,
                                fig_dir / f"{record.case_name}_detail.png"),
        "selector_sync": render_selector_sync(record, metrics,
                                              fig_dir / f"{record.case_name}_selector_sync.png"),
    }
    if paper_panels:
        out["paper"] = render_paper_run_panels(record, metrics, fig_dir / "paper")
    return out
