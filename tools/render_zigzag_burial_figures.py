"""Render figures for the independent zig-zag burial-depth experiment."""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CSV = WORKSPACE_ROOT / "results" / "20260705_zigzag_burial" / "zigzag_burial_sweep.csv"
FIGURE_DIR = WORKSPACE_ROOT / "docs" / "figure"
DL_T_1278_TARGET_M = 0.15

_C_BLUE = "#1f6fb2"
_C_ORANGE = "#d1601a"
_C_GREEN = "#2e7d32"
_C_RED = "#c62828"


def _apply_style() -> None:
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman", "DejaVu Serif"],
        "mathtext.fontset": "stix",
        "font.size": 9.0,
        "axes.linewidth": 0.9,
        "axes.grid": True,
        "grid.alpha": 0.25,
        "legend.fontsize": 8.0,
        "figure.facecolor": "white",
        "savefig.facecolor": "white",
    })


def _read_rows(csv_path: Path) -> list[dict[str, str]]:
    with csv_path.open(encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _float(row: dict[str, str], key: str) -> float:
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return float("nan")


def _save(fig: plt.Figure, stem: str) -> None:
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIGURE_DIR / f"{stem}.png", dpi=200, bbox_inches="tight")
    fig.savefig(FIGURE_DIR / f"{stem}.pdf", bbox_inches="tight")
    plt.close(fig)
    print(f"[fig] wrote {stem}.png / .pdf")


def render_error_vs_angle(rows: list[dict[str, str]]) -> None:
    _apply_style()
    grouped: dict[float, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        grouped[_float(row, "zigzag_angle_deg")].append(row)

    angles = sorted(grouped)
    means = []
    p90s = []
    passed = []
    for angle in angles:
        errors = np.array([_float(row, "cycle_burial_mae_m") for row in grouped[angle]], dtype=float)
        errors = errors[np.isfinite(errors)]
        pass_values = [int(row.get("passed_dl_t_1278", "0")) for row in grouped[angle]]
        means.append(float(np.mean(errors)) if errors.size else np.nan)
        p90s.append(float(np.percentile(errors, 90.0)) if errors.size else np.nan)
        passed.append(any(value > 0 for value in pass_values))

    fig, ax = plt.subplots(figsize=(6.4, 4.0))
    ax.plot(angles, means, "-o", color=_C_BLUE, lw=1.5, label="mean cycle MAE")
    ax.plot(angles, p90s, "--s", color=_C_ORANGE, lw=1.2, label="depth-grid p90 MAE")
    ax.axhline(DL_T_1278_TARGET_M, color=_C_RED, ls=":", lw=1.2, label="0.15 m target")
    for angle, mean, ok in zip(angles, means, passed):
        if ok and math.isfinite(mean):
            ax.plot(angle, mean, marker="*", color=_C_GREEN, ms=11)
    ax.set_xlabel("TRACK zig-zag angle (deg)")
    ax.set_ylabel("burial-depth absolute error (m)")
    ax.set_title("Zig-zag burial estimation: error vs amplitude")
    ax.legend(loc="upper right")
    fig.tight_layout()
    _save(fig, "fig_zigzag_burial_error_vs_angle")


def render_cycle_timeseries(rows: list[dict[str, str]]) -> None:
    _apply_style()
    # This figure is intentionally CSV-derived: it shows per-run cycle posteriors
    # ordered by angle for the nominal 1.5 m depth.
    nominal = [row for row in rows if abs(_float(row, "burial_depth_true_m") - 1.5) < 1e-9]
    nominal.sort(key=lambda row: _float(row, "zigzag_angle_deg"))
    angles = [_float(row, "zigzag_angle_deg") for row in nominal]
    estimate = [
        1.5 + _float(row, "cycle_burial_p50_abs_error_m")
        if math.isfinite(_float(row, "cycle_burial_p50_abs_error_m"))
        else np.nan
        for row in nominal
    ]
    sigma = [_float(row, "cycle_burial_mean_sigma_m") for row in nominal]
    quality = [_float(row, "cycle_burial_mean_quality") for row in nominal]

    fig, ax = plt.subplots(figsize=(6.4, 4.0))
    ax.axhline(1.5, color="#222222", ls=":", lw=1.2, label="true burial = 1.5 m")
    ax.errorbar(
        angles,
        estimate,
        yerr=np.nan_to_num(sigma, nan=0.0),
        fmt="-o",
        color=_C_BLUE,
        ecolor="#888888",
        capsize=3,
        label="cycle posterior (median abs offset shown above truth)",
    )
    for x, y, q in zip(angles, estimate, quality):
        if math.isfinite(y) and math.isfinite(q):
            ax.annotate(f"q={q:.2f}", xy=(x, y), xytext=(0, 6), textcoords="offset points", ha="center", fontsize=8)
    ax.set_xlabel("TRACK zig-zag angle (deg)")
    ax.set_ylabel("burial-depth estimate proxy (m)")
    ax.set_title("Illustrative cycle-local burial posterior (1.5 m depth)")
    ax.legend(loc="upper left")
    fig.tight_layout()
    _save(fig, "fig_zigzag_burial_cycle_timeseries")


def render_tradeoff(rows: list[dict[str, str]]) -> None:
    _apply_style()
    fig, ax = plt.subplots(figsize=(6.4, 4.0))
    for depth, color in ((1.0, _C_BLUE), (1.5, _C_ORANGE), (2.0, _C_GREEN)):
        depth_rows = [row for row in rows if abs(_float(row, "burial_depth_true_m") - depth) < 1e-9]
        depth_rows.sort(key=lambda row: _float(row, "zigzag_angle_deg"))
        x = [_float(row, "track_mean_cross_track_m") for row in depth_rows]
        y = [_float(row, "cycle_burial_mae_m") for row in depth_rows]
        labels = [_float(row, "zigzag_angle_deg") for row in depth_rows]
        ax.plot(x, y, "-o", color=color, lw=1.2, label=f"true depth {depth:.1f} m")
        for xi, yi, angle in zip(x, y, labels):
            if math.isfinite(xi) and math.isfinite(yi):
                ax.annotate(f"{angle:.0f}°", xy=(xi, yi), xytext=(3, 3), textcoords="offset points", fontsize=8)
    ax.axhline(DL_T_1278_TARGET_M, color=_C_RED, ls=":", lw=1.2, label="0.15 m target")
    ax.set_xlabel("TRACK mean cross-track error (m)")
    ax.set_ylabel("cycle burial MAE (m)")
    ax.set_title("Burial accuracy vs tracking-quality tradeoff")
    ax.legend(loc="upper right")
    fig.tight_layout()
    _save(fig, "fig_zigzag_burial_tradeoff")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV)
    args = parser.parse_args()
    rows = _read_rows(args.csv)
    render_error_vs_angle(rows)
    render_cycle_timeseries(rows)
    render_tradeoff(rows)


if __name__ == "__main__":
    main()
