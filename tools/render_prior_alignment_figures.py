"""Render D4 map-frame vs prior-alignment decoupling figures."""

from __future__ import annotations

import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
FIGURE_DIR = WORKSPACE_ROOT / "docs" / "figure"
LANE_RESULT_FILES = (
    WORKSPACE_ROOT / "results" / "20260705_lane_shortcut" / "lane_shortcut_prior_alignment_70.csv",
    WORKSPACE_ROOT / "results" / "20260705_lane_shortcut" / "lane_shortcut_prior_alignment_50.csv",
)

_C_BASELINE = "#2c6da4"
_C_ABLATION = "#c44e52"
_C_MAP = "#4c8c68"
_C_ALIGN = "#8172b3"
_C_GRAY = "#5f6368"


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


def _read_rows() -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for path in LANE_RESULT_FILES:
        if not path.exists():
            raise FileNotFoundError(path)
        with path.open(encoding="utf-8") as handle:
            rows.extend(csv.DictReader(handle))
    rows.sort(key=lambda row: (float(row["lane_spacing_m"]), row["variant"]))
    return rows


def _float(row: dict[str, str], key: str) -> float:
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return float("nan")


def _series(rows: list[dict[str, str]], variant: str, key: str) -> list[float]:
    return [_float(row, key) for row in rows if row["variant"] == variant]


def _save(fig: plt.Figure, stem: str) -> None:
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIGURE_DIR / f"{stem}.png", dpi=220, bbox_inches="tight")
    fig.savefig(FIGURE_DIR / f"{stem}.pdf", bbox_inches="tight")
    plt.close(fig)
    print(f"[fig] wrote {stem}.png / .pdf")


def render_decoupling(rows: list[dict[str, str]]) -> None:
    _apply_style()
    spacings = sorted({int(float(row["lane_spacing_m"])) for row in rows})
    x = np.arange(len(spacings))
    width = 0.34

    baseline_rows = [row for row in rows if row["variant"] == "baseline"]
    ablation_rows = [row for row in rows if row["variant"] == "no_prior_correction"]

    fig, axes = plt.subplots(2, 2, figsize=(7.4, 5.4))
    ax_jump, ax_map, ax_align, ax_health = axes.ravel()

    # A: route-progress jump under the legacy/global metric.
    b_route = _series(rows, "baseline", "route_progress_max_jump_m")
    a_route = _series(rows, "no_prior_correction", "route_progress_max_jump_m")
    ax_jump.bar(x - width / 2, b_route, width, color=_C_BASELINE, label="baseline")
    ax_jump.bar(x + width / 2, a_route, width, color=_C_ABLATION, label="no prior correction")
    ax_jump.set_yscale("log")
    ax_jump.set_ylabel("max route jump (m, log)")
    ax_jump.set_title("(a) Global progress detects task failure")
    ax_jump.set_xticks(x, [f"{s} m" for s in spacings])
    ax_jump.legend(loc="upper left")
    for xi, yi in zip(x + width / 2, a_route):
        ax_jump.annotate(f"{yi:.0f}", xy=(xi, yi), xytext=(0, 3), textcoords="offset points", ha="center", fontsize=8)

    # B: D4 map-frame projection stays continuous.
    b_map = _series(rows, "baseline", "map_frame_progress_max_jump_m")
    a_map = _series(rows, "no_prior_correction", "map_frame_progress_max_jump_m")
    ax_map.bar(x - width / 2, b_map, width, color=_C_BASELINE, label="baseline")
    ax_map.bar(x + width / 2, a_map, width, color=_C_MAP, label="D4 under ablation")
    ax_map.set_ylim(0.0, max(max(b_map), max(a_map), 0.2) + 0.12)
    ax_map.set_ylabel("map-frame jump (m)")
    ax_map.set_title("(b) D4 keeps projection continuous")
    ax_map.set_xticks(x, [f"{s} m" for s in spacings])
    ax_map.legend(loc="upper left")
    for xi, yi in zip(np.r_[x - width / 2, x + width / 2], b_map + a_map):
        ax_map.annotate(f"{yi:.1f}", xy=(xi, yi), xytext=(0, 3), textcoords="offset points", ha="center", fontsize=8)

    # C: prior alignment is the physical pull-back state.
    b_trans = _series(rows, "baseline", "prior_alignment_final_translation_m")
    a_trans = _series(rows, "no_prior_correction", "prior_alignment_final_translation_m")
    b_rot = _series(rows, "baseline", "prior_alignment_final_rotation_deg")
    ax_align.bar(x - width / 2, b_trans, width, color=_C_ALIGN, label="baseline translation")
    ax_align.bar(x + width / 2, a_trans, width, color="#d0d0d0", label="ablation translation")
    ax_align.set_ylim(0.0, max(b_trans + a_trans) + 1.0)
    ax_align.set_ylabel("final translation correction (m)")
    ax_align.set_title("(c) PriorAlignment pulls the map")
    ax_align.set_xticks(x, [f"{s} m" for s in spacings])
    ax_align.legend(loc="lower right")
    ax_rot = ax_align.twinx()
    ax_rot.plot(x, b_rot, "o--", color=_C_GRAY, lw=1.2, label="baseline rotation")
    ax_rot.set_ylabel("rotation correction (deg)")
    ax_rot.grid(False)
    for xi, yi in zip(x, b_trans):
        ax_align.annotate(f"{yi:.2f} m", xy=(xi - width / 2, yi), xytext=(0, 3), textcoords="offset points", ha="center", fontsize=8)
    for xi, yi in zip(x, b_rot):
        ax_rot.annotate(f"{yi:.2f}°", xy=(xi, yi), xytext=(0, -12), textcoords="offset points", ha="center", fontsize=8)

    # D: task health follows alignment, not projection continuity alone.
    b_health = _series(rows, "baseline", "health")
    a_health = _series(rows, "no_prior_correction", "health")
    b_accept = _series(rows, "baseline", "prior_alignment_accept_fraction")
    a_accept = _series(rows, "no_prior_correction", "prior_alignment_accept_fraction")
    ax_health.scatter(b_accept, b_health, s=70, color=_C_BASELINE, marker="o", label="baseline")
    ax_health.scatter(a_accept, a_health, s=70, color=_C_ABLATION, marker="x", linewidths=2.0, label="no prior correction")
    label_offsets = {
        ("baseline", "50"): (-24, -7),
        ("baseline", "70"): (8, 4),
        ("no_prior_correction", "50"): (-26, -8),
        ("no_prior_correction", "70"): (8, 5),
    }
    for row in baseline_rows + ablation_rows:
        spacing_label = str(int(float(row["lane_spacing_m"])))
        ax_health.annotate(
            f"{spacing_label}m",
            xy=(_float(row, "prior_alignment_accept_fraction"), _float(row, "health")),
            xytext=label_offsets.get((row["variant"], spacing_label), (5, 4)),
            textcoords="offset points",
            fontsize=8,
        )
    ax_health.set_xlim(-0.08, 1.08)
    ax_health.set_ylim(30.0, 88.0)
    ax_health.set_xlabel("prior-alignment accept fraction")
    ax_health.set_ylabel("task health score")
    ax_health.set_title("(d) Health requires physical alignment")
    ax_health.legend(loc="lower right")

    fig.suptitle("D4 projection safety vs online prior alignment", y=1.02, fontsize=11)
    fig.tight_layout()
    _save(fig, "fig_d4_prior_alignment_decoupling")


def main() -> None:
    rows = _read_rows()
    render_decoupling(rows)


if __name__ == "__main__":
    main()
