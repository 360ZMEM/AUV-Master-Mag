"""Render B1 / B2 / B3 figures for the pure-magnetic stress-test study.

Outputs (PNG + PDF) land in ``docs/figure/`` so the thesis docs stay
self-contained and version-controlled:

  * ``fig_b1_failure_timeseries``  -- why pure magnetic fails when the online
    prior correction is ablated under a heavy distorted prior (cross-track and
    route-progress time series, baseline vs ablation).
  * ``fig_b2_radius_boundary``     -- task health vs bend radius for the
    tightening-arc family (mid / heavy prior tiers).
  * ``fig_b3_ablation_health``     -- leave-one-out ablation health bars.

The script reads the already-produced sweep CSVs for B2 / B3 and re-runs two
deterministic simulations for the B1 time series.
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.config import build_default_scenarios  # noqa: E402
from auv_mag_tracking.viz.recorder import simulate_run  # noqa: E402

FIGURE_DIR = WORKSPACE_ROOT / "docs" / "figure"
RADIUS_CSV = WORKSPACE_ROOT / "results" / "20260630_radius_boundary" / "radius_sweep.csv"
ABLATION_CSV = WORKSPACE_ROOT / "results" / "20260630_ablation" / "ablation_sweep.csv"

_C_BASELINE = "#1f6fb2"
_C_ABLATION = "#c62828"
_C_MID = "#1f6fb2"
_C_HEAVY = "#d1601a"


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


def _save(fig: plt.Figure, stem: str) -> None:
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIGURE_DIR / f"{stem}.png", dpi=200, bbox_inches="tight")
    fig.savefig(FIGURE_DIR / f"{stem}.pdf", bbox_inches="tight")
    plt.close(fig)
    print(f"[fig] wrote {stem}.png / .pdf")


def _cross_track(record) -> np.ndarray:
    return np.hypot(
        record["pos_x_m"] - record["true_nearest_x_m"],
        record["pos_y_m"] - record["true_nearest_y_m"],
    )


def render_b1_failure_timeseries() -> None:
    """Baseline vs no-prior-correction (heavy tier) time series."""
    scenarios = build_default_scenarios()
    base = scenarios["case_maze_sonar_dropout_prior_heavy"]

    record_ok = simulate_run(base)

    import copy

    ablated = copy.deepcopy(base)
    ablated.name = "case_maze_sonar_dropout_prior_heavy__no_prior_correction"
    ablated.tracking.nominal_route_prior_observation_correction_enabled = False
    record_bad = simulate_run(ablated)

    t_ok = record_ok["time_s"]
    t_bad = record_bad["time_s"]
    xt_ok = _cross_track(record_ok)
    xt_bad = _cross_track(record_bad)
    rp_ok = record_ok["route_progress_m"]
    rp_bad = record_bad["route_progress_m"]

    # Locate the largest single-step route-progress jump in the ablated run.
    deltas = np.diff(rp_bad)
    finite = np.isfinite(deltas)
    jump_idx = int(np.argmax(np.where(finite, deltas, -np.inf)))
    jump_t = float(t_bad[jump_idx + 1])
    jump_val = float(deltas[jump_idx])

    fig, (ax_xt, ax_rp) = plt.subplots(2, 1, figsize=(7.0, 5.2), sharex=True)
    _apply_style()

    ax_xt.plot(t_ok, xt_ok, color=_C_BASELINE, lw=1.4, label="baseline (online prior correction ON)")
    ax_xt.plot(t_bad, xt_bad, color=_C_ABLATION, lw=1.4, label="ablation (prior correction OFF)")
    ax_xt.set_ylabel("cross-track error (m)")
    ax_xt.set_title("B1: pure-magnetic failure under a distorted heavy prior")
    ax_xt.legend(loc="upper left")

    ax_rp.plot(t_ok, rp_ok, color=_C_BASELINE, lw=1.4, label="baseline route progress")
    ax_rp.plot(t_bad, rp_bad, color=_C_ABLATION, lw=1.4, label="ablation route progress")
    if jump_val > 25.0:
        ax_rp.axvline(jump_t, color="#555555", ls="--", lw=1.0)
        ax_rp.annotate(
            f"lane shortcut\n+{jump_val:.1f} m jump @ {jump_t:.0f} s",
            xy=(jump_t, float(rp_bad[jump_idx + 1])),
            xytext=(0.55, 0.25),
            textcoords="axes fraction",
            fontsize=8,
            arrowprops=dict(arrowstyle="->", color="#555555", lw=0.9),
        )
    ax_rp.set_ylabel("route progress (m)")
    ax_rp.set_xlabel("time (s)")
    ax_rp.legend(loc="upper left")

    fig.tight_layout()
    _save(fig, "fig_b1_failure_timeseries")


def render_b2_radius_boundary() -> None:
    if not RADIUS_CSV.exists():
        print(f"[fig] skip B2: {RADIUS_CSV} missing")
        return
    rows = list(csv.DictReader(RADIUS_CSV.open(encoding="utf-8")))
    fig, ax = plt.subplots(figsize=(6.4, 4.0))
    _apply_style()
    for tier, color in (("mid", _C_MID), ("heavy", _C_HEAVY)):
        tier_rows = sorted(
            (r for r in rows if r["prior_tier"] == tier),
            key=lambda r: float(r["radius_m"]),
        )
        radii = [float(r["radius_m"]) for r in tier_rows]
        health = [float(r["health"]) for r in tier_rows]
        passed = [int(r["passed"]) for r in tier_rows]
        ax.plot(radii, health, "-o", color=color, lw=1.4, label=f"{tier} prior tier")
        for x, y, p in zip(radii, health, passed):
            if not p:
                ax.plot(x, y, "x", color="#000000", ms=8, mew=1.6)
    ax.axvline(30.0, color="#888888", ls=":", lw=1.0)
    ax.annotate("environment hard\nfloor = 30 m", xy=(30.0, ax.get_ylim()[0]),
                xytext=(34, ax.get_ylim()[0] + 1.5), fontsize=8)
    ax.set_xlabel("bend radius (m)")
    ax.set_ylabel("task health score")
    ax.set_title("B2: pure-magnetic health vs bend radius (90$^\\circ$ tightening arc)")
    ax.invert_xaxis()
    ax.legend(loc="lower left")
    fig.tight_layout()
    _save(fig, "fig_b2_radius_boundary")


def render_b3_ablation_health() -> None:
    if not ABLATION_CSV.exists():
        print(f"[fig] skip B3: {ABLATION_CSV} missing")
        return
    rows = list(csv.DictReader(ABLATION_CSV.open(encoding="utf-8")))
    order = [
        ("baseline_all_on", "baseline\n(all on)"),
        ("no_progress_guard", "no progress\nguard"),
        ("no_prior_correction", "no prior\ncorrection"),
        ("no_magnetic_path", "no magnetic\npath (sonar only)"),
        ("no_zigzag", "no adaptive\nzig-zag"),
    ]
    labels = [lbl for _, lbl in order]
    x = np.arange(len(order))
    width = 0.38

    fig, ax = plt.subplots(figsize=(7.6, 4.2))
    _apply_style()
    for offset, tier, color in ((-width / 2, "mid", _C_MID), (width / 2, "heavy", _C_HEAVY)):
        by_variant = {r["variant"]: r for r in rows if r["tier"] == tier}
        health = [float(by_variant[v]["health"]) for v, _ in order]
        passed = [int(by_variant[v]["passed"]) for v, _ in order]
        bars = ax.bar(x + offset, health, width, color=color, label=f"{tier} prior tier")
        for bar, p in zip(bars, passed):
            if not p:
                bar.set_hatch("///")
                bar.set_edgecolor("#000000")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("task health score")
    ax.set_title("B3: leave-one-out ablation (hatched = task failed)")
    ax.legend(loc="upper right")
    fig.tight_layout()
    _save(fig, "fig_b3_ablation_health")


def main() -> None:
    render_b1_failure_timeseries()
    render_b2_radius_boundary()
    render_b3_ablation_health()


if __name__ == "__main__":
    main()
