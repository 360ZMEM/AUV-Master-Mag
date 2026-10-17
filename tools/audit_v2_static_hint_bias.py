#!/usr/bin/env python3
"""Synthetic counterexample: a narrow but displaced static target interval.

This is an algorithm-level admission audit, not a ROS, PVS, or field trial.
The second source is a sum of scalar anomaly envelopes, not a multi-core cable
field model. Existing published runs are not changed.
"""
from __future__ import annotations

from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

MAG_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = MAG_ROOT.parent
sys.path.insert(0, str(MAG_ROOT / "src"))
sys.path.insert(0, str(PROJECT_ROOT / "tools"))

from auv_mag_tracking.api import CableMap, MagneticInput, NavigationInput  # noqa: E402
from auv_mag_tracking.v2 import FlythroughConfig, FlythroughInspectionPipelineV2  # noqa: E402
from auv_mag_tracking.v2.config import InspectionConfig, V2Config  # noqa: E402
from thesis_plot_style import (BASELINE_1, GRID_COLOR, PROPOSED, REFERENCE,  # noqa: E402
                               WARNING, apply_thesis_style, figure_size)

OUTPUT_ROOT = (MAG_ROOT / "docs/v2/publication/fwhm_v2_experiment_results")
TARGET_Y_M = 6.0
NEIGHBOR_Y_M = 9.0
NEIGHBOR_GAIN_NT = 10.0
INTERVALS = {"aligned": (5.5, 6.4), "biased": (6.6, 7.5)}
SEEDS = (101, 102, 103)


def anomaly(y: float | np.ndarray, neighbor: bool) -> float | np.ndarray:
    value = 1. + 20. / np.sqrt(1. + ((y - TARGET_Y_M) / 3.) ** 2)
    if neighbor:
        value = value + NEIGHBOR_GAIN_NT / np.sqrt(1. + ((y - NEIGHBOR_Y_M) / 3.) ** 2)
    return value


def run_case(interval_name: str, neighbor: bool, seed: int) -> dict:
    lower, upper = INTERVALS[interval_name]
    boundary = np.array([[-270., -30.], [120., -30.], [120., 30.], [-270., 30.]])
    config = replace(V2Config(), inspection=InspectionConfig(
        enabled=True, maximum_scan_duration_s=600.,
        maximum_observation_age_s=1000., maximum_observation_distance_m=250.,
        boundary_stop_margin_m=10.,
    ))
    fly = FlythroughConfig(
        side_hint_required=True, side_hint_min_m=lower, side_hint_max_m=upper,
        side_hint_source="survey_map", maximum_side_hint_width_m=1.,
        minimum_halfmax_tail_m=7., side_hint_observation_tolerance_m=.5,
    )
    pipeline = FlythroughInspectionPipelineV2(
        object(), CableMap(np.array([[0., 0.], [80., 0.]])),
        v2_config=config, search_boundary_xy_m=boundary, flythrough_config=fly,
    )
    start, end = pipeline._fly_start, pipeline._fly_end
    heading = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
    rng = np.random.default_rng(seed)
    for index, fraction in enumerate(np.linspace(0., 1., 2300)):
        point = start + fraction * (end - start)
        t = index * .25
        field = float(anomaly(float(point[1]), neighbor) + rng.normal(0., .03))
        navigation = NavigationInput(
            t, np.array([*point, -10.]), heading,
            speed_mps=.3, position_std_m=.1,
        )
        magnetic = MagneticInput(t, np.tile([field, 0., 0.], (4, 1)), 50.)
        output, guidance = pipeline.step_with_guidance(navigation, magnetic)
    lateral = pipeline._last_lateral
    return {
        "interval": interval_name,
        "hint_m": [lower, upper],
        "hint_width_m": upper - lower,
        "observation_tolerance_m": fly.side_hint_observation_tolerance_m,
        "neighbor": neighbor,
        "seed": seed,
        "magnetic_center_m": lateral.center_x_m,
        "target_center_error_m": None if lateral.center_x_m is None else lateral.center_x_m - TARGET_Y_M,
        "lateral_valid": bool(lateral.valid),
        "mode_after_confirm": output.mode,
        "reason_codes": list(lateral.reason_codes),
        "guidance_speed_mps": guidance.speed_mps,
        "burial_depth_m": output.burial_depth_m,
    }


def plot_cases(cases: list[dict], path: Path) -> None:
    apply_thesis_style(layout="full")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=figure_size(height=3.5),
                             gridspec_kw={"width_ratios": [1.4, 1.]})
    ax = axes[0]
    y = np.linspace(-2., 15., 600)
    ax.plot(y, anomaly(y, False), color=PROPOSED, label="单源磁异常")
    ax.plot(y, anomaly(y, True), color=WARNING, label="目标＋邻源")
    ax.axvline(TARGET_Y_M, color=REFERENCE, linestyle="--", linewidth=1., label="目标真值 6 m")
    ax.axvline(NEIGHBOR_Y_M, color=BASELINE_1, linestyle=":", linewidth=1., label="邻源 9 m")
    ax.axvspan(*INTERVALS["aligned"], color=PROPOSED, alpha=.12)
    ax.axvspan(*INTERVALS["biased"], color=WARNING, alpha=.12)
    ax.set(xlim=(3.5, 10.5), xlabel="横向位置 / m", ylabel="合成磁异常 / nT",
           title="(a) 地图区间偏移后的峰位")
    ax.legend(loc="upper left", bbox_to_anchor=(0., -.29), fontsize=7,
              ncol=2, borderaxespad=0.)

    ax = axes[1]
    ax.set(xlim=(-.5, 1.5), ylim=(-.5, 1.5), xticks=(0, 1), yticks=(0, 1),
           xticklabels=("单源", "目标＋邻源"),
           yticklabels=("准确窄提示", "偏移窄提示"),
           xlabel="磁场条件", title="(b) 确认阶段的目标准入")
    ax.invert_yaxis()
    ax.grid(color=GRID_COLOR, alpha=.3)
    ax.tick_params(axis="y", labelsize=8, pad=2)
    for row, name in enumerate(("aligned", "biased")):
        for col, neighbor in enumerate((False, True)):
            item = next(c for c in cases if c["interval"] == name
                        and c["neighbor"] == neighbor and c["seed"] == 101)
            accepted = item["mode_after_confirm"] == "track"
            ax.scatter(col, row, s=1200, color=PROPOSED if accepted else WARNING,
                       marker="s", zorder=3)
            ax.text(col, row, "准入" if accepted else "拒绝", color="white",
                    ha="center", va="center", fontsize=9, zorder=4)
    fig.subplots_adjust(wspace=.46, bottom=.28, top=.88)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path.with_suffix(".png"), dpi=600)
    fig.savefig(path.with_suffix(".pdf"))
    plt.close(fig)


def main() -> None:
    cases = [run_case(name, neighbor, seed)
             for seed in SEEDS
             for name in INTERVALS
             for neighbor in (False, True)]
    expected = {
        ("aligned", False): "track", ("aligned", True): "hold",
        ("biased", False): "hold", ("biased", True): "track",
    }
    for item in cases:
        assert item["mode_after_confirm"] == expected[(item["interval"], item["neighbor"])], item
        assert item["burial_depth_m"] is None, item
        if item["mode_after_confirm"] == "hold":
            assert "side_hint_observation_inconsistent" in item["reason_codes"], item
            assert item["guidance_speed_mps"] == 0., item
    script_hash = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    payload = {
        "role": "algorithm_level_static_hint_bias_counterexample",
        "source_script_sha256": script_hash,
        "target_y_m": TARGET_Y_M,
        "neighbor_y_m": NEIGHBOR_Y_M,
        "neighbor_gain_nt": NEIGHBOR_GAIN_NT,
        "magnetic_noise_sigma_nt": .03,
        "seeds": SEEDS,
        "caveat": "Synthetic scalar-envelope sum; algorithm-level confirm admission only, no ROS/PVS/field completion claim.",
        "cases": cases,
    }
    data = OUTPUT_ROOT / "data/54_static_hint_bias_admission.json"
    data.parent.mkdir(parents=True, exist_ok=True)
    data.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n")
    plot_cases(cases, OUTPUT_ROOT / "figures/54_static_hint_bias_admission")
    print(json.dumps({"cases": len(cases), "data": str(data),
                      "pattern": {f"{k[0]}_{k[1]}": v for k, v in expected.items()}},
                     ensure_ascii=False))


if __name__ == "__main__":
    main()
