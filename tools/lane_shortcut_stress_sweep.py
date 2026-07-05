"""Lane-spacing stress sweep for cross-lane shortcut failure modes."""

from __future__ import annotations

import argparse
import csv
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from copy import deepcopy
from pathlib import Path
from typing import Callable

import numpy as np

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.config import ScenarioConfig, build_default_scenarios  # noqa: E402
from auv_mag_tracking.perception import CableMapFrameTracker  # noqa: E402
from auv_mag_tracking.viz.metrics import compute_health_metrics, health_score  # noqa: E402
from auv_mag_tracking.viz.recorder import simulate_run  # noqa: E402

LANE_SPACING_GRID_M = (100.0, 70.0, 50.0, 40.0)
PRIOR_TIERS = ("mid", "heavy")
MAP_FRAME_UNTRUSTED_DISTANCE_M = 20.0
MAP_FRAME_UNTRUSTED_CONSISTENCY = 0.20

FIELDNAMES = (
    "lane_spacing_m",
    "effective_turn_radius_m",
    "tier",
    "variant",
    "case",
    "health",
    "route",
    "endpoint",
    "geometry",
    "route_progress_max_jump_m",
    "route_progress_large_jump_count",
    "lane_shortcut_indicator",
    "track_mean_cross_track_m",
    "mag_path_fraction",
    "map_frame_progress_max_jump_m",
    "map_frame_mean_projection_distance_m",
    "map_frame_mean_consistency_score",
    "map_frame_untrusted_indicator",
    "prior_alignment_accept_fraction",
    "prior_alignment_mean_residual_m",
    "prior_alignment_final_translation_m",
    "prior_alignment_final_rotation_deg",
    "passed",
)


def _disable_progress_guard(scenario: ScenarioConfig) -> None:
    scenario.tracking.nominal_route_progress_guard_enabled = False


def _disable_prior_correction(scenario: ScenarioConfig) -> None:
    scenario.tracking.nominal_route_prior_observation_correction_enabled = False


def _disable_zigzag(scenario: ScenarioConfig) -> None:
    scenario.tracking.track_active_zigzag_angle_deg = 0.0
    scenario.tracking.curve_track_crossing_angle_deg = 0.0
    scenario.tracking.adaptive_track_zigzag_angle_enabled = False


def _disable_magnetic_path(scenario: ScenarioConfig) -> None:
    scenario.tracking.magnetic_path_observation_enabled = False


VARIANTS: tuple[tuple[str, Callable[[ScenarioConfig], None]], ...] = (
    ("baseline", lambda scenario: None),
    ("no_progress_guard", _disable_progress_guard),
    ("no_prior_correction", _disable_prior_correction),
    ("no_zigzag", _disable_zigzag),
    ("no_magnetic_path", _disable_magnetic_path),
)


def _lane_scenario(base: ScenarioConfig, spacing_m: float, tier: str, variant: str) -> ScenarioConfig:
    scenario = deepcopy(base)
    scenario.name = f"case_maze_lane_spacing_{int(spacing_m)}_dropout_prior_{tier}__{variant}"
    scenario.description = (
        f"Lane-shortcut stress case: maze dropout prior {tier}, lane spacing "
        f"{spacing_m:.0f} m, variant {variant}."
    )
    scenario.environment.maze_lane_spacing_m = float(spacing_m)
    if spacing_m < 2.0 * scenario.environment.maze_turn_radius_m:
        scenario.environment.maze_turn_radius_m = max(spacing_m / 2.0, 1.0)
        scenario.environment.min_cable_curvature_radius_m = scenario.environment.maze_turn_radius_m
    scenario.environment.validate_curvature_on_build = True
    return scenario


def _map_frame_shadow(record) -> tuple[float, float, float]:
    if "map_frame_progress_m" in record.channels:
        progress_arr = np.asarray(record["map_frame_progress_m"], dtype=float)
        progress_delta = np.diff(progress_arr)
        finite_delta = progress_delta[np.isfinite(progress_delta)]
        max_jump = float(np.max(finite_delta)) if finite_delta.size else 0.0
        distances = np.asarray(record["map_frame_projection_distance_m"], dtype=float)
        consistency = np.asarray(record["map_frame_consistency_score"], dtype=float)
        return (
            max_jump,
            float(np.nanmean(distances)) if distances.size else float("nan"),
            float(np.nanmean(consistency)) if consistency.size else float("nan"),
        )
    positions = np.column_stack((record["pos_x_m"], record["pos_y_m"]))
    tracker = CableMapFrameTracker(
        route_xy=record.cable_route_xy_m[:, :2],
        initial_position_xy=positions[0],
    )
    progress = []
    distances = []
    consistency = []
    for position_xy in positions:
        state = tracker.update(position_xy)
        progress.append(state.progress_m)
        distances.append(state.projection_distance_m)
        consistency.append(state.consistency_score)
    progress_arr = np.asarray(progress, dtype=float)
    progress_delta = np.diff(progress_arr)
    finite_delta = progress_delta[np.isfinite(progress_delta)]
    max_jump = float(np.max(finite_delta)) if finite_delta.size else 0.0
    return (
        max_jump,
        float(np.mean(distances)) if distances else float("nan"),
        float(np.mean(consistency)) if consistency else float("nan"),
    )


def _prior_alignment_summary(record) -> tuple[float, float, float, float]:
    if "prior_alignment_accepted" not in record.channels:
        return (float("nan"), float("nan"), float("nan"), float("nan"))
    accepted = np.asarray(record["prior_alignment_accepted"], dtype=float)
    residual = np.asarray(record["prior_alignment_residual_m"], dtype=float)
    tx = np.asarray(record["prior_alignment_translation_x_m"], dtype=float)
    ty = np.asarray(record["prior_alignment_translation_y_m"], dtype=float)
    rot = np.asarray(record["prior_alignment_rotation_deg"], dtype=float)
    accepted_mask = accepted > 0.5
    residual_mask = np.isfinite(residual) & accepted_mask
    final_translation_m = float("nan")
    final_rotation_deg = float("nan")
    finite_translation = np.isfinite(tx) & np.isfinite(ty)
    if np.any(finite_translation):
        final_translation_m = float(np.hypot(tx[finite_translation][-1], ty[finite_translation][-1]))
    finite_rotation = np.isfinite(rot)
    if np.any(finite_rotation):
        final_rotation_deg = float(rot[finite_rotation][-1])
    return (
        float(np.mean(accepted_mask)) if accepted.size else float("nan"),
        float(np.mean(residual[residual_mask])) if np.any(residual_mask) else float("nan"),
        final_translation_m,
        final_rotation_deg,
    )


def _row(
    lane_spacing_m: float,
    tier: str,
    variant: str,
    scenario: ScenarioConfig,
) -> dict[str, object]:
    record = simulate_run(scenario)
    metrics = compute_health_metrics(record)
    passed = (
        metrics.endpoint_completed >= 0.5
        and metrics.maze_geometry_passed >= 0.5
        and metrics.route_progress_large_jump_count == 0
    )
    map_jump, map_dist, map_score = _map_frame_shadow(record)
    align_accept, align_residual, align_translation, align_rotation = _prior_alignment_summary(record)
    map_frame_untrusted = (
        map_dist > MAP_FRAME_UNTRUSTED_DISTANCE_M
        or map_score < MAP_FRAME_UNTRUSTED_CONSISTENCY
    )
    return {
        "lane_spacing_m": f"{lane_spacing_m:.0f}",
        "effective_turn_radius_m": f"{float(scenario.environment.maze_turn_radius_m):.1f}",
        "tier": tier,
        "variant": variant,
        "case": scenario.name,
        "health": f"{health_score(metrics):.1f}",
        "route": f"{metrics.route_completion_ratio:.3f}",
        "endpoint": int(metrics.endpoint_completed >= 0.5),
        "geometry": int(metrics.maze_geometry_passed >= 0.5),
        "route_progress_max_jump_m": f"{metrics.route_progress_max_jump_m:.1f}",
        "route_progress_large_jump_count": metrics.route_progress_large_jump_count,
        "lane_shortcut_indicator": int(metrics.lane_shortcut_indicator >= 0.5),
        "track_mean_cross_track_m": f"{metrics.track_mean_cross_track_m:.1f}",
        "mag_path_fraction": f"{metrics.magnetic_path_observation_fraction:.2f}",
        "map_frame_progress_max_jump_m": f"{map_jump:.1f}",
        "map_frame_mean_projection_distance_m": f"{map_dist:.1f}",
        "map_frame_mean_consistency_score": f"{map_score:.3f}",
        "map_frame_untrusted_indicator": int(map_frame_untrusted),
        "prior_alignment_accept_fraction": f"{align_accept:.3f}",
        "prior_alignment_mean_residual_m": f"{align_residual:.2f}",
        "prior_alignment_final_translation_m": f"{align_translation:.2f}",
        "prior_alignment_final_rotation_deg": f"{align_rotation:.2f}",
        "passed": int(passed),
    }


def _mutator_by_name(variant: str) -> Callable[[ScenarioConfig], None]:
    for name, mutate in VARIANTS:
        if name == variant:
            return mutate
    raise ValueError(f"unknown variant: {variant}")


def _run_job(lane_spacing_m: float, tier: str, variant: str) -> dict[str, object]:
    scenarios = build_default_scenarios()
    base = scenarios[f"case_maze_sonar_dropout_prior_{tier}"]
    scenario = _lane_scenario(base, lane_spacing_m, tier, variant)
    _mutator_by_name(variant)(scenario)
    return _row(lane_spacing_m, tier, variant, scenario)

def _sort_key(row: dict[str, object]) -> tuple[float, str, str]:
    return (float(row["lane_spacing_m"]), str(row["tier"]), str(row["variant"]))


def _job_key(lane_spacing_m: float, tier: str, variant: str) -> tuple[str, str, str]:
    return (f"{lane_spacing_m:.0f}", tier, variant)


def _row_key(row: dict[str, object]) -> tuple[str, str, str]:
    return (str(row["lane_spacing_m"]), str(row["tier"]), str(row["variant"]))


def _load_existing_rows(output_csv: Path) -> dict[tuple[str, str, str], dict[str, object]]:
    if not output_csv.exists():
        return {}
    with output_csv.open(newline="", encoding="utf-8") as handle:
        return {_row_key(row): row for row in csv.DictReader(handle)}


def _parse_float_filter(value: str | None, default_values: tuple[float, ...]) -> tuple[float, ...]:
    if value is None:
        return default_values
    return tuple(float(item.strip()) for item in value.split(",") if item.strip())


def _parse_str_filter(value: str | None, default_values: tuple[str, ...]) -> tuple[str, ...]:
    if value is None:
        return default_values
    return tuple(item.strip() for item in value.split(",") if item.strip())


def run_sweep(
    output_csv: Path,
    workers: int = 4,
    lane_spacings: tuple[float, ...] = LANE_SPACING_GRID_M,
    tiers: tuple[str, ...] = PRIOR_TIERS,
    variants: tuple[str, ...] = tuple(name for name, _ in VARIANTS),
    resume: bool = False,
) -> list[dict[str, object]]:
    jobs = [
        (lane_spacing_m, tier, variant)
        for tier in tiers
        for lane_spacing_m in lane_spacings
        for variant in variants
    ]
    existing_by_key = _load_existing_rows(output_csv) if resume else {}
    rows: list[dict[str, object]] = list(existing_by_key.values())
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)
        handle.flush()
        pending_jobs: list[tuple[float, str, str]] = []
        for lane_spacing_m, tier, variant in jobs:
            key = _job_key(lane_spacing_m, tier, variant)
            if key in existing_by_key:
                continue
            pending_jobs.append((lane_spacing_m, tier, variant))

        if workers <= 1:
            for lane_spacing_m, tier, variant in pending_jobs:
                row = _run_job(lane_spacing_m, tier, variant)
                rows.append(row)
                key = _job_key(lane_spacing_m, tier, variant)
                existing_by_key[key] = row
                writer.writerow(row)
                handle.flush()
                print(
                    f"[lane] spacing={lane_spacing_m:.0f} tier={tier} "
                    f"variant={variant} passed={row['passed']} "
                    f"jump={row['route_progress_max_jump_m']} "
                    f"map_jump={row['map_frame_progress_max_jump_m']}",
                    flush=True,
                )
        else:
            with ProcessPoolExecutor(max_workers=workers) as executor:
                future_to_job = {
                    executor.submit(_run_job, lane_spacing_m, tier, variant): (
                        lane_spacing_m,
                        tier,
                        variant,
                    )
                    for lane_spacing_m, tier, variant in pending_jobs
                }
                for future in as_completed(future_to_job):
                    lane_spacing_m, tier, variant = future_to_job[future]
                    row = future.result()
                    rows.append(row)
                    existing_by_key[_job_key(lane_spacing_m, tier, variant)] = row
                    writer.writerow(row)
                    handle.flush()
                    print(
                        f"[lane] spacing={lane_spacing_m:.0f} tier={tier} "
                        f"variant={variant} passed={row['passed']} "
                        f"jump={row['route_progress_max_jump_m']} "
                        f"map_jump={row['map_frame_progress_max_jump_m']}",
                        flush=True,
                    )
    rows.sort(key=_sort_key)
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=WORKSPACE_ROOT / "results" / "20260705_lane_shortcut" / "lane_shortcut_sweep.csv",
    )
    parser.add_argument("--workers", type=int, default=4, help="Parallel simulation workers.")
    parser.add_argument("--spacing", help="Comma-separated lane spacing filter, e.g. 100,40.")
    parser.add_argument("--tier", help="Comma-separated prior-tier filter, e.g. mid,heavy.")
    parser.add_argument("--variant", help="Comma-separated variant filter.")
    parser.add_argument("--resume", action="store_true", help="Reuse rows already present in --output.")
    args = parser.parse_args()
    rows = run_sweep(
        args.output,
        workers=max(1, int(args.workers)),
        lane_spacings=_parse_float_filter(args.spacing, LANE_SPACING_GRID_M),
        tiers=_parse_str_filter(args.tier, PRIOR_TIERS),
        variants=_parse_str_filter(args.variant, tuple(name for name, _ in VARIANTS)),
        resume=bool(args.resume),
    )
    print(f"[lane] wrote {len(rows)} rows to {args.output}")


if __name__ == "__main__":
    main()
