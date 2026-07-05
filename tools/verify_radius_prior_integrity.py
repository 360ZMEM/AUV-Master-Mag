"""Verify that radius-boundary cases really inherit dropout + distorted priors.

This script is intentionally lightweight and deterministic.  It does not run
the simulator; it inspects registered ``case_radius_*`` scenarios and writes a
CSV evidence table that can be cited by docs/29 when explaining why the 30 m
pure-magnetic result is not an artifact of an un-distorted prior.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.config import build_default_scenarios  # noqa: E402


EXPECTED_PRIORS = {
    "mid": {
        "translation_y_m": 7.5,
        "rotation_deg": 3.0,
        "scale_x": 0.99,
    },
    "heavy": {
        "translation_y_m": 10.0,
        "rotation_deg": 5.0,
        "scale_x": 0.98,
    },
}

FIELDNAMES = (
    "case",
    "radius_m",
    "prior_tier",
    "route_mode",
    "arc_radius_m",
    "sonar_fail_after_track_active",
    "sonar_fail_after_track_delay_s",
    "use_nominal_route_prior",
    "prior_translation_x_m",
    "prior_translation_y_m",
    "prior_rotation_deg",
    "prior_scale_x",
    "prior_scale_y",
    "prior_correction_enabled",
    "progress_guard_enabled",
    "magnetic_path_observation_enabled",
    "track_active_zigzag_angle_deg",
    "adaptive_track_zigzag_angle_enabled",
)


def _tier_from_name(case_name: str) -> str:
    if case_name.endswith("_prior_mid"):
        return "mid"
    if case_name.endswith("_prior_heavy"):
        return "heavy"
    raise AssertionError(f"Unexpected radius case tier in {case_name!r}")


def _radius_from_name(case_name: str) -> float:
    # case_radius_120_dropout_prior_mid
    parts = case_name.split("_")
    if len(parts) < 3 or parts[0] != "case" or parts[1] != "radius":
        raise AssertionError(f"Unexpected radius case name: {case_name!r}")
    return float(parts[2])


def _assert_close(actual: float, expected: float, label: str, case_name: str) -> None:
    if abs(actual - expected) > 1e-9:
        raise AssertionError(
            f"{case_name}: {label} expected {expected}, got {actual}"
        )


def inspect_radius_cases() -> list[dict[str, object]]:
    scenarios = build_default_scenarios()
    case_names = sorted(name for name in scenarios if name.startswith("case_radius_"))
    if not case_names:
        raise AssertionError("No case_radius_* scenarios are registered")

    rows: list[dict[str, object]] = []
    for case_name in case_names:
        scenario = scenarios[case_name]
        tier = _tier_from_name(case_name)
        expected = EXPECTED_PRIORS[tier]
        radius_from_name = _radius_from_name(case_name)

        tracking = scenario.tracking
        sonar = scenario.sonar
        environment = scenario.environment

        assert environment.cable_route_mode == "tightening_arc", case_name
        _assert_close(float(environment.arc_radius_m), radius_from_name, "arc_radius_m", case_name)
        assert sonar.fail_after_track_active, f"{case_name}: sonar dropout is disabled"
        _assert_close(float(sonar.fail_after_track_delay_s), 0.0, "fail_after_track_delay_s", case_name)
        assert tracking.use_nominal_route_prior, f"{case_name}: nominal route prior disabled"
        _assert_close(
            float(tracking.nominal_route_prior_translation_xy_m[1]),
            expected["translation_y_m"],
            "prior_translation_y_m",
            case_name,
        )
        _assert_close(
            float(tracking.nominal_route_prior_rotation_deg),
            expected["rotation_deg"],
            "prior_rotation_deg",
            case_name,
        )
        _assert_close(
            float(tracking.nominal_route_prior_scale_xy[0]),
            expected["scale_x"],
            "prior_scale_x",
            case_name,
        )
        assert tracking.nominal_route_prior_observation_correction_enabled, (
            f"{case_name}: online prior correction disabled"
        )
        assert tracking.nominal_route_progress_guard_enabled, (
            f"{case_name}: progress guard disabled"
        )
        assert tracking.magnetic_path_observation_enabled, (
            f"{case_name}: magnetic path observation disabled"
        )

        rows.append({
            "case": case_name,
            "radius_m": f"{radius_from_name:.0f}",
            "prior_tier": tier,
            "route_mode": environment.cable_route_mode,
            "arc_radius_m": f"{float(environment.arc_radius_m):.0f}",
            "sonar_fail_after_track_active": int(sonar.fail_after_track_active),
            "sonar_fail_after_track_delay_s": f"{float(sonar.fail_after_track_delay_s):.1f}",
            "use_nominal_route_prior": int(tracking.use_nominal_route_prior),
            "prior_translation_x_m": f"{float(tracking.nominal_route_prior_translation_xy_m[0]):.1f}",
            "prior_translation_y_m": f"{float(tracking.nominal_route_prior_translation_xy_m[1]):.1f}",
            "prior_rotation_deg": f"{float(tracking.nominal_route_prior_rotation_deg):.1f}",
            "prior_scale_x": f"{float(tracking.nominal_route_prior_scale_xy[0]):.3f}",
            "prior_scale_y": f"{float(tracking.nominal_route_prior_scale_xy[1]):.3f}",
            "prior_correction_enabled": int(
                tracking.nominal_route_prior_observation_correction_enabled
            ),
            "progress_guard_enabled": int(tracking.nominal_route_progress_guard_enabled),
            "magnetic_path_observation_enabled": int(
                tracking.magnetic_path_observation_enabled
            ),
            "track_active_zigzag_angle_deg": (
                f"{float(tracking.track_active_zigzag_angle_deg):.1f}"
            ),
            "adaptive_track_zigzag_angle_enabled": int(
                tracking.adaptive_track_zigzag_angle_enabled
            ),
        })
    return rows


def write_csv(rows: list[dict[str, object]], output_csv: Path) -> None:
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=WORKSPACE_ROOT / "results" / "20260705_integrity" / "radius_prior_integrity.csv",
        help="CSV path for radius prior/dropout integrity evidence.",
    )
    args = parser.parse_args()
    rows = inspect_radius_cases()
    write_csv(rows, args.output)
    for row in rows:
        print(",".join(str(row[field]) for field in FIELDNAMES), flush=True)
    print(f"[integrity] verified {len(rows)} radius cases; csv written to {args.output}")


if __name__ == "__main__":
    main()
