"""Shadow readiness scoring for magnetic lookahead/probe integration."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ShadowReadinessScore:
    supply: float
    selection: float
    consumption: float
    total: float
    bottleneck_code: float


def _clip01(value: float) -> float:
    if not math.isfinite(value):
        return 0.0
    return max(0.0, min(1.0, float(value)))


def score_shadow_hypothesis_readiness(
    *,
    magnetic_path_valid: bool,
    magnetic_phase_valid: bool,
    magnetic_lookahead_valid: bool,
    magnetic_lookahead_confidence: float,
    lookahead_feed_allowed: bool,
    cycle_burial_valid: bool,
    cycle_burial_quality: float,
    local_path_confidence: float,
    local_path_residual_m: float,
    local_path_max_residual_m: float,
    guidance_source: str,
    route_progress_rate_mps: float,
    yaw_rate_abs_fraction: float,
) -> ShadowReadinessScore:
    """Score where the magnetic hypothesis pipeline is blocked.

    The score is diagnostic only.  It deliberately avoids modifying perception
    or control state; it summarizes whether a magnetic/probe hypothesis has
    enough supply, passes current selection gates, and is being consumed by the
    controller in a way that still produces forward route progress.
    """

    supply = 0.0
    if magnetic_path_valid:
        supply += 0.35
    if magnetic_phase_valid:
        supply += 0.20
    if magnetic_lookahead_valid:
        supply += 0.25 * _clip01(magnetic_lookahead_confidence / 0.5)
    if cycle_burial_valid:
        supply += 0.20 * _clip01(cycle_burial_quality / 0.4)
    supply = _clip01(supply)

    residual_credit = 0.0
    if math.isfinite(local_path_residual_m) and local_path_max_residual_m > 1e-9:
        residual_credit = 1.0 - local_path_residual_m / local_path_max_residual_m
    local_path_credit = 0.5 * _clip01(local_path_confidence / 0.5) + 0.5 * _clip01(residual_credit)
    feed_credit = 1.0 if lookahead_feed_allowed else (0.35 if magnetic_lookahead_valid else 0.0)
    selection = _clip01(0.55 * feed_credit + 0.45 * local_path_credit)

    source_credit = {
        "LOCAL_PATH": 1.0,
        "MAGNETIC_LOOKAHEAD": 0.9,
        "SONAR": 0.75,
        "SONAR_SEED": 0.65,
        "MEMORY": 0.35,
        "MAGNETIC": 0.30,
        "BLIND": 0.15,
        "SEARCH": 0.0,
    }.get(guidance_source, 0.0)
    progress_credit = _clip01(route_progress_rate_mps / 0.6)
    yaw_credit = _clip01(1.0 - yaw_rate_abs_fraction)
    consumption = _clip01(0.45 * source_credit + 0.40 * progress_credit + 0.15 * yaw_credit)

    total = _clip01(supply * selection * consumption)
    bottleneck_code = 1.0
    bottleneck_value = min(supply, selection, consumption)
    if total < 0.25:
        if bottleneck_value == supply:
            bottleneck_code = 2.0
        elif bottleneck_value == selection:
            bottleneck_code = 3.0
        else:
            bottleneck_code = 4.0

    return ShadowReadinessScore(
        supply=supply,
        selection=selection,
        consumption=consumption,
        total=total,
        bottleneck_code=bottleneck_code,
    )
