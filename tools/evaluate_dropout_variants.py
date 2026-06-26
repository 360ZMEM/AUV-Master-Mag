"""Evaluate representative case_maze_sonar_dropout tuning variants.

This script is intentionally small and deterministic: it runs curated D0-D4 and
zig-zag probe representative points without doing a parameter grid search.
"""

from __future__ import annotations

import argparse
import copy
import sys
from pathlib import Path
from typing import Callable, List, Tuple

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.config import ScenarioConfig, build_default_scenarios  # noqa: E402
from auv_mag_tracking.viz import compute_health_metrics, health_score  # noqa: E402
from auv_mag_tracking.viz.recorder import simulate_run  # noqa: E402


VariantBuilder = Callable[[ScenarioConfig], ScenarioConfig]


def _variant(name: str, apply: Callable[[ScenarioConfig], None]) -> Tuple[str, VariantBuilder]:
    def build(base: ScenarioConfig) -> ScenarioConfig:
        scenario = copy.deepcopy(base)
        scenario.name = name
        apply(scenario)
        return scenario

    return name, build


def _variants() -> List[Tuple[str, VariantBuilder]]:
    return [
        _variant("d0_baseline", lambda s: None),
        _variant("d0_p36_route_baseline", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_feed_local_path=True,
            lookahead_feed_extrapolated_scale=0.25,
            lookahead_feed_max_age_s=60.0,
            lookahead_feed_max_phase_age_s=60.0,
            lookahead_feed_max_innovation_m=14.0,
            lookahead_feed_max_axis_delta_deg=35.0,
            lookahead_feed_max_local_residual_m=5.0,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("d0_p44_curveflip_counterexample", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_feed_local_path=True,
            lookahead_feed_extrapolated_scale=0.25,
            curve_track_flip_to_vehicle=True,
            lookahead_feed_max_age_s=60.0,
            lookahead_feed_max_phase_age_s=60.0,
            lookahead_feed_max_innovation_m=14.0,
            lookahead_feed_max_axis_delta_deg=35.0,
            lookahead_feed_max_local_residual_m=5.0,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("d0_sparse_sonar_anchor", _sparse_sonar),
        _variant("d1_delay60", lambda s: setattr(s.sonar, "fail_after_track_delay_s", 60.0)),
        _variant("d1_delay180", lambda s: setattr(s.sonar, "fail_after_track_delay_s", 180.0)),
        _variant("d1_sparse_sonar", _sparse_sonar),
        _variant("d2_local_age180", lambda s: setattr(s.tracking, "local_path_max_age_s", 180.0)),
        _variant("d2_local_capacity36", lambda s: setattr(s.tracking, "local_path_capacity", 36)),
        _variant("d2_spacing2m", lambda s: setattr(s.tracking, "local_path_min_observation_spacing_m", 2.0)),
        _variant("d2_forgetting060", lambda s: setattr(s.tracking, "forgetting_factor", 0.60)),
        _variant("d3_progressive_gate", _progressive_gate),
        _variant("d4_sparse035", lambda s: _sparse_sonar_prob(s, 0.35)),
        _variant("d4_sparse050", lambda s: _sparse_sonar_prob(s, 0.50)),
        _variant("p1_probe3_nomag", lambda s: _zigzag_probe(s, angle_deg=3.0, magnetic_path=False)),
        _variant("p2_probe6_nomag", lambda s: _zigzag_probe(s, angle_deg=6.0, magnetic_path=False)),
        _variant("p3_probe10_nomag", lambda s: _zigzag_probe(s, angle_deg=10.0, magnetic_path=False)),
        _variant("p4_probe3_mag", lambda s: _zigzag_probe(s, angle_deg=3.0, magnetic_path=True)),
        _variant("p5_probe6_mag", lambda s: _zigzag_probe(s, angle_deg=6.0, magnetic_path=True)),
        _variant("p6_probe10_mag", lambda s: _zigzag_probe(s, angle_deg=10.0, magnetic_path=True)),
        _variant("p6d_probe10_mag_diag", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            feed_local_path=False,
        )),
        _variant("p7_probe6_mag_age180", lambda s: _zigzag_probe(s, angle_deg=6.0, magnetic_path=True, local_age_s=180.0)),
        _variant("p8_probe10_mag_age180", lambda s: _zigzag_probe(s, angle_deg=10.0, magnetic_path=True, local_age_s=180.0)),
        _variant("p9_probe10_mag_gate20", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p10_probe10_mag_gate10", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            feed_max_innovation_m=10.0,
            feed_max_axis_delta_deg=35.0,
        )),
        _variant("p11_probe10_mag_phase", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            phase_gate=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p12_probe10_mag_phase_loose", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            feed_max_innovation_m=float("inf"),
            feed_max_axis_delta_deg=90.0,
        )),
        _variant("p13_probe10_mag_phase_lowoffset", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            feed_max_innovation_m=float("inf"),
            feed_max_axis_delta_deg=90.0,
        )),
        _variant("p14_probe10_mag_phase_latch", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            phase_latch_duration_s=20.0,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p15_probe10_mag_lookahead", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            feed_local_path=False,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            local_path_guidance=False,
        )),
        _variant("p16_probe10_mag_lookahead_local", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p17_probe10_mag_lookahead_age180", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_max_age_s=180.0,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p18_probe10_lookahead_pursuit", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_pursuit=True,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p19_probe10_lookahead_pursuit_age180", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_max_age_s=180.0,
            lookahead_pursuit=True,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p20_probe14_lookahead_pursuit", lambda s: _zigzag_probe(
            s,
            angle_deg=14.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_pursuit=True,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p21_probe10_lookahead_lowphase", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.25,
            lookahead=True,
            lookahead_pursuit=True,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p22_probe10_lookahead_feedlocal", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_pursuit=True,
            lookahead_feed_local_path=True,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p23_probe10_feedlocal_age45", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_max_age_s=45.0,
            lookahead_pursuit=True,
            lookahead_feed_local_path=True,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p24_probe10_feedlocal_local60", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=60.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_pursuit=True,
            lookahead_feed_local_path=True,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p25_probe10_feedlocal_gate60", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_pursuit=True,
            lookahead_feed_local_path=True,
            lookahead_feed_max_age_s=60.0,
            lookahead_feed_max_phase_age_s=60.0,
            lookahead_feed_max_innovation_m=14.0,
            lookahead_feed_max_axis_delta_deg=35.0,
            lookahead_feed_max_local_residual_m=5.0,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p26_probe10_feedlocal_gate90", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_pursuit=True,
            lookahead_feed_local_path=True,
            lookahead_feed_max_age_s=90.0,
            lookahead_feed_max_phase_age_s=90.0,
            lookahead_feed_max_innovation_m=20.0,
            lookahead_feed_max_axis_delta_deg=45.0,
            lookahead_feed_max_local_residual_m=8.0,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p27_probe10_gate60_nopursuit", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_feed_local_path=True,
            lookahead_feed_max_age_s=60.0,
            lookahead_feed_max_phase_age_s=60.0,
            lookahead_feed_max_innovation_m=14.0,
            lookahead_feed_max_axis_delta_deg=35.0,
            lookahead_feed_max_local_residual_m=5.0,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p28_probe10_gate45_conservative", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_feed_local_path=True,
            lookahead_feed_max_age_s=45.0,
            lookahead_feed_max_phase_age_s=45.0,
            lookahead_feed_max_innovation_m=10.0,
            lookahead_feed_max_axis_delta_deg=25.0,
            lookahead_feed_max_local_residual_m=4.0,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p29_probe10_gate60_mid", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_feed_local_path=True,
            lookahead_feed_max_age_s=60.0,
            lookahead_feed_max_phase_age_s=60.0,
            lookahead_feed_max_innovation_m=12.0,
            lookahead_feed_max_axis_delta_deg=30.0,
            lookahead_feed_max_local_residual_m=5.0,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p30_probe10_gate75_mid", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_feed_local_path=True,
            lookahead_feed_max_age_s=75.0,
            lookahead_feed_max_phase_age_s=75.0,
            lookahead_feed_max_innovation_m=12.0,
            lookahead_feed_max_axis_delta_deg=30.0,
            lookahead_feed_max_local_residual_m=5.0,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p31_probe10_gate60_heading30", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_feed_local_path=True,
            lookahead_feed_max_age_s=60.0,
            lookahead_feed_max_phase_age_s=60.0,
            lookahead_feed_max_innovation_m=14.0,
            lookahead_feed_max_axis_delta_deg=30.0,
            lookahead_feed_max_local_residual_m=5.0,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p32_probe10_gate60_heading40", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_feed_local_path=True,
            lookahead_feed_max_age_s=60.0,
            lookahead_feed_max_phase_age_s=60.0,
            lookahead_feed_max_innovation_m=14.0,
            lookahead_feed_max_axis_delta_deg=40.0,
            lookahead_feed_max_local_residual_m=5.0,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p33_probe10_tiered_anchor_low", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_feed_local_path=True,
            lookahead_feed_phase_anchor=True,
            lookahead_feed_extrapolated_scale=0.25,
            lookahead_feed_max_age_s=60.0,
            lookahead_feed_max_phase_age_s=60.0,
            lookahead_feed_max_innovation_m=14.0,
            lookahead_feed_max_axis_delta_deg=35.0,
            lookahead_feed_max_local_residual_m=5.0,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p34_probe10_anchor_only", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_feed_local_path=True,
            lookahead_feed_phase_anchor=True,
            lookahead_feed_extrapolated_scale=0.0,
            lookahead_feed_max_age_s=60.0,
            lookahead_feed_max_phase_age_s=60.0,
            lookahead_feed_max_innovation_m=14.0,
            lookahead_feed_max_axis_delta_deg=35.0,
            lookahead_feed_max_local_residual_m=5.0,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p35_probe10_tiered_anchor_mid", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_feed_local_path=True,
            lookahead_feed_phase_anchor=True,
            lookahead_feed_extrapolated_scale=0.50,
            lookahead_feed_max_age_s=60.0,
            lookahead_feed_max_phase_age_s=60.0,
            lookahead_feed_max_innovation_m=14.0,
            lookahead_feed_max_axis_delta_deg=35.0,
            lookahead_feed_max_local_residual_m=5.0,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p36_probe10_extrapolated_low", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_feed_local_path=True,
            lookahead_feed_extrapolated_scale=0.25,
            lookahead_feed_max_age_s=60.0,
            lookahead_feed_max_phase_age_s=60.0,
            lookahead_feed_max_innovation_m=14.0,
            lookahead_feed_max_axis_delta_deg=35.0,
            lookahead_feed_max_local_residual_m=5.0,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p37_probe10_extrapolated_mid", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_feed_local_path=True,
            lookahead_feed_extrapolated_scale=0.50,
            lookahead_feed_max_age_s=60.0,
            lookahead_feed_max_phase_age_s=60.0,
            lookahead_feed_max_innovation_m=14.0,
            lookahead_feed_max_axis_delta_deg=35.0,
            lookahead_feed_max_local_residual_m=5.0,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p38_probe10_extrapolated_tiny", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_feed_local_path=True,
            lookahead_feed_extrapolated_scale=0.10,
            lookahead_feed_max_age_s=60.0,
            lookahead_feed_max_phase_age_s=60.0,
            lookahead_feed_max_innovation_m=14.0,
            lookahead_feed_max_axis_delta_deg=35.0,
            lookahead_feed_max_local_residual_m=5.0,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p39_probe10_extrapolated_low_pursuit", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_pursuit=True,
            lookahead_feed_local_path=True,
            lookahead_feed_extrapolated_scale=0.25,
            lookahead_feed_max_age_s=60.0,
            lookahead_feed_max_phase_age_s=60.0,
            lookahead_feed_max_innovation_m=14.0,
            lookahead_feed_max_axis_delta_deg=35.0,
            lookahead_feed_max_local_residual_m=5.0,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("d2_shadow_axis_selector", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_feed_local_path=True,
            lookahead_feed_extrapolated_scale=0.25,
            lookahead_feed_max_age_s=60.0,
            lookahead_feed_max_phase_age_s=60.0,
            lookahead_feed_max_innovation_m=14.0,
            lookahead_feed_max_axis_delta_deg=35.0,
            lookahead_feed_max_local_residual_m=5.0,
            shadow_hypothesis=True,
            shadow_validation_max_age_s=30.0,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("d3_shadow_axis_selector_age45", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_feed_local_path=True,
            lookahead_feed_extrapolated_scale=0.25,
            lookahead_feed_max_age_s=60.0,
            lookahead_feed_max_phase_age_s=60.0,
            lookahead_feed_max_innovation_m=14.0,
            lookahead_feed_max_axis_delta_deg=35.0,
            lookahead_feed_max_local_residual_m=5.0,
            shadow_hypothesis=True,
            shadow_validation_max_age_s=45.0,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("d3_shadow_axis_dual_gate_shadow", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_feed_local_path=True,
            lookahead_feed_extrapolated_scale=0.25,
            lookahead_feed_max_age_s=60.0,
            lookahead_feed_max_phase_age_s=60.0,
            lookahead_feed_max_innovation_m=14.0,
            lookahead_feed_max_axis_delta_deg=35.0,
            lookahead_feed_max_local_residual_m=5.0,
            shadow_hypothesis=True,
            shadow_validation_max_age_s=45.0,
            shadow_dual_gate_shadow=True,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("d4_progress_aligned_shadow", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_feed_local_path=True,
            lookahead_feed_extrapolated_scale=0.25,
            lookahead_feed_max_age_s=60.0,
            lookahead_feed_max_phase_age_s=60.0,
            lookahead_feed_max_innovation_m=14.0,
            lookahead_feed_max_axis_delta_deg=35.0,
            lookahead_feed_max_local_residual_m=5.0,
            shadow_hypothesis=True,
            shadow_validation_max_age_s=45.0,
            shadow_dual_gate_shadow=True,
            shadow_progress_alignment=True,
              shadow_progress_proxy_hold=True,
              shadow_route_bound_progress_proxy=True,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p48_forward_sweep_zigzag_ab", lambda s: _zigzag_probe(
            s,
            angle_deg=22.0,
            magnetic_path=True,
            local_age_s=180.0,
            min_width_m=5.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_feed_local_path=True,
            lookahead_feed_extrapolated_scale=0.25,
            lookahead_feed_max_age_s=60.0,
            lookahead_feed_max_phase_age_s=60.0,
            lookahead_feed_max_innovation_m=14.0,
            lookahead_feed_max_axis_delta_deg=35.0,
            lookahead_feed_max_local_residual_m=5.0,
            shadow_hypothesis=True,
            shadow_validation_max_age_s=45.0,
            shadow_dual_gate_shadow=True,
            shadow_progress_alignment=True,
            shadow_progress_proxy_hold=True,
            shadow_route_bound_progress_proxy=True,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p49_forward_sweep_a14_wdefault", lambda s: _forward_sweep_zigzag_ab(
            s,
            angle_deg=14.0,
            min_width_m=None,
        )),
        _variant("p50_forward_sweep_a14_w375", lambda s: _forward_sweep_zigzag_ab(
            s,
            angle_deg=14.0,
            min_width_m=3.75,
        )),
        _variant("p51_forward_sweep_a18_wdefault", lambda s: _forward_sweep_zigzag_ab(
            s,
            angle_deg=18.0,
            min_width_m=None,
        )),
        _variant("p52_forward_sweep_a18_w375", lambda s: _forward_sweep_zigzag_ab(
            s,
            angle_deg=18.0,
            min_width_m=3.75,
        )),
        _variant("p53_forward_sweep_a22_wdefault", lambda s: _forward_sweep_zigzag_ab(
            s,
            angle_deg=22.0,
            min_width_m=None,
        )),
        _variant("p54_forward_sweep_a15_wdefault", lambda s: _forward_sweep_zigzag_ab(
            s,
            angle_deg=15.0,
            min_width_m=None,
        )),
        _variant("p55_forward_sweep_a16_wdefault", lambda s: _forward_sweep_zigzag_ab(
            s,
            angle_deg=16.0,
            min_width_m=None,
        )),
        _variant("p56_forward_sweep_a17_wdefault", lambda s: _forward_sweep_zigzag_ab(
            s,
            angle_deg=17.0,
            min_width_m=None,
        )),
        _variant("p57_decoupled_lateral_target_ab", _decoupled_lateral_target_ab),
        _variant("p58_decoupled_lateral_blend035_max8", lambda s: _decoupled_lateral_target_ab(
            s,
            blend=0.35,
            max_correction_deg=8.0,
        )),
        _variant("p59_decoupled_lateral_blend015_max3", lambda s: _decoupled_lateral_target_ab(
            s,
            blend=0.15,
            max_correction_deg=3.0,
        )),
        _variant("p60_decoupled_lateral_blend010_max2", lambda s: _decoupled_lateral_target_ab(
            s,
            blend=0.10,
            max_correction_deg=2.0,
        )),
        _variant("p61_decoupled_lateral_pure_minflip10", lambda s: _decoupled_lateral_target_ab(
            s,
            min_flip_interval_s=10.0,
        )),
        _variant("p62_decoupled_lateral_blend035_max8_minflip10", lambda s: _decoupled_lateral_target_ab(
            s,
            blend=0.35,
            max_correction_deg=8.0,
            min_flip_interval_s=10.0,
        )),
        _variant("p63_decoupled_lateral_pure_lookahead30_minflip10", lambda s: _decoupled_lateral_target_ab(
            s,
            lookahead_m=30.0,
            min_flip_interval_s=10.0,
        )),
        _variant("p64_decoupled_lateral_pure_lookahead30_minflip20", lambda s: _decoupled_lateral_target_ab(
            s,
            lookahead_m=30.0,
            min_flip_interval_s=20.0,
        )),
        _variant("p65_decoupled_lateral_pure_lookahead60_minflip20", lambda s: _decoupled_lateral_target_ab(
            s,
            lookahead_m=60.0,
            min_flip_interval_s=20.0,
        )),
        _variant("p66_decoupled_lateral_pure_lookahead60_minflip30", lambda s: _decoupled_lateral_target_ab(
            s,
            lookahead_m=60.0,
            min_flip_interval_s=30.0,
        )),
        _variant("p67_probe_burst20_recovery80", lambda s: _probe_burst_recovery_ab(
            s,
            burst_duration_s=20.0,
            recovery_duration_s=80.0,
        )),
        _variant("p68_probe_burst10_recovery90", lambda s: _probe_burst_recovery_ab(
            s,
            burst_duration_s=10.0,
            recovery_duration_s=90.0,
        )),
        _variant("p69_probe_burst5_recovery95", lambda s: _probe_burst_recovery_ab(
            s,
            burst_duration_s=5.0,
            recovery_duration_s=95.0,
        )),
        _variant("p70_probe_burst10_recovery190", lambda s: _probe_burst_recovery_ab(
            s,
            burst_duration_s=10.0,
            recovery_duration_s=190.0,
        )),
        _variant("p71_probe_burst5_recovery195", lambda s: _probe_burst_recovery_ab(
            s,
            burst_duration_s=5.0,
            recovery_duration_s=195.0,
        )),
        _variant("p72_probe_burst10_recovery190_governed", lambda s: _probe_burst_recovery_ab(
            s,
            burst_duration_s=10.0,
            recovery_duration_s=190.0,
            route_governor=True,
        )),
        _variant("p73_probe_burst5_recovery195_governed", lambda s: _probe_burst_recovery_ab(
            s,
            burst_duration_s=5.0,
            recovery_duration_s=195.0,
            route_governor=True,
        )),
        _variant("p74_probe_burst10_recovery190_delay200", lambda s: _probe_burst_recovery_ab(
            s,
            burst_duration_s=10.0,
            recovery_duration_s=190.0,
            start_delay_s=200.0,
        )),
        _variant("p75_probe_burst5_recovery195_delay200", lambda s: _probe_burst_recovery_ab(
            s,
            burst_duration_s=5.0,
            recovery_duration_s=195.0,
            start_delay_s=200.0,
        )),
        _variant("p76_probe_burst10_recovery190_centerline", lambda s: _probe_burst_recovery_ab(
            s,
            burst_duration_s=10.0,
            recovery_duration_s=190.0,
            centerline_recovery=True,
        )),
        _variant("p77_probe_burst5_recovery195_centerline", lambda s: _probe_burst_recovery_ab(
            s,
            burst_duration_s=5.0,
            recovery_duration_s=195.0,
            centerline_recovery=True,
        )),
    ]


def _variant_group(name: str) -> str:
    """Return the plan-level role for a curated variant."""
    if name in {"d0_p36_route_baseline", "p36_probe10_extrapolated_low"}:
        return "baseline"
    if name in {
        "d2_shadow_axis_selector",
        "d3_shadow_axis_selector_age45",
        "d3_shadow_axis_dual_gate_shadow",
        "d4_progress_aligned_shadow",
    }:
        return "shadow_positive"
    if name in {
        "p48_forward_sweep_zigzag_ab",
        "p49_forward_sweep_a14_wdefault",
        "p50_forward_sweep_a14_w375",
        "p51_forward_sweep_a18_wdefault",
        "p52_forward_sweep_a18_w375",
        "p53_forward_sweep_a22_wdefault",
        "p54_forward_sweep_a15_wdefault",
        "p55_forward_sweep_a16_wdefault",
        "p56_forward_sweep_a17_wdefault",
    }:
        return "forward_sweep_ab"
    if name in {
        "p57_decoupled_lateral_target_ab",
        "p58_decoupled_lateral_blend035_max8",
        "p59_decoupled_lateral_blend015_max3",
        "p60_decoupled_lateral_blend010_max2",
        "p61_decoupled_lateral_pure_minflip10",
        "p62_decoupled_lateral_blend035_max8_minflip10",
        "p63_decoupled_lateral_pure_lookahead30_minflip10",
        "p64_decoupled_lateral_pure_lookahead30_minflip20",
        "p65_decoupled_lateral_pure_lookahead60_minflip20",
        "p66_decoupled_lateral_pure_lookahead60_minflip30",
    }:
        return "decoupled_lateral_ab"
    if name in {
        "p67_probe_burst20_recovery80",
        "p68_probe_burst10_recovery90",
        "p69_probe_burst5_recovery95",
        "p70_probe_burst10_recovery190",
        "p71_probe_burst5_recovery195",
        "p72_probe_burst10_recovery190_governed",
        "p73_probe_burst5_recovery195_governed",
        "p74_probe_burst10_recovery190_delay200",
        "p75_probe_burst5_recovery195_delay200",
        "p76_probe_burst10_recovery190_centerline",
        "p77_probe_burst5_recovery195_centerline",
    }:
        return "probe_burst_recovery"
    if name in {"d0_sparse_sonar_anchor", "d1_sparse_sonar", "d4_sparse035", "d4_sparse050"}:
        return "sonar_anchor"
    if name in {"d0_p44_curveflip_counterexample"} or name.startswith((
        "p33_",
        "p34_",
        "p35_",
        "p40_",
        "p41_",
        "p42_",
        "p43_",
        "p44_",
        "p45_",
        "p46_",
        "p47_",
    )):
        return "negative_evidence"
    if name.startswith(("p", "d2_local", "d2_spacing", "d2_forgetting", "d3_progressive")):
        return "deprecated_tuning"
    return "reference"


def _sparse_sonar(scenario: ScenarioConfig) -> None:
    _sparse_sonar_prob(scenario, 0.20)


def _sparse_sonar_prob(scenario: ScenarioConfig, probability: float) -> None:
    scenario.sonar.fail_after_track_active = False
    scenario.sonar.prob_detection = probability


def _progressive_gate(scenario: ScenarioConfig) -> None:
    scenario.tracking.reacquire_region_progressive_forward_enabled = True


def _forward_sweep_zigzag_ab(
    scenario: ScenarioConfig,
    *,
    angle_deg: float,
    min_width_m: float | None,
) -> None:
    _zigzag_probe(
        scenario,
        angle_deg=angle_deg,
        magnetic_path=True,
        local_age_s=180.0,
        min_width_m=min_width_m,
        phase_gate=True,
        phase_min_offset_m=0.5,
        lookahead=True,
        lookahead_feed_local_path=True,
        lookahead_feed_extrapolated_scale=0.25,
        lookahead_feed_max_age_s=60.0,
        lookahead_feed_max_phase_age_s=60.0,
        lookahead_feed_max_innovation_m=14.0,
        lookahead_feed_max_axis_delta_deg=35.0,
        lookahead_feed_max_local_residual_m=5.0,
        shadow_hypothesis=True,
        shadow_validation_max_age_s=45.0,
        shadow_dual_gate_shadow=True,
        shadow_progress_alignment=True,
        shadow_progress_proxy_hold=True,
        shadow_route_bound_progress_proxy=True,
        local_path_guidance=True,
        feed_max_innovation_m=20.0,
        feed_max_axis_delta_deg=45.0,
    )


def _decoupled_lateral_target_ab(
    scenario: ScenarioConfig,
    *,
    blend: float = 1.0,
    max_correction_deg: float = 180.0,
    lookahead_m: float = 12.0,
    min_flip_interval_s: float = 0.0,
) -> None:
    _zigzag_probe(
        scenario,
        angle_deg=10.0,
        magnetic_path=True,
        local_age_s=180.0,
        phase_gate=True,
        phase_min_offset_m=0.5,
        lookahead=True,
        lookahead_feed_local_path=True,
        lookahead_feed_extrapolated_scale=0.25,
        lookahead_feed_max_age_s=60.0,
        lookahead_feed_max_phase_age_s=60.0,
        lookahead_feed_max_innovation_m=14.0,
        lookahead_feed_max_axis_delta_deg=35.0,
        lookahead_feed_max_local_residual_m=5.0,
        shadow_hypothesis=True,
        shadow_validation_max_age_s=45.0,
        shadow_dual_gate_shadow=True,
        shadow_progress_alignment=True,
        shadow_progress_proxy_hold=True,
        shadow_route_bound_progress_proxy=True,
        local_path_guidance=True,
        feed_max_innovation_m=20.0,
        feed_max_axis_delta_deg=45.0,
    )
    scenario.tracking.decoupled_lateral_target_control_enabled = True
    scenario.tracking.decoupled_lateral_target_control_blend = blend
    scenario.tracking.decoupled_lateral_target_control_max_correction_deg = max_correction_deg
    scenario.tracking.magnetic_shadow_decoupled_lateral_lookahead_m = lookahead_m
    scenario.tracking.decoupled_lateral_target_min_flip_interval_s = min_flip_interval_s


def _probe_burst_recovery_ab(
    scenario: ScenarioConfig,
    *,
    burst_duration_s: float,
    recovery_duration_s: float,
    start_delay_s: float = 0.0,
    route_governor: bool = False,
    centerline_recovery: bool = False,
) -> None:
    _decoupled_lateral_target_ab(
        scenario,
        lookahead_m=60.0,
        min_flip_interval_s=20.0,
    )
    scenario.tracking.decoupled_lateral_probe_burst_enabled = True
    scenario.tracking.decoupled_lateral_probe_burst_start_delay_s = start_delay_s
    scenario.tracking.decoupled_lateral_probe_burst_duration_s = burst_duration_s
    scenario.tracking.decoupled_lateral_probe_recovery_duration_s = recovery_duration_s
    scenario.tracking.decoupled_lateral_probe_recovery_control_enabled = centerline_recovery
    scenario.tracking.decoupled_lateral_probe_route_governor_enabled = route_governor


def _zigzag_probe(
    scenario: ScenarioConfig,
    *,
    angle_deg: float,
    magnetic_path: bool,
    feed_local_path: bool = True,
    local_age_s: float | None = None,
    min_width_m: float | None = None,
    feed_max_innovation_m: float | None = None,
    feed_max_axis_delta_deg: float | None = None,
    phase_gate: bool = False,
    phase_min_offset_m: float = 1.0,
    phase_min_duration_s: float = 2.0,
    phase_max_axis_delta_deg: float = 35.0,
    phase_latch_duration_s: float = 0.0,
    lookahead: bool = False,
    lookahead_max_age_s: float = 90.0,
    lookahead_axis_selection: bool = False,
    lookahead_axis_selection_min_progress_m: float = 3.0,
    lookahead_axis_hysteresis: bool = False,
    lookahead_axis_hysteresis_threshold: float = 2.0,
    lookahead_axis_score_decay: float = 0.6,
    lookahead_pursuit: bool = False,
    lookahead_feed_local_path: bool = False,
    lookahead_feed_max_age_s: float | None = None,
    lookahead_feed_max_phase_age_s: float | None = None,
    lookahead_feed_max_innovation_m: float | None = None,
    lookahead_feed_max_axis_delta_deg: float | None = None,
    lookahead_feed_max_local_residual_m: float | None = None,
    lookahead_feed_phase_anchor: bool = False,
    lookahead_feed_extrapolated_scale: float = 1.0,
    lookahead_feed_heading_smoothing: bool = False,
    lookahead_feed_heading_max_step_deg: float = 12.0,
    curve_track_flip_to_vehicle: bool = False,
    magnetic_crossing_probe_control: bool = False,
    shadow_hypothesis: bool = False,
    shadow_validation_max_age_s: float | None = None,
    shadow_dual_gate_shadow: bool = False,
    shadow_progress_alignment: bool = False,
    shadow_progress_proxy_hold: bool = False,
    shadow_route_bound_progress_proxy: bool = False,
    local_path_guidance: bool | None = None,
) -> None:
    scenario.tracking.track_active_zigzag_angle_deg = angle_deg
    scenario.tracking.curve_track_crossing_angle_deg = angle_deg
    scenario.tracking.local_path_curve_track_flip_to_vehicle_enabled = curve_track_flip_to_vehicle
    scenario.tracking.magnetic_crossing_probe_control_enabled = magnetic_crossing_probe_control
    scenario.tracking.magnetic_shadow_hypothesis_enabled = shadow_hypothesis
    if shadow_validation_max_age_s is not None:
        scenario.tracking.magnetic_shadow_validation_max_age_s = shadow_validation_max_age_s
    scenario.tracking.magnetic_shadow_dual_gate_shadow_enabled = shadow_dual_gate_shadow
    scenario.tracking.magnetic_shadow_progress_alignment_enabled = shadow_progress_alignment
    scenario.tracking.magnetic_shadow_progress_proxy_hold_enabled = shadow_progress_proxy_hold
    scenario.tracking.magnetic_shadow_route_bound_progress_proxy_enabled = shadow_route_bound_progress_proxy
    scenario.tracking.magnetic_path_observation_enabled = magnetic_path
    scenario.tracking.magnetic_path_feed_local_path = feed_local_path
    scenario.tracking.magnetic_path_min_horizontal_field_nt = 5.0
    scenario.tracking.magnetic_path_max_cross_track_m = 25.0
    if local_age_s is not None:
        scenario.tracking.local_path_max_age_s = local_age_s
    if min_width_m is not None:
        scenario.tracking.min_zigzag_width_m = min_width_m
        scenario.tracking.max_zigzag_width_m = max(scenario.tracking.max_zigzag_width_m, min_width_m)
    if feed_max_innovation_m is not None:
        scenario.tracking.magnetic_path_feed_max_innovation_m = feed_max_innovation_m
    if feed_max_axis_delta_deg is not None:
        scenario.tracking.magnetic_path_feed_max_heading_delta_deg = feed_max_axis_delta_deg
    if phase_gate:
        scenario.tracking.magnetic_path_phase_gate_enabled = True
        scenario.tracking.magnetic_path_phase_min_offset_m = phase_min_offset_m
        scenario.tracking.magnetic_path_phase_min_duration_s = phase_min_duration_s
        scenario.tracking.magnetic_path_phase_max_duration_s = 45.0
        scenario.tracking.magnetic_path_phase_max_axis_delta_deg = phase_max_axis_delta_deg
        scenario.tracking.magnetic_path_phase_latch_duration_s = phase_latch_duration_s
    if lookahead:
        scenario.tracking.magnetic_lookahead_enabled = True
        scenario.tracking.magnetic_lookahead_max_age_s = lookahead_max_age_s
        scenario.tracking.magnetic_lookahead_distance_m = 20.0
        scenario.tracking.magnetic_lookahead_heading_blend = 0.45
        scenario.tracking.magnetic_lookahead_min_confidence = 0.10
        scenario.tracking.magnetic_lookahead_axis_selection_enabled = lookahead_axis_selection
        scenario.tracking.magnetic_lookahead_axis_selection_min_progress_m = lookahead_axis_selection_min_progress_m
        scenario.tracking.magnetic_lookahead_axis_hysteresis_enabled = lookahead_axis_hysteresis
        scenario.tracking.magnetic_lookahead_axis_hysteresis_threshold = lookahead_axis_hysteresis_threshold
        scenario.tracking.magnetic_lookahead_axis_score_decay = lookahead_axis_score_decay
        scenario.tracking.magnetic_lookahead_feed_local_path = lookahead_feed_local_path
        scenario.tracking.magnetic_lookahead_feed_phase_anchor_enabled = lookahead_feed_phase_anchor
        scenario.tracking.magnetic_lookahead_feed_extrapolated_confidence_scale = lookahead_feed_extrapolated_scale
        scenario.tracking.magnetic_lookahead_feed_heading_smoothing_enabled = lookahead_feed_heading_smoothing
        scenario.tracking.magnetic_lookahead_feed_heading_max_step_deg = lookahead_feed_heading_max_step_deg
        if lookahead_feed_max_age_s is not None:
            scenario.tracking.magnetic_lookahead_feed_max_age_s = lookahead_feed_max_age_s
        if lookahead_feed_max_phase_age_s is not None:
            scenario.tracking.magnetic_lookahead_feed_max_phase_age_s = lookahead_feed_max_phase_age_s
        if lookahead_feed_max_innovation_m is not None:
            scenario.tracking.magnetic_lookahead_feed_max_innovation_m = lookahead_feed_max_innovation_m
        if lookahead_feed_max_axis_delta_deg is not None:
            scenario.tracking.magnetic_lookahead_feed_max_heading_delta_deg = lookahead_feed_max_axis_delta_deg
        if lookahead_feed_max_local_residual_m is not None:
            scenario.tracking.magnetic_lookahead_feed_max_local_residual_m = lookahead_feed_max_local_residual_m
    if lookahead_pursuit:
        scenario.tracking.magnetic_lookahead_pursuit_enabled = True
        scenario.tracking.magnetic_lookahead_pursuit_gain = 0.45
        scenario.tracking.magnetic_lookahead_pursuit_max_correction_deg = 18.0
    if local_path_guidance is not None:
        scenario.tracking.local_path_guidance_enabled = local_path_guidance


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--phase",
        choices=("all", "d0", "d1", "d2", "d3", "d4", "probe"),
        default="all",
        help="subset of representative variants to run",
    )
    parser.add_argument("--max-steps", type=int, default=None, help="optional simulation step cap")
    parser.add_argument(
        "--name",
        action="append",
        default=[],
        help="run only the named variant; may be passed multiple times",
    )
    args = parser.parse_args()

    base = build_default_scenarios()["case_maze_sonar_dropout"]
    print(
        "group,name,health,mean_err,track_xt,track_vehicle_err,route,final_dist,"
        "track_pct,switches,mag_probe_pct,mag_axis_err,mag_pos_err,mag_offset,"
        "mag_phase_pct,mag_phase_axis_err,mag_phase_pos_err,mag_phase_amp,"
        "phase_emit_pct,phase_no_pair_pct,phase_offset_reject_pct,phase_duration_reject_pct,"
        "phase_axis_reject_pct,phase_waiting_pct,phase_candidate_duration_s,phase_axis_delta,"
        "probe_active_pct,probe_cycles,probe_flips,probe_mag_crossings,probe_mag_cross_per_cycle,"
        "probe_forward_leg_pct,probe_backward_leg_pct,probe_stall_leg_pct,"
        "probe_cross_forward_leg_pct,probe_cross_backward_leg_pct,probe_cross_stall_leg_pct,"
        "probe_forward_leg_delta_m,probe_backward_leg_delta_m,"
        "probe_forward_phase_pct,probe_forward_phase_crossings,probe_forward_phase_cross_pct,"
        "probe_forward_phase_mag_path_pct,probe_forward_phase_mag_phase_pct,"
        "probe_forward_phase_lookahead_pct,probe_forward_phase_candidate_pct,"
        "shadow_forward_zigzag_valid_pct,shadow_forward_zigzag_feasible_pct,"
        "shadow_forward_zigzag_forward_dot,shadow_forward_zigzag_lateral_dot,"
        "shadow_forward_zigzag_forward_rate,shadow_forward_zigzag_lateral_rate,"
        "shadow_forward_zigzag_leg_feasible_pct,shadow_forward_zigzag_leg_delta,"
        "shadow_forward_zigzag_leg_sweep,"
        "shadow_forward_sweep_angle,shadow_forward_sweep_multiplier,"
        "shadow_forward_sweep_feasible_pct,shadow_forward_sweep_leg_delta,"
        "shadow_forward_sweep_leg_sweep,shadow_forward_sweep_forward_dot,"
        "shadow_forward_sweep_lateral_dot,"
        "shadow_decoupled_lateral_valid_pct,shadow_decoupled_lateral_feasible_pct,"
        "shadow_decoupled_lateral_forward_dot,shadow_decoupled_lateral_target_dot,"
        "shadow_decoupled_lateral_abs_error,shadow_decoupled_lateral_forward_rate,"
        "shadow_decoupled_lateral_target_rate,shadow_decoupled_lateral_leg_feasible_pct,"
        "shadow_decoupled_lateral_leg_delta,shadow_decoupled_lateral_leg_sweep,"
        "probe_forced_flips,probe_missed_crossings,probe_cross_wait_s,"
        "probe_cycle_s,probe_peak_xt,probe_phase_per_cycle,"
        "probe_field_ratio,probe_bperp,probe_burial_cov,probe_burial_mae,"
        "probe_cycle_burial_cov,probe_cycle_burial_mae,probe_cycle_burial_sigma,probe_cycle_burial_quality,"
        "shadow_supply,shadow_selection,shadow_consumption,shadow_ready,"
        "shadow_bottleneck_supply,shadow_bottleneck_selection,shadow_bottleneck_consumption,"
        "shadow_axis_pct,shadow_axis_score,shadow_axis_margin,shadow_axis_score_pos,shadow_axis_score_neg,"
        "shadow_axis_pos_pct,shadow_axis_age,"
        "shadow_axis_pass,shadow_axis_reject_nohyp,shadow_axis_reject_candidates,shadow_axis_reject_score,"
        "shadow_axis_reject_margin,shadow_axis_reject_age,shadow_axis_reject_selector_expired,"
        "shadow_axis_score_def,shadow_axis_margin_def,shadow_axis_age_over,"
        "shadow_axis_supply_pct,shadow_axis_validation_pct,shadow_axis_selection_pct,shadow_axis_consumption_pct,"
        "shadow_dual_active_pct,shadow_dual_pass_pct,shadow_dual_reject_val_pct,shadow_dual_reject_feed_pct,"
        "shadow_dual_pass_progressing_pct,shadow_val_pass_progressing_pct,feed_allowed_progressing_pct,"
        "progressing_when_dual_pass_pct,"
        "shadow_progress_align_pct,shadow_progress_align_pass_pct,shadow_progress_align_reverse_pct,"
        "shadow_progress_align_dot,shadow_progress_dual_pass_pct,shadow_progress_dual_reject_dual_pct,"
        "shadow_progress_dual_reject_progress_pct,shadow_progress_dual_pass_progressing_pct,"
        "progressing_when_progress_dual_pass_pct,"
        "shadow_progress_candidate_pct,shadow_progress_candidate_score,shadow_progress_candidate_task_score,"
        "shadow_progress_candidate_combined_score,shadow_progress_candidate_combined_pass_pct,"
        "shadow_progress_candidate_margin,shadow_progress_candidate_dot,shadow_progress_candidate_pos_pct,"
        "shadow_progress_candidate_noaligned_pct,shadow_progress_oracle_pct,shadow_progress_oracle_consistency_pct,"
        "shadow_progress_candidate_forward_pct,shadow_progress_candidate_backward_pct,"
        "shadow_progress_proxy_pct,shadow_progress_proxy_held_pct,shadow_progress_proxy_local_pct,"
        "shadow_progress_proxy_sonar_pct,shadow_progress_proxy_age,shadow_progress_proxy_conf,"
        "shadow_route_proxy_pct,shadow_route_proxy_distance,shadow_route_candidate_dot,"
        "shadow_route_oracle_consistency_pct,"
        "mag_lookahead_pct,mag_lookahead_axis_err,mag_lookahead_pos_err,mag_lookahead_age,"
        "lookahead_feed_pct,feed_reject_age,feed_reject_phase_age,feed_reject_residual,"
        "feed_reject_heading,feed_reject_innovation,feed_phase_age,feed_innovation,"
        "feed_axis_delta,feed_local_residual,"
        "burial_cov,burial_mae,stop",
        flush=True,
    )
    for name, build in _variants():
        if args.name and name not in set(args.name):
            continue
        if args.phase == "d0" and not name.startswith("d0_"):
            continue
        if args.phase == "d1" and not name.startswith(("d0_", "d1_")):
            continue
        if args.phase == "d2" and not name.startswith(("d0_", "d2_")):
            continue
        if args.phase == "d3" and not name.startswith(("d0_", "d3_")):
            continue
        if args.phase == "d4" and not name.startswith(("d0_", "d4_")):
            continue
        if args.phase == "probe" and not name.startswith(("d0_", "p")):
            continue
        if args.phase == "probe" and not args.name and name.startswith((
            "p9_",
            "p10_",
            "p11_",
            "p12_",
            "p13_",
            "p14_",
            "p15_",
            "p16_",
            "p17_",
            "p18_",
            "p19_",
            "p20_",
            "p21_",
            "p22_",
            "p23_",
            "p24_",
            "p25_",
            "p26_",
            "p27_",
            "p28_",
            "p29_",
            "p30_",
            "p31_",
            "p32_",
            "p33_",
            "p34_",
            "p35_",
            "p36_",
            "p37_",
            "p38_",
            "p39_",
        )):
            continue
        scenario = build(base)
        record = simulate_run(scenario, max_steps=args.max_steps)
        metrics = compute_health_metrics(record)
        print(
            f"{_variant_group(name)},{name},{health_score(metrics):.1f},{metrics.mean_heading_error_deg:.1f},"
            f"{metrics.track_mean_cross_track_m:.1f},"
            f"{metrics.track_mean_vehicle_heading_error_deg:.1f},"
            f"{metrics.route_completion_ratio * 100.0:.1f},"
            f"{metrics.final_route_distance_m:.1f},"
            f"{metrics.track_active_fraction * 100.0:.1f},"
            f"{metrics.mode_switches},"
            f"{metrics.magnetic_path_observation_fraction * 100.0:.1f},"
            f"{metrics.magnetic_path_mean_axis_error_deg:.1f},"
            f"{metrics.magnetic_path_mean_position_error_m:.1f},"
            f"{metrics.magnetic_path_mean_cross_track_offset_m:.1f},"
            f"{metrics.magnetic_phase_observation_fraction * 100.0:.1f},"
            f"{metrics.magnetic_phase_mean_axis_error_deg:.1f},"
            f"{metrics.magnetic_phase_mean_position_error_m:.1f},"
            f"{metrics.magnetic_phase_mean_amplitude_m:.1f},"
            f"{metrics.magnetic_phase_detector_emit_fraction * 100.0:.1f},"
            f"{metrics.magnetic_phase_detector_reject_no_pair_fraction * 100.0:.1f},"
            f"{metrics.magnetic_phase_detector_reject_offset_fraction * 100.0:.1f},"
            f"{metrics.magnetic_phase_detector_reject_duration_fraction * 100.0:.1f},"
            f"{metrics.magnetic_phase_detector_reject_axis_fraction * 100.0:.1f},"
            f"{metrics.magnetic_phase_detector_waiting_fraction * 100.0:.1f},"
            f"{metrics.magnetic_phase_detector_mean_candidate_duration_s:.1f},"
            f"{metrics.magnetic_phase_detector_mean_axis_delta_deg:.1f},"
            f"{metrics.zigzag_probe_active_fraction * 100.0:.1f},"
            f"{metrics.zigzag_probe_cycle_count},"
            f"{metrics.zigzag_probe_leg_flip_count},"
            f"{metrics.zigzag_probe_magnetic_crossing_count},"
            f"{metrics.zigzag_probe_magnetic_crossings_per_cycle:.2f},"
            f"{metrics.zigzag_probe_forward_leg_fraction * 100.0:.1f},"
            f"{metrics.zigzag_probe_backward_leg_fraction * 100.0:.1f},"
            f"{metrics.zigzag_probe_stall_leg_fraction * 100.0:.1f},"
            f"{metrics.zigzag_probe_crossing_forward_leg_fraction * 100.0:.1f},"
            f"{metrics.zigzag_probe_crossing_backward_leg_fraction * 100.0:.1f},"
            f"{metrics.zigzag_probe_crossing_stall_leg_fraction * 100.0:.1f},"
            f"{metrics.zigzag_probe_mean_forward_leg_delta_m:.2f},"
            f"{metrics.zigzag_probe_mean_backward_leg_delta_m:.2f},"
            f"{metrics.zigzag_probe_forward_phase_fraction * 100.0:.1f},"
            f"{metrics.zigzag_probe_forward_phase_crossing_count},"
            f"{metrics.zigzag_probe_forward_phase_crossing_fraction * 100.0:.1f},"
            f"{metrics.zigzag_probe_forward_phase_magnetic_path_fraction * 100.0:.1f},"
            f"{metrics.zigzag_probe_forward_phase_magnetic_phase_fraction * 100.0:.1f},"
            f"{metrics.zigzag_probe_forward_phase_lookahead_fraction * 100.0:.1f},"
            f"{metrics.zigzag_probe_forward_phase_candidate_fraction * 100.0:.1f},"
            f"{metrics.shadow_forward_zigzag_valid_fraction * 100.0:.1f},"
            f"{metrics.shadow_forward_zigzag_feasible_fraction * 100.0:.1f},"
            f"{metrics.shadow_forward_zigzag_mean_forward_dot:.3f},"
            f"{metrics.shadow_forward_zigzag_mean_lateral_dot_abs:.3f},"
            f"{metrics.shadow_forward_zigzag_mean_forward_rate_mps:.3f},"
            f"{metrics.shadow_forward_zigzag_mean_lateral_rate_mps:.3f},"
            f"{metrics.shadow_forward_zigzag_completed_leg_feasible_fraction * 100.0:.1f},"
            f"{metrics.shadow_forward_zigzag_mean_leg_route_delta_m:.2f},"
            f"{metrics.shadow_forward_zigzag_mean_leg_lateral_sweep_m:.2f},"
            f"{metrics.shadow_forward_sweep_best_angle_deg:.1f},"
            f"{metrics.shadow_forward_sweep_best_leg_duration_multiplier:.1f},"
            f"{metrics.shadow_forward_sweep_best_feasible_fraction * 100.0:.1f},"
            f"{metrics.shadow_forward_sweep_best_mean_leg_route_delta_m:.2f},"
            f"{metrics.shadow_forward_sweep_best_mean_leg_lateral_sweep_m:.2f},"
            f"{metrics.shadow_forward_sweep_best_forward_dot:.3f},"
            f"{metrics.shadow_forward_sweep_best_lateral_dot_abs:.3f},"
            f"{metrics.shadow_decoupled_lateral_valid_fraction * 100.0:.1f},"
            f"{metrics.shadow_decoupled_lateral_feasible_fraction * 100.0:.1f},"
            f"{metrics.shadow_decoupled_lateral_mean_forward_dot:.3f},"
            f"{metrics.shadow_decoupled_lateral_mean_targeting_dot:.3f},"
            f"{metrics.shadow_decoupled_lateral_mean_abs_error_m:.2f},"
            f"{metrics.shadow_decoupled_lateral_mean_forward_rate_mps:.3f},"
            f"{metrics.shadow_decoupled_lateral_mean_targeting_rate_mps:.3f},"
            f"{metrics.shadow_decoupled_lateral_completed_leg_feasible_fraction * 100.0:.1f},"
            f"{metrics.shadow_decoupled_lateral_mean_leg_route_delta_m:.2f},"
            f"{metrics.shadow_decoupled_lateral_mean_leg_sweep_m:.2f},"
            f"{metrics.magnetic_crossing_probe_forced_flip_count},"
            f"{metrics.magnetic_crossing_probe_missed_count},"
            f"{metrics.magnetic_crossing_probe_mean_wait_s:.1f},"
            f"{metrics.zigzag_probe_mean_cycle_duration_s:.1f},"
            f"{metrics.zigzag_probe_mean_peak_abs_cross_track_m:.1f},"
            f"{metrics.zigzag_probe_phase_events_per_cycle:.2f},"
            f"{metrics.zigzag_probe_mean_abs_field_ratio:.2f},"
            f"{metrics.zigzag_probe_mean_abs_b_perp_nt:.1f},"
            f"{metrics.zigzag_probe_burial_coverage * 100.0:.1f},"
            f"{metrics.zigzag_probe_burial_mae_m:.3f},"
            f"{metrics.zigzag_probe_cycle_burial_coverage * 100.0:.1f},"
            f"{metrics.zigzag_probe_cycle_burial_mae_m:.3f},"
            f"{metrics.zigzag_probe_cycle_burial_mean_sigma_m:.3f},"
            f"{metrics.zigzag_probe_cycle_burial_mean_quality:.3f},"
            f"{metrics.shadow_hypothesis_mean_supply_score:.3f},"
            f"{metrics.shadow_hypothesis_mean_selection_score:.3f},"
            f"{metrics.shadow_hypothesis_mean_consumption_score:.3f},"
            f"{metrics.shadow_hypothesis_mean_readiness_score:.3f},"
            f"{metrics.shadow_hypothesis_bottleneck_supply_fraction * 100.0:.1f},"
            f"{metrics.shadow_hypothesis_bottleneck_selection_fraction * 100.0:.1f},"
            f"{metrics.shadow_hypothesis_bottleneck_consumption_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_hypothesis_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_mean_score:.3f},"
            f"{metrics.shadow_axis_mean_margin:.3f},"
            f"{metrics.shadow_axis_mean_positive_score:.3f},"
            f"{metrics.shadow_axis_mean_negative_score:.3f},"
            f"{metrics.shadow_axis_positive_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_mean_age_s:.1f},"
            f"{metrics.shadow_axis_validation_pass_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_validation_reject_no_hypothesis_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_validation_reject_insufficient_candidates_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_validation_reject_low_score_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_validation_reject_low_margin_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_validation_reject_stale_age_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_validation_reject_selector_expired_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_validation_mean_score_deficit:.3f},"
            f"{metrics.shadow_axis_validation_mean_margin_deficit:.3f},"
            f"{metrics.shadow_axis_validation_mean_age_over_s:.1f},"
            f"{metrics.shadow_axis_supply_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_validation_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_selection_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_consumption_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_dual_gate_active_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_dual_gate_pass_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_dual_gate_reject_validation_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_dual_gate_reject_feed_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_dual_gate_pass_while_progressing_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_validation_pass_while_progressing_fraction * 100.0:.1f},"
            f"{metrics.magnetic_lookahead_feed_allowed_while_progressing_fraction * 100.0:.1f},"
            f"{metrics.route_progressing_while_dual_gate_pass_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_progress_alignment_active_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_progress_alignment_pass_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_progress_alignment_reject_reverse_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_progress_alignment_mean_dot:.3f},"
            f"{metrics.shadow_axis_progress_aligned_dual_gate_pass_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_progress_aligned_dual_gate_reject_dual_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_progress_aligned_dual_gate_reject_progress_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_progress_aligned_dual_gate_pass_while_progressing_fraction * 100.0:.1f},"
            f"{metrics.route_progressing_while_progress_aligned_dual_pass_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_progress_aligned_candidate_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_progress_aligned_candidate_mean_score:.3f},"
            f"{metrics.shadow_axis_progress_aligned_candidate_mean_task_score:.3f},"
            f"{metrics.shadow_axis_progress_aligned_candidate_mean_combined_score:.3f},"
            f"{metrics.shadow_axis_progress_aligned_candidate_combined_pass_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_progress_aligned_candidate_mean_margin:.3f},"
            f"{metrics.shadow_axis_progress_aligned_candidate_mean_dot:.3f},"
            f"{metrics.shadow_axis_progress_aligned_candidate_positive_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_progress_aligned_candidate_reject_no_aligned_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_progress_oracle_active_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_progress_oracle_consistency_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_progress_candidate_forward_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_progress_candidate_backward_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_progress_proxy_valid_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_progress_proxy_held_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_progress_proxy_local_path_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_progress_proxy_sonar_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_progress_proxy_mean_age_s:.1f},"
            f"{metrics.shadow_axis_progress_proxy_mean_confidence:.3f},"
            f"{metrics.shadow_axis_route_bound_proxy_valid_fraction * 100.0:.1f},"
            f"{metrics.shadow_axis_route_bound_proxy_mean_distance_m:.3f},"
            f"{metrics.shadow_axis_route_bound_candidate_mean_dot:.3f},"
            f"{metrics.shadow_axis_route_bound_oracle_consistency_fraction * 100.0:.1f},"
            f"{metrics.magnetic_lookahead_fraction * 100.0:.1f},"
            f"{metrics.magnetic_lookahead_mean_axis_error_deg:.1f},"
            f"{metrics.magnetic_lookahead_mean_position_error_m:.1f},"
            f"{metrics.magnetic_lookahead_mean_age_s:.1f},"
            f"{metrics.magnetic_lookahead_feed_allowed_fraction * 100.0:.1f},"
            f"{metrics.magnetic_lookahead_feed_reject_age_fraction * 100.0:.1f},"
            f"{metrics.magnetic_lookahead_feed_reject_phase_age_fraction * 100.0:.1f},"
            f"{metrics.magnetic_lookahead_feed_reject_residual_fraction * 100.0:.1f},"
            f"{metrics.magnetic_lookahead_feed_reject_heading_fraction * 100.0:.1f},"
            f"{metrics.magnetic_lookahead_feed_reject_innovation_fraction * 100.0:.1f},"
            f"{metrics.magnetic_lookahead_feed_mean_phase_age_s:.1f},"
            f"{metrics.magnetic_lookahead_feed_mean_innovation_m:.1f},"
            f"{metrics.magnetic_lookahead_feed_mean_axis_delta_deg:.1f},"
            f"{metrics.magnetic_lookahead_feed_mean_local_residual_m:.1f},"
            f"{metrics.burial_inversion_coverage * 100.0:.1f},"
            f"{metrics.burial_inversion_mae_m:.3f},"
            f"{record.metadata.get('stop_reason')}",
            flush=True,
        )


if __name__ == "__main__":
    main()
