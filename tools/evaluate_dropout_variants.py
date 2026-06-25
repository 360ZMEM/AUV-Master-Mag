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
        _variant("p40_probe10_extrapolated_low_smooth12", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_feed_local_path=True,
            lookahead_feed_extrapolated_scale=0.25,
            lookahead_feed_heading_smoothing=True,
            lookahead_feed_heading_max_step_deg=12.0,
            lookahead_feed_max_age_s=60.0,
            lookahead_feed_max_phase_age_s=60.0,
            lookahead_feed_max_innovation_m=14.0,
            lookahead_feed_max_axis_delta_deg=35.0,
            lookahead_feed_max_local_residual_m=5.0,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p41_probe10_extrapolated_low_smooth6", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_feed_local_path=True,
            lookahead_feed_extrapolated_scale=0.25,
            lookahead_feed_heading_smoothing=True,
            lookahead_feed_heading_max_step_deg=6.0,
            lookahead_feed_max_age_s=60.0,
            lookahead_feed_max_phase_age_s=60.0,
            lookahead_feed_max_innovation_m=14.0,
            lookahead_feed_max_axis_delta_deg=35.0,
            lookahead_feed_max_local_residual_m=5.0,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p42_probe10_extrapolated_low_smooth9", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_feed_local_path=True,
            lookahead_feed_extrapolated_scale=0.25,
            lookahead_feed_heading_smoothing=True,
            lookahead_feed_heading_max_step_deg=9.0,
            lookahead_feed_max_age_s=60.0,
            lookahead_feed_max_phase_age_s=60.0,
            lookahead_feed_max_innovation_m=14.0,
            lookahead_feed_max_axis_delta_deg=35.0,
            lookahead_feed_max_local_residual_m=5.0,
            local_path_guidance=True,
            feed_max_innovation_m=20.0,
            feed_max_axis_delta_deg=45.0,
        )),
        _variant("p43_probe10_extrapolated_low_axis", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_axis_selection=True,
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
        _variant("p44_probe10_extrapolated_low_curveflip", lambda s: _zigzag_probe(
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
        _variant("p45_probe10_extrapolated_low_axis_hyst", lambda s: _zigzag_probe(
            s,
            angle_deg=10.0,
            magnetic_path=True,
            local_age_s=180.0,
            phase_gate=True,
            phase_min_offset_m=0.5,
            lookahead=True,
            lookahead_axis_selection=True,
            lookahead_axis_hysteresis=True,
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
    ]


def _sparse_sonar(scenario: ScenarioConfig) -> None:
    _sparse_sonar_prob(scenario, 0.20)


def _sparse_sonar_prob(scenario: ScenarioConfig, probability: float) -> None:
    scenario.sonar.fail_after_track_active = False
    scenario.sonar.prob_detection = probability


def _progressive_gate(scenario: ScenarioConfig) -> None:
    scenario.tracking.reacquire_region_progressive_forward_enabled = True


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
    local_path_guidance: bool | None = None,
) -> None:
    scenario.tracking.track_active_zigzag_angle_deg = angle_deg
    scenario.tracking.curve_track_crossing_angle_deg = angle_deg
    scenario.tracking.local_path_curve_track_flip_to_vehicle_enabled = curve_track_flip_to_vehicle
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
        scenario.tracking.magnetic_path_phase_min_duration_s = 2.0
        scenario.tracking.magnetic_path_phase_max_duration_s = 45.0
        scenario.tracking.magnetic_path_phase_max_axis_delta_deg = 35.0
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
        "name,health,mean_err,track_xt,track_vehicle_err,route,final_dist,"
        "track_pct,switches,mag_probe_pct,mag_axis_err,mag_pos_err,mag_offset,"
        "mag_phase_pct,mag_phase_axis_err,mag_phase_pos_err,mag_phase_amp,"
        "probe_active_pct,probe_cycles,probe_flips,probe_cycle_s,probe_peak_xt,probe_phase_per_cycle,"
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
            "p40_",
            "p41_",
            "p42_",
            "p43_",
            "p44_",
            "p45_",
        )):
            continue
        scenario = build(base)
        record = simulate_run(scenario, max_steps=args.max_steps)
        metrics = compute_health_metrics(record)
        print(
            f"{name},{health_score(metrics):.1f},{metrics.mean_heading_error_deg:.1f},"
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
            f"{metrics.zigzag_probe_active_fraction * 100.0:.1f},"
            f"{metrics.zigzag_probe_cycle_count},"
            f"{metrics.zigzag_probe_leg_flip_count},"
            f"{metrics.zigzag_probe_mean_cycle_duration_s:.1f},"
            f"{metrics.zigzag_probe_mean_peak_abs_cross_track_m:.1f},"
            f"{metrics.zigzag_probe_phase_events_per_cycle:.2f},"
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
