"""Observable contract failures, not simulated truth, control publication."""
import numpy as np
import pytest
from dataclasses import replace

from auv_mag_tracking.api import CableGuidanceOutput, CableTrackingOutput, MagneticInput, NavigationInput
from auv_mag_tracking.v2.robustness_supervisor import RobustnessContract, RobustnessSupervisor


def inputs(t=0.0, x=10.0, y=6.0, center=6.0, complete=False):
    nav = NavigationInput(t, np.array([x, y, -10.0]), 0.0, speed_mps=0.3)
    nav.ground_velocity_ned_mps = np.array([0.3, 0.0, 0.0])
    nav.ground_velocity_frame_valid = True
    mag = MagneticInput(t, np.tile([21.0, 0., 0.], (4, 1)), 50.0)
    out = CableTrackingOutput(t, np.array([x, center]), y - center, x, 0., None, None, 0.99,
                             'track', {'lateral_valid': True, 'endpoint_complete': complete})
    guidance = CableGuidanceOutput(0.0, 10.0, 0.3, 'track', 'magnetic_v2')
    px = np.arange(-15.0, 16.0, 0.1)
    profile = np.column_stack([px, 1.0 + 20.0 / np.sqrt(1.0 + ((px - 6.0) / 3.0) ** 2)])
    return nav, mag, out, guidance, profile


def apply(gate, **kwargs):
    nav, mag, out, guidance, profile = inputs(**kwargs)
    return gate.apply(nav, mag, out, guidance, profile=profile)


def test_high_confidence_wrong_anchor_never_becomes_target_completion():
    gate = RobustnessSupervisor()
    out, command = apply(gate, center=7.0, complete=True)
    assert out.mode == 'hold' and command.speed_mps == 0
    assert not out.diagnostics['endpoint_complete']
    assert not out.diagnostics['lateral_valid']
    assert out.diagnostics['robustness_reason'] == 'robustness_target_anchor_inconsistent'
    # A later apparently healthy input cannot clear a withdrawn mission.
    out, command = apply(gate, t=0.25, x=10.075)
    assert out.mode == 'hold' and command.safe_lock_active


def test_position_jump_is_detected_even_with_zero_reported_covariance():
    gate = RobustnessSupervisor()
    apply(gate)
    out, command = apply(gate, t=0.25, x=10.075, y=6.8)
    assert out.diagnostics['robustness_reason'] == 'robustness_navigation_jump'
    assert command.speed_mps == 0


def test_missing_world_velocity_cannot_silently_use_position_differences():
    gate = RobustnessSupervisor()
    nav, mag, out, command, profile = inputs()
    del nav.ground_velocity_ned_mps
    held, command = gate.apply(nav, mag, out, command, profile=profile)
    assert held.diagnostics['robustness_reason'] == 'robustness_world_velocity_unavailable'
    assert command.speed_mps == 0


def test_frozen_scan_reference_rejects_persistent_magnetic_change():
    gate = RobustnessSupervisor()
    for i in range(20):
        nav, mag, out, command, profile = inputs(t=i * 0.25, x=10.0 + i * 0.075)
        mag.sample_block_nt *= 0.5
        held, command = gate.apply(nav, mag, out, command, profile=profile)
    assert held.mode == 'hold' and command.speed_mps == 0
    assert held.diagnostics['robustness_reason'] == 'robustness_scan_track_inconsistent'


def test_clean_measured_scan_allows_progress_without_publishing_depth():
    gate = RobustnessSupervisor()
    for i in range(25):
        out, command = apply(gate, t=i * 0.25, x=10.0 + i * 0.075)
        assert out.mode == 'track' and command.speed_mps > 0
        assert out.burial_depth_m is None
    assert out.diagnostics['robustness_max_offset_upper_bound_m'] == pytest.approx(0.15)


def test_scan_reference_is_required_to_avoid_truth_or_future_sample_fill():
    gate = RobustnessSupervisor()
    nav, mag, out, command, _ = inputs()
    held, command = gate.apply(nav, mag, out, command)
    assert held.diagnostics['robustness_reason'] == 'robustness_scan_reference_unavailable'
    assert command.speed_mps == 0


def test_rejoin_outside_measured_profile_is_not_constant_tail_extrapolation():
    gate = RobustnessSupervisor()
    out, command = apply(gate, x=-60.0, y=18.0)
    assert out.mode == 'track' and command.speed_mps > 0
    assert out.diagnostics['robustness_profile_supported'] is False
    assert out.diagnostics['robustness_profile_monitor_status'] == 'outside_scanned_support'
    assert 'robustness_expected_magnetic_nt' not in out.diagnostics


def test_no_external_identity_never_authorizes_a_named_target():
    gate = RobustnessSupervisor(replace(RobustnessContract(), enforce_target_anchor=False,
                                        target_anchor_center_m=0.0))
    out, command = apply(gate, complete=True)
    assert out.mode == 'track' and command.speed_mps > 0
    assert out.diagnostics['endpoint_complete'] is False
    assert out.diagnostics['geometric_route_complete'] is True
    assert out.diagnostics['route_quality_valid'] is False
    assert out.diagnostics['named_target_completed'] is False
    assert out.diagnostics['named_target_authorized'] is False
    assert out.diagnostics['source'] == 'unidentified_magnetic_anomaly'


def test_core_disturbance_change_does_not_need_simulated_current_truth():
    gate = RobustnessSupervisor()
    for i in range(18):
        nav, mag, out, command, profile = inputs(t=i * 0.25, x=10.0 + i * 0.075)
        out.diagnostics['flythrough_lateral_disturbance_proxy_xy_mps'] = [0.0, 0.0 if i < 15 else 0.05]
        held, command = gate.apply(nav, mag, out, command, profile=profile)
    assert held.diagnostics['robustness_reason'] == 'robustness_lateral_disturbance_changed'
    assert command.speed_mps == 0


def test_latched_withdrawal_requests_upstream_recovery_after_finite_budget():
    gate = RobustnessSupervisor()
    apply(gate, center=7.0)
    out, command = apply(gate, t=30.0, x=19.0)
    assert out.diagnostics['robustness_higher_level_recovery_requested'] is True
    assert out.diagnostics['robustness_hold_elapsed_s'] == 30.0
    assert out.mode == 'hold' and command.speed_mps == 0.0
