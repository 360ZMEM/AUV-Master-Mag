from __future__ import annotations

from dataclasses import replace
import json
from types import SimpleNamespace

import numpy as np
from scipy.signal import find_peaks

from auv_mag_tracking.api import CableMap, MagneticInput, NavigationInput
from auv_mag_tracking.v2 import (
    AuvMagTrackingPipelineV2, CrossScanConfig, DetectionConfig, FwhmConfig,
    InspectionConfig, V2Config, estimate_fwhm,
)
from auv_mag_tracking.v2.lateral import qualify_lateral_center


def nav(t, x=5.0, y=0.0, heading=90.0):
    return NavigationInput(t, np.array([x, y, -10.0]), heading)


def mag(t, value=1.0):
    return MagneticInput(t, np.tile([value, 0.0, 0.0], (4, 1)), 50.0)


def pipeline(**policy):
    config = V2Config(
        detection=DetectionConfig(window_samples=30, min_ready_samples=10, consecutive_on=2),
        cross_scan=CrossScanConfig(half_span_levels_m=(8.0, 12.0), arrival_tolerance_m=0.01),
        fwhm=FwhmConfig(altitude_m=0.5, smoothing_window=3),
        inspection=InspectionConfig(enabled=True, **policy),
    )
    return AuvMagTrackingPipelineV2(object(), CableMap(np.array([[0., 0.], [80., 0.]])), v2_config=config)


def confirm_wire(p):
    t = 0.0
    for _ in range(12):
        p.step(nav(t, y=-12), mag(t))
        t += 0.2
    for _ in range(2):
        p.step(nav(t), mag(t, 21.0))
        t += 0.2
    assert p.export_state()["mission_phase"] == "confirm"
    waypoints = p.last_output.diagnostics["cross_scan_waypoints_xy_m"]
    first, second = waypoints[:2]
    p.step(nav(t, y=first[1]), mag(t, 1 + 20 / np.sqrt(1 + (first[1] / 1.5) ** 2)))
    t += 0.2
    heading = 90.0 if second[1] > first[1] else -90.0
    for y in np.linspace(first[1], second[1], 161):
        result = p.step(nav(t, y=y, heading=heading), mag(t, 1 + 20 / np.sqrt(1 + (y / 1.5) ** 2)))
        t += 0.2
    assert result.mode == "confirm", result.diagnostics
    return_xy = result.diagnostics["cross_scan_waypoints_xy_m"][2]
    result, guidance = p.step_with_guidance(nav(t, y=return_xy[1], heading=-90), mag(t, 21.0))
    return result, guidance, t


def test_unknown_depth_does_not_disable_confirmed_lateral_tracking():
    p = pipeline()
    result, guidance, _ = confirm_wire(p)
    assert result.mode == "track"
    assert result.diagnostics["lateral_valid"]
    assert not result.diagnostics["fwhm_valid"]
    assert not result.diagnostics["depth_valid"]
    assert result.burial_depth_m is None
    assert "depth_calibration_required" in result.diagnostics["fwhm_reason_codes"]
    assert abs(result.estimated_cable_xy_m[1]) < 0.1
    assert guidance.speed_mps > 0
    assert result.confidence > 0.65
    json.dumps(result.diagnostics, allow_nan=False)


def test_intermediate_field_shape_uses_lateral_fallback_not_uncalibrated_depth():
    # A p=0.7 family lies between the two declared shapes. It can pass a
    # fixed dipole FWHM gate while the physical width-to-distance factor is
    # wrong; the inspection publication gate must still withhold depth.
    p = pipeline()
    x = np.linspace(-50., 50., 501)
    true_vertical_m = 3.
    y = 1. + 20. / (1. + (x / true_vertical_m) ** 2) ** .7
    p._scan_x_m = list(x)
    p._scan_envelope_nt = list(y)
    p._scan_noise_nt = [.08] * len(x)
    p._detection_at_confirm = SimpleNamespace(baseline_nt=1.)
    raw = p._estimate_current_scan_unqualified(altitude_m=.5)
    assert raw.valid, raw.reason_codes
    assert abs(raw.vertical_separation_m - true_vertical_m) > .5
    assert qualify_lateral_center(raw, p.v2_config.fwhm, p.v2_config.inspection).valid
    qualified = p._estimate_current_scan(altitude_m=.5)
    assert not qualified.valid
    assert qualified.vertical_separation_m is None
    assert qualified.burial_depth_m is None
    assert qualified.confidence == 0.
    assert "depth_calibration_required" in qualified.reason_codes


def test_lateral_authority_expires_and_reacquisition_is_bounded():
    p = pipeline(maximum_observation_age_s=5.0)
    _, _, t = confirm_wire(p)
    result, guidance = p.step_with_guidance(nav(t + 6, x=8), mag(t + 6, 21.0))
    assert result.mode == "confirm"
    assert not result.diagnostics["lateral_valid"]
    assert result.burial_depth_m is None
    assert result.diagnostics["continuity_reason"] == "lateral_observation_expired"
    assert 0 <= guidance.speed_mps <= p.v2_config.inspection.maximum_guidance_speed_mps


def test_bad_inputs_hold_and_recover_without_resetting_process():
    p = pipeline()
    p.step_with_guidance(nav(0), mag(0))
    faults = [
        (nav(1, x=np.nan), mag(1)),
        (nav(2), mag(2, np.nan)),
        (nav(3), mag(0)),
        (replace(nav(4), position_std_m=10), mag(4)),
        (replace(nav(5), position_ned_m=np.array([1.])), mag(5)),
        (replace(nav(5), heading_deg="90"), mag(5)),
    ]
    for navigation, magnetic in faults:
        result, guidance = p.step_with_guidance(navigation, magnetic, speed_mps=20)
        assert result.mode == "hold"
        assert guidance.safe_lock_active and guidance.speed_mps == 0
        assert np.isfinite([guidance.desired_heading_deg, guidance.target_depth_m]).all()
        assert not result.diagnostics["depth_valid"]
        json.dumps(result.diagnostics, allow_nan=False)
    result, guidance = p.step_with_guidance(nav(6), mag(6))
    assert result.mode != "hold"
    assert np.isfinite(guidance.speed_mps)


def test_opt_in_stop_buffer_holds_before_boundary_and_recovers_inside():
    boundary = np.array([[-30., -30.], [100., -30.], [100., 36.], [-30., 36.]])
    p = pipeline(boundary_stop_margin_m=10.)
    p.search_boundary_xy_m = boundary
    normal, guidance = p.step_with_guidance(nav(0, y=25.), mag(0))
    assert normal.mode == 'search' and guidance.speed_mps > 0
    held, guidance = p.step_with_guidance(nav(1, y=26.1), mag(1))
    assert held.mode == 'hold'
    assert held.diagnostics['continuity_reason'] == 'boundary_stop_buffer'
    assert guidance.safe_lock_active and guidance.speed_mps == 0
    assert not held.diagnostics['lateral_valid'] and not held.diagnostics['depth_valid']
    recovered, guidance = p.step_with_guidance(nav(2, y=25.), mag(2))
    assert recovered.mode == 'search' and guidance.speed_mps > 0


def test_complete_multiple_peaks_cannot_authorize_tracking():
    x = np.linspace(-12, 12, 481)
    y = 1 + 20 / (1 + ((x - 3) / 0.7) ** 2) + 18 / (1 + ((x + 3) / 0.7) ** 2)
    config = FwhmConfig(altitude_m=0)
    result = estimate_fwhm(x, y, config, baseline_nt=1, noise_nt=0.05)
    lateral = qualify_lateral_center(result, config, InspectionConfig(enabled=True))
    assert not lateral.valid
    assert "ambiguous_anomaly_peaks" in lateral.reason_codes


def test_sub_noise_split_of_one_summit_remains_one_peak():
    x = np.linspace(-15, 15, 1201)
    y = 1 + 20 / np.sqrt(1 + (x / 3) ** 2)
    config = FwhmConfig(altitude_m=0)
    result = estimate_fwhm(x, y, config, baseline_nt=1, noise_nt=0.08)
    smooth = result.smoothed_envelope_nt.copy()
    peak = int(np.argmax(smooth))
    smooth[peak - 1:peak + 2] = [21.03, 21.027, 21.03]
    split = replace(result, smoothed_envelope_nt=smooth)
    raw_peaks, _ = find_peaks(smooth, prominence=0.25 * result.prominence_nt)
    assert len(raw_peaks) == 2
    lateral = qualify_lateral_center(split, config, InspectionConfig(enabled=True))
    assert lateral.valid and "ambiguous_anomaly_peaks" not in lateral.reason_codes

    smooth[peak] = 20.5
    distinct = qualify_lateral_center(
        replace(result, smoothed_envelope_nt=smooth), config, InspectionConfig(enabled=True)
    )
    assert not distinct.valid and "ambiguous_anomaly_peaks" in distinct.reason_codes


def test_stalled_scan_times_out_instead_of_accumulating_forever():
    p = pipeline(maximum_scan_duration_s=5.0)
    for i in range(12):
        p.step(nav(i * 0.2, y=-12), mag(i * 0.2))
    p.step(nav(3), mag(3, 21))
    p.step(nav(4), mag(4, 21))
    result, guidance = p.step_with_guidance(nav(10), mag(10, 21))
    assert result.mode == "reacquire"
    assert result.diagnostics["continuity_reason"] == "scan_timeout"
    assert result.diagnostics["cross_scan_sample_count"] < 10
    assert np.isfinite(guidance.speed_mps)


def test_endpoint_stop_cannot_be_undone_by_requested_speed():
    p = pipeline(maximum_observation_distance_m=100)
    _, _, t = confirm_wire(p)
    p.step_with_guidance(nav(t + 1, x=0), mag(t + 1, 21))
    result, guidance = p.step_with_guidance(nav(t + 2, x=80), mag(t + 2, 21), speed_mps=5)
    assert result.diagnostics["endpoint_complete"]
    assert guidance.speed_mps == 0
    assert guidance.safe_lock_active
    result, guidance = p.step_with_guidance(nav(t + 200, x=80), mag(t + 200, 21), speed_mps=5)
    assert result.mode == "complete"
    assert guidance.speed_mps == 0


def test_malformed_optional_altitude_and_commands_do_not_crash():
    p = pipeline()
    result, guidance = p.step_with_guidance(
        nav(0), mag(0), SimpleNamespace(valid=True, altitude_m="invalid", time_s=None),
        target_depth_m="invalid", speed_mps="invalid",
    )
    assert result.burial_depth_m is None
    assert guidance.safe_lock_active and guidance.speed_mps == 0
    assert guidance.target_depth_m == 10.0


def test_monotonic_flank_only_recentres_retry_never_authorizes_position():
    p = pipeline()
    for i in range(12):
        p.step(nav(i * .2, y=-12), mag(i * .2))
    p.step(nav(3), mag(3, 21))
    start = p.step(nav(4), mag(4, 21))
    first, second = start.diagnostics["cross_scan_waypoints_xy_m"][:2]
    p.step(nav(4.2, y=first[1]), mag(4.2, 5))
    heading = 90. if second[1] > first[1] else -90.
    for i, y in enumerate(np.linspace(first[1], second[1], 161)):
        result = p.step(nav(4.4 + i*.2, y=y, heading=heading), mag(4.4+i*.2, 12+y*.5))
    assert result.mode == "reacquire"
    assert not result.diagnostics["lateral_valid"]
    assert result.burial_depth_m is None
    assert result.diagnostics["cross_scan_center_strategy"] == "bounded_flank_recenter"
    assert abs(result.diagnostics["cross_scan_center_xy_m"][1]) <= 8.
