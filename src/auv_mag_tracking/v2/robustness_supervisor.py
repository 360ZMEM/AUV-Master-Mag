"""Optional observation-only supervisor for a surveyed straight toy corridor.

This module does not identify arbitrary magnetic sources or estimate a hidden
navigation bias. Its surveyed anchor and navigation error bounds are explicit
external assumptions. It never accepts plant state or evaluation truth.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
import math

import numpy as np


@dataclass(frozen=True)
class RobustnessContract:
    enforce_target_anchor: bool = True
    target_anchor_center_m: float = 6.0
    target_anchor_half_width_m: float = 0.05
    navigation_position_bound_m: float = 0.10
    magnetic_center_bound_m: float = 0.10
    maximum_true_mean_offset_m: float = 1.0
    maximum_true_offset_m: float = 2.0
    consistency_window_s: float = 20.0
    position_increment_margin_m: float = 0.35
    ground_velocity_bias_bound_mps: float = 0.01
    profile_residual_limit: float = 0.08
    profile_window_samples: int = 16
    maximum_disturbance_change_mps: float = 0.025
    disturbance_window_s: float = 4.0
    minimum_quality_samples: int = 200
    core_start_m: float = 5.0
    core_end_m: float = 75.0
    hold_budget_s: float = 30.0

    def __post_init__(self):
        values = [self.target_anchor_center_m, self.target_anchor_half_width_m,
                  self.navigation_position_bound_m, self.magnetic_center_bound_m,
                  self.maximum_true_mean_offset_m, self.maximum_true_offset_m,
                  self.consistency_window_s, self.position_increment_margin_m,
                  self.ground_velocity_bias_bound_mps, self.profile_residual_limit,
                  self.core_start_m, self.core_end_m]
        if not all(math.isfinite(v) for v in values):
            raise ValueError('robustness contract must be finite')
        if any(v <= 0 for v in values[1:11]) or self.core_end_m <= self.core_start_m:
            raise ValueError('invalid robustness error or time budget')
        if self.profile_window_samples < 2 or self.minimum_quality_samples < 1:
            raise ValueError('invalid robustness sample budget')
        if self.maximum_true_mean_offset_m > self.maximum_true_offset_m:
            raise ValueError('mean limit cannot exceed maximum limit')
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.maximum_disturbance_change_mps, self.disturbance_window_s, self.hold_budget_s)):
            raise ValueError('invalid disturbance window')


class RobustnessSupervisor:
    """Latch zero guidance on inconsistency; keep legacy tracking API types.

    ``profile`` contains measured normal positions and envelopes from the same
    accepted cross scan. The reference is frozen and is not updated from later
    magnetic inputs, so a persistent fault cannot be learned away.
    """

    def __init__(self, contract: RobustnessContract | None = None):
        self.contract = contract or RobustnessContract()
        self.reason = None
        self.withdrawal_time_s = None
        self._previous = None
        self._history = deque()
        self._integrated_velocity = np.zeros(2)
        self._profile = None
        self._residuals = deque(maxlen=self.contract.profile_window_samples)
        self._quality_sum = 0.0
        self._quality_count = 0
        self._quality_max = 0.0
        self._disturbance_history = deque()

    def _withdraw(self, reason, time_s):
        if self.reason is None:
            self.reason = reason
            self.withdrawal_time_s = float(time_s) if math.isfinite(float(time_s)) else 0.0

    def apply(self, navigation, magnetic, output, guidance, *, profile=None):
        c = self.contract
        now = float(navigation.time_s)
        xy = np.asarray(navigation.position_ned_m, dtype=float)[:2]
        velocity = getattr(navigation, 'ground_velocity_ned_mps', None)
        diag = dict(output.diagnostics)
        diag.update(robustness_supervisor_enabled=True,
                    robustness_scope='surveyed_straight_toy_corridor',
                    robustness_truth_used=False,
                    robustness_anchor_center_m=c.target_anchor_center_m if c.enforce_target_anchor else None,
                    robustness_anchor_half_width_m=c.target_anchor_half_width_m if c.enforce_target_anchor else None,
                    named_target_authorized=False,
                    robustness_navigation_bound_m=c.navigation_position_bound_m)
        if output.mode == 'hold' and self.reason is None:
            self._withdraw(str(diag.get('continuity_reason') or 'robustness_upstream_hold'), now)
        if self.reason is None:
            if velocity is None or np.asarray(velocity).shape != (3,) or not np.isfinite(velocity).all():
                self._withdraw('robustness_world_velocity_unavailable', now)
            elif not np.isfinite(xy).all() or not math.isfinite(now):
                self._withdraw('robustness_navigation_invalid', now)
            elif getattr(navigation, 'ground_velocity_frame_valid', True) is not True:
                self._withdraw('robustness_world_velocity_frame_invalid', now)
            else:
                v = np.asarray(velocity, dtype=float)[:2]
                if self._previous is not None:
                    pt, pxy, pv = self._previous
                    dt = now - pt
                    if not 0 < dt <= 1.0:
                        self._withdraw('robustness_navigation_time_invalid', now)
                    else:
                        dv = dt * (pv + v) / 2.0
                        self._integrated_velocity += dv
                        increment_error = float(np.linalg.norm(xy - pxy - dv))
                        diag['robustness_position_increment_residual_m'] = increment_error
                        if increment_error > c.position_increment_margin_m:
                            self._withdraw('robustness_navigation_jump', now)
                self._previous = (now, xy.copy(), v.copy())
                self._history.append((now, xy.copy(), self._integrated_velocity.copy()))
                while len(self._history) > 1 and self._history[1][0] <= now - c.consistency_window_s:
                    self._history.popleft()
                t0, xy0, integral0 = self._history[0]
                elapsed = now - t0
                if elapsed >= c.consistency_window_s:
                    residual = float(np.linalg.norm(xy - xy0 - (self._integrated_velocity - integral0)))
                    limit = c.position_increment_margin_m + c.ground_velocity_bias_bound_mps * elapsed
                    diag.update(robustness_navigation_window_residual_m=residual,
                                robustness_navigation_window_limit_m=limit)
                    if residual > limit:
                        self._withdraw('robustness_navigation_drift', now)

        if output.mode == 'track' and self.reason is None:
            center = float(output.estimated_cable_xy_m[1])
            residual = abs(center - c.target_anchor_center_m)
            limit = c.target_anchor_half_width_m + c.navigation_position_bound_m + c.magnetic_center_bound_m
            diag.update(robustness_target_anchor_residual_m=residual,
                        robustness_target_anchor_limit_m=limit)
            if c.enforce_target_anchor and (not math.isfinite(center) or residual > limit):
                self._withdraw('robustness_target_anchor_inconsistent', now)
            if not c.enforce_target_anchor:
                diag.update(source='unidentified_magnetic_anomaly',
                            target_identity_authority='unavailable',
                            robustness_completion_scope='geometric_anomaly_route_only',
                            geometric_route_complete=bool(diag.get('endpoint_complete')),
                            geometric_route_quality_valid=bool(diag.get('route_quality_valid')),
                            endpoint_complete=False, route_quality_valid=False)
            if self._profile is None:
                measured = np.asarray(profile if profile is not None else [], dtype=float)
                if measured.ndim != 2 or measured.shape[0] < 20 or measured.shape[1] < 2 or not np.isfinite(measured[:, :2]).all():
                    self._withdraw('robustness_scan_reference_unavailable', now)
                else:
                    bins = np.round(measured[:, 0] / 0.2).astype(int)
                    keys = np.unique(bins)
                    xs = np.array([np.median(measured[bins == k, 0]) for k in keys])
                    ys = np.array([np.median(measured[bins == k, 1]) for k in keys])
                    amplitude = float(np.max(ys) - np.min(ys))
                    if len(keys) < 10 or amplitude < 1.0:
                        self._withdraw('robustness_scan_reference_insufficient', now)
                    else:
                        self._profile = (xs, ys, amplitude)
            if self._profile is not None:
                px, py, amplitude = self._profile
                supported = bool(px[0] <= xy[1] <= px[-1])
                diag['robustness_profile_supported'] = supported
                if supported:
                    raw = np.asarray(magnetic.sample_block_nt, dtype=float)
                    measured_value = float(np.mean(np.linalg.norm(raw, axis=1)))
                    expected = float(np.interp(xy[1], px, py))
                    value_residual = abs(measured_value - expected) / amplitude
                    if math.isfinite(value_residual):
                        self._residuals.append(value_residual)
                    else:
                        self._withdraw('robustness_magnetic_invalid', now)
                    diag.update(robustness_expected_magnetic_nt=expected,
                                robustness_profile_residual=value_residual,
                                robustness_profile_residual_limit=c.profile_residual_limit)
                    if len(self._residuals) == c.profile_window_samples:
                        window_residual = float(np.mean(self._residuals))
                        diag['robustness_profile_window_residual'] = window_residual
                        if window_residual > c.profile_residual_limit:
                            self._withdraw('robustness_scan_track_inconsistent', now)
                else:
                    # Rejoin can briefly leave the observed interval. Do not
                    # extrapolate a frozen constant tail or call it valid.
                    self._residuals.clear()
                    diag['robustness_profile_monitor_status'] = 'outside_scanned_support'
            if c.core_start_m <= xy[0] <= c.core_end_m:
                # Velocity-minus-heading also includes sideslip. Limit only
                # short-term changes on the inspection leg, not rejoin turns.
                proxy = diag.get('flythrough_lateral_disturbance_proxy_xy_mps')
                if proxy is not None and len(proxy) >= 2 and np.isfinite(proxy).all():
                    self._disturbance_history.append((now, float(proxy[1])))
                    while len(self._disturbance_history) > 1 and self._disturbance_history[1][0] <= now - c.disturbance_window_s:
                        self._disturbance_history.popleft()
                    t0, p0 = self._disturbance_history[0]
                    if now - t0 >= c.disturbance_window_s:
                        change = float(proxy[1]) - p0
                        diag.update(robustness_disturbance_change_mps=change,
                                    robustness_disturbance_change_limit_mps=c.maximum_disturbance_change_mps)
                        if abs(change) > c.maximum_disturbance_change_mps:
                            self._withdraw('robustness_lateral_disturbance_changed', now)
                # Triangle inequality against a separately surveyed target:
                # |true vehicle - true target| <= |nav - anchor| + Bnav + Banchor.
                bound = abs(xy[1] - c.target_anchor_center_m) + c.navigation_position_bound_m + c.target_anchor_half_width_m
                if c.enforce_target_anchor:
                    self._quality_sum += bound
                    self._quality_count += 1
                    self._quality_max = max(self._quality_max, bound)
                    diag['robustness_true_offset_upper_bound_m'] = bound
                    if bound > c.maximum_true_offset_m:
                        self._withdraw('robustness_offset_budget_exceeded', now)
            if self._quality_count:
                diag.update(robustness_mean_offset_upper_bound_m=self._quality_sum / self._quality_count,
                            robustness_max_offset_upper_bound_m=self._quality_max,
                            robustness_quality_samples=self._quality_count)
            if diag.get('endpoint_complete') and c.enforce_target_anchor:
                mean = self._quality_sum / self._quality_count if self._quality_count else math.inf
                if self._quality_count < c.minimum_quality_samples or mean > c.maximum_true_mean_offset_m:
                    self._withdraw('robustness_mean_offset_budget_rejected', now)
                else:
                    diag['robustness_conditional_quality_valid'] = True
            if c.enforce_target_anchor and self.reason is None:
                diag['named_target_authorized'] = True
            diag['named_target_completed'] = bool(diag.get('endpoint_complete') and diag['named_target_authorized'])
        if self.reason is not None:
            elapsed = max(0.0, now - self.withdrawal_time_s) if math.isfinite(now) else 0.0
            diag.update(endpoint_complete=False, route_quality_valid=False,
                        robustness_conditional_quality_valid=False,
                        robustness_reason=self.reason, robustness_withdrawal_time_s=self.withdrawal_time_s,
                        continuity_reason=self.reason, lateral_valid=False, depth_valid=False)
            diag.update(named_target_authorized=False, named_target_completed=False,
                        geometric_route_complete=False, robustness_hold_elapsed_s=elapsed,
                        robustness_hold_budget_s=c.hold_budget_s,
                        robustness_higher_level_recovery_requested=elapsed >= c.hold_budget_s,
                        robustness_hold_scope='zero_commands_then_request_upstream_recovery; not station_keeping')
            output = replace(output, mode='hold', confidence=0.0, burial_depth_m=None, burial_sigma_m=None,
                             diagnostics=diag)
            guidance = replace(guidance, mode='hold', speed_mps=0.0, safe_lock_active=True,
                               guidance_source='v2_robustness_supervisor', diagnostics=dict(diag))
        else:
            output = replace(output, diagnostics=diag)
        return output, guidance
