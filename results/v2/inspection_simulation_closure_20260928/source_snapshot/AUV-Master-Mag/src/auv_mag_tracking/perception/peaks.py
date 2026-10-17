"""Magnetic field peak detection state machine."""

from collections import deque
from typing import Deque, Optional

import numpy as np

from .state import PeakEvent, PeakZoneSample


class PeakDetector:
    """基于上升-峰区-下降状态机检测磁场峰值事件。"""

    def __init__(
        self,
        min_peak_strength_nt: float,
        turn_trigger_ratio: float,
        hysteresis_fraction: float,
        cooldown_s: float,
        ascending_min_samples: int = 2,
        descending_min_samples: int = 2,
        peak_zone_window_size: int = 20,
    ) -> None:
        """初始化峰值检测器的阈值、滞回和冷却参数。"""
        self.min_peak_strength_nt = min_peak_strength_nt
        self.turn_trigger_ratio = turn_trigger_ratio
        self.hysteresis_fraction = hysteresis_fraction
        self.cooldown_s = cooldown_s
        self.ascending_min_samples = max(1, ascending_min_samples)
        self.descending_min_samples = max(1, descending_min_samples)
        self.peak_zone_window_size = max(3, peak_zone_window_size)
        self.current_peak_strength_nt = 0.0
        self.current_peak_time_s = -1e9
        self.current_peak_position_xy_m: Optional[np.ndarray] = None
        self.cooldown_until_s = -1e9
        self.state = "IDLE"
        self.previous_strength_nt: Optional[float] = None
        self.previous_time_s: Optional[float] = None
        self.ascending_count = 0
        self.descending_count = 0
        self.recent_samples: Deque[PeakZoneSample] = deque(maxlen=self.peak_zone_window_size)
        self.peak_zone_samples: Deque[PeakZoneSample] = deque(maxlen=self.peak_zone_window_size)
        # Deep reset mechanism
        self.armed = True

    def _reset_tracking_state(self) -> None:
        """重置峰区追踪状态，回到空闲等待下一次峰值。"""
        self.state = "IDLE"
        self.ascending_count = 0
        self.descending_count = 0
        self.current_peak_strength_nt = 0.0
        self.current_peak_time_s = self.previous_time_s if self.previous_time_s is not None else -1e9
        self.current_peak_position_xy_m = None
        self.peak_zone_samples.clear()

    def _update_current_peak(self, sample: PeakZoneSample) -> None:
        """使用当前样本刷新峰区中的最大峰值记录。"""
        if sample.strength_nt >= self.current_peak_strength_nt:
            self.current_peak_strength_nt = sample.strength_nt
            self.current_peak_time_s = sample.time_s
            self.current_peak_position_xy_m = None if sample.position_xy_m is None else sample.position_xy_m.copy()

    def _emit_peak_event(
        self,
        time_s: float,
        vehicle_heading_deg: Optional[float] = None,
        use_nominal_route_prior: bool = True,
    ) -> PeakEvent:
        """从峰区样本中构造最终峰值事件并清空内部状态。"""
        if not self.peak_zone_samples:
            self._reset_tracking_state()
            self.cooldown_until_s = time_s + self.cooldown_s
            return PeakEvent(detected=False)

        strengths_nt = np.array([sample.strength_nt for sample in self.peak_zone_samples], dtype=float)
        times_s = np.array([sample.time_s for sample in self.peak_zone_samples], dtype=float)
        if np.max(strengths_nt) < self.min_peak_strength_nt:
            self._reset_tracking_state()
            self.cooldown_until_s = time_s + self.cooldown_s
            return PeakEvent(detected=False)

        # --- Parabolic interpolation for sub-sample peak time ---
        peak_idx = int(np.argmax(strengths_nt))
        y_peak = float(strengths_nt[peak_idx])
        interpolated_peak_time_s = times_s[peak_idx]
        if len(strengths_nt) >= 3 and peak_idx > 0 and peak_idx < len(strengths_nt) - 1:
            y_prev = strengths_nt[peak_idx - 1]
            y_next = strengths_nt[peak_idx + 1]
            denominator = 2.0 * (2.0 * y_peak - y_prev - y_next)
            if abs(denominator) > 1e-12:
                offset = (y_prev - y_next) / denominator
                offset = float(np.clip(offset, -0.5, 0.5))
                dt_samples = times_s[peak_idx + 1] - times_s[peak_idx]
                interpolated_peak_time_s = times_s[peak_idx] + offset * dt_samples

        # Interpolated peak strength via parabolic estimate
        interpolated_strength = y_peak
        if len(strengths_nt) >= 3 and peak_idx > 0 and peak_idx < len(strengths_nt) - 1:
            y_prev = strengths_nt[peak_idx - 1]
            y_next = strengths_nt[peak_idx + 1]
            curvature = y_prev - 2.0 * y_peak + y_next
            if abs(curvature) > 1e-12:
                interpolated_strength = y_peak - ((y_prev - y_next) ** 2) / (8.0 * curvature)
                if not np.isfinite(interpolated_strength):
                    interpolated_strength = y_peak

        weights = np.maximum(strengths_nt, 1e-6)
        peak_time_s = interpolated_peak_time_s

        centroid_xy_m = None
        interpolated_position_xy_m = None
        weighted_positions = [
            (sample.position_xy_m, weight)
            for sample, weight in zip(self.peak_zone_samples, weights)
            if sample.position_xy_m is not None
        ]
        if weighted_positions:
            positions_xy = np.vstack([position_xy for position_xy, _ in weighted_positions])
            position_weights = np.array([weight for _, weight in weighted_positions], dtype=float)
            centroid_xy_m = np.sum(positions_xy * position_weights[:, None], axis=0) / np.sum(position_weights)

            if len(self.peak_zone_samples) >= 2:
                sample_times = np.array([sample.time_s for sample in self.peak_zone_samples], dtype=float)
                sample_positions = np.array([
                    np.asarray(sample.position_xy_m, dtype=float)
                    for sample in self.peak_zone_samples
                    if sample.position_xy_m is not None
                ], dtype=float)
                valid_time_mask = np.array([sample.position_xy_m is not None for sample in self.peak_zone_samples], dtype=bool)
                sample_times = sample_times[valid_time_mask]
                if sample_times.size >= 2 and sample_positions.shape[0] >= 2:
                    order = np.argsort(sample_times)
                    sample_times = sample_times[order]
                    sample_positions = sample_positions[order]
                    if peak_time_s <= sample_times[0]:
                        interpolated_position_xy_m = sample_positions[0].copy()
                    elif peak_time_s >= sample_times[-1]:
                        interpolated_position_xy_m = sample_positions[-1].copy()
                    else:
                        upper_index = int(np.searchsorted(sample_times, peak_time_s, side="right"))
                        lower_index = max(0, upper_index - 1)
                        upper_index = min(upper_index, sample_times.size - 1)
                        t0 = float(sample_times[lower_index])
                        t1 = float(sample_times[upper_index])
                        p0 = sample_positions[lower_index]
                        p1 = sample_positions[upper_index]
                        if t1 > t0:
                            alpha = float(np.clip((peak_time_s - t0) / (t1 - t0), 0.0, 1.0))
                            interpolated_position_xy_m = (1.0 - alpha) * p0 + alpha * p1
                        else:
                            interpolated_position_xy_m = p0.copy()

        # When the vehicle heading is known, estimate the cable heading from
        # the crossing direction.  An infinite straight wire produces maximum
        # field strength when the vehicle passes perpendicular to it, so the
        # cable direction is approximately ±90° from the vehicle heading at
        # peak.  We record both candidates; disambiguation happens later when
        # multiple peaks are available.
        estimated_cable_heading_deg = None
        if vehicle_heading_deg is not None:
            estimated_cable_heading_deg = vehicle_heading_deg + 90.0

        event = PeakEvent(
            detected=True,
            peak_strength_nt=interpolated_strength,
            peak_time_s=peak_time_s,
            peak_position_xy_m=(
                interpolated_position_xy_m.copy()
                if (interpolated_position_xy_m is not None and not use_nominal_route_prior)
                else (centroid_xy_m.copy() if centroid_xy_m is not None else None)
            ),
            estimated_cable_heading_deg=estimated_cable_heading_deg,
        )
        self._reset_tracking_state()
        self.cooldown_until_s = time_s + self.cooldown_s
        return event

    def update(
        self,
        strength_nt: float,
        time_s: float,
        position_xy_m: Optional[np.ndarray] = None,
        vehicle_heading_deg: Optional[float] = None,
        use_nominal_route_prior: bool = True,
    ) -> PeakEvent:
        """输入新的强度样本，推动峰值状态机并在必要时输出事件。"""
        sample = PeakZoneSample(
            time_s=float(time_s),
            strength_nt=float(strength_nt),
            position_xy_m=None if position_xy_m is None else np.asarray(position_xy_m, dtype=float).copy(),
        )
        self.recent_samples.append(sample)

        if self.previous_strength_nt is None or self.previous_time_s is None:
            self.previous_strength_nt = sample.strength_nt
            self.previous_time_s = sample.time_s
            self.current_peak_strength_nt = sample.strength_nt
            self.current_peak_time_s = sample.time_s
            self.current_peak_position_xy_m = None if sample.position_xy_m is None else sample.position_xy_m.copy()
            return PeakEvent(detected=False)

        # --- Deep Reset / Armed Check ---
        if not self.armed:
            recovery_threshold = max(0.35 * self.current_peak_strength_nt, self.min_peak_strength_nt)
            if sample.strength_nt < recovery_threshold:
                self.armed = True
            else:
                self.previous_strength_nt = sample.strength_nt
                self.previous_time_s = sample.time_s
                return PeakEvent(detected=False)

        if time_s < self.cooldown_until_s:
            self.previous_strength_nt = sample.strength_nt
            self.previous_time_s = sample.time_s
            return PeakEvent(detected=False)

        # --- Local-slope morphology with consecutive-count gating ---
        morphology_trend = self._get_morphology_trend(sample.strength_nt)

        if morphology_trend == "rising":
            self.ascending_count += 1
            self.descending_count = 0
        elif morphology_trend == "falling":
            self.descending_count += 1
            self.ascending_count = 0
        else:
            self.ascending_count = 0
            self.descending_count = 0

        if self.state == "IDLE" and self.ascending_count >= self.ascending_min_samples:
            self.state = "ASCENDING"
            self.peak_zone_samples = deque(self.recent_samples, maxlen=self.peak_zone_window_size)
            self._update_current_peak(sample)
        elif self.state == "ASCENDING":
            self.peak_zone_samples.append(sample)
            self._update_current_peak(sample)
            if morphology_trend != "rising":
                self.state = "PEAK_ZONE"
                self.descending_count = 1 if morphology_trend == "falling" else 0
        elif self.state == "PEAK_ZONE":
            self.peak_zone_samples.append(sample)
            self._update_current_peak(sample)
            if morphology_trend == "falling" and self.descending_count >= self.descending_min_samples:
                event = self._emit_peak_event(
                    sample.time_s,
                    vehicle_heading_deg=vehicle_heading_deg,
                    use_nominal_route_prior=use_nominal_route_prior,
                )
                if event.detected:
                    self.armed = False
                self.previous_strength_nt = sample.strength_nt
                self.previous_time_s = sample.time_s
                return event
            if morphology_trend == "rising":
                self.descending_count = 0

        self.previous_strength_nt = sample.strength_nt
        self.previous_time_s = sample.time_s
        return PeakEvent(detected=False)

    def _get_morphology_trend(self, strength_nt: float) -> str:
        """Classify the local trend from the previous sample to this one.

        A peak detector must react to the *local* slope, not an average over a
        long window (which stays biased by the ascent and masks the descent).
        ``hysteresis_fraction`` of the running peak forms a symmetric deadband
        so sensor noise near the crest does not flip the trend, while a drop
        below ``turn_trigger_ratio`` of the running peak forces ``falling`` as a
        fast path even inside that deadband.
        """
        if self.previous_strength_nt is None:
            return "flat"

        deadband_nt = self.hysteresis_fraction * max(self.current_peak_strength_nt, abs(strength_nt))
        delta_nt = strength_nt - self.previous_strength_nt

        if self.current_peak_strength_nt > 1e-6 and strength_nt < self.turn_trigger_ratio * self.current_peak_strength_nt:
            return "falling"
        if delta_nt > deadband_nt:
            return "rising"
        if delta_nt < -deadband_nt:
            return "falling"
        return "flat"
