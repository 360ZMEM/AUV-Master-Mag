"""Robust sliding-window magnetic detection with Schmitt hysteresis."""

from __future__ import annotations

from collections import deque
import math

import numpy as np

from .config import DetectionConfig
from .types import DetectionResult


class MagneticDetector:
    def __init__(self, config: DetectionConfig) -> None:
        config.validate()
        self.config = config
        self._values: deque[float] = deque(maxlen=config.window_samples)
        self._detected = False
        self._on_count = 0
        self._off_count = 0
        self._previous_score = 0.0
        self._rising = False

    def reset(self) -> None:
        self._values.clear()
        self._detected = False
        self._on_count = 0
        self._off_count = 0
        self._previous_score = 0.0
        self._rising = False

    @property
    def detected(self) -> bool:
        return self._detected

    def update(self, time_s: float, envelope_nt: float) -> DetectionResult:
        value = float(envelope_nt)
        reasons: list[str] = []
        if not math.isfinite(value):
            return DetectionResult(
                time_s=float(time_s), envelope_nt=value, baseline_nt=float("nan"),
                noise_nt=float("nan"), score=0.0, detected=self._detected,
                peak_evidence=False, sample_count=len(self._values),
                reason_codes=("non_finite_measurement",),
            )

        history = np.asarray(self._values, dtype=float)
        if history.size:
            # The lower quantile follows background without letting a broad
            # spatial cable response immediately redefine itself as baseline.
            baseline = float(np.quantile(history, 0.20))
            differences = np.diff(history)
            if differences.size:
                difference_center = float(np.median(differences))
                # Difference MAD rejects a smooth spatial gradient.  sqrt(2)
                # maps independent sample differences back to sample noise.
                mad = float(
                    np.median(np.abs(differences - difference_center)) / math.sqrt(2.0)
                )
            else:
                mad = 0.0
        else:
            baseline, mad = value, 0.0
        noise = max(1.4826 * mad, self.config.minimum_noise_nt)
        score = max(0.0, value - baseline) / noise
        ready = history.size >= self.config.min_ready_samples
        if not ready:
            reasons.append("warming_up")

        peak_evidence = bool(
            ready
            and self._rising
            and score < self._previous_score
            and self._previous_score >= self.config.threshold_on + self.config.peak_prominence_score
        )
        self._rising = score > self._previous_score

        if ready:
            if not self._detected:
                self._on_count = self._on_count + 1 if score >= self.config.threshold_on else 0
                if self._on_count >= self.config.consecutive_on:
                    self._detected = True
                    self._off_count = 0
            else:
                self._off_count = self._off_count + 1 if score <= self.config.threshold_off else 0
                if self._off_count >= self.config.consecutive_off:
                    self._detected = False
                    self._on_count = 0

        self._previous_score = score
        self._values.append(value)
        return DetectionResult(
            time_s=float(time_s), envelope_nt=value, baseline_nt=baseline, noise_nt=noise,
            score=float(score), detected=self._detected, peak_evidence=peak_evidence,
            sample_count=len(self._values), reason_codes=tuple(reasons),
        )
