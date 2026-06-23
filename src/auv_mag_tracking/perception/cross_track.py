"""Peak-free magnetic cross-track estimator for buried-cable tracking."""

from collections import deque
from typing import Deque, Optional

import numpy as np


class MagneticCrossTrackEstimator:
    """Estimate the signed cross-track offset from the magnetic anomaly ratio.

    For a straight line current at vertical separation ``d`` below the vehicle,
    the anomaly's vertical and cable-perpendicular horizontal components obey
    ``B_down / B_perp == y / d``, where ``y`` is the signed cross-track offset.
    Both components are driven by the same line current, so their ratio cancels
    the current and is invariant to its (unknown) amplitude.  A buried cable's
    field is too gentle for peak detection, but this ratio is well defined every
    frame, giving a continuous signed steering signal once the cable orientation
    is known.

    The slope ``y/d`` is the principal axis of the ``(B_perp, B_down)`` scatter
    (total least squares through the origin), which stays robust through AC phase
    and current zero-crossings.  The eigenvalue ratio is reported as a quality
    score so callers can gate out the curved/far regimes where the straight-line
    model no longer holds.
    """

    def __init__(self, window: int, min_perp_amplitude_nt: float, quality_gate: float) -> None:
        """初始化滑动窗口、横向幅值下限与拟合质量门限。"""
        self.window = max(8, window)
        self.min_perp_amplitude_nt = max(min_perp_amplitude_nt, 1e-3)
        self.quality_gate = float(np.clip(quality_gate, 0.0, 1.0))
        self._perp: Deque[float] = deque(maxlen=self.window)
        self._down: Deque[float] = deque(maxlen=self.window)
        self.slope: float = 0.0
        self.quality: float = 0.0

    def update(self, b_perp_nt: float, b_down_nt: float) -> None:
        """追加一组异常分量样本并刷新斜率与质量估计。"""
        self._perp.append(float(b_perp_nt))
        self._down.append(float(b_down_nt))
        self.slope = 0.0
        self.quality = 0.0
        if len(self._perp) < self.window // 2:
            return
        perp = np.asarray(self._perp, dtype=float)
        down = np.asarray(self._down, dtype=float)
        if np.sqrt(np.mean(perp * perp)) < self.min_perp_amplitude_nt:
            return
        covariance = np.array(
            [[np.mean(perp * perp), np.mean(perp * down)],
             [np.mean(perp * down), np.mean(down * down)]],
            dtype=float,
        )
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        principal = eigenvectors[:, int(np.argmax(eigenvalues))]
        total = float(np.sum(eigenvalues))
        if abs(principal[0]) < 1e-9 or total < 1e-12:
            return
        self.slope = float(principal[1] / principal[0])
        self.quality = float(np.max(eigenvalues) / total)

    def cross_track_offset_m(self, vertical_separation_m: float) -> Optional[float]:
        """在拟合质量达标时返回带符号横向偏移，否则返回 None。"""
        if self.quality < self.quality_gate:
            return None
        return self.slope * float(vertical_separation_m)
