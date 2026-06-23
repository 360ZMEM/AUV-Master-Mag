"""SNR-weighted sliding-window line fitter for cable centerline estimation."""

from collections import deque
from typing import Deque

import numpy as np

from .state import FitResult, PeakObservation


class WeightedSlidingWindowFitter:
    """结合 SNR 权重的滑动窗口拟合器，用于部署模式下的稳健中心线估计。"""

    def __init__(
        self,
        capacity: int,
        snr_floor: float,
        washout_residual_m: float = 5.0,
        washout_snr_linear_threshold: float = 10.0,
        washout_retention_count: int = 2,
        spatial_exclusion_m: float = 8.0,
    ) -> None:
        """初始化滑动窗口容量、权重下限与洗出阈值。"""
        self.capacity = max(2, capacity)
        self.snr_floor = max(snr_floor, 1.0001)
        self.washout_residual_m = max(washout_residual_m, 0.5)
        self.washout_snr_linear_threshold = max(washout_snr_linear_threshold, self.snr_floor)
        self.washout_retention_count = max(1, washout_retention_count)
        self.spatial_exclusion_m = max(0.0, spatial_exclusion_m)
        self.peak_observations: Deque[PeakObservation] = deque(maxlen=self.capacity)
        self.last_detection_time_s = -1e9

    def _fit_observations(self, observations: Deque[PeakObservation]) -> FitResult:
        """对给定观测序列执行加权直线拟合。"""
        if len(observations) < 2:
            return FitResult(origin_xy_m=None, direction_xy=None, residual_m=float("inf"), covariance_xy_m2=None)

        points = np.vstack([observation.position_xy_m for observation in observations])
        weights = np.array([np.log10(max(observation.snr_linear, self.snr_floor)) for observation in observations], dtype=float)
        weights = np.maximum(weights, 1e-3)
        weights = weights / np.sum(weights)
        centroid = np.sum(points * weights[:, None], axis=0)
        centered = points - centroid
        covariance = np.zeros((2, 2), dtype=float)
        for weight, point in zip(weights, centered):
            covariance += weight * np.outer(point, point)

        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        direction = eigenvectors[:, int(np.argmax(eigenvalues))]
        direction = direction / max(np.linalg.norm(direction), 1e-9)

        # Chronological Sign Correction: 确保特征向量与宏观时间流向一致
        oldest_point = observations[0].position_xy_m
        latest_point = observations[-1].position_xy_m
        macro_vec = latest_point - oldest_point
        if np.dot(direction, macro_vec) < 0:
            direction = -direction

        orthogonal = np.array([-direction[1], direction[0]], dtype=float)
        residual = float(np.sqrt(np.sum(weights * (centered @ orthogonal) ** 2)))
        return FitResult(origin_xy_m=centroid, direction_xy=direction, residual_m=residual, covariance_xy_m2=covariance)

    def add_peak(self, position_xy_m: np.ndarray, snr_linear: float, confidence: float, time_s: float) -> bool:
        """添加峰值观测，并在异常偏离时触发洗出处理。"""
        washout_triggered = False
        position_xy_m = np.asarray(position_xy_m, dtype=float)

        # Spatial mutual exclusion filter: prevent dense clusters from dominating PCA fit
        if self.spatial_exclusion_m > 0.0:
            for i, obs in enumerate(self.peak_observations):
                dist = float(np.linalg.norm(position_xy_m - obs.position_xy_m))
                if dist < self.spatial_exclusion_m:
                    # New point is too close to an existing point
                    if snr_linear > obs.snr_linear:
                        # Replace old point with better SNR new point
                        self.peak_observations[i] = PeakObservation(
                            position_xy_m=position_xy_m,
                            snr_linear=float(max(snr_linear, self.snr_floor)),
                            confidence=float(confidence),
                            time_s=float(time_s),
                        )
                        self.last_detection_time_s = time_s
                        return washout_triggered
                    else:
                        # Old point is better, discard new point
                        self.last_detection_time_s = time_s
                        return washout_triggered

        if len(self.peak_observations) >= 2 and snr_linear >= self.washout_snr_linear_threshold:
            current_fit = self._fit_observations(self.peak_observations)
            if current_fit.direction_xy is not None and np.isfinite(current_fit.residual_m) and current_fit.origin_xy_m is not None:
                orthogonal_xy = np.array([-current_fit.direction_xy[1], current_fit.direction_xy[0]], dtype=float)
                residual_m = abs(float(np.dot(position_xy_m - current_fit.origin_xy_m, orthogonal_xy)))
                if residual_m > self.washout_residual_m:
                    retained = list(self.peak_observations)[-self.washout_retention_count :]
                    self.peak_observations = deque(retained, maxlen=self.capacity)
                    washout_triggered = True
        self.peak_observations.append(
            PeakObservation(
                position_xy_m=position_xy_m,
                snr_linear=float(max(snr_linear, self.snr_floor)),
                confidence=float(confidence),
                time_s=float(time_s),
            )
        )
        self.last_detection_time_s = time_s
        return washout_triggered

    def fit(self) -> FitResult:
        """返回当前滑动窗口中的稳健拟合结果。"""
        return self._fit_observations(self.peak_observations)
