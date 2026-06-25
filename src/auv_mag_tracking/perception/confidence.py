"""Fused confidence estimation from magnetic, fit and sonar cues."""

from typing import Optional

import numpy as np


class ConfidenceEstimator:
    """将磁信号、拟合质量与声呐信息融合为统一置信度。"""

    def __init__(self, lost_timeout_s: float) -> None:
        """初始化丢失超时参数。"""
        self.lost_timeout_s = max(lost_timeout_s, 0.1)

    def magnetic_confidence(
        self,
        snr: float,
        fit_residual_m: float,
        detection_age_s: float,
        weak_signal_flag: bool,
        zigzag_width_m: float = 0.0,
        speed_mps: float = 1.0,
    ) -> float:
        """根据信噪比、拟合残差和动态检测时效评估磁感知置信度。"""
        snr_score = np.clip((snr - 1.0) / 8.0, 0.0, 1.0)
        fit_score = float(np.exp(-fit_residual_m / 10.0)) if np.isfinite(fit_residual_m) else 0.0
        
        # 动态容忍时间：横切一整个宽度所需时间
        dynamic_timeout_s = max(self.lost_timeout_s, (zigzag_width_m * 2.0) / max(speed_mps, 0.5))
        age_score = float(np.exp(-detection_age_s / dynamic_timeout_s))
        
        weak_penalty = 0.35 if weak_signal_flag else 1.0
        return float(np.clip((0.45 * snr_score + 0.35 * fit_score + 0.20 * age_score) * weak_penalty, 0.0, 1.0))

    def fused_confidence(
        self,
        magnetic_confidence: float,
        sonar_confidence: float,
        guidance_source: str,
        fit_residual_m: float = float("inf"),
        fit_covariance_xy_m2: Optional[np.ndarray] = None,
    ) -> float:
        """根据当前引导来源融合磁与声呐置信度。"""
        fit_quality = 0.0
        if np.isfinite(fit_residual_m):
            fit_quality = float(np.exp(-fit_residual_m / 8.0))
        if fit_covariance_xy_m2 is not None:
            covariance_xy_m2 = np.asarray(fit_covariance_xy_m2, dtype=float)
            if covariance_xy_m2.shape == (2, 2):
                eigenvalues = np.linalg.eigvalsh(covariance_xy_m2)
                major_axis_m = float(np.sqrt(max(float(np.max(eigenvalues)), 0.0)))
                minor_axis_m = float(np.sqrt(max(float(np.min(eigenvalues)), 0.0)))
                fit_quality = 0.5 * fit_quality + 0.5 * float(np.exp(-(major_axis_m + 0.5 * minor_axis_m) / 18.0))

        if guidance_source == "SONAR":
            confidence = sonar_confidence if magnetic_confidence <= 0.0 else 0.8 * sonar_confidence + 0.2 * magnetic_confidence
        elif guidance_source == "MAGNETIC":
            confidence = magnetic_confidence if sonar_confidence <= 0.0 else 0.8 * magnetic_confidence + 0.2 * sonar_confidence
        elif guidance_source == "MEMORY":
            confidence = 0.24 + 0.52 * magnetic_confidence + 0.10 * max(sonar_confidence, magnetic_confidence) + 0.14 * fit_quality
            confidence = min(0.82, confidence)
        elif guidance_source == "BLIND":
            confidence = min(0.4, 0.6 * magnetic_confidence + 0.4 * sonar_confidence)
        elif guidance_source == "SONAR_SEED":
            confidence = min(0.6, max(sonar_confidence * 0.8, magnetic_confidence))
        elif guidance_source == "LOCAL_PATH":
            confidence = 0.28 + 0.42 * fit_quality + 0.20 * sonar_confidence + 0.10 * magnetic_confidence
            confidence = min(0.82, confidence)
        else:
            confidence = max(magnetic_confidence, sonar_confidence * 0.75)
        return float(np.clip(confidence, 0.0, 1.0))
