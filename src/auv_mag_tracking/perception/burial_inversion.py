"""Calibrated-amplitude magnetic burial-depth inversion.

The spec's original peak-amplitude inverter (§5) assumed the zig-zag would raise
detectable magnetic peaks whose amplitude follows the infinite-wire law
``B = mu0 * I / (2*pi*d)``.  Empirically that path is dead for a *buried* cable:
the field is too gentle to ever fire the peak detector (``#peaks == 0`` on case1),
the discrete-segment simulator field is ~21x weaker than the analytic formula,
and the geometry is ill-conditioned (burial 1.5 m << altitude 6 m), so a
current-independent ratio inversion lands 4-13 m off.

This module replaces that with a *calibrated* amplitude inversion: a single
offline coupling constant ``K`` (nT*m per A_rms) folds the simulator's geometry
and processing-chain attenuation into one number, so the slant range follows

    d_3d = K * I_rms / B_track        (B_track = processed tracking strength, nT)
    burial = sqrt(d_3d**2 - lateral**2) - altitude

``K`` is a per-deployment constant (it depends on the signal mode and the whole
filter chain), not a quantity to be jointly estimated frame-by-frame — estimating
it online re-introduces the ill-conditioning.  Burial is (slowly) constant along a
deployment, so the per-frame inversions are fused with a cumulative robust median,
which is the minimum-variance stable estimator and rejects the transient outliers
from current zero-crossings, far passes and curve segments.
"""

from __future__ import annotations

import bisect
import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional


@dataclass
class BurialEstimate:
    """单帧融合后的埋深反演输出（纯数据）。"""

    depth_m: float
    sigma_m: float            # 1σ 不确定度（由样本四分位距导出）
    fit_quality: float        # [0,1]，融合样本量与离散度的综合可信度


@dataclass
class BurialCycleEstimate:
    """单个 zig-zag cycle 内的埋深后验估计（shadow 诊断）。"""

    depth_m: float
    sigma_m: float
    fit_quality: float
    sample_count: int


class MagneticBurialInverter:
    """标定幅度法磁埋深反演器（累积稳健中位数融合）。"""

    def __init__(
        self,
        coupling_constant_nt_m_per_a_rms: float,
        current_rms_a: float,
        altitude_m: float,
        snr_gate_db: float = 6.0,
        min_strength_nt: float = 1.0,
        min_samples: int = 20,
        max_lateral_offset_m: Optional[float] = None,
        max_samples: Optional[int] = None,
    ) -> None:
        """记录标定常数与门限，并清空累积样本。"""
        self.coupling_constant_nt_m_per_a_rms = float(coupling_constant_nt_m_per_a_rms)
        self.current_rms_a = float(current_rms_a)
        self.altitude_m = float(altitude_m)
        self.snr_gate_db = float(snr_gate_db)
        self.min_strength_nt = max(float(min_strength_nt), 1e-6)
        self.min_samples = max(int(min_samples), 1)
        self.max_lateral_offset_m = (
            float(max_lateral_offset_m) if max_lateral_offset_m is not None else None
        )
        self.max_samples = int(max_samples) if max_samples is not None and int(max_samples) > 0 else None
        self._samples_sorted: List[float] = []
        self._samples_fifo: Deque[float] = deque()

    def reset(self) -> None:
        """清空累积样本，用于重新部署或场景切换。"""
        self._samples_sorted.clear()
        self._samples_fifo.clear()

    @property
    def sample_count(self) -> int:
        return len(self._samples_sorted)

    def update(
        self,
        strength_nt: float,
        lateral_offset_m: float,
        snr_db: float,
    ) -> Optional[BurialEstimate]:
        """消费一帧跟踪强度与横偏，返回融合埋深估计或 None。

        当本帧信号不达标（强度/SNR 过低、几何不自洽）时跳过累积；在累积样本
        未达 ``min_samples`` 前返回 None（暖机门控），避免输出早期噪声值。
        """
        self._accumulate(strength_nt, lateral_offset_m, snr_db)
        if len(self._samples_sorted) < self.min_samples:
            return None
        return self._fuse()

    def _accumulate(self, strength_nt: float, lateral_offset_m: float, snr_db: float) -> None:
        """把一帧合格观测转换为单帧埋深样本并插入有序样本表。

        仅在【近过线】帧入样：burial = sqrt(d3d² - lateral²) - altitude 在
        lateral→0 处对 lateral 误差一阶不敏感（导数趋零），且此处磁场最强、
        SNR 最高，故近过线幅度反演远比远离段稳健。这把 spec §5"在过线峰值处
        反演幅度"的意图落到一个横向门控上（无需依赖始终不触发的磁峰检测）。
        """
        if not (math.isfinite(strength_nt) and strength_nt >= self.min_strength_nt):
            return
        if not (math.isfinite(snr_db) and snr_db >= self.snr_gate_db):
            return
        lateral_m = abs(float(lateral_offset_m)) if math.isfinite(lateral_offset_m) else None
        if lateral_m is None:
            return
        if self.max_lateral_offset_m is not None and lateral_m > self.max_lateral_offset_m:
            return
        slant_range_m = self.coupling_constant_nt_m_per_a_rms * self.current_rms_a / strength_nt
        if slant_range_m <= lateral_m:                       # geometry inconsistent
            return
        burial_m = math.sqrt(slant_range_m * slant_range_m - lateral_m * lateral_m) - self.altitude_m
        bisect.insort(self._samples_sorted, burial_m)
        self._samples_fifo.append(burial_m)
        if self.max_samples is not None:
            while len(self._samples_fifo) > self.max_samples:
                old = self._samples_fifo.popleft()
                idx = bisect.bisect_left(self._samples_sorted, old)
                if idx < len(self._samples_sorted):
                    self._samples_sorted.pop(idx)

    def _fuse(self) -> BurialEstimate:
        """对累积样本做稳健中位数融合，并导出不确定度与可信度。"""
        samples = self._samples_sorted
        n = len(samples)
        depth_m = samples[n // 2] if n % 2 else 0.5 * (samples[n // 2 - 1] + samples[n // 2])
        q25 = samples[int(0.25 * (n - 1))]
        q75 = samples[int(0.75 * (n - 1))]
        sigma_m = max((q75 - q25) / 1.349, 0.0)             # IQR -> Gaussian 1σ
        sample_credit = min(n / float(4 * self.min_samples), 1.0)
        dispersion_credit = 1.0 / (1.0 + sigma_m)
        fit_quality = sample_credit * dispersion_credit
        return BurialEstimate(depth_m=depth_m, sigma_m=sigma_m, fit_quality=fit_quality)


class MagneticBurialCycleEstimator:
    """Cycle-local burial posterior for zig-zag probe diagnostics.

    Unlike :class:`MagneticBurialInverter`, this estimator does not accumulate
    across the whole run.  It answers a narrower D5 question: whether the current
    zig-zag cycle itself contains enough near-crossing magnetic geometry to form
    a stable burial posterior.  It is intentionally shadow-only; callers reset it
    at cycle boundaries and must decide separately whether the result is mature
    enough for any downstream use.
    """

    def __init__(
        self,
        coupling_constant_nt_m_per_a_rms: float,
        current_rms_a: float,
        altitude_m: float,
        snr_gate_db: float = 6.0,
        min_strength_nt: float = 1.0,
        min_samples: int = 3,
        max_lateral_offset_m: float = 2.0,
    ) -> None:
        self.coupling_constant_nt_m_per_a_rms = float(coupling_constant_nt_m_per_a_rms)
        self.current_rms_a = float(current_rms_a)
        self.altitude_m = float(altitude_m)
        self.snr_gate_db = float(snr_gate_db)
        self.min_strength_nt = max(float(min_strength_nt), 1e-6)
        self.min_samples = max(int(min_samples), 1)
        self.max_lateral_offset_m = max(float(max_lateral_offset_m), 1e-6)
        self._samples: List[tuple[float, float]] = []

    def reset(self) -> None:
        self._samples.clear()

    @property
    def sample_count(self) -> int:
        return len(self._samples)

    def update(
        self,
        strength_nt: float,
        lateral_offset_m: float,
        snr_db: float,
    ) -> Optional[BurialCycleEstimate]:
        sample = self._sample(strength_nt, lateral_offset_m, snr_db)
        if sample is not None:
            self._samples.append(sample)
        if len(self._samples) < self.min_samples:
            return None
        return self._fuse()

    def _sample(
        self,
        strength_nt: float,
        lateral_offset_m: float,
        snr_db: float,
    ) -> Optional[tuple[float, float]]:
        if not (math.isfinite(strength_nt) and strength_nt >= self.min_strength_nt):
            return None
        if not (math.isfinite(snr_db) and snr_db >= self.snr_gate_db):
            return None
        lateral_m = abs(float(lateral_offset_m)) if math.isfinite(lateral_offset_m) else None
        if lateral_m is None or lateral_m > self.max_lateral_offset_m:
            return None
        slant_range_m = self.coupling_constant_nt_m_per_a_rms * self.current_rms_a / strength_nt
        if slant_range_m <= lateral_m:
            return None
        burial_m = math.sqrt(slant_range_m * slant_range_m - lateral_m * lateral_m) - self.altitude_m
        if not math.isfinite(burial_m):
            return None
        lateral_credit = max(0.05, 1.0 - lateral_m / self.max_lateral_offset_m)
        snr_credit = min(max((snr_db - self.snr_gate_db) / 18.0, 0.05), 1.0)
        weight = lateral_credit * snr_credit
        return burial_m, weight

    def _fuse(self) -> BurialCycleEstimate:
        values = sorted(self._samples, key=lambda item: item[0])
        depth_m = self._weighted_quantile(values, 0.5)
        q25 = self._weighted_quantile(values, 0.25)
        q75 = self._weighted_quantile(values, 0.75)
        sigma_m = max((q75 - q25) / 1.349, 0.0)
        sample_credit = min(len(values) / float(max(self.min_samples * 2, 1)), 1.0)
        dispersion_credit = 1.0 / (1.0 + sigma_m)
        weight_credit = min(sum(weight for _, weight in values) / float(max(self.min_samples, 1)), 1.0)
        fit_quality = sample_credit * dispersion_credit * weight_credit
        return BurialCycleEstimate(
            depth_m=depth_m,
            sigma_m=sigma_m,
            fit_quality=float(max(0.0, min(1.0, fit_quality))),
            sample_count=len(values),
        )

    @staticmethod
    def _weighted_quantile(sorted_values: List[tuple[float, float]], quantile: float) -> float:
        total_weight = sum(max(weight, 0.0) for _, weight in sorted_values)
        if total_weight <= 1e-12:
            return sorted_values[len(sorted_values) // 2][0]
        target = float(quantile) * total_weight
        cumulative = 0.0
        for value, weight in sorted_values:
            cumulative += max(weight, 0.0)
            if cumulative >= target:
                return value
        return sorted_values[-1][0]
