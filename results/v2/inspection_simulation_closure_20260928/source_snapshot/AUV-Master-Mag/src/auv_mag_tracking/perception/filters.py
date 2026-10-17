"""Streaming signal filters used by the perception layer."""

from collections import deque
from typing import Deque

import numpy as np
try:
    from scipy.signal import butter, sosfilt
except ModuleNotFoundError:
    butter = None
    sosfilt = None


class LowPassFilter:
    """实现一阶离散低通滤波器，用于平滑瞬时测量值。"""

    def __init__(self, time_constant_s: float) -> None:
        """初始化低通滤波器并设置时间常数。"""
        self.time_constant_s = max(time_constant_s, 1e-3)
        self.value = 0.0
        self.initialized = False

    def update(self, measurement: float, dt_s: float) -> float:
        """根据新测量值更新滤波输出。"""
        alpha = dt_s / (self.time_constant_s + dt_s)
        if not self.initialized:
            self.value = measurement
            self.initialized = True
        else:
            self.value = (1.0 - alpha) * self.value + alpha * measurement
        return self.value


class MedianWindowFilter:
    """实现固定窗口中值滤波器，用于抑制瞬态离群点。"""

    def __init__(self, window_size: int) -> None:
        """初始化中值滤波窗口大小。"""
        self.buffer: Deque[float] = deque(maxlen=max(1, window_size))

    def update(self, measurement: float) -> float:
        """写入新样本并返回窗口中值。"""
        self.buffer.append(measurement)
        return float(np.median(np.asarray(self.buffer, dtype=float)))


class StreamingBandpassFilter:
    """实现流式带通滤波器，用于提取目标工频磁信号分量。"""

    def __init__(self, sample_rate_hz: float, center_frequency_hz: float, half_width_hz: float, order: int = 2) -> None:
        """根据采样率和目标频率构建带通 SOS 滤波器。"""
        self.enabled = butter is not None and sosfilt is not None
        if not self.enabled:
            self.sos = np.zeros((0, 6), dtype=float)
            self.zi = np.zeros((0, 2, 3), dtype=float)
            return
        nyquist_hz = 0.5 * max(sample_rate_hz, 1e-6)
        low_hz = max(0.5, center_frequency_hz - half_width_hz)
        high_hz = min(nyquist_hz * 0.95, center_frequency_hz + half_width_hz)
        if low_hz >= high_hz:
            high_hz = min(nyquist_hz * 0.95, low_hz + max(1.0, 0.1 * center_frequency_hz))
        low_normalized = max(1e-4, low_hz / nyquist_hz)
        high_normalized = min(0.999, high_hz / nyquist_hz)
        self.sos = butter(order, [low_normalized, high_normalized], btype="bandpass", output="sos")
        self.zi = np.zeros((self.sos.shape[0], 2, 3), dtype=float)

    def update(self, vector_nt: np.ndarray) -> np.ndarray:
        """对三轴磁场向量执行逐轴流式滤波。"""
        if not self.enabled:
            return np.asarray(vector_nt, dtype=float)
        filtered = np.zeros(3, dtype=float)
        for axis_index in range(3):
            result, self.zi[:, :, axis_index] = sosfilt(
                self.sos,
                [float(vector_nt[axis_index])],
                zi=self.zi[:, :, axis_index],
            )
            filtered[axis_index] = float(result[0])
        return filtered


class RMSExtractor:
    """在滑动窗口内提取均方根幅值，用于形成跟踪强度。"""

    def __init__(self, sample_rate_hz: float, minimum_frequency_hz: float) -> None:
        """初始化 RMS 窗口大小，保证至少覆盖一个完整周期。"""
        min_window_s = 2.0 / max(minimum_frequency_hz, 1e-6)
        self.window_size_samples = max(3, int(np.ceil(sample_rate_hz * min_window_s)))
        self.buffer: Deque[float] = deque(maxlen=self.window_size_samples)

    def update(self, sample_value: float) -> float:
        """写入新样本并返回当前窗口 RMS 值。"""
        self.buffer.append(sample_value)
        if not self.buffer:
            return 0.0
        values = np.asarray(self.buffer, dtype=float)
        return float(np.sqrt(np.mean(values**2)))
