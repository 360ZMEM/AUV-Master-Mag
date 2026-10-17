"""Metric helpers shared by v2 experiment runners."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class SearchMetrics:
    detected: bool
    detection_time_s: Optional[float]
    travelled_distance_m: float
    false_alarm_count: int = 0


@dataclass(frozen=True)
class ErrorMetrics:
    count: int
    bias: float
    mae: float
    rmse: float
    q90_abs: float


def error_metrics(estimate: np.ndarray, truth: np.ndarray) -> ErrorMetrics:
    estimate = np.asarray(estimate, dtype=float)
    truth = np.asarray(truth, dtype=float)
    valid = np.isfinite(estimate) & np.isfinite(truth)
    errors = estimate[valid] - truth[valid]
    if not errors.size:
        return ErrorMetrics(0, float("nan"), float("nan"), float("nan"), float("nan"))
    return ErrorMetrics(
        count=int(errors.size), bias=float(np.mean(errors)), mae=float(np.mean(np.abs(errors))),
        rmse=float(np.sqrt(np.mean(errors ** 2))), q90_abs=float(np.percentile(np.abs(errors), 90.0)),
    )

