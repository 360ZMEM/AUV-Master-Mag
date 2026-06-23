"""Three-state mission FSM (+ emergency terminal) that replaces ``behavior_tree``.

The mission manager is the single strategic layer above the kinematic controller.
It consumes the perception contract and a sparse prior route, and emits a
:class:`MissionDecision` describing *what* the vehicle should do (sweep, align, or
track).  All state-transition thresholds live in :class:`MissionThresholds` so they
can be surfaced in configuration; the manager itself holds only the minimal counters
needed to debounce transitions.

States (per Spec v0.1 §3):

    SEARCH_ZIGZAG  -> sweep across the prior corridor until the cable is sensed
    LOCK_ALIGN     -> slow down, let the line fit converge on the crossing
    TRACK_ACTIVE   -> sonar/magnetic co-operative steady-state follow
    EMERGENCY_SURFACE (terminal) -> dual-blind; record only (speed -> 0 + flag)

LOCK_ALIGN -> TRACK_ACTIVE uses a PCA covariance proxy for the (absent) EKF P_yy:
the smaller eigenvalue of the weighted scatter matrix is the perpendicular spread of
the fitted line, so ``eig_perp < cov_perp_converged_m2`` signals a converged fit.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional

import numpy as np


class MissionState(str, Enum):
    """任务层有限状态机的状态定义。"""

    SEARCH_ZIGZAG = "search"
    LOCK_ALIGN = "align"
    TRACK_ACTIVE = "track"
    EMERGENCY_SURFACE = "emergency"


@dataclass
class MissionThresholds:
    """任务层所有可调阈值（集中配置，便于人工修改）。"""

    mag_lock_threshold_nT: float = 50.0
    sonar_confidence_threshold: float = 0.6
    lock_streak_required: int = 5
    loss_streak_required: int = 3
    align_speed_factor: float = 0.5
    # EKF P_yy 代理：PCA 拟合协方差较小特征值（垂直方向散布）。
    cov_perp_converged_m2: float = 1.0
    yaw_err_converged_deg: float = 5.0
    track_confidence_required: float = 0.65
    track_streak_required: int = 5
    system_confidence_floor: float = 0.1
    emergency_hold_s: float = 5.0


@dataclass
class MissionInput:
    """任务层每一步消费的感知/位姿摘要（从 PerceptionState + Pose 投影而来）。"""

    time_s: float
    mag_strength_nT: float
    sonar_confidence: float
    confidence: float
    fused_heading_deg: Optional[float]
    yaw_error_deg: Optional[float]
    fit_covariance_xy_m2: Optional[np.ndarray]
    peak_detected: bool


@dataclass
class MissionDecision:
    """任务层输出给控制器的策略决策。"""

    state: MissionState
    speed_factor: float
    guidance_source: str
    emergency_flag: bool = False


def _perpendicular_spread_m2(covariance_xy_m2: Optional[np.ndarray]) -> Optional[float]:
    """从加权散布协方差中提取垂直方向散布（较小特征值），作为 EKF P_yy 代理。"""
    if covariance_xy_m2 is None:
        return None
    covariance = np.asarray(covariance_xy_m2, dtype=float)
    if covariance.shape != (2, 2) or not np.all(np.isfinite(covariance)):
        return None
    eigenvalues = np.linalg.eigvalsh(covariance)
    return float(np.min(eigenvalues))


class MissionManager:
    """三态任务 FSM：SEARCH_ZIGZAG → LOCK_ALIGN → TRACK_ACTIVE（+ EMERGENCY 终态）。"""

    def __init__(self, thresholds: Optional[MissionThresholds] = None) -> None:
        """以给定阈值初始化任务管理器及其去抖计数器。"""
        self.thresholds = thresholds or MissionThresholds()
        self.state = MissionState.SEARCH_ZIGZAG
        self._lock_streak = 0
        self._loss_streak = 0
        self._track_streak = 0
        self._low_confidence_since_s: Optional[float] = None

    def reset(self) -> None:
        """复位 FSM 到初始搜索态并清空计数器。"""
        self.state = MissionState.SEARCH_ZIGZAG
        self._lock_streak = 0
        self._loss_streak = 0
        self._track_streak = 0
        self._low_confidence_since_s = None

    def _signal_present(self, mission_input: MissionInput) -> bool:
        """判断当前是否检测到电缆信号（磁强或声呐任一达标）。"""
        return (
            mission_input.mag_strength_nT >= self.thresholds.mag_lock_threshold_nT
            or mission_input.sonar_confidence >= self.thresholds.sonar_confidence_threshold
        )

    def _fit_converged(self, mission_input: MissionInput) -> bool:
        """判断拟合是否收敛（垂直散布与偏航误差均达标且置信度足够）。"""
        spread_m2 = _perpendicular_spread_m2(mission_input.fit_covariance_xy_m2)
        if spread_m2 is None or spread_m2 >= self.thresholds.cov_perp_converged_m2:
            return False
        if mission_input.yaw_error_deg is None:
            return False
        if abs(mission_input.yaw_error_deg) >= self.thresholds.yaw_err_converged_deg:
            return False
        return mission_input.confidence >= self.thresholds.track_confidence_required

    def update(self, mission_input: MissionInput) -> MissionDecision:
        """根据感知输入推进 FSM 一步并返回任务决策。"""
        thresholds = self.thresholds

        if self.state == MissionState.EMERGENCY_SURFACE:
            return self._decision()

        signal_present = self._signal_present(mission_input)
        self._lock_streak = self._lock_streak + 1 if signal_present else 0
        self._loss_streak = 0 if signal_present else self._loss_streak + 1

        if self.state == MissionState.SEARCH_ZIGZAG:
            if self._lock_streak >= thresholds.lock_streak_required:
                self._enter(MissionState.LOCK_ALIGN)

        elif self.state == MissionState.LOCK_ALIGN:
            if self._loss_streak >= thresholds.loss_streak_required:
                self._enter(MissionState.SEARCH_ZIGZAG)
            elif self._fit_converged(mission_input):
                self._track_streak += 1
                if self._track_streak >= thresholds.track_streak_required:
                    self._enter(MissionState.TRACK_ACTIVE)
            else:
                self._track_streak = 0

        elif self.state == MissionState.TRACK_ACTIVE:
            if mission_input.confidence < thresholds.system_confidence_floor:
                if self._low_confidence_since_s is None:
                    self._low_confidence_since_s = mission_input.time_s
                elif mission_input.time_s - self._low_confidence_since_s >= thresholds.emergency_hold_s:
                    self._enter(MissionState.EMERGENCY_SURFACE)
            else:
                self._low_confidence_since_s = None
                if not signal_present and self._loss_streak >= thresholds.loss_streak_required:
                    self._enter(MissionState.LOCK_ALIGN)

        return self._decision(mission_input)

    def _enter(self, state: MissionState) -> None:
        """切换到目标状态并清空与转移相关的计数器。"""
        if state == self.state:
            return
        self.state = state
        self._track_streak = 0
        if state != MissionState.TRACK_ACTIVE:
            self._low_confidence_since_s = None

    def _decision(self, mission_input: Optional[MissionInput] = None) -> MissionDecision:
        """根据当前状态构造任务决策。"""
        if self.state == MissionState.EMERGENCY_SURFACE:
            return MissionDecision(
                state=self.state,
                speed_factor=0.0,
                guidance_source="EMERGENCY",
                emergency_flag=True,
            )
        if self.state == MissionState.SEARCH_ZIGZAG:
            return MissionDecision(self.state, 1.0, "SEARCH")
        if self.state == MissionState.LOCK_ALIGN:
            peak = bool(mission_input and mission_input.peak_detected)
            return MissionDecision(
                self.state,
                self.thresholds.align_speed_factor,
                "MAGNETIC_PEAK" if peak else "MAGNETIC",
            )
        # TRACK_ACTIVE
        peak = bool(mission_input and mission_input.peak_detected)
        return MissionDecision(self.state, 1.0, "MAGNETIC_PEAK" if peak else "MAGNETIC")
