"""Behavior tree for sonar-magnetic cable tracking."""

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class BehaviorMode(str, Enum):
    """定义行为树输出的运行模式。"""

    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    TURN = "TURN"
    HOLD = "HOLD"
    LOST = "LOST"
    SPIRAL_SEARCH = "SPIRAL_SEARCH"


@dataclass
class BehaviorContext:
    """行为树评估所需的完整上下文输入。"""

    time_s: float
    nominal_heading_deg: float
    intercept_heading_deg: float
    nominal_distance_m: float
    confidence: float
    has_detection_history: bool
    last_detection_age_s: float
    fused_heading_deg: Optional[float]
    blind_heading_deg: Optional[float]
    sonar_status: str
    weak_signal_flag: bool
    safe_lock_active: bool
    peak_detected: bool
    zigzag_width_m: float
    high_confidence_threshold: float
    low_confidence_threshold: float
    lost_timeout_s: float
    guidance_memory_timeout_s: float
    consecutive_miss_threshold: int
    spiral_entry_window_s: float
    search_speed_mps: float
    cruise_speed_mps: float
    guidance_source: str
    fit_residual_m: float
    deployment_heading_confidence: float = 0.0
    tracking_maturity: float = 0.0
    safe_lock_criterion_b_active: bool = False
    deployment_hold_maturity_threshold: float = 0.8
    deployment_lost_timeout_high_maturity_multiplier: float = 1.5
    deployment_mode: bool = False
    deployment_reacquire_required: bool = False


@dataclass
class BehaviorDecision:
    """行为树评估后的控制决策结果。"""

    mode: BehaviorMode
    base_heading_deg: float
    speed_mps: float
    zigzag_width_m: float
    guidance_source: str
    force_centerline: bool = False


class BehaviorNode:
    """行为树节点基类，定义统一的运行接口。"""

    def should_run(self, context: BehaviorContext) -> bool:
        """判断当前节点是否应接管决策。"""
        raise NotImplementedError

    def run(self, context: BehaviorContext) -> BehaviorDecision:
        """生成当前节点对应的行为决策。"""
        raise NotImplementedError


class SafeLockNode(BehaviorNode):
    """在安全锁条件满足时返回中心线保持决策。"""

    def should_run(self, context: BehaviorContext) -> bool:
        """判断是否应进入安全锁模式。"""
        return (
            context.safe_lock_active
            and context.fused_heading_deg is not None
            and context.has_detection_history
            and context.last_detection_age_s <= context.lost_timeout_s
        )

    def run(self, context: BehaviorContext) -> BehaviorDecision:
        """构造安全锁模式下的低速保持决策。"""
        return BehaviorDecision(
            mode=BehaviorMode.TURN,
            base_heading_deg=context.fused_heading_deg,
            speed_mps=0.9 * context.cruise_speed_mps,
            zigzag_width_m=0.0,
            guidance_source="SAFE_LOCK",
            force_centerline=True,
        )


class TurnNode(BehaviorNode):
    """在峰值跨越时返回转弯决策。"""

    def should_run(self, context: BehaviorContext) -> bool:
        """判断是否处于需要转弯的峰值事件窗口。"""
        return context.peak_detected and context.fused_heading_deg is not None

    def run(self, context: BehaviorContext) -> BehaviorDecision:
        """构造峰值跨越阶段的转弯决策。"""
        return BehaviorDecision(
            mode=BehaviorMode.TURN,
            base_heading_deg=context.fused_heading_deg,
            speed_mps=context.cruise_speed_mps,
            zigzag_width_m=context.zigzag_width_m,
            guidance_source="MAGNETIC_PEAK",
        )


class HoldNode(BehaviorNode):
    """在高置信且跟踪成熟时保持当前航向。"""

    def should_run(self, context: BehaviorContext) -> bool:
        """判断是否满足保持决策的置信度与成熟度要求。"""
        maturity_ready = context.tracking_maturity >= context.deployment_hold_maturity_threshold
        memory_hold_ready = (
            context.fused_heading_deg is not None
            and context.guidance_source in {"MEMORY", "BLIND", "BLIND_RECOVERY"}
            and context.last_detection_age_s >= context.guidance_memory_timeout_s
            and context.confidence >= context.low_confidence_threshold
            and context.fit_residual_m <= 3.5
        )
        if context.deployment_mode and context.guidance_source == "MEMORY":
            memory_hold_ready = (
                memory_hold_ready
                and context.deployment_heading_confidence >= context.high_confidence_threshold
                and context.fit_residual_m <= 1.8
            )
        base_hold_ready = (
            context.fused_heading_deg is not None
            and context.confidence >= context.high_confidence_threshold
            and not context.weak_signal_flag
        ) or memory_hold_ready
        return base_hold_ready and maturity_ready and not context.safe_lock_criterion_b_active

    def run(self, context: BehaviorContext) -> BehaviorDecision:
        """构造保持模式的控制决策。"""
        return BehaviorDecision(
            mode=BehaviorMode.HOLD,
            base_heading_deg=context.fused_heading_deg,
            speed_mps=context.cruise_speed_mps,
            zigzag_width_m=0.55 * context.zigzag_width_m,
            guidance_source="FUSION_HOLD",
        )


class ApproachNode(BehaviorNode):
    """在接近电缆时输出靠近决策。"""

    def should_run(self, context: BehaviorContext) -> bool:
        """判断是否可以进入接近阶段。"""
        return context.fused_heading_deg is not None and context.confidence >= context.low_confidence_threshold

    def run(self, context: BehaviorContext) -> BehaviorDecision:
        """构造接近阶段的控制决策。"""
        source = "SONAR" if context.sonar_status == "ONLINE" and context.weak_signal_flag else "MAGNETIC"
        return BehaviorDecision(
            mode=BehaviorMode.APPROACH,
            base_heading_deg=context.fused_heading_deg,
            speed_mps=0.95 * context.cruise_speed_mps,
            zigzag_width_m=context.zigzag_width_m,
            guidance_source=source,
            force_centerline=source == "SONAR",
        )


class LostNode(BehaviorNode):
    """在长时间无有效检测时返回失锁恢复决策。"""

    def should_run(self, context: BehaviorContext) -> bool:
        """判断是否已经超时且需要进入失锁恢复。"""
        effective_lost_timeout_s = context.lost_timeout_s
        if context.deployment_mode and context.tracking_maturity >= context.deployment_hold_maturity_threshold:
            effective_lost_timeout_s *= context.deployment_lost_timeout_high_maturity_multiplier
        return (
            context.has_detection_history
            and context.last_detection_age_s > effective_lost_timeout_s
            and context.sonar_status != "ONLINE"
            and context.blind_heading_deg is None
            and context.fused_heading_deg is None
        )

    def run(self, context: BehaviorContext) -> BehaviorDecision:
        """构造失锁或螺旋搜索恢复决策。"""
        if context.deployment_mode and context.deployment_reacquire_required:
            return BehaviorDecision(
                mode=BehaviorMode.SPIRAL_SEARCH,
                base_heading_deg=context.nominal_heading_deg,
                speed_mps=context.search_speed_mps,
                zigzag_width_m=0.0,
                guidance_source="REACQUIRE_SPIRAL",
                force_centerline=True,
            )
        if context.deployment_mode and context.fused_heading_deg is None and context.blind_heading_deg is None:
            return BehaviorDecision(
                mode=BehaviorMode.SPIRAL_SEARCH,
                base_heading_deg=context.nominal_heading_deg,
                speed_mps=context.search_speed_mps,
                zigzag_width_m=0.0,
                guidance_source="RECOVERY_SPIRAL",
                force_centerline=True,
            )
        return BehaviorDecision(
            mode=BehaviorMode.LOST,
            base_heading_deg=context.blind_heading_deg if context.blind_heading_deg is not None else context.nominal_heading_deg,
            speed_mps=context.search_speed_mps,
            zigzag_width_m=context.zigzag_width_m,
            guidance_source="LOST_RECOVERY",
        )


class SearchNode(BehaviorNode):
    """默认兜底节点，负责生成搜索态决策。"""

    def should_run(self, context: BehaviorContext) -> bool:
        """搜索节点始终可运行，作为行为树兜底。"""
        return True

    def run(self, context: BehaviorContext) -> BehaviorDecision:
        """根据当前先验和历史状态生成搜索或螺旋恢复决策。"""
        if context.deployment_mode and context.tracking_maturity >= context.deployment_hold_maturity_threshold and context.blind_heading_deg is not None:
            return BehaviorDecision(
                mode=BehaviorMode.APPROACH,
                base_heading_deg=context.blind_heading_deg,
                speed_mps=0.95 * context.cruise_speed_mps,
                zigzag_width_m=0.35 * context.zigzag_width_m,
                guidance_source="BLIND_INERTIA",
                force_centerline=True,
            )
        if context.deployment_mode and not context.has_detection_history:
            return BehaviorDecision(
                mode=BehaviorMode.SPIRAL_SEARCH,
                base_heading_deg=context.nominal_heading_deg,
                speed_mps=context.search_speed_mps,
                zigzag_width_m=0.0,
                guidance_source="BOOTSTRAP_SPIRAL",
                force_centerline=True,
            )
        if context.deployment_mode and context.deployment_reacquire_required and context.tracking_maturity < context.deployment_hold_maturity_threshold:
            return BehaviorDecision(
                mode=BehaviorMode.SPIRAL_SEARCH,
                base_heading_deg=context.nominal_heading_deg,
                speed_mps=context.search_speed_mps,
                zigzag_width_m=0.0,
                guidance_source="REACQUIRE_SPIRAL",
                force_centerline=True,
            )
        if context.deployment_mode and context.deployment_heading_confidence < context.high_confidence_threshold and not (
            context.tracking_maturity >= context.deployment_hold_maturity_threshold and context.blind_heading_deg is not None
        ):
            return BehaviorDecision(
                mode=BehaviorMode.SPIRAL_SEARCH,
                base_heading_deg=context.nominal_heading_deg,
                speed_mps=context.search_speed_mps,
                zigzag_width_m=0.0,
                guidance_source="DEPLOYMENT_SPIRAL",
                force_centerline=True,
            )
        if context.deployment_mode and context.fused_heading_deg is None and context.blind_heading_deg is None:
            return BehaviorDecision(
                mode=BehaviorMode.SPIRAL_SEARCH,
                base_heading_deg=context.nominal_heading_deg,
                speed_mps=context.search_speed_mps,
                zigzag_width_m=0.0,
                guidance_source="RECOVERY_SPIRAL",
                force_centerline=True,
            )
        if context.blind_heading_deg is not None:
            return BehaviorDecision(
                mode=BehaviorMode.APPROACH,
                base_heading_deg=context.blind_heading_deg,
                speed_mps=0.95 * context.cruise_speed_mps,
                zigzag_width_m=0.35 * context.zigzag_width_m,
                guidance_source="BLIND_RECOVERY",
                force_centerline=True,
            )
        heading_deg = context.intercept_heading_deg if context.nominal_distance_m > 3.0 else context.nominal_heading_deg
        if context.blind_heading_deg is not None and context.nominal_distance_m <= 3.0:
            heading_deg = context.blind_heading_deg
        return BehaviorDecision(
            mode=BehaviorMode.SEARCH,
            base_heading_deg=heading_deg,
            speed_mps=context.search_speed_mps,
            zigzag_width_m=context.zigzag_width_m,
            guidance_source="BLIND" if context.blind_heading_deg is not None else "NOMINAL",
        )


class BehaviorTree:
    """按优先级组合多个行为节点并生成最终决策。"""

    def __init__(self) -> None:
        """初始化行为树内部状态和节点顺序。"""
        self.consecutive_miss_counter = 0
        self.turn_recovery_active = False
        self.last_turn_time_s = -1e9
        self.nodes = [
            SafeLockNode(),
            TurnNode(),
            HoldNode(),
            ApproachNode(),
            LostNode(),
            SearchNode(),
        ]

    def _should_force_spiral_search(self, context: BehaviorContext) -> bool:
        """判断当前是否应强制切换到螺旋搜索。"""
        within_recovery_window = context.time_s - self.last_turn_time_s <= context.spiral_entry_window_s
        return (
            self.turn_recovery_active
            and within_recovery_window
            and context.sonar_status != "ONLINE"
            and self.consecutive_miss_counter >= context.consecutive_miss_threshold
        )

    def _update_turn_recovery(self, context: BehaviorContext) -> None:
        """更新转弯后恢复状态与连续漏检计数。"""
        if context.peak_detected:
            self.consecutive_miss_counter = 0
            self.turn_recovery_active = False
            return

        if not self.turn_recovery_active:
            return

        if context.time_s - self.last_turn_time_s > context.spiral_entry_window_s:
            self.consecutive_miss_counter = 0
            self.turn_recovery_active = False
            return

        self.consecutive_miss_counter += 1

    def _finalize_decision(self, decision: BehaviorDecision, context: BehaviorContext) -> BehaviorDecision:
        """在返回决策前同步行为树内部恢复状态。"""
        if decision.mode == BehaviorMode.TURN:
            self.turn_recovery_active = True
            self.consecutive_miss_counter = 0
            self.last_turn_time_s = context.time_s
        elif decision.mode != BehaviorMode.SPIRAL_SEARCH and context.peak_detected:
            self.turn_recovery_active = False
            self.consecutive_miss_counter = 0
        return decision

    def evaluate(self, context: BehaviorContext) -> BehaviorDecision:
        """按节点优先级评估上下文并返回首个命中的行为决策。"""
        self._update_turn_recovery(context)
        if context.deployment_mode and context.deployment_reacquire_required:
            return BehaviorDecision(
                mode=BehaviorMode.SPIRAL_SEARCH,
                base_heading_deg=context.nominal_heading_deg,
                speed_mps=context.search_speed_mps,
                zigzag_width_m=0.0,
                guidance_source="REACQUIRE_SPIRAL",
                force_centerline=True,
            )
        if self._should_force_spiral_search(context):
            return BehaviorDecision(
                mode=BehaviorMode.SPIRAL_SEARCH,
                base_heading_deg=context.blind_heading_deg if context.blind_heading_deg is not None else context.nominal_heading_deg,
                speed_mps=context.search_speed_mps,
                zigzag_width_m=context.zigzag_width_m,
                guidance_source="SPIRAL_RECOVERY",
            )
        for node in self.nodes:
            if node.should_run(context):
                return self._finalize_decision(node.run(context), context)
        return self._finalize_decision(BehaviorDecision(
            mode=BehaviorMode.SEARCH,
            base_heading_deg=context.nominal_heading_deg,
            speed_mps=context.search_speed_mps,
            zigzag_width_m=context.zigzag_width_m,
            guidance_source="DEFAULT",
        ), context)