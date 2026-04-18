"""Behavior tree for sonar-magnetic cable tracking."""

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class BehaviorMode(str, Enum):
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    TURN = "TURN"
    HOLD = "HOLD"
    LOST = "LOST"
    SPIRAL_SEARCH = "SPIRAL_SEARCH"


@dataclass
class BehaviorContext:
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
    deployment_mode: bool = False
    deployment_reacquire_required: bool = False


@dataclass
class BehaviorDecision:
    mode: BehaviorMode
    base_heading_deg: float
    speed_mps: float
    zigzag_width_m: float
    guidance_source: str
    force_centerline: bool = False


class BehaviorNode:
    def should_run(self, context: BehaviorContext) -> bool:
        raise NotImplementedError

    def run(self, context: BehaviorContext) -> BehaviorDecision:
        raise NotImplementedError


class SafeLockNode(BehaviorNode):
    def should_run(self, context: BehaviorContext) -> bool:
        return (
            context.safe_lock_active
            and context.fused_heading_deg is not None
            and context.has_detection_history
            and context.last_detection_age_s <= context.lost_timeout_s
        )

    def run(self, context: BehaviorContext) -> BehaviorDecision:
        return BehaviorDecision(
            mode=BehaviorMode.TURN,
            base_heading_deg=context.fused_heading_deg,
            speed_mps=0.9 * context.cruise_speed_mps,
            zigzag_width_m=0.0,
            guidance_source="SAFE_LOCK",
            force_centerline=True,
        )


class TurnNode(BehaviorNode):
    def should_run(self, context: BehaviorContext) -> bool:
        return context.peak_detected and context.fused_heading_deg is not None

    def run(self, context: BehaviorContext) -> BehaviorDecision:
        return BehaviorDecision(
            mode=BehaviorMode.TURN,
            base_heading_deg=context.fused_heading_deg,
            speed_mps=context.cruise_speed_mps,
            zigzag_width_m=context.zigzag_width_m,
            guidance_source="MAGNETIC_PEAK",
        )


class HoldNode(BehaviorNode):
    def should_run(self, context: BehaviorContext) -> bool:
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
        return (
            context.fused_heading_deg is not None
            and context.confidence >= context.high_confidence_threshold
            and not context.weak_signal_flag
        ) or memory_hold_ready

    def run(self, context: BehaviorContext) -> BehaviorDecision:
        return BehaviorDecision(
            mode=BehaviorMode.HOLD,
            base_heading_deg=context.fused_heading_deg,
            speed_mps=context.cruise_speed_mps,
            zigzag_width_m=0.55 * context.zigzag_width_m,
            guidance_source="FUSION_HOLD",
        )


class ApproachNode(BehaviorNode):
    def should_run(self, context: BehaviorContext) -> bool:
        return context.fused_heading_deg is not None and context.confidence >= context.low_confidence_threshold

    def run(self, context: BehaviorContext) -> BehaviorDecision:
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
    def should_run(self, context: BehaviorContext) -> bool:
        return (
            context.has_detection_history
            and context.last_detection_age_s > context.lost_timeout_s
            and context.sonar_status != "ONLINE"
            and context.blind_heading_deg is None
            and context.fused_heading_deg is None
        )

    def run(self, context: BehaviorContext) -> BehaviorDecision:
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
    def should_run(self, context: BehaviorContext) -> bool:
        return True

    def run(self, context: BehaviorContext) -> BehaviorDecision:
        if context.deployment_mode and not context.has_detection_history:
            return BehaviorDecision(
                mode=BehaviorMode.SPIRAL_SEARCH,
                base_heading_deg=context.nominal_heading_deg,
                speed_mps=context.search_speed_mps,
                zigzag_width_m=0.0,
                guidance_source="BOOTSTRAP_SPIRAL",
                force_centerline=True,
            )
        if context.deployment_mode and context.deployment_heading_confidence < context.high_confidence_threshold:
            return BehaviorDecision(
                mode=BehaviorMode.SPIRAL_SEARCH,
                base_heading_deg=context.nominal_heading_deg,
                speed_mps=context.search_speed_mps,
                zigzag_width_m=0.0,
                guidance_source="DEPLOYMENT_SPIRAL",
                force_centerline=True,
            )
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
    def __init__(self) -> None:
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
        within_recovery_window = context.time_s - self.last_turn_time_s <= context.spiral_entry_window_s
        return (
            self.turn_recovery_active
            and within_recovery_window
            and context.sonar_status != "ONLINE"
            and self.consecutive_miss_counter >= context.consecutive_miss_threshold
        )

    def _update_turn_recovery(self, context: BehaviorContext) -> None:
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
        if decision.mode == BehaviorMode.TURN:
            self.turn_recovery_active = True
            self.consecutive_miss_counter = 0
            self.last_turn_time_s = context.time_s
        elif decision.mode != BehaviorMode.SPIRAL_SEARCH and context.peak_detected:
            self.turn_recovery_active = False
            self.consecutive_miss_counter = 0
        return decision

    def evaluate(self, context: BehaviorContext) -> BehaviorDecision:
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