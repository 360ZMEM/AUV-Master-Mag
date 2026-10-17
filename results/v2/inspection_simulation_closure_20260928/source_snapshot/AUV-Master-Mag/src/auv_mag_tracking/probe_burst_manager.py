"""State machine for bounded magnetic probe bursts with route recovery."""

from dataclasses import dataclass
from enum import Enum


class ProbeBurstState(Enum):
    """High-level control phases for controlled aggressive probing."""

    IDLE_BASELINE = "IDLE_BASELINE"
    BURST_COLLECT_EVIDENCE = "BURST_COLLECT_EVIDENCE"
    RECOVER_ROUTE = "RECOVER_ROUTE"
    COOLDOWN = "COOLDOWN"


@dataclass
class ProbeBurstThresholds:
    """State transition thresholds for :class:`ProbeBurstManager`."""

    idle_min_duration_s: float = 30.0
    entry_min_route_delta_m: float = 3.0
    entry_max_abs_cross_track_m: float = 80.0
    burst_min_duration_s: float = 4.0
    burst_max_duration_s: float = 12.0
    burst_target_evidence_count: int = 1
    recovery_min_duration_s: float = 20.0
    recovery_target_route_delta_m: float = 8.0
    recovery_max_abs_cross_track_m: float = 3.0
    recovery_timeout_s: float = 120.0
    cooldown_duration_s: float = 120.0


@dataclass
class ProbeBurstInput:
    """Per-frame observations needed by the burst manager."""

    time_s: float
    route_progress_m: float
    abs_cross_track_m: float
    evidence_available: bool
    enabled: bool
    control_allowed: bool = True


@dataclass
class ProbeBurstDecision:
    """Current manager state and the requested controller action."""

    state: ProbeBurstState
    burst_active: bool
    recovery_active: bool
    reason: str
    state_elapsed_s: float
    route_delta_in_state_m: float
    evidence_count_in_state: int
    control_allowed: bool
    entry_abs_cross_track_m: float


class ProbeBurstManager:
    """Explicit state machine for controlled probe bursts.

    The manager intentionally owns only strategy-level phase decisions. Heading
    geometry stays in ``ZigZagController`` so the existing p36 controller remains
    the fallback behavior whenever the manager returns IDLE or COOLDOWN.
    """

    def __init__(self, thresholds: ProbeBurstThresholds | None = None) -> None:
        self.thresholds = thresholds or ProbeBurstThresholds()
        self.state = ProbeBurstState.IDLE_BASELINE
        self.state_start_time_s = 0.0
        self.state_start_route_progress_m = 0.0
        self.idle_accumulated_enabled_s = 0.0
        self.idle_accumulated_route_delta_m = 0.0
        self.last_update_time_s = 0.0
        self.last_route_progress_m = 0.0
        self.phase_accumulated_control_s = 0.0
        self.phase_accumulated_route_delta_m = 0.0
        self.evidence_count_in_state = 0
        self.last_evidence_available = False
        self.entry_abs_cross_track_m = float("nan")

    def reset(self, time_s: float = 0.0, route_progress_m: float = 0.0) -> None:
        self.state = ProbeBurstState.IDLE_BASELINE
        self.state_start_time_s = time_s
        self.state_start_route_progress_m = route_progress_m
        self.idle_accumulated_enabled_s = 0.0
        self.idle_accumulated_route_delta_m = 0.0
        self.last_update_time_s = time_s
        self.last_route_progress_m = route_progress_m
        self.phase_accumulated_control_s = 0.0
        self.phase_accumulated_route_delta_m = 0.0
        self.evidence_count_in_state = 0
        self.last_evidence_available = False
        self.entry_abs_cross_track_m = float("nan")

    def update(self, obs: ProbeBurstInput) -> ProbeBurstDecision:
        if not obs.enabled:
            if self.state != ProbeBurstState.IDLE_BASELINE:
                self.reset(obs.time_s, obs.route_progress_m)
            else:
                self.last_update_time_s = obs.time_s
                self.last_route_progress_m = obs.route_progress_m
                self.last_evidence_available = False
            return self._decision(obs, "disabled")

        dt_s = max(obs.time_s - self.last_update_time_s, 0.0)
        route_step_delta_m = obs.route_progress_m - self.last_route_progress_m
        self.last_update_time_s = obs.time_s
        self.last_route_progress_m = obs.route_progress_m
        if obs.control_allowed and self.state != ProbeBurstState.IDLE_BASELINE:
            self.phase_accumulated_control_s += dt_s
            self.phase_accumulated_route_delta_m += route_step_delta_m

        if obs.control_allowed and obs.evidence_available and not self.last_evidence_available:
            self.evidence_count_in_state += 1
        self.last_evidence_available = obs.evidence_available

        elapsed_s = obs.time_s - self.state_start_time_s
        route_delta_m = obs.route_progress_m - self.state_start_route_progress_m
        reason = "hold"

        if self.state == ProbeBurstState.IDLE_BASELINE:
            self.idle_accumulated_enabled_s += dt_s
            self.idle_accumulated_route_delta_m += max(route_step_delta_m, 0.0)
            if (
                self.idle_accumulated_enabled_s >= self.thresholds.idle_min_duration_s
                and self.idle_accumulated_route_delta_m >= self.thresholds.entry_min_route_delta_m
                and obs.abs_cross_track_m <= self.thresholds.entry_max_abs_cross_track_m
            ):
                self._transition(
                    ProbeBurstState.BURST_COLLECT_EVIDENCE,
                    obs.time_s,
                    obs.route_progress_m,
                    entry_abs_cross_track_m=obs.abs_cross_track_m,
                )
                reason = "enter_burst"
        elif self.state == ProbeBurstState.BURST_COLLECT_EVIDENCE:
            evidence_target_met = (
                self.evidence_count_in_state >= self.thresholds.burst_target_evidence_count
                and self.phase_accumulated_control_s >= self.thresholds.burst_min_duration_s
            )
            if evidence_target_met:
                self._transition(ProbeBurstState.RECOVER_ROUTE, obs.time_s, obs.route_progress_m)
                reason = "evidence_target"
            elif self.phase_accumulated_control_s >= self.thresholds.burst_max_duration_s:
                self._transition(ProbeBurstState.RECOVER_ROUTE, obs.time_s, obs.route_progress_m)
                reason = "burst_timeout"
        elif self.state == ProbeBurstState.RECOVER_ROUTE:
            recovered = (
                self.phase_accumulated_control_s >= self.thresholds.recovery_min_duration_s
                and self.phase_accumulated_route_delta_m >= self.thresholds.recovery_target_route_delta_m
                and obs.abs_cross_track_m <= self.thresholds.recovery_max_abs_cross_track_m
            )
            if recovered:
                self._transition(ProbeBurstState.COOLDOWN, obs.time_s, obs.route_progress_m)
                reason = "recovery_complete"
            elif self.phase_accumulated_control_s >= self.thresholds.recovery_timeout_s:
                self._transition(ProbeBurstState.COOLDOWN, obs.time_s, obs.route_progress_m)
                reason = "recovery_timeout"
        elif self.state == ProbeBurstState.COOLDOWN:
            if elapsed_s >= self.thresholds.cooldown_duration_s:
                self._transition(ProbeBurstState.IDLE_BASELINE, obs.time_s, obs.route_progress_m)
                reason = "cooldown_complete"

        return self._decision(obs, reason)

    def _transition(
        self,
        state: ProbeBurstState,
        time_s: float,
        route_progress_m: float,
        entry_abs_cross_track_m: float | None = None,
    ) -> None:
        self.state = state
        self.state_start_time_s = time_s
        self.state_start_route_progress_m = route_progress_m
        if entry_abs_cross_track_m is not None:
            self.entry_abs_cross_track_m = entry_abs_cross_track_m
        elif state == ProbeBurstState.IDLE_BASELINE:
            self.entry_abs_cross_track_m = float("nan")
        self.idle_accumulated_enabled_s = 0.0
        self.idle_accumulated_route_delta_m = 0.0
        self.last_update_time_s = time_s
        self.last_route_progress_m = route_progress_m
        self.phase_accumulated_control_s = 0.0
        self.phase_accumulated_route_delta_m = 0.0
        self.evidence_count_in_state = 0
        self.last_evidence_available = False

    def _decision(self, obs: ProbeBurstInput, reason: str) -> ProbeBurstDecision:
        if self.state == ProbeBurstState.IDLE_BASELINE:
            state_elapsed_s = self.idle_accumulated_enabled_s
            route_delta_in_state_m = self.idle_accumulated_route_delta_m
        elif self.state in {ProbeBurstState.BURST_COLLECT_EVIDENCE, ProbeBurstState.RECOVER_ROUTE}:
            state_elapsed_s = self.phase_accumulated_control_s
            route_delta_in_state_m = self.phase_accumulated_route_delta_m
        else:
            state_elapsed_s = obs.time_s - self.state_start_time_s
            route_delta_in_state_m = obs.route_progress_m - self.state_start_route_progress_m
        entry_abs_cross_track_m = (
            obs.abs_cross_track_m
            if self.state == ProbeBurstState.IDLE_BASELINE
            else self.entry_abs_cross_track_m
        )
        return ProbeBurstDecision(
            state=self.state,
            burst_active=self.state == ProbeBurstState.BURST_COLLECT_EVIDENCE and obs.control_allowed,
            recovery_active=self.state == ProbeBurstState.RECOVER_ROUTE and obs.control_allowed,
            reason=reason,
            state_elapsed_s=state_elapsed_s,
            route_delta_in_state_m=route_delta_in_state_m,
            evidence_count_in_state=self.evidence_count_in_state,
            control_allowed=obs.control_allowed,
            entry_abs_cross_track_m=entry_abs_cross_track_m,
        )
