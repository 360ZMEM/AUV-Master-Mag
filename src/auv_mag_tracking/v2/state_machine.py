"""Explicit v2 mission phase transitions with hysteresis-friendly inputs."""

from __future__ import annotations

from dataclasses import dataclass

from .types import MissionPhase


@dataclass(frozen=True)
class PhaseDecision:
    phase: MissionPhase
    changed: bool
    reason: str


class V2StateMachine:
    def __init__(self) -> None:
        self.phase = MissionPhase.SEARCH

    def reset(self) -> None:
        self.phase = MissionPhase.SEARCH

    def update(
        self,
        *,
        detected: bool,
        scan_complete: bool = False,
        fwhm_valid: bool = False,
        tracking_confidence: float = 0.0,
        reacquire_requested: bool = False,
        lateral_valid: bool = False,
    ) -> PhaseDecision:
        previous = self.phase
        reason = "hold"
        if self.phase == MissionPhase.SEARCH and detected:
            self.phase, reason = MissionPhase.CONFIRM, "stable_magnetic_detection"
        elif self.phase == MissionPhase.CONFIRM and scan_complete:
            if fwhm_valid or lateral_valid:
                self.phase, reason = MissionPhase.TRACK, (
                    "valid_lateral_confirmation" if lateral_valid else "valid_fwhm_confirmation"
                )
            else:
                self.phase, reason = MissionPhase.REACQUIRE, "invalid_fwhm_confirmation"
        elif self.phase == MissionPhase.TRACK and (reacquire_requested or tracking_confidence < 0.2):
            self.phase, reason = MissionPhase.REACQUIRE, "tracking_confidence_lost"
        elif self.phase == MissionPhase.REACQUIRE and detected:
            self.phase, reason = MissionPhase.CONFIRM, "magnetic_reacquired"
        return PhaseDecision(self.phase, self.phase != previous, reason)
