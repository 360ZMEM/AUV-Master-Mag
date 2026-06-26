import sys
import unittest
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.probe_burst_manager import (
    ProbeBurstInput,
    ProbeBurstManager,
    ProbeBurstState,
    ProbeBurstThresholds,
)


class ProbeBurstManagerTest(unittest.TestCase):
    def _manager(self) -> ProbeBurstManager:
        return ProbeBurstManager(
            ProbeBurstThresholds(
                idle_min_duration_s=10.0,
                entry_min_route_delta_m=5.0,
                entry_max_abs_cross_track_m=3.0,
                burst_min_duration_s=2.0,
                burst_max_duration_s=6.0,
                burst_target_evidence_count=1,
                recovery_min_duration_s=5.0,
                recovery_target_route_delta_m=4.0,
                recovery_max_abs_cross_track_m=2.0,
                recovery_timeout_s=20.0,
                cooldown_duration_s=8.0,
            )
        )

    def _input(
        self,
        *,
        time_s: float,
        route_progress_m: float,
        abs_cross_track_m: float = 1.0,
        evidence_available: bool = False,
        enabled: bool = True,
    ) -> ProbeBurstInput:
        return ProbeBurstInput(
            time_s=time_s,
            route_progress_m=route_progress_m,
            abs_cross_track_m=abs_cross_track_m,
            evidence_available=evidence_available,
            enabled=enabled,
        )

    def test_idle_enters_burst_only_after_route_is_stable(self) -> None:
        manager = self._manager()

        early = manager.update(self._input(time_s=9.0, route_progress_m=10.0))
        self.assertEqual(early.state, ProbeBurstState.IDLE_BASELINE)
        self.assertFalse(early.burst_active)

        far_from_route = manager.update(self._input(time_s=10.0, route_progress_m=10.0, abs_cross_track_m=4.0))
        self.assertEqual(far_from_route.state, ProbeBurstState.IDLE_BASELINE)

        ready = manager.update(self._input(time_s=11.0, route_progress_m=10.0, abs_cross_track_m=2.0))
        self.assertEqual(ready.state, ProbeBurstState.BURST_COLLECT_EVIDENCE)
        self.assertTrue(ready.burst_active)
        self.assertEqual(ready.reason, "enter_burst")

    def test_burst_exits_to_recovery_after_evidence_and_min_duration(self) -> None:
        manager = self._manager()
        manager.update(self._input(time_s=11.0, route_progress_m=10.0))

        still_burst = manager.update(self._input(time_s=12.0, route_progress_m=11.0, evidence_available=True))
        self.assertEqual(still_burst.state, ProbeBurstState.BURST_COLLECT_EVIDENCE)

        recovery = manager.update(self._input(time_s=13.0, route_progress_m=12.0))
        self.assertEqual(recovery.state, ProbeBurstState.RECOVER_ROUTE)
        self.assertTrue(recovery.recovery_active)
        self.assertEqual(recovery.reason, "evidence_target")

    def test_burst_exits_to_recovery_on_timeout_without_evidence(self) -> None:
        manager = self._manager()
        manager.update(self._input(time_s=11.0, route_progress_m=10.0))

        recovery = manager.update(self._input(time_s=17.0, route_progress_m=12.0))
        self.assertEqual(recovery.state, ProbeBurstState.RECOVER_ROUTE)
        self.assertEqual(recovery.reason, "burst_timeout")

    def test_recovery_requires_route_delta_and_cross_track_before_cooldown(self) -> None:
        manager = self._manager()
        manager.update(self._input(time_s=11.0, route_progress_m=10.0))
        manager.update(self._input(time_s=13.0, route_progress_m=11.0, evidence_available=True))

        not_recovered = manager.update(self._input(time_s=18.0, route_progress_m=13.0, abs_cross_track_m=2.5))
        self.assertEqual(not_recovered.state, ProbeBurstState.RECOVER_ROUTE)

        cooldown = manager.update(self._input(time_s=19.0, route_progress_m=16.0, abs_cross_track_m=1.5))
        self.assertEqual(cooldown.state, ProbeBurstState.COOLDOWN)
        self.assertEqual(cooldown.reason, "recovery_complete")

        idle = manager.update(self._input(time_s=27.0, route_progress_m=20.0, abs_cross_track_m=1.0))
        self.assertEqual(idle.state, ProbeBurstState.IDLE_BASELINE)
        self.assertEqual(idle.reason, "cooldown_complete")

    def test_disabled_resets_to_idle(self) -> None:
        manager = self._manager()
        manager.update(self._input(time_s=11.0, route_progress_m=10.0))

        disabled = manager.update(self._input(time_s=12.0, route_progress_m=11.0, enabled=False))
        self.assertEqual(disabled.state, ProbeBurstState.IDLE_BASELINE)
        self.assertFalse(disabled.burst_active)
        self.assertEqual(disabled.reason, "disabled")

    def test_disabled_idle_refreshes_baseline_before_reenabled(self) -> None:
        manager = self._manager()

        disabled = manager.update(self._input(time_s=100.0, route_progress_m=50.0, enabled=False))
        self.assertEqual(disabled.state, ProbeBurstState.IDLE_BASELINE)
        self.assertEqual(disabled.state_elapsed_s, 0.0)

        reenabled = manager.update(self._input(time_s=101.0, route_progress_m=60.0))
        self.assertEqual(reenabled.state, ProbeBurstState.IDLE_BASELINE)
        self.assertFalse(reenabled.burst_active)

    def test_idle_accumulates_enabled_time_across_short_disabled_gap(self) -> None:
        manager = self._manager()

        first = manager.update(self._input(time_s=5.0, route_progress_m=3.0))
        self.assertEqual(first.state, ProbeBurstState.IDLE_BASELINE)
        self.assertAlmostEqual(first.state_elapsed_s, 5.0)
        self.assertAlmostEqual(first.route_delta_in_state_m, 3.0)

        disabled = manager.update(self._input(time_s=6.0, route_progress_m=3.0, enabled=False))
        self.assertEqual(disabled.state, ProbeBurstState.IDLE_BASELINE)
        self.assertAlmostEqual(disabled.state_elapsed_s, 5.0)

        ready = manager.update(self._input(time_s=11.0, route_progress_m=6.0))
        self.assertEqual(ready.state, ProbeBurstState.BURST_COLLECT_EVIDENCE)
        self.assertEqual(ready.reason, "enter_burst")

    def test_idle_can_enter_pending_burst_before_control_is_allowed(self) -> None:
        manager = self._manager()

        blocked = manager.update(
            ProbeBurstInput(
                time_s=11.0,
                route_progress_m=10.0,
                abs_cross_track_m=1.0,
                evidence_available=False,
                enabled=True,
                control_allowed=False,
            )
        )
        self.assertEqual(blocked.state, ProbeBurstState.BURST_COLLECT_EVIDENCE)
        self.assertFalse(blocked.burst_active)
        self.assertEqual(blocked.reason, "enter_burst")

        ready = manager.update(self._input(time_s=12.0, route_progress_m=11.0))
        self.assertEqual(ready.state, ProbeBurstState.BURST_COLLECT_EVIDENCE)
        self.assertTrue(ready.burst_active)
        self.assertAlmostEqual(ready.state_elapsed_s, 1.0)

    def test_entry_cross_track_is_frozen_during_burst_and_recovery(self) -> None:
        manager = self._manager()

        burst = manager.update(self._input(time_s=11.0, route_progress_m=10.0, abs_cross_track_m=2.5))
        self.assertEqual(burst.state, ProbeBurstState.BURST_COLLECT_EVIDENCE)
        self.assertAlmostEqual(burst.entry_abs_cross_track_m, 2.5)

        still_burst = manager.update(
            self._input(
                time_s=12.0,
                route_progress_m=11.0,
                abs_cross_track_m=20.0,
                evidence_available=True,
            )
        )
        self.assertEqual(still_burst.state, ProbeBurstState.BURST_COLLECT_EVIDENCE)
        self.assertAlmostEqual(still_burst.entry_abs_cross_track_m, 2.5)

        recovery = manager.update(self._input(time_s=13.0, route_progress_m=12.0, abs_cross_track_m=30.0))
        self.assertEqual(recovery.state, ProbeBurstState.RECOVER_ROUTE)
        self.assertAlmostEqual(recovery.entry_abs_cross_track_m, 2.5)

    def test_disabled_during_burst_resets_accumulated_baseline(self) -> None:
        manager = self._manager()
        manager.update(self._input(time_s=11.0, route_progress_m=10.0))

        disabled = manager.update(self._input(time_s=12.0, route_progress_m=11.0, enabled=False))
        self.assertEqual(disabled.state, ProbeBurstState.IDLE_BASELINE)
        self.assertAlmostEqual(disabled.state_elapsed_s, 0.0)

        reenabled = manager.update(self._input(time_s=13.0, route_progress_m=20.0))
        self.assertEqual(reenabled.state, ProbeBurstState.IDLE_BASELINE)
        self.assertFalse(reenabled.burst_active)


if __name__ == "__main__":
    unittest.main()
