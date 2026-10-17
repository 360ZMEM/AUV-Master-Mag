"""Workflow-level checks for the opt-in multi-crossing mission adapter."""
from __future__ import annotations

from types import SimpleNamespace
import unittest

import numpy as np

from auv_mag_tracking.api import CableMap, MagneticInput, NavigationInput
from auv_mag_tracking.v2 import (
    ShallowSurveyInspectionPipelineV2, ShallowSurveyMissionConfig,
    SurveyCurrentEvidence, V2Config, InspectionConfig,
)


class ShallowSurveyWorkflowTest(unittest.TestCase):
    def setUp(self):
        self.boundary = np.array([[-10., -45.], [760., -45.],
                                  [760., 45.], [-10., 45.]])
        self.v2 = V2Config(inspection=InspectionConfig(enabled=True,
                                                      boundary_stop_margin_m=5.))

    def pipeline(self):
        return ShallowSurveyInspectionPipelineV2(
            object(), CableMap(np.array([[0., 0.], [750., 0.]])),
            v2_config=self.v2, search_boundary_xy_m=self.boundary,
            survey_config=ShallowSurveyMissionConfig(),
        )

    @staticmethod
    def inputs(t, x, y, *, case="wire"):
        nav = NavigationInput(t, np.array([x, y, -10.]), 0., speed_mps=.3,
                              position_std_m=.1, heading_std_deg=.2)
        target = 5.5 + .02 * x
        if case == "fused":
            value = (2. + 20. / np.sqrt(1. + ((y - target) / 3.) ** 2)
                     + 20. / np.sqrt(1. + ((y - target - 1.) / 3.) ** 2))
        elif case == "none":
            value = 2.
        else:
            value = 2. + 20. / np.sqrt(1. + ((y - target) / 3.) ** 2)
        return nav, MagneticInput(t, np.tile([value, 0., 0.], (4, 1)), 50.)

    @staticmethod
    def evidence(pipeline, t, x, *, current=.04, cue=True):
        pipeline.set_current_evidence(SurveyCurrentEvidence(
            t, (0., current), .004, "synthetic_current_fixture"))
        pipeline.set_runtime_identity_cue(
            SimpleNamespace(time_s=t, lateral_center_m=5.5 + .02 * x,
                            half_width_m=.25, source="synthetic_sonar_fixture",
                            calibration_id="synthetic_fixture") if cue else None)

    def test_missing_independent_evidence_latches_hold(self):
        pipeline = self.pipeline()
        nav, mag = self.inputs(1., 0., -24.)
        output, guidance = pipeline.step_with_guidance(nav, mag)
        self.assertEqual(output.diagnostics["continuity_reason"], "survey_current_evidence_unavailable")
        self.assertTrue(guidance.safe_lock_active)
        self.evidence(pipeline, 2., 0.)
        nav, mag = self.inputs(2., 0., -24.)
        self.assertEqual(pipeline.step(nav, mag).mode, "hold")

    def test_three_crossings_rejoin_track_and_depth_stays_null(self):
        pipeline = self.pipeline()
        # Prime the exact opt-in route from independently supplied evidence.
        self.evidence(pipeline, 0., 0.)
        nav, mag = self.inputs(0., 0., -24.)
        output, guidance = pipeline.step_with_guidance(nav, mag)
        self.assertEqual(output.mode, "search")
        self.assertFalse(guidance.safe_lock_active)
        self.assertEqual(pipeline._survey_policy.station_count, 3)
        plan = pipeline._survey_plan
        for i, point in enumerate(plan.waypoints_xy_m[1:], 1):
            t = i * .25
            x, y = map(float, point)
            self.evidence(pipeline, t, x)
            nav, mag = self.inputs(t, x, y)
            output, guidance = pipeline.step_with_guidance(nav, mag)
            self.assertIsNone(output.burial_depth_m)
            if output.mode == "hold":
                self.fail(output.diagnostics["continuity_reason"])
        self.assertEqual(pipeline._survey_phase, "rejoin")
        self.assertTrue(pipeline._survey_estimate.geometry_valid)
        last_x = float(plan.waypoints_xy_m[-1, 0])
        last_y = float(plan.waypoints_xy_m[-1, 1])
        track_seen = False
        for j, x in enumerate(np.arange(last_x + .5, last_x + 165., .5), 1):
            t += .25
            # This is an interface/state-machine test. Native PVS dynamics
            # are separately archived in figure 50.
            fraction = min((x - last_x) / 75., 1.)
            y = (1. - fraction) * last_y + fraction * (5.5 + .02 * x)
            self.evidence(pipeline, t, x)
            nav, mag = self.inputs(t, x, y)
            output, guidance = pipeline.step_with_guidance(nav, mag)
            self.assertIsNone(output.burial_depth_m)
            if output.mode == "hold":
                self.fail(output.diagnostics["continuity_reason"])
            track_seen |= pipeline._survey_phase == "track"
            if output.mode == "complete":
                self.assertTrue(guidance.safe_lock_active)
                break
        self.assertTrue(track_seen)
        self.assertEqual(output.mode, "complete")

    def test_fused_neighbor_identity_rejects_after_magnetic_geometry(self):
        pipeline = self.pipeline()
        self.evidence(pipeline, 0., 0.)
        nav, mag = self.inputs(0., 0., -24., case="fused")
        pipeline.step(nav, mag)
        for i, point in enumerate(pipeline._survey_plan.waypoints_xy_m[1:], 1):
            t = i * .25
            x, y = map(float, point)
            self.evidence(pipeline, t, x)
            nav, mag = self.inputs(t, x, y, case="fused")
            output = pipeline.step(nav, mag)
            if output.mode == "hold":
                break
        self.assertTrue(pipeline._survey_estimate.geometry_valid)
        self.assertEqual(output.mode, "hold")
        self.assertEqual(output.diagnostics["continuity_reason"], "survey_identity_rejected")

    def test_negative_current_selects_four_stations_and_right_staging(self):
        pipeline = self.pipeline()
        self.evidence(pipeline, 0., 0., current=-.04)
        nav, mag = self.inputs(0., 0., 27.)
        output = pipeline.step(nav, mag)
        self.assertEqual(output.mode, "search")
        self.assertEqual(pipeline._survey_policy.station_count, 4)
        self.assertEqual(pipeline._survey_policy.initial_lateral_sign, 1)

    def test_no_source_rejects_and_current_loss_holds_zero_guidance(self):
        empty = self.pipeline()
        self.evidence(empty, 0., 0.)
        nav, mag = self.inputs(0., 0., -24., case="none")
        empty.step(nav, mag)
        for i, point in enumerate(empty._survey_plan.waypoints_xy_m[1:], 1):
            t = i * .25
            x, y = map(float, point)
            self.evidence(empty, t, x)
            nav, mag = self.inputs(t, x, y, case="none")
            result = empty.step(nav, mag)
            if result.mode == "hold":
                break
        self.assertEqual(result.mode, "hold")
        self.assertIn("survey_geometry_rejected", result.diagnostics["continuity_reason"])

        stale = self.pipeline()
        self.evidence(stale, 0., 0.)
        nav, mag = self.inputs(0., 0., -24.)
        stale.step_with_guidance(nav, mag)
        nav, mag = self.inputs(3., 0.5, -24.)
        output, guidance = stale.step_with_guidance(nav, mag)
        self.assertEqual(output.diagnostics["continuity_reason"], "survey_current_evidence_lost")
        self.assertEqual(guidance.speed_mps, 0.)
        self.assertTrue(guidance.safe_lock_active)


if __name__ == "__main__":
    unittest.main()
