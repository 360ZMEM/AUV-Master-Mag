"""Contract checks for the opt-in prealigned flythrough mode."""
from __future__ import annotations

from dataclasses import replace
import math
from types import SimpleNamespace
import unittest

import numpy as np

from auv_mag_tracking.api import CableMap, MagneticInput, NavigationInput
from auv_mag_tracking.v2 import FlythroughConfig, FlythroughInspectionPipelineV2
from auv_mag_tracking.v2.config import V2Config, InspectionConfig


class FlythroughContractTest(unittest.TestCase):
    def setUp(self):
        self.boundary = np.array([[-270., -30.], [120., -30.], [120., 30.], [-270., 30.]])
        self.config = replace(V2Config(), inspection=InspectionConfig(
            enabled=True, maximum_scan_duration_s=450.,
            maximum_observation_age_s=1000., maximum_observation_distance_m=250.,
            boundary_stop_margin_m=10.,
        ))
        self.map = CableMap(np.array([[0., 0.], [80., 0.]]))

    def make_pipeline(self):
        return FlythroughInspectionPipelineV2(
            object(), self.map, v2_config=self.config,
            search_boundary_xy_m=self.boundary,
        )

    def test_staging_and_prior_preconditions_fail_closed(self):
        pipeline = self.make_pipeline()
        nav = NavigationInput(0., np.array([0., -15., -10.]), 0., speed_mps=.3, position_std_m=.1)
        magnetic = MagneticInput(0., np.tile([1., 0., 0.], (4, 1)), 50.)
        output, guidance = pipeline.step_with_guidance(nav, magnetic)
        self.assertEqual(output.mode, 'hold')
        self.assertEqual(output.diagnostics['continuity_reason'], 'flythrough_staging_invalid')
        self.assertEqual(guidance.speed_mps, 0.)
        with self.assertRaisesRegex(ValueError, 'straight'):
            FlythroughInspectionPipelineV2(
                object(), CableMap(np.array([[0., 0.], [40., 0.], [80., 20.]])),
                v2_config=self.config, search_boundary_xy_m=self.boundary,
            )

    def test_required_side_hint_rejects_missing_ambiguous_and_wrong_direction(self):
        cases = (
            (FlythroughConfig(side_hint_required=True), "flythrough_side_hint_unavailable"),
            (FlythroughConfig(side_hint_required=True, side_hint_min_m=-2.,
                              side_hint_max_m=10., side_hint_source="survey_map"),
             "flythrough_side_hint_ambiguous"),
            (FlythroughConfig(side_hint_required=True, side_hint_min_m=-10.,
                              side_hint_max_m=-2., side_hint_source="survey_map"),
             "flythrough_side_hint_direction_mismatch"),
            (FlythroughConfig(side_hint_required=True, side_hint_min_m=10.,
                              side_hint_max_m=20., side_hint_source="survey_map"),
             "flythrough_side_hint_outside_scan"),
            (FlythroughConfig(side_hint_required=True, side_hint_min_m=2.,
                              side_hint_max_m=10., side_hint_source="survey_map",
                              minimum_halfmax_tail_m=7.),
             "flythrough_side_hint_support_insufficient"),
            (FlythroughConfig(side_hint_required=True, side_hint_min_m=2.,
                              side_hint_max_m=8., side_hint_source="survey_map",
                              maximum_side_hint_width_m=1., minimum_halfmax_tail_m=7.),
             "flythrough_side_hint_too_wide"),
            (FlythroughConfig(side_hint_required=True, side_hint_min_m=2.,
                              side_hint_max_m=8., side_hint_source="survey_map"),
             "flythrough_side_hint_support_unconfigured"),
            (FlythroughConfig(side_hint_required=True, side_hint_min_m="unknown",
                              side_hint_max_m=10., side_hint_source="survey_map"),
             "flythrough_side_hint_invalid"),
            (FlythroughConfig(side_hint_required=True, side_hint_min_m=float("nan"),
                              side_hint_max_m=10., side_hint_source="survey_map"),
             "flythrough_side_hint_invalid"),
            (FlythroughConfig(side_hint_required=True, side_hint_min_m=10.,
                              side_hint_max_m=2., side_hint_source="survey_map"),
             "flythrough_side_hint_invalid"),
            (FlythroughConfig(side_hint_required=True, side_hint_min_m=2.,
                              side_hint_max_m=10., side_hint_source="synthetic_test_fixture",
                              allow_synthetic_side_hint=False),
             "flythrough_side_hint_simulation_only"),
        )
        for policy, expected in cases:
            with self.subTest(expected=expected):
                pipeline = FlythroughInspectionPipelineV2(
                    object(), self.map, v2_config=self.config,
                    search_boundary_xy_m=self.boundary, flythrough_config=policy,
                )
                start, end = pipeline._fly_start, pipeline._fly_end
                heading = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
                nav = NavigationInput(0., np.array([*start, -10.]), heading,
                                      speed_mps=.3, position_std_m=.1)
                magnetic = MagneticInput(0., np.tile([1., 0., 0.], (4, 1)), 50.)
                output, guidance = pipeline.step_with_guidance(nav, magnetic)
                self.assertEqual(output.mode, "hold")
                self.assertEqual(output.diagnostics["continuity_reason"], expected)
                self.assertEqual(guidance.speed_mps, 0.)

    def test_required_side_hint_accepts_matching_both_directions(self):
        for policy in (
            FlythroughConfig(side_hint_required=True, side_hint_min_m=2.,
                             side_hint_max_m=8., side_hint_source="survey_map",
                             minimum_halfmax_tail_m=7.),
            FlythroughConfig(start_lateral_m=15., end_lateral_m=-15.,
                             side_hint_required=True, side_hint_min_m=-8.,
                             side_hint_max_m=-2., side_hint_source="previous_track",
                             minimum_halfmax_tail_m=7.),
        ):
            with self.subTest(end_lateral_m=policy.end_lateral_m):
                pipeline = FlythroughInspectionPipelineV2(
                    object(), self.map, v2_config=self.config,
                    search_boundary_xy_m=self.boundary, flythrough_config=policy,
                )
                start, end = pipeline._fly_start, pipeline._fly_end
                heading = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
                nav = NavigationInput(0., np.array([*start, -10.]), heading,
                                      speed_mps=.3, position_std_m=.1)
                magnetic = MagneticInput(0., np.tile([1., 0., 0.], (4, 1)), 50.)
                output, guidance = pipeline.step_with_guidance(nav, magnetic)
                self.assertEqual(output.mode, "search")
                self.assertGreater(guidance.speed_mps, 0.)

    def test_crossing_confirms_lateral_without_depth(self):
        pipeline = self.make_pipeline()
        start, end = pipeline._fly_start, pipeline._fly_end
        heading = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
        rng = np.random.default_rng(101)
        first_confirm = None
        final = None
        for index, fraction in enumerate(np.linspace(0., 1., 2300)):
            point = start + fraction * (end - start)
            y = float(point[1])
            value = 1. + 20. / math.sqrt(1. + ((y - 6.) / 3.) ** 2) + rng.normal(0., .03)
            time_s = index * .25
            nav = NavigationInput(time_s, np.array([*point, -10.]), heading, speed_mps=.3, position_std_m=.1)
            magnetic = MagneticInput(time_s, np.tile([value, 0., 0.], (4, 1)), 50.)
            final, guidance = pipeline.step_with_guidance(nav, magnetic)
            if final.mode == 'confirm' and first_confirm is None:
                first_confirm = time_s
            self.assertIsNone(final.burial_depth_m)
            self.assertFalse(final.diagnostics.get('depth_valid', False))
        self.assertIsNotNone(first_confirm)
        self.assertEqual(final.mode, 'track')
        self.assertTrue(final.diagnostics['lateral_valid'])
        self.assertAlmostEqual(float(final.estimated_cable_xy_m[1]), 6., delta=.5)

    def test_inspection_route_maximum_is_enforced_before_endpoint(self):
        fly = FlythroughConfig(route_quality_mean_limit_m=1.,
                               route_quality_max_limit_m=2.,
                               route_quality_min_samples=1)
        pipeline = FlythroughInspectionPipelineV2(
            object(), self.map, v2_config=self.config,
            search_boundary_xy_m=self.boundary, flythrough_config=fly,
        )
        start, end = pipeline._fly_start, pipeline._fly_end
        heading = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
        for index, fraction in enumerate(np.linspace(0., 1., 2300)):
            point = start + fraction * (end - start)
            value = 1. + 20. / math.sqrt(1. + ((float(point[1]) - 6.) / 3.) ** 2)
            t = index * .25
            output, _ = pipeline.step_with_guidance(
                NavigationInput(t, np.array([*point, -10.]), heading,
                                speed_mps=.3, position_std_m=.1),
                MagneticInput(t, np.tile([value, 0., 0.], (4, 1)), 50.),
            )
        self.assertEqual(output.mode, "track")
        pipeline._inspection_started = True
        estimated_y = float(output.estimated_cable_xy_m[1])
        for time_s, lateral_error, expected_mode in ((575., 1.5, "track"),
                                                     (575.25, 2.25, "hold")):
            nav = NavigationInput(time_s, np.array([10., estimated_y + lateral_error, -10.]),
                                  0., speed_mps=.3, position_std_m=.1)
            magnetic = MagneticInput(time_s, np.tile([1., 0., 0.], (4, 1)), 50.)
            output, guidance = pipeline.step_with_guidance(nav, magnetic)
            self.assertEqual(output.mode, expected_mode)
        self.assertEqual(output.diagnostics["continuity_reason"],
                         "flythrough_route_deviation_exceeded")
        self.assertGreater(output.diagnostics["route_observed_lateral_max_m"], 2.)
        self.assertFalse(output.diagnostics["route_quality_valid"])
        self.assertEqual(guidance.speed_mps, 0.)

    def test_short_tail_rejects_depth_even_with_experimental_depth_override(self):
        setting = replace(self.config, inspection=replace(
            self.config.inspection, allow_uncalibrated_depth=True))
        pipeline = FlythroughInspectionPipelineV2(
            object(), self.map, v2_config=setting,
            search_boundary_xy_m=self.boundary,
        )
        x = np.linspace(-15., 15., 241)
        pipeline._scan_x_m = list(x)
        pipeline._scan_envelope_nt = list(1. + 20. / np.sqrt(1. + ((x - 6.) / 3.) ** 2))
        result = pipeline._estimate_current_scan(altitude_m=0.)
        self.assertIsNotNone(result.width_m)
        self.assertIn('depth_shape_support_insufficient', result.reason_codes)
        self.assertIn('depth_shape_ambiguous', result.reason_codes)
        self.assertIsNone(result.burial_depth_m)

    def test_wide_matched_dipole_can_pass_shape_gate(self):
        setting = replace(self.config, inspection=replace(
            self.config.inspection, allow_uncalibrated_depth=True))
        pipeline = FlythroughInspectionPipelineV2(
            object(), self.map, v2_config=setting,
            search_boundary_xy_m=self.boundary,
        )
        x = np.linspace(-30., 30., 601)
        pipeline._scan_x_m = list(x)
        pipeline._scan_envelope_nt = list(1. + 20. / (1. + (x / 3.) ** 2))
        result = pipeline._estimate_current_scan(altitude_m=0.)
        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.vertical_separation_m, 3., delta=.1)

    def test_reverse_side_hint_confirms_negative_offset(self):
        fly = FlythroughConfig(start_lateral_m=15., end_lateral_m=-15.)
        pipeline = FlythroughInspectionPipelineV2(
            object(), self.map, v2_config=self.config,
            search_boundary_xy_m=self.boundary, flythrough_config=fly,
        )
        start, end = pipeline._fly_start, pipeline._fly_end
        heading = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
        rng = np.random.default_rng(101)
        for index, fraction in enumerate(np.linspace(0., 1., 2300)):
            point = start + fraction * (end - start)
            value = 1. + 20. / math.sqrt(1. + ((float(point[1]) + 6.) / 3.) ** 2)
            value += rng.normal(0., .03)
            t = index * .25
            nav = NavigationInput(t, np.array([*point, -10.]), heading,
                                  speed_mps=.3, position_std_m=.1)
            magnetic = MagneticInput(t, np.tile([value, 0., 0.], (4, 1)), 50.)
            output, _ = pipeline.step_with_guidance(nav, magnetic)
        self.assertEqual(output.mode, 'track')
        self.assertTrue(output.diagnostics['lateral_valid'])
        self.assertAlmostEqual(float(output.estimated_cable_xy_m[1]), -6., delta=.5)
        self.assertIsNone(output.burial_depth_m)

    def test_measured_center_outside_declared_side_hint_cannot_start_tracking(self):
        config = replace(self.config, inspection=replace(
            self.config.inspection, maximum_scan_duration_s=600.))
        fly = FlythroughConfig(
            side_hint_required=True, side_hint_min_m=2., side_hint_max_m=8.,
            side_hint_source="survey_map", minimum_halfmax_tail_m=7.,
        )
        for truth, expected_mode in ((6., "track"), (-6., "hold")):
            with self.subTest(truth=truth):
                pipeline = FlythroughInspectionPipelineV2(
                    object(), self.map, v2_config=config,
                    search_boundary_xy_m=self.boundary, flythrough_config=fly,
                )
                start, end = pipeline._fly_start, pipeline._fly_end
                heading = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
                rng = np.random.default_rng(101)
                for index, fraction in enumerate(np.linspace(0., 1., 2300)):
                    point = start + fraction * (end - start)
                    value = 1. + 20. / math.sqrt(1. + ((float(point[1]) - truth) / 3.) ** 2)
                    value += rng.normal(0., .03)
                    t = index * .25
                    nav = NavigationInput(t, np.array([*point, -10.]), heading,
                                          speed_mps=.3, position_std_m=.1)
                    magnetic = MagneticInput(t, np.tile([value, 0., 0.], (4, 1)), 50.)
                    output, _ = pipeline.step_with_guidance(nav, magnetic)
                self.assertEqual(output.mode, expected_mode)
                self.assertEqual(bool(output.diagnostics["lateral_valid"]), truth > 0)
                if truth < 0:
                    self.assertIn("side_hint_observation_inconsistent",
                                  pipeline._last_lateral.reason_codes)
                    self.assertIsNone(output.burial_depth_m)

    def test_runtime_identity_cue_controls_track_transition(self):
        config = replace(self.config, inspection=replace(
            self.config.inspection, maximum_scan_duration_s=600.))
        fly = FlythroughConfig(
            side_hint_required=True, side_hint_min_m=2., side_hint_max_m=8.,
            side_hint_source="survey_map", minimum_halfmax_tail_m=7.,
            require_runtime_identity_cue=True,
        )
        for cue_center, cue_age, cue_half_width, expected_mode, reason in (
            (None, 0., .35, "hold", "runtime_identity_cue_unavailable"),
            (6., 0., .35, "track", None),
            (7.5, 0., .35, "hold", "runtime_identity_cue_inconsistent"),
            (6., 1., .35, "hold", "runtime_identity_cue_stale_or_invalid"),
            (6., 0., .55, "hold", "runtime_identity_cue_uncertain"),
        ):
            with self.subTest(cue_center=cue_center, cue_age=cue_age):
                pipeline = FlythroughInspectionPipelineV2(
                    object(), self.map, v2_config=config,
                    search_boundary_xy_m=self.boundary, flythrough_config=fly,
                )
                start, end = pipeline._fly_start, pipeline._fly_end
                heading = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
                for index, fraction in enumerate(np.linspace(0., 1., 2300)):
                    point = start + fraction * (end - start)
                    value = 1. + 20. / math.sqrt(1. + ((float(point[1]) - 6.) / 3.) ** 2)
                    t = index * .25
                    if cue_center is not None:
                        pipeline.set_runtime_identity_cue(SimpleNamespace(
                            time_s=t - cue_age, lateral_center_m=cue_center,
                            half_width_m=cue_half_width,
                        ))
                    nav = NavigationInput(t, np.array([*point, -10.]), heading,
                                          speed_mps=.3, position_std_m=.1)
                    magnetic = MagneticInput(t, np.tile([value, 0., 0.], (4, 1)), 50.)
                    output, guidance = pipeline.step_with_guidance(nav, magnetic)
                self.assertEqual(output.mode, expected_mode)
                if reason is not None:
                    self.assertIn(reason, pipeline._last_lateral.reason_codes)
                    self.assertEqual(guidance.speed_mps, 0.)

    def test_runtime_identity_can_supersede_biased_static_center_only_when_fresh(self):
        config = replace(self.config, inspection=replace(
            self.config.inspection, maximum_scan_duration_s=600.))
        with self.assertRaisesRegex(ValueError, "requires a runtime identity cue"):
            FlythroughConfig(runtime_identity_supersedes_side_hint_center=True).validate()
        with self.assertRaisesRegex(ValueError, "anchor interval must be complete"):
            FlythroughConfig(require_runtime_identity_cue=True,
                             runtime_target_anchor_min_m=5.8).validate()
        with self.assertRaisesRegex(ValueError, "exclusive corridor must be complete"):
            FlythroughConfig(require_runtime_identity_cue=True,
                             runtime_target_anchor_min_m=5.8,
                             runtime_target_anchor_max_m=6.2,
                             runtime_target_exclusive_min_m=5.45).validate()
        with self.assertRaisesRegex(ValueError, "exclusive corridor must enclose"):
            FlythroughConfig(require_runtime_identity_cue=True,
                             runtime_target_anchor_min_m=5.8,
                             runtime_target_anchor_max_m=6.2,
                             runtime_target_exclusive_min_m=5.9,
                             runtime_target_exclusive_max_m=6.55).validate()
        cases = (
            (False, False, 6., False, "hold", "side_hint_observation_inconsistent"),
            (True, False, 6., False, "track", None),
            (True, True, 6., False, "hold", "runtime_identity_cue_inconsistent"),
            (True, False, None, False, "hold", "runtime_identity_cue_unavailable"),
            (True, True, 7.026, False, "track", None),
            (True, True, 7.026, True, "hold", "runtime_target_anchor_inconsistent"),
            (True, False, 6., True, "track", None),
        )
        for supersede, neighbor, cue_y, anchor, expected_mode, reason in cases:
            with self.subTest(supersede=supersede, neighbor=neighbor, cue_y=cue_y,
                              anchor=anchor):
                fly = FlythroughConfig(
                    side_hint_required=True, side_hint_min_m=6.6, side_hint_max_m=7.5,
                    side_hint_source="survey_map", maximum_side_hint_width_m=1.,
                    minimum_halfmax_tail_m=7., require_runtime_identity_cue=True,
                    runtime_identity_supersedes_side_hint_center=supersede,
                    runtime_target_anchor_min_m=5.8 if anchor else None,
                    runtime_target_anchor_max_m=6.2 if anchor else None,
                )
                pipeline = FlythroughInspectionPipelineV2(
                    object(), self.map, v2_config=config,
                    search_boundary_xy_m=self.boundary, flythrough_config=fly,
                )
                start, end = pipeline._fly_start, pipeline._fly_end
                heading = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
                rng = np.random.default_rng(101)
                for index, fraction in enumerate(np.linspace(0., 1., 2300)):
                    point = start + fraction * (end - start)
                    y = float(point[1])
                    value = 1. + 20. / math.sqrt(1. + ((y - 6.) / 3.) ** 2)
                    if neighbor:
                        value += 10. / math.sqrt(1. + ((y - 9.) / 3.) ** 2)
                    value += rng.normal(0., .03)
                    t = index * .25
                    if cue_y is not None:
                        pipeline.set_runtime_identity_cue(SimpleNamespace(
                            time_s=t, lateral_center_m=cue_y, half_width_m=.35,
                        ))
                    nav = NavigationInput(t, np.array([*point, -10.]), heading,
                                          speed_mps=.3, position_std_m=.1)
                    magnetic = MagneticInput(t, np.tile([value, 0., 0.], (4, 1)), 50.)
                    output, guidance = pipeline.step_with_guidance(nav, magnetic)
                self.assertEqual(output.mode, expected_mode)
                self.assertIsNone(output.burial_depth_m)
                if reason is not None:
                    self.assertIn(reason, pipeline._last_lateral.reason_codes)
                    self.assertEqual(guidance.speed_mps, 0.)
                else:
                    self.assertEqual(output.diagnostics["target_identity_authority"],
                                     ("runtime_sonar_with_independent_anchor" if anchor
                                      else "runtime_identity_cue"))

    def test_exclusive_corridor_rejects_near_neighbor_that_overlaps_anchor(self):
        config = replace(self.config, inspection=replace(
            self.config.inspection, maximum_scan_duration_s=600.))
        for truth, exclusive, expected_mode, reason in (
                (6.6, False, "track", None),
                (6.6, True, "hold", "runtime_target_exclusive_corridor_inconsistent"),
                (6.0, True, "track", None)):
            with self.subTest(truth=truth, exclusive=exclusive):
                fly = FlythroughConfig(
                    side_hint_required=True, side_hint_min_m=6.6, side_hint_max_m=7.5,
                    side_hint_source="survey_map", minimum_halfmax_tail_m=7.,
                    require_runtime_identity_cue=True,
                    runtime_identity_supersedes_side_hint_center=True,
                    runtime_target_anchor_min_m=5.8,
                    runtime_target_anchor_max_m=6.2,
                    runtime_target_exclusive_min_m=5.45 if exclusive else None,
                    runtime_target_exclusive_max_m=6.55 if exclusive else None,
                )
                pipeline = FlythroughInspectionPipelineV2(
                    object(), self.map, v2_config=config,
                    search_boundary_xy_m=self.boundary, flythrough_config=fly,
                )
                start, end = pipeline._fly_start, pipeline._fly_end
                heading = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
                for index, fraction in enumerate(np.linspace(0., 1., 2300)):
                    point = start + fraction * (end - start)
                    y = float(point[1])
                    value = 1. + 20. / math.sqrt(1. + ((y - truth) / 3.) ** 2)
                    t = index * .25
                    pipeline.set_runtime_identity_cue(SimpleNamespace(
                        time_s=t, lateral_center_m=truth, half_width_m=.5,
                    ))
                    nav = NavigationInput(t, np.array([*point, -10.]), heading,
                                          speed_mps=.3, position_std_m=.1)
                    magnetic = MagneticInput(t, np.tile([value, 0., 0.], (4, 1)), 50.)
                    output, guidance = pipeline.step_with_guidance(nav, magnetic)
                self.assertEqual(output.mode, expected_mode)
                self.assertIsNone(output.burial_depth_m)
                if reason:
                    self.assertIn(reason, pipeline._last_lateral.reason_codes)
                    self.assertEqual(guidance.speed_mps, 0.)

    def test_track_reassociation_outside_exclusive_corridor_latches_hold(self):
        config = replace(self.config, inspection=replace(
            self.config.inspection, maximum_scan_duration_s=600.))
        for exclusive, expected_mode in ((False, "track"), (True, "hold")):
            with self.subTest(exclusive=exclusive):
                fly = FlythroughConfig(
                    side_hint_required=True, side_hint_min_m=6.6, side_hint_max_m=7.5,
                    side_hint_source="survey_map", minimum_halfmax_tail_m=7.,
                    require_runtime_identity_cue=True,
                    runtime_identity_supersedes_side_hint_center=True,
                    runtime_target_anchor_min_m=5.8,
                    runtime_target_anchor_max_m=6.2,
                    runtime_target_exclusive_min_m=5.45 if exclusive else None,
                    runtime_target_exclusive_max_m=6.55 if exclusive else None,
                )
                pipeline = FlythroughInspectionPipelineV2(
                    object(), self.map, v2_config=config,
                    search_boundary_xy_m=self.boundary, flythrough_config=fly,
                )
                start, end = pipeline._fly_start, pipeline._fly_end
                heading = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
                for index, fraction in enumerate(np.linspace(0., 1., 2300)):
                    point = start + fraction * (end - start)
                    y = float(point[1])
                    value = 1. + 20. / math.sqrt(1. + ((y - 6.) / 3.) ** 2)
                    t = index * .25
                    pipeline.set_runtime_identity_cue(SimpleNamespace(
                        time_s=t, lateral_center_m=6., half_width_m=.43,
                    ))
                    nav = NavigationInput(t, np.array([*point, -10.]), heading,
                                          speed_mps=.3, position_std_m=.1)
                    magnetic = MagneticInput(t, np.tile([value, 0., 0.], (4, 1)), 50.)
                    output, guidance = pipeline.step_with_guidance(nav, magnetic)
                self.assertEqual(output.mode, "track")
                t += .25
                pipeline.set_runtime_identity_cue(SimpleNamespace(
                    time_s=t, lateral_center_m=6.59, half_width_m=.6,
                ))
                output, guidance = pipeline.step_with_guidance(
                    NavigationInput(t, np.array([*point, -10.]), heading,
                                    speed_mps=.3, position_std_m=.1), magnetic,
                )
                self.assertEqual(output.mode, expected_mode)
                if exclusive:
                    self.assertEqual(output.diagnostics["continuity_reason"],
                                     "flythrough_runtime_identity_exclusive_corridor_inconsistent")
                    self.assertEqual(guidance.speed_mps, 0.)

    def test_runtime_identity_loss_or_divergence_during_track_stops_guidance(self):
        config = replace(self.config, inspection=replace(
            self.config.inspection, maximum_scan_duration_s=600.))
        fly = FlythroughConfig(
            side_hint_required=True, side_hint_min_m=2., side_hint_max_m=8.,
            side_hint_source="survey_map", minimum_halfmax_tail_m=7.,
            require_runtime_identity_cue=True, maximum_runtime_identity_gap_s=2.,
        )
        for fault, expected in (("switch", "flythrough_runtime_identity_diverged"),
                                ("dropout", "flythrough_runtime_identity_lost")):
            with self.subTest(fault=fault):
                pipeline = FlythroughInspectionPipelineV2(
                    object(), self.map, v2_config=config,
                    search_boundary_xy_m=self.boundary, flythrough_config=fly,
                )
                start, end = pipeline._fly_start, pipeline._fly_end
                heading = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
                for index, fraction in enumerate(np.linspace(0., 1., 2300)):
                    point = start + fraction * (end - start)
                    value = 1. + 20. / math.sqrt(1. + ((float(point[1]) - 6.) / 3.) ** 2)
                    t = index * .25
                    pipeline.set_runtime_identity_cue(SimpleNamespace(
                        time_s=t, lateral_center_m=6., half_width_m=.35,
                    ))
                    nav = NavigationInput(t, np.array([*point, -10.]), heading,
                                          speed_mps=.3, position_std_m=.1)
                    magnetic = MagneticInput(t, np.tile([value, 0., 0.], (4, 1)), 50.)
                    output, _ = pipeline.step_with_guidance(nav, magnetic)
                self.assertEqual(output.mode, "track")
                next_time = t + (.25 if fault == "switch" else 2.25)
                pipeline.set_runtime_identity_cue(
                    SimpleNamespace(time_s=next_time, lateral_center_m=9., half_width_m=.35)
                    if fault == "switch" else None
                )
                nav = NavigationInput(next_time, np.array([*end, -10.]), heading,
                                      speed_mps=.3, position_std_m=.1)
                magnetic = MagneticInput(next_time, np.tile([value, 0., 0.], (4, 1)), 50.)
                held, guidance = pipeline.step_with_guidance(nav, magnetic)
                self.assertEqual(held.mode, "hold")
                self.assertEqual(held.diagnostics["continuity_reason"], expected)
                self.assertEqual(guidance.speed_mps, 0.)

    def test_rejoin_runin_is_selected_from_current_relative_to_cable_side(self):
        fly = FlythroughConfig(rejoin_runin_m=20., rejoin_opposed_runin_m=5.,
                               rejoin_current_direction_threshold_mps=.02)
        pipeline = FlythroughInspectionPipelineV2(
            object(), self.map, v2_config=self.config,
            search_boundary_xy_m=self.boundary, flythrough_config=fly,
        )
        for offset, current_normal, expected in (
            (2., .05, 20.), (2., -.05, 5.),
            (-2., -.05, 20.), (-2., .05, 5.),
            (2., 0., 5.),
        ):
            with self.subTest(offset=offset, current_normal=current_normal):
                pipeline._observation_prior_offset_m = offset
                pipeline._fly_current_xy = current_normal * pipeline._fly_normal
                self.assertEqual(pipeline._choose_rejoin_runin_m(), expected)

    def test_world_velocity_proxy_does_not_differentiate_noisy_position(self):
        pipeline = self.make_pipeline()
        start, end = pipeline._fly_start, pipeline._fly_end
        heading = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
        heading_rad = math.radians(heading)
        magnetic = lambda t: MagneticInput(t, np.tile([1., 0., 0.], (4, 1)), 50.)
        for index, displacement in enumerate((np.zeros(2), np.array([.5, .8]))):
            t = .25 * index
            nav = NavigationInput(t, np.array([*(start + displacement), -10.]),
                                  heading, speed_mps=.3, position_std_m=.1)
            nav.ground_velocity_ned_mps = np.array([
                .3 * math.cos(heading_rad), .3 * math.sin(heading_rad) + .05, 0.
            ])
            output, _ = pipeline.step_with_guidance(nav, magnetic(t))
            self.assertEqual(output.mode, "search")
        self.assertAlmostEqual(pipeline._fly_current_xy[1], .05 * (1. - .95**2), places=6)
        self.assertLess(np.linalg.norm(pipeline._fly_current_xy), .08)

    def test_malformed_world_velocity_holds_without_poisoning_proxy(self):
        pipeline = self.make_pipeline()
        start, end = pipeline._fly_start, pipeline._fly_end
        heading = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
        nav = NavigationInput(0., np.array([*start, -10.]), heading, speed_mps=.3)
        nav.ground_velocity_ned_mps = np.array([float("nan"), 0., 0.])
        mag = MagneticInput(0., np.tile([1., 0., 0.], (4, 1)), 50.)
        output, guidance = pipeline.step_with_guidance(nav, mag)
        self.assertEqual(output.mode, "hold")
        self.assertEqual(output.diagnostics["continuity_reason"], "navigation_invalid")
        self.assertEqual(guidance.speed_mps, 0.)
        self.assertTrue(np.isfinite(pipeline._fly_current_xy).all())

    def test_navigation_uncertainty_gates_reject_heading_and_velocity(self):
        for heading_sigma, velocity_sigma, expected in (
            (3., .01, "navigation_heading_uncertain"),
            (1., .04, "navigation_velocity_uncertain"),
        ):
            with self.subTest(expected=expected):
                pipeline = self.make_pipeline()
                start, end = pipeline._fly_start, pipeline._fly_end
                heading = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
                nav = NavigationInput(0., np.array([*start, -10.]), heading,
                                      speed_mps=.3, position_std_m=.1,
                                      heading_std_deg=heading_sigma)
                nav.ground_velocity_ned_mps = np.array([.3, 0., 0.])
                nav.ground_velocity_std_mps = velocity_sigma
                mag = MagneticInput(0., np.tile([1., 0., 0.], (4, 1)), 50.)
                output, guidance = pipeline.step_with_guidance(nav, mag)
                self.assertEqual(output.mode, "hold")
                self.assertEqual(output.diagnostics["continuity_reason"], expected)
                self.assertEqual(guidance.speed_mps, 0.)


    def test_opt_in_track_disturbance_gate_requires_velocity_and_detects_change(self):
        fly = FlythroughConfig(
            maximum_track_lateral_disturbance_change_mps=.01,
            track_disturbance_window_s=4.,
        )
        pipeline = FlythroughInspectionPipelineV2(
            object(), self.map, v2_config=self.config,
            search_boundary_xy_m=self.boundary, flythrough_config=fly,
        )

        def nav(t, velocity=True):
            sample = NavigationInput(t, np.array([0., 0., -10.]), 0., speed_mps=.3)
            if velocity:
                sample.ground_velocity_ned_mps = np.array([.3, 0., 0.])
                sample.ground_velocity_std_mps = .01
            return sample

        self.assertEqual(pipeline._track_disturbance_change(nav(0., False))[0],
                         "flythrough_track_disturbance_evidence_unavailable")
        pipeline._fly_current_xy = np.array([0., .05])
        self.assertEqual(pipeline._track_disturbance_change(nav(1.)), (None, None))
        pipeline._fly_current_xy = np.array([0., .047])
        self.assertEqual(pipeline._track_disturbance_change(nav(5.))[0], None)
        pipeline._fly_current_xy = np.array([0., .035])
        reason, delta = pipeline._track_disturbance_change(nav(6.))
        self.assertEqual(reason, "flythrough_track_disturbance_changed")
        self.assertAlmostEqual(delta, -.015)
        pipeline.reset()
        self.assertFalse(pipeline._track_disturbance_history)


if __name__ == '__main__':
    unittest.main()
