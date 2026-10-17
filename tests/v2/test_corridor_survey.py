from __future__ import annotations

import unittest
from dataclasses import replace

import numpy as np

from auv_mag_tracking.v2 import (
    CorridorSurveyConfig, FwhmConfig, InspectionConfig, ShallowZigzagConfig,
    SurveyProfile, assess_turn_feasibility, build_corridor_survey,
    build_shallow_zigzag_survey, estimate_survey_line, select_shallow_start_side,
)


class CorridorSurveyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = CorridorSurveyConfig()
        self.boundary = np.array([[-10., -24.], [75., -24.], [75., 24.], [-10., 24.]])
        self.plan = build_corridor_survey(
            np.array([[0., 0.], [60., 0.]]), self.boundary, self.config,
        )
        self.fwhm = FwhmConfig(shape_model="thesis_dipole", altitude_m=0.,
                                minimum_prominence_nt=0.25)
        self.inspection = InspectionConfig(enabled=True)

    def profiles(self, *, shape: str = "wire", shift_m: float = 6.,
                 slope: float = 0.05, noise: float = 0.02,
                 double_source: bool = False, spike_station: int | None = None):
        rng = np.random.default_rng(102)
        profiles = []
        for idx, s in enumerate(self.plan.station_along_m):
            x = np.linspace(-15., 15., 151)
            center = shift_m + slope * s
            q = ((x - center) / 3.) ** 2
            anomaly = 20. / np.sqrt(1. + q) if shape == "wire" else 20. / (1. + q) ** .7
            if double_source:
                anomaly += 19. / np.sqrt(1. + ((x - center + 7.) / 2.5) ** 2)
            if spike_station == idx:
                anomaly = 20. / np.sqrt(1. + ((x + 5.) / 3.) ** 2)
            positions = self.plan.station_centers_xy_m[idx] + x[:, None] * self.plan.prior_normal_xy
            profiles.append(SurveyProfile(idx, positions, 2. + anomaly + rng.normal(0., noise, len(x)), noise))
        return profiles

    def test_three_crossings_recover_line_from_displaced_prior_with_wrong_shape(self) -> None:
        for shape in ("wire", "intermediate"):
            result = estimate_survey_line(self.plan, self.profiles(shape=shape),
                                          self.config, self.fwhm, self.inspection)
            self.assertTrue(result.geometry_valid, result.reason_codes)
            self.assertAlmostEqual(result.line_start_xy_m[1], 6., delta=0.25)
            self.assertAlmostEqual(result.line_end_xy_m[1], 8., delta=0.25)
            self.assertAlmostEqual(result.heading_deviation_deg, np.degrees(np.arctan(.05)), delta=.5)
            self.assertIsNone(result.burial_depth_m)

    def test_one_transient_crossing_cannot_define_consistent_line(self) -> None:
        result = estimate_survey_line(self.plan, self.profiles(spike_station=1),
                                      self.config, self.fwhm, self.inspection)
        self.assertFalse(result.geometry_valid)
        self.assertIn("survey_line_inconsistent", result.reason_codes)
        self.assertIsNone(result.line_start_xy_m)

    def test_separated_neighbor_rejects_but_fused_neighbor_exposes_identity_limit(self) -> None:
        separated = estimate_survey_line(self.plan, self.profiles(double_source=True),
                                         self.config, self.fwhm, self.inspection)
        self.assertFalse(separated.geometry_valid)
        profiles = self.profiles()
        for idx, profile in enumerate(profiles):
            x = profile.positions_xy_m[:, 1]
            target = 6. + .05 * self.plan.station_along_m[idx]
            blended = profile.envelope_nt + 20. / np.sqrt(1. + ((x - target - 1.) / 3.) ** 2)
            profiles[idx] = SurveyProfile(idx, profile.positions_xy_m, blended, .02)
        fused = estimate_survey_line(self.plan, profiles, self.config,
                                     self.fwhm, self.inspection)
        self.assertTrue(fused.geometry_valid, fused.reason_codes)
        self.assertAlmostEqual(fused.line_start_xy_m[1], 6.5, delta=.1)
        self.assertIsNone(fused.burial_depth_m)

    def test_missing_and_gapped_crossings_reject(self) -> None:
        profiles = self.profiles()
        missing = estimate_survey_line(self.plan, profiles[:-1], self.config,
                                       self.fwhm, self.inspection)
        self.assertIn("survey_station_set_incomplete_or_duplicated", missing.reason_codes)
        profile = profiles[0]
        keep = np.abs(profile.positions_xy_m[:, 1] - 6.) > 1.5
        profiles[0] = SurveyProfile(0, profile.positions_xy_m[keep],
                                    profile.envelope_nt[keep], profile.noise_nt)
        gapped = estimate_survey_line(self.plan, profiles, self.config,
                                      self.fwhm, self.inspection)
        self.assertIn("station_0_spatial_gap", gapped.reason_codes)

    def test_boundary_and_prior_envelope_are_explicit(self) -> None:
        with self.assertRaisesRegex(ValueError, "boundary/stop margin"):
            build_corridor_survey(np.array([[0., 0.], [60., 0.]]),
                                  self.boundary + np.array([0., -6.]), self.config)
        concave = np.array([[-10., -24.], [75., -24.], [75., 24.], [30., 0.], [-10., 24.]])
        with self.assertRaisesRegex(ValueError, "convex"):
            build_corridor_survey(np.array([[0., 0.], [60., 0.]]), concave, self.config)
        narrow_heading = replace(self.config, maximum_heading_deviation_deg=8.)
        result = estimate_survey_line(self.plan, self.profiles(shift_m=0., slope=.2),
                                      narrow_heading, self.fwhm, self.inspection)
        self.assertFalse(result.geometry_valid)
        self.assertIn("survey_heading_outside_prior_envelope", result.reason_codes)

    def test_original_right_angle_route_fails_representative_turn_radius(self) -> None:
        assessment = assess_turn_feasibility(self.plan, 50.)
        self.assertFalse(assessment.straight_run_sufficient)
        self.assertIn("survey_waypoint_turns_exceed_straight_runs", assessment.reason_codes)
        self.assertGreater(assessment.required_tangent_m[0], assessment.available_segment_m[0])


class ShallowZigzagTests(unittest.TestCase):
    def setUp(self) -> None:
        self.zigzag = ShallowZigzagConfig(waypoint_spacing_m=.5)
        self.total_m = self.zigzag.station_count * self.zigzag.half_wavelength_m
        self.boundary = np.array([[-10., -42.], [self.total_m + 10., -42.],
                                  [self.total_m + 10., 42.], [-10., 42.]])
        self.plan = build_shallow_zigzag_survey(np.array([[0., 0.], [self.total_m, 0.]]),
                                                self.boundary, self.zigzag)
        self.quality = CorridorSurveyConfig(
            half_width_m=self.zigzag.amplitude_m, station_spacing_m=self.zigzag.half_wavelength_m,
            maximum_along_excursion_m=self.zigzag.half_wavelength_m + .1,
        )
        self.fwhm = FwhmConfig(shape_model="thesis_dipole", altitude_m=0.)
        self.inspection = InspectionConfig(enabled=True)

    def profiles(self, *, seed: int = 101, transient: bool = False,
                 neighbor_m: float | None = None) -> list[SurveyProfile]:
        rng = np.random.default_rng(seed)
        profiles = []
        for idx, (start, stop) in enumerate(self.plan.profile_index_ranges):
            points = self.plan.waypoints_xy_m[start:stop]
            x, y = points[:, 0], points[:, 1]
            true_y = 6. + .02 * x
            if transient and idx == 1:
                true_y = -5. + .02 * x
            anomaly = 20. / np.sqrt(1. + ((y - true_y) / 3.) ** 2)
            if neighbor_m is not None:
                anomaly += 20. / np.sqrt(1. + ((y - true_y - neighbor_m) / 3.) ** 2)
            profiles.append(SurveyProfile(idx, points, 2. + anomaly + rng.normal(0., .02, len(x)), .02))
        return profiles

    def test_smooth_reference_and_oblique_centres_recover_local_line(self) -> None:
        radius = self.zigzag.half_wavelength_m ** 2 / (self.zigzag.amplitude_m * np.pi ** 2)
        self.assertGreater(radius, self.zigzag.minimum_turn_radius_m)
        self.assertEqual(self.plan.path_kind, "shallow_cosine_zigzag")
        self.assertEqual(len(self.plan.profile_index_ranges), 3)
        result = estimate_survey_line(self.plan, self.profiles(), self.quality,
                                      self.fwhm, self.inspection)
        self.assertTrue(result.geometry_valid, result.reason_codes)
        expected = 6. + .02 * result.station_centers_xy_m[:, 0]
        self.assertLess(float(np.mean(np.abs(result.station_centers_xy_m[:, 1] - expected))), .15)
        self.assertAlmostEqual(result.heading_deviation_deg, np.degrees(np.arctan(.02)), delta=.3)
        self.assertIsNone(result.burial_depth_m)

    def test_mirrored_start_side_keeps_curvature_and_line_geometry(self) -> None:
        mirrored = build_shallow_zigzag_survey(
            np.array([[0., 0.], [self.total_m, 0.]]), self.boundary,
            replace(self.zigzag, initial_lateral_sign=1),
        )
        self.assertAlmostEqual(mirrored.estimated_distance_m, self.plan.estimated_distance_m)
        np.testing.assert_allclose(mirrored.waypoints_xy_m[:, 1],
                                   -self.plan.waypoints_xy_m[:, 1])
        profiles = []
        for idx, (start, stop) in enumerate(mirrored.profile_index_ranges):
            positions = mirrored.waypoints_xy_m[start:stop]
            truth = 6. + .02 * positions[:, 0]
            envelope = 2. + 20. / np.sqrt(1. + ((positions[:, 1] - truth) / 3.) ** 2)
            profiles.append(SurveyProfile(idx, positions, envelope, .02))
        result = estimate_survey_line(mirrored, profiles, self.quality,
                                      self.fwhm, self.inspection)
        self.assertTrue(result.geometry_valid, result.reason_codes)
        self.assertLess(float(np.mean(np.abs(result.station_centers_xy_m[:, 1]
                                            - (6. + .02 * result.station_centers_xy_m[:, 0])))), .15)

    def test_shallow_crossing_rejects_transient_and_separated_neighbor(self) -> None:
        transient = estimate_survey_line(self.plan, self.profiles(transient=True),
                                         self.quality, self.fwhm, self.inspection)
        self.assertFalse(transient.geometry_valid)
        separated = estimate_survey_line(self.plan, self.profiles(neighbor_m=7.),
                                         self.quality, self.fwhm, self.inspection)
        self.assertFalse(separated.geometry_valid)

    def test_coverage_curvature_and_boundary_are_preflight_constraints(self) -> None:
        with self.assertRaisesRegex(ValueError, "amplitude cannot cover"):
            build_shallow_zigzag_survey(np.array([[0., 0.], [self.total_m, 0.]]),
                                        self.boundary, replace(self.zigzag, amplitude_m=18.))
        with self.assertRaisesRegex(ValueError, "curvature exceeds"):
            build_shallow_zigzag_survey(np.array([[0., 0.], [self.total_m, 0.]]),
                                        self.boundary, replace(self.zigzag, minimum_turn_radius_m=100.))
        with self.assertRaisesRegex(ValueError, "boundary/stop margin"):
            build_shallow_zigzag_survey(np.array([[0., 0.], [self.total_m, 0.]]),
                                        self.boundary + np.array([0., -17.]), self.zigzag)

    def test_start_side_requires_bounded_crosscurrent_direction(self) -> None:
        tangent = np.array([1., 0.])
        self.assertEqual(select_shallow_start_side(tangent, np.array([0., .04]), .005).initial_lateral_sign, -1)
        self.assertEqual(select_shallow_start_side(tangent, np.array([0., -.04]), .005).initial_lateral_sign, 1)
        self.assertIsNone(select_shallow_start_side(tangent, np.array([0., -.04]), .01).initial_lateral_sign)
        self.assertEqual(select_shallow_start_side(tangent, np.array([0., 0.]), .005).initial_lateral_sign, -1)
        self.assertIn("survey_crosscurrent_direction_ambiguous",
                      select_shallow_start_side(tangent, np.array([0., .01]), .01).reason_codes)


if __name__ == "__main__":
    unittest.main()
