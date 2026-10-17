from __future__ import annotations

import unittest

import numpy as np

from auv_mag_tracking.v2 import DetectionConfig, MagneticDetector, SearchConfig, build_prior_guided_search


class SearchPlannerTests(unittest.TestCase):
    def test_prior_corridor_generates_alternating_lanes(self) -> None:
        prior = np.array([[0.0, 0.0], [20.0, 0.0]])
        boundary = np.array([[-2.0, -6.0], [22.0, -6.0], [22.0, 6.0], [-2.0, 6.0]])
        plan = build_prior_guided_search(
            prior,
            SearchConfig(search_width_m=8.0, lane_spacing_m=4.0, waypoint_spacing_m=5.0),
            initial_position_xy_m=np.array([0.0, -4.0]),
            search_boundary_xy_m=boundary,
        )
        self.assertTrue(plan.valid, plan.reason_codes)
        self.assertEqual(plan.lane_offsets_m.size, 3)
        self.assertGreater(plan.estimated_distance_m, 60.0)
        self.assertTrue(np.all(plan.waypoints_xy_m[:, 0] >= -2.0))
        self.assertTrue(np.all(plan.waypoints_xy_m[:, 0] <= 22.0))

    def test_uncertainty_expands_requested_width(self) -> None:
        prior = np.array([[0.0, 0.0], [10.0, 0.0]])
        plan = build_prior_guided_search(
            prior,
            SearchConfig(search_width_m=4.0, lane_spacing_m=2.0),
            initial_position_uncertainty_m=4.0,
        )
        self.assertAlmostEqual(np.ptp(plan.lane_offsets_m), 8.0)

    def test_boundary_rejects_uncontained_corridor(self) -> None:
        prior = np.array([[0.0, 0.0], [10.0, 0.0]])
        boundary = np.array([[0.0, -1.0], [10.0, -1.0], [10.0, 1.0], [0.0, 1.0]])
        plan = build_prior_guided_search(
            prior,
            SearchConfig(search_width_m=6.0, lane_spacing_m=2.0),
            search_boundary_xy_m=boundary,
        )
        self.assertFalse(plan.valid)
        self.assertIn("corridor_outside_search_boundary", plan.reason_codes)


class DetectorTests(unittest.TestCase):
    def test_hysteretic_detection_and_release(self) -> None:
        detector = MagneticDetector(
            DetectionConfig(
                window_samples=40,
                min_ready_samples=10,
                threshold_on=5.0,
                threshold_off=2.0,
                consecutive_on=2,
                consecutive_off=3,
                minimum_noise_nt=0.1,
            )
        )
        time_s = 0.0
        for value in np.linspace(9.95, 10.05, 15):
            result = detector.update(time_s, value)
            time_s += 0.1
        self.assertFalse(result.detected)
        result = detector.update(time_s, 12.0)
        result = detector.update(time_s + 0.1, 12.1)
        self.assertTrue(result.detected)
        for idx in range(5):
            result = detector.update(time_s + 0.2 + idx * 0.1, 10.0)
        self.assertFalse(result.detected)


if __name__ == "__main__":
    unittest.main()

