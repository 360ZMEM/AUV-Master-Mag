from __future__ import annotations

from types import SimpleNamespace
import unittest

import numpy as np

from auv_mag_tracking.api import CableMap, MagneticInput, NavigationInput
from auv_mag_tracking.v2 import (
    AuvMagTrackingPipelineV2,
    CrossScanConfig,
    DetectionConfig,
    FwhmConfig,
    ShapeCalibration,
    SearchConfig,
    V2Config,
)


def _nav(time_s: float, x_m: float, y_m: float, heading_deg: float = 0.0) -> NavigationInput:
    return NavigationInput(
        time_s=time_s,
        position_ned_m=np.array([x_m, y_m, -2.0]),
        heading_deg=heading_deg,
        speed_mps=0.4,
    )


def _mag(time_s: float, envelope_nt: float) -> MagneticInput:
    return MagneticInput(
        time_s=time_s,
        sample_block_nt=np.tile(np.array([[envelope_nt, 0.0, 0.0]]), (4, 1)),
        sample_rate_hz=50.0,
        sensor_frame="ned",
    )


class PipelineV2Tests(unittest.TestCase):
    def test_opt_in_depth_calibration_policy_rejects_simulation_provenance(self) -> None:
        cable_map = CableMap(np.array([[0.0, 0.0], [20.0, 0.0]]), burial_depth_m=None)
        config = FwhmConfig(altitude_m=0.0, minimum_prominence_nt=0.1)
        simulated = ShapeCalibration(
            "thesis_dipole", 0.5, 0.02, 10, 10, 1.0, 0.01, 0.03,
            True, (), "simulation",
        )
        pipeline = AuvMagTrackingPipelineV2(
            object(), cable_map, v2_config=V2Config(fwhm=config),
            depth_calibrations=((config, simulated),),
        )
        x = np.linspace(-12, 12, 241)
        pipeline._scan_x_m.extend(x)
        pipeline._scan_envelope_nt.extend(2.0 + 20.0 / (1.0 + (x / 3.0) ** 2))
        pipeline._scan_noise_nt.extend([0.02] * len(x))
        result = pipeline._estimate_current_scan(0.0)
        self.assertFalse(result.valid)
        self.assertIn("no_accepted_shape_calibration", result.reason_codes)
        self.assertIsNone(result.burial_depth_m)

    def test_search_confirm_fwhm_track_chain(self) -> None:
        cable_map = CableMap(np.array([[0.0, 0.0], [20.0, 0.0]]), burial_depth_m=None)
        config = V2Config(
            search=SearchConfig(search_width_m=8.0, lane_spacing_m=4.0),
            detection=DetectionConfig(
                window_samples=30,
                min_ready_samples=10,
                threshold_on=5.0,
                threshold_off=2.0,
                consecutive_on=2,
                consecutive_off=3,
                minimum_noise_nt=0.05,
            ),
            cross_scan=CrossScanConfig(
                half_span_levels_m=(3.0, 5.0, 7.0),
                velocity_mps=0.4,
                arrival_tolerance_m=0.01,
            ),
            fwhm=FwhmConfig(
                minimum_samples=21,
                smoothing_window=3,
                minimum_prominence_nt=0.1,
                minimum_snr_db=3.0,
                altitude_m=0.5,
            ),
        )
        pipeline = AuvMagTrackingPipelineV2(object(), cable_map, v2_config=config)

        time_s = 0.0
        for _ in range(12):
            output = pipeline.step(_nav(time_s, 5.0, -4.0), _mag(time_s, 1.0))
            time_s += 0.1
        self.assertEqual(output.mode, "search")

        output = pipeline.step(_nav(time_s, 5.0, 0.0), _mag(time_s, 11.0))
        time_s += 0.1
        output = pipeline.step(_nav(time_s, 5.0, 0.0), _mag(time_s, 11.0))
        time_s += 0.1
        self.assertEqual(output.mode, "confirm")

        first_waypoint = np.asarray(pipeline._scan_plan.waypoints_xy_m[0])
        output = pipeline.step(
            _nav(time_s, first_waypoint[0], first_waypoint[1], 90.0),
            _mag(time_s, 1.0 + 10.0 / (1.0 + (first_waypoint[1] / 1.5) ** 2)),
        )
        time_s += 0.1
        self.assertEqual(output.diagnostics["cross_scan_waypoint_index"], 1)

        for y_m in np.linspace(first_waypoint[1], -first_waypoint[1], 61):
            envelope = 1.0 + 10.0 / (1.0 + (y_m / 1.5) ** 2)
            output = pipeline.step(_nav(time_s, 5.0, y_m, 90.0), _mag(time_s, envelope))
            time_s += 0.1

        self.assertEqual(output.mode, "confirm")
        return_waypoint = np.asarray(pipeline._scan_plan.waypoints_xy_m[2])
        output = pipeline.step(
            _nav(time_s, return_waypoint[0], return_waypoint[1], -90.0),
            _mag(time_s, 11.0),
        )
        time_s += 0.1

        self.assertEqual(output.mode, "track")
        self.assertTrue(output.diagnostics["fwhm_valid"], output.diagnostics["fwhm_reason_codes"])
        self.assertAlmostEqual(output.diagnostics["fwhm_width_m"], 3.0, delta=0.12)
        self.assertAlmostEqual(output.burial_depth_m, 1.0, delta=0.08)
        self.assertTrue(output.diagnostics["observation_only_available"])
        self.assertEqual(output.diagnostics["source"], "fwhm_observation")
        self.assertIn("center_returned", output.diagnostics["cross_scan_completion_reason"])

        # TRACK first commands a perpendicular return to the estimated cable,
        # then switches to longitudinal lookahead after the line is rejoined.
        output, guidance = pipeline.step_with_guidance(
            _nav(time_s, 10.0, 4.0), _mag(time_s, 1.0)
        )
        np.testing.assert_allclose(output.diagnostics["tracking_target_xy_m"], [10.0, 0.0], atol=0.1)
        self.assertEqual(output.diagnostics["tracking_stage"], "rejoin_estimated_cable")
        self.assertAlmostEqual(abs(guidance.desired_heading_deg), 90.0, delta=0.1)

        output, guidance = pipeline.step_with_guidance(
            _nav(time_s + 0.05, 10.0, 0.0), _mag(time_s + 0.05, 1.0)
        )
        np.testing.assert_allclose(output.diagnostics["tracking_target_xy_m"], [18.0, 0.0], atol=0.1)
        self.assertEqual(output.diagnostics["tracking_stage"], "inspect_along_cable")

        # A local observation must correct the finite prior corridor rather
        # than create an infinite straight line beyond the cable endpoint.
        output, guidance = pipeline.step_with_guidance(
            _nav(time_s, 30.0, -3.0), _mag(time_s, 1.0)
        )
        np.testing.assert_allclose(output.estimated_cable_xy_m, [20.0, 0.0], atol=0.1)
        np.testing.assert_allclose(output.diagnostics["tracking_target_xy_m"], [20.0, 0.0], atol=0.1)
        self.assertEqual(guidance.guidance_source, "v2_observation_corrected_route")
        self.assertGreater(abs(guidance.desired_heading_deg), 90.0)

        output, guidance = pipeline.step_with_guidance(
            _nav(time_s + 0.1, 20.0, 0.0), _mag(time_s + 0.1, 1.0)
        )
        self.assertTrue(output.diagnostics["endpoint_complete"])
        self.assertEqual(guidance.guidance_source, "v2_endpoint_hold")
        self.assertEqual(guidance.speed_mps, 0.0)

    def test_step_with_guidance_preserves_public_output_types(self) -> None:
        cable_map = CableMap(np.array([[0.0, 0.0], [20.0, 0.0]]))
        pipeline = AuvMagTrackingPipelineV2(object(), cable_map)
        output, guidance = pipeline.step_with_guidance(_nav(0.0, 0.0, 0.0), _mag(0.0, 1.0))
        self.assertEqual(output.diagnostics["algorithm_version"], "v2")
        self.assertEqual(guidance.diagnostics["algorithm_version"], "v2")
        self.assertEqual(output.mode, "search")

    def test_dynamic_altitude_is_read_without_changing_v1_input_contract(self) -> None:
        cable_map = CableMap(np.array([[0.0, 0.0], [20.0, 0.0]]))
        pipeline = AuvMagTrackingPipelineV2(object(), cable_map)
        output = pipeline.step(
            _nav(0.0, 0.0, 0.0),
            _mag(0.0, 1.0),
            SimpleNamespace(valid=True, altitude_m=4.25),
        )
        self.assertEqual(output.diagnostics["fwhm_altitude_source"], "sonar")
        self.assertEqual(output.diagnostics["fwhm_altitude_m"], 4.25)

    def test_cross_scan_rejects_excessive_along_track_excursion(self) -> None:
        cable_map = CableMap(np.array([[0.0, 0.0], [20.0, 0.0]]))
        config = V2Config(
            search=SearchConfig(search_width_m=8.0, lane_spacing_m=4.0),
            detection=DetectionConfig(
                window_samples=30, min_ready_samples=10,
                threshold_on=5.0, threshold_off=2.0,
                consecutive_on=2, consecutive_off=3,
                minimum_noise_nt=0.05,
            ),
            cross_scan=CrossScanConfig(
                half_span_levels_m=(3.0,), arrival_tolerance_m=0.01,
                maximum_along_track_range_m=1.0,
                early_complete_when_valid=False,
            ),
            fwhm=FwhmConfig(
                minimum_samples=21, smoothing_window=3,
                minimum_prominence_nt=0.1, minimum_snr_db=3.0,
                altitude_m=0.0,
            ),
        )
        pipeline = AuvMagTrackingPipelineV2(object(), cable_map, v2_config=config)
        time_s = 0.0
        for _ in range(12):
            pipeline.step(_nav(time_s, 5.0, -4.0), _mag(time_s, 1.0)); time_s += 0.1
        pipeline.step(_nav(time_s, 5.0, 0.0), _mag(time_s, 11.0)); time_s += 0.1
        pipeline.step(_nav(time_s, 5.0, 0.0), _mag(time_s, 11.0)); time_s += 0.1
        first, second = pipeline._scan_plan.waypoints_xy_m[:2]
        pipeline.step(_nav(time_s, *first, 90.0), _mag(time_s, 1.0)); time_s += 0.1
        for index, point in enumerate(np.linspace(first, second, 61)):
            x_drift = 5.0 + 3.0 * index / 60.0
            envelope = 1.0 + 10.0 / (1.0 + (point[1] / 1.5) ** 2)
            output = pipeline.step(
                _nav(time_s, x_drift, point[1], 90.0), _mag(time_s, envelope)
            )
            time_s += 0.1
        self.assertEqual(output.mode, "reacquire")
        self.assertFalse(output.diagnostics["fwhm_valid"])
        self.assertIn("along_track_excursion_excessive", output.diagnostics["fwhm_reason_codes"])

    def test_invalid_scan_retries_about_original_detection_center(self) -> None:
        cable_map = CableMap(np.array([[0.0, 0.0], [20.0, 0.0]]))
        config = V2Config(
            search=SearchConfig(search_width_m=8.0, lane_spacing_m=4.0),
            detection=DetectionConfig(
                window_samples=30,
                min_ready_samples=10,
                threshold_on=5.0,
                threshold_off=2.0,
                consecutive_on=2,
                consecutive_off=2,
                minimum_noise_nt=0.05,
            ),
            cross_scan=CrossScanConfig(
                half_span_levels_m=(1.0, 2.0, 3.0), arrival_tolerance_m=0.01
            ),
            fwhm=FwhmConfig(minimum_samples=21, minimum_prominence_nt=0.5, altitude_m=0.0),
        )
        pipeline = AuvMagTrackingPipelineV2(object(), cable_map, v2_config=config)
        time_s = 0.0
        for _ in range(12):
            pipeline.step(_nav(time_s, 5.0, -4.0), _mag(time_s, 1.0))
            time_s += 0.1
        pipeline.step(_nav(time_s, 5.0, 0.0), _mag(time_s, 11.0)); time_s += 0.1
        pipeline.step(_nav(time_s, 5.0, 0.0), _mag(time_s, 11.0)); time_s += 0.1
        original_center = pipeline._scan_plan.center_xy_m.copy()
        first, second = pipeline._scan_plan.waypoints_xy_m[:2]
        pipeline.step(_nav(time_s, *first, 90.0), _mag(time_s, 1.0)); time_s += 0.1
        for point in np.linspace(first, second, 31):
            output = pipeline.step(_nav(time_s, *point, 90.0), _mag(time_s, 1.0))
            time_s += 0.1
        self.assertEqual(output.mode, "reacquire")
        self.assertTrue(output.diagnostics["cross_scan_retry_pending"])
        output = pipeline.step(_nav(time_s, *second), _mag(time_s, 1.0))
        self.assertEqual(output.mode, "confirm")
        self.assertEqual(output.diagnostics["cross_scan_attempt"], 1)
        np.testing.assert_allclose(pipeline._scan_plan.center_xy_m, original_center)
        self.assertAlmostEqual(pipeline._scan_plan.half_span_m, 2.0)


if __name__ == "__main__":
    unittest.main()
