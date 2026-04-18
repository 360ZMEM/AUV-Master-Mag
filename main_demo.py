"""Root entrypoint for the AUV magnetic cable tracking demo."""

import argparse
import copy
import sys
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parent
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.config import build_default_scenarios
from auv_mag_tracking.main_viz import AuvCableTrackingSimulation
from auv_mag_tracking.simulator_connector import build_connector


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="AUV magnetic cable tracking simulation demo")
    parser.add_argument("--case", default="case1", help="Scenario name, e.g. case1/case2/case3/case4/case5/case6/case_hf_phone/case_hf_industrial")
    parser.add_argument("--list-cases", action="store_true", help="List all available scenarios and exit")
    parser.add_argument("--no-viz", action="store_true", help="Run the simulation without live matplotlib animation")
    parser.add_argument("--headless", action="store_true", help="Alias for --no-viz")
    parser.add_argument(
        "--magnetometer-mode",
        choices=["auto", "standard", "high-fidelity"],
        default="auto",
        help="Select the magnetometer backend. auto uses the scenario default.",
    )
    parser.add_argument("--hf-sampling-rate-hz", type=float, default=None, help="Override the high-fidelity magnetometer sampling rate.")
    parser.add_argument("--hf-bit-depth", type=int, default=None, help="Override the high-fidelity magnetometer bit depth.")
    parser.add_argument("--connector", choices=["none", "mock"], default="none", help="Reserve a simulator connector backend for future HoloOcean integration.")
    parser.add_argument("--phyphox-ip", default=None, help="Run the Phyphox hardware adapter demo against this phone IP instead of the simulator.")
    parser.add_argument("--phyphox-port", type=int, default=8080, help="Phyphox HTTP port.")
    parser.add_argument("--phyphox-sample-rate-hz", type=float, default=20.0, help="Polling rate used by the hardware demo loop.")
    parser.add_argument("--phyphox-duration-s", type=float, default=30.0, help="Duration of the hardware demo in seconds.")
    parser.add_argument("--phyphox-endpoint-path", default="/get?magX&magY&magZ&accX&accY&accZ", help="Phyphox endpoint path.")
    parser.add_argument("--phyphox-timeout-s", type=float, default=1.5, help="HTTP timeout used by the hardware adapter.")
    parser.add_argument("--phyphox-calibration-seconds", type=float, default=3.0, help="Initial DC calibration window for the adapter.")
    parser.add_argument("--phyphox-lowpass-window-seconds", type=float, default=0.35, help="Sliding-average window length for the adapter.")
    parser.add_argument("--phyphox-no-viz", action="store_true", help="Disable matplotlib for the hardware demo and print samples instead.")
    parser.add_argument("--deployment-mode", action="store_true", help="Disable nominal-route priors for a deployment-safe configuration.")
    parser.add_argument(
        "--sonar-mode",
        choices=["auto", "off", "degraded", "reliable_absence"],
        default="auto",
        help="Override the sonar behavior. auto keeps the scenario default.",
    )
    return parser.parse_args()


def configure_high_fidelity_mode(scenario, args: argparse.Namespace):
    if args.magnetometer_mode == "standard":
        scenario.sensor.high_fidelity.enabled = False
        return scenario

    if args.magnetometer_mode == "high-fidelity":
        scenario.sensor.high_fidelity.enabled = True

    if scenario.sensor.high_fidelity.enabled:
        if args.hf_sampling_rate_hz is not None:
            scenario.sensor.high_fidelity.sampling_rate_hz = args.hf_sampling_rate_hz
        if args.hf_bit_depth is not None:
            scenario.sensor.high_fidelity.bit_depth = args.hf_bit_depth
        scenario.sensor.magnetometer_sample_rate_hz = scenario.sensor.high_fidelity.sampling_rate_hz
    return scenario


def configure_deployment_mode(scenario, args: argparse.Namespace):
    if args.deployment_mode:
        scenario.tracking.use_nominal_route_prior = False
        scenario.tracking.spiral_max_radius_m = max(scenario.tracking.spiral_max_radius_m, 60.0)
    return scenario


def configure_sonar_mode(scenario, args: argparse.Namespace):
    if args.sonar_mode != "auto":
        scenario.sonar.mode = args.sonar_mode
    return scenario


def main() -> int:
    args = parse_args()
    if args.phyphox_ip:
        from auv_mag_tracking.tools.phyphox_adapter import run_demo

        return run_demo(
            phone_ip=args.phyphox_ip,
            port=args.phyphox_port,
            sample_rate_hz=args.phyphox_sample_rate_hz,
            duration_s=args.phyphox_duration_s,
            endpoint_path=args.phyphox_endpoint_path,
            no_viz=args.phyphox_no_viz or args.no_viz,
            timeout_s=args.phyphox_timeout_s,
            calibration_seconds=args.phyphox_calibration_seconds,
            lowpass_window_seconds=args.phyphox_lowpass_window_seconds,
        )

    scenarios = build_default_scenarios()
    if args.list_cases:
        for name, scenario in scenarios.items():
            print(f"{name}: {scenario.description}")
        return 0

    scenario = scenarios.get(args.case)
    if scenario is None:
        print(f"Unknown case: {args.case}")
        print("Use --list-cases to view the available scenarios.")
        return 2

    scenario = configure_sonar_mode(configure_deployment_mode(configure_high_fidelity_mode(copy.deepcopy(scenario), args), args), args)

    connector = build_connector(args.connector)
    if args.connector == "mock":
        connector.connect()

    simulation = AuvCableTrackingSimulation(scenario)
    report = simulation.run(enable_visualization=not (args.no_viz or args.headless))
    connector.disconnect()
    print(f"Case: {report.case_name}")
    print(f"Duration: {report.duration_s:.1f} s")
    print(f"Peaks captured: {report.peak_count}")
    print(f"Final confidence: {report.final_confidence:.2f}")
    print(f"Final controller mode: {report.final_mode}")
    print(f"Tracked distance: {report.tracked_distance_m:.1f} m")
    if report.cable_heading_error_deg is not None:
        print(f"[Deployment] Cable heading error: {report.cable_heading_error_deg:.1f} deg")
    if report.mean_lateral_deviation_m is not None:
        print(f"[Deployment] Mean lateral deviation: {report.mean_lateral_deviation_m:.2f} m")
    if report.along_track_coverage_ratio is not None:
        print(f"[Deployment] Along-track coverage: {report.along_track_coverage_ratio:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
