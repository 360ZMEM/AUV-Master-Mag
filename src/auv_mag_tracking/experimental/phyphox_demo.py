"""Standalone demo entrypoint for the Phyphox magnetometer adapter."""

import argparse
import sys
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.experimental.phyphox_adapter import run_demo


def parse_args() -> argparse.Namespace:
    """解析 Phyphox 演示所需的命令行参数。"""
    parser = argparse.ArgumentParser(description="Live Phyphox magnetometer adapter demo")
    parser.add_argument("--phone-ip", required=True, help="Phyphox phone IP address")
    parser.add_argument("--port", type=int, default=8080, help="Phyphox HTTP port")
    parser.add_argument("--sample-rate-hz", type=float, default=20.0, help="Polling frequency for the demo loop")
    parser.add_argument("--duration-s", type=float, default=30.0, help="Demo duration in seconds")
    parser.add_argument("--endpoint-path", default="/get?magX&magY&magZ&accX&accY&accZ", help="Phyphox endpoint path")
    parser.add_argument("--timeout-s", type=float, default=1.5, help="HTTP timeout in seconds")
    parser.add_argument("--calibration-seconds", type=float, default=3.0, help="Initial DC calibration window")
    parser.add_argument("--lowpass-window-seconds", type=float, default=0.35, help="Sliding-average window length")
    parser.add_argument("--no-viz", action="store_true", help="Disable matplotlib and print samples instead")
    return parser.parse_args()


def main() -> int:
    """程序入口，组装参数并启动演示循环。"""
    args = parse_args()
    return run_demo(
        phone_ip=args.phone_ip,
        port=args.port,
        sample_rate_hz=args.sample_rate_hz,
        duration_s=args.duration_s,
        endpoint_path=args.endpoint_path,
        no_viz=args.no_viz,
        timeout_s=args.timeout_s,
        calibration_seconds=args.calibration_seconds,
        lowpass_window_seconds=args.lowpass_window_seconds,
    )


if __name__ == "__main__":
    raise SystemExit(main())