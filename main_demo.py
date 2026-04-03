"""Root entrypoint for the AUV magnetic cable tracking demo."""

import argparse
import sys
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parent
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.config import build_default_scenarios
from auv_mag_tracking.main_viz import AuvCableTrackingSimulation


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="AUV magnetic cable tracking simulation demo")
    parser.add_argument("--case", default="case1", help="Scenario name, e.g. case1/case2/case3/case4/case5")
    parser.add_argument("--list-cases", action="store_true", help="List all available scenarios and exit")
    parser.add_argument("--no-viz", action="store_true", help="Run the simulation without live matplotlib animation")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
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

    simulation = AuvCableTrackingSimulation(scenario)
    report = simulation.run(enable_visualization=not args.no_viz)
    print(f"Case: {report.case_name}")
    print(f"Duration: {report.duration_s:.1f} s")
    print(f"Peaks captured: {report.peak_count}")
    print(f"Final confidence: {report.final_confidence:.2f}")
    print(f"Final controller mode: {report.final_mode}")
    print(f"Tracked distance: {report.tracked_distance_m:.1f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
