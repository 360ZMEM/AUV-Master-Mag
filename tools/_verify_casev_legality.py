"""One-off verification for case1v..case6v legality (Task A). Not committed."""
from __future__ import annotations

import sys
import warnings
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

import numpy as np  # noqa: E402

from auv_mag_tracking.config import build_default_scenarios  # noqa: E402
from auv_mag_tracking.environment import CableEnvironment  # noqa: E402
from auv_mag_tracking.viz.metrics import compute_health_metrics, health_score  # noqa: E402
from auv_mag_tracking.viz.recorder import simulate_run  # noqa: E402

CASES = ("case1v", "case2v", "case3v", "case4v", "case5v", "case6v")


def first_segment_is_straight(route_xy: np.ndarray, n: int = 20) -> bool:
    head = route_xy[: min(n, len(route_xy))]
    if len(head) < 3:
        return True
    deltas = np.diff(head, axis=0)
    headings = np.arctan2(deltas[:, 1], deltas[:, 0])
    return bool(np.ptp(np.rad2deg(headings)) < 2.0)


def main() -> None:
    scenarios = build_default_scenarios()
    print(f"{'case':8} {'warn':5} {'len_m':>8} {'straight0':>9} "
          f"{'health':>7} {'ep_goal':>7} {'ep_done':>7} {'route%':>7} "
          f"{'geometry':>8} {'jumps':>5} {'max_jump':>8} {'trk_xt':>7} verdict")
    for name in CASES:
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            scenario = scenarios[name]
            env = CableEnvironment(scenario)
            route = env.route.sample_xy()
            total_len = env.route.total_length_m
            curvature_warn = any("curvature" in str(w.message).lower() for w in caught)
        straight0 = first_segment_is_straight(route)
        metrics = compute_health_metrics(simulate_run(scenario))
        ep_goal = metrics.endpoint_goal_enabled >= 0.5
        endpoint = metrics.endpoint_completed >= 0.5
        geometry = metrics.maze_geometry_passed >= 0.5
        jumps = metrics.route_progress_large_jump_count
        route_pct = metrics.route_completion_ratio
        # Legal = geometry passes, no large jump, healthy tracking; endpoint only
        # gates when an endpoint goal is actually configured.
        endpoint_ok = endpoint if ep_goal else True
        verdict = "LEGAL" if (endpoint_ok and geometry and jumps == 0) else "CHECK"
        print(f"{name:8} {str(curvature_warn):5} {total_len:8.1f} {str(straight0):>9} "
              f"{health_score(metrics):7.1f} {str(ep_goal):>7} {str(endpoint):>7} "
              f"{route_pct:7.3f} {str(geometry):>8} "
              f"{jumps:5d} {metrics.route_progress_max_jump_m:8.1f} "
              f"{metrics.track_mean_cross_track_m:7.2f} {verdict}")


if __name__ == "__main__":
    main()
