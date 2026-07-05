import numpy as np
import sys
from pathlib import Path

SRC_ROOT = Path(__file__).resolve().parents[1] / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.perception import CableMapFrameTracker


def test_map_frame_progress_advances_monotonically_on_straight_route():
    route_xy = np.array([[0.0, 0.0], [100.0, 0.0]], dtype=float)
    tracker = CableMapFrameTracker(route_xy, np.array([0.0, 2.0]), max_forward_step_m=5.0)

    progress = []
    for x_m in (1.0, 3.0, 6.0, 10.0):
        state = tracker.update(np.array([x_m, 2.0], dtype=float))
        progress.append(state.progress_m)

    assert np.all(np.diff(progress) > 0.0)
    assert abs(tracker.state.lateral_m - 2.0) < 1e-6
    assert tracker.state.projection_distance_m == 2.0


def test_map_frame_local_window_rejects_far_parallel_lane_shortcut():
    route_xy = np.array(
        [
            [0.0, 0.0],
            [100.0, 0.0],
            [100.0, 40.0],
            [0.0, 40.0],
        ],
        dtype=float,
    )
    tracker = CableMapFrameTracker(
        route_xy,
        np.array([10.0, 0.0], dtype=float),
        lookback_m=5.0,
        lookahead_m=20.0,
        max_forward_step_m=5.0,
    )

    state = tracker.update(np.array([20.0, 39.0], dtype=float))

    assert state.progress_m < 35.0
    assert state.projection_distance_m > 30.0
    assert state.consistency_score < 0.05


def test_map_frame_observation_correction_is_window_limited():
    route_xy = np.array([[0.0, 0.0], [100.0, 0.0], [100.0, 40.0]], dtype=float)
    tracker = CableMapFrameTracker(
        route_xy,
        np.array([10.0, 0.0], dtype=float),
        lookback_m=5.0,
        lookahead_m=15.0,
        max_forward_step_m=5.0,
        correction_gain=1.0,
    )

    state = tracker.update(
        np.array([12.0, 0.0], dtype=float),
        observation_xy=np.array([100.0, 35.0], dtype=float),
        observation_confidence=1.0,
    )

    assert 10.0 <= state.progress_m <= 30.0
    assert state.progress_m < 100.0
