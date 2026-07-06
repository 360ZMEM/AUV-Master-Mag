import sys
from pathlib import Path

import numpy as np

SRC_ROOT = Path(__file__).resolve().parents[1] / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.api import (  # noqa: E402
    AuvMagTrackingPipeline,
    CableMap,
    DeploymentPerceptionConfig,
    MagneticInput,
    NavigationInput,
)
from auv_mag_tracking.config import build_default_scenarios  # noqa: E402


def _cable_map() -> CableMap:
    return CableMap(
        points_xy_m=np.array([[0.0, 0.0], [100.0, 0.0]], dtype=float),
        burial_depth_m=1.5,
    )


def _config(*, online: bool) -> DeploymentPerceptionConfig:
    return DeploymentPerceptionConfig(
        enable_online_prior_alignment=online,
        magnetic_noise_floor_nt=1.0,
        burial_min_samples=1,
        confidence_min_ready=0.2,
        route_offset_ready_m=2.0,
    )


def _pipeline(*, online: bool) -> AuvMagTrackingPipeline:
    scenarios = build_default_scenarios()
    return AuvMagTrackingPipeline(scenarios["case1"], _cable_map(), deployment_config=_config(online=online))


def test_disabled_online_alignment_is_bit_for_bit_regression():
    """Default (disabled) path must not touch the projection cache or diagnostics."""
    pipeline = _pipeline(online=False)
    baseline_starts = pipeline._cache.segment_starts_xy.copy()

    nav = NavigationInput(time_s=1.0, position_ned_m=np.array([10.0, 0.5, -5.0]), heading_deg=0.0)
    for i in range(30):
        magnetic = MagneticInput(
            time_s=1.0 + 0.1 * i,
            sample_block_nt=np.full((8, 3), 90.0),
            sample_rate_hz=10.0,
        )
        out = pipeline.step(nav, magnetic)
        assert "prior_alignment_online" not in out.diagnostics
        assert "prior_alignment_connected" not in out.diagnostics

    # Cache never mutated when disabled.
    assert np.array_equal(pipeline._cache.segment_starts_xy, baseline_starts)

    # cross_track equals the raw map projection distance (unchanged contract).
    from auv_mag_tracking.math_utils import nearest_point_on_polyline

    _, _, dist_m, _, _ = nearest_point_on_polyline(np.array([10.0, 0.5]), pipeline._cache)
    assert abs(out.cross_track_m - dist_m) < 1e-9


def test_online_alignment_accumulates_correction_toward_magnetic_observation():
    """Enabled path derives an independent cross-track observation and shifts the prior."""
    pipeline = _pipeline(online=True)
    d = pipeline._vertical_separation_m
    assert d > 0.0

    # Vehicle sits on the (undistorted-in-test) prior line y=0, but the magnetic
    # anomaly ratio reports the true cable is offset on the +normal side.
    target_offset_m = 8.0
    slope = target_offset_m / d  # y/d == B_down/B_perp for the line-current model
    nav = NavigationInput(time_s=0.0, position_ned_m=np.array([10.0, 0.0, -5.0]), heading_deg=0.0)

    last = None
    translation_history = []
    rng = np.random.default_rng(0)
    for i in range(30):
        # Cable along +x -> normal = (0, 1): anomaly_y == B_perp, anomaly_z == B_down.
        # Vary magnitude so the (B_perp, B_down) scatter traces the ray of slope y/d.
        scale = 1.0 + 0.2 * float(rng.uniform(-1.0, 1.0))
        b_perp = 100.0 * scale
        b_down = slope * b_perp
        block = np.tile(np.array([0.0, b_perp, b_down]), (8, 1))
        magnetic = MagneticInput(time_s=0.1 * i, sample_block_nt=block, sample_rate_hz=10.0)
        last = pipeline.step(nav, magnetic)
        translation_history.append(last.diagnostics.get("prior_alignment_translation_norm_m", 0.0))

    assert last is not None
    assert last.diagnostics["prior_alignment_online"] is True
    assert last.diagnostics["prior_alignment_observed"] is True
    # The cross-track quality gate must open on the clean ratio.
    assert last.diagnostics["prior_alignment_cross_track_quality"] > 0.985
    # Observed offset recovers the injected value (line-current model is exact here).
    assert abs(last.diagnostics["prior_alignment_observed_offset_m"] - target_offset_m) < 0.5
    # Correction accumulated and the prior was pulled toward the observation.
    assert last.diagnostics["prior_alignment_translation_norm_m"] > 0.5
    assert last.diagnostics["prior_alignment_accepted"] is True
    assert translation_history[-1] >= translation_history[5]

    # The corrected prior's nearest point moved to the -normal side (toward true cable).
    from auv_mag_tracking.math_utils import nearest_point_on_polyline

    point_xy, _, _, _, _ = nearest_point_on_polyline(np.array([10.0, 0.0]), pipeline._cache)
    assert point_xy[1] < -0.5


def test_online_alignment_reset_restores_base_prior():
    pipeline = _pipeline(online=True)
    d = pipeline._vertical_separation_m
    slope = 8.0 / d
    nav = NavigationInput(time_s=0.0, position_ned_m=np.array([10.0, 0.0, -5.0]), heading_deg=0.0)
    for i in range(30):
        block = np.tile(np.array([0.0, 100.0 * (1.0 + 0.1 * ((i % 5) - 2)), slope * 100.0 * (1.0 + 0.1 * ((i % 5) - 2))]), (8, 1))
        pipeline.step(nav, MagneticInput(time_s=0.1 * i, sample_block_nt=block, sample_rate_hz=10.0))

    moved = pipeline._cache.segment_starts_xy.copy()
    pipeline.reset()
    base = pipeline._cache.segment_starts_xy
    # Reset rebuilds from the base route; corrected cache differs from reset cache.
    assert not np.allclose(moved, base)
    assert np.allclose(base, np.array([[0.0, 0.0]]))
