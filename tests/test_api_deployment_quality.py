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


def _pipeline(min_samples: int = 3) -> AuvMagTrackingPipeline:
    scenarios = build_default_scenarios()
    cable_map = CableMap(
        points_xy_m=np.array([[0.0, 0.0], [100.0, 0.0]], dtype=float),
        burial_depth_m=1.5,
    )
    config = DeploymentPerceptionConfig(
        magnetic_noise_floor_nt=1.0,
        burial_coupling_constant_nt_m_per_a_rms=11.4329,
        burial_current_rms_a=100.0,
        burial_altitude_m=6.0,
        burial_snr_gate_db=6.0,
        burial_min_strength_nt=1.0,
        burial_min_samples=min_samples,
        burial_max_lateral_offset_m=1.0,
        confidence_min_ready=0.2,
        route_offset_ready_m=2.0,
    )
    return AuvMagTrackingPipeline(scenarios["case1"], cable_map, deployment_config=config)


def test_deployment_quality_makes_confidence_dynamic():
    pipeline = _pipeline()
    nav = NavigationInput(time_s=1.0, position_ned_m=np.array([10.0, 0.2, -5.0]), heading_deg=0.0)
    weak = pipeline.step(
        nav,
        MagneticInput(time_s=1.0, sample_block_nt=np.full((8, 3), 0.05), sample_rate_hz=10.0),
    )
    strong = pipeline.step(
        nav,
        MagneticInput(time_s=2.0, sample_block_nt=np.full((8, 3), 90.0), sample_rate_hz=10.0),
    )

    assert weak.confidence != strong.confidence
    assert weak.diagnostics["deployment_quality_connected"] is True
    assert "weak_magnetic_signal" in weak.diagnostics["quality_flags"]
    assert strong.diagnostics["magnetic_used"] is True


def test_deployment_quality_outputs_burial_sigma_after_warmup():
    pipeline = _pipeline(min_samples=2)
    nav = NavigationInput(time_s=1.0, position_ned_m=np.array([10.0, 0.2, -5.0]), heading_deg=0.0)
    magnetic = MagneticInput(
        time_s=1.0,
        sample_block_nt=np.array([[152.0, 0.0, 0.0], [153.0, 0.0, 0.0], [151.0, 0.0, 0.0]]),
        sample_rate_hz=10.0,
    )

    first = pipeline.step(nav, magnetic)
    second = pipeline.step(nav, magnetic)

    assert first.burial_sigma_m is None
    assert second.burial_sigma_m is not None
    assert second.diagnostics["burial_status"] == "ready"
    assert second.diagnostics["burial_sample_count"] >= 2


def test_external_magnetic_quality_flags_reduce_confidence():
    nav = NavigationInput(time_s=1.0, position_ned_m=np.array([10.0, 0.2, -5.0]), heading_deg=0.0)
    clean_pipeline = _pipeline(min_samples=1)
    flagged_pipeline = _pipeline(min_samples=1)
    clean = clean_pipeline.step(
        nav,
        MagneticInput(time_s=1.0, sample_block_nt=np.full((8, 3), 90.0), sample_rate_hz=10.0),
    )
    flagged = flagged_pipeline.step(
        nav,
        MagneticInput(
            time_s=1.0,
            sample_block_nt=np.full((8, 3), 90.0),
            sample_rate_hz=10.0,
            quality_flags={"saturated": True, "calibration_valid": False},
        ),
    )

    assert flagged.confidence < clean.confidence
    assert "sensor_saturated" in flagged.diagnostics["quality_flags"]
    assert "weak_magnetic_signal" in flagged.diagnostics["quality_flags"]


def test_step_with_guidance_reports_quality_connected():
    pipeline = _pipeline(min_samples=1)
    nav = NavigationInput(time_s=1.0, position_ned_m=np.array([10.0, 0.2, -5.0]), heading_deg=0.0)
    magnetic = MagneticInput(time_s=1.0, sample_block_nt=np.full((8, 3), 90.0), sample_rate_hz=10.0)

    _tracking, guidance = pipeline.step_with_guidance(nav, magnetic, target_depth_m=12.0, speed_mps=0.8)

    assert guidance.diagnostics["deployment_quality_connected"] is True
    assert guidance.diagnostics["full_perception_stack_connected"] is True
