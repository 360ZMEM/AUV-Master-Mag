import json
import sys
from pathlib import Path

import numpy as np
import pytest

SRC_ROOT = Path(__file__).resolve().parents[1] / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.api import (  # noqa: E402
    AuvMagTrackingPipeline,
    CableGuidanceOutput,
    CableMap,
    MagneticInput,
    NavigationInput,
    SonarInput,
    export_tracking_outputs,
    validate_cable_map_csv,
    validate_navigation_csv,
)
from auv_mag_tracking.config import build_default_scenarios  # noqa: E402


def test_cable_map_imports_csv_and_geojson(tmp_path):
    csv_path = tmp_path / "cable_map.csv"
    csv_path.write_text(
        "x_m,y_m,burial_depth_m\n0,0,1.2\n10,0,1.3\n",
        encoding="utf-8",
    )
    cable_map = CableMap.from_csv(csv_path)

    assert cable_map.points_xy_m.shape == (2, 2)
    assert np.allclose(cable_map.burial_depth_m, [1.2, 1.3])

    geojson_path = tmp_path / "cable_map.geojson"
    geojson_path.write_text(
        json.dumps({"type": "LineString", "coordinates": [[0, 0], [10, 0], [20, 5]]}),
        encoding="utf-8",
    )
    geojson_map = CableMap.from_geojson(geojson_path)

    assert geojson_map.points_xy_m.shape == (3, 2)


def test_cable_map_rejects_partial_burial_column(tmp_path):
    csv_path = tmp_path / "partial_burial.csv"
    csv_path.write_text("x_m,y_m,burial_depth_m\n0,0,1.2\n10,0,\n", encoding="utf-8")

    with pytest.raises(ValueError, match="burial_depth_m"):
        CableMap.from_csv(csv_path)


def test_schema_validators_report_missing_columns(tmp_path):
    cable_path = tmp_path / "cable.csv"
    cable_path.write_text("x_m,y_m\n0,0\n1,0\n", encoding="utf-8")
    assert validate_cable_map_csv(cable_path) == ["x_m", "y_m"]

    nav_path = tmp_path / "nav.csv"
    nav_path.write_text("time_s,position_x_m,heading_deg\n0,0,90\n", encoding="utf-8")
    with pytest.raises(ValueError, match="position_y_m"):
        validate_navigation_csv(nav_path)


def _pipeline() -> AuvMagTrackingPipeline:
    scenarios = build_default_scenarios()
    cable_map = CableMap(points_xy_m=np.array([[0.0, 0.0], [100.0, 0.0]], dtype=float), burial_depth_m=1.5)
    return AuvMagTrackingPipeline(scenarios["case1"], cable_map)


def test_pipeline_step_returns_map_projection_without_sonar():
    pipeline = _pipeline()
    output = pipeline.step(
        NavigationInput(time_s=1.0, position_ned_m=np.array([10.0, 2.0, -5.0]), heading_deg=0.0),
        MagneticInput(time_s=1.0, sample_block_nt=np.array([[1.0, 2.0, 3.0]]), sample_rate_hz=10.0),
    )

    assert np.allclose(output.estimated_cable_xy_m, [10.0, 0.0])
    assert output.cross_track_m == 2.0
    assert output.burial_depth_m == 1.5
    assert output.diagnostics["source"] == "map_projection"
    assert output.diagnostics["map_frame"] == "local_ned"
    assert output.diagnostics["magnetic_used"] is False


def test_pipeline_step_uses_valid_sonar_observation():
    pipeline = _pipeline()
    output = pipeline.step(
        NavigationInput(time_s=1.0, position_ned_m=np.array([10.0, 2.0, -5.0]), heading_deg=0.0),
        MagneticInput(time_s=1.0, sample_block_nt=np.array([[1.0, 2.0, 3.0]]), sample_rate_hz=10.0),
        SonarInput(time_s=1.0, relative_position_body_m=np.array([1.0, -2.0]), confidence=0.8, valid=True),
    )

    assert np.allclose(output.estimated_cable_xy_m, [11.0, 0.0])
    assert output.confidence == 0.8
    assert output.diagnostics["source"] == "sonar"


def test_pipeline_step_with_guidance_returns_controller_contract():
    pipeline = _pipeline()
    output, guidance = pipeline.step_with_guidance(
        NavigationInput(time_s=1.0, position_ned_m=np.array([10.0, 2.0, -5.0]), heading_deg=0.0),
        MagneticInput(time_s=1.0, sample_block_nt=np.array([[1.0, 2.0, 3.0]]), sample_rate_hz=10.0),
        target_depth_m=12.0,
        speed_mps=0.8,
    )

    assert isinstance(guidance, CableGuidanceOutput)
    assert output.diagnostics["guidance_source"] == "api_route_projection"
    assert guidance.target_depth_m == 12.0
    assert guidance.speed_mps == 0.8
    assert guidance.diagnostics["full_perception_stack_connected"] is False
    assert guidance.desired_heading_deg < 0.0


def test_export_tracking_outputs_writes_ops_files(tmp_path):
    output = _pipeline().step(
        NavigationInput(time_s=1.0, position_ned_m=np.array([10.0, 2.0, -5.0]), heading_deg=0.0),
        MagneticInput(time_s=1.0, sample_block_nt=np.array([[1.0, 2.0, 3.0]]), sample_rate_hz=10.0),
    )

    export_tracking_outputs([output], tmp_path)

    assert (tmp_path / "cable_ops_points.csv").exists()
    assert (tmp_path / "burial_profile.csv").exists()
    diagnostics = json.loads((tmp_path / "diagnostics.json").read_text(encoding="utf-8"))
    assert diagnostics["point_count"] == 1
    assert diagnostics["burial_point_count"] == 1
