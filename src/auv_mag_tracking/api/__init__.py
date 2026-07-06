"""Public deployment-facing API for AUV magnetic cable tracking."""

from .cable_map import CableMap
from .export import export_tracking_outputs
from .pipeline import AuvMagTrackingPipeline
from .schema import (
    validate_cable_map_csv,
    validate_magnetometer_csv,
    validate_navigation_csv,
    validate_sonar_csv,
)
from .types import (
    CableGuidanceOutput,
    CableTrackingOutput,
    DeploymentPerceptionConfig,
    MagneticInput,
    NavigationInput,
    SonarInput,
)

__all__ = [
    "AuvMagTrackingPipeline",
    "CableMap",
    "CableGuidanceOutput",
    "CableTrackingOutput",
    "DeploymentPerceptionConfig",
    "MagneticInput",
    "NavigationInput",
    "SonarInput",
    "export_tracking_outputs",
    "validate_cable_map_csv",
    "validate_magnetometer_csv",
    "validate_navigation_csv",
    "validate_sonar_csv",
]
