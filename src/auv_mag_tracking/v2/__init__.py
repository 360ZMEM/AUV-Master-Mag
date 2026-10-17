"""Independent, interface-compatible v2 cable search and FWHM toolkit."""

from .cable_estimator import CableLineEstimator
from .artifacts import write_experiment_manifest
from .config import CrossScanConfig, DetectionConfig, FwhmConfig, InspectionConfig, SearchConfig, TrackingConfig, V2Config
from .cross_scan import build_cross_scan
from .corridor_survey import (
    CorridorSurveyConfig, ShallowZigzagConfig, StartSideDecision, SurveyEstimate,
    SurveyPlan, SurveyProfile, TurnFeasibility, assess_turn_feasibility,
    build_corridor_survey, build_shallow_zigzag_survey, estimate_survey_line,
    select_shallow_start_side,
)
from .detection import MagneticDetector
from .fwhm import estimate_fwhm
from .flythrough import FlythroughConfig, FlythroughInspectionPipelineV2
from .shape_calibration import (
    AffineWidthCalibration, CalibrationScan, ShapeCalibration,
    calibrate_affine_width, calibrate_shape_factor, estimate_with_affine_calibration,
    estimate_with_calibrated_models,
)
from .pipeline import AuvMagTrackingPipelineV2
from .physics import physical_finite_wire_field_nT, physical_polyline_field_nT
from .search import build_prior_guided_search
from .shallow_survey import (
    ShallowSurveyInspectionPipelineV2, ShallowSurveyMissionConfig, SurveyCurrentEvidence,
)
from .state_machine import V2StateMachine
from .tracking import compute_tracking_correction
from .types import (
    CableObservation,
    CrossScanPlan,
    DetectionResult,
    FwhmResult,
    MissionPhase,
    SearchPlan,
    TrackingCorrection,
)

__all__ = [
    "AuvMagTrackingPipelineV2", "CableLineEstimator", "CableObservation", "CrossScanConfig", "CrossScanPlan",
    "DetectionConfig", "DetectionResult", "FwhmConfig", "FwhmResult",
    "InspectionConfig", "FlythroughConfig", "FlythroughInspectionPipelineV2",
    "MagneticDetector", "MissionPhase", "SearchConfig", "SearchPlan",
    "TrackingConfig", "TrackingCorrection", "V2Config", "V2StateMachine",
    "build_cross_scan", "build_prior_guided_search", "compute_tracking_correction",
    "estimate_fwhm",
    "AffineWidthCalibration", "CalibrationScan", "ShapeCalibration",
    "calibrate_affine_width", "calibrate_shape_factor", "estimate_with_affine_calibration",
    "estimate_with_calibrated_models",
    "physical_finite_wire_field_nT", "physical_polyline_field_nT",
    "write_experiment_manifest",
    "CorridorSurveyConfig", "ShallowZigzagConfig", "SurveyEstimate", "SurveyPlan",
    "SurveyProfile", "TurnFeasibility", "StartSideDecision", "assess_turn_feasibility",
    "build_corridor_survey", "build_shallow_zigzag_survey", "estimate_survey_line",
    "select_shallow_start_side",
    "ShallowSurveyInspectionPipelineV2", "ShallowSurveyMissionConfig", "SurveyCurrentEvidence",
]
