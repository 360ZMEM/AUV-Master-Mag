"""Perception layer: numeric-only sonar-magnetic fusion, filtering and path estimation.

This package was split from the former monolithic ``perception.py``. The public
symbols are re-exported here so existing imports (``from .perception import X``)
keep working unchanged.
"""

from .burial_inversion import (
    BurialCycleEstimate,
    BurialEstimate,
    MagneticBurialCycleEstimator,
    MagneticBurialInverter,
)
from .confidence import ConfidenceEstimator
from .filters import (
    LowPassFilter,
    MedianWindowFilter,
    RMSExtractor,
    StreamingBandpassFilter,
)
from .fitter import WeightedSlidingWindowFitter
from .hypotheses import MagneticLookaheadHypothesis, MagneticShadowHypothesisSelection, ZigzagProbeCycleSummary
from .local_path import LocalCableState, LocalCableStateEstimator, LocalPathObservation, LocalPathTrackingState
from .map_frame_tracker import CableMapFrameState, CableMapFrameTracker
from .magnetic_path import (
    MagneticLookaheadTarget,
    MagneticLookaheadTargetBuilder,
    MagneticPathObservation,
    MagneticPathObservationBuilder,
    MagneticShadowHypothesisSelector,
    MagneticZigzagPhaseDetector,
    MagneticZigzagPhaseObservation,
)
from .orchestrator import MagneticCablePerception
from .peaks import PeakDetector
from .prior_alignment import PriorAlignmentEstimator, PriorAlignmentState
from .reacquire_region import ObservableRegion, ObservableRegionSelector
from .state import (
    FitResult,
    PeakEvent,
    PeakObservation,
    PeakZoneSample,
    PerceptionState,
)
from .vector import (
    EnvelopeGradientTracker,
    MagneticVectorAnalyzer,
    StreamingVectorPCAFitter,
)

__all__ = [
    "BurialEstimate",
    "BurialCycleEstimate",
    "CableMapFrameState",
    "CableMapFrameTracker",
    "ConfidenceEstimator",
    "EnvelopeGradientTracker",
    "FitResult",
    "LowPassFilter",
    "LocalCableState",
    "LocalCableStateEstimator",
    "LocalPathObservation",
    "LocalPathTrackingState",
    "MagneticBurialInverter",
    "MagneticBurialCycleEstimator",
    "MagneticCablePerception",
    "MagneticLookaheadHypothesis",
    "MagneticLookaheadTarget",
    "MagneticLookaheadTargetBuilder",
    "MagneticShadowHypothesisSelection",
    "MagneticShadowHypothesisSelector",
    "MagneticZigzagPhaseDetector",
    "MagneticZigzagPhaseObservation",
    "MagneticVectorAnalyzer",
    "MedianWindowFilter",
    "ObservableRegion",
    "ObservableRegionSelector",
    "PeakDetector",
    "PeakEvent",
    "PeakObservation",
    "PeakZoneSample",
    "PerceptionState",
    "PriorAlignmentEstimator",
    "PriorAlignmentState",
    "RMSExtractor",
    "StreamingBandpassFilter",
    "StreamingVectorPCAFitter",
    "WeightedSlidingWindowFitter",
    "ZigzagProbeCycleSummary",
]
