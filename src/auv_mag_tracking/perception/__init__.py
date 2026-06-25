"""Perception layer: numeric-only sonar-magnetic fusion, filtering and path estimation.

This package was split from the former monolithic ``perception.py``. The public
symbols are re-exported here so existing imports (``from .perception import X``)
keep working unchanged.
"""

from .burial_inversion import BurialEstimate, MagneticBurialInverter
from .confidence import ConfidenceEstimator
from .filters import (
    LowPassFilter,
    MedianWindowFilter,
    RMSExtractor,
    StreamingBandpassFilter,
)
from .fitter import WeightedSlidingWindowFitter
from .local_path import LocalCableState, LocalCableStateEstimator, LocalPathObservation, LocalPathTrackingState
from .magnetic_path import (
    MagneticPathObservation,
    MagneticPathObservationBuilder,
    MagneticZigzagPhaseDetector,
    MagneticZigzagPhaseObservation,
)
from .orchestrator import MagneticCablePerception
from .peaks import PeakDetector
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
    "ConfidenceEstimator",
    "EnvelopeGradientTracker",
    "FitResult",
    "LowPassFilter",
    "LocalCableState",
    "LocalCableStateEstimator",
    "LocalPathObservation",
    "LocalPathTrackingState",
    "MagneticBurialInverter",
    "MagneticCablePerception",
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
    "RMSExtractor",
    "StreamingBandpassFilter",
    "StreamingVectorPCAFitter",
    "WeightedSlidingWindowFitter",
]
