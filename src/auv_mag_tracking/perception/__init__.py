"""Perception layer: numeric-only sonar-magnetic fusion, filtering and path estimation.

This package was split from the former monolithic ``perception.py``. The public
symbols are re-exported here so existing imports (``from .perception import X``)
keep working unchanged.
"""

from .confidence import ConfidenceEstimator
from .filters import (
    LowPassFilter,
    MedianWindowFilter,
    RMSExtractor,
    StreamingBandpassFilter,
)
from .fitter import WeightedSlidingWindowFitter
from .orchestrator import MagneticCablePerception
from .peaks import PeakDetector
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
    "ConfidenceEstimator",
    "EnvelopeGradientTracker",
    "FitResult",
    "LowPassFilter",
    "MagneticCablePerception",
    "MagneticVectorAnalyzer",
    "MedianWindowFilter",
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
