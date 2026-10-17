"""Experimental backends isolated from the core tracking pipeline.

These modules are not part of the simulation's core perception/control loop:
``simulator_connector`` is a forward-looking HoloOcean/hardware bridge stub, and
``phyphox_adapter`` streams a phone magnetometer for live hardware demos.  They
are kept here so the core package stays focused on the sonar-magnetic tracking
stack while remaining importable for opt-in experiments.
"""

from .phyphox_adapter import (
    MagnetometerReading,
    PhyphoxMagnetometerAdapter,
    PhyphoxSample,
    PhyphoxStreamer,
    SignalProcessor,
    Visualizer,
)
from .simulator_connector import (
    ConnectorStatus,
    HoloOceanConnector,
    HoloOceanConnectorMock,
    NullHoloOceanConnector,
    RawSensorBundle,
    build_connector,
)

__all__ = [
    "MagnetometerReading",
    "PhyphoxMagnetometerAdapter",
    "PhyphoxSample",
    "PhyphoxStreamer",
    "SignalProcessor",
    "Visualizer",
    "ConnectorStatus",
    "HoloOceanConnector",
    "HoloOceanConnectorMock",
    "NullHoloOceanConnector",
    "RawSensorBundle",
    "build_connector",
]
