"""Connector abstraction for future HoloOcean or real-world integration."""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np

from .math_utils import Pose


@dataclass
class RawSensorBundle:
    time_s: float
    magnetometer_block_nt: Optional[np.ndarray] = None
    sonar_relative_position_body_m: Optional[np.ndarray] = None
    sonar_heading_deg: Optional[float] = None
    imu_heading_deg: Optional[float] = None
    burial_depth_m: Optional[float] = None


@dataclass
class ConnectorStatus:
    connected: bool
    backend_name: str
    message: str = ""


class HoloOceanConnector(ABC):
    @abstractmethod
    def connect(self) -> bool:
        raise NotImplementedError

    @abstractmethod
    def send_pose(self, pose: Pose) -> bool:
        raise NotImplementedError

    @abstractmethod
    def recv_sensor_updates(self) -> Dict[str, np.ndarray]:
        raise NotImplementedError

    @abstractmethod
    def disconnect(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def status(self) -> ConnectorStatus:
        raise NotImplementedError


@dataclass
class HoloOceanConnectorMock(HoloOceanConnector):
    connected: bool = False
    backend_name: str = "mock"
    last_pose: Optional[Pose] = None
    last_bundle: Optional[RawSensorBundle] = None

    def connect(self) -> bool:
        self.connected = True
        return True

    def send_pose(self, pose: Pose) -> bool:
        self.last_pose = pose.copy()
        return self.connected

    def recv_sensor_updates(self) -> Dict[str, np.ndarray]:
        if self.last_bundle is None:
            return {}
        payload: Dict[str, np.ndarray] = {}
        if self.last_bundle.magnetometer_block_nt is not None:
            payload["magnetometer_block_nt"] = np.asarray(self.last_bundle.magnetometer_block_nt, dtype=float)
        if self.last_bundle.sonar_relative_position_body_m is not None:
            payload["sonar_relative_position_body_m"] = np.asarray(self.last_bundle.sonar_relative_position_body_m, dtype=float)
        return payload

    def disconnect(self) -> None:
        self.connected = False

    def status(self) -> ConnectorStatus:
        return ConnectorStatus(connected=self.connected, backend_name=self.backend_name, message="mock connector")


@dataclass
class NullHoloOceanConnector(HoloOceanConnector):
    backend_name: str = "null"

    def connect(self) -> bool:
        return False

    def send_pose(self, pose: Pose) -> bool:
        return False

    def recv_sensor_updates(self) -> Dict[str, np.ndarray]:
        return {}

    def disconnect(self) -> None:
        return None

    def status(self) -> ConnectorStatus:
        return ConnectorStatus(connected=False, backend_name=self.backend_name, message="disabled")


def build_connector(mode: str) -> HoloOceanConnector:
    if mode == "mock":
        return HoloOceanConnectorMock()
    return NullHoloOceanConnector()