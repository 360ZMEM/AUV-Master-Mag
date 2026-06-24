"""Connector abstraction for future HoloOcean or real-world integration."""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

import numpy as np

from ..controller import GuidanceCommand
from ..math_utils import Pose


@dataclass
class RawSensorBundle:
    """表示来自外部仿真器或设备的一组原始传感器数据。"""

    time_s: float
    magnetometer_block_nt: Optional[np.ndarray] = None
    sonar_relative_position_body_m: Optional[np.ndarray] = None
    sonar_heading_deg: Optional[float] = None
    sonar_confidence: Optional[float] = None
    imu_heading_deg: Optional[float] = None
    imu_pitch_deg: Optional[float] = None
    imu_roll_deg: Optional[float] = None
    vehicle_position_ned_m: Optional[np.ndarray] = None
    vehicle_speed_mps: Optional[float] = None
    burial_depth_m: Optional[float] = None


@dataclass
class ConnectorStatus:
    """表示连接器当前连接状态及后端名称。"""

    connected: bool
    backend_name: str
    message: str = ""


class HoloOceanConnector(ABC):
    """定义与外部仿真器或真实设备通信的统一接口。"""

    @abstractmethod
    def connect(self) -> bool:
        """建立底层连接并返回是否成功。"""
        raise NotImplementedError

    @abstractmethod
    def send_pose(self, pose: Pose) -> bool:
        """发送当前位姿到外部系统。"""
        raise NotImplementedError

    @abstractmethod
    def recv_sensor_updates(self) -> RawSensorBundle:
        """接收外部系统返回的原始传感器更新。"""
        raise NotImplementedError

    def send_command(self, command: GuidanceCommand) -> bool:
        """发送控制指令到底层载体；默认实现表示后端暂不支持。"""
        return False

    @abstractmethod
    def disconnect(self) -> None:
        """关闭底层连接。"""
        raise NotImplementedError

    @abstractmethod
    def status(self) -> ConnectorStatus:
        """返回当前连接状态摘要。"""
        raise NotImplementedError


@dataclass
class HoloOceanConnectorMock(HoloOceanConnector):
    """用于本地调试的模拟连接器实现。"""

    connected: bool = False
    backend_name: str = "mock"
    last_pose: Optional[Pose] = None
    last_bundle: Optional[RawSensorBundle] = None
    last_command: Optional[GuidanceCommand] = None

    def connect(self) -> bool:
        """标记模拟连接为已连接。"""
        self.connected = True
        return True

    def send_pose(self, pose: Pose) -> bool:
        """缓存最近一次发送的位姿。"""
        self.last_pose = pose.copy()
        return self.connected

    def recv_sensor_updates(self) -> RawSensorBundle:
        """返回缓存的模拟传感器数据。"""
        if self.last_bundle is None:
            return RawSensorBundle(time_s=0.0)
        bundle = self.last_bundle
        magnetometer_block_nt = None
        sonar_relative_position_body_m = None
        if self.last_bundle.magnetometer_block_nt is not None:
            magnetometer_block_nt = np.asarray(self.last_bundle.magnetometer_block_nt, dtype=float)
        if self.last_bundle.sonar_relative_position_body_m is not None:
            sonar_relative_position_body_m = np.asarray(self.last_bundle.sonar_relative_position_body_m, dtype=float)
        vehicle_position_ned_m = None
        if self.last_bundle.vehicle_position_ned_m is not None:
            vehicle_position_ned_m = np.asarray(self.last_bundle.vehicle_position_ned_m, dtype=float)
        return RawSensorBundle(
            time_s=bundle.time_s,
            magnetometer_block_nt=magnetometer_block_nt,
            sonar_relative_position_body_m=sonar_relative_position_body_m,
            sonar_heading_deg=bundle.sonar_heading_deg,
            sonar_confidence=bundle.sonar_confidence,
            imu_heading_deg=bundle.imu_heading_deg,
            imu_pitch_deg=bundle.imu_pitch_deg,
            imu_roll_deg=bundle.imu_roll_deg,
            vehicle_position_ned_m=vehicle_position_ned_m,
            vehicle_speed_mps=bundle.vehicle_speed_mps,
            burial_depth_m=bundle.burial_depth_m,
        )

    def send_command(self, command: GuidanceCommand) -> bool:
        """缓存最近一次控制指令。"""
        self.last_command = command
        return self.connected

    def disconnect(self) -> None:
        """断开模拟连接。"""
        self.connected = False

    def status(self) -> ConnectorStatus:
        """返回模拟连接器状态。"""
        return ConnectorStatus(connected=self.connected, backend_name=self.backend_name, message="mock connector")


@dataclass
class NullHoloOceanConnector(HoloOceanConnector):
    """禁用外部连接时使用的空实现。"""

    backend_name: str = "null"

    def connect(self) -> bool:
        """始终返回未连接。"""
        return False

    def send_pose(self, pose: Pose) -> bool:
        """忽略所有位姿发送请求。"""
        return False

    def recv_sensor_updates(self) -> RawSensorBundle:
        """始终返回空更新。"""
        return RawSensorBundle(time_s=0.0)

    def disconnect(self) -> None:
        """空操作。"""
        return None

    def status(self) -> ConnectorStatus:
        """返回禁用状态。"""
        return ConnectorStatus(connected=False, backend_name=self.backend_name, message="disabled")


def build_connector(mode: str) -> HoloOceanConnector:
    """根据模式字符串构建对应的连接器实现。"""
    if mode == "mock":
        return HoloOceanConnectorMock()
    return NullHoloOceanConnector()
