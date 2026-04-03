"""Environment and magnetic field models."""

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np

from .config import EnvironmentConfig, ScenarioConfig, SignalConfig
from .math_utils import closest_point_on_segment, finite_wire_field_nT, polyline_length


@dataclass
class CableFitTruth:
    nearest_point_xy_m: np.ndarray
    tangent_xy: np.ndarray
    burial_depth_m: float
    cable_depth_m: float


class SeabedProfile:
    def __init__(self, base_depth_m: float, undulation_m: float, wavelength_m: float) -> None:
        self.base_depth_m = base_depth_m
        self.undulation_m = undulation_m
        self.wavelength_m = max(wavelength_m, 1.0)

    def depth_at_xy(self, x_m: float, y_m: float) -> float:
        phase_x = 2.0 * np.pi * x_m / self.wavelength_m
        phase_y = 2.0 * np.pi * y_m / (self.wavelength_m * 0.8)
        return float(self.base_depth_m + self.undulation_m * np.sin(phase_x) * np.cos(phase_y))


class CableRoute:
    def __init__(self, waypoints_xy_m: Tuple[Tuple[float, float], ...]) -> None:
        self.waypoints_xy_m = np.asarray(waypoints_xy_m, dtype=float)
        if self.waypoints_xy_m.ndim != 2 or self.waypoints_xy_m.shape[1] != 2:
            raise ValueError("Cable waypoints must be an Nx2 sequence.")
        if len(self.waypoints_xy_m) < 2:
            raise ValueError("Cable route requires at least two waypoints.")

    def sample_xy(self, step_m: float = 2.0) -> np.ndarray:
        sampled = [self.waypoints_xy_m[0]]
        for start, end in zip(self.waypoints_xy_m[:-1], self.waypoints_xy_m[1:]):
            segment = end - start
            distance = np.linalg.norm(segment)
            count = max(2, int(np.ceil(distance / max(step_m, 0.5))) + 1)
            for alpha in np.linspace(0.0, 1.0, count)[1:]:
                sampled.append(start + alpha * segment)
        return np.asarray(sampled, dtype=float)

    def nearest_point_and_tangent(self, position_xy_m: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
        best_point = None
        best_tangent = None
        best_distance = float("inf")
        for start, end in zip(self.waypoints_xy_m[:-1], self.waypoints_xy_m[1:]):
            candidate_point, _ = closest_point_on_segment(position_xy_m, start, end)
            distance = float(np.linalg.norm(candidate_point - position_xy_m))
            if distance < best_distance:
                best_distance = distance
                best_point = candidate_point
                tangent = end - start
                tangent_norm = np.linalg.norm(tangent)
                best_tangent = tangent / max(tangent_norm, 1e-9)
        return best_point, best_tangent, best_distance

    @property
    def total_length_m(self) -> float:
        return polyline_length(self.waypoints_xy_m)


class MagneticFieldModel:
    def __init__(
        self,
        route: CableRoute,
        seabed: SeabedProfile,
        signal: SignalConfig,
        burial_depth_m: float,
        suspended_height_m: float,
        segment_length_m: float,
    ) -> None:
        self.route = route
        self.seabed = seabed
        self.signal = signal
        self.burial_depth_m = burial_depth_m
        self.suspended_height_m = suspended_height_m
        self.segment_length_m = segment_length_m
        self.route_points_ned_m = self._build_3d_route()

    def _build_3d_route(self) -> np.ndarray:
        route_xy = self.route.sample_xy(step_m=self.segment_length_m)
        route_points = []
        for x_m, y_m in route_xy:
            seabed_depth_m = self.seabed.depth_at_xy(x_m, y_m)
            cable_depth_m = seabed_depth_m + self.burial_depth_m - self.suspended_height_m
            route_points.append((x_m, y_m, cable_depth_m))
        return np.asarray(route_points, dtype=float)

    def cable_field_ned_nt(self, position_ned_m: np.ndarray, time_s: float) -> np.ndarray:
        current_a = self.signal.current_at_time(time_s)
        if abs(current_a) < 1e-9:
            return np.zeros(3, dtype=float)
        total_field = np.zeros(3, dtype=float)
        for start, end in zip(self.route_points_ned_m[:-1], self.route_points_ned_m[1:]):
            total_field += finite_wire_field_nT(position_ned_m, start, end, current_a)
        return total_field


class CableEnvironment:
    def __init__(self, scenario: ScenarioConfig) -> None:
        self.config = scenario.environment
        self.signal = scenario.signal
        self.seabed = SeabedProfile(
            base_depth_m=self.config.seabed_depth_m,
            undulation_m=self.config.seabed_undulation_m,
            wavelength_m=self.config.seabed_wavelength_m,
        )
        self.route = CableRoute(self.config.cable_waypoints_xy_m)
        self.field_model = MagneticFieldModel(
            route=self.route,
            seabed=self.seabed,
            signal=self.signal,
            burial_depth_m=self.config.burial_depth_m,
            suspended_height_m=self.config.suspended_height_m,
            segment_length_m=self.config.field_segment_length_m,
        )
        self.background_field_ned_nt = np.asarray(self.config.background_field_ned_nt, dtype=float)

    def full_field_ned_nt(self, position_ned_m: np.ndarray, time_s: float) -> np.ndarray:
        return self.background_field_ned_nt + self.field_model.cable_field_ned_nt(position_ned_m, time_s)

    def cable_truth_at_xy(self, position_xy_m: np.ndarray) -> CableFitTruth:
        nearest_xy, tangent_xy, _ = self.route.nearest_point_and_tangent(position_xy_m)
        seabed_depth_m = self.seabed.depth_at_xy(nearest_xy[0], nearest_xy[1])
        cable_depth_m = seabed_depth_m + self.config.burial_depth_m - self.config.suspended_height_m
        return CableFitTruth(
            nearest_point_xy_m=nearest_xy,
            tangent_xy=tangent_xy,
            burial_depth_m=self.config.burial_depth_m,
            cable_depth_m=cable_depth_m,
        )

    def seabed_depth_m(self, position_xy_m: np.ndarray) -> float:
        return self.seabed.depth_at_xy(position_xy_m[0], position_xy_m[1])

    def sampled_cable_route_ned_m(self) -> np.ndarray:
        return self.field_model.route_points_ned_m.copy()
