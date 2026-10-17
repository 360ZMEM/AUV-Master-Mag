"""Environment and magnetic field models."""

from dataclasses import dataclass
from typing import Tuple

import numpy as np

from .config import EnvironmentConfig, ScenarioConfig, SignalConfig
from .math_utils import (
    build_polyline_projection_cache,
    batch_finite_wire_field_nT,
    estimate_polyline_curvature,
    finite_wire_field_nT,
    nearest_point_on_polyline,
    polyline_length,
    sample_serpentine_path,
    sample_sine_overlay_path,
    sample_spline_path,
    sample_tightening_arc_path,
)


@dataclass
class CableFitTruth:
    """表示电缆在当前位置的几何真值与埋深信息。"""

    nearest_point_xy_m: np.ndarray
    tangent_xy: np.ndarray
    heading_deg: float
    burial_depth_m: float
    cable_depth_m: float
    curvature_1pm: float
    progress_m: float


class SeabedProfile:
    def __init__(self, base_depth_m: float, undulation_m: float, wavelength_m: float) -> None:
        """初始化海床基准深度及其周期起伏模型。"""
        self.base_depth_m = base_depth_m
        self.undulation_m = undulation_m
        self.wavelength_m = max(wavelength_m, 1.0)

    def depth_at_xy(self, x_m: float, y_m: float) -> float:
        """返回指定平面位置处的海床深度。"""
        phase_x = 2.0 * np.pi * x_m / self.wavelength_m
        phase_y = 2.0 * np.pi * y_m / (self.wavelength_m * 0.8)
        return float(self.base_depth_m + self.undulation_m * np.sin(phase_x) * np.cos(phase_y))


class CableRoute:
    def __init__(self, config: EnvironmentConfig) -> None:
        """根据环境配置构建电缆路线与采样缓存。"""
        self.config = config
        waypoints_xy_m = config.cable_waypoints_xy_m
        self.waypoints_xy_m = np.asarray(waypoints_xy_m, dtype=float)
        if self.waypoints_xy_m.ndim != 2 or self.waypoints_xy_m.shape[1] != 2:
            raise ValueError("Cable waypoints must be an Nx2 sequence.")
        if len(self.waypoints_xy_m) < 2:
            raise ValueError("Cable route requires at least two waypoints.")
        self._sample_cache = {}
        self._projection_cache = {}
        if config.validate_curvature_on_build and config.min_cable_curvature_radius_m > 0:
            self._validate_curvature(step_m=max(config.field_segment_length_m * 0.5, 1.0))

    def sample_xy(self, step_m: float = 2.0) -> np.ndarray:
        """按指定步长采样电缆平面轨迹。"""
        cache_key = round(float(step_m), 4)
        if cache_key in self._sample_cache:
            return self._sample_cache[cache_key].copy()

        if self.config.cable_route_mode == "spline":
            sampled = sample_spline_path(self.waypoints_xy_m, step_m)
        elif self.config.cable_route_mode == "serpentine":
            sampled = sample_serpentine_path(
                turn_count=self.config.maze_turn_count,
                straight_length_m=self.config.maze_straight_length_m,
                lane_spacing_m=self.config.maze_lane_spacing_m,
                turn_radius_m=self.config.maze_turn_radius_m,
                step_m=step_m,
            )
        elif self.config.cable_route_mode == "tightening_arc":
            sampled = sample_tightening_arc_path(
                initial_straight_length_m=self.config.arc_initial_straight_length_m,
                turn_angle_deg=self.config.arc_turn_angle_deg,
                radius_m=self.config.arc_radius_m,
                step_m=step_m,
            )
        elif self.config.cable_route_mode == "sine":
            sampled = sample_sine_overlay_path(
                self.waypoints_xy_m,
                step_m,
                amplitudes_m=self.config.sine_amplitudes_m,
                wavelengths_m=self.config.sine_wavelengths_m,
            )
        else:
            sampled = [self.waypoints_xy_m[0]]
            for start, end in zip(self.waypoints_xy_m[:-1], self.waypoints_xy_m[1:]):
                segment = end - start
                distance = np.linalg.norm(segment)
                count = max(2, int(np.ceil(distance / max(step_m, 0.5))) + 1)
                for alpha in np.linspace(0.0, 1.0, count)[1:]:
                    sampled.append(start + alpha * segment)
            sampled = np.asarray(sampled, dtype=float)

        self._sample_cache[cache_key] = np.asarray(sampled, dtype=float)
        return self._sample_cache[cache_key].copy()

    def nearest_point_and_tangent(self, position_xy_m: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
        """返回当前位置到电缆路线的最近点、切向和距离。"""
        projection_cache = self.projection_cache(step_m=max(self.config.field_segment_length_m * 0.5, 1.0))
        best_point, best_tangent, best_distance, _, _ = nearest_point_on_polyline(position_xy_m, projection_cache)
        return best_point, best_tangent, best_distance

    def truth_at_xy(self, position_xy_m: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float, float, int]:
        """返回当前位置在电缆路线上的完整几何投影真值。"""
        projection_cache = self.projection_cache(step_m=max(self.config.field_segment_length_m * 0.5, 1.0))
        return nearest_point_on_polyline(position_xy_m, projection_cache)

    def curvature_at_xy(self, position_xy_m: np.ndarray) -> float:
        """估计当前位置对应的电缆离散曲率。"""
        projection_cache = self.projection_cache(step_m=max(self.config.field_segment_length_m * 0.5, 1.0))
        _, _, _, _, segment_index = nearest_point_on_polyline(position_xy_m, projection_cache)
        return estimate_polyline_curvature(projection_cache.polyline_xy, segment_index)

    def projection_cache(self, step_m: float) -> object:
        """返回指定采样步长下的折线投影缓存。"""
        cache_key = round(float(step_m), 4)
        if cache_key not in self._projection_cache:
            self._projection_cache[cache_key] = build_polyline_projection_cache(self.sample_xy(step_m))
        return self._projection_cache[cache_key]

    def _validate_curvature(self, step_m: float) -> None:
        """检查采样后的电缆路线是否满足最小曲率半径约束。"""
        sampled = self.sample_xy(step_m)
        min_radius_m = self.config.min_cable_curvature_radius_m
        violations = []
        for i in range(1, len(sampled) - 1):
            curvature_1pm = estimate_polyline_curvature(sampled, i)
            if abs(curvature_1pm) > 1e-9:
                radius_m = 1.0 / abs(curvature_1pm)
                if radius_m < min_radius_m - 1e-6:
                    violations.append((i, radius_m))
        if violations:
            worst_idx, worst_radius = min(violations, key=lambda v: v[1])
            import warnings
            warnings.warn(
                f"Cable route has {len(violations)} curvature violation(s): "
                f"min radius {worst_radius:.1f}m at segment {worst_idx} "
                f"(limit: {min_radius_m:.1f}m). "
                f"Tracking may be unreliable in tight bends.",
                stacklevel=3,
            )

    @property
    def total_length_m(self) -> float:
        """返回电缆路线总长度。"""
        return polyline_length(self.sample_xy(step_m=max(self.config.field_segment_length_m * 0.5, 1.0)))


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
        """初始化电缆磁场模型及其离散导线段。"""
        self.route = route
        self.seabed = seabed
        self.signal = signal
        self.burial_depth_m = burial_depth_m
        self.suspended_height_m = suspended_height_m
        self.segment_length_m = segment_length_m
        self.route_points_ned_m = self._build_3d_route()
        self.route_segment_starts_ned_m = self.route_points_ned_m[:-1].copy()
        self.route_segment_ends_ned_m = self.route_points_ned_m[1:].copy()

    def _build_3d_route(self) -> np.ndarray:
        """根据海床和埋深信息生成三维电缆路径。"""
        route_xy = self.route.sample_xy(step_m=self.segment_length_m)
        route_points = []
        for x_m, y_m in route_xy:
            seabed_depth_m = self.seabed.depth_at_xy(x_m, y_m)
            cable_depth_m = seabed_depth_m + self.burial_depth_m - self.suspended_height_m
            route_points.append((x_m, y_m, cable_depth_m))
        return np.asarray(route_points, dtype=float)

    def cable_field_ned_nt(self, position_ned_m: np.ndarray, time_s: float) -> np.ndarray:
        """返回给定时刻与位置下的电缆磁场。"""
        current_a = self.signal.current_at_time(time_s)
        if abs(current_a) < 1e-9:
            return np.zeros(3, dtype=float)
        return self.cable_field_gain_ned_nt(position_ned_m) * current_a

    def cable_field_gain_ned_nt(self, position_ned_m: np.ndarray) -> np.ndarray:
        """返回单位电流下的电缆磁场增益。"""
        return batch_finite_wire_field_nT(
            position_ned_m,
            self.route_segment_starts_ned_m,
            self.route_segment_ends_ned_m,
            1.0,
        )


class CableEnvironment:
    def __init__(self, scenario: ScenarioConfig) -> None:
        """根据场景配置初始化完整的环境与磁场模型。"""
        self.config = scenario.environment
        self.signal = scenario.signal
        self.seabed = SeabedProfile(
            base_depth_m=self.config.seabed_depth_m,
            undulation_m=self.config.seabed_undulation_m,
            wavelength_m=self.config.seabed_wavelength_m,
        )
        self.route = CableRoute(self.config)
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
        """返回背景场与电缆场叠加后的总磁场。"""
        return self.background_field_ned_nt + self.field_model.cable_field_ned_nt(position_ned_m, time_s)

    def cable_truth_at_xy(self, position_xy_m: np.ndarray) -> CableFitTruth:
        """返回当前位置对应的电缆真值摘要。"""
        nearest_xy, tangent_xy, _, progress_m, segment_index = self.route.truth_at_xy(position_xy_m)
        seabed_depth_m = self.seabed.depth_at_xy(nearest_xy[0], nearest_xy[1])
        cable_depth_m = seabed_depth_m + self.config.burial_depth_m - self.config.suspended_height_m
        heading_deg = float(np.rad2deg(np.arctan2(tangent_xy[1], tangent_xy[0])))
        projection_cache = self.route.projection_cache(step_m=max(self.config.field_segment_length_m * 0.5, 1.0))
        curvature_1pm = estimate_polyline_curvature(
            projection_cache.polyline_xy,
            segment_index,
        )
        return CableFitTruth(
            nearest_point_xy_m=nearest_xy,
            tangent_xy=tangent_xy,
            heading_deg=heading_deg,
            burial_depth_m=self.config.burial_depth_m,
            cable_depth_m=cable_depth_m,
            curvature_1pm=curvature_1pm,
            progress_m=progress_m,
        )

    def seabed_depth_m(self, position_xy_m: np.ndarray) -> float:
        """返回当前位置处的海床深度。"""
        return self.seabed.depth_at_xy(position_xy_m[0], position_xy_m[1])

    def sampled_cable_route_ned_m(self) -> np.ndarray:
        """返回当前三维电缆路径采样点。"""
        return self.field_model.route_points_ned_m.copy()
