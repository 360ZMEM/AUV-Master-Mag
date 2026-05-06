"""声-磁融合演示的配置对象与预置场景集合。

本模块集中定义所有实验可调参数，按“信号 -> 传感器 -> 声呐 -> 跟踪 ->
车辆 -> 环境 -> 观测 -> 可视化 -> 场景快照”的顺序组织，便于在修改实验
条件时快速定位对应参数。
"""

import copy
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import numpy as np


Vector3Tuple = Tuple[float, float, float]
Vector2Tuple = Tuple[float, float]


@dataclass
class HighFidelityMagnetometerConfig:
    """高保真磁力计建模参数。

    这组参数用于模拟更接近真实设备的磁力计输出，重点覆盖采样速率、量化
    位宽、量程限制、AUV 本体静态干扰，以及白噪声、粉红噪声和脉冲干扰。
    通过调整这些字段，可以快速切换“理想仿真”和“设备级噪声仿真”的实验
    条件。

    Attributes:
        enabled: 是否启用高保真磁力计模型。关闭时通常退化为基础噪声模型。
        sampling_rate_hz: 高保真通道的采样率，决定模拟设备的刷新频率。
        bit_depth: ADC 量化位宽，越低越容易出现离散化误差和饱和问题。
        full_scale_nt: 量程上限，单位 nT，用于判断是否发生量程裁剪。
        auv_static_interference_body_nt: AUV 机体坐标系下的静态干扰偏置，
            主要用于模拟电机、电子设备和结构磁化带来的固定背景场。
        white_noise_std_nt: 白噪声标准差，控制短时随机抖动强度。
        pink_noise_std_nt: 粉红噪声标准差，用于模拟低频漂移和相关噪声。
        pink_noise_exponent: 粉红噪声谱指数，数值越大低频成分越明显。
        impulse_probability: 单个采样点触发脉冲干扰的概率。
        impulse_amplitude_nt: 脉冲干扰峰值幅度，单位 nT。
        impulse_decay_samples: 脉冲干扰的衰减样本数，越大表示拖尾越长。
    """

    enabled: bool = False
    sampling_rate_hz: float = 1000.0
    bit_depth: int = 24
    full_scale_nt: float = 100000.0
    auv_static_interference_body_nt: Vector3Tuple = (35.0, -20.0, 15.0)
    white_noise_std_nt: float = 0.03
    pink_noise_std_nt: float = 0.08
    pink_noise_exponent: float = 1.0
    impulse_probability: float = 0.002
    impulse_amplitude_nt: float = 60.0
    impulse_decay_samples: int = 8


@dataclass
class SignalProcessingConfig:
    """信号处理参数集合。

    这些参数控制磁场信号在进入感知层前的频域分析、插值、带通筛选、
    RMS/锁相解调和诊断输出。它们直接影响 AC 特征提取的稳定性、响应速度
    与抗噪能力。

    Attributes:
        window_size: FFT 或谱分析窗口长度，越大频率分辨率越高，但时延更大。
        overlap: 相邻窗口重叠比例，用于平滑连续输出并提高时间连续性。
        window_function: 窗函数名称，例如 hann，用于降低频谱泄漏。
        target_frequency_tolerance_hz: 目标频点容差，决定频率匹配的宽松程度。
        peak_search_half_width_hz: 峰值搜索的半宽，控制在目标频点附近搜索的范围。
        use_centroid_frequency_estimation: 是否启用谱质心估计，改善频率偏移判断。
        min_ac_frequency_hz: 判定为 AC 信号时允许的最低频率阈值。
        ac_energy_ratio_threshold: AC 能量占比阈值，越大越保守。
        snr_detection_threshold_db: 信噪比判定阈值，低于该值通常认为特征不可靠。
        axis_combination_mode: 多轴合成策略，例如 dominant_axis。
        enable_interpolation: 是否在低采样率场景下进行插值重采样。
        interpolation_target_rate_hz: 插值后目标采样率，用于统一处理尺度。
        interpolation_input_rate_threshold_hz: 低于该输入采样率时才启用插值。
        bandpass_order: 带通滤波器阶数，越高滚降越陡但更容易产生振铃。
        rms_cycle_count: RMS 统计覆盖的周期数，越大越平稳但响应越慢。
        lockin_enabled: 是否启用锁相解调，用于增强指定频点的成分。
        lockin_cycle_count: 锁相解调累积的周期数。
        diagnostics_use_fft: 是否输出 FFT 诊断信息，便于调参和排障。
    """

    window_size: int = 256
    overlap: float = 0.75
    window_function: str = "hann"
    target_frequency_tolerance_hz: float = 2.0
    peak_search_half_width_hz: float = 2.0
    use_centroid_frequency_estimation: bool = True
    min_ac_frequency_hz: float = 8.0
    ac_energy_ratio_threshold: float = 0.18
    snr_detection_threshold_db: float = 6.0
    axis_combination_mode: str = "dominant_axis"
    enable_interpolation: bool = True
    interpolation_target_rate_hz: float = 1000.0
    interpolation_input_rate_threshold_hz: float = 250.0
    bandpass_order: int = 2
    rms_cycle_count: int = 3
    lockin_enabled: bool = True
    lockin_cycle_count: int = 1
    diagnostics_use_fft: bool = True


@dataclass
class SignalConfig:
    """电缆电流信号配置。

    该配置决定电缆中激励电流的时间波形，是磁场仿真的根源输入之一。修改
    这里可以切换直流、50 Hz AC、演示频率等不同实验模式。

    Attributes:
        mode: 电流工作模式，常见取值包括 dc、ac_50hz、ac_demo、ac_60hz。
        frequency_hz: AC 模式下的基波频率。
        dc_current_a: 直流模式下的恒定电流幅值。
        ac_current_amplitude_a: AC 模式下的峰值电流幅值。
        phase_rad: 相位偏移，用于控制波形初相。
        bandpass_half_width_hz: 带通滤波半宽，决定围绕基频保留的频带宽度。
    """

    mode: str = "ac_50hz"
    frequency_hz: float = 50.0
    dc_current_a: float = 0.0
    ac_current_amplitude_a: float = 600.0
    phase_rad: float = 0.0
    bandpass_half_width_hz: float = 8.0

    def current_for_times(self, time_s: np.ndarray) -> np.ndarray:
        """根据时间序列生成电缆电流波形。

        Args:
            time_s: 一维或多维时间采样点，单位秒。

        Returns:
            与输入时间形状一致的电流数组。
        """
        time_s = np.asarray(time_s, dtype=float)
        if self.mode == "dc":
            return np.full_like(time_s, fill_value=self.dc_current_a, dtype=float)
        if self.mode in {"ac_50hz", "ac_demo", "ac_60hz"}:
            return self.ac_current_amplitude_a * np.sin(2.0 * np.pi * self.frequency_hz * time_s + self.phase_rad)
        return np.full_like(time_s, fill_value=self.dc_current_a, dtype=float)

    def current_at_time(self, time_s: float) -> float:
        """返回单个时刻的瞬时电流值。

        该方法是对 `current_for_times` 的标量封装，适合控制循环中按时刻查询。
        """
        return float(self.current_for_times(np.asarray([time_s], dtype=float))[0])


@dataclass
class SensorConfig:
    """基础传感器模型参数。

    这组参数描述磁力计、IMU 和安装误差的基础行为，用来控制普通仿真模式下
    的测量噪声、偏置漂移、姿态误差和动态范围限制。它是实验噪声水平和姿态
    补偿能力评估的主要入口。

    Attributes:
        magnetometer_sample_rate_hz: 磁力计采样率，决定原始磁场数据刷新速度。
        noise_std_nt: 基础白噪声标准差，控制随机测量误差。
        bias_drift_std_nt_per_s: 偏置漂移的时间增长速率，用于模拟慢变零点偏移。
        nonorthogonality_deg: 传感器三轴不正交误差角，影响轴间串扰。
        static_rotation_euler_deg: 传感器安装姿态的欧拉角偏差。
        dynamic_range_nt: 传感器动态范围，超出时视为饱和或裁剪风险。
        imu_heading_noise_deg: 航向角测量噪声。
        imu_tilt_noise_deg: 俯仰和横滚测量噪声。
        weak_signal_threshold_nt: 弱磁信号阈值，低于该值时通常需要更保守的感知策略。
        high_fidelity: 高保真磁力计扩展配置，支持更复杂的硬件噪声建模。
    """

    magnetometer_sample_rate_hz: float = 200.0
    noise_std_nt: float = 0.05
    bias_drift_std_nt_per_s: float = 0.01
    nonorthogonality_deg: float = 0.2
    static_rotation_euler_deg: Vector3Tuple = (0.0, 0.0, 0.0)
    dynamic_range_nt: float = 100000.0
    imu_heading_noise_deg: float = 0.1
    imu_tilt_noise_deg: float = 0.05
    weak_signal_threshold_nt: float = 18.0
    high_fidelity: HighFidelityMagnetometerConfig = field(default_factory=HighFidelityMagnetometerConfig)


@dataclass
class SonarConfig:
    """声呐观测模型参数。

    声呐用于在磁场不稳定、埋深变化或目标信号较弱时提供辅助先验。通过这些
    参数可以调节声呐的最大探测距离、视场角、命中率、测距误差和航向误差，
    也可以模拟“缺失”“退化”和“优势存在”等不同可用性条件。

    Attributes:
        mode: 声呐工作模式，用于区分可靠、退化或其他策略。
        max_range_m: 最大有效探测距离。
        horizontal_fov_deg: 水平视场角，决定声呐覆盖扇区宽度。
        prob_detection: 基础探测概率，越低越容易漏检。
        position_noise_std_m: 位置观测噪声标准差。
        heading_noise_deg: 航向观测噪声标准差。
        buried_loss_factor: 电缆埋深导致的探测衰减系数。
        update_rate_hz: 声呐更新频率。
        absence_range_m: 目标在此距离外时可视为基本不可见的阈值。
        advantage_probability: 优势观测出现概率，用于模拟偶发高质量回波。
        advantage_position_noise_scale: 优势观测时的位置噪声缩放系数。
        advantage_heading_noise_scale: 优势观测时的航向噪声缩放系数。
        advantage_confidence_floor: 优势观测的最低置信度下限。
    """

    mode: str = "degraded"
    max_range_m: float = 15.0
    horizontal_fov_deg: float = 120.0
    prob_detection: float = 0.70
    position_noise_std_m: float = 0.45
    heading_noise_deg: float = 5.0
    buried_loss_factor: float = 0.08
    update_rate_hz: float = 8.0
    absence_range_m: float = 18.0
    advantage_probability: float = 0.15
    advantage_position_noise_scale: float = 0.25
    advantage_heading_noise_scale: float = 0.35
    advantage_confidence_floor: float = 0.90


@dataclass
class TrackingConfig:
    """跟踪、调度与恢复策略参数。

    这是本项目最关键的调参集合，覆盖峰值检测、记忆衰减、路线跟踪、失联
    恢复、螺旋搜索、部署成熟度和安全锁定等行为。修改这里会直接改变 AUV 在
    不同实验条件下的搜索风格、收敛速度、失联恢复和转弯处理方式。

    Attributes:
        approach_angle_deg: 常规接近电缆时的目标入射角。
        approach_angle_min_deg: 允许的最小接近角，用于限制过于平行的接近。
        approach_angle_max_deg: 允许的最大接近角，用于限制过于激进的切入。
        turn_trigger_ratio: 转弯触发比例阈值，越小越容易提前转弯。
        hysteresis_fraction: 峰值检测滞回比例，降低阈值抖动带来的误判。
        smoothing_time_constant_s: 主信号平滑时间常数。
        envelope_time_constant_s: 包络跟踪时间常数。
        noise_floor_time_constant_s: 噪声底估计时间常数。
        peak_cooldown_s: 两次峰值事件之间的最小冷却时间。
        min_peak_strength_nt: 判定为有效峰值所需的最小磁场强度。
        fit_history_size: 拟合历史缓存长度。
        forgetting_factor: 记忆衰减系数，越小越偏向近期样本。
        median_window_samples: 中值滤波窗口长度，用于抑制孤立异常点。
        lost_timeout_s: 失联超时阈值，超过该时长可认为目标丢失。
        high_confidence_threshold: 高置信度阈值。
        low_confidence_threshold: 低置信度阈值。
        search_leg_time_s: 之字形搜索单腿时长。
        sonar_preferred_distance_m: 声呐最偏好的工作距离。
        magnetic_takeover_strength_nt: 磁感知接管控制所需的强度阈值。
        min_zigzag_width_m: 之字形最小横向宽度。
        max_zigzag_width_m: 之字形最大横向宽度。
        zigzag_width_gain_m_per_nt: 强度到横向宽度的增益系数。
        safe_lock_peak_drop_nt: 进入安全锁定时允许的峰值下降阈值。
        blind_follow_memory_size: 盲跟踪记忆长度。
        fit_acceptance_residual_m: 拟合结果可接受的最大残差。
        weighted_fitter_capacity: 加权拟合器缓存容量。
        weighted_fitter_snr_floor: 加权拟合的最低 SNR 门限。
        fit_reject_heading_delta_deg: 与历史航向偏差过大时的拟合拒绝阈值。
        fit_reject_confidence_threshold: 低于该置信度时拒绝拟合结果。
        consecutive_miss_threshold: 连续漏检后触发退化策略的次数阈值。
        spiral_radius_growth_mps: 螺旋搜索半径增长速度。
        spiral_max_radius_m: 螺旋搜索最大半径。
        spiral_entry_window_s: 进入螺旋搜索前的观察窗口。
        guidance_memory_timeout_s: 引导记忆超时时间。
        memory_guidance_confidence_floor: 记忆引导可用性的最低置信度。
        use_nominal_route_prior: 是否使用名义路径先验。
        peak_ascending_min_samples: 峰值上升段最少样本数。
        peak_descending_min_samples: 峰值下降段最少样本数。
        peak_zone_window_size: 峰值区间窗口大小。
        peak_outlier_rejection_distance_m: 峰值离群点剔除距离阈值。
        deployment_bootstrap_min_peak_count: 部署初始化所需的最小峰值计数。
        deployment_bootstrap_min_span_m: 初始化峰值最小跨度。
        deployment_tracking_maturity_gain: 跟踪成熟度增长速率。
        deployment_tracking_maturity_decay_per_s: 跟踪成熟度随时间衰减速率。
        deployment_tracking_maturity_residual_threshold_m: 残差高于该值时成熟度难以提升。
        deployment_tracking_maturity_stale_age_s: 认为成熟度过时的时间阈值。
        deployment_hold_maturity_threshold: 进入保持状态所需的成熟度阈值。
        deployment_washout_residual_m: 洗出/失配判定残差阈值。
        deployment_washout_snr_linear_threshold: 洗出判定的线性 SNR 阈值。
        deployment_washout_retention_count: 保留最近有效观测的数量。
        deployment_washout_reacquire_holdoff_s: 重新捕获前的冷却时间。
        deployment_lost_timeout_high_maturity_multiplier: 高成熟度场景下的失联超时放大倍数。
        envelope_savgol_window: 包络 Savitzky-Golay 平滑窗口长度。
        envelope_savgol_polyorder: 包络平滑多项式阶数。
        spatial_gradient_min_speed_mps: 计算空间梯度所需的最低速度。
        parabolic_interpolation_enabled: 是否启用抛物线插值以细化峰值位置。
        peak_position_delay_s: 峰值位置输出的延迟补偿。
        bootstrap_min_heading_diff_deg: 启动拟合所需的最小航向差。
        weighted_ransac_iterations: 加权 RANSAC 迭代次数，0 表示关闭。
        weighted_ransac_inlier_threshold_m: RANSAC 内点阈值。
        weighted_ransac_min_inlier_ratio: RANSAC 最小内点比例。
        safe_lock_strength_ratio_threshold: 安全锁定所需的强度比阈值。
        safe_lock_ideal_field_width_m: 理想磁场宽度，用于锁定判定。
        safe_lock_displacement_factor: 安全锁定的位移放大因子。
        safe_lock_gradient_angle_threshold_deg: 梯度角度阈值。
        safe_lock_gradient_confidence_penalty: 梯度不稳定时的置信度惩罚。
        vector_heading_enabled: 是否启用向量航向分析。
        vector_heading_confidence_weight: 向量航向在综合置信度中的权重。
    """

    approach_angle_deg: float = 45.0
    approach_angle_min_deg: float = 30.0
    approach_angle_max_deg: float = 45.0
    turn_trigger_ratio: float = 0.60
    hysteresis_fraction: float = 0.08
    smoothing_time_constant_s: float = 0.20
    envelope_time_constant_s: float = 0.25
    noise_floor_time_constant_s: float = 1.50
    peak_cooldown_s: float = 0.80
    min_peak_strength_nt: float = 80.0
    fit_history_size: int = 5
    forgetting_factor: float = 0.72
    median_window_samples: int = 5
    lost_timeout_s: float = 4.0
    high_confidence_threshold: float = 0.65
    low_confidence_threshold: float = 0.35
    search_leg_time_s: float = 4.0
    sonar_preferred_distance_m: float = 7.5
    magnetic_takeover_strength_nt: float = 55.0
    min_zigzag_width_m: float = 2.5
    max_zigzag_width_m: float = 9.0
    zigzag_width_gain_m_per_nt: float = 0.02
    safe_lock_peak_drop_nt: float = 15.0
    blind_follow_memory_size: int = 3
    fit_acceptance_residual_m: float = 12.0
    weighted_fitter_capacity: int = 8
    weighted_fitter_snr_floor: float = 1.05
    fit_reject_heading_delta_deg: float = 30.0
    fit_reject_confidence_threshold: float = 0.60
    consecutive_miss_threshold: int = 3
    spiral_radius_growth_mps: float = 0.55
    spiral_max_radius_m: float = 20.0
    spiral_entry_window_s: float = 2.0
    guidance_memory_timeout_s: float = 7.5
    memory_guidance_confidence_floor: float = 0.38
    use_nominal_route_prior: bool = True
    peak_ascending_min_samples: int = 2
    peak_descending_min_samples: int = 2
    peak_zone_window_size: int = 20
    peak_outlier_rejection_distance_m: float = 2.0
    deployment_bootstrap_min_peak_count: int = 3
    deployment_bootstrap_min_span_m: float = 6.0
    deployment_tracking_maturity_gain: float = 0.2
    deployment_tracking_maturity_decay_per_s: float = 0.1
    deployment_tracking_maturity_residual_threshold_m: float = 8.0
    deployment_tracking_maturity_stale_age_s: float = 2.0
    deployment_hold_maturity_threshold: float = 0.8
    deployment_washout_residual_m: float = 5.0
    deployment_washout_snr_linear_threshold: float = 10.0
    deployment_washout_retention_count: int = 2
    deployment_washout_reacquire_holdoff_s: float = 1.5
    deployment_lost_timeout_high_maturity_multiplier: float = 1.5
    # --- Signal enhancement & gradient parameters ---
    envelope_savgol_window: int = 7
    envelope_savgol_polyorder: int = 2
    spatial_gradient_min_speed_mps: float = 0.3
    # --- Robust peak finding parameters ---
    parabolic_interpolation_enabled: bool = True
    peak_position_delay_s: float = 0.04
    # --- Global estimation parameters ---
    bootstrap_min_heading_diff_deg: float = 30.0
    weighted_ransac_iterations: int = 0
    weighted_ransac_inlier_threshold_m: float = 5.0
    weighted_ransac_min_inlier_ratio: float = 0.5
    # --- Perception safe-lock parameters ---
    safe_lock_strength_ratio_threshold: float = 0.20
    safe_lock_ideal_field_width_m: float = 12.0
    safe_lock_displacement_factor: float = 1.5
    safe_lock_gradient_angle_threshold_deg: float = 45.0
    safe_lock_gradient_confidence_penalty: float = 0.3
    # --- Vector heading analysis ---
    vector_heading_enabled: bool = True
    vector_heading_confidence_weight: float = 0.15


@dataclass
class VehicleConfig:
    """AUV 运动学与初始状态参数。

    这些参数控制平台的巡航速度、搜索速度、最大转向能力、最小转弯半径以及
    初始位置和姿态。调这些值可以快速模拟更灵敏的平台、更保守的平台或更大
    的初始偏差。

    Attributes:
        cruise_speed_mps: 巡航速度。
        search_speed_mps: 搜索阶段速度。
        max_yaw_rate_deg_s: 最大偏航角速度。
        min_turning_radius_m: 物理可实现的最小转弯半径。
        altitude_above_seabed_m: 目标航高。
        initial_position_ned_m: 初始 NED 坐标。
        initial_heading_deg: 初始航向角。
        pitch_amplitude_deg: 俯仰扰动幅值。
        roll_amplitude_deg: 横滚扰动幅值。
        pitch_frequency_hz: 俯仰扰动频率。
        roll_frequency_hz: 横滚扰动频率。
    """

    cruise_speed_mps: float = 1.0
    search_speed_mps: float = 1.0
    max_yaw_rate_deg_s: float = 36.0
    min_turning_radius_m: float = 2.5
    altitude_above_seabed_m: float = 6.0
    initial_position_ned_m: Vector3Tuple = (-90.0, -45.0, 0.0)
    initial_heading_deg: float = 30.0
    pitch_amplitude_deg: float = 2.0
    roll_amplitude_deg: float = 3.0
    pitch_frequency_hz: float = 0.04
    roll_frequency_hz: float = 0.05


@dataclass
class EnvironmentConfig:
    """环境与电缆几何参数。

    该配置定义电缆路线、海床起伏、埋深、背景地磁场和曲率约束，是场景差异
    的主要来源。修改这部分可以直接改变路径形状、磁场背景和几何可行性检查
    条件。

    Attributes:
        cable_waypoints_xy_m: 电缆路径控制点，单位为米。
        cable_route_mode: 路径生成方式，例如 spline 或 sine。
        spline_tension: 样条张力参数，影响曲线光滑程度。
        sine_amplitudes_m: 正弦形路径振幅序列。
        sine_wavelengths_m: 正弦形路径波长序列。
        seabed_depth_m: 海床深度。
        seabed_undulation_m: 海床起伏幅度。
        seabed_wavelength_m: 海床起伏波长。
        burial_depth_m: 电缆埋深。
        suspended_height_m: 电缆悬空高度。
        background_field_ned_nt: 背景地磁场，NED 坐标系下的 nT 值。
        nominal_route_heading_deg: 名义路线航向，用于先验约束。
        field_segment_length_m: 场段离散长度。
        min_cable_curvature_radius_m: 允许的最小曲率半径。
        validate_curvature_on_build: 构建场景时是否检查曲率约束。
    """

    cable_waypoints_xy_m: Tuple[Vector2Tuple, ...] = ((-320.0, 0.0), (360.0, 0.0))
    cable_route_mode: str = "spline"
    spline_tension: float = 0.0
    sine_amplitudes_m: Tuple[float, ...] = (6.0, 2.5)
    sine_wavelengths_m: Tuple[float, ...] = (140.0, 55.0)
    seabed_depth_m: float = 30.0
    seabed_undulation_m: float = 0.8
    seabed_wavelength_m: float = 220.0
    burial_depth_m: float = 1.5
    suspended_height_m: float = 0.0
    background_field_ned_nt: Vector3Tuple = (25000.0, 0.0, 42000.0)
    nominal_route_heading_deg: float = 0.0
    field_segment_length_m: float = 4.0
    min_cable_curvature_radius_m: float = 50.0
    validate_curvature_on_build: bool = True


@dataclass
class SurveyConfig:
    """埋深观测通道参数。

    埋深通道通常用于辅助判断电缆相对海床的位置关系。这里的参数决定更新
    频率、噪声强度与丢包概率，可用来模拟声学或测深数据的稳定性差异。

    Attributes:
        burial_depth_update_rate_hz: 埋深观测更新频率。
        burial_depth_noise_std_m: 埋深测量噪声标准差。
        burial_depth_dropout_probability: 埋深通道丢失观测的概率。
    """

    burial_depth_update_rate_hz: float = 2.0
    burial_depth_noise_std_m: float = 0.12
    burial_depth_dropout_probability: float = 0.04


@dataclass
class VisualizationConfig:
    """可视化刷新与历史显示参数。

    这些参数只影响 UI 展示，不改变仿真逻辑。它们控制历史窗口长度、绘图刷新
    频率、标题文本、频谱显示上限以及状态平滑强度。

    Attributes:
        history_seconds: 图表中保留的历史时间长度。
        frame_interval_ms: UI 刷新周期。
        figure_title: 主图标题。
        psd_max_frequency_hz: 频谱图显示的最高频率。
        update_stride_steps: 每隔多少仿真步刷新一次可视化。
        uncertainty_smoothing_alpha: 不确定度曲线的平滑系数。
    """

    history_seconds: float = 20.0
    frame_interval_ms: int = 40
    figure_title: str = "AUV Sonar-Magnetic Cable Tracking Demo"
    psd_max_frequency_hz: float = 120.0
    update_stride_steps: int = 6
    uncertainty_smoothing_alpha: float = 0.22


@dataclass
class ScenarioConfig:
    """完整场景配置快照。

    一个场景把信号、传感器、声呐、跟踪、车辆、环境、埋深观测和可视化参数
    打包在一起，代表一次可直接运行的实验条件。它是本仓库场景切换和参数对比
    的最小单元。

    Attributes:
        name: 场景名称，通常也是命令行中用于选择场景的 key。
        description: 场景用途说明，建议写清“适合验证什么”。
        duration_s: 仿真总时长。
        dt_s: 主循环时间步长。
        signal: 电缆电流信号配置。
        sensor: 传感器模型配置。
        signal_processing: 信号处理配置。
        sonar: 声呐配置。
        tracking: 跟踪与恢复策略配置。
        vehicle: AUV 运动学配置。
        environment: 电缆与环境配置。
        survey: 埋深观测配置。
        visualization: 可视化配置。
    """

    name: str
    description: str
    duration_s: float
    dt_s: float
    signal: SignalConfig = field(default_factory=SignalConfig)
    sensor: SensorConfig = field(default_factory=SensorConfig)
    signal_processing: SignalProcessingConfig = field(default_factory=SignalProcessingConfig)
    sonar: SonarConfig = field(default_factory=SonarConfig)
    tracking: TrackingConfig = field(default_factory=TrackingConfig)
    vehicle: VehicleConfig = field(default_factory=VehicleConfig)
    environment: EnvironmentConfig = field(default_factory=EnvironmentConfig)
    survey: SurveyConfig = field(default_factory=SurveyConfig)
    visualization: VisualizationConfig = field(default_factory=VisualizationConfig)


def build_default_scenarios() -> Dict[str, ScenarioConfig]:
    """构建并返回全部默认示例场景。

    返回值中的每个场景都用于覆盖一类典型实验条件：
    - case1: 基线直线电缆跟踪
    - case2: 转弯/曲线适应性
    - case3: 高噪声鲁棒性
    - case4: 大姿态扰动补偿
    - case5: 低频演示模式对比
    - case6: 声呐与磁感知融合
    - case_hf_phone: 手机级高保真噪声
    - case_hf_industrial: 工业级高保真噪声
    - case8: 小曲率半径边界验证

    这些场景参数是实验调参的事实来源，建议优先通过复制场景再做局部修改，
    而不是直接改默认基线。
    """
    # 基线场景：尽量保持最少扰动，作为其他场景的比较基准。
    standard = ScenarioConfig(
        name="case1",
        description="Straight cable tracking baseline using the default 50 Hz AC mode.",
        duration_s=200.0,
        dt_s=0.05,
        signal=SignalConfig(mode="ac_50hz", frequency_hz=50.0, ac_current_amplitude_a=800.0),
        sensor=SensorConfig(
            magnetometer_sample_rate_hz=500.0,
            noise_std_nt=0.05,
            bias_drift_std_nt_per_s=0.01,
            nonorthogonality_deg=0.18,
            static_rotation_euler_deg=(0.0, 0.0, 0.0),
            dynamic_range_nt=100000.0,
        ),
        tracking=TrackingConfig(
            approach_angle_deg=45.0,
            turn_trigger_ratio=0.70,
            smoothing_time_constant_s=0.18,
            envelope_time_constant_s=0.22,
            peak_cooldown_s=0.75,
            min_peak_strength_nt=120.0,
            weighted_fitter_capacity=8,
        ),
        sonar=SonarConfig(prob_detection=0.75, position_noise_std_m=0.30, heading_noise_deg=4.0, update_rate_hz=10.0),
        vehicle=VehicleConfig(initial_position_ned_m=(-90.0, -10.0, 0.0), initial_heading_deg=35.0),
        environment=EnvironmentConfig(
            cable_waypoints_xy_m=((-320.0, 0.0), (360.0, 0.0)),
            cable_route_mode="spline",
            nominal_route_heading_deg=0.0,
            burial_depth_m=1.5,
        ),
    )

    # 转弯场景：通过多控制点路径和更宽松的声呐/跟踪参数测试转弯适应性。
    turning = ScenarioConfig(
        name="case2",
        description="Polyline cable turn scenario to verify forgetting-factor and centerline fitting updates.",
        duration_s=200.0,
        dt_s=0.05,
        signal=SignalConfig(mode="ac_50hz", frequency_hz=50.0, ac_current_amplitude_a=690.0, bandpass_half_width_hz=7.0),
        sonar=SonarConfig(prob_detection=0.82, position_noise_std_m=0.28, heading_noise_deg=3.5, update_rate_hz=10.0),
        tracking=TrackingConfig(
            approach_angle_deg=40.0,
            approach_angle_min_deg=32.0,
            approach_angle_max_deg=42.0,
            turn_trigger_ratio=0.80,
            envelope_time_constant_s=0.18,
            forgetting_factor=0.68,
            fit_history_size=8,
            min_peak_strength_nt=130.0,
            weighted_fitter_capacity=8,
            lost_timeout_s=6.0,
            sonar_preferred_distance_m=8.5,
            spiral_entry_window_s=3.5,
            spiral_radius_growth_mps=0.40,
            safe_lock_peak_drop_nt=14.0,
            guidance_memory_timeout_s=10.0,
            memory_guidance_confidence_floor=0.40,
        ),
        vehicle=VehicleConfig(
            cruise_speed_mps=1.05,
            search_speed_mps=1.05,
            max_yaw_rate_deg_s=38.0,
            min_turning_radius_m=2.5,
            initial_position_ned_m=(-110.0, -46.0, 0.0),
            initial_heading_deg=33.0,
        ),
        environment=EnvironmentConfig(
            cable_waypoints_xy_m=((-320.0, 0.0), (-40.0, 0.0), (120.0, 26.0), (260.0, 96.0), (420.0, 168.0)),
            cable_route_mode="spline",
            nominal_route_heading_deg=0.0,
            burial_depth_m=1.3,
        ),
    )

    # 高噪声场景：模拟手机级磁力计和更强姿态/噪声扰动。
    noisy = ScenarioConfig(
        name="case3",
        description="High-noise smartphone-like scenario to verify low-pass, hysteresis, and confidence degradation.",
        duration_s=200.0,
        dt_s=0.05,
        signal=SignalConfig(mode="ac_50hz", frequency_hz=50.0, ac_current_amplitude_a=700.0, bandpass_half_width_hz=6.0),
        sensor=SensorConfig(
            magnetometer_sample_rate_hz=200.0,
            noise_std_nt=150.0,
            bias_drift_std_nt_per_s=0.15,
            nonorthogonality_deg=1.5,
            static_rotation_euler_deg=(8.0, -5.0, 18.0),
            dynamic_range_nt=100000.0,
        ),
        tracking=TrackingConfig(
            approach_angle_deg=45.0,
            turn_trigger_ratio=0.86,
            smoothing_time_constant_s=0.35,
            envelope_time_constant_s=0.50,
            noise_floor_time_constant_s=2.5,
            hysteresis_fraction=0.14,
            peak_cooldown_s=1.2,
            min_peak_strength_nt=160.0,
            forgetting_factor=0.75,
            median_window_samples=9,
            search_leg_time_s=5.0,
            safe_lock_peak_drop_nt=22.0,
            magnetic_takeover_strength_nt=85.0,
            sonar_preferred_distance_m=9.0,
            weighted_fitter_capacity=8,
        ),
        sonar=SonarConfig(prob_detection=0.55, position_noise_std_m=0.9, heading_noise_deg=8.0),
        survey=SurveyConfig(burial_depth_noise_std_m=0.18, burial_depth_dropout_probability=0.10),
        vehicle=VehicleConfig(initial_position_ned_m=(-90.0, -42.0, 0.0), initial_heading_deg=28.0),
    )

    # 姿态扰动场景：突出静态安装误差与机体运动对坐标变换的影响。
    tilt = ScenarioConfig(
        name="case4",
        description="Large attitude disturbance scenario to verify static installation matrix and body-to-NED compensation.",
        duration_s=200.0,
        dt_s=0.05,
        signal=SignalConfig(mode="ac_50hz", frequency_hz=50.0, ac_current_amplitude_a=620.0, bandpass_half_width_hz=7.0),
        sensor=SensorConfig(
            magnetometer_sample_rate_hz=200.0,
            noise_std_nt=0.10,
            bias_drift_std_nt_per_s=0.02,
            nonorthogonality_deg=0.45,
            static_rotation_euler_deg=(20.0, -12.0, 35.0),
            dynamic_range_nt=100000.0,
            imu_heading_noise_deg=0.15,
            imu_tilt_noise_deg=0.08,
        ),
        vehicle=VehicleConfig(
            initial_position_ned_m=(-90.0, -44.0, 0.0),
            initial_heading_deg=35.0,
            pitch_amplitude_deg=12.0,
            roll_amplitude_deg=18.0,
            pitch_frequency_hz=0.06,
            roll_frequency_hz=0.08,
        ),
        tracking=TrackingConfig(
            approach_angle_deg=45.0,
            turn_trigger_ratio=0.84,
            smoothing_time_constant_s=0.22,
            envelope_time_constant_s=0.30,
            peak_cooldown_s=0.90,
            min_peak_strength_nt=100.0,
            median_window_samples=7,
            forgetting_factor=0.74,
            safe_lock_peak_drop_nt=18.0,
            weighted_fitter_capacity=8,
        ),
        sonar=SonarConfig(prob_detection=0.62, position_noise_std_m=0.55, heading_noise_deg=6.0),
    )

    # 演示场景：保留较低频率的实验模式，便于和早期方案对照。
    demo = ScenarioConfig(
        name="case5",
        description="10-20 Hz demo mode for comparison with the original experimental concept.",
        duration_s=200.0,
        dt_s=0.05,
        signal=SignalConfig(mode="ac_demo", frequency_hz=15.0, ac_current_amplitude_a=520.0, bandpass_half_width_hz=5.0),
        tracking=TrackingConfig(
            approach_angle_deg=45.0,
            turn_trigger_ratio=0.84,
            smoothing_time_constant_s=0.22,
            envelope_time_constant_s=0.26,
            peak_cooldown_s=0.85,
            min_peak_strength_nt=90.0,
            weighted_fitter_capacity=8,
        ),
        vehicle=VehicleConfig(initial_position_ned_m=(-85.0, -40.0, 0.0), initial_heading_deg=30.0),
        environment=EnvironmentConfig(
            cable_waypoints_xy_m=((-200.0, 0.0), (-60.0, 14.0), (60.0, -12.0), (200.0, 6.0)),
            cable_route_mode="sine",
            nominal_route_heading_deg=0.0,
            burial_depth_m=1.4,
        ),
    )

    # 融合场景：让声呐在磁信号不稳定时提供辅助引导。
    fusion = ScenarioConfig(
        name="case6",
        description="Sonar-magnetic fusion scenario with intermittent sonar and curved cable tracking.",
        duration_s=200.0,
        dt_s=0.05,
        signal=SignalConfig(mode="ac_50hz", frequency_hz=50.0, ac_current_amplitude_a=680.0, bandpass_half_width_hz=7.0),
        sensor=SensorConfig(
            magnetometer_sample_rate_hz=200.0,
            noise_std_nt=0.12,
            bias_drift_std_nt_per_s=0.03,
            nonorthogonality_deg=0.35,
            static_rotation_euler_deg=(5.0, -3.0, 10.0),
            dynamic_range_nt=100000.0,
            weak_signal_threshold_nt=20.0,
        ),
        sonar=SonarConfig(prob_detection=0.70, position_noise_std_m=0.35, heading_noise_deg=4.0),
        tracking=TrackingConfig(
            approach_angle_deg=40.0,
            turn_trigger_ratio=0.84,
            smoothing_time_constant_s=0.24,
            envelope_time_constant_s=0.24,
            peak_cooldown_s=0.85,
            min_peak_strength_nt=100.0,
            magnetic_takeover_strength_nt=65.0,
            safe_lock_peak_drop_nt=16.0,
            fit_acceptance_residual_m=10.0,
            weighted_fitter_capacity=8,
        ),
        vehicle=VehicleConfig(initial_position_ned_m=(-95.0, -48.0, 0.0), initial_heading_deg=32.0),
        environment=EnvironmentConfig(
            cable_waypoints_xy_m=((-220.0, -4.0), (-120.0, 22.0), (0.0, -18.0), (120.0, 28.0), (220.0, -2.0)),
            cable_route_mode="spline",
            nominal_route_heading_deg=0.0,
            burial_depth_m=1.5,
        ),
    )

    # 手机级高保真场景：保留基线结构，仅增强硬件噪声与较低采样率。
    hf_phone = copy.deepcopy(standard)
    hf_phone.name = "case_hf_phone"
    hf_phone.description = "High-fidelity phone-grade magnetometer scenario with 15 Hz AC and 100 Hz sampling."
    hf_phone.sensor.magnetometer_sample_rate_hz = 100.0
    hf_phone.sensor.high_fidelity = HighFidelityMagnetometerConfig(
        enabled=True,
        sampling_rate_hz=100.0,
        bit_depth=24,
        full_scale_nt=100000.0,
        auv_static_interference_body_nt=(26.0, -12.0, 8.0),
        white_noise_std_nt=0.04,
        pink_noise_std_nt=0.12,
        impulse_probability=0.004,
        impulse_amplitude_nt=42.0,
        impulse_decay_samples=6,
    )
    hf_phone.signal = SignalConfig(mode="ac_demo", frequency_hz=15.0, ac_current_amplitude_a=540.0, bandpass_half_width_hz=5.0)
    hf_phone.signal_processing = SignalProcessingConfig(
        window_size=96,
        overlap=0.8,
        window_function="hann",
        target_frequency_tolerance_hz=2.5,
        peak_search_half_width_hz=2.5,
        min_ac_frequency_hz=8.0,
        ac_energy_ratio_threshold=0.15,
        snr_detection_threshold_db=5.0,
        axis_combination_mode="dominant_axis",
    )
    hf_phone.visualization.psd_max_frequency_hz = 40.0
    hf_phone.visualization.update_stride_steps = 5

    # 工业级高保真场景：高采样率、较低随机噪声和更高频率的处理窗口。
    hf_industrial = copy.deepcopy(fusion)
    hf_industrial.name = "case_hf_industrial"
    hf_industrial.description = "High-fidelity industrial magnetometer scenario with 50 Hz AC and 1000 Hz sampling."
    hf_industrial.sensor.magnetometer_sample_rate_hz = 1000.0
    hf_industrial.sensor.high_fidelity = HighFidelityMagnetometerConfig(
        enabled=True,
        sampling_rate_hz=1000.0,
        bit_depth=24,
        full_scale_nt=100000.0,
        auv_static_interference_body_nt=(35.0, -20.0, 15.0),
        white_noise_std_nt=0.025,
        pink_noise_std_nt=0.06,
        impulse_probability=0.002,
        impulse_amplitude_nt=60.0,
        impulse_decay_samples=8,
    )
    hf_industrial.signal_processing = SignalProcessingConfig(
        window_size=512,
        overlap=0.85,
        window_function="hann",
        target_frequency_tolerance_hz=1.5,
        peak_search_half_width_hz=1.5,
        min_ac_frequency_hz=8.0,
        ac_energy_ratio_threshold=0.18,
        snr_detection_threshold_db=6.0,
        axis_combination_mode="dominant_axis",
    )
    hf_industrial.visualization.psd_max_frequency_hz = 80.0
    hf_industrial.visualization.update_stride_steps = 4

    # 紧曲率场景：用于验证小半径弯道、曲率约束和安全锁定恢复行为。
    tight_bend = ScenarioConfig(
        name="case8",
        description="Tight-curve scenario with curvature approaching 50 m minimum radius. Tests safe-lock and fit recovery at bends.",
        duration_s=300.0,
        dt_s=0.05,
        signal=SignalConfig(mode="ac_50hz", frequency_hz=50.0, ac_current_amplitude_a=680.0, bandpass_half_width_hz=7.0),
        sensor=SensorConfig(
            magnetometer_sample_rate_hz=200.0,
            noise_std_nt=0.08,
            bias_drift_std_nt_per_s=0.015,
        ),
        tracking=TrackingConfig(
            approach_angle_deg=40.0,
            turn_trigger_ratio=0.82,
            envelope_time_constant_s=0.20,
            peak_cooldown_s=0.80,
            min_peak_strength_nt=110.0,
            forgetting_factor=0.65,
            weighted_fitter_capacity=10,
            lost_timeout_s=5.0,
            safe_lock_peak_drop_nt=18.0,
            safe_lock_strength_ratio_threshold=0.20,
            safe_lock_ideal_field_width_m=12.0,
            safe_lock_displacement_factor=1.5,
            safe_lock_gradient_angle_threshold_deg=45.0,
            bootstrap_min_heading_diff_deg=30.0,
        ),
        vehicle=VehicleConfig(
            cruise_speed_mps=0.9,
            min_turning_radius_m=2.5,
            initial_position_ned_m=(-200.0, -50.0, 0.0),
            initial_heading_deg=25.0,
        ),
        environment=EnvironmentConfig(
            cable_waypoints_xy_m=(
                (-300.0, 0.0),
                (-100.0, 0.0),
                (-30.0, 40.0),
                (50.0, 50.0),
                (120.0, 10.0),
                (300.0, 10.0),
            ),
            cable_route_mode="spline",
            nominal_route_heading_deg=0.0,
            burial_depth_m=1.4,
            min_cable_curvature_radius_m=50.0,
            validate_curvature_on_build=True,
        ),
    )

    return {
        standard.name: standard,
        turning.name: turning,
        noisy.name: noisy,
        tilt.name: tilt,
        demo.name: demo,
        fusion.name: fusion,
        hf_phone.name: hf_phone,
        hf_industrial.name: hf_industrial,
        tight_bend.name: tight_bend,
    }


def get_scenario(case_name: str) -> Optional[ScenarioConfig]:
    """按名称查找默认场景配置。

    Args:
        case_name: 场景名称，通常对应命令行中的 case key。

    Returns:
        找到时返回对应的场景快照；找不到时返回 None。
    """
    return build_default_scenarios().get(case_name)
