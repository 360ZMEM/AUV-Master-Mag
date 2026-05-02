import numpy as np

class DataSimulator:
    """
    合成磁力计数据生成器。
    
    模拟一个完美的球面磁场（例如，50,000 nT 的地球磁场），
    并引入以下畸变：
    1. 比例因子误差（软铁误差 / 传感器增益不匹配）
    2. 非正交性（坐标轴之间不完全是 90 度）
    3. 硬铁偏移（恒定偏置）
    4. 高斯噪声
    """
    
    def __init__(self, B_earth=50000.0):
        self.B_earth = B_earth
        
    def generate(self, num_points=2000, seed=42):
        """
        生成原始和真实的磁场数据。
        
        :param num_points: 要生成的数据点数量
        :param seed: 用于保证结果可重复的随机种子
        :return: (B_raw, B_true) 形式为 Nx3 的数组
        """
        np.random.seed(seed)
        
        # 1. 生成理想的磁场矢量（完美球面）
        # 使用随机高斯点并归一化到单位球面
        pts = np.random.randn(num_points, 3)
        pts /= np.linalg.norm(pts, axis=1)[:, np.newaxis]
        B_true = pts * self.B_earth
        
        # 2. 比例因子误差（例如，各轴具有略微不同的增益）
        # x: 1.01 (1% 误差), y: 0.98 (-2% 误差), z: 1.02 (2% 误差)
        scale_matrix = np.diag([1.01, 0.98, 1.02])
        
        # 3. 非正交性
        # 不对准角度（弧度）
        alpha = np.radians(0.2)   # X-Y 不对准（89.8度而非90度）
        beta = np.radians(0.3)    # X-Z 不对准
        gamma = np.radians(-0.1)  # Y-Z 不对准
        
        # 构造非正交矩阵
        # 假设 X 轴对齐，Y 轴在 XY 平面内，Z 轴倾斜
        non_ortho_matrix = np.array([
            [1.0, 0.0, 0.0],
            [np.sin(alpha), np.cos(alpha), 0.0],
            [np.sin(beta), np.sin(gamma), np.sqrt(1.0 - np.sin(beta)**2 - np.sin(gamma)**2)]
        ])
        
        # 组合畸变变换矩阵
        # B_distorted = non_ortho_matrix * scale_matrix * B_true
        T_distortion = non_ortho_matrix @ scale_matrix
        
        # 4. 硬铁偏移
        # 不同轴上具有 300 到 500 nT 的偏置
        offset = np.array([300.0, -500.0, 400.0])
        
        # 应用变换
        # 对于 Nx3 数组，等价于：B_raw = B_true @ T_distortion.T + offset
        B_raw = B_true @ T_distortion.T + offset
        
        # 5. 添加高斯噪声（0.01 nT 级别，用于亚纳特斯拉精度测试）
        noise = np.random.normal(0, 0.01, (num_points, 3))
        B_raw += noise
        
        return B_raw, B_true

    def generate_trajectory(self, duration=100.0, sample_rate=10.0, seed=42):
        """
        基于动态 AUV 轨迹（八字形并伴有俯仰/横滚摇摆）生成合成数据。
        
        :param duration: 机动总时间（秒）。
        :param sample_rate: 传感器采样率（Hz）。
        :param seed: 随机种子。
        :return: (t, B_raw, B_true, euler_angles)
        """
        np.random.seed(seed)
        from scipy.spatial.transform import Rotation
        
        # 时间向量
        t = np.arange(0, duration, 1.0 / sample_rate)
        num_points = len(t)
        
        # 模拟 AUV 随时间变化的姿态
        # 航向角 (Yaw)：八字形或扫掠（例如，从 0 到 4*pi）
        # 我们将在航向角上进行完整的双圆旋转，并结合俯仰角和横滚角的正弦波动
        yaw = np.linspace(0, 4 * np.pi, num_points)
        
        # 俯仰角 (Pitch)：在 -30 到 +30 度之间摇摆
        pitch_max = np.radians(30)
        pitch = pitch_max * np.sin(2 * np.pi * t / (duration / 4))
        
        # 横滚角 (Roll)：在 -20 到 +20 度之间摇摆（频率不同）
        roll_max = np.radians(20)
        roll = roll_max * np.sin(2 * np.pi * t / (duration / 3))
        
        euler_angles = np.column_stack((yaw, pitch, roll))
        
        # 创建从机体系到导航系的旋转矩阵
        # 'ZYX' 欧拉角对应 航向(Yaw), 俯仰(Pitch), 横滚(Roll)
        rotations = Rotation.from_euler('ZYX', euler_angles)
        
        # 定义导航系中的地球恒定磁场（北、东、地）
        # 假设磁倾角约为 60 度，总模长为 B_earth
        dip_angle = np.radians(60)
        B_ned = np.array([self.B_earth * np.cos(dip_angle), 0.0, self.B_earth * np.sin(dip_angle)])
        
        # 传感器坐标系（机体系）中的真实磁场 B_true 为 R_nav2body * B_ned
        # R_nav2body 是 R_body2nav 的逆矩阵
        rotations_inv = rotations.inv()
        B_true = rotations_inv.apply(B_ned)
        
        # 应用相同的传感器畸变
        scale_matrix = np.diag([1.01, 0.98, 1.02])
        
        alpha = np.radians(0.2)
        beta = np.radians(0.3)
        gamma = np.radians(-0.1)
        
        non_ortho_matrix = np.array([
            [1.0, 0.0, 0.0],
            [np.sin(alpha), np.cos(alpha), 0.0],
            [np.sin(beta), np.sin(gamma), np.sqrt(1.0 - np.sin(beta)**2 - np.sin(gamma)**2)]
        ])
        
        T_distortion = non_ortho_matrix @ scale_matrix
        offset = np.array([300.0, -500.0, 400.0])
        
        # 将 B_true 变换为 B_raw
        B_raw = B_true @ T_distortion.T + offset
        
        # 添加噪声
        noise = np.random.normal(0, 0.01, (num_points, 3))
        B_raw += noise
        
        return t, B_raw, B_true, euler_angles
