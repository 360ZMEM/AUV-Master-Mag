import numpy as np
from scipy.linalg import sqrtm

class MagCalibrator:
    """
    基于椭球拟合的磁力计校准引擎。
    
    数学原理：
    畸变的磁场读数 (B_raw) 可以建模为：
        B_raw = M_distortion * B_true + Offset
    其中 B_true 位于半径为 R 的完美球面上（例如地球磁场）。
    
    重排等式，得到：
        B_true = M_correction * (B_raw - Offset)
    其中 M_correction = M_distortion^(-1)。
    
    因为 ||B_true||^2 = R^2，我们可以写出：
        (B_raw - Offset)^T * (M_correction^T * M_correction) * (B_raw - Offset) = R^2
    令 A = M_correction^T * M_correction / R^2，等式变为：
        (B_raw - Offset)^T * A * (B_raw - Offset) = 1
        
    展开此式可得椭球的一般方程：
        v^T * A * v - 2 * (A * Offset)^T * v + Offset^T * A * Offset - 1 = 0
    其中 v = B_raw。
    
    通过拟合二次曲面 (a*x^2 + b*y^2 + ... = 1) 的 9 个参数，
    我们可以恢复对称正定矩阵 A 和偏移向量 (Offset)。
    最后，通过对 (R^2 * A) 求矩阵平方根即可得到 M_correction。
    """
    
    def __init__(self, target_norm=50000.0):
        """
        :param target_norm: 期望的磁场模长（例如，50,000 nT）
        """
        self.target_norm = target_norm
        self.M = np.eye(3)
        self.offset = np.zeros(3)

    def fit(self, B_raw):
        """
        使用线性最小二乘法（椭球拟合）求解 9 参数模型。
        
        :param B_raw: Nx3 的原始磁力计数据 Numpy 数组
        :return: (M, offset) 其中 M 是 3x3 校正矩阵，offset 是 3x1 偏移向量
        """
        x = B_raw[:, 0]
        y = B_raw[:, 1]
        z = B_raw[:, 2]

        # 设计矩阵 D，用于方程：a*x^2 + b*y^2 + c*z^2 + 2d*xy + 2e*xz + 2f*yz + g*x + h*y + i*z = 1
        D = np.column_stack([
            x**2, y**2, z**2,
            2 * x * y,
            2 * x * z,
            2 * y * z,
            x, y, z
        ])

        # 目标向量（全 1）
        ones = np.ones(len(x))

        # 使用最小二乘法求解 D * P = 1
        P, _, _, _ = np.linalg.lstsq(D, ones, rcond=None)

        # 提取 9 个参数
        a, b, c, d, e, f, g, h, i = P

        # 构造对称矩阵 A_fit 和向量 U_fit
        A_fit = np.array([
            [a, d, e],
            [d, b, f],
            [e, f, c]
        ])
        U_fit = np.array([g, h, i])

        # 计算偏移量 (Offset)
        # 根据推导：U_fit = -2 * A_fit @ offset => offset = -0.5 * inv(A_fit) @ U_fit
        self.offset = -0.5 * np.linalg.inv(A_fit) @ U_fit

        # 计算比例因子 c_scale
        # 常数项为：Offset^T * A_fit * Offset + c_scale = 1  => c_scale = 1 / (1 + Offset^T * A_fit * Offset)
        # 实际的矩阵 A_actual = A_fit / (1 + Offset^T A_fit Offset)
        c_scale = 1.0 / (1.0 + self.offset.T @ A_fit @ self.offset)
        A_actual = A_fit * c_scale

        # 计算 W = M^T * M
        W = (self.target_norm ** 2) * A_actual

        # 计算 M，即 W 的矩阵平方根
        # 由于 W 是对称正定矩阵，其平方根也是实对称矩阵。
        self.M = np.real(sqrtm(W))

        return self.M, self.offset

    def correct(self, B_raw):
        """
        对原始数据应用校准。
        
        :param B_raw: Nx3 的原始磁力计数据 Numpy 数组
        :return: Nx3 的校正后数据 Numpy 数组
        """
        # B_corrected = M * (B_raw - Offset)
        # 对于 Nx3 数组，这等价于：(B_raw - Offset) @ M.T
        return (B_raw - self.offset) @ self.M.T
