import numpy as np
import matplotlib.pyplot as plt
from calibration_engine import MagCalibrator
from data_simulator import DataSimulator

def set_axes_equal(ax):
    """
    使 3D 图形的各个坐标轴具有相同的比例，从而使球体看起来像球体而不是椭球。
    """
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

def main():
    B_EARTH = 50000.0
    
    # 1. 生成合成数据
    print("正在生成合成磁力计数据...")
    simulator = DataSimulator(B_earth=B_EARTH)
    B_raw, B_true = simulator.generate(num_points=2000)
    
    # 2. 训练校准器
    print("正在使用椭球拟合训练 MagCalibrator...")
    calibrator = MagCalibrator(target_norm=B_EARTH)
    M_corr, offset = calibrator.fit(B_raw)
    
    # 3. 应用校准
    print("正在应用校准矩阵...")
    B_corrected = calibrator.correct(B_raw)
    
    # 4. 评估结果
    norm_raw = np.linalg.norm(B_raw, axis=1)
    norm_corrected = np.linalg.norm(B_corrected, axis=1)
    
    error_raw = norm_raw - B_EARTH
    error_corrected = norm_corrected - B_EARTH
    
    print("\n" + "="*40)
    print("             校准结果")
    print("="*40)
    print(f"估计的偏移量 (nT): \n{offset}")
    print(f"\n估计的校正矩阵:\n{M_corr}")
    
    print("\n" + "="*40)
    print("           误差分析（模长）")
    print("="*40)
    print(f"原始数据误差（标准差）: {np.std(error_raw):8.4f} nT")
    print(f"原始数据误差（最大值）: {np.max(np.abs(error_raw)):8.4f} nT")
    print("-" * 40)
    print(f"校正后误差（标准差）:   {np.std(error_corrected):8.4f} nT")
    print(f"校正后误差（最大值）:   {np.max(np.abs(error_corrected)):8.4f} nT")
    print("="*40)
    
    if np.std(error_corrected) < 0.05:
        print("\n成功：校正后的模长方差证明已达到亚纳特斯拉级精度（目标：0.05 nT）。")
    else:
        print("\n警告：未达到 0.05 nT 的目标精度。")
        
    # 5. 可视化
    print("\n正在生成 3D 可视化图形...")
    fig = plt.figure(figsize=(14, 6))
    
    # 原始数据图
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.scatter(B_raw[:, 0], B_raw[:, 1], B_raw[:, 2], s=2, c='r', alpha=0.5, label='原始数据（畸变）')
    ax1.set_title("原始磁力计数据\n(蛋形且带偏移)")
    ax1.set_xlabel("X (nT)")
    ax1.set_ylabel("Y (nT)")
    ax1.set_zlabel("Z (nT)")
    set_axes_equal(ax1)
    ax1.legend()
    
    # 校正后数据图
    ax2 = fig.add_subplot(122, projection='3d')
    ax2.scatter(B_corrected[:, 0], B_corrected[:, 1], B_corrected[:, 2], s=2, c='b', alpha=0.5, label='校正后数据（球面）')
    ax2.set_title("校正后的磁力计数据\n(完美球面)")
    ax2.set_xlabel("X (nT)")
    ax2.set_ylabel("Y (nT)")
    ax2.set_zlabel("Z (nT)")
    set_axes_equal(ax2)
    ax2.legend()
    
    plt.tight_layout()
    plt.savefig('calibration_result.png', dpi=150)
    print("可视化结果已保存为 'calibration_result.png'。")
    plt.show()

if __name__ == "__main__":
    main()
