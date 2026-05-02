import numpy as np
import matplotlib.pyplot as plt
from calibration_engine import MagCalibrator
from data_simulator import DataSimulator
from evaluate_demo import set_axes_equal

def main():
    B_EARTH = 50000.0
    
    print("正在模拟动态 AUV 轨迹（八字形与螺旋摇摆）...")
    simulator = DataSimulator(B_earth=B_EARTH)
    
    # 生成 100 秒的轨迹数据，采样率为 10Hz
    t, B_raw, B_true, euler = simulator.generate_trajectory(duration=100.0, sample_rate=10.0)
    
    print(f"已生成 {len(t)} 个数据样本。")
    
    print("正在使用动态轨迹数据训练 MagCalibrator...")
    calibrator = MagCalibrator(target_norm=B_EARTH)
    M_corr, offset = calibrator.fit(B_raw)
    
    print("正在将校准应用到轨迹数据...")
    B_corrected = calibrator.correct(B_raw)
    
    # 计算模长和误差
    norm_raw = np.linalg.norm(B_raw, axis=1)
    norm_corrected = np.linalg.norm(B_corrected, axis=1)
    
    error_raw = norm_raw - B_EARTH
    error_corrected = norm_corrected - B_EARTH
    
    print("\n" + "="*45)
    print("         动态轨迹校准结果")
    print("="*45)
    print(f"估计的偏移量 (nT): \n{offset}")
    print(f"\n估计的校正矩阵:\n{M_corr}")
    
    print("\n" + "="*45)
    print("               误差分析（模长）")
    print("="*45)
    print(f"原始数据误差（标准差）: {np.std(error_raw):8.4f} nT")
    print(f"原始数据误差（最大值）: {np.max(np.abs(error_raw)):8.4f} nT")
    print("-" * 45)
    print(f"校正后误差（标准差）:   {np.std(error_corrected):8.4f} nT")
    print(f"校正后误差（最大值）:   {np.max(np.abs(error_corrected)):8.4f} nT")
    print("="*45)
    
    if np.std(error_corrected) < 0.05:
        print("\n成功：在动态仿真下已满足目标 0.05 nT 的精度要求。")
    else:
        print("\n警告：未满足目标精度要求。")
        
    # 可视化
    fig = plt.figure(figsize=(15, 10))
    
    # 图 1：欧拉角
    ax1 = fig.add_subplot(221)
    ax1.plot(t, np.degrees(euler[:, 0]), label='航向角 (Yaw)')
    ax1.plot(t, np.degrees(euler[:, 1]), label='俯仰角 (Pitch)')
    ax1.plot(t, np.degrees(euler[:, 2]), label='横滚角 (Roll)')
    ax1.set_title("AUV 姿态（欧拉角）")
    ax1.set_xlabel("时间 (s)")
    ax1.set_ylabel("角度 (度)")
    ax1.grid(True)
    ax1.legend()
    
    # 图 2：模长误差随时间的变化
    ax2 = fig.add_subplot(222)
    ax2.plot(t, error_raw, 'r', alpha=0.5, label='原始误差')
    ax2.plot(t, error_corrected, 'b', label='校正后误差')
    ax2.set_title("模长误差随时间的变化")
    ax2.set_xlabel("时间 (s)")
    ax2.set_ylabel("误差 (nT)")
    ax2.grid(True)
    ax2.legend()
    
    # 图 3：3D 原始轨迹
    ax3 = fig.add_subplot(223, projection='3d')
    p3 = ax3.scatter(B_raw[:, 0], B_raw[:, 1], B_raw[:, 2], c=t, cmap='viridis', s=5, alpha=0.8)
    ax3.set_title("原始磁场轨迹（畸变）")
    ax3.set_xlabel("X (nT)")
    ax3.set_ylabel("Y (nT)")
    ax3.set_zlabel("Z (nT)")
    set_axes_equal(ax3)
    fig.colorbar(p3, ax=ax3, label="时间 (s)", shrink=0.5)
    
    # 图 4：3D 校正后轨迹
    ax4 = fig.add_subplot(224, projection='3d')
    p4 = ax4.scatter(B_corrected[:, 0], B_corrected[:, 1], B_corrected[:, 2], c=t, cmap='viridis', s=5, alpha=0.8)
    ax4.set_title("校正后的磁场轨迹（球面）")
    ax4.set_xlabel("X (nT)")
    ax4.set_ylabel("Y (nT)")
    ax4.set_zlabel("Z (nT)")
    set_axes_equal(ax4)
    fig.colorbar(p4, ax=ax4, label="时间 (s)", shrink=0.5)
    
    plt.tight_layout()
    plt.savefig('trajectory_result.png', dpi=150)
    print("可视化结果已保存为 'trajectory_result.png'。")
    plt.show()

if __name__ == "__main__":
    main()
