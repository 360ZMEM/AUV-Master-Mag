# AUV 磁力计非正交性校准引擎

本项目提供了一个基于“椭球拟合”算法的 Python 代码库，专门用于校准磁力计的非正交误差、比例因子误差（软铁）和零偏误差（硬铁）。校准后的数据能够达到亚纳特斯拉（< 0.05 nT）级别的精度要求，非常适合自主水下航行器 (AUV) 的高精度地磁导航任务。

## 文件结构

- `calibration_engine.py`: 核心校准引擎，实现了 `MagCalibrator` 类，包含基于最小二乘法的 9 参数模型拟合和数据校正功能。
- `data_simulator.py`: 合成数据生成器，包含 `DataSimulator` 类，能够生成静态球面点云数据和动态 AUV 轨迹（八字/螺旋摇摆）数据，并注入各种磁场畸变。
- `evaluate_demo.py`: 静态散点评估脚本，验证校准算法，并绘制校正前后的 3D 散点对比图。
- `evaluate_trajectory.py`: 动态轨迹评估脚本，模拟 AUV 真实机动情况下的磁场变化，并绘制误差曲线与轨迹对比图。
- `requirements.txt`: 依赖包列表。

## 环境安装

请确保你已经安装了 Python 3。建议在虚拟环境中运行：

```bash
# 创建并激活虚拟环境
python -m venv venv
source venv/bin/activate  # macOS/Linux
# venv\Scripts\activate    # Windows

# 安装依赖
pip install -r requirements.txt
```

## 使用方法

### 1. 运行静态评估测试

用于测试随机姿态下生成的球面散点数据的校准效果。

```bash
python evaluate_demo.py
```
运行后，终端将输出校正矩阵和偏移量，并展示误差统计结果。同时会弹出一个 3D 可视化窗口（并保存为 `calibration_result.png`），你可以直观地看到畸变的“蛋形”磁场被校正成了完美的球面。

### 2. 运行动态轨迹测试

模拟真实的 AUV 水下机动（如航向转圈、俯仰横滚摇摆）并验证校准精度。

```bash
python evaluate_trajectory.py
```
运行后，不仅会输出误差分析结果，还会生成一个包含 4 个子图的窗口（并保存为 `trajectory_result.png`），详细展示 AUV 姿态变化、模长误差随时间的变化曲线，以及校准前后的 3D 磁场轨迹。

### 3. 在你自己的项目中使用 `MagCalibrator`

你可以非常容易地将核心引擎集成到自己的代码中：

```python
import numpy as np
from calibration_engine import MagCalibrator

# 1. 准备你的原始磁力计数据 (Nx3 numpy 数组)
# B_raw = np.array([...])

# 2. 初始化校准器 (设置目标磁场模长，如当地地球磁场 50000 nT)
calibrator = MagCalibrator(target_norm=50000.0)

# 3. 拟合参数
M_corr, offset = calibrator.fit(B_raw)
print("校正矩阵 M:\n", M_corr)
print("偏移量 Offset:\n", offset)

# 4. 校正新的实时数据
B_corrected = calibrator.correct(B_raw)
```

## 算法原理

关于椭球拟合和数学推导的详细说明，请参考 [ALGORITHM.md](ALGORITHM.md)。