# AUV Magnetic Cable Tracking Demo

这是一个基于洋葱架构拆分的 AUV 磁电缆跟踪数学仿真实验室 Phase 1 实现。它的目标不是复刻完整海上测绘系统，而是先把“数学灵魂”跑通：磁场生成、坐标补偿、传感器误差、过峰检测、中心线拟合、Zig-zag 控制和实时可视化在一个可调参、可扩展的 Python 工程里闭环联通。

## 架构

- `main_demo.py`: 根入口，负责选择案例并启动仿真。
- `src/auv_mag_tracking/config/__init__.py`: 场景配置、标准模式与 Demo 模式参数。
- `src/auv_mag_tracking/math_utils.py`: 旋转矩阵、坐标变换、有限线段 Biot-Savart 求场。
- `src/auv_mag_tracking/environment.py`: 电缆路由、海床、埋深真值和磁场环境。
- `src/auv_mag_tracking/sensor_model.py`: 磁力计、IMU 与埋深观测模拟。
- `src/auv_mag_tracking/perception/`: 感知包——带通提取、滑动 RMS、中值抑毛刺、迟滞过峰、中心线拟合、置信度融合、磁横偏与磁法埋深反演。
- `src/auv_mag_tracking/mission_manager.py`: 三态任务 FSM（SEARCH_ZIGZAG → LOCK_ALIGN → TRACK_ACTIVE，+ EMERGENCY 终态）。
- `src/auv_mag_tracking/controller.py`: 纵向/横向分轴运动学控制律。
- `src/auv_mag_tracking/main_viz.py`: 实时动画和状态看板。
- `src/auv_mag_tracking/experimental/`: 实验件隔离区（Phyphox 手机磁力计适配器、HoloOcean 连接器桩），不属于核心管线。

## 标准映射

本项目参考了以下两份 MinerU 解析文本中的检测语义与设备能力约束：

- `标准文档/MinerU_markdown_DLT+1278—2025+海底电力电缆运行规程_2039753922079608832.md`
- `标准文档/MinerU_markdown_1767166645680508_2039754169673572352.md`

当前落实到代码里的关键点包括：

- 默认 AC 模式使用 50Hz 工频，并保留 10-20Hz Demo 模式。
- 磁力计标准档噪声参考 0.05nT，手机档噪声参考 150nT。
- AC 场景下使用带通提取和两周期以上 RMS 窗口，减少采样走样和宽带噪声污染。
- “埋深”同时显示真值与估计值。除模拟的辅助埋深观测通道外，Phase 4 已实现纯磁法埋深反演（标定幅度法 + 横向门控，详见 `docs/08_磁法埋深反演.md`），在过线点附近由磁强度直接推算埋深。

没有直接硬编码进控制律的标准条款包括船载测线间距、尾拖拖鱼距底高度、船速等。这些更适合用来解释工业探测场景，不应直接变成 AUV 本体的运动控制参数。

## 运行

列出所有案例：

```bash
/usr/bin/python3 main_demo.py --list-cases
```

运行默认直线场景并显示实时动画：

```bash
/usr/bin/python3 main_demo.py --case case1
```

无图模式快速验证：

```bash
/usr/bin/python3 main_demo.py --case case3 --no-viz
```

## 预设案例

- `case1`: 50Hz 标准直线电缆跟踪。
- `case2`: 电缆折线转弯，重点看拟合遗忘因子是否跟得上。
- `case3`: 手机级大噪声，重点看低通、带通、迟滞与置信度降级。
- `case4`: 大姿态扰动，重点看静态安装矩阵与 Body 到 NED 补偿。
- `case5`: 10-20Hz Demo 模式，用于对照最初实验设想。

## 当前验证结果

本地已完成：

- `python3 -m unittest discover -s tests`
- `python3 main_demo.py --case case1 --no-viz`
- `python3 main_demo.py --case case2 --no-viz`
- `python3 main_demo.py --case case3 --no-viz`
- `python3 main_demo.py --case case4 --no-viz`
- `python3 main_demo.py --case case5 --no-viz`

## 下一步建议

如果继续推进到下一轮，优先级建议如下：

1. 为 case3 和 case4 加入自动调参脚本，系统搜索更优的阈值和滤波参数。
2. 将可视化输出保存为图片或视频，便于实验留档和论文插图。
3. 在 Phase 2 中引入更强的运动学或水动力学模型，把“贴缆前行”的视觉效果做得更接近实机。 