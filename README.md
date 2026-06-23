# AUV Magnetic Cable Tracking Demo

基于洋葱架构的 AUV 磁电缆跟踪数学仿真实验室。它不复刻完整海上测绘系统，而是把"数学灵魂"跑通并保持高度可维护：磁场生成、坐标补偿、传感器误差、过峰检测、中心线拟合、三态任务 FSM、纵/横分轴控制、磁法埋深反演与实时可视化在一个可调参、可扩展的 Python 工程里闭环联通。

> 完整文档体系（方法论 / 数学 / 配置 / 调优）见 [`docs/README.md`](docs/README.md)。

## 架构

- `main_demo.py`: 根入口，负责选择案例并启动仿真。
- `src/auv_mag_tracking/config/__init__.py`: 场景配置、标准模式与 Demo 模式参数。
- `src/auv_mag_tracking/math_utils.py`: 旋转矩阵、坐标变换、有限线段 Biot-Savart 求场。
- `src/auv_mag_tracking/environment.py`: 电缆路由、海床、埋深真值和磁场环境。
- `src/auv_mag_tracking/sensor_model.py`: 磁力计、IMU 与埋深观测模拟。
- `src/auv_mag_tracking/perception/`: 感知包——带通提取、滑动 RMS、中值抑毛刺、迟滞过峰、中心线拟合、置信度融合、磁横偏与磁法埋深反演。
- `src/auv_mag_tracking/mission_manager.py`: 三态任务 FSM（SEARCH_ZIGZAG → LOCK_ALIGN → TRACK_ACTIVE，+ EMERGENCY 终态）。
- `src/auv_mag_tracking/controller.py`: 纵向基准与横向压线分轴的运动学控制律。
- `src/auv_mag_tracking/main_viz.py`: 实时动画和状态看板。
- `src/auv_mag_tracking/viz/`: 离线可视化与回归对比体系（RunRecorder 单一采集契约、指标、图表、成果报告）。
- `src/auv_mag_tracking/experimental/`: 实验件隔离区（Phyphox 手机磁力计适配器、HoloOcean 连接器桩），不属于核心管线。

## 标准映射

本项目参考了以下两份 MinerU 解析文本中的检测语义与设备能力约束：

- `标准文档/MinerU_markdown_DLT+1278—2025+海底电力电缆运行规程_2039753922079608832.md`
- `标准文档/MinerU_markdown_1767166645680508_2039754169673572352.md`

当前落实到代码里的关键点包括：

- 默认 AC 模式使用 50Hz 工频，并保留 10-20Hz Demo 模式。
- 磁力计标准档噪声参考 0.05nT，手机档噪声参考 150nT。
- AC 场景下使用带通提取和两周期以上 RMS 窗口，减少采样走样和宽带噪声污染。
- "埋深"同时显示真值与估计值。除模拟的辅助埋深观测通道外，已实现纯磁法埋深反演（标定幅度法 + 横向门控，详见 [`docs/08_磁法埋深反演.md`](docs/08_磁法埋深反演.md)），在过线点附近由磁强度直接推算埋深。

没有直接硬编码进控制律的标准条款包括船载测线间距、尾拖拖鱼距底高度、船速等。这些更适合用来解释工业探测场景，不应直接变成 AUV 本体的运动控制参数。

## 运行

列出所有案例：

```bash
python3 main_demo.py --list-cases
```

运行默认直线场景并显示实时动画：

```bash
python3 main_demo.py --case case1
```

无图模式快速验证：

```bash
python3 main_demo.py --case case3 --no-viz
```

部署模式（禁用名义路由先验）：

```bash
python3 main_demo.py --case case1 --deployment-mode
```

离线生成全量案例的回归对比与成果报告（输出至 `results/<timestamp>/`）：

```bash
python3 tools/visualize.py --all
```

## 预设案例

- `case1`: 50Hz 标准直线电缆跟踪（基线，含磁法埋深反演标定）。
- `case2`: 电缆折线转弯，重点看拟合遗忘因子是否跟得上。
- `case3`: 手机级大噪声，重点看低通、带通、迟滞与置信度降级。
- `case4`: 大姿态扰动，重点看静态安装矩阵与 Body 到 NED 补偿。
- `case5`: 10-20Hz Demo 模式，用于对照最初实验设想。
- `case6`: 声呐-磁感知融合，间歇声呐 + 曲线电缆。
- `case8`: 小曲率半径（接近 50m 最小半径）边界验证。
- `case_hf_phone`: 手机级高保真磁力计（粉红噪声 + 脉冲噪声）。
- `case_hf_industrial`: 工业级高保真磁力计（高采样率、低噪声）。

## 当前验证结果

本地已完成：

- `python3 -m unittest discover -s tests`（60 用例全绿）
- `python3 main_demo.py --case case{1..6} --no-viz`
- `python3 tools/visualize.py --all`（case1–5 + showcase + 进度对比报告）

核心指标（`tools/visualize.py --all` 实测）：

- 模式切换次数全 case 收敛到 2–4 次（相比行为树时代大幅降抖）。
- case1–4 融合航向误差 0.4–2.7°；case5 因直线拟合在急弯冻结达 22.8°（曲率感知拟合为后续项）。
- case1 磁法埋深反演 MAE 0.056m；其余非标定场景正确门控为 NaN（不输出错误埋深）。

## 下一步建议

如果继续推进到下一轮，优先级建议如下：

1. **曲率感知拟合**：现有 `WeightedSlidingWindowFitter` 为直线模型，在 R < 50m 急弯（如 case5）拟合冻结导致显著航向偏差，需引入圆弧/样条模型。
2. **自动调参**：为高噪声场景（case3）与高保真场景接入 `tools/sweep_tracking_params.py` 系统搜索更优阈值。
3. **实时集成**：将 `experimental/simulator_connector` 对接 HoloOcean，把"贴缆前行"推向更接近实机的水动力学闭环。
