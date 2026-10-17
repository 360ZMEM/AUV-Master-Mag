# AUV Magnetic Cable Tracking v2

新增[扰流／导航／多源的条件性稳健性实验](../../results/v2/defense_robustness_20260929/README.md)与[风险边界](DEFENSE_ROBUSTNESS_BOUNDARIES_20260929.md)：冻结留出 120 次、无参考 9 次、仅改接入 3 次；新增独立 opt-in 监督和方向接入。默认 ROS 工厂与生产控制器未接入这些实验模块，本组不含 DDS 时序或实艇部署证据。

新增可选 [独立巡检输出契约（20260926）](INSPECTION_CONTRACT_20260926.md)：横向定位与埋深独立授权，有界侧翼移心、观测更新与故障保持。默认关闭，使用专用 `cable_tracking_v2_inspection.yaml`；40 次运动学复核和 DDS smoke 仅覆盖报告所列条件。

原生 PVS 后续诊断见 [浅角度穿越与场形拒绝](FLYTHROUGH_PVS_AND_SHAPE_GATE_20260926.md)：短外侧尾部时旧残差比较可能偏向错误场形，巡检深度发布现要求等自由度背景比较及双侧各 2 个 FWHM 的形状支持；横向中心继续独立授权。独立 `FlythroughInspectionPipelineV2` 在 3+3 个有源种子完成协议编解码驱动的 PVS 诊断任务，3 个无源种子均停止。后续已用显式 `v2.mission_geometry: flythrough` 配置接入 ROS2，并完成 ROS2＋PID＋协议编解码＋原生 PVS 的零流、双向 0.05 m/s 横流及短时磁故障闭环；单直导线零流、南北横流、磁故障、无源、超包络和偶极零流也经过真实自治守卫与本地 UDP 桥回环。PC104、实际上位机链路和实物磁源仍未接入，完整指标及图见同一报告。

持续故障和接管的执行许可见[独立安全链路报告](INSPECTION_SAFETY_CHAIN_20260926.md)：巡检控制器现在在自治状态缺失、撤销或守卫拒绝时自行输出零执行器，与桥的最终零推力形成两层控制。手动取消、持续坏磁输入和过期遥测各 241 周期，以及短时坏导航 60 周期，已在本地 UDP/PVS 回路验证；仍需实艇停止距离与硬件链路验收。

侧向先验选择穿越方向的条件化包络见[镜像航迹与错误方向反例](INSPECTION_SIDE_HINT_20260926.md)：同一种子与零流下，电缆相对先验位于 $+6/-6$ m 时，分别从南/北侧起步均完成 80 m 跟踪；侧向符号选错则扫描超时并保持。先验符号由试验预置，尚未验证真实地图或声呐来源，埋深保持未发布。

ROS 浅角度模式现强制检查侧向先验区间及来源标签：缺失、非数值、跨先验中心线、方向不符或超出扫描端均在启动首周期保持、PID 与桥双层零推进。镜像零流各 3 个种子完成，北侧起步在南北向 0.05 m/s 横流各完成一个种子；图 23 与逐例数据见上述报告。当前 YAML 的来源为 `synthetic_test_fixture`，仅用于仿真，不能作为现场侧向真值证明。

非仿真 ROS 时钟下，节点还会拒绝 `synthetic_test_fixture` 来源并保持零速。无运行时声呐的地图/上一段航迹路径现须提供[静态目标先验审查清单](STATIC_TARGET_PRIOR.md)，绑定源文件、路线点、坐标系、窄侧向区间及有效期；接令时任务与目标 ID 须匹配并重验文件，运行中过期则锁存零速 Hold。这能防止配置误改、任务串线和过期继续推进，来源真实性仍由现场目标包验收。运行时声呐路径见[声磁身份门](INSPECTION_RUNTIME_SONAR_IDENTITY_20260927.md)。

清单门改动 ROS 入口后的[同次合成链复核](INSPECTION_OPERATOR_SAME_RUN_STATIC_PRIOR_20260927.md)保留独立原始 CSV、界面轨迹与审计哈希；现场许可边界不变。

[受限运行契约与现场验收入口](INSPECTION_RELEASE_ENVELOPE_20260926.md)把任务预检、横向/深度双输出许可、跟踪与执行撤权放在同一流程中，并逐项列出把仿真候选变为实艇作业包络所需的测量证据。

[现场目标包与执行包预检](INSPECTION_FIELD_PACKAGE_PREFLIGHT_20260927.md)提供可填写清单、原始数据追溯检查和描述性指标提取；当前空模板不通过，现场许可仍为否。

[独立进程任务与故障输出复核](INSPECTION_PROCESS_RESTART_DIAGNOSTICS_20260927.md)用当前源码分别重启正常巡检、上行失鲜和线索改指合成任务，提供图 37、原始 CSV 哈希与逐周期审计。两次正常任务完成，故障保持零命令但 PVS 仍滑行；它不代表实艇或连续耐久试验。

侧向幅值扫掠另发现最初演示区间 `±(2–10) m` 过宽：偏差 ±10 m 的外侧半高交点超出 ±15 m 扫描终点，PVS 双向都拒绝横向确认。图 24 给出解析原因和双向结果；正式示例区间已收紧为 `±(2–8) m`，并在启动门要求 7 m 峰外几何预算，详见[方向与幅值报告](INSPECTION_SIDE_HINT_20260926.md)。

图 25 的侧偏／横流八角点复核先修正了假双峰和有限扫描时限，八例均到终点，但四个近侧案例仍超出示例横偏均值门。后续[有界上游接入段复核](INSPECTION_REJOIN_POLICY_20260926.md)把回归路线起点的转弯前移，并按导航估得的横向流方向选择 20/5 m 接入段；图 26 展示完整沿缆误差的改善。该结果只覆盖合成场、预对准走廊和所列离散角点，未改变埋深拒绝与实测标定要求。

精确导航输入的适用边界进一步见[导航误差与路线质量拒绝](INSPECTION_NAVIGATION_QUALITY_20260926.md)：v2 明确世界系地速与新鲜协方差合同，并在路由终点用独立的沿缆观测质量门决定任务是否有效；固定导航偏差超出候选包络时保持且不误报完成。图 27 给出位置差分误停、质量通过和质量拒绝的因果对照。

v2 是与现有实现并列的独立模块，闭合以下任务链：

```text
Prior-guided Search
→ Magnetic Detection
→ Active Cross Scan
→ Explicit FWHM
→ Cable Tracking / Reacquire
```

v1 的源码、默认入口、配置和历史结果保持不变。v2 的 Python 包位于
`src/auv_mag_tracking/v2/`，ROS2 适配器是主仓中的独立包
`brain_linux/src/auv_cable_tracking_v2/`。

## 模块

| 文件 | 职责 |
|---|---|
| `search.py` | 根据电缆先验、不确定度、搜索宽度和航线间距生成交替覆盖航点 |
| `detection.py` | 滑窗 median/MAD 分数、双阈值迟滞、连续帧门控和峰值证据 |
| `cross_scan.py` | 沿局部电缆法向生成分档扩幅的往返横切航点；采样前执行航向对准 |
| `fwhm.py` | 构建 `B(x)`，计算背景修正半峰、左右插值交点、宽度和有效性 |
| `cable_estimator.py` | 将横切中心变换为独立电缆观测点并拟合局部走向 |
| `tracking.py` | 将观测横向修正投到有限先验走廊，执行预瞄、限幅、调速和终点保持 |
| `state_machine.py` | SEARCH、CONFIRM、TRACK、REACQUIRE 状态转换 |
| `pipeline.py` | 与 v1 `step/step_with_guidance` 输入输出形态兼容的门面 |
| `physics.py` | 与 v1 隔离的量纲一致有限直导线物理对照模型 |

## FWHM 约定

v2 优先使用导航位置投影后的局部法向空间坐标 `x`，不依赖匀速假设。半峰定义为：

```text
B_half = B_background + 0.5 * (B_peak - B_background)
```

Detection 阶段的独立背景估计应传给 FWHM。仅用横切窗口边缘估背景是降级路径，因为边缘可能仍含电缆尾场。

深度换算必须携带场模型：

```text
vertical_separation = shape_factor * FWHM
burial_depth = vertical_separation - altitude
```

- 论文偶极包络 `B=C/(x²+z²)`：`shape_factor=0.5`；
- 物理单直导线 `B∝1/sqrt(x²+z²)`：`shape_factor=1/(2sqrt(3))≈0.288675`；
- v1 legacy 线段公式在 case1 的 2–8 m 扫描中得到 0.433–0.495，不能与前两者混用。

因此这里的“无参”只表示半高位置不受整体幅值尺度影响。场形系数、传感器高度和动态横切几何仍必须给出不确定度。v2 输出 `vertical_separation_sigma_m` 与 `burial_depth_sigma_m`，后者同时包含交点分辨率、shape factor 相对不确定度和高度不确定度。

现在发布埋深前还会比较声明场形与备选场形在同一实测峰值、中心和宽度下的归一化残差。备选场形明显更优时返回 `competing_shape_model_preferred`；若任一侧峰外支持不足 0.65 个 FWHM，则返回 `shape_evidence_insufficient`。两者都不发布垂向距离或埋深，原因码和两种残差进入 diagnostics。该门已在偶极包络与解析单直导线的双向失配留出集验证；PVS 中点电流元和真实多芯铠装海缆仍需独立标定。单条剖面也无法检测正确形状配上错误的深度系数。

部署巡检模式另有更严格的深度门：声明和备选场形各自拟合一个背景量，并要求声明残差至少低 0.02；双侧最短支持达到 2 个 FWHM。原生 PVS 浅角度穿越发现旧比较在短侧翼真直导线场景会误偏向偶极，见上述诊断报告。这些门不能替代现场系数与高度标定。

详见 [MODEL_CONSISTENCY.md](MODEL_CONSISTENCY.md)。

## Python 快速验证

```bash
cd AUV-Master-Mag
PYTHONPATH=src python3 -m unittest discover -s tests/v2 -v
```

运行场模型一致性实验：

```bash
PYTHONPATH=src python3 tools/v2_fwhm_model_consistency.py
PYTHONPATH=src python3 tools/run_v2_shape_guard_validation.py
```

输出到 `results/v2/fwhm_model_consistency/`，包含三种模型的横截面、CSV 和摘要。

运行 Cases A–D 与跟踪基线：

```bash
PYTHONPATH=src python3 tools/run_v2_experiments.py --seeds 20
```

输出到 `results/v2/cases_a_d/`。每次运行重写 v2 自己的同名结果，不读取或覆盖 v1 历史目录。

运行 50 种子预声明消融：

```bash
PYTHONPATH=src python3 tools/run_v2_ablations.py --seeds 50
```

输出到 `results/v2/ablations/`，覆盖背景修正、空间坐标、横切方式与幅度、检测迟滞、
FWHM/幅值反演、位置来源和重复扫描；结论见 [ABLATION_BASELINE.md](ABLATION_BASELINE.md)。

用现有同步 TMR--ArUco 数据审计真实剖面的 FWHM 适用性：

```bash
PYTHONPATH=src python3 tools/v2_replay_tmr_joint.py
```

该工具只读原实验数据并输出到 `results/v2/tmr_joint_replay/`。当前 9 条候选
扫描腿均缺少完整双侧半高交点，因此全部拒绝发布深度；详见
[TMR_JOINT_REPLAY.md](TMR_JOINT_REPLAY.md)。

## Python API

```python
from auv_mag_tracking.v2 import AuvMagTrackingPipelineV2, V2Config

pipeline = AuvMagTrackingPipelineV2(
    scenario,
    cable_map,
    v2_config=V2Config(),
    search_boundary_xy_m=boundary_xy,
    initial_position_uncertainty_m=15.0,
)

tracking, guidance = pipeline.step_with_guidance(navigation, magnetic)
```

输出仍是现有 `CableTrackingOutput` 和 `CableGuidanceOutput`。新增信息位于 diagnostics：

- `mission_phase`、`detection_score`、`detected`；
- Search/Confirm 航点索引与 scan id；
- 峰值、背景、左右半高交点和 FWHM；
- `fwhm_validity_score`、`fwhm_confidence`、深度/埋深 sigma 与原因码；
- `cross_scan_aligned`、沿缆漂移范围和采样数；
- 独立观测是否可用、相对先验横向修正、有限走廊目标点和 `endpoint_complete`。

## ROS2

构建独立包：

```bash
cd brain_linux
source /opt/ros/humble/setup.bash
colcon build --packages-select auv_cable_tracking_v2 --symlink-install
source install/setup.bash
```

启动：

```bash
ros2 launch auv_cable_tracking_v2 cable_tracking_v2.launch.py
```

它复用 v1 节点已经稳定的订阅、发布、传感器门控和 setpoint 限幅，但将算法 facade 换成 v2。旧正弦 probe 被强制关闭，避免和 v2 Confirm 横切航点叠加。原 `auv_decision_ros/cable_tracking_node.py` 及其 launch 不变。
v2 适配器另订阅 `/auv/sensors/altitude`：在 `altitude_timeout_s` 内收到有效值时，FWHM 使用动态离底高度；数据缺失或过期时回退到配置中的 `fwhm.altitude_m`，并在 diagnostics 标明来源。

启动主 ROS 栈并用 v2 替换原 cable node：

```bash
ros2 launch auv_cable_tracking_v2 auv_stack_v2.launch.py \
  bridge_backend:=protocol_udp \
  enable_bridge:=true \
  enable_localization:=true \
  enable_controller:=true
```

该 overlay 将原栈的 `enable_cable_tracking` 固定为 `false`，再启动独立 v2 节点；定位、控制、仲裁、传感器监督、协议桥和任务注入仍来自现有主栈。PVS/HoloOcean 或实物端按原流程在栈外提供传感器与动力学数据。

使用解耦仿真做 ROS 闭环 smoke：

```bash
ros2 run auv_decision_ros decoupled_cable_sim_node --ros-args \
  -p tracking_config_file:=/home/zmem063/AUV-Master-Project/brain_linux/src/auv_cable_tracking_v2/config/cable_tracking_v2.yaml
```

另一个终端启动 v2 launch，然后检查：

```bash
ros2 topic echo --once /auv/cable/tracking
ros2 topic echo --once /auv/control/setpoint
```

## 参数

算法默认参数见 `config/v2/default.yaml`，ROS 参数见独立包的
`config/cable_tracking_v2.yaml`。横切幅度默认分为 4/8/14/20 m；8 m 和 14 m 分别接近约 7.5 m 垂直间距下的偶极与物理单直导线半高位置，20 m 档为检测中心尚未落在峰值处保留覆盖裕量。首侧到达后先以 `alignment_speed_mps` 调整到横切方向，再开始构造 B(x)；若采样段沿缆范围超过 `maximum_along_track_range_m`，FWHM 被硬判无效。Track 使用 `route_lookahead_m`，到先验终点 `endpoint_tolerance_m` 内发布零速保持。实际部署仍受搜索边界、横向速度、航向变化和最小转弯半径限制。

## 当前证据边界

- 解析 FWHM、状态链和接口兼容已有单元测试；
- Cases A–D 是轻量数学仿真，用于回归和调参方向，不代表 PVS 或海试精度；
- ROS 解耦仿真已经证明理想单直导线下的 Search→Confirm→FWHM→Track 闭环；
- PVS 链路已完成 Search→Confirm/扩幅→Track→终点保持；同时保留 midpoint 场模型固定系数产生约 1 m 深度偏差的负证据，见 `ROS_CLOSED_LOOP_SMOKE.md`；
- 额外 24 m 单档校准复测因沿缆漂移 14.189 m 且缺少左半高交点被质量门拒绝，说明摆幅应逐档增加并受轨迹质量约束；
- 实测 TMR--ArUco 数据已接入 v2 离线质量审计；其自由手持三维轨迹没有任何扫描腿满足双侧 FWHM，故不能作为埋深精度证据；
- 本轮按任务范围终止 HoloOcean 资产安装，不把 HoloOcean 作为交付证据；具有同步高度真值的受控 TMR 双侧横切仍需后续采集。

完整需求映射、验证命令和剩余边界见 [IMPLEMENTATION_REPORT.md](IMPLEMENTATION_REPORT.md)。

面向独立发布的完整实验报告与统一风格图组见
[publication/fwhm_v2_experiment_results/README.md](publication/fwhm_v2_experiment_results/README.md)。
该目录自带全部 PNG/PDF 图和 SHA-256 发布清单，不依赖被 Git 忽略的实验结果目录来显示图像。

场形系数的受控标定、独立留出与保守双模型发布见
[SHAPE_CALIBRATION.md](SHAPE_CALIBRATION.md)。当前只有流程仿真审计，尚无合格真实横切数据。

连续幂律、双导线、多高度及有限线段模型的跨场型可辨识性结果见
[CROSS_SHAPE_IDENTIFIABILITY.md](CROSS_SHAPE_IDENTIFIABILITY.md)。该审计明确区分可改进的已知场形和仍需实测标定的未知场形。
[先场形判别、失败后降级发布的执行决定](MODEL_MISMATCH_PATH_1_TO_3_20260927.md)
把这两条证据链及非仿真配置的误开启拒绝连在一起；横向中心有效不意味着埋深获许可。

浅角度巡检的侧偏／横流组合复核、峰顶假双峰修正与错误侧向符号在线拒止见
[INSPECTION_CURRENT_OFFSET_JOINT_20260926.md](INSPECTION_CURRENT_OFFSET_JOINT_20260926.md)。其八个固定种子角点均到达终点，但近侧四例未达到示例横偏精度门限。

目标缆身份的合成双源反例和运行时声呐线索门分别见
[INSPECTION_MULTISOURCE_IDENTITY_20260927.md](INSPECTION_MULTISOURCE_IDENTITY_20260927.md)、
[INSPECTION_RUNTIME_SONAR_IDENTITY_20260927.md](INSPECTION_RUNTIME_SONAR_IDENTITY_20260927.md)。进入 Track 后持续检查线索丢失与改指邻缆的受控闭环见
[INSPECTION_IDENTITY_CONTINUITY_20260927.md](INSPECTION_IDENTITY_CONTINUITY_20260927.md)。这两组运行时试验只验证注入合成声呐线和概率时的 ROS 许可/拒绝逻辑，物理声呐仍待验收。
现场输入缺口与物理标定文件预检见[声呐输入清点](INSPECTION_FIELD_INPUT_READINESS_20260927.md)。
上行失鲜后桥状态独立送达上位机的[同次 ROS/PVS/Zenoh/PySide6 闭环](INSPECTION_OPERATOR_SAME_RUN_20260927.md)补齐本机软件链，物理 PC104 与跨设备链路仍按[现场冻结矩阵](INSPECTION_V2_FREEZE_AND_FIELD_GATE_20260927.md)另验。
[实物证据来源复核](INSPECTION_FIELD_EVIDENCE_RECONCILIATION_20260927.md)明确历史 PC104 台架报告可以支持什么，以及本机哪些原始包尚未取得。
身份故障后 60 s 的原生 PVS 运动与走廊余量见[零推力续审](INSPECTION_IDENTITY_STOP_DRIFT_20260927.md)：双层零命令仍有惯性/横流位移，故 Hold 须有时限和现场认证的上级处置。

当前路线 Hold 预算配置下，15 s 导航无效后的[感知与执行分层复核](INSPECTION_NAVIGATION_DROPOUT_DISPOSITION_20260927.md)显示感知 Track 可恢复，但安全告警和桥撤权保持锁存；图 38 与逐周期审计给出独立证据。旧配置的自动续航结果不应跨版本外推。

[启动与重启边界复核](INSPECTION_LAUNCH_RESTART_BOUNDARY_20260927.md)让通用 `stack-v2` 默认不注入任务，并拒绝把飞越巡检 YAML 接到通用控制器和桥；专用巡检入口与物理执行许可仍需分开验收。

[被动全栈影子接线](INSPECTION_SHADOW_STACK_INTEGRATION_20260927.md)将定位、传感器监督和可视化接在专用巡检节点旁，活 ROS 图核对六节点及专用桥 `passive_mode=True`。它是接线 smoke，不是任务完成或现场放行证据。
其中[图 39](publication/fwhm_v2_experiment_results/figures/39_shadow_scene_truth_gate.png)对照原演示的模拟场景回退与影子入口的零虚构电缆/真值输出，防止显示层给现场能力造成错误印象。

当前 F 剖面的任务输出合同见[图 62 与独立报告](INSPECTION_TASK_OUTCOME_OPERATOR_20260927.md)：软件有限路线到端、埋深许可和桥执行许可分开显示，任务状态失鲜即未知。安装版 ROS/PID/守卫/UDP/PVS 两例分别验证正常到端与反向流撤权；“软件到端”仍不代表物理目标验收或实艇停住。

[图 63 与独立报告](INSPECTION_TRACK_TRANSIENT_WITHDRAWAL_20260927.md)给有限 Track 增加默认关闭的横向扰动代理变化门：同幅反向阶跃提前 35 s 撤权，但零命令后仍物理越出合成巡检带。高负载导航失鲜的未完成探针与三条完整任务分开保存；此候选未取得现场许可。
