# 巡检 v2 全栈影子接线：专用执行链与定位显示共存

通用 `stack-v2` 已禁止直接接收飞越巡检配置，但此前只有单独的巡检节点启动器，定位、传感器监督和显示需人工另行拼装。新增 [`inspection_shadow_stack.launch.py`](../../../brain_linux/src/auv_cable_tracking_v2/launch/inspection_shadow_stack.launch.py) 作为**默认被动的完整 ROS 接线入口**：它引入基础栈的定位、传感器监督和 Zenoh 可视化，明确关闭基础栈的通用桥、通用控制器、决策节点、旧电缆跟踪节点及任务自动注入，再启动 v2 巡检算法、巡检控制器和巡检专用桥。该入口强制要求明确传入 `v2.mission_geometry: flythrough` 的配置文件；桥参数固定 `passive_mode=True`，不提供通过本 launch 改为主动下发的开关。

在 ROS Humble、隔离 `ROS_DOMAIN_ID=190`、`ROS_LOCALHOST_ONLY=1`、合成飞越 YAML 和本地临时数据目录下，运行时 [ROS 图审计](../../results/v2/inspection_shadow_stack_20260927/graph_audit.json) 得到以下六个节点，`checks_passed=true`：

| 基础栈保留 | 巡检专用 |
|---|---|
| `auv_localization_node` | `auv_cable_tracking_v2_node` |
| `auv_sensor_supervisor_node` | `auv_inspection_controller_node` |
| `zenoh_viz_bridge_node` | `auv_inspection_bridge_node` |

同次参数查询返回 `Boolean value is: True`，即专用桥处于被动模式；实时图中没有通用桥/控制器、通用决策或自动任务注入节点。使用普通 `cable_tracking_v2.yaml` 对该入口做反向启动，得到预期的“需要 flythrough 配置”错误，且在创建节点前退出。最初的默认启动曾因仓内 `auv_viz_bridge` 源码虽在、安装空间未注册而失败；补做 `colcon build --packages-select auv_viz_bridge` 后包可发现，默认六节点运行。v2 包已将定位及可视化声明为执行依赖，避免在完整构建时漏掉它们。

后续核对发现合成巡检节点使用 `/clock`，基础定位、监督与显示原先仍用墙钟；空闲启动掩盖了此错配。影子入口现将同一 `use_sim_time` 参数传给三个基础节点，活图审计分别查询到 `Boolean value is: True`。专用桥仍用墙钟处理通信新鲜度，不把仿真任务时间当作真实 UDP watchdog。该检查证明时间参数一致；本次 `/clock` 仅在十秒显示审计期间推进，尚未运行基础定位输入到任务完成的长程同次任务。

## 场景来源的显示门

集成检查又发现，原数字孪生显示桥即使 `mock_mode=false`，只要 Zenoh 场景失鲜就会自动绘制模拟海床、电缆及“真值”艇位。这适合演示，但现场审查可能把模拟电缆误读为目标观测。保留原显示入口的演示行为；影子巡检入口固定传入 `viz_allow_synthetic_scene_fallback=false` 和 `viz_publish_sim_ground_truth=false`。在该模式中，缺少新鲜 Zenoh 真值时清除场景标记与点云；只收到部分新鲜场景层时仅绘制收到的层，不补虚拟电缆、海床或视域。已收到的场景层超过显示超时后清除旧标记；旧巡检状态超过显示超时则显示 `STATUS_STALE`，不继续显示旧 `TRACK`。任务状态本身仍可从巡检 ROS 消息显示。

![图39：数字孪生回退与巡检影子显示门的 DDS 消息对照](publication/fwhm_v2_experiment_results/figures/39_shadow_scene_truth_gate.png)

无 Zenoh 场景输入时，原独立显示节点 7 s 观察收到模拟场景 32 条、电缆 ADD 31 条、“真值”艇位 ADD 31 条；当前六节点影子入口 10 s 观察三者均为零，任务状态仍收到 100 条。两个时间窗不同，图中给原始消息条数，不用于比较吞吐率或算法精度。[两次 DDS 原始计数与参数](publication/fwhm_v2_experiment_results/data/inspection_v2_shadow_scene_fallback_audit_20260927.json)、[原入口反例](publication/fwhm_v2_experiment_results/data/inspection_v2_legacy_scene_counterexample_audit_20260927.json)及[图像/源码哈希](publication/fwhm_v2_experiment_results/data/39_shadow_scene_truth_gate_manifest.json)可复核。随后增加旧 `TRACK` 超时标签与仿真真值转发关闭的回归，显示包和巡检包合计 59 项通过。六节点活图、被动桥参数和空闲启动日志用同一新版入口重新采集；合成配置需显式 `use_sim_time:=true`，非仿真时钟下的合成声呐标定会被巡检节点正确拒绝。此处没有向系统注入巡检任务，不能据此断言完整任务在基础定位链下完成。

复现影子启动与活图审计：

```bash
source /opt/ros/humble/setup.bash
cd brain_linux
colcon build --packages-select auv_viz_bridge auv_cable_tracking_v2
cd ..
source brain_linux/install/setup.bash
mkdir -p /tmp/inspection_shadow_data
AUV_DATA_ROOT=/tmp/inspection_shadow_data ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=190 \
  ros2 launch auv_cable_tracking_v2 inspection_shadow_stack.launch.py \
  config_file:=/absolute/path/to/reviewed_flythrough.yaml use_sim_time:=true
```

在另一个已加载同一 ROS 环境的终端、启动进程仍存活时运行：

```bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=190 python3 tools/audit_v2_inspection_shadow_graph.py
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=190 python3 tools/audit_v2_shadow_scene_fallback.py --publish-sim-clock
```

原始传感器链的后续复核见[同版探针正反对照](INSPECTION_SHADOW_RAW_SENSOR_FLOW_20260927.md)。此影子 launch 现默认向定位传入 `inspection_navigation_imu_max_age_s:=0.75`，同时检查 IMU 的接收时间与采集时间；停 IMU 后不再反复发布旧滤波导航。通用基础栈默认 `navigation_imu_max_age_s:=0.0` 保留原有行为，故对照可在同一影子 launch 中显式设 `inspection_navigation_imu_max_age_s:=0.0` 复现。该 0.75 s 值仅是合成 smoke 的候选，实测传感器时延及现场门限尚未冻结。

非仿真墙钟另可显式传 `inspection_navigation_heading_uncertainty_source:=imu_orientation_covariance`，令 v2 只从与滤波导航姿态和采集时间匹配的 IMU 协方差取得航向质量；默认仍为 `ekf`。它需要真实传感器经独立标定并由桥转发非零姿态协方差。当前协议桥并未提供，因而真实艇上不能只加这一启动参数就宣称导航可用。[墙钟时间戳重放对照](INSPECTION_WALL_CLOCK_HEADING_AND_STAMP_20260927.md)使用显式标注的合成协方差，只验证门控软件语义。

协议桥另有独立、默认关闭的 PC104 组包时间门，见[图 42 与本地 UDP→ROS 对照](INSPECTION_PROTOCOL_IMU_TIME_PROVENANCE_20260927.md)。它要求有效、递增且未过期的组包时间才刷新 ROS IMU；组包时间并非 IMU 原生采集时间，也不能代替上段所需的物理协方差。影子入口的合成原始 IMU 测试与该桥门试验分别验证两层，尚未构成真实 PC104 到任务完成的一次现场链路。

合成 smoke 的 `config_file` 应指向仓内 `cable_tracking_v2_flythrough_sonar_identity_corridor_budget_sim.yaml`，并设置 `use_sim_time:=true`；其声呐和 AMD 上行均是脚本/配置夹具，不是现场输入。图检查只抓取**活 ROS 图快照**与桥参数，场景检查只断言缺 Zenoh 输入时不造假，不证明任务完成、实物 PC104 收帧或实艇运动。当前 70 s 启动日志的首次 `Traceback` 之前，六个进程已启动且没有运行期 `[ERROR]`；外部 `timeout -s INT 70` 调用返回 124，之后的 `KeyboardInterrupt` 属主动结束过程。原清单曾误把终止栈记为运行期错误，现按[可重算的启动清单](publication/fwhm_v2_experiment_results/data/inspection_v2_shadow_stack_manifest_20260927.json)和[原始日志](../../results/v2/inspection_shadow_stack_20260927/launch.log)分开记录。可运行 `python3 tools/audit_v2_inspection_shadow_launch_log.py --observed-timeout-exit-code 124` 复核；124 是当次外部调用的观测，不能仅由日志推断。此结果不用于声称长期稳定。当前入口持续保持被动，现场主动执行仍须另行完成目标身份、传感器标定、PC104/运动包络与外部复核后的启动许可，见[现场冻结矩阵](INSPECTION_V2_FREEZE_AND_FIELD_GATE_20260927.md)。
