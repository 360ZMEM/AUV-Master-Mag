# 巡检 v2 启动与重启边界复核

限时 Hold 预算的 `RECOVERY_REQUIRED` 在当前巡检进程内锁存，任务话题重发不能清除。但整栈重启会重新建立进程内状态，因此启动链必须避免自行重投旧任务。复核发现：基础 ROS 栈的任务自动注入默认关闭，旧 `auv_stack_v2.launch.py` 却把默认值改成开启；该集成入口还沿用通用控制器和桥，不能承担飞越巡检的独立预算撤权出口。这里不改变旧实验轨迹或 FWHM 算法，而是收紧启动组合。

| 入口 | 当前可复核行为 | 用途边界 |
|---|---|---|
| `auv_stack_v2.launch.py` 默认启动 | `enable_cable_mission_autostart=false`；需外部显式发任务 | 普通 v2 栈；不自动重投巡检任务 |
| 通用栈传入 `v2.mission_geometry: flythrough` | 在创建任何节点前拒绝，说明应使用专用巡检控制器与桥 | 阻止“算法是巡检版、执行出口仍为通用版”的错配 |
| `flythrough_nodes.launch.py` | 保留专用节点；桥默认不启动，若选择桥则为被动模式 | 受控接线/影子验证入口；并非已批准的实艇主动巡检启动器 |
| 答辩仿真脚本 | `run_defense_perception_1366.sh`、`run_defense_demo.sh`、`run_cable_closedloop_distorted.sh` 显式传 `enable_cable_mission_autostart:=true` | 已声明的仿真任务注入，不改变这些演示的预期流程 |

以 ROS Humble 加当前安装空间复核：`colcon build --packages-select auv_cable_tracking_v2` 成功，安装空间与源码的集成 launch SHA-256 同为 `c3372c114866f12ba8973ff75392f3cf72b363b6cfdce12901c9dbe443cddfc1`；`ros2 launch ... --show-args` 显示默认 `false`。直接以隔离 ROS 域 183 传入合成飞越配置，同时显式关闭桥、控制器、定位和任务注入，launch 以状态 1 报出预期的配置拒绝，未留下任何 AUV 节点进程。源码函数用通用配置返回允许、用飞越配置返回拒绝；v2 ROS 包测试 `47 passed`，演示脚本 `bash -n` 通过。共享文件夹不支持符号链接，首次 `colcon --symlink-install` 因环境文件系统约束失败，随后使用正常复制式构建通过；这与算法行为无关。

可重复的最小检查：

```bash
source /opt/ros/humble/setup.bash
source brain_linux/install/setup.bash
ros2 launch auv_cable_tracking_v2 auv_stack_v2.launch.py --show-args
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=183 ros2 launch auv_cable_tracking_v2 auv_stack_v2.launch.py \
  config_file:=/absolute/path/to/cable_tracking_v2_flythrough_sonar_identity_corridor_budget_sim.yaml \
  enable_bridge:=false enable_controller:=false enable_localization:=false \
  enable_sensor_supervisor:=false enable_viz_bridge:=false \
  enable_cable_mission_autostart:=false
```

第二条命令的**预期结果是启动前拒绝**，不可当作“巡检任务失败”。复核用路径须替换成自己工作区的实际文件；也可直接检查[启动文件](../../../brain_linux/src/auv_cable_tracking_v2/launch/auv_stack_v2.launch.py)及[演示显式参数](../../../scripts/run_defense_perception_1366.sh)。

这两个软件门不能把进程内告警变成跨重启的持久安全锁。真正重新起航仍须外部确认故障原因、车辆当前运动与边界、目标线索和新任务输入，再启动专用巡检执行链；通用整栈、重复任务消息或简单进程重启都不算批准。当前尚无同版物理 PC104、实艇停机包络和目标身份验收，现场许可继续为 `false`。最快现场收口仍按[目标包与执行包预检](INSPECTION_FIELD_PACKAGE_PREFLIGHT_20260927.md)及[冻结矩阵](INSPECTION_V2_FREEZE_AND_FIELD_GATE_20260927.md)逐项取证。
