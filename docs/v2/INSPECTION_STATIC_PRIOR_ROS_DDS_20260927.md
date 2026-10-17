# 非仿真静态目标清单的 ROS DDS 准入复核

已知目标、无运行时声呐的单次浅角穿越现在使用 [schema v2 审查清单](STATIC_TARGET_PRIOR_MISSION_ENVELOPE_20260927.md)，同时绑定源文件、`cable_tracking` 任务配置、任务与目标 ID、深度和速度。此前的证据主要是直接调用节点方法的包测试。本补项在**墙钟 `use_sim_time=false`**、隔离 ROS 域中实际建立安装空间的 v2 节点和 DDS 任务话题，核对消息准入与启动拒绝。

| 同一合成清单下的输入 | ROS 观察 | 任务接收 |
| --- | --- | --- |
| 正确任务／目标 ID，速度从配置的 0.3 改为 0.2 m/s | `/auv/cable/inspection_status` 收到 `STATIC_TARGET_PRIOR_COMMAND_MISMATCH` | 否 |
| 正确 ID，深度 10 m、速度 0.3 m/s 与已审查配置一致 | 节点经 DDS 收到任务 | 是 |
| 保留原清单，另把任务搜索边界的一个坐标改变 1 m | 新节点构造返回 `mission config hash mismatch` | 未启动 |

探针运行时断言 `CableTrackingV2Node` 与静态先验模块均来自 `brain_linux/install`，而非只测试源码目录；逐文件哈希固定在[独立摘要](publication/fwhm_v2_experiment_results/data/static_target_prior_ros_dds_admission_20260927.json)。[原始合成清单、配置及摘要](../../../AUV-Master-Mag/results/v2/static_target_prior_ros_admission_20260927/installed_verified/summary.json)留在独立结果目录，旧实验结果未覆盖。复现需先加载 ROS Humble 与本仓安装空间，并设置有效的隔离域：

```bash
source /opt/ros/humble/setup.bash
source brain_linux/install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=225
python3 tools/probe_v2_static_prior_ros_admission.py \
  --output-dir AUV-Master-Mag/results/v2/static_target_prior_ros_admission_20260927/installed_verified
```

这是**同一 Python 进程中的真实 ROS 节点和 DDS 投递**，不是完整 launch、ROS→PID→守卫→UDP→PC104→实艇的同次航行。目标测绘文件由脚本生成且明确标为 `fixture_only`；没有导航、磁测、声呐、控制器或车辆运动。匹配任务“被接收”只证明准入接线，不等于启动安全航行，也不证明测绘线确属海缆。现场仍需经审查的真实目标包、导航原生采样、走廊和执行包络；`field_release_authorized=false`。
