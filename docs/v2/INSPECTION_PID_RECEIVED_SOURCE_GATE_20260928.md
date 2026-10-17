# 巡检 PID 接收端 Setpoint 来源时效门

**结论。** 独立 Setpoint 监督门检查候选来源戳，但原巡检 PID 只检查消息的接收年龄；一条来源已旧、刚送达 PID 的正向 Setpoint 仍可能进入控制律。现为巡检专用控制器增加 `inspection_setpoint_source_timeout_s`：正向指令的原始 header stamp 在 PID 计算前再次按本地 ROS 时钟核对，过期或时钟无效则重置控制器并输出零推力。默认值 `0.0` 保留历史行为；两个候选 launch 仅暴露显式开关，没有默认启用，也没有改动旧任务结果。

前一轮[接收端来源年龄图](../../results/v2/inspection_low_speed_envelope_20260928/received_source_age_timeline.png)显示合成墙钟夹具的最终正向 Setpoint 有 9/63 条到独立观察端时已超过 1.3 s 候选年龄上限。该图来自**另一趟**短探针，只提供修复动机，不能被当作本次 PID 运行中的拒绝计数。修复在物理模式 `use_sim_time=false` 下以同一主机时钟解释 ROS header；实艇仍需证明上游时间戳已经可靠对齐。仿真 `/clock` 排队可能使 ROS 时间落后墙钟，故不能借本门的模拟时钟测试推断实际接收时延。

## 同次 DDS 验证

隔离 ROS domain 232，只启动已安装版巡检 PID、独立发布／订阅节点，不启动算法、桥、PC104 或载体。给有效导航和自治状态后，先发送**来源年龄 1.030 s、接收刚发生**的正向 Setpoint；启用的来源门为 0.35 s，控制器输出 `0.0%` 推力，原因为 `inspection_setpoint_source_stale`。随后发送来源年龄 0.030 s 的同速新指令，控制器输出 `13.53%` 推力。两次执行使用同一个节点和控制器状态，证明拒绝旧来源后并未永久锁死本地 PID。结果在[原始摘要](../../results/v2/inspection_controller_source_age_20260928/wallclock_dds_trial2/summary.json)，其 SHA-256 为 `fc2bc2ef063f6106ed78baba5f5d951ac8c1a2a6efb7e4c1ef8dee9ac1a03e63`；安装版与源码控制器 SHA-256 同为 `0291dc8b3d3ed5102d0ea11376992ac4d8cd15cbc1df0dc79a0ef7c3fd0a0b57`。

源码定向测试 17/17、全包目录直接 pytest 98/98 通过；`colcon build --packages-select auv_common auv_cable_tracking_v2` 成功，两个候选 launch 的 `--show-args` 均显示新参数默认 `0.0`。初始两次 `colcon test` 在收集阶段因安装空间的 `auv_common` 版本缺少后来新增的 `common.imu_provenance_sidecar` 而失败；重建该依赖后，安装版可直接导入 IMV1，标准 `colcon test --packages-select auv_cable_tracking_v2 --pytest-args -q` 得到 **98 项、0 错误、0 失败、0 跳过**。这个修正刷新安装空间，没有改动 IMV1 源码；它也暴露了部署包应核对安装版模块而非只看源码的必要性。

运行方式：

```bash
source /opt/ros/humble/setup.bash
source brain_linux/install/setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=232 python3 \
  tools/smoke_v2_inspection_controller_source.py \
  --output-dir /path/to/new/run
```

候选双进程 launch 可显式传 `controller_setpoint_source_timeout_s:=0.35`；影子栈 launch 可传 `inspection_controller_setpoint_source_timeout_s:=0.35`。这两个值只是本次控制器单点探针的时效门，不是现场冻结参数。实际任务须同时满足[同次时序包 schema 2](INSPECTION_SAME_SESSION_TIMING_ENVELOPE_20260928.md)的最终 Setpoint 到达年龄、PID/UDP/PC104 输出与故障零尾，以及目标身份、导航、流速和完整路线约束。现在 `field_release_authorized=false`；本修复只关闭 PID 层“晚到旧正向指令仍计算正推”的一条路径，**未证明 10 Hz 持续巡检、全链不断流或实艇运动安全**。
