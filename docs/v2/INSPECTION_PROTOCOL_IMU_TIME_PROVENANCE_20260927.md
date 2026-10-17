# 协议桥 IMU 时间来源与被动门控对照

图 41 证明“上游消息时间戳冻结”可以在定位入口被拒绝，但它的合成 IMU 已有明确的采集时间戳。当前 PC104 `$AUV` 上行中的 `pc104_uptime_ms` 表示**报文组包时间**；缺少有效时间标记时，通用桥会退回 ROS 接收时刻。若把这两种时间一概当作 IMU 采集时间，持续到达的旧姿态仍可能显得新鲜。这是本次针对现场接入的时间来源缺口。

协议桥现提供独立、默认关闭的 `bridge.protocol_udp.require_packet_time_for_imu`。启用时必须同时给出正数 `max_packet_time_age_s`；仅当板端时间标记存在、相对上次严格递增、映射到 ROS 墙钟后未超过年龄门，桥才发布 ROS IMU。重复、无标记、倒退和“略有递增但已过期”的报文不刷新 IMU；该门只作用于 IMU，其他遥测继续接收。发生倒退时也不重置板端时间映射基准，避免下一帧被错误重新解释为当前时间。测试夹具使用 `0.75 s`，它不是实物阈值。

两组在隔离 ROS 域运行**同一版**本地合成 `$AUV` UDP 探针与被动 `protocol_udp_bridge_node`，仅门控配置不同。六帧按顺序进入桥，ROS DVL 均为 `6/6`，用于确认并非 UDP 丢帧。观察如下：

| 注入报文时间 | 关闭门控：ROS IMU | 启用门控：ROS IMU |
|---|---:|---:|
| 有效初始 `1000 ms` | 1 | 1 |
| 新鲜递增 `1700 ms` | 1 | 1 |
| 重复 `1700 ms` | 1 | 0 |
| 无有效标记 | 1 | 0 |
| 倒退 `900 ms` | 1 | 0 |
| 递增到 `1710 ms`，但到达时已旧约 2.6 s | 1 | 0 |
| **合计** | **6/6** | **2/6** |

![图42：六帧本地 UDP 输入下，协议桥 IMU 组包时间门控的同版正反对照](publication/fwhm_v2_experiment_results/figures/42_protocol_imu_packet_time_gate.png)

[对照原始 JSON](publication/fwhm_v2_experiment_results/data/42_control_201.json)、[门控原始 JSON](publication/fwhm_v2_experiment_results/data/42_gated_202.json)与[源码、配置、图像 SHA-256 清单](publication/fwhm_v2_experiment_results/data/42_protocol_imu_packet_time_gate_manifest.json)可核对每帧结果。绘图脚本为 [`tools/plot_v2_protocol_imu_packet_time.py`](../../../tools/plot_v2_protocol_imu_packet_time.py)，探针为 [`tools/probe_v2_protocol_imu_packet_time.py`](../../../tools/probe_v2_protocol_imu_packet_time.py)。两组使用 ROS 域 201/202、本地端口 53665 和被动桥；`physical_pc104_tested=false`、`field_release_authorized=false`。门控组另检查了倒退之后未重新对齐时间基准。

可在项目根目录、完成 `auv_bridge` 构建后，以隔离 ROS 域复现门控组：

```bash
source /opt/ros/humble/setup.bash
source brain_linux/install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=202
ros2 run auv_bridge protocol_udp_bridge_node --ros-args \
  -p params_file:=brain_linux/src/auv_cable_tracking_v2/config/inspection_bridge_packet_time_sim.yaml \
  -p passive_mode:=true
```

另一个同域终端运行：

```bash
source /opt/ros/humble/setup.bash
source brain_linux/install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=202
python3 tools/probe_v2_protocol_imu_packet_time.py \
  --output-json /tmp/inspection_packet_time_gated.json
```

对照组换用 `inspection_bridge_packet_time_legacy_sim.yaml`、另一隔离域并给探针加 `--expect-control`。这些 YAML 只绑定回环地址，不可作为实艇配置。

**证据边界：**板端组包时间可信，也不证明 IMU 自身在同一时刻完成采样。若 PC104 用冻结的姿态值不断组新包，本门控会把这些报文当作新帧；若固件没有时间标记，严格门会一直拒绝 IMU。现场应优先由 IMU 驱动提供传感器原生采样时间或单调样本计数，记录采样到组包、组包到 ROS 的时延分布，再配置经过实测的年龄门；同时提供经独立标定的姿态协方差，并复核它与同一 IMU 样本绑定。做不到时应保持巡检 Hold。真实 PC104、传感器原生时间、姿态协方差、目标身份及整艇停止包络目前均未在本试验中验证，不能据图 42 放行现场自主运动。

后续核对找到一条更具体的来源路径：[3DM-GX3-25 `0xCF` 原始应答](INSPECTION_GX3_TIMER_RECOVERY_PLAN_20260927.md)本身含 32 位传感器 Timer 和校验和；当前 PC104 代码只取姿态/角速度，没有验证完整帧或将 Timer 上传。因此缺口主要在固件接收与版本化上行，不必先假设传感器硬件没有原生时间锚。此发现不改变图 42 的证据边界；原始 Timer 与真实采样周期的关系还要实测确认。
