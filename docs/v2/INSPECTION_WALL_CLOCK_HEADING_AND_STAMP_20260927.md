# 墙钟巡检的航向质量来源与 IMU 时间戳重放

先前图 40 用合成 `/clock` 验证停止 IMU 后的降级，但现场 ROS 使用墙钟，且非仿真巡检会额外检查导航协方差。本次保持六节点被动影子栈，改用墙钟：合成静止 IMU、DVL、深度、磁测持续输入，在第 10 秒让 **IMU 消息继续到达而采集时间戳冻结**，其余消息仍带新时间戳。这是对“接收时间新鲜却物理采集时间过期”的独立反例。

墙钟诊断先暴露航向协方差口径错配：定位节点按配置直接采用 IMU 四元数作为输出姿态，但 ES-EKF 的 15 维预测协方差并未因这一直接替换而得到姿态观测更新。静止合成输入下，P15 的航向方差约 5 s 时已达 `0.06 rad²`，对应约 14°，超过巡检配置的 2° 门；旧口径即使导航位置与时间戳正常也只能 Hold。独立 v2 节点现提供显式 opt-in 的 `imu_orientation_covariance` 来源：仅当 IMU 报告非零、半正定的姿态协方差，接收时间新鲜，采集时间与滤波导航相差不超过 0.15 s，且两者姿态相差不超过 2°，才用其最大特征值给航向一倍标准差；否则按 180° 处理并拒绝运动。位置和速度仍由 P15 给出，未修改旧 ES-EKF 或历史实验。测试夹具声明 `0.0001 rad²`，仅用于证明接口，不是物理标定。

本次非仿真配置仍带 `synthetic_test_fixture` 侧向线索，因此两组在故障前都按 `flythrough_side_hint_simulation_only` 保持零目标速度，**没有搜索、航行或任务完成**。这同时验证了不能把仿真目标线索带到墙钟任务。只比较时间锚失效后软件是否继续发布旧导航：

| 同版墙钟探针 | 第 10 秒后 IMU 时间戳 | 1 秒宽限期后的导航消息 | 巡检能力 | 跟踪状态 |
|---|---|---:|---|---|
| 时效门关闭，域 229 | 冻结，消息仍持续到达 | 59 条 | 保持可用 | Hold |
| 时效门开启，域 228 | 同上 | 0 条 | 撤销 | `sensor_degraded` |

![图41：墙钟下 IMU 时间戳冻结时，导航续发与巡检能力的正反对照](publication/fwhm_v2_experiment_results/figures/41_wall_clock_imu_stamp_replay.png)

[逐时 CSV、摘要与哈希](publication/fwhm_v2_experiment_results/data/41_wall_clock_imu_stamp_replay_manifest.json)、[探针源码](../../../tools/probe_v2_shadow_wall_stamp_replay.py)和[绘图源码](../../../tools/plot_v2_shadow_wall_stamp_replay.py)可复核。复现时加载 ROS 与 `brain_linux/install`，在隔离 `ROS_DOMAIN_ID` 下运行图 40 的影子 launch 命令，改为 `use_sim_time:=false inspection_navigation_heading_uncertainty_source:=imu_orientation_covariance`；对照组另加 `inspection_navigation_imu_max_age_s:=0.0`。第二终端在同域运行 `python3 tools/probe_v2_shadow_wall_stamp_replay.py --output-dir /tmp/wall_stamp_probe --expect-safe`，对照组不加 `--expect-safe`。

实际协议桥当前填入 PC104 姿态四元数，但**未填 IMU 姿态协方差**，因此不能直接启用这一可选航向来源来宣称实艇巡检可运行。现场需由独立姿态真值或受控标定给出保守协方差，桥负责连同原始采集时间戳传送；定位必须显式采用该姿态源，并在当前 v2 输入链验收时间与姿态匹配。若没有该证据，巡检保持 Hold。图 41 是合成输入、被动桥的失效处置证据，未连接真实 PC104，也不证明物理停车或目标身份。

后续[协议桥时间来源对照](INSPECTION_PROTOCOL_IMU_TIME_PROVENANCE_20260927.md)发现，现有 PC104 有效时间标记描述的是报文组包，而不是 IMU 原生采样；无标记时旧桥还可能回退到 ROS 接收时刻。图 42 的可选桥门已对组包时间的重复、倒退、缺失与过期给出本地 UDP→ROS 正反验证，但不能识别“旧姿态被持续重新组包”。因此上段要求的“原始采集时间戳”在当前物理协议中尚未具备，不能将板端组包时间替代它。
