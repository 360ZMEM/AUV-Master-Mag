# 巡检 v2 同次输入—输出—板端时序验收入口

当前可引用的巡检 v2 结论是受限任务下的**方法与合成软件链**，不是实艇可连续执行的冻结版本。十四趟固定供数短探针的严格总门均失败；增加决策节拍、改最新值队列、隔离部分诊断发布也没有给出连续正向 Setpoint 的可靠证据。因此下一项实测应是**目标平台完整任务的同次时序包**，而不是继续用本机几秒钟的负例调参数。此入口为该实测提供可执行的检查合同；目前[schema 2 空清单结果](publication/fwhm_v2_experiment_results/data/inspection_v2_timing_envelope_pending_schema2_20260928.json)是 `data_reviewable=false`、`timing_bounds_passed=false`、`field_release_authorized=false`。

将[空白清单](inspection_timing_envelope_template.json)复制到一次现场运行的原始包目录。当前 schema 2 要求在独立 ROS 接收端记录并对齐正向 Setpoint 的原始候选来源时间，以识别“门内有效、接收时已旧”的消息；schema 1 不再能通过此检查。`session_id` 必须唯一；任务 ID、目标 ID、目标平台 ID、软件快照、原始归档和跨设备对时轨迹均需给出 SHA-256。`time_s` 是经真实对时后归一到**同一单调秒轴**的事件时刻，`clock_alignment.measured_max_error_ms` 是这次运行实测的最大对时误差，不可把 ROS `/clock`、Jetson monotonic、PC104 uptime 原值直接拼列。量纲和转换过程需随原始包保存。`case_kind=nominal` 只有健康窗；`input_loss` 另有故障起点和记录终点，要求保留撤权后持续记录。

各 CSV 至少含以下列；可额外保留原始序号、来源时钟和诊断字段。

| 导出角色 | 最小列 | 来源和含义 |
|---|---|---|
| `gate_ticks` | `time_s,end_s,decision_speed_mps,navigation_receipt_age_s,navigation_source_age_s,magnetic_receipt_age_s,magnetic_source_age_s,covariance_receipt_age_s,candidate_receipt_age_s,candidate_source_age_s,budget_receipt_age_s,budget_latched` | 监督门逐周期仪表；速度是门在本周期决定发送的值，不能冒充外部实际收到值。零速时未知年龄可留空，正向时必须全部有界。 |
| `ros_setpoint` | `time_s,target_speed_mps,candidate_source_time_s` | 独立 DDS 订阅者接收到的最终 Setpoint；`candidate_source_time_s` 是从正向消息保留的 header stamp 转换到同一单调秒轴的来源时间，零命令也须填其自身 header stamp 的转换值。本运行应只有该监督门是最终话题发布者。 |
| `pid` | `time_s,thrust_percent` | 控制器输出的推进命令观察值。 |
| `udp_downlink` | `time_s,frame_number,main_motor_rpm` | 实际下发的 `$CKTH` 原始包解码，`frame_number` 是 0–255 的协议帧号。 |
| `pc104_receipt` | 板端日志：`time_s,frame_number,main_motor_rpm`；固件回声：`time_s,frame_number,pc104_recv_uptime_ms` | `receipt_mode=board_command_log` 时 RPM 为板端收到的下行命令；`checksum_valid_echo` 时回声只有帧号与接收 uptime，命令 RPM 必须从唯一配对的原始 UDP 帧取得。两者均不能从电机反馈猜测。 |
| `mission_events` | `time_s,event` | 同次 ROS 任务事件，首项 `MISSION_START`，标称任务末项 `ROUTE_COMPLETE`，失鲜故障任务末项 `SUSPENDED`；健康窗须落在任务内，故障须发生在暂停前。 |

运行 `tools/trace_v2_setpoint_supervisor_callback.py --output-jsonl /path/supervisor.jsonl --gate-csv /path/gate_ticks.csv --ros-args ...` 可以在隔离的目标节点进程中导出 `gate_ticks`；它保留产品节点的原始监督逻辑与参数。实测还应从同次 rosbag/板端日志导出其余执行表和任务事件，保留原始包。诊断包装器只报告本地门决定，并不证明 DDS、桥、PC104 或推进器实际执行。实际采样前要单独验证仪表开销和时基转换，不把诊断记录器作为产品默认启动项。本机仅做了无传感器、无任务、无桥的约 259 周期记录器启动检查；这不是正向运行验证。

独立 DDS 接收端现在可用[只订阅的 ROS 时序记录器](INSPECTION_ROS_RECEIPT_RECORDER_20260928.md)直接导出 `ros_setpoint.csv` 和 `pid.csv`，同时保存候选、导航、磁测、状态和本机墙钟／单调钟配对轨迹。其隔离合成烟测验证了有效来源戳、已旧正向消息以及零时戳的记录和拒绝；本机转换不能替代跨设备对时，更不能替代任务事件、UDP、PC104 或运动证据。上述约 259 周期仅指监督门诊断包装器的历史空载启动，不包含本次接收端烟测。

[同次合成短探针](INSPECTION_SAME_SESSION_ROS_RECEIPT_DIAG_20260928.md)已经把监督门 `gate_ticks.csv` 与独立 DDS 接收表联接，在两趟默认 PID 与一趟开启 PID 来源门的运行中均观察到健康输出不连续，并直接记录接收时已旧的正向消息。三趟固定供数最大缺口也都略/明显超预设门，不能据此作算法独立最坏时延归因；一趟断流守卫窗还有晚到正向 Setpoint。该合成探针只验证时序采证和失效机制，不替代目标平台整包。

每条正向 ROS Setpoint 的接收时间减 `candidate_source_time_s` 必须落在声明的 `max_candidate_age_s` 内；检查器对接收时间与来源时间各计一次对时误差。来源时间必须由**该条消息** header 转换，不能用接收时间、上一个 `/clock` 值或附近导航时间代填。低速诊断探针曾观测到正向消息到达时来源年龄最高 1.757 s，说明仅检查监督门当时的年龄和输出间隔不足以发现晚到消息；该探针仍是合成夹具，不是现场数据。

`limits` 必须在看结果前按目标平台资源、物理速度和边界余量声明。监督门、ROS、PID、UDP、PC104 的允许最大间隔分别独立配置。尤其现有真实 PC104 零命令台架的下行接收任务约 3.33 Hz，不能强行要求其每条 10 Hz UDP 帧都到板端；用 `min_pc104_receipt_fraction` 和 `max_pc104_gap_s` 表达现场实际可接受的板端覆盖下界与最大空档。每个已观测板端接收事件仍须在有界时窗内与**唯一**的 `$CKTH` 帧号/命令值配对。回声模式须先按“帧号＋接收 uptime”去重，再把 PC104 uptime 对齐到同一时间轴；`time_s` 应代表板端**接收**事件，不能直接填 Jetson 看见回声的上行到达时间。`max_local_zero_delay_s` 约束门、ROS、PID、UDP 的撤权，`max_fault_zero_delay_s` 约束板端收到零命令；零尾段还必须有至少两样本、直到记录终点保持零且无过大采样间隙。板端采样较慢不能借一个未观测末尾零样本被称为持续停车。

运行：

```bash
python3 tools/audit_v2_inspection_timing_envelope.py \
  --manifest /path/to/field/timing_manifest.json \
  --output-json /path/to/field/timing_audit.json
```

返回码 `0` 只表示**导出表内部**通过所声明时序界，`2` 表示结构缺口或红旗；两种情况下 `field_release_authorized` 永远为 `false`。脚本按次序比较监督门发送决定与 DDS 观察结果，并核对数量、值和到达时差；消息没有天然统一因果 ID，因此发生多发布者、跨进程重排或重传时必须人工回看原始 rosbag，不能靠近邻时间匹配冒充精确因果。它不能认证原始包来自真实磁测/导航，也不能从板端收到零命令推断推进器瞬时停转或实艇保持位置。

本工具与既有[目标包／执行包预检](INSPECTION_FIELD_PACKAGE_PREFLIGHT_20260927.md)及[冻结矩阵](INSPECTION_V2_FREEZE_AND_FIELD_GATE_20260927.md)并行：目标身份、真实线位、IMU 原生采样、流速/滑行/边界和上级处置仍需各自现场证据；同次时序包不替代这些内容。仅当目标平台完整任务、预期峰值并发负载、健康与故障分段、实际 PC104 收帧和实艇运动同时归档，且现场批准的性能门逐项通过，才能讨论 F 剖面的物理冻结。宽先验多横切和 FWHM 埋深另有身份及几何标定门，不能被本时序结果自动解锁。

验证：构造数据单测覆盖完整健康/故障包、接收端已旧而门内年龄合格的正向消息、拒绝旧 schema、旧导航来源戳、健康零速、板端命令值篡改、文件哈希篡改、任务终态错配、三分之一下行帧的板端采样、校验回声模式和标称包。构造正例只检验检查器本身；未产生现场正例、真实成功率或持续巡检声明。
