# 巡检 v2 现场目标包与执行包预检

当前软件候选尚无可复核的真实目标包和本版本 PC104／实艇执行包，因此**现场巡检许可仍为否**。本入口只检查现场数据是否已按角色归档、能否打开并追溯到原始包；它不自动证明数据来自实物，也不计算错目标率、停止距离或签发实艇许可。

复制 [空白清单](inspection_field_packages_template.json) 到现场数据目录，填写任务、目标、站点、统一坐标系、软件快照、目标身份来源和跨设备对时信息。所有 `x_m/y_m`、`lateral_y_m` 与独立真值 `true_target_y_m` 须变换到声明的同一坐标系；原始外参与变换过程留在原始包供人工复核。每个 `path` 可相对该清单所在目录或为绝对路径，并填对应文件的 SHA-256。保留原始采集包，另从原始包导出表格，不要只保存筛选后的统计结果。`reviewed_static_prior` 的 `identity_source` 必须是包含同一 `mission_id`、`target_id` 且 `reviewed_for_mission=true` 的审查清单；若使用 `physical_sonar`，保存声呐原始包及目标对应关系，预检只验证身份文件存在，仍须人工确认其来源和内容。

目标包至少包含真目标、邻缆和线索丢失三类案例。每例有磁测、导航不确定度、身份线索、独立真值和**逐周期任务决策**五种 CSV，以及独立于磁测/FWHM/PVS 的真值取得方法。`target_decision` 必须导出横向中心及其有效位、目标确认、任务完成、自治许可和埋深发布位，避免只有输入而没有真实输出的包通过预检；无横向结果时 `lateral_y_m` 可留空，但 `lateral_valid` 必须为假。布尔列用 `0/1` 或 `false/true`。执行包至少包含标称与上行失联两类案例；每例保留 ROS 指令、原始 UDP 下行、PC104 接收证据、实艇运动、边界净距和故障事件。ROS setpoint 没有与协议帧相同的天然序号，不能拿桥状态 `command_sequence` 填充 `$CKTH` 的 `frame_number`（0–255，可能回绕）。

`receipt_mode=board_command_log` 时，`pc104_receipt` 从板端原始接收日志导出 `time_s,frame_number,main_motor_rpm`，其中 RPM 是收到的**下行命令值**。`receipt_mode=checksum_valid_echo` 时，它从 `$AUV` 上行导出去重后的 `time_s,frame_number,pc104_recv_uptime_ms`，`frame_number` 必须是 `pc104_downlink_echo_frame`，不得用上行包自身的帧号；`pc104_recv_uptime_ms` 来自实际板端下行接收时间。当前 VxWorks 接收路径只在 `$CKTH` 校验和通过后记录该回声，因此在源码与固件版本一致、回声为真且时间窗内能唯一配对的前提下，可用被配对的原始 UDP 下行帧读取命令 RPM，证明该帧已被 PC104 接收。回声自身**不含命令 RPM**，不能从上行电机反馈栏冒充下行命令；还须排除固件诊断用的人工回声注入，且接收不等于执行器已实施零推力。

所有表格的 `time_s` 须为同一对时基准上的秒数；`clock_alignment` 记录实际对时方法、最大测量误差与原始对时轨迹。`max_receipt_lag_s` 应来自本次现场通信时序上界审查，不能为促成配对任意放宽。分析器要求每条板端收帧在该时间窗内与**唯一**的 UDP 帧号配对，并核对命令 RPM；出现帧号回绕歧义、无配对或值不一致会分别报告。最小列名由[预检脚本](../../../tools/preflight_v2_inspection_field_packages.py)定义；额外列可以保留。原始包必须为不同案例分别归档，不能用一个文件冒充多个案例。

运行：

```bash
python3 tools/preflight_v2_inspection_field_packages.py \
  --manifest /path/to/field/inspection_packages.json \
  --output-json /path/to/field/preflight.json
```

返回码 `0` 仅表示清单结构、文件哈希、所需列、有限值、时间顺序、案例覆盖与所声明对时误差下的跨流时间交集通过；返回码 `2` 表示缺项，`blockers` 列出逐项原因。仓库空白清单的[当前预检报告](publication/fwhm_v2_experiment_results/data/inspection_v2_field_package_preflight_20260927.json)预期为 `preflight_passed=false`、`field_release_authorized=false`。

预检通过后运行描述性指标提取：

```bash
python3 tools/analyze_v2_inspection_field_packages.py \
  --manifest /path/to/field/inspection_packages.json \
  --output-json /path/to/field/observed_metrics.json
```

该脚本逐例输出真目标横向误差、邻缆是否被报完成、线索丢失后的自治许可样本、是否发埋深、失联后 ROS／UDP／PC104 **零命令尾段**的起点、样本数、覆盖时长和最大采样间隔，以及之后 60 s 窗口内实艇的实际观测时长、位移、路径长度和边界最小净距。单个末尾零样本不算零命令尾段。`analysis_ready` 只表示有足够记录计算指标；`observed_protocol_red_flags_clear` 单列邻缆误确认或误完成、持续线索丢失后仍许可／确认／完成、持续失联到记录结束仍有非零指令、横向优先验收包中误发埋深等观测红旗。线索中断或失联后记录不足时 `analysis_ready=false`。清除红旗仍不等于实艇停住，也不从单例推算成功率；采样间隙、对时误差、物理来源和独立任务留出须审查。即使 `analysis_ready=true`，`field_release_authorized` 始终为 `false`，真实准入还需对照现场批准的定量阈值人工裁定。`cue_loss` 与 `uplink_loss` 案例按**持续故障直至记录结束**导出；若中途恢复，另开案例而不伪装持续故障。

预检通过后仍须人工和定量复核：确认采集来源与独立真值、目标与邻缆按完整任务分训练/留出、线索中断时撤权、每个失联事件从故障到 ROS／UDP／PC104 的实际零指令和实艇停止时距、边界剩余量与上级处置时延。通过现场测得的余量才能替换 PVS 的候选阈值；埋深还需独立的[物理垂距标定包](SHAPE_CALIBRATION.md)。[冻结矩阵](INSPECTION_V2_FREEZE_AND_FIELD_GATE_20260927.md)始终是最终解释口径。

上述两包预检没有检查 IMU **原生采样时间**或物理姿态协方差。即使两包 `preflight_passed=true`，仍须单独提交[IMU 原生采样与姿态协方差证据包](INSPECTION_IMU_ACQUISITION_PREFLIGHT_20260927.md)，并在同次 ROS 链审查其与定位姿态的匹配。PC104 组包时间递增不满足这项要求；当前模板结果仍为 `navigation_evidence_reviewable=false`。

目标平台完整任务的计算与输出连续性另用[同次时序包及可执行审计](INSPECTION_SAME_SESSION_TIMING_ENVELOPE_20260928.md)核对监督门逐周期来源年龄、ROS/PID/UDP/PC104 输出节拍与故障零尾段。它引用同次原始包和独立接收日志；现有两包预检不自动包含这项时序验收。仓库空白时序清单仍返回未就绪、现场许可 `false`。
