# 巡检 v2 独立 ROS 接收端时序记录器

目标平台的[同次时序验收合同](INSPECTION_SAME_SESSION_TIMING_ENVELOPE_20260928.md)需要最终 Setpoint 在**独立 DDS 接收端**的到达时间与该条消息原始候选时间。目前已有监督门逐周期诊断脚本，却缺这份接收端表。新增只订阅、不发布控制话题的 [`record_v2_inspection_ros_timing.py`](../../../tools/record_v2_inspection_ros_timing.py)，把最终 Setpoint、候选、PID、导航、磁测和操作状态写入同一采集目录。它不更改产品节点、默认 launch 或已有实验结果。

在 ROS Humble 与 `brain_linux/install` 环境中启动；`session_id` 与输出目录应是本次任务独有的值。没有 `--duration-s` 时用 SIGTERM/SIGINT 停止；每次记录器创建新目录，已有目录拒绝覆盖。

```bash
source /opt/ros/humble/setup.bash
source brain_linux/install/setup.bash
python3 tools/record_v2_inspection_ros_timing.py \
  --output-dir /path/to/session/ros_receipt \
  --session-id field_run_001
```

`ros_setpoint.csv` 的 `time_s` 是回调入口采样的本机单调时刻，`header_stamp_s` 是消息原始 ROS 时间戳，`candidate_source_time_s` 是用同一回调的墙钟／单调钟配对换算到本机单调轴的来源时刻，`source_age_at_receipt_s` 是接收墙钟减原始时间戳。正向最终 Setpoint 的 header 在现有监督门中保留候选来源时间；零速 Hold 的 header 是该 Hold 自身的时间。`pid.csv` 的 `time_s,thrust_percent` 可直接作为同次验收的 ROS/PID 接收表。另有 `candidate.csv`、`navigation.csv`、`magnetic.csv`、`operational_status.jsonl` 和 `clock_alignment_trace.csv`。状态 JSONL 只保存原文，不能由记录器擅自推断任务起点、终点或目标身份。

每份 `capture_summary.json` 包含文件 SHA-256、各话题接收/写入/内部队列丢弃计数、可能的非系统历元时间戳计数、时钟采样区间和最大观察到的最终 Setpoint 发布者数。`capture_complete=true` 只说明记录器内部写入结束；`data_reviewable=true` 还要求有最终 Setpoint/PID、同历元时戳和采样时只观察到一个最终话题发布者。它们都**不是**时序门通过，也不证明 DDS 没丢来自发布者的消息。发布者数每秒采样一次，可能漏掉短暂的第二发布者；现场仍须对照 rosbag/ROS graph。来源时戳必须用 Unix 系统历元；ROS `/clock` 仿真时间或未填 header 会被标为不可审阅。

若合成预热阶段只有零速 Hold 的 `/clock` 尚未初始化，整包可能 `data_reviewable=false`，但正向行仍可单独检查。新增 `positive_setpoint_source_reviewable` 与正向历元异常计数，用来明确这一区别；该布尔值只验证**可计算来源年龄**，不裁决来源年龄是否超过任务门限，也不使整包验收通过。[同次诊断](INSPECTION_SAME_SESSION_ROS_RECEIPT_DIAG_20260928.md)给出了一个真实触发此区别的合成例子。

本地时钟配对只解决**同一台主机**内的转换。若监督门、ROS 记录器、UDP 下行观察者与 PC104 不在同一主机或同一受校准时基，必须另外归档跨设备时钟同步/往返标定轨迹，并给出同次测得的最大误差；不能把此文件的微秒级本地采样区间填成清单的 `clock_alignment.measured_max_error_ms`。原始包还须有任务事件、监督门 `gate_ticks.csv`、UDP 下行、PC104 接收及实艇运动记录；时间轴归一与哈希登记后再运行时序审计脚本。场形、目标身份、磁测来源与停车运动许可另按各自合同检查。

隔离 DDS 烟测入口：

```bash
source /opt/ros/humble/setup.bash
source brain_linux/install/setup.bash
ROS_DOMAIN_ID=139 python3 tools/smoke_v2_inspection_ros_timing_recorder.py \
  --output-root /path/to/new/smoke_directory
```

留存的[最终合成烟测摘要](../../results/v2/inspection_ros_timing_recorder_20260928/smoke_trial7/smoke_summary.json)中，两组均写入 12/12 最终 Setpoint 与 PID，内部无丢弃。有效系统历元组 `data_reviewable=true`，并主动注入一条来源已旧的正向消息，接收端最大来源年龄 **1.038 s**；这个年龄仍须由场景预先声明的限值裁决。零时戳组 `data_reviewable=false`，12 条最终 Setpoint 均标为不可信时钟域。两组均为本机合成输入，没有完整任务、桥、UDP、PC104、真实目标或物理时钟校准；`field_release_authorized=false`。先前烟测中的 DDS 发现超时和定时结束造成的缺样保留在同一结果目录作调试记录，不作为通过结果。
