# 低速巡检与来源时效：独立 ROS 短探针负结果

**结论。** 将合成任务速度由 0.3 降到 0.2 m/s、候选时效放宽至 1.3 s，并未使独立进程链通过既定的连续输出验收。四条诊断记录均为 `passed=false`；其中一条因非仿真身份门拒绝而未进入任务。唯一采用“仿真身份＋墙钟推进 `/clock`”且健康供数达标的运行，健康段最终 Setpoint 为 24/42 条正向，最大接收空档 0.325 s；故障守卫窗 16/16 条 Setpoint、9/9 条 PID 为零。这是**合成软件探针**，不含 PVS、桥、PC104、运动或完整路线，不能作为实艇连续巡检通过证据。

![合成墙钟夹具中接收端 Setpoint 与来源年龄](../../results/v2/inspection_low_speed_envelope_20260928/received_source_age_timeline.png)

图中上幅是独立 DDS 观察者收到的最终速度，下幅只显示正向消息到达时的原始候选 header 来源年龄。橙色虚线是本次候选时效 1.3 s，灰色区为人为断流。图显示健康供数仍有间歇性零速和晚到的旧正向消息；曲线连线只帮助观察序列，不代表消息间持续输出。

## 试验边界与判据

使用隔离 ROS domain、10 Hz 静止合成导航/磁测/协方差源，预热 5 s、健康 5 s、断流 3 s、恢复 4 s。算法、独立 Setpoint 监督门、PID 和供数夹具各在单独进程，观察者在第五进程。低速候选配置在[独立 YAML](../../results/v2/inspection_low_speed_envelope_20260928/low_speed_sim_config.yaml)，产品默认配置和历史结果没有覆盖。门限固定为健康供数至少 40 条且最大缺口 ≤0.25 s、最终 Setpoint 至少 40 条且全部正向、最大空档 ≤0.30 s、故障后规定守卫窗全部零 Setpoint/零 PID；不能在看结果后删去启动或断流附近的零值来报“通过”。

0.2 m/s 目标速度＋声明的 0.05 m/s 横流上界，在候选 1.3 s 年龄时给出 0.325 m 的位移预算，低于本次 0.36 m 门槛。这只是速度和年龄的代数界；流速真实性、航迹误差、滑行和推进执行没有验证。算法配置中的传感器许可 0.75 s，独立输出门分别试 0.75/0.60 s；不能把放宽时效解释成传感器质量改善。

| 记录 | 健康供数最大缺口 | 健康正向/全部 Setpoint | 最终输出最大空档 | 故障守卫零 Setpoint | 结果 |
|---|---:|---:|---:|---:|---|
| 0.75 s 监督门、原 1+elapsed 仿真时钟 | 0.199 s | 31/38 | 0.455 s | 14/15 | 失败，漏出一条正向 |
| 0.60 s 监督门、原仿真时钟 | 0.602 s | 21/32 | 0.663 s | 17/17 | 失败，供数自身失约 |
| `use_sim_time=false`＋合成标定 | 未进入任务 | — | — | — | 启动安全门拒绝合成声呐标定 |
| 0.60 s 监督门、仿真身份＋墙钟 `/clock` | 0.221 s | 24/42 | 0.325 s | 16/16 | 失败，健康连续性不足 |

第一条在首次正向后健康段曾连续 31/31 条正向，但启动有 7 条预算状态未就绪零值，故障窗又漏一条，不能改报成功。第二条与第四条都出现健康段间歇性失鲜；由于本机进程负载、DDS 排队及供数缺口不同，不能从这四条非配对短试推断“0.60 s 比 0.75 s 更安全”或“低速必定稳定”。

## 来源时钟与接收端发现

原仿真夹具以 `1+elapsed` 发布 `/clock`。第二条有两条正向 Setpoint 在观察端到达时，按夹具预定的单调墙钟日程折算，其 header 来源年龄约为 1.86/1.99 s；这说明异步仿真时钟和数据可以以不同延迟到达，单看监督门内部 `/clock` 年龄不足以解释外部消息的真实墙钟年龄。直接以 `use_sim_time=false` 运行合成标定，被算法的非仿真目标身份门拒绝，这是应保留的安全约束。第四条改用同一主机墙钟推进 `/clock` 和传感器来源戳，仍保持 `use_sim_time=true` 与合成标定。该运行 63 条正向 Setpoint 中，独立接收端有 **9 条**来源年龄超过 1.3 s，最高 **1.757 s**。这只证明该观察端看到了旧候选消息；尚未证明 PID、桥或板端也接收了同一条，也没有给出真实海况发生率。

这一发现已转化为[同次时序验收 schema 2](INSPECTION_SAME_SESSION_TIMING_ENVELOPE_20260928.md)：真实目标运行需将最终 Setpoint 每条原始候选 header 转换到统一单调时轴，在**独立接收端**检查正向消息的到达时来源年龄；旧 schema 1 不再可通过。检查器构造数据 6 项通过，其中包含“门内年龄合格、接收时已旧”的负例。另有[巡检 PID 接收端来源门](INSPECTION_PID_RECEIVED_SOURCE_GATE_20260928.md)作为默认关闭候选，隔离 DDS 已验证旧正向消息在该层触发零推；它与本图不同次，不能把验证结果合并成完整链通过。现有板端包、完整路线和目标平台峰值负载数据仍为空，`field_release_authorized=false`。

另以相同低速 YAML 尝试 `wire` 种子 101 的 ROS/PID/本地 UDP/PVS 合成完整任务，预设仿真上限 2500 s、墙钟上限 300 s。该[运行状态记录](../../results/v2/inspection_low_speed_envelope_20260928/full_mission_ros_pvs_wire_seed101/run_status.json)为超时退出码 124，未生成完整任务摘要；因此既没有低速到端正例，也不能从超时断言路线几何不可能。需要在目标平台、稳定资源条件下重新取得同次任务包。结束时的 ROS context-invalid 回溯发生在超时 SIGTERM 后，不计作独立算法根因。

## 复现与原始记录

在 ROS Humble 和 `brain_linux/install` 环境中运行 `tools/smoke_v2_fixed_feeder_process.py`，传入 `--config-file` 指向上述 YAML、`--mission-speed-mps 0.2 --candidate-timeout-s 1.3 --displacement-bound-m 0.36 --snapshot-freshness --recoverable-input-stale --output-thread`；第四条另加 `--sensor-timeout-s 0.6 --wall-clock-fixture`，并选择独立 `ROS_DOMAIN_ID`/`--output-dir`。绘图命令：

```bash
python3 tools/plot_v2_inspection_received_age.py \
  --run-dir AUV-Master-Mag/results/v2/inspection_low_speed_envelope_20260928/low_speed_02_wallclock_fixture_timeout06_trial1 \
  --output AUV-Master-Mag/results/v2/inspection_low_speed_envelope_20260928/received_source_age_timeline.png
```

原始目录分别为 [0.75 s](../../results/v2/inspection_low_speed_envelope_20260928/low_speed_02_trial1/summary.json)、[0.60 s](../../results/v2/inspection_low_speed_envelope_20260928/low_speed_02_timeout06_trial1/summary.json)、[非仿真拒绝](../../results/v2/inspection_low_speed_envelope_20260928/low_speed_02_wallclock_timeout06_trial1/summary.json)和[仿真墙钟](../../results/v2/inspection_low_speed_envelope_20260928/low_speed_02_wallclock_fixture_timeout06_trial1/summary.json)。每个 summary 保留参数与源码 SHA-256；第四条 `summary.json` SHA-256 为 `7ca248e0dc98466857702402d19f68e4fd77e7287b26e6e66d9efc6ddc9adb1d`，观察事件 CSV 为 `68fab48c595fdce4085aa635af8572b451c971d304e499952c2e2285c41375b3`，图为 `cf81d93f1426359c644e3e2fa2ce4fed53853adb9cd4c51897a99c7e42e3100a`。图像只作诊断，不进入冻结论文主结论。
