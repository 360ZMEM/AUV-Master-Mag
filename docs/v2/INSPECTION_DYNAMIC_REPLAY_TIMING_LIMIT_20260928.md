# 移除在线 PVS 后的动态航迹实时重放边界

[分进程 PVS 短试](INSPECTION_PROCESS_PVS_LIVENESS_20260928.md)发现模拟器供数本身可能超时。为单独检验产品 ROS 输入输出，本补项从[图 61 的一条历史全路线](../../results/v2/inspection_route_budget_clockfix_20260927/steady_plus_101/wire_seed_101.csv)提取最初 12 s 位姿、航向和导航地速，在墙钟 10 Hz 进行开环插值重放；磁异常与声呐目标线索仍由脚本按合成场重新生成。已安装算法、PID、供数器和独立 DDS 记录器继续分进程运行，**不启动在线 PVS，也不把本次指令反馈给历史轨迹**。它是动态输入时序试验，不是再次完成图 61，也不是物理传感器复现。

![图 70：动态航迹重放的接收空档与发布阶段耗时](publication/fwhm_v2_experiment_results/figures/70_dynamic_replay_without_pvs_timing.png)

| 独立短试 | 导航／磁测名义周期 | 合成声呐与协方差 | 磁测发布／独立接收最大空档 | Setpoint／PID 独立接收最大空档 | 正向 Setpoint／PID | 结果 |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| `dynamic_search_01` | 0.10 s | 每周期 | 1.077 / 1.167 s | 1.220 / 1.187 s | 0/46、0/217 | 仅搜索开头；未到 Confirm/Track |
| `dynamic_search_02_sonar_5hz` | 0.10 s | 每 2 周期，即名义 5 Hz | 1.063 / 0.820 s | 0.929 / 0.946 s | 1/34、3/198 | 仅搜索开头；未到 Confirm/Track |

两趟各有 120 次导航和磁测发布，独立 DDS 接收端逐条收到并可用同机单调钟与来源戳联表，审计 `recording_reviewable=true`，运行前后各自 9 个关键输入文件哈希一致。但**这不是受控配对降频实验**：供数夹具在两趟之间增加了阶段 CPU 计时，进程调度也不可重放。第二趟声呐名义 5 Hz 且最大空档较短，仍有约 0.82 s 的磁测接收空档和几乎全部非正向输出，不能据此冻结 5 Hz 方案或称身份线索可靠。两趟没有实际桥、PC104、运动反馈、物理声呐或真实磁标定；现场许可均为 `false`。

图 70 使用第二趟单次原始记录。在线 PVS 积分耗时已近零，图 (a) 仍见磁测与最终 Setpoint 接收长空档。图 (b) 分开本机供数器传感器发布阶段的墙钟与线程 CPU：中位约 0.039/0.025 s，最长约 0.731/0.253 s。墙钟减去线程 CPU 的差额包含调度、阻塞或其他等待，**不能逐点自动归因到 DDS**；但发布阶段本身已经能超过 0.10 s 名义周期，解释了本机夹具仍无法稳定供数的至少一部分。此前固定供数、监测分频与分进程 PVS 试验也分别出现连续性负例，因此不再靠本机频率／阈值小范围试探宣称修复。

重放文件来自受限合成路线，其轨迹已预先决定；重新合成磁测使用本次夹具声磁模型和随机种子，不等于逐点复制历史磁传感器记录。独立记录器对 `/clock` 秒数按 Unix 历元检查会拒绝来源年龄解释；[联表审计器](../../../tools/audit_v2_pvs_process_capture.py)只用本机发布时间和接收时刻，不声称跨设备对时。重放端为了检查本地协议编解码仍计算转速，但 `plant_actuation_applied=false`，任何非零编解码指令都不能写作艇体推进；本两趟实际没有非零转速组合。原始清单、日志和审计在[数据目录](../../results/v2/inspection_process_replay_20260928/)。

下一步有效的冻结判据仍是目标 Jetson 上的真实部署拓扑，先量出导航／磁测原生采集到 ROS、算法决策、最终 Setpoint、PID 与桥 UDP 的**同次**来源时效和最大空档；若仍超门，再按目标平台上的实际热点改动。正常到端、故障后持续归零、板端真实收帧和艇体滑行/转向需遵守[时序验收合同](INSPECTION_SAME_SESSION_TIMING_ENVELOPE_20260928.md)与物理目标证据门。现有本机数据支持“合成方法在限定路线可推进，并能按软件门撤权”的答辩口径，不支持“已经稳定实时部署”。

后续另起一趟[原生 ext4 记录目录短试](../../results/v2/inspection_native_log_replay_20260928/README.md)，避免本仓 `vboxsf` 上的逐周期 CSV 刷新；120 条磁测完整可联表，发布/接收最大空档仍为 0.736/0.915 s，最终 Setpoint 3/39 正向、最大空档 1.178 s，未到 Confirm/Track。此趟使用已修复概率/年龄域的 v2 入口，但正常夹具值合法；没有正式时序合同或物理板端输入。该单趟不构成受控 I/O 收益对照，也不证明共享文件系统是唯一失效原因；它再次限定“记录环境改进”不能替代任务连续性验收。原两趟及图 70 数据不改，不另增主讲图。

复算：

```bash
python3 tools/audit_v2_pvs_process_capture.py \
  --run-dir AUV-Master-Mag/results/v2/inspection_process_replay_20260928/dynamic_search_01
python3 tools/audit_v2_pvs_process_capture.py \
  --run-dir AUV-Master-Mag/results/v2/inspection_process_replay_20260928/dynamic_search_02_sonar_5hz
python3 tools/plot_v2_dynamic_replay_budget.py \
  --run-dir AUV-Master-Mag/results/v2/inspection_process_replay_20260928/dynamic_search_02_sonar_5hz \
  --output-base AUV-Master-Mag/docs/v2/publication/fwhm_v2_experiment_results/figures/70_dynamic_replay_without_pvs_timing
```

再跑时须先 `source brain_linux/install/setup.bash`，设置隔离的 `ROS_DOMAIN_ID` 与 `ROS_LOCALHOST_ONLY=1`，给包装器传新的 `--output-dir`、历史 CSV 的 `--replay-route-csv` 和可选 `--sonar-every-n 2`。退出码 0 仅表示短循环和记录器正常结束。
