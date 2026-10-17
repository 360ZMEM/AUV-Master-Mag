# 上行失鲜时的同次 PVS—上位机运行状态闭环

先前的 ROS/PID/协议桥/原生 PVS 故障实验与桥—Zenoh—PySide6 回环是两次独立运行。复核发现：旧 `cable_monitor` 主要随 AMD 上行遥测转发；上行恰好失鲜时，桥虽在 ROS 与本地 UDP 侧撤权，上位机却可能只能等待自身 1.5 s 超时，而收不到桥当期明确的 `AMD_UPLINK_STALE` 原因。现由 v2 专用桥在每次有效仲裁状态更新后，另外通过可选 Zenoh 侧通道发送轻量 `inspection_operational_status` 消息。它包含命令序号、状态时间戳、定位标签、自治许可和原因。传输异常只记录告警，不打断 ROS 状态或本地 UDP 的安全决策；上位机只更新许可标签，不把这类轻量消息当作完整声磁遥测。

![同一次合成任务中上位机收到上行失鲜撤权](publication/fwhm_v2_experiment_results/figures/operator_console_same_run_guard_loss.png)

在一个隔离 ROS domain、本机 UDP 和 Zenoh 端口的运行中，合成单直导线、合成目标声呐及合成 AMD 健康夹具驱动 ROS2 巡检节点、PID、专用协议桥和原生 PVS。进入 Track 后主动停止刷新 AMD 上行时间戳。PySide6 诊断页在同一次任务中接收桥决策路径发出的状态，而不依赖新的 AMD 上行报文。

| 同次运行观察 | 原始数据复算 |
| --- | ---: |
| PVS 配对周期 | 2,577 |
| 撤权前自治许可周期 | 2,336 |
| 上行失鲜后撤权且 UDP 主推零 RPM | 241/241 |
| 桥状态与上位机按命令序号、时间戳配对 | 7,732/7,732 |
| 上位机收到 `AMD_UPLINK_STALE` 且显示禁止自治 | 723 条状态 |
| 失鲜后的定位标签 | `LOCALIZED_DEPTH_PENDING` |
| 埋深发布 | 0 |

状态发布可在同一命令序号内发生多次，因此审计用 `(command_sequence, timestamp_s)` 配对，不能只按序号去重。上位机最后显示“定位: 已定位（埋深未许可） | 自治许可: 禁止（上行失鲜）”。这证明**本机合成故障运行**中的定位与执行许可显示不会因 AMD 上行停止而只剩模糊超时；也证明 UDP 下行主推已在桥出口归零。它不证明实艇静止、PC104 收到报文、跨设备 Zenoh 连续性，或真实 AMD 故障后的所有处理时延。

[独立逐文件审计](publication/fwhm_v2_experiment_results/data/inspection_v2_same_run_operator_chain_20260927.json)固定 PVS CSV、上位机状态轨迹、截图和源代码哈希，并将 `field_release_authorized` 保持为 `false`。复现入口：

```bash
source /opt/ros/humble/setup.bash
source brain_linux/install/setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=<空闲域> QT_QPA_PLATFORM=offscreen \
  python3 tools/smoke_v2_operator_chain_same_run.py \
  --output-dir AUV-Master-Mag/results/v2/operator_chain/same_run_guard_loss_verified
python3 tools/audit_v2_same_run_operator_chain.py
```

本机原有 `brain_linux/install` 中若干包缺失 `ament` 资源索引，导致源码测试中一个 launch 构建项无法找到包。先在 `/tmp` 隔离构建确认源码，再以非符号链接方式重建仓库安装空间的 v2 及六个依赖；现在正常 `source brain_linux/install/setup.bash` 后，ROS 包测试 `46/46` 通过，上位机状态测试 `4/4` 通过。该构建修复只是当前开发机可复现性，不是现场固件发布或硬件验收。
