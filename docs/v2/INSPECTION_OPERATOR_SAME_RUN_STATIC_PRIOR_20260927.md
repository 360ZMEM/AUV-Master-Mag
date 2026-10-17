# 静态目标先验启动门之后的同次合成链复核

> 本页固定的是清单启动门初版。后续增加任务与目标 ID 接令绑定后，又以当前源码[独立审计](publication/fwhm_v2_experiment_results/data/inspection_v2_same_run_mission_binding_20260927.json)完成同配方复跑；本页的原始目录和哈希不回写。

新增[无声呐静态目标先验清单门](STATIC_TARGET_PRIOR.md)改动了 v2 ROS 节点，因此原[同次链报告](INSPECTION_OPERATOR_SAME_RUN_20260927.md)固定的 7,732 条状态只属于门控加入前的源码。本次保留旧数据，另以独立目录重跑种子 101 的合成上行失鲜案例，重新经过 ROS 巡检节点、PID、自治守卫、桥、本地 UDP、原生 PVS、Zenoh 与 PySide6。新门只在非仿真、无运行时声呐的静态来源路径激活；这次仍使用合成声呐线和合成 AMD 上行，故主要验证改动后软件链无回归。

| 逐周期/状态检查 | 本次结果 |
| --- | ---: |
| PVS 更新周期 | 2,577 |
| 故障前允许自治的周期 | 2,336 |
| 上行失鲜后零 PID 推力、零 UDP 主推周期 | 241/241 |
| 桥与上位机按序号、时间戳配对的状态 | 7,730/7,730 |
| 界面明确显示上行失鲜且禁止自治的状态 | 721 |
| 未标定埋深发布 | 0 |

桥/界面状态消息量由异步周期调度决定，不应把 7,730 与旧跑次 7,732 的差异当作任务事件数量变化；固定的 2,577 个 PVS 周期与故障后的 241 个零推进周期分别逐行复核。[最终独立审计 JSON](publication/fwhm_v2_experiment_results/data/inspection_v2_same_run_static_gate_pinned_v2_20260927.json)从落盘的**桥和上位机两份状态轨迹**重算逐条配对，并校验原始 CSV、截图、同次摘要和 ROS/桥/界面/夹具源码 SHA-256，保持 `field_release_authorized=false`。[审计脚本](../../../tools/audit_v2_same_run_static_gate.py)可在本机重跑。此前仅落盘界面轨迹的中间审计保留为试验过程记录，不作为最终配对证据。

复跑时先遇到两类夹具问题：测试脚本仍等待守卫拒绝后的**新 MPC 时间戳**，并曾把桥序号采样放在 PC 模式命令之后，可能错过本周期已完成的安全决策；健康上行时间戳只在主循环刷新，同进程界面处理偶发超过桥守卫的 200 ms 墙钟门限，导致预定故障前就锁存上行失鲜。脚本现从该周期所有桥输入之前记录序号，在撤权分支等待新桥决策，并让合成健康上行在独立短周期刷新，到预定故障点才停止。安全守卫的 200 ms 阈值没有改动，最终仍逐包检查实际 UDP 输出与操作员状态。失败试跑未并入成功统计，旧结果也未重写；最终 CSV 对每个故障周期均记录零 PID 推力和零协议主推。这些是测试夹具与同步修正，未改变安全守卫或目标算法。

复现命令需先加载 ROS Humble 和 `brain_linux/install/setup.bash`，再设隔离 `ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY=1`、`QT_QPA_PLATFORM=offscreen`；运行：

```bash
python3 tools/smoke_v2_operator_chain_same_run.py \
  --output-dir AUV-Master-Mag/results/v2/operator_chain/same_run_guard_loss_static_gate_pinned_v2_20260927
python3 tools/audit_v2_same_run_static_gate.py
```

原始运行目录位于本机忽略的 `AUV-Master-Mag/results/v2/operator_chain/same_run_guard_loss_static_gate_pinned_v2_20260927/`。此处没有物理 PC104 收帧、跨设备 Zenoh、真实目标测绘或实艇流漂；零推力意味着停止推进命令，不能推断艇体已经停住。真实发布仍按[现场冻结矩阵](INSPECTION_V2_FREEZE_AND_FIELD_GATE_20260927.md)完成目标与执行两个实测验收包。
