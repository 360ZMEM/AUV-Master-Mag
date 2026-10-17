# 巡检任务到端、埋深与执行许可的分层输出

原 ROS 巡检 JSON 已有 endpoint_complete，但上位机桥接摘要不转发这一终态：正常完成有限 Track 时，界面可继续看到 TRACK 和 LOCALIZED_DEPTH_PENDING，无法直接区分“路线已到端”与“仍在巡检”。本补项只给 v2 巡检新增**软件任务结果**，沿用既有定位、深度有效标志与桥执行许可，不把它们合并成一个含糊的“成功”灯。

![图 62：正常到端与反向阶跃撤权时的任务、埋深及执行状态](publication/fwhm_v2_experiment_results/figures/62_inspection_task_depth_authority_outcomes.png)

## 输出合同

桥从巡检跟踪完整 JSON 提取 v2 诊断，向已有运行状态增加 task_status 和 task_status_fresh：

| 状态 | 成立条件 | 操作员解释 |
| --- | --- | --- |
| IN_PROGRESS | v2 Search／Confirm／Reacquire／Track，尚无合格终点 | 巡检进行中 |
| SOFTWARE_ROUTE_COMPLETE | Track／Complete 且 endpoint_complete=true，路线质量未拒绝 | **有限路线到端（软件）**；不含物理目标身份、埋深或艇体已停 |
| SUSPENDED | Hold 或 Sensor Degraded | 暂停／撤权，不报到端 |
| UNKNOWN | 非 v2、无效诊断、跟踪状态失鲜 | 不沿用上一次到端标签 |

桥以单调墙钟对任务诊断计时，超过与定位状态相同的候选 1.5 s 时改为 UNKNOWN；上位机若收不到新的桥状态，也按自己的 1.5 s 接收时效显示“未知”。界面将任务阶段改为“有限路线到端（软件）”或“暂停／撤权”，并继续单独显示定位/埋深及自治许可。执行许可仍来自实际仲裁、守卫和桥预算；任务到端**不会**自行重新授权推进。

## 两条同版安装链路证据

使用[图 61 的独立合成配置](../../results/v2/inspection_route_budget_clockfix_20260927/sonar_route_budget_sim.yaml)，两个种子 101 的任务均加载**安装空间 v2 节点**，经过 ROS2/DDS、巡检 PID、自治守卫、本地 UDP 编解码与原生 PVS；合成声呐目标线持续合格，世界系地速由理想 PVS 位置差分给出。原始跟踪、桥状态和每周期协议输出在[结果目录](../../results/v2/inspection_task_status_20260927/)独立保存。

| 条件 | 软件任务结果 | 深度与定位 | 终态执行输出 |
| --- | --- | --- | --- |
| 稳态正向 0.05 m/s 横流 | 1106.0 s 有限 80 m Track 到端；桥与运行状态均为 SOFTWARE_ROUTE_COMPLETE | 深度发布 0 次，定位状态仍为 LOCALIZED_DEPTH_PENDING | 当周期目标速度、PID 推力、UDP 主推均为 0，桥自治许可为假 |
| 巡检后突加反向 0.05 m/s 横流 | 883.5 s 因 2.009 m 路线横偏 Hold；桥与运行状态均为 SUSPENDED，从未出现 endpoint_complete=true | 深度发布 0 次，913.5 s 显示 RECOVERY_REQUIRED | Hold 后 60 s 的 241/241 周期三层零命令、桥自治许可为假；PVS 仍漂移 5.683 m |

正常组末样本的 PVS 地速仍约 **0.304 m/s**：任务软件到端与零命令不代表载体已物理停止。故障组发出的也是**上级处置请求**，并没有实际执行回收。图 62(c) 的两组终态执行许可均关闭：正常组桥原因码为 mpc_command_unavailable、终点目标速度归零，故障组因处置锁存；相同布尔值不能替代原因码。

从仓库根目录运行 python3 tools/audit_v2_task_outcome_same_run.py 可核查[逐例摘要、CSV、日志、配置、源码与图像哈希](publication/fwhm_v2_experiment_results/data/62_inspection_task_depth_authority_outcomes.json)。图像、生成器和对应节点／桥／上位机源码已在[冻结快照目录](publication/fwhm_v2_experiment_results/repro_sources/)固定；后续源码演进不会覆写图 62 的来源。本次 ROS v2 包 83 项和上位机状态视图 5 项测试通过，论文图像沿用宋体绘图样式。

本补项解决**已得软件证据如何向操作员无歧义输出**，不提高合成声呐、地图或横流的物理证据等级。真实目标关联、PC104 同次收帧、实艇转向/滑行和上级处置未验收，现场自治许可继续关闭。
