# 偏移地图下的运行时身份授权：从拒绝真实目标到可完成巡检

图 54 证明了一个窄但偏移的静态地图会把真实单源拒绝、把邻源融合峰放行。本补项不修改历史任务或旧参数：在 v2 浅角穿越中新增默认关闭的 `runtime_identity_supersedes_side_hint_center`。启用后，静态侧向区间仍必须通过方向、扫描范围和峰外余量预检，但**不再作为 Confirm 的目标中心真值**；Confirm 和 Track 由新鲜、有界的运行时声呐线索核对磁中心。开关没有同时启用 `require_runtime_identity_cue` 时直接拒绝配置。无声呐或证据冲突仍撤销 Track 许可，埋深独立保持空值。

![图 55：偏移地图下声呐线索授权顺序、单源恢复、邻源拒止与失鲜 Hold](publication/fwhm_v2_experiment_results/figures/55_sonar_map_bias_recovery.png)

## 同条件闭环与因果对照

使用同一合成目标缆 $y=6$ m、偏移地图 `[6.6,7.5]` m、零流、种子 101、30 m 浅角横切和原生 PVS；声呐夹具始终指向目标 6 m。为隔离授权顺序，再加一例“声呐已经输入，但仍先执行旧静态地图中心否决”。两类双源磁场都在 $y=9$ m 加入半幅标量包络。每例经 ROS DDS、巡检 PID、自治守卫和本地 UDP 桥闭环；数据不是物理声呐或多芯海缆场形。

| 配方 | 磁中心 (m) | Confirm／终态 | 相对目标真值的完整 Track MAE／最大 (m) |
|---|---:|---|---:|
| 地图单独＋单源 | 6.014 | 577 s 拒绝真实目标 | — |
| 地图单独＋邻源 | 7.026 | 1105 s **误报完成** | 1.090／2.309 |
| 声呐存在＋旧授权顺序＋单源 | 6.014 | 577 s 仍被静态地图否决，声呐质量显示已接受 | — |
| **声呐主导＋单源** | 6.014 | **1105.25 s 完成 80 m Track** | **0.754／1.346** |
| 声呐主导＋邻源 | 7.026 | 577 s 因 `runtime_identity_cue_inconsistent` 拒绝 | — |

图 55(a) 使用第一条 Track 样本的真实软件线索中心 6.000 m、半宽 0.430 m，而非人为画出的固定带。图 55(b) 显示相同地图下的五条任务轨迹；归档还包含图 54 的地图单独＋单源对照，共六例。新顺序的双源组在确认终点的 PID 推力和 UDP 主推 RPM 均为零；单源完成组的在线 Track MAE 为 0.754 m。两组均无埋深发布。[六例逐时 CSV、摘要及源码/配置 SHA-256](publication/fwhm_v2_experiment_results/data/55_sonar_map_bias_recovery.json)由 `python3 tools/audit_v2_sonar_map_bias_recovery.py` 重算。

在新顺序的单源任务进入 Track 后中断声呐线索，587 s 转为 `flythrough_runtime_identity_lost`。继续观察 15 s，**61/61 个 Hold 周期目标速度、PID 推力及 UDP 主推均为零**，无任务完成或埋深发布；原生 PVS 仍位移 2.815 m，因此零命令不等于原地停止。此例同样归入图 55 与上述逐例哈希，证明授权顺序调整没有破坏原有 Track 失鲜撤权合同。

算法级新增回归还检查：偏移静态图＋旧顺序拒绝真实单源；新顺序＋合格线索准入真实单源；新顺序＋邻源或线索缺失均拒绝。v2 磁巡检 15 项和 ROS 巡检包 66 项测试通过。旧 v1、默认 v2 配置、历史 5600 次稳定性基线和图 54 原结果均不改。

## 工程适用域与现场口径

这给出一条比“地图必须准到目标中心”更易收口的任务输入合同：地图或上一段航迹只需提供**可信目标侧向符号、预对准起点和覆盖/边界预算**；实际目标中心与 Track 连续身份由独立的声呐线索校核。对受控合成的 1.1 m 地图偏移，新工作流既恢复了正确单源任务，又保留了融合邻源的拒绝出口。创新点是**把搜索几何先验与目标身份真值解耦，并用证据失鲜撤权维持任务连续性**，而非给 FWHM 重新调系数。

运行时 `p_track` 只表示声呐线拟合/跟踪质量，**并不自动证明线属于指定海缆**。现场 F 剖面仍须以独立测绘、可信上段航迹或人工确认建立目标关联，并在邻缆场景验证该关联；声呐原始采集时间、外参、协方差和概率标定须用实物留出数据验收。若声呐改指另一条线而磁场也随之吻合，本软件的几何一致性门不能凭空识别共同错误。物理声呐与 PC104 同次原始包仍缺，现场许可保持 `false`；这是一条**工程上可审查的条件化候选路径**，不是实艇成功率。

## 安装与复现

之前的隔离脚本会自行把 `AUV-Master-Mag/src` 加到 `sys.path`，因而脚本成功不证明 ROS 安装空间能直接导入磁算法。本补项给子仓新增最小 `setup.py`；已在本机用系统 Python 执行 `/usr/bin/python3 -m pip install --user --no-deps -e AUV-Master-Mag`。从 `/tmp` 仅 source ROS Humble 和 `brain_linux/install`，即可同时导入已安装 ROS 节点与磁算法，并用安装入口加载新开关为 `True` 的合成配置。该本机 editable 安装是开发 smoke；现场应把磁算法版本作为固定发布依赖，不依赖临时 `PYTHONPATH` 或可变源码路径。

此外，从 `/tmp` 在未注入任务、未开启桥的条件下执行 12 s 安装版 `ros2 launch auv_cable_tracking_v2 flythrough_nodes.launch.py config_file:=<本次合成配置> use_sim_time:=true`，巡检节点和 PID 节点都成功启动，未出现磁包导入失败。[图 55 数据 JSON](publication/fwhm_v2_experiment_results/data/55_sonar_map_bias_recovery.json)同时固定安装版空闲启动日志与 `setup.py` 的 SHA-256；日志在忽略的 `results/v2/inspection_sonar_map_bias_recovery_20260927/installed_launch_idle.log`。这是空闲启动 smoke，不含巡检任务或真实 PC104 动作。

完整复现实验需先由 [`cable_tracking_v2_flythrough_sonar_identity_sim.yaml`](../../../brain_linux/src/auv_cable_tracking_v2/config/cable_tracking_v2_flythrough_sonar_identity_sim.yaml) 复制两个合成配置：均改 `side_hint_min_m/max_m` 为 `6.6/7.5`；新顺序配置设 `runtime_identity_supersedes_side_hint_center: true`，旧顺序设 `false`。本机生成的两份配置、逐时日志位于 `AUV-Master-Mag/results/v2/inspection_sonar_map_bias_recovery_20260927/`，配置 SHA-256 已写入图 55 JSON。运行现有 `tools/smoke_v2_flythrough_ros_pvs.py`，为每例设置隔离 `ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY=1`、`--with-bridge --case wire --seed 101 --cable-lateral-m 6 --sonar-identity-mode target`；双源另加 `--secondary-cable-lateral-m 9 --secondary-cable-gain-nt 10`，失鲜例另加 `--sonar-track-fault dropout --post-hold-coast-s 15 --max-time-s 750`。最后运行 `python3 tools/audit_v2_sonar_map_bias_recovery.py`。这些参数仅用于合成复核，不是实艇参数包。
