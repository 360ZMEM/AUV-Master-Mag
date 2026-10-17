# v2 目标缆身份线索的跟踪期连续性

图 29 验证了 Confirm 阶段的运行时声呐身份门，但未覆盖进入 Track 后线索丢失或改指邻缆。本补项将同一可选线索延续到 Track：若新鲜声呐线与当前跟踪目标的横向位置不一致，立即进入 Hold；若没有合格线索持续超过 2 s，也进入 Hold。两种情况均撤销横向许可与任务完成许可，控制器目标推力和本地 UDP 协议转速归零。埋深仍须通过独立场形与标定门；本补项四组均不发布埋深。

![图 30：身份线索连续性、故障停止与分阶段精度预算](publication/fwhm_v2_experiment_results/figures/30_runtime_identity_continuity.png)

## 门控合同与反例

可选配置 `require_runtime_identity_cue=true` 时，Confirm 要求磁中心落入合格声呐线索的两倍标准差区间，且该区间半宽不超过 `0.5 m`。Track 继续要求预测目标线落入相应区间，容许半宽上限为 `0.6 m`，最后一次一致观测至多间隔 `2 s`。线索质量、时间戳、坐标系、线拟合、概率校准、原始点一致性和外参审核由[前一补项](INSPECTION_RUNTIME_SONAR_IDENTITY_20260927.md)的转换器承担。这个 `0.5/0.6 m` 分阶段预算只是当前合成试验包络，尚非实测声呐误差上界。

直接沿用 Confirm 的 `0.5 m` 上限到 Track 时，正常合成回归段在 `625.5 s` 因观测半宽稍大于 `0.5 m` 被误停。这是保留的探索反例，不计入下表最终版结果。分阶段预算后，正常运行在 Track 中的最大有效半宽为 `0.5011 m`，仍低于 `0.6 m`；邻缆改指 `3 m` 的冲突仍被立即拒绝。全任务最大有效半宽约 `0.5999 m` 出现在 Search，不应误称 Track 的观测半宽。

上述直接 `0.5 m` 误停反例原始摘要仍在 `results/v2/inspection_identity_continuity_20260927/clean/wire_seed_101_summary.json`，SHA-256 为 `a18e79c060ca04a4d01ed96c83afc59b2ff7023c30763147a3c0c687f50a1f04`；它早于下表最终版，不与其混合统计。

合成声呐线由 **PVS 载体真值位姿**生成，运行时转换器只取得独立的导航估计；导航受扰组因此不会因同一带误差位姿同时生成观测与转换观测而发生误差抵消。四组均为种子 101、零流、目标直线 `y=6 m`、合成声呐线和直接注入的“已标定”质量概率，经过 ROS DDS、PID、自治守卫、本地 UDP 编解码及原生 PVS 内环；未连接物理声呐或 PC104。

| 最终版配方 | 首次 Track | 结果 | 完整正向 Track 真值横偏 MAE / 最大值 | 末周期 PID / UDP | 埋深 |
| --- | ---: | --- | ---: | ---: | --- |
| 持续合格线索 | 577 s | 1105.25 s 到终点 | 0.754 / 1.346 m | 0 / 0（终点） | 空 |
| Track 后线索丢失 | 577 s | 587 s Hold：`flythrough_runtime_identity_lost` | 未形成完整航段 | 0 / 0 | 空 |
| Track 后改指 `y=9 m` 邻线 | 577 s | 585 s Hold：`flythrough_runtime_identity_diverged` | 未形成完整航段 | 0 / 0 | 空 |
| 导航受扰、线索持续合格 | 594 s | 1139.25 s 到终点 | 0.856 / 1.743 m | 0 / 0（终点） | 空 |

导航受扰组注入位置噪声标准差 `0.05 m`、航向固定偏差 `1°` 和世界系速度偏差 `0.01 m/s`。表中误差由 PVS 真值轨迹与目标 `y=6 m` 的距离计算，故与节点在线路由误差口径分开；节点在线 MAE 分别为 `0.754/0.853 m`。示例 `1 m` 均值、`2 m` 最大横偏门仅用于本任务配方，不代表现场验收阈值或成功概率。两种故障停止都只是**指令与仿真内环**证据，实艇停止距离仍须测量。

同一最终版配置又在种子 102、103 各复跑四种条件，共 `3×4=12` 次：正常和导航受扰两类各 `3/3` 到终点，完整正向 Track 真值横偏 MAE 分别为 `0.754–0.758 m` 与 `0.856–0.863 m`，最大值分别为 `1.337–1.346 m` 与 `1.743–1.760 m`；线索丢失与改指邻缆各 `3/3` 以原定原因 Hold，末周期 PID/UDP 双零，十二次均不发埋深。[三种子逐例审计](publication/fwhm_v2_experiment_results/data/30_identity_continuity_three_seed_audit.json)列出原始 CSV/JSON 哈希，由 `python3 tools/audit_v2_identity_continuity_repeats.py` 复算。三种子共享同一合成场形、几何、控制器和概率夹具，只改变噪声实现；这些比例不是实海况成功率。

## 复现与证据边界

逐例冻结 CSV、JSON 的路径与 SHA-256，以及实现文件哈希，见[图 30 摘要](publication/fwhm_v2_experiment_results/data/30_runtime_identity_continuity_summary.json)和[图像清单](publication/fwhm_v2_experiment_results/data/30_runtime_identity_continuity_manifest.json)。图 29 保留为较早的 Confirm-only 版本，不能把它的哈希或结果当作 Track 连续性验证。

```bash
source /opt/ros/humble/setup.bash
source brain_linux/install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=224
cfg=brain_linux/src/auv_cable_tracking_v2/config/cable_tracking_v2_flythrough_sonar_identity_sim.yaml
base=AUV-Master-Mag/results/v2/inspection_identity_continuity_20260927
for pair in true_clean:none true_dropout:dropout true_switch:switch; do
  name=${pair%%:*}; fault=${pair##*:}
  python3 tools/smoke_v2_flythrough_ros_pvs.py --case wire --seed 101 \
    --config-file "$cfg" --sonar-identity-mode target \
    --sonar-track-fault "$fault" --with-bridge --output-dir "$base/$name"
done
python3 tools/smoke_v2_flythrough_ros_pvs.py --case wire --seed 101 \
  --config-file "$cfg" --sonar-identity-mode target --with-bridge \
  --nav-position-noise-std-m 0.05 --nav-heading-bias-deg 1 \
  --nav-speed-bias-mps 0.01 --output-dir "$base/true_nav_perturbed"
python3 tools/plot_v2_identity_continuity.py
```

ROS 启动文件另提供默认关闭的 `enable_sonar_quality`，用于显式启动质量节点并传入标定文件；需同时选择可追溯的 `calibration_domain`。合成概率和线点在此由测试夹具构造，`p_track` 只表示声呐线质量，**不能单独证明它是任务目标缆**。若上游声呐也锁到邻缆，声磁一致仍可能共同误认。真实作业须补齐目标线与可信地图/既有航迹的关联、物理采集时间戳、外参与不确定度实测、真实概率留出标定、并行缆误检和 PC104 停机距离；这些条件缺失时应沿路径 3 保持拒绝或只报告磁异常，不宣布目标海缆完成。场形系数和真实埋深仍另需独立几何真值标定。

三种子闭环运行完成后，非仿真 v2 声呐身份入口另增加**标定文件预检**：配置必须指定物理域、显式物理部署批准、ID 匹配的质量曲线文件及其 SHA-256；缺文件、仅改 `synthetic`/`sim_proxy` 名称、未批准或文件被替换均拒绝启动。此后置防误用修改不追溯改写图 30 和三种子的原始数据。当前仓库[现场输入清点](INSPECTION_FIELD_INPUT_READINESS_20260927.md)显示还没有满足这一条件的物理声呐标定或目标缆声呐记录；预检通过也只证明配置文件声明自洽，不能证明真实标定有效。

随后[零推力后的动力学续审](INSPECTION_IDENTITY_STOP_DRIFT_20260927.md)在身份故障 Hold 后保持同一控制/协议链运行约 60 s，量化原生 PVS 的惯性与有流位移。图 30 的末周期零命令与该续审的实际停机运动是两个不同口径，不能相互替代。
