# v2 目标身份线索：从合成闭环到现场输入的缺口

当前 Track 身份连续性已在合成 ROS/PID/守卫/UDP/PVS 回路执行，但“可用于真实目标海缆”还取决于上游声呐线索的来源、目标关联、时间与误差。对工作区现有数据做只读清点：在 `.auv_data/bags`、`log` 和 `results` 下找到的 43 份 ROS bag `metadata.yaml` 中，`/auv/sensors/cable_sonar_observation` 与 `/auv/perception/quality/sonar` 的**非零消息总数均为 0**；唯一非零的 sonar 相关话题是一次仿真 bag 的前向坡度，共 857 条。`brain_linux/config` 内可识别的声呐概率标定是 `simulation_proxy`，明确标记 `physical_deployment_approved=false`，未发现已批准的物理声呐质量标定。

这只是所列三个目录和配置位置的**本地证据清点**，不能推断团队在仓库外也没有数据。逐 bag 元数据路径与 SHA-256、标定文件状态见[可复算清单](publication/fwhm_v2_experiment_results/data/field_sonar_evidence_inventory.json)，由 `python3 tools/audit_v2_sonar_evidence_inventory.py` 生成。图 29、图 30 的声呐线和质量概率来自脚本夹具，不能补足上述缺口。

## 现场输入最小验收包

| 输入或证据 | 必须能核对什么 | 缺失时的运行合同 |
| --- | --- | --- |
| 原始声呐线与质量流 | 同一物理采集时间戳、坐标系、原始点、拟合线、协方差、样本数与有效标志；证明不是消息到达时间代替采集时间 | 不向 Confirm/Track 提供目标身份许可 |
| 声呐几何标定 | 杆臂、安装角及其误差上界，和导航时钟/姿态在同一时域的同步记录 | 不把线索投影为窄世界系区间 |
| 概率标定 | 物理数据训练与独立留出；实际输出 `p_track` 的可靠性及失效/误检案例；标定文件版本、物理批准和 SHA-256 | `p_track` 不得驱动自主身份门 |
| 目标关联 | 已确认航迹、可信测绘或人工标注给出“该声呐线属于任务缆”的独立依据；特别包括邻缆、交叉、假线与失线 | 只报告磁异常中心；不宣布目标海缆完成 |
| IMU 采样与航向质量 | 传感器原生采样时间或计数、原始日志到 PC104/ROS 的同样本映射、实测时延及独立姿态协方差标定；不能用 PC104 组包时间替代 | 导航质量不准入，巡检 Hold；按[独立预检](INSPECTION_IMU_ACQUISITION_PREFLIGHT_20260927.md)归档证据 |
| 执行包络 | 实艇低速转弯、反向流、停机距离、PC104 收到的撤权报文与最终运动 | 仅可报告本地仿真零命令，不宣称现场安全停止 |

建议把首次实测安排成短距离、已知目标线和已知邻线的受控横切。先冻结目标身份、声呐外参与验收阈值，再分别采集目标存在、目标缺失、邻线更强、线索中途丢失/改指四类原始记录；留出组不得参与阈值或概率曲线选择。每次同步记录磁场、导航协方差、声呐原始线与质量、仲裁状态、PID 命令、PC104 收帧和真实轨迹。先离线按同一转换器重放并核对误拒/误放行，再在预留走廊与硬件零推力保护下做闭环，最终才冻结现场配置。具体几何、速度和阈值必须由实测包络代入，不能直接照搬仿真 `0.5/0.6 m` 和 `2 s`。

软件入口现有两条不同路径：没有声呐质量标定时，可使用可信且足够窄的静态侧向线索，但来源真实性必须由外部任务记录证明；选择运行时声呐门时，非仿真配置必须提供与质量节点一致的、物理批准的标定文件及固定哈希，证据不全则拒绝启动。后者是**防误用预检**，即使配置文件自称已批准，也不能替代真实数据与人工签核。

现场配置在 `v2.runtime_identity` 中填写 `calibration_id`、`calibration_file` 和 `calibration_sha256`（文件的完整 SHA-256 十六进制值）；文件须显式包含 `calibration_domain: physical`、`deployment_approved: true`、`physical_deployment_approved: true` 及有效检测/跟踪概率曲线。启动时还要显式设置 `enable_sonar_quality:=true`、`calibration_domain:=physical`，并让质量节点的 `sonar_calibration_file` 指向**同一文件**。v2 入口在启动时核对文件、哈希、ID 和批准字段；目前 ROS 消息本身只带标定 ID，不带文件哈希，因此质量节点与 v2 节点是否加载同一文件还依赖部署配置审计，不应把这一步写成运行时已自动认证。

本轮[软件候选快照](publication/fwhm_v2_experiment_results/data/inspection_v2_candidate_snapshot_20260927.json)同时列出当前实现及证据哈希与未完成的现场验收项，并明确 `field_release_authorized=false`，以免把合成测试通过误作现场参数冻结。

后续[身份故障后的零推力运动续审](INSPECTION_IDENTITY_STOP_DRIFT_20260927.md)又证实，即使控制与 UDP 报文连续 60 s 均为零，原生 PVS 在零流/横流中仍移动约 `4.64/5.48 m`；因此现场输入包还须包含实艇停止与有流漂移测量，以及 Hold 限时后经认证的上级处置。
