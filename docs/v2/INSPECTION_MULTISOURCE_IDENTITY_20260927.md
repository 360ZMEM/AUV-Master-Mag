# 磁异常中心与目标海缆身份：双源干扰闭环审计

本补项针对巡检发布合同中的一个独立问题：磁场剖面具有单个、近似对称的峰，是否足以证明峰中心对应任务海缆。答案是否定的。`lateral_valid` 只表示几何中心通过质量门；完整沿缆的在线横偏也只测量载体到**已选择磁中心**的距离，不是到目标海缆真值的距离。原有 5600 次稳定性比较、场形留出和图 25--27 保持冻结。

## 受控输入与结果

隔离 ROS2--巡检 PID--自治守卫--本地 UDP--原生 PVS 回路保持合成目标缆横向位置 $y=6$ m、直线先验、零流和种子 101。磁输入是两个单直导线型**标量异常包络**之和：目标缆幅值 20 nT，另在 $y=9$ m 加入 10 nT 干扰；这是针对算法的合成反例，**不是多芯海缆电磁矢量场实测或 Biot--Savart 多导线验收**。其余任务及控制参数不变，只把侧向区间从宽提示 `[2,8] m` 换成模拟独立线索 `[5.5,6.4] m`。第三次去掉干扰，以检验窄线索是否妨碍正常单源任务。窄线索仍标记为 `synthetic_test_fixture`，非仿真 ROS 时钟会拒绝启动。

![图 28：双源磁峰、真实航迹、在线与目标真值横偏、身份佐证决策](publication/fwhm_v2_experiment_results/figures/28_multisource_identity_gate.png)

| 条件 | 磁中心 (m) | 在线完整沿缆 MAE/最大 (m) | 相对目标缆真值 MAE/最大 (m) | 终态 |
|---|---:|---:|---:|---|
| 宽提示＋双源 | 7.026 | 0.721 / 1.283 | **1.090 / 2.309** | 错误完成、在线门仍有效 |
| 窄线索＋双源 | 7.026 | — | — | 577 s 确认阶段拒绝，PID/UDP 零推进 |
| 窄线索＋单源 | 6.014 | 0.754 / 1.332 | 0.754 / 1.346 | 正常完成，示例 1/2 m 真值门通过 |

宽提示场景的中心偏移约 1.026 m，但仍落在 `[2,8] m`。在线质量门只看随该中心跟踪的偏差，故把任务判为完成；真值显示完整正向 Track 的平均和最大横偏均超示例门。窄线索的上界加 0.5 m 观测容差为 6.9 m，测得的 7.026 m 中心被现有 `side_hint_observation_inconsistent` 门拒绝；该周期末 PID 推力与桥接 UDP 解码 RPM 都为零，埋深始终为空。去掉干扰后中心为 6.014 m，窄线索正常授权，说明该对照不是恒定禁用任务。

相同三条件再分别以种子 102、103 全链路复跑，得到 **宽提示双源错误完成 3/3、窄线索双源确认拒绝 3/3、窄线索单源合格完成 3/3**，埋深发布 0/9。三种子只改变 0.03 nT 合成磁噪声；干扰位置、强度、单一直线先验和控制包络固定，不能用 3/3 推断其它多源场的识别率。九条日志的中心、在线与真值误差、末周期执行量及 CSV SHA-256 见[三种子逐例审计](publication/fwhm_v2_experiment_results/data/28_multisource_identity_three_seed_audit.json)，可由 `python3 tools/audit_v2_multisource_repeats.py` 重算。图 28 只绘种子 101。

## 发布口径和实物前提

这一反例补出了**第三类许可**：横向几何中心、目标身份佐证、物理埋深。目标身份佐证不能由同一磁峰的 FWHM 质量分数或在线贴峰误差自证。在有多个磁异常可能共存的作业走廊，若没有与磁场独立、位姿同框且足够窄的声学／近距调查／上一段可信航迹线索，则只能报告“磁异常中心”，不能把它作为目标海缆的任务完成。若任务确实提供这种线索，则在 Confirm 阶段比较其区间与实测磁中心，冲突时保持而非误认。当前代码已执行区间一致性门，本实验只新增合成干扰注入、窄线索配置和闭环证据；**没有接入真实声呐横向线索或验证其误差和时间戳**。

这里的 `[5.5,6.4] m` 和 0.5 m 容差仅证明本反例能被一个合成独立线索拦住，不能推出所有双源间距、幅值或海床条件下的通用阈值。字段 `side_hint_source` 仍是配置标签，不是来源真实性证明。实物发布前必须对线索来源、时间、坐标系和误差上界作独立验收；若只能拿到宽且不可靠的地图，当前受限模式应停在搜索／确认或给出“未认证磁异常”，不宣称目标缆巡检完成。

为避免仅将宽地图配置的来源标签改成 `survey_map` 就进入非仿真任务，ROS v2 浅角度入口新增候选启动门：`use_sim_time=false` 时，侧向区间宽度必须不超过 1.0 m。宽区间在首周期返回 `flythrough_side_hint_too_wide` 并零目标速度；若 YAML 试图显式把上限调大，节点直接拒绝启动。仿真配置与历史结果保持原行为；独立窄线索配置可通过该宽度门。**1.0 m 是此候选包络的保守停机阈值，不是任何声呐的已验证误差指标**。即使宽度满足，来源真实性、时间和坐标系仍未由该静态配置门验证，不能据此宣称实物目标身份已闭环。

仓库已有 `auv_interfaces/msg/SonarCableObservation.msg` 与 `/auv/sensors/cable_sonar_observation` 桥接话题，但当前 v2 巡检节点没有把该消息转换成带位姿、外参、时间戳和不确定度的目标侧向区间；现存声呐质量节点还是旁路诊断。下一步真实接入应从该消息生成任务身份线索，在 ROS 时钟、传感器外参和 `world` 坐标系下完成同帧验证，再把线索的来源与有效区间写入任务记录。**仅将配置里的 `synthetic_test_fixture` 改名为 `sonar` 不构成接入。**

复现需先 source ROS Humble、巡检包 colcon 安装空间，设置 `ROS_LOCALHOST_ONLY=1` 与独立 `ROS_DOMAIN_ID`，然后使用 `tools/smoke_v2_flythrough_ros_pvs.py --with-bridge --case wire --seed 101 --cable-lateral-m 6 --secondary-cable-lateral-m 9 --secondary-cable-gain-nt 10`。宽提示使用 `cable_tracking_v2_flythrough.yaml`；窄提示使用独立 `cable_tracking_v2_flythrough_narrow_cue.yaml`；无干扰对照省去两个 secondary 参数。绘图命令为 `python3 tools/plot_v2_multisource_identity.py`。[逐例摘要](publication/fwhm_v2_experiment_results/data/28_multisource_identity_gate_summary.json)含 CSV 与原始摘要 SHA-256，[发布清单](publication/fwhm_v2_experiment_results/data/28_multisource_identity_gate_manifest.json)固定脚本和图像。逐时 CSV 位于本地 `results/v2/inspection_multisource_identity_20260927/`，由上述命令再生。
