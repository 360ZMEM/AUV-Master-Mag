# 独立目标锚点：防止声呐与磁场共同锁定错误线

图 55 的新授权顺序允许偏移地图只决定扫描方向和覆盖，再由运行时声呐核对磁中心；但它留下一个明确的可辨识缺口：声呐线质量概率 $p_{\mathrm{track}}$ 不代表“属于指定目标缆”的概率。如果声呐和磁测共同指向同一条错误融合线，二者的一致性仍会放行。图 56 又说明单靠磁峰外形无法普遍弥补目标身份。本补项采用**局部独立目标锚点 → 声磁几何一致 → Track 连续性**的分层许可，避免将宽/偏移搜索地图重新当作线位真值。

![图 57：共同错线的误完成、独立锚点确认拒绝与真目标保留](publication/fwhm_v2_experiment_results/figures/57_runtime_target_anchor_joint_wrong.png)

## 同条件因果对照

固定目标 $y=6$ m，$y=9$ m 的半幅合成邻源、偏移搜索提示 `[6.6,7.5]` m、零流与种子 101；错误声呐夹具故意指向融合磁中心 $y=7.026$ m，且保持新鲜、已“校准”线质量。三次分别独立运行 ROS DDS、巡检 PID、自治守卫、本地 UDP 和原生 PVS。锚点是另一份**合成**目标真值区间 `[5.8,6.2]` m，只在 Confirm 核对声呐线及其有界不确定区间；之后仍由现有 Track 线索连续门撤权。它不是从同一次磁拟合或声呐线质量反推出来的。

| 条件 | Confirm／终态 | 完整正向 Track 的目标真值 MAE／最大 | 埋深 |
|---|---|---:|---|
| 偏移地图＋声磁共同错线，**无锚点** | 磁中心 7.026 m 获许可，1105.00 s **误报完成**；在线贴峰 MAE 0.721 m | **1.090／2.309 m** | 空 |
| 相同错线，**有独立锚点** | 577.00 s `runtime_target_anchor_inconsistent`，未进入 Track；末周期目标速度、PID 推力、UDP 主推均为零 | — | 空 |
| 真实单源＋正确声呐，**同一锚点** | 磁中心 6.014 m，1105.25 s 完成有限 80 m Track | **0.754／1.346 m** | 空 |

因此锚点改变的是**目标关联许可**，而不是磁剖面几何、FWHM 埋深或控制调参。图 57(a) 的 `[6.6,7.5]` m 仍是偏移搜索地图；`[5.8,6.2]` m 是另一个需单独举证的局部目标锚点。其区间与错误声呐的两倍标准差带不相交时才拒绝；相近并行线或错误锚点仍可能共同误认。图 57(b) 的黑色 `×` 表示确认拒止位置，不能把末周期零命令说成原地停船。

形式化地，若磁中心为 $m$、声呐线中心及两倍标准差半宽为 $(s,h_s)$、独立目标锚点为 $I_a=[a_-,a_+]$，Confirm 同时要求 $|m-s|\le h_s$ 与 $[s-h_s,s+h_s]\cap I_a\ne\varnothing$。错误组在确认站的声呐区间约为 `[6.630,7.422]` m，与锚点 `[5.8,6.2]` m 留有约 **0.430 m** 间隔；正确组的声呐中心 6.000 m，区间与锚点相交。这是一个可解释的有限判据，锚点误差和最近邻线的最小间隔必须由物理目标包另行量测，不能从本合成间隔推定现场可分性。

## 代码与非仿真入口

`FlythroughConfig` 新增两个默认空值的 `runtime_target_anchor_min_m/max_m`，只有显式启用运行时身份线索且两端有限、递增时可用。Confirm 先执行磁中心与声呐区间的原有一致性门，再检验声呐两倍标准差区间是否与独立锚点相交；冲突输出 `runtime_target_anchor_inconsistent` 并不进入 Track。历史无锚点配置、图 55、原稳定性实验保持各自冻结结果。

在**非仿真**浅角度任务中，运行时声呐除了原有物理质量标定文件，还必须提供[独立目标锚点清单模板](runtime_target_anchor_manifest_template.json)。节点启动、首次接令分别核对清单及原始来源文件 SHA-256，清单绑定完整任务配置、目标/任务 ID、路线与局部 NED、Confirm 所在沿程、审查人和失效时间；任务消息的 ID、目标深度与速度也须匹配被审查的配置。运行时审查到期会锁存 Hold/处置请求。仿真才允许直接在 `v2.flythrough` 写入两个数值以构造因果夹具；非仿真把它们写入 YAML 而无独立清单会拒绝。

清单的 `source_kind` 可为 `survey_tie_point` 或 `confirmed_handoff`。应以**与本次声呐拟合及磁峰提取独立**的现场测量，在飞越确认站附近给出目标横向区间、沿程覆盖、坐标变换及真实邻线情况。候选软件门要求区间宽度不超过 1 m，但这**不是**现场精度验收阈值；现场还要把锚点误差、声呐区间、磁中心偏差和邻缆间隔放入同一可分辨性预算。示例清单默认未批准，本仓无物理锚点或真实声呐目标关联包。SHA、`evidence_domain=physical` 文本及审查名字均可被人工误填，程序无法由此证明测绘真伪、操作者权限或声呐实际锁定哪条缆。物理现场许可仍为 `false`。

**版本边界**：本页图 57 与三条全链结果是仅有锚点相交门时的冻结快照。后续[图 58 的近邻反例与目标专属走廊](INSPECTION_TARGET_EXCLUSIVE_CORRIDOR_20260927.md)表明，相交不足以排除很近的邻线；[图 59 的 Track 连续性](INSPECTION_TRACK_IDENTITY_ROUTE_CORRIDOR_20260927.md)又要求专属走廊覆盖整段有限巡检路线。当前非仿真模板为 schema 3，Confirm 和 Track 均要求声呐两倍标准差区间完整落入独立调查证明的目标专属走廊。下文复现命令仍复现旧快照，不应把新门冒充为图 57 的执行结果。

准备任务包时，先冻结完整任务 YAML，再由 `runtime_anchor_mission_sha256(mission['cable_tracking'])` 计算配置摘要，由 `prior_points_sha256(mission['cable_tracking']['prior'])` 计算路线摘要；原始独立测绘文件另外计算 SHA-256。把这些值、目标/任务 ID、确认站沿程覆盖、横向区间和有效期填入模板后，最后计算**清单本身**的 SHA-256，并在 YAML 的 `cable_tracking.v2.runtime_target_anchor.evidence_file/evidence_sha256` 中引用。清单自引用字段不进入配置摘要，其余任务参数均进入；配置、测绘来源或审查期限变化必须重新审查。运行时声呐任务消息须显式提供与清单一致的 `mission_id`、`target_id`、`target_depth` 和 `target_speed_mps`。这一准备顺序防止错任务文件被意外复用，但不替代独立现场审查。

无声呐时不能靠这条门启航；应遵守[图 56 的独立测绘及邻源排除条件](INSPECTION_MAGNETIC_ISOLATION_LIMIT_20260927.md)。缺锚点、锚点失效或声磁冲突时，只保留未具名磁异常/诊断与有限 Hold，不发布指定目标完成。真实连续运作仍需要经实艇测得的制动与上级接管包络；本试验仅证明软件拒止出口。

## 复现与证据

从仓库根目录运行 `python3 tools/audit_v2_runtime_target_anchor.py` 可重算[图 57 JSON](publication/fwhm_v2_experiment_results/data/57_runtime_target_anchor_joint_wrong.json)中的真值误差、末周期命令、逐例 CSV/摘要及源码/配置/图像哈希。最终三次运行均先 source ROS Humble 和 `brain_linux/install`，设置 `AUV_V2_HARNESS_INSTALLED_NODE=1`、`ROS_LOCALHOST_ONLY=1` 及各自独立的 `ROS_DOMAIN_ID`；逐例摘要确认实际导入的是安装空间的 v2 ROS 节点，安装文件与源码 SHA-256 相同。合成运行由 `tools/smoke_v2_flythrough_ros_pvs.py` 完成：无锚点错误声呐用图 55 的声呐主导偏移地图配置、`--sonar-identity-mode fixed_line --sonar-line-lateral-m 7.026 --secondary-cable-lateral-m 9 --secondary-cable-gain-nt 10`；有锚点两例用本次结果目录中的 `sonar_with_independent_anchor_sim.yaml`，分别传相同错误声呐或 `--sonar-identity-mode target` 单源；三例均加 `--with-bridge --case wire --seed 101 --cable-lateral-m 6`。最终结果分别位于忽略的 `AUV-Master-Mag/results/v2/inspection_joint_wrong_target_20260927/final_*/`，不覆盖旧实验。

磁算法测试 15/15，ROS v2 包测试 74/74（含新增锚点清单 8 项）通过。数字和图像只是一种固定合成工况下的条件化反例与修复，不是物理声呐、独立测绘、PC104 或实艇成功率。
