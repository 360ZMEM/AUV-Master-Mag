# 局部目标专属走廊：独立锚点的近邻可分辨性预算

图 57 的独立目标锚点能挡住离得较远的共同错线，但原来的“声呐区间与锚点相交”仍会放行近邻。设目标锚点 $I_a=[5.8,6.2]$ m，真实目标 $y=6.0$ m，邻线 $y=6.6$ m，声呐两倍标准差半宽 $h_s=0.5$ m。邻线的声呐区间 $[6.1,7.1]$ m 与锚点在 $[6.1,6.2]$ m 相交；若磁峰也指向邻线，原 Confirm 两门都通过。这不是目标关联的充分证据。

![图 58：近邻声呐区间与独立目标专属走廊的判别预算](publication/fwhm_v2_experiment_results/figures/58_target_exclusive_corridor_budget.png)

现在为可选的运行时锚点增加**目标专属横向走廊** $I_e=[e_-,e_+]$。它必须来自与本次声呐拟合和磁峰提取独立的短段调查，且物理调查需证明在所覆盖的 Confirm 沿程内，走廊中没有另一条可能被识别为目标的缆。软件只核对清单来源和几何；不能凭清单文字证明“没有邻线”。Confirm 的目标身份许可现为

$$
|m-s|\le h_s,\qquad [s-h_s,s+h_s]\cap I_a\ne\varnothing,
\qquad [s-h_s,s+h_s]\subseteq I_e .
$$

新条件把**声呐不确定区间整体**放入经调查的专属走廊，而非只让中心点落入。示例 $I_e=[5.45,6.55]$ m：目标声呐 $6.0\pm0.5$ m 的区间 `[5.5,6.5]` m 在内；邻线声呐 $6.6\pm0.5$ m 的区间 `[6.1,7.1]` m 尽管与锚点相交，却越出走廊，Confirm 输出 `runtime_target_exclusive_corridor_inconsistent`，零目标速度，不进入 Track。

## 同版受控结果

三条无噪单源 v2 Confirm 扫描均用相同飞越几何、侧向地图 `[6.6,7.5]` m 和半宽 0.5 m；只改磁源／声呐线位置及是否提供专属走廊。它们是**磁算法层的合成扫描**，不是图 57 的 ROS/UDP/PVS 全链重跑。

| 场景 | 磁中心 | Confirm 后状态 | 埋深 |
| --- | ---: | --- | --- |
| 邻线 6.6 m，只有锚点 | 6.600 m | 错误进入 Track | 空 |
| 同一邻线，有目标专属走廊 | 6.600 m | Hold；原因 `runtime_target_exclusive_corridor_inconsistent`，速度 0 | 空 |
| 真实目标 6.0 m，同一走廊 | 6.000 m | 进入 Track | 空 |

图 58(b) 将声呐中心 5.5–7.2 m 和半宽 0.1–0.5 m 离散成 13,851 个**几何网格单元**。其中旧相交门放行 7,741 格，新专属走廊门放行 3,833 格；在声呐中心处于示例专属走廊右侧、旧门仍放行的 225 格中，新门放行 0 格。网格格数不是随机试验、概率或现场检出率。中心若落在走廊内而真实调查漏掉邻线，软件仍可能误放；声呐偏差超过其报告区间时，包含判据也不再有保证。

## 安装版 ROS 全链复核

另在最终源码上重建 v2 ROS 包，用三个独立 `ROS_DOMAIN_ID` 和安装空间节点执行 ROS DDS、巡检 PID、自治守卫、本地 UDP 及原生 PVS。任务指定目标始终位于 $y=6$ m；两条负例使目标磁幅为 0、只让 $y=6.6$ m 邻线产生 20 nT 磁异常，合成声呐亦指向 6.6 m。旧、新负例的任务 YAML 除专属走廊两个字段外逐项相同。正例使目标在 6 m 产生 20 nT 磁异常、无邻源，声呐指向目标。

| 同版全链条件 | Confirm 声呐线及 2σ 半宽 | 任务终态 | 指定目标真值 Track MAE／最大 |
| --- | --- | --- | ---: |
| 近邻、只有锚点 | $6.600\pm0.410$ m | 1105.00 s **错误完成指定目标** | 0.871／1.913 m |
| 同一近邻、加入专属走廊 | $6.600\pm0.410$ m | 577.00 s Confirm 拒绝，未进入 Track；末周期目标速度、PID 推力、UDP 主推均为零 | — |
| 真目标、同一专属走廊 | $6.000\pm0.430$ m | 1105.25 s 完成有限 80 m Track | 0.754／1.346 m |

三例均未发布埋深。旧负例的目标真值 MAE 仍小于示例 1 m 门，而目标磁源本身已消失，说明**仅凭轨迹贴近目标线也无法证明跟的是指定缆**。新门改变的是 Confirm 的身份许可；在错误负例，磁中心仍为 6.606 m，拒绝原因是 `runtime_target_exclusive_corridor_inconsistent`。它没有证明实艇停住，只有零命令。安装节点路径、源码与安装哈希、逐例 CSV／摘要／日志、两份配置差异及终态核验见[全链审计 JSON](publication/fwhm_v2_experiment_results/data/58_target_exclusive_corridor_ros_pvs_audit.json)；可运行 `python3 tools/audit_v2_target_exclusive_corridor_ros.py` 复核。声呐、磁源与专属走廊仍是合成夹具，三组单种子不是成功概率。

## 现场输入合同与兼容性

[锚点清单模板](runtime_target_anchor_manifest_template.json)在图 58 阶段曾升级为 schema 2，新增 `exclusive_lateral_min_m/max_m`。该阶段只证明 Confirm 的局部目标关联，图 58 及本页三条全链结果保留其源码哈希，不能冒充后续 Track 门。当前[图 59 的连续性补项](INSPECTION_TRACK_IDENTITY_ROUTE_CORRIDOR_20260927.md)把非仿真模板升级为 **schema 3**：再加专属走廊的沿程覆盖，并在 Track 持续核对声呐区间。来源文件、路线、整份任务配置、任务/目标 ID、审查人与时效仍固定；旧 schema 1/2 或在任务 YAML 中直接写走廊数值均拒绝。仿真可以显式提供区间做反例；没有启用新走廊的旧仿真配置、图 57 冻结结果和原 v1 实验不会改写。

选择真实 $I_e$ 时必须单独记录目标线位误差、声呐 2σ 区间的物理覆盖率、坐标／外参误差、最近邻缆或平行设施的排除范围及 Confirm 沿程。尤其要核对目标声呐区间有足够余量完整落入 $I_e$，否则正确目标也会被拒；这是一项可计算的任务可行性条件，不是通过调宽清单即可自证的许可。清单哈希与 `evidence_domain=physical` 仍无法证明调查真实或完整，物理输入、PC104 和实艇验收缺失，现场自治许可保持 `false`。

从仓库根目录运行 `python3 tools/audit_v2_target_exclusive_corridor.py` 可重做三条算法扫描、图 58 与[几何 JSON](publication/fwhm_v2_experiment_results/data/58_target_exclusive_corridor_budget.json)。源码、脚本与图文件 SHA-256 均在 JSON 中。该阶段磁算法 16 项、ROS v2 包 77 项测试通过；图 58 是算法与几何解释，独立全链审计补足当时的 ROS/PID/UDP/PVS 执行证据。图 57 保留更早的旧门快照，当前 Track 门证据另见图 59。
