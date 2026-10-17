# 有限巡检任务推进与撤权：独立轨迹审计

这份审计把“任务能到端”和“异常后命令归零”分开核查。它重读图 61 的八趟**同版合成** ROS2/DDS—PID—自治守卫—UDP—原生 PVS 轨迹，没有重新仿真、调参或覆盖旧结果。审计器及其限值固定在[源码](../../../tools/audit_v2_inspection_task_liveness.py)；逐趟 CSV/摘要哈希、指标和判断保存在[机器可读审计](../../results/v2/inspection_task_liveness_audit_20260928/audit.json)。

![图 61：稳态和阶跃横流下的巡检路线与撤权机制](publication/fwhm_v2_experiment_results/figures/61_finite_track_current_envelope_and_clock.png)

| 合成条件 | 软件有限路线到端 | 估计线最大横偏 | 异常处理 |
| --- | ---: | ---: | --- |
| 稳态 +0.05 m/s 横流，磁噪声种子 101–103 | 3/3 | 0.742–0.743 m | 无 Hold |
| 稳态 −0.05 m/s 横流，磁噪声种子 101–103 | 3/3 | 1.233–1.234 m | 无 Hold |
| Track 后阶跃 +0.05 m/s，种子 101 | 1/1 | 0.960 m | 无 Hold |
| Track 后阶跃 −0.05 m/s，种子 101 | **0/1** | **2.009 m** | 883.5 s 撤权；后续 241 个周期目标速度、PID 推力、UDP 主推均零；30 s 后请求恢复 |

审计要求 CSV 采样时间严格递增、最大间隔不超过 0.5 s、样本数与运行摘要相符；完整任务必须经历 Confirm 和 Track，沿本次向东的 80 m 巡检段实际前进至少 78 m（2 m 终点容差），最后一个且唯一的到端标志与零命令一致、相对冻结估计线的横偏不超过配置的 2 m。若发生短暂 Hold，只有其全过程三层命令均零、单次不超过 30 s、随后仍真实到端才算可恢复完成；这一条为后续目标平台试验留下可执行定义，本组七条完成轨迹均**没有** Hold。撤权样本必须在记录末尾保持 Hold、无到端标志、三层命令一直为零且出现恢复请求。两种判断相互独立，不能把撤权说成完成。

本组结果是 **7 条软件路线到端、1 条软件撤权轨迹符合零命令检查**，不是“7/8 现场巡检成功率”。七条到端轨迹的沿缆 $x$ 净推进为 79.88–79.99 m，反向阶跃未到端轨迹仅推进约 9.23 m。八条轨迹只改变磁噪声种子和两个横流时间型，声呐目标、导航地速与动力学仍来自同源/理想夹具。反向阶跃样本从首次 Hold 到最后一帧 PVS 仍位移约 5.67 m，原实验从故障时刻定义的 60 s 位移为 **5.683 m**；末地速约 0.052 m/s。可见零推进和请求上级处置均不等于艇已停稳。[图 66 的独立接收时序负证据](INSPECTION_SAME_SESSION_ROS_RECEIPT_DIAG_20260928.md)也不能与本组历史长任务拼成“同次连续运行通过”：本审计的 CSV 没有独立 DDS 到达、候选来源年龄、PC104 收帧或实艇运动。

当前可向答辩表述为：**受限已知目标剖面 F 的合成全链已验证有限路线推进；反向突变流时软件能撤权并持续归零，但物理滑行和目标平台输出连续性尚未验收。** 埋深保持空值，目标身份仍需独立物理来源。现场自治许可维持 `false`。

从仓库根目录复核：

```bash
args=()
for dir in AUV-Master-Mag/results/v2/inspection_route_budget_clockfix_20260927/{steady_plus_101,steady_plus_102,steady_plus_103,steady_minus_101,steady_minus_102,steady_minus_103,step_plus_101,step_minus_coast_101}; do
  args+=(--run "$dir")
done
python3 tools/audit_v2_inspection_task_liveness.py "${args[@]}" \
  --output-json AUV-Master-Mag/results/v2/inspection_task_liveness_audit_20260928/audit.json
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q tests/test_v2_inspection_task_liveness.py
```

返回码 0 仅表示这些合成轨迹结构可审阅，**不代表**现场许可。下一次有价值的试验是目标平台同一次完整 F 任务的[时序包合同](INSPECTION_SAME_SESSION_TIMING_ENVELOPE_20260928.md)与[目标/执行两包预检](INSPECTION_FIELD_PACKAGE_PREFLIGHT_20260927.md)：任务事件、独立 ROS/PID 接收、真实 UDP/PC104 和运动必须共享可核验时间轴，并记录正常到端与输入故障后的全过程。没有这些原始包，本审计不会推断“持续 10 Hz”“实艇停住”或物理目标巡检完成。
