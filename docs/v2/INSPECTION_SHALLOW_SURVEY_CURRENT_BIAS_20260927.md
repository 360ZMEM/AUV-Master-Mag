# 多横切横流报告偏差：同版合成故障注入

**结论与边界。** 多横切 ROS 配方此前把输入给规划器的横流值设为 PVS 真横流。图 53 在同一源码、同一种子 101、同一真实北向横流 $0.040\,\mathrm{m/s}$ 下，只改决策节点收到的合成横流消息。低报 $0.005\,\mathrm{m/s}$ 时有限 80 m Track 仍完成；低报 $0.020\,\mathrm{m/s}$ 时接入长度耗尽、进入 Hold，未报告完成或埋深。这是**任务末端的保守拒绝**，不是软件识别了横流传感器偏差。全部目标身份消息仍来自合成真值线，未接自治守卫、物理流速计或 PC104；现场许可不变。

![同版报告横流偏差、扫描参考偏差及接入拒绝](publication/fwhm_v2_experiment_results/figures/53_shallow_survey_current_report_bias.png)

| 报告北向横流 | 相对真值偏差 | 扫描参考最大偏差 | 接入用时 | 完整 Track 真值横偏 MAE | 任务终态 |
| ---: | ---: | ---: | ---: | ---: | --- |
| $0.040$ m/s | $0$ | $0.964$ m | $196.75$ s | $0.334$ m | 完成有限 80 m Track |
| $0.035$ m/s | $-0.005$ m/s | $1.101$ m | $203.00$ s | $0.317$ m | 完成有限 80 m Track |
| $0.020$ m/s | $-0.020$ m/s | $2.130$ m | $397.50$ s | 无 Track 样本 | `survey_rejoin_length_exhausted`，Hold |

三组消息均声明 $\sigma=0.004\,\mathrm{m/s}$。$0.005\,\mathrm{m/s}$ 注入位于声明的 $2\sigma$ 区间内；$0.020\,\mathrm{m/s}$ 注入明显超出，故后者是**错误不确定度声明的故障反例**，不可放入常规精度统计。入口目前只检查报告值、时效、坐标系和 $|v_\perp|+2\sigma\leq0.05\,\mathrm{m/s}$。严重低报的消息仍满足该代数门，扫描参考偏差也始终低于当前 $3$ m 门，直到有限接入预算耗尽才停止。这说明仅靠消息自报的 $\sigma$ 不能证明真横流被充分约束。严重低报组在 Hold 后的 $61/61$ 周期目标速度、PID 推力和本地协议转速均为零，但 15 s 合成时间内 PVS 仍漂移 $2.823$ m；该探针不含实际 UDP 守卫或艇端动作。

这项负例不改变“已知目标、短直线段、预对准起点的单次浅角穿越优先做现场首验”的[剖面决定](INSPECTION_V2_FIELD_FIRST_PROFILE_20260927.md)。宽先验多横切仍是仿真研究剖面。要把它转为物理任务，须先取得独立流速来源，核查真流向和误差分布、时间戳、坐标变换与传感器失效/偏差，并在真实走廊和停机包络中重算三／四段起点及接入长度。单次固定偏差注入只能定位敏感环节，不能给出实海流的成功概率或保证阈值。场形失配的“路径 1→3”仍按[独立场形与标定契约](MODEL_MISMATCH_PATH_1_TO_3_20260927.md)处理：场形或标定证据不足时横向中心可单独审查，埋深保持空值；横流误差试验不替代该契约。

复现前先设置隔离 ROS 域、加载 Humble 和本仓安装空间，然后顺序运行：

```bash
source /opt/ros/humble/setup.bash
source brain_linux/install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=227
python3 tools/probe_v2_shallow_survey_current_bias.py --seed 101 --true-current-y-mps .04 --report-bias-mps 0 --output-dir AUV-Master-Mag/results/v2/shallow_survey_current_bias_20260927/zero_bias
python3 tools/probe_v2_shallow_survey_current_bias.py --seed 101 --true-current-y-mps .04 --report-bias-mps -.005 --output-dir AUV-Master-Mag/results/v2/shallow_survey_current_bias_20260927
python3 tools/probe_v2_shallow_survey_current_bias.py --seed 101 --true-current-y-mps .04 --report-bias-mps -.02 --output-dir AUV-Master-Mag/results/v2/shallow_survey_current_bias_20260927/underreport_020
python3 tools/plot_v2_shallow_survey_current_bias.py
```

上述三次 ROS 运行须**顺序**执行；必要时为每次运行改用不同且有效的 `ROS_DOMAIN_ID`。逐例 CSV、摘要及源码哈希在 `AUV-Master-Mag/results/v2/shallow_survey_current_bias_20260927/`；[图 53 清单](publication/fwhm_v2_experiment_results/data/53_shallow_survey_current_report_bias_manifest.json)核对原始哈希、图像哈希和每组终态。旧图 51、52 与既有试验文件未覆盖。
