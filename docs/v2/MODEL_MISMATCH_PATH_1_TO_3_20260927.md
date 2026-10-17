# 场形失配：先判别，再降级发布的执行决定

本页固定当前答辩和软件口径。历史 5600 次方法运行、图 11–12 及其 `2.753 m` 失配 MAE 保持原样。选择的第一步是用完整双侧剖面检查声明场形；只要场形不能被充分区分、扫描外翼不够、宽度—距离系数未通过独立几何留出，巡检仍可使用经质量门确认的**横向中心**，但垂距和埋深必须是空值。单次高 `confidence` 不代替上述许可。

![已知双模型的留出拒绝](publication/fwhm_v2_experiment_results/figures/13_shape_guard_held_out_validation.png)

在预定的偶极包络与单直导线双向失配留出集中，新门将两组各 200 次的大误差发布率由 100% 降为 0；正确声明直导线的 200 次保留发布。短动态横切可能让错误场形的残差更小，所以巡检模式另以等自由度背景拟合和两侧各 `2 W` 的外翼支持做保守检查，证据不够时报告 `depth_shape_ambiguous` 或 `depth_shape_support_insufficient`，而不是改选看起来残差最低的模型。

![连续场形及系数变化的反例](figures/14_cross_shape_identifiability.png)

第一步有明确失败边界：真实幂律指数 `p=0.7` 时，固定偶极门在 100 个留出种子全部发布超过 0.5 m 的误差，发布 MAE 为 `0.928 m`。限定幂律家族的连续拟合在这组解析式留出上降至 `0.034 m`，但近距双导线仍可近似单源；有限线段模型也可形状近似偶极而系数随距离变化。因此连续择模只保留为离线研究，当前 ROS 答辩配置不自动切换模型。

新增直接回归测试用同一个中间场形构造完整横切：原始固定偶极 FWHM 仍判有效且垂距误差超过 `0.5 m`，横向单峰中心通过质量门；巡检发布契约因缺独立标定返回 `depth_calibration_required`，垂距、埋深和发布置信度均置空。这把“第一步失败则走降级路径”固定为可执行的软件断言，而不只写在图注中。

第二步采用独立真值标定和保守降级：完整扫描按 `scan_id` 分离训练与留出；常系数、必要时仿射宽度映射只在已验证工作范围内使用。相同形状但留出宽度比例漂移 50% 的解析式负例，留出 MAE `1.809 m`，标定被拒绝。多模型同时可信却给出冲突距离时也拒绝深度。没有合格真实横切与同步高度的现行 ROS 配置保持 `allow_uncalibrated_depth: false`；非仿真时钟下若有人把它改为 `true`，节点现在拒绝启动。此软件门不能核实实测标定文件内容，也不能防止实艇误设仿真时钟；现场仍需独立几何真值、标定来源和适用范围审查。

为避免把调用者的 `controlled_measurement` 标签误当成物理验收，标定命令现在把留出统计结论单列为 `statistical_holdout_accepted`，始终保持 `deployment_qualified=false`。新增[逐扫描来源预检](SHAPE_CALIBRATION.md)核对标定输入、原始磁测、独立几何真值及同步离底/导航的文件哈希和会话隔离；它仍不认证文件的物理真实性，预检全过也只进入人工复核，不能直接打开现场埋深发布。

![横向继续、埋深置空的巡检过程](publication/fwhm_v2_experiment_results/figures/15_inspection_contract_process.png)

复核入口：

```bash
PYTHONPATH=AUV-Master-Mag/src python3 -m pytest -q AUV-Master-Mag/tests/v2/test_fwhm.py AUV-Master-Mag/tests/v2/test_shape_calibration.py AUV-Master-Mag/tests/v2/test_flythrough.py
PYTHONPATH=AUV-Master-Mag/src python3 -m pytest -q AUV-Master-Mag/tests/v2/test_inspection_contract.py -k intermediate_field_shape
PYTHONPATH=AUV-Master-Mag/src python3 AUV-Master-Mag/tools/run_v2_shape_guard_validation.py --seed-start 200 --seeds 200 --output-dir /tmp/v2_shape_guard_recheck
```

上述图与数字均来自合成场形及现有原生 PVS 受控试验，不是多芯铠装海缆现场精度。实物若无法取得足够完整双侧剖面或独立垂距真值，最终发布层级限定为磁异常检测和横向定位，埋深长期保持空值；PPT 与论文均应按此口径表述。
本次相关四个测试文件合计 `47/47` 通过；新增中间幂律回归属于路径选择的正反软件断言，不提升真实场形的可辨识性等级。

横向定位的发布层级仍需区别“未具名磁异常中心”和“指定目标海缆”。[窄静态地图整体偏移反例](INSPECTION_STATIC_HINT_BIAS_20260927.md)表明，0.9 m 宽的地图区间平移 1.1 m 后，单源被拒而融合邻源获准；带 ROS/PID/守卫/UDP/PVS 的单种子双源任务还会报完成。路径 1→3 消除了未经标定的高置信埋深发布，却不能从同一磁峰认证目标身份。缺测绘误差界或邻源排除证据时只保留未认证磁异常，不宣称指定目标完成。
