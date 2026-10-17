# v2 Cases A–D 首轮基线

命令：

```bash
PYTHONPATH=src python3 tools/run_v2_experiments.py --seeds 20
```

该实验是可重复的轻量数学仿真，每个水平 20 个随机种子。它用于验证模块方向和建立回归基线，不能替代 PVS/HoloOcean 或海试。

| Case | 运行数 | 首轮结果 |
|---|---:|---|
| A 大初始误差 Search | 100 | detection probability 1.0；平均检测时间 187.19 s；平均检测前航程 131.03 m |
| B 不同埋深 | 100 | FWHM valid rate 1.0；埋深 MAE 0.0134 m；q90 0.0299 m |
| C 不同噪声 | 120 | valid rate 1.0；总体 MAE 0.0486 m；1.0 nT 档 MAE 0.1433 m |
| D 不同航线间距 | 100 | detection probability 1.0；2 m 间距平均航程 811.73 m，8 m 间距 214.15 m |
| 无目标对照 | 20 | false alarm probability 0.0 |
| 简化 Track | 80 | 平均 tail RMSE 0.0359 m；q90 RMSE 0.0400 m |

Case D 当前目标偏移和检测带宽下，12 m 间距仍全部检出；这不能证明 12 m 普遍安全。下一轮应扩展目标横向偏移、检测半径、噪声和航迹误差的全因子组合，寻找检测率开始下降的边界。

Case C 在 1.0 nT 噪声下仍被判为有效，但误差显著扩大，说明现有 validity 更偏向“交点完整”而非“误差保证”。后续需要用 confidence calibration 或更严格的重复横切一致性门控压低高噪声置信度。

完整分组统计和逐次运行记录位于 `results/v2/cases_a_d/`。
