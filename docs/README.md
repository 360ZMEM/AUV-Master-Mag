## 🔧 快速命令参考

```bash
# 列出所有场景
python main_demo.py --list-cases

# 运行默认场景（带可视化）
python main_demo.py --case case1

# 无图模式（快速验证）
python main_demo.py --case case1 --no-viz

# 部署模式（无名义路由先验）
python main_demo.py --case case1 --deployment-mode

# 覆盖传感器模式
python main_demo.py --case case1 --magnetometer-mode high-fidelity

# 覆盖声纳模式
python main_demo.py --case case1 --sonar-mode off

# 运行测试
python -m unittest discover -s tests
```

---

## 🎯 场景选择指南

| 目标 | 推荐场景 | 理由 |
|------|----------|------|
| 快速验证 | case1 | 直缆基线、最稳定（航向误差 2.32°） |
| 转弯测试 | case2 | 折线 spline 转弯 |
| 噪声鲁棒性 | case3, case_hf_phone | 手机级大噪声、真实传感器 |
| 坐标验证 | case4 | 强姿态扰动（±18° roll） |
| 缓弯 / 低频兼容 | case5 | 正弦缓弯 + 15Hz demo（航向误差 9.85°） |
| 强弯融合压力 | case6 | 连续 S 弯（已知未收口，27.58°） |
| 迷宫有声呐 | case_maze_sonar | 1× serpentine，Phase D 区域重捕获，当前 endpoint |
| 迷宫声呐失效 | case_maze_sonar_dropout | 初期有声呐，TRACK 后强制离线，用于验证是否依赖持续声呐 |
| 迷宫稀疏声呐 | case_maze_sparse_sonar | 低频声呐锚点 + 较长 local path 记忆，当前 endpoint |
| 紧曲率边界 | case8 | minR≈50m，曲率约束与安全锁恢复 |
| 高性能验证 | case_hf_industrial | 高采样率（1000Hz）、低噪声 |

> 完整场景详述与实测指标见 [场景配置指南](06_场景配置指南.md) §3；跨 case 指标矩阵见 [实施进度与成果](09_实施进度与成果.md) §5。
