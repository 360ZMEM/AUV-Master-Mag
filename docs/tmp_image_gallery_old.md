# 图片样例总览

> 生成命令：`python tools/visualize.py --all`
> 生成时间：2026-06-27 14:17:36
> 输出目录：`results/20260627_141736/`

---

## case1（直缆基线）

### overview
![case1_overview](../results/20260627_141736/case1/figures/case1_overview.png)

### detail
![case1_detail](../results/20260627_141736/case1/figures/case1_detail.png)

### selector_sync
![case1_selector_sync](../results/20260627_141736/case1/figures/case1_selector_sync.png)

---

## case2（折线转弯）

### overview
![case2_overview](../results/20260627_141736/case2/figures/case2_overview.png)

### detail
![case2_detail](../results/20260627_141736/case2/figures/case2_detail.png)

### selector_sync
![case2_selector_sync](../results/20260627_141736/case2/figures/case2_selector_sync.png)

---

## case3（噪声鲁棒性）

### overview
![case3_overview](../results/20260627_141736/case3/figures/case3_overview.png)

### detail
![case3_detail](../results/20260627_141736/case3/figures/case3_detail.png)

### selector_sync
![case3_selector_sync](../results/20260627_141736/case3/figures/case3_selector_sync.png)

---

## case4（强姿态扰动）

### overview
![case4_overview](../results/20260627_141736/case4/figures/case4_overview.png)

### detail
![case4_detail](../results/20260627_141736/case4/figures/case4_detail.png)

### selector_sync
![case4_selector_sync](../results/20260627_141736/case4/figures/case4_selector_sync.png)

---

## case5（正弦缓弯）

### overview
![case5_overview](../results/20260627_141736/case5/figures/case5_overview.png)

### detail
![case5_detail](../results/20260627_141736/case5/figures/case5_detail.png)

### selector_sync
![case5_selector_sync](../results/20260627_141736/case5/figures/case5_selector_sync.png)

---

## case6（连续 S 弯）

### overview
![case6_overview](../results/20260627_141736/case6/figures/case6_overview.png)

### detail
![case6_detail](../results/20260627_141736/case6/figures/case6_detail.png)

### selector_sync
![case6_selector_sync](../results/20260627_141736/case6/figures/case6_selector_sync.png)

---

## showcase（跨 case 汇总）

![showcase](../results/20260627_141736/showcase.png)

---

## 图片类型说明

| 图片 | 面板数 | 用途 |
| --- | --- | --- |
| `overview.png` | 4 面板 | 轨迹俯视图 + 航向误差 + FSM 状态 + 关键指标 |
| `detail.png` | 9 面板 | 诊断仪表盘：SNR、置信度、拟合残差、横偏、磁峰事件、导航源等 |
| `selector_sync.png` | 多面板 | shadow 候选选择器与 oracle 一致性时间序列 |
| `showcase.png` | 跨 case | case1-6 横向对比 |
