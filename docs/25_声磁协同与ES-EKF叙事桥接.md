# 声磁协同与 ES-EKF 叙事桥接

> **定位**：解决合龙的**头号矛盾**——既有论文 [paper/03](file:///Users/auv_user/coding/AUV-Master-Mag/thesis/paper/03_state_estimation.md) 全章以 **ES-EKF 全状态滤波器**叙述声磁协同状态估计；而本仓库 [docs/22 §1.1](file:///Users/auv_user/coding/AUV-Master-Mag/docs/22_估计器现状与调参设计文档.md) 明确"估计器**不**作为独立 Kalman / 粒子滤波器存在，而是嵌入 orchestrator 的一组**在线修正机制**"（route-prior-correction）。本文给出统一叙事框架，供写作 paper/03、paper/04 时引用，避免把两套估计器混为一谈或互相矛盾。
>
> **约束**：本文为**新建待整合材料**，不修改任何既有 md。上游入口见 [docs/24 合龙总纲](24_声磁协同论文合龙总纲.md)。

---

## 1. 核心命题：两者不冲突，是上下游两级估计

ES-EKF 与 route-prior-correction 解决**不同层级**的问题，组合起来恰好构成完整链路：

```
┌─────────────────────────────────────────────────────────────┐
│ 载体级估计（paper/03 ES-EKF）                                 │
│   输入：IMU + DVL + 深度 + 声磁观测                            │
│   输出：AUV 位姿（位置/姿态/速度/零偏）+ 协方差 → 标量置信度    │
│   回答："AUV 自己在哪、朝向哪、有多大把握"                      │
└───────────────────────────┬─────────────────────────────────┘
                            │ navigation pose（含 DR/INS 慢漂）
                            │ 作为下游的"自定位"输入
                            ▼
┌─────────────────────────────────────────────────────────────┐
│ 电缆几何级估计（本仓库 route-prior-correction）                │
│   输入：nominal route prior + 声呐/局部路径观测 + navigation pose│
│   输出：在线修正先验平移/旋转/缩放 → 控制器消费的 route cache   │
│   附加：progress guard（几何安全约束）+ zig-zag probe（可观测性激励）│
│   回答："电缆在哪、AUV 相对电缆偏多少"                          │
└─────────────────────────────────────────────────────────────┘
```

**关键论点**：本仓库 [docs/22 §1.5](file:///Users/auv_user/coding/AUV-Master-Mag/docs/22_估计器现状与调参设计文档.md) 的 DR/INS `NavigationSimulator`（true pose 与 navigation pose 分离 + 慢漂）正是"ES-EKF 输出的 navigation pose 含残余漂移"的**仿真代理**。即本仓库不重复实现 ES-EKF，而是**假设上游已有 ES-EKF 提供带漂移的 navigation pose**，专注解决"在此输入下如何稳定跟踪电缆几何"。

---

## 2. 两级估计对照表

| 维度 | 载体级 ES-EKF（paper/03） | 电缆几何级 route-prior-correction（本仓库） |
|---|---|---|
| 状态量 | 位置 p、姿态 q、速度 v_b、IMU 零偏 b_a/b_g（≈15 维全状态） | 先验平移修正 t_x/t_y、旋转修正 r_θ（3 维修正量，docs/22 §1.3.5 EKF 模式） |
| 估计对象 | AUV 自身位姿 | 名义航线先验相对真实电缆的偏差 |
| 观测 | DVL 速度、深度、声磁特征 | 声呐/局部路径电缆点 + 观测航向 |
| 不确定性机制 | 自适应 R（NIS + 质量分数）、cov_to_conf 置信度 | EKF 协方差 `meas_std/confidence` 缩放 + 三重门限（gain 模式） |
| 安全约束 | 协方差膨胀触发行为树降级 | progress guard 限制投影窗口防跨 lane |
| 主动感知 | （无，被动融合） | zig-zag probe 主动激励磁可观测性（docs/21） |
| 代码位置 | 另一仓库 AUV-Master-Project | `src/auv_mag_tracking/perception/orchestrator.py` |

---

## 3. 写作映射：可直接套 03 章框架 vs 必须改写

写 paper/03、paper/04 时，逐节按下表处理。

| paper 章节 | 处理方式 | 说明 |
|---|---|---|
| §3.3.1 状态向量与运动学 | **不动**（载体级） | 本仓库不涉及 15 维全状态，沿用 paper 原文 |
| §3.3.2 声学观测模型 | **可深化** | 本仓库声呐 regime（sonar/sparse/dropout）+ zig-zag 主动观测，作为"如何在弱信号下激励几何观测"补充 |
| §3.3.3 磁学观测模型 | **可深化** | 本仓库 zig-zag crossing 证据采集机制（docs/21 §8），补充"主动激励磁观测"维度 |
| §3.3.4 声磁接力与动态权重 | **可深化（强对接）** | 本仓库三态接力（声呐主导/磁场主导/两者沉默）有实证 sweep，直接作为该节证据 |
| §3.4 不确定性量化 | **理念一致，标注同源** | 本仓库置信度衰减驱动动态航速，与 paper cov_to_conf 同理念；标注"两套实现、理念一致"，不声称联合验证 |
| §3.5 仿真验证 | **新增小节** | 把 DR/INS 鲁棒边界 sweep 作为"电缆几何级跟踪鲁棒性"新增证据；**必须标 n=1 或补多 seed** |
| §4.3.1 之字形扫描 | **填占位符** | paper/04 §4.3.1 含"表格占位符：扫描间距计算公式 + 覆盖率表"，本仓库 `d=0.8·min(2R_sonar,2R_mag)`（docs/21）可填 |
| §4.3.2 动态航速 | **可深化** | 本仓库置信度动态航速实现可作为该节工程实例 |
| §4.5 控制鲁棒性 | **新增证据** | progress guard A/B + 失效边界作为"电缆跟踪控制鲁棒性边界"证据 |

### 3.1 必须改写的红线（防混淆）

1. **不要把 route-prior-correction 写成 15 维 ES-EKF**。它是 3 维修正量滤波（t_x, t_y, r_θ），叙述时明确"这是在 ES-EKF 输出之上的电缆几何级修正层"。
2. **不要把 DR/INS NavigationSimulator 写成真机惯导**。它是仿真代理，叙述时标注"模拟 ES-EKF 残余漂移"。
3. **不要把仿真 sweep 写成 ES-EKF 声磁融合实测**。本仓库证据是"给定带漂移自定位下的电缆跟踪鲁棒性"，不是端到端声磁融合定位精度。

---

## 4. paper/04 §4.3 接入点（扫描间距占位符）

[paper/04 §4.3.1](file:///Users/auv_user/coding/AUV-Master-Mag/thesis/paper/04_decision_and_control.md) 已有原则性叙述（`d = 0.8 × min(2·R_sonar, 2·R_mag)`，20% 重叠裕度）并留"表格占位符"。本仓库 zig-zag 探针补充的是**主动感知激励维度**：

> 之字形不只是被动覆盖扫描，更是受控主动感知激励——通过有限状态机（ProbeBurstManager）在 TRACK 健康时安排短时 burst，临时增大横向激励采集磁 crossing 证据，再进入 route recovery，从而在弱磁信号下提升可观测性而不破坏任务推进。

可粘贴方法段落见 §5。

---

## 5. 可粘贴方法段落（引自 docs/21 §12）

> 以下段落出自 [docs/21 §12](file:///Users/auv_user/coding/AUV-Master-Mag/docs/21_zigzag论文写作自包含说明.md)，可直接作为 paper/03 §3.3 或 paper/04 §4.3 的方法描述草稿。

**英文**：

```text
To improve magnetic observability during cable tracking, we introduce a
bounded zig-zag probing mechanism. Instead of continuously increasing the
tracking oscillation amplitude, the controller schedules short probe bursts
through a finite-state manager. The manager remains idle during nominal
tracking and only enters a burst after sufficient healthy TRACK duration,
forward route progress, and an entry cross-track bound are satisfied. During
the burst, lateral excitation is temporarily increased to collect magnetic
crossing evidence. The controller then enters a recovery phase that prioritizes
route progress before cooling down.
```

**中文**：

```text
为提高磁观测可见性，系统引入受限 zig-zag 探测机制。控制器不是持续增大横向振荡幅度，而是通过有限状态机安排短时探测脉冲。状态机在正常跟踪时保持 idle，只有当 TRACK 健康时间、前向 route 进度和进入横偏门限同时满足时才进入 burst。burst 阶段临时增加横向激励以采集磁 crossing 证据，随后进入 route recovery 阶段恢复任务推进，并在 cooldown 后才允许下一次探测。
```

> 局限性（docs/21 §13）：80m 为代表点门限、不应声称适用所有海缆几何；dropout endpoint 依赖 bounded safe-window，不能简化为"打开 zig-zag 即成功"。写作时必须保留这些边界声明。

---

## 6. 一句话总纲

> ES-EKF 回答"AUV 在哪"，route-prior-correction 回答"电缆在哪、AUV 相对电缆偏多少"；zig-zag 是可观测性激励层，progress guard 是几何安全约束层。四者构成"载体级 → 电缆几何级 + 主动激励 + 安全约束"的完整声磁协同链路，合龙时按此分层叙述即可消除矛盾。
