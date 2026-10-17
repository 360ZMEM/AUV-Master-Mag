# ROS2 v2 闭环 Smoke 记录

日期：2026-09-19。

链路：

```text
auv_decision_ros/decoupled_cable_sim_node
  → /auv/state/filtered + /auv/sensors/magnetic + mission
  → auv_cable_tracking_v2
  → /auv/control/setpoint + /auv/cable/tracking
  → decoupled vehicle kinematics
```

实验把真实电缆放在 `y=12 m`，而 ROS 先验中心位于 `y=0 m`，初始 AUV 位于 `y=7.5 m`。算法配置保持 4/8/14/20 m 分档、物理单直导线 `shape_factor=0.288675`。为缩短 smoke 实时时间，仅在 `/tmp` 配置中把 Search/Confirm 速度和对应动力学上限提高到 1.2 m/s；正式配置仍为 Search 0.7 m/s、Confirm 0.45 m/s。

观测到的状态序列：

```text
SEARCH
→ CONFIRM(4 m，半高覆盖不足)
→ REACQUIRE
→ CONFIRM(8 m，半高覆盖不足)
→ REACQUIRE
→ CONFIRM(14 m，检测中心偏差导致覆盖不足)
→ REACQUIRE
→ CONFIRM(20 m)
→ TRACK
```

最终结果：

| 指标 | 估计 | 仿真真值/参考 | 误差 |
|---|---:|---:|---:|
| FWHM | 26.1055 m | 物理直导线预期约 25.98 m | 约 0.13 m |
| 垂直距离 | 7.5360 m | 7.5 m | 0.0360 m |
| 埋深 | 1.5360 m | 1.5 m | 0.0360 m |
| observation-only 电缆 y | 11.9167 m | 12.0 m | -0.0833 m |
| validity | 0.9944 | — | — |
| reason codes | 空 | — | — |

这次 smoke 同时验证了：

- 大先验横偏下 Search 能触发 Confirm；
- 复扫时检测释放不会清除原始扫描中心和背景；
- 覆盖不足的前三档不会输出伪埋深；
- 物理单直导线模型与 FWHM shape factor 一致；
- v2 输出经原 ROS 消息、setpoint、限幅和解耦动力学形成闭环。

## PVS / protocol_udp 全链路

随后在真实 PVS side-channel 与 ROS2 协议栈上运行：

```text
PVS mock AMD + midpoint-current-element magnetic source
  → protocol_udp + Zenoh magnetic side-channel
  → ROS2 bridge/localization/controller/arbiter
  → auv_cable_tracking_v2
  → Search / Confirm / FWHM / Track / endpoint hold
```

第一次使用仓库原 PVS 几何的运行成功进入 Track，FWHM 局部中心点距配置电缆仅 0.016 m，但也暴露出 Track 把局部观测直线无限外推、越过先验终点继续航行的问题。v2 随后改为把横切得到的独立横向修正投到有限先验走廊，并在终点输出零速保持；对应回归测试覆盖“越过终点后航向返回”和“到达终点后零速”。

几何受控复跑把 AUV 名义深度、电缆深度和海底深度分别设为 8.0、15.5 和 15.0 m。主动横切增加“到达首侧后先对准反向航向再采样”，并对沿缆漂移设置 5 m 硬门限。观测结果为：

| 指标 | 结果 |
|---|---:|
| 状态链 | Search → Confirm/扩幅 → Track → endpoint hold |
| FWHM | 19.2790 m |
| 横切采样数 | 176 |
| 横切沿缆范围 | 3.5476 m（通过 5 m 门限） |
| profile residual | 0.0435 |
| validity / profile confidence | 0.9739 / 0.9546 |
| 终点保持 | true |

该运行也给出一条不能省略的负证据：从静态场剖面得到的 `shape_factor=0.44115` 在动态 PVS 横切上估计垂直距离 8.505 m，而名义几何为 7.5 m，误差 1.005 m；减去 7 m 高度后，0.5 m 真值埋深被估为 1.505 m。原因是 PVS 当前磁源采用线段中点电流元离散，系数随局部几何与动态扫描轨迹变化。v2 因此把 `shape_factor_relative_sigma` 与 `altitude_sigma_m` 纳入埋深不确定度，并将 FWHM 结论表述为“幅值尺度无关、场形模型有条件”，不再把高 profile confidence 等同于高埋深准确度。

证据文件位于 `results/v2/pvs_closed_loop/` 与 `results/v2/pvs_geometry_controlled/`。前者证明接口和局部 XY 反演，后者证明有限走廊、轨迹质量门和终点保持，同时保留未经事后美化的深度模型偏差。

又执行了一次“用上一轮动态剖面反算系数、单档放大到 24 m”的校准检查。该次运行采集 343 点，但沿缆范围达到 14.189 m，超过 5 m 硬门限，且缺少左侧半高交点。状态机按设计返回 `REACQUIRE`，没有使用校准系数发布深度。这个结果说明增大摆幅本身不能保证更好的反演：当转向动力学使空间轨迹偏离局部法向时，更大的扫描会把沿缆变化混入 `B(x)`。证据保存在 `results/v2/pvs_geometry_calibrated/`，其性质是轨迹质量门的负向验收，不是系数精度验证。

## 证据边界

- 解耦闭环证明理想物理单直导线条件下的 FWHM 数值正确性；
- PVS 闭环证明 protocol_udp/ROS2/控制链可运行，并发现及修复了局部直线无限外推问题；
- PVS 单次几何受控运行不是多种子精度统计，当前 midpoint 模型系数不能外推到真实海缆；
- 24 m 单档扫描因沿缆漂移和单侧交点缺失被拒，支持保留 4/8/14/20 m 逐级扩幅，而非无条件增大摆幅；
- HoloOcean 当前环境未安装，实测 TMR 回放尚未接入 v2 全状态链，二者仍是后续验证项。
