"""Generate the editable Draw.io method figures used by AUV-Master-Mag."""

from __future__ import annotations

from pathlib import Path
from xml.sax.saxutils import escape


OUT = Path(__file__).resolve().parent
VERSION = "30.3.14"
FONT_FAMILY = "Songti SC"

PALETTE = {
    "blue": ("#EAF2F8", "#5F8FB8"),
    "green": ("#EEF6EF", "#6E9F78"),
    "yellow": ("#FFF7E8", "#C69B42"),
    "orange": ("#FBEFE5", "#C9824D"),
    "purple": ("#F2EDF7", "#8F78AF"),
    "red": ("#F8EEEE", "#B86F73"),
    "gray": ("#F6F7F8", "#7F8E9A"),
    "teal": ("#EAF6F5", "#5D9E99"),
}

EDGE = (
    "edgeStyle=orthogonalEdgeStyle;rounded=1;orthogonalLoop=1;jettySize=auto;"
    "html=1;strokeColor=#536F87;strokeWidth=2.4;"
    f"fontFamily={FONT_FAMILY};fontSize=17;fontColor=#25384A;"
    "labelBackgroundColor=#FFFFFF;labelBorderColor=none;"
    "endArrow=block;endFill=1;endSize=10;"
)


def xml_value(text: str) -> str:
    return escape(text, {'"': "&quot;"}).replace("\n", "&#xa;")


def shape_style(kind: str, color: str, extra: str = "") -> str:
    fill, stroke = PALETTE[color]
    shapes = {
        "process": "rounded=0;",
        "module": "rounded=1;arcSize=6;",
        "io": "shape=parallelogram;perimeter=parallelogramPerimeter;fixedSize=1;",
        "decision": "rhombus;",
        "gate": "shape=trapezoid;perimeter=trapezoidPerimeter;fixedSize=1;direction=north;",
        "data": "shape=cylinder3;boundedLbl=1;backgroundOutline=1;size=12;",
        "state": "rounded=1;arcSize=18;",
        "terminal": "ellipse;",
    }
    return (
        shapes[kind]
        + "whiteSpace=wrap;html=1;align=center;verticalAlign=middle;"
        f"fillColor={fill};strokeColor={stroke};strokeWidth=2.5;spacing=8;"
        f"fontFamily={FONT_FAMILY};fontSize=19;fontColor=#26323D;"
        + extra
    )


def add_shape(
    cells: list[str],
    id_: str,
    title: str,
    detail: str,
    x: float,
    y: float,
    w: float,
    h: float,
    color: str,
    kind: str,
    parent: str = "1",
    extra: str = "",
) -> None:
    value = f"<b>{title}</b>" + (f"<br>{detail}" if detail else "")
    cells.append(
        f'<mxCell id="{id_}" value="{xml_value(value)}" '
        f'style="{shape_style(kind, color, extra)}" vertex="1" parent="{parent}">'
        f'<mxGeometry x="{x}" y="{y}" width="{w}" height="{h}" as="geometry" />'
        "</mxCell>"
    )


def add_lane(
    cells: list[str],
    id_: str,
    label: str,
    x: float,
    y: float,
    w: float,
    h: float,
    color: str,
    start_size: int = 40,
) -> None:
    fill, stroke = PALETTE[color]
    style = (
        "swimlane;whiteSpace=wrap;html=1;rounded=0;collapsible=0;childLayout=none;"
        f"startSize={start_size};fillColor={fill};strokeColor={stroke};strokeWidth=2.5;"
        f"fontFamily={FONT_FAMILY};fontSize=21;fontStyle=1;fontColor=#26323D;"
    )
    cells.append(
        f'<mxCell id="{id_}" value="{xml_value(label)}" style="{style}" '
        f'vertex="1" parent="1"><mxGeometry x="{x}" y="{y}" width="{w}" '
        f'height="{h}" as="geometry" /></mxCell>'
    )


def add_text(
    cells: list[str],
    id_: str,
    label: str,
    x: float,
    y: float,
    w: float,
    h: float,
    *,
    size: int = 18,
    bold: bool = False,
    color: str = "#34495E",
) -> None:
    style = (
        "text;html=1;strokeColor=none;fillColor=none;align=center;"
        "verticalAlign=middle;whiteSpace=wrap;"
        f"fontFamily={FONT_FAMILY};fontSize={size};fontColor={color};"
        + ("fontStyle=1;" if bold else "")
    )
    cells.append(
        f'<mxCell id="{id_}" value="{xml_value(label)}" style="{style}" '
        f'vertex="1" parent="1"><mxGeometry x="{x}" y="{y}" width="{w}" '
        f'height="{h}" as="geometry" /></mxCell>'
    )


def add_edge(
    cells: list[str],
    id_: str,
    source: str,
    target: str,
    label: str = "",
    *,
    points: list[tuple[float, float]] | None = None,
    extra: str = "",
    main: bool = False,
    dashed: bool = False,
    safe: bool = False,
) -> None:
    style = EDGE
    if main:
        style = style.replace("strokeWidth=2.4", "strokeWidth=3.2").replace(
            "fontSize=17", "fontSize=18;fontStyle=1"
        )
    if dashed:
        style += "dashed=1;dashPattern=7 5;"
    if safe:
        style = style.replace("strokeColor=#536F87", "strokeColor=#A34F55")
    if points:
        point_xml = "".join(f'<mxPoint x="{x}" y="{y}" />' for x, y in points)
        geometry = (
            '<mxGeometry relative="1" as="geometry"><Array as="points">'
            f"{point_xml}</Array></mxGeometry>"
        )
    else:
        geometry = '<mxGeometry relative="1" as="geometry" />'
    cells.append(
        f'<mxCell id="{id_}" value="{xml_value(label)}" style="{style + extra}" '
        f'edge="1" parent="1" source="{source}" target="{target}">{geometry}</mxCell>'
    )


def add_free_edge(
    cells: list[str],
    id_: str,
    source: tuple[float, float],
    target: tuple[float, float],
    *,
    points: list[tuple[float, float]] | None = None,
    color: str = "#536F87",
    width: float = 2.5,
    dashed: bool = False,
    start_arrow: str = "none",
    end_arrow: str = "none",
) -> None:
    point_xml = "".join(f'<mxPoint x="{x}" y="{y}" />' for x, y in (points or []))
    array_xml = f'<Array as="points">{point_xml}</Array>' if point_xml else ""
    style = (
        "edgeStyle=none;rounded=0;html=1;"
        f"strokeColor={color};strokeWidth={width};"
        f"startArrow={start_arrow};startFill=1;endArrow={end_arrow};endFill=1;"
    )
    if dashed:
        style += "dashed=1;dashPattern=7 5;"
    cells.append(
        f'<mxCell id="{id_}" value="" style="{style}" edge="1" parent="1">'
        '<mxGeometry relative="1" as="geometry">'
        f'<mxPoint x="{source[0]}" y="{source[1]}" as="sourcePoint" />'
        f"{array_xml}"
        f'<mxPoint x="{target[0]}" y="{target[1]}" as="targetPoint" />'
        "</mxGeometry></mxCell>"
    )


def add_marker(
    cells: list[str],
    id_: str,
    x: float,
    y: float,
    *,
    size: float = 14,
    color: str = "yellow",
) -> None:
    fill, stroke = PALETTE[color]
    style = (
        "ellipse;html=1;aspect=fixed;"
        f"fillColor={fill};strokeColor={stroke};strokeWidth=2.5;"
    )
    cells.append(
        f'<mxCell id="{id_}" value="" style="{style}" vertex="1" parent="1">'
        f'<mxGeometry x="{x - size / 2}" y="{y - size / 2}" width="{size}" '
        f'height="{size}" as="geometry" /></mxCell>'
    )


def write(name: str, cells: list[str], width: int, height: int) -> None:
    body = "\n        ".join(['<mxCell id="0" />', '<mxCell id="1" parent="0" />'] + cells)
    xml = f"""<?xml version="1.0" encoding="UTF-8"?>
<mxfile host="drawio" version="{VERSION}">
  <diagram name="{xml_value(name)}">
    <mxGraphModel dx="1200" dy="760" grid="1" gridSize="10" guides="1"
      tooltips="1" connect="1" arrows="1" fold="1" page="1" pageScale="1"
      pageWidth="{width}" pageHeight="{height}" math="0" shadow="0">
      <root>
        {body}
      </root>
    </mxGraphModel>
  </diagram>
</mxfile>
"""
    path = OUT / f"{name}.drawio"
    path.write_text(xml, encoding="utf-8")
    print(path)


def zigzag_probe() -> None:
    cells: list[str] = []
    add_free_edge(cells, "prior", (100, 185), (1300, 145), color="#A7ADB3", dashed=True)
    add_free_edge(cells, "truth", (100, 325), (1300, 285), color="#6B6F73", width=4.0)
    add_free_edge(cells, "estimate", (100, 345), (1300, 300), color="#5C9E48", width=4.0)
    zigzag = [
        (205, 150),
        (315, 455),
        (425, 145),
        (535, 450),
        (645, 140),
        (755, 445),
        (865, 135),
        (975, 440),
        (1085, 140),
        (1195, 425),
    ]
    add_free_edge(
        cells,
        "trajectory",
        (95, 300),
        (1305, 385),
        points=zigzag,
        color="#5F88C5",
        width=4.0,
    )
    marker_x = [155, 260, 370, 480, 590, 700, 810, 920, 1030, 1140, 1250]
    for i, x in enumerate(marker_x):
        y = 325 - (x - 100) * 40 / 1200
        add_marker(cells, f"peak{i}", x, y)
    add_shape(
        cells,
        "sonar",
        "声呐锚点",
        "消除 180° 方向歧义",
        1040,
        55,
        250,
        72,
        "purple",
        "io",
    )
    add_free_edge(
        cells,
        "sonar_link",
        (1165, 127),
        (1115, 300),
        points=[(1165, 220), (1115, 220)],
        color="#8F78AF",
        width=2.8,
        dashed=True,
        end_arrow="block",
    )
    add_text(cells, "t_path", "AUV 受控横切航迹", 480, 55, 360, 42, size=20, bold=True, color="#456FA8")
    add_text(cells, "t_prior", "先验走廊", 115, 125, 190, 38, size=18, color="#777777")
    add_text(cells, "t_truth", "真值电缆", 1120, 245, 170, 38, size=18, bold=True, color="#555555")
    add_text(cells, "t_est", "估计中心线（加权 PCA）", 870, 335, 360, 42, size=19, bold=True, color="#4D8B3C")
    add_text(cells, "t_peak", "穿缆峰值观测", 110, 350, 240, 42, size=18, bold=True, color="#A87000")
    add_text(cells, "t_angle", "横切角 θ", 265, 245, 180, 40, size=19, bold=True)
    add_free_edge(cells, "guide1", (645, 320), (645, 505), color="#999999", dashed=True)
    add_free_edge(cells, "guide2", (755, 315), (755, 505), color="#999999", dashed=True)
    add_free_edge(
        cells,
        "dimension",
        (645, 490),
        (755, 490),
        color="#555555",
        width=2.2,
        start_arrow="block",
        end_arrow="block",
    )
    add_text(cells, "t_distance", "扫描间距 d", 620, 495, 160, 38, size=18, bold=True)
    write("fig_zigzag_probe", cells, 1400, 560)


def perception_pipeline() -> None:
    cells: list[str] = []
    add_lane(cells, "sampling_lane", "高频磁采样与特征提取（现实现 200 Hz）", 30, 30, 1340, 155, "purple")
    add_lane(cells, "geometry_lane", "声磁几何估计", 30, 205, 1340, 245, "yellow")
    add_lane(cells, "output_lane", "置信度融合与统一输出（现实现约 20 Hz）", 30, 470, 1340, 150, "blue")
    for spec in [
        ("raw", "原始磁场块", "三轴块采样", 35, 65, 240, 68, "purple", "io", "sampling_lane"),
        ("mag_filter", "滤波与包络", "稳健去噪 / RMS", 325, 65, 240, 68, "yellow", "process", "sampling_lane"),
        ("vector", "梯度与磁矢量", "幅值 / 方向特征", 615, 65, 240, 68, "yellow", "process", "sampling_lane"),
        ("peaks", "峰值检测", "升峰 / 落峰 / 时效", 905, 65, 240, 68, "yellow", "decision", "sampling_lane"),
        ("sonar", "声呐定位观测", "稀疏绝对锚点", 30, 80, 195, 72, "purple", "io", "geometry_lane"),
        ("fit", "中心线拟合", "加权 PCA", 260, 80, 195, 72, "green", "process", "geometry_lane"),
        ("path", "局部路径几何", "中心线 / 前视", 490, 80, 210, 72, "green", "data", "geometry_lane"),
        ("mag_path", "磁路径与影子", "候选路径 / 时效", 735, 80, 210, 72, "orange", "process", "geometry_lane"),
        ("cross", "横偏与埋深", "近过线反演 / 质量", 980, 80, 210, 72, "yellow", "process", "geometry_lane"),
        ("region", "失锁重捕区", "有界候选区域", 1090, 160, 210, 62, "yellow", "gate", "geometry_lane"),
        ("confidence", "置信度融合", "质量 / 新鲜度 / 一致性", 335, 55, 260, 72, "yellow", "process", "output_lane"),
        ("state", "统一感知状态", "路径 / 横偏 / 埋深 / 置信度", 750, 50, 320, 82, "blue", "data", "output_lane"),
    ]:
        add_shape(cells, *spec)
    for i, (a, b) in enumerate(
        [("raw", "mag_filter"), ("mag_filter", "vector"), ("vector", "peaks")], 1
    ):
        add_edge(cells, f"p{i}", a, b, "", main=True, extra="exitX=1;entryX=0;")
    add_edge(cells, "p4", "peaks", "fit", "峰值序列", main=True, points=[(1250, 195), (388, 195)], extra="exitX=0.5;exitY=1;entryX=0.5;entryY=0;")
    add_edge(cells, "p5", "sonar", "fit", "方向锚定", dashed=True, extra="exitX=1;entryX=0;")
    add_edge(cells, "p6", "fit", "path", "", main=True, extra="exitX=1;entryX=0;")
    add_edge(cells, "p7", "path", "mag_path", "", main=True, extra="exitX=1;entryX=0;")
    add_edge(cells, "p8", "mag_path", "cross", "", main=True, extra="exitX=1;entryX=0;")
    add_edge(cells, "p9", "cross", "region", "失锁触发", dashed=True, extra="exitX=0.5;exitY=1;entryX=0.5;entryY=0;")
    add_edge(cells, "p10", "cross", "confidence", "", points=[(1085, 460), (465, 460)], extra="exitX=0.5;exitY=1;entryX=0.5;entryY=0;")
    add_edge(cells, "p11", "region", "confidence", "", dashed=True, points=[(1265, 460), (550, 460)], extra="exitX=0.5;exitY=1;entryX=0.8;entryY=0;")
    add_edge(cells, "p12", "confidence", "state", "", main=True, extra="exitX=1;entryX=0;")
    write("fig_perception_pipeline", cells, 1400, 650)


def architecture_layers() -> None:
    cells: list[str] = []
    lanes = [
        ("viz", "可视化与编排层", 30, 30, 1020, 115, "blue"),
        ("control", "控制与决策层", 30, 155, 1020, 115, "green"),
        ("perception", "感知与融合层", 30, 280, 1020, 125, "yellow"),
        ("physics", "环境与物理层", 30, 415, 1020, 115, "gray"),
        ("sensor", "传感器输入层", 30, 540, 1020, 115, "purple"),
    ]
    for spec in lanes:
        add_lane(cells, *spec)
    add_lane(cells, "loop", "每帧闭环（约 20 Hz）", 1080, 30, 290, 625, "teal")
    nodes = [
        ("v1", "运行编排", "场景 / 任务 / 记录", 35, 55, 270, 55, "blue", "process", "viz"),
        ("v2", "实时看板", "态势 / 诊断 / 报告", 375, 55, 270, 55, "blue", "data", "viz"),
        ("v3", "数据源驱动", "仿真 / 实物", 715, 55, 250, 55, "blue", "io", "viz"),
        ("c1", "任务模式", "搜索 / 锁定 / 跟踪 / 重捕", 35, 55, 280, 55, "green", "state", "control"),
        ("c2", "受控横切", "制导 / 横向激励", 370, 55, 260, 55, "green", "process", "control"),
        ("c3", "探测窗口", "触发 / 恢复 / 冷却", 690, 55, 270, 55, "green", "gate", "control"),
        ("p1", "磁感知编排", "块采样 / 时效", 35, 60, 210, 55, "yellow", "process", "perception"),
        ("p2", "滤波与特征", "包络 / 峰值 / 矢量", 275, 60, 220, 55, "yellow", "process", "perception"),
        ("p3", "中心线与路径", "PCA / 局部几何", 525, 60, 220, 55, "yellow", "data", "perception"),
        ("p4", "横偏与埋深", "反演 / 置信度", 775, 60, 205, 55, "yellow", "process", "perception"),
        ("e1", "电缆磁场环境", "场景与物理响应", 40, 55, 250, 55, "gray", "io", "physics"),
        ("e2", "几何变换工具", "投影 / 拟合", 365, 55, 250, 55, "gray", "process", "physics"),
        ("e3", "路线先验", "走廊 / 约束", 690, 55, 250, 55, "gray", "data", "physics"),
        ("s1", "磁力计", "三轴磁场", 30, 55, 210, 55, "purple", "io", "sensor"),
        ("s2", "惯导与姿态", "IMU / 航位", 275, 55, 210, 55, "purple", "io", "sensor"),
        ("s3", "声呐定位", "稀疏锚点", 520, 55, 210, 55, "purple", "io", "sensor"),
        ("s4", "埋深观测", "周期后验", 765, 55, 210, 55, "purple", "data", "sensor"),
        ("l1", "传感器采样", "", 35, 55, 220, 52, "purple", "io", "loop"),
        ("l2", "信号特征", "", 35, 145, 220, 52, "yellow", "process", "loop"),
        ("l3", "感知更新", "", 35, 235, 220, 52, "yellow", "process", "loop"),
        ("l4", "任务决策", "", 35, 325, 220, 52, "green", "state", "loop"),
        ("l5", "控制指令", "", 35, 415, 220, 52, "green", "process", "loop"),
        ("l6", "载体推进", "", 35, 505, 220, 52, "gray", "io", "loop"),
    ]
    for spec in nodes:
        add_shape(cells, *spec)
    for i in range(1, 6):
        add_edge(cells, f"loop{i}", f"l{i}", f"l{i+1}", "", main=True, extra="exitX=0.5;exitY=1;entryX=0.5;entryY=0;")
    add_edge(
        cells,
        "feedback",
        "l6",
        "l1",
        "反馈",
        dashed=True,
        points=[(1360, 615), (1360, 90)],
        extra="exitX=1;entryX=1;",
    )
    write("fig_arch_onion", cells, 1400, 690)


def mission_fsm() -> None:
    cells: list[str] = []
    for spec in [
        ("search", "SEARCH", "之字形覆盖搜索", 70, 120, 240, 82, "blue", "state"),
        ("lock", "LOCK ALIGN", "对齐锁定 / 减速", 415, 120, 240, 82, "green", "state"),
        ("track", "TRACK ACTIVE", "声磁协同跟踪", 760, 120, 240, 82, "green", "state"),
        ("reacquire", "REACQUIRE", "有界区域重捕", 760, 345, 240, 82, "yellow", "gate"),
        ("emergency", "EMERGENCY SURFACE", "应急上浮 / 终态", 360, 545, 310, 86, "red", "terminal"),
    ]:
        add_shape(cells, *spec)
    add_edge(cells, "m1", "search", "lock", "信号迟滞锁存", main=True, extra="exitX=1;entryX=0;")
    add_edge(cells, "m2", "lock", "track", "拟合收敛", main=True, extra="exitX=1;entryX=0;")
    add_edge(
        cells,
        "m3",
        "track",
        "lock",
        "信号丢失：保留中心线",
        points=[(880, 75), (535, 75)],
        extra="exitX=0.5;exitY=0;entryX=0.5;entryY=0;",
    )
    add_edge(cells, "m4", "track", "reacquire", "重捕区就绪", main=True, extra="exitX=0.5;exitY=1;entryX=0.5;entryY=0;")
    add_edge(
        cells,
        "m5",
        "reacquire",
        "lock",
        "信号恢复",
        points=[(700, 385), (700, 260), (535, 260)],
        extra="exitX=0;entryX=0.5;entryY=1;",
    )
    add_edge(
        cells,
        "m6",
        "reacquire",
        "search",
        "重捕区超时",
        dashed=True,
        points=[(1070, 385), (1070, 55), (190, 55)],
        extra="exitX=1;entryX=0.5;entryY=0;",
    )
    add_edge(
        cells,
        "m7",
        "track",
        "emergency",
        "置信度持续破底板",
        safe=True,
        points=[(1080, 240), (1080, 500), (515, 500)],
        extra="exitX=0.5;exitY=1;entryX=0.5;entryY=0;",
    )
    add_edge(
        cells,
        "m8",
        "reacquire",
        "emergency",
        "",
        safe=True,
        points=[(880, 500), (515, 500)],
        extra="exitX=0.5;exitY=1;entryX=0.5;entryY=0;",
    )
    write("fig_mission_fsm", cells, 1140, 690)


def state_machine_hierarchy() -> None:
    cells: list[str] = []
    add_lane(cells, "bt", "行为树任务调度", 30, 30, 1320, 155, "green")
    add_lane(cells, "fsm", "电缆跟踪模式 FSM", 30, 205, 1320, 175, "blue")
    add_lane(cells, "mechanisms", "控制与感知子机制", 30, 400, 1320, 220, "yellow")
    for spec in [
        ("bt_node", "电缆跟踪子树", "安全抢占 / 任务授权 / 回退入口", 410, 65, 500, 70, "green", "module", "bt"),
        ("fsm_node", "五态模式管理", "搜索 → 锁定 → 跟踪 → 重捕 → 应急", 350, 75, 620, 78, "blue", "state", "fsm"),
        ("control_node", "控制子机制", "基线跟踪 / 受限横切 / 恢复 / 冷却", 120, 75, 440, 82, "green", "process", "mechanisms"),
        ("perception_node", "感知子机制", "局部路径 / 候选区域 / 埋深 / 置信度", 760, 75, 440, 82, "yellow", "data", "mechanisms"),
    ]:
        add_shape(cells, *spec)
    add_edge(
        cells,
        "h1",
        "bt_node",
        "fsm_node",
        "",
        main=True,
        points=[(690, 195), (300, 195), (300, 255), (690, 255)],
        extra="exitX=0.5;exitY=1;entryX=0.5;entryY=0;",
    )
    add_edge(
        cells,
        "h2",
        "fsm_node",
        "control_node",
        "",
        main=True,
        points=[(690, 390), (370, 390)],
        extra="exitX=0.5;exitY=1;entryX=0.5;entryY=0;",
    )
    add_edge(
        cells,
        "h3",
        "fsm_node",
        "perception_node",
        "",
        points=[(690, 390), (1010, 390)],
        extra="exitX=0.5;exitY=1;entryX=0.5;entryY=0;",
    )
    add_edge(
        cells,
        "h4",
        "perception_node",
        "fsm_node",
        "路径 / 置信度 / 候选区",
        dashed=True,
        points=[(1260, 515), (1260, 300), (1040, 300)],
        extra="exitX=1;entryX=1;",
    )
    add_edge(cells, "h5", "control_node", "perception_node", "主动观测激励", dashed=True, extra="exitX=1;entryX=0;")
    write("fig_statemachine_hierarchy", cells, 1400, 650)


def main() -> None:
    zigzag_probe()
    perception_pipeline()
    architecture_layers()
    mission_fsm()
    state_machine_hierarchy()


if __name__ == "__main__":
    main()
