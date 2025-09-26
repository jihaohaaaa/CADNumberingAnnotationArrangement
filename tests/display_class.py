import matplotlib.pyplot as plt
from matplotlib import rcParams
from CADAlgo.parser import Schema
from shapely.geometry import (
    Point,
    LineString,
    Polygon,
    MultiPoint,
    MultiLineString,
    MultiPolygon,
)
from shapely.geometry.base import BaseGeometry

# 设置字体为 SimHei(黑体)，以支持中文
rcParams["font.sans-serif"] = ["SimHei"]  # 使用黑体
rcParams["axes.unicode_minus"] = False  # 解决负号显示问题


def plot_schema(schema: Schema):
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect("equal")
    ax.set_title(f"Schema: {schema.schema_name}")
    ax.grid(True, linestyle="--", alpha=0.5)

    # 画目标外轮廓
    exterior = schema.target_exterior
    x, y = exterior.exterior.xy
    ax.plot(x, y, color="blue", linewidth=2, label="Target Exterior")

    # 画障碍物盒子(填充多边形)
    for i, box in enumerate(schema.obstacle_boxes):
        poly = box.geometry
        x, y = poly.exterior.xy
        ax.fill(
            x, y, color="orange", alpha=0.5, label="Obstacle Box" if i == 0 else None
        )

    # 画屏障线(红色线段)
    for i, line in enumerate(schema.barrier_lines):
        x, y = line.geometry.xy
        ax.plot(
            x, y, color="red", linewidth=2, label="Barrier Line" if i == 0 else None
        )

    # 画线候选(绿色细线)
    for candidate in schema.line_candidates:
        for line in candidate.lines:
            x, y = line.geometry.xy
            ax.plot(x, y, color="green", linewidth=1, alpha=0.7, label="Line Candidate")

    # 画点候选(黑点)
    for candidate in schema.point_candidates:
        for pt in candidate.points:
            ax.plot(
                pt.geometry.x,
                pt.geometry.y,
                "ko",
                markersize=5,
                label="Point Candidate",
            )

    # 为避免重复图例, 只显示一次每个标签
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys())

    return fig, ax


def plot_geometries(geoms: list[BaseGeometry], fig=None, ax=None):
    if fig is None or ax is None:
        fig, ax = plt.subplots(figsize=(8, 6))

    for geom in geoms:
        if geom.is_empty:
            continue
        if isinstance(geom, Point):
            ax.plot(geom.x, geom.y, "o", color="blue")
        elif isinstance(geom, LineString):
            x, y = geom.xy
            ax.plot(x, y, "-", color="blue")
        elif isinstance(geom, Polygon):
            x, y = geom.exterior.xy
            ax.plot(x, y, "-", color="blue")
            # 绘制内洞
            for interior in geom.interiors:
                x, y = interior.xy
                ax.plot(x, y, "--", color="blue")
        elif isinstance(geom, MultiPoint):
            for pt in geom.geoms:
                ax.plot(pt.x, pt.y, "o", color="blue")
        elif isinstance(geom, MultiLineString):
            for line in geom.geoms:
                x, y = line.xy
                ax.plot(x, y, "-", color="blue")
        elif isinstance(geom, MultiPolygon):
            for poly in geom.geoms:
                x, y = poly.exterior.xy
                ax.plot(x, y, "-", color="blue")
                for interior in poly.interiors:
                    x, y = interior.xy
                    ax.plot(x, y, "--", color="blue")
        else:
            raise TypeError(f"Unsupported geometry type: {type(geom)}")

    ax.set_aspect("equal", adjustable="box")
    return fig, ax
