from typing import List, Set, Tuple
from shapely.geometry import Point, Polygon, LineString, box, GeometryCollection
from shapely.geometry.base import BaseGeometry
from heapq import heappush, heappop
import math


# TODO 可以让远离其他线段的逻辑可以变更,越大越不挤在一起
def is_valid_line(
    line: LineString,
    polygon: Polygon,
    obstacles: List[Polygon],
    existing_lines: List[LineString],
    dispel_lines: List[LineString],
    distance_threshold: float = 5,
) -> bool:
    """
    Check if a line is valid for connection:
    - Does not cross the polygon boundary
    - Does not cross any obstacles
    - Does not intersect with existing lines
    - Is not horizontal or vertical

    Args:
        line (LineString): The line to be checked.
        polygon (Polygon): The polygon boundary.
        obstacles (List[BaseGeometry]): List of obstacles to avoid.
        existing_lines (List[LineString]): List of already used lines.
        dispel_lines (List[LineString]): List of lines to avoid.
        distance_threshold (float): Distance threshold for line avoidance.

    Returns:
        bool: True if the line is valid, False otherwise.
    """

    # 获取角度
    def get_line_angle(line: LineString) -> float:
        """Calculate the angle of a line in degrees relative to the horizontal."""
        start, end = line.coords[0], line.coords[-1]
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        angle_rad = math.atan2(dy, dx)
        angle_deg = math.degrees(angle_rad)
        return angle_deg

    # 检查角度是否在水平或竖直范围内
    angle = get_line_angle(line)
    tolerant_angle = 15
    # 排除水平和竖直线段, tolerant 为容忍角度
    if (
        abs(angle) <= tolerant_angle
        or 90 - tolerant_angle <= abs(angle) <= 90 + tolerant_angle
        or abs(angle) >= 180 - tolerant_angle
    ):
        return False

    # 不能穿过边界线
    if line.crosses(polygon):
        return False
    # 不能穿过障碍物
    # 优化：预筛选需要精确检测的障碍物
    # 思路：
    # 1) 按直线方向所在象限，排除明显在反方向的障碍物(通过质心与起点的相对位置判断)。
    # 2) 使用包围盒快速相交测试(与线段包围盒无交则不可能相交)。
    # 注意：为确保正确性，满足任一条件(同象限 或 与线段包围盒相交)即可进入精确检测列表。
    start_pt = Point(line.coords[0])
    end_pt = Point(line.coords[-1])

    def _sign(v: float, eps: float = 1e-9) -> int:
        return 1 if v > eps else (-1 if v < -eps else 0)

    dx = end_pt.x - start_pt.x
    dy = end_pt.y - start_pt.y
    dir_sx, dir_sy = _sign(dx), _sign(dy)

    def _in_same_quadrant(ob: Polygon) -> bool:
        # 使用代表点(位于多边形内部，计算稳定)判定相对起点的象限
        rp = ob.representative_point()
        vx, vy = rp.x - start_pt.x, rp.y - start_pt.y
        sx, sy = _sign(vx), _sign(vy)
        # 如果某轴方向为0(近似水平/竖直)，则忽略该轴的象限约束
        return (dir_sx == 0 or sx == dir_sx) and (dir_sy == 0 or sy == dir_sy)

    lminx, lminy, lmaxx, lmaxy = line.bounds

    def _bbox_overlaps_line_bbox(ob: Polygon) -> bool:
        ominx, ominy, omaxx, omaxy = ob.bounds
        return not (omaxx < lminx or ominx > lmaxx or omaxy < lminy or ominy > lmaxy)

    candidate_obstacles = [
        ob for ob in obstacles if _in_same_quadrant(ob) or _bbox_overlaps_line_bbox(ob)
    ]

    if any(line.crosses(ob) for ob in candidate_obstacles):
        return False
    # 不能与现有的线相交
    if any(line.intersects(l) for l in existing_lines):
        return False
    # 终点远离驱散线
    for dispel_line in dispel_lines:
        if point_approximate_line_string(
            Point(line.coords[-1]), dispel_line, distance_threshold
        ):
            return False
    # 线段远离其他线段
    for other in existing_lines:
        if point_approximate_line_string(Point(line.coords[0]), other, 1):
            return False
        if point_approximate_line_string(Point(line.coords[-1]), other, 1):
            return False
        if point_approximate_line_string(Point(other.coords[0]), line, 1):
            return False
        if point_approximate_line_string(Point(other.coords[-1]), line, 1):
            return False
    return True


def point_approximate_line_string(point: Point, line: LineString, threshold: float = 5):
    """
    Check if a point is approximately along a line.

    Args:
        point (Point): The point to check.
        line (LineString): The line to check against.
        threshold (float, optional): The maximum distance for the
        point to be considered along the line. Defaults to 5.

    Returns:
        bool: True if the point is approximately along the line, False otherwise.
    """
    if point.distance(line) <= threshold:
        return True
    else:
        return False
