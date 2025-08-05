from typing import List, Set, Tuple
from shapely.geometry import Point, Polygon, LineString, box, GeometryCollection
from shapely.geometry.base import BaseGeometry
from heapq import heappush, heappop


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

    Args:
        line (LineString): The line to be checked.
        polygon (Polygon): The polygon boundary.
        obstacles (List[BaseGeometry]): List of obstacles to avoid.
        existing_lines (List[LineString]): List of already used lines.
        dispel_lines (List[LineString]): List of lines to avoid.

    Returns:
        bool: True if the line is valid, False otherwise.
    """

    # 不能穿过边界线
    if line.crosses(polygon):
        return False
    # 不能穿过障碍物
    if any(line.crosses(ob) for ob in obstacles):
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
        if point_approximate_line_string(
            Point(line.coords[0]), other, distance_threshold
        ):
            return False
        if point_approximate_line_string(
            Point(line.coords[-1]), other, distance_threshold
        ):
            return False
        if point_approximate_line_string(
            Point(other.coords[0]), line, distance_threshold
        ):
            return False
        if point_approximate_line_string(
            Point(other.coords[-1]), line, distance_threshold
        ):
            return False
    return True


def point_approximate_line_string(point: Point, line: LineString, threshold: float = 5):
    """
    Check if a point is approximately along a line.

    Args:
        point (Point): The point to check.
        line (LineString): The line to check against.
        threshold (float, optional): The maximum distance for the
        point to be considered along the line.. Defaults to 0.5.

    Returns:
        bool: True if the point is approximately along the line, False otherwise.
    """

    if point.distance(line) <= threshold:
        return True
    else:
        return False
