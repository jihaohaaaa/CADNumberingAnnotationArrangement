from typing import List, Set, Tuple
from shapely.geometry import Point, Polygon, LineString, box, GeometryCollection
from shapely.geometry.base import BaseGeometry
from heapq import heappush, heappop


def is_valid_line(
    line: LineString,
    polygon: Polygon,
    obstacles: List[BaseGeometry],
    existing_lines: List[LineString],
    dispel_line: GeometryCollection = None,
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

    Returns:
        bool: True if the line is valid, False otherwise.
    """

    if line.crosses(polygon):
        return False
    if any(line.crosses(ob) for ob in obstacles):
        return False
    if any(line.crosses(l) for l in existing_lines):
        return False
    return True


def point_approximate_line_string(
    point: Point, line: LineString, threshold: float = 0.5
):
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
