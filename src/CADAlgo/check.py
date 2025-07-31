from typing import List, Set, Tuple
from shapely.geometry import Point, Polygon, LineString, box
from shapely.geometry.base import BaseGeometry
from heapq import heappush, heappop

def is_valid_line(
    line: LineString,
    polygon: Polygon,
    obstacles: List[BaseGeometry],
    existing_lines: List[LineString]
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
    if any(line.intersects(l) for l in existing_lines):
        return False
    return True
