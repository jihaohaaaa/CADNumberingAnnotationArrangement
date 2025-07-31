from typing import List, Set, Tuple
from shapely.geometry import Point, Polygon, LineString, box
from shapely.geometry.base import BaseGeometry
from heapq import heappush, heappop
from CADAlgo.check import is_valid_line

#TODO: sampling points along boundary by length instead of pieces

def generate_sampled_points(
    polygon: Polygon,
    samples_per_edge: int = 20
) -> List[Point]:
    """
    Sample points along the edges of a polygon

    Args: 
        polygon (Polygon): The polygon boundary.
        samples_per_edge (int): Number of samples per edge. 

    Returns:
        List[Point]: List of sampled points along the polygon boundary.
    """
    boundary_coords = list(polygon.exterior.coords)
    sampled_boundary_pts = []

    for i in range(len(boundary_coords) - 1):
        start = Point(boundary_coords[i])
        end = Point(boundary_coords[i + 1])
        for j in range(samples_per_edge):
            f = j / samples_per_edge
            x = start.x + f * (end.x - start.x)
            y = start.y + f * (end.y - start.y)
            sampled_boundary_pts.append(Point(x, y))
    return sampled_boundary_pts

#TODO: use generate_sampled_points to generate sampled points along the polygon boundary
def generate_connection_lines(
    points: List[Point],
    polygon: Polygon,
    obstacles: List[BaseGeometry],
    samples_per_edge: int = 20
) -> List[LineString]:
    """
    Generate connection lines from points to the polygon boundary,
    avoiding obstacles and existing lines.

    Args:
        points (List[Point]): List of points to connect.
        polygon (Polygon): The polygon boundary.
        obstacles (List[BaseGeometry]): List of obstacles to avoid.
        samples_per_edge (int): Number of samples per edge of the polygon.
    
    Returns:
        List[LineString]: List of generated connection lines.
    """
    # 1. Sample polygon boundary
    boundary_coords: List[Tuple[float, float]] = list(polygon.exterior.coords)
    sampled_boundary_pts: List[Point] = []

    for i in range(len(boundary_coords) - 1):
        start = Point(boundary_coords[i])
        end = Point(boundary_coords[i + 1])
        for j in range(samples_per_edge):
            f = j / samples_per_edge
            x = start.x + f * (end.x - start.x)
            y = start.y + f * (end.y - start.y)
            sampled_boundary_pts.append(Point(x, y))

    # 2. For each point, generate valid connection candidates
    point_candidates: List[List[Tuple[float, int, LineString]]] = []
    counter = 0  # Unique tiebreaker

    for pt in points:
        candidate_lines: List[Tuple[float, int, LineString]] = []
        for boundary_pt in sampled_boundary_pts:
            line = LineString([pt, boundary_pt])
            if not line.crosses(polygon) and not any(line.crosses(ob) for ob in obstacles):
                heappush(candidate_lines, (line.length, counter, line))
                counter += 1  # Increment counter for uniqueness
        point_candidates.append(candidate_lines)

    # 3. Greedy selection without intersection
    used_lines: List[LineString] = []
    used_points: Set[int] = set()

    for idx, candidates in enumerate(point_candidates):
        while candidates:
            _, _, line = heappop(candidates)  # Extract line, ignore counter
            if is_valid_line(line, polygon, obstacles, used_lines):
                used_lines.append(line)
                used_points.add(idx)
                break

    return used_lines

