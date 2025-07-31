from typing import List, Set, Tuple
from shapely.geometry import Point, Polygon, LineString, box
from shapely.geometry.base import BaseGeometry
import matplotlib.pyplot as plt
from heapq import heappush, heappop

def is_valid_line(
    line: LineString,
    polygon: Polygon,
    obstacles: List[BaseGeometry],
    existing_lines: List[LineString]
) -> bool:
    if line.crosses(polygon):
        return False
    if any(line.crosses(ob) for ob in obstacles):
        return False
    if any(line.crosses(l) for l in existing_lines):
        return False
    return True

def generate_connection_lines(
    points: List[Point],
    polygon: Polygon,
    obstacles: List[BaseGeometry],
    samples_per_edge: int = 20
) -> List[LineString]:
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

def generate_sampled_points(
    polygon: Polygon,
    samples_per_edge: int = 20
) -> List[Point]:
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

# Define polygon boundary and points
polygon = Polygon([(0, 0), (50, 25), (100, 0), (100, 100), (0, 100), (25, 50)])
points = [Point(25, 25), Point(30, 30), Point(75, 25), Point(75, 75), Point(25, 75)]


obstacles = [box(84,72, 88, 78)]

# Generate connection lines
lines = generate_connection_lines(points, polygon, obstacles, samples_per_edge=20)

# Plotting
fig, ax = plt.subplots()
x_poly, y_poly = polygon.exterior.xy
ax.plot(x_poly, y_poly, 'k-', linewidth=1, label='Polygon')

x_coords = [pt.x for pt in generate_sampled_points(polygon)]
y_coords = [pt.y for pt in generate_sampled_points(polygon)]
ax.plot(x_coords, y_coords, 'ro', markersize=3, label='Sampled Points')

x_point_coords = [pt.x for pt in points]
y_point_coords = [pt.y for pt in points]
ax.plot(x_point_coords, y_point_coords, 'bo', markersize=3, label='Points')

for obstacle in obstacles:
    x_coords_ob, y_coords_ob = obstacle.exterior.xy
    ax.fill(x_coords_ob, y_coords_ob, color='gray', alpha=0.5, label='Obstacle')

# 绘制连接线
for line in lines:
    x_line, y_line = line.xy
    ax.plot(x_line, y_line, color='green', linewidth=1.5)

ax.set_aspect('equal')
ax.set_title('Sampled Points on Polygon Boundary')
ax.legend()
plt.grid(True)
plt.show()