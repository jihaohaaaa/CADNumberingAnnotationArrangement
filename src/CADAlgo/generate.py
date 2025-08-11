from shapely.geometry import Point, Polygon, LineString, box
from heapq import heappush, heappop
from CADAlgo.check import is_valid_line
from CADAlgo.parser import *


# def generate_sampled_points(
#     polygon: Polygon, samples_per_edge: int = 20
# ) -> list[Point]:
#     """
#     Sample points along the edges of a polygon

#     Args:
#         polygon (Polygon): The polygon boundary.
#         samples_per_edge (int): Number of samples per edge.

#     Returns:
#         List[Point]: List of sampled points along the polygon boundary.
#     """
#     boundary_coords = list(polygon.exterior.coords)
#     sampled_boundary_pts = []

#     for i in range(len(boundary_coords) - 1):
#         start = Point(boundary_coords[i])
#         end = Point(boundary_coords[i + 1])
#         for j in range(samples_per_edge):
#             f = j / samples_per_edge
#             x = start.x + f * (end.x - start.x)
#             y = start.y + f * (end.y - start.y)
#             sampled_boundary_pts.append(Point(x, y))
#     return sampled_boundary_pts


def generate_sampled_points_by_length(polygon: Polygon, step: float) -> list[Point]:
    """
    Sample points along the boundary of a polygon at fixed intervals.

    Args:
        polygon (Polygon): The polygon boundary.
        step (float): Sampling distance interval.

    Returns:
        List[Point]: List of sampled points along the polygon boundary.
    """
    boundary = polygon.exterior
    length = boundary.length
    num_samples = int(length // step)

    sampled_points = [
        boundary.interpolate(distance)
        for distance in [i * step for i in range(num_samples + 1)]
    ]
    return sampled_points


def generate_connection_lines(
    points: list[Point],
    polygon: Polygon,
    obstacles: list[Polygon],
    samples_distance: int = 20,
) -> list[LineString]:
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
    sampled_boundary_pts = generate_sampled_points_by_length(polygon, samples_distance)

    # 2. For each point, generate valid connection candidates
    point_candidates: list[list[tuple[float, int, LineString]]] = []
    counter = 0  # Unique tiebreaker

    for pt in points:
        candidate_lines: list[tuple[float, int, LineString]] = []
        for boundary_pt in sampled_boundary_pts:
            line = LineString([pt, boundary_pt])
            if not line.crosses(polygon) and not any(
                line.crosses(ob) for ob in obstacles
            ):
                heappush(candidate_lines, (line.length, counter, line))
                counter += 1  # Increment counter for uniqueness
        point_candidates.append(candidate_lines)

    # 3. Greedy selection without intersection
    used_lines: list[LineString] = []
    used_points: set[int] = set()

    for idx, candidates in enumerate(point_candidates):
        while candidates:
            _, _, line = heappop(candidates)  # Extract line, ignore counter
            if is_valid_line(line, polygon, obstacles, used_lines, []):
                used_lines.append(line)
                used_points.add(idx)
                break

    return used_lines


# TODO 线条需要不垂直于外围边界线, 应该是一个范围的角度内(例如10°至80°).
# TODO 有些线候选太短, 需要确定确定规则采样. 小于某个值就是当成点(只取中间点), 某个范围内固定步长采样, 超过某个范围就固定个数采样.
# TODO 优化建议, samples_distance 从大往小变化 可以根据外围盒子的周长的100份, 或者外围盒子的周长的目标个数份
# TODO 预处理 将 point_candidates line_candidates 排序, 按照几何中心距离包围盒的距离进行排序
def generate_connection_lines_from_point_candidates(
    point_candidates: list[PartPointCandidate],
    line_candidates: list[PartLineCandidate],
    exterior: Polygon,
    obstacles: list[Polygon],
    dispel_lines: list[LineString],
    samples_distance: int = 1,
) -> list[LineString]:
    """
    For each PartPointCandidate, attempt to connect one of its points to the polygon boundary
    with a valid, non-intersecting line segment. Only one line per candidate is allowed.

    Args:
        candidates (List[PartPointCandidate]): List of candidates with multiple points each.
        polygon (Polygon): The polygon boundary.
        obstacles (List[BaseGeometry]): List of obstacles to avoid.
        samples_per_edge (int): Number of samples per edge of the polygon.

    Returns:
        List[LineString]: List of generated connection lines (one per successful candidate).
    """
    # 1. Sample the polygon boundary
    sampled_boundary_pts = generate_sampled_points_by_length(exterior, samples_distance)

    # 2. Attempt to connect one point per candidate
    used_lines: list[LineString] = []
    counter = 0  # Unique tie-breaker for heap

    # 3. Convert line candidates to point candidates
    total_candidates = []
    total_candidates.extend(point_candidates)
    from_line_point_candidate = []
    for line_candidate in line_candidates:
        from_line_point_candidate.append(line_candidate.to_point_candidate(10))
    total_candidates.extend(from_line_point_candidate)

    for candidate in total_candidates:
        candidate_lines: list[tuple[float, int, LineString]] = []
        for pt in candidate.points:
            for boundary_pt in sampled_boundary_pts:
                line = LineString([pt, boundary_pt])
                if not line.crosses(exterior) and not any(
                    line.crosses(ob) for ob in obstacles
                ):
                    heappush(candidate_lines, (line.length, counter, line))
                    counter += 1

        # Select the shortest valid line that doesn't intersect others
        while candidate_lines:
            _, _, line = heappop(candidate_lines)
            if is_valid_line(
                line,
                exterior,
                obstacles,
                used_lines,
                dispel_lines,
            ):
                used_lines.append(line)
                break  # Only one line per candidate

    return used_lines


def generate_connection_lines_from_point_candidates_backtracking(
    point_candidates: list[PartPointCandidate],
    line_candidates: list[PartLineCandidate],
    exterior: Polygon,
    obstacles: list[Polygon],
    dispel_lines: list[LineString],
    samples_distance: int = 1,
) -> list[LineString]:
    """
    回溯版本：为每个候选点/线尝试连接到外轮廓，避免交叉/冲突。
    保证全局尽可能多地连接成功（而不是局部贪心）。
    """

    sampled_boundary_pts = generate_sampled_points_by_length(exterior, samples_distance)

    # 合并点候选和线候选, 统一为点候选
    total_candidates = []
    total_candidates.extend(point_candidates)
    for line_candidate in line_candidates:
        # 线候选转为点候选
        total_candidates.append(line_candidate.to_point_candidate(10))

    # 为每个候选预生成所有可能线（按长度升序）
    all_candidate_lines = []
    counter = 0
    for candidate in total_candidates:
        lines_heap = []
        for pt in candidate.points:
            for boundary_pt in sampled_boundary_pts:
                line = LineString([pt, boundary_pt])
                if not line.crosses(exterior) and not any(
                    line.crosses(ob) for ob in obstacles
                ):
                    heappush(lines_heap, (line.length, counter, line))
                    counter += 1
        # 排序成列表（按长度）
        sorted_lines = [item[2] for item in sorted(lines_heap)]
        all_candidate_lines.append(sorted_lines)

    best_solution = []

    def dfs(idx, used_lines, current_solution):
        nonlocal best_solution

        # 如果所有候选都处理完
        if idx == len(all_candidate_lines):
            if len(current_solution) > len(best_solution):
                best_solution = current_solution.copy()
            return

        # 剪枝：即使后面全成功，也不可能超过当前最佳
        remaining_possible = len(all_candidate_lines) - idx
        if len(current_solution) + remaining_possible <= len(best_solution):
            return

        # 尝试当前候选的每一条线
        placed = False  # 这个变量冗余了
        for line in all_candidate_lines[idx]:
            if is_valid_line(line, exterior, obstacles, used_lines, dispel_lines):
                used_lines.append(line)
                current_solution.append(line)

                dfs(idx + 1, used_lines, current_solution)

                # 回溯
                used_lines.pop()
                current_solution.pop()
                placed = True

        # 也允许跳过当前候选
        dfs(idx + 1, used_lines, current_solution)

    dfs(0, [], [])

    return best_solution
