from shapely.geometry import Point, Polygon, LineString, box
from heapq import heappush, heappop
from CADAlgo.check import is_valid_line
from CADAlgo.parser import *
import logging

# Module-level logger
logger = logging.getLogger("CADAlgo.generate")
if not logger.handlers:
    # Set up a simple console handler if the app hasn't configured logging
    handler = logging.StreamHandler()
    formatter = logging.Formatter(
        fmt="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    handler.setFormatter(formatter)
    handler.setLevel(logging.INFO)
    logger.addHandler(handler)
    # File handler for detailed DFS logs
    try:
        file_handler = logging.FileHandler(
            filename="backtracking_dfs.log", mode="w", encoding="utf-8"
        )
        file_handler.setFormatter(formatter)
        file_handler.setLevel(logging.DEBUG)
        logger.addHandler(file_handler)
    except Exception:
        # If file handler can't be created, continue with console only
        pass
    # Capture DEBUG for file, while console stays at INFO via handler level
    logger.setLevel(logging.DEBUG)


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


# TODO 线条需要不垂直于外围边界线, 应该是一个范围的角度内(例如10°至80°).
# TODO 优化建议, samples_distance 从大往小变化 可以根据外围盒子的周长的100份, 或者外围盒子的周长的目标个数份
# TODO 预处理 将 point_candidates line_candidates 排序, 按照几何中心距离包围盒的距离进行排序
def generate_connection_lines_from_point_candidates_backtracking(
    point_candidates: list[PartPointCandidate],
    line_candidates: list[PartLineCandidate],
    exterior: Polygon,
    obstacles: list[Polygon],
    dispel_lines: list[LineString],
    samples_distance: int = 1,
) -> list[LineString]:
    """
    回溯版本：为每个候选点/线尝试连接到外轮廓, 避免交叉/冲突。
    保证全局尽可能多地连接成功。
    """

    sampled_boundary_pts = generate_sampled_points_by_length(exterior, samples_distance)

    # 合并点候选和线候选, 统一为点候选
    total_candidates: list[PartPointCandidate] = []
    total_candidates.extend(point_candidates)
    for line_candidate in line_candidates:
        # 线候选转为点候选
        total_candidates.append(line_candidate.to_point_candidate_v2())

    # 为每个候选预生成所有可能线(按长度升序)
    all_candidate_lines: list[list[LineString]] = []
    counter = 0
    for candidate in total_candidates:
        lines_heap: list[tuple[float, float, int, LineString]] = []
        for pt in candidate.points:
            pt_priority = pt.priority
            for boundary_pt in sampled_boundary_pts:
                # pt is MyPoint; use its geometry
                line = LineString([pt.geometry, boundary_pt])
                if not line.crosses(exterior) and not any(
                    line.crosses(ob) for ob in obstacles
                ):
                    # 优先按点的优先级排序, 其次按线长排序
                    heappush(lines_heap, (pt_priority, line.length, counter, line))
                    counter += 1
        # 排序成列表(按优先级, 再按长度)
        sorted_lines: list[LineString] = [item[3] for item in sorted(lines_heap)]
        all_candidate_lines.append(sorted_lines)

    best_solution: list[LineString] = []
    logged_full = False  # avoid duplicate logs for multiple complete solutions

    def dfs(
        idx: int,
        selected_lines: list[LineString],
    ) -> None:
        nonlocal best_solution

        remaining_possible = len(all_candidate_lines) - idx
        logger.debug(
            f"DFS enter: idx={idx}, selected={len(selected_lines)}, best={len(best_solution)}, remaining={remaining_possible}",
        )

        # 即使后面全成功, 也不可能超过当前最佳时, 进行剪枝操作
        if len(selected_lines) + remaining_possible <= len(best_solution):
            logger.debug(
                f"Prune at idx={idx}: selected({len(selected_lines)})+remaining({remaining_possible}) <= best({len(best_solution)})",
            )
            return

        # 如果所有候选都处理完
        if idx == len(all_candidate_lines):
            if len(selected_lines) > len(best_solution):
                best_solution = selected_lines.copy()
                logger.debug(f"New best at leaf: size={len(best_solution)}")
            # 如果当前解覆盖了所有候选, 说明找到整个 schema 的可行解
            if len(selected_lines) == len(all_candidate_lines):
                nonlocal logged_full
                if not logged_full:
                    logger.info(
                        f"Feasible solution found: {len(selected_lines)}/{len(all_candidate_lines)} connections established (backtracking)",
                    )
                    logged_full = True
            return

        # 尝试当前候选的每一条线
        for line in all_candidate_lines[idx]:
            try_start = tuple(line.coords[0])
            try_end = tuple(line.coords[-1])
            logger.debug(
                f"Try line at idx={idx}: start={try_start} end={try_end} len={float(line.length)}",
            )
            if is_valid_line(line, exterior, obstacles, selected_lines, dispel_lines):
                logger.debug(
                    f"Choose line at idx={idx}: start={try_start} end={try_end}",
                )
                selected_lines.append(line)
                dfs(idx + 1, selected_lines)
                # 回溯
                popped = selected_lines.pop()
                logger.debug(
                    f"Backtrack at idx={idx}: pop line start={tuple(popped.coords[0])} end={tuple(popped.coords[-1])}",
                )

            # 分支限界：若此层的理论上限(当前已选 + 剩余候选数)已等于全局最优，
            # 则本层其他线也不可能带来更优结果，直接终止本层循环。
            depth_upper_bound = len(selected_lines) + (len(all_candidate_lines) - idx)
            if len(best_solution) == depth_upper_bound:
                logger.debug(
                    f"Cut remaining lines at idx={idx}: depth_upper_bound({depth_upper_bound}) == best({len(best_solution)})",
                )
                break

        # 也允许跳过当前候选
        # 预判跳过分支的上界: 若 selected + (remaining_possible - 1) <= best，
        # 则跳过分支也不可能改写最优，直接略过该分支。
        if len(selected_lines) + (len(all_candidate_lines) - idx - 1) <= len(
            best_solution
        ):
            logger.debug(
                f"Skip branch pruned at idx={idx}: selected({len(selected_lines)}) + remaining({remaining_possible - 1}) <= best({len(best_solution)})",
            )
        else:
            logger.debug(f"Skip candidate at idx={idx}")
            dfs(idx + 1, selected_lines)

    dfs(0, [])

    # 如果存在整个 schema 的可行解, 再次确认记录(回溯可能找到)
    if (
        best_solution
        and len(best_solution) == len(all_candidate_lines)
        and not logged_full
    ):
        logger.info(
            f"Feasible solution found: {len(best_solution)}/{len(all_candidate_lines)} connections established (backtracking)",
        )

    return best_solution
