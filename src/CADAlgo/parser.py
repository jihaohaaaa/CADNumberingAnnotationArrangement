from __future__ import annotations

from functools import total_ordering
from shapely.geometry import Point, LineString, Polygon
from shapely.ops import unary_union

from typing import TYPE_CHECKING


@total_ordering
class MyLine:
    start: tuple[float, float]
    end: tuple[float, float]
    geometry: LineString
    priority: float = 1.0  # 从 JSON 的 weight 映射过来
    part_id: int = -1  # 归属零件 ID（-1 表示非零件，如 barrier）

    def __init__(
        self,
        start: tuple[float, float],
        end: tuple[float, float],
        priority: float = 1.0,
        part_id: int = -1,
    ):
        self.start = start
        self.end = end
        self.geometry = LineString([start, end])
        self.priority = float(priority)
        self.part_id = part_id

    def __lt__(self, other: "MyLine") -> bool:
        return self.geometry.length < other.geometry.length

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, MyLine):
            return NotImplemented
        return self.geometry.length == other.geometry.length


class ObstacleBox:
    vertices: list[tuple[float, float]]
    geometry: Polygon

    def __init__(self, vertices: list[tuple[float, float]]):
        self.vertices = vertices
        self.geometry = Polygon(vertices)


class MyPoint:
    coordinate: tuple[float, float]
    geometry: Point
    priority: float = 1.0  # 从 JSON 的 weight 映射过来
    part_id: int = -1  # 归属零件 ID（-1 表示非零件）

    def __init__(
        self,
        coordinate: tuple[float, float],
        priority: float = 1.0,
        part_id: int = -1,
    ):
        self.coordinate = coordinate
        self.geometry = Point(coordinate)
        self.priority = float(priority)
        self.part_id = part_id


class PartLineCandidate:
    part_name: str
    part_id: int
    lines: list[MyLine]

    def __init__(self, part_name: str, lines: list[MyLine], part_id: int):
        self.part_name = part_name
        self.part_id = part_id
        self.lines = lines

    def to_point_candidate(self, step: float):
        """
        Convert LineCandidate to PointCandidate by sample lines to points

        Args:
            step (float): `step` is the sampling distance interval.

        Returns:
            PartPointCandidate: A new PartPointCandidate instance.
            Represent the sampled points from the lines.
        """
        # 直接基于 MyLine 进行整路径采样, 返回 MyPoint 列表
        my_points = interpolate_whole_path(self.lines, step)
        return PartPointCandidate(self.part_name, my_points, part_id=self.part_id)

    def to_point_candidate_v2(self):
        """
        v2: 将 LineCandidate 转为 PointCandidate, 使用每段单独采样规则。

        参见 interpolate_per_line_v2。
        """
        my_points = interpolate_per_line_v2(self.lines)
        return PartPointCandidate(self.part_name, my_points, part_id=self.part_id)


class PartPointCandidate:
    part_name: str
    part_id: int
    points: list[MyPoint]

    def __init__(self, part_name: str, points: list[MyPoint], part_id: int):
        self.part_name = part_name
        self.part_id = part_id
        self.points = points


class Schema:
    json_data: dict
    schema_name: str
    center: tuple[float, float]  # 几何中心
    target_exterior: Polygon
    obstacle_boxes_raw: list[ObstacleBox]  # 原始障碍框
    obstacle_boxes: list[ObstacleBox]  # 过滤/合并后的障碍框
    barrier_lines: list[MyLine]
    line_candidates: list[PartLineCandidate]
    point_candidates: list[PartPointCandidate]

    def __init__(
        self,
        schema_name: str,
        obstacle_boxes: list[ObstacleBox],
        barrier_lines: list[MyLine],
        target_exterior: Polygon,
        line_candidates: list[PartLineCandidate],
        point_candidates: list[PartPointCandidate],
        center: tuple[float, float],
        json_data: dict,
    ):
        self.schema_name = schema_name
        # 几何中心：由 JSON 提供
        self.center = center
        # 先设置 target_exterior, 便于基于它过滤障碍框
        self.target_exterior = target_exterior
        # 为保留原始障碍框信息, 增加 raw 版本（不过滤/不合并）
        self.obstacle_boxes_raw = list(obstacle_boxes)
        # 由专用方法完成障碍框的过滤与合并
        self.obstacle_boxes = self._build_obstacle_boxes(
            self.obstacle_boxes_raw, self.target_exterior
        )
        self.barrier_lines = barrier_lines
        self.line_candidates = line_candidates
        self.point_candidates = point_candidates
        self.json_data = json_data

    def generate_solution(
        self,
        *,
        samples_distance: int = 3,
        proximity_threshold: float = 1.0,
    ) -> list[MyLine]:
        """Generate connection line solution for this schema.

        This is a high-level wrapper that calls the legacy/underlying
        backtracking generator defined in `generate.underlying_generate_solution`.

        Params:
            samples_distance: boundary sampling step for exterior polygon.
            proximity_threshold: minimal allowed distance between any two selected lines.
        Returns:
            Solution: encapsulated result with convenient (de)serialization.
        """
        # Local imports to avoid circular dependency at module import time
        from .generate import underlying_generate_solution

        exterior = self.target_exterior
        obstacles = [ob.geometry for ob in self.obstacle_boxes]
        dispel_lines = [dl.geometry for dl in self.barrier_lines]

        lines = underlying_generate_solution(
            self.point_candidates,
            self.line_candidates,
            exterior,
            obstacles,
            dispel_lines,
            samples_distance=samples_distance,
        )
        return lines

    # Convenience shortcut
    def export_solution(
        self,
        path: str,
        *,
        samples_distance: int = 3,
        proximity_threshold: float = 1.0,
        indent: int | None = 2,
        ensure_ascii: bool = False,
    ) -> dict:
        """Generate solution then save to a JSON file.

        Returns the in-memory Solution instance for further use.
        """
        lines = self.generate_solution(
            samples_distance=samples_distance,
            proximity_threshold=proximity_threshold,
        )
        import json as _json
        solution_block = {
            "line_count": len(lines),
            "lines": [
                {
                    "start": {"x": ln.start[0], "y": ln.start[1]},
                    "end": {"x": ln.end[0], "y": ln.end[1]},
                    "part_id": ln.part_id,
                }
                for ln in lines
            ],
        }
        merged = dict(self.json_data)
        merged["solution"] = solution_block
        with open(path, "w", encoding="utf-8") as f:
            f.write(_json.dumps(merged, indent=indent, ensure_ascii=ensure_ascii))
        return solution_block

    @staticmethod
    def _build_obstacle_boxes(
        obstacle_boxes: list[ObstacleBox],
        target_exterior: Polygon,
        delta: float = 1.0,
    ) -> list[ObstacleBox]:
        """
        基于目标外轮廓 target_exterior 对障碍框执行三步处理:
        1) 过滤：仅保留与 target_exterior 相交的障碍框;
        2) 去除被包含：按面积从大到小, 剔除被已保留集合覆盖(covers)的小框;
        3) 合并：将距离<=delta(含相交/相邻)的障碍框聚类并合并为其最小外接旋转矩形。

        返回处理后的障碍框列表。
        """
        # 仅保留与 target_exterior 相交的障碍框; 完全在外部的剔除
        filtered_obs = [
            ob for ob in obstacle_boxes if ob.geometry.intersects(target_exterior)
        ]

        # 若大的障碍框包住了小的, 则保留大的, 剔除被包含的小的
        sorted_obs = sorted(filtered_obs, key=lambda ob: ob.geometry.area, reverse=True)
        kept: list[ObstacleBox] = []
        for ob in sorted_obs:
            if any(k.geometry.covers(ob.geometry) for k in kept):
                continue
            kept.append(ob)

        # 将相互距离小于等于 delta(含重叠/相邻)的障碍框合并为更大的一个
        if not kept:
            return []

        polys = [ob.geometry for ob in kept]
        n = len(polys)
        visited = [False] * n
        clusters: list[list[int]] = []
        for i in range(n):
            if visited[i]:
                continue
            # BFS/DFS 聚类：距离<=delta 即连通
            queue = [i]
            visited[i] = True
            cluster: list[int] = []
            while queue:
                p = queue.pop(0)
                cluster.append(p)
                for q in range(n):
                    if not visited[q] and polys[p].distance(polys[q]) <= delta:
                        visited[q] = True
                        queue.append(q)
            clusters.append(cluster)

        merged_boxes: list[ObstacleBox] = []
        for idxs in clusters:
            geoms = [polys[idx] for idx in idxs]
            uni = unary_union(geoms)
            if uni.is_empty:
                continue
            # 用最小外接旋转矩形作为合并结果(OBB)
            mrr = uni.minimum_rotated_rectangle
            verts: list[tuple[float, float]] | None = None
            # 仅当是 Polygon 时才有 exterior
            if isinstance(mrr, Polygon):
                coords = list(mrr.exterior.coords)
                # Shapely 返回首尾相同点, 取前四个顶点
                verts = [(float(x), float(y)) for (x, y) in coords[:-1][:4]]
            if verts and len(verts) == 4:
                merged_boxes.append(ObstacleBox(verts))
            else:
                # 兜底：如果异常, 回退为该簇的外包矩形(轴对齐)
                minx, miny, maxx, maxy = map(float, uni.bounds)
                fallback = [(minx, miny), (maxx, miny), (maxx, maxy), (minx, maxy)]
                merged_boxes.append(ObstacleBox(fallback))

        return merged_boxes


class SchemaParseError(Exception):
    pass


def parse_schema(json_data: dict) -> Schema:
    def xy(p):
        try:
            return (p["x"], p["y"])
        except KeyError as e:
            raise SchemaParseError(f"Point dict missing key: {e}")

    # 必须字段检查
    if "obstacle_box" not in json_data:
        raise SchemaParseError("Missing 'obstacle_box' field")
    if "barrier_line" not in json_data:
        raise SchemaParseError("Missing 'barrier_line' field")
    if "target_exterior" not in json_data:
        raise SchemaParseError("Missing 'target_exterior' field")
    if "part" not in json_data:
        raise SchemaParseError("Missing 'part' field")
    if "schema_name" not in json_data:
        raise SchemaParseError("Missing 'schema_name' field")

    obstacle_boxes: list[ObstacleBox] = []
    for box in json_data["obstacle_box"]:
        try:
            vertices = [xy(box["a"]), xy(box["b"]), xy(box["c"]), xy(box["d"])]
        except KeyError as e:
            raise SchemaParseError(f"ObstacleBox missing vertex key: {e}")
        obstacle_boxes.append(ObstacleBox(vertices))

    barrier_lines: list[MyLine] = []
    for line in json_data["barrier_line"]:
        try:
            start = (line["x1"], line["y1"])
            end = (line["x2"], line["y2"])
        except KeyError as e:
            raise SchemaParseError(f"BarrierLine missing coordinate key: {e}")
        barrier_lines.append(MyLine(start, end))

    try:
        target_exterior = Polygon([xy(pt) for pt in json_data["target_exterior"]])
    except Exception as e:
        raise SchemaParseError(f"Invalid 'target_exterior' polygon points: {e}")

    part_data = json_data["part"]

    line_candidates: list[PartLineCandidate] = []
    for item in part_data.get("line_candidates", []):
        if "part_name" not in item or "lines" not in item or "part_id" not in item:
            raise SchemaParseError(
                "line_candidates item missing 'part_name', 'lines' or 'part_id'"
            )
        part_id = int(item["part_id"])  # 必有
        lines: list[MyLine] = []
        for line in item["lines"]:
            # 仅支持新格式：{"lineSegment": {x1,y1,x2,y2}, "weight": number}
            if "lineSegment" not in line:
                raise SchemaParseError(
                    "line_candidates line item missing 'lineSegment' (new format only)"
                )
            seg = line["lineSegment"]
            try:
                start = (seg["x1"], seg["y1"])  # type: ignore
                end = (seg["x2"], seg["y2"])  # type: ignore
            except KeyError as e:
                raise SchemaParseError(
                    f"LineCandidate lineSegment missing coordinate key: {e}"
                )
            priority = float(line.get("weight", 1.0))
            lines.append(MyLine(start, end, priority, part_id=part_id))
        line_candidates.append(
            PartLineCandidate(item["part_name"], lines, part_id=part_id)
        )

    point_candidates: list[PartPointCandidate] = []
    for item in part_data.get("point_candidates", []):
        if "part_name" not in item or "points" not in item or "part_id" not in item:
            raise SchemaParseError(
                "point_candidates item missing 'part_name', 'points' or 'part_id'"
            )
        part_id = int(item["part_id"])  # 必有
        points: list[MyPoint] = []
        for pt in item["points"]:
            # 仅支持新格式：{"coordinate": {x,y}, "weight": number}
            if "coordinate" not in pt:
                raise SchemaParseError(
                    "point_candidates point item missing 'coordinate' (new format only)"
                )
            coord = pt["coordinate"]
            try:
                x: float = coord["x"]
                y: float = coord["y"]
            except KeyError as e:
                raise SchemaParseError(f"PointCandidate coordinate missing key: {e}")
            priority = float(pt.get("weight", 1.0))
            points.append(MyPoint((x, y), priority, part_id=part_id))
        point_candidates.append(
            PartPointCandidate(item["part_name"], points, part_id=part_id)
        )
    # 几何中心x
    center: tuple[float, float] = (json_data["center"]["x"], json_data["center"]["y"])

    return Schema(
        json_data["schema_name"],
        obstacle_boxes,
        barrier_lines,
        target_exterior,
        line_candidates,
        point_candidates,
        center,
        json_data,
    )


def interpolate_whole_path(lines: list[MyLine], step: float) -> list[MyPoint]:
    """
    按给定步长在整条折线路径上等距采样, 输入使用 MyLine, 输出 MyPoint。

    说明：
    - 路径由若干直线 MyLine 顺序拼接而成。
    - 采样位置覆盖起点 0 与终点 total_length; 中间按 step 递增。
    - 对每段线做线性插值, 不依赖 shapely 的 interpolate, 避免类型不匹配。
    - 采样点的 priority 继承自所在线段 MyLine.priority。
    """
    result: list[MyPoint] = []

    # 预计算每段长度
    seg_lengths: list[float] = []
    for ln in lines:
        (x1, y1) = ln.start
        (x2, y2) = ln.end
        dx = x2 - x1
        dy = y2 - y1
        seg_lengths.append((dx * dx + dy * dy) ** 0.5)

    total_length: float = sum(seg_lengths)
    if total_length <= 0:
        return result

    # 构造采样距离序列
    distances: list[float] = []
    d = 0.0
    # 避免浮点误差导致漏采最后一点
    eps = 1e-9
    while d <= total_length + eps:
        distances.append(min(d, total_length))
        d += step
    if distances[-1] < total_length - eps:
        distances.append(total_length)  # 确保终点被采样

    current_line_index = 0
    current_offset = 0.0  # 当前线段在整体路径的起点累计长度

    for distance in distances:
        # 移动到包含当前 distance 的线段
        while (
            current_line_index < len(lines)
            and current_offset + seg_lengths[current_line_index] < distance - eps
        ):
            current_offset += seg_lengths[current_line_index]
            current_line_index += 1
            if current_line_index >= len(lines):
                # 超出路径长度, 提前返回结果
                return result

        # 当前段与本地距离
        ln = lines[current_line_index]
        seg_len = max(seg_lengths[current_line_index], eps)
        local_distance = max(0.0, min(distance - current_offset, seg_len))
        t = local_distance / seg_len
        x = ln.start[0] + (ln.end[0] - ln.start[0]) * t
        y = ln.start[1] + (ln.end[1] - ln.start[1]) * t
        # 采样点的权重与 part_id 继承所在段
        result.append(MyPoint((x, y), ln.priority, part_id=ln.part_id))

    return result


def interpolate_per_line_v2(lines: list[MyLine]) -> list[MyPoint]:
    """
    v2 版采样: 对每段 MyLine 单独采样, 不看作整体路径。

    规则:
    - 若线段长度 < 3, 取 1 个点, 位于 1/2 处;
    - 若 3 <= 长度 <= 9, 取 2 个点, 位于 1/3 与 2/3 处;
    - 若 长度 > 9, 取 3 个点, 位于 1/4, 2/4, 3/4 处。

    采样点继承所在段的 priority 与 part_id。
    """
    result: list[MyPoint] = []
    eps = 1e-9
    for ln in lines:
        (x1, y1) = ln.start
        (x2, y2) = ln.end
        dx = x2 - x1
        dy = y2 - y1
        seg_len: float = (dx * dx + dy * dy) ** 0.5

        # 冗余保护: 极短线段/零长线段统一按中点
        if seg_len < eps or seg_len < 3:
            t_list = [0.5]
        elif seg_len <= 9:
            t_list = [1 / 3, 2 / 3]
        else:
            t_list = [1 / 4, 2 / 4, 3 / 4]

        for t in t_list:
            x = x1 + dx * t
            y = y1 + dy * t
            result.append(MyPoint((x, y), ln.priority, part_id=ln.part_id))

    return result
