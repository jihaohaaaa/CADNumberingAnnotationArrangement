from functools import total_ordering
from shapely.geometry import Point, LineString, Polygon
from shapely.ops import unary_union


def interpolate_whole_path(lines: list[LineString], step: float) -> list[Point]:
    """
    改进后 interpolate_whole_path, 解决边界点和索引越界问题。
    """
    result = []
    total_length = sum(line.length for line in lines)

    distances = []
    d = 0.0
    while d <= total_length:
        distances.append(d)
        d += step
    if distances[-1] < total_length:
        distances.append(total_length)  # 确保终点被采样

    current_line_index = 0
    current_line = lines[current_line_index]
    current_offset = 0.0  # 当前线段在整体路径的起点累计长度

    for distance in distances:
        # 移动到包含当前 distance 的线段
        while (
            current_line_index < len(lines)
            and current_offset + current_line.length < distance
        ):
            current_offset += current_line.length
            current_line_index += 1
            if current_line_index < len(lines):
                current_line = lines[current_line_index]
            else:
                # 超出路径长度, 提前返回结果
                return result

        local_distance = distance - current_offset
        pt = current_line.interpolate(local_distance)
        result.append(pt)

    return result


@total_ordering
class MyLine:
    start: tuple[float, float]
    end: tuple[float, float]
    geometry: LineString
    priority: float = 1.0  # 从 JSON 的 weight 映射过来(占位符)

    def __init__(
        self,
        start: tuple[float, float],
        end: tuple[float, float],
        priority: float = 1.0,
    ):
        self.start = start
        self.end = end
        self.geometry = LineString([start, end])
        self.priority = float(priority)

    def __lt__(self, other: "MyLine") -> bool:
        return self.geometry.length < other.geometry.length

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, MyLine):
            return NotImplemented
        return self.geometry.length == other.geometry.length


class ObstacleBox:
    def __init__(self, vertices: list[tuple[float, float]]):
        self.vertices = vertices
        self.geometry = Polygon(vertices)


class MyPoint:
    coordinate: tuple[float, float]
    geometry: Point
    priority: float = 1.0  # 从 JSON 的 weight 映射过来(占位符)

    def __init__(self, coordinate: tuple[float, float], priority: float = 1.0):
        self.coordinate = coordinate
        self.geometry = Point(coordinate)
        self.priority = float(priority)


class PartLineCandidate:
    def __init__(self, part_name: str, lines: list[MyLine]):
        self.part_name = part_name
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
        line_string_list: list[LineString] = [line.geometry for line in self.lines]
        points = interpolate_whole_path(line_string_list, step)
        # 采样点暂无权重来源，统一占位 1.0
        my_points = [MyPoint((pt.x, pt.y), 1.0) for pt in points]
        return PartPointCandidate(self.part_name, my_points)


class PartPointCandidate:
    def __init__(self, part_name: str, points: list[MyPoint]):
        self.part_name = part_name
        self.points = points


class Schema:
    def __init__(
        self,
        schema_name: str,
        obstacle_boxes: list[ObstacleBox],
        barrier_lines: list[MyLine],
        target_exterior: Polygon,
        line_candidates: list[PartLineCandidate],
        point_candidates: list[PartPointCandidate],
    ):
        self.schema_name = schema_name
        # 先设置 target_exterior，便于基于它过滤障碍框
        self.target_exterior = target_exterior
        # 仅保留与 target_exterior 相交的障碍框；完全在外部的剔除
        # 如需更严格的“完全在内部”可将 intersects 改为 within
        filtered_obs = [
            ob for ob in obstacle_boxes if ob.geometry.intersects(self.target_exterior)
        ]
        # 若大的障碍框包住了小的，则保留大的，剔除被包含的小的
        # 策略：按面积从大到小遍历，若当前被已保留集合中的任一覆盖(covers)，则跳过
        sorted_obs = sorted(filtered_obs, key=lambda ob: ob.geometry.area, reverse=True)
        kept: list[ObstacleBox] = []
        for ob in sorted_obs:
            if any(k.geometry.covers(ob.geometry) for k in kept):
                continue
            kept.append(ob)
        # 第三步：将相互距离小于等于 delta(含重叠/相邻)的障碍框合并为更大的一个
        delta = 1.0
        if kept:
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
                cluster = []
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
                    # Shapely 返回首尾相同点，取前四个顶点
                    verts = [(float(x), float(y)) for (x, y) in coords[:-1][:4]]
                if verts and len(verts) == 4:
                    merged_boxes.append(ObstacleBox(verts))
                else:
                    # 兜底：如果异常，回退为该簇的外包矩形(轴对齐)
                    minx, miny, maxx, maxy = map(float, uni.bounds)
                    fallback = [(minx, miny), (maxx, miny), (maxx, maxy), (minx, maxy)]
                    merged_boxes.append(ObstacleBox(fallback))

            self.obstacle_boxes = merged_boxes
        else:
            self.obstacle_boxes = kept
        self.barrier_lines = barrier_lines
        self.line_candidates = line_candidates
        self.point_candidates = point_candidates


class SchemaParseError(Exception):
    pass


def parse_schema(json_data: dict) -> "Schema":
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

    obstacle_boxes = []
    for box in json_data["obstacle_box"]:
        try:
            vertices = [xy(box["a"]), xy(box["b"]), xy(box["c"]), xy(box["d"])]
        except KeyError as e:
            raise SchemaParseError(f"ObstacleBox missing vertex key: {e}")
        obstacle_boxes.append(ObstacleBox(vertices))

    barrier_lines = []
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

    line_candidates = []
    for item in part_data.get("line_candidates", []):
        if "part_name" not in item or "lines" not in item:
            raise SchemaParseError(
                "line_candidates item missing 'part_name' or 'lines'"
            )
        lines = []
        for line in item["lines"]:
            # 新格式：{"lineSegment": {x1,y1,x2,y2}, "weight": number}
            if "lineSegment" in line:
                seg = line["lineSegment"]
                try:
                    start = (seg["x1"], seg["y1"])  # type: ignore
                    end = (seg["x2"], seg["y2"])  # type: ignore
                except KeyError as e:
                    raise SchemaParseError(
                        f"LineCandidate lineSegment missing coordinate key: {e}"
                    )
                priority = float(line.get("weight", 1.0))
                lines.append(MyLine(start, end, priority))
            else:
                # 旧格式：直接 x1,y1,x2,y2(无权重)
                try:
                    start = (line["x1"], line["y1"])  # type: ignore
                    end = (line["x2"], line["y2"])  # type: ignore
                except KeyError as e:
                    raise SchemaParseError(
                        f"LineCandidate line missing coordinate key: {e}"
                    )
                lines.append(MyLine(start, end, 1.0))
        line_candidates.append(PartLineCandidate(item["part_name"], lines))

    point_candidates = []
    for item in part_data.get("point_candidates", []):
        if "part_name" not in item or "points" not in item:
            raise SchemaParseError(
                "point_candidates item missing 'part_name' or 'points'"
            )
        points: list[MyPoint] = []
        for pt in item["points"]:
            # 新格式：{"coordinate": {x,y}, "weight": number}
            if "coordinate" in pt:
                coord = pt["coordinate"]
                try:
                    x, y = coord["x"], coord["y"]
                except KeyError as e:
                    raise SchemaParseError(
                        f"PointCandidate coordinate missing key: {e}"
                    )
                priority = float(pt.get("weight", 1.0))
                points.append(MyPoint((x, y), priority))
            else:
                # 旧格式：直接 {x,y}
                try:
                    x, y = xy(pt)
                except SchemaParseError as e:
                    raise SchemaParseError(f"PointCandidate point invalid: {e}")
                points.append(MyPoint((x, y), 1.0))
        point_candidates.append(PartPointCandidate(item["part_name"], points))

    return Schema(
        schema_name=json_data["schema_name"],
        obstacle_boxes=obstacle_boxes,
        barrier_lines=barrier_lines,
        target_exterior=target_exterior,
        line_candidates=line_candidates,
        point_candidates=point_candidates,
    )
