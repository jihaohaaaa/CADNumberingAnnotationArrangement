from functools import total_ordering
from shapely.geometry import Point, LineString, Polygon


def interpolate_whole_path(lines: list[LineString], step: float) -> list[Point]:
    """
    改进后 interpolate_whole_path，解决边界点和索引越界问题。
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
                # 超出路径长度，提前返回结果
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

    priority_level: (
        int  # 候选线的优先级, 数值越小优先级越高, 1为最高优先级, 正整数, 默认1
    )

    def __init__(
        self,
        start: tuple[float, float],
        end: tuple[float, float],
        priority_level: int = 1,
    ):
        self.start = start
        self.end = end
        self.geometry = LineString([start, end])
        self.priority_level = priority_level

    def __lt__(self, other: "MyLine") -> bool:
        # Compare by priority first (lower number = higher priority)
        if self.priority_level != other.priority_level:
            return self.priority_level < other.priority_level
        # If priorities are equal, compare by length (shorter is better)
        return self.geometry.length < other.geometry.length

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, MyLine):
            return NotImplemented
        return (self.priority_level == other.priority_level and 
                self.geometry.length == other.geometry.length)


class ObstacleBox:
    def __init__(self, vertices: list[tuple[float, float]]):
        self.vertices = vertices
        self.geometry = Polygon(vertices)


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
        return PartPointCandidate(self.part_name, points)


class PartPointCandidate:
    part_name: str
    points: list[Point]

    # 候选点的优先级, 数值越小优先级越高, 1为最高优先级, 正整数, 默认1
    priority_level: int

    def __init__(self, part_name: str, points: list[Point], priority_level: int = 1):
        self.part_name = part_name
        self.points = points
        self.priority_level = priority_level


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
        self.obstacle_boxes = obstacle_boxes
        self.barrier_lines = barrier_lines
        self.target_exterior = target_exterior
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
            try:
                start = (line["x1"], line["y1"])
                end = (line["x2"], line["y2"])
            except KeyError as e:
                raise SchemaParseError(
                    f"LineCandidate line missing coordinate key: {e}"
                )
            lines.append(MyLine(start, end))
        line_candidates.append(PartLineCandidate(item["part_name"], lines))

    point_candidates = []
    for item in part_data.get("point_candidates", []):
        if "part_name" not in item or "points" not in item:
            raise SchemaParseError(
                "point_candidates item missing 'part_name' or 'points'"
            )
        points = []
        for pt in item["points"]:
            try:
                points.append(Point(xy(pt)))
            except SchemaParseError as e:
                raise SchemaParseError(f"PointCandidate point invalid: {e}")
        point_candidates.append(PartPointCandidate(item["part_name"], points))

    return Schema(
        schema_name=json_data["schema_name"],
        obstacle_boxes=obstacle_boxes,
        barrier_lines=barrier_lines,
        target_exterior=target_exterior,
        line_candidates=line_candidates,
        point_candidates=point_candidates,
    )
