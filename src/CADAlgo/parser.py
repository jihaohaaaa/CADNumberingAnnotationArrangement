from shapely.geometry import Point, LineString, Polygon
import json


class MyLine:
    def __init__(self, start: tuple[float, float], end: tuple[float, float]):
        self.start = start
        self.end = end
        self.geometry = LineString([start, end])


class ObstacleBox:
    def __init__(self, vertices: list[tuple[float, float]]):
        self.vertices = vertices
        self.geometry = Polygon(vertices)


class PartLineCandidate:
    def __init__(self, part_name: str, lines: list[MyLine]):
        self.part_name = part_name
        self.lines = lines


class PartPointCandidate:
    def __init__(self, part_name: str, points: list[Point]):
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
        self.obstacle_boxes = obstacle_boxes
        self.barrier_lines = barrier_lines
        self.target_exterior = target_exterior
        self.line_candidates = line_candidates
        self.point_candidates = point_candidates


def parse_schema(json_data: dict) -> Schema:
    def xy(p):
        return (p["x"], p["y"])

    obstacle_boxes = [
        ObstacleBox(
            [
                xy(box["a"]),
                xy(box["b"]),
                xy(box["c"]),
                xy(box["d"]),
            ]
        )
        for box in json_data.get("obstacle_box", [])
    ]

    barrier_lines = [
        MyLine((line["x1"], line["y1"]), (line["x2"], line["y2"]))
        for line in json_data.get("barrier_line", [])
    ]

    target_exterior = Polygon([xy(pt) for pt in json_data["target_exterior"]])

    line_candidates = [
        PartLineCandidate(
            part_name=item["part_name"],
            lines=[
                MyLine((line["x1"], line["y1"]), (line["x2"], line["y2"]))
                for line in item.get("lines", [])
            ],
        )
        for item in json_data.get("part", {}).get("line_candidates", [])
    ]

    point_candidates = [
        PartPointCandidate(
            part_name=item["part_name"],
            points=[Point(xy(pt)) for pt in item.get("points", [])],
        )
        for item in json_data.get("part", {}).get("point_candidates", [])
    ]

    return Schema(
        schema_name=json_data["schema_name"],
        obstacle_boxes=obstacle_boxes,
        barrier_lines=barrier_lines,
        target_exterior=target_exterior,
        line_candidates=line_candidates,
        point_candidates=point_candidates,
    )
