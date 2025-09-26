from shapely.geometry import LineString
import math


def line_angle(line: LineString) -> float:
    """Return the angle in degrees relative to horizontal (range -180..180]."""
    start, end = line.coords[0], line.coords[-1]
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    angle_rad = math.atan2(dy, dx)
    return math.degrees(angle_rad)


def is_angle_forbidden(line: LineString, tolerant_angle: float = 15) -> bool:
    """Return True if the line is (near) horizontal or vertical within tolerance."""
    ang = abs(line_angle(line)) % 180
    # near horizontal: ang ~ 0 or ~ 180; near vertical: ang ~ 90
    if ang <= tolerant_angle:
        return True
    if abs(90 - ang) <= tolerant_angle:
        return True
    if ang >= 180 - tolerant_angle:
        return True
    return False


def iter_check_line(
    line: LineString, existing_lines: list[LineString], proximity_threshold=1
) -> bool:
    """
    Check if a line is valid for connection during iteration:
    - Does not intersect with existing lines
    - Do stay away from existing lines by a proximity threshold

    Args:
        line (LineString): The line to be checked.
        existing_lines (List[LineString]): List of already used lines.

    Returns:
        bool: True if the line is valid, False otherwise.
    """

    # 迭代运行阶段: 与已存在线段的整体距离需大于阈值（改为线段-线段距离而非端点距离）

    for other in existing_lines:
        if line.distance(other) <= proximity_threshold:
            return False

    return True
