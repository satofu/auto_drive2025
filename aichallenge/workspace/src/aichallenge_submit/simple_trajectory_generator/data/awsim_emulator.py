import csv
import math
import os
import xml.etree.ElementTree as ET
from typing import List, Tuple

DT = 0.01
SIM_TIME = 420.0  # seconds
RADIUS = 1.0  # collision model radius [m]


def load_raceline(path: str) -> List[Tuple[float, float, float]]:
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        return [(float(r["x"]), float(r["y"]), float(r["speed"])) for r in reader]


def parse_reference(path: str) -> dict:
    tree = ET.parse(path)
    root = tree.getroot()
    vars_ = {e.get("name"): float(e.get("value")) for e in root.findall(".//let") if e.get("value")}
    params = {}
    for param in root.findall(".//param"):
        name = param.get("name")
        value = param.get("value")
        if value is None:
            continue
        if value.startswith("$(var "):
            vname = value[len("$(var "):-1]
            value = vars_.get(vname)
        try:
            params[name] = float(value)
        except (TypeError, ValueError):
            continue
    params.setdefault("wheel_base", 2.14)
    return params


def load_lanelet_polygons(path: str) -> List[List[Tuple[float, float]]]:
    tree = ET.parse(path)
    root = tree.getroot()
    nodes = {}
    for node in root.findall("node"):
        nid = node.get("id")
        x = y = None
        for tag in node.findall("tag"):
            if tag.get("k") == "local_x":
                x = float(tag.get("v"))
            elif tag.get("k") == "local_y":
                y = float(tag.get("v"))
        if x is not None and y is not None:
            nodes[nid] = (x, y)
    ways = {}
    for way in root.findall("way"):
        wid = way.get("id")
        refs = [nd.get("ref") for nd in way.findall("nd")]
        ways[wid] = refs
    polygons = []
    for rel in root.findall("relation"):
        if not any(tag.get("k") == "type" and tag.get("v") == "lanelet" for tag in rel.findall("tag")):
            continue
        left_ref = right_ref = None
        for mem in rel.findall("member"):
            role = mem.get("role")
            if role == "left":
                left_ref = ways.get(mem.get("ref"))
            elif role == "right":
                right_ref = ways.get(mem.get("ref"))
        if not left_ref or not right_ref:
            continue
        left_pts = [nodes[ref] for ref in left_ref if ref in nodes]
        right_pts = [nodes[ref] for ref in right_ref if ref in nodes]
        if left_pts and right_pts:
            polygons.append(left_pts + list(reversed(right_pts)))
    return polygons


def point_in_polygon(x: float, y: float, poly: List[Tuple[float, float]]) -> bool:
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-9) + xi):
            inside = not inside
        j = i
    return inside


def distance_to_segment(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
    dx, dy = x2 - x1, y2 - y1
    if dx == 0 and dy == 0:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
    projx, projy = x1 + t * dx, y1 + t * dy
    return math.hypot(px - projx, py - projy)


def is_outside_track(x: float, y: float, polygons: List[List[Tuple[float, float]]]) -> bool:
    for poly in polygons:
        if point_in_polygon(x, y, poly):
            min_d = min(
                distance_to_segment(
                    x,
                    y,
                    poly[i][0],
                    poly[i][1],
                    poly[(i + 1) % len(poly)][0],
                    poly[(i + 1) % len(poly)][1],
                )
                for i in range(len(poly))
            )
            return min_d < RADIUS
    return True


def nearest_point_index(path: List[Tuple[float, float, float]], x: float, y: float) -> int:
    best_idx = 0
    best_dist = float("inf")
    for i, (px, py, _) in enumerate(path):
        d = (px - x) ** 2 + (py - y) ** 2
        if d < best_dist:
            best_idx = i
            best_dist = d
    return best_idx


def simulate():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    raceline = load_raceline(os.path.join(base_dir, "raceline_awsim_30km_from_garage.csv"))
    params = parse_reference(os.path.join(base_dir, "../../aichallenge_submit_launch/launch/reference.launch.xml"))
    polygons = load_lanelet_polygons(os.path.join(base_dir, "../../aichallenge_submit_launch/map/wide_lanelet2_map.osm"))

    x, y, _ = raceline[0]
    x2, y2, _ = raceline[1]
    yaw = math.atan2(y2 - y, x2 - x)
    v = 0.0

    t = 0.0
    out_rows = [("time(s)", "x", "y", "is_outside_track")]
    steps = int(SIM_TIME / DT)
    for step in range(steps):
        idx = nearest_point_index(raceline, x, y)
        target_speed = raceline[idx][2]
        accel = params.get("speed_proportional_gain", 1.0) * (target_speed - v)
        lookahead_distance = (
            params.get("lookahead_gain", 0.0) * target_speed
            + params.get("lookahead_min_distance", 0.0)
        )
        rear_x = x - params["wheel_base"] / 2 * math.cos(yaw)
        rear_y = y - params["wheel_base"] / 2 * math.sin(yaw)
        look_idx = idx
        while True:
            look_idx = (look_idx + 1) % len(raceline)
            lx, ly, _ = raceline[look_idx]
            if math.hypot(lx - rear_x, ly - rear_y) >= lookahead_distance:
                break
        alpha = math.atan2(ly - rear_y, lx - rear_x) - yaw
        delta_raw = math.atan2(2.0 * params["wheel_base"] * math.sin(alpha), lookahead_distance)
        _steer_cmd = params.get("steering_tire_angle_gain", 1.0) * delta_raw
        v += accel * DT
        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
        yaw += v * math.tan(delta_raw) / params["wheel_base"] * DT
        t += DT
        if step % int(1.0 / DT) == 0:
            outside = is_outside_track(x, y, polygons)
            out_rows.append((f"{t:.0f}", f"{x:.3f}", f"{y:.3f}", str(outside)))
    with open(os.path.join(base_dir, "simulation_output.csv"), "w", newline="") as f:
        csv.writer(f).writerows(out_rows)


if __name__ == "__main__":
    simulate()
