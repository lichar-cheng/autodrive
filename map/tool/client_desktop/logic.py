from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any


SCAN_FUSION_PRESETS: dict[str, dict[str, Any]] = {
    "sim_clean": {
        "preset": "sim_clean",
        "voxel_size": 0.12,
        "occupied_min_hits": 3,
        "occupied_over_free_ratio": 0.90,
        "turn_skip_wz": 0.35,
        "skip_turn_frames": True,
    },
    "indoor_balanced": {
        "preset": "indoor_balanced",
        "voxel_size": 0.08,
        "occupied_min_hits": 2,
        "occupied_over_free_ratio": 0.75,
        "turn_skip_wz": 0.45,
        "skip_turn_frames": True,
    },
    "indoor_sensitive": {
        "preset": "indoor_sensitive",
        "voxel_size": 0.06,
        "occupied_min_hits": 1,
        "occupied_over_free_ratio": 0.55,
        "turn_skip_wz": 0.60,
        "skip_turn_frames": False,
    },
    "warehouse_sparse": {
        "preset": "warehouse_sparse",
        "voxel_size": 0.10,
        "occupied_min_hits": 2,
        "occupied_over_free_ratio": 0.65,
        "turn_skip_wz": 0.50,
        "skip_turn_frames": True,
    },
}


@dataclass
class Point:
    x: float
    y: float
    name: str = ""
    yaw: float = 0.0
    lat: float | None = None
    lon: float | None = None
    poi_id: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "x": float(self.x),
            "y": float(self.y),
            "name": self.name,
            "yaw": float(self.yaw),
            "lat": None if self.lat is None else float(self.lat),
            "lon": None if self.lon is None else float(self.lon),
            "poi_id": self.poi_id,
        }


@dataclass
class Segment:
    id: str
    start: Point
    end: Point
    source: str = "free"
    clearance: float = 0.0
    points: list[Point] | None = None


def parse_optional_float(value: Any) -> float | None:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def resolve_scan_fusion_config(preset: str | None = None, overrides: dict[str, Any] | None = None) -> dict[str, Any]:
    selected = str(preset or "indoor_balanced")
    base = SCAN_FUSION_PRESETS.get(selected, SCAN_FUSION_PRESETS["indoor_balanced"])
    config = dict(base)
    config["preset"] = selected if selected in SCAN_FUSION_PRESETS else "indoor_balanced"
    if overrides:
        for key, value in overrides.items():
            if value is None:
                continue
            config[key] = value
    config["voxel_size"] = max(0.02, float(config["voxel_size"]))
    config["occupied_min_hits"] = max(1, int(config["occupied_min_hits"]))
    config["occupied_over_free_ratio"] = max(0.0, float(config["occupied_over_free_ratio"]))
    config["turn_skip_wz"] = max(0.0, float(config["turn_skip_wz"]))
    config["skip_turn_frames"] = bool(config["skip_turn_frames"])
    return config


def should_skip_scan_by_turn(wz: float, keyframe: bool, config: dict[str, Any]) -> bool:
    if keyframe or not bool(config.get("skip_turn_frames", True)):
        return False
    return abs(float(wz)) >= float(config.get("turn_skip_wz", 0.0))


def is_occupied_scan_cell(cell: dict[str, Any], free: dict[str, Any] | None, config: dict[str, Any]) -> bool:
    hits = int(cell.get("hits", 0) or 0)
    if hits < int(config.get("occupied_min_hits", 1)):
        return False
    free_hits = int((free or {}).get("hits", 0) or 0)
    return hits >= free_hits * float(config.get("occupied_over_free_ratio", 0.0))


def build_scan_fusion_metadata(config: dict[str, Any]) -> dict[str, Any]:
    resolved = resolve_scan_fusion_config(str(config.get("preset", "indoor_balanced")), config)
    return {
        "preset": str(resolved["preset"]),
        "voxel_size": float(resolved["voxel_size"]),
        "occupied_min_hits": int(resolved["occupied_min_hits"]),
        "occupied_over_free_ratio": float(resolved["occupied_over_free_ratio"]),
        "turn_skip_wz": float(resolved["turn_skip_wz"]),
        "skip_turn_frames": bool(resolved["skip_turn_frames"]),
    }


def extract_scan_fusion_config(manifest: dict[str, Any], default_preset: str = "indoor_balanced") -> dict[str, Any]:
    overrides: dict[str, Any] = {}
    scan_fusion = manifest.get("scan_fusion")
    if isinstance(scan_fusion, dict):
        overrides.update(scan_fusion)
    notes = manifest.get("notes")
    if isinstance(notes, str) and notes.strip():
        try:
            parsed_notes = json.loads(notes)
        except Exception:  # noqa: BLE001
            parsed_notes = None
        if isinstance(parsed_notes, dict) and "voxelSize" in parsed_notes and "voxel_size" not in overrides:
            overrides["voxel_size"] = parsed_notes["voxelSize"]
    preset = str(overrides.get("preset") or default_preset)
    return resolve_scan_fusion_config(preset, overrides)


def parse_batch_poi_text(text: str) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    errors: list[int] = []
    for index, line in enumerate(text.splitlines(), start=1):
        raw = line.strip()
        if not raw:
            continue
        parts = [part.strip() for part in raw.split(",")]
        name = parts[0]
        if not name:
            errors.append(index)
            continue
        lon = parse_optional_float(parts[1]) if len(parts) >= 3 else None
        lat = parse_optional_float(parts[2]) if len(parts) >= 3 else None
        yaw = parse_optional_float(parts[3]) if len(parts) >= 4 else None
        if len(parts) >= 3 and (lon is None or lat is None):
            errors.append(index)
            continue
        if len(parts) >= 4 and yaw is None:
            errors.append(index)
            continue
        rows.append({"name": name, "lon": lon, "lat": lat, "yaw": yaw})
    geo_count = sum(1 for row in rows if row["lat"] is not None and row["lon"] is not None)
    if geo_count in (1, 2):
        raise ValueError("Batch mode requires at least 3 POI with lon,lat when any geo is provided")
    if errors:
        raise ValueError(f"Invalid batch POI input: {', '.join(f'line {item}' for item in errors)}")
    return rows


def solve_linear3(matrix: list[list[float]], values: list[float]) -> list[float]:
    a = [row[:] + [values[index]] for index, row in enumerate(matrix)]
    for pivot in range(3):
        pivot_row = max(range(pivot, 3), key=lambda row: abs(a[row][pivot]))
        if abs(a[pivot_row][pivot]) < 1e-9:
            raise ValueError("Geo anchor points are degenerate and cannot define a transform")
        if pivot_row != pivot:
            a[pivot], a[pivot_row] = a[pivot_row], a[pivot]
        factor = a[pivot][pivot]
        for col in range(pivot, 4):
            a[pivot][col] /= factor
        for row in range(3):
            if row == pivot:
                continue
            factor = a[row][pivot]
            for col in range(pivot, 4):
                a[row][col] -= factor * a[pivot][col]
    return [a[index][3] for index in range(3)]


def infer_missing_geo_points(points: list[Point]) -> None:
    anchors = [point for point in points if point.lat is not None and point.lon is not None]
    if not anchors:
        return
    if len(anchors) < 3:
        raise ValueError("At least 3 POI with lon,lat are required when any geo is provided")
    solved = False
    last_error: ValueError | None = None
    lon_coeffs: list[float] = []
    lat_coeffs: list[float] = []
    for i in range(len(anchors) - 2):
        for j in range(i + 1, len(anchors) - 1):
            for k in range(j + 1, len(anchors)):
                chosen = [anchors[i], anchors[j], anchors[k]]
                matrix = [[anchor.x, anchor.y, 1.0] for anchor in chosen]
                lon_values = [float(anchor.lon) for anchor in chosen if anchor.lon is not None]
                lat_values = [float(anchor.lat) for anchor in chosen if anchor.lat is not None]
                try:
                    lon_coeffs = solve_linear3(matrix, lon_values)
                    lat_coeffs = solve_linear3(matrix, lat_values)
                    solved = True
                    break
                except ValueError as exc:
                    last_error = exc
            if solved:
                break
        if solved:
            break
    if not solved:
        raise last_error or ValueError("Geo anchor points are degenerate and cannot define a transform")
    for point in points:
        if point.lon is None:
            point.lon = lon_coeffs[0] * point.x + lon_coeffs[1] * point.y + lon_coeffs[2]
        if point.lat is None:
            point.lat = lat_coeffs[0] * point.x + lat_coeffs[1] * point.y + lat_coeffs[2]


def build_poi_copy_text(points: list[Point]) -> str:
    lines = []
    for poi in points:
        lines.append(
            ",".join(
                [
                    poi.name,
                    f"{float(poi.x):.3f}",
                    f"{float(poi.y):.3f}",
                    f"{float(poi.yaw):.3f}",
                    "" if poi.lat is None else f"{float(poi.lat):.6f}",
                    "" if poi.lon is None else f"{float(poi.lon):.6f}",
                ]
            )
        )
    return "\n".join(lines)


def distance_between(a: Point, b: Point) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def solve_nearest_loop(points: list[Point]) -> list[Point]:
    if not points:
        return []
    ordered = sorted(points, key=lambda item: (item.x, item.y))
    route = [ordered[0]]
    remaining = ordered[1:]
    while remaining:
        last = route[-1]
        best_index = min(range(len(remaining)), key=lambda idx: distance_between(last, remaining[idx]))
        route.append(remaining.pop(best_index))
    return route


def total_loop_distance(route: list[Point]) -> float:
    if len(route) < 2:
        return 0.0
    total = 0.0
    for index in range(len(route)):
        total += distance_between(route[index], route[(index + 1) % len(route)])
    return total


def optimize_loop_with_two_opt(route: list[Point]) -> list[Point]:
    if len(route) < 4:
        return list(route)
    best = list(route)
    improved = True
    while improved:
        improved = False
        for i in range(1, len(best) - 2):
            for j in range(i + 1, len(best) - 1):
                candidate = best[:i] + list(reversed(best[i : j + 1])) + best[j + 1 :]
                if total_loop_distance(candidate) + 1e-6 < total_loop_distance(best):
                    best = candidate
                    improved = True
    return best


def world_to_cell(x: float, y: float, voxel_size: float) -> tuple[int, int]:
    return round(x / voxel_size), round(y / voxel_size)


def cell_key(ix: int, iy: int) -> str:
    return f"{ix}:{iy}"


def simplify_cell_path(path: list[tuple[int, int]]) -> list[tuple[int, int]]:
    if len(path) <= 2:
        return path
    simplified = [path[0]]
    prev_dx = path[1][0] - path[0][0]
    prev_dy = path[1][1] - path[0][1]
    for index in range(1, len(path) - 1):
        dx = path[index + 1][0] - path[index][0]
        dy = path[index + 1][1] - path[index][1]
        if dx != prev_dx or dy != prev_dy:
            simplified.append(path[index])
            prev_dx = dx
            prev_dy = dy
    simplified.append(path[-1])
    return simplified


def rasterize_line_cells(start_ix: int, start_iy: int, end_ix: int, end_iy: int) -> list[tuple[int, int]]:
    points: list[tuple[int, int]] = []

    def push(ix: int, iy: int) -> None:
        if not points or points[-1] != (ix, iy):
            points.append((ix, iy))

    x = start_ix
    y = start_iy
    dx = end_ix - start_ix
    dy = end_iy - start_iy
    nx = abs(dx)
    ny = abs(dy)
    sign_x = 0 if dx == 0 else (1 if dx > 0 else -1)
    sign_y = 0 if dy == 0 else (1 if dy > 0 else -1)
    ix = 0
    iy = 0
    push(x, y)
    while ix < nx or iy < ny:
        next_horizontal = (0.5 + ix) / max(nx, 1)
        next_vertical = (0.5 + iy) / max(ny, 1)
        if next_horizontal < next_vertical:
            x += sign_x
            ix += 1
        elif next_vertical < next_horizontal:
            y += sign_y
            iy += 1
        else:
            x += sign_x
            y += sign_y
            ix += 1
            iy += 1
        push(x, y)
    return points


def is_cell_blocked(
    ix: int,
    iy: int,
    occupied_cells: dict[tuple[int, int], Any],
    clearance_cells: int = 0,
    allow_start_key: str = "",
    allow_end_key: str = "",
) -> bool:
    for dx in range(-clearance_cells, clearance_cells + 1):
        for dy in range(-clearance_cells, clearance_cells + 1):
            if math.hypot(dx, dy) > clearance_cells + 1e-6:
                continue
            key = cell_key(ix + dx, iy + dy)
            if key == allow_start_key or key == allow_end_key:
                continue
            if (ix + dx, iy + dy) in occupied_cells:
                return True
    return False


def plan_path_cells(
    start: Point,
    end: Point,
    voxel_size: float,
    occupied_cells: dict[tuple[int, int], Any],
    clearance: float,
) -> list[tuple[int, int]] | None:
    start_ix, start_iy = world_to_cell(start.x, start.y, voxel_size)
    end_ix, end_iy = world_to_cell(end.x, end.y, voxel_size)
    start_key = cell_key(start_ix, start_iy)
    end_key = cell_key(end_ix, end_iy)
    clearance_cells = max(0, math.ceil(clearance / voxel_size))
    occupied_keys = list(occupied_cells.keys())
    occupied_min_x = min([item[0] for item in occupied_keys], default=min(start_ix, end_ix))
    occupied_max_x = max([item[0] for item in occupied_keys], default=max(start_ix, end_ix))
    occupied_min_y = min([item[1] for item in occupied_keys], default=min(start_iy, end_iy))
    occupied_max_y = max([item[1] for item in occupied_keys], default=max(start_iy, end_iy))

    if not is_cell_blocked(start_ix, start_iy, occupied_cells, clearance_cells, start_key, end_key) and not is_cell_blocked(
        end_ix, end_iy, occupied_cells, clearance_cells, start_key, end_key
    ):
        line_path = rasterize_line_cells(start_ix, start_iy, end_ix, end_iy)
        if not any(is_cell_blocked(ix, iy, occupied_cells, clearance_cells, start_key, end_key) for ix, iy in line_path):
            return simplify_cell_path(line_path)

    min_x = min(start_ix, end_ix, occupied_min_x) - clearance_cells - 20
    max_x = max(start_ix, end_ix, occupied_max_x) + clearance_cells + 20
    min_y = min(start_iy, end_iy, occupied_min_y) - clearance_cells - 20
    max_y = max(start_iy, end_iy, occupied_max_y) + clearance_cells + 20
    open_nodes = [{"ix": start_ix, "iy": start_iy, "key": start_key, "g": 0.0, "f": math.hypot(end_ix - start_ix, end_iy - start_iy)}]
    best = {start_key: open_nodes[0]}
    came_from: dict[str, str] = {}
    closed: set[str] = set()
    neighbors = [
        (1, 0, 1.0),
        (-1, 0, 1.0),
        (0, 1, 1.0),
        (0, -1, 1.0),
        (1, 1, math.sqrt(2)),
        (1, -1, math.sqrt(2)),
        (-1, 1, math.sqrt(2)),
        (-1, -1, math.sqrt(2)),
    ]
    while open_nodes:
        open_nodes.sort(key=lambda node: node["f"])
        current = open_nodes.pop(0)
        if current["key"] in closed:
            continue
        if current["key"] == end_key:
            path = [(end_ix, end_iy)]
            cursor = end_key
            while cursor in came_from:
                cursor = came_from[cursor]
                ix_text, iy_text = cursor.split(":")
                path.append((int(ix_text), int(iy_text)))
            path.reverse()
            return simplify_cell_path(path)
        closed.add(current["key"])
        for dx, dy, cost in neighbors:
            next_ix = current["ix"] + dx
            next_iy = current["iy"] + dy
            if next_ix < min_x or next_ix > max_x or next_iy < min_y or next_iy > max_y:
                continue
            next_key = cell_key(next_ix, next_iy)
            if next_key in closed:
                continue
            if is_cell_blocked(next_ix, next_iy, occupied_cells, clearance_cells, start_key, end_key):
                continue
            if dx != 0 and dy != 0:
                if is_cell_blocked(current["ix"] + dx, current["iy"], occupied_cells, clearance_cells, start_key, end_key):
                    continue
                if is_cell_blocked(current["ix"], current["iy"] + dy, occupied_cells, clearance_cells, start_key, end_key):
                    continue
            g_cost = current["g"] + cost
            prev = best.get(next_key)
            if prev and g_cost >= prev["g"]:
                continue
            node = {
                "ix": next_ix,
                "iy": next_iy,
                "key": next_key,
                "g": g_cost,
                "f": g_cost + math.hypot(end_ix - next_ix, end_iy - next_iy),
            }
            came_from[next_key] = current["key"]
            best[next_key] = node
            open_nodes.append(node)
    return None


def plan_path_points(
    start: Point,
    end: Point,
    voxel_size: float,
    occupied_cells: dict[tuple[int, int], Any],
    clearance: float,
) -> list[Point]:
    path_cells = plan_path_cells(start, end, voxel_size, occupied_cells, clearance)
    if not path_cells:
        raise ValueError(f"No obstacle-free path found from ({start.x:.2f}, {start.y:.2f}) to ({end.x:.2f}, {end.y:.2f}) with clearance {clearance:.2f} m")
    points: list[Point] = []
    for index, (ix, iy) in enumerate(path_cells):
        if index == 0:
            points.append(Point(**start.to_dict()))
        elif index == len(path_cells) - 1:
            points.append(Point(**end.to_dict()))
        else:
            points.append(Point(x=ix * voxel_size, y=iy * voxel_size))
    return points


def build_segment(
    segment_id: str,
    start: Point,
    end: Point,
    voxel_size: float,
    occupied_cells: dict[tuple[int, int], Any],
    clearance: float,
    source: str,
) -> Segment:
    return Segment(
        id=segment_id,
        start=Point(**start.to_dict()),
        end=Point(**end.to_dict()),
        source=source,
        clearance=clearance,
        points=plan_path_points(start, end, voxel_size, occupied_cells, clearance),
    )


def build_auto_loop_segments(points: list[Point], voxel_size: float, occupied_cells: dict[tuple[int, int], Any], clearance: float = 0.0) -> list[Segment]:
    route = optimize_loop_with_two_opt(solve_nearest_loop(points))
    if len(route) < 2:
        return []
    segments: list[Segment] = []
    next_id = 1
    for index in range(len(route) - 1):
        segments.append(build_segment(f"seg-{next_id}", route[index], route[index + 1], voxel_size, occupied_cells, clearance, "auto"))
        next_id += 1
    if len(route) > 2:
        segments.append(build_segment(f"seg-{next_id}", route[-1], route[0], voxel_size, occupied_cells, clearance, "auto"))
    return segments


def path_validation_tolerance(voxel_size: float) -> float:
    return max(0.15, voxel_size * 1.5)


def resolve_validation_node(point: Point, clusters: list[dict[str, Any]], tolerance: float) -> dict[str, Any]:
    poi_key = f"poi:{point.poi_id}" if point.poi_id else ""
    if poi_key:
        for cluster in clusters:
            if cluster["poi_key"] == poi_key:
                return cluster
    for cluster in clusters:
        if poi_key or cluster["poi_key"]:
            continue
        if math.hypot(cluster["x"] - point.x, cluster["y"] - point.y) <= tolerance:
            return cluster
    cluster = {"id": poi_key or f"node:{len(clusters) + 1}", "poi_key": poi_key, "x": point.x, "y": point.y}
    clusters.append(cluster)
    return cluster


def compute_path_closed_loop_validation(path_segments: list[dict[str, Any]], voxel_size: float) -> dict[str, Any]:
    invalid_ids: set[str] = set()
    if len(path_segments) < 3:
        return {
            "checked": True,
            "ok": False,
            "invalid_ids": {item["id"] for item in path_segments},
            "message": "Closed-loop check failed: at least 3 path segments are required.",
        }
    tolerance = path_validation_tolerance(voxel_size)
    clusters: list[dict[str, Any]] = []
    endpoint_map: dict[str, list[str]] = {}
    adjacency: dict[str, set[str]] = {}
    for segment in path_segments:
        start = segment["start"]
        end = segment["end"]
        start_node = resolve_validation_node(start, clusters, tolerance)
        end_node = resolve_validation_node(end, clusters, tolerance)
        endpoint_map.setdefault(start_node["id"], []).append(segment["id"])
        endpoint_map.setdefault(end_node["id"], []).append(segment["id"])
        adjacency.setdefault(start_node["id"], set()).add(end_node["id"])
        adjacency.setdefault(end_node["id"], set()).add(start_node["id"])
    bad_nodes = 0
    for segment_ids in endpoint_map.values():
        if len(segment_ids) != 2:
            bad_nodes += 1
            invalid_ids.update(segment_ids)
    visited: set[str] = set()
    components = 0
    for node in list(adjacency.keys()):
        if node in visited:
            continue
        components += 1
        stack = [node]
        while stack:
            current = stack.pop()
            if current in visited:
                continue
            visited.add(current)
            stack.extend(next_node for next_node in adjacency.get(current, set()) if next_node not in visited)
    if components != 1:
        invalid_ids.update(segment["id"] for segment in path_segments)
    if not invalid_ids:
        message = "Closed-loop check passed."
    else:
        parts = []
        if bad_nodes:
            parts.append(f"{bad_nodes} endpoint(s) do not have degree 2")
        if components != 1:
            parts.append(f"path is split into {components} disconnected component(s)")
        message = f"Closed-loop check failed: {'; '.join(parts)}."
    return {"checked": True, "ok": not invalid_ids, "invalid_ids": invalid_ids, "message": message}
