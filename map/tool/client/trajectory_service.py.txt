from __future__ import annotations

import json
import math
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any


@dataclass
class Point:
    x: float
    y: float
    lat: float | None = None
    lon: float | None = None
    poi_id: str | None = None
    name: str | None = None

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "Point":
        return cls(
            x=float(payload["x"]),
            y=float(payload["y"]),
            lat=_optional_float(payload.get("lat")),
            lon=_optional_float(payload.get("lon")),
            poi_id=payload.get("poiId") or payload.get("poi_id"),
            name=payload.get("name"),
        )

    def to_node(self) -> dict[str, Any]:
        return {
            "x": self.x,
            "y": self.y,
            "lat": self.lat,
            "lon": self.lon,
        }


@dataclass
class Segment:
    start: Point
    end: Point
    geometry: str = "line"
    curve_offset: float = 1.2
    source: str = "service"

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "Segment":
        return cls(
            start=Point.from_dict(payload["start"]),
            end=Point.from_dict(payload["end"]),
            geometry=str(payload.get("geometry", "line")),
            curve_offset=float(payload.get("curveOffset", payload.get("curve_offset", 1.2))),
            source=str(payload.get("source", "service")),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "start": self.start.to_node(),
            "end": self.end.to_node(),
            "geometry": self.geometry,
            "curveOffset": self.curve_offset,
            "source": self.source,
        }


def _optional_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _distance(a: Point, b: Point) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def _total_loop_distance(route: list[Point]) -> float:
    if len(route) < 2:
        return 0.0
    total = 0.0
    for index, point in enumerate(route):
        total += _distance(point, route[(index + 1) % len(route)])
    return total


def solve_nearest_loop(points: list[Point]) -> list[Point]:
    if not points:
        return []
    ordered = sorted(points, key=lambda item: (item.x, item.y))
    route = [ordered[0]]
    remaining = ordered[1:]
    while remaining:
        last = route[-1]
        best_index = min(range(len(remaining)), key=lambda idx: _distance(last, remaining[idx]))
        route.append(remaining.pop(best_index))
    return route


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
                if _total_loop_distance(candidate) + 1e-6 < _total_loop_distance(best):
                    best = candidate
                    improved = True
    return best


def build_auto_loop_segments(points: list[Point]) -> list[Segment]:
    route = optimize_loop_with_two_opt(solve_nearest_loop(points))
    if len(route) < 2:
        return []
    segments: list[Segment] = []
    for index in range(len(route) - 1):
        segments.append(Segment(start=route[index], end=route[index + 1], geometry="line", source="auto"))
    if len(route) > 2:
        segments.append(Segment(start=route[-1], end=route[0], geometry="line", source="auto"))
    return segments


def sample_segment(segment: Segment, min_steps: int = 12, max_steps: int = 48) -> list[dict[str, Any]]:
    if segment.geometry != "curve":
        return [segment.start.to_node(), segment.end.to_node()]
    dx = segment.end.x - segment.start.x
    dy = segment.end.y - segment.start.y
    length = math.hypot(dx, dy) or 1.0
    mid_x = (segment.start.x + segment.end.x) / 2
    mid_y = (segment.start.y + segment.end.y) / 2
    normal_x = -dy / length
    normal_y = dx / length
    control_x = mid_x + normal_x * segment.curve_offset
    control_y = mid_y + normal_y * segment.curve_offset
    steps = max(min_steps, min(max_steps, round(length * 6)))
    nodes: list[dict[str, Any]] = []
    for step in range(steps + 1):
        t = step / steps
        omt = 1 - t
        nodes.append(
            {
                "x": omt * omt * segment.start.x + 2 * omt * t * control_x + t * t * segment.end.x,
                "y": omt * omt * segment.start.y + 2 * omt * t * control_y + t * t * segment.end.y,
                "lat": None,
                "lon": None,
            }
        )
    nodes[0]["lat"] = segment.start.lat
    nodes[0]["lon"] = segment.start.lon
    nodes[-1]["lat"] = segment.end.lat
    nodes[-1]["lon"] = segment.end.lon
    return nodes


def flatten_segments(segments: list[Segment]) -> list[dict[str, Any]]:
    nodes: list[dict[str, Any]] = []
    for segment in segments:
        nodes.extend(sample_segment(segment))
    return nodes


class TrajectoryServiceHandler(BaseHTTPRequestHandler):
    server_version = "TrajectoryService/1.0"

    def do_GET(self) -> None:
        if self.path == "/health":
            self._send_json(200, {"ok": True, "service": "trajectory"})
            return
        self._send_json(404, {"ok": False, "error": "not found"})

    def do_POST(self) -> None:
        if self.path == "/trajectory/auto-loop":
            payload = self._read_json()
            points = [Point.from_dict(item) for item in payload.get("poi", [])]
            segments = build_auto_loop_segments(points)
            self._send_json(
                200,
                {
                    "ok": True,
                    "segments": [segment.to_dict() for segment in segments],
                    "nodes": flatten_segments(segments),
                },
            )
            return

        if self.path == "/trajectory/segment/sample":
            payload = self._read_json()
            segment = Segment.from_dict(payload["segment"])
            self._send_json(200, {"ok": True, "nodes": sample_segment(segment)})
            return

        if self.path == "/trajectory/flatten":
            payload = self._read_json()
            segments = [Segment.from_dict(item) for item in payload.get("segments", [])]
            self._send_json(200, {"ok": True, "nodes": flatten_segments(segments)})
            return

        self._send_json(404, {"ok": False, "error": "not found"})

    def log_message(self, format: str, *args: Any) -> None:
        return

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length) if length > 0 else b"{}"
        return json.loads(raw.decode("utf-8"))

    def _send_json(self, status: int, payload: dict[str, Any]) -> None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.end_headers()
        self.wfile.write(data)

    def do_OPTIONS(self) -> None:
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.end_headers()


def run(host: str = "0.0.0.0", port: int = 8091) -> None:
    server = ThreadingHTTPServer((host, port), TrajectoryServiceHandler)
    print(f"Trajectory service listening at http://{host}:{port}")
    server.serve_forever()


if __name__ == "__main__":
    run()
