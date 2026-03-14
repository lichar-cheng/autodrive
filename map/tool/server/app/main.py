from __future__ import annotations

import asyncio
import hashlib
import json
import logging
import time
from collections import defaultdict
from pathlib import Path
from typing import Any

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from .config import CONFIG
from .models import AddPoiRequest, LoadMapRequest, MoveCommand, PlanPathRequest, SaveMapRequest
from .ros_bridge import RosRuntime, detect_ros
from .simulator import Simulator
from .stcm_codec import load_stcm, save_stcm
from .topic_bus import TopicBus


logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(levelname)s | %(name)s | %(message)s")
logger = logging.getLogger("autodrive.server")

app = FastAPI(title="AutoDrive Mapping Server", version="0.6.0")
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

bus = TopicBus(queue_size=CONFIG.ws_queue_size)
sim = Simulator(bus, rate_hz=CONFIG.sim_rate_hz, points_per_scan=CONFIG.lidar_points_per_scan)
ros: RosRuntime = RosRuntime(enabled=False, reason="ROS runtime not initialized")
map_dir = Path("data/maps")
latest_points: list[tuple[float, float, float]] = []
seq_by_topic: dict[str, int] = defaultdict(int)
ws_clients: set[int] = set()


STREAM_TOPICS = [
    "/robot/pose",
    "/robot/gps",
    "/chassis/odom",
    "/chassis/status",
    "/lidar/front",
    "/lidar/rear",
    "/camera/1/compressed",
    "/camera/2/compressed",
    "/camera/3/compressed",
    "/camera/4/compressed",
    "/map/grid",
]

# 高频传感器流控策略：保留关键帧 + 稀疏非关键帧。
TOPIC_MIN_INTERVAL_SEC = {
    "/camera/1/compressed": 0.2,
    "/camera/2/compressed": 0.2,
    "/camera/3/compressed": 0.2,
    "/camera/4/compressed": 0.2,
}
LIDAR_MAX_WS_POINTS = 1200
LIDAR_KEYFRAME_INTERVAL_SEC = 1.0

QUEUE_NEAR_CAPACITY_RATIO = 0.8
QUEUE_WARN_INTERVAL_SEC = 5.0
CLIENT_IDLE_TIMEOUT_SEC = 20.0

SERVER_RUNTIME = {
    "ws_overflow_total": 0,
    "ws_near_capacity_total": 0,
    "ws_last_warn_at": 0.0,
    "active_ws_connections_peak": 0,
    "forced_disconnect_total": 0,
}


SCAN_SESSION = {
    "active": False,
    "started_at": 0.0,
    "stopped_at": 0.0,
    "voxel_size": 0.12,
    "front_frames": 0,
    "rear_frames": 0,
    "raw_points": 0,
    "accumulated": {},
}


def _checksum(topic: str, stamp: float, seq: int, payload: dict) -> str:
    payload_json = json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    raw = f"{topic}|{stamp:.6f}|{seq}|{payload_json}".encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def _pack_message(message: dict) -> dict:
    topic = message["topic"]
    stamp = float(message["stamp"])
    payload = message["payload"]
    seq_by_topic[topic] += 1
    seq = seq_by_topic[topic]
    return {
        "topic": topic,
        "stamp": stamp,
        "server_time_ms": int(time.time() * 1000),
        "seq": seq,
        "payload": payload,
        "checksum": _checksum(topic, stamp, seq, payload),
    }


def _thin_points(points: list[list[float]] | list[tuple[float, float, float]], limit: int) -> list[list[float]]:
    if len(points) <= limit:
        return [list(p) for p in points]
    step = max(1, len(points) // limit)
    sampled = points[::step][:limit]
    return [list(p) for p in sampled]


def _reset_scan_session(voxel_size: float | None = None, keep_points: bool = False) -> None:
    SCAN_SESSION["active"] = False
    SCAN_SESSION["started_at"] = 0.0
    SCAN_SESSION["stopped_at"] = 0.0
    SCAN_SESSION["front_frames"] = 0
    SCAN_SESSION["rear_frames"] = 0
    SCAN_SESSION["raw_points"] = 0
    if voxel_size is not None:
        SCAN_SESSION["voxel_size"] = max(0.02, float(voxel_size))
    if not keep_points:
        SCAN_SESSION["accumulated"] = {}


def _scan_point_key(x: float, y: float, voxel_size: float) -> str:
    ix = round(x / voxel_size)
    iy = round(y / voxel_size)
    return f"{ix}:{iy}"


def _accumulate_scan(points: list[tuple[float, float, float]] | list[list[float]], source: str) -> None:
    if not SCAN_SESSION["active"] or not points:
        return

    voxel_size = float(SCAN_SESSION["voxel_size"])
    acc = SCAN_SESSION["accumulated"]
    SCAN_SESSION["raw_points"] += len(points)
    if source == "front":
        SCAN_SESSION["front_frames"] += 1
    elif source == "rear":
        SCAN_SESSION["rear_frames"] += 1

    for point in points:
        x, y, intensity = float(point[0]), float(point[1]), float(point[2])
        key = _scan_point_key(x, y, voxel_size)
        slot = acc.get(key)
        if slot is None:
            acc[key] = {
                "x": round(round(x / voxel_size) * voxel_size, 4),
                "y": round(round(y / voxel_size) * voxel_size, 4),
                "intensity": round(float(intensity), 4),
                "hits": 1,
                "source": source,
            }
        else:
            slot["hits"] += 1
            slot["intensity"] = max(slot["intensity"], round(float(intensity), 4))
            slot["source"] = source


def _accumulated_points() -> list[tuple[float, float, float]]:
    return [
        (float(item["x"]), float(item["y"]), float(item["intensity"]))
        for item in SCAN_SESSION["accumulated"].values()
    ]


def _server_capacity_summary() -> dict[str, Any]:
    return {
        "limits": {
            "ws_queue_size": CONFIG.ws_queue_size * 2,
            "queue_near_capacity_ratio": QUEUE_NEAR_CAPACITY_RATIO,
            "camera_min_interval_sec": TOPIC_MIN_INTERVAL_SEC,
            "lidar_max_ws_points": LIDAR_MAX_WS_POINTS,
            "lidar_keyframe_interval_sec": LIDAR_KEYFRAME_INTERVAL_SEC,
            "client_idle_timeout_sec": CLIENT_IDLE_TIMEOUT_SEC,
        },
        "runtime": {
            "ws_overflow_total": int(SERVER_RUNTIME["ws_overflow_total"]),
            "ws_near_capacity_total": int(SERVER_RUNTIME["ws_near_capacity_total"]),
            "ws_last_warn_at": float(SERVER_RUNTIME["ws_last_warn_at"]),
            "active_ws_connections_peak": int(SERVER_RUNTIME["active_ws_connections_peak"]),
            "forced_disconnect_total": int(SERVER_RUNTIME["forced_disconnect_total"]),
        },
    }


def _scan_summary() -> dict[str, Any]:
    started_at = float(SCAN_SESSION["started_at"])
    stopped_at = float(SCAN_SESSION["stopped_at"])
    if started_at <= 0:
        elapsed = 0.0
    elif stopped_at > started_at:
        elapsed = stopped_at - started_at
    else:
        elapsed = time.time() - started_at

    return {
        "active": bool(SCAN_SESSION["active"]),
        "started_at": started_at,
        "stopped_at": stopped_at,
        "elapsed_sec": round(max(0.0, elapsed), 2),
        "voxel_size": float(SCAN_SESSION["voxel_size"]),
        "front_frames": int(SCAN_SESSION["front_frames"]),
        "rear_frames": int(SCAN_SESSION["rear_frames"]),
        "raw_points": int(SCAN_SESSION["raw_points"]),
        "accumulated_points": len(SCAN_SESSION["accumulated"]),
    }


def _ros_diag() -> dict[str, Any]:
    if ros.enabled and ros.bridge is not None:
        return ros.bridge.diagnostics()
    return {}


def _current_map_points() -> list[tuple[float, float, float]]:
    global latest_points

    if ros.enabled and ros.bridge is not None:
        ros_points = ros.bridge.latest_map_points()
        if ros_points:
            return ros_points
    accumulated = _accumulated_points()
    if accumulated:
        return accumulated
    return latest_points


@app.on_event("startup")
async def startup() -> None:
    global ros

    map_dir.mkdir(parents=True, exist_ok=True)
    loop = asyncio.get_running_loop()
    ros = detect_ros(bus=bus, loop=loop, config=CONFIG.ros)

    if ros.enabled:
        logger.info("Server startup. ROS enabled=True reason=%s", ros.reason)
    else:
        logger.warning("Server startup. ROS enabled=False reason=%s", ros.reason)
        if CONFIG.ros.fallback_to_simulator_on_failure:
            await sim.start()
            logger.info("Simulator fallback started")


@app.on_event("shutdown")
async def shutdown() -> None:
    if ros.enabled and ros.bridge is not None:
        ros.bridge.stop()
    if sim._running:
        await sim.stop()


@app.get("/health")
async def health() -> dict:
    return {
        "ok": True,
        "ros_enabled": ros.enabled,
        "ros_reason": ros.reason,
        "scan_active": bool(SCAN_SESSION["active"]),
        "ws_clients": len(ws_clients),
        "topics": STREAM_TOPICS,
        "scan_summary": _scan_summary(),
        "ros_diag": _ros_diag(),
        "simulator_active": bool(sim._running),
        "map_source": "occupancy_grid" if CONFIG.ros.topics.occupancy_grid else "laser_accumulation",
        "frames": {
            "odom": CONFIG.ros.topics.odom_frame,
            "base": CONFIG.ros.topics.robot_base_frame,
            "lidar": CONFIG.ros.topics.lidar_frame,
        },
        "capacity": _server_capacity_summary(),
    }


@app.get("/diag/stream_stats")
async def diag_stream_stats() -> dict:
    return {
        "ok": True,
        "ws_clients": len(ws_clients),
        "topic_stats": bus.stats(),
        "seq_by_topic": dict(seq_by_topic),
        "server_time_ms": int(time.time() * 1000),
        "scan_summary": _scan_summary(),
        "ros_diag": _ros_diag(),
        "capacity": _server_capacity_summary(),
    }


@app.post("/scan/start")
async def start_scan() -> dict:
    _reset_scan_session()
    SCAN_SESSION["active"] = True
    SCAN_SESSION["started_at"] = time.time()
    if ros.enabled and ros.bridge is not None:
        ros.bridge.set_scan_active(True)
    else:
        sim.scanning = True
    logger.info("scan started")
    return {"ok": True, "scan_active": True, "scan_summary": _scan_summary(), "ros_enabled": ros.enabled}


@app.post("/scan/stop")
async def stop_scan() -> dict:
    SCAN_SESSION["active"] = False
    SCAN_SESSION["stopped_at"] = time.time()
    if ros.enabled and ros.bridge is not None:
        ros.bridge.set_scan_active(False)
    else:
        sim.scanning = False
    logger.info("scan stopped")
    return {"ok": True, "scan_active": False, "scan_summary": _scan_summary(), "ros_enabled": ros.enabled}


@app.post("/scan/reset")
async def reset_scan() -> dict:
    _reset_scan_session()
    return {"ok": True, "scan_summary": _scan_summary()}


@app.post("/control/move")
async def move(cmd: MoveCommand) -> dict:
    if ros.enabled and ros.bridge is not None:
        ros.bridge.publish_cmd_vel(cmd.velocity, cmd.yaw_rate)
        await asyncio.sleep(cmd.duration)
        ros.bridge.stop_motion()
        state = {
            "pose": ros.bridge.latest_pose(),
            "gps": ros.bridge.latest_gps(),
            "chassis": ros.bridge.latest_chassis(),
        }
        return {"ok": True, "msg": "ros cmd_vel applied", "state": state}

    sim.set_motion(cmd.velocity, cmd.yaw_rate)
    await asyncio.sleep(cmd.duration)
    sim.stop_motion()
    return {"ok": True, "msg": "sim motion applied", "state": sim.state.__dict__}


@app.post("/control/stop")
async def stop() -> dict:
    if ros.enabled and ros.bridge is not None:
        ros.bridge.stop_motion()
    else:
        sim.stop_motion()
    return {"ok": True}


@app.post("/path/plan")
async def plan_path(req: PlanPathRequest) -> dict:
    sim.state.path = [node.model_dump() for node in req.nodes]
    return {"ok": True, "path_nodes": sim.state.path, "algo": "manual-waypoint"}


@app.post("/map/poi")
async def add_poi(req: AddPoiRequest) -> dict:
    sim.state.poi.append(req.poi.model_dump())
    return {"ok": True, "poi_count": len(sim.state.poi)}


@app.post("/map/save")
async def save_map(req: SaveMapRequest) -> dict:
    points_to_save = _current_map_points()
    if req.voxel_size is not None:
        SCAN_SESSION["voxel_size"] = max(0.02, float(req.voxel_size))
    if not points_to_save:
        points_to_save = [(0.0, 0.0, 1.0)]

    pose = ros.bridge.latest_pose() if ros.enabled and ros.bridge is not None else {
        "x": sim.state.x,
        "y": sim.state.y,
        "yaw": sim.state.yaw,
    }
    gps = ros.bridge.latest_gps() if ros.enabled and ros.bridge is not None else {}
    imu = ros.bridge.latest_imu() if ros.enabled and ros.bridge is not None else {}
    filename = f"{req.name}_{int(time.time())}.stcm"
    target = map_dir / filename
    bundle = {
        "version": "stcm.v2",
        "notes": req.notes,
        "created_at": time.time(),
        "source": "ros" if ros.enabled else "sim",
        "map_source": "occupancy_grid" if CONFIG.ros.topics.occupancy_grid else "laser_accumulation",
        "pose": pose,
        "gps": gps,
        "imu": imu,
        "poi": sim.state.poi,
        "path": sim.state.path,
        "trajectory": sim.state.trajectory,
        "gps_track": sim.state.gps_track,
        "chassis_track": sim.state.chassis_track,
        "scan_summary": _scan_summary(),
        "ros_diag": _ros_diag(),
        "radar_points": points_to_save,
    }
    save_stcm(target, bundle)
    response = {
        "ok": True,
        "file": str(target),
        "contains": {
            "poi": len(sim.state.poi),
            "path": len(sim.state.path),
            "trajectory": len(sim.state.trajectory),
            "gps_track": len(sim.state.gps_track),
            "chassis_track": len(sim.state.chassis_track),
            "radar_points": len(points_to_save),
        },
        "scan_summary": _scan_summary(),
        "ros_enabled": ros.enabled,
    }
    if getattr(req, "reset_after_save", False):
        _reset_scan_session(voxel_size=float(SCAN_SESSION["voxel_size"]))
    return response


@app.post("/map/load")
async def load_map(req: LoadMapRequest) -> dict:
    global latest_points

    bundle = load_stcm(map_dir / req.filename)
    latest_points = [tuple(p) for p in bundle.get("radar_points", [])]
    sim.state.poi = bundle.get("poi", [])
    sim.state.path = bundle.get("path", [])
    sim.state.trajectory = bundle.get("trajectory", [])
    sim.state.gps_track = bundle.get("gps_track", [])
    sim.state.chassis_track = bundle.get("chassis_track", [])

    _reset_scan_session(keep_points=True)
    SCAN_SESSION["accumulated"] = {
        _scan_point_key(float(p[0]), float(p[1]), float(SCAN_SESSION["voxel_size"])): {
            "x": float(p[0]),
            "y": float(p[1]),
            "intensity": float(p[2]),
            "hits": 1,
            "source": "load",
        }
        for p in latest_points
    }
    return {
        "ok": True,
        "point_count": len(latest_points),
        "poi_count": len(sim.state.poi),
        "path_count": len(sim.state.path),
        "chassis_count": len(sim.state.chassis_track),
        "scan_summary": _scan_summary(),
    }


@app.get("/map/list")
async def list_map() -> dict:
    files = sorted([path.name for path in map_dir.glob("*.stcm")])
    return {"ok": True, "files": files}


@app.websocket("/ws/stream")
async def ws_stream(websocket: WebSocket) -> None:
    global latest_points

    await websocket.accept()
    ws_id = id(websocket)
    ws_clients.add(ws_id)
    SERVER_RUNTIME["active_ws_connections_peak"] = max(int(SERVER_RUNTIME["active_ws_connections_peak"]), len(ws_clients))
    logger.info("ws connected id=%s clients=%s", ws_id, len(ws_clients))
    tasks = []
    outbound_queue: asyncio.Queue = asyncio.Queue(maxsize=CONFIG.ws_queue_size * 2)
    last_sent_at: dict[str, float] = {}
    last_lidar_keyframe_at: dict[str, float] = {}

    ws_overflow_local = 0
    ws_near_capacity_local = 0
    ws_last_warn_local = 0.0
    last_client_activity = time.time()
    closed_by_server_timeout = False

    def maybe_warn_capacity(fill_ratio: float, reason: str) -> None:
        nonlocal ws_last_warn_local
        now = time.time()
        if now - ws_last_warn_local >= QUEUE_WARN_INTERVAL_SEC:
            logger.warning(
                "ws queue near limit id=%s reason=%s fill=%.2f qsize=%s max=%s overflow_total=%s",
                ws_id,
                reason,
                fill_ratio,
                outbound_queue.qsize(),
                outbound_queue.maxsize,
                ws_overflow_local,
            )
            ws_last_warn_local = now
            SERVER_RUNTIME["ws_last_warn_at"] = now

    def enqueue_nonblocking(item: tuple[str, dict | str], reason: str) -> None:
        nonlocal ws_overflow_local, ws_near_capacity_local
        while True:
            try:
                if outbound_queue.maxsize > 0:
                    fill_ratio = outbound_queue.qsize() / outbound_queue.maxsize
                    if fill_ratio >= QUEUE_NEAR_CAPACITY_RATIO:
                        ws_near_capacity_local += 1
                        SERVER_RUNTIME["ws_near_capacity_total"] += 1
                        maybe_warn_capacity(fill_ratio, reason)
                outbound_queue.put_nowait(item)
                return
            except asyncio.QueueFull:
                ws_overflow_local += 1
                SERVER_RUNTIME["ws_overflow_total"] += 1
                maybe_warn_capacity(1.0, reason)
                try:
                    outbound_queue.get_nowait()
                except asyncio.QueueEmpty:
                    continue

    async def enqueue_topic(topic: str) -> None:
        global latest_points

        async for message in bus.subscribe(topic):
            now = time.time()
            min_interval = TOPIC_MIN_INTERVAL_SEC.get(topic, 0.0)
            if min_interval > 0:
                last = last_sent_at.get(topic, 0.0)
                if now - last < min_interval:
                    continue
                last_sent_at[topic] = now

            if topic == "/lidar/front":
                latest_points = [tuple(point) for point in message["payload"].get("points", [])[:4000]]
                _accumulate_scan(latest_points, "front")
            elif topic == "/lidar/rear":
                rear_points = [tuple(point) for point in message["payload"].get("points", [])[:4000]]
                if rear_points:
                    latest_points = rear_points
                _accumulate_scan(rear_points, "rear")
            elif topic == "/map/grid" and ros.enabled and ros.bridge is not None:
                ros_points = ros.bridge.latest_map_points()
                if ros_points:
                    latest_points = ros_points

            outbound_message = message
            if topic in {"/lidar/front", "/lidar/rear"}:
                points = message["payload"].get("points", [])
                last_kf = last_lidar_keyframe_at.get(topic, 0.0)
                keyframe = (now - last_kf) >= LIDAR_KEYFRAME_INTERVAL_SEC
                if keyframe:
                    last_lidar_keyframe_at[topic] = now
                thinned = points if keyframe else _thin_points(points, LIDAR_MAX_WS_POINTS)
                outbound_message = {
                    **message,
                    "payload": {
                        **message["payload"],
                        "points": thinned,
                        "raw_points": len(points),
                        "keyframe": keyframe,
                    },
                }

            packed = _pack_message(outbound_message)
            enqueue_nonblocking(("json", packed), reason=topic)

    async def monitor_client_idle() -> None:
        nonlocal closed_by_server_timeout
        while True:
            await asyncio.sleep(2.0)
            idle_sec = time.time() - last_client_activity
            if idle_sec > CLIENT_IDLE_TIMEOUT_SEC:
                SERVER_RUNTIME["forced_disconnect_total"] += 1
                closed_by_server_timeout = True
                logger.warning("ws idle timeout id=%s idle_sec=%.1f, force disconnect", ws_id, idle_sec)
                await websocket.close(code=4001, reason="idle_timeout")
                return

    async def send_outbound() -> None:
        while True:
            message_type, payload = await outbound_queue.get()
            if message_type == "json":
                await websocket.send_json(payload)
            elif message_type == "text":
                await websocket.send_text(payload)

    async def receive_keepalive() -> None:
        nonlocal last_client_activity
        while True:
            client_msg = await websocket.receive_text()
            last_client_activity = time.time()
            if client_msg == "ping":
                enqueue_nonblocking(("text", "pong"), reason="keepalive")

    try:
        for topic in STREAM_TOPICS:
            tasks.append(asyncio.create_task(enqueue_topic(topic)))
        tasks.append(asyncio.create_task(send_outbound()))
        tasks.append(asyncio.create_task(receive_keepalive()))
        tasks.append(asyncio.create_task(monitor_client_idle()))
        done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_EXCEPTION)
        for finished in done:
            exc = finished.exception()
            if exc is not None and not isinstance(exc, WebSocketDisconnect):
                raise exc
    except WebSocketDisconnect:
        if closed_by_server_timeout:
            logger.warning("ws disconnected by server timeout id=%s", ws_id)
        else:
            logger.warning("ws disconnected id=%s", ws_id)
    except asyncio.CancelledError:
        logger.info("ws task cancelled id=%s", ws_id)
        raise
    finally:
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        ws_clients.discard(ws_id)
        if ws_overflow_local > 0:
            logger.warning("ws disconnected with overflow id=%s dropped=%s near_capacity=%s", ws_id, ws_overflow_local, ws_near_capacity_local)
        try:
            await websocket.close()
        except Exception:  # noqa: BLE001
            pass
