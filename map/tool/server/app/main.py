from __future__ import annotations

import asyncio
import hashlib
import json
import logging
import time
from collections import defaultdict
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from .config import CONFIG
from .models import AddPoiRequest, LoadMapRequest, MoveCommand, PlanPathRequest, SaveMapRequest
from .ros_bridge import detect_ros
from .simulator import Simulator
from .stcm_codec import load_stcm, save_stcm
from .topic_bus import TopicBus


logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(levelname)s | %(name)s | %(message)s")
logger = logging.getLogger("autodrive.server")

app = FastAPI(title="AutoDrive Mapping Server", version="0.4.0")
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

bus = TopicBus(queue_size=CONFIG.ws_queue_size)
sim = Simulator(bus, rate_hz=CONFIG.sim_rate_hz, points_per_scan=CONFIG.lidar_points_per_scan)
ros = detect_ros()
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


@app.on_event("startup")
async def startup() -> None:
    await sim.start()
    logger.info("Server startup. ROS enabled=%s reason=%s", ros.enabled, ros.reason)


@app.on_event("shutdown")
async def shutdown() -> None:
    await sim.stop()


@app.get("/health")
async def health() -> dict:
    return {
        "ok": True,
        "ros_enabled": ros.enabled,
        "ros_reason": ros.reason,
        "scan_active": sim.scanning,
        "ws_clients": len(ws_clients),
        "topics": STREAM_TOPICS,
    }


@app.get("/diag/stream_stats")
async def diag_stream_stats() -> dict:
    return {
        "ok": True,
        "ws_clients": len(ws_clients),
        "topic_stats": bus.stats(),
        "seq_by_topic": dict(seq_by_topic),
        "server_time_ms": int(time.time() * 1000),
    }


@app.post("/scan/start")
async def start_scan() -> dict:
    sim.scanning = True
    logger.info("scan started")
    return {"ok": True, "scan_active": True}


@app.post("/scan/stop")
async def stop_scan() -> dict:
    sim.scanning = False
    logger.info("scan stopped")
    return {"ok": True, "scan_active": False}


@app.post("/control/move")
async def move(cmd: MoveCommand) -> dict:
    sim.set_motion(cmd.velocity, cmd.yaw_rate)
    await asyncio.sleep(cmd.duration)
    sim.stop_motion()
    return {"ok": True, "msg": "motion applied", "state": sim.state.__dict__}


@app.post("/control/stop")
async def stop() -> dict:
    sim.stop_motion()
    return {"ok": True}


@app.post("/path/plan")
async def plan_path(req: PlanPathRequest) -> dict:
    sim.state.path = [n.model_dump() for n in req.nodes]
    return {"ok": True, "path_nodes": sim.state.path, "algo": "manual-waypoint"}


@app.post("/map/poi")
async def add_poi(req: AddPoiRequest) -> dict:
    sim.state.poi.append(req.poi.model_dump())
    return {"ok": True, "poi_count": len(sim.state.poi)}


@app.post("/map/save")
async def save_map(req: SaveMapRequest) -> dict:
    global latest_points
    if not latest_points:
        latest_points = [(0.0, 0.0, 1.0)]

    filename = f"{req.name}_{int(time.time())}.stcm"
    target = map_dir / filename
    bundle = {
        "version": "stcm.v2",
        "notes": req.notes,
        "created_at": time.time(),
        "source": "ros" if ros.enabled else "sim",
        "pose": {"x": sim.state.x, "y": sim.state.y, "yaw": sim.state.yaw},
        "poi": sim.state.poi,
        "path": sim.state.path,
        "trajectory": sim.state.trajectory,
        "gps_track": sim.state.gps_track,
        "chassis_track": sim.state.chassis_track,
        "radar_points": latest_points,
    }
    save_stcm(target, bundle)
    return {
        "ok": True,
        "file": str(target),
        "contains": {
            "poi": len(sim.state.poi),
            "path": len(sim.state.path),
            "trajectory": len(sim.state.trajectory),
            "gps_track": len(sim.state.gps_track),
            "chassis_track": len(sim.state.chassis_track),
            "radar_points": len(latest_points),
        },
    }


@app.post("/map/load")
async def load_map(req: LoadMapRequest) -> dict:
    global latest_points
    bundle = load_stcm(map_dir / req.filename)
    latest_points = bundle.get("radar_points", [])
    sim.state.poi = bundle.get("poi", [])
    sim.state.path = bundle.get("path", [])
    sim.state.trajectory = bundle.get("trajectory", [])
    sim.state.gps_track = bundle.get("gps_track", [])
    sim.state.chassis_track = bundle.get("chassis_track", [])
    return {
        "ok": True,
        "point_count": len(latest_points),
        "poi_count": len(sim.state.poi),
        "path_count": len(sim.state.path),
        "chassis_count": len(sim.state.chassis_track),
    }


@app.get("/map/list")
async def list_map() -> dict:
    files = sorted([p.name for p in map_dir.glob("*.stcm")])
    return {"ok": True, "files": files}


@app.websocket("/ws/stream")
async def ws_stream(websocket: WebSocket) -> None:
    global latest_points
    await websocket.accept()
    ws_id = id(websocket)
    ws_clients.add(ws_id)
    logger.info("ws connected id=%s clients=%s", ws_id, len(ws_clients))
    tasks = []

    async def consume_topic(topic: str) -> None:
        async for message in bus.subscribe(topic):
            if topic.startswith("/lidar"):
                latest_points = [tuple(p) for p in message["payload"]["points"][:4000]]
            await websocket.send_json(_pack_message(message))

    async def receive_keepalive() -> None:
        while True:
            client_msg = await websocket.receive_text()
            if client_msg == "ping":
                await websocket.send_text("pong")

    try:
        for t in STREAM_TOPICS:
            tasks.append(asyncio.create_task(consume_topic(t)))
        tasks.append(asyncio.create_task(receive_keepalive()))
        await asyncio.gather(*tasks)
    except WebSocketDisconnect:
        logger.warning("ws disconnected id=%s", ws_id)
    finally:
        for task in tasks:
            task.cancel()
        ws_clients.discard(ws_id)
