from __future__ import annotations

import asyncio
import base64
import copy
import hashlib
import json
import logging
import os
import signal
import subprocess
import threading
import time
from collections import defaultdict
from pathlib import Path
from typing import Any

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from .config import CONFIG
from .models import (
    AddPoiRequest,
    ControlTargetRequest,
    LoadMapRequest,
    MoveCommand,
    PlanPathRequest,
    SaveMapRequest,
    StartScanRequest,
    StopScanRequest,
)
from .ros_bridge import RosRuntime, detect_ros
from .simulator import Simulator
from .stcm_codec import load_stcm, save_stcm
from .topic_bus import TopicBus


logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(levelname)s | %(name)s | %(message)s")
logger = logging.getLogger("autodrive.server")


class _SuccessPostAccessLogFilter(logging.Filter):
    def filter(self, record: logging.LogRecord) -> bool:
        args = record.args if isinstance(record.args, tuple) else ()
        method = str(args[1]) if len(args) >= 5 else ""
        try:
            status_code = int(args[4]) if len(args) >= 5 else 0
        except (TypeError, ValueError):
            status_code = 0
        if method.upper() == "POST" and 200 <= status_code < 300:
            logging.getLogger(record.name).debug(record.getMessage())
            return False
        return True


def _install_access_log_filter() -> None:
    access_logger = logging.getLogger("uvicorn.access")
    if not any(isinstance(existing, _SuccessPostAccessLogFilter) for existing in access_logger.filters):
        access_logger.addFilter(_SuccessPostAccessLogFilter())


_install_access_log_filter()

app = FastAPI(title="AutoDrive Mapping Server", version="0.6.0")
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

bus = TopicBus(queue_size=CONFIG.ws_queue_size)
sim = Simulator(bus, rate_hz=CONFIG.sim_rate_hz, points_per_scan=CONFIG.lidar_points_per_scan)
ros: RosRuntime = RosRuntime(enabled=False, reason="ROS runtime not initialized")
map_dir = Path("data/maps")
latest_points: list[tuple[float, float, float]] = []
seq_by_topic: dict[str, int] = defaultdict(int)
ws_clients: set[int] = set()
motion_command_seq = 0
CONTROL_TARGET = {"velocity": 0.0, "yaw_rate": 0.0, "updated_at": 0.0}
CONTROL_PUBLISH_INTERVAL_SEC = 0.1
CONTROL_TARGET_HOLD_SEC = 1.0
CONTROL_STOP_BURST_TICKS = 3
CONTROL_STOP_BURST_REMAINING = 0
CONTROL_RUNTIME = {
    "last_zero_source": "",
    "last_zero_at": 0.0,
    "last_publish_source": "",
    "last_logged_source": "",
    "last_logged_velocity": None,
    "last_logged_yaw_rate": None,
}
control_task: asyncio.Task | None = None


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
    "mode": "2d",
    "started_at": 0.0,
    "stopped_at": 0.0,
    "voxel_size": 0.12,
    "front_frames": 0,
    "rear_frames": 0,
    "raw_points": 0,
    "dependency_status": {
        "required_nodes": [],
        "missing_nodes": [],
        "started_nodes": [],
        "required_processes": [],
        "missing_processes": [],
        "started_processes": [],
        "errors": [],
    },
    "pcd_transfer_state": "idle",
    "pcd_file": None,
}

LAUNCHED_SCAN_PROCESSES: list[subprocess.Popen[str]] = []
SCAN_DEPENDENCY_POLL_ATTEMPTS = 10
SCAN_DEPENDENCY_POLL_INTERVAL_SEC = 1.0
SCAN_MAPPING_PREREQ_POLL_ATTEMPTS = 10
SCAN_MAPPING_PREREQ_POLL_INTERVAL_SEC = 0.5


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
    SCAN_SESSION["mode"] = "2d"
    SCAN_SESSION["started_at"] = 0.0
    SCAN_SESSION["stopped_at"] = 0.0
    SCAN_SESSION["front_frames"] = 0
    SCAN_SESSION["rear_frames"] = 0
    SCAN_SESSION["raw_points"] = 0
    SCAN_SESSION["dependency_status"] = {
        "required_nodes": [],
        "missing_nodes": [],
        "started_nodes": [],
        "required_processes": [],
        "missing_processes": [],
        "started_processes": [],
        "errors": [],
    }
    SCAN_SESSION["pcd_transfer_state"] = "idle"
    SCAN_SESSION["pcd_file"] = None
    if voxel_size is not None:
        SCAN_SESSION["voxel_size"] = max(0.02, float(voxel_size))


def _normalize_scan_mode(mode: str | None) -> str | None:
    normalized = str(mode or "").strip().lower()
    return normalized if normalized in {"2d", "3d"} else None


def _scan_mode_config(mode: str) -> Any:
    if mode == "2d":
        return CONFIG.scan_modes.mode_2d
    if mode == "3d":
        return CONFIG.scan_modes.mode_3d
    raise ValueError(f"unsupported scan mode: {mode}")


def _list_ros_nodes() -> list[str]:
    completed = subprocess.run(
        ["ros2", "node", "list"],
        check=True,
        capture_output=True,
        text=True,
    )
    return [line.strip() for line in str(completed.stdout or "").splitlines() if line.strip()]


def _check_required_nodes(nodes: list[str]) -> dict[str, Any]:
    if not nodes:
        return {
            "required_nodes": [],
            "missing_nodes": [],
            "started_nodes": [],
            "errors": [],
        }
    try:
        existing = set(_list_ros_nodes())
    except Exception as exc:  # noqa: BLE001
        detail = ""
        if isinstance(exc, subprocess.CalledProcessError):
            detail = str(getattr(exc, "stderr", "") or getattr(exc, "stdout", "") or "").strip()
        if not detail:
            detail = str(exc)
        return {
            "required_nodes": list(nodes),
            "missing_nodes": list(nodes),
            "started_nodes": [],
            "errors": [detail],
        }
    return {
        "required_nodes": list(nodes),
        "missing_nodes": [node for node in nodes if node not in existing],
        "started_nodes": [],
        "errors": [],
    }


def _list_process_matches(pattern: str) -> list[str]:
    completed = subprocess.run(
        ["pgrep", "-af", pattern],
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode not in {0, 1}:
        detail = str(completed.stderr or completed.stdout or "").strip()
        raise RuntimeError(detail or f"pgrep failed for {pattern}")
    return [line.strip() for line in str(completed.stdout or "").splitlines() if line.strip()]


def _check_required_processes(processes: list[str]) -> dict[str, Any]:
    if not processes:
        return {"required_processes": [], "missing_processes": [], "matched_processes": {}, "errors": []}
    missing: list[str] = []
    errors: list[str] = []
    matched: dict[str, list[str]] = {}
    for process in processes:
        try:
            matches = _list_process_matches(process)
        except Exception as exc:  # noqa: BLE001
            missing.append(process)
            errors.append(str(exc))
            matched[process] = []
            continue
        matched[process] = matches[:5]
        if not matches:
            missing.append(process)
    logger.info(
        "scan process check required=%s missing=%s matched=%s errors=%s",
        list(processes),
        missing,
        matched,
        errors,
    )
    return {"required_processes": list(processes), "missing_processes": missing, "matched_processes": matched, "errors": errors}


def _launch_scan_mode_command(argv: list[str]) -> tuple[bool, str]:
    try:
        process = subprocess.Popen(
            argv,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
            start_new_session=True,
        )
    except Exception as exc:  # noqa: BLE001
        return False, str(exc)
    LAUNCHED_SCAN_PROCESSES.append(process)
    logger.info("started scan dependency command argv=%s pid=%s", argv, process.pid)
    return True, f"pid={process.pid}"


def _reap_scan_process(process: subprocess.Popen[str]) -> None:
    try:
        process.wait()
        if process in LAUNCHED_SCAN_PROCESSES:
            LAUNCHED_SCAN_PROCESSES.remove(process)
        logger.info("reaped scan dependency process pid=%s", getattr(process, "pid", "?"))
    except Exception as exc:  # noqa: BLE001
        logger.warning("failed to reap scan dependency process pid=%s err=%s", getattr(process, "pid", "?"), exc)


def _reap_scan_process_async(process: subprocess.Popen[str]) -> None:
    threading.Thread(target=_reap_scan_process, args=(process,), daemon=True).start()


def _merge_scan_dependency_status(node_status: dict[str, Any], process_status: dict[str, Any]) -> dict[str, Any]:
    return {
        "required_nodes": list(node_status.get("required_nodes", [])),
        "missing_nodes": list(node_status.get("missing_nodes", [])),
        "started_nodes": list(node_status.get("started_nodes", [])),
        "required_processes": list(process_status.get("required_processes", [])),
        "missing_processes": list(process_status.get("missing_processes", [])),
        "started_processes": list(process_status.get("started_processes", [])),
        "matched_processes": copy.deepcopy(process_status.get("matched_processes", {})),
        "errors": list(node_status.get("errors", [])) + list(process_status.get("errors", [])),
    }


def _scan_dependency_status(config: Any) -> dict[str, Any]:
    node_status = _check_required_nodes(list(config.required_nodes))
    process_status = _check_required_processes(list(getattr(config, "required_processes", [])))
    return _merge_scan_dependency_status(node_status, process_status)


def _ensure_scan_mode_dependencies(mode: str) -> dict[str, Any]:
    config = _scan_mode_config(mode)
    status = _scan_dependency_status(config)
    logger.info(
        "scan dependency check mode=%s required_nodes=%s missing_nodes=%s required_processes=%s missing_processes=%s errors=%s",
        mode,
        status.get("required_nodes", []),
        status.get("missing_nodes", []),
        status.get("required_processes", []),
        status.get("missing_processes", []),
        status.get("errors", []),
    )
    if not status["missing_nodes"] and not status["missing_processes"]:
        return status
    started_nodes: list[str] = list(status.get("started_nodes", []))
    started_processes: list[str] = list(status.get("started_processes", []))
    errors: list[str] = list(status.get("errors", []))
    for command in config.launch_commands:
        ok, detail = _launch_scan_mode_command(list(command))
        if not ok:
            errors.append(detail or f"failed to launch {command}")
            logger.warning("scan dependency launch failed mode=%s command=%s detail=%s", mode, command, detail)
            continue
        logger.info("waiting for scan dependencies mode=%s command=%s", mode, command)
        refreshed = status
        for attempt in range(1, SCAN_DEPENDENCY_POLL_ATTEMPTS + 1):
            time.sleep(SCAN_DEPENDENCY_POLL_INTERVAL_SEC)
            refreshed = _scan_dependency_status(config)
            logger.info(
                "scan dependency poll mode=%s attempt=%s/%s missing_nodes=%s missing_processes=%s errors=%s",
                mode,
                attempt,
                SCAN_DEPENDENCY_POLL_ATTEMPTS,
                refreshed.get("missing_nodes", []),
                refreshed.get("missing_processes", []),
                refreshed.get("errors", []),
            )
            if not refreshed["missing_nodes"] and not refreshed["missing_processes"]:
                break
        started_nodes = list(dict.fromkeys(started_nodes + list(refreshed.get("required_nodes", []))))
        started_processes = list(dict.fromkeys(started_processes + list(refreshed.get("required_processes", []))))
        status = refreshed
        if not status["missing_nodes"] and not status["missing_processes"]:
            status["started_nodes"] = started_nodes
            status["started_processes"] = started_processes
            status["errors"] = errors
            logger.info(
                "scan dependency launch success mode=%s started_nodes=%s started_processes=%s",
                mode,
                status["started_nodes"],
                status["started_processes"],
            )
            return status
    status["started_nodes"] = started_nodes
    status["started_processes"] = started_processes
    status["errors"] = errors
    return status


def _stop_launched_scan_processes() -> dict[str, Any]:
    stopped_pids: list[int] = []
    errors: list[str] = []
    remaining: list[subprocess.Popen[str]] = []
    for process in LAUNCHED_SCAN_PROCESSES:
        try:
            logger.info("stopping scan dependency process pid=%s signal=SIGINT", process.pid)
            os.killpg(process.pid, signal.SIGINT)
            stopped_pids.append(int(process.pid))
            try:
                process.wait(timeout=3.0 if process.poll() is None else 0.1)
            except subprocess.TimeoutExpired:
                errors.append(f"pid={process.pid}: did not exit after SIGINT")
                remaining.append(process)
                _reap_scan_process_async(process)
        except ProcessLookupError:
            try:
                process.wait(timeout=0.1)
            except Exception:  # noqa: BLE001
                pass
            logger.info("scan dependency process group already exited pid=%s", getattr(process, "pid", "?"))
        except Exception as exc:  # noqa: BLE001
            errors.append(f"pid={getattr(process, 'pid', '?')}: {exc}")
            remaining.append(process)
    LAUNCHED_SCAN_PROCESSES[:] = remaining
    pattern_status = _terminate_process_patterns(list(SCAN_SESSION.get("dependency_status", {}).get("started_processes", [])))
    if stopped_pids:
        logger.info("stopped scan dependency processes pids=%s", stopped_pids)
    errors.extend(pattern_status["errors"])
    if errors:
        logger.warning("failed to stop scan dependency processes errors=%s", errors)
    return {"stopped_pids": stopped_pids, "stopped_patterns": pattern_status["stopped_patterns"], "errors": errors}


def _terminate_process_patterns(patterns: list[str]) -> dict[str, Any]:
    stopped_patterns: list[str] = []
    errors: list[str] = []
    for pattern in dict.fromkeys(str(pattern).strip() for pattern in patterns if str(pattern).strip()):
        logger.info("stopping scan dependency process pattern=%s signal=SIGINT", pattern)
        term = subprocess.run(["pkill", "-INT", "-f", pattern], check=False, capture_output=True, text=True)
        if term.returncode not in {0, 1}:
            detail = str(term.stderr or term.stdout or "").strip()
            errors.append(f"{pattern}: {detail or 'pkill INT failed'}")
            continue
        if term.returncode == 0:
            stopped_patterns.append(pattern)
        time.sleep(0.2)
        remaining = _list_process_matches(pattern)
        if remaining:
            logger.warning("scan dependency process pattern still running after SIGINT pattern=%s matches=%s", pattern, remaining[:5])
            errors.append(f"{pattern}: still running after SIGINT")
    return {"stopped_patterns": stopped_patterns, "errors": errors}


def _wait_for_mapping_prereq_after_dependency_start(dependency_status: dict[str, Any]) -> dict[str, Any]:
    summary = _mapping_prereq_summary()
    if summary["ready"] or not (dependency_status.get("started_nodes") or dependency_status.get("started_processes")):
        return summary
    for _ in range(SCAN_MAPPING_PREREQ_POLL_ATTEMPTS):
        time.sleep(SCAN_MAPPING_PREREQ_POLL_INTERVAL_SEC)
        summary = _mapping_prereq_summary()
        if summary["ready"]:
            return summary
    return summary


def _pcd_output_path_for_mode(mode: str) -> Path | None:
    normalized = _normalize_scan_mode(mode)
    if normalized != "3d":
        return None
    raw = str(_scan_mode_config(normalized).pcd_output_path or "").strip()
    return Path(raw) if raw else None


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
        "accumulated_points": 0,
    }


def _ros_diag() -> dict[str, Any]:
    if ros.enabled and ros.bridge is not None:
        return ros.bridge.diagnostics()
    return {}


def _network_diag_summary() -> dict[str, Any]:
    topic_stats = bus.stats()
    warnings: list[str] = []
    checks = {
        "ws_clients": {"ok": len(ws_clients) > 0, "value": len(ws_clients)},
        "topic_bus": {"ok": True, "topics": []},
    }
    if len(ws_clients) == 0:
        warnings.append("no websocket clients connected")
    degraded_topics: list[str] = []
    for topic, stat in topic_stats.items():
        if float(stat.get("drop_rate", 0.0)) > 0.05 or float(stat.get("peak_fill_ratio", 0.0)) > 0.9:
            degraded_topics.append(topic)
    if degraded_topics:
        warnings.append(f"topic bus pressure on {', '.join(sorted(degraded_topics))}")
        checks["topic_bus"]["ok"] = False
    checks["topic_bus"]["topics"] = degraded_topics
    return {"ok": not warnings, "warnings": warnings, "checks": checks}


def _mapping_prereq_summary() -> dict[str, Any]:
    if ros.enabled and ros.bridge is not None and hasattr(ros.bridge, "mapping_prerequisites"):
        summary = dict(ros.bridge.mapping_prerequisites())
    elif ros.enabled:
        summary = {
            "ready": False,
            "severity": "error",
            "blockers": ["ros bridge does not expose mapping prerequisites"],
            "warnings": [],
            "checks": {"ros_runtime": {"ok": False}},
        }
    else:
        summary = {
            "ready": bool(sim._running),
            "severity": "ok" if bool(sim._running) else "warn",
            "blockers": [] if bool(sim._running) else ["ros disabled and simulator inactive"],
            "warnings": [] if bool(sim._running) else ["mapping data source unavailable"],
            "checks": {"data_source": {"ok": bool(sim._running), "source": "simulator" if bool(sim._running) else "none"}},
        }

    network = _network_diag_summary()
    checks = dict(summary.get("checks", {}))
    checks["network"] = network["checks"]
    warnings = list(summary.get("warnings", []))
    warnings.extend(item for item in network["warnings"] if item not in warnings)
    blockers = list(summary.get("blockers", []))
    severity = "error" if blockers else "warn" if warnings else "ok"
    ready = bool(summary.get("ready", False)) and not blockers
    return {
        "ready": ready,
        "severity": severity,
        "blockers": blockers,
        "warnings": warnings,
        "checks": checks,
    }


def _current_map_points() -> list[tuple[float, float, float]]:
    global latest_points

    if ros.enabled and ros.bridge is not None:
        ros_points = ros.bridge.latest_map_points()
        if ros_points:
            return ros_points
    return latest_points


def _current_map_source() -> str:
    if ros.enabled and ros.bridge is not None and getattr(ros.bridge, "latest_map_points", lambda: [])():
        return "occupancy_grid"
    if latest_points:
        return "loaded_map"
    return "unavailable"


def _occupancy_payload_to_points(payload: dict[str, Any]) -> list[tuple[float, float, float]]:
    if isinstance(payload.get("data"), list):
        resolution = max(0.02, float(payload.get("resolution", 0.05) or 0.05))
        origin = payload.get("origin") if isinstance(payload.get("origin"), dict) else {}
        origin_x = float(origin.get("x", 0.0))
        origin_y = float(origin.get("y", 0.0))
        width = int(payload.get("width", 0) or 0)
        points: list[tuple[float, float, float]] = []
        if width <= 0:
            return points
        for index, value in enumerate(payload.get("data", [])):
            if int(value) < 50:
                continue
            row = index // width
            col = index % width
            x = origin_x + (col + 0.5) * resolution
            y = origin_y + (row + 0.5) * resolution
            points.append((round(x, 3), round(y, 3), 1.0))
        return points
    return [
        (float(cell.get("x", 0.0)), float(cell.get("y", 0.0)), 1.0)
        for cell in payload.get("occupied", [])
    ]


def _effective_control_target(now: float | None = None) -> tuple[float, float, bool, bool]:
    now = time.time() if now is None else float(now)
    updated_at = float(CONTROL_TARGET.get("updated_at", 0.0) or 0.0)
    velocity = float(CONTROL_TARGET.get("velocity", 0.0) or 0.0)
    yaw_rate = float(CONTROL_TARGET.get("yaw_rate", 0.0) or 0.0)
    if updated_at <= 0.0:
        return 0.0, 0.0, False, False
    if now - updated_at <= CONTROL_TARGET_HOLD_SEC:
        return velocity, yaw_rate, False, True
    return 0.0, 0.0, bool(velocity or yaw_rate), False


def _control_target_health(now: float | None = None) -> dict[str, Any]:
    now = time.time() if now is None else float(now)
    updated_at = float(CONTROL_TARGET.get("updated_at", 0.0) or 0.0)
    age_sec = max(0.0, now - updated_at) if updated_at > 0.0 else None
    velocity, yaw_rate, stale, should_publish = _effective_control_target(now=now)
    return {
        "velocity": float(CONTROL_TARGET.get("velocity", 0.0) or 0.0),
        "yaw_rate": float(CONTROL_TARGET.get("yaw_rate", 0.0) or 0.0),
        "effective_velocity": velocity,
        "effective_yaw_rate": yaw_rate,
        "publishing": should_publish,
        "updated_at": updated_at,
        "age_sec": round(age_sec, 3) if age_sec is not None else None,
        "stale": stale,
        "publish_interval_sec": CONTROL_PUBLISH_INTERVAL_SEC,
        "hold_sec": CONTROL_TARGET_HOLD_SEC,
        "last_publish_source": str(CONTROL_RUNTIME["last_publish_source"]),
        "last_zero_source": str(CONTROL_RUNTIME["last_zero_source"]),
        "last_zero_at": float(CONTROL_RUNTIME["last_zero_at"]),
    }


def _record_control_publish_source(source: str, velocity: float, yaw_rate: float) -> None:
    CONTROL_RUNTIME["last_publish_source"] = source
    velocity = float(velocity)
    yaw_rate = float(yaw_rate)
    last_velocity = CONTROL_RUNTIME.get("last_logged_velocity")
    last_yaw_rate = CONTROL_RUNTIME.get("last_logged_yaw_rate")
    velocity_changed = (
        last_velocity is None
        or last_yaw_rate is None
        or abs(float(last_velocity) - velocity) > 1e-9
        or abs(float(last_yaw_rate) - yaw_rate) > 1e-9
    )
    source_changed = CONTROL_RUNTIME.get("last_logged_source") != source
    changed = (
        velocity_changed
        or (source_changed and source != "target_hold")
    )
    zero = abs(velocity) <= 1e-9 and abs(yaw_rate) <= 1e-9
    if changed:
        logger.info(
            "control cmd publish source=%s velocity=%.3f yaw_rate=%.3f zero=%s",
            source,
            velocity,
            yaw_rate,
            zero,
        )
        CONTROL_RUNTIME["last_logged_source"] = source
        CONTROL_RUNTIME["last_logged_velocity"] = velocity
        CONTROL_RUNTIME["last_logged_yaw_rate"] = yaw_rate
    if abs(float(velocity)) <= 1e-9 and abs(float(yaw_rate)) <= 1e-9:
        CONTROL_RUNTIME["last_zero_source"] = source
        CONTROL_RUNTIME["last_zero_at"] = time.time()


def _publish_control_command(velocity: float, yaw_rate: float, source: str) -> None:
    _record_control_publish_source(source, velocity, yaw_rate)
    if ros.enabled and ros.bridge is not None:
        ros.bridge.publish_cmd_vel(velocity, yaw_rate)
    else:
        sim.set_motion(velocity, yaw_rate)


async def _control_publisher_loop() -> None:
    global CONTROL_STOP_BURST_REMAINING
    stale_logged = False
    while True:
        try:
            velocity, yaw_rate, stale, should_publish = _effective_control_target()
            if stale and not stale_logged:
                logger.warning("control target stale; publishing stop for safety")
                stale_logged = True
                CONTROL_STOP_BURST_REMAINING = CONTROL_STOP_BURST_TICKS
                CONTROL_TARGET["velocity"] = 0.0
                CONTROL_TARGET["yaw_rate"] = 0.0
                CONTROL_TARGET["updated_at"] = 0.0
            elif not stale:
                stale_logged = False
            source = "target_stale_stop" if stale else "target_hold"
            if CONTROL_STOP_BURST_REMAINING > 0:
                velocity, yaw_rate = 0.0, 0.0
                should_publish = True
                CONTROL_STOP_BURST_REMAINING -= 1
                source = "target_stale_stop_burst"
            if should_publish:
                _publish_control_command(velocity, yaw_rate, source)
            await asyncio.sleep(CONTROL_PUBLISH_INTERVAL_SEC)
        except asyncio.CancelledError:
            raise
        except Exception:  # noqa: BLE001
            logger.exception("control publisher loop failed")
            await asyncio.sleep(CONTROL_PUBLISH_INTERVAL_SEC)


@app.on_event("startup")
async def startup() -> None:
    global control_task, ros

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
    control_task = asyncio.create_task(_control_publisher_loop())


@app.on_event("shutdown")
async def shutdown() -> None:
    global control_task
    if control_task is not None:
        control_task.cancel()
        try:
            await control_task
        except asyncio.CancelledError:
            pass
        control_task = None
    if ros.enabled and ros.bridge is not None:
        ros.bridge.stop()
    if sim._running:
        await sim.stop()


@app.get("/health")
async def health() -> dict:
    mapping_prereq = _mapping_prereq_summary()
    return {
        "ok": True,
        "ros_enabled": ros.enabled,
        "ros_reason": ros.reason,
        "scan_active": bool(SCAN_SESSION["active"]),
        "scan_mode": str(SCAN_SESSION["mode"]),
        "ws_clients": len(ws_clients),
        "topics": STREAM_TOPICS,
        "scan_summary": _scan_summary(),
        "dependency_status": copy.deepcopy(SCAN_SESSION["dependency_status"]),
        "pcd_transfer_state": str(SCAN_SESSION["pcd_transfer_state"]),
        "pcd_metadata": copy.deepcopy(SCAN_SESSION["pcd_file"]),
        "ros_diag": _ros_diag(),
        "mapping_ready": bool(mapping_prereq["ready"]),
        "mapping_status": mapping_prereq["severity"],
        "mapping_blockers": list(mapping_prereq["blockers"]),
        "mapping_warnings": list(mapping_prereq["warnings"]),
        "control_target": _control_target_health(),
        "simulator_active": bool(sim._running),
        "map_source": _current_map_source(),
        "frames": {
            "odom": CONFIG.ros.topics.odom_frame,
            "base": CONFIG.ros.topics.robot_base_frame,
            "lidar": CONFIG.ros.topics.lidar_frame,
        },
        "capacity": _server_capacity_summary(),
    }


@app.get("/diag/mapping_prereq")
async def diag_mapping_prereq() -> dict:
    return {
        "ok": True,
        "ros_enabled": ros.enabled,
        "simulator_active": bool(sim._running),
        "mapping_prereq": _mapping_prereq_summary(),
        "ros_diag": _ros_diag(),
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
async def start_scan(req: StartScanRequest | None = None) -> dict:
    mode = _normalize_scan_mode(req.mode if req is not None else "2d")
    if mode is None:
        return {
            "ok": False,
            "reason": "invalid_scan_mode",
            "scan_active": False,
        }
    if bool(SCAN_SESSION["active"]):
        return {
            "ok": False,
            "reason": "scan_already_active",
            "scan_active": True,
            "scan_mode": str(SCAN_SESSION["mode"]),
        }
    dependency_status = _ensure_scan_mode_dependencies(mode)
    SCAN_SESSION["dependency_status"] = copy.deepcopy(dependency_status)
    if dependency_status["missing_nodes"] or dependency_status.get("missing_processes") or dependency_status["errors"]:
        return {
            "ok": False,
            "reason": "node_start_failed",
            "scan_active": False,
            "scan_mode": mode,
            "dependency_status": dependency_status,
            "ros_enabled": ros.enabled,
        }
    mapping_prereq = _wait_for_mapping_prereq_after_dependency_start(dependency_status)
    if not mapping_prereq["ready"]:
        logger.warning("scan start rejected blockers=%s warnings=%s", mapping_prereq["blockers"], mapping_prereq["warnings"])
        return {
            "ok": False,
            "reason": "mapping_prereq_failed",
            "scan_active": False,
            "mapping_prereq": mapping_prereq,
            "ros_enabled": ros.enabled,
        }
    _reset_scan_session()
    SCAN_SESSION["mode"] = mode
    SCAN_SESSION["dependency_status"] = copy.deepcopy(dependency_status)
    SCAN_SESSION["active"] = True
    SCAN_SESSION["started_at"] = time.time()
    if ros.enabled and ros.bridge is not None:
        ros.bridge.set_scan_active(True)
    else:
        sim.scanning = True
    logger.info("scan started")
    return {
        "ok": True,
        "scan_active": True,
        "scan_mode": mode,
        "scan_summary": _scan_summary(),
        "dependency_status": dependency_status,
        "ros_enabled": ros.enabled,
    }


@app.post("/scan/stop")
async def stop_scan(req: StopScanRequest | None = None) -> dict:
    mode = _normalize_scan_mode(req.mode if req is not None else str(SCAN_SESSION.get("mode", "2d")))
    if mode is None:
        return {
            "ok": False,
            "reason": "invalid_scan_mode",
            "scan_active": bool(SCAN_SESSION["active"]),
        }
    if not bool(SCAN_SESSION["active"]):
        process_stop_status = _stop_launched_scan_processes()
        logger.info("scan stop requested while inactive process_stop_status=%s", process_stop_status)
        return {
            "ok": False,
            "reason": "scan_not_active",
            "scan_active": False,
            "scan_mode": mode,
            "process_stop_status": process_stop_status,
        }
    SCAN_SESSION["active"] = False
    SCAN_SESSION["stopped_at"] = time.time()
    process_stop_status = _stop_launched_scan_processes()
    if ros.enabled and ros.bridge is not None:
        ros.bridge.set_scan_active(False)
    else:
        sim.scanning = False
    logger.info("scan stopped")
    if str(SCAN_SESSION["mode"]) == "3d":
        pcd_path = _pcd_output_path_for_mode("3d")
        if pcd_path is None:
            SCAN_SESSION["pcd_transfer_state"] = "error"
            return {
                "ok": False,
                "reason": "pcd_path_not_configured",
                "scan_active": False,
                "scan_mode": "3d",
                "process_stop_status": process_stop_status,
            }
        if not pcd_path.exists():
            SCAN_SESSION["pcd_transfer_state"] = "error"
            return {
                "ok": False,
                "reason": "pcd_file_missing",
                "scan_active": False,
                "scan_mode": "3d",
                "error": f"pcd file not found: {pcd_path}",
                "process_stop_status": process_stop_status,
            }
        try:
            SCAN_SESSION["pcd_transfer_state"] = "reading"
            content = pcd_path.read_bytes()
        except OSError as exc:
            SCAN_SESSION["pcd_transfer_state"] = "error"
            return {
                "ok": False,
                "reason": "pcd_read_failed",
                "scan_active": False,
                "scan_mode": "3d",
                "error": str(exc),
                "process_stop_status": process_stop_status,
            }
        encoded = base64.b64encode(content).decode("ascii")
        pcd_file = {
            "name": pcd_path.name,
            "size": len(content),
            "encoding": "base64",
            "content": encoded,
        }
        SCAN_SESSION["pcd_file"] = copy.deepcopy(pcd_file)
        SCAN_SESSION["pcd_transfer_state"] = "ready"
        return {
            "ok": True,
            "scan_active": False,
            "scan_mode": "3d",
            "scan_summary": _scan_summary(),
            "pcd_file": pcd_file,
            "ros_enabled": ros.enabled,
            "process_stop_status": process_stop_status,
        }
    return {
        "ok": True,
        "scan_active": False,
        "scan_mode": str(SCAN_SESSION["mode"]),
        "scan_summary": _scan_summary(),
        "ros_enabled": ros.enabled,
        "process_stop_status": process_stop_status,
    }


@app.post("/scan/reset")
async def reset_scan() -> dict:
    _reset_scan_session()
    return {"ok": True, "scan_summary": _scan_summary()}


@app.post("/control/move")
async def move(cmd: MoveCommand) -> dict:
    if ros.enabled and ros.bridge is not None:
        global motion_command_seq
        motion_command_seq += 1
        command_seq = motion_command_seq
        ros.bridge.publish_cmd_vel(cmd.velocity, cmd.yaw_rate)
        await asyncio.sleep(cmd.duration)
        if command_seq == motion_command_seq:
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


@app.post("/control/target")
async def set_control_target(cmd: ControlTargetRequest) -> dict:
    CONTROL_TARGET["velocity"] = float(cmd.velocity)
    CONTROL_TARGET["yaw_rate"] = float(cmd.yaw_rate)
    CONTROL_TARGET["updated_at"] = time.time()
    if ros.enabled and ros.bridge is not None:
        _record_control_publish_source("api_target", float(cmd.velocity), float(cmd.yaw_rate))
        ros.bridge.publish_cmd_vel(float(cmd.velocity), float(cmd.yaw_rate))
        state = {
            "pose": ros.bridge.latest_pose(),
            "gps": ros.bridge.latest_gps(),
            "chassis": ros.bridge.latest_chassis(),
        }
        return {"ok": True, "msg": "control target applied", "state": state}
    _record_control_publish_source("api_target", float(cmd.velocity), float(cmd.yaw_rate))
    sim.set_motion(float(cmd.velocity), float(cmd.yaw_rate))
    return {"ok": True, "msg": "control target applied", "state": sim.state.__dict__}


@app.post("/control/stop")
async def stop() -> dict:
    global motion_command_seq
    motion_command_seq += 1
    CONTROL_TARGET["velocity"] = 0.0
    CONTROL_TARGET["yaw_rate"] = 0.0
    CONTROL_TARGET["updated_at"] = time.time()
    _record_control_publish_source("api_stop", 0.0, 0.0)
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


@app.post("/map/reset")
async def reset_map() -> dict:
    global latest_points

    if not (ros.enabled and ros.bridge is not None):
        return {"ok": False, "reason": "ros_unavailable", "map_source": _current_map_source()}
    if not hasattr(ros.bridge, "reset_map"):
        return {"ok": False, "reason": "reset_unavailable", "map_source": _current_map_source()}
    if not bool(ros.bridge.reset_map()):
        return {"ok": False, "reason": "slam_toolbox_reset_failed", "map_source": _current_map_source()}

    latest_points = []
    _reset_scan_session()
    logger.info("map reset requested via slam_toolbox")
    return {"ok": True, "map_source": _current_map_source(), "scan_summary": _scan_summary()}


@app.post("/map/save")
async def save_map(req: SaveMapRequest) -> dict:
    points_to_save = _current_map_points()
    if req.voxel_size is not None:
        SCAN_SESSION["voxel_size"] = max(0.02, float(req.voxel_size))
    if not points_to_save:
        return {"ok": False, "reason": "map_unavailable", "map_source": _current_map_source(), "scan_summary": _scan_summary()}

    pose = ros.bridge.latest_pose() if ros.enabled and ros.bridge is not None else {
        "x": sim.state.x,
        "y": sim.state.y,
        "yaw": sim.state.yaw,
    }
    gps = ros.bridge.latest_gps() if ros.enabled and ros.bridge is not None else {}
    imu = ros.bridge.latest_imu() if ros.enabled and ros.bridge is not None else {}
    filename = f"{req.name}_{int(time.time())}.slam"
    target = map_dir / filename
    pcd_file = copy.deepcopy(SCAN_SESSION.get("pcd_file"))
    bundle = {
        "version": "slam.v3",
        "scan_mode": str(SCAN_SESSION.get("mode", "2d")),
        "notes": req.notes,
        "created_at": time.time(),
        "source": "ros" if ros.enabled else "sim",
        "map_source": _current_map_source(),
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
    if isinstance(pcd_file, dict):
        bundle["pcd_file"] = pcd_file
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
            "pcd": isinstance(pcd_file, dict),
        },
        "scan_summary": _scan_summary(),
        "scan_mode": str(SCAN_SESSION.get("mode", "2d")),
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
    SCAN_SESSION["mode"] = str(bundle.get("scan_mode", "2d"))
    SCAN_SESSION["pcd_file"] = copy.deepcopy(bundle.get("pcd_file"))
    return {
        "ok": True,
        "point_count": len(latest_points),
        "poi_count": len(sim.state.poi),
        "path_count": len(sim.state.path),
        "chassis_count": len(sim.state.chassis_track),
        "scan_mode": str(SCAN_SESSION["mode"]),
        "contains": {"pcd": isinstance(bundle.get("pcd_file"), dict)},
        "scan_summary": _scan_summary(),
    }


@app.get("/map/list")
async def list_map() -> dict:
    files = sorted([path.name for path in map_dir.glob("*.slam")])
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
                SCAN_SESSION["front_frames"] += 1
                SCAN_SESSION["raw_points"] += len(message["payload"].get("points", []))
            elif topic == "/lidar/rear":
                SCAN_SESSION["rear_frames"] += 1
                SCAN_SESSION["raw_points"] += len(message["payload"].get("points", []))
            elif topic == "/map/grid":
                latest_points = _occupancy_payload_to_points(message["payload"])

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
        nonlocal last_client_activity
        while True:
            message_type, payload = await outbound_queue.get()
            if message_type == "json":
                await websocket.send_json(payload)
                last_client_activity = time.time()
            elif message_type == "text":
                await websocket.send_text(payload)
                last_client_activity = time.time()

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
