from __future__ import annotations

import base64
import hashlib
import json
import logging
import math
import os
import queue
import re
import struct
import sys
import tempfile
import threading
import time
import tkinter as tk
import zipfile
import traceback
from dataclasses import dataclass
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from urllib.parse import urlparse
from logging.handlers import RotatingFileHandler

import requests
import websocket

try:
    from .native_map_import import NativeMapImportTool
    from .logic import (
        Point,
        build_scan_fusion_metadata,
        build_poi_copy_text,
        compute_path_closed_loop_validation,
        extract_scan_fusion_config,
        infer_missing_geo_points,
        is_occupied_scan_cell,
        optimize_loop_with_two_opt,
        parse_batch_poi_text,
        plan_path_points,
        resolve_scan_fusion_config,
        should_skip_scan_by_turn,
        solve_nearest_loop,
    )
except ImportError:
    from native_map_import import NativeMapImportTool  # type: ignore
    from logic import (  # type: ignore
        Point,
        build_scan_fusion_metadata,
        build_poi_copy_text,
        compute_path_closed_loop_validation,
        extract_scan_fusion_config,
        infer_missing_geo_points,
        is_occupied_scan_cell,
        optimize_loop_with_two_opt,
        parse_batch_poi_text,
        plan_path_points,
        resolve_scan_fusion_config,
        should_skip_scan_by_turn,
        solve_nearest_loop,
    )


@dataclass
class Poi:
    client_id: str
    name: str
    x: float
    y: float
    yaw: float = 0.0
    lat: float | None = None
    lon: float | None = None


def normalize_server_ws_url(raw: str) -> str:
    text = raw.strip()
    if not text:
        return ""
    if "://" not in text:
        text = f"ws://{text}"
    if text.startswith("http://"):
        text = "ws://" + text[len("http://") :]
    elif text.startswith("https://"):
        text = "wss://" + text[len("https://") :]
    parsed = urlparse(text)
    path = parsed.path or ""
    if not path or path == "/":
        path = "/ws/stream"
    elif path.endswith("/"):
        path = f"{path}ws/stream"
    elif path != "/ws/stream":
        path = f"{path}/ws/stream" if not path.endswith("/ws/stream") else path
    return parsed._replace(path=path).geturl()


def normalize_http_base_url(raw: str) -> str:
    text = raw.strip()
    if not text:
        return ""
    if "://" not in text:
        text = f"http://{text}"
    parsed = urlparse(text)
    scheme = "https" if parsed.scheme == "https" else "http"
    if parsed.netloc:
        netloc = parsed.netloc
        path = parsed.path.rstrip("/")
    else:
        netloc = parsed.path.rstrip("/")
        path = ""
    return parsed._replace(scheme=scheme, netloc=netloc, path=path, params="", query="", fragment="").geturl().rstrip("/")


def compose_http_base_url(host: str, port: str) -> str:
    normalized_host = host.strip()
    normalized_port = port.strip()
    if not normalized_host:
        return ""
    if normalized_port:
        return normalize_http_base_url(f"{normalized_host}:{normalized_port}")
    return normalize_http_base_url(normalized_host)


def build_direct_ws_url(host: str, port: str) -> str:
    normalized_host = host.strip()
    normalized_port = port.strip()
    if not normalized_host:
        return ""
    if normalized_port:
        return normalize_server_ws_url(f"{normalized_host}:{normalized_port}")
    return normalize_server_ws_url(normalized_host)


def redact_sensitive_text(text: str) -> str:
    redacted = str(text)
    redacted = re.sub(r"(https?|wss?)://[^\s\"']+", "<redacted-url>", redacted, flags=re.IGNORECASE)
    redacted = re.sub(r"\b\d{1,3}(?:\.\d{1,3}){3}:\d+\b", "<redacted-host>", redacted)
    redacted = re.sub(r"\b(?:[A-Za-z0-9_-]{16,}|[A-Fa-f0-9]{24,})\b", "<redacted-token>", redacted)
    return redacted


class AuthFlowError(RuntimeError):
    def __init__(self, user_message: str, detail: str | None = None) -> None:
        super().__init__(detail or user_message)
        self.user_message = redact_sensitive_text(user_message)
        self.detail = detail or user_message


def load_desktop_client_config(config_path: Path | None = None) -> dict[str, Any]:
    defaults = {
        "login_required": True,
        "gateway_ip": "192.168.3.56",
        "gateway_port": 28080,
        "server_ip": "127.0.0.1",
        "server_port": 8080,
        "username": "admin",
    }
    candidates = [config_path] if config_path is not None else [
        runtime_base_dir() / "client_config.json",
        Path.cwd() / "client_config.json",
    ]
    for candidate in candidates:
        if candidate is None or not candidate.exists():
            continue
        try:
            parsed = json.loads(candidate.read_text(encoding="utf-8"))
        except Exception:
            continue
        if isinstance(parsed, dict):
            merged = dict(defaults)
            merged.update(parsed)
            return merged
    return defaults


def _extract_json_payload(response: Any, action: str) -> dict[str, Any]:
    try:
        payload = response.json()
    except Exception as exc:  # noqa: BLE001
        raise AuthFlowError(f"{action} returned invalid JSON", f"{action} returned invalid JSON: {exc}") from exc
    if not isinstance(payload, dict):
        raise AuthFlowError(f"{action} returned invalid payload")
    ret_code = payload.get("retCode", 200)
    if int(ret_code) != 200:
        raise AuthFlowError(str(payload.get("retMsg", f"{action} failed")), f"{action} failed: {payload.get('retMsg', 'unknown error')}")
    return payload


def bootstrap_authenticated_bridge(
    base_url: str,
    username: str,
    password: str,
    session: requests.Session | None = None,
    timeout_sec: float = 3.0,
) -> dict[str, str]:
    owned_session = session is None
    http = session or requests.Session()
    if hasattr(http, "trust_env"):
        http.trust_env = False
    try:
        login_response = http.post(
            f"{normalize_http_base_url(base_url)}/sysUser/userLogin",
            json={"userName": username, "userPwd": password},
            timeout=timeout_sec,
        )
        login_response.raise_for_status()
        login_payload = _extract_json_payload(login_response, "login")
        login_data = login_payload.get("retData") or {}
        if not isinstance(login_data, dict) or not str(login_data.get("tokenID", "")).strip():
            raise AuthFlowError("login succeeded but tokenID is missing")
        token = str(login_data["tokenID"]).strip()
        url_response = http.post(
            f"{normalize_http_base_url(base_url)}/sys/getVcuUrl",
            json={},
            headers={"Authorization": token},
            timeout=timeout_sec,
        )
        url_response.raise_for_status()
        url_payload = _extract_json_payload(url_response, "getVcuUrl")
        url_data = url_payload.get("retData") or {}
        if not isinstance(url_data, dict):
            raise AuthFlowError("getVcuUrl returned invalid retData")
        ws_url = normalize_server_ws_url(str(url_data.get("ws", "")).strip())
        http_url = str(url_data.get("http", "")).strip()
        if not ws_url:
            raise AuthFlowError("getVcuUrl succeeded but ws address is missing")
        if not http_url:
            raise AuthFlowError("getVcuUrl succeeded but http address is missing")
        return {
            "base_url": normalize_http_base_url(base_url),
            "user_name": str(login_data.get("userName") or username).strip(),
            "token": token,
            "http_url": http_url,
            "ws_url": ws_url,
        }
    finally:
        if owned_session:
            http.close()


def resolve_log_file_path(
    candidates: list[Path],
    mkdir_fn=None,
) -> Path:
    mkdir_fn = mkdir_fn or (lambda path: path.mkdir(parents=True, exist_ok=True))
    last_err: Exception | None = None
    for candidate in candidates:
        try:
            mkdir_fn(candidate.parent)
            with candidate.open("a", encoding="utf-8") as handle:
                handle.write("")
            return candidate
        except Exception as exc:
            last_err = exc
    raise OSError(f"unable to create log file in any candidate path: {last_err}")


def bootstrap_log_write(path: Path, message: str) -> None:
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a", encoding="utf-8") as handle:
            handle.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} | {message}\n")
    except Exception:
        pass


def build_camera_refresh_text(camera_inbox: dict[int, dict]) -> str:
    newest = max((item.get("meta", {}).get("received_at_ms", 0) for item in camera_inbox.values()), default=0)
    if not newest:
        return "No buffered frame"
    return f"Buffered latest {time.strftime('%H:%M:%S', time.localtime(newest / 1000))}"


def parse_camera_topic_id(topic: str) -> int | None:
    parts = topic.split("/")
    if len(parts) < 3:
        return None
    try:
        return int(parts[2])
    except (TypeError, ValueError):
        return None


def safe_mode_translation_key(value: str, mapping: dict[str, str], default_key: str) -> str:
    return mapping.get(value, default_key)


def safe_focus_widget(root: tk.Tk) -> tk.Misc | None:
    try:
        return root.focus_get()
    except (KeyError, tk.TclError):
        return None


def can_zoom_from_widget(widget: object, canvas: tk.Canvas) -> bool:
    current = widget
    while current is not None:
        if current is canvas:
            return True
        current = getattr(current, "master", None)
    return False


def should_clear_focus_on_click(widget: object) -> bool:
    return not isinstance(widget, (tk.Entry, tk.Text, tk.Listbox, ttk.Entry, ttk.Combobox, ttk.Button, ttk.Checkbutton))


def zoom_scale_factor(event: object) -> float:
    delta = getattr(event, "delta", 0)
    if delta > 0:
        return 1.08
    if delta < 0:
        return 0.92
    num = getattr(event, "num", None)
    if num == 4:
        return 1.08
    if num == 5:
        return 0.92
    return 1.0


def strip_legacy_trajectory(manifest: dict) -> dict:
    cleaned = dict(manifest)
    cleaned.pop("trajectory", None)
    return cleaned


def write_slam_archive(path: str | Path, manifest: dict, points: list[list[float]] | list[tuple[float, float, float]], pcd_file: dict[str, Any] | None) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    archive_manifest = dict(manifest)
    archive_manifest.pop("pcd_file", None)
    if isinstance(pcd_file, dict):
        archive_manifest["pcd"] = {"included": True, "file": str(pcd_file.get("name", "map.pcd"))}
    else:
        archive_manifest.setdefault("pcd", {"included": False, "file": ""})
    with zipfile.ZipFile(target, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("manifest.json", json.dumps(archive_manifest, ensure_ascii=False, indent=2))
        zf.writestr("map_points.bin", b"".join(struct.pack("fff", *point) for point in points))
        if isinstance(pcd_file, dict):
            pcd_name = str(pcd_file.get("name", "map.pcd"))
            pcd_content = pcd_file.get("content", b"")
            if isinstance(pcd_content, str):
                pcd_content = pcd_content.encode("utf-8")
            zf.writestr(pcd_name, bytes(pcd_content))
    return target


def read_slam_archive(path: str | Path) -> tuple[dict, list[tuple[float, float, float]], dict[str, Any] | None]:
    target = Path(path)
    with zipfile.ZipFile(target, "r") as zf:
        manifest = json.loads(zf.read("manifest.json"))
        raw = zf.read("map_points.bin")
        pcd_meta = manifest.get("pcd")
        pcd_file = None
        if isinstance(pcd_meta, dict) and pcd_meta.get("included"):
            pcd_name = str(pcd_meta.get("file", "map.pcd"))
            pcd_file = {"name": pcd_name, "content": zf.read(pcd_name)}
    points = [struct.unpack("fff", raw[i:i + 12]) for i in range(0, len(raw), 12) if i + 12 <= len(raw)]
    return manifest, points, pcd_file


KEYUP_STOP_CONFIRM_MS = 50
HEALTH_POLL_INTERVAL_SEC = 15.0
MAX_MESSAGES_PER_TICK = 12
MAX_MESSAGES_DRAIN_PER_TICK = 256
NETWORK_LAG_DEGRADED_MS = 800
NETWORK_SILENCE_UNSTABLE_MS = 2000
NETWORK_RECENT_WS_ISSUE_MS = 5000
NETWORK_CONTROL_FAILURE_UNSTABLE_COUNT = 2
MIN_SCAN_TRANSLATION_DELTA_M = 0.05
MIN_SCAN_ROTATION_DELTA_RAD = 0.03
MAX_FREE_DISPLAY_CELLS = 12000


def summarize_mapping_status(health: dict) -> str:
    status = str(health.get("mapping_status", "ok"))
    blockers = list(health.get("mapping_blockers", []) or [])
    warnings = list(health.get("mapping_warnings", []) or [])
    if blockers:
        return f"blocked: {blockers[0]}"
    if warnings:
        return f"warn: {warnings[0]}"
    if status == "ok" and "mapping_ready" in health:
        return "ready"
    return status


def classify_network_quality(
    bridge_connected: bool,
    stream_health: dict,
    last_message_at_ms: int,
    now_ms: int | None = None,
    recent_ws_issue_ms: int = 0,
) -> str:
    if not bridge_connected:
        return "offline"
    now_ms = int(time.time() * 1000) if now_ms is None else int(now_ms)
    if int(stream_health.get("control_failures_consecutive", 0) or 0) >= NETWORK_CONTROL_FAILURE_UNSTABLE_COUNT:
        return "unstable"
    if recent_ws_issue_ms and now_ms - int(recent_ws_issue_ms) <= NETWORK_RECENT_WS_ISSUE_MS:
        return "unstable"
    if last_message_at_ms and now_ms - int(last_message_at_ms) > NETWORK_SILENCE_UNSTABLE_MS:
        return "unstable"
    if int(stream_health.get("last_lag_ms", 0) or 0) > NETWORK_LAG_DEGRADED_MS:
        return "degraded"
    if int(stream_health.get("gap_err", 0) or 0) > int(stream_health.get("network_quality_gap_seen", 0) or 0):
        return "degraded"
    if int(stream_health.get("checksum_err", 0) or 0) > int(stream_health.get("network_quality_checksum_seen", 0) or 0):
        return "degraded"
    return "ok"


def mapping_prereq_message(summary: dict) -> str:
    blockers = list(summary.get("blockers", []) or [])
    warnings = list(summary.get("warnings", []) or [])
    if blockers:
        return "\n".join(blockers)
    if warnings:
        return "\n".join(warnings)
    return "mapping prerequisites not satisfied"


def _normalize_angle_delta(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def pose_progress_exceeds_threshold(
    previous_pose: dict[str, float] | None,
    current_pose: dict[str, float] | None,
    min_translation_m: float = MIN_SCAN_TRANSLATION_DELTA_M,
    min_rotation_rad: float = MIN_SCAN_ROTATION_DELTA_RAD,
) -> bool:
    if previous_pose is None or current_pose is None:
        return True
    dx = float(current_pose.get("x", 0.0)) - float(previous_pose.get("x", 0.0))
    dy = float(current_pose.get("y", 0.0)) - float(previous_pose.get("y", 0.0))
    if math.hypot(dx, dy) >= float(min_translation_m):
        return True
    yaw_delta = _normalize_angle_delta(float(current_pose.get("yaw", 0.0)) - float(previous_pose.get("yaw", 0.0)))
    return abs(yaw_delta) >= float(min_rotation_rad)


class LatestScanFrameBuffer:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: dict[str, Any] | None = None

    def submit(self, frame: dict[str, Any]) -> None:
        with self._lock:
            self._latest = frame

    def pop_latest(self) -> dict[str, Any] | None:
        with self._lock:
            frame = self._latest
            self._latest = None
            return frame


def prune_scan_cache(scan: dict[str, Any], config: dict[str, Any]) -> None:
    occupied = scan.get("occupied")
    free = scan.get("free")
    if not isinstance(occupied, dict) or not isinstance(free, dict):
        return
    retained_occupied: dict[str, dict[str, Any]] = {}
    for key, cell in occupied.items():
        free_cell = free.get(key)
        if not is_occupied_scan_cell(cell, free_cell, config):
            continue
        retained_occupied[key] = cell
    retained_free_items = sorted(
        (dict(item) for item in free.values()),
        key=lambda item: (int(item.get("hits", 0)), -abs(int(item.get("ix", 0))) - abs(int(item.get("iy", 0)))),
        reverse=True,
    )[:MAX_FREE_DISPLAY_CELLS]
    retained_free = {_cell_key(int(item["ix"]), int(item["iy"])): item for item in retained_free_items}
    scan["occupied"] = retained_occupied
    scan["free"] = retained_free


def coalesce_stream_messages(messages: list[dict[str, Any]]) -> list[dict[str, Any]]:
    latest_by_topic: dict[str, dict[str, Any]] = {}
    passthrough: list[dict[str, Any]] = []
    for msg in messages:
        topic = str(msg.get("topic", ""))
        if topic in {"/robot/pose", "/robot/gps", "/chassis/odom", "/chassis/status", "/lidar/front", "/lidar/rear"} or topic.startswith("/camera/"):
            latest_by_topic[topic] = msg
        else:
            passthrough.append(msg)
    ordered_topics = [
        "/robot/pose",
        "/chassis/odom",
        "/chassis/status",
        "/robot/gps",
        "/lidar/front",
        "/lidar/rear",
    ]
    ordered_topics.extend(sorted(topic for topic in latest_by_topic if topic.startswith("/camera/")))
    coalesced = [latest_by_topic[topic] for topic in ordered_topics if topic in latest_by_topic]
    return coalesced + passthrough


def _cell_key(ix: int, iy: int) -> str:
    return f"{ix}:{iy}"


def _mark_free(scan: dict[str, Any], ix: int, iy: int) -> None:
    key = _cell_key(ix, iy)
    slot = scan["free"].get(key, {"ix": ix, "iy": iy, "hits": 0})
    slot["hits"] += 1
    scan["free"][key] = slot


def _mark_occupied(scan: dict[str, Any], ix: int, iy: int, intensity: float, hits: int = 1) -> None:
    key = _cell_key(ix, iy)
    slot = scan["occupied"].get(key, {"ix": ix, "iy": iy, "hits": 0, "intensity": 0.0})
    slot["hits"] = max(int(slot["hits"]) + hits, hits)
    slot["intensity"] = max(float(slot["intensity"]), float(intensity))
    scan["occupied"][key] = slot


def _world_to_cell(scan: dict[str, Any], x: float, y: float) -> tuple[int, int]:
    voxel = float(scan["voxel"])
    return round(x / voxel), round(y / voxel)


def _raytrace(scan: dict[str, Any], start_x: int, start_y: int, end_x: int, end_y: int) -> None:
    dx = end_x - start_x
    dy = end_y - start_y
    steps = max(abs(dx), abs(dy))
    if steps <= 1:
        return
    for step in range(steps):
        t = step / steps
        _mark_free(scan, round(start_x + dx * t), round(start_y + dy * t))


def process_scan_frame(
    scan: dict[str, Any],
    points: list[Any],
    pose: dict[str, Any],
    keyframe: bool,
    config: dict[str, Any],
    logger: logging.Logger | None = None,
) -> bool:
    if not scan.get("active") or not points:
        return False
    if should_skip_scan_by_turn(float(pose.get("wz", 0.0)), keyframe, config):
        return False
    if not pose_progress_exceeds_threshold(scan.get("last_accum_pose"), pose):
        return False
    scan["raw_points"] += len(points)
    pose_x = float(pose.get("x", 0.0))
    pose_y = float(pose.get("y", 0.0))
    robot_ix, robot_iy = _world_to_cell(scan, pose_x, pose_y)
    changed = False
    for point in points:
        if not isinstance(point, (list, tuple)) or len(point) < 2:
            if logger is not None:
                logger.warning("skip invalid lidar point payload=%s", point)
            continue
        world_x, world_y = float(point[0]), float(point[1])
        intensity = float(point[2]) if len(point) > 2 else 1.0
        ix, iy = _world_to_cell(scan, world_x, world_y)
        if keyframe:
            _raytrace(scan, robot_ix, robot_iy, ix, iy)
        _mark_occupied(scan, ix, iy, intensity)
        changed = True
    if not changed:
        return False
    scan["last_accum_pose"] = {
        "x": float(pose.get("x", 0.0)),
        "y": float(pose.get("y", 0.0)),
        "yaw": float(pose.get("yaw", 0.0)),
    }
    prune_scan_cache(scan, config)
    return True


def runtime_base_dir() -> Path:
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent
    return Path(__file__).resolve().parent


def compute_log_candidates(
    platform_name: str,
    env: dict[str, str],
    home_dir: Path,
    runtime_dir: Path,
    temp_dir: Path,
    cwd: Path,
) -> list[Path]:
    candidates = [runtime_dir / "logs" / "client_desktop.log"]
    if platform_name.startswith("win"):
        local_appdata = env.get("LOCALAPPDATA", "").strip()
        if local_appdata:
            candidates.append(Path(local_appdata) / "AutoDriveClient" / "logs" / "client_desktop.log")
        candidates.append(home_dir / "AppData" / "Local" / "AutoDriveClient" / "logs" / "client_desktop.log")
    elif platform_name == "darwin":
        candidates.append(home_dir / "Library" / "Logs" / "AutoDriveClient" / "logs" / "client_desktop.log")
        candidates.append(home_dir / ".local" / "state" / "AutoDriveClient" / "logs" / "client_desktop.log")
    else:
        xdg_state = env.get("XDG_STATE_HOME", "").strip()
        if xdg_state:
            candidates.append(Path(xdg_state) / "AutoDriveClient" / "logs" / "client_desktop.log")
        candidates.append(home_dir / ".local" / "state" / "AutoDriveClient" / "logs" / "client_desktop.log")
        candidates.append(home_dir / ".cache" / "AutoDriveClient" / "logs" / "client_desktop.log")
    candidates.append(temp_dir / "AutoDriveClient" / "logs" / "client_desktop.log")
    candidates.append(cwd / "logs" / "client_desktop.log")
    return candidates


class ServerBridge:
    def __init__(self, ws_url: str, logger: logging.Logger | None = None) -> None:
        self.ws_url = normalize_server_ws_url(ws_url)
        self.http_base = self.ws_url.replace("ws://", "http://").replace("wss://", "https://").replace("/ws/stream", "")
        self.ws_host = urlparse(self.ws_url).hostname or ""
        self.logger = logger or logging.getLogger("autodrive.client_desktop")
        self.queue: queue.Queue[dict] = queue.Queue()
        self.connected = False
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.ws: websocket.WebSocketApp | None = None
        self.session = requests.Session()
        self.session.trust_env = False
        self.reconnect_count = 0
        self.last_ws_issue_at_ms = 0

    def start(self) -> None:
        if self.thread and self.thread.is_alive():
            return
        self.stop_event.clear()
        self.logger.info("bridge start ws_url=%s http_base=%s", self.ws_url, self.http_base)
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self.connected = False
        self.logger.info("bridge stop")
        if self.ws is not None:
            try:
                self.ws.close()
            except Exception:
                self.logger.exception("bridge ws close failed")

    def post(self, path: str, body: dict, retries: int = 3, timeout_sec: float = 4.0, backoff_base_sec: float = 0.2) -> dict:
        last_err: Exception | None = None
        for index in range(retries + 1):
            try:
                self.logger.info("http post path=%s try=%s", path, index + 1)
                res = self.session.post(f"{self.http_base}{path}", json=body, timeout=timeout_sec)
                res.raise_for_status()
                return res.json()
            except Exception as exc:
                last_err = exc
                self.logger.warning("http post failed path=%s try=%s err=%s", path, index + 1, exc)
                if index < retries and backoff_base_sec > 0:
                    time.sleep(backoff_base_sec * (2**index))
        raise RuntimeError(str(last_err))

    def post_async(self, path: str, body: dict, retries: int = 1) -> None:
        def worker() -> None:
            session = requests.Session()
            session.trust_env = False
            last_err: Exception | None = None
            try:
                for index in range(retries + 1):
                    try:
                        self.logger.info("http post async path=%s try=%s", path, index + 1)
                        res = session.post(f"{self.http_base}{path}", json=body, timeout=1.5)
                        res.raise_for_status()
                        return
                    except Exception as exc:
                        last_err = exc
                        self.logger.warning("http post async failed path=%s try=%s err=%s", path, index + 1, exc)
                        time.sleep(0.1 * (2**index))
                raise RuntimeError(str(last_err))
            except Exception:
                self.logger.exception("http post async aborted path=%s", path)
            finally:
                session.close()

        threading.Thread(target=worker, daemon=True).start()

    def get(self, path: str) -> dict:
        self.logger.info("http get path=%s", path)
        res = self.session.get(f"{self.http_base}{path}", timeout=2)
        res.raise_for_status()
        return res.json()

    def _loop(self) -> None:
        retry = 0

        def on_message(_ws, msg: str) -> None:
            if msg == "pong":
                return
            try:
                self.queue.put(json.loads(msg))
            except Exception:
                self.logger.exception("ws message parse failed")

        def on_open(_ws) -> None:
            nonlocal retry
            retry = 0
            self.connected = True
            self.logger.info("ws opened host=%s", self.ws_host)

        def on_close(_ws, _code, _msg) -> None:
            self.connected = False
            self.last_ws_issue_at_ms = int(time.time() * 1000)
            self.logger.warning("ws closed code=%s msg=%s", _code, _msg)

        def on_error(_ws, _err) -> None:
            self.connected = False
            self.last_ws_issue_at_ms = int(time.time() * 1000)
            self.logger.error("ws error err=%s", _err)

        while not self.stop_event.is_set():
            self.ws = websocket.WebSocketApp(
                self.ws_url,
                on_message=on_message,
                on_open=on_open,
                on_close=on_close,
                on_error=on_error,
            )
            no_proxy_hosts = [self.ws_host, "127.0.0.1", "localhost"]
            self.ws.run_forever(
                ping_interval=5,
                ping_timeout=3,
                http_proxy_host=None,
                http_proxy_port=None,
                http_no_proxy=no_proxy_hosts,
            )
            self.connected = False
            if self.stop_event.is_set():
                break
            retry += 1
            self.reconnect_count += 1
            self.last_ws_issue_at_ms = int(time.time() * 1000)
            self.logger.warning("ws reconnect scheduled retry=%s", retry)
            time.sleep(min(10.0, 0.3 * (2 ** min(retry, 5))))


class DesktopClient:
    def __init__(self) -> None:
        self.i18n = {
            "en": {
                "title": "AutoDrive Desktop Map Tool",
                "server_ws": "Server WS",
                "login": "Login",
                "relogin": "Re-login",
                "login_title": "Login",
                "login_ip": "Gateway IP",
                "login_port": "Port",
                "server_ip": "Server IP",
                "server_port": "Server Port",
                "password": "Password",
                "auth_missing": "Not logged in",
                "auth_ready": "Logged in as {user}",
                "auth_bypass": "Debug direct mode",
                "connect_need_login": "Please log in first.",
                "login_empty_fields": "IP, username, and password are required.",
                "login_failed": "Login bootstrap failed",
                "login_failed_safe": "Login failed. Check credentials or gateway reachability.",
                "connect_failed_safe": "Connection failed. See log for details.",
                "connect": "Connect",
                "disconnect": "Disconnect",
                "refresh_cameras": "Refresh Cameras",
                "language": "Language",
                "english": "English",
                "chinese": "中文",
                "panel_visibility": "Panel Visibility",
                "move": "Move",
                "poi": "POI",
                "path": "Path",
                "map": "Map",
                "scan": "Scan",
                "start_scan": "Start Scan",
                "stop_scan": "Stop Scan",
                "clear": "Clear",
                "scan_starting": "Starting scan...",
                "scan_waiting_mapping": "Waiting for mapping data...",
                "map_name": "Map Name",
                "voxel": "Voxel",
                "notes": "Notes",
                "keyboard_inactive": "Keyboard inactive",
                "forward_speed": "Forward Speed",
                "reverse_speed": "Reverse Speed",
                "turn_rate": "Turn Rate",
                "cmd_duration": "Cmd Duration",
                "repeat_ms": "Repeat (ms)",
                "stop_on_keyup": "Stop on keyup",
                "forward": "Forward",
                "left": "Left",
                "stop": "Stop",
                "right": "Right",
                "reverse": "Reverse",
                "mode": "Mode",
                "batch": "Batch",
                "single": "Single",
                "edit": "Edit Existing",
                "free_points": "Free Points",
                "batch_input_hint": "Batch POI Input (name or name,lon,lat or name,lon,lat,yaw)",
                "batch_examples": "Examples:\nA\nB,120.123456,30.123456\nC,120.223456,30.223456,1.570",
                "start_batch_add": "Start Batch Add",
                "cancel_batch": "Cancel Batch ({count})",
                "name": "Name",
                "x": "X",
                "y": "Y",
                "yaw": "Yaw",
                "geo": "Geo (lon,lat)",
                "add_single_poi": "Add Single POI",
                "edit_hint": "Select exactly one POI in the list before editing.",
                "apply_edit": "Apply Edit",
                "delete_selected": "Delete Selected",
                "copy_poi": "Copy POI",
                "show_poi": "Show POI",
                "poi_list": "POI List",
                "path_hint": "Obstacle-aware route planning, free-point draw, named POI connect, delete, and closed-loop validation.",
                "path_tool": "Path Tool",
                "safe_clearance": "Safe Clearance",
                "start_poi_name": "Start POI Name",
                "end_poi_name": "End POI Name",
                "auto_loop": "Auto Loop",
                "connect_named_poi": "Connect Named POI",
                "clear_selection": "Clear Selection",
                "closed_loop_check": "Closed-Loop Check",
                "delete_segment": "Delete Segment",
                "show_path": "Show Path",
                "map_tool": "Map Tool",
                "brush_radius": "Brush Radius",
                "auto_clear_noise": "Auto Clear Noise",
                "clear_loaded_map": "Clear Loaded Map",
                "reset_map": "Start Over",
                "save_map": "Save Map",
                "load_map": "Load Map",
                "export_pgm": "Export PGM",
                "export_yaml": "Export YAML",
                "export_json": "Export JSON",
                "export_pcd": "Export PCD",
                "scan_mode_label": "Scan Mode",
                "scan_receiving_pcd": "Receiving PCD from server",
                "map_view": "Map View",
                "center_robot": "Center Robot",
                "center_loaded_map": "Center Loaded Map",
                "reset_view": "Reset View",
                "zoom_out": "Zoom Out",
                "zoom_in": "Zoom In",
                "show_robot": "Show Robot",
                "odom_scan": "Odom And Scan",
                "cameras": "Cameras",
                "comm_map": "Communication / Map",
                "disconnected": "Disconnected",
                "no_health": "No health yet",
                "idle": "Idle",
                "poi_idle": "POI idle",
                "scan_session": "Scan Session",
                "tool_view": "Tool: View / Select",
                "left_notice": "Not enough space. Close some left-side panels or enlarge the window.",
                "loaded_map_edit": "View / Select mode active. Load a map file to start second-stage map editing.",
                "ws_offline": "WS offline",
                "connecting": "Connecting",
                "connected": "Connected",
                "status_line": "WS {ws} | scan {scan} | poi {poi} | path {path}",
                "status_detail": "WS ok | clients {clients} | scan {scan} | ros {ros}",
                "status_detail_mapping": "WS ok | clients {clients} | scan {scan} | ros {ros} | map {mapping}",
                "network_ok": "link ok",
                "network_degraded": "network lagging",
                "network_unstable": "network unstable, control may lag",
                "network_offline": "network offline",
                "recording_obstacles": "Recording {count} obstacle cells",
                "stopped_summary": "Stopped | {obs} obs / {free} safe",
                "loaded_badge": "Loaded: {name}",
                "tool_badge": "Tool: {name}{suffix}",
                "tool_view_select": "View / Select",
                "tool_erase_noise": "Erase Noise",
                "tool_draw_obstacle": "Draw Obstacle",
                "stats_badge": "{occ} obstacle cells | {poi} POI | {path} paths",
                "warning_disconnected": "Connect to the server first.",
                "api_error": "API Error",
                "poi_geo_format": "{label} format must be lon,lat.",
                "poi_geo_min3": "When any POI geo is provided, at least 3 POI must have lon,lat.",
                "poi_batch_requires_input": "Batch mode requires input rows.",
                "poi_ready_place": "Ready to place \"{name}\" on canvas",
                "poi_copied": "POI data copied to clipboard.",
                "poi_no_copy": "No POI to copy.",
                "poi_select_delete": "Select POI to delete.",
                "poi_select_first": "Select POI first.",
                "poi_single_requires_name": "Single mode requires a name.",
                "poi_edit_requires_one": "Edit mode requires exactly one selected POI.",
                "poi_added": "Added \"{name}\"",
                "poi_updated": "Updated \"{name}\"",
                "path_need_names": "Input both POI names first.",
                "path_poi_not_found": "POI \"{name}\" not found.",
                "path_poi_duplicate": "POI name \"{name}\" is duplicated.",
                "path_same_poi": "Start and end POI cannot be the same.",
                "path_need_two_poi": "At least two POI are required.",
                "path_browse_only": "Browse only",
                "path_tool_named": "Input POI names to connect ({start} -> {end})",
                "path_tool_free": "Click any two points to connect",
                "path_status": "Path segments {segments} | Nodes {nodes} | Tool {tool}{pending}{validation}",
                "loop_unchecked": " | Loop unchecked",
                "loop_ok": " | Loop OK",
                "loop_error": " | Loop error {count} segment(s)",
                "keyboard_cmd": "Keyboard {cmd}",
                "keyboard_stop_keyup": "Keyboard stop on keyup",
                "map_edit_erase": "Erase Noise mode active. Brush radius {radius:.2f} m",
                "map_edit_obstacle": "Draw Obstacle Line mode active. Click two points on the map.",
                "map_edit_view": "View / Select mode active. You can pan, zoom, and select POI or path.",
                "obstacle_start": "Obstacle start fixed at ({x:.2f}, {y:.2f}). Click end point next.",
                "obstacle_added": "Added obstacle line.",
                "noise_cleared": "Auto cleared {count} noisy cells",
                "noise_none": "No isolated noise found",
                "map_cleared": "Loaded map cleared",
                "reset_map_done": "Map reset requested. Waiting for new /map data.",
                "reset_map_failed": "Map reset failed. Check slam_toolbox reset service.",
                "save_title": "Saved",
                "save_done": "Map saved:\n{path}",
                "load_title": "Loaded",
                "load_done": "Loaded map:\n{name}",
                "export_need_map": "Load or save a map file first.",
                "export_title": "Export",
                "export_done": "Exported {kind}:\n{path}",
                "path_validation_title": "Path Validation",
                "map_loaded_view": "Loaded {name} into main map view",
            },
            "zh": {
                "title": "AutoDrive 桌面扫图工具",
                "server_ws": "服务端 WS",
                "login": "登录",
                "relogin": "重新登录",
                "login_title": "登录",
                "login_ip": "网关 IP",
                "login_port": "端口",
                "server_ip": "服务端 IP",
                "server_port": "服务端端口",
                "password": "密码",
                "auth_missing": "未登录",
                "auth_ready": "已登录 {user}",
                "auth_bypass": "调试直连模式",
                "connect_need_login": "请先登录。",
                "login_empty_fields": "IP、用户名、密码都必须填写。",
                "login_failed": "登录引导失败",
                "login_failed_safe": "登录失败。请检查账号、密码或网关连通性。",
                "connect_failed_safe": "连接失败。详细信息见日志。",
                "connect": "连接",
                "disconnect": "断开",
                "refresh_cameras": "刷新相机",
                "language": "语言",
                "english": "English",
                "chinese": "中文",
                "panel_visibility": "面板显示",
                "move": "移动",
                "poi": "POI 点",
                "path": "路径",
                "map": "地图",
                "scan": "扫描",
                "start_scan": "开始扫描",
                "stop_scan": "停止扫描",
                "clear": "清空",
                "scan_starting": "正在启动扫描...",
                "scan_waiting_mapping": "正在等待建图数据...",
                "map_name": "地图名",
                "voxel": "体素大小",
                "notes": "备注",
                "keyboard_inactive": "键盘未激活",
                "forward_speed": "前进速度",
                "reverse_speed": "后退速度",
                "turn_rate": "转向角速度",
                "cmd_duration": "指令时长",
                "repeat_ms": "重复间隔(ms)",
                "stop_on_keyup": "松键即停",
                "forward": "前进",
                "left": "左转",
                "stop": "停止",
                "right": "右转",
                "reverse": "后退",
                "mode": "模式",
                "batch": "批量添加",
                "single": "单点添加",
                "edit": "编辑已有",
                "free_points": "任意两点",
                "batch_input_hint": "批量 POI 输入（name 或 name,lon,lat 或 name,lon,lat,yaw）",
                "batch_examples": "示例：\nA\nB,120.123456,30.123456\nC,120.223456,30.223456,1.570",
                "start_batch_add": "开始批量添加",
                "cancel_batch": "取消批量添加（剩余 {count}）",
                "name": "名称",
                "x": "X",
                "y": "Y",
                "yaw": "Yaw",
                "geo": "经纬度(lon,lat)",
                "add_single_poi": "添加单点",
                "edit_hint": "先在列表中选中且只选中一个 POI，再进行编辑。",
                "apply_edit": "应用修改",
                "delete_selected": "删除选中",
                "copy_poi": "复制 POI",
                "show_poi": "显示 POI",
                "poi_list": "POI 列表",
                "path_hint": "支持避障路径规划、任意两点画线、按 POI 名称连线、删除和闭环校验。",
                "path_tool": "路径工具",
                "safe_clearance": "安全距离",
                "start_poi_name": "起点 POI 名称",
                "end_poi_name": "终点 POI 名称",
                "auto_loop": "自动闭环",
                "connect_named_poi": "按名称连线",
                "clear_selection": "清空选择",
                "closed_loop_check": "闭环检查",
                "delete_segment": "删除线段",
                "show_path": "显示路径",
                "map_tool": "地图工具",
                "brush_radius": "画刷半径",
                "auto_clear_noise": "自动清噪",
                "clear_loaded_map": "清空已加载地图",
                "reset_map": "从头开始",
                "save_map": "保存地图",
                "load_map": "加载地图",
                "export_pgm": "导出 PGM",
                "export_yaml": "导出 YAML",
                "export_json": "导出 JSON",
                "export_pcd": "导出 PCD",
                "scan_mode_label": "扫描模式",
                "scan_receiving_pcd": "正在从服务端接收 PCD",
                "map_view": "地图视图",
                "center_robot": "居中机器人",
                "center_loaded_map": "居中已加载地图",
                "reset_view": "重置视图",
                "zoom_out": "缩小",
                "zoom_in": "放大",
                "show_robot": "显示机器人",
                "odom_scan": "里程计和扫描",
                "cameras": "相机",
                "comm_map": "通信 / 地图",
                "disconnected": "未连接",
                "no_health": "暂无健康状态",
                "idle": "空闲",
                "poi_idle": "POI 空闲",
                "scan_session": "扫描会话",
                "tool_view": "工具：查看 / 选择",
                "left_notice": "空间不足，请关闭部分左侧窗口或增大窗口。",
                "loaded_map_edit": "当前为查看 / 选择模式。先加载地图文件再进行二次编辑。",
                "ws_offline": "WS 离线",
                "connecting": "连接中",
                "connected": "已连接",
                "status_line": "WS {ws} | 扫描 {scan} | POI {poi} | 路径 {path}",
                "status_detail": "WS 正常 | 客户端 {clients} | 扫描 {scan} | ROS {ros}",
                "status_detail_mapping": "WS 正常 | 客户端 {clients} | 扫描 {scan} | ROS {ros} | 建图 {mapping}",
                "network_ok": "链路正常",
                "network_degraded": "网络延迟偏高",
                "network_unstable": "网络不稳定，控制可能延迟",
                "network_offline": "网络离线",
                "recording_obstacles": "正在记录 {count} 个障碍栅格",
                "stopped_summary": "已停止 | 障碍 {obs} / 空闲 {free}",
                "loaded_badge": "已加载：{name}",
                "tool_badge": "工具：{name}{suffix}",
                "tool_view_select": "查看 / 选择",
                "tool_erase_noise": "擦除噪点",
                "tool_draw_obstacle": "绘制障碍",
                "stats_badge": "{occ} 个障碍栅格 | {poi} 个 POI | {path} 条路径",
                "warning_disconnected": "请先连接服务端。",
                "api_error": "接口错误",
                "poi_geo_format": "{label} 格式必须是 lon,lat。",
                "poi_geo_min3": "只要填写了经纬度，就至少需要 3 个 POI 带 lon,lat。",
                "poi_batch_requires_input": "批量模式需要输入内容。",
                "poi_ready_place": "准备在画布上放置“{name}”",
                "poi_copied": "POI 数据已复制到剪贴板。",
                "poi_no_copy": "没有可复制的 POI。",
                "poi_select_delete": "请选择要删除的 POI。",
                "poi_select_first": "请先选择 POI。",
                "poi_single_requires_name": "单点模式必须填写名称。",
                "poi_edit_requires_one": "编辑模式要求且仅要求选中一个 POI。",
                "poi_added": "已添加“{name}”",
                "poi_updated": "已更新“{name}”",
                "path_need_names": "请先填写起点和终点 POI 名称。",
                "path_poi_not_found": "未找到 POI “{name}”。",
                "path_poi_duplicate": "POI 名称“{name}”重复。",
                "path_same_poi": "起点和终点不能是同一个 POI。",
                "path_need_two_poi": "至少需要两个 POI。",
                "path_browse_only": "仅浏览",
                "path_tool_named": "按 POI 名称连线（{start} -> {end}）",
                "path_tool_free": "在地图上点击任意两点连线",
                "path_status": "路径 {segments} 条 | 节点 {nodes} 个 | 工具 {tool}{pending}{validation}",
                "loop_unchecked": " | 未检查闭环",
                "loop_ok": " | 闭环正常",
                "loop_error": " | 闭环错误 {count} 段",
                "keyboard_cmd": "键盘控制 {cmd}",
                "keyboard_stop_keyup": "松键即停",
                "map_edit_erase": "当前为擦除噪点模式。画刷半径 {radius:.2f} 米",
                "map_edit_obstacle": "当前为绘制障碍线模式。请在地图上点击两个点。",
                "map_edit_view": "当前为查看 / 选择模式。可平移、缩放并选择 POI 或路径。",
                "obstacle_start": "障碍线起点已固定在 ({x:.2f}, {y:.2f})，请点击终点。",
                "obstacle_added": "已添加障碍线。",
                "noise_cleared": "已自动清理 {count} 个噪点",
                "noise_none": "未发现孤立噪点",
                "map_cleared": "已清空已加载地图",
                "reset_map_done": "已请求重置地图，等待新的 /map 数据。",
                "reset_map_failed": "地图重置失败，请检查 slam_toolbox reset 服务。",
                "save_title": "已保存",
                "save_done": "地图已保存：\n{path}",
                "load_title": "已加载",
                "load_done": "已加载地图：\n{name}",
                "export_need_map": "请先加载或保存一个地图文件。",
                "export_title": "导出",
                "export_done": "已导出 {kind}：\n{path}",
                "path_validation_title": "路径校验",
                "map_loaded_view": "已将 {name} 加载到主地图视图",
            },
        }
        self.root = tk.Tk()
        self.current_lang = "en"
        self.lang_choice_var = tk.StringVar(value=self.i18n["en"]["english"])
        self.root.title(self.tr("title"))
        self.root.geometry("1760x980")
        self.root.minsize(1380, 860)
        self.client_config = load_desktop_client_config()
        self.log_path = self.setup_logging()
        self.logger = logging.getLogger("autodrive.client_desktop")
        self.root.report_callback_exception = self.report_callback_exception

        self.bridge: ServerBridge | None = None
        self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0, "vx": 0.0, "wz": 0.0}
        self.gps = {"lat": 0.0, "lon": 0.0}
        self.odom = {"x": 0.0, "y": 0.0, "yaw": 0.0, "vx": 0.0, "wz": 0.0}
        self.chassis = {"mode": "-", "battery": 0.0}
        self.pose_history: list[dict] = []
        self.camera_inbox: dict[int, dict] = {i: {"objects": [], "meta": {}} for i in range(1, 5)}
        self.camera_display: dict[int, dict] = {i: {"objects": [], "meta": {}} for i in range(1, 5)}
        self.last_scan = {
            "front": {"raw_points": 0, "keyframe": False, "stamp": 0},
            "rear": {"raw_points": 0, "keyframe": False, "stamp": 0},
        }
        self.pending_poi: dict | None = None
        self.pending_poi_queue: list[dict] = []
        self.poi_nodes: list[Poi] = []
        self.poi_seed = 1
        self.selected_poi_ids: set[str] = set()

        self.path_segments: list[dict] = []
        self.path_nodes: list[dict] = []
        self.segment_seed = 1
        self.selected_segment_id: str | None = None
        self.pending_free_point: dict | None = None
        self.path_validation = {"checked": False, "ok": None, "invalid_ids": set(), "message": ""}

        self.scan = {
            "active": False,
            "mode": "2d",
            "phase": "idle",
            "error": "",
            "error_reason": "",
            "pending_start": False,
            "pending_mode": "",
            "started_ms": 0,
            "voxel": 0.08,
            "front_frames": 0,
            "rear_frames": 0,
            "raw_points": 0,
            "occupied": {},
            "free": {},
            "last_accum_pose": None,
            "last_saved_file": "",
            "saved_point_count": 0,
            "pcd_name": "",
            "pcd_bytes": b"",
            "pcd_received_at": 0,
        }
        self.server_grid = {"active": False, "resolution": 0.0, "occupied_cells": [], "free_cells": [], "stamp": 0.0}
        self.scan_lock = threading.Lock()
        self.scan_frame_buffer = LatestScanFrameBuffer()
        self.scan_worker_event = threading.Event()
        self.scan_worker_stop = threading.Event()
        self.canvas_dirty = True
        self.canvas_revision = 0
        self.last_render_revision = -1
        self.scan_badges_dirty = False
        self.scan_fusion = resolve_scan_fusion_config()
        self.edit = {
            "tool": "view",
            "pending_obstacle_start": None,
            "erasing": False,
            "loaded_from_stcm": False,
            "loaded_map_name": "",
        }
        self.view = {"scale": 80.0, "pan_x": 0.0, "pan_y": 0.0, "dragging": False, "moved": False, "last_xy": (0, 0)}
        self.keys_down: set[str] = set()
        self.control_lock = threading.Lock()
        self.control_target: tuple[str, dict, str] | None = None
        self.control_sender_event = threading.Event()
        self.control_sender_stop = threading.Event()
        self.pending_keyup_stop_id = None
        self.last_message_at_ms = 0
        self.last_health_poll_at = 0.0
        self.health: dict = {}
        self.stcm_summary: dict = {}
        self.inspector = {"file": "", "manifest": None, "points": [], "pgm": "", "yaml": "", "json": "", "meta": {}, "pcd_file": None}
        self.text_cache: dict[int, str] = {}
        self.responsive_rows: list[dict] = []

        self.server_var = tk.StringVar(value="")
        self.login_ip_var = tk.StringVar(value=str(self.client_config.get("gateway_ip", "192.168.3.56")))
        self.login_port_var = tk.StringVar(value=str(self.client_config.get("gateway_port", 28080)))
        self.direct_server_ip_var = tk.StringVar(value=str(self.client_config.get("server_ip", "127.0.0.1")))
        self.direct_server_port_var = tk.StringVar(value=str(self.client_config.get("server_port", 8080)))
        self.login_user_var = tk.StringVar(value=str(self.client_config.get("username", "admin")))
        self.login_password_var = tk.StringVar(value="")
        self.auth_status_var = tk.StringVar(value=self.tr("auth_missing"))
        self.conn_var = tk.StringVar(value="Disconnected")
        self.status_var = tk.StringVar(value=self.tr("ws_offline"))
        self.status_detail_var = tk.StringVar(value=self.tr("no_health"))
        self.scan_state_var = tk.StringVar(value=self.tr("idle"))
        self.scan_mode_var = tk.StringVar(value=str(self.scan["mode"]))
        self.keyboard_var = tk.StringVar(value=self.tr("keyboard_inactive"))
        self.camera_refresh_var = tk.StringVar(value="No buffered frame")
        self.map_name_var = tk.StringVar(value="desktop_map")
        self.map_notes_var = tk.StringVar(value="Desktop scan session")
        self.voxel_var = tk.StringVar(value=f"{float(self.scan_fusion['voxel_size']):.2f}")
        self.scan_fusion_preset_var = tk.StringVar(value="")
        self.occupied_min_hits_var = tk.StringVar(value=str(int(self.scan_fusion["occupied_min_hits"])))
        self.occupied_over_free_ratio_var = tk.StringVar(value=f"{float(self.scan_fusion['occupied_over_free_ratio']):.2f}")
        self.turn_skip_wz_var = tk.StringVar(value=f"{float(self.scan_fusion['turn_skip_wz']):.2f}")
        self.skip_turn_frames_var = tk.BooleanVar(value=bool(self.scan_fusion["skip_turn_frames"]))
        self.poi_name_var = tk.StringVar()
        self.poi_geo_var = tk.StringVar()
        self.poi_mode_var = tk.StringVar(value="batch")
        self.poi_mode_display_var = tk.StringVar()
        self.single_poi_name_var = tk.StringVar()
        self.single_poi_x_var = tk.StringVar(value="0.0")
        self.single_poi_y_var = tk.StringVar(value="0.0")
        self.single_poi_yaw_var = tk.StringVar(value="0.0")
        self.single_poi_geo_var = tk.StringVar()
        self.edit_poi_name_var = tk.StringVar()
        self.edit_poi_x_var = tk.StringVar()
        self.edit_poi_y_var = tk.StringVar()
        self.edit_poi_yaw_var = tk.StringVar()
        self.edit_poi_geo_var = tk.StringVar()
        self.path_mode_var = tk.StringVar(value="idle")
        self.path_mode_display_var = tk.StringVar()
        self.path_start_var = tk.StringVar()
        self.path_end_var = tk.StringVar()
        self.path_clearance_var = tk.StringVar(value="0.30")
        self.edit_tool_var = tk.StringVar(value="view")
        self.edit_tool_display_var = tk.StringVar()
        self.brush_var = tk.StringVar(value="0.25")
        self.path_status_var = tk.StringVar(value="No path segments yet")
        self.poi_status_var = tk.StringVar(value=self.tr("poi_idle"))
        self.map_badge_var = tk.StringVar(value=self.tr("scan_session"))
        self.tool_badge_var = tk.StringVar(value=self.tr("tool_view"))
        self.stats_badge_var = tk.StringVar(value="0 obstacle cells")
        self.map_edit_status_var = tk.StringVar(value=self.tr("loaded_map_edit"))
        self.left_layout_notice_var = tk.StringVar(value="")
        self.view_metrics_var = tk.StringVar(value="Pan 0.00, 0.00 | Zoom 80.0 px/m")
        self.show_path_var = tk.BooleanVar(value=True)
        self.show_poi_var = tk.BooleanVar(value=True)
        self.show_robot_var = tk.BooleanVar(value=True)
        self.show_scan_card_var = tk.BooleanVar(value=True)
        self.show_move_card_var = tk.BooleanVar(value=True)
        self.show_poi_card_var = tk.BooleanVar(value=True)
        self.show_path_card_var = tk.BooleanVar(value=True)
        self.show_map_card_var = tk.BooleanVar(value=True)
        self.forward_var = tk.StringVar(value="0.35")
        self.reverse_var = tk.StringVar(value="0.35")
        self.turn_var = tk.StringVar(value="0.2")
        self.duration_var = tk.StringVar(value="0.15")
        self.repeat_ms_var = tk.StringVar(value="120")
        self.stop_on_keyup_var = tk.BooleanVar(value=True)

        self.stream_health = {
            "msg_total": 0,
            "checksum_err": 0,
            "checksum_skipped": 0,
            "stale_ts_err": 0,
            "gap_err": 0,
            "retries_http": 0,
            "last_lag_ms": 0,
            "last_api_error": "",
            "control_failures_consecutive": 0,
            "network_quality_gap_seen": 0,
            "network_quality_checksum_seen": 0,
            "network_quality": "offline",
            "last_seq": {},
        }
        self.auth_context = {"base_url": "", "user_name": "", "token": "", "http_url": "", "ws_url": ""}

        self._style()
        self._ui()
        self.refresh_language_state()
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)
        self.root.bind("<MouseWheel>", self.on_mousewheel)
        self.root.bind("<Button-4>", self.on_mousewheel)
        self.root.bind("<Button-5>", self.on_mousewheel)
        self.root.bind("<Button-1>", self.on_root_click, add="+")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.scan_worker = threading.Thread(target=self._scan_worker_loop, daemon=True, name="scan-worker")
        self.scan_worker.start()
        self.control_sender_thread = threading.Thread(target=self._control_sender_loop, daemon=True, name="control-sender")
        self.control_sender_thread.start()
        self.tick()

    def setup_logging(self) -> str:
        log_candidates = compute_log_candidates(
            platform_name=sys.platform,
            env=dict(os.environ),
            home_dir=Path.home(),
            runtime_dir=runtime_base_dir(),
            temp_dir=Path(tempfile.gettempdir()),
            cwd=Path.cwd(),
        )
        log_path = resolve_log_file_path(log_candidates)
        bootstrap_log_write(log_path, "desktop client startup")
        logger = logging.getLogger("autodrive.client_desktop")
        logger.setLevel(logging.INFO)
        logger.propagate = False
        existing_paths = {
            getattr(handler, "baseFilename", "")
            for handler in logger.handlers
            if isinstance(handler, RotatingFileHandler)
        }
        if str(log_path) not in existing_paths:
            handler = RotatingFileHandler(log_path, maxBytes=1_000_000, backupCount=3, encoding="utf-8")
            handler.setFormatter(logging.Formatter("%(asctime)s | %(levelname)s | %(name)s | %(message)s"))
            logger.addHandler(handler)
        logger.info("desktop client startup")
        return str(log_path)

    def report_callback_exception(self, exc_type, exc_value, exc_traceback) -> None:
        details = "".join(traceback.format_exception(exc_type, exc_value, exc_traceback))
        bootstrap_log_write(Path(self.log_path), f"tk callback exception\n{details}")
        self.logger.error("tk callback exception\n%s", details)
        try:
            messagebox.showerror("Runtime Error", f"{exc_value}\n\nSee log:\n{self.log_path}")
        except Exception:
            pass

    def tr(self, key: str, **kwargs) -> str:
        text = self.i18n[self.current_lang].get(key, key)
        return text.format(**kwargs) if kwargs else text

    def on_language_change(self, _event=None) -> None:
        selection = self.lang_choice_var.get()
        self.current_lang = "zh" if selection == self.i18n["en"]["chinese"] else "en"
        self.logger.info("language changed lang=%s", self.current_lang)
        self.rebuild_ui()

    def rebuild_ui(self) -> None:
        for child in self.root.winfo_children():
            child.destroy()
        self.responsive_rows = []
        self.root.title(self.tr("title"))
        self._ui()
        self.refresh_language_state()

    def refresh_language_state(self) -> None:
        self.poi_mode_display_var.set(self.tr(self.poi_mode_var.get()))
        self.path_mode_display_var.set(self.tr(self.path_mode_var.get()))
        self.edit_tool_display_var.set(
            self.tr(
                safe_mode_translation_key(
                    self.edit_tool_var.get(),
                    {"view": "tool_view_select", "erase": "tool_erase_noise", "obstacle": "tool_draw_obstacle"},
                    "tool_view_select",
                )
            )
        )
        self.status_var.set(self.tr("ws_offline") if not (self.bridge and self.bridge.connected) else self.status_var.get())
        if not bool(self.client_config.get("login_required", True)):
            self.auth_status_var.set(self.tr("auth_bypass"))
        else:
            self.auth_status_var.set(self.tr("auth_ready", user=self.auth_context["user_name"]) if self.auth_context.get("user_name") else self.tr("auth_missing"))
        if not self.poi_status_var.get():
            self.poi_status_var.set(self.tr("poi_idle"))
        self.sync_scan_badges()
        self.sync_path_panel()
        self.update_left_panel_notice()

    def sync_mode_from_display(self, kind: str) -> None:
        if kind == "poi":
            reverse = {self.tr("batch"): "batch", self.tr("single"): "single", self.tr("edit"): "edit"}
            self.poi_mode_var.set(reverse.get(self.poi_mode_display_var.get(), "batch"))
            self.sync_poi_mode_ui()
        elif kind == "path":
            reverse = {self.tr("path_browse_only"): "idle", self.tr("poi"): "poi", self.tr("free_points"): "free"}
            self.path_mode_var.set(reverse.get(self.path_mode_display_var.get(), "idle"))
            self.sync_path_panel()
        elif kind == "edit":
            reverse = {self.tr("tool_view_select"): "view", self.tr("tool_erase_noise"): "erase", self.tr("tool_draw_obstacle"): "obstacle"}
            self.edit_tool_var.set(reverse.get(self.edit_tool_display_var.get(), "view"))
            self.edit_tool_changed()

    def _style(self) -> None:
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TLabel", font=("Segoe UI", 11))
        style.configure("TButton", font=("Segoe UI", 10))
        style.configure("Header.TLabel", font=("Segoe UI", 24, "bold"))
        style.configure("Muted.TLabel", foreground="#5c6f7a", font=("Segoe UI", 10))

    def _ui(self) -> None:
        shell = ttk.Frame(self.root, padding=12)
        shell.pack(fill=tk.BOTH, expand=True)

        header = ttk.Frame(shell)
        header.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(header, text=self.tr("title"), style="Header.TLabel").pack(side=tk.LEFT)

        top = ttk.Frame(shell, padding=8)
        top.pack(fill=tk.X, pady=(0, 10))
        ttk.Button(top, text=self.tr("login"), command=self.open_login_dialog).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text=self.tr("connect"), command=self.connect).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text=self.tr("disconnect"), command=self.disconnect).pack(side=tk.LEFT, padx=4)
        ttk.Label(top, text=self.tr("language")).pack(side=tk.LEFT, padx=(12, 4))
        lang_box = ttk.Combobox(top, textvariable=self.lang_choice_var, state="readonly", width=10, values=[self.tr("english"), self.tr("chinese")])
        lang_box.pack(side=tk.LEFT, padx=(0, 8))
        lang_box.bind("<<ComboboxSelected>>", self.on_language_change)
        ttk.Label(top, textvariable=self.auth_status_var, style="Muted.TLabel").pack(side=tk.LEFT, padx=(14, 8))
        ttk.Label(top, textvariable=self.conn_var).pack(side=tk.LEFT, padx=(14, 4))
        ttk.Label(top, textvariable=self.status_var, style="Muted.TLabel").pack(side=tk.LEFT, padx=(4, 8))
        ttk.Label(top, textvariable=self.status_detail_var, style="Muted.TLabel").pack(side=tk.LEFT)

        paned = ttk.Panedwindow(shell, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.BOTH, expand=True)
        left = ttk.Frame(paned)
        center = ttk.Frame(paned)
        right = ttk.Frame(paned)
        paned.add(left, weight=1)
        paned.add(center, weight=9)
        paned.add(right, weight=2)

        self._left(left)
        self._center(center)
        self._right(right)
        self.sync_visibility_cards()
        self.root.after_idle(self.reset_view)

    def _left(self, parent: ttk.Frame) -> None:
        shell = ttk.Frame(parent)
        shell.pack(fill=tk.BOTH, expand=True)
        self.left_canvas = tk.Canvas(shell, highlightthickness=0, borderwidth=0)
        self.left_scrollbar = ttk.Scrollbar(shell, orient=tk.VERTICAL, command=self.left_canvas.yview)
        self.left_canvas.configure(yscrollcommand=self.left_scrollbar.set)
        self.left_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.left_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.left_panel = ttk.Frame(self.left_canvas)
        self.left_window_id = self.left_canvas.create_window((0, 0), window=self.left_panel, anchor="nw")
        self.left_panel.bind("<Configure>", self.on_left_panel_configure)
        self.left_canvas.bind("<Configure>", self.on_left_canvas_configure)
        self.left_canvas.bind_all("<MouseWheel>", self.on_left_mousewheel)

        self._visibility_controls(self.left_panel)
        ttk.Label(self.left_panel, textvariable=self.left_layout_notice_var, style="Muted.TLabel", wraplength=320).pack(fill=tk.X, pady=(0, 8))
        self.scan_card = self._scan_controls(self.left_panel)
        self.move_card = self._move_controls(self.left_panel)
        self.poi_card = self._poi_controls(self.left_panel)
        self.path_card = self._path_controls(self.left_panel)
        self.map_card = self._map_controls(self.left_panel)
        parent.bind("<Configure>", lambda _e: self.root.after_idle(self.update_left_panel_notice))

    def _visibility_controls(self, parent: ttk.Frame) -> None:
        card = ttk.LabelFrame(parent, text=self.tr("panel_visibility"), padding=8)
        card.pack(fill=tk.X, pady=(0, 10))
        for index, (text, var) in enumerate([
            (self.tr("scan"), self.show_scan_card_var),
            (self.tr("move"), self.show_move_card_var),
            (self.tr("poi"), self.show_poi_card_var),
            (self.tr("path"), self.show_path_card_var),
            (self.tr("map"), self.show_map_card_var),
        ]):
            ttk.Checkbutton(card, text=text, variable=var, command=self.sync_visibility_cards).grid(row=index // 2, column=index % 2, sticky="w", padx=6, pady=2)

    def _scan_controls(self, parent: ttk.Frame) -> ttk.LabelFrame:
        card = ttk.LabelFrame(parent, text=self.tr("scan"), padding=8)
        card.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(card, textvariable=self.scan_state_var).pack(anchor=tk.W, pady=(0, 6))
        self.scan_progress = ttk.Progressbar(card, mode="indeterminate", length=160)
        self.scan_progress.pack(fill=tk.X, pady=(0, 6))
        mode_row = ttk.Frame(card)
        mode_row.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(mode_row, text=self.tr("scan_mode_label"), width=18).pack(side=tk.LEFT)
        mode_box = ttk.Combobox(mode_row, textvariable=self.scan_mode_var, state="readonly", values=["2d", "3d"], width=12)
        mode_box.pack(side=tk.LEFT, padx=6)
        scan_row, _ = self._responsive_button_row(
            card,
            [
                (self.tr("start_scan"), self.start_scan),
                (self.tr("stop_scan"), self.stop_scan),
                (self.tr("clear"), self.clear_scan),
            ],
        )
        scan_row.pack(fill=tk.X, pady=(0, 6))
        self._entry(card, self.tr("map_name"), self.map_name_var)
        self._entry(card, self.tr("notes"), self.map_notes_var)
        return card

    def _move_controls(self, parent: ttk.Frame) -> ttk.LabelFrame:
        card = ttk.LabelFrame(parent, text=self.tr("move"), padding=8)
        card.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(card, textvariable=self.keyboard_var).pack(anchor=tk.W, pady=(0, 6))
        self._entry(card, self.tr("forward_speed"), self.forward_var)
        self._entry(card, self.tr("reverse_speed"), self.reverse_var)
        self._entry(card, self.tr("turn_rate"), self.turn_var)
        self._entry(card, self.tr("repeat_ms"), self.repeat_ms_var)
        ttk.Checkbutton(card, text=self.tr("stop_on_keyup"), variable=self.stop_on_keyup_var).pack(anchor=tk.W, pady=(4, 6))
        move_row, _ = self._responsive_button_row(
            card,
            [(self.tr(name.lower()), lambda n=name: self.move_click(n.lower())) for name in ("Forward", "Left", "Stop", "Right", "Reverse")],
        )
        move_row.pack(fill=tk.X, pady=(6, 0))
        return card

    def _poi_controls(self, parent: ttk.Frame) -> ttk.LabelFrame:
        card = ttk.LabelFrame(parent, text=self.tr("poi"), padding=8)
        card.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        mode_row = ttk.Frame(card)
        mode_row.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(mode_row, text=self.tr("mode")).pack(side=tk.LEFT)
        mode_box = ttk.Combobox(mode_row, textvariable=self.poi_mode_display_var, state="readonly", width=18, values=[self.tr("batch"), self.tr("single"), self.tr("edit")])
        mode_box.pack(side=tk.LEFT, padx=6)
        mode_box.bind("<<ComboboxSelected>>", lambda _e: self.sync_mode_from_display("poi"))
        ttk.Label(card, textvariable=self.poi_status_var, style="Muted.TLabel", wraplength=340).pack(anchor=tk.W, pady=(0, 6))
        self.poi_inputs_frame = ttk.Frame(card)
        self.poi_inputs_frame.pack(fill=tk.X, pady=(0, 8))
        self.batch_poi_frame = ttk.Frame(self.poi_inputs_frame)
        self.batch_poi_frame.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(self.batch_poi_frame, text=self.tr("batch_input_hint"), style="Muted.TLabel", wraplength=340, justify=tk.LEFT).pack(anchor=tk.W)
        ttk.Label(self.batch_poi_frame, text=self.tr("batch_examples"), style="Muted.TLabel", wraplength=340, justify=tk.LEFT).pack(anchor=tk.W, pady=(2, 4))
        self.poi_batch_text = tk.Text(self.batch_poi_frame, height=4, bg="#ffffff", fg="#14232d", relief=tk.FLAT, font=("Consolas", 10))
        self.poi_batch_text.pack(fill=tk.X, pady=(0, 6))
        self.batch_action_btn = ttk.Button(self.batch_poi_frame, text=self.tr("start_batch_add"), command=self.toggle_add_poi)
        self.batch_action_btn.pack(anchor=tk.W)

        self.single_poi_frame = ttk.Frame(self.poi_inputs_frame)
        self._entry(self.single_poi_frame, self.tr("name"), self.single_poi_name_var)
        self._entry(self.single_poi_frame, self.tr("x"), self.single_poi_x_var)
        self._entry(self.single_poi_frame, self.tr("y"), self.single_poi_y_var)
        self._entry(self.single_poi_frame, self.tr("yaw"), self.single_poi_yaw_var)
        self._entry(self.single_poi_frame, self.tr("geo"), self.single_poi_geo_var)
        ttk.Button(self.single_poi_frame, text=self.tr("add_single_poi"), command=self.add_single_poi).pack(anchor=tk.W)

        self.edit_poi_frame = ttk.Frame(self.poi_inputs_frame)
        ttk.Label(self.edit_poi_frame, text=self.tr("edit_hint"), style="Muted.TLabel", wraplength=320).pack(anchor=tk.W, pady=(0, 4))
        self._entry(self.edit_poi_frame, self.tr("name"), self.edit_poi_name_var)
        self._entry(self.edit_poi_frame, self.tr("x"), self.edit_poi_x_var)
        self._entry(self.edit_poi_frame, self.tr("y"), self.edit_poi_y_var)
        self._entry(self.edit_poi_frame, self.tr("yaw"), self.edit_poi_yaw_var)
        self._entry(self.edit_poi_frame, self.tr("geo"), self.edit_poi_geo_var)
        ttk.Button(self.edit_poi_frame, text=self.tr("apply_edit"), command=self.apply_poi_edit).pack(anchor=tk.W)

        row = ttk.Frame(card)
        row.pack(fill=tk.X, pady=(0, 6))
        _, poi_buttons = self._responsive_button_row(
            row,
            [
                (self.tr("delete_selected"), self.delete_selected_poi),
                (self.tr("copy_poi"), self.copy_poi_text),
            ],
            min_button_width=140,
        )
        _.pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Checkbutton(row, text=self.tr("show_poi"), variable=self.show_poi_var).pack(side=tk.RIGHT, padx=4)
        ttk.Label(card, text=self.tr("poi_list"), style="Muted.TLabel").pack(anchor=tk.W, pady=(2, 4))
        self.poi_box = tk.Listbox(card, height=10, bg="#ffffff", fg="#14232d", selectbackground="#0c7c78", font=("Segoe UI", 11), selectmode=tk.EXTENDED, exportselection=False)
        self.poi_box.pack(fill=tk.BOTH, expand=True)
        self.poi_box.bind("<<ListboxSelect>>", lambda _e: self.sync_selected_poi())
        self.sync_poi_mode_ui()
        return card

    def _path_controls(self, parent: ttk.Frame) -> ttk.LabelFrame:
        card = ttk.LabelFrame(parent, text=self.tr("path"), padding=8)
        card.pack(fill=tk.BOTH, expand=True)
        ttk.Label(card, text=self.tr("path_hint"), style="Muted.TLabel", wraplength=340).pack(anchor=tk.W, pady=(0, 6))
        row = ttk.Frame(card)
        row.pack(fill=tk.X)
        ttk.Label(row, text=self.tr("path_tool")).pack(side=tk.LEFT)
        mode_box = ttk.Combobox(row, textvariable=self.path_mode_display_var, state="readonly", width=18, values=[self.tr("path_browse_only"), self.tr("poi"), self.tr("free_points")])
        mode_box.pack(side=tk.LEFT, padx=6)
        mode_box.bind("<<ComboboxSelected>>", lambda _e: self.sync_mode_from_display("path"))
        ttk.Label(row, text=self.tr("safe_clearance")).pack(side=tk.LEFT, padx=(12, 0))
        ttk.Entry(row, textvariable=self.path_clearance_var, width=8).pack(side=tk.LEFT, padx=6)
        self._entry(card, self.tr("start_poi_name"), self.path_start_var)
        self._entry(card, self.tr("end_poi_name"), self.path_end_var)
        ttk.Label(card, textvariable=self.path_status_var, style="Muted.TLabel", wraplength=340).pack(anchor=tk.W, pady=(0, 6))
        row2 = ttk.Frame(card)
        row2.pack(fill=tk.X, pady=(0, 6))
        path_action_row, path_buttons = self._responsive_button_row(
            row2,
            [
                (self.tr("auto_loop"), self.auto_loop),
                (self.tr("connect_named_poi"), self.connect_named_poi),
                (self.tr("clear_selection"), self.clear_selection),
                (self.tr("closed_loop_check"), lambda: self.validate_path(True)),
                (self.tr("delete_segment"), self.delete_selected_segment),
            ],
            min_button_width=150,
        )
        path_action_row.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.connect_named_btn = path_buttons[1]
        ttk.Checkbutton(row2, text=self.tr("show_path"), variable=self.show_path_var).pack(side=tk.RIGHT, padx=4)
        self.path_box = tk.Listbox(card, height=10, bg="#ffffff", fg="#14232d", selectbackground="#0c7c78", font=("Segoe UI", 11))
        self.path_box.pack(fill=tk.BOTH, expand=True)
        self.path_box.bind("<<ListboxSelect>>", lambda _e: self.sync_selected_segment())
        self.path_start_var.trace_add("write", lambda *_: self.sync_path_panel())
        self.path_end_var.trace_add("write", lambda *_: self.sync_path_panel())
        return card

    def _map_controls(self, parent: ttk.Frame) -> ttk.LabelFrame:
        card = ttk.LabelFrame(parent, text=self.tr("map"), padding=8)
        card.pack(fill=tk.X, pady=(10, 0))
        row = ttk.Frame(card)
        row.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(row, text=self.tr("map_tool"), width=18).pack(side=tk.LEFT)
        edit_box = ttk.Combobox(row, textvariable=self.edit_tool_display_var, state="readonly", values=[self.tr("tool_view_select"), self.tr("tool_erase_noise"), self.tr("tool_draw_obstacle")])
        edit_box.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)
        edit_box.bind("<<ComboboxSelected>>", lambda _e: self.sync_mode_from_display("edit"))
        self._entry(card, self.tr("brush_radius"), self.brush_var)
        map_action_row, _ = self._responsive_button_row(
            card,
            [
                (self.tr("auto_clear_noise"), self.auto_clear_noise),
                (self.tr("clear_loaded_map"), self.clear_loaded_map),
                (self.tr("reset_map"), self.reset_server_map),
                (self.tr("save_map"), self.save_stcm),
                (self.tr("load_map"), self.load_stcm),
                (self.tr("export_pgm"), lambda: self.export_inspector_file("pgm")),
                (self.tr("export_yaml"), lambda: self.export_inspector_file("yaml")),
                (self.tr("export_json"), lambda: self.export_inspector_file("json")),
                (self.tr("export_pcd"), lambda: self.export_inspector_file("pcd")),
            ],
            min_button_width=145,
        )
        map_action_row.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(card, textvariable=self.map_edit_status_var, style="Muted.TLabel", wraplength=340).pack(anchor=tk.W, pady=(6, 0))
        return card

    def _center(self, parent: ttk.Frame) -> None:
        card = ttk.LabelFrame(parent, text=self.tr("map_view"), padding=8)
        card.pack(fill=tk.BOTH, expand=True)
        row = ttk.Frame(card)
        row.pack(fill=tk.X, pady=(0, 6))
        ttk.Button(row, text=self.tr("center_robot"), command=self.center_robot).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(row, text=self.tr("center_loaded_map"), command=self.center_loaded_map).pack(side=tk.LEFT, padx=4)
        ttk.Button(row, text=self.tr("reset_view"), command=self.reset_view).pack(side=tk.LEFT, padx=4)
        ttk.Button(row, text="-", width=3, command=lambda: self.zoom_view(1 / 1.2), takefocus=False).pack(side=tk.LEFT, padx=(12, 4))
        ttk.Button(row, text="+", width=3, command=lambda: self.zoom_view(1.2), takefocus=False).pack(side=tk.LEFT, padx=4)
        ttk.Checkbutton(row, text=self.tr("show_robot"), variable=self.show_robot_var).pack(side=tk.LEFT, padx=8)
        ttk.Label(row, textvariable=self.view_metrics_var, style="Muted.TLabel").pack(side=tk.RIGHT)
        badges = ttk.Frame(card)
        badges.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(badges, textvariable=self.map_badge_var, style="Muted.TLabel").pack(side=tk.LEFT, padx=(0, 12))
        ttk.Label(badges, textvariable=self.tool_badge_var, style="Muted.TLabel").pack(side=tk.LEFT, padx=12)
        ttk.Label(badges, textvariable=self.stats_badge_var, style="Muted.TLabel").pack(side=tk.LEFT, padx=12)
        self.canvas = tk.Canvas(card, bg="#8f969c", highlightbackground="#c6d2d9", highlightthickness=1)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<ButtonPress-1>", self.canvas_press)
        self.canvas.bind("<B1-Motion>", self.canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self.canvas_release)

    def _right(self, parent: ttk.Frame) -> None:
        self.scan_text = self._card_text(parent, self.tr("odom_scan"), 8)
        self.comm_text = self._card_text(parent, self.tr("comm_map"), 12)

    def _card_text(self, parent: ttk.Frame, title: str, height: int) -> tk.Text:
        card = ttk.LabelFrame(parent, text=title, padding=8)
        card.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        text = tk.Text(card, height=height, bg="#f3f7f9", fg="#14232d", relief=tk.FLAT, font=("Consolas", 10))
        text.pack(fill=tk.BOTH, expand=True)
        return text

    def _entry(self, parent: ttk.Frame, label: str, var: tk.StringVar) -> None:
        row = ttk.Frame(parent)
        row.pack(fill=tk.X, pady=2)
        ttk.Label(row, text=label, width=18).pack(side=tk.LEFT)
        ttk.Entry(row, textvariable=var).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)

    def _responsive_button_row(self, parent: ttk.Frame, buttons: list[tuple[str, object]], min_button_width: int = 120) -> tuple[ttk.Frame, list[ttk.Button]]:
        frame = ttk.Frame(parent)
        created: list[ttk.Button] = []
        for text, command in buttons:
            btn = ttk.Button(frame, text=text, command=command)
            created.append(btn)
        row_spec = {"frame": frame, "buttons": created, "min_width": min_button_width}
        self.responsive_rows.append(row_spec)
        frame.bind("<Configure>", lambda _e, spec=row_spec: self.layout_responsive_row(spec))
        self.root.after_idle(lambda spec=row_spec: self.layout_responsive_row(spec))
        return frame, created

    def layout_responsive_row(self, spec: dict) -> None:
        frame = spec["frame"]
        buttons = spec["buttons"]
        min_width = spec["min_width"]
        width = max(frame.winfo_width(), 1)
        columns = max(1, min(len(buttons), width // min_width))
        for index, button in enumerate(buttons):
            button.grid_forget()
            row = index // columns
            col = index % columns
            button.grid(row=row, column=col, sticky="ew", padx=3, pady=3)
        for col in range(columns):
            frame.grid_columnconfigure(col, weight=1)
        for col in range(columns, len(buttons)):
            frame.grid_columnconfigure(col, weight=0)

    def update_left_panel_notice(self) -> None:
        if not hasattr(self, "left_panel"):
            return
        available = self.left_canvas.winfo_height() if hasattr(self, "left_canvas") else self.left_panel.winfo_height()
        required = self.left_panel.winfo_reqheight()
        needs_scroll = available > 0 and required > available + 4
        if needs_scroll:
            self.left_layout_notice_var.set(self.tr("left_notice"))
        else:
            self.left_layout_notice_var.set("")

    def on_left_panel_configure(self, _event=None) -> None:
        if not hasattr(self, "left_canvas"):
            return
        self.left_canvas.configure(scrollregion=self.left_canvas.bbox("all"))
        self.root.after_idle(self.update_left_panel_notice)

    def on_left_canvas_configure(self, event) -> None:
        if not hasattr(self, "left_canvas"):
            return
        self.left_canvas.itemconfigure(self.left_window_id, width=event.width)
        self.root.after_idle(self.update_left_panel_notice)

    def on_left_mousewheel(self, event) -> None:
        if not hasattr(self, "left_canvas"):
            return
        widget = self.root.winfo_containing(event.x_root, event.y_root)
        if widget is None:
            return
        current = widget
        while current is not None:
            if current == self.left_canvas or current == self.left_panel:
                self.left_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
                return
            current = current.master

    def number(self, var: tk.StringVar, fallback: float) -> float:
        try:
            return float(var.get().strip())
        except Exception:
            return fallback

    def sync_visibility_cards(self) -> None:
        order = [
            (self.scan_card, self.show_scan_card_var.get(), tk.X, False),
            (self.move_card, self.show_move_card_var.get(), tk.X, False),
            (self.poi_card, self.show_poi_card_var.get(), tk.BOTH, True),
            (self.path_card, self.show_path_card_var.get(), tk.BOTH, True),
            (self.map_card, self.show_map_card_var.get(), tk.X, False),
        ]
        for frame, visible, fill_mode, expand in order:
            if frame.winfo_manager():
                frame.pack_forget()
            if visible:
                pady = (10, 0) if frame is self.map_card else (0, 10)
                frame.pack(fill=fill_mode, expand=expand, pady=pady)
        self.root.after_idle(self.update_left_panel_notice)

    def sync_poi_mode_ui(self) -> None:
        mode = self.poi_mode_var.get()
        for frame in (self.batch_poi_frame, self.single_poi_frame, self.edit_poi_frame):
            if frame.winfo_manager():
                frame.pack_forget()
        target = {
            "batch": self.batch_poi_frame,
            "single": self.single_poi_frame,
            "edit": self.edit_poi_frame,
        }[mode]
        target.pack(fill=tk.X, pady=(0, 6))
        self.batch_action_btn.configure(text=self.tr("start_batch_add") if self.pending_poi is None and not self.pending_poi_queue else self.tr("cancel_batch", count=len(self.pending_poi_queue) + (1 if self.pending_poi else 0)))

    def update_health_status_detail(self, health: dict) -> None:
        if (
            hasattr(self, "scan")
            and self.scan.get("phase") == "waiting_mapping"
            and bool(self.scan.get("pending_start"))
            and (self.scan.get("error_reason") or self.scan.get("error")) == "mapping_prereq_failed"
            and bool(health.get("mapping_ready"))
        ):
            mode = str(self.scan.get("pending_mode") or self.scan.get("mode") or "2d")
            response = self.call_api("/scan/start", {"mode": mode})
            self._finish_start_scan(mode, response, show_mapping_warning=False)
        if (
            hasattr(self, "scan")
            and self.scan.get("phase") == "error"
            and (self.scan.get("error_reason") or self.scan.get("error")) == "mapping_prereq_failed"
            and bool(health.get("mapping_ready"))
        ):
            self.scan["phase"] = "idle"
            self.scan["error"] = ""
            self.scan["error_reason"] = ""
            self.sync_scan_badges()
        network_quality = self.update_network_quality()
        self.status_detail_var.set(
            self.tr(
                "status_detail_mapping",
                clients=health.get("ws_clients", "n/a"),
                scan="on" if health.get("scan_active") else "off",
                ros="enabled" if health.get("ros_enabled") else "detected only",
                mapping=f"{summarize_mapping_status(health)} | {self.tr(f'network_{network_quality}')}",
            )
        )

    def update_network_quality(self) -> str:
        if not hasattr(self, "stream_health"):
            self.stream_health = {}
        bridge = getattr(self, "bridge", None)
        recent_ws_issue_ms = int(getattr(bridge, "last_ws_issue_at_ms", 0) or 0) if bridge is not None else 0
        quality = classify_network_quality(
            bool(bridge and bridge.connected),
            self.stream_health,
            int(getattr(self, "last_message_at_ms", 0) or 0),
            recent_ws_issue_ms=recent_ws_issue_ms,
        )
        self.stream_health["network_quality"] = quality
        if quality == "ok":
            self.stream_health["network_quality_gap_seen"] = int(self.stream_health.get("gap_err", 0) or 0)
            self.stream_health["network_quality_checksum_seen"] = int(self.stream_health.get("checksum_err", 0) or 0)
        return quality

    def open_login_dialog(self) -> None:
        dialog = tk.Toplevel(self.root)
        dialog.title(self.tr("login_title"))
        dialog.transient(self.root)
        dialog.grab_set()
        dialog.resizable(False, False)
        body = ttk.Frame(dialog, padding=12)
        body.pack(fill=tk.BOTH, expand=True)
        if bool(self.client_config.get("login_required", True)):
            self._entry(body, self.tr("login_ip"), self.login_ip_var)
            self._entry(body, self.tr("login_port"), self.login_port_var)
            self._entry(body, self.tr("name"), self.login_user_var)
            row = ttk.Frame(body)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=self.tr("password"), width=18).pack(side=tk.LEFT)
            ttk.Entry(row, textvariable=self.login_password_var, show="*").pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)
        else:
            self._entry(body, self.tr("server_ip"), self.direct_server_ip_var)
            self._entry(body, self.tr("server_port"), self.direct_server_port_var)
        actions = ttk.Frame(body)
        actions.pack(fill=tk.X, pady=(10, 0))
        ttk.Button(actions, text=self.tr("connect"), command=lambda: self.login_and_connect(dialog)).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(actions, text=self.tr("disconnect"), command=dialog.destroy).pack(side=tk.LEFT)
        dialog.bind("<Return>", lambda _e: self.login_and_connect(dialog))
        dialog.wait_visibility()
        dialog.focus_set()

    def login_and_connect(self, dialog: tk.Toplevel | None = None) -> None:
        if not bool(self.client_config.get("login_required", True)):
            if dialog is not None:
                dialog.destroy()
            self.connect()
            return
        base_url = compose_http_base_url(self.login_ip_var.get().strip(), self.login_port_var.get().strip())
        username = self.login_user_var.get().strip()
        password = self.login_password_var.get().strip()
        if not base_url or not username or not password:
            messagebox.showwarning(self.tr("login_title"), self.tr("login_empty_fields"))
            return
        try:
            context = bootstrap_authenticated_bridge(base_url, username, password)
            self.auth_context = dict(context)
            self.auth_status_var.set(self.tr("auth_ready", user=context["user_name"]))
            self.server_var.set(context["ws_url"])
            if dialog is not None:
                dialog.destroy()
            self.connect()
        except Exception as exc:
            self.stream_health["last_api_error"] = str(exc)
            self.logger.exception("login bootstrap failed")
            user_message = exc.user_message if isinstance(exc, AuthFlowError) else self.tr("login_failed_safe")
            messagebox.showerror(self.tr("login_failed"), user_message)

    def connect(self) -> None:
        try:
            self.disconnect()
            if not bool(self.client_config.get("login_required", True)):
                raw_url = build_direct_ws_url(self.direct_server_ip_var.get().strip(), self.direct_server_port_var.get().strip())
                self.auth_status_var.set(self.tr("auth_bypass"))
            else:
                raw_url = self.auth_context.get("ws_url", "").strip()
                if not raw_url:
                    messagebox.showwarning(self.tr("connect"), self.tr("connect_need_login"))
                    self.open_login_dialog()
                    return
            normalized_url = normalize_server_ws_url(raw_url)
            self.server_var.set(normalized_url)
            self.logger.info("connect clicked ws_url=%s", normalized_url)
            if not normalized_url:
                raise ValueError("server address is empty")
            self.bridge = ServerBridge(normalized_url, logger=self.logger)
            health = self.bridge.get("/health")
            self.health = health
            self.update_health_status_detail(health)
            self.logger.info("connect preflight ok ws_clients=%s", health.get("ws_clients"))
            self.bridge.start()
            self.conn_var.set(self.tr("connecting"))
        except Exception as exc:
            self.stream_health["last_api_error"] = str(exc)
            self.logger.exception("connect preflight failed")
            messagebox.showerror(self.tr("api_error"), self.tr("connect_failed_safe"))
            self.bridge = None
            self.conn_var.set(self.tr("disconnected"))
            self.status_var.set(self.tr("ws_offline"))
            self.status_detail_var.set(redact_sensitive_text(str(exc)))

    def disconnect(self) -> None:
        self.logger.info("disconnect clicked")
        if self.bridge:
            self.bridge.stop()
        self.bridge = None
        self.conn_var.set(self.tr("disconnected"))
        self.status_var.set(self.tr("ws_offline"))
        self.status_detail_var.set(self.tr("disconnected"))

    def call_api(self, path: str, body: dict) -> dict | None:
        if not self.bridge:
            messagebox.showwarning(self.tr("disconnected"), self.tr("warning_disconnected"))
            return None
        try:
            self.stream_health["last_api_error"] = ""
            return self.bridge.post(path, body)
        except Exception as exc:
            self.stream_health["retries_http"] += 1
            self.stream_health["last_api_error"] = str(exc)
            self.logger.exception("call_api failed path=%s", path)
            messagebox.showerror(self.tr("api_error"), self.tr("connect_failed_safe"))
            return None

    def call_api_async(self, path: str, body: dict) -> None:
        if not self.bridge:
            messagebox.showwarning(self.tr("disconnected"), self.tr("warning_disconnected"))
            return
        self.stream_health["last_api_error"] = ""
        self.bridge.post_async(path, body)

    def poll_health(self) -> None:
        now = time.monotonic()
        if now - self.last_health_poll_at < HEALTH_POLL_INTERVAL_SEC or not self.bridge or not self.bridge.connected:
            return
        self.last_health_poll_at = now
        try:
            self.health = self.bridge.get("/health")
            self.update_health_status_detail(self.health)
        except Exception as exc:
            self.stream_health["last_api_error"] = str(exc)
            self.logger.warning("health poll failed err=%s", exc)

    def tick(self) -> None:
        self.consume_messages()
        if self.scan_badges_dirty:
            self.scan_badges_dirty = False
            self.sync_scan_badges()
        self.poll_health()
        network_quality = self.update_network_quality()
        self.render_canvas_if_needed()
        self.render_text_panels()
        self.conn_var.set(self.tr("connected") if self.bridge and self.bridge.connected else self.tr("disconnected"))
        self.status_var.set(
            self.tr(
                "status_line",
                ws=self.tr(f"network_{network_quality}"),
                scan="on" if self.scan["active"] else "off",
                poi=len(self.poi_nodes),
                path=len(self.path_segments),
            )
        )
        self.root.after(60, self.tick)

    def validate_message(self, msg: dict) -> bool:
        self.stream_health["msg_total"] += 1
        stamp = float(msg.get("stamp", 0.0))
        server_time_ms = int(msg.get("server_time_ms", 0))
        seq = int(msg.get("seq", 0))
        topic = str(msg.get("topic", ""))
        payload = msg.get("payload", {})
        checksum = msg.get("checksum")
        if checksum:
            raw_payload = json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
            raw = f"{topic}|{stamp:.6f}|{seq}|{raw_payload}".encode("utf-8")
            digest = hashlib.sha256(raw).hexdigest()
            if digest != checksum:
                self.stream_health["checksum_err"] += 1
                self.logger.warning("checksum mismatch topic=%s seq=%s", topic, seq)
                return False
        else:
            self.stream_health["checksum_skipped"] += 1
        lag_ms = max(0, int(time.time() * 1000) - server_time_ms) if server_time_ms else 0
        self.stream_health["last_lag_ms"] = lag_ms
        if server_time_ms and abs(int(time.time() * 1000) - server_time_ms) > 5000:
            self.stream_health["stale_ts_err"] += 1
        last = self.stream_health["last_seq"].get(topic)
        if last is not None and seq > last + 1:
            self.stream_health["gap_err"] += seq - last - 1
        self.stream_health["last_seq"][topic] = seq
        self.last_message_at_ms = int(time.time() * 1000)
        return True

    def consume_messages(self) -> None:
        if not self.bridge:
            return
        drained: list[dict[str, Any]] = []
        while True:
            try:
                msg = self.bridge.queue.get_nowait()
            except queue.Empty:
                break
            if not self.validate_message(msg):
                continue
            if len(drained) < MAX_MESSAGES_DRAIN_PER_TICK:
                drained.append(msg)
            else:
                drained.pop(0)
                drained.append(msg)
        processed = 0
        for msg in coalesce_stream_messages(drained):
            if processed >= MAX_MESSAGES_PER_TICK:
                break
            processed += 1
            topic = msg.get("topic")
            payload = msg.get("payload", {})
            if topic == "/robot/pose":
                self.pose = payload
                self.mark_canvas_dirty()
            elif topic == "/robot/gps":
                self.gps = payload
            elif topic == "/chassis/odom":
                self.odom = payload
                self.pose_history.append({"stamp": float(msg.get("stamp", time.time())), "pose": dict(payload)})
                self.pose_history = self.pose_history[-240:]
                self.mark_canvas_dirty()
            elif topic == "/chassis/status":
                self.chassis = payload
            elif topic == "/lidar/front":
                self.scan["front_frames"] += 1
                self.scan["raw_points"] = int(self.scan.get("raw_points", 0)) + int(payload.get("raw_points", len(payload.get("points", []))))
                self.last_scan["front"] = {"raw_points": int(payload.get("raw_points", len(payload.get("points", [])))), "keyframe": bool(payload.get("keyframe")), "stamp": float(msg.get("stamp", 0))}
            elif topic == "/lidar/rear":
                self.scan["rear_frames"] += 1
                self.scan["raw_points"] = int(self.scan.get("raw_points", 0)) + int(payload.get("raw_points", len(payload.get("points", []))))
                self.last_scan["rear"] = {"raw_points": int(payload.get("raw_points", len(payload.get("points", [])))), "keyframe": bool(payload.get("keyframe")), "stamp": float(msg.get("stamp", 0))}
            elif topic == "/map/grid":
                self.update_server_grid(payload, float(msg.get("stamp", 0)))
            elif topic and topic.startswith("/camera/"):
                cam_id = parse_camera_topic_id(topic)
                if cam_id is not None and cam_id in self.camera_inbox:
                    self.camera_inbox[cam_id] = {"objects": payload.get("objects", []), "meta": {"seq": msg.get("seq"), "stamp": msg.get("stamp"), "received_at_ms": int(time.time() * 1000)}}
        self.camera_refresh_var.set(build_camera_refresh_text(self.camera_inbox))

    def queue_scan_frame(self, points: list[Any], stamp: float, keyframe: bool) -> None:
        if not self.scan["active"] or not points:
            return
        frame = {
            "points": points,
            "stamp": float(stamp),
            "keyframe": bool(keyframe),
            "pose": dict(self.pose_for_stamp(stamp)),
            "config": self.effective_scan_fusion_config(),
        }
        self.scan_frame_buffer.submit(frame)
        self.scan_worker_event.set()

    def _scan_worker_loop(self) -> None:
        while not self.scan_worker_stop.is_set():
            self.scan_worker_event.wait(timeout=0.2)
            self.scan_worker_event.clear()
            while True:
                frame = self.scan_frame_buffer.pop_latest()
                if frame is None:
                    break
                with self.scan_lock:
                    changed = process_scan_frame(
                        self.scan,
                        points=frame["points"],
                        pose=frame["pose"],
                        keyframe=bool(frame["keyframe"]),
                        config=frame["config"],
                        logger=self.logger if hasattr(self, "logger") else None,
                    )
                if changed:
                    self.scan_badges_dirty = True
                    self.mark_canvas_dirty()

    def start_scan(self) -> None:
        mode = str(getattr(self, "scan_mode_var", None).get() if getattr(self, "scan_mode_var", None) is not None else self.scan.get("mode", "2d")).strip().lower()
        if self.scan.get("phase") == "starting":
            return
        self.scan["phase"] = "starting"
        self.scan["error"] = ""
        self.scan["error_reason"] = ""
        self.scan["pending_start"] = False
        self.scan["pending_mode"] = ""
        if hasattr(self, "scan_state_var"):
            self.scan_state_var.set(self.tr("scan_starting"))
        if hasattr(self, "scan_progress"):
            self.scan_progress.start(80)
        if getattr(self, "root", None) is not None:
            try:
                self.root.update_idletasks()
            except Exception:
                pass
        if getattr(self, "root", None) is not None and getattr(self, "bridge", None) is not None:
            threading.Thread(target=self._start_scan_worker, args=(mode,), daemon=True, name="scan-start").start()
            return
        response = self.call_api("/scan/start", {"mode": mode})
        self._finish_start_scan(mode, response)

    def _start_scan_worker(self, mode: str) -> None:
        response = None
        try:
            if hasattr(self, "stream_health"):
                self.stream_health["last_api_error"] = ""
            response = self.bridge.post("/scan/start", {"mode": mode}, timeout_sec=30.0)
        except Exception as exc:
            if hasattr(self, "stream_health"):
                self.stream_health["retries_http"] += 1
                self.stream_health["last_api_error"] = str(exc)
            if hasattr(self, "logger"):
                self.logger.exception("scan start failed")
        if getattr(self, "root", None) is not None:
            self.root.after(0, lambda: self._finish_start_scan(mode, response))

    def _finish_start_scan(self, mode: str, response: dict | None, show_mapping_warning: bool = True) -> None:
        if hasattr(self, "scan_progress"):
            self.scan_progress.stop()
        if response is None:
            self.scan["phase"] = "error"
            self.scan["error"] = "request_failed"
            self.scan["error_reason"] = "request_failed"
            self.scan["pending_start"] = False
            self.scan["pending_mode"] = ""
            if hasattr(self, "scan_state_var"):
                self.scan_state_var.set("request_failed")
            return
        if not bool(response.get("ok", True)):
            detail = str(response.get("error") or response.get("reason") or "scan_start_failed")
            if response.get("reason") == "mapping_prereq_failed":
                self.scan["phase"] = "waiting_mapping"
                self.scan["error"] = detail
                self.scan["error_reason"] = "mapping_prereq_failed"
                self.scan["pending_start"] = True
                self.scan["pending_mode"] = mode
                if hasattr(self, "scan_state_var"):
                    self.scan_state_var.set(self.tr("scan_waiting_mapping"))
                if show_mapping_warning:
                    messagebox.showwarning(self.tr("scan"), mapping_prereq_message(response.get("mapping_prereq") or {}))
                return
            self.scan["phase"] = "error"
            self.scan["error"] = detail
            self.scan["error_reason"] = str(response.get("reason") or detail)
            self.scan["pending_start"] = False
            self.scan["pending_mode"] = ""
            if hasattr(self, "scan_state_var"):
                self.scan_state_var.set(detail)
            return
        if response is not None:
            self.clear_scan()
            self.edit["loaded_from_stcm"] = False
            self.edit["loaded_map_name"] = ""
            self.scan["active"] = True
            self.scan["mode"] = str(response.get("scan_mode") or mode)
            self.scan["phase"] = "scanning"
            self.scan["error"] = ""
            self.scan["error_reason"] = ""
            self.scan["pending_start"] = False
            self.scan["pending_mode"] = ""
            self.scan["started_ms"] = int(time.time() * 1000)
            self.sync_scan_badges()

    def stop_scan(self) -> None:
        mode = str(self.scan.get("mode", "2d")).strip().lower()
        self.scan["phase"] = "stopping"
        if hasattr(self, "scan_state_var"):
            self.scan_state_var.set(self.tr("scan_stopping"))
        if hasattr(self, "scan_progress"):
            self.scan_progress.start(80)
        if getattr(self, "root", None) is not None:
            try:
                self.root.update_idletasks()
            except Exception:
                pass
        response = self.call_api("/scan/stop", {"mode": mode})
        if hasattr(self, "scan_progress"):
            self.scan_progress.stop()
        if response is None:
            self.scan["phase"] = "error"
            self.scan["error"] = "request_failed"
            self.scan["error_reason"] = "request_failed"
            if hasattr(self, "scan_state_var"):
                self.scan_state_var.set("request_failed")
            return
        if not bool(response.get("ok", True)):
            detail = str(response.get("error") or response.get("reason") or "scan_stop_failed")
            self.scan["phase"] = "error"
            self.scan["error"] = detail
            self.scan["error_reason"] = str(response.get("reason") or detail)
            if hasattr(self, "scan_state_var"):
                self.scan_state_var.set(detail)
            return
        self.scan["active"] = False
        self.scan["mode"] = str(response.get("scan_mode") or mode)
        pcd_file = response.get("pcd_file")
        if self.scan["mode"] == "3d" and isinstance(pcd_file, dict):
            self.scan["phase"] = "receiving_pcd"
            if hasattr(self, "scan_state_var"):
                self.scan_state_var.set(self.tr("scan_receiving_pcd"))
            encoded = str(pcd_file.get("content") or "")
            self.scan["pcd_name"] = str(pcd_file.get("name") or "map.pcd")
            self.scan["pcd_bytes"] = base64.b64decode(encoded.encode("ascii")) if encoded else b""
            self.scan["pcd_received_at"] = int(time.time() * 1000)
        self.scan["phase"] = "idle"
        self.scan["error"] = ""
        self.scan["error_reason"] = ""
        self.sync_scan_badges()

    def clear_scan(self) -> None:
        with self.scan_lock:
            self.scan["occupied"] = {}
            self.scan["free"] = {}
            self.scan["front_frames"] = 0
            self.scan["rear_frames"] = 0
            self.scan["raw_points"] = 0
            self.scan["last_accum_pose"] = None
            self.scan["saved_point_count"] = 0
            self.scan["last_saved_file"] = ""
            self.scan["pcd_name"] = ""
            self.scan["pcd_bytes"] = b""
            self.scan["pcd_received_at"] = 0
        self.sync_scan_badges()

    def sync_scan_badges(self) -> None:
        occ = len(self.active_occupancy_cells())
        free = len(self.active_free_cells())
        if self.scan.get("phase") == "error" and self.scan.get("error"):
            self.scan_state_var.set(str(self.scan["error"]))
        elif self.scan.get("phase") == "waiting_mapping":
            self.scan_state_var.set(self.tr("scan_waiting_mapping"))
        elif self.scan.get("phase") == "receiving_pcd":
            self.scan_state_var.set(self.tr("scan_receiving_pcd"))
        elif self.scan["active"]:
            self.scan_state_var.set(self.tr("recording_obstacles", count=occ))
        elif occ or free:
            self.scan_state_var.set(self.tr("stopped_summary", obs=occ, free=free))
        else:
            self.scan_state_var.set(self.tr("idle"))
        self.map_badge_var.set(self.tr("loaded_badge", name=self.edit["loaded_map_name"]) if self.edit["loaded_from_stcm"] else self.tr("scan_session"))
        tool_map = {"view": self.tr("tool_view_select"), "erase": self.tr("tool_erase_noise"), "obstacle": self.tr("tool_draw_obstacle")}
        suffix = " | Pick end point" if self.edit["tool"] == "obstacle" and self.edit["pending_obstacle_start"] else ""
        self.tool_badge_var.set(self.tr("tool_badge", name=tool_map.get(self.edit["tool"], self.tr("tool_view_select")), suffix=suffix))
        self.stats_badge_var.set(self.tr("stats_badge", occ=occ, poi=len(self.poi_nodes), path=len(self.path_segments)))
        self.mark_canvas_dirty()

    def mark_canvas_dirty(self) -> None:
        self.canvas_dirty = True
        self.canvas_revision += 1

    def cell_key(self, ix: int, iy: int) -> str:
        return f"{ix}:{iy}"

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        voxel = float(self.scan["voxel"])
        return round(x / voxel), round(y / voxel)

    def world_to_cell_with_resolution(self, x: float, y: float, resolution: float) -> tuple[int, int]:
        voxel = max(0.02, float(resolution))
        return round(x / voxel), round(y / voxel)

    def should_use_server_grid(self) -> bool:
        server_grid = getattr(self, "server_grid", {}) or {}
        edit = getattr(self, "edit", {}) or {}
        return bool(server_grid.get("active")) and not bool(edit.get("loaded_from_stcm"))

    def active_voxel_size(self) -> float:
        if self.should_use_server_grid():
            return max(0.02, float(getattr(self, "server_grid", {}).get("resolution", getattr(self, "scan", {}).get("voxel", 0.08))))
        return float(self.scan["voxel"])

    def active_occupancy_cells(self) -> list[dict]:
        if self.should_use_server_grid():
            return [dict(cell) for cell in getattr(self, "server_grid", {}).get("occupied_cells", [])]
        return self.filtered_occupancy_cells()

    def active_free_cells(self) -> list[dict]:
        if self.should_use_server_grid():
            return [dict(cell) for cell in getattr(self, "server_grid", {}).get("free_cells", [])]
        return self.filtered_free_cells()

    def active_map_fence_xy(self) -> list[dict[str, float]]:
        if self.should_use_server_grid():
            server_grid = getattr(self, "server_grid", {}) or {}
            width = int(server_grid.get("width", 0) or 0)
            height = int(server_grid.get("height", 0) or 0)
            if width > 0 and height > 0:
                resolution = max(0.02, float(server_grid.get("resolution", self.active_voxel_size())))
                origin = server_grid.get("origin") if isinstance(server_grid.get("origin"), dict) else {}
                min_x = float(origin.get("x", 0.0))
                min_y = float(origin.get("y", 0.0))
                max_x = min_x + width * resolution
                max_y = min_y + height * resolution
                return [
                    {"x": round(min_x, 3), "y": round(min_y, 3)},
                    {"x": round(max_x, 3), "y": round(min_y, 3)},
                    {"x": round(max_x, 3), "y": round(max_y, 3)},
                    {"x": round(min_x, 3), "y": round(max_y, 3)},
                    {"x": round(min_x, 3), "y": round(min_y, 3)},
                ]
        cells = self.active_occupancy_cells() + self.active_free_cells()
        if not cells:
            return []
        voxel = self.active_voxel_size()
        min_x = min(float(cell["ix"]) * voxel for cell in cells)
        min_y = min(float(cell["iy"]) * voxel for cell in cells)
        max_x = (max(float(cell["ix"]) for cell in cells) + 1.0) * voxel
        max_y = (max(float(cell["iy"]) for cell in cells) + 1.0) * voxel
        return [
            {"x": round(min_x, 3), "y": round(min_y, 3)},
            {"x": round(max_x, 3), "y": round(min_y, 3)},
            {"x": round(max_x, 3), "y": round(max_y, 3)},
            {"x": round(min_x, 3), "y": round(max_y, 3)},
            {"x": round(min_x, 3), "y": round(min_y, 3)},
        ]

    def update_server_grid(self, payload: dict[str, Any], stamp: float) -> None:
        was_active = bool(self.server_grid.get("active"))
        resolution = max(0.02, float(payload.get("resolution", getattr(self, "scan", {}).get("voxel", 0.08))))
        occupied_cells: list[dict[str, int | float]] = []
        free_cells: list[dict[str, int]] = []
        data = [int(value) for value in payload.get("data", [])] if isinstance(payload.get("data"), list) else []
        width = int(payload.get("width", 0) or 0)
        if data and width > 0:
            for index, value in enumerate(data):
                if value < 0:
                    continue
                row = index // width
                col = index % width
                if value >= 50:
                    occupied_cells.append({"ix": col, "iy": row, "hits": 3, "intensity": 1.0})
                else:
                    free_cells.append({"ix": col, "iy": row, "hits": 3})
        else:
            for cell in payload.get("occupied", []):
                ix, iy = self.world_to_cell_with_resolution(float(cell.get("x", 0.0)), float(cell.get("y", 0.0)), resolution)
                occupied_cells.append({"ix": ix, "iy": iy, "hits": 3, "intensity": 1.0})
            for cell in payload.get("free", []):
                ix, iy = self.world_to_cell_with_resolution(float(cell.get("x", 0.0)), float(cell.get("y", 0.0)), resolution)
                free_cells.append({"ix": ix, "iy": iy, "hits": 3})
        self.server_grid = {
            "active": True,
            "resolution": resolution,
            "occupied_cells": occupied_cells,
            "free_cells": free_cells,
            "data": data,
            "origin": dict(payload.get("origin", {})) if isinstance(payload.get("origin"), dict) else {"x": 0.0, "y": 0.0},
            "width": width,
            "height": int(payload.get("height", 0) or 0),
            "stamp": float(stamp),
        }
        self.sync_scan_badges()
        if not was_active and hasattr(self, "canvas") and not bool(getattr(self, "edit", {}).get("loaded_from_stcm")):
            self.center_loaded_map()

    def effective_scan_fusion_config(self) -> dict:
        overrides = {
            "voxel_size": max(0.02, self.number(self.voxel_var, float(self.scan_fusion.get("voxel_size", 0.08)))),
            "occupied_min_hits": max(1, round(self.number(self.occupied_min_hits_var, float(self.scan_fusion.get("occupied_min_hits", 2))))),
            "occupied_over_free_ratio": max(0.0, self.number(self.occupied_over_free_ratio_var, float(self.scan_fusion.get("occupied_over_free_ratio", 0.75)))),
            "turn_skip_wz": max(0.0, self.number(self.turn_skip_wz_var, float(self.scan_fusion.get("turn_skip_wz", 0.45)))),
            "skip_turn_frames": bool(self.skip_turn_frames_var.get()),
        }
        return resolve_scan_fusion_config(None, overrides)

    def apply_scan_fusion_config(self, config: dict, update_vars: bool = True) -> dict:
        resolved = resolve_scan_fusion_config(str(config.get("preset", "")), config)
        self.scan_fusion = resolved
        self.scan["voxel"] = float(resolved["voxel_size"])
        if update_vars:
            if hasattr(self, "scan_fusion_preset_var"):
                self.scan_fusion_preset_var.set("")
            self.voxel_var.set(f"{float(resolved['voxel_size']):.2f}")
            self.occupied_min_hits_var.set(str(int(resolved["occupied_min_hits"])))
            self.occupied_over_free_ratio_var.set(f"{float(resolved['occupied_over_free_ratio']):.2f}")
            self.turn_skip_wz_var.set(f"{float(resolved['turn_skip_wz']):.2f}")
            self.skip_turn_frames_var.set(bool(resolved["skip_turn_frames"]))
        return resolved

    def apply_scan_fusion_config_from_ui(self) -> dict:
        return self.apply_scan_fusion_config(self.effective_scan_fusion_config(), update_vars=True)

    def on_scan_fusion_preset_selected(self) -> dict:
        return self.apply_scan_fusion_config({"preset": self.scan_fusion_preset_var.get()}, update_vars=True)

    def occupied_lookup(self) -> dict[tuple[int, int], dict]:
        return {(int(cell["ix"]), int(cell["iy"])): cell for cell in self.scan["occupied"].values()}

    def mark_free(self, ix: int, iy: int) -> None:
        _mark_free(self.scan, ix, iy)

    def mark_occupied(self, ix: int, iy: int, intensity: float, hits: int = 1) -> None:
        _mark_occupied(self.scan, ix, iy, intensity, hits)

    def raytrace(self, start_x: int, start_y: int, end_x: int, end_y: int) -> None:
        _raytrace(self.scan, start_x, start_y, end_x, end_y)

    def pose_for_stamp(self, stamp: float) -> dict:
        if not self.pose_history:
            return self.odom
        best = self.pose_history[-1]
        best_delta = abs(best["stamp"] - stamp)
        for item in self.pose_history:
            delta = abs(item["stamp"] - stamp)
            if delta < best_delta:
                best = item
                best_delta = delta
        return best["pose"]

    def accumulate_points(self, points: list, stamp: float, keyframe: bool) -> None:
        with self.scan_lock:
            changed = process_scan_frame(
                self.scan,
                points=points,
                pose=self.pose_for_stamp(stamp),
                keyframe=keyframe,
                config=self.apply_scan_fusion_config_from_ui(),
                logger=self.logger,
            )
        if changed:
            self.sync_scan_badges()

    def filtered_occupancy_cells(self) -> list[dict]:
        config = self.effective_scan_fusion_config()
        cells: list[dict] = []
        for cell in self.scan["occupied"].values():
            free = self.scan["free"].get(self.cell_key(int(cell["ix"]), int(cell["iy"])))
            if not is_occupied_scan_cell(cell, free, config):
                continue
            cells.append({"ix": int(cell["ix"]), "iy": int(cell["iy"]), "hits": int(cell["hits"]), "intensity": float(cell["intensity"])})
        return cells

    def filtered_free_cells(self) -> list[dict]:
        occupied_by_key = {self.cell_key(int(cell["ix"]), int(cell["iy"])): cell for cell in self.scan["occupied"].values()}
        cells: list[dict] = []
        for cell in self.scan["free"].values():
            occ = occupied_by_key.get(self.cell_key(int(cell["ix"]), int(cell["iy"])))
            occ_hits = int(occ["hits"]) if occ else 0
            if int(cell["hits"]) <= occ_hits * 0.8:
                continue
            cells.append({"ix": int(cell["ix"]), "iy": int(cell["iy"]), "hits": int(cell["hits"])})
        return cells

    def occupied_points(self) -> list[list[float]]:
        points = []
        voxel = self.active_voxel_size()
        for cell in self.active_occupancy_cells():
            points.append([float(cell["ix"]) * voxel, float(cell["iy"]) * voxel, float(cell.get("intensity", 1.0))])
        return points or [[0.0, 0.0, 1.0]]

    def browser_occupancy(self) -> dict:
        config = self.effective_scan_fusion_config()
        return {
            "voxel_size": self.active_voxel_size(),
            "scan_fusion": build_scan_fusion_metadata(config),
            "occupied_cells": self.active_occupancy_cells(),
            "free_cells": self.active_free_cells(),
            "map_fence_xy": self.active_map_fence_xy(),
        }

    def point_from_poi(self, poi: Poi) -> Point:
        return Point(x=poi.x, y=poi.y, name=poi.name, yaw=poi.yaw, lat=poi.lat, lon=poi.lon, poi_id=poi.client_id)

    def point_from_dict(self, payload: dict) -> Point:
        return Point(
            x=float(payload.get("x", 0.0)),
            y=float(payload.get("y", 0.0)),
            name=str(payload.get("name", "")),
            yaw=float(payload.get("yaw", 0.0) or 0.0),
            lat=float(payload["lat"]) if payload.get("lat") not in (None, "") else None,
            lon=float(payload["lon"]) if payload.get("lon") not in (None, "") else None,
            poi_id=payload.get("poi_id") or payload.get("poiId"),
        )

    def point_to_payload(self, point: Point) -> dict:
        return {
            "x": float(point.x),
            "y": float(point.y),
            "name": point.name,
            "yaw": float(point.yaw),
            "lat": None if point.lat is None else float(point.lat),
            "lon": None if point.lon is None else float(point.lon),
            "poi_id": point.poi_id,
        }

    def poi_payload(self, poi: Poi) -> dict:
        return self.point_to_payload(self.point_from_poi(poi))

    def parse_geo_text(self, text: str, label: str) -> tuple[float | None, float | None]:
        lat, lon = self.parse_geo(text.strip())
        if text.strip() and lat is None:
            raise ValueError(self.tr("poi_geo_format", label=label))
        return lat, lon

    def apply_geo_rules_to_pois(self, points: list[Poi]) -> list[Poi]:
        copies = [Poi(**poi.__dict__) for poi in points]
        geo_count = sum(1 for poi in copies if poi.lat is not None and poi.lon is not None)
        if geo_count in (1, 2):
            raise ValueError(self.tr("poi_geo_min3"))
        if geo_count >= 3:
            inferred = [self.point_from_poi(poi) for poi in copies]
            infer_missing_geo_points(inferred)
            for poi, point in zip(copies, inferred):
                poi.lat = point.lat
                poi.lon = point.lon
        return copies

    def build_segment(self, start: Point, end: Point, source: str) -> dict:
        clearance = max(0.0, self.number(self.path_clearance_var, 0.3))
        points = plan_path_points(start, end, float(self.scan["voxel"]), self.occupied_lookup(), clearance)
        seg = {
            "id": f"seg-{self.segment_seed}",
            "start": self.point_to_payload(start),
            "end": self.point_to_payload(end),
            "source": source,
            "geometry": "line",
            "curveOffset": 0.0,
            "clearance": clearance,
            "points": [self.point_to_payload(point) for point in points],
        }
        self.segment_seed += 1
        return seg

    def toggle_add_poi(self) -> None:
        if self.pending_poi is not None or self.pending_poi_queue:
            self.pending_poi = None
            self.pending_poi_queue = []
            self.poi_status_var.set(self.tr("poi_idle"))
            self.batch_action_btn.configure(text="Start Batch Add")
            return
        try:
            queue = parse_batch_poi_text(self.poi_batch_text.get("1.0", tk.END))
        except ValueError as exc:
            messagebox.showwarning(self.tr("poi"), str(exc))
            return
        if not queue:
            messagebox.showwarning(self.tr("poi"), self.tr("poi_batch_requires_input"))
            return
        self.pending_poi_queue = queue
        self.batch_action_btn.configure(text=self.tr("cancel_batch", count=len(queue)))
        self.start_next_poi_draft()

    def start_next_poi_draft(self) -> None:
        if self.pending_poi is not None or not self.pending_poi_queue:
            return
        self.pending_poi = self.pending_poi_queue.pop(0)
        self.pending_poi["batch_mode"] = True
        self.poi_status_var.set(self.tr("poi_ready_place", name=self.pending_poi["name"]))
        self.batch_action_btn.configure(text=self.tr("cancel_batch", count=len(self.pending_poi_queue) + 1))

    def parse_geo(self, text: str) -> tuple[float | None, float | None]:
        if not text:
            return None, None
        parts = [p.strip() for p in text.split(",")]
        if len(parts) != 2:
            return None, None
        try:
            lon = float(parts[0])
            lat = float(parts[1])
            if lon < -180 or lon > 180 or lat < -90 or lat > 90:
                return None, None
            return lat, lon
        except Exception:
            return None, None

    def place_poi(self, x: float, y: float) -> None:
        if self.pending_poi is None:
            return
        poi = Poi(
            client_id=f"poi-{self.poi_seed}",
            name=self.pending_poi["name"],
            x=x,
            y=y,
            yaw=float(self.pending_poi.get("yaw") or self.pose.get("yaw", 0.0)),
            lat=self.pending_poi.get("lat"),
            lon=self.pending_poi.get("lon"),
        )
        self.poi_seed += 1
        next_nodes = self.poi_nodes + [poi]
        batch_mode = bool(self.pending_poi.get("batch_mode"))
        if batch_mode and self.pending_poi_queue:
            pass
        else:
            try:
                next_nodes = self.apply_geo_rules_to_pois(next_nodes)
            except ValueError as exc:
                messagebox.showwarning(self.tr("poi"), str(exc))
                return
        self.poi_nodes = next_nodes
        if self.bridge and self.bridge.connected:
            self.call_api("/map/poi", {"poi": self.poi_payload(self.poi_nodes[-1])})
        self.pending_poi = None
        if self.pending_poi_queue:
            self.start_next_poi_draft()
        else:
            self.poi_name_var.set("")
            self.poi_status_var.set(self.tr("poi_idle"))
            self.batch_action_btn.configure(text=self.tr("start_batch_add"))
        self.sync_poi_box()

    def add_single_poi(self) -> None:
        name = self.single_poi_name_var.get().strip()
        if not name:
            messagebox.showwarning(self.tr("poi"), self.tr("poi_single_requires_name"))
            return
        try:
            x = self.number(self.single_poi_x_var, 0.0)
            y = self.number(self.single_poi_y_var, 0.0)
            yaw = self.number(self.single_poi_yaw_var, 0.0)
            lat, lon = self.parse_geo_text(self.single_poi_geo_var.get(), "Geo")
        except ValueError as exc:
            messagebox.showwarning(self.tr("poi"), str(exc))
            return
        next_nodes = self.poi_nodes + [
            Poi(client_id=f"poi-{self.poi_seed}", name=name, x=x, y=y, yaw=yaw, lat=lat, lon=lon)
        ]
        try:
            next_nodes = self.apply_geo_rules_to_pois(next_nodes)
        except ValueError as exc:
            messagebox.showwarning(self.tr("poi"), str(exc))
            return
        self.poi_nodes = next_nodes
        self.poi_seed += 1
        self.single_poi_name_var.set("")
        self.single_poi_geo_var.set("")
        self.sync_poi_box()
        self.poi_status_var.set(self.tr("poi_added", name=name))

    def apply_poi_edit(self) -> None:
        selected = [poi for poi in self.poi_nodes if poi.client_id in self.selected_poi_ids]
        if len(selected) != 1:
            messagebox.showwarning(self.tr("poi"), self.tr("poi_edit_requires_one"))
            return
        target = selected[0]
        try:
            lat, lon = self.parse_geo_text(self.edit_poi_geo_var.get(), "Geo")
            updated = Poi(
                client_id=target.client_id,
                name=self.edit_poi_name_var.get().strip() or target.name,
                x=self.number(self.edit_poi_x_var, target.x),
                y=self.number(self.edit_poi_y_var, target.y),
                yaw=self.number(self.edit_poi_yaw_var, target.yaw),
                lat=lat,
                lon=lon,
            )
        except ValueError as exc:
            messagebox.showwarning(self.tr("poi"), str(exc))
            return
        next_nodes = []
        for poi in self.poi_nodes:
            next_nodes.append(updated if poi.client_id == target.client_id else Poi(**poi.__dict__))
        try:
            next_nodes = self.apply_geo_rules_to_pois(next_nodes)
        except ValueError as exc:
            messagebox.showwarning(self.tr("poi"), str(exc))
            return
        self.poi_nodes = next_nodes
        self.sync_poi_box()
        self.poi_status_var.set(self.tr("poi_updated", name=updated.name))

    def sync_selected_poi(self) -> None:
        self.selected_poi_ids = set()
        for idx in self.poi_box.curselection():
            if 0 <= idx < len(self.poi_nodes):
                self.selected_poi_ids.add(self.poi_nodes[idx].client_id)
        selected = [poi for poi in self.poi_nodes if poi.client_id in self.selected_poi_ids]
        if len(selected) == 1:
            poi = selected[0]
            self.edit_poi_name_var.set(poi.name)
            self.edit_poi_x_var.set(f"{poi.x:.3f}")
            self.edit_poi_y_var.set(f"{poi.y:.3f}")
            self.edit_poi_yaw_var.set(f"{poi.yaw:.3f}")
            self.edit_poi_geo_var.set("" if poi.lat is None or poi.lon is None else f"{poi.lon:.6f},{poi.lat:.6f}")
        self.sync_scan_badges()
        self.sync_path_panel()

    def sync_poi_box(self) -> None:
        self.poi_box.delete(0, tk.END)
        for index, poi in enumerate(self.poi_nodes, start=1):
            self.poi_box.insert(tk.END, f"{index}. {poi.name} ({poi.x:.2f}, {poi.y:.2f}) yaw={poi.yaw:.3f} lat={poi.lat if poi.lat is not None else 'n/a'} lon={poi.lon if poi.lon is not None else 'n/a'}")
        self.sync_scan_badges()

    def delete_selected_poi(self) -> None:
        if not self.selected_poi_ids:
            messagebox.showwarning(self.tr("poi"), self.tr("poi_select_delete"))
            return
        next_nodes = [poi for poi in self.poi_nodes if poi.client_id not in self.selected_poi_ids]
        try:
            next_nodes = self.apply_geo_rules_to_pois(next_nodes)
        except ValueError as exc:
            messagebox.showwarning(self.tr("poi"), str(exc))
            return
        self.poi_nodes = next_nodes
        self.path_segments = [seg for seg in self.path_segments if seg["start"].get("poi_id") not in self.selected_poi_ids and seg["end"].get("poi_id") not in self.selected_poi_ids]
        self.selected_poi_ids = set()
        self.selected_segment_id = None
        self.sync_poi_box()
        self.sync_path_panel()

    def apply_selected_geo(self) -> None:
        if not self.selected_poi_ids:
            messagebox.showwarning(self.tr("poi"), self.tr("poi_select_first"))
            return
        lat, lon = self.parse_geo(self.poi_geo_var.get().strip())
        if self.poi_geo_var.get().strip() and lat is None:
            messagebox.showwarning("POI", "Geo format must be lon,lat.")
            return
        for poi in self.poi_nodes:
            if poi.client_id in self.selected_poi_ids:
                poi.lat = lat
                poi.lon = lon
        self.sync_poi_box()

    def copy_poi_text(self) -> None:
        if not self.poi_nodes:
            messagebox.showwarning(self.tr("poi"), self.tr("poi_no_copy"))
            return
        text = build_poi_copy_text([self.point_from_poi(poi) for poi in self.poi_nodes])
        self.root.clipboard_clear()
        self.root.clipboard_append(text)
        messagebox.showinfo(self.tr("poi"), self.tr("poi_copied"))

    def find_poi_name(self, name: str) -> Poi | None:
        target = name.strip().lower()
        if not target:
            messagebox.showwarning(self.tr("path"), self.tr("path_need_names"))
            return None
        matches = [poi for poi in self.poi_nodes if poi.name.strip().lower() == target]
        if not matches:
            messagebox.showwarning(self.tr("path"), self.tr("path_poi_not_found", name=name))
            return None
        if len(matches) > 1:
            messagebox.showwarning(self.tr("path"), self.tr("path_poi_duplicate", name=name))
            return None
        return matches[0]

    def add_segment(self, seg: dict) -> None:
        self.path_segments.append(seg)
        self.selected_segment_id = seg["id"]
        self.sync_path_panel()

    def rebuild_path_nodes(self) -> None:
        self.path_nodes = []
        for seg in self.path_segments:
            samples = seg.get("points") or [seg["start"], seg["end"]]
            for index, point in enumerate(samples):
                node = {"x": float(point["x"]), "y": float(point["y"]), "lat": point.get("lat"), "lon": point.get("lon")}
                prev = self.path_nodes[-1] if self.path_nodes else None
                if index > 0 and prev and f"{prev['x']:.3f},{prev['y']:.3f}" == f"{node['x']:.3f},{node['y']:.3f}":
                    continue
                self.path_nodes.append(node)

    def connect_named_poi(self) -> None:
        a = self.find_poi_name(self.path_start_var.get().strip())
        b = self.find_poi_name(self.path_end_var.get().strip())
        if a is None or b is None:
            return
        if a.client_id == b.client_id:
            messagebox.showwarning(self.tr("path"), self.tr("path_same_poi"))
            return
        try:
            self.add_segment(self.build_segment(self.point_from_poi(a), self.point_from_poi(b), "poi-name"))
        except ValueError as exc:
            messagebox.showwarning(self.tr("path"), str(exc))

    def auto_loop(self) -> None:
        if len(self.poi_nodes) < 2:
            messagebox.showwarning(self.tr("path"), self.tr("path_need_two_poi"))
            return
        route = optimize_loop_with_two_opt(solve_nearest_loop([self.point_from_poi(poi) for poi in self.poi_nodes]))
        preserved = [seg for seg in self.path_segments if seg["source"] != "auto"]
        auto_segments: list[dict] = []
        try:
            for index in range(len(route) - 1):
                auto_segments.append(self.build_segment(route[index], route[index + 1], "auto"))
            if len(route) > 2:
                auto_segments.append(self.build_segment(route[-1], route[0], "auto"))
        except ValueError as exc:
            messagebox.showwarning(self.tr("path"), str(exc))
            return
        self.path_segments = preserved + auto_segments
        self.selected_segment_id = self.path_segments[-1]["id"] if self.path_segments else None
        self.sync_path_panel()

    def sync_selected_segment(self) -> None:
        sel = self.path_box.curselection()
        self.selected_segment_id = self.path_segments[sel[0]]["id"] if sel and sel[0] < len(self.path_segments) else None
        self.sync_path_panel()

    def delete_selected_segment(self) -> None:
        if self.selected_segment_id is None:
            return
        self.path_segments = [seg for seg in self.path_segments if seg["id"] != self.selected_segment_id]
        self.selected_segment_id = None
        self.sync_path_panel()

    def clear_selection(self) -> None:
        self.selected_segment_id = None
        self.selected_poi_ids = set()
        self.pending_free_point = None
        self.path_validation = {"checked": False, "ok": None, "invalid_ids": set(), "message": ""}
        self.poi_box.selection_clear(0, tk.END)
        self.path_box.selection_clear(0, tk.END)
        self.sync_path_panel()
        self.sync_poi_box()

    def validate_path(self, show_alert: bool) -> bool:
        segments = [{"id": seg["id"], "start": self.point_from_dict(seg["start"]), "end": self.point_from_dict(seg["end"])} for seg in self.path_segments]
        self.path_validation = compute_path_closed_loop_validation(segments, float(self.scan["voxel"]))
        self.selected_segment_id = None
        self.selected_poi_ids = set()
        self.sync_path_panel()
        if show_alert:
            messagebox.showinfo(self.tr("path_validation_title"), self.path_validation["message"])
        return bool(self.path_validation["ok"])

    def path_polyline_length(self, seg: dict) -> float:
        points = seg.get("points") or [seg["start"], seg["end"]]
        total = 0.0
        for index in range(len(points) - 1):
            total += math.hypot(points[index + 1]["x"] - points[index]["x"], points[index + 1]["y"] - points[index]["y"])
        return total

    def sync_path_panel(self) -> None:
        self.rebuild_path_nodes()
        self.path_box.delete(0, tk.END)
        for seg in self.path_segments:
            suffix = " | closed-loop error" if seg["id"] in self.path_validation["invalid_ids"] else ""
            self.path_box.insert(tk.END, f"{seg['source']} | ({seg['start']['x']:.2f}, {seg['start']['y']:.2f}) -> ({seg['end']['x']:.2f}, {seg['end']['y']:.2f}) | len {self.path_polyline_length(seg):.2f} m | key points {len(seg.get('points') or [])}{suffix}")
        tool_map = {
            "idle": self.tr("path_browse_only"),
            "poi": self.tr("path_tool_named", start=self.path_start_var.get().strip() or "?", end=self.path_end_var.get().strip() or "?"),
            "free": self.tr("path_tool_free"),
        }
        pending = ""
        if self.pending_free_point:
            pending = f" | Start ({self.pending_free_point['x']:.2f}, {self.pending_free_point['y']:.2f})"
        validation = self.tr("loop_unchecked") if not self.path_validation["checked"] else self.tr("loop_ok") if self.path_validation["ok"] else self.tr("loop_error", count=len(self.path_validation["invalid_ids"]))
        self.path_status_var.set(self.tr("path_status", segments=len(self.path_segments), nodes=len(self.path_nodes), tool=tool_map[self.path_mode_var.get()], pending=pending, validation=validation))
        if self.path_mode_var.get() == "poi" and self.path_start_var.get().strip() and self.path_end_var.get().strip():
            self.connect_named_btn.state(["!disabled"])
        else:
            self.connect_named_btn.state(["disabled"])
        self.sync_scan_badges()

    def refresh_camera_snapshot(self) -> None:
        self.logger.info("camera snapshot refresh")
        self.camera_display = {idx: dict(payload) for idx, payload in self.camera_inbox.items()}
        self.camera_refresh_var.set(self.camera_refresh_var.get().replace("Buffered", "Displayed", 1) if "Buffered" in self.camera_refresh_var.get() else self.camera_refresh_var.get())

    def move_click(self, name: str) -> None:
        command = self.build_control_command(name)
        if command is None:
            return
        path, body, _label = command
        self.call_api_async(path, body)

    def build_control_command(self, name: str) -> tuple[str, dict, str] | None:
        fwd = self.number(self.forward_var, 0.8)
        rev = self.number(self.reverse_var, 0.5)
        turn = self.number(self.turn_var, 1.0)
        if name == "stop":
            return ("/control/stop", {}, "stop")
        if name == "forward":
            body = {"velocity": fwd, "yaw_rate": 0.0}
        elif name == "reverse":
            body = {"velocity": -rev, "yaw_rate": 0.0}
        elif name == "left":
            body = {"velocity": 0.0, "yaw_rate": turn}
        else:
            body = {"velocity": 0.0, "yaw_rate": -turn}
        return ("/control/target", body, name)

    def keyboard_command(self) -> str | None:
        if "space" in self.keys_down:
            return "stop"
        if "w" in self.keys_down or "up" in self.keys_down:
            return "forward"
        if "s" in self.keys_down or "down" in self.keys_down:
            return "reverse"
        if "a" in self.keys_down or "left" in self.keys_down:
            return "left"
        if "d" in self.keys_down or "right" in self.keys_down:
            return "right"
        return None

    def clear_control_target(self) -> None:
        with self.control_lock:
            self.control_target = None
        self.control_sender_event.set()

    def update_control_target(self, name: str | None) -> None:
        if name is None:
            self.clear_control_target()
            return
        command = self.build_control_command(name)
        if command is None:
            return
        with self.control_lock:
            if self.control_target == command:
                return
            self.control_target = command
        self.control_sender_event.set()

    def send_control_command_now(self, path: str, body: dict) -> None:
        bridge = self.bridge
        if bridge is None or not bridge.connected:
            return
        bridge.post(path, body, retries=0, timeout_sec=0.35, backoff_base_sec=0.0)
        if hasattr(self, "stream_health"):
            self.stream_health["control_failures_consecutive"] = 0

    def _control_sender_loop(self) -> None:
        while not self.control_sender_stop.is_set():
            repeat_sec = max(0.06, self.number(self.repeat_ms_var, 120) / 1000.0)
            self.control_sender_event.wait(timeout=repeat_sec)
            self.control_sender_event.clear()
            with self.control_lock:
                command = self.control_target
            if command is None:
                continue
            path, body, _label = command
            try:
                self.send_control_command_now(path, body)
            except Exception as exc:
                if hasattr(self, "stream_health"):
                    self.stream_health["retries_http"] += 1
                    self.stream_health["control_failures_consecutive"] = int(self.stream_health.get("control_failures_consecutive", 0) or 0) + 1
                    self.stream_health["last_api_error"] = str(exc)
                if hasattr(self, "logger"):
                    self.logger.warning("control sender failed path=%s err=%s", path, exc)

    def ensure_drive_loop(self) -> None:
        command = self.keyboard_command()
        self.update_control_target(command)
        if command is not None:
            self.keyboard_var.set(self.tr("keyboard_cmd", cmd=command))

    def cancel_pending_keyup_stop(self) -> None:
        if self.pending_keyup_stop_id is None:
            return
        try:
            self.root.after_cancel(self.pending_keyup_stop_id)
        except Exception:
            pass
        self.pending_keyup_stop_id = None

    def confirm_keyup_stop(self) -> None:
        self.pending_keyup_stop_id = None
        if not self.stop_on_keyup_var.get() or self.keys_down:
            return
        self.clear_control_target()
        self.move_click("stop")
        self.keyboard_var.set(self.tr("keyboard_stop_keyup"))

    def schedule_keyup_stop(self) -> None:
        self.cancel_pending_keyup_stop()
        self.pending_keyup_stop_id = self.root.after(KEYUP_STOP_CONFIRM_MS, self.confirm_keyup_stop)

    def should_ignore_global_keys(self, widget: tk.Misc | None) -> bool:
        return isinstance(widget, (tk.Entry, tk.Text, tk.Listbox, ttk.Entry, ttk.Combobox))

    def on_key_press(self, event: tk.Event) -> None:
        if self.should_ignore_global_keys(self.root.focus_get()):
            return
        key = event.keysym.lower()
        if key in {"w", "a", "s", "d", "up", "down", "left", "right", "space"}:
            self.cancel_pending_keyup_stop()
            self.keys_down.add(key)
            self.ensure_drive_loop()

    def on_key_release(self, event: tk.Event) -> None:
        if self.should_ignore_global_keys(self.root.focus_get()):
            return
        key = event.keysym.lower()
        self.keys_down.discard(key)
        if self.keys_down:
            self.ensure_drive_loop()
            return
        self.clear_control_target()
        if self.stop_on_keyup_var.get():
            self.schedule_keyup_stop()

    def edit_tool_changed(self) -> None:
        self.edit["tool"] = self.edit_tool_var.get()
        self.edit["pending_obstacle_start"] = None
        if self.edit["tool"] == "erase":
            self.map_edit_status_var.set(self.tr("map_edit_erase", radius=self.number(self.brush_var, 0.25)))
        elif self.edit["tool"] == "obstacle":
            self.map_edit_status_var.set(self.tr("map_edit_obstacle"))
        else:
            self.map_edit_status_var.set(self.tr("map_edit_view"))
        self.sync_scan_badges()

    def canvas_press(self, event: tk.Event) -> None:
        self.view["last_xy"] = (event.x, event.y)
        self.view["moved"] = False
        x, y = self.screen_to_world(event.x, event.y)
        if self.pending_poi is not None:
            self.place_poi(x, y)
            return
        if self.edit["tool"] == "erase":
            self.edit["erasing"] = True
            self.erase_radius(x, y)
            return
        if self.edit["tool"] == "obstacle":
            if self.edit["pending_obstacle_start"] is None:
                self.edit["pending_obstacle_start"] = (x, y)
                self.map_edit_status_var.set(self.tr("obstacle_start", x=x, y=y))
            else:
                self.draw_obstacle_line(self.edit["pending_obstacle_start"], (x, y))
                self.edit["pending_obstacle_start"] = None
                self.map_edit_status_var.set(self.tr("obstacle_added"))
            self.sync_scan_badges()
            return
        self.view["dragging"] = True

    def canvas_drag(self, event: tk.Event) -> None:
        if self.edit["erasing"]:
            x, y = self.screen_to_world(event.x, event.y)
            self.erase_radius(x, y)
            return
        if not self.view["dragging"] or self.edit["tool"] != "view":
            return
        dx = event.x - self.view["last_xy"][0]
        dy = event.y - self.view["last_xy"][1]
        if dx or dy:
            self.view["moved"] = True
        self.view["pan_x"] += dx
        self.view["pan_y"] += dy
        self.view["last_xy"] = (event.x, event.y)
        self.update_view_metrics()

    def canvas_release(self, event: tk.Event) -> None:
        if self.edit["erasing"]:
            self.edit["erasing"] = False
            return
        if not self.view["dragging"]:
            return
        self.view["dragging"] = False
        if self.view["moved"]:
            return
        x, y = self.screen_to_world(event.x, event.y)
        if self.path_mode_var.get() == "free":
            point = Point(x=x, y=y, lat=self.gps.get("lat"), lon=self.gps.get("lon"))
            if self.pending_free_point is None:
                self.pending_free_point = self.point_to_payload(point)
            else:
                try:
                    self.add_segment(self.build_segment(self.point_from_dict(self.pending_free_point), point, "free"))
                except ValueError as exc:
                    messagebox.showwarning(self.tr("path"), str(exc))
                self.pending_free_point = None
            self.sync_path_panel()

    def erase_radius(self, world_x: float, world_y: float) -> None:
        radius = max(0.05, self.number(self.brush_var, 0.25))
        radius_cells = max(1, round(radius / float(self.scan["voxel"])))
        cx, cy = self.world_to_cell(world_x, world_y)
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                if math.hypot(dx, dy) * float(self.scan["voxel"]) > radius:
                    continue
                key = self.cell_key(cx + dx, cy + dy)
                self.scan["occupied"].pop(key, None)
                self.scan["free"].pop(key, None)
        self.sync_scan_badges()

    def draw_obstacle_line(self, start: tuple[float, float], end: tuple[float, float]) -> None:
        sx, sy = self.world_to_cell(start[0], start[1])
        ex, ey = self.world_to_cell(end[0], end[1])
        steps = max(abs(ex - sx), abs(ey - sy), 1)
        for step in range(steps + 1):
            t = step / steps
            self.mark_occupied(round(sx + (ex - sx) * t), round(sy + (ey - sy) * t), 1.0, 3)

    def auto_clear_noise(self) -> None:
        removable = []
        for key, cell in self.scan["occupied"].items():
            if int(cell["hits"]) > 3:
                continue
            neighbors = 0
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    if self.cell_key(int(cell["ix"]) + dx, int(cell["iy"]) + dy) in self.scan["occupied"]:
                        neighbors += 1
            if neighbors <= 1:
                removable.append(key)
        for key in removable:
            self.scan["occupied"].pop(key, None)
        self.map_edit_status_var.set(self.tr("noise_cleared", count=len(removable)) if removable else self.tr("noise_none"))
        self.sync_scan_badges()

    def clear_loaded_map(self) -> None:
        self.clear_scan()
        self.server_grid = {"active": False, "resolution": 0.0, "occupied_cells": [], "free_cells": [], "origin": {"x": 0.0, "y": 0.0}, "width": 0, "height": 0, "stamp": 0.0}
        self.poi_nodes = []
        self.path_segments = []
        self.path_nodes = []
        self.selected_poi_ids = set()
        self.selected_segment_id = None
        self.pending_free_point = None
        self.pending_poi = None
        self.pending_poi_queue = []
        self.edit["loaded_from_stcm"] = False
        self.edit["loaded_map_name"] = ""
        self.path_validation = {"checked": False, "ok": None, "invalid_ids": set(), "message": ""}
        self.sync_poi_box()
        self.sync_path_panel()
        self.map_edit_status_var.set(self.tr("map_cleared"))
        self.reset_view()

    def reset_server_map(self) -> None:
        response = self.call_api("/map/reset", {})
        if not response or not response.get("ok"):
            if hasattr(self, "map_edit_status_var"):
                self.map_edit_status_var.set(self.tr("reset_map_failed"))
            return
        self.clear_loaded_map()
        if hasattr(self, "map_edit_status_var"):
            self.map_edit_status_var.set(self.tr("reset_map_done"))

    def world_to_screen(self, x: float, y: float) -> tuple[float, float]:
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        return width / 2 + self.view["pan_x"] + x * self.view["scale"], height / 2 + self.view["pan_y"] - y * self.view["scale"]

    def screen_to_world(self, x: float, y: float) -> tuple[float, float]:
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        return (x - width / 2 - self.view["pan_x"]) / self.view["scale"], (height / 2 + self.view["pan_y"] - y) / self.view["scale"]

    def on_mousewheel(self, event: tk.Event) -> None:
        if not can_zoom_from_widget(getattr(event, "widget", None), self.canvas):
            return
        x, y = self.screen_to_world(event.x, event.y)
        factor = zoom_scale_factor(event)
        if factor == 1.0:
            return
        self.zoom_view(factor, anchor=(x, y), screen_xy=(event.x, event.y))

    def zoom_view(
        self,
        factor: float,
        anchor: tuple[float, float] | None = None,
        screen_xy: tuple[float, float] | None = None,
    ) -> None:
        if factor <= 0:
            return
        self.view["scale"] = max(8.0, min(80.0, self.view["scale"] * factor))
        if anchor is not None and screen_xy is not None:
            sx, sy = self.world_to_screen(anchor[0], anchor[1])
            self.view["pan_x"] += screen_xy[0] - sx
            self.view["pan_y"] += screen_xy[1] - sy
        self.update_view_metrics()

    def on_root_click(self, event: tk.Event) -> None:
        if should_clear_focus_on_click(getattr(event, "widget", None)):
            self.root.focus_set()

    def fit_world_bounds(
        self,
        min_x: float,
        min_y: float,
        max_x: float,
        max_y: float,
        preferred_scale: float = 80.0,
        padding_px: float = 48.0,
    ) -> None:
        width = max(1.0, float(self.canvas.winfo_width()))
        height = max(1.0, float(self.canvas.winfo_height()))
        usable_width = max(1.0, width - padding_px * 2.0)
        usable_height = max(1.0, height - padding_px * 2.0)
        world_width = max(0.1, float(max_x) - float(min_x))
        world_height = max(0.1, float(max_y) - float(min_y))
        fit_scale = min(usable_width / world_width, usable_height / world_height)
        self.view["scale"] = max(8.0, min(80.0, min(float(preferred_scale), fit_scale)))
        center_x = (float(min_x) + float(max_x)) / 2.0
        center_y = (float(min_y) + float(max_y)) / 2.0
        self.view["pan_x"] = -center_x * self.view["scale"]
        self.view["pan_y"] = center_y * self.view["scale"]
        self.update_view_metrics()

    def active_map_world_bounds(self, include_robot: bool = False) -> tuple[float, float, float, float] | None:
        if self.should_use_server_grid():
            resolution = max(0.02, float(self.server_grid.get("resolution", getattr(self, "scan", {}).get("voxel", 0.08))))
            origin = self.server_grid.get("origin") if isinstance(self.server_grid.get("origin"), dict) else {}
            width = int(self.server_grid.get("width", 0))
            height = int(self.server_grid.get("height", 0))
            if width > 0 and height > 0:
                min_x = float(origin.get("x", 0.0))
                min_y = float(origin.get("y", 0.0))
                max_x = min_x + width * resolution
                max_y = min_y + height * resolution
                if include_robot:
                    min_x = min(min_x, float(self.pose.get("x", 0.0)))
                    min_y = min(min_y, float(self.pose.get("y", 0.0)))
                    max_x = max(max_x, float(self.pose.get("x", 0.0)))
                    max_y = max(max_y, float(self.pose.get("y", 0.0)))
                return (min_x, min_y, max_x, max_y)
        occupied_cells = self.active_occupancy_cells()
        free_cells = self.active_free_cells()
        cells = occupied_cells + free_cells
        if not cells:
            if include_robot:
                robot_x = float(self.pose.get("x", 0.0))
                robot_y = float(self.pose.get("y", 0.0))
                return (robot_x - 1.0, robot_y - 1.0, robot_x + 1.0, robot_y + 1.0)
            return None
        voxel = self.active_voxel_size()
        xs = [float(cell["ix"]) * voxel for cell in cells]
        ys = [float(cell["iy"]) * voxel for cell in cells]
        min_x = min(xs)
        min_y = min(ys)
        max_x = max(xs)
        max_y = max(ys)
        if include_robot:
            min_x = min(min_x, float(self.pose.get("x", 0.0)))
            min_y = min(min_y, float(self.pose.get("y", 0.0)))
            max_x = max(max_x, float(self.pose.get("x", 0.0)))
            max_y = max(max_y, float(self.pose.get("y", 0.0)))
        return (min_x, min_y, max_x, max_y)

    def center_robot(self) -> None:
        bounds = self.active_map_world_bounds(include_robot=True)
        if bounds is not None:
            self.fit_world_bounds(*bounds)
            return
        self.view["pan_x"] = -float(self.pose.get("x", 0.0)) * self.view["scale"]
        self.view["pan_y"] = float(self.pose.get("y", 0.0)) * self.view["scale"]
        self.update_view_metrics()

    def center_loaded_map(self) -> None:
        bounds = self.active_map_world_bounds(include_robot=False)
        if bounds is None:
            self.center_robot()
            return
        self.fit_world_bounds(*bounds)

    def reset_view(self) -> None:
        bounds = self.active_map_world_bounds(include_robot=True)
        if bounds is not None:
            self.fit_world_bounds(*bounds)
            return
        self.view["scale"] = 80.0
        self.center_robot()

    def update_view_metrics(self) -> None:
        pan_x = -self.view["pan_x"] / self.view["scale"]
        pan_y = self.view["pan_y"] / self.view["scale"]
        self.view_metrics_var.set(f"Pan {pan_x:.2f}, {pan_y:.2f} | Zoom {self.view['scale']:.1f} px/m")
        self.mark_canvas_dirty()

    def render_canvas_if_needed(self) -> None:
        if not self.canvas_dirty and self.last_render_revision == self.canvas_revision:
            return
        self.render_canvas_contents()
        self.last_render_revision = self.canvas_revision
        self.canvas_dirty = False

    def render_canvas_contents(self) -> None:
        self.canvas.delete("all")
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        self.canvas.create_rectangle(0, 0, width, height, fill="#8f969c", outline="")
        self.draw_grid()
        self.draw_cells()
        self.draw_pending_obstacle()
        self.draw_paths()
        self.draw_pois()
        self.draw_robot()

    def draw_grid(self) -> None:
        spacing = 2.0
        if spacing * self.view["scale"] < 20:
            return
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        left, bottom = self.screen_to_world(0, height)
        right, top = self.screen_to_world(width, 0)
        x = math.floor(left / spacing) * spacing
        while x <= math.ceil(right / spacing) * spacing:
            sx1, sy1 = self.world_to_screen(x, bottom)
            sx2, sy2 = self.world_to_screen(x, top)
            self.canvas.create_line(sx1, sy1, sx2, sy2, fill="#ffffff", stipple="gray50")
            x += spacing
        y = math.floor(bottom / spacing) * spacing
        while y <= math.ceil(top / spacing) * spacing:
            sx1, sy1 = self.world_to_screen(left, y)
            sx2, sy2 = self.world_to_screen(right, y)
            self.canvas.create_line(sx1, sy1, sx2, sy2, fill="#ffffff", stipple="gray50")
            y += spacing

    def draw_cells(self) -> None:
        voxel = self.active_voxel_size()
        size = max(2.0, voxel * self.view["scale"])
        server_grid = getattr(self, "server_grid", {}) or {}
        if self.should_use_server_grid() and isinstance(server_grid.get("data"), list) and int(server_grid.get("width", 0) or 0) > 0:
            resolution = max(0.02, float(server_grid.get("resolution", voxel)))
            origin = server_grid.get("origin") if isinstance(server_grid.get("origin"), dict) else {}
            origin_x = float(origin.get("x", 0.0))
            origin_y = float(origin.get("y", 0.0))
            width = int(server_grid.get("width", 0) or 0)
            for index, value in enumerate(server_grid.get("data", [])):
                if int(value) < 0:
                    continue
                row = index // width
                col = index % width
                x = origin_x + (col + 0.5) * resolution
                y = origin_y + (row + 0.5) * resolution
                sx, sy = self.world_to_screen(x, y)
                fill = "#0c0f12" if int(value) >= 50 else "#ffffff"
                self.canvas.create_rectangle(sx - size / 2, sy - size / 2, sx + size / 2, sy + size / 2, fill=fill, outline="")
            return
        with self.scan_lock:
            free_cells = self.active_free_cells()
            occupied_cells = self.active_occupancy_cells()
        for cell in free_cells:
            sx, sy = self.world_to_screen(float(cell["ix"]) * voxel, float(cell["iy"]) * voxel)
            self.canvas.create_rectangle(sx - size / 2, sy - size / 2, sx + size / 2, sy + size / 2, fill="#ffffff", outline="")
        for cell in occupied_cells:
            sx, sy = self.world_to_screen(float(cell["ix"]) * voxel, float(cell["iy"]) * voxel)
            self.canvas.create_rectangle(sx - size / 2, sy - size / 2, sx + size / 2, sy + size / 2, fill="#0c0f12", outline="")

    def draw_pending_obstacle(self) -> None:
        start = self.edit["pending_obstacle_start"]
        if self.edit["tool"] != "obstacle" or start is None:
            return
        sx, sy = self.world_to_screen(start[0], start[1])
        self.canvas.create_oval(sx - 9, sy - 9, sx + 9, sy + 9, outline="#101214", width=2)
        self.canvas.create_oval(sx - 3, sy - 3, sx + 3, sy + 3, outline="#101214", fill="#101214")

    def draw_paths(self) -> None:
        if not self.show_path_var.get():
            return
        for seg in self.path_segments:
            points = seg.get("points") or [seg["start"], seg["end"]]
            invalid = seg["id"] in self.path_validation["invalid_ids"]
            color = "#cc4b37" if invalid else "#ff7b54" if seg["id"] == self.selected_segment_id else "#f3b441"
            width = 4 if invalid or seg["id"] == self.selected_segment_id else 2
            for index in range(len(points) - 1):
                sx1, sy1 = self.world_to_screen(points[index]["x"], points[index]["y"])
                sx2, sy2 = self.world_to_screen(points[index + 1]["x"], points[index + 1]["y"])
                self.canvas.create_line(sx1, sy1, sx2, sy2, fill=color, width=width)
        if self.pending_free_point is not None:
            sx, sy = self.world_to_screen(self.pending_free_point["x"], self.pending_free_point["y"])
            self.canvas.create_oval(sx - 8, sy - 8, sx + 8, sy + 8, outline="#4fd1c5", dash=(6, 4), width=2)

    def draw_pois(self) -> None:
        if not self.show_poi_var.get():
            return
        for poi in self.poi_nodes:
            sx, sy = self.world_to_screen(poi.x, poi.y)
            fill = "#7c3aed" if poi.client_id in self.selected_poi_ids else "#d94a4a"
            self.canvas.create_oval(sx - 5, sy - 5, sx + 5, sy + 5, fill=fill, outline=fill)
            if poi.client_id in self.selected_poi_ids:
                self.canvas.create_oval(sx - 9, sy - 9, sx + 9, sy + 9, outline="#ffffff", width=2)
            text_id = self.canvas.create_text(sx + 8, sy - 8, text=poi.name, anchor="sw", fill="#182833", font=("Segoe UI", 11, "bold"))
            bbox = self.canvas.bbox(text_id)
            if bbox:
                self.canvas.create_rectangle(bbox[0] - 4, bbox[1] - 2, bbox[2] + 4, bbox[3] + 2, fill="#fff5d6", outline="")
                self.canvas.tag_raise(text_id)

    def draw_robot(self) -> None:
        if not self.show_robot_var.get():
            return
        sx, sy = self.world_to_screen(float(self.pose.get("x", 0.0)), float(self.pose.get("y", 0.0)))
        self.canvas.create_rectangle(sx - 10, sy - 6, sx + 10, sy + 6, fill="#13766e", outline="")
        yaw = -float(self.pose.get("yaw", 0.0))
        self.canvas.create_line(sx, sy, sx + math.cos(yaw) * 14, sy + math.sin(yaw) * 14, fill="#ffffff", width=2)

    def build_pgm_export(self, manifest: dict, points: list[list[float]] | list[tuple[float, float, float]], resolution: float, padding_cells: int = 8) -> dict:
        browser = manifest.get("browser_occupancy") or {}
        occupancy_voxel = max(0.02, float(browser.get("voxel_size", resolution)))
        occupied_cells = browser.get("occupied_cells") if isinstance(browser.get("occupied_cells"), list) else None
        occupied_set: set[tuple[int, int]] = set()
        min_cell_x = math.inf
        max_cell_x = -math.inf
        min_cell_y = math.inf
        max_cell_y = -math.inf
        if occupied_cells:
            for cell in occupied_cells:
                ix = round(float(cell.get("ix", 0)))
                iy = round(float(cell.get("iy", 0)))
                occupied_set.add((ix, iy))
                min_cell_x = min(min_cell_x, ix)
                max_cell_x = max(max_cell_x, ix)
                min_cell_y = min(min_cell_y, iy)
                max_cell_y = max(max_cell_y, iy)
        else:
            if not points:
                raise ValueError("No radar points in SLAM")
            for point in points:
                ix = round(float(point[0]) / resolution)
                iy = round(float(point[1]) / resolution)
                occupied_set.add((ix, iy))
                min_cell_x = min(min_cell_x, ix)
                max_cell_x = max(max_cell_x, ix)
                min_cell_y = min(min_cell_y, iy)
                max_cell_y = max(max_cell_y, iy)
        padded_min_x = int(min_cell_x) - padding_cells
        padded_min_y = int(min_cell_y) - padding_cells
        padded_max_x = int(max_cell_x) + padding_cells
        padded_max_y = int(max_cell_y) + padding_cells
        width = max(1, padded_max_x - padded_min_x + 1)
        height = max(1, padded_max_y - padded_min_y + 1)
        grid = [205] * (width * height)
        for ix, iy in occupied_set:
            x = ix - padded_min_x
            y = iy - padded_min_y
            flipped_y = height - 1 - y
            grid[flipped_y * width + x] = 0
        rows = []
        for row in range(height):
            start = row * width
            rows.append(" ".join(str(grid[start + col]) for col in range(width)))
        origin = [round(padded_min_x * occupancy_voxel, 3), round(padded_min_y * occupancy_voxel, 3), 0]
        return {
            "pgm": f"P2\n# Generated from SLAM occupancy\n{width} {height}\n255\n" + "\n".join(rows) + "\n",
            "origin": origin,
            "width": width,
            "height": height,
            "occupied_cells": len(occupied_set),
        }

    def build_yaml_export(self, file_name: str, resolution: float, origin: list[float]) -> str:
        stem = Path(file_name).with_suffix(".pgm").name
        return "\n".join(
            [
                f"image: {stem}",
                "mode: trinary",
                f"resolution: {resolution:.3f}",
                f"origin: [{origin[0]:.3f}, {origin[1]:.3f}, {int(origin[2])}]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
            ]
        )

    def set_inspector_bundle_state(self, file_name: str, manifest: dict, points: list) -> None:
        config = extract_scan_fusion_config(manifest, default_preset="")
        resolution = float(config["voxel_size"])
        pgm = self.build_pgm_export(manifest, points, resolution)
        yaml_text = self.build_yaml_export(file_name, resolution, pgm["origin"])
        export_manifest = strip_legacy_trajectory(manifest)
        export_manifest.pop("browser_occupancy", None)
        self.inspector = {
            "file": file_name,
            "manifest": manifest,
            "points": points,
            "pgm": pgm["pgm"],
            "yaml": yaml_text,
            "json": json.dumps(export_manifest, ensure_ascii=False, indent=2),
            "meta": pgm,
            "pcd_file": dict(manifest.get("pcd") or {}) if isinstance(manifest.get("pcd"), dict) else None,
        }

    def save_stcm(self) -> None:
        self.rebuild_path_nodes()
        config = self.apply_scan_fusion_config_from_ui()
        voxel_size = self.active_voxel_size()
        notes = {
            "text": self.map_notes_var.get().strip(),
            "voxelSize": voxel_size,
            "loadedFromStcm": self.edit["loaded_from_stcm"],
            "loadedMapName": self.edit["loaded_map_name"] or None,
            "manualCameraSnapshotAt": self.camera_refresh_var.get(),
            "editTool": self.edit["tool"],
        }
        bundle = {
            "version": "slam.v3",
            "scan_mode": str(self.scan.get("mode", "2d")),
            "notes": json.dumps(notes, ensure_ascii=False, indent=2),
            "created_at": time.time(),
            "source": "desktop",
            "map_source": "stcm_editor" if self.edit["loaded_from_stcm"] else "server_occupancy_grid" if self.should_use_server_grid() else "laser_accumulation",
            "scan_fusion": build_scan_fusion_metadata(config),
            "occupancy": self.browser_occupancy(),
            "pose": self.pose,
            "gps": self.gps,
            "chassis": self.chassis,
            "poi": [self.poi_payload(poi) for poi in self.poi_nodes],
            "path": [
                {
                    "id": seg["id"],
                    "source": seg["source"],
                    "clearance": float(seg.get("clearance", 0.0)),
                    "start": dict(seg["start"]),
                    "end": dict(seg["end"]),
                    "points": [dict(point) for point in (seg.get("points") or [])],
                }
                for seg in self.path_segments
            ],
            "gps_track": [],
            "chassis_track": [],
            "scan_summary": {
                "scanActive": self.scan["active"],
                "elapsedSec": round(max(0.0, (int(time.time() * 1000) - self.scan["started_ms"]) / 1000), 1) if self.scan["started_ms"] else 0.0,
                "obstacleCells": len(self.active_occupancy_cells()),
                "safeCells": len(self.active_free_cells()),
                "rawLidarPoints": self.scan["raw_points"],
                "frontFrames": self.scan["front_frames"],
                "rearFrames": self.scan["rear_frames"],
                "voxelSize": voxel_size,
            },
            "radar_points": self.occupied_points(),
        }
        pcd_file = None
        if self.scan.get("pcd_bytes"):
            pcd_name = str(self.scan.get("pcd_name") or "map.pcd")
            pcd_file = {"name": pcd_name, "content": bytes(self.scan.get("pcd_bytes") or b"")}
            bundle["pcd"] = {"included": True, "file": pcd_name}
        else:
            bundle["pcd"] = {"included": False, "file": ""}
        target = filedialog.asksaveasfilename(parent=self.root, defaultextension=".slam", filetypes=[("SLAM", "*.slam")], initialfile=f"{self.map_name_var.get().strip() or 'desktop_map'}.slam")
        if not target:
            return
        manifest = strip_legacy_trajectory({k: v for k, v in bundle.items() if k != "radar_points"})
        write_slam_archive(target, manifest, bundle["radar_points"], pcd_file)
        self.scan["last_saved_file"] = target
        self.scan["saved_point_count"] = len(bundle["radar_points"])
        self.sync_scan_badges()
        self.set_inspector_bundle_state(Path(target).name, manifest, bundle["radar_points"])
        self.logger.info("map saved path=%s points=%s", target, len(bundle["radar_points"]))
        messagebox.showinfo(self.tr("save_title"), self.tr("save_done", path=target))

    def load_stcm(self) -> None:
        target = filedialog.askopenfilename(parent=self.root, filetypes=[("SLAM / ZIP / YAML / PGM", "*.slam *.zip *.yaml *.yml *.pgm"), ("SLAM", "*.slam"), ("ZIP", "*.zip"), ("YAML", "*.yaml *.yml"), ("PGM", "*.pgm")])
        if not target:
            return
        suffix = Path(target).suffix.lower()
        if suffix in {".yaml", ".yml", ".pgm"}:
            loaded = NativeMapImportTool.import_map(target)
            file_name = loaded.file_name
            manifest = loaded.manifest
            points = loaded.radar_points
            pcd_file = None
        else:
            manifest, points, pcd_file = read_slam_archive(target)
            file_name = Path(target).name
        self.apply_stcm(file_name, manifest, points, pcd_file=pcd_file)
        self.set_inspector_bundle_state(file_name, manifest, points)
        self.logger.info("map loaded path=%s points=%s", target, len(points))
        messagebox.showinfo(self.tr("load_title"), self.tr("load_done", name=file_name))

    def apply_stcm(self, file_name: str, manifest: dict, points: list[tuple[float, float, float]], pcd_file: dict[str, Any] | None = None) -> None:
        self.clear_scan()
        self.scan["active"] = False
        self.scan["mode"] = str(manifest.get("scan_mode", "2d")).lower()
        config = extract_scan_fusion_config(manifest, default_preset="")
        self.apply_scan_fusion_config(config, update_vars=True)
        occ = manifest.get("occupancy") or manifest.get("browser_occupancy", {})
        if isinstance(occ, dict) and isinstance(occ.get("occupied_cells"), list):
            self.scan["voxel"] = max(0.02, float(occ.get("voxel_size", float(config["voxel_size"]))))
            for cell in occ.get("occupied_cells", []):
                self.mark_occupied(int(cell.get("ix", 0)), int(cell.get("iy", 0)), float(cell.get("intensity", 1.0)), int(cell.get("hits", 3)))
            for cell in occ.get("free_cells", []):
                self.scan["free"][self.cell_key(int(cell.get("ix", 0)), int(cell.get("iy", 0)))] = {"ix": int(cell.get("ix", 0)), "iy": int(cell.get("iy", 0)), "hits": int(cell.get("hits", 1))}
        else:
            self.scan["voxel"] = float(config["voxel_size"])
            for point in points:
                ix, iy = self.world_to_cell(float(point[0]), float(point[1]))
                self.mark_occupied(ix, iy, float(point[2]), 3)
        self.poi_nodes = []
        self.poi_seed = 1
        for poi in manifest.get("poi", []):
            self.poi_nodes.append(Poi(client_id=f"poi-{self.poi_seed}", name=str(poi.get("name", f"POI {self.poi_seed}")), x=float(poi.get("x", 0.0)), y=float(poi.get("y", 0.0)), yaw=float(poi.get("yaw", 0.0) or 0.0), lat=float(poi["lat"]) if poi.get("lat") not in (None, "") else None, lon=float(poi["lon"]) if poi.get("lon") not in (None, "") else None))
            self.poi_seed += 1
        self.path_segments = []
        self.segment_seed = 1
        for seg in manifest.get("path", []):
            self.path_segments.append(
                {
                    "id": seg.get("id", f"seg-{self.segment_seed}"),
                    "start": self.point_to_payload(self.point_from_dict(seg.get("start", {}))),
                    "end": self.point_to_payload(self.point_from_dict(seg.get("end", {}))),
                    "source": seg.get("source", "stcm"),
                    "geometry": seg.get("geometry", "line"),
                    "curveOffset": float(seg.get("curveOffset", 0.0) or 0.0),
                    "clearance": float(seg.get("clearance", 0.0) or 0.0),
                    "points": [self.point_to_payload(self.point_from_dict(point)) for point in seg.get("points", [])],
                }
            )
            self.segment_seed += 1
        self.edit["loaded_from_stcm"] = True
        self.edit["loaded_map_name"] = file_name
        self.stcm_summary = {"file": file_name, "mapSource": manifest.get("map_source", "unknown"), "radarPoints": len(points), "poiCount": len(self.poi_nodes), "pathCount": len(self.path_segments), "hasBrowserOccupancy": bool(occ), "restoredFreeCells": len(occ.get("free_cells", [])) if isinstance(occ, dict) else 0}
        if isinstance(pcd_file, dict):
            self.scan["pcd_name"] = str(pcd_file.get("name") or "map.pcd")
            self.scan["pcd_bytes"] = bytes(pcd_file.get("content") or b"")
            self.scan["pcd_received_at"] = int(time.time() * 1000)
        if manifest.get("notes"):
            try:
                notes = json.loads(manifest["notes"])
                if isinstance(notes, dict) and notes.get("text"):
                    self.map_notes_var.set(str(notes["text"]))
            except Exception:
                pass
        self.apply_scan_fusion_config(config, update_vars=True)
        self.map_name_var.set(file_name.replace(".slam", ""))
        self.pending_free_point = None
        self.selected_segment_id = None
        self.selected_poi_ids = set()
        self.sync_poi_box()
        self.sync_path_panel()
        self.center_loaded_map()
        self.sync_scan_badges()
        self.map_edit_status_var.set(self.tr("map_loaded_view", name=file_name))

    def export_inspector_file(self, kind: str) -> None:
        if not self.inspector["file"]:
            messagebox.showwarning(self.tr("export_title"), self.tr("export_need_map"))
            return
        mapping = {
            "pgm": (self.inspector["pgm"], ".pgm"),
            "yaml": (self.inspector["yaml"], ".yaml"),
            "json": (self.inspector["json"], ".json"),
        }
        if kind == "pcd":
            pcd_bytes = bytes(self.scan.get("pcd_bytes") or b"")
            if not pcd_bytes:
                messagebox.showwarning(self.tr("export_title"), "pcd_export_unavailable")
                return
            ext = ".pcd"
            initial_name = str(self.scan.get("pcd_name") or Path(self.inspector["file"]).with_suffix(".pcd").name)
            path = filedialog.asksaveasfilename(parent=self.root, defaultextension=ext, filetypes=[("PCD", "*.pcd")], initialfile=initial_name)
            if not path:
                return
            Path(path).write_bytes(pcd_bytes)
            self.logger.info("map export kind=%s path=%s", kind, path)
            messagebox.showinfo(self.tr("export_title"), self.tr("export_done", kind=kind.upper(), path=path))
            return
        content, ext = mapping[kind]
        path = filedialog.asksaveasfilename(parent=self.root, defaultextension=ext, filetypes=[(kind.upper(), f"*{ext}")], initialfile=Path(self.inspector["file"]).with_suffix(ext).name)
        if not path:
            return
        Path(path).write_text(content, encoding="utf-8")
        self.logger.info("map export kind=%s path=%s", kind, path)
        messagebox.showinfo(self.tr("export_title"), self.tr("export_done", kind=kind.upper(), path=path))

    def render_text_panels(self) -> None:
        self.write_text(
            self.scan_text,
            {
                "scan": self.scan_state_var.get(),
                "odom": {key: self.odom.get(key) for key in ("x", "y", "yaw")},
                "last_scan": self.last_scan,
            },
        )
        self.write_text(
            self.comm_text,
            {
                "connection": self.conn_var.get(),
                "status": self.status_var.get(),
                "status_detail": self.status_detail_var.get(),
                "stream_health": self.stream_health,
                "log_file": self.log_path,
                "health": self.health,
                "path_validation": self.path_validation,
                "map_summary": self.stcm_summary,
                "inspector": {
                    "file": self.inspector["file"],
                    "meta": self.inspector["meta"],
                },
                "last_saved_file": self.scan["last_saved_file"],
                "saved_point_count": self.scan["saved_point_count"],
            },
        )

    def write_text(self, widget: tk.Text, payload: dict) -> None:
        text = json.dumps(payload, ensure_ascii=False, indent=2, default=lambda value: list(value) if isinstance(value, set) else str(value))
        widget_id = id(widget)
        if safe_focus_widget(self.root) is widget:
            return
        if self.text_cache.get(widget_id) == text:
            return
        self.text_cache[widget_id] = text
        widget.delete("1.0", tk.END)
        widget.insert(tk.END, text)

    def on_close(self) -> None:
        self.disconnect()
        self.control_sender_stop.set()
        self.control_sender_event.set()
        if hasattr(self, "control_sender_thread") and self.control_sender_thread.is_alive():
            self.control_sender_thread.join(timeout=1.0)
        self.scan_worker_stop.set()
        self.scan_worker_event.set()
        if hasattr(self, "scan_worker") and self.scan_worker.is_alive():
            self.scan_worker.join(timeout=1.0)
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    DesktopClient().run()


if __name__ == "__main__":
    main()
