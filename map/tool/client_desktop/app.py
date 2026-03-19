from __future__ import annotations

import json
import math
import queue
import struct
import threading
import time
import tkinter as tk
import zipfile
from dataclasses import dataclass
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import requests
import websocket


@dataclass
class Poi:
    client_id: str
    name: str
    x: float
    y: float
    yaw: float = 0.0
    lat: float | None = None
    lon: float | None = None


class ServerBridge:
    def __init__(self, ws_url: str) -> None:
        self.ws_url = ws_url.strip()
        self.http_base = self.ws_url.replace("ws://", "http://").replace("wss://", "https://").replace("/ws/stream", "")
        self.queue: queue.Queue[dict] = queue.Queue()
        self.connected = False
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.ws: websocket.WebSocketApp | None = None

    def start(self) -> None:
        if self.thread and self.thread.is_alive():
            return
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self.connected = False
        if self.ws is not None:
            try:
                self.ws.close()
            except Exception:
                pass

    def post(self, path: str, body: dict, retries: int = 3) -> dict:
        last_err = None
        for index in range(retries + 1):
            try:
                res = requests.post(f"{self.http_base}{path}", json=body, timeout=4)
                res.raise_for_status()
                return res.json()
            except Exception as exc:
                last_err = exc
                time.sleep(0.2 * (2**index))
        raise RuntimeError(str(last_err))

    def _loop(self) -> None:
        retry = 0

        def on_message(_ws, msg: str) -> None:
            if msg == "pong":
                return
            try:
                self.queue.put(json.loads(msg))
            except Exception:
                pass

        def on_open(_ws) -> None:
            nonlocal retry
            retry = 0
            self.connected = True

        def on_close(_ws, _code, _msg) -> None:
            self.connected = False

        def on_error(_ws, _err) -> None:
            self.connected = False

        while not self.stop_event.is_set():
            self.ws = websocket.WebSocketApp(
                self.ws_url,
                on_message=on_message,
                on_open=on_open,
                on_close=on_close,
                on_error=on_error,
            )
            self.ws.run_forever(ping_interval=5, ping_timeout=3)
            self.connected = False
            if self.stop_event.is_set():
                break
            retry += 1
            time.sleep(min(10.0, 0.3 * (2 ** min(retry, 5))))


class DesktopClient:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("AutoDrive Desktop Map Tool")
        self.root.geometry("1680x980")
        self.root.minsize(1420, 860)

        self.bridge: ServerBridge | None = None
        self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0, "vx": 0.0, "wz": 0.0}
        self.gps = {"lat": 0.0, "lon": 0.0}
        self.odom = {"x": 0.0, "y": 0.0, "yaw": 0.0, "vx": 0.0, "wz": 0.0}
        self.chassis = {"mode": "-", "battery": 0.0}
        self.pose_history: list[dict] = []
        self.camera_data: dict[int, dict] = {i: {"objects": [], "meta": {}} for i in range(1, 5)}
        self.last_scan = {
            "front": {"raw_points": 0, "keyframe": False, "stamp": 0},
            "rear": {"raw_points": 0, "keyframe": False, "stamp": 0},
        }

        self.poi_nodes: list[Poi] = []
        self.poi_seed = 1
        self.selected_poi_ids: set[str] = set()
        self.pending_poi: dict | None = None

        self.path_segments: list[dict] = []
        self.path_nodes: list[dict] = []
        self.segment_seed = 1
        self.selected_segment_id: str | None = None
        self.pending_free_point: tuple[float, float] | None = None
        self.path_validation = {"checked": False, "ok": None, "invalid_ids": set(), "message": ""}

        self.scan = {
            "active": False,
            "started_ms": 0,
            "voxel": 0.12,
            "front_frames": 0,
            "rear_frames": 0,
            "raw_points": 0,
            "occupied": {},
            "free": {},
            "last_saved_file": "",
            "saved_point_count": 0,
        }

        self.edit = {"tool": "view", "pending_obstacle_start": None, "erasing": False, "loaded_from_stcm": False, "loaded_map_name": ""}
        self.view = {"scale": 25.0, "pan_x": 0.0, "pan_y": 0.0, "dragging": False, "last_xy": (0, 0)}
        self.stcm_summary: dict = {}

        self.server_var = tk.StringVar(value="ws://127.0.0.1:8080/ws/stream")
        self.conn_var = tk.StringVar(value="Disconnected")
        self.status_var = tk.StringVar(value="WS offline")
        self.scan_state_var = tk.StringVar(value="Idle")
        self.keyboard_var = tk.StringVar(value="Keyboard inactive")
        self.map_name_var = tk.StringVar(value="desktop_map")
        self.voxel_var = tk.StringVar(value="0.12")
        self.poi_name_var = tk.StringVar()
        self.poi_geo_var = tk.StringVar()
        self.path_mode_var = tk.StringVar(value="poi")
        self.path_start_var = tk.StringVar()
        self.path_end_var = tk.StringVar()
        self.edit_tool_var = tk.StringVar(value="view")
        self.brush_var = tk.StringVar(value="0.25")
        self.path_status_var = tk.StringVar(value="No path segments yet")
        self.poi_status_var = tk.StringVar(value="POI idle")
        self.map_badge_var = tk.StringVar(value="Scan Session")
        self.tool_badge_var = tk.StringVar(value="Tool: View / Select")
        self.stats_badge_var = tk.StringVar(value="0 obstacle cells")
        self.map_edit_status_var = tk.StringVar(value="View / Select mode active. Load a STCM file to start second-stage map editing.")
        self.view_metrics_var = tk.StringVar(value="Pan 0.00, 0.00 | Zoom 25.0 px/m")
        self.show_path_var = tk.BooleanVar(value=True)
        self.show_poi_var = tk.BooleanVar(value=True)
        self.show_robot_var = tk.BooleanVar(value=True)
        self.forward_var = tk.StringVar(value="0.8")
        self.reverse_var = tk.StringVar(value="0.5")
        self.turn_var = tk.StringVar(value="1.0")
        self.duration_var = tk.StringVar(value="0.15")

        self._style()
        self._ui()
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<MouseWheel>", self.on_mousewheel)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.tick()

    def _style(self) -> None:
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TLabel", font=("Segoe UI", 11))
        style.configure("TButton", font=("Segoe UI", 11))
        style.configure("Header.TLabel", font=("Segoe UI", 24, "bold"))
        style.configure("Muted.TLabel", foreground="#5c6f7a", font=("Segoe UI", 10))

    def _ui(self) -> None:
        shell = ttk.Frame(self.root, padding=12)
        shell.pack(fill=tk.BOTH, expand=True)

        header = ttk.Frame(shell)
        header.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(header, text="AutoDrive Desktop Map Tool", style="Header.TLabel").pack(side=tk.LEFT)
        ttk.Label(header, text="Native desktop rewrite of the browser client", style="Muted.TLabel").pack(side=tk.LEFT, padx=12, pady=8)

        top = ttk.Frame(shell, padding=8)
        top.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(top, text="Server WS").pack(side=tk.LEFT)
        ttk.Entry(top, textvariable=self.server_var, width=42).pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="Connect", command=self.connect).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Disconnect", command=self.disconnect).pack(side=tk.LEFT, padx=4)
        ttk.Label(top, textvariable=self.conn_var).pack(side=tk.LEFT, padx=(14, 4))
        ttk.Label(top, textvariable=self.status_var, style="Muted.TLabel").pack(side=tk.LEFT)

        paned = ttk.Panedwindow(shell, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.BOTH, expand=True)
        left = ttk.Frame(paned)
        center = ttk.Frame(paned)
        right = ttk.Frame(paned)
        paned.add(left, weight=2)
        paned.add(center, weight=5)
        paned.add(right, weight=2)

        self._left(left)
        self._center(center)
        self._right(right)

    def _left(self, parent: ttk.Frame) -> None:
        self.scan_text = self._card_text(parent, "Odom And Scan", 16)
        self._scan_controls(parent)
        self._move_controls(parent)
        self._poi_controls(parent)
        self._path_controls(parent)

    def _scan_controls(self, parent: ttk.Frame) -> None:
        card = ttk.LabelFrame(parent, text="Scan", padding=8)
        card.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(card, textvariable=self.scan_state_var).pack(anchor=tk.W, pady=(0, 6))
        row = ttk.Frame(card)
        row.pack(fill=tk.X, pady=(0, 6))
        ttk.Button(row, text="Start Scan", command=self.start_scan).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(row, text="Stop Scan", command=self.stop_scan).pack(side=tk.LEFT, padx=4)
        ttk.Button(row, text="Clear", command=self.clear_scan).pack(side=tk.LEFT, padx=4)
        self._entry(card, "Map Name", self.map_name_var)
        self._entry(card, "Voxel", self.voxel_var)
        row2 = ttk.Frame(card)
        row2.pack(fill=tk.X, pady=(6, 0))
        ttk.Button(row2, text="Save STCM", command=self.save_stcm).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(row2, text="Load STCM", command=self.load_stcm).pack(side=tk.LEFT, padx=4)

    def _move_controls(self, parent: ttk.Frame) -> None:
        card = ttk.LabelFrame(parent, text="Move", padding=8)
        card.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(card, textvariable=self.keyboard_var).pack(anchor=tk.W, pady=(0, 6))
        self._entry(card, "Forward Speed", self.forward_var)
        self._entry(card, "Reverse Speed", self.reverse_var)
        self._entry(card, "Turn Rate", self.turn_var)
        self._entry(card, "Cmd Duration", self.duration_var)
        row = ttk.Frame(card)
        row.pack(fill=tk.X, pady=(6, 0))
        for name in ("Forward", "Left", "Stop", "Right", "Reverse"):
            ttk.Button(row, text=name, command=lambda n=name: self.move_click(n.lower())).pack(side=tk.LEFT, padx=3)

    def _poi_controls(self, parent: ttk.Frame) -> None:
        card = ttk.LabelFrame(parent, text="POI", padding=8)
        card.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        self._entry(card, "POI Name", self.poi_name_var)
        self._entry(card, "Manual Geo (lon,lat)", self.poi_geo_var)
        ttk.Label(card, textvariable=self.poi_status_var, style="Muted.TLabel", wraplength=320).pack(anchor=tk.W, pady=(0, 6))
        row = ttk.Frame(card)
        row.pack(fill=tk.X, pady=(0, 6))
        ttk.Button(row, text="Add POI", command=self.toggle_add_poi).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(row, text="Delete Selected", command=self.delete_selected_poi).pack(side=tk.LEFT, padx=4)
        ttk.Button(row, text="Apply Geo", command=self.apply_selected_geo).pack(side=tk.LEFT, padx=4)
        self.poi_box = tk.Listbox(card, height=10, bg="#ffffff", fg="#14232d", selectbackground="#0c7c78", font=("Segoe UI", 11))
        self.poi_box.pack(fill=tk.BOTH, expand=True)
        self.poi_box.bind("<<ListboxSelect>>", lambda _e: self.sync_selected_poi())

    def _path_controls(self, parent: ttk.Frame) -> None:
        card = ttk.LabelFrame(parent, text="Path", padding=8)
        card.pack(fill=tk.BOTH, expand=True)
        ttk.Label(card, text="Path supports auto loop, POI-name connect, any-point connect, delete, and closed-loop validation.", style="Muted.TLabel", wraplength=320).pack(anchor=tk.W, pady=(0, 6))
        row = ttk.Frame(card)
        row.pack(fill=tk.X)
        ttk.Label(row, text="Path Tool").pack(side=tk.LEFT)
        mode_box = ttk.Combobox(row, textvariable=self.path_mode_var, state="readonly", width=18, values=["poi", "free"])
        mode_box.pack(side=tk.LEFT, padx=6)
        mode_box.bind("<<ComboboxSelected>>", lambda _e: self.sync_path_panel())
        self._entry(card, "Start POI Name", self.path_start_var)
        self._entry(card, "End POI Name", self.path_end_var)
        ttk.Label(card, textvariable=self.path_status_var, style="Muted.TLabel", wraplength=320).pack(anchor=tk.W, pady=(0, 6))
        row2 = ttk.Frame(card)
        row2.pack(fill=tk.X, pady=(0, 6))
        ttk.Button(row2, text="Auto Loop", command=self.auto_loop).pack(side=tk.LEFT, padx=(0, 4))
        self.connect_named_btn = ttk.Button(row2, text="Connect Named POI", command=self.connect_named_poi)
        self.connect_named_btn.pack(side=tk.LEFT, padx=4)
        ttk.Button(row2, text="Closed-Loop Check", command=lambda: self.validate_path(True)).pack(side=tk.LEFT, padx=4)
        ttk.Button(row2, text="Delete Segment", command=self.delete_selected_segment).pack(side=tk.LEFT, padx=4)
        self.path_box = tk.Listbox(card, height=10, bg="#ffffff", fg="#14232d", selectbackground="#0c7c78", font=("Segoe UI", 11))
        self.path_box.pack(fill=tk.BOTH, expand=True)
        self.path_box.bind("<<ListboxSelect>>", lambda _e: self.sync_selected_segment())
        self.path_start_var.trace_add("write", lambda *_: self.sync_path_panel())
        self.path_end_var.trace_add("write", lambda *_: self.sync_path_panel())

    def _center(self, parent: ttk.Frame) -> None:
        card = ttk.LabelFrame(parent, text="Map View", padding=8)
        card.pack(fill=tk.BOTH, expand=True)
        row = ttk.Frame(card)
        row.pack(fill=tk.X, pady=(0, 6))
        ttk.Button(row, text="Center Robot", command=self.center_robot).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(row, text="Reset View", command=self.reset_view).pack(side=tk.LEFT, padx=4)
        ttk.Checkbutton(row, text="Show Path", variable=self.show_path_var).pack(side=tk.LEFT, padx=8)
        ttk.Checkbutton(row, text="Show POI", variable=self.show_poi_var).pack(side=tk.LEFT, padx=4)
        ttk.Checkbutton(row, text="Show Robot", variable=self.show_robot_var).pack(side=tk.LEFT, padx=4)
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
        map_row = ttk.Frame(card)
        map_row.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(map_row, text="Map").pack(side=tk.LEFT)
        edit_box = ttk.Combobox(map_row, textvariable=self.edit_tool_var, state="readonly", width=18, values=["view", "erase", "obstacle"])
        edit_box.pack(side=tk.LEFT, padx=6)
        edit_box.bind("<<ComboboxSelected>>", lambda _e: self.edit_tool_changed())
        ttk.Label(map_row, text="Brush Radius").pack(side=tk.LEFT, padx=(12, 0))
        ttk.Entry(map_row, textvariable=self.brush_var, width=8).pack(side=tk.LEFT, padx=6)
        ttk.Button(map_row, text="Auto Clear Noise", command=self.auto_clear_noise).pack(side=tk.LEFT, padx=4)
        ttk.Button(map_row, text="Clear Loaded Map", command=self.clear_loaded_map).pack(side=tk.LEFT, padx=4)
        ttk.Label(map_row, textvariable=self.map_edit_status_var, style="Muted.TLabel").pack(side=tk.LEFT, padx=(12, 0))

    def _right(self, parent: ttk.Frame) -> None:
        self.camera_text = self._card_text(parent, "Cameras", 18)
        self.comm_text = self._card_text(parent, "Communication / STCM", 18)

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

    def connect(self) -> None:
        self.disconnect()
        self.bridge = ServerBridge(self.server_var.get().strip())
        self.bridge.start()
        self.conn_var.set("Connecting")

    def disconnect(self) -> None:
        if self.bridge:
            self.bridge.stop()
        self.bridge = None
        self.conn_var.set("Disconnected")
        self.status_var.set("WS offline")

    def call_api(self, path: str, body: dict) -> dict | None:
        if not self.bridge:
            messagebox.showwarning("Disconnected", "Connect to the server first.")
            return None
        try:
            return self.bridge.post(path, body)
        except Exception as exc:
            messagebox.showerror("API Error", str(exc))
            return None

    def tick(self) -> None:
        self.consume_messages()
        self.render_canvas()
        self.render_text_panels()
        self.conn_var.set("Connected" if self.bridge and self.bridge.connected else "Disconnected")
        self.status_var.set(f"WS {'ok' if self.bridge and self.bridge.connected else 'offline'} | scan {'on' if self.scan['active'] else 'off'} | poi {len(self.poi_nodes)} | path {len(self.path_segments)}")
        self.root.after(60, self.tick)

    def consume_messages(self) -> None:
        if not self.bridge:
            return
        while True:
            try:
                msg = self.bridge.queue.get_nowait()
            except queue.Empty:
                break
            topic = msg.get("topic")
            payload = msg.get("payload", {})
            if topic == "/robot/pose":
                self.pose = payload
            elif topic == "/robot/gps":
                self.gps = payload
            elif topic == "/chassis/odom":
                self.odom = payload
                self.pose_history.append({"stamp": float(msg.get("stamp", time.time())), "pose": dict(payload)})
                self.pose_history = self.pose_history[-240:]
            elif topic == "/chassis/status":
                self.chassis = payload
            elif topic == "/lidar/front":
                self.scan["front_frames"] += 1
                self.last_scan["front"] = {"raw_points": int(payload.get("raw_points", len(payload.get("points", [])))), "keyframe": bool(payload.get("keyframe")), "stamp": float(msg.get("stamp", 0))}
                self.accumulate_points(payload.get("points", []), float(msg.get("stamp", 0)), bool(payload.get("keyframe")))
            elif topic == "/lidar/rear":
                self.scan["rear_frames"] += 1
                self.last_scan["rear"] = {"raw_points": int(payload.get("raw_points", len(payload.get("points", [])))), "keyframe": bool(payload.get("keyframe")), "stamp": float(msg.get("stamp", 0))}
                self.accumulate_points(payload.get("points", []), float(msg.get("stamp", 0)), bool(payload.get("keyframe")))
            elif topic and topic.startswith("/camera/"):
                cam_id = int(topic.split("/")[2])
                self.camera_data[cam_id] = {"objects": payload.get("objects", []), "meta": {"seq": msg.get("seq"), "stamp": msg.get("stamp")}}

    def number(self, var: tk.StringVar, fallback: float) -> float:
        try:
            return float(var.get().strip())
        except Exception:
            return fallback

    def start_scan(self) -> None:
        if self.call_api("/scan/start", {}) is not None:
            self.clear_scan()
            self.edit["loaded_from_stcm"] = False
            self.edit["loaded_map_name"] = ""
            self.scan["active"] = True
            self.scan["started_ms"] = int(time.time() * 1000)
            self.sync_scan_badges()

    def stop_scan(self) -> None:
        if self.call_api("/scan/stop", {}) is not None:
            self.scan["active"] = False
            self.sync_scan_badges()

    def clear_scan(self) -> None:
        self.scan["occupied"] = {}
        self.scan["free"] = {}
        self.scan["front_frames"] = 0
        self.scan["rear_frames"] = 0
        self.scan["raw_points"] = 0
        self.scan["saved_point_count"] = 0
        self.scan["last_saved_file"] = ""
        self.sync_scan_badges()

    def sync_scan_badges(self) -> None:
        occ = len(self.scan["occupied"])
        free = len(self.scan["free"])
        if self.scan["active"]:
            self.scan_state_var.set(f"Recording {occ} obstacle cells")
        elif occ or free:
            self.scan_state_var.set(f"Stopped | {occ} obs / {free} safe")
        else:
            self.scan_state_var.set("Idle")
        self.map_badge_var.set(f"Loaded: {self.edit['loaded_map_name']}" if self.edit["loaded_from_stcm"] else "Scan Session")
        tool_map = {"view": "View / Select", "erase": "Erase Noise", "obstacle": "Draw Obstacle"}
        suffix = " | Pick end point" if self.edit["tool"] == "obstacle" and self.edit["pending_obstacle_start"] else ""
        self.tool_badge_var.set(f"Tool: {tool_map.get(self.edit['tool'], 'View / Select')}{suffix}")
        self.stats_badge_var.set(f"{occ} obstacle cells | {len(self.poi_nodes)} POI | {len(self.path_segments)} paths")

    def cell_key(self, ix: int, iy: int) -> str:
        return f"{ix}:{iy}"

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        voxel = float(self.scan["voxel"])
        return round(x / voxel), round(y / voxel)

    def mark_free(self, ix: int, iy: int) -> None:
        key = self.cell_key(ix, iy)
        slot = self.scan["free"].get(key, {"ix": ix, "iy": iy, "hits": 0})
        slot["hits"] += 1
        self.scan["free"][key] = slot

    def mark_occupied(self, ix: int, iy: int, intensity: float, hits: int = 1) -> None:
        key = self.cell_key(ix, iy)
        slot = self.scan["occupied"].get(key, {"ix": ix, "iy": iy, "hits": 0, "intensity": 0.0})
        slot["hits"] = max(int(slot["hits"]) + hits, hits)
        slot["intensity"] = max(float(slot["intensity"]), float(intensity))
        self.scan["occupied"][key] = slot

    def raytrace(self, start_x: int, start_y: int, end_x: int, end_y: int) -> None:
        dx = end_x - start_x
        dy = end_y - start_y
        steps = max(abs(dx), abs(dy))
        if steps <= 1:
            return
        for step in range(steps):
            t = step / steps
            self.mark_free(round(start_x + dx * t), round(start_y + dy * t))

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
        if not self.scan["active"] or not points:
            return
        self.scan["voxel"] = max(0.02, self.number(self.voxel_var, 0.12))
        pose = self.pose_for_stamp(stamp)
        if not keyframe and abs(float(pose.get("wz", 0.0))) >= 0.35:
            return
        self.scan["raw_points"] += len(points)
        robot_ix, robot_iy = self.world_to_cell(float(pose.get("x", 0.0)), float(pose.get("y", 0.0)))
        for point in points:
            x, y = float(point[0]), float(point[1])
            intensity = float(point[2]) if len(point) > 2 else 1.0
            ix, iy = self.world_to_cell(x, y)
            self.raytrace(robot_ix, robot_iy, ix, iy)
            self.mark_occupied(ix, iy, intensity)
        self.sync_scan_badges()

    def occupied_points(self) -> list[list[float]]:
        points = []
        for cell in self.scan["occupied"].values():
            free = self.scan["free"].get(self.cell_key(int(cell["ix"]), int(cell["iy"])))
            free_hits = int(free["hits"]) if free else 0
            if int(cell["hits"]) < 3 or int(cell["hits"]) < free_hits * 0.9:
                continue
            points.append([float(cell["ix"]) * float(self.scan["voxel"]), float(cell["iy"]) * float(self.scan["voxel"]), float(cell["intensity"])])
        return points or [[0.0, 0.0, 1.0]]

    def browser_occupancy(self) -> dict:
        return {
            "voxel_size": float(self.scan["voxel"]),
            "occupied_cells": [{"ix": int(c["ix"]), "iy": int(c["iy"]), "hits": int(c["hits"]), "intensity": float(c["intensity"])} for c in self.scan["occupied"].values()],
            "free_cells": [{"ix": int(c["ix"]), "iy": int(c["iy"]), "hits": int(c["hits"])} for c in self.scan["free"].values()],
        }

    def save_stcm(self) -> None:
        self.rebuild_path_nodes()
        bundle = {
            "version": "stcm.v2",
            "notes": json.dumps({"text": "desktop client", "voxelSize": float(self.scan["voxel"]), "loadedFromStcm": self.edit["loaded_from_stcm"], "loadedMapName": self.edit["loaded_map_name"] or None}, ensure_ascii=False, indent=2),
            "created_at": time.time(),
            "source": "desktop",
            "map_source": "stcm_editor" if self.edit["loaded_from_stcm"] else "laser_accumulation",
            "browser_occupancy": self.browser_occupancy(),
            "pose": self.pose,
            "gps": self.gps,
            "chassis": self.chassis,
            "poi": [self.poi_payload(poi) for poi in self.poi_nodes],
            "path": self.path_nodes,
            "trajectory": [{"id": seg["id"], "source": seg["source"], "geometry": "line", "curveOffset": 0.0, "start": self.path_node(seg["start"]), "end": self.path_node(seg["end"])} for seg in self.path_segments],
            "gps_track": [],
            "chassis_track": [],
            "scan_summary": {"scanActive": self.scan["active"], "obstacleCells": len(self.scan["occupied"]), "safeCells": len(self.scan["free"]), "rawLidarPoints": self.scan["raw_points"], "frontFrames": self.scan["front_frames"], "rearFrames": self.scan["rear_frames"], "voxelSize": float(self.scan["voxel"])},
            "radar_points": self.occupied_points(),
        }
        target = filedialog.asksaveasfilename(parent=self.root, defaultextension=".stcm", filetypes=[("STCM", "*.stcm")], initialfile=f"{self.map_name_var.get().strip() or 'desktop_map'}.stcm")
        if not target:
            return
        manifest = {k: v for k, v in bundle.items() if k != "radar_points"}
        with zipfile.ZipFile(target, "w", compression=zipfile.ZIP_DEFLATED) as zf:
            zf.writestr("manifest.json", json.dumps(manifest, ensure_ascii=False, indent=2))
            zf.writestr("radar_points.bin", b"".join(struct.pack("fff", *point) for point in bundle["radar_points"]))
        self.scan["last_saved_file"] = target
        self.scan["saved_point_count"] = len(bundle["radar_points"])
        self.sync_scan_badges()
        messagebox.showinfo("Saved", f"Map saved:\n{target}")

    def load_stcm(self) -> None:
        target = filedialog.askopenfilename(parent=self.root, filetypes=[("STCM", "*.stcm"), ("ZIP", "*.zip")])
        if not target:
            return
        with zipfile.ZipFile(target, "r") as zf:
            manifest = json.loads(zf.read("manifest.json"))
            raw = zf.read("radar_points.bin")
        points = [struct.unpack("fff", raw[i:i + 12]) for i in range(0, len(raw), 12) if i + 12 <= len(raw)]
        self.apply_stcm(Path(target).name, manifest, points)
        messagebox.showinfo("Loaded", f"Loaded map:\n{Path(target).name}")

    def apply_stcm(self, file_name: str, manifest: dict, points: list[tuple[float, float, float]]) -> None:
        self.clear_scan()
        self.scan["active"] = False
        occ = manifest.get("browser_occupancy", {})
        if isinstance(occ, dict) and isinstance(occ.get("occupied_cells"), list):
            self.scan["voxel"] = max(0.02, float(occ.get("voxel_size", self.number(self.voxel_var, 0.12))))
            for cell in occ.get("occupied_cells", []):
                self.mark_occupied(int(cell.get("ix", 0)), int(cell.get("iy", 0)), float(cell.get("intensity", 1.0)), int(cell.get("hits", 3)))
            for cell in occ.get("free_cells", []):
                self.scan["free"][self.cell_key(int(cell.get("ix", 0)), int(cell.get("iy", 0)))] = {"ix": int(cell.get("ix", 0)), "iy": int(cell.get("iy", 0)), "hits": int(cell.get("hits", 1))}
        else:
            self.scan["voxel"] = max(0.02, self.number(self.voxel_var, 0.12))
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
        for seg in manifest.get("trajectory", []):
            self.path_segments.append({"id": seg.get("id", f"seg-{self.segment_seed}"), "start": self.make_point(seg.get("start", {})), "end": self.make_point(seg.get("end", {})), "source": seg.get("source", "stcm")})
            self.segment_seed += 1
        self.edit["loaded_from_stcm"] = True
        self.edit["loaded_map_name"] = file_name
        self.stcm_summary = {"file": file_name, "mapSource": manifest.get("map_source", "unknown"), "radarPoints": len(points), "poiCount": len(self.poi_nodes), "pathCount": len(self.path_segments), "hasBrowserOccupancy": bool(occ), "restoredFreeCells": len(occ.get("free_cells", [])) if isinstance(occ, dict) else 0}
        self.voxel_var.set(f"{float(self.scan['voxel']):.2f}")
        self.map_name_var.set(file_name.replace(".stcm", ""))
        self.pending_free_point = None
        self.selected_segment_id = None
        self.selected_poi_ids = set()
        self.sync_poi_box()
        self.sync_path_panel()
        self.center_loaded_map()
        self.sync_scan_badges()
        self.map_edit_status_var.set(f"Loaded {file_name} into main map view")

    def make_point(self, payload: dict) -> dict:
        return {"x": float(payload.get("x", 0.0)), "y": float(payload.get("y", 0.0)), "lat": payload.get("lat"), "lon": payload.get("lon"), "poi_id": payload.get("poi_id"), "name": payload.get("name", "")}

    def poi_payload(self, poi: Poi) -> dict:
        return {"name": poi.name, "x": float(poi.x), "y": float(poi.y), "yaw": float(poi.yaw), "lat": float(poi.lat) if poi.lat is not None else None, "lon": float(poi.lon) if poi.lon is not None else None}

    def path_node(self, node: dict) -> dict:
        return {"x": float(node["x"]), "y": float(node["y"]), "lat": float(node["lat"]) if node.get("lat") is not None else None, "lon": float(node["lon"]) if node.get("lon") is not None else None}

    def toggle_add_poi(self) -> None:
        if self.pending_poi is not None:
            self.pending_poi = None
            self.poi_status_var.set("POI idle")
            return
        name = self.poi_name_var.get().strip()
        if not name:
            messagebox.showwarning("POI", "Input POI name first.")
            return
        lat, lon = self.parse_geo(self.poi_geo_var.get().strip())
        if self.poi_geo_var.get().strip() and lat is None:
            messagebox.showwarning("POI", "Geo format must be lon,lat.")
            return
        self.pending_poi = {"name": name, "lat": lat, "lon": lon}
        self.poi_status_var.set(f'Ready to place "{name}" on canvas')

    def parse_geo(self, text: str) -> tuple[float | None, float | None]:
        if not text:
            return None, None
        parts = [p.strip() for p in text.split(",")]
        if len(parts) != 2:
            return None, None
        try:
            lon = float(parts[0])
            lat = float(parts[1])
            return lat, lon
        except Exception:
            return None, None

    def place_poi(self, x: float, y: float) -> None:
        if self.pending_poi is None:
            return
        poi = Poi(client_id=f"poi-{self.poi_seed}", name=self.pending_poi["name"], x=x, y=y, yaw=float(self.pose.get("yaw", 0.0)), lat=self.pending_poi["lat"], lon=self.pending_poi["lon"])
        self.poi_seed += 1
        self.poi_nodes.append(poi)
        if self.bridge and self.bridge.connected:
            self.call_api("/map/poi", {"poi": self.poi_payload(poi)})
        self.pending_poi = None
        self.poi_name_var.set("")
        self.poi_status_var.set("POI idle")
        self.sync_poi_box()

    def sync_selected_poi(self) -> None:
        self.selected_poi_ids = set()
        for idx in self.poi_box.curselection():
            if 0 <= idx < len(self.poi_nodes):
                self.selected_poi_ids.add(self.poi_nodes[idx].client_id)
        self.sync_scan_badges()

    def sync_poi_box(self) -> None:
        self.poi_box.delete(0, tk.END)
        for poi in self.poi_nodes:
            self.poi_box.insert(tk.END, f"{poi.name} ({poi.x:.2f}, {poi.y:.2f}) lat={poi.lat if poi.lat is not None else 'n/a'} lon={poi.lon if poi.lon is not None else 'n/a'}")
        self.sync_scan_badges()

    def delete_selected_poi(self) -> None:
        if not self.selected_poi_ids:
            messagebox.showwarning("POI", "Select POI to delete.")
            return
        self.poi_nodes = [poi for poi in self.poi_nodes if poi.client_id not in self.selected_poi_ids]
        self.path_segments = [seg for seg in self.path_segments if seg["start"].get("poi_id") not in self.selected_poi_ids and seg["end"].get("poi_id") not in self.selected_poi_ids]
        self.selected_poi_ids = set()
        self.selected_segment_id = None
        self.sync_poi_box()
        self.sync_path_panel()

    def apply_selected_geo(self) -> None:
        if not self.selected_poi_ids:
            messagebox.showwarning("POI", "Select POI first.")
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

    def segment_point(self, item: dict | Poi) -> dict:
        if isinstance(item, Poi):
            return {"x": float(item.x), "y": float(item.y), "lat": item.lat, "lon": item.lon, "poi_id": item.client_id, "name": item.name}
        return {"x": float(item.get("x", 0.0)), "y": float(item.get("y", 0.0)), "lat": item.get("lat"), "lon": item.get("lon"), "poi_id": item.get("poi_id"), "name": item.get("name", "")}

    def create_segment(self, start: dict | Poi, end: dict | Poi, source: str) -> dict:
        seg = {"id": f"seg-{self.segment_seed}", "start": self.segment_point(start), "end": self.segment_point(end), "source": source}
        self.segment_seed += 1
        return seg

    def add_segment(self, seg: dict) -> None:
        self.path_segments.append(seg)
        self.selected_segment_id = seg["id"]
        self.sync_path_panel()

    def rebuild_path_nodes(self) -> None:
        self.path_nodes = []
        for seg in self.path_segments:
            self.path_nodes.extend([self.path_node(seg["start"]), self.path_node(seg["end"])])

    def find_poi_name(self, name: str) -> Poi | None:
        if not name:
            messagebox.showwarning("Path", "Input both POI names first.")
            return None
        matches = [poi for poi in self.poi_nodes if poi.name.strip().lower() == name.lower()]
        if not matches:
            messagebox.showwarning("Path", f'POI "{name}" not found.')
            return None
        if len(matches) > 1:
            messagebox.showwarning("Path", f'POI name "{name}" is duplicated.')
            return None
        return matches[0]

    def connect_named_poi(self) -> None:
        a = self.find_poi_name(self.path_start_var.get().strip())
        b = self.find_poi_name(self.path_end_var.get().strip())
        if a is None or b is None:
            return
        if a.client_id == b.client_id:
            messagebox.showwarning("Path", "Start and end POI cannot be the same.")
            return
        self.add_segment(self.create_segment(a, b, "poi-name"))

    def auto_loop(self) -> None:
        if len(self.poi_nodes) < 2:
            messagebox.showwarning("Path", "At least two POI are required.")
            return
        ordered = sorted(self.poi_nodes, key=lambda poi: (poi.x, poi.y))
        self.path_segments = [seg for seg in self.path_segments if seg["source"] != "auto"]
        route = [ordered[0]]
        remaining = ordered[1:]
        while remaining:
            last = route[-1]
            idx = min(range(len(remaining)), key=lambda i: math.hypot(last.x - remaining[i].x, last.y - remaining[i].y))
            route.append(remaining.pop(idx))
        for i in range(len(route) - 1):
            self.path_segments.append(self.create_segment(route[i], route[i + 1], "auto"))
        if len(route) > 2:
            self.path_segments.append(self.create_segment(route[-1], route[0], "auto"))
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

    def path_key(self, node: dict) -> str:
        return f"{float(node['x']):.3f},{float(node['y']):.3f}"

    def validate_path(self, show_alert: bool) -> bool:
        self.path_validation = {"checked": True, "ok": None, "invalid_ids": set(), "message": ""}
        if len(self.path_segments) < 3:
            self.path_validation["ok"] = False
            self.path_validation["invalid_ids"] = {seg["id"] for seg in self.path_segments}
            self.path_validation["message"] = "Closed-loop check failed: at least 3 path segments are required."
        else:
            endpoint_map, adjacency = {}, {}
            for seg in self.path_segments:
                a, b = self.path_key(seg["start"]), self.path_key(seg["end"])
                endpoint_map.setdefault(a, []).append(seg["id"])
                endpoint_map.setdefault(b, []).append(seg["id"])
                adjacency.setdefault(a, set()).add(b)
                adjacency.setdefault(b, set()).add(a)
            invalid = set()
            bad_nodes = 0
            for ids in endpoint_map.values():
                if len(ids) != 2:
                    bad_nodes += 1
                    invalid.update(ids)
            visited, components = set(), 0
            for node in list(adjacency.keys()):
                if node in visited:
                    continue
                components += 1
                stack = [node]
                while stack:
                    cur = stack.pop()
                    if cur in visited:
                        continue
                    visited.add(cur)
                    stack.extend(n for n in adjacency.get(cur, set()) if n not in visited)
            if components != 1:
                invalid.update(seg["id"] for seg in self.path_segments)
            self.path_validation["ok"] = not invalid
            self.path_validation["invalid_ids"] = invalid
            if self.path_validation["ok"]:
                self.path_validation["message"] = "Closed-loop check passed."
            else:
                parts = []
                if bad_nodes:
                    parts.append(f"{bad_nodes} endpoint(s) do not have degree 2")
                if components != 1:
                    parts.append(f"path is split into {components} disconnected component(s)")
                self.path_validation["message"] = f"Closed-loop check failed: {'; '.join(parts)}."
        self.sync_path_panel()
        if show_alert:
            messagebox.showinfo("Path Validation", self.path_validation["message"])
        return bool(self.path_validation["ok"])

    def sync_path_panel(self) -> None:
        self.rebuild_path_nodes()
        self.path_box.delete(0, tk.END)
        for seg in self.path_segments:
            suffix = " | closed-loop error" if seg["id"] in self.path_validation["invalid_ids"] else ""
            dist = math.hypot(seg["start"]["x"] - seg["end"]["x"], seg["start"]["y"] - seg["end"]["y"])
            self.path_box.insert(tk.END, f"{seg['source']} | ({seg['start']['x']:.2f}, {seg['start']['y']:.2f}) -> ({seg['end']['x']:.2f}, {seg['end']['y']:.2f}) | {dist:.2f} m{suffix}")
        tool = f"Input POI names to connect ({self.path_start_var.get().strip() or '?'} -> {self.path_end_var.get().strip() or '?'})" if self.path_mode_var.get() == "poi" else "Click any two points to connect"
        pending = f" | Start ({self.pending_free_point[0]:.2f}, {self.pending_free_point[1]:.2f})" if self.pending_free_point else ""
        validation = " | Loop unchecked" if not self.path_validation["checked"] else " | Loop OK" if self.path_validation["ok"] else f" | Loop error {len(self.path_validation['invalid_ids'])} segment(s)"
        self.path_status_var.set(f"Path segments {len(self.path_segments)} | Nodes {len(self.path_nodes)} | Tool {tool}{pending}{validation}")
        if self.path_mode_var.get() == "poi" and self.path_start_var.get().strip() and self.path_end_var.get().strip():
            self.connect_named_btn.state(["!disabled"])
        else:
            self.connect_named_btn.state(["disabled"])
        self.sync_scan_badges()

    def move_click(self, name: str) -> None:
        fwd = self.number(self.forward_var, 0.8)
        rev = self.number(self.reverse_var, 0.5)
        turn = self.number(self.turn_var, 1.0)
        dur = self.number(self.duration_var, 0.15)
        if name == "stop":
            self.call_api("/control/stop", {})
            return
        if name == "forward":
            body = {"velocity": fwd, "yaw_rate": 0.0, "duration": dur}
        elif name == "reverse":
            body = {"velocity": -rev, "yaw_rate": 0.0, "duration": dur}
        elif name == "left":
            body = {"velocity": max(fwd * 0.5, 0.2), "yaw_rate": turn, "duration": dur}
        else:
            body = {"velocity": max(fwd * 0.5, 0.2), "yaw_rate": -turn, "duration": dur}
        self.call_api("/control/move", body)

    def on_key_press(self, event: tk.Event) -> None:
        key = event.keysym.lower()
        if key == "w":
            self.move_click("forward")
            self.keyboard_var.set("W forward")
        elif key == "s":
            self.move_click("reverse")
            self.keyboard_var.set("S reverse")
        elif key == "a":
            self.move_click("left")
            self.keyboard_var.set("A left")
        elif key == "d":
            self.move_click("right")
            self.keyboard_var.set("D right")
        elif key == "space":
            self.move_click("stop")
            self.keyboard_var.set("Space stop")

    def edit_tool_changed(self) -> None:
        self.edit["tool"] = self.edit_tool_var.get()
        self.edit["pending_obstacle_start"] = None
        if self.edit["tool"] == "erase":
            self.map_edit_status_var.set(f"Erase Noise mode active. Brush radius {self.number(self.brush_var, 0.25):.2f} m")
        elif self.edit["tool"] == "obstacle":
            self.map_edit_status_var.set("Draw Obstacle Line mode active. Click two points on the map.")
        else:
            self.map_edit_status_var.set("View / Select mode active. You can pan, zoom, and select POI or path.")
        self.sync_scan_badges()

    def canvas_press(self, event: tk.Event) -> None:
        self.view["last_xy"] = (event.x, event.y)
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
                self.map_edit_status_var.set(f"Obstacle start fixed at ({x:.2f}, {y:.2f}). Click end point next.")
            else:
                self.draw_obstacle_line(self.edit["pending_obstacle_start"], (x, y))
                self.edit["pending_obstacle_start"] = None
                self.map_edit_status_var.set("Added obstacle line.")
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
        x, y = self.screen_to_world(event.x, event.y)
        if self.path_mode_var.get() == "free":
            if self.pending_free_point is None:
                self.pending_free_point = (x, y)
            else:
                self.add_segment(self.create_segment({"x": self.pending_free_point[0], "y": self.pending_free_point[1], "lat": self.gps.get("lat"), "lon": self.gps.get("lon")}, {"x": x, "y": y, "lat": self.gps.get("lat"), "lon": self.gps.get("lon")}, "free"))
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
        self.map_edit_status_var.set(f"Auto cleared {len(removable)} noisy cells" if removable else "No isolated noise found")
        self.sync_scan_badges()

    def clear_loaded_map(self) -> None:
        self.clear_scan()
        self.poi_nodes = []
        self.path_segments = []
        self.path_nodes = []
        self.selected_poi_ids = set()
        self.selected_segment_id = None
        self.pending_free_point = None
        self.pending_poi = None
        self.edit["loaded_from_stcm"] = False
        self.edit["loaded_map_name"] = ""
        self.path_validation = {"checked": False, "ok": None, "invalid_ids": set(), "message": ""}
        self.sync_poi_box()
        self.sync_path_panel()
        self.map_edit_status_var.set("Loaded map cleared")

    def world_to_screen(self, x: float, y: float) -> tuple[float, float]:
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        return width / 2 + self.view["pan_x"] + x * self.view["scale"], height / 2 + self.view["pan_y"] - y * self.view["scale"]

    def screen_to_world(self, x: float, y: float) -> tuple[float, float]:
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        return (x - width / 2 - self.view["pan_x"]) / self.view["scale"], (height / 2 + self.view["pan_y"] - y) / self.view["scale"]

    def on_mousewheel(self, event: tk.Event) -> None:
        if event.widget is not self.canvas:
            return
        x, y = self.screen_to_world(event.x, event.y)
        self.view["scale"] = max(8.0, min(80.0, self.view["scale"] * (1.08 if event.delta > 0 else 0.92)))
        sx, sy = self.world_to_screen(x, y)
        self.view["pan_x"] += event.x - sx
        self.view["pan_y"] += event.y - sy
        self.update_view_metrics()

    def center_robot(self) -> None:
        self.view["pan_x"] = -float(self.pose.get("x", 0.0)) * self.view["scale"]
        self.view["pan_y"] = float(self.pose.get("y", 0.0)) * self.view["scale"]
        self.update_view_metrics()

    def center_loaded_map(self) -> None:
        cells = list(self.scan["occupied"].values())
        if not cells:
            self.center_robot()
            return
        xs = [float(cell["ix"]) * float(self.scan["voxel"]) for cell in cells]
        ys = [float(cell["iy"]) * float(self.scan["voxel"]) for cell in cells]
        self.view["pan_x"] = -((min(xs) + max(xs)) / 2) * self.view["scale"]
        self.view["pan_y"] = ((min(ys) + max(ys)) / 2) * self.view["scale"]
        self.update_view_metrics()

    def reset_view(self) -> None:
        self.view["scale"] = 25.0
        self.center_robot()

    def update_view_metrics(self) -> None:
        pan_x = -self.view["pan_x"] / self.view["scale"]
        pan_y = self.view["pan_y"] / self.view["scale"]
        self.view_metrics_var.set(f"Pan {pan_x:.2f}, {pan_y:.2f} | Zoom {self.view['scale']:.1f} px/m")

    def render_canvas(self) -> None:
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
        size = max(2.0, float(self.scan["voxel"]) * self.view["scale"])
        for cell in self.scan["free"].values():
            occ = self.scan["occupied"].get(self.cell_key(int(cell["ix"]), int(cell["iy"])))
            occ_hits = int(occ["hits"]) if occ else 0
            if int(cell["hits"]) <= occ_hits * 0.8:
                continue
            sx, sy = self.world_to_screen(float(cell["ix"]) * float(self.scan["voxel"]), float(cell["iy"]) * float(self.scan["voxel"]))
            self.canvas.create_rectangle(sx - size / 2, sy - size / 2, sx + size / 2, sy + size / 2, fill="#ffffff", outline="")
        for cell in self.scan["occupied"].values():
            free = self.scan["free"].get(self.cell_key(int(cell["ix"]), int(cell["iy"])))
            free_hits = int(free["hits"]) if free else 0
            if int(cell["hits"]) < 3 or int(cell["hits"]) < free_hits * 0.9:
                continue
            sx, sy = self.world_to_screen(float(cell["ix"]) * float(self.scan["voxel"]), float(cell["iy"]) * float(self.scan["voxel"]))
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
            sx1, sy1 = self.world_to_screen(seg["start"]["x"], seg["start"]["y"])
            sx2, sy2 = self.world_to_screen(seg["end"]["x"], seg["end"]["y"])
            invalid = seg["id"] in self.path_validation["invalid_ids"]
            color = "#cc4b37" if invalid else "#ff7b54" if seg["id"] == self.selected_segment_id else "#f3b441"
            width = 4 if invalid or seg["id"] == self.selected_segment_id else 2
            self.canvas.create_line(sx1, sy1, sx2, sy2, fill=color, width=width)
        if self.pending_free_point is not None:
            sx, sy = self.world_to_screen(self.pending_free_point[0], self.pending_free_point[1])
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

    def render_text_panels(self) -> None:
        self.write_text(self.scan_text, {"scan": self.scan_state_var.get(), "pose": self.pose, "odom": self.odom, "gps": self.gps, "last_scan": self.last_scan})
        self.write_text(self.camera_text, {f"camera_{idx}": self.camera_data[idx] for idx in range(1, 5)})
        self.write_text(self.comm_text, {"connection": self.conn_var.get(), "status": self.status_var.get(), "path_validation": self.path_validation, "stcm_summary": self.stcm_summary, "last_saved_file": self.scan["last_saved_file"], "saved_point_count": self.scan["saved_point_count"]})

    def write_text(self, widget: tk.Text, payload: dict) -> None:
        widget.delete("1.0", tk.END)
        widget.insert(tk.END, json.dumps(payload, ensure_ascii=False, indent=2, default=lambda value: list(value) if isinstance(value, set) else str(value)))

    def on_close(self) -> None:
        self.disconnect()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    DesktopClient().run()


if __name__ == "__main__":
    main()
