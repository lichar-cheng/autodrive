from __future__ import annotations

import json
import random
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, simpledialog, ttk

import requests
import websocket


@dataclass
class Poi:
    name: str
    x: float
    y: float
    lat: float
    lon: float


class ServerBridge:
    def __init__(self, base_http: str, ws_url: str) -> None:
        self.base_http = base_http.rstrip("/")
        self.ws_url = ws_url
        self.ws: websocket.WebSocketApp | None = None
        self.connected = False
        self.on_message = None

        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

    def post(self, path: str, body: dict, retries: int = 3) -> dict:
        last_err = None
        for i in range(retries + 1):
            try:
                response = requests.post(f"{self.base_http}{path}", json=body, timeout=3)
                response.raise_for_status()
                return response.json()
            except Exception as e:  # noqa: BLE001
                last_err = e
                time.sleep(0.2 * (2**i))
        raise RuntimeError(f"POST failed: {path}: {last_err}")

    def start(self) -> None:
        with self._lock:
            if self._thread and self._thread.is_alive():
                return
            self._stop_event.clear()
            self._thread = threading.Thread(target=self.connect_ws, daemon=True)
            self._thread.start()

    def stop(self) -> None:
        with self._lock:
            self._stop_event.set()
            self.connected = False
            if self.ws is not None:
                try:
                    self.ws.close()
                except Exception:  # noqa: BLE001
                    pass

    def connect_ws(self) -> None:
        retry = 0

        def _on_message(_ws, msg: str) -> None:
            if msg == "pong":
                return
            if self.on_message:
                self.on_message(json.loads(msg))

        def _on_open(_ws) -> None:
            nonlocal retry
            retry = 0
            self.connected = True

        def _on_close(_ws, _code, _msg) -> None:
            self.connected = False

        def _on_error(_ws, _err) -> None:
            self.connected = False

        while not self._stop_event.is_set():
            self.ws = websocket.WebSocketApp(
                self.ws_url,
                on_message=_on_message,
                on_open=_on_open,
                on_close=_on_close,
                on_error=_on_error,
            )
            self.ws.run_forever(ping_interval=5, ping_timeout=3)
            if self._stop_event.is_set():
                break

            retry += 1
            backoff = min(10.0, (2 ** min(retry, 6)) * 0.2 + random.uniform(0.0, 0.3))
            time.sleep(backoff)


class MappingClientUI:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("AutoDrive Desktop Client")
        self.root.geometry("1480x900")
        self.root.configure(bg="#0f172a")

        self.poi_list: list[Poi] = []
        self.path_nodes: list[tuple[float, float]] = []
        self.lines: list[tuple[tuple[float, float], tuple[float, float], str]] = []
        self.pending_free_point: tuple[float, float] | None = None
        self.mode = tk.StringVar(value="free")
        self.conn_state = tk.StringVar(value="未连接")

        self.bridge: ServerBridge | None = None
        self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.chassis = {}
        self.front: list = []
        self.rear: list = []

        self._configure_style()
        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _configure_style(self) -> None:
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("Root.TFrame", background="#0f172a")
        style.configure("Card.TLabelframe", background="#111827", foreground="#e5e7eb", borderwidth=1)
        style.configure("Card.TLabelframe.Label", background="#111827", foreground="#93c5fd")
        style.configure("Card.TFrame", background="#111827")
        style.configure("TLabel", background="#111827", foreground="#e5e7eb")
        style.configure("Header.TLabel", font=("Segoe UI", 14, "bold"), foreground="#dbeafe", background="#0f172a")
        style.configure("Sub.TLabel", foreground="#93c5fd", background="#0f172a")
        style.configure("Primary.TButton", padding=6)

    def _build_ui(self) -> None:
        root_frame = ttk.Frame(self.root, style="Root.TFrame", padding=12)
        root_frame.pack(fill=tk.BOTH, expand=True)

        header = ttk.Frame(root_frame, style="Root.TFrame")
        header.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(header, text="AutoDrive 桌面扫图客户端", style="Header.TLabel").pack(side=tk.LEFT)
        ttk.Label(header, text="参考浏览器版信息面板重构", style="Sub.TLabel").pack(side=tk.LEFT, padx=10)

        toolbar = ttk.Frame(root_frame, style="Card.TFrame", padding=10)
        toolbar.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(toolbar, text="Server WS").pack(side=tk.LEFT)
        self.ws_entry = ttk.Entry(toolbar, width=42)
        self.ws_entry.insert(0, "ws://127.0.0.1:8080/ws/stream")
        self.ws_entry.pack(side=tk.LEFT, padx=6)

        ttk.Button(toolbar, text="连接", command=self.connect, style="Primary.TButton").pack(side=tk.LEFT, padx=4)
        ttk.Button(toolbar, text="开始扫图", command=lambda: self.call_api("/scan/start", {})).pack(side=tk.LEFT, padx=4)
        ttk.Button(toolbar, text="停止扫图", command=lambda: self.call_api("/scan/stop", {})).pack(side=tk.LEFT, padx=4)
        ttk.Button(toolbar, text="保存地图", command=self.save_map).pack(side=tk.LEFT, padx=4)

        ttk.Label(toolbar, text="连接状态:").pack(side=tk.LEFT, padx=(14, 4))
        self.conn_label = ttk.Label(toolbar, textvariable=self.conn_state)
        self.conn_label.pack(side=tk.LEFT)

        mode_frame = ttk.LabelFrame(toolbar, text="轨迹方式", style="Card.TLabelframe", padding=6)
        mode_frame.pack(side=tk.RIGHT)
        ttk.Radiobutton(mode_frame, text="任意两点连线", variable=self.mode, value="free").pack(side=tk.LEFT)
        ttk.Radiobutton(mode_frame, text="两POI自动连线", variable=self.mode, value="poi").pack(side=tk.LEFT, padx=6)

        content = ttk.Panedwindow(root_frame, orient=tk.HORIZONTAL)
        content.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(content, style="Card.TFrame", padding=8)
        right = ttk.Frame(content, style="Card.TFrame", padding=8)
        content.add(left, weight=1)
        content.add(right, weight=4)

        status_box = ttk.LabelFrame(left, text="实时状态", style="Card.TLabelframe", padding=6)
        status_box.pack(fill=tk.BOTH, expand=False)
        self.status_text = tk.Text(
            status_box,
            width=40,
            height=14,
            bg="#0b1220",
            fg="#dbeafe",
            insertbackground="#93c5fd",
            relief=tk.FLAT,
            font=("Consolas", 10),
        )
        self.status_text.pack(fill=tk.BOTH, expand=True)

        poi_box_frame = ttk.LabelFrame(left, text="POI与路径", style="Card.TLabelframe", padding=6)
        poi_box_frame.pack(fill=tk.BOTH, expand=True, pady=(8, 0))

        ttk.Button(poi_box_frame, text="添加POI(含经纬度)", command=self.add_poi_dialog).pack(fill=tk.X, pady=(0, 6))
        self.poi_box = tk.Listbox(
            poi_box_frame,
            width=40,
            height=16,
            selectmode=tk.MULTIPLE,
            bg="#0b1220",
            fg="#e5e7eb",
            selectbackground="#2563eb",
            relief=tk.FLAT,
        )
        self.poi_box.pack(fill=tk.BOTH, expand=True)
        ttk.Button(poi_box_frame, text="使用选中2个POI连线", command=self.link_selected_poi).pack(fill=tk.X, pady=6)
        ttk.Button(poi_box_frame, text="下发路径到Server", command=self.send_path).pack(fill=tk.X)

        self.canvas = tk.Canvas(right, bg="#0a0d14", width=960, height=720, highlightthickness=1, highlightbackground="#1f2937")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Button-1>", self.on_canvas_click)

        self.root.bind("<KeyPress>", self.on_key_press)
        self._tick()

    def connect(self) -> None:
        ws = self.ws_entry.get().strip()
        http = ws.replace("ws://", "http://").replace("/ws/stream", "")

        if self.bridge is not None:
            self.bridge.stop()

        self.bridge = ServerBridge(base_http=http, ws_url=ws)
        self.bridge.on_message = self.handle_topic
        self.bridge.start()

    def handle_topic(self, msg: dict) -> None:
        topic = msg.get("topic")
        payload = msg.get("payload", {})
        if topic == "/robot/pose":
            self.pose = payload
        elif topic == "/chassis/status":
            self.chassis = payload
        elif topic == "/lidar/front":
            self.front = payload.get("points", [])
        elif topic == "/lidar/rear":
            self.rear = payload.get("points", [])

    def call_api(self, path: str, body: dict) -> None:
        if not self.bridge:
            return
        try:
            self.bridge.post(path, body)
        except Exception as e:  # noqa: BLE001
            messagebox.showerror("API错误", str(e))

    def save_map(self) -> None:
        self.call_api("/map/save", {"name": "desktop", "notes": "windows client"})

    def add_poi_dialog(self) -> None:
        name = simpledialog.askstring("POI", "POI名称", parent=self.root)
        if not name:
            return
        x = simpledialog.askfloat("POI", "x(米)", parent=self.root)
        y = simpledialog.askfloat("POI", "y(米)", parent=self.root)
        lat = simpledialog.askfloat("POI", "lat", parent=self.root)
        lon = simpledialog.askfloat("POI", "lon", parent=self.root)
        if None in (x, y, lat, lon):
            return
        poi = Poi(name=name, x=x, y=y, lat=lat, lon=lon)
        self.poi_list.append(poi)
        self.poi_box.insert(tk.END, f"{poi.name}: ({poi.x:.2f},{poi.y:.2f}) lat={poi.lat:.6f} lon={poi.lon:.6f}")
        self.call_api("/map/poi", {"poi": poi.__dict__})

    def link_selected_poi(self) -> None:
        sel = self.poi_box.curselection()
        if len(sel) != 2:
            messagebox.showwarning("提示", "请选择2个POI")
            return
        p1, p2 = self.poi_list[sel[0]], self.poi_list[sel[1]]
        self.lines.append(((p1.x, p1.y), (p2.x, p2.y), "poi"))
        self.path_nodes.extend([(p1.x, p1.y), (p2.x, p2.y)])

    def on_canvas_click(self, event) -> None:  # noqa: ANN001
        x, y = self.screen_to_world(event.x, event.y)
        if self.mode.get() == "free":
            if self.pending_free_point is None:
                self.pending_free_point = (x, y)
            else:
                p1 = self.pending_free_point
                p2 = (x, y)
                self.lines.append((p1, p2, "free"))
                self.path_nodes.extend([p1, p2])
                self.pending_free_point = None

    def send_path(self) -> None:
        if not self.path_nodes:
            return
        nodes = [{"x": p[0], "y": p[1], "lat": None, "lon": None} for p in self.path_nodes]
        self.call_api("/path/plan", {"nodes": nodes})

    def on_key_press(self, event) -> None:  # noqa: ANN001
        if event.keysym in ("space", "Escape"):
            self.call_api("/control/stop", {})
            return
        key = event.keysym.lower()
        if key == "w":
            self.call_api("/control/move", {"velocity": 0.8, "yaw_rate": 0.0, "duration": 0.2})
        elif key == "s":
            self.call_api("/control/move", {"velocity": 0.4, "yaw_rate": 3.14, "duration": 0.2})
        elif key == "a":
            self.call_api("/control/move", {"velocity": 0.4, "yaw_rate": 1.0, "duration": 0.2})
        elif key == "d":
            self.call_api("/control/move", {"velocity": 0.4, "yaw_rate": -1.0, "duration": 0.2})

    def world_to_screen(self, x: float, y: float) -> tuple[float, float]:
        return 480 + x * 25, 360 - y * 25

    def screen_to_world(self, x: float, y: float) -> tuple[float, float]:
        return (x - 480) / 25, (360 - y) / 25

    def _tick(self) -> None:
        self.canvas.delete("all")
        self.draw_grid()
        self.draw_points(self.front, "#22d3ee")
        self.draw_points(self.rear, "#f59e0b")
        self.draw_robot()
        self.draw_lines()
        self.draw_poi()

        connected = self.bridge.connected if self.bridge else False
        self.conn_state.set("已连接" if connected else "未连接")

        self.status_text.delete("1.0", tk.END)
        self.status_text.insert(
            tk.END,
            json.dumps(
                {
                    "connected": connected,
                    "pose": self.pose,
                    "chassis": self.chassis,
                    "poi_count": len(self.poi_list),
                    "line_count": len(self.lines),
                    "front_points": len(self.front),
                    "rear_points": len(self.rear),
                },
                ensure_ascii=False,
                indent=2,
            ),
        )
        self.root.after(80, self._tick)

    def draw_grid(self) -> None:
        for i in range(0, 960, 50):
            self.canvas.create_line(i, 0, i, 720, fill="#18202c")
        for j in range(0, 720, 50):
            self.canvas.create_line(0, j, 960, j, fill="#18202c")

    def draw_points(self, points: list, color: str) -> None:
        for point in points[:2000]:
            sx, sy = self.world_to_screen(float(point[0]), float(point[1]))
            self.canvas.create_rectangle(sx, sy, sx + 1, sy + 1, outline=color)

    def draw_robot(self) -> None:
        x, y = self.world_to_screen(self.pose.get("x", 0.0), self.pose.get("y", 0.0))
        self.canvas.create_rectangle(x - 7, y - 5, x + 7, y + 5, outline="#f43f5e")

    def draw_lines(self) -> None:
        for p1, p2, mode in self.lines:
            x1, y1 = self.world_to_screen(*p1)
            x2, y2 = self.world_to_screen(*p2)
            color = "#facc15" if mode == "free" else "#86efac"
            self.canvas.create_line(x1, y1, x2, y2, fill=color, width=2)

    def draw_poi(self) -> None:
        for poi in self.poi_list:
            x, y = self.world_to_screen(poi.x, poi.y)
            self.canvas.create_oval(x - 4, y - 4, x + 4, y + 4, outline="#fb7185")

    def _on_close(self) -> None:
        if self.bridge is not None:
            self.bridge.stop()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    MappingClientUI().run()
