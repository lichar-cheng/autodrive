from __future__ import annotations

import json
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

    def post(self, path: str, body: dict, retries: int = 3) -> dict:
        last_err = None
        for i in range(retries + 1):
            try:
                r = requests.post(f"{self.base_http}{path}", json=body, timeout=3)
                r.raise_for_status()
                return r.json()
            except Exception as e:  # noqa: BLE001
                last_err = e
                time.sleep(0.2 * (2**i))
        raise RuntimeError(f"POST failed: {path}: {last_err}")

    def connect_ws(self) -> None:
        def _on_message(_ws, msg: str) -> None:
            if msg == "pong":
                return
            if self.on_message:
                self.on_message(json.loads(msg))

        def _on_open(_ws) -> None:
            self.connected = True

        def _on_close(_ws, _code, _msg) -> None:
            self.connected = False

        while True:
            self.ws = websocket.WebSocketApp(
                self.ws_url,
                on_message=_on_message,
                on_open=_on_open,
                on_close=_on_close,
            )
            self.ws.run_forever(ping_interval=5, ping_timeout=2)
            time.sleep(1.0)


class MappingClientUI:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("AutoDrive Desktop Client")
        self.root.geometry("1400x850")

        self.poi_list: list[Poi] = []
        self.path_nodes: list[tuple[float, float]] = []
        self.lines: list[tuple[tuple[float, float], tuple[float, float], str]] = []
        self.pending_free_point: tuple[float, float] | None = None
        self.mode = tk.StringVar(value="free")

        self.bridge: ServerBridge | None = None
        self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.chassis = {}
        self.front = []
        self.rear = []

        self._build_ui()

    def _build_ui(self) -> None:
        top = ttk.Frame(self.root)
        top.pack(fill=tk.X, padx=8, pady=8)

        ttk.Label(top, text="Server WS").pack(side=tk.LEFT)
        self.ws_entry = ttk.Entry(top, width=40)
        self.ws_entry.insert(0, "ws://127.0.0.1:8080/ws/stream")
        self.ws_entry.pack(side=tk.LEFT, padx=4)

        ttk.Button(top, text="连接", command=self.connect).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="开始扫图", command=lambda: self.call_api("/scan/start", {})).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="停止扫图", command=lambda: self.call_api("/scan/stop", {})).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="保存地图", command=self.save_map).pack(side=tk.LEFT, padx=4)

        mode_frame = ttk.LabelFrame(top, text="轨迹方式")
        mode_frame.pack(side=tk.LEFT, padx=8)
        ttk.Radiobutton(mode_frame, text="任意两点连线", variable=self.mode, value="free").pack(side=tk.LEFT)
        ttk.Radiobutton(mode_frame, text="两POI自动连线", variable=self.mode, value="poi").pack(side=tk.LEFT)

        body = ttk.Frame(self.root)
        body.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(body)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=8)
        self.status_text = tk.Text(left, width=40, height=14)
        self.status_text.pack(fill=tk.X)
        ttk.Button(left, text="添加POI(含经纬度)", command=self.add_poi_dialog).pack(fill=tk.X, pady=6)

        self.poi_box = tk.Listbox(left, width=40, height=18, selectmode=tk.MULTIPLE)
        self.poi_box.pack(fill=tk.BOTH, expand=True)
        ttk.Button(left, text="使用选中2个POI连线", command=self.link_selected_poi).pack(fill=tk.X, pady=6)
        ttk.Button(left, text="下发路径到Server", command=self.send_path).pack(fill=tk.X, pady=6)

        self.canvas = tk.Canvas(body, bg="#0a0d14", width=960, height=720)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.canvas.bind("<Button-1>", self.on_canvas_click)

        self.root.bind("<KeyPress>", self.on_key_press)

        self._tick()

    def connect(self) -> None:
        ws = self.ws_entry.get().strip()
        http = ws.replace("ws://", "http://").replace("/ws/stream", "")
        self.bridge = ServerBridge(base_http=http, ws_url=ws)
        self.bridge.on_message = self.handle_topic
        threading.Thread(target=self.bridge.connect_ws, daemon=True).start()

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
        self.draw_points(self.front, "#00d1ff")
        self.draw_points(self.rear, "#ffae00")
        self.draw_robot()
        self.draw_lines()
        self.draw_poi()

        self.status_text.delete("1.0", tk.END)
        self.status_text.insert(
            tk.END,
            json.dumps(
                {
                    "connected": self.bridge.connected if self.bridge else False,
                    "pose": self.pose,
                    "chassis": self.chassis,
                    "poi_count": len(self.poi_list),
                    "line_count": len(self.lines),
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
        for p in points[:2000]:
            sx, sy = self.world_to_screen(float(p[0]), float(p[1]))
            self.canvas.create_rectangle(sx, sy, sx + 1, sy + 1, outline=color)

    def draw_robot(self) -> None:
        x, y = self.world_to_screen(self.pose.get("x", 0.0), self.pose.get("y", 0.0))
        self.canvas.create_rectangle(x - 7, y - 5, x + 7, y + 5, outline="#ff4d6d")

    def draw_lines(self) -> None:
        for p1, p2, mode in self.lines:
            x1, y1 = self.world_to_screen(*p1)
            x2, y2 = self.world_to_screen(*p2)
            color = "#ffd166" if mode == "free" else "#90ee90"
            self.canvas.create_line(x1, y1, x2, y2, fill=color, width=2)

    def draw_poi(self) -> None:
        for poi in self.poi_list:
            x, y = self.world_to_screen(poi.x, poi.y)
            self.canvas.create_oval(x - 4, y - 4, x + 4, y + 4, outline="#ff6b6b")

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    MappingClientUI().run()
