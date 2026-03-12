# AutoDrive 扫图工具（map/tool）

你提出的端侧划分已经按“**server 在机器人，client 在 Windows 且可打包 EXE，主要功能在 client**”重新整理：

- `server/`：机器人侧轻量服务，负责接收控制、topic 汇聚与转发、地图文件保存/加载。
- `client_desktop/`：Windows 主客户端（可 PyInstaller 打包为独立 exe），承载主要交互与可视化功能。
- `client/`：浏览器版调试客户端（可选），用于快速联调。

---

## 1. 完整代码结构
本目录提供一套可运行的 **Client + Server** 完整实现，针对“机器人连 WiFi 扫图”场景，支持：

- 2 路雷达 topic（模拟大疆 16 线半固态：`/lidar/front`, `/lidar/rear`）
- 4 路相机 topic（`/camera/1..4/compressed`）
- **底盘 topic**：`/chassis/odom`、`/chassis/status`
- 单个 `.stcm` 文件内整合：雷达点云 + POI + 路径轨迹 + GPS + 底盘轨迹
- 路径打点与下发、近场键盘控制（WASD/方向键）
- 类 RViz 风格可视化（点云、机器人姿态、轨迹、路径、POI、目标簇）
- ROS2 可用时预留接入（不可用自动降级为全量仿真 topic）

## 完整代码结构

```text
map/tool/
├── README.md
├── client/                    # 可选：浏览器调试端
├── client/
│   ├── index.html
│   ├── main.js
│   ├── style.css
│   └── run_client.py
├── client_desktop/            # 主客户端（Windows EXE）
│   ├── app.py
│   ├── run_client.py
│   ├── requirements.txt
│   ├── build_windows.bat
│   └── README.md
└── server/                    # 机器人侧
    ├── requirements.txt
    ├── run_server.py
    └── app/
        ├── config.py
        ├── models.py
        ├── main.py
        ├── simulator.py
        ├── topic_bus.py
        ├── stcm_codec.py
        └── ros_bridge.py
```

---

## 2. 你关心的功能实现核对

### 2.1 轨迹两种方式
在 `client_desktop` 已实现：

1) **点击两个 POI 自动连线**：左侧 POI 列表可多选 2 个，点击“使用选中2个POI连线”。
2) **点击任意两点连线**：切到“任意两点连线”模式，画布点击两次自动生成线段。

### 2.2 POI 经纬度配置
在 `client_desktop` 的“添加POI(含经纬度)”流程中，输入 `lat/lon` 后会：
- 本地保存在 POI 列表；
- 调用 server `/map/poi` 下发；
- 后续可进 STCM 单文件归档。

### 2.3 底盘 topic
server 仿真与转发已含：
- `/chassis/odom`
- `/chassis/status`

并在 client 展示底盘状态。

---

## 3. 启动方式

### 3.1 启动机器人 server
└── server/
    ├── requirements.txt
    ├── run_server.py
    └── app/
        ├── __init__.py
        ├── config.py          # 参数配置（队列、频率、时钟阈值）
        ├── models.py          # API 数据模型
        ├── main.py            # API + WS + 诊断 + 打包校验
        ├── simulator.py       # 雷达/相机/底盘/GPS 仿真 topic
        ├── topic_bus.py       # 异步 topic 总线 + 丢帧统计
        ├── stcm_codec.py      # stcm 单文件编解码
        └── ros_bridge.py      # ROS2 可用性探测
```

## 通信稳定性与排障增强

- **WS 自动重连**：客户端断线后指数退避重连（含抖动）
- **HTTP 重试**：关键控制/保存请求自动重试
- **链路保活**：WebSocket ping/pong
- **完整性校验**：每条 topic 消息携带 `checksum(SHA256)`，客户端二次校验
- **时间正确性检测**：`server_time_ms` 与客户端时钟差监控（默认 ±5s 阈值）
- **序列连续性检测**：每 topic `seq` 连续增长，发现 gap 自动计数
- **服务端统计**：`/diag/stream_stats` 可查看发布数、丢帧数、订阅者数、序列号
- **客户端通信面板**：展示重连次数、校验错误、时间异常、丢包估计、HTTP 重试次数

## 启动

### 1) 启动 server（机器人侧 / 仿真侧）

```bash
cd map/tool/server
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python run_server.py
```

### 3.2 启动 Windows 主客户端

```bash
cd map/tool/client_desktop
pip install -r requirements.txt
python run_client.py
```

### 3.3 Windows 打包 EXE

运行：`map/tool/client_desktop/build_windows.bat`

---

## 4. STCM 单文件

STCM 仍保持单文件：
- `manifest.json`：`poi/path/trajectory/gps_track/chassis_track/pose/notes`
- `radar_points.bin`：雷达点云二进制

---

## 5. 通信稳定性与排障

- WS 自动重连（指数退避）
- HTTP 重试
- ping/pong 保活
- checksum/seq/time 校验
- `/diag/stream_stats` 诊断接口
默认监听 `0.0.0.0:8080`。

### 2) 启动 client（PC 侧）

```bash
cd map/tool/client
python run_client.py
```

浏览器打开 `http://<PC_IP>:8090`，WebSocket 指向机器人 IP（如 `ws://192.168.1.23:8080/ws/stream`）。

## 键盘控制

- `W/S` 或 `↑/↓`：前后
- `A/D` 或 `←/→`：转向
- `Space`：急停

## STCM v2（单文件）

- `manifest.json`: `poi`, `path`, `trajectory`, `gps_track`, `chassis_track`, `pose`, `notes`
- `radar_points.bin`: `(x,y,intensity)` float32 三元组

> 注意：外部仍是单个 `.stcm` 文件，满足归档和传输便利性。

## 接口摘要

- `GET /health`：状态检查（含 ROS 可用性）
- `GET /diag/stream_stats`：链路诊断统计
- `POST /scan/start|stop`：启停扫图
- `POST /control/move|stop`：运动控制
- `POST /path/plan`：路径打点下发（可带经纬度）
- `POST /map/poi`：新增 POI
- `POST /map/save|load`、`GET /map/list`：地图管理
- `WS /ws/stream`：topic 流（姿态、GPS、底盘、雷达、相机、地图，含 seq/time/checksum）
