# AutoDrive 扫图工具（map/tool）

你提出的端侧划分已经按“**server 在机器人，client 在 Windows 且可打包 EXE，主要功能在 client**”重新整理：

- `server/`：机器人侧轻量服务，负责接收控制、topic 汇聚与转发、地图文件保存/加载。
- `client_desktop/`：Windows 主客户端（可 PyInstaller 打包为独立 exe），承载主要交互与可视化功能。
- `client/`：浏览器版调试客户端（可选），用于快速联调。

---

## 1. 完整代码结构

```text
map/tool/
├── README.md
├── client/                    # 可选：浏览器调试端
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
