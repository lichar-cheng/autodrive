# AutoDrive 扫图工具

`map/tool` 提供机器人侧服务、浏览器调试端和桌面主客户端。

## 目录

```text
map/tool/
├── README.md
├── client/                 # 浏览器调试端
│   ├── index.html
│   ├── main.js
│   ├── run_client.py
│   ├── style.css
│   └── trajectory_service.py
├── client_desktop/         # Tk 桌面主客户端
│   ├── app.py
│   ├── logic.py
│   ├── run_client.py
│   ├── requirements.txt
│   ├── build_windows.bat
│   ├── build_linux.sh
│   ├── build_macos.sh
│   └── README.md
├── server/                 # 机器人侧服务
│   ├── requirements.txt
│   ├── run_server.py
│   └── app/
│       ├── config.py
│       ├── main.py
│       ├── models.py
│       ├── ros_bridge.py
│       ├── simulator.py
│       ├── stcm_codec.py
│       └── topic_bus.py
└── tests/
    └── test_client_desktop_logic.py
```

## 当前职责

- `server/`
  - 提供 `/health`、`/diag/stream_stats`、扫描控制、运动控制、POI 下发、地图保存/加载、WebSocket topic 流
- `client/`
  - 浏览器版调试端，作为桌面端功能对齐的基准实现
- `client_desktop/`
  - 当前主客户端，可直接在桌面环境运行，并支持 PyInstaller 打包

## 桌面端能力

`client_desktop` 当前已覆盖这些主要能力：

- WebSocket 连接、自动重连、ping/pong、消息校验、序列 gap 统计、时延统计、`/health` 轮询
- 雷达扫图累计、主画布显示、机器人姿态/轨迹显示
- 本地 `.slam` 地图保存和加载
- 地图导出 `PGM`、`YAML`、`JSON`
- 二阶段地图编辑
- 噪点擦除、障碍线绘制
- POI 单个添加、批量添加、删除、经纬度应用、剪贴板复制
- 按 POI 名称连线
- 任意两点连线
- 带安全距离的避障路径规划
- 自动闭环和闭环校验
- 键盘运动控制、重复发指令、松键停
- 相机缓冲快照刷新
- 面板显隐控制

## 启动

### 1. 启动 server

```bash
cd server
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
python3 run_server.py
```

### 2. 启动浏览器调试端

```bash
cd client
python3 run_client.py
```

打开 `http://<PC_IP>:8090`。

### 3. 启动桌面端

```bash
cd client_desktop
python3 -m pip install -r requirements.txt
python3 run_client.py
```

## 打包

### Windows

```bat
cd client_desktop
build_windows.bat
```

产物位于 `client_desktop/dist/AutoDriveClient.exe`。

### Linux

```bash
cd client_desktop
chmod +x build_linux.sh
./build_linux.sh
```

产物位于 `client_desktop/dist/AutoDriveClient`。

### macOS

```bash
cd client_desktop
chmod +x build_macos.sh
./build_macos.sh
```

产物位于 `client_desktop/dist/AutoDriveClient.app`。

该脚本需要在 macOS 上执行。

## Map File

保存格式使用单个 `.slam` 文件，包含：

- `manifest.json`
  - `poi`
  - `path`
  - `trajectory`
  - `gps_track`
  - `chassis_track`
  - `pose`
  - `notes`
  - `browser_occupancy`
- `radar_points.bin`
  - `(x, y, intensity)` 的 `float32` 三元组

`.slam` 还可以导出为：

- `.pgm`
- `.yaml`
- `.json`

独立导出工具位于：

- `tools/slam_export/java`
- `tools/slam_export/cpp`

两套工具都支持：

- 内存接口，直接返回 `.pgm` / `.yaml` / `.json` 文本
- 文件接口，直接把 3 个导出文件写到目标目录

## 验证

桌面端共享逻辑测试：

```bash
python3 -m pytest tests/test_client_desktop_logic.py -v
```
