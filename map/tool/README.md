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
  - 提供 `/health`、`/diag/stream_stats`、扫描控制、扫描模式切换、运动控制、POI 下发、地图保存/加载、WebSocket topic 流
- `client/`
  - 浏览器版调试端，作为桌面端功能对齐的基准实现
- `client_desktop/`
  - 当前主客户端，可直接在桌面环境运行，并支持 PyInstaller 打包

## 桌面端能力

`client_desktop` 当前已覆盖这些主要能力：

- WebSocket 连接、自动重连、ping/pong、消息校验、序列 gap 统计、时延统计、`/health` 轮询
- `local | cloud` 双认证，统一归一到后端 `host + port + token`
- `2d | 3d` 扫图模式切换
- 雷达扫图累计、主画布显示、机器人姿态/轨迹显示
- `PointCloud2` 3D 预览显示和有界保存
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

## 认证模式

浏览器端和桌面端都支持两种认证方式：

- `local`
  - 向目标设备的登录接口发送固定用户名和密码
  - 从响应里解析 `backend_host`、`backend_port`、`token`
- `cloud`
  - 向云端认证接口发送用户名和密码
  - 从响应里解析 `backend_host`、`backend_port`、`token`

两种方式后续都会统一成一个后端连接描述符，再去连接机器人服务。

默认接口路径：

- local: `/login`
- cloud: `/api/auth/login`

桌面端固定凭据可通过环境变量覆盖：

- `AUTODRIVE_LOCAL_AUTH_USERNAME`
- `AUTODRIVE_LOCAL_AUTH_PASSWORD`

注意：

- 当前仓库里没有真实认证服务契约样例
- 如果现场接口字段不是 `host/ip/port/token` 这一类常见字段，需要再对接实际返回格式

## 2D / 3D 扫图

系统现在支持两套互斥扫图流程：

- `2d`
  - 数据源是 `LaserScan`
  - 主地图显示为黑白占据栅格
  - `.slam` 中保存 `radar_points.bin`
- `3d`
  - 数据源是 `sensor_msgs/PointCloud2`
  - 前端显示的是降采样后的 3D 预览点
  - `.slam` 中保存 `point_cloud.bin`

运行时可通过客户端切换 `Scan Mode`，服务端会同步切到对应模式。

3D 当前版本的约束：

- 只做预览和保存，不做重型 3D 编辑
- 3D 画布目前是只读预览
- 点云会先做体素降采样和点数上限控制，避免前端卡顿和保存文件过大

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
  - `scan_mode`
  - `poi`
  - `path`
  - `trajectory`
  - `gps_track`
  - `chassis_track`
  - `pose`
  - `notes`
  - `browser_occupancy`
- `radar_points.bin`
  - 2D 模式下保存 `(x, y, intensity)` 的 `float32` 三元组
- `point_cloud.bin`
  - 3D 模式下保存 `(x, y, z, intensity)` 的 `float32` 四元组

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

## Scan Fusion Presets

扫描融合支持场景预设和参数覆盖，主要用于适配仿真和真实环境里不同的障碍物密度。

预设包括：

- `sim_clean`
- `indoor_balanced`
- `indoor_sensitive`
- `warehouse_sparse`

核心参数包括：

- `voxel_size`
- `occupied_min_hits`
- `occupied_over_free_ratio`
- `turn_skip_wz`
- `skip_turn_frames`

建议：

- 仿真默认用 `sim_clean`
- 实车默认用 `indoor_balanced`
- 如果桌椅、细腿、边缘障碍容易丢失，切到 `indoor_sensitive`

详细的场景选择、参数说明和调参指南见：

- `docs/product_manual.md` 的“5.2 到 5.7”

## 地图清理行为

噪点清理和橡皮擦清理现在都会把被移除的障碍格转成 `free cell`。

这意味着：

- 清理后的区域会显示为白色
- 不再是“直接抠掉后露出底色”
- 更符合“这里是已确认可通行区域”的编辑语义

## 验证

桌面端共享逻辑测试：

```bash
python3 -m pytest tests/test_client_desktop_logic.py -v
```
