# AutoDrive Server README

## 1. 服务端作用概览

`tool/server` 是地图采集服务端，职责主要有四类：

1. 对外提供 HTTP 和 WebSocket 接口。
2. 接 ROS2 真实数据，或在 ROS 不可用时切到内置模拟器。
3. 聚合位姿、底盘、雷达、相机、地图数据，并通过统一主题总线分发。
4. 保存/加载 `.stcm` 地图文件，给客户端提供扫描、路径、POI、地图管理能力。

默认监听地址：

- `http://0.0.0.0:8080`
- `ws://0.0.0.0:8080/ws/stream`

服务入口见 [run_server.py](C:\Users\Dell\Documents\my_test\autodrive\map\tool\server\run_server.py)。

## 2. 目录结构

```text
tool/server/
├─ run_server.py              # Uvicorn 启动入口
├─ requirements.txt           # 运行依赖
└─ app/
   ├─ main.py                 # FastAPI 路由、WS 推流、扫描会话、地图管理主逻辑
   ├─ config.py               # 服务端配置、ROS topic 配置
   ├─ models.py               # HTTP 请求模型定义
   ├─ topic_bus.py            # 异步 topic 总线，负责发布/订阅和丢帧统计
   ├─ simulator.py            # ROS 不可用时的模拟数据源
   ├─ ros_bridge.py           # ROS2 桥接，订阅真实 topic 并转成本服务内部 topic
   └─ stcm_codec.py           # .stcm 单文件存取
```

## 3. 核心模块说明

### 3.1 `run_server.py`

- 只负责读取 `CONFIG.host`、`CONFIG.port` 启动 Uvicorn。
- 实际业务入口在 `app.main:app`。

### 3.2 `app/main.py`

- 初始化 `FastAPI`、`TopicBus`、`Simulator`。
- 启动时自动检测 ROS2。
- 若 ROS2 可用，启用 `RosBridge`。
- 若 ROS2 不可用且允许降级，启动 `Simulator`。
- 提供全部 HTTP 接口。
- 提供 `/health` 和 `/diag/mapping_prereq` 诊断接口。
- 提供 `/ws/stream` 实时推流。
- 维护扫描会话状态：
  - 是否在采集中
  - 前后雷达帧数
  - 原始点数
  - 体素降采样后的累计点云
- 维护 WebSocket 发送队列、限流、保活、空闲超时和统计信息。

### 3.3 `app/config.py`

定义服务端运行参数，当前是代码内默认值，没有额外配置文件。

关键配置包括：

- `host` / `port`
- `ws_queue_size`
- `sim_rate_hz`
- `lidar_points_per_scan`
- `allowed_clock_drift_sec`
- `ros.topics.*`

ROS 相关配置支持：

- 里程计 `odom`
- GPS `gps`
- 栅格地图 `occupancy_grid`
- 前/后雷达 `lidar_front` / `lidar_rear`
- 单雷达兜底 `lidar_fallback`
- IMU `imu`
- TF `tf` / `tf_static`
- 相机列表 `camera_topics`
- 速度控制 `cmd_vel`

### 3.4 `app/models.py`

定义接口入参：

- `MoveCommand`
- `SaveMapRequest`
- `LoadMapRequest`
- `PlanPathRequest`
- `AddPoiRequest`

### 3.5 `app/topic_bus.py`

内部异步消息总线，服务端各数据源先发布到总线，再由 WebSocket 统一消费。

特点：

- 按 topic 建独立订阅队列。
- 发布端不阻塞，慢消费者会被丢弃旧消息。
- 自动统计：
  - `published`
  - `dropped`
  - `drop_rate`
  - `near_capacity_events`
  - `peak_fill_ratio`
  - `subscribers`

### 3.6 `app/simulator.py`

ROS 不可用时的内置仿真器，周期性生成：

- `/robot/pose`
- `/robot/gps`
- `/chassis/odom`
- `/chassis/status`
- `/camera/1..4/compressed`
- `/lidar/front`
- `/lidar/rear`
- `/map/grid`

同时维护：

- 机器人位姿
- 轨迹 `trajectory`
- GPS 轨迹 `gps_track`
- 底盘轨迹 `chassis_track`
- 路径点 `path`
- POI `poi`

### 3.7 `app/ros_bridge.py`

ROS2 桥接层，作用是把 ROS topic 统一转成服务端内部 topic。

主要能力：

- 订阅 odom / imu / tf / tf_static。
- 可选订阅 gps / occupancy_grid / lidar / camera。
- 将雷达 `LaserScan` 转成世界坐标点云。
- 根据 TF 解析雷达相对车体安装位姿。
- 发布 `/cmd_vel` 控制机器人。
- 提供 ROS 诊断信息。
- 提供建图前置条件检查。

内部转换后的统一 topic 仍然是：

- `/robot/pose`
- `/robot/gps`
- `/chassis/odom`
- `/chassis/status`
- `/lidar/front`
- `/lidar/rear`
- `/camera/{id}/compressed`
- `/map/grid`

### 3.8 `app/stcm_codec.py`

负责 `.stcm` 文件编解码。

当前格式本质是 zip 包，包含：

- `manifest.json`
- `radar_points.bin`

其中：

- `manifest.json` 保存 POI、路径、轨迹、GPS、底盘轨迹、位姿、备注等结构化信息。
- `radar_points.bin` 保存 `(x, y, intensity)` 的 `float32` 点云数据。

## 4. 数据流与订阅取数说明

## 4.1 总体链路

服务端的取数和对外推流链路如下：

```text
ROS2 topic / Simulator
        ↓
   RosBridge / Simulator
        ↓
      TopicBus
        ↓
   main.py /ws/stream
        ↓
      Client
```

## 4.2 ROS 模式取数

启动时 `startup()` 会调用 `detect_ros()`：

- 如果本机可以导入 `rclpy` 且桥接启动成功，服务进入 ROS 模式。
- `RosBridge` 会订阅配置中的 ROS topic。
- ROS 回调把消息转换成服务端统一 payload。
- 转换后的消息通过 `TopicBus.publish()` 发布。
- `/ws/stream` 对这些统一 topic 做订阅并推给客户端。

当前 ROS 输入到内部 topic 的映射关系如下：

| ROS 来源 | 内部 topic | 说明 |
| --- | --- | --- |
| `odom` | `/robot/pose`、`/chassis/odom`、`/chassis/status` | 位姿、速度、底盘状态 |
| `gps` | `/robot/gps` | 经纬度 |
| `LaserScan` 前雷达 | `/lidar/front` | 世界坐标点云 |
| `LaserScan` 后雷达 | `/lidar/rear` | 世界坐标点云 |
| `CompressedImage` | `/camera/{id}/compressed` | 当前仅传元信息，不透传图像字节 |
| `OccupancyGrid` | `/map/grid` | 栅格抽样后推送 |
| `tf` / `tf_static` | 不直接外发 | 仅用于坐标变换计算 |

补充说明：

- 雷达数据会结合 `odom + tf` 变换到世界坐标。
- 若配置了 `occupancy_grid`，保存地图时优先使用栅格图抽样点。
- 若没配 `occupancy_grid`，保存时回退为激光累计点云。

## 4.3 模拟模式取数

ROS2 不可用且允许 fallback 时：

- `Simulator.start()` 启动异步循环。
- 按 `sim_rate_hz` 周期生成位姿、GPS、底盘、相机、雷达和简化地图。
- 所有消息同样进入 `TopicBus`。

## 5. 健康检查与建图前置条件

### 5.1 `/health`

`/health` 提供轻量摘要，适合客户端低频轮询。

重点字段：

- `mapping_ready`
- `mapping_status`
- `mapping_blockers`
- `mapping_warnings`

保留的基础运行信息：

- `ros_enabled`
- `scan_active`
- `ws_clients`
- `scan_summary`
- `ros_diag`

设计意图：

- 客户端对连接不稳定的主要感知仍来自 WebSocket 自身状态、消息 gap、延迟和 API 错误。
- `/health` 不再承担高频链路探测职责，更多用于提供“是否具备开始建图条件”的摘要。

### 5.2 `/diag/mapping_prereq`

`/diag/mapping_prereq` 返回建图前置条件的详细结果，结构包含：

- `ready`
- `severity`
- `blockers`
- `warnings`
- `checks`

当前检查重点：

- `odom` topic 是否存在且足够新鲜
- 前/后雷达或 fallback lidar 是否存在且足够新鲜
- `robot_base_frame -> lidar_frame` 或实际观测到的 `scan_frame` 是否可解析
- 动态 TF 是否过旧
- websocket client 和 topic bus 是否存在明显链路压力

约定：

- `blockers` 会阻止开始建图
- `warnings` 只提示风险，不阻止开始建图

### 5.3 `/scan/start` 前置拦截

调用 `/scan/start` 时，服务端会先执行建图前置检查。

如果条件不满足，不会启动扫描，而是返回：

```json
{
  "ok": false,
  "reason": "mapping_prereq_failed",
  "scan_active": false,
  "mapping_prereq": {
    "ready": false,
    "severity": "error",
    "blockers": ["..."],
    "warnings": ["..."],
    "checks": {}
  }
}
```

这样可以避免 TF 树未准备好、里程计缺失或激光输入异常时误开始建图。

### 5.4 TF 树要求

ROS 模式下，建图至少要求：

- 有可用的 `odom`
- 有可用的 lidar topic
- 服务端可以从 `robot_base_frame` 解析到雷达 frame

说明：

- 服务端优先使用实际收到的 `LaserScan.header.frame_id` 作为雷达 frame
- `lidar_frame` 是配置兜底值，当雷达消息里没有可用 frame 或尚未收到 scan 时使用
- 若解析出的雷达 frame 与 `robot_base_frame` 相同，视为 identity transform
- `tf_static` 来源的变换可以长期有效
- 动态 `tf` 若长时间不更新，会被视为 stale 并进入 blocker

### 5.4.1 新车适配需要改哪些配置

适配一台新车时，优先检查并修改 `server/app/config.py` 里的 `RosTopicConfig`。

通常至少需要确认这些字段：

- `odom`
- `imu`
- `tf`
- `tf_static`
- `cmd_vel`
- `lidar_fallback`，或者 `lidar_front` / `lidar_rear`
- `odom_frame`
- `robot_base_frame`
- `lidar_frame`

如果现场真实 TF 是：

```text
odom -> base_link -> laser
```

那默认建议配置就是：

```python
odom_frame = "odom"
robot_base_frame = "base_link"
lidar_frame = "laser"
lidar_fallback = "/scan"
```

推荐做法是两层都保留：

- 配置里明确写这台车的标准 frame/topic
- 运行时允许服务端根据实际 `scan_frame` 自动兜底

这样既方便诊断，也不会把 TF 树完全写死。

### 5.5 网络不稳定时的处理原则

区分两类问题：

1. 数据源前置条件失效
例如 odom 或 lidar 实际断流，导致建图基础条件不成立
这类进入 `blockers`

2. 客户端连接或推流质量波动
例如没有 websocket client、topic bus 有压力、掉帧风险升高
这类进入 `warnings`

这样可以把“不能开始建图”和“可以建图但链路质量不好”分开表达。
- 所以客户端不需要区分 ROS 模式和模拟模式。

## 4.4 WebSocket 推流机制

客户端连接 `WS /ws/stream` 后，服务端会同时订阅这些 topic：

- `/robot/pose`
- `/robot/gps`
- `/chassis/odom`
- `/chassis/status`
- `/lidar/front`
- `/lidar/rear`
- `/camera/1/compressed`
- `/camera/2/compressed`
- `/camera/3/compressed`
- `/camera/4/compressed`
- `/map/grid`

每条推送消息都会统一封装为：

```json
{
  "topic": "/robot/pose",
  "stamp": 1710000000.123,
  "server_time_ms": 1710000000123,
  "seq": 1,
  "payload": {},
  "checksum": "sha256..."
}
```

字段说明：

- `topic`: 消息主题
- `stamp`: 原始消息时间戳
- `server_time_ms`: 服务端打包时间
- `seq`: 按 topic 单独递增的序号
- `payload`: 业务数据
- `checksum`: 基于 `topic + stamp + seq + payload` 的 SHA256

### 4.4.1 队列与限流

服务端对 WS 推流做了保护：

- 每个连接有独立发送队列。
- 相机 topic 最小发送间隔是 `0.2s`。
- 雷达 topic 非关键帧时会抽稀到最多 `1200` 点。
- 雷达每 `1s` 至少发一次关键帧。
- 客户端超过 `20s` 无活动会被强制断开。
- 客户端发 `"ping"`，服务端返回 `"pong"`。

### 4.4.2 雷达数据在 WS 中的附加字段

`/lidar/front` 和 `/lidar/rear` 的 `payload` 在 WebSocket 下会附加：

- `raw_points`: 原始点数
- `keyframe`: 是否关键帧

示例：

```json
{
  "topic": "/lidar/front",
  "payload": {
    "points": [[1.0, 2.0, 0.8]],
    "raw_points": 4000,
    "keyframe": false
  }
}
```

## 4.5 扫描数据累计逻辑

扫描开始后：

- `/scan/start` 会把会话置为 `active=true`。
- 前后雷达点云进入 `_accumulate_scan()`。
- 服务端按 `voxel_size` 做二维体素去重累计。
- `/scan/stop` 停止累计。
- `/map/save` 把累计点云或占据栅格点导出到 `.stcm`。

扫描统计可从：

- `GET /health`
- `GET /diag/stream_stats`

查看。

## 5. 启动方式

## 5.1 安装依赖

```bash
cd tool/server
pip install -r requirements.txt
```

## 5.2 启动服务

```bash
cd tool/server
python run_server.py
```

启动后默认监听：

- HTTP: `http://127.0.0.1:8080`
- WS: `ws://127.0.0.1:8080/ws/stream`

## 5.3 运行模式判断

可通过 `GET /health` 查看：

- `ros_enabled`
- `ros_reason`
- `simulator_active`
- `ros_diag`

## 6. 对外接口文档

## 6.1 健康检查

### `GET /health`

作用：

- 查看服务是否正常
- 查看当前是 ROS 模式还是模拟模式
- 查看扫描状态、topic 列表、坐标系、容量配置

响应示例：

```json
{
  "ok": true,
  "ros_enabled": false,
  "ros_reason": "ROS2 unavailable",
  "scan_active": false,
  "ws_clients": 1,
  "topics": ["/robot/pose", "/robot/gps"],
  "scan_summary": {},
  "ros_diag": {},
  "simulator_active": true,
  "map_source": "laser_accumulation",
  "frames": {
    "odom": "odom",
    "base": "base_link",
    "lidar": "laser"
  },
  "capacity": {}
}
```

## 6.2 推流诊断

### `GET /diag/stream_stats`

作用：

- 查看 topic 总线发布/丢帧情况
- 查看每个 topic 的序号
- 查看 WS 连接数和扫描统计

关键响应字段：

- `topic_stats`
- `seq_by_topic`
- `scan_summary`
- `capacity`
- `ros_diag`

## 6.3 扫描控制

### `POST /scan/start`

作用：

- 重置扫描会话
- 开始累计雷达点云

请求体：

- 无

响应字段：

- `ok`
- `scan_active`
- `scan_summary`
- `ros_enabled`

### `POST /scan/stop`

作用：

- 停止扫描累计

请求体：

- 无

### `POST /scan/reset`

作用：

- 清空当前扫描累计数据和统计

请求体：

- 无

## 6.4 运动控制

### `POST /control/move`

作用：

- 控制机器人按给定速度和角速度运动一段时间
- ROS 模式下发 `cmd_vel`
- 模拟模式下驱动内部状态机

请求体：

```json
{
  "velocity": 0.5,
  "yaw_rate": 0.0,
  "duration": 1.0
}
```

字段说明：

- `velocity`: 线速度，范围 `-2.0 ~ 2.0`
- `yaw_rate`: 角速度，范围 `-2.0 ~ 2.0`
- `duration`: 持续时间，范围 `0.05 ~ 10.0`

### `POST /control/stop`

作用：

- 立即停止机器人运动

请求体：

- 无

## 6.5 路径与 POI

### `POST /path/plan`

作用：

- 下发手工路径点
- 当前服务端只做保存，不做自动寻路

请求体：

```json
{
  "nodes": [
    { "x": 1.0, "y": 2.0, "lat": 31.23, "lon": 121.47 },
    { "x": 3.0, "y": 4.0 }
  ]
}
```

响应字段：

- `ok`
- `path_nodes`
- `algo`

### `POST /map/poi`

作用：

- 新增一个 POI 点

请求体：

```json
{
  "poi": {
    "name": "P1",
    "x": 1.0,
    "y": 2.0,
    "lat": 31.23,
    "lon": 121.47
  }
}
```

响应字段：

- `ok`
- `poi_count`

## 6.6 地图文件管理

### `POST /map/save`

作用：

- 保存当前地图为 `.stcm`
- 同时保存 POI、路径、轨迹、GPS、底盘轨迹、扫描统计等

请求体：

```json
{
  "name": "session",
  "notes": "test",
  "voxel_size": 0.12,
  "reset_after_save": false
}
```

字段说明：

- `name`: 文件名前缀
- `notes`: 备注
- `voxel_size`: 保存前更新扫描体素尺寸，可选
- `reset_after_save`: 保存后是否重置扫描会话

响应字段：

- `ok`
- `file`
- `contains`
- `scan_summary`
- `ros_enabled`

说明：

- 默认保存到 `tool/server/data/maps/*.stcm`
- 若当前没有可保存点云，会兜底写入一个点 `(0,0,1)`

### `POST /map/load`

作用：

- 加载已有 `.stcm`
- 恢复点云、POI、路径、轨迹等内存状态

请求体：

```json
{
  "filename": "session_1710000000.stcm"
}
```

响应字段：

- `ok`
- `point_count`
- `poi_count`
- `path_count`
- `chassis_count`
- `scan_summary`

### `GET /map/list`

作用：

- 列出 `data/maps` 下可加载的 `.stcm` 文件

响应示例：

```json
{
  "ok": true,
  "files": ["session_1710000000.stcm"]
}
```

## 6.7 WebSocket 实时订阅接口

### `WS /ws/stream`

作用：

- 订阅服务端统一 topic 实时流

客户端接入约定：

- 建连地址：`ws://<server_ip>:8080/ws/stream`
- 客户端应定期发送 `"ping"`
- 服务端回复 `"pong"`

当前会推送的 topic：

- `/robot/pose`
- `/robot/gps`
- `/chassis/odom`
- `/chassis/status`
- `/lidar/front`
- `/lidar/rear`
- `/camera/1/compressed`
- `/camera/2/compressed`
- `/camera/3/compressed`
- `/camera/4/compressed`
- `/map/grid`

各 topic 的典型 `payload` 示例：

### `/robot/pose`

```json
{
  "x": 1.2,
  "y": 0.8,
  "yaw": 0.5,
  "vx": 0.3,
  "wz": 0.1
}
```

### `/robot/gps`

```json
{
  "lat": 31.2304,
  "lon": 121.4737
}
```

### `/chassis/odom`

```json
{
  "x": 1.2,
  "y": 0.8,
  "yaw": 0.5,
  "vx": 0.3,
  "wz": 0.1
}
```

### `/chassis/status`

```json
{
  "wheel_speed_l": 0.2,
  "wheel_speed_r": 0.2,
  "battery": 99.8,
  "mode": "AUTO_MAP"
}
```

### `/lidar/front` 或 `/lidar/rear`

```json
{
  "points": [[1.0, 2.0, 0.8], [1.1, 2.1, 0.9]],
  "raw_points": 4000,
  "keyframe": true
}
```

ROS 模式下还可能包含：

```json
{
  "scan_frame": "laser_frame",
  "base_frame": "base_footprint",
  "mount": {
    "tx": 0.1,
    "ty": 0.0,
    "yaw": 0.0
  }
}
```

### `/camera/{id}/compressed`

当前不是原图透传，主要是元数据：

```json
{
  "camera_id": 1,
  "format": "jpeg",
  "byte_size": 20480,
  "objects": [
    { "label": "image:1", "confidence": 1.0 }
  ]
}
```

模拟模式下则是随机目标列表。

### `/map/grid`

ROS 栅格模式示例：

```json
{
  "occupied": [{ "x": 1.0, "y": 2.0, "value": 100 }],
  "free": [{ "x": 1.1, "y": 2.1, "value": 0 }],
  "resolution": 0.05,
  "origin": { "x": 0.0, "y": 0.0 },
  "width": 100,
  "height": 100
}
```

模拟模式示例：

```json
{
  "clusters": [{ "x": 1.0, "y": 2.0, "value": 0.9 }]
}
```

## 7. 当前实现边界

需要明确几件事，避免 README 误解：

1. `/path/plan` 当前只保存路径节点，不执行路径规划算法。
2. `/camera/*/compressed` 当前只输出元信息，不直接透传图像内容。
3. 配置目前写死在 [config.py](C:\Users\Dell\Documents\my_test\autodrive\map\tool\server\app\config.py) 默认值中，没有环境变量或外部配置文件。
4. `README` 中的接口文档基于当前代码实现，不代表未来协议冻结。
