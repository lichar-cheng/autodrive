# AutoDrive Server

`server` 是当前桌面地图工具配套的 FastAPI 后端，职责已经收敛到 4 件事：

1. 提供远控接口。
2. 提供扫描启停和 3D PCD 下载接口。
3. 把 ROS2 的 `/robot/pose` 和 `/map/grid` 转成统一 WebSocket 流。
4. 提供 `/health` 和 `/diag/mapping_prereq` 诊断，帮助 client 判断能不能开始扫描。

## 当前边界

`server` 现在不再负责这些能力：

- 模拟器 fallback
- camera / imu / gps / chassis 透传
- lidar front / rear WebSocket 流
- server 端 `.slam` 保存、加载、上传、下载
- server 端 POI / 路径管理
- 超声波安全限速

换句话说，当前 server 只保留了“控制 + 扫描 + 位姿 + 栅格 + TF/建图前置检查”这条最小闭环。

## 目录

```text
server/
├─ run_map_server.py         # Uvicorn 入口
├─ start_map_server.sh       # 启动脚本
├─ requirements.txt
└─ app/
   ├─ main.py                # FastAPI 路由、控制平滑、扫描会话、WS 推流
   ├─ ros_bridge.py          # ROS2 -> pose/grid 桥接与建图前置检查
   ├─ config.py              # server/ROS/scan 配置
   ├─ models.py              # 请求模型
   ├─ topic_bus.py           # 异步 topic 总线
   └─ sample_pcd_file.py     # 3D PCD 降采样缓存
```

## HTTP / WS 接口

当前保留的接口：

- `GET /health`
- `GET /diag/mapping_prereq`
- `GET /diag/stream_stats`
- `POST /scan/start`
- `POST /scan/stop`
- `POST /scan/reset`
- `GET /scan/pcd`
- `POST /control/move`
- `POST /control/target`
- `POST /control/stop`
- `WS /ws/stream`

默认地址由 `CONFIG.host` / `CONFIG.port` 决定。

## ROS 数据流

`ros_bridge.py` 现在只订阅这些 ROS topic：

- `odom`
- `tf`
- `tf_static`
- `occupancy_grid`

内部输出只保留两个实时 topic：

- `/robot/pose`
- `/map/grid`

其中：

- `/robot/pose` 来自 `odom`，并在 `prefer_tf_pose=True` 时优先使用 `map->odom` 变换修正显示位姿。
- `/map/grid` 直接携带完整 occupancy grid：`data / width / height / resolution / origin / frame_id`。

`WS /ws/stream` 目前也只订阅这两个 topic。

## 控制链路

远控走两级保护：

1. `client_desktop` 高频重复发送 `POST /control/target`
2. `server` 后台循环按 `CONTROL_PUBLISH_INTERVAL_SEC` 持续发布，目标超时后自动补 0 速

平滑策略保留在线速度和角速度发布前：

- 起步加速度被单独压低，避免“起步太猛”
- 常规减速度和紧急停车减速度分开
- 目标过期后会发送有限次 stop burst，防止旧命令残留

这部分现在不再依赖超声波配置。

## 扫描与 Launch

`POST /scan/start` 的流程：

1. 校验模式 `2d` / `3d`
2. 根据 `config.scan_modes` 检查或拉起外部 launch 进程
3. 轮询 `mapping_prerequisites`
4. 通过后切换扫描会话为 active

`POST /scan/stop` 的流程：

1. 取消正在进行的 start 流程
2. 停止由 server 拉起或追踪到的进程
3. 清理扫描状态

当前 launch 依赖只按进程名模式和配置命令跟踪，不再检查旧的节点级多传感器状态。

## 3D PCD 下载

`GET /scan/pcd` 只在 `3d` 模式下可用。

行为如下：

- 根据 `config.scan_modes.mode_3d.pcd_output_path` 找到原始 PCD
- 调用 `sample_pcd_file.py` 做降采样
- 返回降采样后的文件，而不是原始全量 PCD

默认降采样参数：

- `voxel_size=0.05`

响应头会带这些信息，供 client 展示：

- `X-Scan-PCD-Downsampled`
- `X-Scan-PCD-Voxel-Size`
- `X-Scan-PCD-Name`
- `X-Scan-PCD-Size`
- `X-Scan-PCD-Source-Name`
- `X-Scan-PCD-Source-Size`

## 建图前置检查

`/health` 返回的是轻量摘要，适合低频轮询。

重点字段：

- `mapping_ready`
- `mapping_status`
- `mapping_blockers`
- `mapping_warnings`
- `scan_active`
- `scan_summary`
- `control_target`
- `topics`
- `ros_diag`

`/diag/mapping_prereq` 返回更细的检查结果。

当前检查重点是：

- `odom` 是否新鲜
- `tf` 是否新鲜
- `occupancy_grid` 是否新鲜
- `tf_static` 是否存在
- WebSocket/TopicBus 是否有明显压力

## WebSocket 行为

消息格式：

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

当前保留的稳定性保护：

- 每个 client 独立发送队列
- 队列满时丢弃最旧消息，不阻塞主发布路径
- 空闲 client 超时断开
- `ping` -> `pong`
- send-after-close 竞态保护

## 运行

```bash
cd server
python3 -m pip install -r requirements.txt
python3 run_map_server.py
```

如果你用脚本启动：

```bash
cd server
./start_map_server.sh
```
