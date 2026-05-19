# Desktop Client

`client_desktop` 是当前 AutoDrive 地图工具的 Tk 桌面端，已经按最小闭环收口到这些能力：

- 连接 server 的 HTTP / WebSocket
- 开始 / 停止 2D、3D 扫描
- 实时显示 `/robot/pose` 和 `/map/grid`
- 本地 `.slam` 保存 / 加载
- 原生地图导入：`.zip` / `.yaml` / `.yml` / `.pgm`
- 导出 `ZIP` / `PGM` / `YAML` / `JSON` / `PCD`
- 栅格二次编辑
- POI 和路径编辑
- 键盘远控

## 当前边界

桌面端现在不再依赖这些实时流：

- `/robot/gps`
- `/chassis/odom`
- `/chassis/status`
- `/lidar/front`
- `/lidar/rear`
- `/camera/*`

也就是说，主流程实时数据只靠两类 topic：

- `/robot/pose`
- `/map/grid`

## 和 server 的接口关系

桌面端当前依赖的 server 接口主要是：

- `GET /health`
- `GET /diag/mapping_prereq`
- `POST /scan/start`
- `POST /scan/stop`
- `GET /scan/pcd`
- `POST /control/target`
- `POST /control/stop`
- `WS /ws/stream`

其中：

- `/health` 低频轮询，只用于显示 server 诊断和建图是否 ready
- `WS /ws/stream` 是主实时链路
- 3D 导出 PCD 时会走 `/scan/pcd`

## 实时消息

client 收到 WebSocket 消息后，会做合流，只保留最新的：

- `/robot/pose`
- `/map/grid`

这样做的目的是：

- 降低 UI 压力
- 避免旧消息堆积
- 保持画面和位姿跟手

当前版本不再把 lidar / camera / gps / chassis 作为实时 UI 输入。

## 键盘远控

键盘远控现在的机制是：

1. 按键按下后，后台线程重复发送 `POST /control/target`
2. 松键后，按配置延迟一个很短的确认窗口再发 `POST /control/stop`
3. 如果 client 中断发送，server 侧 hold 超时后也会自动停车

相关默认值：

- `repeat (ms)` 默认 `120`
- 松键确认窗口 `50ms`

这套链路就是当前“远控顺滑 + 通信异常可自停”的主要保障。

## 扫描模式

### 2D

- 用 server 下发的 occupancy grid 做实时主视图
- 停止后可直接保存本地 `.slam`

### 3D

- 实时主视图仍然使用 occupancy grid
- 需要导出或保存带点云的结果时，client 会向 server 请求 `/scan/pcd`

当前 3D PCD 下载行为：

- server 先做降采样，再返回
- 默认 `voxel_size=0.05`
- client 本地缓存下载到的 PCD，供保存 `.slam` 或导出 `.pcd`

## `.slam` 归档

当前桌面端写入的归档结构是：

- `manifest.json`
- `grid.bin`
- 可选 `map.pcd`

`grid.bin` 存的是 occupancy grid 的 `int8`：

- `-1` 未知
- `0` 空闲
- `100` 障碍

`manifest.json` 主要保存：

- 栅格元信息
- POI
- 路径
- 笔记
- pose
- scan summary
- scan fusion 配置

当前文档里不再把 GPS / chassis 当作核心归档依赖说明，因为主流程已经不依赖它们。

## 加载与导出

### 加载

可以加载：

- `.slam`
- `.zip`
- `.yaml` / `.yml`
- `.pgm`

ZIP 导入规则：

- ZIP 内必须有 YAML
- YAML 引用的 PGM 必须存在
- `map.json` 可有可无，不是导入前提

### 导出

`Export ZIP` 会生成：

- `map.pgm`
- `map.yaml`
- `map.json`

`Export PCD` 只有在当前会话或已加载 `.slam` 持有 PCD 内容时才可用。

如果当前内存里没有 PCD，但扫描模式是 3D，client 会尝试重新向 server 拉 `/scan/pcd`。

## 地图编辑

当前地图编辑围绕 occupancy grid：

- 擦除噪点
- 画障碍线
- POI 编辑
- 路径连接
- 闭环校验

路径规划优先使用“当前活跃栅格”的分辨率，不再把旧的 scan accumulation 当作主数据源。

## 运行

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

### Linux

```bash
cd client_desktop
chmod +x build_linux.sh
./build_linux.sh
```

### macOS

```bash
cd client_desktop
chmod +x build_macos.sh
./build_macos.sh
```
