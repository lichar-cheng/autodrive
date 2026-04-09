# AutoDrive 扫图工具产品说明文档

## 1. 产品简介

AutoDrive 扫图工具是一套面向机器人扫图、地图编辑、POI 标注、路径规划和近场控制的桌面工作台。

当前产品由三部分组成：

- `server`：机器人侧服务
- `client_desktop`：桌面主客户端
- `client`：浏览器调试端

其中，`client_desktop` 是主要交付形态。

---

## 2. 主要功能

桌面端当前支持：

- 连接机器人服务端
- 开始/停止扫描
- 地图保存与加载
- 地图导出 `PGM`、`YAML`、`JSON`
- 地图噪点清理和障碍编辑
- POI 批量添加、单点添加、编辑已有
- 自动推算缺失经纬度
- 按 POI 或任意两点生成路径
- 自动闭环和闭环检查
- 近场键盘和按钮控制
- 相机缓冲快照刷新
- 中英文切换
- 本地日志落盘

---

## 3. 快速开始

## 3.1 启动服务端

```bash
cd server
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
python3 run_server.py
```

## 3.2 启动桌面端

```bash
cd client_desktop
python3 -m pip install -r requirements.txt
python3 run_client.py
```

## 3.3 连接机器人

1. 在顶部 `Server WS` 输入地址，例如：
   `ws://192.168.3.56:8080/ws/stream`
2. 点击 `Connect`
3. 程序会先对 `/health` 进行预检查
4. 如果预检查成功，再启动 WebSocket 连接

如果连接失败，会弹出错误信息，并在日志中记录详细原因。

---

## 4. 界面说明

## 4.1 左侧功能区

左侧功能区包含：

- Scan
- Move
- POI
- Path
- Map

支持：

- 按钮自动换行
- 左侧区域纵向滚动
- 空间不足提示

## 4.2 中间地图区

中间是主地图画布，用于：

- 查看实时地图
- 查看机器人位置
- 显示 POI 和路径
- 擦除噪点
- 绘制障碍线

## 4.3 右侧信息区

右侧包含：

- Odom And Scan
- Cameras
- Communication / Map

主要用于查看状态和排查问题。

---

## 5. 扫描流程

1. 输入地图名称和备注
2. 点击 `Start Scan`
3. 控制机器人移动进行扫描
4. 查看中间地图累计结果
5. 点击 `Stop Scan`
6. 在 `Map` 区点击 `Save Map`

保存后会生成单个 `.slam` 文件。

---

## 6. 地图编辑流程

1. 在 `Map` 区点击 `Load Map`
2. 加载已有 `.slam` 文件
3. 选择地图工具：
   - View / Select
   - Erase Noise
   - Draw Obstacle
4. 对地图进行编辑
5. 点击 `Save Map` 保存新的地图文件

---

## 7. POI 使用说明

POI 区支持 3 种模式。

## 7.1 Batch

默认模式。

用途：

- 批量输入多个 POI
- 逐个落到地图上
- 对缺失经纬度的点自动推算

规则：

- 如果没有任何点带经纬度，可以正常使用
- 如果有点带经纬度，则至少需要 3 个点带 `lon,lat`
- 满足 3 个以上控制点后，会自动推算其他点的经纬度

输入格式：

- `name`
- `name,lon,lat`
- `name,lon,lat,yaw`

## 7.2 Single

用于新增单个 POI。

可填写：

- 名称
- `x`
- `y`
- `yaw`
- `lon,lat`

## 7.3 Edit Existing

用于编辑已有 POI。

使用方式：

1. 在列表中选中一个且仅一个 POI
2. 切到 `Edit Existing`
3. 修改名称、坐标、航向角和经纬度
4. 点击 `Apply Edit`

---

## 8. 路径使用说明

路径区支持：

- 按 POI 名称连线
- 地图任意两点连线
- 自动闭环
- 闭环检查
- 删除线段

### 常用流程

#### 方式 A：按 POI 连线

1. 选择 `Path Tool = POI`
2. 输入起点和终点 POI 名称
3. 点击 `Connect Named POI`

#### 方式 B：任意两点连线

1. 选择 `Path Tool = Free Points`
2. 在地图上点击起点
3. 再点击终点

#### 方式 C：自动闭环

1. 创建多个 POI
2. 点击 `Auto Loop`
3. 使用 `Closed-Loop Check` 检查路径闭环性

---

## 9. 移动控制说明

Move 区支持按钮和键盘控制。

### 按钮

- Forward：前进
- Reverse：后退
- Left：原地左转
- Right：原地右转
- Stop：停止

### 键盘

- `W / ↑`：前进
- `S / ↓`：后退
- `A / ←`：左转
- `D / →`：右转
- `Space`：停止

如果启用了 `Stop on keyup`，松开按键会自动停止。

---

## 10. 地图文件说明

地图使用单个 `.slam` 文件保存。

内容包括：

- `manifest.json`
- `radar_points.bin`

主要保存的数据：

- POI
- 路径
- 轨迹
- 备注
- 占据栅格
- 雷达点

---

## 11. 日志与排障

## 11.1 日志位置

- Windows：
  `%LOCALAPPDATA%\AutoDriveClient\logs\client_desktop.log`
- macOS：
  `~/Library/Logs/AutoDriveClient/logs/client_desktop.log`
- Linux：
  `~/.local/state/AutoDriveClient/logs/client_desktop.log`

## 11.2 常见排查方法

### 连接不上

检查：

- 服务端是否启动
- IP 和端口是否正确
- 桌面端 `Connect` 点击后是否弹出预检查错误
- 日志中是否有 `connect preflight failed`

### 左侧功能区看不到内容

左侧功能区支持滚动。若空间不足，会显示提示并可滚动查看下方内容。

### 地图保存或加载失败

检查：

- 路径是否可写
- `.slam` 文件是否完整
- 日志中是否有 `map saved` / `map loaded`

---

## 12. 打包说明

### Windows

```bat
cd client_desktop
build_windows.bat
```

输出：

- `dist/AutoDriveClient.exe`

### Linux

```bash
cd client_desktop
chmod +x build_linux.sh
./build_linux.sh
```

输出：

- `dist/AutoDriveClient`

### macOS

```bash
cd client_desktop
chmod +x build_macos.sh
./build_macos.sh
```

输出：

- `dist/AutoDriveClient.app`

---

## 13. 当前限制

- 桌面端已支持主要中英文切换，但少量深层错误描述仍可能保留英文风格
- macOS 打包当前未包含签名和 notarization
- 产品内暂未提供完整日志查看器，仅提供日志落盘和路径显示
