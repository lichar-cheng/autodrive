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
- `local` / `cloud` 双认证
- `2d` / `3d` 双扫图模式
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

有两种连接方式。

### 方式 A：直接输入 WS 地址

1. 在顶部 `Server WS` 输入地址，例如：
   `ws://192.168.3.56:8080/ws/stream`
2. 点击 `Connect`
3. 程序会先对 `/health` 进行预检查
4. 如果预检查成功，再启动 WebSocket 连接

### 方式 B：先认证再连接

客户端支持两种认证模式：

- `local`
  - 输入目标设备 `IP / Port / Path`
  - 客户端会向该登录接口发送固定用户名和密码
  - 从响应中解析后端 `host / port / token`
- `cloud`
  - 输入云端认证 `Host / Port / Path / Username / Password`
  - 客户端会向云端认证接口发起鉴权
  - 从响应中解析后端 `host / port / token`

认证成功后：

- `Server WS` 会自动切换为解析后的机器人服务地址
- 后续 HTTP 请求会自动带 `Bearer token`
- 后续 WebSocket 连接也会优先使用认证后的后端描述符

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

1. 选择 `Scan Mode`
2. 输入地图名称和备注
3. 点击 `Start Scan`
4. 控制机器人移动进行扫描
5. 查看中间地图累计结果
6. 点击 `Stop Scan`
7. 在 `Map` 区点击 `Save Map`

保存后会生成单个 `.slam` 文件。

### 5.0 扫图模式说明

系统支持两种互斥模式：

- `2d`
  - 数据源是 `LaserScan`
  - 主地图是黑白占据图
  - 支持噪点擦除、障碍线绘制和二阶段地图编辑
  - 地图保存为 `manifest.json + radar_points.bin`
- `3d`
  - 数据源是 `sensor_msgs/PointCloud2`
  - 主画布显示轻量 3D 点云预览
  - 当前版本只支持预览和保存，不支持 3D 画布编辑
  - 地图保存为 `manifest.json + point_cloud.bin`

切换模式时：

- 客户端会同步调用服务端切换运行模式
- 当前累计数据会清空，避免 2D / 3D 数据混用
- 建议在切换前先停止当前扫描

### 5.1 扫描融合预设

扫描区支持场景预设和参数覆盖，用来适配不同环境下的障碍物显著性。

预设包括：

- `sim_clean`
- `indoor_balanced`
- `indoor_sensitive`
- `warehouse_sparse`

可调参数包括：

- `Voxel`
- `Min Occupied Hits`
- `Occ/Free Ratio`
- `Turn Skip Wz`
- `Skip Turn Frames`

推荐使用方式：

- 仿真环境使用 `sim_clean`
- 普通室内实测使用 `indoor_balanced`
- 若桌子、椅子、细腿、边缘障碍不明显，切换到 `indoor_sensitive`
- 空旷仓储环境可尝试 `warehouse_sparse`

### 5.2 不同场景怎么选

#### `sim_clean`

适用场景：

- 内置仿真器
- 激光点云稳定、连续、密集
- 希望地图更干净，少保留边缘噪点

特点：

- 更保守
- 对弱障碍物不敏感
- 更容易得到清晰墙线和边界

不适合：

- 实车室内细腿桌椅
- 玻璃边缘、窄柱子、桌角这类回波稀疏目标

#### `indoor_balanced`

适用场景：

- 普通办公室
- 教室
- 家居环境
- 大多数实车室内首轮建图

特点：

- 是默认实车配置
- 在“保住障碍”和“控制噪点”之间折中
- 墙、柜子、普通桌椅通常都能保住

建议：

- 实测时先从这个 preset 开始
- 如果地图整体够清楚，就不要继续调激进

#### `indoor_sensitive`

适用场景：

- 桌子、椅子、细腿、窄边框不明显
- 实际环境回波弱
- 机器人运行一次就要尽量把小障碍也留下

特点：

- 更激进
- 更容易保住细小障碍
- 也更容易把噪点、拖影、边缘毛刺一起保留下来

建议：

- 当你发现“墙很清楚，但桌椅几乎没有”时优先切这个
- 切换后要重点观察转弯区域是否出现明显拖尾

#### `warehouse_sparse`

适用场景：

- 障碍物稀疏
- 空间开阔
- 货架、柱子、少量大物体为主

特点：

- 比 `indoor_balanced` 稍敏感
- 适合大空间，但不追求极细小结构

建议：

- 如果场地很空，但又不是仿真，可尝试这个 preset

### 5.3 每个参数的作用

#### `Voxel`

作用：

- 控制栅格粒度
- 决定一个障碍点会落到多大的格子里

调小的效果：

- 更容易保住细障碍物
- 地图更细
- 噪点会变多
- 计算和显示上的散点感会更强

调大的效果：

- 墙体更厚、更稳定
- 地图更干净
- 细腿、桌边、窄障碍更容易被吞掉

使用建议：

- 看到“桌椅只有一点点”时，优先适当减小
- 看到“满屏毛刺和碎点”时，适当增大

#### `Min Occupied Hits`

作用：

- 一个格子至少被命中多少次才被当作障碍

调小的效果：

- 更不容易漏障碍
- 一次掠过的目标也可能留下
- 但误检和孤立噪点会增加

调大的效果：

- 更干净
- 但弱障碍和细障碍容易消失

使用建议：

- 桌椅看不见时，从 3 降到 2 或 1
- 噪点太多时，再往上提

#### `Occ/Free Ratio`

作用：

- 判断某个格子的占据命中，是否足以压过同位置附近的 free 命中

为什么有这个参数：

- 扫描过程中一条射线会经过很多空闲格
- 如果占据命中很弱，而 free 命中过强，格子可能被视为不稳定障碍

调小的效果：

- 更容易把弱障碍判成障碍
- 桌腿、边角、窄边框更容易保住

调大的效果：

- 更强调“这个格子必须明显更像障碍”
- 地图会更干净，但弱障碍更容易被抹掉

使用建议：

- 如果“有一点点，但很快又没了”，通常可以把这个参数再降一点

#### `Turn Skip Wz`

作用：

- 当机器人角速度过大时，用这个阈值判断是否跳过当前扫描帧

调小的效果：

- 更容易跳过转弯帧
- 可以减少转弯拖影
- 但会少掉一些真实障碍数据

调大的效果：

- 更多转弯帧会被保留
- 有助于在复杂环境保住更多障碍
- 但转弯时更容易出现拉丝和重影

使用建议：

- 直线扫描为主时影响不大
- 转弯多、地图拖影重时可以适当减小

#### `Skip Turn Frames`

作用：

- 是否启用“转弯帧跳过”机制

关闭后的效果：

- 所有帧都参与累计
- 对细障碍最友好
- 但也是最容易带来转弯拖影的配置

使用建议：

- 只有在 `indoor_sensitive` 或特殊实测场景下才建议关闭

### 5.4 页面切换场景是否立即生效

会立即生效。

切换 preset 或修改参数后：

- 后续新进入的扫描帧会立即按新配置累计
- 当前画面的占据/空闲显示也会立即按新阈值过滤
- 保存 `.slam` 时会把当前有效配置写进地图元数据

需要注意：

- 已经累计在内存里的 `hits` 和 `free hits` 不会因为切换 preset 被清空
- 变化的是“如何解释这些累计结果”

如果你想做严格对比，建议：

1. `Clear`
2. 切换 preset
3. 重新扫同一路径

这样最容易看出不同场景配置的差异

### 5.5 推荐调参顺序

建议按下面顺序调，不要一上来同时改 5 个参数。

#### 情况 A：墙清楚，但桌椅、细腿几乎没有

1. 先切 `indoor_sensitive`
2. 还是不够，再减小 `Voxel`
3. 再降低 `Min Occupied Hits`
4. 还不够，再降低 `Occ/Free Ratio`

#### 情况 B：障碍出来了，但噪点很多

1. 先切回 `indoor_balanced`
2. 适当增大 `Voxel`
3. 提高 `Min Occupied Hits`
4. 再提高 `Occ/Free Ratio`

#### 情况 C：转弯位置拖影明显

1. 保持当前 preset 不变
2. 减小 `Turn Skip Wz`
3. 如果 `Skip Turn Frames` 是关闭的，先打开
4. 还不行，再回退到更保守的 preset

### 5.6 常见现象与建议

现象：墙很清楚，但桌子、椅子不明显  
建议：切 `indoor_sensitive`

现象：地图里到处都是毛刺、碎点  
建议：切 `indoor_balanced` 或增大 `Voxel`

现象：转弯处像刷子一样拖尾  
建议：打开 `Skip Turn Frames`，并减小 `Turn Skip Wz`

现象：障碍一会儿有、一会儿没  
建议：降低 `Occ/Free Ratio`，必要时降低 `Min Occupied Hits`

### 5.7 保存与加载后的行为

保存 `.slam` 时，当前扫描融合配置会写入地图元数据。

重新加载 `.slam` 后：

- 会恢复当时保存的融合 preset
- 会恢复对应的有效参数
- 导出的 `.pgm` / `.yaml` / `.json` 也会继续沿用这份结果解释

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

说明：

- 当前地图编辑能力只适用于 `2d`
- `3d` 模式下画布是只读预览，不支持直接擦点或画障碍

### 6.1 清噪后为什么是白色

现在无论是：

- 手动橡皮擦清理
- 自动孤立噪点清理

被清掉的障碍格都会转成白色 free cell，而不是直接露出背景色。

这样做的含义是：

- 该位置被明确标记为可通行
- 后续保存 `.slam` 时，这个 free 信息也会被保留
- 地图语义更稳定，不会出现“看起来像被挖空”的未知区域

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
- `point_cloud.bin`

主要保存的数据：

- POI
- 路径
- 轨迹
- 备注
- 占据栅格
- 雷达点
- 点云点

说明：

- `2d` 模式使用 `radar_points.bin`
- `3d` 模式使用 `point_cloud.bin`
- `manifest.json` 中会记录 `scan_mode`

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
