# AutoDrive Server

`server` is the FastAPI backend for AutoDrive map capture, live streaming, motion control, and `.slam` map management.

## Overview

The server is responsible for:

1. Exposing HTTP and WebSocket APIs.
2. Bridging ROS2 topics when ROS is available, or falling back to the built-in simulator.
3. Aggregating pose, chassis, lidar, camera, and occupancy-grid data onto a unified internal topic bus.
4. Saving and loading `.slam` map files for client-side map editing, POI, and path workflows.

Default endpoints:

- `http://0.0.0.0:8080`
- `ws://0.0.0.0:8080/ws/stream`

Service entry:

- [run_server.py](/home/autodrive2/autodrive/server/run_server.py)

## Directory Layout

```text
server/
├─ run_server.py              # Uvicorn entrypoint
├─ requirements.txt           # Runtime dependencies
└─ app/
   ├─ main.py                 # FastAPI routes, WS stream, scan session, map management
   ├─ config.py               # Server and ROS topic configuration
   ├─ models.py               # Request models
   ├─ topic_bus.py            # Async topic bus with drop statistics
   ├─ simulator.py            # Built-in fallback data source
   ├─ ros_bridge.py           # ROS2 bridge
   └─ stcm_codec.py           # .slam archive read/write
```

## Runtime Model

### `run_server.py`

- Reads `CONFIG.host` and `CONFIG.port`
- Starts Uvicorn with `app.main:app`

### `app/main.py`

- Initializes `FastAPI`, `TopicBus`, and `Simulator`
- Detects ROS2 on startup
- Starts `RosBridge` when ROS2 is available
- Starts the simulator when ROS2 is unavailable and fallback is allowed
- Exposes HTTP APIs
- Exposes `/health` and `/diag/mapping_prereq`
- Exposes `/ws/stream`
- Maintains scan session state, latest occupancy grid, map files, and control target state

### `app/topic_bus.py`

Internal async bus used by both ROS and simulator sources.

It tracks:

- `published`
- `dropped`
- `drop_rate`
- `near_capacity_events`
- `peak_fill_ratio`
- `subscribers`

### `app/ros_bridge.py`

Bridges ROS2 topics into the server topic model.

Main responsibilities:

- Subscribe to `odom`, `imu`, `tf`, `tf_static`
- Optionally subscribe to `gps`, `occupancy_grid`, lidar, and camera topics
- Convert `LaserScan` into world-space points
- Resolve lidar mounting pose from TF
- Publish `/cmd_vel`
- Report mapping prerequisite diagnostics

### `app/simulator.py`

Used when ROS2 is unavailable.

It periodically publishes:

- `/robot/pose`
- `/robot/gps`
- `/chassis/odom`
- `/chassis/status`
- `/camera/1..4/compressed`
- `/lidar/front`
- `/lidar/rear`
- `/map/grid`

## `.slam` Format

The server reads and writes `slam.v4` archives through [app/stcm_codec.py](/home/autodrive2/autodrive/server/app/stcm_codec.py).

Archive layout:

- `manifest.json`
- `grid.bin`
- optional `map.pcd`

`grid.bin` stores packed occupancy-grid `int8` values:

- `-1`: unknown
- `0`: free
- `100`: occupied

`manifest.json` stores:

- grid metadata
- POI
- path
- pose
- GPS
- chassis
- notes
- scan summary

The server save/load path now treats occupancy grid as the primary persisted map representation. It no longer stores the map as expanded point dictionaries inside the `.slam` archive.

## Data Flow

### End-to-end

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

### ROS Mode

When ROS2 is available:

- `startup()` detects ROS support
- `RosBridge` subscribes to configured ROS topics
- callbacks convert ROS messages into internal payloads
- payloads are published onto `TopicBus`
- `/ws/stream` subscribes to those topics and forwards them to clients

Current main mappings:

| ROS source | Internal topic | Notes |
| --- | --- | --- |
| `odom` | `/robot/pose`, `/chassis/odom`, `/chassis/status` | pose, velocity, chassis |
| `gps` | `/robot/gps` | GPS data |
| `LaserScan` front | `/lidar/front` | world-space point cloud |
| `LaserScan` rear | `/lidar/rear` | world-space point cloud |
| `CompressedImage` | `/camera/{id}/compressed` | metadata stream |
| `OccupancyGrid` | `/map/grid` | occupancy grid |
| `tf`, `tf_static` | internal only | transforms for mapping |

Notes:

- Lidar points are transformed using odom and TF.
- If an `OccupancyGrid` source is configured, the latest grid is used directly for save/load and stream workflows.
- Scan accumulation still exists for scan summary and fallback map generation, but `.slam` persistence is now occupancy-grid based.

### Simulator Mode

When ROS2 is unavailable and fallback is enabled:

- `Simulator.start()` begins the async loop
- it generates pose, GPS, chassis, camera, lidar, and simplified map data at `sim_rate_hz`
- all messages flow through the same `TopicBus`

## WebSocket Stream

Clients connect to `WS /ws/stream`.

The server subscribes to:

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

Message shape:

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

Protection and throttling:

- each client has its own send queue
- camera topics are rate-limited
- lidar is decimated on non-keyframes
- lidar sends periodic keyframes
- idle WebSocket clients are disconnected
- `"ping"` receives `"pong"`

## Mapping Readiness

### `GET /health`

`/health` is a lightweight summary for low-frequency polling.

Key fields:

- `mapping_ready`
- `mapping_status`
- `mapping_blockers`
- `mapping_warnings`
- `ros_enabled`
- `scan_active`
- `ws_clients`
- `scan_summary`
- `control_target`
- `ros_diag`

### `GET /diag/mapping_prereq`

Detailed mapping prerequisite result, including:

- `ready`
- `severity`
- `blockers`
- `warnings`
- `checks`

Typical blockers and warnings cover:

- stale or missing `odom`
- stale or missing lidar
- TF from `robot_base_frame` to lidar frame not resolvable
- stale dynamic TF
- visible stream pressure

`POST /scan/start` runs the prerequisite check before starting a scan session.

## Motion Control

The current control path has two APIs:

- `POST /control/target`
- `POST /control/stop`

`POST /control/move` still exists, but the desktop client no longer uses it for repeated keyboard drive.

Current keepalive behavior in [app/main.py](/home/autodrive2/autodrive/server/app/main.py):

- control publish interval: `0.1s`
- target hold window: `1.0s`
- stale target warning: `control target stale; publishing stop for safety`
- stale stop burst: the server sends several explicit zero commands after hold expiry

Control sequence:

1. client repeatedly sends `/control/target`
2. server stores the latest target and timestamp
3. background publisher republishes the effective command every `100ms`
4. if no new target arrives within `1.0s`, the server marks the target stale and publishes stop for safety
5. `POST /control/stop` zeros the current target immediately

This means movement will not continue forever after the client disappears. Once the hold window expires, the server-side control loop publishes stop.

## Scan Session

During scanning:

- `/scan/start` marks the scan session active
- lidar frames are accumulated into the current scan session
- scan summary tracks frame counts, raw point totals, voxel size, and elapsed time
- `/scan/stop` stops scan accumulation
- `/scan/reset` clears the accumulated scan state

## API Summary

### `GET /health`

Returns current service health and runtime summary.

### `GET /diag/mapping_prereq`

Returns detailed mapping readiness checks.

### `POST /scan/start`

Starts scan accumulation after prerequisite checks pass.

### `POST /scan/stop`

Stops scan accumulation.

### `POST /scan/reset`

Clears current scan accumulation and statistics.

### `POST /control/target`

Sets the current target velocity and yaw-rate.

Request body:

```json
{
  "velocity": 0.5,
  "yaw_rate": 0.0
}
```

### `POST /control/stop`

Immediately clears the current motion target.

### `POST /control/move`

One-shot duration-based move command retained for compatibility and ad hoc use.

Request body:

```json
{
  "velocity": 0.5,
  "yaw_rate": 0.0,
  "duration": 1.0
}
```

### `POST /path/plan`

Stores manual path nodes. The server does not run obstacle-aware path planning here.

### `POST /map/poi`

Adds a POI.

### `POST /map/save`

Saves the current map as `.slam`.

The saved payload includes:

- occupancy grid
- POI
- path
- pose
- GPS
- chassis
- scan summary
- notes

### `POST /map/load`

Loads a `.slam` file and restores the latest occupancy grid and related map state.

## Run

Install dependencies:

```bash
cd server
python3 -m pip install -r requirements.txt
```

Start the service:

```bash
cd server
python3 run_server.py
```

Default endpoints after startup:

- HTTP: `http://127.0.0.1:8080`
- WS: `ws://127.0.0.1:8080/ws/stream`
