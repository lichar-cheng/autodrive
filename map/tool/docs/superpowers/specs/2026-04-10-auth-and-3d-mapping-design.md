# Auth And 3D Mapping Design

**Goal:** Add configurable authentication modes and a selectable 2D or 3D mapping workflow so the client can authenticate through local or cloud login, then run either a `LaserScan`-based 2D mapping flow or a `PointCloud2`-based 3D mapping flow.

## Scope

This version covers:

- login mode selection
- local and cloud authentication flows
- one unified connection descriptor after authentication
- scan mode selection: `2d` or `3d`
- full 2D and 3D save/load pipelines
- lightweight 3D preview and bounded 3D save size

This version does not cover:

- simultaneous 2D and 3D acquisition
- heavy 3D editing tools
- full raw point cloud archival
- cloud proxy streaming beyond initial authentication and backend resolution

## Current Baseline

Current behavior is:

- direct connection to a fixed backend URL
- no authentication abstraction
- one 2D mapping flow
- `.slam` archive built around 2D occupancy and radar points

## Feature 1: Authentication Modes

### User Requirement

Two login schemes must be supported and configurable:

1. Local login
   - send fixed username and password to a target IP login page
   - receive token and backend `ip + port`
2. Cloud login
   - send username and password to a cloud auth service
   - receive token and backend `ip + port`

Both schemes converge into the same post-login output:

- `backend_host`
- `backend_port`
- `token`
- optional metadata like expiry, user id, mode

### Design

Introduce an auth configuration model:

- `auth_mode = local | cloud`
- `local_auth`
  - login endpoint template
  - fixed username
  - fixed password
- `cloud_auth`
  - auth endpoint
  - entered username
  - entered password

Introduce one normalized connection descriptor:

```json
{
  "auth_mode": "local|cloud",
  "backend_host": "10.0.0.12",
  "backend_port": 8080,
  "token": "abc",
  "expires_at": null
}
```

All downstream connection code uses only this descriptor.

### Client Changes

Desktop and browser clients need:

- login mode selector
- local login form or cloud login form
- login action that resolves the normalized descriptor
- connection logic that derives:
  - HTTP base
  - WS stream URL
  - auth header or token propagation

### Server Assumption

Authentication endpoints are external to the current map server. The mapping backend still serves the same map APIs after the client receives host, port, and token.

## Feature 2: Mapping Mode Selection

### User Requirement

2D and 3D mapping must both be supported, but only one mode is active at a time.

### Design

Introduce:

- `scan_mode = 2d | 3d`

Mode affects:

- ROS subscriptions
- preview processing
- save format contents
- load logic
- UI labels and status

## 2D Flow

The existing 2D `LaserScan` flow stays intact, with current occupancy accumulation and `.slam` save/load behavior preserved.

## 3D Flow

### Source

3D source is ROS `sensor_msgs/PointCloud2`.

### Core Constraint

Raw 3D point clouds are too large to:

- render directly in the client at full rate
- save without aggressive size control

So the system must split 3D handling into three layers.

### 3D Data Layers

1. Raw ingest layer
   - decode `PointCloud2`
   - kept transiently
   - not pushed in full to the UI

2. Preview layer
   - strong downsampling
   - bounded point count
   - optimized for smooth interaction

3. Save layer
   - medium-density voxelized aggregate
   - higher quality than preview
   - much smaller than raw cloud

### Preview Strategy

Recommended first version:

- server decodes `PointCloud2`
- server voxel-downsamples for preview
- preview stream has a hard max point count per frame
- optional rate limiting or keyframe sampling

Preview target:

- browser and desktop must stay interactive
- no full raw cloud rendering in the UI

### Save Strategy

Do not save full raw `PointCloud2` frames.

Instead, save:

- accumulated voxelized cloud
- point statistics
- spatial bounds
- mode metadata
- optional preview/save voxel parameters

This keeps file size bounded and reproducible.

## File Format

### Keep `.slam`

Keep the `.slam` extension, but version the manifest more explicitly by mode.

### 2D Archive

Current core entries:

- `manifest.json`
- `radar_points.bin`

### 3D Archive

Add a 3D mode payload:

- `manifest.json`
- `point_cloud.bin`

`manifest.json` must include:

- `scan_mode`
- `point_count`
- `preview_voxel_size`
- `save_voxel_size`
- coordinate metadata
- pose/path/poi/notes like 2D

`point_cloud.bin` stores bounded voxelized point data, not raw full-fidelity cloud history.

### Manifest Mode Field

Example:

```json
{
  "version": "stcm.v3",
  "scan_mode": "3d"
}
```

## Backend Changes

### ROS Bridge

Add optional `PointCloud2` subscription config:

- `pointcloud`
- `pointcloud_frame`

Bridge responsibilities in 3D mode:

- subscribe `PointCloud2`
- transform into base/world frame
- downsample preview cloud
- aggregate save cloud
- publish lightweight preview topic to clients

### API

Current scan endpoints can remain, but need mode-aware semantics:

- `/scan/start`
- `/scan/stop`
- `/map/save`
- `/map/load`

Payloads and health should expose:

- active `scan_mode`
- 2D or 3D source readiness
- 3D preview/save stats when in 3D mode

## UI Changes

### Authentication UI

Add:

- auth mode selector
- local login panel
- cloud login panel
- resolved backend summary

### Mapping UI

Add:

- scan mode selector
- 3D preview stats
- 3D save size / point count hints

2D and 3D must appear as two parallel, complete user flows, but only one active at a time.

## Performance Constraints

### 3D Preview

Guardrails:

- max preview points per frame
- frame throttling if needed
- downsample before transport to UI

### 3D Save

Guardrails:

- voxel aggregation before persistence
- no raw full-frame history
- bounded binary representation

## Testing

### Auth

Test:

- local login normalization
- cloud login normalization
- connection URL/token propagation

### 2D

Test:

- existing 2D behavior still works
- current `.slam` save/load remains valid

### 3D

Test:

- `PointCloud2` decode path
- preview downsampling
- bounded save cloud generation
- 3D `.slam` save/load
- mode separation between 2D and 3D

## Recommended Implementation Order

1. Auth abstraction and mode selection
2. Unified connection descriptor integration
3. Scan mode abstraction
4. 3D ROS ingest and preview pipeline
5. 3D save/load format
6. UI integration for auth and scan mode
7. performance tuning and limits
