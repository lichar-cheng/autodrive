# Client Desktop 2D/3D Scan Mode Design

**Goal:** Add explicit `2d` and `3d` scan modes to the desktop client and server, including ROS dependency node checks, server-side launch commands from config, 3D PCD retrieval on stop, PCD export, and a new unified `.slam` archive format.

**Scope:** This design covers server config, scan lifecycle APIs, client state and UI changes, `.slam` archive structure, error handling, and test strategy. It replaces the current single-mode scan behavior and does not preserve backward compatibility with the existing `.slam` layout.

## Current Baseline

- [client_desktop/app.py](/home/autodrive2/autodrive/client_desktop/app.py) currently exposes a single `start_scan()` and `stop_scan()` flow.
- The desktop client saves `.slam` as a zip archive with `manifest.json` and `radar_points.bin`.
- Map export currently supports `PGM`, `YAML`, and `JSON`, but not `PCD`.
- [server/app/main.py](/home/autodrive2/autodrive/server/app/main.py) exposes `/scan/start`, `/scan/stop`, and `/map/save`, but does not manage external ROS mapping dependencies by mode.
- [server/app/config.py](/home/autodrive2/autodrive/server/app/config.py) has ROS topic and bridge defaults, but no per-mode launch configuration.

## Functional Requirements

### 2D mode

- The desktop client must let the user choose `2D` mode before starting a scan.
- On start, the server must verify that the configured 2D dependency nodes exist.
- If required 2D nodes do not exist, the server must attempt to start them using configured launch commands.
- If node checks or startup fail, the server must return a structured error and the client must show it.
- Once started, the map view continues to use the current `/map` synchronization flow and behaves the same as the existing 2D scan view.

### 3D mode

- The desktop client must let the user choose `3D` mode before starting a scan.
- On start, the server must verify that the configured 3D dependency nodes exist.
- If required 3D nodes do not exist, the server must attempt to start them using configured launch commands.
- During scanning, the desktop map view remains the same as 2D mode and continues to use `/map` static map data.
- On stop, the server must retrieve the configured Point-LIO `pcd` file and return it to the client.
- While the 3D `pcd` file is being transferred, the client must show a receiving state.
- If the `pcd` file is missing, unreadable, or fails to transfer, the user must receive an explicit error.

### Save and export

- The save action must write a new `.slam` archive format for both 2D and 3D sessions.
- The new `.slam` format must include current 2D occupancy/map data.
- In 3D mode, the new `.slam` format must also include the retrieved `pcd` file.
- The desktop map tooling must support exporting `pcd` when the current in-memory session or loaded `.slam` contains it.

## Architecture

The server becomes the source of truth for scan mode, dependency checking, node startup, and 3D `pcd` retrieval. The desktop client remains responsible for operator workflow, local state display, archive save/load, and local export.

This keeps all ROS execution and dependency error handling on the server, while letting the client work with a uniform session payload regardless of whether the map data came from a live scan or a previously loaded `.slam`.

## Server Configuration

[server/app/config.py](/home/autodrive2/autodrive/server/app/config.py) gains explicit per-mode scan configuration.

Recommended models:

```python
class ScanModeRuntimeConfig(BaseModel):
    required_nodes: list[str] = Field(default_factory=list)
    launch_commands: list[list[str]] = Field(default_factory=list)
    pcd_output_path: str = ""


class ScanModesConfig(BaseModel):
    mode_2d: ScanModeRuntimeConfig = Field(default_factory=ScanModeRuntimeConfig)
    mode_3d: ScanModeRuntimeConfig = Field(default_factory=ScanModeRuntimeConfig)
```

`launch_commands` are stored as `list[list[str]]` instead of raw shell strings so the server can execute them without shell parsing ambiguity and can report which exact argv failed.

Recommended defaults:

- `mode_2d.required_nodes`: the 2D mapping nodes that must exist before scan start
- `mode_2d.launch_commands`: commands such as `["ros2", "launch", "caddie_hardware", "navigation_slam_based.launch.py"]`
- `mode_3d.required_nodes`: the 3D mapping nodes that must exist before scan start
- `mode_3d.launch_commands`: commands such as `["ros2", "launch", "slam_toolbox", "slam_toobox.launch.py"]`
- `mode_3d.pcd_output_path`: fixed filesystem path for the Point-LIO output file

## Server Runtime State

[server/app/main.py](/home/autodrive2/autodrive/server/app/main.py) must track:

- `scan_mode`: `2d` or `3d`
- `scan_active`
- `dependency_status`
- `pcd_transfer_state`
- `pcd_metadata`

The existing scan session dictionary can be extended rather than replaced if that keeps the current code localized.

`dependency_status` should include:

- `required_nodes`
- `missing_nodes`
- `started_nodes`
- `errors`

`pcd_transfer_state` should include one of:

- `idle`
- `pending`
- `reading`
- `ready`
- `error`

## Node Check and Startup Flow

### `POST /scan/start`

Request:

```json
{
  "mode": "2d"
}
```

Server flow:

1. Validate `mode` against `2d` and `3d`.
2. Reject if a scan is already active.
3. Load the matching mode config from `CONFIG`.
4. Check whether every configured `required_node` is present.
5. If nodes are missing, run each configured launch command in order.
6. After each launch attempt, poll for node existence with bounded retries.
7. If required nodes still do not exist, return a structured failure.
8. If dependencies are ready, reset the current scan session, store `scan_mode`, and enter the existing scan activation path.

Success response shape:

```json
{
  "ok": true,
  "scan_active": true,
  "scan_mode": "2d",
  "scan_summary": {},
  "dependency_status": {
    "required_nodes": ["/slam_toolbox"],
    "missing_nodes": [],
    "started_nodes": [],
    "errors": []
  }
}
```

Failure response shape:

```json
{
  "ok": false,
  "reason": "node_start_failed",
  "scan_active": false,
  "scan_mode": "3d",
  "dependency_status": {
    "required_nodes": ["/point_lio"],
    "missing_nodes": ["/point_lio"],
    "started_nodes": [],
    "errors": ["failed to launch ['ros2', 'launch', '...']"]
  }
}
```

### `POST /scan/stop`

Request:

```json
{
  "mode": "3d"
}
```

Server flow:

1. Reject if no scan is active.
2. Stop the active scan using the existing logic.
3. If `scan_mode == "2d"`, return the normal stop payload.
4. If `scan_mode == "3d"`, look up `pcd_output_path` from config.
5. Validate that `pcd_output_path` is configured and the file exists.
6. Read the `pcd` file and return it as encoded payload plus metadata.
7. Set `pcd_transfer_state` so `/health` and the client UI can show transfer progress.

2D success response:

```json
{
  "ok": true,
  "scan_active": false,
  "scan_mode": "2d",
  "scan_summary": {}
}
```

3D success response:

```json
{
  "ok": true,
  "scan_active": false,
  "scan_mode": "3d",
  "scan_summary": {},
  "pcd_file": {
    "name": "map.pcd",
    "size": 123456,
    "encoding": "base64",
    "content": "..."
  }
}
```

3D failure response:

```json
{
  "ok": false,
  "reason": "pcd_file_missing",
  "scan_active": false,
  "scan_mode": "3d",
  "error": "pcd file not found: /path/to/file"
}
```

## Health and Status Reporting

The server should extend `/health` or the mapping summary payload to include:

- `scan_mode`
- `dependency_status`
- `pcd_transfer_state`
- `pcd_metadata`

This lets the desktop client reconnect cleanly and redraw the current operator state without guessing from stale local flags.

## Client State Model

[client_desktop/app.py](/home/autodrive2/autodrive/client_desktop/app.py) should move from a single `scan["active"]` flag to an explicit state model.

Minimum new fields:

- `scan_mode`: `2d | 3d`
- `scan_active`: `bool`
- `scan_phase`: `idle | starting | scanning | stopping | receiving_pcd | error`
- `scan_error`: latest user-visible error text
- `pcd_name`
- `pcd_bytes`
- `pcd_received_at`

The existing `scan` dictionary can be extended if that is less invasive than introducing a new model object.

## Client UI Behavior

The desktop scan controls should add:

- mode selector for `2D` and `3D`
- start action
- stop action

Button/state rules:

- start enabled in `idle` and recoverable `error`
- stop enabled while `starting`, `scanning`, or `stopping`
- 3D stop transitions into `receiving_pcd` until the response is fully handled

Status text must explicitly cover:

- checking dependency nodes
- starting dependency nodes
- scan running
- receiving PCD
- last failure reason

All service errors must surface both as a popup and as status text, so the user can still see the last failure after dismissing a dialog.

## Map Rendering Behavior

The scan canvas and map view stay aligned with the current 2D behavior in both scan modes.

- 2D mode continues to use the current `/map` data path.
- 3D mode also keeps using `/map` for the live map view.
- No separate 3D point cloud renderer is introduced in this change.

This keeps the operator experience stable during scan collection and limits the new work to dependency management, session storage, and export.

## New `.slam` Archive Format

Backward compatibility is explicitly not required.

The new `.slam` remains a zip archive, but uses a new layout:

- `manifest.json`
- `map_points.bin`
- `map.pcd` only when 3D data exists

`manifest.json` must include:

```json
{
  "version": "slam.v3",
  "scan_mode": "3d",
  "map_source": "server_occupancy_grid",
  "created_at": 1760000000,
  "notes": "...",
  "pose": {},
  "gps": {},
  "chassis": {},
  "scan_summary": {},
  "poi": [],
  "path": [],
  "occupancy": {
    "voxel_size": 0.1,
    "occupied_cells": [],
    "free_cells": []
  },
  "pcd": {
    "included": true,
    "file": "map.pcd"
  }
}
```

Rules:

- 2D save writes `manifest.json` and `map_points.bin`
- 3D save writes `manifest.json`, `map_points.bin`, and `map.pcd`
- load restores 2D occupancy/map data, POI, path, notes, and optional `pcd`
- the current session should keep optional `pcd` bytes in memory after a 3D stop or after loading a `.slam` containing `map.pcd`

## Save and Export Behavior

The desktop save flow in [client_desktop/app.py](/home/autodrive2/autodrive/client_desktop/app.py) should be updated so:

- save always writes the new `.slam` layout
- save includes current 2D occupancy/map data regardless of mode
- save includes `pcd` only when the session has it

The desktop export tooling should be extended with `PCD` export:

- if the active in-memory session contains `pcd`, export writes it directly
- if the currently loaded `.slam` contains `pcd`, export writes the stored `pcd`
- if no `pcd` exists, the client should show a user-facing warning instead of creating an empty file

No additional server export endpoint is required because the `pcd` is already transferred to the desktop during the 3D stop flow.

## Error Model

The server should standardize these error reasons so the desktop can map them to clear messages:

- `invalid_scan_mode`
- `scan_already_active`
- `scan_not_active`
- `node_missing`
- `node_start_failed`
- `node_check_failed`
- `scan_stop_failed`
- `pcd_path_not_configured`
- `pcd_file_missing`
- `pcd_read_failed`
- `pcd_transfer_failed`
- `slam_save_failed`
- `slam_load_failed`
- `pcd_export_unavailable`

The client should not infer causes from HTTP failures alone when a structured server response exists. It should prefer the explicit `reason` and any attached detail message.

## Testing Strategy

### Server tests

The server test suite should cover:

- 2D start when required nodes already exist
- 2D start when nodes are missing but launch succeeds
- 2D start when node launch fails
- 3D start when required nodes already exist
- 3D stop when `pcd` file exists and is returned
- 3D stop when `pcd_output_path` is missing from config
- 3D stop when `pcd` file does not exist
- `.slam` save metadata for 2D without `pcd`
- `.slam` save metadata for 3D with `pcd`

### Client tests

The desktop test suite should cover:

- mode selector drives the `mode` value sent to `/scan/start`
- successful 2D start updates local mode and scan phase
- successful 3D stop enters `receiving_pcd` and stores returned `pcd`
- server-side error reasons surface as popup and persisted status text
- saving a 2D session creates a `.slam` without `map.pcd`
- saving a 3D session creates a `.slam` with `map.pcd`
- loading a `.slam` with `map.pcd` restores `pcd` for later export
- PCD export fails cleanly when no `pcd` is present

## Migration Notes

- Existing `.slam` files are out of scope and do not need migration support.
- Existing save/load code, helper names, and tests can be renamed away from `stcm` semantics where that reduces confusion.
- Existing map rendering and occupancy accumulation behavior should be preserved unless required to support the new archive structure.

## Review Notes

This spec was reviewed manually in-session. The normal subagent spec review loop described by the brainstorming workflow could not be used here because this session does not have permission to delegate work unless the user explicitly requests subagents.
