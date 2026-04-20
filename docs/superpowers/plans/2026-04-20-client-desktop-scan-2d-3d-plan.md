# Client Desktop 2D/3D Scan Mode Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add 2D/3D scan mode control, server-side dependency node startup, 3D PCD retrieval on stop, PCD export, and a new `.slam` archive format across the server and desktop client.

**Architecture:** The server becomes the source of truth for scan mode, dependency checks, node launch commands, and 3D PCD retrieval. The desktop client manages operator state, save/load/export, and displays explicit scan lifecycle states while continuing to render the live map from the existing 2D map stream.

**Tech Stack:** FastAPI, Pydantic, Python stdlib zip/struct/pathlib/base64/subprocess, Tkinter desktop UI, pytest

---

## File Map

**Modify:**

- `server/app/config.py`
- `server/app/models.py`
- `server/app/main.py`
- `server/app/stcm_codec.py`
- `client_desktop/app.py`
- `client_desktop/README.md`
- `server/README.md`
- `tests/test_server_motion_control.py`
- `tests/test_client_desktop_app_helpers.py`

**Potentially create if extraction is needed during implementation:**

- `server/app/scan_mode_runtime.py`
- `client_desktop/slam_archive.py`
- `tests/test_server_stcm_codec.py`
- `tests/test_client_desktop_slam_archive.py`

Keep extraction optional. If the current files can absorb the changes cleanly without making them materially worse, prefer modifying the existing files directly.

### Task 1: Add server scan-mode config and request models

**Files:**
- Modify: `server/app/config.py`
- Modify: `server/app/models.py`
- Test: `tests/test_server_motion_control.py`

- [ ] **Step 1: Write the failing tests for scan mode request validation and config defaults**

Add tests that assert:
- a scan start request model accepts `mode="2d"` and `mode="3d"`
- invalid mode is rejected before starting scan
- config exposes `mode_2d` and `mode_3d`
- `mode_3d.pcd_output_path` defaults to empty string

```python
def test_scan_start_request_accepts_2d_and_3d():
    assert StartScanRequest(mode="2d").mode == "2d"
    assert StartScanRequest(mode="3d").mode == "3d"
```

- [ ] **Step 2: Run the targeted tests to verify they fail**

Run: `pytest tests/test_server_motion_control.py -k "scan_mode or start_scan_request" -v`

Expected: FAIL because the new request model and config fields do not exist yet.

- [ ] **Step 3: Implement the minimal config and request models**

Add:
- Pydantic model for scan start request with `mode`
- Pydantic model for scan stop request if separate payload is kept
- scan mode runtime config models in `server/app/config.py`
- defaults for `mode_2d` and `mode_3d`

- [ ] **Step 4: Re-run the targeted tests to verify they pass**

Run: `pytest tests/test_server_motion_control.py -k "scan_mode or start_scan_request" -v`

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add server/app/config.py server/app/models.py tests/test_server_motion_control.py
git commit -m "feat: add server scan mode config models"
```

### Task 2: Add server-side dependency checks and mode-aware scan start/stop

**Files:**
- Modify: `server/app/main.py`
- Modify: `server/app/models.py`
- Test: `tests/test_server_motion_control.py`

- [ ] **Step 1: Write the failing tests for dependency checks and scan lifecycle**

Add tests that assert:
- `/scan/start` with `mode="2d"` fails with `invalid_scan_mode` for bad mode
- `/scan/start` records `scan_mode`
- start rejects when scan is already active
- missing nodes trigger configured launch commands
- launch failure returns `node_start_failed`
- `/scan/stop` rejects when scan is not active

Use monkeypatched helpers to avoid requiring ROS:

```python
def test_start_scan_records_mode_and_activates_bridge(monkeypatch):
    bridge = FakeBridge()
    monkeypatch.setattr(main, "ros", SimpleNamespace(enabled=True, bridge=bridge, reason="ok"))
    monkeypatch.setattr(main, "_resolve_required_nodes", lambda mode: ["/slam_toolbox"])
    monkeypatch.setattr(main, "_check_required_nodes", lambda nodes: {"missing_nodes": [], "required_nodes": nodes, "started_nodes": [], "errors": []})

    result = asyncio.run(main.start_scan(StartScanRequest(mode="2d")))

    assert result["ok"] is True
    assert result["scan_mode"] == "2d"
    assert main.SCAN_SESSION["mode"] == "2d"
```

- [ ] **Step 2: Run the targeted tests to verify they fail**

Run: `pytest tests/test_server_motion_control.py -k "start_scan or stop_scan or dependency" -v`

Expected: FAIL because mode-aware requests and dependency helpers are not implemented.

- [ ] **Step 3: Implement minimal dependency and lifecycle logic**

In `server/app/main.py`:
- extend `SCAN_SESSION` with `mode`, `dependency_status`, `pcd_transfer_state`, `pcd_file`
- add helpers for:
  - loading mode config
  - checking nodes
  - launching missing nodes from config
  - polling for nodes after launch
- update `/scan/start` to accept request body, validate mode, manage dependency status, and store active mode
- update `/scan/stop` to validate active session and include `scan_mode`

Keep subprocess execution isolated in helper functions so tests can monkeypatch them easily.

- [ ] **Step 4: Re-run the targeted tests to verify they pass**

Run: `pytest tests/test_server_motion_control.py -k "start_scan or stop_scan or dependency" -v`

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add server/app/main.py server/app/models.py tests/test_server_motion_control.py
git commit -m "feat: add mode-aware scan lifecycle on server"
```

### Task 3: Add 3D PCD retrieval and server health/status fields

**Files:**
- Modify: `server/app/main.py`
- Modify: `tests/test_server_motion_control.py`

- [ ] **Step 1: Write the failing tests for 3D stop PCD retrieval**

Add tests that assert:
- 3D stop returns `pcd_file` when configured file exists
- stop returns `pcd_path_not_configured` when 3D config lacks `pcd_output_path`
- stop returns `pcd_file_missing` when the configured file is absent
- `/health` includes `scan_mode`, `dependency_status`, and `pcd_transfer_state`

```python
def test_stop_scan_3d_returns_base64_pcd_when_file_exists(tmp_path, monkeypatch):
    pcd = tmp_path / "map.pcd"
    pcd.write_bytes(b"pcd-bytes")
    main.SCAN_SESSION["active"] = True
    main.SCAN_SESSION["mode"] = "3d"
    monkeypatch.setattr(main, "_pcd_output_path_for_mode", lambda mode: pcd)

    result = asyncio.run(main.stop_scan(StopScanRequest(mode="3d")))

    assert result["ok"] is True
    assert result["pcd_file"]["name"] == "map.pcd"
```

- [ ] **Step 2: Run the targeted tests to verify they fail**

Run: `pytest tests/test_server_motion_control.py -k "pcd or health" -v`

Expected: FAIL because stop currently has no PCD retrieval or extended health status.

- [ ] **Step 3: Implement minimal PCD retrieval and status fields**

In `server/app/main.py`:
- add helper to resolve `pcd_output_path` for 3D mode
- read the file on 3D stop
- base64-encode file content in the response
- set and clear `pcd_transfer_state` around file read
- include `scan_mode`, `dependency_status`, `pcd_transfer_state`, and `pcd_metadata` in `/health`

- [ ] **Step 4: Re-run the targeted tests to verify they pass**

Run: `pytest tests/test_server_motion_control.py -k "pcd or health" -v`

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add server/app/main.py tests/test_server_motion_control.py
git commit -m "feat: return 3d pcd on scan stop"
```

### Task 4: Replace the archive codec with the new `.slam` format

**Files:**
- Modify: `server/app/stcm_codec.py`
- Modify: `server/app/main.py`
- Modify: `client_desktop/app.py`
- Test: `tests/test_server_motion_control.py`
- Test: `tests/test_client_desktop_app_helpers.py`

- [ ] **Step 1: Write the failing tests for new archive layout**

Add tests that assert:
- server-side save writes `manifest.json` and `map_points.bin`
- 3D save writes `map.pcd` when the bundle contains it
- load restores `pcd` bytes when `map.pcd` exists
- client-side load/save helpers can round-trip the new file layout

If the codec logic becomes unwieldy inside the existing modules, extract helper functions and test them directly.

```python
def test_save_slam_writes_optional_pcd(tmp_path):
    target = tmp_path / "demo.slam"
    save_stcm(target, {"scan_mode": "3d", "radar_points": [(1.0, 2.0, 3.0)], "pcd_file": {"name": "map.pcd", "content": b"pcd"}})
    ...
```

- [ ] **Step 2: Run the targeted tests to verify they fail**

Run: `pytest tests/test_server_motion_control.py -k "slam or save_map" -v`

Run: `pytest tests/test_client_desktop_app_helpers.py -k "slam or pcd" -v`

Expected: FAIL because the current codec still uses `radar_points.bin` and has no optional PCD entry.

- [ ] **Step 3: Implement the minimal new archive codec**

Update archive behavior to:
- write `manifest.json`
- write `map_points.bin`
- optionally write `map.pcd`
- load optional `map.pcd` and expose it in a consistent bundle field
- update `/map/save` and `/map/load` to use the new archive bundle fields

Only rename `save_stcm` and `load_stcm` if the old names become materially misleading during implementation. If renamed, update all call sites and tests in the same task.

- [ ] **Step 4: Re-run the targeted tests to verify they pass**

Run: `pytest tests/test_server_motion_control.py -k "slam or save_map" -v`

Run: `pytest tests/test_client_desktop_app_helpers.py -k "slam or pcd" -v`

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add server/app/stcm_codec.py server/app/main.py client_desktop/app.py tests/test_server_motion_control.py tests/test_client_desktop_app_helpers.py
git commit -m "feat: adopt new slam archive format"
```

### Task 5: Add desktop scan mode state and start/stop flow

**Files:**
- Modify: `client_desktop/app.py`
- Test: `tests/test_client_desktop_app_helpers.py`

- [ ] **Step 1: Write the failing tests for desktop scan phase state**

Add tests that assert:
- the client sends `{"mode": "2d"}` or `{"mode": "3d"}` to `/scan/start`
- successful 2D start sets `scan_phase="scanning"`
- 3D stop enters a receiving phase before storing PCD payload
- server error reasons set persistent status text

Use lightweight fake client instances created via `DesktopClient.__new__(DesktopClient)` with monkeypatched `call_api`, `sync_scan_badges`, and Tk variables.

```python
def test_start_scan_sends_selected_mode(monkeypatch):
    client = DesktopClient.__new__(DesktopClient)
    client.scan = {"mode": "3d", "active": False}
    ...
```

- [ ] **Step 2: Run the targeted tests to verify they fail**

Run: `pytest tests/test_client_desktop_app_helpers.py -k "start_scan or stop_scan or scan_phase" -v`

Expected: FAIL because the desktop client has no explicit mode/phase model yet.

- [ ] **Step 3: Implement minimal desktop scan state and requests**

In `client_desktop/app.py`:
- extend client scan state with mode, phase, error text, and PCD storage
- update start/stop methods to send mode-aware API payloads
- add helper methods for status transitions
- store returned `pcd_file` after a 3D stop
- keep the current canvas accumulation and map rendering intact

- [ ] **Step 4: Re-run the targeted tests to verify they pass**

Run: `pytest tests/test_client_desktop_app_helpers.py -k "start_scan or stop_scan or scan_phase" -v`

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add client_desktop/app.py tests/test_client_desktop_app_helpers.py
git commit -m "feat: add desktop 2d 3d scan state"
```

### Task 6: Add desktop UI controls and user-facing error/status handling

**Files:**
- Modify: `client_desktop/app.py`
- Test: `tests/test_client_desktop_app_helpers.py`

- [ ] **Step 1: Write the failing tests for mode selection and error presentation**

Add tests that assert:
- scan mode selector updates stored mode
- error reason text persists after a failed start or stop
- 3D stop with returned PCD updates the visible status text to reflect receiving/success

Prefer unit tests around helper methods that compute or assign status text instead of brittle full-widget tests.

- [ ] **Step 2: Run the targeted tests to verify they fail**

Run: `pytest tests/test_client_desktop_app_helpers.py -k "mode selector or error status or pcd status" -v`

Expected: FAIL

- [ ] **Step 3: Implement minimal UI and status text updates**

In `client_desktop/app.py`:
- add `2D/3D` mode selector controls near the scan buttons
- wire selection into scan state
- update badge or status text generation to include:
  - checking dependency nodes
  - starting dependency nodes
  - scanning
  - receiving PCD
  - last error
- keep current localization style intact when adding strings

- [ ] **Step 4: Re-run the targeted tests to verify they pass**

Run: `pytest tests/test_client_desktop_app_helpers.py -k "mode selector or error status or pcd status" -v`

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add client_desktop/app.py tests/test_client_desktop_app_helpers.py
git commit -m "feat: add scan mode ui and status messaging"
```

### Task 7: Add desktop PCD export and load/save integration

**Files:**
- Modify: `client_desktop/app.py`
- Test: `tests/test_client_desktop_app_helpers.py`

- [ ] **Step 1: Write the failing tests for PCD export and `.slam` round-trip**

Add tests that assert:
- exporting PCD warns when no PCD exists
- exporting PCD writes the exact loaded or received bytes
- saving a 3D session includes `map.pcd`
- loading a `.slam` containing `map.pcd` restores it for export

- [ ] **Step 2: Run the targeted tests to verify they fail**

Run: `pytest tests/test_client_desktop_app_helpers.py -k "export_pcd or load_slam or save_slam" -v`

Expected: FAIL because the desktop client has no PCD export path yet.

- [ ] **Step 3: Implement minimal PCD export and archive integration**

In `client_desktop/app.py`:
- add `PCD` export action beside existing exports
- include optional PCD payload in save bundle
- restore optional PCD bytes on load
- keep inspector state coherent for both 2D-only and 3D sessions

- [ ] **Step 4: Re-run the targeted tests to verify they pass**

Run: `pytest tests/test_client_desktop_app_helpers.py -k "export_pcd or load_slam or save_slam" -v`

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add client_desktop/app.py tests/test_client_desktop_app_helpers.py
git commit -m "feat: add desktop pcd export and slam integration"
```

### Task 8: Update docs and run final verification

**Files:**
- Modify: `client_desktop/README.md`
- Modify: `server/README.md`
- Modify: any touched file if verification exposes defects

- [ ] **Step 1: Write the failing documentation checks**

This task does not need a code-level failing test. Instead, define the verification target first:
- README must mention 2D/3D scan modes
- README must mention server-side node startup config
- README must mention 3D stop PCD retrieval and PCD export
- README must describe the new `.slam` layout at a high level

- [ ] **Step 2: Update the documentation**

Revise:
- `client_desktop/README.md`
- `server/README.md`

Keep examples aligned with the actual request and response fields implemented earlier.

- [ ] **Step 3: Run the focused automated test suite**

Run:

```bash
pytest tests/test_server_motion_control.py tests/test_client_desktop_app_helpers.py -v
```

Expected: PASS

- [ ] **Step 4: Run a broader sanity pass if the focused suite passes**

Run:

```bash
pytest tests/test_server_ros_bridge.py tests/test_client_desktop_run_client.py -v
```

Expected: PASS, or document any unrelated failures without masking them.

- [ ] **Step 5: Commit**

```bash
git add client_desktop/README.md server/README.md server/app/config.py server/app/models.py server/app/main.py server/app/stcm_codec.py client_desktop/app.py tests/test_server_motion_control.py tests/test_client_desktop_app_helpers.py
git commit -m "feat: add 2d 3d scan workflow"
```

## Review Notes

- This plan is based on [2026-04-20-client-desktop-scan-2d-3d-design.md](/home/autodrive2/autodrive/docs/superpowers/specs/2026-04-20-client-desktop-scan-2d-3d-design.md).
- The normal subagent plan review loop was not run because this session does not have permission to spawn subagents unless the user explicitly requests delegation.
- During execution, prefer monkeypatched helper tests over real ROS process invocation.
- Do not revert unrelated worktree changes; this repository already contains unrelated local modifications.
