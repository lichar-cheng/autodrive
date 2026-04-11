# Auth And 3D Mapping Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add selectable local or cloud authentication plus a selectable 2D or 3D mapping workflow, including bounded 3D preview and save pipelines.

**Architecture:** Normalize both login modes into a single connection descriptor used by browser and desktop clients. Extend the existing mapping stack with an explicit `scan_mode` split so 2D `LaserScan` and 3D `PointCloud2` use parallel but mode-gated pipelines, including mode-aware save/load and performance-limited 3D preview transport.

**Tech Stack:** Python, JavaScript, FastAPI, ROS2, Tkinter, browser UI, pytest

---

### Task 1: Define shared auth and scan-mode models

**Files:**
- Modify: `server/app/models.py`
- Modify: `server/app/config.py`
- Modify: `client_desktop/logic.py`
- Test: `tests/test_client_desktop_logic.py`

- [ ] **Step 1: Write failing tests for auth descriptor normalization and scan-mode config defaults**

```python
def test_normalize_auth_descriptor_local():
    result = normalize_auth_descriptor("local", {"ip": "10.0.0.2", "port": 8080, "token": "abc"})
    assert result["backend_host"] == "10.0.0.2"
    assert result["backend_port"] == 8080
    assert result["token"] == "abc"

def test_resolve_scan_mode_defaults_to_2d():
    config = resolve_scan_mode_config({})
    assert config["scan_mode"] == "2d"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTHONPATH=. python3 -m pytest -q tests/test_client_desktop_logic.py`
Expected: FAIL with missing auth/scan-mode helpers

- [ ] **Step 3: Write minimal implementation**

Add shared helpers and config structs for:

- auth mode normalization
- backend connection descriptor
- `scan_mode = 2d | 3d`

- [ ] **Step 4: Run test to verify it passes**

Run: `PYTHONPATH=. python3 -m pytest -q tests/test_client_desktop_logic.py`
Expected: PASS

### Task 2: Add local and cloud auth flows to desktop client

**Files:**
- Modify: `client_desktop/app.py`
- Modify: `tests/test_client_desktop_app_helpers.py`

- [ ] **Step 1: Write failing tests for local and cloud auth result handling**

```python
def test_apply_local_auth_result_updates_backend_descriptor():
    ...

def test_apply_cloud_auth_result_updates_backend_descriptor():
    ...
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTHONPATH=. python3 -m pytest -q tests/test_client_desktop_app_helpers.py`
Expected: FAIL with missing auth UI helpers or connection descriptor state

- [ ] **Step 3: Write minimal implementation**

Add:

- auth mode selector
- local auth inputs
- cloud auth inputs
- login action
- descriptor-driven HTTP/WS base derivation
- token propagation in desktop requests

- [ ] **Step 4: Run test to verify it passes**

Run: `PYTHONPATH=. python3 -m pytest -q tests/test_client_desktop_app_helpers.py`
Expected: PASS

### Task 3: Add auth flow to browser client

**Files:**
- Modify: `client/index.html`
- Modify: `client/main.js`

- [ ] **Step 1: Add browser auth state tests or syntax-safe assertions in helper functions**

Create small helper coverage in existing browser logic where practical.

- [ ] **Step 2: Run verification to catch syntax or wiring errors**

Run: `node --check client/main.js`
Expected: PASS after implementation

- [ ] **Step 3: Write minimal implementation**

Add:

- auth mode selector
- local/cloud login fields
- login submit flow
- normalized backend descriptor
- token-aware backend requests

- [ ] **Step 4: Run verification to verify it passes**

Run: `node --check client/main.js`
Expected: PASS

### Task 4: Add scan-mode state and desktop 2D/3D mode gating

**Files:**
- Modify: `client_desktop/app.py`
- Modify: `client_desktop/logic.py`
- Modify: `tests/test_client_desktop_logic.py`
- Modify: `tests/test_client_desktop_app_helpers.py`

- [ ] **Step 1: Write failing tests for scan mode switching**

```python
def test_scan_mode_3d_disables_2d_assumptions():
    ...

def test_scan_mode_2d_preserves_existing_behavior():
    ...
```

- [ ] **Step 2: Run tests to verify failure**

Run: `PYTHONPATH=. python3 -m pytest -q tests/test_client_desktop_logic.py tests/test_client_desktop_app_helpers.py`
Expected: FAIL with missing scan-mode behavior

- [ ] **Step 3: Write minimal implementation**

Add scan mode selector and runtime state so only one of:

- 2D accumulation pipeline
- 3D accumulation pipeline

is active at a time.

- [ ] **Step 4: Run tests to verify pass**

Run: `PYTHONPATH=. python3 -m pytest -q tests/test_client_desktop_logic.py tests/test_client_desktop_app_helpers.py`
Expected: PASS

### Task 5: Add server-side 3D ingest and preview pipeline

**Files:**
- Modify: `server/app/config.py`
- Modify: `server/app/ros_bridge.py`
- Modify: `server/app/main.py`
- Modify: `tests/test_server_ros_bridge.py`
- Modify: `tests/test_server_motion_control.py`

- [ ] **Step 1: Write failing tests for PointCloud2 readiness and preview downsampling**

```python
def test_pointcloud_mode_accepts_pointcloud_topic_and_reports_ready():
    ...

def test_pointcloud_preview_is_bounded():
    ...
```

- [ ] **Step 2: Run tests to verify failure**

Run: `PYTHONPATH=. python3 -m pytest -q tests/test_server_ros_bridge.py tests/test_server_motion_control.py`
Expected: FAIL with missing pointcloud mode support

- [ ] **Step 3: Write minimal implementation**

Add:

- `PointCloud2` topic config
- decode path
- frame transform path
- preview voxel downsampling
- preview point count cap
- mode-aware diagnostics and readiness

- [ ] **Step 4: Run tests to verify pass**

Run: `PYTHONPATH=. python3 -m pytest -q tests/test_server_ros_bridge.py tests/test_server_motion_control.py`
Expected: PASS

### Task 6: Extend `.slam` save/load for 2D and 3D

**Files:**
- Modify: `server/app/stcm_codec.py`
- Modify: `server/app/main.py`
- Modify: `client_desktop/app.py`
- Modify: `client/main.js`
- Create: `tests/test_server_stcm_codec.py`

- [ ] **Step 1: Write failing tests for mode-aware `.slam` save/load**

```python
def test_save_and_load_3d_slam_uses_point_cloud_bin():
    ...

def test_save_and_load_2d_slam_keeps_radar_points_bin():
    ...
```

- [ ] **Step 2: Run tests to verify failure**

Run: `PYTHONPATH=. python3 -m pytest -q tests/test_server_stcm_codec.py`
Expected: FAIL with missing mode-aware archive support

- [ ] **Step 3: Write minimal implementation**

Add:

- manifest `scan_mode`
- `point_cloud.bin` for 3D
- compatibility path for 2D `radar_points.bin`
- desktop/browser save and load branching by mode

- [ ] **Step 4: Run tests to verify pass**

Run: `PYTHONPATH=. python3 -m pytest -q tests/test_server_stcm_codec.py`
Expected: PASS

### Task 7: Add bounded 3D preview and save UX

**Files:**
- Modify: `client/index.html`
- Modify: `client/main.js`
- Modify: `client_desktop/app.py`
- Modify: `docs/product_manual.md`
- Modify: `README.md`

- [ ] **Step 1: Add UI indicators for 3D point count and save limits**
- [ ] **Step 2: Document 2D vs 3D flow selection and 3D limits**
- [ ] **Step 3: Run syntax and regression checks**

Run:

- `node --check client/main.js`
- `PYTHONPATH=. python3 -m pytest -q tests/test_client_desktop_logic.py tests/test_client_desktop_app_helpers.py tests/test_server_ros_bridge.py tests/test_server_motion_control.py tests/test_server_stcm_codec.py`

Expected: PASS

### Task 8: Final verification

**Files:**
- Modify: `README.md`
- Modify: `docs/product_manual.md`

- [ ] **Step 1: Run browser syntax verification**

Run: `node --check client/main.js`
Expected: PASS

- [ ] **Step 2: Run Python test verification**

Run: `PYTHONPATH=. python3 -m pytest -q tests/test_client_desktop_logic.py tests/test_client_desktop_app_helpers.py tests/test_server_ros_bridge.py tests/test_server_motion_control.py tests/test_server_stcm_codec.py`
Expected: PASS

- [ ] **Step 3: Summarize residual risks**

Mention any unverified areas such as live ROS `PointCloud2` integration or external auth endpoint contracts if they were not exercised locally.
