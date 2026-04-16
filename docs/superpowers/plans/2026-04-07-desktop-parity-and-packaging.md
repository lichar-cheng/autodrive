# Desktop Parity And Packaging Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Bring `client_desktop` to functional parity with the browser client, add Linux packaging, remove stale `.txt` temporary copies, and update docs to match the shipped behavior.

**Architecture:** Keep the Tkinter desktop client as the shipped UI, but port the browser-only behavior into a small shared Python helper module so desktop path planning, POI parsing, copy/export, and STCM metadata behave like the web client. Packaging stays PyInstaller-based, with separate Windows and Linux entry scripts/documentation.

**Tech Stack:** Python 3, Tkinter, requests, websocket-client, PyInstaller, pytest

---

### Task 1: Create shared parity helpers

**Files:**
- Create: `client_desktop/logic.py`
- Test: `tests/test_client_desktop_logic.py`

- [ ] **Step 1: Write the failing tests**

```python
from client_desktop.logic import parse_batch_poi_text, build_poi_copy_text, build_auto_loop_segments


def test_parse_batch_poi_text_supports_name_geo_and_yaw():
    rows = parse_batch_poi_text("a\nb,120.1,30.2\nc,120.2,30.3,1.57")
    assert [row["name"] for row in rows] == ["a", "b", "c"]
    assert rows[1]["lon"] == 120.1
    assert rows[2]["yaw"] == 1.57
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_client_desktop_logic.py -v`
Expected: FAIL with import or missing function errors

- [ ] **Step 3: Write minimal implementation**

```python
def parse_batch_poi_text(text: str) -> list[dict]:
    ...
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_client_desktop_logic.py -v`
Expected: PASS

### Task 2: Port desktop path and POI behavior

**Files:**
- Modify: `client_desktop/app.py`
- Modify: `client_desktop/requirements.txt`
- Test: `tests/test_client_desktop_logic.py`

- [ ] **Step 1: Write failing tests for path planning and validation helpers**
- [ ] **Step 2: Run `python3 -m pytest tests/test_client_desktop_logic.py -v` and confirm the failures**
- [ ] **Step 3: Update desktop client to use shared helpers for auto-loop, obstacle-aware segment planning, POI batch parsing, POI copy text, idle mode, selection clearing, notes persistence, and richer path metadata**
- [ ] **Step 4: Run `python3 -m pytest tests/test_client_desktop_logic.py -v` and confirm pass**

### Task 3: Add desktop-only UI parity features

**Files:**
- Modify: `client_desktop/app.py`
- Test: `tests/test_client_desktop_logic.py`

- [ ] **Step 1: Write failing tests for export/diagnostic helper behavior where practical**
- [ ] **Step 2: Run targeted tests and confirm failure**
- [ ] **Step 3: Add UI support for camera refresh snapshots, panel visibility toggles, communication diagnostics, STCM inspector/export metadata, and Linux-safe save/export behavior**
- [ ] **Step 4: Re-run targeted tests**

### Task 4: Add Linux packaging and clean stale files

**Files:**
- Create: `client_desktop/build_linux.sh`
- Modify: `client_desktop/build_windows.bat`
- Delete: `client/trajectory_service.py.txt`
- Delete: `server/app/main3.py.txt`
- Delete: `server/app/ros_bridge.py.txt`
- Delete: `server/app/ros_bridge3.py.txt`
- Modify: `client_desktop/README.md`
- Modify: `README.md`

- [ ] **Step 1: Add packaging scripts for Windows and Linux with shared PyInstaller assumptions**
- [ ] **Step 2: Remove stale `.txt` temporary files and bad references**
- [ ] **Step 3: Update README files so structure, functionality, and packaging instructions match the code**

### Task 5: Verify end-to-end

**Files:**
- Modify: `docs/superpowers/plans/2026-04-07-desktop-parity-and-packaging.md`

- [ ] **Step 1: Run `python3 -m pytest tests/test_client_desktop_logic.py -v`**
- [ ] **Step 2: Run `python3 -m py_compile client_desktop/app.py client_desktop/logic.py client/trajectory_service.py`**
- [ ] **Step 3: Report any remaining gaps honestly in the final handoff**
