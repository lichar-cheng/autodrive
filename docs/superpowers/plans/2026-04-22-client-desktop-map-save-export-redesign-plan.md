# Client Desktop Map Save/Export Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Redesign `client_desktop` map save/export so `.slam` remains the editable project format, 3D saves default to `2D only`, and a new `Export ZIP` produces deployment-oriented `pgm + yaml + json`.

**Architecture:** Keep the implementation centered in [client_desktop/app.py](/home/autodrive2/autodrive/client_desktop/app.py), reusing the existing manifest, archive, and export generation helpers. Add a small 3D save-choice UI branch, extend archive metadata minimally, and package current export artifacts into a dedicated zip flow without changing the server contract.

**Tech Stack:** Python 3, Tkinter, `zipfile`, existing `.slam` zip format helpers, `pytest`

---

## File Structure

### Existing files to modify

- Modify: [client_desktop/app.py](/home/autodrive2/autodrive/client_desktop/app.py)
  - keep `.slam` save logic
  - add 3D save payload choice helper
  - add `Export ZIP` action
  - extend archive metadata minimally
  - wire UI labels and export buttons
- Modify: [tests/test_client_desktop_app_helpers.py](/home/autodrive2/autodrive/tests/test_client_desktop_app_helpers.py)
  - add regression coverage for 2D save, 3D save default, 3D save with PCD, and export zip behavior

### Files to reference while implementing

- Reference: [docs/superpowers/specs/2026-04-22-client-desktop-map-save-export-redesign-design.md](/home/autodrive2/autodrive/docs/superpowers/specs/2026-04-22-client-desktop-map-save-export-redesign-design.md)
- Reference: [client_desktop/README.md](/home/autodrive2/autodrive/client_desktop/README.md)

## Task 1: Add Save-Payload Selection Helper

**Files:**
- Modify: [client_desktop/app.py](/home/autodrive2/autodrive/client_desktop/app.py)
- Test: [tests/test_client_desktop_app_helpers.py](/home/autodrive2/autodrive/tests/test_client_desktop_app_helpers.py)

- [ ] **Step 1: Write the failing test for 3D default save payload**

```python
def test_choose_3d_save_payload_defaults_to_2d_only(monkeypatch) -> None:
    client = build_minimal_client()
    client.root = None

    payload = DesktopClient.choose_3d_save_payload(client)

    assert payload == "2d_only"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTHONPATH=. pytest tests/test_client_desktop_app_helpers.py -k 'choose_3d_save_payload_defaults_to_2d_only' -q`
Expected: FAIL with missing helper or missing behavior

- [ ] **Step 3: Write minimal implementation**

Add a focused helper in `DesktopClient`, for example:

```python
def choose_3d_save_payload(self) -> str:
    return "2d_only"
```

Keep this first pass minimal so the initial test is green before wiring the dialog.

- [ ] **Step 4: Run test to verify it passes**

Run: `PYTHONPATH=. pytest tests/test_client_desktop_app_helpers.py -k 'choose_3d_save_payload_defaults_to_2d_only' -q`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add client_desktop/app.py tests/test_client_desktop_app_helpers.py
git commit -m "test: add default 3d save payload helper"
```

## Task 2: Replace 3D Save Flow With Explicit Payload Modes

**Files:**
- Modify: [client_desktop/app.py](/home/autodrive2/autodrive/client_desktop/app.py)
- Test: [tests/test_client_desktop_app_helpers.py](/home/autodrive2/autodrive/tests/test_client_desktop_app_helpers.py)

- [ ] **Step 1: Write the failing tests for save behavior**

Add tests for:

```python
def test_save_stcm_3d_default_uses_2d_only_without_requesting_pcd(tmp_path: Path, monkeypatch) -> None:
    ...
    client.scan["mode"] = "3d"
    client.choose_3d_save_payload = lambda: "2d_only"
    client.ensure_scan_pcd = lambda mode=None: (_ for _ in ()).throw(AssertionError("pcd should not be requested"))
    ...
    DesktopClient.save_stcm(client)
    assert writes[0][3] is None
    assert writes[0][1]["save_payload"] == "2d_only"


def test_save_stcm_3d_with_pcd_requests_and_writes_pcd(tmp_path: Path, monkeypatch) -> None:
    ...
    client.scan["mode"] = "3d"
    client.choose_3d_save_payload = lambda: "2d_pcd"
    client.ensure_scan_pcd = lambda mode=None: {"name": "map.pcd", "content": b"pcd-bytes"}
    ...
    DesktopClient.save_stcm(client)
    assert writes[0][3]["name"] == "map.pcd"
    assert writes[0][1]["save_payload"] == "2d_pcd"


def test_save_stcm_3d_with_pcd_aborts_when_pcd_missing(tmp_path: Path, monkeypatch) -> None:
    ...
    client.scan["mode"] = "3d"
    client.choose_3d_save_payload = lambda: "2d_pcd"
    client.ensure_scan_pcd = lambda mode=None: None
    ...
    DesktopClient.save_stcm(client)
    assert not writes
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `PYTHONPATH=. pytest tests/test_client_desktop_app_helpers.py -k 'save_stcm_3d_default_uses_2d_only_without_requesting_pcd or save_stcm_3d_with_pcd_requests_and_writes_pcd or save_stcm_3d_with_pcd_aborts_when_pcd_missing' -q`
Expected: FAIL because `save_stcm()` still derives behavior from the current scan mode only

- [ ] **Step 3: Write minimal implementation**

Update `save_stcm()` to:

- branch only when `current_scan_mode == "3d"`
- call `choose_3d_save_payload()`
- map payload choice to:
  - `2d_only` -> skip `ensure_scan_pcd()`
  - `2d_pcd` -> require `ensure_scan_pcd("3d")`
- write minimal manifest metadata:

```python
bundle["save_payload"] = payload_choice
bundle["pcd"] = {"included": bool(pcd_file), "file": "..."}
```

Keep the existing 2D path unchanged.

- [ ] **Step 4: Run tests to verify they pass**

Run: `PYTHONPATH=. pytest tests/test_client_desktop_app_helpers.py -k 'save_stcm_3d_default_uses_2d_only_without_requesting_pcd or save_stcm_3d_with_pcd_requests_and_writes_pcd or save_stcm_3d_with_pcd_aborts_when_pcd_missing' -q`
Expected: PASS

- [ ] **Step 5: Run related regression tests**

Run: `PYTHONPATH=. pytest tests/test_client_desktop_app_helpers.py -k 'save_stcm or ensure_scan_pcd or export_pcd' -q`
Expected: PASS with no regressions in existing save/export flows

- [ ] **Step 6: Commit**

```bash
git add client_desktop/app.py tests/test_client_desktop_app_helpers.py
git commit -m "feat: add 3d save payload modes"
```

## Task 3: Implement the Actual 3D Save Choice Dialog

**Files:**
- Modify: [client_desktop/app.py](/home/autodrive2/autodrive/client_desktop/app.py)
- Test: [tests/test_client_desktop_app_helpers.py](/home/autodrive2/autodrive/tests/test_client_desktop_app_helpers.py)

- [ ] **Step 1: Write the failing test for the dialog fallback contract**

Use a minimal logic-level test that asserts the helper returns one of the two supported values and defaults cleanly when UI is unavailable:

```python
def test_choose_3d_save_payload_returns_supported_value_without_ui() -> None:
    client = build_minimal_client()
    client.root = None

    result = DesktopClient.choose_3d_save_payload(client)

    assert result in {"2d_only", "2d_pcd"}
    assert result == "2d_only"
```

- [ ] **Step 2: Run test to verify expected behavior**

Run: `PYTHONPATH=. pytest tests/test_client_desktop_app_helpers.py -k 'choose_3d_save_payload_returns_supported_value_without_ui' -q`
Expected: PASS or adjust helper until it passes cleanly

- [ ] **Step 3: Implement the Tkinter choice dialog**

Add a small modal helper that:

- shows only in 3D save flow
- presents:
  - `2D only`
  - `2D + PCD`
- defaults to `2D only`
- returns `None` if the user cancels

Keep the method isolated so the rest of `save_stcm()` stays readable.

- [ ] **Step 4: Update `save_stcm()` to respect cancel**

If the dialog returns `None`, exit without writing files.

- [ ] **Step 5: Run focused regression tests**

Run: `PYTHONPATH=. pytest tests/test_client_desktop_app_helpers.py -k 'choose_3d_save_payload or save_stcm_3d' -q`
Expected: PASS

- [ ] **Step 6: Commit**

```bash
git add client_desktop/app.py tests/test_client_desktop_app_helpers.py
git commit -m "feat: add 3d save choice dialog"
```

## Task 4: Add Export ZIP Packaging

**Files:**
- Modify: [client_desktop/app.py](/home/autodrive2/autodrive/client_desktop/app.py)
- Test: [tests/test_client_desktop_app_helpers.py](/home/autodrive2/autodrive/tests/test_client_desktop_app_helpers.py)

- [ ] **Step 1: Write the failing test for export zip contents**

Add a focused export test:

```python
def test_export_zip_writes_pgm_yaml_json_only(tmp_path: Path, monkeypatch) -> None:
    target = tmp_path / "map_export.zip"
    monkeypatch.setattr("client_desktop.app.filedialog.asksaveasfilename", lambda **_kwargs: str(target))
    monkeypatch.setattr("client_desktop.app.messagebox.showinfo", lambda *_args, **_kwargs: None)
    client = build_minimal_client()
    client.inspector = {
        "file": "demo.slam",
        "manifest": {},
        "points": [],
        "pgm": "P2\n1 1\n255\n0\n",
        "yaml": "image: map.pgm\n",
        "json": "{\"demo\": true}",
        "meta": {},
        "pcd_file": None,
    }
    client.logger = types.SimpleNamespace(info=lambda *args, **kwargs: None)

    DesktopClient.export_inspector_file(client, "zip")

    with zipfile.ZipFile(target, "r") as zf:
        assert set(zf.namelist()) == {"map.pgm", "map.yaml", "map.json"}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTHONPATH=. pytest tests/test_client_desktop_app_helpers.py -k 'export_zip_writes_pgm_yaml_json_only' -q`
Expected: FAIL because `zip` export does not exist yet

- [ ] **Step 3: Write minimal implementation**

Extend `export_inspector_file()` to support `kind == "zip"`:

- require current inspector content
- choose a destination via save dialog
- create a zip archive using:
  - `map.pgm`
  - `map.yaml`
  - `map.json`
- exclude point cloud artifacts and `.slam` internals

- [ ] **Step 4: Wire button label and command**

Update the map tools button row and translations to add `Export ZIP` / `导出 ZIP`.

- [ ] **Step 5: Run focused tests**

Run: `PYTHONPATH=. pytest tests/test_client_desktop_app_helpers.py -k 'export_zip_writes_pgm_yaml_json_only or export_pcd' -q`
Expected: PASS

- [ ] **Step 6: Commit**

```bash
git add client_desktop/app.py tests/test_client_desktop_app_helpers.py
git commit -m "feat: add map export zip package"
```

## Task 5: Preserve Import Semantics and Manifest Compatibility

**Files:**
- Modify: [client_desktop/app.py](/home/autodrive2/autodrive/client_desktop/app.py)
- Test: [tests/test_client_desktop_app_helpers.py](/home/autodrive2/autodrive/tests/test_client_desktop_app_helpers.py)

- [ ] **Step 1: Write the failing compatibility tests**

Add tests for:

```python
def test_read_slam_archive_ignores_save_payload_metadata(tmp_path: Path) -> None:
    ...
    manifest, points, pcd_file = read_slam_archive(target)
    assert manifest["save_payload"] == "2d_only"
    assert points == [(1.0, 2.0, 3.0)]
    assert pcd_file is None


def test_load_export_zip_uses_native_import_path(monkeypatch, tmp_path: Path) -> None:
    ...
```

The first test is required; the second can remain a logic-level regression around suffix routing if a full zip-native import fixture is too heavy.

- [ ] **Step 2: Run tests to verify failures where applicable**

Run: `PYTHONPATH=. pytest tests/test_client_desktop_app_helpers.py -k 'save_payload or load_export_zip_uses_native_import_path' -q`
Expected: FAIL if import compatibility is not yet preserved

- [ ] **Step 3: Write minimal compatibility implementation**

Ensure:

- `read_slam_archive()` keeps unknown manifest fields intact
- `.slam` load path remains unchanged
- `export.zip` continues to flow through native import semantics rather than engineering-state restoration

- [ ] **Step 4: Run the focused tests**

Run: `PYTHONPATH=. pytest tests/test_client_desktop_app_helpers.py -k 'save_payload or load_export_zip_uses_native_import_path or load_stcm' -q`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add client_desktop/app.py tests/test_client_desktop_app_helpers.py
git commit -m "test: preserve map import compatibility"
```

## Task 6: Update User-Facing Documentation

**Files:**
- Modify: [client_desktop/README.md](/home/autodrive2/autodrive/client_desktop/README.md)

- [ ] **Step 1: Add documentation for the new save/export split**

Document:

- `.slam` as editable project format
- 3D save options:
  - `2D only`
  - `2D + PCD`
- `Export ZIP` as `pgm + yaml + json`
- `Export PCD` unchanged

- [ ] **Step 2: Run a quick consistency read**

Check that names in the README match UI labels exactly.

- [ ] **Step 3: Commit**

```bash
git add client_desktop/README.md
git commit -m "docs: describe map save and export redesign"
```

## Task 7: Final Verification

**Files:**
- Modify: none
- Test: [tests/test_client_desktop_app_helpers.py](/home/autodrive2/autodrive/tests/test_client_desktop_app_helpers.py)

- [ ] **Step 1: Run the full targeted desktop helper suite**

Run: `PYTHONPATH=. pytest tests/test_client_desktop_app_helpers.py -q`
Expected: PASS

- [ ] **Step 2: Manually sanity-check the archive outputs**

Run these from the repo root after creating one 2D save, one 3D `2D only` save, one 3D `2D + PCD` save, and one `Export ZIP`:

```bash
unzip -l /path/to/2d_map.slam
unzip -l /path/to/3d_2d_only.slam
unzip -l /path/to/3d_with_pcd.slam
unzip -l /path/to/export_map.zip
```

Expected:

- `2d_map.slam` -> `manifest.json`, `map_points.bin`
- `3d_2d_only.slam` -> `manifest.json`, `map_points.bin`
- `3d_with_pcd.slam` -> `manifest.json`, `map_points.bin`, `map.pcd`
- `export_map.zip` -> `map.pgm`, `map.yaml`, `map.json`

- [ ] **Step 3: Verify no regression in PCD export**

Run: `PYTHONPATH=. pytest tests/test_client_desktop_app_helpers.py -k 'export_pcd' -q`
Expected: PASS

- [ ] **Step 4: Commit final polish**

```bash
git add client_desktop/app.py tests/test_client_desktop_app_helpers.py client_desktop/README.md
git commit -m "feat: redesign desktop map save and export flows"
```
