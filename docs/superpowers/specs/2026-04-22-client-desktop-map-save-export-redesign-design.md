# Client Desktop Map Save/Export Redesign

**Goal:** Separate editable project saves from deployment-oriented exports in `client_desktop`, while keeping the current `.slam` workflow for engineering use.

**Scope:** This design covers `client_desktop` map save behavior, archive contents, export behaviors, import expectations, minimal manifest changes, UI changes, and test strategy. It does not change the server-side scan lifecycle contract except where existing client behavior depends on it.

## Current Baseline

- [client_desktop/app.py](/home/autodrive2/autodrive/client_desktop/app.py) currently uses one `Save Map` action that always writes a `.slam` archive.
- A `.slam` archive currently stores:
  - `manifest.json`
  - `map_points.bin`
  - optional `map.pcd`
- `Export` currently exists as separate single-file actions:
  - `PGM`
  - `YAML`
  - `JSON`
  - `PCD`
- `.slam` is serving two jobs at once:
  - editable project file
  - distribution/export container

This mixes engineering-state preservation with deployment output and makes 3D save behavior harder to reason about.

## Product Direction

The redesigned flow should distinguish between:

- **Project save**
  - preserves editable map state
  - remains `.slam`
- **Deployment export**
  - produces native map deliverables
  - becomes a new zip export containing `pgm + yaml + json`

This keeps `.slam` as the source-of-truth editing format and makes exports intentionally lossy and deployment-focused.

## Functional Requirements

### 1. Save Map

`Save Map` must continue to save a `.slam` archive.

#### 2D mode

- Save the current 2D editable project state.
- Include full 2D point data plus existing JSON structure.
- Do not include `pcd`.

#### 3D mode

- Present a save-content choice when saving a `.slam`.
- The choice must have exactly two options:
  - `2D only`
  - `2D + PCD`
- Default selection must be `2D only`.

##### 3D `2D only`

- Save exactly the same engineering payload as a 2D `.slam`:
  - 2D point data
  - JSON structure
- Do not require `pcd`.
- Must succeed even if no valid `pcd` is currently available.

##### 3D `2D + PCD`

- Save the same 2D engineering payload plus `map.pcd`.
- Reuse the current `pcd` retrieval path.
- If `pcd` cannot be retrieved, save must fail with an explicit user-visible error.

### 2. Export ZIP

Add a new `Export ZIP` action.

- Output must be a zip archive.
- The archive must contain exactly:
  - `map.pgm`
  - `map.yaml`
  - `map.json`
- It must not contain:
  - `map_points.bin`
  - `map.pcd`
- It is intended for native/deployment use, not full engineering-state recovery.

### 3. Export PCD

`Export PCD` must keep the current behavior.

- If the current session or loaded `.slam` contains `pcd`, export it.
- If no `pcd` is available, show the current warning behavior.

## File Format Design

## `.slam` remains the engineering format

The `.slam` archive remains zip-based and continues to preserve editable state.

### 2D `.slam`

Required entries:

- `manifest.json`
- `map_points.bin`

Manifest expectations:

- preserve existing structured fields such as:
  - `scan_mode`
  - `notes`
  - `occupancy`
  - `pose`
  - `gps`
  - `chassis`
  - `poi`
  - `path`
  - `scan_summary`
  - `scan_fusion`

### 3D `.slam` with default `2D only`

Required entries:

- `manifest.json`
- `map_points.bin`

Optional manifest metadata:

```json
{
  "scan_mode": "3d",
  "save_payload": "2d_only"
}
```

This metadata is not required for functionality, but it is useful for diagnostics and future compatibility.

### 3D `.slam` with `2D + PCD`

Required entries:

- `manifest.json`
- `map_points.bin`
- `map.pcd`

Optional manifest metadata:

```json
{
  "scan_mode": "3d",
  "save_payload": "2d_pcd",
  "pcd": {
    "included": true,
    "file": "map.pcd"
  }
}
```

## `Export ZIP` is the deployment format

The new zip export is intentionally not equivalent to `.slam`.

Required entries:

- `map.pgm`
- `map.yaml`
- `map.json`

It should be treated as a deployment/native package, not a project-save surrogate.

## Import Behavior

Import behavior should stay intentionally asymmetric.

### Import `.slam`

- Restore editable engineering state as today.
- If the archive includes `map.pcd`, restore it into the in-memory session.
- Preserve second-stage editing behavior.

### Import `export.zip`

- Treat it like native map import, equivalent in intent to loading `pgm/yaml`.
- Support viewing and native map use.
- Do not attempt to restore engineering-state point data.
- Do not attempt to reconstruct `.slam`-grade edit fidelity.

This preserves a clean distinction between project files and export files.

## UI Design

Keep the main map workflow familiar and add the smallest possible UI surface.

### Save Map

- Keep the existing `Save Map` button.
- In 2D mode:
  - save immediately as `.slam`
- In 3D mode:
  - show a small modal/dialog before writing the file
  - dialog options:
    - `2D only`
    - `2D + PCD`
  - default must be `2D only`

Recommended dialog copy:

- Title: `3D Save Options`
- Body: `Choose whether to include the current PCD in the project archive.`

### Export Actions

Keep single-purpose export actions explicit.

- Add `Export ZIP`
- Keep:
  - `Export PGM`
  - `Export YAML`
  - `Export JSON`
  - `Export PCD`

Recommended button grouping:

- `Save Map`
- `Load Map`
- `Export ZIP`
- `Export PGM`
- `Export YAML`
- `Export JSON`
- `Export PCD`

## Data Flow

### Save Map in 2D

1. Gather editable state and occupied points.
2. Build manifest JSON.
3. Write `.slam` with `manifest.json + map_points.bin`.

### Save Map in 3D with `2D only`

1. Gather editable state and occupied points.
2. Do not request `pcd`.
3. Write `.slam` with `manifest.json + map_points.bin`.
4. Mark manifest payload as `2d_only` if the optional metadata is adopted.

### Save Map in 3D with `2D + PCD`

1. Gather editable state and occupied points.
2. Retrieve `pcd` through the existing desktop retrieval path.
3. If retrieval fails, abort with a warning.
4. Write `.slam` with `manifest.json + map_points.bin + map.pcd`.

### Export ZIP

1. Use the current inspector/export generation path for `pgm`, `yaml`, and `json`.
2. Package those three outputs into a zip archive.
3. Do not include point data or `pcd`.

## Compatibility

### Backward compatibility

- Existing `.slam` files must continue to load.
- Existing `.slam` files that include `map.pcd` must continue to restore `pcd`.
- Existing `.slam` files without `pcd` must continue to load as normal 2D engineering files.

### Forward compatibility

- Adding `save_payload` to `manifest.json` should not break older readers that ignore unknown fields.
- `export.zip` should be treated as a new export artifact, not as a replacement for `.slam`.

## Error Handling

### Save Map 2D

- Fail only on normal file-write or serialization errors.

### Save Map 3D `2D only`

- Must not fail because `pcd` is unavailable.

### Save Map 3D `2D + PCD`

- If `pcd` cannot be retrieved:
  - show a warning
  - abort the save
- The error should remain explicit about whether failure came from:
  - not being in a retrievable 3D state
  - missing `pcd`
  - failed download

### Export ZIP

- If export prerequisites are missing, follow the same guardrails as current single-file exports.
- Do not silently produce partial archives.

## Implementation Boundaries

This redesign should remain mostly in `client_desktop`.

### Primary write scope

- [client_desktop/app.py](/home/autodrive2/autodrive/client_desktop/app.py)

Likely touch points:

- `save_stcm()`
- `write_slam_archive()`
- `export_inspector_file()`
- save/export button setup
- translation tables
- new small modal/helper for 3D save content selection

### Minimal or no server changes

- Server APIs do not need redesign for this feature.
- Existing `pcd` retrieval remains sufficient for the `2D + PCD` branch.

## Test Strategy

At minimum, cover the following:

### Save Map

- 2D save writes `.slam` without `pcd`
- 3D save default branch writes `.slam` without `pcd`
- 3D save `2D + PCD` writes `.slam` with `map.pcd`
- 3D save `2D + PCD` aborts cleanly when `pcd` is unavailable

### Archive structure

- `.slam` 2D contains only `manifest.json + map_points.bin`
- `.slam` 3D default contains only `manifest.json + map_points.bin`
- `.slam` 3D with `pcd` contains `manifest.json + map_points.bin + map.pcd`

### Export ZIP

- zip contains exactly `map.pgm`, `map.yaml`, and `map.json`
- zip excludes `map_points.bin`
- zip excludes `map.pcd`

### Import

- importing `.slam` preserves engineering editing behavior
- importing `export.zip` follows native-map import behavior only
- importing old `.slam` files remains supported

## Recommendation

Proceed with the redesign exactly as described:

- keep `.slam` as the editable engineering/project format
- add a separate `Export ZIP` deployment artifact
- in 3D save, default to `2D only`
- include `pcd` only when the user explicitly chooses `2D + PCD`

This gives the smallest user-facing change that also fixes the current conceptual confusion between source files and exported deliverables.
