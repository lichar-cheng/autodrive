# SLAM Export Tooling Design

**Goal:** Provide independent Java and C++ utility libraries that read a `.slam` archive and generate `.pgm`, `.yaml`, and `.json` export content in memory or on disk.

**Scope:** This design covers archive parsing, radar point decoding, export generation rules, public APIs, and test strategy. It does not change the existing server, browser, or desktop save format.

## Source Format

A `.slam` file is a zip archive containing:

- `manifest.json`
- `radar_points.bin`

`manifest.json` is structured JSON and may include `browser_occupancy`, `poi`, `path`, `trajectory`, `pose`, `gps_track`, `chassis_track`, and `notes`.

`radar_points.bin` is a little-endian sequence of `float32` triples:

- `x`
- `y`
- `intensity`

## Export Rules

The Java and C++ implementations must match the browser export logic in `client/main.js`.

### PGM

- Output format is ASCII PGM `P2`
- Occupied cells are `0`
- Free or unknown cells are `205`
- Max gray value is `255`
- If `manifest.browser_occupancy.occupied_cells` exists and is non-empty, build the raster from those cells
- Otherwise, rasterize `radar_points.bin` using:
  - `ix = round(x / resolution)`
  - `iy = round(y / resolution)`
- `padding_cells` expands min and max occupied cell bounds in all directions
- `origin` is:
  - `[(padded_min_cell_x * occupancy_voxel), (padded_min_cell_y * occupancy_voxel), 0]`
- `occupancy_voxel` is:
  - `max(0.02, manifest.browser_occupancy.voxel_size or resolution)`
- Bounds are computed from unpadded occupied cells and emitted in meters

### YAML

The YAML output must be:

- `image: <slam-name>.pgm`
- `mode: trinary`
- `resolution: <resolution to 3 decimals>`
- `origin: [x.xxx, y.yyy, 0]`
- `negate: 0`
- `occupied_thresh: 0.65`
- `free_thresh: 0.196`

### JSON

The JSON output must contain:

- `source_file`
- `map_yaml`
- `pgm_meta`
- `manifest`

`map_yaml` mirrors the YAML content in JSON form.

`pgm_meta` contains:

- `width`
- `height`
- `occupied_cells`
- `bounds`

The exported `manifest` is a deep copy of the source manifest with these fields removed:

- `browser_occupancy`
- `trajectory`

## Public APIs

Both Java and C++ libraries expose two layers:

1. In-memory API
   - load `.slam`
   - access parsed manifest and radar points
   - build `.pgm`, `.yaml`, `.json` strings
2. File export API
   - write `.pgm`, `.yaml`, `.json` beside or under a chosen directory

## Dependencies

- Java:
  - standard zip APIs
  - Jackson for JSON
  - JUnit for tests
- C++:
  - `minizip` for zip read and test fixture creation
  - `nlohmann/json` for JSON
  - CMake and CTest for build and test orchestration

## Test Strategy

Each implementation must cover:

- archive load from temp `.slam`
- export generation from `browser_occupancy`
- fallback rasterization from radar points when `browser_occupancy` is missing
- file export for `.pgm`, `.yaml`, `.json`

The same sample manifest and radar points are used in both test suites so the output shape stays aligned across languages.
