# Scan Fusion Presets Design

**Goal:** Make scan fusion behavior configurable by scene preset and parameter overrides so real-world mapping can preserve subtle obstacles like tables and chairs without hardcoding one threshold set for every environment.

## Problem

The current live mapping path uses fixed fusion rules in the browser and desktop clients:

- voxel size defaults to `0.12`
- turning frames may be skipped
- occupied cells are only kept when `hits >= 3`
- occupied cells also need to beat free-hit pressure

These defaults work well in simulation where returns are dense and stable, but they suppress weak or sparse real-world obstacles such as chair legs, table edges, and partial returns.

## Design

Introduce a shared scan fusion config model with:

- `preset`
- `voxel_size`
- `occupied_min_hits`
- `occupied_over_free_ratio`
- `turn_skip_wz`
- `skip_turn_frames`

Behavior uses two layers:

1. Scene preset
   - provides a full default bundle for a known environment
2. Parameter overrides
   - allows operators to adjust one or more fields without creating a new preset

## Presets

### `sim_clean`

- `voxel_size = 0.12`
- `occupied_min_hits = 3`
- `occupied_over_free_ratio = 0.90`
- `turn_skip_wz = 0.35`
- `skip_turn_frames = true`

### `indoor_balanced`

- `voxel_size = 0.08`
- `occupied_min_hits = 2`
- `occupied_over_free_ratio = 0.75`
- `turn_skip_wz = 0.45`
- `skip_turn_frames = true`

### `indoor_sensitive`

- `voxel_size = 0.06`
- `occupied_min_hits = 1`
- `occupied_over_free_ratio = 0.55`
- `turn_skip_wz = 0.60`
- `skip_turn_frames = false`

### `warehouse_sparse`

- `voxel_size = 0.10`
- `occupied_min_hits = 2`
- `occupied_over_free_ratio = 0.65`
- `turn_skip_wz = 0.50`
- `skip_turn_frames = true`

## Default Selection

- simulator default: `sim_clean`
- real vehicle default: `indoor_balanced`
- field fallback when subtle obstacles are missing: `indoor_sensitive`

## Integration Rules

The same config must drive:

- live scan accumulation
- map display filtering
- save/export filtering
- `.slam` browser occupancy payload

This avoids a mismatch where the operator sees one threshold set on screen but saves another.

## Storage

The selected preset and effective fusion parameters should be included in saved map metadata so later reload and export can explain how the map was generated.

## Implementation Notes

- Centralize preset resolution in one shared helper per client implementation
- Remove fixed literals like `0.12`, `3`, `0.9`, and `0.35` from accumulation and filtering paths
- Keep current behavior available via `sim_clean`

## Verification

Tests should cover:

- preset resolution
- override merging
- occupied-cell filtering differences between presets
- turning-frame behavior when `skip_turn_frames` changes
- save/load persistence of effective scan fusion settings
