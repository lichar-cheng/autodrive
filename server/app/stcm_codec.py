from __future__ import annotations

import json
import zipfile
from pathlib import Path


def _grid_to_bytes(grid: dict) -> bytes:
    data = list(grid.get("data", []))
    width = int(grid.get("width", 0) or 0)
    height = int(grid.get("height", 0) or 0)
    expected = width * height
    if width <= 0 or height <= 0 or len(data) != expected:
        raise ValueError("occupancy_grid data length must equal width * height")
    return bytes((int(value) & 0xFF) for value in data)


def _bytes_to_grid_data(blob: bytes) -> list[int]:
    return [value - 256 if value >= 128 else value for value in blob]


def _grid_manifest(grid: dict) -> dict:
    return {
        "width": int(grid.get("width", 0) or 0),
        "height": int(grid.get("height", 0) or 0),
        "resolution": float(grid.get("resolution", 0.0) or 0.0),
        "origin": dict(grid.get("origin", {"x": 0.0, "y": 0.0})),
        "encoding": "int8",
        "values": {"unknown": -1, "free": 0, "occupied": 100},
    }


def save_stcm(path: Path, bundle: dict) -> Path:
    """
    单个 .stcm 文件包含完整数据：
    - manifest.json: grid 元信息和 poi/path/trajectory/gps 等结构化信息
    - grid.bin: OccupancyGrid int8 栅格，-1 unknown, 0 free, 100 occupied
    - map.pcd: 可选 3D 点云文件
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    grid = bundle.get("occupancy_grid")
    if not isinstance(grid, dict):
        raise ValueError("slam.v4 requires occupancy_grid")
    raw = _grid_to_bytes(grid)
    pcd_file = bundle.get("pcd_file")
    manifest = {k: v for k, v in bundle.items() if k not in {"occupancy_grid", "pcd_file"}}
    manifest["version"] = "slam.v4"
    manifest["map_storage"] = "occupancy_grid"
    manifest["occupancy_grid"] = _grid_manifest(grid)
    if isinstance(pcd_file, dict):
        manifest["pcd_file"] = {
            "name": str(pcd_file.get("name", "map.pcd")),
            "included": True,
        }

    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("manifest.json", json.dumps(manifest, ensure_ascii=False, indent=2))
        zf.writestr("grid.bin", raw)
        if isinstance(pcd_file, dict):
            pcd_name = str(pcd_file.get("name", "map.pcd"))
            pcd_content = pcd_file.get("content", b"")
            if isinstance(pcd_content, str):
                pcd_content = pcd_content.encode("utf-8")
            zf.writestr(pcd_name, bytes(pcd_content))
    return path


def load_stcm(path: Path) -> dict:
    with zipfile.ZipFile(path, "r") as zf:
        manifest = json.loads(zf.read("manifest.json"))
        blob = zf.read("grid.bin")
        pcd_meta = manifest.get("pcd_file")
        if isinstance(pcd_meta, dict) and pcd_meta.get("included"):
            pcd_name = str(pcd_meta.get("name", "map.pcd"))
            manifest["pcd_file"] = {
                "name": pcd_name,
                "content": zf.read(pcd_name),
            }

    grid = dict(manifest.get("occupancy_grid") or {})
    grid["data"] = _bytes_to_grid_data(blob)
    manifest["occupancy_grid"] = grid
    return manifest
