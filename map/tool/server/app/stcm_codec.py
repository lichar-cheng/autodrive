from __future__ import annotations

import json
import struct
import zipfile
from pathlib import Path


def save_stcm(path: Path, bundle: dict) -> Path:
    """
    单个 .stcm 文件包含完整数据：
    - manifest.json: poi/path/trajectory/gps 等结构化信息
    - radar_points.bin: 雷达点云 (x,y,intensity) float32 三元组
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    points = bundle.get("radar_points", [])
    raw = b"".join(struct.pack("fff", *p) for p in points)
    manifest = {k: v for k, v in bundle.items() if k != "radar_points"}

    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("manifest.json", json.dumps(manifest, ensure_ascii=False, indent=2))
        zf.writestr("radar_points.bin", raw)
    return path


def load_stcm(path: Path) -> dict:
    with zipfile.ZipFile(path, "r") as zf:
        manifest = json.loads(zf.read("manifest.json"))
        blob = zf.read("radar_points.bin")

    points = [
        struct.unpack("fff", blob[i : i + 12])
        for i in range(0, len(blob), 12)
        if i + 12 <= len(blob)
    ]
    manifest["radar_points"] = points
    return manifest
