from __future__ import annotations

import json
import struct
import zipfile
from pathlib import Path


def save_stcm(path: Path, bundle: dict) -> Path:
    """
    单个 .stcm 文件包含完整数据：
    - manifest.json: poi/path/trajectory/gps 等结构化信息
    - map_points.bin: 雷达点云 (x,y,intensity) float32 三元组
    - map.pcd: 可选 3D 点云文件
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    points = bundle.get("radar_points", [])
    raw = b"".join(struct.pack("fff", *p) for p in points)
    pcd_file = bundle.get("pcd_file")
    manifest = {k: v for k, v in bundle.items() if k not in {"radar_points", "pcd_file"}}
    if isinstance(pcd_file, dict):
        manifest["pcd_file"] = {
            "name": str(pcd_file.get("name", "map.pcd")),
            "included": True,
        }

    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("manifest.json", json.dumps(manifest, ensure_ascii=False, indent=2))
        zf.writestr("map_points.bin", raw)
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
        blob = zf.read("map_points.bin")
        pcd_meta = manifest.get("pcd_file")
        if isinstance(pcd_meta, dict) and pcd_meta.get("included"):
            pcd_name = str(pcd_meta.get("name", "map.pcd"))
            manifest["pcd_file"] = {
                "name": pcd_name,
                "content": zf.read(pcd_name),
            }

    points = [
        struct.unpack("fff", blob[i : i + 12])
        for i in range(0, len(blob), 12)
        if i + 12 <= len(blob)
    ]
    manifest["radar_points"] = points
    return manifest
