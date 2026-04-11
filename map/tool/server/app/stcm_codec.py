from __future__ import annotations

import json
import struct
import zipfile
from pathlib import Path


def _encode_points(points: list[tuple | list], fmt: str) -> bytes:
    return b"".join(struct.pack(fmt, *point) for point in points)


def _decode_points(blob: bytes, fmt: str) -> list[tuple]:
    stride = struct.calcsize(fmt)
    return [
        struct.unpack(fmt, blob[i : i + stride])
        for i in range(0, len(blob), stride)
        if i + stride <= len(blob)
    ]


def save_stcm(path: Path, bundle: dict) -> Path:
    """
    单个 .slam 文件包含完整数据：
    - manifest.json: poi/path/gps 等结构化信息
    - radar_points.bin: 2D 雷达点 (x,y,intensity)
    - point_cloud.bin: 3D 点云点 (x,y,z,intensity)
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    scan_mode = str(bundle.get("scan_mode") or "2d").strip().lower()
    if scan_mode == "3d":
        points = bundle.get("point_cloud", [])
        raw = _encode_points(points, "ffff")
        manifest = {k: v for k, v in bundle.items() if k != "point_cloud"}
        payload_name = "point_cloud.bin"
    else:
        points = bundle.get("radar_points", [])
        raw = _encode_points(points, "fff")
        manifest = {k: v for k, v in bundle.items() if k != "radar_points"}
        payload_name = "radar_points.bin"

    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("manifest.json", json.dumps(manifest, ensure_ascii=False, indent=2))
        zf.writestr(payload_name, raw)
    return path


def load_stcm(path: Path) -> dict:
    with zipfile.ZipFile(path, "r") as zf:
        manifest = json.loads(zf.read("manifest.json"))
        scan_mode = str(manifest.get("scan_mode") or "2d").strip().lower()
        if scan_mode == "3d":
            blob = zf.read("point_cloud.bin")
            manifest["point_cloud"] = _decode_points(blob, "ffff")
        else:
            blob = zf.read("radar_points.bin")
            manifest["radar_points"] = _decode_points(blob, "fff")
    return manifest
