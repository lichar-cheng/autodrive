from __future__ import annotations

import ast
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass
class NativeMapImport:
    file_name: str
    manifest: dict[str, Any]
    radar_points: list[tuple[float, float, float]]


class NativeMapImportTool:
    @staticmethod
    def resolve_yaml_path(path: str | Path) -> Path:
        target = Path(path)
        if target.suffix.lower() in {".yaml", ".yml"}:
            return target
        if target.suffix.lower() == ".pgm":
            for suffix in (".yaml", ".yml"):
                candidate = target.with_suffix(suffix)
                if candidate.exists():
                    return candidate
        raise FileNotFoundError(f"unable to resolve yaml metadata for native map: {target}")

    @staticmethod
    def import_map(path: str | Path) -> NativeMapImport:
        yaml_path = NativeMapImportTool.resolve_yaml_path(path)
        metadata = NativeMapImportTool._parse_yaml(yaml_path)
        pgm_path = yaml_path.parent / str(metadata["image"])
        width, height, pixels = NativeMapImportTool._read_pgm(pgm_path)
        resolution = float(metadata["resolution"])
        origin = list(metadata["origin"])
        negate = int(metadata.get("negate", 0))
        occupied_thresh = float(metadata.get("occupied_thresh", 0.65))
        free_thresh = float(metadata.get("free_thresh", 0.196))

        occupied_cells: list[dict[str, float | int]] = []
        free_cells: list[dict[str, int]] = []
        radar_points: list[tuple[float, float, float]] = []
        origin_ix = float(origin[0]) / resolution
        origin_iy = float(origin[1]) / resolution
        for row in range(height):
            for col in range(width):
                value = int(pixels[row * width + col])
                normalized = max(0.0, min(1.0, value / 255.0))
                occupancy = normalized if negate else (1.0 - normalized)
                ix = round(origin_ix + col)
                iy = round(origin_iy + (height - 1 - row))
                world_x = float(origin[0]) + col * resolution
                world_y = float(origin[1]) + (height - 1 - row) * resolution
                if occupancy >= occupied_thresh:
                    occupied_cells.append({"ix": ix, "iy": iy, "hits": 3, "intensity": 1.0})
                    radar_points.append((round(world_x, 4), round(world_y, 4), 1.0))
                elif occupancy <= free_thresh:
                    free_cells.append({"ix": ix, "iy": iy, "hits": 3})

        manifest = {
            "version": "stcm.v2",
            "source": "imported",
            "map_source": "native_pgm_yaml",
            "notes": json.dumps(
                {
                    "text": f"Imported from native map {yaml_path.name}",
                    "voxelSize": resolution,
                    "loadedFromStcm": False,
                    "loadedMapName": yaml_path.name,
                },
                ensure_ascii=False,
                indent=2,
            ),
            "scan_fusion": {
                "preset": "indoor_balanced",
                "voxel_size": resolution,
                "occupied_min_hits": 1,
                "occupied_over_free_ratio": 0.0,
                "turn_skip_wz": 0.45,
                "skip_turn_frames": True,
            },
            "browser_occupancy": {
                "voxel_size": resolution,
                "occupied_cells": occupied_cells,
                "free_cells": free_cells,
            },
            "poi": [],
            "path": [],
            "gps_track": [],
            "chassis_track": [],
        }
        return NativeMapImport(file_name=yaml_path.name, manifest=manifest, radar_points=radar_points)

    @staticmethod
    def _parse_yaml(path: Path) -> dict[str, Any]:
        parsed: dict[str, Any] = {}
        for raw_line in path.read_text(encoding="utf-8").splitlines():
            line = raw_line.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            key, value = line.split(":", 1)
            key = key.strip()
            value = value.strip()
            if key == "origin":
                parsed[key] = list(ast.literal_eval(value))
            elif key in {"resolution", "occupied_thresh", "free_thresh"}:
                parsed[key] = float(value)
            elif key == "negate":
                parsed[key] = int(value)
            else:
                parsed[key] = value.strip("\"'")
        if "image" not in parsed or "resolution" not in parsed or "origin" not in parsed:
            raise ValueError(f"native map yaml missing required fields: {path}")
        return parsed

    @staticmethod
    def _read_pgm(path: Path) -> tuple[int, int, list[int]]:
        data = path.read_bytes()
        tokens: list[bytes] = []
        index = 0
        while len(tokens) < 4 and index < len(data):
            while index < len(data) and chr(data[index]).isspace():
                index += 1
            if index < len(data) and data[index:index + 1] == b"#":
                while index < len(data) and data[index:index + 1] not in {b"\n", b"\r"}:
                    index += 1
                continue
            start = index
            while index < len(data) and not chr(data[index]).isspace():
                index += 1
            if start < index:
                tokens.append(data[start:index])
        if len(tokens) < 4:
            raise ValueError(f"invalid pgm header: {path}")
        magic = tokens[0].decode("ascii")
        width = int(tokens[1])
        height = int(tokens[2])
        max_value = int(tokens[3])
        while index < len(data) and chr(data[index]).isspace():
            index += 1
        if magic == "P2":
            values = [int(item) for item in data[index:].decode("ascii").split() if item.strip()]
        elif magic == "P5":
            if max_value > 255:
                raise ValueError(f"unsupported pgm max value: {max_value}")
            values = list(data[index : index + width * height])
        else:
            raise ValueError(f"unsupported pgm format: {magic}")
        if len(values) < width * height:
            raise ValueError(f"pgm pixel data truncated: {path}")
        return width, height, values[: width * height]
