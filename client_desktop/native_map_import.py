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
    occupancy_grid: dict[str, Any]


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

        grid_data = [-1] * (width * height)
        for row in range(height):
            for col in range(width):
                value = int(pixels[row * width + col])
                normalized = max(0.0, min(1.0, value / 255.0))
                occupancy = normalized if negate else (1.0 - normalized)
                grid_index = (height - 1 - row) * width + col
                if occupancy >= occupied_thresh:
                    grid_data[grid_index] = 100
                elif occupancy <= free_thresh:
                    grid_data[grid_index] = 0

        occupancy_grid = {
            "width": width,
            "height": height,
            "resolution": resolution,
            "origin": {"x": float(origin[0]), "y": float(origin[1])},
            "data": grid_data,
        }

        manifest = {
            "version": "slam.v4",
            "source": "imported",
            "map_source": "native_pgm_yaml",
            "map_storage": "occupancy_grid",
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
            "occupancy_grid": {
                "width": width,
                "height": height,
                "resolution": resolution,
                "origin": {"x": float(origin[0]), "y": float(origin[1])},
                "encoding": "int8",
                "values": {"unknown": -1, "free": 0, "occupied": 100},
            },
            "poi": [],
            "path": [],
            "gps_track": [],
            "chassis_track": [],
        }
        return NativeMapImport(file_name=yaml_path.name, manifest=manifest, occupancy_grid=occupancy_grid)

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
