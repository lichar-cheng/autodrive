from __future__ import annotations

import ast
import json
import struct
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass
class LoadedSlam:
    manifest: dict[str, Any]
    radar_points: list[tuple[float, float, float]]


@dataclass
class ExportArtifacts:
    pgm_text: str
    yaml_text: str
    json_text: str
    pgm_meta: dict[str, Any]


class SlamExportTool:
    @staticmethod
    def resolve_native_map_yaml_path(path: str | Path) -> Path:
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
    def import_native_map(path: str | Path) -> LoadedSlam:
        yaml_path = SlamExportTool.resolve_native_map_yaml_path(path)
        metadata = SlamExportTool._parse_native_map_yaml(yaml_path)
        pgm_path = yaml_path.parent / str(metadata["image"])
        width, height, pixels = SlamExportTool._read_pgm(pgm_path)
        resolution = float(metadata["resolution"])
        origin = list(metadata["origin"])
        negate = int(metadata.get("negate", 0))
        occupied_thresh = float(metadata.get("occupied_thresh", 0.65))
        free_thresh = float(metadata.get("free_thresh", 0.196))

        occupied_cells: list[dict[str, float | int]] = []
        free_cells: list[dict[str, int]] = []
        radar_points: list[tuple[float, float, float]] = []
        for row in range(height):
            for col in range(width):
                value = int(pixels[row * width + col])
                normalized = max(0.0, min(1.0, value / 255.0))
                occupancy = normalized if negate else (1.0 - normalized)
                iy = round((float(origin[1]) / resolution) + (height - 1 - row))
                ix = round((float(origin[0]) / resolution) + col)
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
        return LoadedSlam(manifest=manifest, radar_points=radar_points)

    @staticmethod
    def load(path: str | Path) -> LoadedSlam:
        slam_path = Path(path)
        with zipfile.ZipFile(slam_path, "r") as zf:
            manifest = json.loads(zf.read("manifest.json"))
            blob = zf.read("radar_points.bin")
        radar_points = [
            struct.unpack("fff", blob[index : index + 12])
            for index in range(0, len(blob), 12)
            if index + 12 <= len(blob)
        ]
        return LoadedSlam(manifest=manifest, radar_points=radar_points)

    @staticmethod
    def build_exports(
        source_file: str,
        manifest: dict[str, Any],
        radar_points: list[tuple[float, float, float]] | list[list[float]],
        resolution: float,
        padding_cells: int = 8,
    ) -> ExportArtifacts:
        pgm = SlamExportTool._build_pgm(manifest, radar_points, resolution, padding_cells)
        yaml_text = SlamExportTool._build_yaml(source_file, resolution, pgm["origin"])
        export_manifest = dict(manifest)
        export_manifest.pop("browser_occupancy", None)
        export_manifest.pop("trajectory", None)
        json_text = json.dumps(
            {
                "source_file": source_file,
                "map_yaml": {
                    "image": f"{Path(source_file).stem}.pgm",
                    "mode": "trinary",
                    "resolution": float(resolution),
                    "origin": list(pgm["origin"]),
                    "negate": 0,
                    "occupied_thresh": 0.65,
                    "free_thresh": 0.196,
                },
                "pgm_meta": {
                    "width": int(pgm["width"]),
                    "height": int(pgm["height"]),
                    "occupied_cells": int(pgm["occupied_cells"]),
                    "bounds": dict(pgm["bounds"]),
                },
                "manifest": export_manifest,
            },
            ensure_ascii=False,
            indent=2,
        )
        return ExportArtifacts(pgm_text=pgm["pgm"], yaml_text=yaml_text, json_text=json_text, pgm_meta=pgm)

    @staticmethod
    def export(path: str | Path, output_dir: str | Path, resolution: float, padding_cells: int = 8) -> ExportArtifacts:
        slam_path = Path(path)
        output_path = Path(output_dir)
        loaded = SlamExportTool.load(slam_path)
        artifacts = SlamExportTool.build_exports(
            slam_path.name,
            loaded.manifest,
            loaded.radar_points,
            resolution=resolution,
            padding_cells=padding_cells,
        )
        output_path.mkdir(parents=True, exist_ok=True)
        stem = slam_path.stem
        (output_path / f"{stem}.pgm").write_text(artifacts.pgm_text, encoding="utf-8")
        (output_path / f"{stem}.yaml").write_text(artifacts.yaml_text, encoding="utf-8")
        (output_path / f"{stem}.json").write_text(artifacts.json_text, encoding="utf-8")
        return artifacts

    @staticmethod
    def _build_pgm(
        manifest: dict[str, Any],
        radar_points: list[tuple[float, float, float]] | list[list[float]],
        resolution: float,
        padding_cells: int,
    ) -> dict[str, Any]:
        browser = manifest.get("browser_occupancy") if isinstance(manifest.get("browser_occupancy"), dict) else {}
        occupancy_voxel = max(0.02, float(browser.get("voxel_size", resolution)))
        occupied_cells = browser.get("occupied_cells") if isinstance(browser.get("occupied_cells"), list) else None

        occupied_set: set[tuple[int, int]] = set()
        min_cell_x = float("inf")
        max_cell_x = float("-inf")
        min_cell_y = float("inf")
        max_cell_y = float("-inf")

        if occupied_cells:
            for cell in occupied_cells:
                ix = round(float(cell.get("ix", 0)))
                iy = round(float(cell.get("iy", 0)))
                occupied_set.add((ix, iy))
                min_cell_x = min(min_cell_x, ix)
                max_cell_x = max(max_cell_x, ix)
                min_cell_y = min(min_cell_y, iy)
                max_cell_y = max(max_cell_y, iy)
        else:
            if not radar_points:
                raise ValueError("No radar points in SLAM")
            for point in radar_points:
                ix = round(float(point[0]) / resolution)
                iy = round(float(point[1]) / resolution)
                occupied_set.add((ix, iy))
                min_cell_x = min(min_cell_x, ix)
                max_cell_x = max(max_cell_x, ix)
                min_cell_y = min(min_cell_y, iy)
                max_cell_y = max(max_cell_y, iy)

        padded_min_x = int(min_cell_x) - padding_cells
        padded_min_y = int(min_cell_y) - padding_cells
        padded_max_x = int(max_cell_x) + padding_cells
        padded_max_y = int(max_cell_y) + padding_cells
        width = max(1, padded_max_x - padded_min_x + 1)
        height = max(1, padded_max_y - padded_min_y + 1)
        grid = [205] * (width * height)

        for ix, iy in occupied_set:
            x = ix - padded_min_x
            y = iy - padded_min_y
            flipped_y = height - 1 - y
            grid[flipped_y * width + x] = 0

        rows = []
        for row in range(height):
            start = row * width
            rows.append(" ".join(str(grid[start + col]) for col in range(width)))

        return {
            "pgm": f"P2\n# Generated from SLAM occupancy\n{width} {height}\n255\n" + "\n".join(rows) + "\n",
            "origin": [round(padded_min_x * occupancy_voxel, 3), round(padded_min_y * occupancy_voxel, 3), 0],
            "width": width,
            "height": height,
            "occupied_cells": len(occupied_set),
            "bounds": {
                "minX": round(min_cell_x * occupancy_voxel, 3),
                "maxX": round(max_cell_x * occupancy_voxel, 3),
                "minY": round(min_cell_y * occupancy_voxel, 3),
                "maxY": round(max_cell_y * occupancy_voxel, 3),
            },
        }

    @staticmethod
    def _build_yaml(source_file: str, resolution: float, origin: list[float]) -> str:
        return "\n".join(
            [
                f"image: {Path(source_file).stem}.pgm",
                "mode: trinary",
                f"resolution: {float(resolution):.3f}",
                f"origin: [{float(origin[0]):.3f}, {float(origin[1]):.3f}, {int(origin[2])}]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
            ]
        )

    @staticmethod
    def _parse_native_map_yaml(path: Path) -> dict[str, Any]:
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
        data_len = len(data)
        while len(tokens) < 4 and index < data_len:
            while index < data_len and chr(data[index]).isspace():
                index += 1
            if index < data_len and data[index:index + 1] == b"#":
                while index < data_len and data[index:index + 1] not in {b"\n", b"\r"}:
                    index += 1
                continue
            start = index
            while index < data_len and not chr(data[index]).isspace():
                index += 1
            if start < index:
                tokens.append(data[start:index])
        if len(tokens) < 4:
            raise ValueError(f"invalid pgm header: {path}")
        magic = tokens[0].decode("ascii")
        width = int(tokens[1])
        height = int(tokens[2])
        max_value = int(tokens[3])
        while index < data_len and chr(data[index]).isspace():
            index += 1
        if magic == "P2":
            body = data[index:].decode("ascii")
            values = [int(item) for item in body.split() if item.strip()]
        elif magic == "P5":
            if max_value > 255:
                raise ValueError(f"unsupported pgm max value: {max_value}")
            values = list(data[index : index + width * height])
        else:
            raise ValueError(f"unsupported pgm format: {magic}")
        if len(values) < width * height:
            raise ValueError(f"pgm pixel data truncated: {path}")
        return width, height, values[: width * height]
