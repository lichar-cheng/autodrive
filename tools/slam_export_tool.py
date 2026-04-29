from __future__ import annotations

import ast
import json
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass
class LoadedSlam:
    manifest: dict[str, Any]
    occupancy_grid: dict[str, Any]


@dataclass
class ExportArtifacts:
    pgm_text: str
    yaml_text: str
    json_text: str
    pgm_meta: dict[str, Any]
    pcd_bytes: bytes | None = None


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
        return LoadedSlam(
            manifest=manifest,
            occupancy_grid={
                "width": width,
                "height": height,
                "resolution": resolution,
                "origin": {"x": float(origin[0]), "y": float(origin[1])},
                "data": grid_data,
            },
        )

    @staticmethod
    def load(path: str | Path) -> LoadedSlam:
        slam_path = Path(path)
        with zipfile.ZipFile(slam_path, "r") as zf:
            manifest = json.loads(zf.read("manifest.json"))
            blob = zf.read("grid.bin")
            pcd_payload = SlamExportTool._load_embedded_pcd(manifest, zf)
            if pcd_payload is not None:
                manifest["pcd_file"] = pcd_payload
        occupancy_grid = dict(manifest.get("occupancy_grid") or {})
        occupancy_grid["data"] = [value - 256 if value >= 128 else value for value in blob]
        return LoadedSlam(manifest=manifest, occupancy_grid=occupancy_grid)

    @staticmethod
    def build_exports(
        source_file: str,
        manifest: dict[str, Any],
        occupancy_grid: dict[str, Any],
        resolution: float,
        padding_cells: int = 8,
    ) -> ExportArtifacts:
        pgm = SlamExportTool._build_pgm(occupancy_grid, resolution, padding_cells)
        yaml_text = SlamExportTool._build_yaml(source_file, resolution, pgm["origin"])
        export_manifest = dict(manifest)
        pcd_bytes = None
        pcd_meta = export_manifest.get("pcd_file")
        if isinstance(pcd_meta, dict) and pcd_meta.get("included"):
            pcd_bytes = pcd_meta.get("content")
            export_manifest["pcd_file"] = {
                "name": f"{Path(source_file).stem}.pcd",
                "included": True,
            }
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
        return ExportArtifacts(
            pgm_text=pgm["pgm"],
            yaml_text=yaml_text,
            json_text=json_text,
            pgm_meta=pgm,
            pcd_bytes=bytes(pcd_bytes) if pcd_bytes is not None else None,
        )

    @staticmethod
    def export(path: str | Path, output_dir: str | Path, resolution: float, padding_cells: int = 8) -> ExportArtifacts:
        slam_path = Path(path)
        output_path = Path(output_dir)
        loaded = SlamExportTool.load(slam_path)
        artifacts = SlamExportTool.build_exports(
            slam_path.name,
            loaded.manifest,
            loaded.occupancy_grid,
            resolution=resolution,
            padding_cells=padding_cells,
        )
        output_path.mkdir(parents=True, exist_ok=True)
        stem = slam_path.stem
        (output_path / f"{stem}.pgm").write_text(artifacts.pgm_text, encoding="utf-8")
        (output_path / f"{stem}.yaml").write_text(artifacts.yaml_text, encoding="utf-8")
        (output_path / f"{stem}.json").write_text(artifacts.json_text, encoding="utf-8")
        if artifacts.pcd_bytes is not None:
            (output_path / f"{stem}.pcd").write_bytes(artifacts.pcd_bytes)
        return artifacts

    @staticmethod
    def _build_pgm(
        occupancy_grid: dict[str, Any],
        resolution: float,
        padding_cells: int,
    ) -> dict[str, Any]:
        del padding_cells
        resolution = max(0.02, float(occupancy_grid.get("resolution", resolution)))
        width = int(occupancy_grid.get("width", 0) or 0)
        height = int(occupancy_grid.get("height", 0) or 0)
        data = [int(value) for value in occupancy_grid.get("data", [])]
        if width <= 0 or height <= 0 or len(data) != width * height:
            raise ValueError("No occupancy grid in SLAM")
        rows = []
        for row in range(height):
            source_row = height - 1 - row
            start = source_row * width
            values = []
            for col in range(width):
                value = data[start + col]
                values.append("0" if value >= 50 else "254" if value == 0 else "205")
            rows.append(" ".join(values))
        origin_meta = occupancy_grid.get("origin") if isinstance(occupancy_grid.get("origin"), dict) else {}
        occupied_cells = sum(1 for value in data if int(value) >= 50)
        return {
            "pgm": f"P2\n# Generated from SLAM occupancy\n{width} {height}\n255\n" + "\n".join(rows) + "\n",
            "origin": [round(float(origin_meta.get("x", 0.0)), 3), round(float(origin_meta.get("y", 0.0)), 3), 0],
            "width": width,
            "height": height,
            "occupied_cells": occupied_cells,
            "bounds": {
                "minX": round(float(origin_meta.get("x", 0.0)), 3),
                "maxX": round(float(origin_meta.get("x", 0.0)) + width * resolution, 3),
                "minY": round(float(origin_meta.get("y", 0.0)), 3),
                "maxY": round(float(origin_meta.get("y", 0.0)) + height * resolution, 3),
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
    def _load_embedded_pcd(manifest: dict[str, Any], archive: zipfile.ZipFile) -> dict[str, Any] | None:
        pcd_meta = manifest.get("pcd_file")
        if isinstance(pcd_meta, dict) and pcd_meta.get("included"):
            pcd_name = str(pcd_meta.get("name", "map.pcd"))
            return {"name": pcd_name, "included": True, "content": archive.read(pcd_name)}
        legacy_meta = manifest.get("pcd")
        if isinstance(legacy_meta, dict) and legacy_meta.get("included"):
            pcd_name = str(legacy_meta.get("file", "map.pcd"))
            return {"name": pcd_name, "included": True, "content": archive.read(pcd_name)}
        return None

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
