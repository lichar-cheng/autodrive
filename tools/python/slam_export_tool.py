from __future__ import annotations

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


__all__ = ["ExportArtifacts", "LoadedSlam", "SlamExportTool"]
