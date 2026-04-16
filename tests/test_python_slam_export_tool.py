import json
import struct
import zipfile
from pathlib import Path

from tool.slam_export_tool import SlamExportTool


def write_slam(path: Path, manifest: dict, points: list[tuple[float, float, float]]) -> None:
    raw = b"".join(struct.pack("fff", *point) for point in points)
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("manifest.json", json.dumps(manifest, ensure_ascii=False, indent=2))
        zf.writestr("radar_points.bin", raw)


def test_load_reads_manifest_and_radar_points(tmp_path: Path) -> None:
    slam_path = tmp_path / "demo.slam"
    write_slam(
        slam_path,
        {"name": "demo", "browser_occupancy": {"voxel_size": 0.2, "occupied_cells": [{"ix": 1, "iy": 2, "hits": 3, "intensity": 0.8}]}},
        [(1.0, 2.0, 0.8)],
    )

    loaded = SlamExportTool.load(slam_path)

    assert loaded.manifest["name"] == "demo"
    assert len(loaded.radar_points) == 1
    assert loaded.radar_points[0][0] == 1.0
    assert loaded.radar_points[0][1] == 2.0
    assert abs(loaded.radar_points[0][2] - 0.8) < 1e-6


def test_build_exports_prefers_browser_occupancy_cells(tmp_path: Path) -> None:
    manifest = {
        "notes": "demo",
        "browser_occupancy": {
            "voxel_size": 0.2,
            "occupied_cells": [
                {"ix": 0, "iy": 0, "hits": 3, "intensity": 0.8},
                {"ix": 1, "iy": 0, "hits": 3, "intensity": 0.7},
            ],
        },
        "trajectory": [{"legacy": True}],
    }

    artifacts = SlamExportTool.build_exports("demo.slam", manifest, [(9.0, 9.0, 1.0)], resolution=0.2, padding_cells=1)

    assert "demo.pgm" in artifacts.yaml_text
    assert "\"trajectory\"" not in artifacts.json_text
    assert artifacts.pgm_meta["occupied_cells"] == 2
    assert "0 0" in artifacts.pgm_text


def test_export_writes_pgm_yaml_and_json(tmp_path: Path) -> None:
    slam_path = tmp_path / "demo.slam"
    output_dir = tmp_path / "out"
    write_slam(
        slam_path,
        {
            "name": "demo",
            "browser_occupancy": {"voxel_size": 0.1, "occupied_cells": [{"ix": 1, "iy": 1, "hits": 3, "intensity": 1.0}]},
        },
        [(0.1, 0.1, 1.0)],
    )

    artifacts = SlamExportTool.export(slam_path, output_dir, resolution=0.1, padding_cells=2)

    assert (output_dir / "demo.pgm").exists()
    assert (output_dir / "demo.yaml").exists()
    assert (output_dir / "demo.json").exists()
    assert artifacts.pgm_meta["width"] >= 1
