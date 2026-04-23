import json
import zipfile
from pathlib import Path

from tools.slam_export_tool import SlamExportTool


def write_slam(path: Path, manifest: dict, grid: dict) -> None:
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("manifest.json", json.dumps(manifest, ensure_ascii=False, indent=2))
        zf.writestr("grid.bin", bytes((int(value) & 0xFF) for value in grid["data"]))


def test_load_reads_manifest_and_occupancy_grid(tmp_path: Path) -> None:
    slam_path = tmp_path / "demo.slam"
    write_slam(
        slam_path,
        {
            "name": "demo",
            "version": "slam.v4",
            "map_storage": "occupancy_grid",
            "occupancy_grid": {
                "width": 2,
                "height": 2,
                "resolution": 0.2,
                "origin": {"x": 1.0, "y": 2.0},
                "encoding": "int8",
                "values": {"unknown": -1, "free": 0, "occupied": 100},
            },
        },
        {"width": 2, "height": 2, "resolution": 0.2, "origin": {"x": 1.0, "y": 2.0}, "data": [-1, 0, 100, 0]},
    )

    loaded = SlamExportTool.load(slam_path)

    assert loaded.manifest["name"] == "demo"
    assert loaded.occupancy_grid["data"] == [-1, 0, 100, 0]
    assert loaded.occupancy_grid["origin"] == {"x": 1.0, "y": 2.0}


def test_build_exports_uses_occupancy_grid_data(tmp_path: Path) -> None:
    manifest = {
        "notes": "demo",
        "version": "slam.v4",
        "map_storage": "occupancy_grid",
        "occupancy_grid": {
            "width": 2,
            "height": 2,
            "resolution": 0.2,
            "origin": {"x": 0.0, "y": 0.0},
        },
    }
    grid = {"width": 2, "height": 2, "resolution": 0.2, "origin": {"x": 0.0, "y": 0.0}, "data": [100, 100, 0, -1]}

    artifacts = SlamExportTool.build_exports("demo.slam", manifest, grid, resolution=0.2, padding_cells=1)

    assert "demo.pgm" in artifacts.yaml_text
    assert artifacts.pgm_meta["occupied_cells"] == 2
    assert "0 0" in artifacts.pgm_text


def test_export_writes_pgm_yaml_and_json(tmp_path: Path) -> None:
    slam_path = tmp_path / "demo.slam"
    output_dir = tmp_path / "out"
    write_slam(
        slam_path,
        {
            "name": "demo",
            "version": "slam.v4",
            "map_storage": "occupancy_grid",
            "occupancy_grid": {
                "width": 1,
                "height": 1,
                "resolution": 0.1,
                "origin": {"x": 0.1, "y": 0.1},
                "encoding": "int8",
                "values": {"unknown": -1, "free": 0, "occupied": 100},
            },
        },
        {"width": 1, "height": 1, "resolution": 0.1, "origin": {"x": 0.1, "y": 0.1}, "data": [100]},
    )

    artifacts = SlamExportTool.export(slam_path, output_dir, resolution=0.1, padding_cells=2)

    assert (output_dir / "demo.pgm").exists()
    assert (output_dir / "demo.yaml").exists()
    assert (output_dir / "demo.json").exists()
    assert artifacts.pgm_meta["width"] >= 1


def test_import_native_map_reads_yaml_and_pgm_into_occupancy_grid(tmp_path: Path) -> None:
    yaml_path = tmp_path / "native.yaml"
    pgm_path = tmp_path / "native.pgm"
    pgm_path.write_text("P2\n3 2\n255\n0 205 254\n254 0 205\n", encoding="utf-8")
    yaml_path.write_text(
        "\n".join(
            [
                "image: native.pgm",
                "mode: trinary",
                "resolution: 0.5",
                "origin: [1.0, 2.0, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
            ]
        ),
        encoding="utf-8",
    )

    loaded = SlamExportTool.import_native_map(yaml_path)

    assert loaded.manifest["map_source"] == "native_pgm_yaml"
    assert loaded.manifest["map_storage"] == "occupancy_grid"
    assert loaded.occupancy_grid["resolution"] == 0.5
    assert loaded.occupancy_grid["data"] == [0, 100, -1, 100, -1, 0]


def test_import_native_map_supports_yaml_referenced_from_pgm_selection(tmp_path: Path) -> None:
    yaml_path = tmp_path / "demo.yaml"
    pgm_path = tmp_path / "demo.pgm"
    pgm_path.write_text("P2\n1 1\n255\n0\n", encoding="utf-8")
    yaml_path.write_text(
        "\n".join(
            [
                "image: demo.pgm",
                "resolution: 0.2",
                "origin: [0.0, 0.0, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
            ]
        ),
        encoding="utf-8",
    )

    resolved = SlamExportTool.resolve_native_map_yaml_path(pgm_path)

    assert resolved == yaml_path
