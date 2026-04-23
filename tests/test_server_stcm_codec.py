import json
import zipfile
from pathlib import Path

from server.app.stcm_codec import load_stcm, save_stcm


def test_save_stcm_writes_manifest_and_grid_bin(tmp_path: Path) -> None:
    target = tmp_path / "demo.slam"

    save_stcm(
        target,
        {
            "version": "slam.v4",
            "scan_mode": "2d",
            "occupancy_grid": {
                "width": 3,
                "height": 2,
                "resolution": 0.1,
                "origin": {"x": -0.1, "y": -0.2},
                "data": [-1, 0, 100, 0, 100, -1],
            },
        },
    )

    with zipfile.ZipFile(target, "r") as zf:
        assert set(zf.namelist()) == {"manifest.json", "grid.bin"}
        manifest = json.loads(zf.read("manifest.json"))
        grid_bytes = zf.read("grid.bin")

    assert manifest["version"] == "slam.v4"
    assert manifest["scan_mode"] == "2d"
    assert manifest["occupancy_grid"] == {
        "width": 3,
        "height": 2,
        "resolution": 0.1,
        "origin": {"x": -0.1, "y": -0.2},
        "encoding": "int8",
        "values": {"unknown": -1, "free": 0, "occupied": 100},
    }
    assert list(grid_bytes) == [255, 0, 100, 0, 100, 255]


def test_save_stcm_writes_optional_map_pcd(tmp_path: Path) -> None:
    target = tmp_path / "demo_3d.slam"

    save_stcm(
        target,
        {
            "version": "slam.v3",
            "scan_mode": "3d",
            "occupancy_grid": {"width": 1, "height": 1, "resolution": 0.1, "origin": {"x": 0.0, "y": 0.0}, "data": [100]},
            "pcd_file": {"name": "map.pcd", "content": b"pcd-bytes"},
        },
    )

    with zipfile.ZipFile(target, "r") as zf:
        assert set(zf.namelist()) == {"manifest.json", "grid.bin", "map.pcd"}
        assert zf.read("map.pcd") == b"pcd-bytes"


def test_load_stcm_restores_optional_pcd_payload(tmp_path: Path) -> None:
    target = tmp_path / "roundtrip.slam"
    save_stcm(
        target,
        {
            "version": "slam.v3",
            "scan_mode": "3d",
            "occupancy_grid": {"width": 2, "height": 1, "resolution": 0.1, "origin": {"x": 0.0, "y": 0.0}, "data": [0, 100]},
            "pcd_file": {"name": "map.pcd", "content": b"pcd-bytes"},
        },
    )

    bundle = load_stcm(target)

    assert bundle["occupancy_grid"]["data"] == [0, 100]
    assert bundle["pcd_file"]["name"] == "map.pcd"
    assert bundle["pcd_file"]["content"] == b"pcd-bytes"
