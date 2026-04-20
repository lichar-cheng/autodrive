import json
import zipfile
from pathlib import Path

from server.app.stcm_codec import load_stcm, save_stcm


def test_save_stcm_writes_manifest_and_map_points_bin(tmp_path: Path) -> None:
    target = tmp_path / "demo.slam"

    save_stcm(
        target,
        {
            "version": "slam.v3",
            "scan_mode": "2d",
            "radar_points": [(1.0, 2.0, 3.0)],
        },
    )

    with zipfile.ZipFile(target, "r") as zf:
        assert set(zf.namelist()) == {"manifest.json", "map_points.bin"}
        manifest = json.loads(zf.read("manifest.json"))

    assert manifest["version"] == "slam.v3"
    assert manifest["scan_mode"] == "2d"


def test_save_stcm_writes_optional_map_pcd(tmp_path: Path) -> None:
    target = tmp_path / "demo_3d.slam"

    save_stcm(
        target,
        {
            "version": "slam.v3",
            "scan_mode": "3d",
            "radar_points": [(1.0, 2.0, 3.0)],
            "pcd_file": {"name": "map.pcd", "content": b"pcd-bytes"},
        },
    )

    with zipfile.ZipFile(target, "r") as zf:
        assert set(zf.namelist()) == {"manifest.json", "map_points.bin", "map.pcd"}
        assert zf.read("map.pcd") == b"pcd-bytes"


def test_load_stcm_restores_optional_pcd_payload(tmp_path: Path) -> None:
    target = tmp_path / "roundtrip.slam"
    save_stcm(
        target,
        {
            "version": "slam.v3",
            "scan_mode": "3d",
            "radar_points": [(1.0, 2.0, 3.0)],
            "pcd_file": {"name": "map.pcd", "content": b"pcd-bytes"},
        },
    )

    bundle = load_stcm(target)

    assert bundle["radar_points"] == [(1.0, 2.0, 3.0)]
    assert bundle["pcd_file"]["name"] == "map.pcd"
    assert bundle["pcd_file"]["content"] == b"pcd-bytes"
