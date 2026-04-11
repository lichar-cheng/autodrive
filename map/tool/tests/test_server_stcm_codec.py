from pathlib import Path

from server.app.stcm_codec import load_stcm, save_stcm


def test_save_and_load_2d_stcm_round_trip(tmp_path: Path) -> None:
    target = tmp_path / "demo-2d.slam"

    save_stcm(
        target,
        {
            "version": "stcm.v2",
            "scan_mode": "2d",
            "radar_points": [(1.0, 2.0, 0.5), (3.0, 4.0, 1.0)],
            "poi": [{"name": "A"}],
        },
    )

    bundle = load_stcm(target)

    assert bundle["scan_mode"] == "2d"
    assert bundle["poi"] == [{"name": "A"}]
    assert bundle["radar_points"] == [(1.0, 2.0, 0.5), (3.0, 4.0, 1.0)]


def test_save_and_load_3d_stcm_round_trip(tmp_path: Path) -> None:
    target = tmp_path / "demo-3d.slam"

    save_stcm(
        target,
        {
            "version": "stcm.v3",
            "scan_mode": "3d",
            "point_cloud": [(1.0, 2.0, 3.0, 0.5), (4.0, 5.0, 6.0, 1.0)],
            "poi": [{"name": "A"}],
        },
    )

    bundle = load_stcm(target)

    assert bundle["scan_mode"] == "3d"
    assert bundle["poi"] == [{"name": "A"}]
    assert bundle["point_cloud"] == [(1.0, 2.0, 3.0, 0.5), (4.0, 5.0, 6.0, 1.0)]
