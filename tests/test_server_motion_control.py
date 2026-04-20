import asyncio
import base64
import subprocess
import zipfile
from pathlib import Path
from types import SimpleNamespace

import pytest

from server.app import main
from server.app.config import ScanModesConfig
from server.app.models import ControlTargetRequest, MoveCommand, SaveMapRequest, StartScanRequest, StopScanRequest


class FakeBridge:
    def __init__(self) -> None:
        self.commands: list[tuple[str, float, float]] = []
        self.scan_states: list[bool] = []
        self.reset_calls = 0
        self.mapping_prereq = {"ready": True, "severity": "ok", "blockers": [], "warnings": [], "checks": {}}

    def publish_cmd_vel(self, velocity: float, yaw_rate: float) -> None:
        self.commands.append(("move", float(velocity), float(yaw_rate)))

    def stop_motion(self) -> None:
        self.commands.append(("stop", 0.0, 0.0))

    def latest_pose(self) -> dict:
        return {}

    def latest_gps(self) -> dict:
        return {}

    def latest_chassis(self) -> dict:
        return {}

    def latest_map_points(self) -> list[tuple[float, float, float]]:
        return []

    def set_scan_active(self, active: bool) -> None:
        self.scan_states.append(bool(active))

    def mapping_prerequisites(self) -> dict:
        return dict(self.mapping_prereq)

    def diagnostics(self) -> dict:
        return {"bridge": "fake"}

    def reset_map(self) -> bool:
        self.reset_calls += 1
        return True


def test_older_move_request_does_not_stop_newer_ros_command(monkeypatch) -> None:
    bridge = FakeBridge()
    monkeypatch.setattr(main, "ros", SimpleNamespace(enabled=True, bridge=bridge))

    async def scenario() -> None:
        first = asyncio.create_task(main.move(MoveCommand(velocity=0.6, yaw_rate=0.0, duration=0.15)))
        await asyncio.sleep(0.05)
        second = asyncio.create_task(main.move(MoveCommand(velocity=0.8, yaw_rate=0.0, duration=0.15)))
        await asyncio.sleep(0.11)

        assert bridge.commands == [
            ("move", 0.6, 0.0),
            ("move", 0.8, 0.0),
        ]

        await asyncio.gather(first, second)
        assert bridge.commands[-1] == ("stop", 0.0, 0.0)
        assert bridge.commands.count(("stop", 0.0, 0.0)) == 1

    asyncio.run(scenario())


def test_scan_start_request_accepts_2d_and_3d() -> None:
    assert StartScanRequest(mode="2d").mode == "2d"
    assert StartScanRequest(mode="3d").mode == "3d"


def test_scan_start_request_rejects_invalid_mode() -> None:
    with pytest.raises(ValueError):
        StartScanRequest(mode="bad")


def test_scan_mode_config_exposes_2d_and_3d_defaults() -> None:
    config = ScanModesConfig()

    assert config.mode_2d.required_nodes == ["/slam_toolbox"]
    assert config.mode_2d.launch_commands == [["ros2", "launch", "slam_toolbox", "online_async_launch.py"]]
    assert config.mode_3d.required_nodes == ["/point_lio", "/slam_toolbox"]
    assert config.mode_3d.pcd_output_path == "/tmp/point_lio_map.pcd"


def test_start_scan_rejects_when_mapping_prereq_not_ready(monkeypatch) -> None:
    bridge = FakeBridge()
    bridge.mapping_prereq = {
        "ready": False,
        "severity": "error",
        "blockers": ["tf base->lidar missing", "odom topic stale"],
        "warnings": [],
        "checks": {"tf_tree": {"ok": False}, "odom": {"ok": False}},
    }
    monkeypatch.setattr(main, "ros", SimpleNamespace(enabled=True, bridge=bridge, reason="ok"))
    monkeypatch.setattr(main, "sim", SimpleNamespace(_running=False, scanning=False))
    monkeypatch.setattr(main, "_ensure_scan_mode_dependencies", lambda mode: {"required_nodes": [], "missing_nodes": [], "started_nodes": [], "errors": []})
    main._reset_scan_session()

    result = asyncio.run(main.start_scan())

    assert result["ok"] is False
    assert result["scan_active"] is False
    assert result["mapping_prereq"]["ready"] is False
    assert bridge.scan_states == []
    assert main.SCAN_SESSION["active"] is False


def test_start_scan_rejects_invalid_mode(monkeypatch) -> None:
    bridge = FakeBridge()
    monkeypatch.setattr(main, "ros", SimpleNamespace(enabled=True, bridge=bridge, reason="ok"))
    monkeypatch.setattr(main, "sim", SimpleNamespace(_running=False, scanning=False))

    result = asyncio.run(main.start_scan(StartScanRequest.model_construct(mode="bad")))

    assert result["ok"] is False
    assert result["reason"] == "invalid_scan_mode"
    assert result["scan_active"] is False


def test_start_scan_records_mode_and_dependency_status(monkeypatch) -> None:
    bridge = FakeBridge()
    monkeypatch.setattr(main, "ros", SimpleNamespace(enabled=True, bridge=bridge, reason="ok"))
    monkeypatch.setattr(main, "sim", SimpleNamespace(_running=False, scanning=False))
    monkeypatch.setattr(main, "_mapping_prereq_summary", lambda: {"ready": True, "severity": "ok", "blockers": [], "warnings": [], "checks": {}})
    monkeypatch.setattr(main, "_ensure_scan_mode_dependencies", lambda mode: {"required_nodes": ["/slam_toolbox"], "missing_nodes": [], "started_nodes": [], "errors": []})
    main._reset_scan_session()

    result = asyncio.run(main.start_scan(StartScanRequest(mode="2d")))

    assert result["ok"] is True
    assert result["scan_mode"] == "2d"
    assert result["dependency_status"]["required_nodes"] == ["/slam_toolbox"]
    assert bridge.scan_states == [True]
    assert main.SCAN_SESSION["mode"] == "2d"


def test_start_scan_rejects_when_scan_already_active(monkeypatch) -> None:
    bridge = FakeBridge()
    monkeypatch.setattr(main, "ros", SimpleNamespace(enabled=True, bridge=bridge, reason="ok"))
    monkeypatch.setattr(main, "sim", SimpleNamespace(_running=False, scanning=False))
    main._reset_scan_session()
    main.SCAN_SESSION["active"] = True

    result = asyncio.run(main.start_scan(StartScanRequest(mode="2d")))

    assert result["ok"] is False
    assert result["reason"] == "scan_already_active"


def test_start_scan_returns_node_start_failed_when_dependencies_fail(monkeypatch) -> None:
    bridge = FakeBridge()
    monkeypatch.setattr(main, "ros", SimpleNamespace(enabled=True, bridge=bridge, reason="ok"))
    monkeypatch.setattr(main, "sim", SimpleNamespace(_running=False, scanning=False))
    monkeypatch.setattr(main, "_mapping_prereq_summary", lambda: {"ready": True, "severity": "ok", "blockers": [], "warnings": [], "checks": {}})
    monkeypatch.setattr(
        main,
        "_ensure_scan_mode_dependencies",
        lambda mode: {"required_nodes": ["/point_lio"], "missing_nodes": ["/point_lio"], "started_nodes": [], "errors": ["failed to launch"]},
    )
    main._reset_scan_session()

    result = asyncio.run(main.start_scan(StartScanRequest(mode="3d")))

    assert result["ok"] is False
    assert result["reason"] == "node_start_failed"
    assert result["dependency_status"]["missing_nodes"] == ["/point_lio"]
    assert bridge.scan_states == []


def test_stop_scan_rejects_when_scan_not_active(monkeypatch) -> None:
    bridge = FakeBridge()
    monkeypatch.setattr(main, "ros", SimpleNamespace(enabled=True, bridge=bridge, reason="ok"))
    monkeypatch.setattr(main, "sim", SimpleNamespace(_running=False, scanning=False))
    main._reset_scan_session()

    result = asyncio.run(main.stop_scan(StopScanRequest(mode="2d")))

    assert result["ok"] is False
    assert result["reason"] == "scan_not_active"


def test_check_required_nodes_reports_missing_nodes(monkeypatch) -> None:
    monkeypatch.setattr(main, "_list_ros_nodes", lambda: ["/slam_toolbox"])

    result = main._check_required_nodes(["/slam_toolbox", "/point_lio"])

    assert result["required_nodes"] == ["/slam_toolbox", "/point_lio"]
    assert result["missing_nodes"] == ["/point_lio"]
    assert result["errors"] == []


def test_check_required_nodes_reports_ros2_node_list_failure(monkeypatch) -> None:
    def raise_error():
        raise subprocess.CalledProcessError(returncode=1, cmd=["ros2", "node", "list"], stderr="boom")

    monkeypatch.setattr(main, "_list_ros_nodes", raise_error)

    result = main._check_required_nodes(["/slam_toolbox"])

    assert result["missing_nodes"] == ["/slam_toolbox"]
    assert result["errors"] == ["boom"]


def test_ensure_scan_mode_dependencies_launches_missing_nodes(monkeypatch) -> None:
    monkeypatch.setattr(
        main,
        "_scan_mode_config",
        lambda mode: SimpleNamespace(
            required_nodes=["/point_lio"],
            launch_commands=[["ros2", "launch", "point_lio", "mapping.launch.py"]],
        ),
    )
    states = iter(
        [
            {"required_nodes": ["/point_lio"], "missing_nodes": ["/point_lio"], "started_nodes": [], "errors": []},
            {"required_nodes": ["/point_lio"], "missing_nodes": [], "started_nodes": [], "errors": []},
        ]
    )
    monkeypatch.setattr(main, "_check_required_nodes", lambda nodes: next(states))
    launches = []
    monkeypatch.setattr(main, "_launch_scan_mode_command", lambda argv: launches.append(argv) or (True, "started"))

    result = main._ensure_scan_mode_dependencies("3d")

    assert launches == [["ros2", "launch", "point_lio", "mapping.launch.py"]]
    assert result["missing_nodes"] == []
    assert result["started_nodes"] == ["/point_lio"]


def test_stop_scan_3d_returns_base64_pcd_when_file_exists(tmp_path: Path, monkeypatch) -> None:
    bridge = FakeBridge()
    pcd = tmp_path / "map.pcd"
    pcd.write_bytes(b"pcd-bytes")
    monkeypatch.setattr(main, "ros", SimpleNamespace(enabled=True, bridge=bridge, reason="ok"))
    monkeypatch.setattr(main, "sim", SimpleNamespace(_running=False, scanning=False))
    monkeypatch.setattr(main, "_pcd_output_path_for_mode", lambda mode: pcd)
    main._reset_scan_session()
    main.SCAN_SESSION["active"] = True
    main.SCAN_SESSION["mode"] = "3d"

    result = asyncio.run(main.stop_scan(StopScanRequest(mode="3d")))

    assert result["ok"] is True
    assert result["scan_mode"] == "3d"
    assert result["pcd_file"]["name"] == "map.pcd"
    assert result["pcd_file"]["content"] == base64.b64encode(b"pcd-bytes").decode("ascii")
    assert bridge.scan_states == [False]


def test_stop_scan_3d_requires_configured_pcd_path(monkeypatch) -> None:
    bridge = FakeBridge()
    monkeypatch.setattr(main, "ros", SimpleNamespace(enabled=True, bridge=bridge, reason="ok"))
    monkeypatch.setattr(main, "sim", SimpleNamespace(_running=False, scanning=False))
    monkeypatch.setattr(main, "_pcd_output_path_for_mode", lambda mode: None)
    main._reset_scan_session()
    main.SCAN_SESSION["active"] = True
    main.SCAN_SESSION["mode"] = "3d"

    result = asyncio.run(main.stop_scan(StopScanRequest(mode="3d")))

    assert result["ok"] is False
    assert result["reason"] == "pcd_path_not_configured"


def test_stop_scan_3d_rejects_missing_pcd_file(tmp_path: Path, monkeypatch) -> None:
    bridge = FakeBridge()
    monkeypatch.setattr(main, "ros", SimpleNamespace(enabled=True, bridge=bridge, reason="ok"))
    monkeypatch.setattr(main, "sim", SimpleNamespace(_running=False, scanning=False))
    monkeypatch.setattr(main, "_pcd_output_path_for_mode", lambda mode: tmp_path / "missing.pcd")
    main._reset_scan_session()
    main.SCAN_SESSION["active"] = True
    main.SCAN_SESSION["mode"] = "3d"

    result = asyncio.run(main.stop_scan(StopScanRequest(mode="3d")))

    assert result["ok"] is False
    assert result["reason"] == "pcd_file_missing"


def test_health_includes_mapping_prereq_summary(monkeypatch) -> None:
    bridge = FakeBridge()
    bridge.mapping_prereq = {
        "ready": False,
        "severity": "warn",
        "blockers": [],
        "warnings": ["ws stream unstable"],
        "checks": {"network": {"ok": False, "level": "warn"}},
    }
    monkeypatch.setattr(main, "ros", SimpleNamespace(enabled=True, bridge=bridge, reason="ok"))
    monkeypatch.setattr(main, "sim", SimpleNamespace(_running=False))
    main._reset_scan_session()

    result = asyncio.run(main.health())

    assert result["mapping_ready"] is False
    assert result["mapping_status"] == "warn"
    assert result["mapping_blockers"] == []
    assert "ws stream unstable" in result["mapping_warnings"]
    assert "no websocket clients connected" in result["mapping_warnings"]
    assert result["scan_mode"] == "2d"
    assert result["dependency_status"]["required_nodes"] == []
    assert result["pcd_transfer_state"] == "idle"


def test_current_map_points_ignores_scan_accumulation_without_occupancy_grid(monkeypatch) -> None:
    monkeypatch.setattr(main, "latest_points", [(9.0, 9.0, 1.0)])
    monkeypatch.setattr(
        main,
        "ros",
        SimpleNamespace(
            enabled=True,
            bridge=SimpleNamespace(latest_map_points=lambda: []),
        ),
    )
    main.SCAN_SESSION["accumulated"] = {"1:1": {"x": 1.0, "y": 1.0, "intensity": 1.0}}

    points = main._current_map_points()

    assert points == [(9.0, 9.0, 1.0)]


def test_save_map_fails_when_no_map_source_is_available(monkeypatch) -> None:
    monkeypatch.setattr(main, "latest_points", [])
    monkeypatch.setattr(
        main,
        "ros",
        SimpleNamespace(
            enabled=True,
            bridge=SimpleNamespace(latest_map_points=lambda: [], latest_pose=lambda: {}, latest_gps=lambda: {}, latest_imu=lambda: {}),
        ),
    )
    monkeypatch.setattr(main, "sim", SimpleNamespace(state=SimpleNamespace(x=0.0, y=0.0, yaw=0.0, poi=[], path=[], trajectory=[], gps_track=[], chassis_track=[])))

    result = asyncio.run(main.save_map(SaveMapRequest(name="demo", notes="demo", voxel_size=0.1, reset_after_save=False)))

    assert result["ok"] is False
    assert result["reason"] == "map_unavailable"


def test_save_map_writes_new_slam_layout_without_pcd(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr(main, "latest_points", [(1.0, 2.0, 1.0)])
    monkeypatch.setattr(main, "map_dir", tmp_path)
    monkeypatch.setattr(
        main,
        "ros",
        SimpleNamespace(
            enabled=False,
            bridge=None,
            reason="disabled",
        ),
    )
    monkeypatch.setattr(
        main,
        "sim",
        SimpleNamespace(
            state=SimpleNamespace(x=0.0, y=0.0, yaw=0.0, poi=[], path=[], trajectory=[], gps_track=[], chassis_track=[]),
        ),
    )
    main._reset_scan_session()

    result = asyncio.run(main.save_map(SaveMapRequest(name="demo", notes="demo", voxel_size=0.1, reset_after_save=False)))

    assert result["ok"] is True
    assert result["contains"]["pcd"] is False
    target = Path(result["file"])
    with zipfile.ZipFile(target, "r") as zf:
        assert set(zf.namelist()) == {"manifest.json", "map_points.bin"}


def test_save_map_writes_optional_pcd_into_slam(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr(main, "latest_points", [(1.0, 2.0, 1.0)])
    monkeypatch.setattr(main, "map_dir", tmp_path)
    monkeypatch.setattr(
        main,
        "ros",
        SimpleNamespace(
            enabled=False,
            bridge=None,
            reason="disabled",
        ),
    )
    monkeypatch.setattr(
        main,
        "sim",
        SimpleNamespace(
            state=SimpleNamespace(x=0.0, y=0.0, yaw=0.0, poi=[], path=[], trajectory=[], gps_track=[], chassis_track=[]),
        ),
    )
    main._reset_scan_session()
    main.SCAN_SESSION["mode"] = "3d"
    main.SCAN_SESSION["pcd_file"] = {"name": "map.pcd", "content": b"pcd-bytes"}

    result = asyncio.run(main.save_map(SaveMapRequest(name="demo3d", notes="demo", voxel_size=0.1, reset_after_save=False)))

    assert result["ok"] is True
    assert result["contains"]["pcd"] is True
    target = Path(result["file"])
    with zipfile.ZipFile(target, "r") as zf:
        assert set(zf.namelist()) == {"manifest.json", "map_points.bin", "map.pcd"}
        assert zf.read("map.pcd") == b"pcd-bytes"


def test_reset_map_calls_bridge_and_clears_cached_points(monkeypatch) -> None:
    bridge = FakeBridge()
    monkeypatch.setattr(main, "latest_points", [(1.0, 2.0, 1.0)])
    monkeypatch.setattr(main, "ros", SimpleNamespace(enabled=True, bridge=bridge, reason="ok"))

    result = asyncio.run(main.reset_map())

    assert result["ok"] is True
    assert bridge.reset_calls == 1
    assert main.latest_points == []


def test_set_control_target_updates_server_target_without_sleeping(monkeypatch) -> None:
    bridge = FakeBridge()
    monkeypatch.setattr(main, "ros", SimpleNamespace(enabled=True, bridge=bridge))
    monkeypatch.setattr(main, "CONTROL_TARGET", {"velocity": 0.0, "yaw_rate": 0.0, "updated_at": 0.0})

    result = asyncio.run(main.set_control_target(ControlTargetRequest(velocity=0.6, yaw_rate=0.2)))

    assert result["ok"] is True
    assert bridge.commands == [("move", 0.6, 0.2)]
    assert main.CONTROL_TARGET["velocity"] == 0.6
    assert main.CONTROL_TARGET["yaw_rate"] == 0.2
