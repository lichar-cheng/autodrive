import asyncio
from pathlib import Path
from types import SimpleNamespace

from server.app import main
from server.app.models import MoveCommand


class FakeBridge:
    def __init__(self) -> None:
        self.commands: list[tuple[str, float, float]] = []
        self.scan_states: list[bool] = []
        self.mapping_prereq = {"ready": True, "severity": "ok", "blockers": [], "warnings": [], "checks": {}}
        self.map_points: list[tuple] = []
        self.config = SimpleNamespace(scan_mode="2d")

    def publish_cmd_vel(self, velocity: float, yaw_rate: float) -> None:
        self.commands.append(("move", float(velocity), float(yaw_rate)))

    def stop_motion(self) -> None:
        self.commands.append(("stop", 0.0, 0.0))

    def latest_pose(self) -> dict:
        return {}

    def latest_gps(self) -> dict:
        return {}

    def latest_imu(self) -> dict:
        return {}

    def latest_chassis(self) -> dict:
        return {}

    def latest_map_points(self) -> list[tuple]:
        return list(self.map_points)

    def set_scan_active(self, active: bool) -> None:
        self.scan_states.append(bool(active))

    def mapping_prerequisites(self) -> dict:
        return dict(self.mapping_prereq)

    def diagnostics(self) -> dict:
        return {"bridge": "fake"}


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
    main._reset_scan_session()

    result = asyncio.run(main.start_scan())

    assert result["ok"] is False
    assert result["scan_active"] is False
    assert result["mapping_prereq"]["ready"] is False
    assert bridge.scan_states == []
    assert main.SCAN_SESSION["active"] is False


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

    result = asyncio.run(main.health())

    assert result["mapping_ready"] is False
    assert result["mapping_status"] == "warn"
    assert result["mapping_blockers"] == []
    assert "ws stream unstable" in result["mapping_warnings"]
    assert "no websocket clients connected" in result["mapping_warnings"]


def test_save_map_uses_point_cloud_payload_in_3d_mode(monkeypatch, tmp_path) -> None:
    bridge = FakeBridge()
    bridge.map_points = [(1.0, 2.0, 3.0, 0.5), (4.0, 5.0, 6.0, 1.0)]
    monkeypatch.setattr(main, "ros", SimpleNamespace(enabled=True, bridge=bridge, reason="ok"))
    monkeypatch.setattr(main, "map_dir", tmp_path)
    monkeypatch.setattr(main.CONFIG, "scan_mode", "3d")

    result = asyncio.run(main.save_map(main.SaveMapRequest(name="demo", scan_mode="3d")))

    assert result["ok"] is True
    assert result["scan_mode"] == "3d"
    assert result["contains"]["point_cloud"] == 2
    bundle = main.load_stcm(tmp_path / Path(result["file"]).name)
    assert bundle["scan_mode"] == "3d"
    assert bundle["point_cloud"] == bridge.map_points


def test_set_scan_mode_updates_runtime_config_and_resets_scan(monkeypatch) -> None:
    bridge = FakeBridge()
    monkeypatch.setattr(main, "ros", SimpleNamespace(enabled=True, bridge=bridge, reason="ok"))
    monkeypatch.setattr(main, "sim", SimpleNamespace(_running=False, scanning=False))
    main.SCAN_SESSION["active"] = True
    main.SCAN_SESSION["raw_points"] = 42

    result = asyncio.run(main.set_scan_mode(main.ScanModeRequest(scan_mode="3d")))

    assert result["ok"] is True
    assert result["scan_mode"] == "3d"
    assert main.CONFIG.scan_mode == "3d"
    assert main.CONFIG.ros.scan_mode == "3d"
    assert bridge.config.scan_mode == "3d"
    assert main.SCAN_SESSION["active"] is False
    assert main.SCAN_SESSION["raw_points"] == 0
