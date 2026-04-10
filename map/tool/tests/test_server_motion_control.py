import asyncio
from types import SimpleNamespace

from server.app import main
from server.app.models import MoveCommand


class FakeBridge:
    def __init__(self) -> None:
        self.commands: list[tuple[str, float, float]] = []
        self.scan_states: list[bool] = []
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
