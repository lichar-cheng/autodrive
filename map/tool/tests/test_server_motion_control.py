import asyncio
from types import SimpleNamespace

from server.app import main
from server.app.models import MoveCommand


class FakeBridge:
    def __init__(self) -> None:
        self.commands: list[tuple[str, float, float]] = []

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
