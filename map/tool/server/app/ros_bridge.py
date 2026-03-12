from __future__ import annotations

from dataclasses import dataclass


@dataclass
class RosRuntime:
    enabled: bool
    reason: str


def detect_ros() -> RosRuntime:
    try:
        import rclpy  # noqa: F401
        return RosRuntime(enabled=True, reason="ROS2(rclpy) 可用，将优先接入真实 topic")
    except Exception as exc:
        return RosRuntime(enabled=False, reason=f"ROS2 不可用，启用仿真 topic：{exc}")
