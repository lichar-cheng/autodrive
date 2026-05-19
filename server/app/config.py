from __future__ import annotations

from typing import Any

from pydantic import BaseModel, Field, model_validator


class ScanLaunchCommandConfig(BaseModel):
    command: list[str] = Field(default_factory=list)
    processes: list[str] = Field(default_factory=list)


class ScanModeRuntimeConfig(BaseModel):
    launch_commands: list[ScanLaunchCommandConfig] = Field(default_factory=list)
    pcd_output_path: str = ""

    @model_validator(mode="before")
    @classmethod
    def _normalize_launch_commands(cls, data: Any) -> Any:
        if not isinstance(data, dict):
            return data
        commands = data.get("launch_commands")
        if not isinstance(commands, list):
            return data
        normalized: list[Any] = []
        changed = False
        for item in commands:
            if isinstance(item, list):
                normalized.append({"command": item, "processes": [str(item[-1])] if item else []})
                changed = True
            else:
                normalized.append(item)
        if changed:
            updated = dict(data)
            updated["launch_commands"] = normalized
            return updated
        return data


class ScanModesConfig(BaseModel):
    mode_2d: ScanModeRuntimeConfig = Field(
        default_factory=lambda: ScanModeRuntimeConfig(
            launch_commands=[
                ScanLaunchCommandConfig(
                    command=["ros2", "launch", "slam_toolbox", "online_async_launch.py"],
                    processes=["online_async_launch.py"],
                ),
            ],
        )
    )
    mode_3d: ScanModeRuntimeConfig = Field(
        default_factory=lambda: ScanModeRuntimeConfig(
            launch_commands=[
                ScanLaunchCommandConfig(
                    command=["ros2", "launch", "caddie_hardware", "navigation_hardware.launch.py"],
                    processes=["navigation_hardware.launch.py"],
                )
            ],
            pcd_output_path="/tmp/point_lio_map.pcd",
        )
    )


class RosTopicConfig(BaseModel):
    odom: str = "/odom"
    occupancy_grid: str = "/map"
    tf: str = "/tf"
    tf_static: str = "/tf_static"
    odom_frame: str = "odom"
    robot_base_frame: str = "base_link"
    cmd_vel: str = "/cmd_vel"


class RosBridgeConfig(BaseModel):
    enabled: bool = True
    node_name: str = "autodrive_server_bridge"
    spin_hz: float = 30.0
    prefer_tf_pose: bool = True
    topics: RosTopicConfig = Field(default_factory=RosTopicConfig)


class ServerConfig(BaseModel):
    host: str = "0.0.0.0"
    port: int = 8180
    ws_queue_size: int = 200
    allowed_clock_drift_sec: float = 5.0
    ros: RosBridgeConfig = Field(default_factory=RosBridgeConfig)
    scan_modes: ScanModesConfig = Field(default_factory=ScanModesConfig)


CONFIG = ServerConfig()
