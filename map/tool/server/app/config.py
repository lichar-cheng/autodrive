from __future__ import annotations

from pydantic import BaseModel, Field


class RosTopicConfig(BaseModel):
    odom: str = "/odom"
    gps: str = ""
    occupancy_grid: str = ""
    lidar_front: str = ""
    lidar_rear: str = ""
    lidar_fallback: str = "/scan"
    imu: str = "/imu"
    tf: str = "/tf"
    tf_static: str = "/tf_static"
    odom_frame: str = "odom"
    robot_base_frame: str = "base_link"
    lidar_frame: str = "laser"
    camera_topics: list[str] = Field(default_factory=list)
    cmd_vel: str = "/cmd_vel"


class RosBridgeConfig(BaseModel):
    enabled: bool = True
    node_name: str = "autodrive_map_bridge"
    spin_hz: float = 30.0
    occupancy_stride: int = Field(default=2, ge=1, le=20)
    occupancy_sample_limit: int = Field(default=12000, ge=1000, le=100000)
    lidar_max_points_per_scan: int = Field(default=4000, ge=100, le=50000)
    use_occupancy_grid_for_save: bool = True
    fallback_to_simulator_on_failure: bool = True
    prefer_tf_pose: bool = True
    topics: RosTopicConfig = Field(default_factory=RosTopicConfig)


class ServerConfig(BaseModel):
    host: str = "0.0.0.0"
    port: int = 8080
    ws_queue_size: int = 200
    sim_rate_hz: float = 10.0
    lidar_points_per_scan: int = Field(default=700, ge=50, le=5000)
    map_resolution: float = 0.1
    map_size: int = 300
    allowed_clock_drift_sec: float = 5.0
    ros: RosBridgeConfig = Field(default_factory=RosBridgeConfig)


CONFIG = ServerConfig()
