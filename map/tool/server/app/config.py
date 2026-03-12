from pydantic import BaseModel, Field


class ServerConfig(BaseModel):
    host: str = "0.0.0.0"
    port: int = 8080
    ws_queue_size: int = 200
    sim_rate_hz: float = 10.0
    lidar_points_per_scan: int = Field(default=700, ge=50, le=5000)
    map_resolution: float = 0.1
    map_size: int = 300
    allowed_clock_drift_sec: float = 5.0


CONFIG = ServerConfig()
