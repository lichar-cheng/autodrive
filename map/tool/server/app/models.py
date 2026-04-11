from __future__ import annotations

from typing import Literal, List, Optional

from pydantic import BaseModel, Field


AuthMode = Literal["local", "cloud"]
ScanMode = Literal["2d", "3d"]


class MoveCommand(BaseModel):
    velocity: float = Field(default=0.5, ge=-2.0, le=2.0)
    yaw_rate: float = Field(default=0.0, ge=-2.0, le=2.0)
    duration: float = Field(default=1.0, ge=0.05, le=10.0)


class SaveMapRequest(BaseModel):
    name: str = "session"
    notes: str = ""
    voxel_size: Optional[float] = Field(default=None, ge=0.02, le=1.0)
    scan_mode: Optional[ScanMode] = None
    reset_after_save: bool = False


class LoadMapRequest(BaseModel):
    filename: str


class PathNode(BaseModel):
    x: float
    y: float
    lat: Optional[float] = None
    lon: Optional[float] = None


class PlanPathRequest(BaseModel):
    nodes: List[PathNode]


class PoiPoint(BaseModel):
    name: str
    x: float
    y: float
    yaw: float = 0.0
    lat: Optional[float] = None
    lon: Optional[float] = None


class AddPoiRequest(BaseModel):
    poi: PoiPoint


class BackendConnectionDescriptor(BaseModel):
    auth_mode: AuthMode = "local"
    backend_host: str
    backend_port: int = Field(default=8080, ge=1, le=65535)
    token: str = ""
    expires_at: Optional[str] = None


class ScanModeConfig(BaseModel):
    scan_mode: ScanMode = "2d"


class ScanModeRequest(BaseModel):
    scan_mode: ScanMode = "2d"
