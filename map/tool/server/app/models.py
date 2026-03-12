from __future__ import annotations

from pydantic import BaseModel, Field
from typing import List, Optional


class MoveCommand(BaseModel):
    velocity: float = Field(default=0.5, ge=0.0, le=2.0)
    yaw_rate: float = Field(default=0.0, ge=-2.0, le=2.0)
    duration: float = Field(default=1.0, ge=0.05, le=10.0)


class SaveMapRequest(BaseModel):
    name: str = "session"
    notes: str = ""


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
    lat: Optional[float] = None
    lon: Optional[float] = None


class AddPoiRequest(BaseModel):
    poi: PoiPoint
