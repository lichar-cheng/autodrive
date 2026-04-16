from __future__ import annotations

import asyncio
import math
import random
import time
from dataclasses import dataclass, field

from .topic_bus import TopicBus


@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    wz: float = 0.0
    battery: float = 100.0
    wheel_speed_l: float = 0.0
    wheel_speed_r: float = 0.0
    path: list[dict] = field(default_factory=list)
    poi: list[dict] = field(default_factory=list)
    trajectory: list[dict] = field(default_factory=list)
    gps_track: list[dict] = field(default_factory=list)
    chassis_track: list[dict] = field(default_factory=list)


class Simulator:
    def __init__(self, bus: TopicBus, rate_hz: float = 10.0, points_per_scan: int = 700):
        self.bus = bus
        self.rate_hz = rate_hz
        self.points_per_scan = points_per_scan
        self.state = RobotState()
        self.scanning = False
        self._running = False
        self._task: asyncio.Task | None = None
        self.base_lat = 31.2304
        self.base_lon = 121.4737
        self.wheel_base = 0.52

    async def start(self) -> None:
        self._running = True
        self._task = asyncio.create_task(self._loop())

    async def stop(self) -> None:
        self._running = False
        if self._task:
            await self._task

    def set_motion(self, vx: float, wz: float) -> None:
        self.state.vx = vx
        self.state.wz = wz

    def stop_motion(self) -> None:
        self.state.vx = 0.0
        self.state.wz = 0.0

    def xy_to_latlon(self, x: float, y: float) -> tuple[float, float]:
        dlat = y / 111111.0
        dlon = x / (111111.0 * math.cos(math.radians(self.base_lat)))
        return self.base_lat + dlat, self.base_lon + dlon

    async def _loop(self) -> None:
        dt = 1.0 / self.rate_hz
        while self._running:
            t0 = time.time()
            self.state.yaw += self.state.wz * dt
            self.state.x += self.state.vx * math.cos(self.state.yaw) * dt
            self.state.y += self.state.vx * math.sin(self.state.yaw) * dt
            self.state.battery = max(0.0, self.state.battery - (0.001 + abs(self.state.vx) * 0.002))

            self.state.wheel_speed_l = self.state.vx - self.state.wz * self.wheel_base / 2.0
            self.state.wheel_speed_r = self.state.vx + self.state.wz * self.wheel_base / 2.0

            stamp = time.time()
            lat, lon = self.xy_to_latlon(self.state.x, self.state.y)

            self.state.trajectory.append({"x": self.state.x, "y": self.state.y, "stamp": stamp})
            self.state.gps_track.append({"lat": lat, "lon": lon, "stamp": stamp})
            self.state.chassis_track.append(
                {
                    "vx": self.state.vx,
                    "wz": self.state.wz,
                    "wheel_speed_l": self.state.wheel_speed_l,
                    "wheel_speed_r": self.state.wheel_speed_r,
                    "battery": self.state.battery,
                    "stamp": stamp,
                }
            )
            if len(self.state.trajectory) > 3000:
                self.state.trajectory = self.state.trajectory[-3000:]
                self.state.gps_track = self.state.gps_track[-3000:]
                self.state.chassis_track = self.state.chassis_track[-3000:]

            await self.bus.publish(
                "/robot/pose",
                {
                    "topic": "/robot/pose",
                    "stamp": stamp,
                    "payload": {
                        "x": self.state.x,
                        "y": self.state.y,
                        "yaw": self.state.yaw,
                        "vx": self.state.vx,
                        "wz": self.state.wz,
                    },
                },
            )

            await self.bus.publish(
                "/robot/gps",
                {"topic": "/robot/gps", "stamp": stamp, "payload": {"lat": lat, "lon": lon}},
            )

            await self.bus.publish(
                "/chassis/odom",
                {
                    "topic": "/chassis/odom",
                    "stamp": stamp,
                    "payload": {
                        "x": self.state.x,
                        "y": self.state.y,
                        "yaw": self.state.yaw,
                        "vx": self.state.vx,
                        "wz": self.state.wz,
                    },
                },
            )

            await self.bus.publish(
                "/chassis/status",
                {
                    "topic": "/chassis/status",
                    "stamp": stamp,
                    "payload": {
                        "wheel_speed_l": self.state.wheel_speed_l,
                        "wheel_speed_r": self.state.wheel_speed_r,
                        "battery": round(self.state.battery, 2),
                        "mode": "AUTO_MAP" if self.scanning else "IDLE",
                    },
                },
            )

            for idx in range(4):
                await self.bus.publish(
                    f"/camera/{idx+1}/compressed",
                    {
                        "topic": f"/camera/{idx+1}/compressed",
                        "stamp": stamp,
                        "payload": {
                            "camera_id": idx + 1,
                            "objects": [
                                {
                                    "label": random.choice(["cone", "pedestrian", "car"]),
                                    "confidence": round(random.uniform(0.6, 0.98), 2),
                                }
                                for _ in range(random.randint(1, 3))
                            ],
                        },
                    },
                )

            if self.scanning:
                front = self._generate_lidar(angle_offset=0.0, noise=0.06)
                rear = self._generate_lidar(angle_offset=math.pi, noise=0.09)
                await self.bus.publish("/lidar/front", {"topic": "/lidar/front", "stamp": stamp, "payload": {"points": front}})
                await self.bus.publish("/lidar/rear", {"topic": "/lidar/rear", "stamp": stamp, "payload": {"points": rear}})
                await self.bus.publish("/map/grid", {"topic": "/map/grid", "stamp": stamp, "payload": {"clusters": self._clusters(front + rear)}})

            elapsed = time.time() - t0
            await asyncio.sleep(max(0.0, dt - elapsed))

    def _generate_lidar(self, angle_offset: float, noise: float) -> list[list[float]]:
        pts = []
        for i in range(self.points_per_scan):
            a = angle_offset + (2 * math.pi * i / self.points_per_scan)
            r = 7 + 2 * math.sin(4 * a) + random.uniform(-noise, noise)
            x = self.state.x + r * math.cos(a)
            y = self.state.y + r * math.sin(a)
            intensity = random.uniform(0.3, 1.0)
            pts.append([x, y, intensity])
        return pts

    def _clusters(self, pts: list[list[float]]) -> list[dict]:
        sampled = random.sample(pts, min(20, len(pts)))
        return [{"x": p[0], "y": p[1], "value": round(p[2], 2)} for p in sampled]
