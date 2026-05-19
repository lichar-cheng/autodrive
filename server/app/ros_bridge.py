from __future__ import annotations

import asyncio
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Any

MAPPING_TOPIC_MAX_AGE_SEC = 2.5
MAPPING_TF_MAX_AGE_SEC = 2.5


@dataclass
class RosRuntime:
    enabled: bool
    reason: str
    bridge: "RosBridge | None" = None


@dataclass
class RosBridgeState:
    connected_topics: list[str] = field(default_factory=list)
    last_message_time_by_topic: dict[str, float] = field(default_factory=dict)
    last_pose: dict[str, float] = field(default_factory=lambda: {"x": 0.0, "y": 0.0, "yaw": 0.0, "vx": 0.0, "wz": 0.0})
    pose_history: list[dict[str, Any]] = field(default_factory=list)
    latest_occupancy_payload: dict[str, Any] = field(default_factory=dict)
    latest_occupancy_points: list[tuple[float, float, float]] = field(default_factory=list)
    latest_occupancy_frame: str = ""
    tf_tree: dict[tuple[str, str], dict[str, float | str]] = field(default_factory=dict)
    scanning: bool = False


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _normalize_frame(frame_id: str) -> str:
    return str(frame_id).lstrip("/")


def _compose_transform(
    first: dict[str, float | str],
    second: dict[str, float | str],
    source: str | None = None,
) -> dict[str, float | str]:
    first_yaw = float(first["yaw"])
    second_yaw = float(second["yaw"])
    cos_yaw = math.cos(first_yaw)
    sin_yaw = math.sin(first_yaw)
    second_tx = float(second["tx"])
    second_ty = float(second["ty"])
    return {
        "tx": float(first["tx"]) + cos_yaw * second_tx - sin_yaw * second_ty,
        "ty": float(first["ty"]) + sin_yaw * second_tx + cos_yaw * second_ty,
        "tz": float(first["tz"]) + float(second["tz"]),
        "yaw": first_yaw + second_yaw,
        "source": source or f"{first['source']}+{second['source']}",
        "stamp": max(float(first.get("stamp", 0.0) or 0.0), float(second.get("stamp", 0.0) or 0.0)),
    }


def _stamp_to_seconds(stamp: Any) -> float:
    try:
        return float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) / 1_000_000_000.0
    except Exception:  # noqa: BLE001
        return 0.0


def _build_tf_static_qos(qos_module: Any, sensor_profile: Any) -> Any:
    try:
        return qos_module.QoSProfile(
            depth=1,
            durability=qos_module.DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=qos_module.ReliabilityPolicy.RELIABLE,
            history=qos_module.HistoryPolicy.KEEP_LAST,
        )
    except Exception:  # noqa: BLE001
        return sensor_profile


class RosBridge:
    def __init__(self, bus, loop: asyncio.AbstractEventLoop, config) -> None:
        self.bus = bus
        self.loop = loop
        self.config = config
        self.state = RosBridgeState()
        # diagnostics() 和 mapping_prerequisites() 都会重入 TF 查询，因此这里使用可重入锁。
        self._lock = threading.RLock()
        self._running = False
        self._spin_thread: threading.Thread | None = None
        self._rclpy = None
        self._node = None
        self._cmd_pub = None
        self._twist_cls = None
        self._reset_client = None
        self._reset_request_cls = None
        self._subscriptions: list[Any] = []
        self._import_error: str | None = None

    def import_error(self) -> str | None:
        return self._import_error

    def start(self) -> None:
        if self._running:
            return

        try:
            import rclpy
            from geometry_msgs.msg import Twist
            from nav_msgs.msg import OccupancyGrid, Odometry
            from rclpy import qos as rclpy_qos
            from rclpy.qos import qos_profile_sensor_data
            from tf2_msgs.msg import TFMessage
        except Exception as exc:  # noqa: BLE001
            self._import_error = str(exc)
            raise RuntimeError(f"ROS imports failed: {exc}") from exc

        self._rclpy = rclpy
        self._twist_cls = Twist
        if not rclpy.ok():
            rclpy.init(args=None)

        self._node = rclpy.create_node(self.config.node_name)
        self._cmd_pub = self._node.create_publisher(Twist, self.config.topics.cmd_vel, 10)
        try:
            from slam_toolbox.srv import Reset

            self._reset_client = self._node.create_client(Reset, "/slam_toolbox/reset")
            self._reset_request_cls = Reset.Request
        except Exception:  # noqa: BLE001
            self._reset_client = None
            self._reset_request_cls = None

        tf_static_qos = _build_tf_static_qos(rclpy_qos, qos_profile_sensor_data)
        self._subscriptions = [
            self._node.create_subscription(Odometry, self.config.topics.odom, self._on_odom, qos_profile_sensor_data),
            self._node.create_subscription(TFMessage, self.config.topics.tf, self._on_tf, qos_profile_sensor_data),
            self._node.create_subscription(TFMessage, self.config.topics.tf_static, self._on_tf_static, tf_static_qos),
        ]

        if self.config.topics.occupancy_grid:
            self._subscriptions.append(
                self._node.create_subscription(
                    OccupancyGrid, self.config.topics.occupancy_grid, self._on_occupancy_grid, qos_profile_sensor_data
                )
            )

        

        self.state.connected_topics = [
            self.config.topics.odom,
            self.config.topics.tf,
            self.config.topics.tf_static,
        ]
        if self.config.topics.occupancy_grid:
            self.state.connected_topics.append(self.config.topics.occupancy_grid)
        self._running = True
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True, name="ros-bridge-spin")
        self._spin_thread.start()

    def stop(self) -> None:
        self._running = False
        if self._spin_thread:
            self._spin_thread.join(timeout=2.0)
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._rclpy is not None and self._rclpy.ok():
            self._rclpy.shutdown()

    def diagnostics(self) -> dict[str, Any]:
        with self._lock:
            return {
                "connected_topics": list(self.state.connected_topics),
                "last_message_time_by_topic": dict(self.state.last_message_time_by_topic),
                "scanning": self.state.scanning,
                "occupancy_points": len(self.state.latest_occupancy_points),
                "has_occupancy_grid": bool(self.config.topics.occupancy_grid),
                "tf_pairs": [f"{parent}->{child}" for parent, child in self.state.tf_tree.keys()],
            }

    def mapping_prerequisites(self, now: float | None = None) -> dict[str, Any]:
        now = float(now if now is not None else time.time())
        checks: dict[str, dict[str, Any]] = {}
        blockers: list[str] = []
        warnings: list[str] = []

        def add_topic_check(name: str, topic_name: str, required: bool = True) -> None:
            if not topic_name:
                checks[name] = {"ok": not required, "required": required, "topic": topic_name, "reason": "not configured"}
                if required:
                    blockers.append(f"{name} topic not configured")
                return
            last_at = float(self.state.last_message_time_by_topic.get(topic_name, 0.0))
            age_sec = round(max(0.0, now - last_at), 3) if last_at > 0 else None
            ok = last_at > 0 and (now - last_at) <= MAPPING_TOPIC_MAX_AGE_SEC
            checks[name] = {"ok": ok, "required": required, "topic": topic_name, "age_sec": age_sec}
            if required and not ok:
                blockers.append(f"{name} topic stale or missing")

        with self._lock:
            add_topic_check("odom", self.config.topics.odom, required=True)
            add_topic_check("tf", self.config.topics.tf, required=True)
            add_topic_check("occupancy_grid", self.config.topics.occupancy_grid, required=True)
            last_tf_static = float(self.state.last_message_time_by_topic.get(self.config.topics.tf_static, 0.0))
            tf_pairs = [f"{parent}->{child}" for parent, child in self.state.tf_tree.keys()]

        tf_static_age_sec = round(max(0.0, now - last_tf_static), 3) if last_tf_static > 0 else None
        checks["tf_static"] = {
            "ok": last_tf_static > 0 and (now - last_tf_static) <= MAPPING_TF_MAX_AGE_SEC,
            "required": False,
            "topic": self.config.topics.tf_static,
            "age_sec": tf_static_age_sec,
            "pairs": tf_pairs,
        }
        if not tf_pairs:
            warnings.append("tf tree is empty")

        readiness = not blockers
        return {
            "ready": readiness,
            "severity": "ok" if readiness and not warnings else "warn" if not blockers else "error",
            "blockers": blockers,
            "warnings": warnings,
            "checks": checks,
        }

    def latest_map_points(self) -> list[tuple[float, float, float]]:
        with self._lock:
            return list(self.state.latest_occupancy_points)

    def latest_pose(self) -> dict[str, float]:
        with self._lock:
            return dict(self.state.last_pose)

    def pose_for_stamp(self, stamp: float) -> dict[str, float]:
        with self._lock:
            if not self.state.pose_history:
                return dict(self.state.last_pose)
            best = self.state.pose_history[-1]
            best_delta = abs(float(best.get("stamp", 0.0)) - float(stamp))
            for item in self.state.pose_history:
                delta = abs(float(item.get("stamp", 0.0)) - float(stamp))
                if delta < best_delta:
                    best = item
                    best_delta = delta
            return dict(best["pose"])


    def set_scan_active(self, active: bool) -> None:
        with self._lock:
            self.state.scanning = active

    def publish_cmd_vel(self, velocity: float, yaw_rate: float) -> None:
        if self._cmd_pub is None or self._node is None or self._twist_cls is None:
            return
        twist = self._twist_cls()
        twist.linear.x = float(velocity)
        twist.angular.z = float(yaw_rate)
        self._cmd_pub.publish(twist)

    def stop_motion(self) -> None:
        self.publish_cmd_vel(0.0, 0.0)

    def reset_map(self, timeout_sec: float = 3.0) -> bool:
        if self._reset_client is None or self._reset_request_cls is None:
            return False
        try:
            if not self._reset_client.wait_for_service(timeout_sec=float(timeout_sec)):
                return False
            future = self._reset_client.call_async(self._reset_request_cls())
            deadline = time.time() + max(0.1, float(timeout_sec))
            while time.time() < deadline:
                if future.done():
                    future.result()
                    with self._lock:
                        self.state.latest_occupancy_payload = {}
                        self.state.latest_occupancy_points = []
                    return True
                time.sleep(0.02)
        except Exception:  # noqa: BLE001
            return False
        return False

    def _spin_loop(self) -> None:
        assert self._rclpy is not None
        while self._running and self._rclpy.ok():
            self._rclpy.spin_once(self._node, timeout_sec=1.0 / max(1.0, float(self.config.spin_hz)))

    def _publish_async(self, topic: str, payload: dict[str, Any], stamp: float | None = None) -> None:
        message = {
            "topic": topic,
            "stamp": float(stamp if stamp is not None else time.time()),
            "payload": payload,
        }
        asyncio.run_coroutine_threadsafe(self.bus.publish(topic, message), self.loop)

    def _mark_topic(self, topic_name: str) -> None:
        with self._lock:
            self.state.last_message_time_by_topic[topic_name] = time.time()

    def _store_transform(self, parent: str, child: str, tx: float, ty: float, tz: float, yaw: float, source: str) -> None:
        key = (_normalize_frame(parent), _normalize_frame(child))
        with self._lock:
            self.state.tf_tree[key] = {
                "tx": float(tx),
                "ty": float(ty),
                "tz": float(tz),
                "yaw": float(yaw),
                "source": source,
                "stamp": time.time(),
            }

    def _lookup_transform(self, parent: str, child: str) -> dict[str, float | str] | None:
        normalized_parent = _normalize_frame(parent)
        normalized_child = _normalize_frame(child)
        with self._lock:
            tf_tree = {key: dict(value) for key, value in self.state.tf_tree.items()}

        def inverse(transform: dict[str, float | str]) -> dict[str, float | str]:
            yaw = -float(transform["yaw"])
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            tx = -float(transform["tx"]) * cos_yaw + float(transform["ty"]) * sin_yaw
            ty = -float(transform["tx"]) * sin_yaw - float(transform["ty"]) * cos_yaw
            return {
                "tx": tx,
                "ty": ty,
                "tz": -float(transform["tz"]),
                "yaw": yaw,
                "source": f"inverse:{transform['source']}",
                "stamp": transform["stamp"],
            }

        direct = tf_tree.get((normalized_parent, normalized_child))
        if direct is not None:
            return dict(direct)
        reverse = tf_tree.get((normalized_child, normalized_parent))
        if reverse is not None:
            return inverse(reverse)

        queue: list[tuple[str, dict[str, float | str]]] = [
            (
                normalized_parent,
                {
                    "tx": 0.0,
                    "ty": 0.0,
                    "tz": 0.0,
                    "yaw": 0.0,
                    "source": "identity",
                    "stamp": time.time(),
                },
            )
        ]
        visited = {normalized_parent}
        while queue:
            current_frame, current_transform = queue.pop(0)
            if current_frame == normalized_child:
                return current_transform
            for (edge_parent, edge_child), edge_transform in tf_tree.items():
                next_frame = None
                step_transform = None
                if edge_parent == current_frame and edge_child not in visited:
                    next_frame = edge_child
                    step_transform = dict(edge_transform)
                elif edge_child == current_frame and edge_parent not in visited:
                    next_frame = edge_parent
                    step_transform = inverse(edge_transform)
                if next_frame is None or step_transform is None:
                    continue
                visited.add(next_frame)
                queue.append(
                    (
                        next_frame,
                        _compose_transform(current_transform, step_transform),
                    )
                )
        return None

    def _on_odom(self, msg) -> None:  # noqa: ANN001
        pose = msg.pose.pose
        twist = msg.twist.twist
        yaw = _yaw_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        stamp = _stamp_to_seconds(getattr(getattr(msg, "header", None), "stamp", None)) or time.time()
        payload = {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": float(yaw),
            "vx": float(twist.linear.x),
            "wz": float(twist.angular.z),
        }
        if self.config.prefer_tf_pose:
            with self._lock:
                map_frame = str(self.state.latest_occupancy_frame)
            if map_frame:
                map_to_odom = self._lookup_transform(map_frame, self.config.topics.odom_frame)
                if map_to_odom is not None:
                    odom_to_base = {
                        "tx": float(pose.position.x),
                        "ty": float(pose.position.y),
                        "tz": float(getattr(pose.position, "z", 0.0)),
                        "yaw": float(yaw),
                        "source": "odom_msg",
                        "stamp": float(stamp),
                    }
                    map_pose = _compose_transform(map_to_odom, odom_to_base, source=f"{map_to_odom['source']}+odom_msg")
                    payload["x"] = float(map_pose["tx"])
                    payload["y"] = float(map_pose["ty"])
                    payload["yaw"] = float(map_pose["yaw"])
                    payload["frame"] = map_frame
        with self._lock:
            self.state.last_pose = dict(payload)
            self.state.pose_history.append({"stamp": float(stamp), "pose": dict(payload)})
            self.state.pose_history = self.state.pose_history[-240:]
        self._mark_topic(self.config.topics.odom)
        self._publish_async("/robot/pose", payload, stamp)

    def _on_tf(self, msg) -> None:  # noqa: ANN001
        self._mark_topic(self.config.topics.tf)
        for transform in msg.transforms:
            yaw = _yaw_from_quaternion(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )
            self._store_transform(
                transform.header.frame_id,
                transform.child_frame_id,
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
                yaw,
                "tf",
            )

    def _on_tf_static(self, msg) -> None:  # noqa: ANN001
        self._mark_topic(self.config.topics.tf_static)
        for transform in msg.transforms:
            yaw = _yaw_from_quaternion(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )
            self._store_transform(
                transform.header.frame_id,
                transform.child_frame_id,
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
                yaw,
                "tf_static",
            )

    def _on_occupancy_grid(self, msg) -> None:  # noqa: ANN001
        info = msg.info
        occupied_points: list[tuple[float, float, float]] = []

        width = int(info.width)
        height = int(info.height)
        resolution = float(info.resolution)
        origin_x = float(info.origin.position.x)
        origin_y = float(info.origin.position.y)
        orientation = getattr(info.origin, "orientation", None)
        origin_yaw = 0.0
        if orientation is not None:
            origin_yaw = _yaw_from_quaternion(
                float(getattr(orientation, "x", 0.0)),
                float(getattr(orientation, "y", 0.0)),
                float(getattr(orientation, "z", 0.0)),
                float(getattr(orientation, "w", 1.0)),
            )
        frame_id = _normalize_frame(getattr(getattr(msg, "header", None), "frame_id", ""))
        data = [int(value) for value in msg.data]

        for row in range(height):
            for col in range(width):
                value = int(data[row * width + col])
                if value < 0:
                    continue
                if value >= 50:
                    cell_x = (col + 0.5) * resolution
                    cell_y = (row + 0.5) * resolution

                    cos_yaw = math.cos(origin_yaw)
                    sin_yaw = math.sin(origin_yaw)

                    x = origin_x + cell_x * cos_yaw - cell_y * sin_yaw
                    y = origin_y + cell_x * sin_yaw + cell_y * cos_yaw
                    occupied_points.append((round(x, 3), round(y, 3), 1.0))

        payload = {
            "data": data,
            "resolution": resolution,
            "origin": {"x": origin_x, "y": origin_y, "yaw": round(origin_yaw, 6)},
            "width": width,
            "height": height,
            "frame_id": frame_id,
        }
        with self._lock:
            self.state.latest_occupancy_payload = payload
            self.state.latest_occupancy_points = occupied_points
            self.state.latest_occupancy_frame = frame_id
        self._mark_topic(self.config.topics.occupancy_grid)
        self._publish_async("/map/grid", payload, time.time())

def detect_ros(bus=None, loop: asyncio.AbstractEventLoop | None = None, config=None) -> RosRuntime:
    try:
        import rclpy  # noqa: F401
    except Exception as exc:  # noqa: BLE001
        return RosRuntime(enabled=False, reason=f"ROS2 unavailable: {exc}")

    if bus is None or loop is None or config is None:
        return RosRuntime(enabled=True, reason="ROS2 available")

    bridge = RosBridge(bus=bus, loop=loop, config=config)
    try:
        bridge.start()
    except Exception as exc:  # noqa: BLE001
        return RosRuntime(enabled=False, reason=f"ROS2 detected but bridge start failed: {exc}", bridge=None)
    return RosRuntime(enabled=True, reason="ROS2 bridge active", bridge=bridge)
