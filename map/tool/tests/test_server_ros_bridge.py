import asyncio
import struct
from types import SimpleNamespace

from server.app.config import RosBridgeConfig, RosTopicConfig
from server.app.ros_bridge import RosBridge, _build_tf_static_qos


class DummyBus:
    async def publish(self, _topic, _message) -> None:
        return None


def test_build_tf_static_qos_prefers_transient_local_profile() -> None:
    calls = []

    class FakeQoSModule:
        class QoSProfile:
            def __init__(self, **kwargs) -> None:
                calls.append(kwargs)
                self.kwargs = kwargs

        class DurabilityPolicy:
            TRANSIENT_LOCAL = "transient_local"

        class ReliabilityPolicy:
            RELIABLE = "reliable"

        class HistoryPolicy:
            KEEP_LAST = "keep_last"

    sensor_profile = object()

    profile = _build_tf_static_qos(FakeQoSModule, sensor_profile)

    assert calls == [
        {
            "depth": 1,
            "durability": "transient_local",
            "reliability": "reliable",
            "history": "keep_last",
        }
    ]
    assert profile.kwargs["durability"] == "transient_local"


def test_mapping_prerequisites_use_observed_scan_frame_when_configured_lidar_frame_is_wrong() -> None:
    loop = asyncio.new_event_loop()
    try:
        bridge = RosBridge(
            bus=DummyBus(),
            loop=loop,
            config=RosBridgeConfig(
                topics=RosTopicConfig(
                    odom="/odom",
                    imu="/imu",
                    tf="/tf",
                    tf_static="/tf_static",
                    lidar_fallback="/scan",
                    robot_base_frame="base_link",
                    lidar_frame="base_scan",
                )
            ),
        )
        bridge._publish_async = lambda *_args, **_kwargs: None
        bridge._store_transform("base_link", "laser", 0.3, 0.0, 0.0, 3.1415926, "tf_static")
        bridge.state.last_message_time_by_topic["/odom"] = 100.0

        scan_msg = SimpleNamespace(
            header=SimpleNamespace(frame_id="laser"),
            angle_min=0.0,
            angle_increment=0.1,
            ranges=[],
            range_min=0.0,
            range_max=30.0,
        )
        bridge._on_lidar(scan_msg, "front")

        result = bridge.mapping_prerequisites(now=100.5)

        assert result["ready"] is True
        assert result["checks"]["tf_tree"]["ok"] is True
    finally:
        loop.close()


def test_mapping_prerequisites_require_pointcloud_topic_in_3d_mode() -> None:
    loop = asyncio.new_event_loop()
    try:
        bridge = RosBridge(
            bus=DummyBus(),
            loop=loop,
            config=RosBridgeConfig(
                scan_mode="3d",
                topics=RosTopicConfig(
                    odom="/odom",
                    imu="/imu",
                    tf="/tf",
                    tf_static="/tf_static",
                    pointcloud="/points",
                    robot_base_frame="base_link",
                    pointcloud_frame="lidar",
                )
            ),
        )
        bridge.state.last_message_time_by_topic["/odom"] = 100.0
        bridge.state.last_message_time_by_topic["/points"] = 100.0
        bridge._store_transform("base_link", "lidar", 0.0, 0.0, 0.2, 0.0, "tf_static")

        result = bridge.mapping_prerequisites(now=100.5)

        assert result["ready"] is True
        assert result["checks"]["pointcloud"]["ok"] is True
        assert result["checks"]["tf_tree"]["pointcloud_frame"] == "lidar"
    finally:
        loop.close()


def test_pointcloud_updates_preview_and_save_cloud_when_scanning() -> None:
    loop = asyncio.new_event_loop()
    try:
        bridge = RosBridge(
            bus=DummyBus(),
            loop=loop,
            config=RosBridgeConfig(
                scan_mode="3d",
                pointcloud_preview_voxel_size=0.2,
                pointcloud_save_voxel_size=0.1,
                pointcloud_preview_limit=100,
                pointcloud_save_limit=1000,
                topics=RosTopicConfig(
                    odom="/odom",
                    imu="/imu",
                    tf="/tf",
                    tf_static="/tf_static",
                    pointcloud="/points",
                    robot_base_frame="base_link",
                    pointcloud_frame="lidar",
                ),
            ),
        )
        bridge._publish_async = lambda *_args, **_kwargs: None
        bridge._store_transform("base_link", "lidar", 0.0, 0.0, 0.2, 0.0, "tf_static")
        bridge.set_scan_active(True)
        bridge.state.last_pose = {"x": 1.0, "y": 2.0, "yaw": 0.0, "vx": 0.0, "wz": 0.0}

        raw = b"".join(
            [
                struct.pack("<ffff", 0.0, 0.0, 0.0, 0.5),
                struct.pack("<ffff", 0.1, 0.1, 0.2, 0.8),
                struct.pack("<ffff", 1.0, 0.0, 0.5, 1.0),
            ]
        )
        msg = SimpleNamespace(
            header=SimpleNamespace(frame_id="lidar"),
            fields=[
                SimpleNamespace(name="x", offset=0),
                SimpleNamespace(name="y", offset=4),
                SimpleNamespace(name="z", offset=8),
                SimpleNamespace(name="intensity", offset=12),
            ],
            point_step=16,
            width=3,
            height=1,
            data=raw,
        )

        bridge._on_pointcloud(msg)

        preview = bridge.state.latest_pointcloud_preview
        saved = bridge.state.latest_pointcloud_save
        assert len(preview) >= 2
        assert len(saved) >= 2
        assert all(len(point) == 4 for point in preview)
        assert bridge.latest_map_points() == saved
    finally:
        loop.close()


def test_pointcloud_is_ignored_when_runtime_mode_is_2d() -> None:
    loop = asyncio.new_event_loop()
    try:
        bridge = RosBridge(
            bus=DummyBus(),
            loop=loop,
            config=RosBridgeConfig(
                scan_mode="2d",
                pointcloud_preview_voxel_size=0.2,
                pointcloud_save_voxel_size=0.1,
                pointcloud_preview_limit=100,
                pointcloud_save_limit=1000,
                topics=RosTopicConfig(
                    odom="/odom",
                    imu="/imu",
                    tf="/tf",
                    tf_static="/tf_static",
                    pointcloud="/points",
                    robot_base_frame="base_link",
                    pointcloud_frame="lidar",
                ),
            ),
        )
        bridge._publish_async = lambda *_args, **_kwargs: None
        raw = struct.pack("<ffff", 0.0, 0.0, 0.0, 0.5)
        msg = SimpleNamespace(
            header=SimpleNamespace(frame_id="lidar"),
            fields=[
                SimpleNamespace(name="x", offset=0),
                SimpleNamespace(name="y", offset=4),
                SimpleNamespace(name="z", offset=8),
                SimpleNamespace(name="intensity", offset=12),
            ],
            point_step=16,
            width=1,
            height=1,
            data=raw,
        )

        bridge._on_pointcloud(msg)

        assert bridge.state.latest_pointcloud_preview == []
        assert bridge.state.latest_pointcloud_save == []
    finally:
        loop.close()
