import asyncio
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


def test_on_lidar_filters_near_and_isolated_beams() -> None:
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
                    lidar_frame="base_link",
                )
            ),
        )
        published = []
        bridge._publish_async = lambda topic, payload, stamp=None: published.append((topic, payload, stamp))
        scan_msg = SimpleNamespace(
            header=SimpleNamespace(frame_id="base_link"),
            angle_min=0.0,
            angle_increment=0.1,
            ranges=[float("inf"), 0.35, float("inf"), 1.0, float("inf"), 1.02, 1.01, float("inf")],
            range_min=0.3,
            range_max=30.0,
        )

        bridge._on_lidar(scan_msg, "front")

        assert len(bridge.state.latest_front_points) == 2
        assert published
        assert published[0][0] == "/lidar/front"
        assert len(published[0][1]["points"]) == 2
    finally:
        loop.close()


def test_on_lidar_uses_pose_closest_to_scan_stamp() -> None:
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
                    lidar_frame="base_link",
                )
            ),
        )
        bridge._publish_async = lambda *_args, **_kwargs: None

        odom_early = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0)),
            pose=SimpleNamespace(
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=1.0, y=0.0),
                    orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                )
            ),
            twist=SimpleNamespace(
                twist=SimpleNamespace(
                    linear=SimpleNamespace(x=0.0),
                    angular=SimpleNamespace(z=0.0),
                )
            ),
        )
        odom_late = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=20, nanosec=0)),
            pose=SimpleNamespace(
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=9.0, y=0.0),
                    orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                )
            ),
            twist=SimpleNamespace(
                twist=SimpleNamespace(
                    linear=SimpleNamespace(x=0.0),
                    angular=SimpleNamespace(z=0.0),
                )
            ),
        )
        bridge._on_odom(odom_early)
        bridge._on_odom(odom_late)

        scan_msg = SimpleNamespace(
            header=SimpleNamespace(frame_id="base_link", stamp=SimpleNamespace(sec=10, nanosec=100000000)),
            angle_min=0.0,
            angle_increment=0.1,
            ranges=[1.0, 1.01, 1.02],
            range_min=0.3,
            range_max=30.0,
        )

        bridge._on_lidar(scan_msg, "front")

        first_point = bridge.state.latest_front_points[0]
        assert abs(first_point[0] - 2.0) < 0.2
    finally:
        loop.close()


def test_latest_map_points_only_uses_occupancy_grid_points() -> None:
    loop = asyncio.new_event_loop()
    try:
        bridge = RosBridge(
            bus=DummyBus(),
            loop=loop,
            config=RosBridgeConfig(topics=RosTopicConfig()),
        )
        bridge.state.latest_front_points = [[1.0, 2.0, 1.0]]
        bridge.state.latest_rear_points = [[3.0, 4.0, 1.0]]

        assert bridge.latest_map_points() == []

        bridge.state.latest_occupancy_points = [(5.0, 6.0, 1.0)]

        assert bridge.latest_map_points() == [(5.0, 6.0, 1.0)]
    finally:
        loop.close()


def test_on_occupancy_grid_publishes_full_grid_payload() -> None:
    loop = asyncio.new_event_loop()
    try:
        bridge = RosBridge(
            bus=DummyBus(),
            loop=loop,
            config=RosBridgeConfig(topics=RosTopicConfig(occupancy_grid="/map")),
        )
        published = []
        bridge._publish_async = lambda topic, payload, stamp=None: published.append((topic, payload, stamp))
        msg = SimpleNamespace(
            info=SimpleNamespace(
                width=3,
                height=2,
                resolution=0.5,
                origin=SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0)),
            ),
            data=[-1, 0, 100, 51, -1, 1],
        )

        bridge._on_occupancy_grid(msg)

        assert published
        topic, payload, _stamp = published[0]
        assert topic == "/map/grid"
        assert payload["data"] == [-1, 0, 100, 51, -1, 1]
        assert payload["width"] == 3
        assert payload["height"] == 2
        assert payload["origin"] == {"x": 1.0, "y": 2.0}
        assert bridge.latest_map_points() == [(2.25, 2.25, 1.0), (1.25, 2.75, 1.0)]
    finally:
        loop.close()


def test_on_odom_prefers_map_to_odom_transform_with_current_odom_pose() -> None:
    loop = asyncio.new_event_loop()
    try:
        bridge = RosBridge(
            bus=DummyBus(),
            loop=loop,
            config=RosBridgeConfig(
                prefer_tf_pose=True,
                topics=RosTopicConfig(
                    odom="/odom",
                    imu="/imu",
                    tf="/tf",
                    tf_static="/tf_static",
                    occupancy_grid="/map",
                    odom_frame="odom",
                    robot_base_frame="base_link",
                ),
            ),
        )
        bridge._publish_async = lambda *_args, **_kwargs: None
        bridge._store_transform("map", "odom", 10.0, 20.0, 0.0, 0.0, "tf")
        bridge._on_occupancy_grid(
            SimpleNamespace(
                header=SimpleNamespace(frame_id="map"),
                info=SimpleNamespace(
                    width=1,
                    height=1,
                    resolution=1.0,
                    origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
                ),
                data=[0],
            )
        )
        odom_msg = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0)),
            pose=SimpleNamespace(
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=1.5, y=-2.0),
                    orientation=SimpleNamespace(x=0.0, y=0.0, z=0.1494381325, w=0.9887710779),
                )
            ),
            twist=SimpleNamespace(
                twist=SimpleNamespace(
                    linear=SimpleNamespace(x=0.0),
                    angular=SimpleNamespace(z=0.0),
                )
            ),
        )

        bridge._on_odom(odom_msg)

        assert bridge.latest_pose()["x"] == 11.5
        assert bridge.latest_pose()["y"] == 18.0
        assert round(bridge.latest_pose()["yaw"], 3) == 0.3
    finally:
        loop.close()
