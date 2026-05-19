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


def test_mapping_prerequisites_require_fresh_odom_tf_and_grid_topics() -> None:
    loop = asyncio.new_event_loop()
    try:
        bridge = RosBridge(
            bus=DummyBus(),
            loop=loop,
            config=RosBridgeConfig(
                topics=RosTopicConfig(
                    odom="/odom",
                    tf="/tf",
                    tf_static="/tf_static",
                    occupancy_grid="/map",
                )
            ),
        )
        bridge._publish_async = lambda *_args, **_kwargs: None
        bridge.state.last_message_time_by_topic["/odom"] = 100.0
        bridge.state.last_message_time_by_topic["/tf"] = 100.1
        bridge.state.last_message_time_by_topic["/map"] = 100.2

        result = bridge.mapping_prerequisites(now=100.5)

        assert result["ready"] is True
        assert result["checks"]["tf"]["ok"] is True
        assert result["checks"]["occupancy_grid"]["ok"] is True
    finally:
        loop.close()


def test_mapping_prerequisites_fail_when_tf_topic_is_stale() -> None:
    loop = asyncio.new_event_loop()
    try:
        bridge = RosBridge(
            bus=DummyBus(),
            loop=loop,
            config=RosBridgeConfig(
                topics=RosTopicConfig(
                    odom="/odom",
                    tf="/tf",
                    tf_static="/tf_static",
                    occupancy_grid="/map",
                )
            ),
        )
        bridge.state.last_message_time_by_topic["/odom"] = 100.0
        bridge.state.last_message_time_by_topic["/map"] = 100.1
        bridge.state.last_message_time_by_topic["/tf"] = 95.0

        result = bridge.mapping_prerequisites(now=100.5)

        assert result["ready"] is False
        assert "tf topic stale or missing" in result["blockers"]
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
                origin=SimpleNamespace(
                    position=SimpleNamespace(x=1.0, y=2.0),
                    orientation=SimpleNamespace(x=0.0, y=0.0, z=0.1494381325, w=0.9887710779),
                ),
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
        assert payload["origin"] == {"x": 1.0, "y": 2.0, "yaw": 0.3}
        assert bridge.latest_map_points() == [(2.12, 2.608, 1.0), (1.017, 2.79, 1.0)]
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
