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
