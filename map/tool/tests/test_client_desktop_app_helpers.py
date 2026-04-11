from pathlib import Path

import client_desktop.app as app_module
from client_desktop.app import (
    DesktopClient,
    build_camera_refresh_text,
    can_zoom_from_widget,
    compute_log_candidates,
    normalize_server_ws_url,
    parse_camera_topic_id,
    resolve_log_file_path,
    safe_focus_widget,
    safe_mode_translation_key,
    strip_legacy_trajectory,
    should_clear_focus_on_click,
    zoom_scale_factor,
)


class Var:
    def __init__(self, value=None) -> None:
        self.value = value

    def get(self):
        return self.value

    def set(self, value) -> None:
        self.value = value


class DummyLogger:
    def info(self, *args, **kwargs):
        pass

    def warning(self, *args, **kwargs):
        pass

    def error(self, *args, **kwargs):
        pass

    def exception(self, *args, **kwargs):
        pass


class FakeFrame:
    def __init__(self) -> None:
        self.visible = False

    def winfo_manager(self):
        return "pack" if self.visible else ""

    def pack(self, *args, **kwargs):
        self.visible = True

    def pack_forget(self):
        self.visible = False


def test_normalize_server_ws_url_accepts_host_only_and_adds_stream_path() -> None:
    assert normalize_server_ws_url("192.168.3.56:8080") == "ws://192.168.3.56:8080/ws/stream"
    assert normalize_server_ws_url("ws://192.168.3.56:8080") == "ws://192.168.3.56:8080/ws/stream"
    assert normalize_server_ws_url("http://192.168.3.56:8080") == "ws://192.168.3.56:8080/ws/stream"


def test_sync_auth_mode_ui_toggles_local_and_cloud_forms() -> None:
    client = DesktopClient.__new__(DesktopClient)
    client.auth_mode_var = Var("local")
    client.auth_local_frame = FakeFrame()
    client.auth_cloud_frame = FakeFrame()

    DesktopClient.sync_auth_mode_ui(client)
    assert client.auth_local_frame.visible is True
    assert client.auth_cloud_frame.visible is False

    client.auth_mode_var.set("cloud")
    DesktopClient.sync_auth_mode_ui(client)
    assert client.auth_local_frame.visible is False
    assert client.auth_cloud_frame.visible is True


def test_apply_local_auth_result_updates_backend_descriptor_and_request_prep() -> None:
    client = DesktopClient.__new__(DesktopClient)
    client.tr = lambda key, **kwargs: key
    client.auth_mode_var = Var("cloud")
    client.server_var = Var("")
    client.auth_status_var = Var("")
    client.auth_cloud_expires_var = Var("")

    descriptor = DesktopClient.apply_auth_result(
        client,
        "local",
        {"ip": "10.0.0.2", "port": 8080, "token": "abc"},
    )

    assert descriptor["auth_mode"] == "local"
    assert client.backend_auth_descriptor == descriptor
    assert client.backend_http_base == "http://10.0.0.2:8080"
    assert client.backend_ws_url == "ws://10.0.0.2:8080/ws/stream"
    assert client.auth_mode_var.get() == "local"
    assert client.server_var.get() == "ws://10.0.0.2:8080/ws/stream"
    assert client.auth_status_var.get() == "auth_resolved: 10.0.0.2:8080"
    assert DesktopClient.backend_request_headers(client) == {"Authorization": "Bearer abc"}


def test_apply_cloud_auth_result_updates_backend_descriptor_and_request_prep() -> None:
    client = DesktopClient.__new__(DesktopClient)
    client.tr = lambda key, **kwargs: key
    client.auth_mode_var = Var("local")
    client.server_var = Var("")
    client.auth_status_var = Var("")
    client.auth_cloud_expires_var = Var("")

    descriptor = DesktopClient.apply_auth_result(
        client,
        "cloud",
        {"backend_host": "cloud.example.com", "backend_port": 8443, "token": "xyz", "expires_at": "2026-04-10T12:00:00Z"},
    )

    assert descriptor["auth_mode"] == "cloud"
    assert client.backend_auth_descriptor == descriptor
    assert client.backend_http_base == "https://cloud.example.com:8443"
    assert client.backend_ws_url == "wss://cloud.example.com:8443/ws/stream"
    assert client.auth_mode_var.get() == "cloud"
    assert client.server_var.get() == "wss://cloud.example.com:8443/ws/stream"
    assert client.auth_cloud_expires_var.get() == "2026-04-10T12:00:00Z"
    assert "auth_expires_at: 2026-04-10T12:00:00Z" in client.auth_status_var.get()
    assert DesktopClient.backend_request_headers(client) == {"Authorization": "Bearer xyz"}


def test_perform_local_auth_posts_fixed_credentials_and_applies_descriptor(monkeypatch) -> None:
    called = {}

    def fake_auth_request(self, url, payload):
        called["url"] = url
        called["payload"] = payload
        return {"token": "abc", "ip": "10.0.0.2", "port": 8080}

    monkeypatch.setattr(DesktopClient, "auth_request", fake_auth_request)

    client = DesktopClient.__new__(DesktopClient)
    client.logger = DummyLogger()
    client.tr = lambda key, **kwargs: key
    client.auth_mode_var = Var("local")
    client.auth_local_ip_var = Var("10.0.0.2")
    client.auth_local_port_var = Var("8088")
    client.auth_local_path_var = Var("/login")
    client.auth_cloud_expires_var = Var("")
    client.auth_status_var = Var("")
    client.server_var = Var("")

    descriptor = DesktopClient.perform_local_auth(client)

    assert called["url"] == "http://10.0.0.2:8088/login"
    assert called["payload"]["username"] == app_module.DEFAULT_LOCAL_AUTH_USERNAME
    assert called["payload"]["password"] == app_module.DEFAULT_LOCAL_AUTH_PASSWORD
    assert descriptor["backend_host"] == "10.0.0.2"
    assert descriptor["backend_port"] == 8080
    assert descriptor["token"] == "abc"
    assert client.server_var.get() == "ws://10.0.0.2:8080/ws/stream"


def test_perform_cloud_auth_posts_entered_credentials_and_applies_descriptor(monkeypatch) -> None:
    called = {}

    def fake_auth_request(self, url, payload):
        called["url"] = url
        called["payload"] = payload
        return {
            "data": {
                "backend_host": "cloud-backend.example.com",
                "backend_port": 9443,
                "token": "xyz",
                "expires_at": "2026-04-10T12:00:00Z",
            }
        }

    monkeypatch.setattr(DesktopClient, "auth_request", fake_auth_request)

    client = DesktopClient.__new__(DesktopClient)
    client.logger = DummyLogger()
    client.tr = lambda key, **kwargs: key
    client.auth_mode_var = Var("cloud")
    client.auth_cloud_host_var = Var("auth.example.com")
    client.auth_cloud_port_var = Var("8443")
    client.auth_cloud_path_var = Var("/api/auth/login")
    client.auth_cloud_username_var = Var("alice")
    client.auth_cloud_password_var = Var("secret")
    client.auth_cloud_expires_var = Var("")
    client.auth_status_var = Var("")
    client.server_var = Var("")

    descriptor = DesktopClient.perform_cloud_auth(client)

    assert called["url"] == "https://auth.example.com:8443/api/auth/login"
    assert called["payload"] == {"username": "alice", "password": "secret"}
    assert descriptor["backend_host"] == "cloud-backend.example.com"
    assert descriptor["backend_port"] == 9443
    assert descriptor["token"] == "xyz"
    assert client.server_var.get() == "wss://cloud-backend.example.com:9443/ws/stream"


def test_login_local_auth_then_connect_uses_descriptor_not_server_var(monkeypatch) -> None:
    created = []

    class FakeBridge:
        def __init__(self, ws_url, auth_token="", logger=None) -> None:
            self.ws_url = ws_url
            self.auth_token = auth_token
            self.logger = logger
            self.connected = True
            self.started = False
            self.stop_called = False
            self.queue = type("Q", (), {"get_nowait": staticmethod(lambda: (_ for _ in ()).throw(Exception("empty")))})()
            created.append(self)

        def get(self, path):
            assert path == "/health"
            return {"ws_clients": 1, "scan_active": False, "ros_enabled": True, "mapping_status": "ok"}

        def start(self) -> None:
            self.started = True

        def stop(self) -> None:
            self.stop_called = True

    monkeypatch.setattr(app_module, "ServerBridge", FakeBridge)

    client = DesktopClient.__new__(DesktopClient)
    client.logger = DummyLogger()
    client.tr = lambda key, **kwargs: key
    client.disconnect = lambda: None
    client.auth_mode_var = Var("local")
    client.auth_local_ip_var = Var("10.0.0.2")
    client.auth_local_port_var = Var("8080")
    client.auth_local_path_var = Var("/login")
    client.auth_cloud_host_var = Var("")
    client.auth_cloud_port_var = Var("")
    client.auth_cloud_path_var = Var("")
    client.auth_cloud_username_var = Var("")
    client.auth_cloud_password_var = Var("")
    client.auth_cloud_expires_var = Var("")
    client.auth_status_var = Var("")
    client.server_var = Var("ws://bad.example:9999/ws/stream")
    client.conn_var = Var("")
    client.status_var = Var("")
    client.status_detail_var = Var("")
    client.stream_health = {"retries_http": 0, "last_api_error": ""}

    monkeypatch.setattr(
        DesktopClient,
        "auth_request",
        lambda self, url, payload: {"token": "abc", "ip": "10.0.0.2", "port": 8080},
    )

    DesktopClient.login_local_auth(client)
    client.server_var.set("ws://bad.example:9999/ws/stream")

    DesktopClient.connect(client)

    assert created
    assert created[0].ws_url == "ws://10.0.0.2:8080/ws/stream"
    assert created[0].auth_token == "abc"
    assert created[0].started is True
    assert client.server_var.get() == "ws://10.0.0.2:8080/ws/stream"
    assert client.backend_ws_url == "ws://10.0.0.2:8080/ws/stream"


def test_server_bridge_passes_bearer_token_to_websocket_client(monkeypatch) -> None:
    captured = {}
    bridge = app_module.ServerBridge("ws://10.0.0.2:8080/ws/stream", auth_token="abc", logger=DummyLogger())

    class FakeWSApp:
        def __init__(self, url, **kwargs) -> None:
            captured["url"] = url
            captured["kwargs"] = kwargs

        def run_forever(self, **kwargs) -> None:
            captured["run_kwargs"] = kwargs
            bridge.stop_event.set()

        def close(self) -> None:
            pass

    monkeypatch.setattr(app_module.websocket, "WebSocketApp", FakeWSApp)

    bridge._loop()

    assert captured["url"] == "ws://10.0.0.2:8080/ws/stream"
    assert captured["kwargs"]["header"] == ["Authorization: Bearer abc"]


def test_apply_scan_mode_updates_mode_and_optionally_clears_scan() -> None:
    client = DesktopClient.__new__(DesktopClient)
    client.scan = {
        "scan_mode": "2d",
        "occupied": {"0:0": {"ix": 0, "iy": 0, "hits": 1, "intensity": 1.0}},
        "free": {},
        "point_cloud_preview": [[0.0, 0.0, 0.0, 1.0]],
        "front_frames": 1,
        "rear_frames": 0,
        "pointcloud_frames": 1,
        "raw_points": 3,
        "saved_point_count": 1,
        "last_saved_file": "demo.slam",
        "active": False,
    }
    client.scan_mode_var = Var("2d")
    client.sync_scan_badges = lambda: None

    assert DesktopClient.apply_scan_mode(client, "3d", clear_existing=False) == "3d"
    assert client.scan["scan_mode"] == "3d"
    assert client.scan_mode_var.get() == "3d"

    DesktopClient.apply_scan_mode(client, "2d", clear_existing=True)
    assert client.scan["scan_mode"] == "2d"
    assert client.scan["occupied"] == {}
    assert client.scan["point_cloud_preview"] == []


def test_occupied_points_returns_point_cloud_preview_in_3d_mode() -> None:
    client = DesktopClient.__new__(DesktopClient)
    client.scan = {
        "scan_mode": "3d",
        "point_cloud_preview": [[1.0, 2.0, 3.0, 0.5], [4.0, 5.0, 6.0, 1.0]],
    }

    assert DesktopClient.occupied_points(client) == [[1.0, 2.0, 3.0, 0.5], [4.0, 5.0, 6.0, 1.0]]


def test_resolve_log_file_path_falls_back_to_second_candidate(tmp_path: Path) -> None:
    first = tmp_path / "missing-parent" / "nested" / "client_desktop.log"
    second = tmp_path / "fallback" / "client_desktop.log"

    def flaky_mkdir(path: Path) -> None:
        if path == first.parent:
            raise OSError("blocked")
        path.mkdir(parents=True, exist_ok=True)

    chosen = resolve_log_file_path([first, second], mkdir_fn=flaky_mkdir)

    assert chosen == second
    assert second.exists()


def test_build_camera_refresh_text_handles_empty_inbox() -> None:
    assert build_camera_refresh_text({1: {"objects": [], "meta": {}}, 2: {"objects": [], "meta": {}}}) == "No buffered frame"


def test_parse_camera_topic_id_rejects_invalid_topic() -> None:
    assert parse_camera_topic_id("/camera/1") == 1
    assert parse_camera_topic_id("/camera/not-a-number") is None
    assert parse_camera_topic_id("/camera") is None


def test_safe_mode_translation_key_falls_back_to_default() -> None:
    assert safe_mode_translation_key("bad", {"view": "tool_view_select"}, "tool_view_select") == "tool_view_select"


def test_safe_focus_widget_handles_popdown_error() -> None:
    class DummyRoot:
        def focus_get(self):
            raise KeyError("popdown")

    assert safe_focus_widget(DummyRoot()) is None


def test_can_zoom_from_widget_accepts_canvas_and_children() -> None:
    class Canvas:
        pass

    class Child:
        def __init__(self, parent):
            self.master = parent

    canvas = Canvas()
    assert can_zoom_from_widget(canvas, canvas) is True
    assert can_zoom_from_widget(Child(canvas), canvas) is True
    assert can_zoom_from_widget(object(), canvas) is False


def test_should_clear_focus_on_click_only_for_blank_areas() -> None:
    class Blank:
        pass

    assert should_clear_focus_on_click(Blank()) is True


def test_zoom_scale_factor_supports_mousewheel_and_linux_buttons() -> None:
    class Event:
        def __init__(self, delta=0, num=None):
            self.delta = delta
            self.num = num

    assert zoom_scale_factor(Event(delta=120)) > 1.0
    assert zoom_scale_factor(Event(delta=-120)) < 1.0
    assert zoom_scale_factor(Event(num=4)) > 1.0
    assert zoom_scale_factor(Event(num=5)) < 1.0


def test_strip_legacy_trajectory_removes_exported_trajectory() -> None:
    manifest = {"poi": [], "path": [], "trajectory": [{"id": "old"}]}
    cleaned = strip_legacy_trajectory(manifest)

    assert "trajectory" not in cleaned
    assert "trajectory" in manifest


def test_compute_log_candidates_prefers_runtime_logs_dir() -> None:
    runtime_dir = Path("/tmp/runtime-app")
    candidates = compute_log_candidates(
        platform_name="win32",
        env={"LOCALAPPDATA": r"C:\Users\demo\AppData\Local"},
        home_dir=Path("/home/demo"),
        runtime_dir=runtime_dir,
        temp_dir=Path("/tmp"),
        cwd=Path("/work"),
    )

    assert candidates[0] == runtime_dir / "logs" / "client_desktop.log"


def test_move_click_uses_async_api_for_forward() -> None:
    class Var:
        def __init__(self, value: str) -> None:
            self.value = value

        def get(self) -> str:
            return self.value

    client = DesktopClient.__new__(DesktopClient)
    client.forward_var = Var("0.8")
    client.reverse_var = Var("0.5")
    client.turn_var = Var("1.0")
    client.duration_var = Var("0.15")
    client.number = DesktopClient.number.__get__(client, DesktopClient)
    calls: list[tuple[str, str, dict]] = []
    client.call_api = lambda path, body: calls.append(("sync", path, body))
    client.call_api_async = lambda path, body: calls.append(("async", path, body))

    DesktopClient.move_click(client, "forward")

    assert calls == [("async", "/control/move", {"velocity": 0.8, "yaw_rate": 0.0, "duration": 0.15})]


def test_move_click_uses_async_api_for_stop() -> None:
    class Var:
        def __init__(self, value: str) -> None:
            self.value = value

        def get(self) -> str:
            return self.value

    client = DesktopClient.__new__(DesktopClient)
    client.forward_var = Var("0.8")
    client.reverse_var = Var("0.5")
    client.turn_var = Var("1.0")
    client.duration_var = Var("0.15")
    client.number = DesktopClient.number.__get__(client, DesktopClient)
    calls: list[tuple[str, str, dict]] = []
    client.call_api = lambda path, body: calls.append(("sync", path, body))
    client.call_api_async = lambda path, body: calls.append(("async", path, body))

    DesktopClient.move_click(client, "stop")

    assert calls == [("async", "/control/stop", {})]


def test_on_key_release_delays_stop_until_after_confirmation_window() -> None:
    class Root:
        def __init__(self) -> None:
            self.scheduled: list[tuple[int, object]] = []

        def focus_get(self):
            return None

        def after(self, delay_ms: int, callback):
            token = object()
            self.scheduled.append((delay_ms, callback))
            return token

        def after_cancel(self, _token) -> None:
            raise AssertionError("should not cancel in this scenario")

    class Var:
        def __init__(self, value) -> None:
            self.value = value

        def get(self):
            return self.value

        def set(self, value) -> None:
            self.value = value

    class Event:
        keysym = "w"

    client = DesktopClient.__new__(DesktopClient)
    client.root = Root()
    client.keys_down = set()
    client.stop_on_keyup_var = Var(True)
    client.keyboard_var = Var("")
    client.pending_keyup_stop_id = None
    client.move_click = lambda name: (_ for _ in ()).throw(AssertionError(f"unexpected immediate move_click({name})"))
    client.tr = lambda key, **_kwargs: key

    DesktopClient.on_key_release(client, Event())

    assert len(client.root.scheduled) == 1
    assert client.root.scheduled[0][0] > 0


def test_on_key_press_cancels_pending_keyup_stop_before_it_fires() -> None:
    class Root:
        def __init__(self) -> None:
            self.cancelled = []

        def focus_get(self):
            return None

        def after(self, _delay_ms: int, _callback):
            raise AssertionError("after should not be called by key press")

        def after_cancel(self, token) -> None:
            self.cancelled.append(token)

    class Var:
        def __init__(self, value) -> None:
            self.value = value

        def get(self):
            return self.value

    class Event:
        keysym = "w"

    client = DesktopClient.__new__(DesktopClient)
    client.root = Root()
    client.keys_down = set()
    client.stop_on_keyup_var = Var(True)
    client.pending_keyup_stop_id = "pending-stop"
    ensured = []
    client.ensure_drive_loop = lambda: ensured.append("loop")

    DesktopClient.on_key_press(client, Event())

    assert client.root.cancelled == ["pending-stop"]
    assert client.pending_keyup_stop_id is None
    assert client.keys_down == {"w"}
    assert ensured == ["loop"]


def test_browser_occupancy_includes_scan_fusion_metadata() -> None:
    class Var:
        def __init__(self, value) -> None:
            self.value = value

        def get(self):
            return self.value

        def set(self, value) -> None:
            self.value = value

    client = DesktopClient.__new__(DesktopClient)
    client.scan = {"voxel": 0.06, "occupied": {"1:2": {"ix": 1, "iy": 2, "hits": 1, "intensity": 0.8}}, "free": {}}
    client.scan_fusion = {"preset": "indoor_sensitive", "voxel_size": 0.06, "occupied_min_hits": 1, "occupied_over_free_ratio": 0.55, "turn_skip_wz": 0.6, "skip_turn_frames": False}
    client.scan_fusion_preset_var = Var("indoor_sensitive")
    client.voxel_var = Var("0.06")
    client.occupied_min_hits_var = Var("1")
    client.occupied_over_free_ratio_var = Var("0.55")
    client.turn_skip_wz_var = Var("0.60")
    client.skip_turn_frames_var = Var(False)
    client.number = DesktopClient.number.__get__(client, DesktopClient)
    client.effective_scan_fusion_config = DesktopClient.effective_scan_fusion_config.__get__(client, DesktopClient)

    payload = DesktopClient.browser_occupancy(client)

    assert payload["scan_fusion"]["preset"] == "indoor_sensitive"
    assert payload["scan_fusion"]["occupied_min_hits"] == 1


def test_apply_scan_fusion_config_updates_runtime_and_vars() -> None:
    class Var:
        def __init__(self, value) -> None:
            self.value = value

        def get(self):
            return self.value

        def set(self, value) -> None:
            self.value = value

    client = DesktopClient.__new__(DesktopClient)
    client.scan = {"voxel": 0.08}
    client.scan_fusion = {}
    client.scan_fusion_preset_var = Var("indoor_balanced")
    client.voxel_var = Var("0.08")
    client.occupied_min_hits_var = Var("2")
    client.occupied_over_free_ratio_var = Var("0.75")
    client.turn_skip_wz_var = Var("0.45")
    client.skip_turn_frames_var = Var(True)

    DesktopClient.apply_scan_fusion_config(client, {"preset": "warehouse_sparse", "occupied_min_hits": 4}, update_vars=True)

    assert client.scan["voxel"] == 0.1
    assert client.scan_fusion_preset_var.get() == "warehouse_sparse"
    assert client.occupied_min_hits_var.get() == "4"


def test_start_scan_does_not_activate_local_scan_when_server_rejects_prereq(monkeypatch) -> None:
    warnings = []
    monkeypatch.setattr("client_desktop.app.messagebox.showwarning", lambda title, message: warnings.append((title, message)))

    client = DesktopClient.__new__(DesktopClient)
    client.scan = {"active": False}
    client.edit = {"loaded_from_stcm": True, "loaded_map_name": "demo"}
    client.tr = lambda key, **_kwargs: key
    client.call_api = lambda _path, _body: {
        "ok": False,
        "reason": "mapping_prereq_failed",
        "mapping_prereq": {"blockers": ["tf base->lidar missing", "odom topic stale"]},
    }
    client.clear_scan = lambda: (_ for _ in ()).throw(AssertionError("clear_scan should not run"))
    client.sync_scan_badges = lambda: (_ for _ in ()).throw(AssertionError("sync_scan_badges should not run"))

    DesktopClient.start_scan(client)

    assert client.scan["active"] is False
    assert warnings


def test_erase_radius_converts_removed_obstacle_to_free_cell() -> None:
    client = DesktopClient.__new__(DesktopClient)
    client.scan = {
        "voxel": 0.1,
        "occupied": {"0:0": {"ix": 0, "iy": 0, "hits": 3, "intensity": 1.0}},
        "free": {},
    }
    client.brush_var = Var("0.1")
    client.number = DesktopClient.number.__get__(client, DesktopClient)
    client.world_to_cell = DesktopClient.world_to_cell.__get__(client, DesktopClient)
    client.cell_key = DesktopClient.cell_key.__get__(client, DesktopClient)
    client.sync_scan_badges = lambda: None

    DesktopClient.erase_radius(client, 0.0, 0.0)

    assert "0:0" not in client.scan["occupied"]
    assert client.scan["free"]["0:0"]["hits"] >= 1


def test_auto_clear_noise_converts_isolated_noise_to_free_cell() -> None:
    client = DesktopClient.__new__(DesktopClient)
    client.scan = {
        "voxel": 0.1,
        "occupied": {"0:0": {"ix": 0, "iy": 0, "hits": 1, "intensity": 1.0}},
        "free": {},
    }
    client.map_edit_status_var = Var("")
    client.tr = lambda key, **kwargs: key.format(**kwargs) if kwargs else key
    client.sync_scan_badges = lambda: None

    DesktopClient.auto_clear_noise(client)

    assert "0:0" not in client.scan["occupied"]
    assert client.scan["free"]["0:0"]["hits"] >= 1
