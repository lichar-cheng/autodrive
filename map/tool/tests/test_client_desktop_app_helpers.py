from pathlib import Path

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


def test_normalize_server_ws_url_accepts_host_only_and_adds_stream_path() -> None:
    assert normalize_server_ws_url("192.168.3.56:8080") == "ws://192.168.3.56:8080/ws/stream"
    assert normalize_server_ws_url("ws://192.168.3.56:8080") == "ws://192.168.3.56:8080/ws/stream"
    assert normalize_server_ws_url("http://192.168.3.56:8080") == "ws://192.168.3.56:8080/ws/stream"


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
