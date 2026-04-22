from pathlib import Path

import types

from client_desktop.app import (
    AuthFlowError,
    DesktopClient,
    MAX_MESSAGES_DRAIN_PER_TICK,
    bootstrap_authenticated_bridge,
    build_camera_refresh_text,
    build_direct_ws_url,
    can_zoom_from_widget,
    classify_network_quality,
    coalesce_stream_messages,
    compose_http_base_url,
    compute_log_candidates,
    load_desktop_client_config,
    normalize_http_base_url,
    normalize_server_ws_url,
    parse_camera_topic_id,
    redact_sensitive_text,
    resolve_log_file_path,
    read_slam_archive,
    safe_focus_widget,
    safe_mode_translation_key,
    strip_legacy_trajectory,
    should_clear_focus_on_click,
    write_slam_archive,
    zoom_scale_factor,
)


class DummyVar:
    def __init__(self, value="") -> None:
        self.value = value

    def set(self, value) -> None:
        self.value = value

    def get(self):
        return self.value


class DummyLock:
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        return False


class DummyProgress:
    def __init__(self) -> None:
        self.started: list[int] = []
        self.stopped = 0

    def start(self, interval: int) -> None:
        self.started.append(interval)

    def stop(self) -> None:
        self.stopped += 1


def build_minimal_client() -> DesktopClient:
    client = DesktopClient.__new__(DesktopClient)
    client.root = None
    client.scan = {
        "active": False,
        "mode": "2d",
        "phase": "idle",
        "error": "",
        "error_reason": "",
        "pending_start": False,
        "pending_mode": "",
        "started_ms": 0,
        "voxel": 0.08,
        "front_frames": 0,
        "rear_frames": 0,
        "raw_points": 0,
        "occupied": {},
        "free": {},
        "last_accum_pose": None,
        "last_saved_file": "",
        "saved_point_count": 0,
        "pcd_name": "",
        "pcd_bytes": b"",
        "pcd_received_at": 0,
    }
    client.edit = {"loaded_from_stcm": False, "loaded_map_name": "", "tool": "view", "pending_obstacle_start": None}
    client.scan_lock = DummyLock()
    client.poi_nodes = []
    client.path_segments = []
    client.scan_state_var = DummyVar("")
    client.map_badge_var = DummyVar("")
    client.tool_badge_var = DummyVar("")
    client.stats_badge_var = DummyVar("")
    client.map_edit_status_var = DummyVar("")
    client.scan_mode_var = DummyVar("2d")
    client.tr = lambda key, **kwargs: f"{key}:{kwargs}" if kwargs else key
    client.active_occupancy_cells = lambda: []
    client.active_free_cells = lambda: []
    client.mark_canvas_dirty = lambda: None
    client.clear_scan = DesktopClient.clear_scan.__get__(client, DesktopClient)
    client.sync_scan_badges = DesktopClient.sync_scan_badges.__get__(client, DesktopClient)
    return client


def test_normalize_server_ws_url_accepts_host_only_and_adds_stream_path() -> None:
    assert normalize_server_ws_url("192.168.3.56:8080") == "ws://192.168.3.56:8080/ws/stream"
    assert normalize_server_ws_url("ws://192.168.3.56:8080") == "ws://192.168.3.56:8080/ws/stream"
    assert normalize_server_ws_url("http://192.168.3.56:8080") == "ws://192.168.3.56:8080/ws/stream"


def test_normalize_http_base_url_accepts_host_only() -> None:
    assert normalize_http_base_url("192.168.3.56:8080") == "http://192.168.3.56:8080"
    assert normalize_http_base_url("https://demo.local/api") == "https://demo.local/api"


def test_compose_http_base_url_uses_host_and_port() -> None:
    assert compose_http_base_url("192.168.3.56", "28080") == "http://192.168.3.56:28080"
    assert compose_http_base_url("192.168.3.56", "") == "http://192.168.3.56"


def test_build_direct_ws_url_uses_host_and_port() -> None:
    assert build_direct_ws_url("192.168.3.56", "28080") == "ws://192.168.3.56:28080/ws/stream"


def test_redact_sensitive_text_hides_urls_and_hosts() -> None:
    redacted = redact_sensitive_text("connect failed to ws://192.168.3.56:8080/ws/stream token TOKEN-ABC-1234567890")

    assert "192.168.3.56:8080" not in redacted
    assert "ws://" not in redacted


def test_auth_flow_error_keeps_user_message_but_redacts_sensitive_values() -> None:
    err = AuthFlowError("网关错误: ws://192.168.3.56:28080/ws/stream token TOKEN-ABC-1234567890")

    assert "网关错误" in err.user_message
    assert "192.168.3.56" not in err.user_message
    assert "TOKEN-ABC-1234567890" not in err.user_message


def test_load_desktop_client_config_reads_local_override(tmp_path: Path) -> None:
    config_path = tmp_path / "client_config.json"
    config_path.write_text('{"login_required": false, "gateway_ip": "10.0.0.8", "gateway_port": 28080, "server_ip": "10.0.0.9", "server_port": 18080, "username": "debug", "scan_start_timeout_sec": 12}', encoding="utf-8")

    config = load_desktop_client_config(config_path)

    assert config["login_required"] is False
    assert config["gateway_ip"] == "10.0.0.8"
    assert config["server_ip"] == "10.0.0.9"
    assert config["scan_start_timeout_sec"] == 12


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


def test_bootstrap_authenticated_bridge_logs_in_and_fetches_vcu_urls() -> None:
    calls = []

    class Response:
        def __init__(self, payload) -> None:
            self.payload = payload

        def raise_for_status(self) -> None:
            return None

        def json(self):
            return self.payload

    class Session:
        trust_env = True

        def post(self, url, json=None, headers=None, timeout=None):
            calls.append((url, json, headers, timeout))
            if url.endswith("/sysUser/userLogin"):
                return Response({"retCode": 200, "retMsg": "Success", "retData": {"tokenID": "TOKEN-1", "userName": "admin"}})
            return Response({"retCode": 200, "retMsg": "Success", "retData": {"http": "http://192.168.3.56:8080/health", "ws": "ws://192.168.3.56:8080/ws/stream"}})

        def close(self) -> None:
            return None

    result = bootstrap_authenticated_bridge("192.168.3.99", "admin", "123456", session=Session(), timeout_sec=2.0)

    assert result["token"] == "TOKEN-1"
    assert result["ws_url"] == "ws://192.168.3.56:8080/ws/stream"
    assert calls[1][2] == {"Authorization": "TOKEN-1"}


def test_bootstrap_authenticated_bridge_surfaces_login_failure() -> None:
    class Response:
        def raise_for_status(self) -> None:
            return None

        def json(self):
            return {"retCode": 401, "retMsg": "Bad credentials"}

    class Session:
        trust_env = True

        def post(self, *_args, **_kwargs):
            return Response()

        def close(self) -> None:
            return None

    try:
        bootstrap_authenticated_bridge("192.168.3.99", "admin", "bad", session=Session())
    except RuntimeError as exc:
        assert "Bad credentials" in str(exc)
    else:
        raise AssertionError("expected login failure")


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


def test_zoom_view_clamps_scale_and_refreshes_metrics() -> None:
    client = DesktopClient.__new__(DesktopClient)
    client.view = {"scale": 25.0, "pan_x": 0.0, "pan_y": 0.0}
    refreshed: list[float] = []
    client.update_view_metrics = lambda: refreshed.append(client.view["scale"])

    DesktopClient.zoom_view(client, 1.2)
    DesktopClient.zoom_view(client, 0.01)
    DesktopClient.zoom_view(client, 100.0)

    assert refreshed == [30.0, 8.0, 80.0]


def test_fit_world_bounds_prefers_80px_when_map_fits_canvas() -> None:
    class Canvas:
        def winfo_width(self):
            return 1200

        def winfo_height(self):
            return 900

    client = DesktopClient.__new__(DesktopClient)
    client.canvas = Canvas()
    client.view = {"scale": 25.0, "pan_x": 0.0, "pan_y": 0.0}
    refreshed = []
    client.update_view_metrics = lambda: refreshed.append((client.view["scale"], client.view["pan_x"], client.view["pan_y"]))

    DesktopClient.fit_world_bounds(client, 0.0, 0.0, 4.0, 3.0)

    assert refreshed
    assert client.view["scale"] == 80.0


def test_reset_view_defaults_to_80px_without_map_bounds() -> None:
    client = DesktopClient.__new__(DesktopClient)
    client.view = {"scale": 25.0, "pan_x": 0.0, "pan_y": 0.0}
    client.pose = {"x": 0.0, "y": 0.0}
    client.active_map_world_bounds = lambda include_robot=True: None
    client.center_robot = DesktopClient.center_robot.__get__(client, DesktopClient)
    refreshed = []
    client.update_view_metrics = lambda: refreshed.append(client.view["scale"])

    DesktopClient.reset_view(client)

    assert refreshed
    assert client.view["scale"] == 80.0


def test_strip_legacy_trajectory_removes_exported_trajectory() -> None:
    manifest = {"poi": [], "path": [], "trajectory": [{"id": "old"}]}
    cleaned = strip_legacy_trajectory(manifest)

    assert "trajectory" not in cleaned
    assert "trajectory" in manifest


def test_start_scan_sends_selected_mode_and_updates_phase() -> None:
    client = build_minimal_client()
    calls = []
    client.call_api = lambda path, body: calls.append((path, body)) or {"ok": True, "scan_mode": "3d"}
    client.scan_mode_var.set("3d")

    DesktopClient.start_scan(client)

    assert calls == [("/scan/start", {"mode": "3d"})]
    assert client.scan["active"] is True
    assert client.scan["mode"] == "3d"
    assert client.scan["phase"] == "scanning"


def test_start_scan_shows_starting_feedback_before_request_returns() -> None:
    client = build_minimal_client()

    def call_api(_path, _body):
        assert client.scan["phase"] == "starting"
        assert client.scan_state_var.get() == "scan_starting"
        return {"ok": True, "scan_mode": "2d"}

    client.call_api = call_api

    DesktopClient.start_scan(client)

    assert client.scan["phase"] == "scanning"


def test_start_scan_ignores_click_while_waiting_for_mapping() -> None:
    client = build_minimal_client()
    client.scan["phase"] = "waiting_mapping"
    client.scan["pending_start"] = True
    client.call_api = lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("start should not be called"))

    DesktopClient.start_scan(client)

    assert client.scan["phase"] == "waiting_mapping"
    assert client.scan["pending_start"] is True


def test_start_scan_cancelled_returns_idle() -> None:
    client = build_minimal_client()
    client.call_api = lambda path, body: {"ok": False, "reason": "scan_start_cancelled", "scan_mode": "3d"}
    client.scan_mode_var.set("3d")

    DesktopClient.start_scan(client)

    assert client.scan["active"] is False
    assert client.scan["phase"] == "idle"
    assert client.scan["pending_start"] is False


def test_start_scan_worker_sends_stop_cleanup_after_timeout() -> None:
    calls = []

    class Bridge:
        def post(self, path, body, retries=3, timeout_sec=4.0, backoff_base_sec=0.2):
            calls.append((path, body, retries, timeout_sec, backoff_base_sec))
            if path == "/scan/start":
                raise TimeoutError("timed out")
            return {"ok": True, "scan_mode": "3d"}

    class Root:
        def after(self, delay, callback):
            del delay
            callback()

    client = build_minimal_client()
    client.bridge = Bridge()
    client.root = Root()
    client.client_config = {"scan_start_timeout_sec": 7.0, "scan_start_cleanup_timeout_sec": 1.5}
    client.stream_health = {"retries_http": 0, "last_api_error": ""}
    client.logger = types.SimpleNamespace(exception=lambda *args, **kwargs: None, warning=lambda *args, **kwargs: None)

    DesktopClient._start_scan_worker(client, "3d")

    assert calls[0] == ("/scan/start", {"mode": "3d"}, 3, 7.0, 0.2)
    assert calls[1] == ("/scan/stop", {"mode": "3d"}, 0, 1.5, 0.0)
    assert client.scan["phase"] == "error"
    assert client.scan["error_reason"] == "request_failed"


def test_stop_scan_3d_returns_idle_without_downloading_pcd() -> None:
    client = build_minimal_client()
    client.scan["active"] = True
    client.scan["mode"] = "3d"
    client.scan["pcd_name"] = ""
    client.scan["pcd_bytes"] = b""
    client.call_api = lambda path, body: {"ok": True, "scan_mode": "3d"}

    DesktopClient.stop_scan(client)

    assert client.scan["active"] is False
    assert client.scan["phase"] == "idle"
    assert client.scan["pcd_name"] == ""
    assert client.scan["pcd_bytes"] == b""


def test_stop_scan_shows_stopping_feedback_before_request_returns() -> None:
    client = build_minimal_client()
    progress = DummyProgress()
    client.scan_progress = progress

    def call_api(_path, _body):
        assert client.scan["phase"] == "stopping"
        assert client.scan_state_var.get() == "scan_stopping"
        assert progress.started == [80]
        return {"ok": True, "scan_mode": "2d"}

    client.call_api = call_api

    DesktopClient.stop_scan(client)

    assert progress.stopped == 1
    assert client.scan["phase"] == "idle"


def test_stop_scan_not_active_cancels_pending_start_and_returns_idle() -> None:
    client = build_minimal_client()
    client.scan["active"] = False
    client.scan["mode"] = "3d"
    client.scan["phase"] = "waiting_mapping"
    client.scan["pending_start"] = True
    client.scan["pending_mode"] = "3d"
    client.scan["error"] = "tf missing"
    client.scan["error_reason"] = "mapping_prereq_failed"
    client.call_api = lambda path, body: {
        "ok": False,
        "reason": "scan_not_active",
        "scan_mode": "3d",
        "process_stop_status": {"stopped_pids": [1234], "errors": []},
    }

    DesktopClient.stop_scan(client)

    assert client.scan["phase"] == "idle"
    assert client.scan["pending_start"] is False
    assert client.scan["pending_mode"] == ""
    assert client.scan["error"] == ""
    assert client.scan["error_reason"] == ""


def test_start_scan_persists_server_error_reason_in_status() -> None:
    client = build_minimal_client()
    client.call_api = lambda path, body: {"ok": False, "reason": "node_start_failed", "error": "failed to launch"}

    DesktopClient.start_scan(client)

    assert client.scan["phase"] == "error"
    assert client.scan["error"] == "failed to launch"
    assert "failed to launch" in client.scan_state_var.get()


def test_health_ready_retries_mapping_prereq_pending_scan() -> None:
    client = build_minimal_client()
    client.scan["phase"] = "waiting_mapping"
    client.scan["pending_start"] = True
    client.scan["pending_mode"] = "3d"
    client.scan["error"] = "tf base->lidar missing"
    client.scan["error_reason"] = "mapping_prereq_failed"
    client.update_network_banner = lambda health: None
    client.status_detail_var = DummyVar("")
    calls = []
    client.call_api = lambda path, body: calls.append((path, body)) or {"ok": True, "scan_mode": "3d"}

    DesktopClient.update_health_status_detail(
        client,
        {"ws_clients": 1, "scan_active": False, "ros_enabled": True, "mapping_status": "ok", "mapping_ready": True},
    )

    assert calls == [("/scan/start", {"mode": "3d"})]
    assert client.scan["phase"] == "scanning"
    assert client.scan["active"] is True
    assert client.scan["error"] == ""
    assert client.scan["error_reason"] == ""
    assert client.scan["pending_start"] is False


def test_write_slam_archive_omits_pcd_for_2d(tmp_path: Path) -> None:
    target = tmp_path / "demo.slam"

    write_slam_archive(target, {"version": "slam.v3", "scan_mode": "2d"}, [(1.0, 2.0, 3.0)], None)

    manifest, points, pcd_file = read_slam_archive(target)

    assert manifest["scan_mode"] == "2d"
    assert points == [(1.0, 2.0, 3.0)]
    assert pcd_file is None


def test_write_slam_archive_round_trips_optional_pcd(tmp_path: Path) -> None:
    target = tmp_path / "demo3d.slam"

    write_slam_archive(
        target,
        {"version": "slam.v3", "scan_mode": "3d"},
        [(1.0, 2.0, 3.0)],
        {"name": "map.pcd", "content": b"pcd-bytes"},
    )

    manifest, points, pcd_file = read_slam_archive(target)

    assert manifest["scan_mode"] == "3d"
    assert points == [(1.0, 2.0, 3.0)]
    assert pcd_file == {"name": "map.pcd", "content": b"pcd-bytes"}


def test_ensure_scan_pcd_downloads_and_stores_bytes() -> None:
    class Response:
        def __init__(self) -> None:
            self.status_code = 200
            self.headers = {
                "Content-Length": "9",
                "Content-Disposition": 'attachment; filename="map.pcd"',
            }
            self.text = ""

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            return False

        def iter_content(self, chunk_size: int = 0):
            del chunk_size
            yield b"pcd-"
            yield b"bytes"

    client = build_minimal_client()
    client.scan["mode"] = "3d"
    client.bridge = types.SimpleNamespace(
        connected=True,
        http_base="http://demo",
        session=types.SimpleNamespace(get=lambda *args, **kwargs: Response()),
    )
    client._create_progress_dialog = lambda title: {"title": title}
    client._update_progress_dialog = lambda *_args, **_kwargs: None
    client._close_progress_dialog = lambda *_args, **_kwargs: None

    pcd_file = DesktopClient.ensure_scan_pcd(client)

    assert pcd_file == {"name": "map.pcd", "content": b"pcd-bytes"}
    assert client.scan["pcd_name"] == "map.pcd"
    assert client.scan["pcd_bytes"] == b"pcd-bytes"


def test_save_stcm_downloads_pcd_before_writing_archive(tmp_path: Path, monkeypatch) -> None:
    target = tmp_path / "demo.slam"
    writes = []
    infos = []
    monkeypatch.setattr("client_desktop.app.filedialog.asksaveasfilename", lambda **_kwargs: str(target))
    monkeypatch.setattr("client_desktop.app.write_slam_archive", lambda path, manifest, points, pcd_file: writes.append((path, manifest, points, pcd_file)))
    monkeypatch.setattr("client_desktop.app.messagebox.showinfo", lambda title, message: infos.append((title, message)))

    client = build_minimal_client()
    client.map_name_var = DummyVar("demo")
    client.map_notes_var = DummyVar("")
    client.camera_refresh_var = DummyVar("")
    client.pose = {}
    client.gps = {}
    client.chassis = {}
    client.logger = types.SimpleNamespace(info=lambda *args, **kwargs: None)
    client.inspector = {"file": "", "manifest": None, "points": [], "pgm": "", "yaml": "", "json": "", "meta": {}, "pcd_file": None}
    client.rebuild_path_nodes = lambda: None
    client.apply_scan_fusion_config_from_ui = lambda: {"voxel_size": 0.1, "occupied_min_hits": 2, "occupied_over_free_ratio": 0.75, "turn_skip_wz": 0.45, "skip_turn_frames": True}
    client.active_voxel_size = lambda: 0.1
    client.should_use_server_grid = lambda: False
    client.browser_occupancy = lambda: {"occupied_cells": [], "free_cells": [], "map_fence_xy": [], "voxel_size": 0.1, "scan_fusion": {}}
    client.occupied_points = lambda: [[1.0, 2.0, 1.0]]
    client.set_inspector_bundle_state = lambda *_args, **_kwargs: None
    client.ensure_scan_pcd = lambda: {"name": "map.pcd", "content": b"pcd-bytes"}

    DesktopClient.save_stcm(client)

    assert writes
    assert writes[0][3] == {"name": "map.pcd", "content": b"pcd-bytes"}
    assert infos


def test_export_pcd_warns_when_no_pcd_exists(monkeypatch) -> None:
    warnings = []
    monkeypatch.setattr("client_desktop.app.messagebox.showwarning", lambda title, message: warnings.append((title, message)))

    client = build_minimal_client()
    client.inspector = {"file": "demo.slam", "manifest": None, "points": [], "pgm": "", "yaml": "", "json": "", "meta": {}, "pcd_file": None}
    client.logger = types.SimpleNamespace(info=lambda *args, **kwargs: None)

    DesktopClient.export_inspector_file(client, "pcd")

    assert warnings
    assert warnings[0][1] == "pcd_export_unavailable"


def test_export_pcd_writes_current_pcd_bytes(tmp_path: Path, monkeypatch) -> None:
    exported = tmp_path / "exported.pcd"
    infos = []
    monkeypatch.setattr("client_desktop.app.filedialog.asksaveasfilename", lambda **_kwargs: str(exported))
    monkeypatch.setattr("client_desktop.app.messagebox.showinfo", lambda title, message: infos.append((title, message)))

    client = build_minimal_client()
    client.scan["pcd_name"] = "map.pcd"
    client.scan["pcd_bytes"] = b"pcd-bytes"
    client.inspector = {"file": "demo.slam", "manifest": None, "points": [], "pgm": "", "yaml": "", "json": "", "meta": {}, "pcd_file": None}
    client.logger = types.SimpleNamespace(info=lambda *args, **kwargs: None)

    DesktopClient.export_inspector_file(client, "pcd")

    assert exported.read_bytes() == b"pcd-bytes"
    assert infos


def test_export_pcd_downloads_when_local_copy_missing(tmp_path: Path, monkeypatch) -> None:
    exported = tmp_path / "downloaded.pcd"
    infos = []
    monkeypatch.setattr("client_desktop.app.filedialog.asksaveasfilename", lambda **_kwargs: str(exported))
    monkeypatch.setattr("client_desktop.app.messagebox.showinfo", lambda title, message: infos.append((title, message)))

    client = build_minimal_client()
    client.inspector = {"file": "demo.slam", "manifest": None, "points": [], "pgm": "", "yaml": "", "json": "", "meta": {}, "pcd_file": None}
    client.logger = types.SimpleNamespace(info=lambda *args, **kwargs: None)
    client.scan["mode"] = "3d"
    client.ensure_scan_pcd = lambda: {"name": "map.pcd", "content": b"pcd-bytes"}

    DesktopClient.export_inspector_file(client, "pcd")

    assert exported.read_bytes() == b"pcd-bytes"
    assert infos


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

    assert calls == [("async", "/control/target", {"velocity": 0.8, "yaw_rate": 0.0})]


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


def test_send_control_command_now_uses_fast_timeout_without_retries() -> None:
    client = DesktopClient.__new__(DesktopClient)
    seen = []

    class Bridge:
        connected = True

        def post(self, path, body, retries=3, timeout_sec=4.0, backoff_base_sec=0.2):
            seen.append((path, body, retries, timeout_sec, backoff_base_sec))
            return {}

    client.bridge = Bridge()

    DesktopClient.send_control_command_now(client, "/control/move", {"velocity": 0.8})

    assert seen == [("/control/move", {"velocity": 0.8}, 0, 0.35, 0.0)]


def test_network_quality_classifies_existing_signals_without_extra_polling() -> None:
    assert classify_network_quality(False, {}, 10_000, now_ms=10_100) == "offline"
    assert classify_network_quality(True, {"control_failures_consecutive": 2}, 10_000, now_ms=10_100) == "unstable"
    assert classify_network_quality(True, {}, 7_000, now_ms=10_100) == "unstable"
    assert classify_network_quality(True, {"last_lag_ms": 900}, 10_000, now_ms=10_100) == "degraded"
    assert classify_network_quality(True, {"gap_err": 2, "network_quality_gap_seen": 1}, 10_000, now_ms=10_100) == "degraded"
    assert classify_network_quality(True, {"last_lag_ms": 20}, 10_000, now_ms=10_100) == "ok"


def test_update_health_status_detail_includes_network_warning() -> None:
    client = DesktopClient.__new__(DesktopClient)
    client.status_detail_var = DummyVar("")
    client.stream_health = {"control_failures_consecutive": 2}
    client.last_message_at_ms = 10_000
    client.bridge = types.SimpleNamespace(connected=True, last_ws_issue_at_ms=0)
    client.tr = lambda key, **kwargs: key if not kwargs else f"{key}:{kwargs}"

    DesktopClient.update_health_status_detail(client, {"ws_clients": 1, "scan_active": False, "ros_enabled": True, "mapping_status": "ok", "mapping_ready": True})

    assert "network_unstable" in client.status_detail_var.get()


def test_control_sender_loop_counts_consecutive_control_failures() -> None:
    class Event:
        def wait(self, timeout=None):
            return False

        def clear(self) -> None:
            return None

    class Stop:
        def __init__(self) -> None:
            self.calls = 0

        def is_set(self) -> bool:
            self.calls += 1
            return self.calls > 3

    class Var:
        def get(self):
            return "120"

    client = DesktopClient.__new__(DesktopClient)
    client.control_sender_event = Event()
    client.control_sender_stop = Stop()
    client.control_lock = type("Lock", (), {"__enter__": lambda self: None, "__exit__": lambda self, exc_type, exc, tb: None})()
    client.control_target = ("/control/target", {"velocity": 0.8, "yaw_rate": 0.0}, "forward")
    client.repeat_ms_var = Var()
    client.stream_health = {"retries_http": 0, "control_failures_consecutive": 0, "last_api_error": ""}
    client.number = DesktopClient.number.__get__(client, DesktopClient)
    client.send_control_command_now = lambda _path, _body: (_ for _ in ()).throw(RuntimeError("timeout"))

    DesktopClient._control_sender_loop(client)

    assert client.stream_health["control_failures_consecutive"] == 3
    assert client.stream_health["retries_http"] == 3


def test_control_sender_loop_uses_repeat_ms_interval() -> None:
    class Event:
        def __init__(self) -> None:
            self.timeouts = []

        def wait(self, timeout=None):
            self.timeouts.append(timeout)
            return False

        def clear(self) -> None:
            return None

        def set(self) -> None:
            return None

    class Stop:
        def __init__(self) -> None:
            self.calls = 0

        def is_set(self) -> bool:
            self.calls += 1
            return self.calls > 2

    class Var:
        def __init__(self, value) -> None:
            self.value = value

        def get(self):
            return self.value

    client = DesktopClient.__new__(DesktopClient)
    client.control_sender_event = Event()
    client.control_sender_stop = Stop()
    client.control_lock = type("Lock", (), {"__enter__": lambda self: None, "__exit__": lambda self, exc_type, exc, tb: None})()
    client.control_target = ("/control/target", {"velocity": 0.8, "yaw_rate": 0.0}, "forward")
    client.repeat_ms_var = Var("120")
    client.number = DesktopClient.number.__get__(client, DesktopClient)
    sent = []
    client.send_control_command_now = lambda path, body: sent.append((path, body))

    DesktopClient._control_sender_loop(client)

    assert client.control_sender_event.timeouts == [0.12, 0.12]
    assert sent == [
        ("/control/target", {"velocity": 0.8, "yaw_rate": 0.0}),
        ("/control/target", {"velocity": 0.8, "yaw_rate": 0.0}),
    ]


def test_connect_uses_direct_ws_url_when_login_is_disabled() -> None:
    class Var:
        def __init__(self, value) -> None:
            self.value = value

        def get(self):
            return self.value

        def set(self, value) -> None:
            self.value = value

    client = DesktopClient.__new__(DesktopClient)
    client.client_config = {"login_required": False}
    client.direct_server_ip_var = Var("192.168.3.56")
    client.direct_server_port_var = Var("28080")
    client.auth_status_var = Var("")
    client.server_var = Var("")
    client.stream_health = {"last_api_error": ""}
    client.conn_var = Var("")
    client.status_var = Var("")
    client.status_detail_var = Var("")
    client.health = {}
    client.logger = type("Logger", (), {"info": lambda *args, **kwargs: None, "exception": lambda *args, **kwargs: None})()
    client.tr = lambda key, **kwargs: key if not kwargs else key
    client.disconnect = lambda: None
    client.update_health_status_detail = lambda health: None

    class Bridge:
        def __init__(self, ws_url, logger=None) -> None:
            self.ws_url = ws_url
            self.connected = False

        def get(self, path):
            assert path == "/health"
            return {"ws_clients": 0}

        def start(self):
            return None

        def stop(self):
            return None

    import client_desktop.app as app_module

    original_bridge = app_module.ServerBridge
    app_module.ServerBridge = Bridge
    try:
        DesktopClient.connect(client)
    finally:
        app_module.ServerBridge = original_bridge

    assert client.server_var.get() == "ws://192.168.3.56:28080/ws/stream"


def test_reset_server_map_calls_api_and_clears_local_state() -> None:
    client = DesktopClient.__new__(DesktopClient)
    seen = []
    client.call_api = lambda path, body: seen.append((path, body)) or {"ok": True}
    client.clear_loaded_map = lambda: seen.append(("cleared", {}))
    client.tr = lambda key, **kwargs: key if not kwargs else key

    DesktopClient.reset_server_map(client)

    assert seen == [("/map/reset", {}), ("cleared", {})]


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
    client.clear_control_target = lambda: None
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


def test_ensure_drive_loop_updates_control_target_immediately() -> None:
    class Var:
        def __init__(self, value) -> None:
            self.value = value

        def get(self):
            return self.value

        def set(self, value) -> None:
            self.value = value

    client = DesktopClient.__new__(DesktopClient)
    client.keys_down = {"w"}
    client.keyboard_var = Var("")
    client.tr = lambda key, **kwargs: kwargs.get("cmd", key)
    seen = []
    client.update_control_target = lambda name: seen.append(name)

    DesktopClient.ensure_drive_loop(client)

    assert seen == ["forward"]
    assert client.keyboard_var.get() == "forward"


def test_update_control_target_skips_duplicate_command() -> None:
    class Event:
        def __init__(self) -> None:
            self.calls = 0

        def set(self) -> None:
            self.calls += 1

    client = DesktopClient.__new__(DesktopClient)
    client.control_lock = type("Lock", (), {"__enter__": lambda self: None, "__exit__": lambda self, exc_type, exc, tb: None})()
    client.control_sender_event = Event()
    client.control_target = ("/control/target", {"velocity": 0.8, "yaw_rate": 0.0}, "forward")
    client.build_control_command = lambda name: ("/control/target", {"velocity": 0.8, "yaw_rate": 0.0}, name)

    DesktopClient.update_control_target(client, "forward")

    assert client.control_sender_event.calls == 0


def test_on_key_release_updates_control_target_for_remaining_keys() -> None:
    class Root:
        def focus_get(self):
            return None

    class Var:
        def __init__(self, value) -> None:
            self.value = value

        def get(self):
            return self.value

    class Event:
        keysym = "a"

    client = DesktopClient.__new__(DesktopClient)
    client.root = Root()
    client.keys_down = {"w", "a"}
    client.stop_on_keyup_var = Var(True)
    client.clear_control_target = lambda: (_ for _ in ()).throw(AssertionError("should not clear target when keys remain"))
    ensured = []
    client.ensure_drive_loop = lambda: ensured.append("loop")

    DesktopClient.on_key_release(client, Event())

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

    assert "preset" not in payload["scan_fusion"]
    assert payload["scan_fusion"]["occupied_min_hits"] == 1
    assert payload["occupied_cells"] == [{"ix": 1, "iy": 2, "hits": 1, "intensity": 0.8}]


def test_browser_occupancy_filters_out_non_occupied_cells() -> None:
    class Var:
        def __init__(self, value) -> None:
            self.value = value

        def get(self):
            return self.value

        def set(self, value) -> None:
            self.value = value

    client = DesktopClient.__new__(DesktopClient)
    client.scan = {
        "voxel": 0.2,
        "occupied": {
            "1:1": {"ix": 1, "iy": 1, "hits": 1, "intensity": 0.4},
            "2:2": {"ix": 2, "iy": 2, "hits": 4, "intensity": 0.9},
        },
        "free": {"1:1": {"ix": 1, "iy": 1, "hits": 3}},
    }
    client.scan_fusion = {"preset": "indoor_balanced", "voxel_size": 0.2, "occupied_min_hits": 3, "occupied_over_free_ratio": 0.75, "turn_skip_wz": 0.45, "skip_turn_frames": True}
    client.scan_fusion_preset_var = Var("indoor_balanced")
    client.voxel_var = Var("0.20")
    client.occupied_min_hits_var = Var("3")
    client.occupied_over_free_ratio_var = Var("0.75")
    client.turn_skip_wz_var = Var("0.45")
    client.skip_turn_frames_var = Var(True)
    client.number = DesktopClient.number.__get__(client, DesktopClient)
    client.effective_scan_fusion_config = DesktopClient.effective_scan_fusion_config.__get__(client, DesktopClient)
    client.filtered_occupancy_cells = DesktopClient.filtered_occupancy_cells.__get__(client, DesktopClient)

    payload = DesktopClient.browser_occupancy(client)

    assert payload["occupied_cells"] == [{"ix": 2, "iy": 2, "hits": 4, "intensity": 0.9}]


def test_browser_occupancy_filters_out_hidden_free_cells() -> None:
    class Var:
        def __init__(self, value) -> None:
            self.value = value

        def get(self):
            return self.value

        def set(self, value) -> None:
            self.value = value

    client = DesktopClient.__new__(DesktopClient)
    client.scan = {
        "voxel": 0.2,
        "occupied": {
            "1:1": {"ix": 1, "iy": 1, "hits": 5, "intensity": 0.9},
        },
        "free": {
            "1:1": {"ix": 1, "iy": 1, "hits": 3},
            "2:2": {"ix": 2, "iy": 2, "hits": 4},
        },
    }
    client.scan_fusion = {"preset": "indoor_balanced", "voxel_size": 0.2, "occupied_min_hits": 3, "occupied_over_free_ratio": 0.75, "turn_skip_wz": 0.45, "skip_turn_frames": True}
    client.scan_fusion_preset_var = Var("indoor_balanced")
    client.voxel_var = Var("0.20")
    client.occupied_min_hits_var = Var("3")
    client.occupied_over_free_ratio_var = Var("0.75")
    client.turn_skip_wz_var = Var("0.45")
    client.skip_turn_frames_var = Var(True)
    client.number = DesktopClient.number.__get__(client, DesktopClient)
    client.effective_scan_fusion_config = DesktopClient.effective_scan_fusion_config.__get__(client, DesktopClient)

    payload = DesktopClient.browser_occupancy(client)

    assert payload["free_cells"] == [{"ix": 2, "iy": 2, "hits": 4}]


def test_server_grid_browser_occupancy_overrides_local_scan_when_not_editing_loaded_map() -> None:
    class Var:
        def __init__(self, value) -> None:
            self.value = value

        def get(self):
            return self.value

    client = DesktopClient.__new__(DesktopClient)
    client.scan = {
        "voxel": 0.2,
        "occupied": {"9:9": {"ix": 9, "iy": 9, "hits": 5, "intensity": 0.9}},
        "free": {},
    }
    client.server_grid = {
        "active": True,
        "resolution": 0.5,
        "occupied_cells": [{"ix": 1, "iy": 2, "hits": 3, "intensity": 1.0}],
        "free_cells": [{"ix": 3, "iy": 4, "hits": 2}],
    }
    client.edit = {"loaded_from_stcm": False}
    client.scan_fusion = {"preset": "indoor_balanced", "voxel_size": 0.2, "occupied_min_hits": 3, "occupied_over_free_ratio": 0.75, "turn_skip_wz": 0.45, "skip_turn_frames": True}
    client.scan_fusion_preset_var = Var("indoor_balanced")
    client.voxel_var = Var("0.20")
    client.occupied_min_hits_var = Var("3")
    client.occupied_over_free_ratio_var = Var("0.75")
    client.turn_skip_wz_var = Var("0.45")
    client.skip_turn_frames_var = Var(True)
    client.number = DesktopClient.number.__get__(client, DesktopClient)
    client.effective_scan_fusion_config = DesktopClient.effective_scan_fusion_config.__get__(client, DesktopClient)

    payload = DesktopClient.browser_occupancy(client)

    assert payload["voxel_size"] == 0.5
    assert payload["occupied_cells"] == [{"ix": 1, "iy": 2, "hits": 3, "intensity": 1.0}]
    assert payload["free_cells"] == [{"ix": 3, "iy": 4, "hits": 2}]


def test_browser_occupancy_includes_closed_map_fence_from_server_grid() -> None:
    client = build_minimal_client()
    client.server_grid = {
        "active": True,
        "resolution": 0.5,
        "occupied_cells": [{"ix": 1, "iy": 2, "hits": 3, "intensity": 1.0}],
        "free_cells": [],
        "origin": {"x": -1.0, "y": 2.0},
        "width": 4,
        "height": 3,
    }
    client.edit = {"loaded_from_stcm": False}
    client.active_map_fence_xy = DesktopClient.active_map_fence_xy.__get__(client, DesktopClient)
    client.effective_scan_fusion_config = lambda: {}

    payload = DesktopClient.browser_occupancy(client)

    assert payload["map_fence_xy"] == [
        {"x": -1.0, "y": 2.0},
        {"x": 1.0, "y": 2.0},
        {"x": 1.0, "y": 3.5},
        {"x": -1.0, "y": 3.5},
        {"x": -1.0, "y": 2.0},
    ]


def test_browser_occupancy_includes_closed_map_fence_from_active_cells() -> None:
    client = build_minimal_client()
    client.scan["voxel"] = 0.5
    client.active_occupancy_cells = lambda: [{"ix": 2, "iy": 1, "hits": 3, "intensity": 1.0}]
    client.active_free_cells = lambda: [{"ix": 4, "iy": 3, "hits": 1}]
    client.active_voxel_size = lambda: 0.5
    client.should_use_server_grid = lambda: False
    client.active_map_fence_xy = DesktopClient.active_map_fence_xy.__get__(client, DesktopClient)
    client.effective_scan_fusion_config = lambda: {}

    payload = DesktopClient.browser_occupancy(client)

    assert payload["map_fence_xy"] == [
        {"x": 1.0, "y": 0.5},
        {"x": 2.5, "y": 0.5},
        {"x": 2.5, "y": 2.0},
        {"x": 1.0, "y": 2.0},
        {"x": 1.0, "y": 0.5},
    ]


def test_consume_messages_stores_server_grid_payload() -> None:
    import queue

    class Bridge:
        def __init__(self) -> None:
            self.queue = queue.Queue()

    class Var:
        def __init__(self) -> None:
            self.value = ""

        def set(self, value) -> None:
            self.value = value

    client = DesktopClient.__new__(DesktopClient)
    client.bridge = Bridge()
    client.camera_inbox = {}
    client.server_grid = {"active": False, "resolution": 0.0, "occupied_cells": [], "free_cells": []}
    client.pose = {}
    client.gps = {}
    client.odom = {}
    client.chassis = {}
    client.pose_history = []
    client.scan = {"front_frames": 0, "rear_frames": 0}
    client.last_scan = {"front": {}, "rear": {}}
    client.last_message_at_ms = 0
    client.camera_refresh_var = Var()
    client.mark_canvas_dirty = lambda: None
    client.sync_scan_badges = lambda: None
    client.validate_message = lambda msg: True
    client.consume_messages = DesktopClient.consume_messages.__get__(client, DesktopClient)

    client.bridge.queue.put(
        {
            "topic": "/map/grid",
            "stamp": 1.25,
            "payload": {
                "resolution": 0.4,
                "origin": {"x": 0.0, "y": 0.0},
                "width": 3,
                "height": 2,
                "data": [-1, 0, 100, 50, -1, 0],
            },
        }
    )

    DesktopClient.consume_messages(client)

    assert client.server_grid["active"] is True
    assert client.server_grid["resolution"] == 0.4
    assert client.server_grid["data"] == [-1, 0, 100, 50, -1, 0]
    assert client.server_grid["occupied_cells"] == [
        {"ix": 2, "iy": 0, "hits": 3, "intensity": 1.0},
        {"ix": 0, "iy": 1, "hits": 3, "intensity": 1.0},
    ]
    assert client.server_grid["free_cells"] == [{"ix": 1, "iy": 0, "hits": 3}, {"ix": 2, "iy": 1, "hits": 3}]


def test_consume_messages_does_not_queue_lidar_for_map_accumulation() -> None:
    import queue

    class Bridge:
        def __init__(self) -> None:
            self.queue = queue.Queue()

    class Var:
        def __init__(self) -> None:
            self.value = ""

        def set(self, value) -> None:
            self.value = value

    client = DesktopClient.__new__(DesktopClient)
    client.bridge = Bridge()
    client.camera_inbox = {}
    client.server_grid = {"active": False, "resolution": 0.0, "occupied_cells": [], "free_cells": []}
    client.pose = {}
    client.gps = {}
    client.odom = {}
    client.chassis = {}
    client.pose_history = []
    client.scan = {"front_frames": 0, "rear_frames": 0, "active": True}
    client.last_scan = {"front": {}, "rear": {}}
    client.last_message_at_ms = 0
    client.camera_refresh_var = Var()
    client.mark_canvas_dirty = lambda: None
    client.validate_message = lambda msg: True
    client.sync_scan_badges = lambda: None

    client.bridge.queue.put(
        {
            "topic": "/lidar/front",
            "stamp": 1.25,
            "payload": {
                "points": [[1.0, 2.0, 1.0]],
                "raw_points": 1,
                "keyframe": True,
            },
        }
    )

    DesktopClient.consume_messages(client)

    assert client.scan["front_frames"] == 1


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

    DesktopClient.apply_scan_fusion_config(client, {"voxel_size": 0.1, "occupied_min_hits": 4, "occupied_over_free_ratio": 0.65, "turn_skip_wz": 0.5}, update_vars=True)

    assert client.scan["voxel"] == 0.1
    assert client.occupied_min_hits_var.get() == "4"


def test_apply_scan_fusion_config_accepts_legacy_preset_defaults() -> None:
    class Var:
        def __init__(self, value) -> None:
            self.value = value

        def get(self):
            return self.value

        def set(self, value) -> None:
            self.value = value

    client = DesktopClient.__new__(DesktopClient)
    client.scan = {"voxel": 0.2}
    client.scan_fusion = {"voxel_size": 0.2, "occupied_min_hits": 2, "occupied_over_free_ratio": 0.75, "turn_skip_wz": 0.45, "skip_turn_frames": True}
    client.scan_fusion_preset_var = Var("")
    client.voxel_var = Var("0.20")
    client.occupied_min_hits_var = Var("2")
    client.occupied_over_free_ratio_var = Var("0.75")
    client.turn_skip_wz_var = Var("0.45")
    client.skip_turn_frames_var = Var(True)
    client.apply_scan_fusion_config = DesktopClient.apply_scan_fusion_config.__get__(client, DesktopClient)

    DesktopClient.apply_scan_fusion_config(client, {"preset": "warehouse_sparse"}, update_vars=True)

    assert client.scan["voxel"] == 0.10
    assert client.voxel_var.get() == "0.10"
    assert client.occupied_min_hits_var.get() == "2"
    assert client.occupied_over_free_ratio_var.get() == "0.65"
    assert client.turn_skip_wz_var.get() == "0.50"


def test_start_scan_waits_when_server_rejects_prereq(monkeypatch) -> None:
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
    assert client.scan["phase"] == "waiting_mapping"
    assert client.scan["pending_start"] is True
    assert client.scan["error_reason"] == "mapping_prereq_failed"
    assert warnings


def test_coalesce_stream_messages_prefers_latest_pose_and_lidar() -> None:
    messages = [
        {"topic": "/robot/pose", "stamp": 1.0, "payload": {"x": 1}},
        {"topic": "/lidar/front", "stamp": 1.0, "payload": {"points": [[0, 0, 1]]}},
        {"topic": "/robot/pose", "stamp": 2.0, "payload": {"x": 2}},
        {"topic": "/lidar/front", "stamp": 2.0, "payload": {"points": [[1, 0, 1]]}},
    ]

    merged = coalesce_stream_messages(messages)

    assert [item["topic"] for item in merged] == ["/robot/pose", "/lidar/front"]
    assert merged[0]["payload"]["x"] == 2
    assert merged[1]["payload"]["points"] == [[1, 0, 1]]


def test_consume_messages_coalesces_and_prioritizes_pose_updates() -> None:
    class Bridge:
        def __init__(self) -> None:
            self.queue = __import__("queue").Queue()

    class Var:
        def __init__(self, value) -> None:
            self.value = value

        def get(self):
            return self.value

        def set(self, value) -> None:
            self.value = value

    client = DesktopClient.__new__(DesktopClient)
    client.bridge = Bridge()
    client.camera_inbox = {1: {"objects": [], "meta": {}}}
    client.camera_refresh_var = Var("")
    client.pose = {}
    client.gps = {}
    client.odom = {}
    client.chassis = {}
    client.pose_history = []
    client.scan = {"front_frames": 0, "rear_frames": 0, "active": True}
    client.last_scan = {"front": {}, "rear": {}}
    client.validate_message = lambda msg: True
    dirty = []
    client.mark_canvas_dirty = lambda: dirty.append("dirty")

    for index in range(MAX_MESSAGES_DRAIN_PER_TICK):
        client.bridge.queue.put({"topic": "/lidar/front", "stamp": float(index), "payload": {"points": [[index, 0, 1]], "raw_points": 1, "keyframe": False}})
    client.bridge.queue.put({"topic": "/robot/pose", "stamp": 99.0, "payload": {"x": 9.0}})

    DesktopClient.consume_messages(client)

    assert client.pose == {"x": 9.0}
    assert dirty


def test_render_canvas_if_needed_skips_when_not_dirty() -> None:
    client = DesktopClient.__new__(DesktopClient)
    client.canvas_dirty = False
    client.canvas_revision = 3
    client.last_render_revision = 3
    calls = []
    client.render_canvas_contents = lambda: calls.append("render")

    DesktopClient.render_canvas_if_needed(client)

    assert calls == []


def test_render_canvas_if_needed_renders_when_dirty() -> None:
    client = DesktopClient.__new__(DesktopClient)
    client.canvas_dirty = True
    client.canvas_revision = 4
    client.last_render_revision = 3
    calls = []
    client.render_canvas_contents = lambda: calls.append("render")

    DesktopClient.render_canvas_if_needed(client)

    assert calls == ["render"]
    assert client.canvas_dirty is False
    assert client.last_render_revision == 4
