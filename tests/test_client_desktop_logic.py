from client_desktop.logic import (
    Point,
    build_auto_loop_segments,
    build_scan_fusion_metadata,
    build_poi_copy_text,
    compute_path_closed_loop_validation,
    extract_scan_fusion_config,
    is_occupied_scan_cell,
    infer_missing_geo_points,
    parse_batch_poi_text,
    plan_path_points,
    resolve_scan_fusion_config,
    should_skip_scan_by_turn,
)


def test_parse_batch_poi_text_supports_name_geo_and_yaw() -> None:
    rows = parse_batch_poi_text("a,120.0,30.0\nb,120.1,30.2\nc,120.2,30.3,1.57\nd")
    assert [row["name"] for row in rows] == ["a", "b", "c", "d"]
    assert rows[0]["lat"] == 30.0
    assert rows[1]["lon"] == 120.1
    assert rows[1]["lat"] == 30.2
    assert rows[2]["yaw"] == 1.57
    assert rows[3]["lat"] is None


def test_parse_batch_poi_text_rejects_one_or_two_geo_points() -> None:
    try:
        parse_batch_poi_text("a,120.0,30.0\nb")
    except ValueError as exc:
        assert "at least 3 POI" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_infer_missing_geo_points_fills_points_after_three_anchors() -> None:
    points = [
        Point(name="A", x=0, y=0, lon=120.0, lat=30.0),
        Point(name="B", x=10, y=0, lon=120.1, lat=30.0),
        Point(name="C", x=0, y=10, lon=120.0, lat=30.1),
        Point(name="D", x=10, y=10),
    ]
    infer_missing_geo_points(points)
    assert points[3].lon is not None
    assert points[3].lat is not None
    assert abs(points[3].lon - 120.1) < 1e-6
    assert abs(points[3].lat - 30.1) < 1e-6


def test_build_poi_copy_text_matches_web_format() -> None:
    text = build_poi_copy_text(
        [
            Point(name="A", x=1, y=2, yaw=0.25, lat=30.1234567, lon=120.7654321),
            Point(name="B", x=3, y=4),
        ]
    )
    assert text == (
        "A,1.000,2.000,0.250,30.123457,120.765432\n"
        "B,3.000,4.000,0.000,,"
    )


def test_build_auto_loop_segments_returns_closed_route() -> None:
    points = [
        Point(name="A", x=0, y=0, poi_id="poi-1"),
        Point(name="B", x=4, y=0, poi_id="poi-2"),
        Point(name="C", x=4, y=4, poi_id="poi-3"),
        Point(name="D", x=0, y=4, poi_id="poi-4"),
    ]
    segments = build_auto_loop_segments(points, voxel_size=0.5, occupied_cells={})
    assert len(segments) == 4
    assert segments[-1].end.poi_id == segments[0].start.poi_id


def test_plan_path_points_detours_around_obstacle_cells() -> None:
    occupied = {
        (1, 0): {"ix": 1, "iy": 0},
        (2, 0): {"ix": 2, "iy": 0},
        (3, 0): {"ix": 3, "iy": 0},
    }
    points = plan_path_points(
        Point(name="A", x=0, y=0),
        Point(name="B", x=4, y=0),
        voxel_size=1.0,
        occupied_cells=occupied,
        clearance=0.0,
    )
    assert points[0].x == 0
    assert points[-1].x == 4
    assert any(point.y != 0 for point in points[1:-1])


def test_compute_path_closed_loop_validation_reports_disconnected_path() -> None:
    a = Point(name="A", x=0, y=0, poi_id="poi-1")
    b = Point(name="B", x=1, y=0, poi_id="poi-2")
    c = Point(name="C", x=2, y=0, poi_id="poi-3")
    d = Point(name="D", x=10, y=0, poi_id="poi-4")
    e = Point(name="E", x=11, y=0, poi_id="poi-5")
    result = compute_path_closed_loop_validation(
        [
            {"id": "seg-1", "start": a, "end": b},
            {"id": "seg-2", "start": b, "end": c},
            {"id": "seg-3", "start": d, "end": e},
        ],
        voxel_size=0.12,
    )
    assert result["ok"] is False
    assert "Closed-loop check failed" in result["message"]
    assert result["invalid_ids"] == {"seg-1", "seg-2", "seg-3"}


def test_resolve_scan_fusion_config_merges_defaults_and_overrides_without_preset() -> None:
    config = resolve_scan_fusion_config(None, {"occupied_min_hits": 5, "skip_turn_frames": False})

    assert "preset" not in config
    assert config["voxel_size"] == 0.15
    assert config["occupied_min_hits"] == 5
    assert config["occupied_over_free_ratio"] == 0.6
    assert config["skip_turn_frames"] is False


def test_is_occupied_scan_cell_uses_effective_thresholds() -> None:
    strict = resolve_scan_fusion_config(None, {"occupied_min_hits": 3, "occupied_over_free_ratio": 0.9})
    sensitive = resolve_scan_fusion_config(None, {"occupied_min_hits": 1, "occupied_over_free_ratio": 0.55})
    cell = {"hits": 2}
    free = {"hits": 2}

    assert is_occupied_scan_cell(cell, free, strict) is False
    assert is_occupied_scan_cell(cell, free, sensitive) is True


def test_should_skip_scan_by_turn_depends_on_effective_config() -> None:
    skip_turns = resolve_scan_fusion_config(None, {"turn_skip_wz": 0.35, "skip_turn_frames": True})
    keep_turns = resolve_scan_fusion_config(None, {"turn_skip_wz": 0.60, "skip_turn_frames": False})

    assert should_skip_scan_by_turn(0.4, False, skip_turns) is True
    assert should_skip_scan_by_turn(0.4, False, keep_turns) is False


def test_extract_scan_fusion_config_prefers_manifest_metadata_and_ignores_preset_output() -> None:
    config = extract_scan_fusion_config(
        {
            "scan_fusion": {
                "preset": "indoor_sensitive",
                "voxel_size": 0.07,
                "occupied_min_hits": 2,
                "occupied_over_free_ratio": 0.6,
                "turn_skip_wz": 0.5,
                "skip_turn_frames": False,
            },
            "notes": "{\"voxelSize\": 0.11}",
        },
        default_preset="indoor_balanced",
    )

    assert "preset" not in config
    assert config["voxel_size"] == 0.07
    assert config["skip_turn_frames"] is False


def test_extract_scan_fusion_config_uses_notes_voxel_when_manifest_metadata_missing() -> None:
    config = extract_scan_fusion_config({"notes": "{\"voxelSize\": 0.09}"}, default_preset="indoor_balanced")

    assert "preset" not in config
    assert config["voxel_size"] == 0.09
    assert config["occupied_min_hits"] == 1


def test_build_scan_fusion_metadata_keeps_effective_values() -> None:
    config = resolve_scan_fusion_config(None, {"voxel_size": 0.1, "occupied_min_hits": 4, "occupied_over_free_ratio": 0.65, "turn_skip_wz": 0.5})
    payload = build_scan_fusion_metadata(config)

    assert payload == {
        "voxel_size": 0.1,
        "occupied_min_hits": 4,
        "occupied_over_free_ratio": 0.65,
        "turn_skip_wz": 0.5,
        "skip_turn_frames": True,
    }
