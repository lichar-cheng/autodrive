from client_desktop.logic import (
    Point,
    build_auto_loop_segments,
    build_poi_copy_text,
    compute_path_closed_loop_validation,
    infer_missing_geo_points,
    parse_batch_poi_text,
    plan_path_points,
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
