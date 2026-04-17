from pathlib import Path

from client_desktop import run_client


def test_load_main_from_local_app_returns_callable() -> None:
    main = run_client.load_main_from_local_app(Path(run_client.__file__).resolve().parent / "app.py")

    assert callable(main)


def test_load_main_from_local_app_fails_fast_when_frozen_bundle_misses_app(monkeypatch) -> None:
    monkeypatch.setattr(run_client, "_main", None)
    monkeypatch.setattr(run_client, "_main_import_error", ModuleNotFoundError("No module named 'app'"))
    monkeypatch.setattr(run_client.sys, "frozen", True, raising=False)

    try:
        run_client.load_main_from_local_app(Path("/tmp/should-not-be-used/app.py"))
    except RuntimeError as exc:
        assert "bundle is missing app module" in str(exc)
        assert "ModuleNotFoundError" in str(exc)
    else:
        raise AssertionError("expected RuntimeError for missing bundled app module")
