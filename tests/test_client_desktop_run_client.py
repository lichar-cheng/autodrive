from pathlib import Path

from client_desktop import run_client


def test_load_main_from_local_app_returns_callable() -> None:
    main = run_client.load_main_from_local_app(Path(run_client.__file__).resolve().parent / "app.py")

    assert callable(main)
