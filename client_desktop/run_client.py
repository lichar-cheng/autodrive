from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


_main_import_error: Exception | None = None

try:
    from .app import main as _main
except Exception as exc:
    _main_import_error = exc
    try:
        from app import main as _main  # type: ignore
        _main_import_error = None
    except Exception as inner_exc:
        _main_import_error = inner_exc
        _main = None


def is_frozen_runtime() -> bool:
    return bool(getattr(sys, "frozen", False))


def load_main_from_local_app(app_path: Path | None = None):
    if callable(_main):
        return _main
    if is_frozen_runtime():
        detail = ""
        if _main_import_error is not None:
            detail = f" Original import error: {type(_main_import_error).__name__}: {_main_import_error}"
        raise RuntimeError(
            "frozen bundle is missing app module or failed during app import; rebuild with PyInstaller hidden imports for app and logic."
            + detail
        )

    target = Path(app_path) if app_path is not None else Path(__file__).resolve().with_name("app.py")
    target_dir = str(target.resolve().parent)
    if target_dir not in sys.path:
        sys.path.insert(0, target_dir)

    spec = importlib.util.spec_from_file_location("autodrive_client_desktop_app", target)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"unable to load app module from {target}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    main = getattr(module, "main", None)
    if not callable(main):
        raise RuntimeError(f"main() not found in {target}")
    return main


if __name__ == "__main__":
    load_main_from_local_app()()
