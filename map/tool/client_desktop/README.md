# Desktop Client Rewrite

`client_desktop` now includes rewritten desktop-client source copies in:

- `app.py.txt`
- `run_client.py.txt`

This follows the workspace rule for this task: Python changes are provided as full `.txt` copies and do not modify the original `.py` files directly.

## Coverage

The rewritten desktop client copy includes the main browser-side capabilities:

- websocket + HTTP server connection
- scan accumulation and map canvas rendering
- local STCM save / load
- second-stage map editing
- erase noise
- draw obstacle line
- POI add / delete / geo apply
- Path connect by POI name
- Path connect by any two points
- closed-loop validation with invalid path highlight
- move controls
- status / camera / communication panels

## How To Use

1. Review `app.py.txt`.
2. When you are ready to adopt it, replace `app.py` with the contents of `app.py.txt`.
3. If needed, replace `run_client.py` with `run_client.py.txt`.
4. Run the desktop client as before.
