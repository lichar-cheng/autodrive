#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

APP_NAME="${APP_NAME:-maptool_001_20260408}"

python3 -m pip install -r requirements.txt
python3 -m pip install pyinstaller
python3 -c "import tkinter"
rm -rf build dist
python3 -m PyInstaller \
  --noconfirm \
  --windowed \
  --name "$APP_NAME" \
  --paths "$PWD" \
  --hidden-import app \
  --hidden-import logic \
  --hidden-import tkinter \
  --hidden-import tkinter.ttk \
  --hidden-import requests \
  --hidden-import websocket \
  run_client.py
