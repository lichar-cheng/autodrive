#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

APP_NAME="${APP_NAME:-maptool_001_20260408}"

python3 -m pip install -r requirements.txt
python3 -m pip install pyinstaller
pyinstaller --noconfirm --windowed --name "$APP_NAME" run_client.py
