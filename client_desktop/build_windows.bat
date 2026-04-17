@echo off
REM Windows 打包为独立 EXE
if "%APP_NAME%"=="" set APP_NAME=maptool_001_20260408
python -m pip install -r requirements.txt
python -m pip install pyinstaller
python -c "import tkinter"
if errorlevel 1 (
  echo Current Python does not include tkinter/Tk support. Install a Python build with tkinter before packaging.
  exit /b 1
)
if exist build rmdir /s /q build
if exist dist rmdir /s /q dist
python -m PyInstaller ^
  --noconfirm ^
  --onefile ^
  --windowed ^
  --name %APP_NAME% ^
  --paths "%CD%" ^
  --hidden-import app ^
  --hidden-import logic ^
  --hidden-import tkinter ^
  --hidden-import tkinter.ttk ^
  --hidden-import requests ^
  --hidden-import websocket ^
  run_client.py
