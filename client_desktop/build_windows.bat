@echo off
REM Windows 打包为独立 EXE
if "%APP_NAME%"=="" set APP_NAME=maptool_001_20260408
python -m pip install -r requirements.txt
python -m pip install pyinstaller
pyinstaller --noconfirm --onefile --windowed --name %APP_NAME% run_client.py
