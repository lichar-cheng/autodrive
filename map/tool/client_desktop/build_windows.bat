@echo off
REM Windows 打包为独立 EXE
python -m pip install -r requirements.txt
python -m pip install pyinstaller
pyinstaller --noconfirm --onefile --windowed --name AutoDriveClient run_client.py
