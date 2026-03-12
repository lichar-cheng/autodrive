# Windows Desktop Client (主功能端)

这是面向 Windows 的主客户端，支持打包为独立 `exe`。

## 功能
- 连接机器人 server（WS + HTTP）
- 两种画轨迹方式：
  1) 选择两个 POI 自动连线
  2) 点击任意两点连线
- POI 配置经纬度（lat/lon）并下发 server
- 键盘近场控制（W/A/S/D，Space急停）

## 运行

```bash
cd map/tool/client_desktop
pip install -r requirements.txt
python run_client.py
```

## 打包 EXE（Windows）
运行 `build_windows.bat`。

产物在 `dist/AutoDriveClient.exe`。
