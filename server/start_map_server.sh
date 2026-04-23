#!/bin/bash

# 当前执行目录
PWD_DIR="$(pwd)"
echo "当前执行目录: $PWD_DIR"

# 日志文件（放在当前目录下）
LOG_FILE="$PWD_DIR/server.log"
LOG_DIR=$(dirname "$LOG_FILE")

# 如果日志目录不存在则创建
if [ ! -d "$LOG_DIR" ]; then
	    mkdir -p "$LOG_DIR"
fi

# 加载环境
source ~/caddie/install/setup.bash

# 启动服务
nohup python3 /home/root/ros2_ws/server/run_map_server.py > "$LOG_FILE" 2>&1 &

echo "Server started with PID: $!"
