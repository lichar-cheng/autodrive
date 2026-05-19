# -*- coding: utf-8 -*-
# AutoDrive Mapping Server 服务端主入口。
#
# 这个文件负责：
# 1. 启动 FastAPI 服务，并提供控制、扫描、诊断、WebSocket 数据流等接口；
# 2. 在 ROS 可用时桥接 `/robot/pose` 与 `/map/grid` 两类核心数据；
# 3. 管理 2D/3D 扫描会话、PCD 下载和 WebSocket 流控；
# 4. 统一维护运行时状态，便于 /health 和 /diag/* 接口诊断。

# 启用延迟类型注解，避免运行时过早解析类型。
from __future__ import annotations

# 标准库依赖：异步任务、进程管理、哈希、JSON、日志、文件路径等。
import asyncio
import copy
import hashlib
import json
import logging
import os
import signal
import subprocess
import threading
import time
from collections import defaultdict
from pathlib import Path
from typing import Any

# FastAPI 相关依赖：HTTP 接口、WebSocket 和响应对象。
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware

# 项目内部模块：配置、请求模型、ROS 桥接、主题总线和 PCD 处理。
from .sample_pcd_file import downsample_pcd_to_cache
from .config import CONFIG
from .models import (
    ControlTargetRequest,
    MoveCommand,
    StartScanRequest,
    StopScanRequest,
)
from .ros_bridge import RosRuntime, detect_ros
from .topic_bus import TopicBus


# 初始化全局日志格式，便于定位服务端运行状态和异常。
logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(levelname)s | %(name)s | %(message)s")
logger = logging.getLogger("autodrive.server")


# 过滤成功的 POST 访问日志，避免高频控制接口刷屏；失败或非 POST 请求仍然保留。
class _SuccessPostAccessLogFilter(logging.Filter):
    def filter(self, record: logging.LogRecord) -> bool:
        args = record.args if isinstance(record.args, tuple) else ()
        method = str(args[1]) if len(args) >= 5 else ""
        try:
            status_code = int(args[4]) if len(args) >= 5 else 0
        except (TypeError, ValueError):
            status_code = 0
        if method.upper() == "POST" and 200 <= status_code < 300:
            logging.getLogger(record.name).debug(record.getMessage())
            return False
        return True


# 安装 uvicorn.access 日志过滤器，只安装一次，避免重复添加 filter。
def _install_access_log_filter() -> None:
    access_logger = logging.getLogger("uvicorn.access")
    if not any(isinstance(existing, _SuccessPostAccessLogFilter) for existing in access_logger.filters):
        access_logger.addFilter(_SuccessPostAccessLogFilter())


_install_access_log_filter()

# 创建 FastAPI 应用实例，并开放跨域，方便前端或调试工具直接访问。
app = FastAPI(title="AutoDrive Mapping Server", version="0.6.0")
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

# 核心运行时对象：主题总线、ROS 运行时状态和最近一份地图栅格缓存。
bus = TopicBus(queue_size=CONFIG.ws_queue_size)
ros: RosRuntime = RosRuntime(enabled=False, reason="ROS runtime not initialized")
latest_occupancy_grid: dict[str, Any] | None = None
seq_by_topic: dict[str, int] = defaultdict(int)
ws_clients: set[int] = set()
motion_command_seq = 0
# 运动控制目标。velocity 为线速度，yaw_rate 为角速度，updated_at 用于判断指令是否过期。
CONTROL_TARGET = {"velocity": 0.0, "yaw_rate": 0.0, "updated_at": 0.0}
CONTROL_PUBLISH_INTERVAL_SEC = 0.1
CONTROL_TARGET_HOLD_SEC = 1.0
CONTROL_STOP_BURST_TICKS = 3
CONTROL_STOP_BURST_REMAINING = 0
# 运动控制运行时诊断信息，用于记录最近一次发布、归零和日志输出状态。
CONTROL_RUNTIME = {
    "last_zero_source": "",
    "last_zero_at": 0.0,
    "last_publish_source": "",
    "last_logged_source": "",
    "last_logged_velocity": None,
    "last_logged_yaw_rate": None,
}
CONTROL_OUTPUT_STATE = {"velocity": 0.0, "yaw_rate": 0.0, "updated_at": 0.0}
control_task: asyncio.Task | None = None

# 控制平滑参数：只保留起步/转向缓启与停车减速，不再依赖超声波配置。
CONTROL_LINEAR_ACCEL_MPS2 = 0.16
CONTROL_LINEAR_DECEL_MPS2 = 0.6
CONTROL_LINEAR_EMERGENCY_DECEL_MPS2 = 1.0
CONTROL_ANGULAR_ACCEL_RPS2 = 0.45
CONTROL_ANGULAR_DECEL_RPS2 = 1.2
CONTROL_ANGULAR_EMERGENCY_DECEL_RPS2 = 2.0
# 默认 voxel 边长，单位 m。
# 数值越大，点云越稀疏，文件越小；数值越小，点云越密。
PCD_DEFAULT_VOXEL_SIZE = 0.05

# WebSocket 默认订阅的实时数据主题。
STREAM_TOPICS = [
    "/robot/pose",
    "/map/grid",
]

# 仅保留 pose 与 occupancy_grid 两类流，因此这里只需要空的按 topic 限频表。
TOPIC_MIN_INTERVAL_SEC: dict[str, float] = {}

# WebSocket 队列和客户端保活相关参数。
QUEUE_NEAR_CAPACITY_RATIO = 0.8
QUEUE_WARN_INTERVAL_SEC = 5.0
CLIENT_IDLE_TIMEOUT_SEC = 20.0

# 服务端流控统计，主要用于诊断 WebSocket 队列压力和主动断开次数。
SERVER_RUNTIME = {
    "ws_overflow_total": 0,
    "ws_near_capacity_total": 0,
    "ws_last_warn_at": 0.0,
    "active_ws_connections_peak": 0,
    "forced_disconnect_total": 0,
}

# 当前扫描会话状态：是否正在扫描、模式、帧数、点数、依赖状态和 PCD 传输状态。
SCAN_SESSION = {
    "active": False,
    "mode": "3d",
    "started_at": 0.0,
    "stopped_at": 0.0,
    "voxel_size": 0.12,
    "front_frames": 0,
    "rear_frames": 0,
    "raw_points": 0,
    "dependency_status": {
        "required_nodes": [],
        "missing_nodes": [],
        "started_nodes": [],
        "required_processes": [],
        "missing_processes": [],
        "started_processes": [],
        "errors": [],
    },
    "pcd_transfer_state": "idle",
    "pcd_file": None,
}

# 由服务端拉起的扫描依赖进程，停止扫描时会尝试一起清理。
LAUNCHED_SCAN_PROCESSES: list[subprocess.Popen[str]] = []
LAUNCHED_SCAN_COMMANDS: dict[str, list[subprocess.Popen[str]]] = {}
SCAN_START_LOCK = threading.Lock()
SCAN_START_CANCEL_SEQ = 0
SCAN_DEPENDENCY_POLL_ATTEMPTS = 10
SCAN_DEPENDENCY_POLL_INTERVAL_SEC = 1.0
SCAN_PROCESS_STOP_POLL_ATTEMPTS = 15
SCAN_PROCESS_STOP_POLL_INTERVAL_SEC = 0.2
SCAN_MAPPING_PREREQ_POLL_ATTEMPTS = 10
SCAN_MAPPING_PREREQ_POLL_INTERVAL_SEC = 0.5


# 根据 topic、时间戳、序号和 payload 生成消息校验值，便于前端判断消息是否完整或重复。
def _checksum(topic: str, stamp: float, seq: int, payload: dict) -> str:
    payload_json = json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    raw = f"{topic}|{stamp:.6f}|{seq}|{payload_json}".encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


# 统一封装即将通过 WebSocket 发送的消息，补充服务端时间、递增序号和校验和。
def _pack_message(message: dict) -> dict:
    topic = message["topic"]
    stamp = float(message["stamp"])
    payload = message["payload"]
    seq_by_topic[topic] += 1
    seq = seq_by_topic[topic]
    return {
        "topic": topic,
        "stamp": stamp,
        "server_time_ms": int(time.time() * 1000),
        "seq": seq,
        "payload": payload,
        "checksum": _checksum(topic, stamp, seq, payload),
    }


# 判断 WebSocket 是否已经关闭后仍尝试发送，便于优雅处理断开场景。
def _is_websocket_send_after_close_error(exc: RuntimeError) -> bool:
    return 'Cannot call "send" once a close message has been sent.' in str(exc)


# 重置扫描会话状态；可选保留或更新 voxel_size，但不会直接清理真实文件。
def _reset_scan_session(voxel_size: float | None = None, keep_points: bool = False) -> None:
    SCAN_SESSION["active"] = False
    SCAN_SESSION["mode"] = "3d"
    SCAN_SESSION["started_at"] = 0.0
    SCAN_SESSION["stopped_at"] = 0.0
    SCAN_SESSION["front_frames"] = 0
    SCAN_SESSION["rear_frames"] = 0
    SCAN_SESSION["raw_points"] = 0
    SCAN_SESSION["dependency_status"] = {
        "required_nodes": [],
        "missing_nodes": [],
        "started_nodes": [],
        "required_processes": [],
        "missing_processes": [],
        "started_processes": [],
        "errors": [],
    }
    SCAN_SESSION["pcd_transfer_state"] = "idle"
    SCAN_SESSION["pcd_file"] = None
    if voxel_size is not None:
        SCAN_SESSION["voxel_size"] = max(0.02, float(voxel_size))


# 规范化扫描模式，只允许 2d 或 3d，非法值返回 None。
def _normalize_scan_mode(mode: str | None) -> str | None:
    normalized = str(mode or "").strip().lower()
    return normalized if normalized in {"2d", "3d"} else None


# 获取当前 3D 扫描对应的 PCD 文件路径，并返回结构化错误信息。
def _scan_pcd_path(mode: str | None = None) -> tuple[Path | None, dict | None]:
    normalized_mode = _normalize_scan_mode(mode or str(SCAN_SESSION.get("mode", "2d")))
    if normalized_mode != "3d":
        return None, {"reason": "pcd_unavailable_for_mode", "error": "pcd is only available in 3d mode", "status_code": 409}
    pcd_path = _pcd_output_path_for_mode("3d")
    if pcd_path is None:
        return None, {"reason": "pcd_path_not_configured", "status_code": 404}
    if not pcd_path.exists():
        return None, {"reason": "pcd_file_missing", "error": f"pcd file not found: {pcd_path}", "status_code": 404}
    return pcd_path, None


# 根据扫描模式选择对应配置；未知模式直接抛错，防止静默使用错误配置。
def _scan_mode_config(mode: str) -> Any:
    if mode == "2d":
        return CONFIG.scan_modes.mode_2d
    if mode == "3d":
        return CONFIG.scan_modes.mode_3d
    raise ValueError(f"unsupported scan mode: {mode}")


# 使用 pgrep 查找符合模式的进程，用于判断外部 ROS/扫描依赖是否已启动。
def _list_process_matches(pattern: str) -> list[str]:
    completed = subprocess.run(
        ["pgrep", "-af", pattern],
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode not in {0, 1}:
        detail = str(completed.stderr or completed.stdout or "").strip()
        raise RuntimeError(detail or f"pgrep failed for {pattern}")
    return [line.strip() for line in str(completed.stdout or "").splitlines() if line.strip()]


# 检查一组必需进程是否存在，返回缺失项、匹配结果和错误列表。
def _check_required_processes(processes: list[str]) -> dict[str, Any]:
    if not processes:
        return {"required_processes": [], "missing_processes": [], "matched_processes": {}, "errors": []}
    missing: list[str] = []
    errors: list[str] = []
    matched: dict[str, list[str]] = {}
    for process in processes:
        try:
            matches = _list_process_matches(process)
        except Exception as exc:  # noqa: BLE001
            missing.append(process)
            errors.append(str(exc))
            matched[process] = []
            continue
        matched[process] = matches[:5]
        if not matches:
            missing.append(process)
    logger.info(
        "scan process check required=%s missing=%s matched=%s errors=%s",
        list(processes),
        missing,
        matched,
        errors,
    )
    return {"required_processes": list(processes), "missing_processes": missing, "matched_processes": matched, "errors": errors}


# 从 pgrep 输出行中提取 PID。
def _extract_pid_from_process_line(line: str) -> int | None:
    parts = str(line).strip().split(maxsplit=1)
    if not parts:
        return None
    try:
        pid = int(parts[0])
    except ValueError:
        return None
    return pid if pid > 0 else None


# 从匹配到的进程信息中提取去重后的 PID 列表，后续停止扫描时使用。
def _matched_process_pids(matched_processes: dict[str, list[str]]) -> list[int]:
    pids: list[int] = []
    for lines in matched_processes.values():
        for line in lines:
            pid = _extract_pid_from_process_line(line)
            if pid is not None:
                pids.append(pid)
    return list(dict.fromkeys(pids))


# 把命令参数拼成稳定 key，用来追踪同一条启动命令是否已经运行。
def _scan_command_key(argv: list[str]) -> str:
    return "\x00".join(str(part) for part in argv)


# 判断启动命令对应的依赖是否已经满足，避免重复启动相同依赖。
def _command_targets_satisfied_dependency(argv: list[str], required_processes: list[str], missing_processes: list[str]) -> bool:
    command_text = " ".join(str(part).lower() for part in argv)
    missing = {str(process).lower() for process in missing_processes}
    targeted = [str(process).lower() for process in required_processes if str(process).lower() in command_text]
    return bool(targeted) and not any(process in missing for process in targeted)


# 从配置项中提取真正要执行的命令参数列表。
def _launch_command_argv(command: Any) -> list[str]:
    if isinstance(command, dict):
        return [str(part) for part in command.get("command", [])]
    raw = getattr(command, "command", command)
    return [str(part) for part in raw]


# 从配置项中提取该命令负责启动或检查的进程名。
def _launch_command_processes(command: Any) -> list[str]:
    if isinstance(command, dict):
        raw = command.get("processes", [])
    elif isinstance(command, list):
        raw = [str(command[-1])] if command else []
    else:
        raw = getattr(command, "processes", [])
    return [str(process) for process in raw if str(process).strip()]


# 检查系统里是否已经存在与某条启动命令匹配的进程。
def _scan_command_process_matches(argv: list[str]) -> list[str]:
    tokens = [str(part) for part in argv if str(part).strip()]
    if not tokens:
        return []
    pattern = tokens[-1]
    try:
        matches = _list_process_matches(pattern)
    except Exception as exc:  # noqa: BLE001
        logger.warning("scan launch command process check failed argv=%s err=%s", argv, exc)
        return []
    required_tokens = tokens[1:] if tokens[0] == "ros2" else tokens
    filtered = [line for line in matches if all(token in line for token in required_tokens)]
    return filtered[:5]


# 判断当前依赖状态下是否需要执行某条扫描启动命令。
def _scan_launch_command_needed(command: Any, status: dict[str, Any]) -> bool:
    processes = _launch_command_processes(command)
    missing_processes = {str(process) for process in status.get("missing_processes", [])}
    if processes:
        return any(process in missing_processes for process in processes)
    return bool(status.get("missing_processes"))


# 从扫描模式的启动命令配置中汇总所有必需进程。
def _scan_required_processes_from_launch_commands(config: Any) -> list[str]:
    processes: list[str] = []
    for command in getattr(config, "launch_commands", []):
        processes.extend(_launch_command_processes(command))
    return list(dict.fromkeys(processes))


# 真正拉起扫描依赖命令，并记录进程对象，方便后续停止和回收。
def _launch_scan_mode_command(argv: list[str]) -> tuple[bool, str]:
    key = _scan_command_key(argv)
    existing_processes = LAUNCHED_SCAN_COMMANDS.get(key, [])
    live_processes: list[subprocess.Popen[str]] = []
    for process in existing_processes:
        if process.poll() is None:
            live_processes.append(process)
    if live_processes:
        LAUNCHED_SCAN_COMMANDS[key] = live_processes
        pids = [str(process.pid) for process in live_processes]
        logger.info("scan dependency command already running argv=%s pids=%s", argv, pids)
        return True, f"already_running pid={pids[0]}"
    LAUNCHED_SCAN_COMMANDS[key] = []
    try:
        process = subprocess.Popen(
            argv,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
            start_new_session=True,
        )
    except Exception as exc:  # noqa: BLE001
        return False, str(exc)
    LAUNCHED_SCAN_PROCESSES.append(process)
    LAUNCHED_SCAN_COMMANDS.setdefault(key, []).append(process)
    logger.info("started scan dependency command argv=%s pid=%s", argv, process.pid)
    return True, f"pid={process.pid}"


# 等待并回收已启动的扫描依赖进程，同时从跟踪表中移除。
def _reap_scan_process(process: subprocess.Popen[str]) -> None:
    try:
        process.wait()
        if process in LAUNCHED_SCAN_PROCESSES:
            LAUNCHED_SCAN_PROCESSES.remove(process)
        for key, processes in list(LAUNCHED_SCAN_COMMANDS.items()):
            LAUNCHED_SCAN_COMMANDS[key] = [item for item in processes if item is not process]
            if not LAUNCHED_SCAN_COMMANDS[key]:
                LAUNCHED_SCAN_COMMANDS.pop(key, None)
        logger.info("reaped scan dependency process pid=%s", getattr(process, "pid", "?"))
    except Exception as exc:  # noqa: BLE001
        logger.warning("failed to reap scan dependency process pid=%s err=%s", getattr(process, "pid", "?"), exc)


# 在后台线程中回收进程，避免阻塞当前请求处理。
def _reap_scan_process_async(process: subprocess.Popen[str]) -> None:
    threading.Thread(target=_reap_scan_process, args=(process,), daemon=True).start()


# 合并节点依赖和进程依赖的检查结果，形成统一依赖状态。
def _merge_scan_dependency_status(node_status: dict[str, Any], process_status: dict[str, Any]) -> dict[str, Any]:
    return {
        "required_nodes": list(node_status.get("required_nodes", [])),
        "missing_nodes": list(node_status.get("missing_nodes", [])),
        "started_nodes": list(node_status.get("started_nodes", [])),
        "required_processes": list(process_status.get("required_processes", [])),
        "missing_processes": list(process_status.get("missing_processes", [])),
        "started_processes": list(process_status.get("started_processes", [])),
        "matched_processes": copy.deepcopy(process_status.get("matched_processes", {})),
        "tracked_pids": _matched_process_pids(process_status.get("matched_processes", {})),
        "errors": list(node_status.get("errors", [])) + list(process_status.get("errors", [])),
    }


# 获取某个扫描模式当前的依赖状态。
def _scan_dependency_status(config: Any) -> dict[str, Any]:
    node_status = {"required_nodes": [], "missing_nodes": [], "started_nodes": [], "errors": []}
    process_status = _check_required_processes(_scan_required_processes_from_launch_commands(config))
    return _merge_scan_dependency_status(node_status, process_status)


# 确保扫描模式所需依赖已启动；必要时自动执行配置里的启动命令并轮询等待。
def _ensure_scan_mode_dependencies(mode: str) -> dict[str, Any]:
    config = _scan_mode_config(mode)
    status = _scan_dependency_status(config)
    logger.info(
        "scan dependency check mode=%s required_processes=%s missing_processes=%s errors=%s",
        mode,
        status.get("required_processes", []),
        status.get("missing_processes", []),
        status.get("errors", []),
    )
    if not status["missing_processes"]:
        status["started_processes"] = list(status.get("required_processes", []))
        return status
    started_nodes: list[str] = list(status.get("started_nodes", []))
    started_processes: list[str] = list(status.get("started_processes", []))
    errors: list[str] = list(status.get("errors", []))
    launched_or_waiting = False
    for command in config.launch_commands:
        argv = _launch_command_argv(command)
        if not argv:
            continue
        command_processes = _launch_command_processes(command)
        if command_processes and not _scan_launch_command_needed(command, status):
            logger.info(
                "skip scan dependency command; configured processes already running mode=%s command=%s processes=%s missing_processes=%s",
                mode,
                argv,
                command_processes,
                status.get("missing_processes", []),
            )
            continue
        if _command_targets_satisfied_dependency(
            argv,
            _scan_required_processes_from_launch_commands(config),
            list(status.get("missing_processes", [])),
        ):
            logger.info(
                "skip scan dependency command; targeted process already running mode=%s command=%s missing_processes=%s",
                mode,
                argv,
                status.get("missing_processes", []),
            )
            continue
        existing_command_matches = _scan_command_process_matches(argv)
        if existing_command_matches:
            logger.info(
                "scan dependency command already exists in system mode=%s command=%s matches=%s",
                mode,
                argv,
                existing_command_matches,
            )
            launched_or_waiting = True
            continue
        ok, detail = _launch_scan_mode_command(argv)
        if not ok:
            errors.append(detail or f"failed to launch {argv}")
            logger.warning("scan dependency launch failed mode=%s command=%s detail=%s", mode, argv, detail)
            continue
        launched_or_waiting = True
        logger.info("started scan dependency launch mode=%s command=%s detail=%s processes=%s", mode, argv, detail, command_processes)
    if launched_or_waiting:
        logger.info("waiting for scan dependencies mode=%s", mode)
        for attempt in range(1, SCAN_DEPENDENCY_POLL_ATTEMPTS + 1):
            time.sleep(SCAN_DEPENDENCY_POLL_INTERVAL_SEC)
            status = _scan_dependency_status(config)
            logger.info(
                "scan dependency poll mode=%s attempt=%s/%s missing_processes=%s errors=%s",
                mode,
                attempt,
                SCAN_DEPENDENCY_POLL_ATTEMPTS,
                status.get("missing_processes", []),
                status.get("errors", []),
            )
            if not status["missing_processes"]:
                break
        started_nodes = list(dict.fromkeys(started_nodes + list(status.get("required_nodes", []))))
        started_processes = list(dict.fromkeys(started_processes + list(status.get("required_processes", []))))
        if not status["missing_processes"]:
            status["started_nodes"] = started_nodes
            status["started_processes"] = started_processes
            status["errors"] = errors
            status["tracked_pids"] = _matched_process_pids(status.get("matched_processes", {}))
            logger.info(
                "scan dependency launch success mode=%s started_processes=%s",
                mode,
                status["started_processes"],
            )
            return status
    status["started_nodes"] = started_nodes
    status["started_processes"] = started_processes
    status["errors"] = errors
    status["tracked_pids"] = _matched_process_pids(status.get("matched_processes", {}))
    return status


# 按 PID 停止已追踪到的扫描依赖进程。
def _stop_tracked_scan_pids(pids: list[int]) -> dict[str, Any]:
    stopped_pids: list[int] = []
    errors: list[str] = []
    for pid in dict.fromkeys(int(pid) for pid in pids if int(pid) > 0):
        logger.info("stopping tracked scan dependency pid=%s signal=SIGINT", pid)
        try:
            os.kill(pid, signal.SIGINT)
            stopped_pids.append(pid)
        except ProcessLookupError:
            logger.info("tracked scan dependency pid already exited pid=%s", pid)
        except Exception as exc:  # noqa: BLE001
            errors.append(f"pid={pid}: {exc}")
    for pid in stopped_pids:
        if _wait_for_pid_exit(pid):
            logger.info("tracked scan dependency pid exited pid=%s", pid)
        else:
            logger.warning("tracked scan dependency pid still running after SIGINT pid=%s", pid)
            errors.append(f"pid={pid}: still running after SIGINT")
    return {"stopped_pids": stopped_pids, "errors": errors}


# 等待指定 PID 退出，并尝试回收子进程。
def _wait_for_pid_exit(pid: int) -> bool:
    proc_path = Path(f"/proc/{pid}")
    for _ in range(SCAN_PROCESS_STOP_POLL_ATTEMPTS):
        if _reap_child_pid(pid):
            return True
        if not proc_path.exists():
            return True
        time.sleep(SCAN_PROCESS_STOP_POLL_INTERVAL_SEC)
    if _reap_child_pid(pid):
        return True
    return not proc_path.exists()


# 非阻塞回收指定子进程 PID。
def _reap_child_pid(pid: int) -> bool:
    try:
        reaped_pid, _status = os.waitpid(pid, os.WNOHANG)
    except ChildProcessError:
        return False
    except OSError:
        return False
    if reaped_pid == pid:
        logger.info("reaped scan dependency child pid=%s", pid)
        return True
    return False


# 停止所有由服务端启动或跟踪的扫描依赖进程。
def _stop_launched_scan_processes() -> dict[str, Any]:
    stopped_pids: list[int] = []
    errors: list[str] = []
    remaining: list[subprocess.Popen[str]] = []
    for process in LAUNCHED_SCAN_PROCESSES:
        try:
            logger.info("stopping scan dependency process pid=%s signal=SIGINT", process.pid)
            os.killpg(process.pid, signal.SIGINT)
            stopped_pids.append(int(process.pid))
            try:
                process.wait(timeout=3.0 if process.poll() is None else 0.1)
            except subprocess.TimeoutExpired:
                errors.append(f"pid={process.pid}: did not exit after SIGINT")
                remaining.append(process)
                _reap_scan_process_async(process)
        except ProcessLookupError:
            try:
                process.wait(timeout=0.1)
            except Exception:  # noqa: BLE001
                pass
            logger.info("scan dependency process group already exited pid=%s", getattr(process, "pid", "?"))
        except Exception as exc:  # noqa: BLE001
            errors.append(f"pid={getattr(process, 'pid', '?')}: {exc}")
            remaining.append(process)
    LAUNCHED_SCAN_PROCESSES[:] = remaining
    for key, processes in list(LAUNCHED_SCAN_COMMANDS.items()):
        LAUNCHED_SCAN_COMMANDS[key] = [process for process in processes if process in remaining]
        if not LAUNCHED_SCAN_COMMANDS[key]:
            LAUNCHED_SCAN_COMMANDS.pop(key, None)
    if stopped_pids:
        logger.info("stopped scan dependency processes pids=%s", stopped_pids)
    tracked_status = _stop_tracked_scan_pids(list(SCAN_SESSION.get("dependency_status", {}).get("tracked_pids", [])))
    stopped_pids = list(dict.fromkeys(stopped_pids + tracked_status["stopped_pids"]))
    errors.extend(tracked_status["errors"])
    pattern_status = _terminate_process_patterns(list(SCAN_SESSION.get("dependency_status", {}).get("started_processes", [])))
    errors.extend(pattern_status["errors"])
    if errors:
        logger.warning("failed to stop scan dependency processes errors=%s", errors)
    return {"stopped_pids": stopped_pids, "stopped_patterns": pattern_status["stopped_patterns"], "errors": errors}


# 按进程名模式批量发送 SIGINT，用于清理扫描依赖。
def _terminate_process_patterns(patterns: list[str]) -> dict[str, Any]:
    stopped_patterns: list[str] = []
    errors: list[str] = []
    for pattern in dict.fromkeys(str(pattern).strip() for pattern in patterns if str(pattern).strip()):
        logger.info("stopping scan dependency process pattern=%s signal=SIGINT", pattern)
        term = subprocess.run(["pkill", "-INT", "-f", pattern], check=False, capture_output=True, text=True)
        if term.returncode not in {0, 1}:
            detail = str(term.stderr or term.stdout or "").strip()
            errors.append(f"{pattern}: {detail or 'pkill INT failed'}")
            continue
        if term.returncode == 0:
            stopped_patterns.append(pattern)
        remaining: list[str] = []
        for _ in range(SCAN_PROCESS_STOP_POLL_ATTEMPTS):
            remaining = _list_process_matches(pattern)
            if not remaining:
                break
            time.sleep(SCAN_PROCESS_STOP_POLL_INTERVAL_SEC)
        if remaining:
            logger.warning("scan dependency process pattern still running after SIGINT pattern=%s matches=%s", pattern, remaining[:5])
            errors.append(f"{pattern}: still running after SIGINT")
    return {"stopped_patterns": stopped_patterns, "errors": errors}


# 依赖启动后等待建图前置条件就绪，给 ROS 节点一些初始化时间。
def _wait_for_mapping_prereq_after_dependency_start(dependency_status: dict[str, Any]) -> dict[str, Any]:
    summary = _mapping_prereq_summary()
    if summary["ready"] or not (dependency_status.get("started_nodes") or dependency_status.get("started_processes")):
        return summary
    for _ in range(SCAN_MAPPING_PREREQ_POLL_ATTEMPTS):
        time.sleep(SCAN_MAPPING_PREREQ_POLL_INTERVAL_SEC)
        summary = _mapping_prereq_summary()
        if summary["ready"]:
            return summary
    return summary


# 根据扫描模式配置找到 3D PCD 输出路径。
def _pcd_output_path_for_mode(mode: str) -> Path | None:
    normalized = _normalize_scan_mode(mode)
    if normalized != "3d":
        return None
    raw = str(_scan_mode_config(normalized).pcd_output_path or "").strip()
    return Path(raw) if raw else None


# 汇总 WebSocket 队列和客户端连接容量相关的配置与运行时指标。
def _server_capacity_summary() -> dict[str, Any]:
    return {
        "limits": {
            "ws_queue_size": CONFIG.ws_queue_size * 2,
            "queue_near_capacity_ratio": QUEUE_NEAR_CAPACITY_RATIO,
            "topic_min_interval_sec": TOPIC_MIN_INTERVAL_SEC,
            "client_idle_timeout_sec": CLIENT_IDLE_TIMEOUT_SEC,
        },
        "runtime": {
            "ws_overflow_total": int(SERVER_RUNTIME["ws_overflow_total"]),
            "ws_near_capacity_total": int(SERVER_RUNTIME["ws_near_capacity_total"]),
            "ws_last_warn_at": float(SERVER_RUNTIME["ws_last_warn_at"]),
            "active_ws_connections_peak": int(SERVER_RUNTIME["active_ws_connections_peak"]),
            "forced_disconnect_total": int(SERVER_RUNTIME["forced_disconnect_total"]),
        },
    }


# 生成当前扫描会话摘要，包括耗时、帧数、原始点数等。
def _scan_summary() -> dict[str, Any]:
    started_at = float(SCAN_SESSION["started_at"])
    stopped_at = float(SCAN_SESSION["stopped_at"])
    if started_at <= 0:
        elapsed = 0.0
    elif stopped_at > started_at:
        elapsed = stopped_at - started_at
    else:
        elapsed = time.time() - started_at

    return {
        "active": bool(SCAN_SESSION["active"]),
        "started_at": started_at,
        "stopped_at": stopped_at,
        "elapsed_sec": round(max(0.0, elapsed), 2),
        "voxel_size": float(SCAN_SESSION["voxel_size"]),
        "front_frames": int(SCAN_SESSION["front_frames"]),
        "rear_frames": int(SCAN_SESSION["rear_frames"]),
        "raw_points": int(SCAN_SESSION["raw_points"]),
        "accumulated_points": 0,
    }


# 获取 ROS 桥接层诊断信息；ROS 不可用时返回空字典。
def _ros_diag() -> dict[str, Any]:
    if ros.enabled and ros.bridge is not None:
        return ros.bridge.diagnostics()
    return {}


# 根据 WebSocket 客户端和 TopicBus 压力生成网络诊断摘要。
def _network_diag_summary() -> dict[str, Any]:
    topic_stats = bus.stats()
    warnings: list[str] = []
    checks = {
        "ws_clients": {"ok": len(ws_clients) > 0, "value": len(ws_clients)},
        "topic_bus": {"ok": True, "topics": []},
    }
    if len(ws_clients) == 0:
        warnings.append("no websocket clients connected")
    degraded_topics: list[str] = []
    for topic, stat in topic_stats.items():
        if float(stat.get("drop_rate", 0.0)) > 0.05 or float(stat.get("peak_fill_ratio", 0.0)) > 0.9:
            degraded_topics.append(topic)
    if degraded_topics:
        warnings.append(f"topic bus pressure on {', '.join(sorted(degraded_topics))}")
        checks["topic_bus"]["ok"] = False
    checks["topic_bus"]["topics"] = degraded_topics
    return {"ok": not warnings, "warnings": warnings, "checks": checks}


# 综合 ROS 与网络状态，判断当前是否满足建图前置条件。
def _mapping_prereq_summary() -> dict[str, Any]:
    if ros.enabled and ros.bridge is not None and hasattr(ros.bridge, "mapping_prerequisites"):
        summary = dict(ros.bridge.mapping_prerequisites())
    elif ros.enabled:
        summary = {
            "ready": False,
            "severity": "error",
            "blockers": ["ros bridge does not expose mapping prerequisites"],
            "warnings": [],
            "checks": {"ros_runtime": {"ok": False}},
        }
    else:
        summary = {
            "ready": False,
            "severity": "error",
            "blockers": ["ros bridge unavailable"],
            "warnings": ["mapping data source unavailable"],
            "checks": {"data_source": {"ok": False, "source": "none"}},
        }

    network = _network_diag_summary()
    checks = dict(summary.get("checks", {}))
    checks["network"] = network["checks"]
    warnings = list(summary.get("warnings", []))
    warnings.extend(item for item in network["warnings"] if item not in warnings)
    blockers = list(summary.get("blockers", []))
    severity = "error" if blockers else "warn" if warnings else "ok"
    ready = bool(summary.get("ready", False)) and not blockers
    return {
        "ready": ready,
        "severity": severity,
        "blockers": blockers,
        "warnings": warnings,
        "checks": checks,
    }


# 判断当前地图数据来源，便于诊断接口展示。
def _current_map_source() -> str:
    if latest_occupancy_grid:
        return "occupancy_grid"
    return "unavailable"


# 把地图 payload 标准化为栅格结构；这里只接受完整 occupancy_grid。
def _occupancy_payload_to_grid(payload: dict[str, Any]) -> dict[str, Any] | None:
    data = [int(value) for value in payload.get("data", [])] if isinstance(payload.get("data"), list) else []
    width = int(payload.get("width", 0) or 0)
    height = int(payload.get("height", 0) or 0)
    if data and width > 0 and height > 0 and len(data) == width * height:
        origin = payload.get("origin") if isinstance(payload.get("origin"), dict) else {}
        return {
            "width": width,
            "height": height,
            "resolution": max(0.02, float(payload.get("resolution", 0.05) or 0.05)),
            "origin": {
                "x": float(origin.get("x", 0.0)),
                "y": float(origin.get("y", 0.0)),
                "yaw": float(origin.get("yaw", 0.0) or 0.0),
            },
            "data": [100 if value >= 50 else 0 if value >= 0 else -1 for value in data],
        }
    return None


# 根据指令更新时间计算当前有效控制目标；超时则触发安全停车。
def _effective_control_target(now: float | None = None) -> tuple[float, float, bool, bool]:
    now = time.time() if now is None else float(now)
    updated_at = float(CONTROL_TARGET.get("updated_at", 0.0) or 0.0)
    velocity = float(CONTROL_TARGET.get("velocity", 0.0) or 0.0)
    yaw_rate = float(CONTROL_TARGET.get("yaw_rate", 0.0) or 0.0)
    if updated_at <= 0.0:
        return 0.0, 0.0, False, False
    if now - updated_at <= CONTROL_TARGET_HOLD_SEC:
        return velocity, yaw_rate, False, True
    return 0.0, 0.0, bool(velocity or yaw_rate), False


# 输出控制目标与平滑控制模块的健康状态。
def _control_target_health(now: float | None = None) -> dict[str, Any]:
    now = time.time() if now is None else float(now)
    updated_at = float(CONTROL_TARGET.get("updated_at", 0.0) or 0.0)
    age_sec = max(0.0, now - updated_at) if updated_at > 0.0 else None
    velocity, yaw_rate, stale, should_publish = _effective_control_target(now=now)
    return {
        "velocity": float(CONTROL_TARGET.get("velocity", 0.0) or 0.0),
        "yaw_rate": float(CONTROL_TARGET.get("yaw_rate", 0.0) or 0.0),
        "effective_velocity": velocity,
        "effective_yaw_rate": yaw_rate,
        "publishing": should_publish,
        "updated_at": updated_at,
        "age_sec": round(age_sec, 3) if age_sec is not None else None,
        "stale": stale,
        "publish_interval_sec": CONTROL_PUBLISH_INTERVAL_SEC,
        "hold_sec": CONTROL_TARGET_HOLD_SEC,
        "last_publish_source": str(CONTROL_RUNTIME["last_publish_source"]),
        "last_zero_source": str(CONTROL_RUNTIME["last_zero_source"]),
        "last_zero_at": float(CONTROL_RUNTIME["last_zero_at"])
    }



# 按照最大加速度/减速度限制，让当前值平滑逼近目标值。
def _step_towards(current: float, target: float, max_accel: float, max_decel: float, dt: float) -> float:
    current = float(current)
    target = float(target)
    if target > current:
        delta = min(target - current, max_accel * dt)
        return current + delta
    delta = min(current - target, max_decel * dt)
    return current - delta


# 对线速度和角速度做平滑处理，避免控制输出突变。
# 这里把加速度单独压低，重点解决“起步太猛”，但保持停车减速能力不变。
def _smooth_control_command(target_velocity: float, target_yaw_rate: float, now: float | None = None, forced_stop: bool = False) -> tuple[float, float]:
    current_time = time.time() if now is None else float(now)
    updated_at = float(CONTROL_OUTPUT_STATE.get("updated_at", 0.0) or 0.0)
    dt = CONTROL_PUBLISH_INTERVAL_SEC if updated_at <= 0.0 else max(0.001, min(1.0, current_time - updated_at))
    linear_accel = CONTROL_LINEAR_ACCEL_MPS2
    linear_decel = CONTROL_LINEAR_EMERGENCY_DECEL_MPS2 if forced_stop else CONTROL_LINEAR_DECEL_MPS2
    angular_accel = CONTROL_ANGULAR_ACCEL_RPS2
    angular_decel = CONTROL_ANGULAR_EMERGENCY_DECEL_RPS2 if forced_stop else CONTROL_ANGULAR_DECEL_RPS2
    next_velocity = _step_towards(float(CONTROL_OUTPUT_STATE.get("velocity", 0.0) or 0.0), float(target_velocity), linear_accel, linear_decel, dt)
    next_yaw_rate = _step_towards(float(CONTROL_OUTPUT_STATE.get("yaw_rate", 0.0) or 0.0), float(target_yaw_rate), angular_accel, angular_decel, dt)
    CONTROL_OUTPUT_STATE["velocity"] = float(next_velocity)
    CONTROL_OUTPUT_STATE["yaw_rate"] = float(next_yaw_rate)
    CONTROL_OUTPUT_STATE["updated_at"] = current_time
    return float(next_velocity), float(next_yaw_rate)


# 记录控制指令发布来源，并在速度或来源变化时输出日志。
def _record_control_publish_source(source: str, velocity: float, yaw_rate: float) -> None:
    CONTROL_RUNTIME["last_publish_source"] = source
    velocity = float(velocity)
    yaw_rate = float(yaw_rate)
    last_velocity = CONTROL_RUNTIME.get("last_logged_velocity")
    last_yaw_rate = CONTROL_RUNTIME.get("last_logged_yaw_rate")
    velocity_changed = (
        last_velocity is None
        or last_yaw_rate is None
        or abs(float(last_velocity) - velocity) > 1e-9
        or abs(float(last_yaw_rate) - yaw_rate) > 1e-9
    )
    source_changed = CONTROL_RUNTIME.get("last_logged_source") != source
    changed = (
        velocity_changed
        or (source_changed and source != "target_hold")
    )
    zero = abs(velocity) <= 1e-9 and abs(yaw_rate) <= 1e-9
    if changed:
        logger.info(
            "control cmd publish source=%s velocity=%.3f yaw_rate=%.3f zero=%s",
            source,
            velocity,
            yaw_rate,
            zero,
        )
        CONTROL_RUNTIME["last_logged_source"] = source
        CONTROL_RUNTIME["last_logged_velocity"] = velocity
        CONTROL_RUNTIME["last_logged_yaw_rate"] = yaw_rate
    if abs(float(velocity)) <= 1e-9 and abs(float(yaw_rate)) <= 1e-9:
        CONTROL_RUNTIME["last_zero_source"] = source
        CONTROL_RUNTIME["last_zero_at"] = time.time()


# 发布最终控制命令：先做平滑，再发给 ROS。
def _publish_control_command(velocity: float, yaw_rate: float, source: str, now: float | None = None) -> None:
    if ros.bridge is None:
        return
    smoothed_velocity, smoothed_yaw_rate = _smooth_control_command(
        velocity,
        yaw_rate,
        now=now,
        forced_stop=False,
    )
    _record_control_publish_source(source, smoothed_velocity, smoothed_yaw_rate)
    ros.bridge.publish_cmd_vel(smoothed_velocity, smoothed_yaw_rate)


# 后台控制发布循环：持续维持目标速度，并在目标超时后发送停车 burst。
async def _control_publisher_loop() -> None:
    global CONTROL_STOP_BURST_REMAINING
    stale_logged = False
    while True:
        try:
            velocity, yaw_rate, stale, should_publish = _effective_control_target()
            if stale and not stale_logged:
                logger.warning("control target stale; publishing stop for safety")
                stale_logged = True
                CONTROL_STOP_BURST_REMAINING = CONTROL_STOP_BURST_TICKS
                CONTROL_TARGET["velocity"] = 0.0
                CONTROL_TARGET["yaw_rate"] = 0.0
                CONTROL_TARGET["updated_at"] = 0.0
            elif not stale:
                stale_logged = False
            source = "target_stale_stop" if stale else "target_hold"
            if CONTROL_STOP_BURST_REMAINING > 0:
                velocity, yaw_rate = 0.0, 0.0
                should_publish = True
                CONTROL_STOP_BURST_REMAINING -= 1
                source = "target_stale_stop_burst"
            if should_publish:
                _publish_control_command(velocity, yaw_rate, source)
            await asyncio.sleep(CONTROL_PUBLISH_INTERVAL_SEC)
        except asyncio.CancelledError:
            raise
        except Exception:  # noqa: BLE001
            logger.exception("control publisher loop failed")
            await asyncio.sleep(CONTROL_PUBLISH_INTERVAL_SEC)


# 服务启动钩子：检测 ROS 并启动控制后台任务。
@app.on_event("startup")
async def startup() -> None:
    global control_task, ros

    loop = asyncio.get_running_loop()
    ros = detect_ros(bus=bus, loop=loop, config=CONFIG.ros)

    logger.info("server startup ros_enabled=%s reason=%s", ros.enabled, ros.reason)

    control_task = asyncio.create_task(_control_publisher_loop())


# 服务关闭钩子：取消后台任务并停止 ROS 桥接。
@app.on_event("shutdown")
async def shutdown() -> None:
    global control_task
    if control_task is not None:
        control_task.cancel()
        try:
            await control_task
        except asyncio.CancelledError:
            pass
        control_task = None

    if ros.bridge is not None:
        ros.bridge.stop()



# 健康检查接口：返回 ROS、扫描、PCD、控制、地图来源和容量等综合状态。
@app.get("/health")
async def health() -> dict:
    mapping_prereq = _mapping_prereq_summary()
    return {
        "ok": True,
        "ros_enabled": ros.enabled,
        "ros_reason": ros.reason,
        "scan_active": bool(SCAN_SESSION["active"]),
        "scan_mode": str(SCAN_SESSION["mode"]),
        "ws_clients": len(ws_clients),
        "topics": STREAM_TOPICS,
        "scan_summary": _scan_summary(),
        "dependency_status": copy.deepcopy(SCAN_SESSION["dependency_status"]),
        "pcd_transfer_state": str(SCAN_SESSION["pcd_transfer_state"]),
        "pcd_metadata": copy.deepcopy(SCAN_SESSION["pcd_file"]),
        "ros_diag": _ros_diag(),
        "mapping_ready": bool(mapping_prereq["ready"]),
        "mapping_status": mapping_prereq["severity"],
        "mapping_blockers": list(mapping_prereq["blockers"]),
        "mapping_warnings": list(mapping_prereq["warnings"]),
        "control_target": _control_target_health(),
        "map_source": _current_map_source(),
        "frames": {
            "odom": CONFIG.ros.topics.odom_frame,
            "base": CONFIG.ros.topics.robot_base_frame,
        },
        "capacity": _server_capacity_summary(),
    }


# 建图前置条件诊断接口：便于前端展示为什么当前不能开始建图。
@app.get("/diag/mapping_prereq")
async def diag_mapping_prereq() -> dict:
    return {
        "ok": True,
        "ros_enabled": ros.enabled,
        "mapping_prereq": _mapping_prereq_summary(),
        "ros_diag": _ros_diag(),
    }


# 实时数据流诊断接口：返回 TopicBus 统计、序号、扫描摘要和容量信息。
@app.get("/diag/stream_stats")
async def diag_stream_stats() -> dict:
    return {
        "ok": True,
        "ws_clients": len(ws_clients),
        "topic_stats": bus.stats(),
        "seq_by_topic": dict(seq_by_topic),
        "server_time_ms": int(time.time() * 1000),
        "scan_summary": _scan_summary(),
        "ros_diag": _ros_diag(),
        "capacity": _server_capacity_summary(),
    }


# 开始扫描接口：校验模式、启动依赖、等待建图条件，然后切换扫描状态。
@app.post("/scan/start")
async def start_scan(req: StartScanRequest | None = None) -> dict:
    global SCAN_START_CANCEL_SEQ

    mode = _normalize_scan_mode(req.mode if req is not None else "3d")
    logger.info(
        "scan start requested requested_mode=%s session_mode=%s active=%s",
        req.mode if req is not None else None,
        str(SCAN_SESSION.get("mode", "2d")),
        bool(SCAN_SESSION["active"]),
    )
    # 扫描模式非法时直接拒绝，避免进入依赖启动流程。
    if mode is None:
        return {
            "ok": False,
            "reason": "invalid_scan_mode",
            "scan_active": False,
        }
    # 同一时间只允许一个扫描会话，防止重复启动外部依赖。
    if bool(SCAN_SESSION["active"]):
        return {
            "ok": False,
            "reason": "scan_already_active",
            "scan_active": True,
            "scan_mode": str(SCAN_SESSION["mode"]),
        }
    # 使用非阻塞锁保护启动流程，避免多个 /scan/start 并发进入。
    if not SCAN_START_LOCK.acquire(blocking=False):
        logger.info("scan start ignored because another start is already in progress mode=%s", mode)
        return {
            "ok": False,
            "reason": "scan_start_in_progress",
            "scan_active": False,
            "scan_mode": mode,
        }
    start_cancel_seq = int(SCAN_START_CANCEL_SEQ)
    try:
        # 先确保模式依赖进程可用，例如建图或点云处理相关 ROS 节点。
        dependency_status = _ensure_scan_mode_dependencies(mode)
        SCAN_SESSION["dependency_status"] = copy.deepcopy(dependency_status)
        if dependency_status.get("missing_processes") or dependency_status["errors"]:
            return {
                "ok": False,
                "reason": "node_start_failed",
                "scan_active": False,
                "scan_mode": mode,
                "dependency_status": dependency_status,
                "ros_enabled": ros.enabled,
            }
        if int(SCAN_START_CANCEL_SEQ) != start_cancel_seq:
            logger.info("scan start cancelled after dependency start mode=%s", mode)
            return {
                "ok": False,
                "reason": "scan_start_cancelled",
                "scan_active": False,
                "scan_mode": mode,
                "dependency_status": dependency_status,
                "ros_enabled": ros.enabled,
            }
        # 依赖启动后继续检查建图输入、网络和数据源是否真的就绪。
        mapping_prereq = _wait_for_mapping_prereq_after_dependency_start(dependency_status)
        if not mapping_prereq["ready"]:
            logger.warning("scan start rejected blockers=%s warnings=%s", mapping_prereq["blockers"], mapping_prereq["warnings"])
            return {
                "ok": False,
                "reason": "mapping_prereq_failed",
                "scan_active": False,
                "mapping_prereq": mapping_prereq,
                "ros_enabled": ros.enabled,
            }
        if int(SCAN_START_CANCEL_SEQ) != start_cancel_seq:
            logger.info("scan start cancelled after mapping prereq mode=%s", mode)
            return {
                "ok": False,
                "reason": "scan_start_cancelled",
                "scan_active": False,
                "scan_mode": mode,
                "dependency_status": dependency_status,
                "ros_enabled": ros.enabled,
            }
        # 所有前置条件通过后，再重置并开启新的扫描会话。
        _reset_scan_session()
        SCAN_SESSION["mode"] = mode
        SCAN_SESSION["dependency_status"] = copy.deepcopy(dependency_status)
        SCAN_SESSION["active"] = True
        SCAN_SESSION["started_at"] = time.time()
        if ros.enabled and ros.bridge is not None:
            ros.bridge.set_scan_active(True)
        logger.info("scan started mode=%s active=%s", mode, bool(SCAN_SESSION["active"]))
        return {
            "ok": True,
            "scan_active": True,
            "scan_mode": mode,
            "scan_summary": _scan_summary(),
            "dependency_status": dependency_status,
            "ros_enabled": ros.enabled,
        }
    finally:
        SCAN_START_LOCK.release()


# 停止扫描接口：取消正在进行的启动流程，停止依赖进程，并关闭 ROS 扫描标志。
@app.post("/scan/stop")
async def stop_scan(req: StopScanRequest | None = None) -> dict:
    global SCAN_START_CANCEL_SEQ

    mode = _normalize_scan_mode(req.mode if req is not None else str(SCAN_SESSION.get("mode", "2d")))
    logger.info(
        "scan stop requested requested_mode=%s session_mode=%s active=%s",
        req.mode if req is not None else None,
        str(SCAN_SESSION.get("mode", "2d")),
        bool(SCAN_SESSION["active"]),
    )
    if mode is None:
        return {
            "ok": False,
            "reason": "invalid_scan_mode",
            "scan_active": bool(SCAN_SESSION["active"]),
        }
    # 即使当前没有 active 扫描，也递增取消序号并尝试清理可能残留的依赖进程。
    if not bool(SCAN_SESSION["active"]):
        SCAN_START_CANCEL_SEQ += 1
        process_stop_status = _stop_launched_scan_processes()
        logger.info("scan stop requested while inactive process_stop_status=%s", process_stop_status)
        return {
            "ok": False,
            "reason": "scan_not_active",
            "scan_active": False,
            "scan_mode": mode,
            "process_stop_status": process_stop_status,
        }
    SCAN_START_CANCEL_SEQ += 1
    # 标记会话停止，并记录停止时间，供 scan_summary 计算耗时。
    SCAN_SESSION["active"] = False
    SCAN_SESSION["stopped_at"] = time.time()
    process_stop_status = _stop_launched_scan_processes()
    if ros.enabled and ros.bridge is not None:
        ros.bridge.set_scan_active(False)
    logger.info("scan stopped session_mode=%s active=%s", str(SCAN_SESSION.get("mode", "2d")), bool(SCAN_SESSION["active"]))
    if str(SCAN_SESSION["mode"]) == "3d":
        SCAN_SESSION["pcd_transfer_state"] = "idle"
        SCAN_SESSION["pcd_file"] = None
    return {
        "ok": True,
        "scan_active": False,
        "scan_mode": str(SCAN_SESSION["mode"]),
        "scan_summary": _scan_summary(),
        "ros_enabled": ros.enabled,
        "process_stop_status": process_stop_status,
    }


# 下载 3D 扫描 PCD 接口：先降采样再返回，避免直接传输超大原始点云。
@app.get("/scan/pcd", response_model=None)
async def download_scan_pcd(voxel_size: float =PCD_DEFAULT_VOXEL_SIZE):
    logger.info(
        "scan pcd request session_mode=%s active=%s transfer_state=%s voxel_size=%.3f",
        str(SCAN_SESSION.get("mode", "2d")),
        bool(SCAN_SESSION.get("active", False)),
        str(SCAN_SESSION.get("pcd_transfer_state", "idle")),
        float(voxel_size),
    )

    # 找到当前 3D 扫描生成的原始 PCD 文件。
    pcd_path, error = _scan_pcd_path()
    if error is not None or pcd_path is None:
        logger.warning(
            "scan pcd request rejected session_mode=%s active=%s reason=%s error=%s",
            str(SCAN_SESSION.get("mode", "2d")),
            bool(SCAN_SESSION.get("active", False)),
            error["reason"],
            error.get("error", ""),
        )
        return JSONResponse(
            {
                "ok": False,
                "reason": error["reason"],
                "error": error.get("error", ""),
                "scan_mode": str(SCAN_SESSION.get("mode", "2d")),
            },
            status_code=int(error.get("status_code", 400)),
        )

    # 先 stat 原始文件，确认文件可读，也用于 response header 记录源文件大小。
    try:
        source_size = pcd_path.stat().st_size
    except OSError as exc:
        SCAN_SESSION["pcd_transfer_state"] = "error"
        return JSONResponse(
            {
                "ok": False,
                "reason": "pcd_stat_failed",
                "error": str(exc),
                "scan_mode": str(SCAN_SESSION.get("mode", "2d")),
            },
            status_code=500,
        )

    try:
        # 标记当前正在降采样，health 接口里可以看到状态。
        SCAN_SESSION["pcd_transfer_state"] = "downsampling"

        # 生成或复用降采样缓存文件。
        # 注意：这里不会把原始全量 PCD 返回给 client。
        downsampled_path, downsample_meta = downsample_pcd_to_cache(pcd_path, float(voxel_size))
        downsampled_size = downsampled_path.stat().st_size

    except Exception as exc:  # noqa: BLE001
        SCAN_SESSION["pcd_transfer_state"] = "error"
        logger.exception(
            "scan pcd downsample failed path=%s voxel_size=%.3f",
            pcd_path,
            float(voxel_size),
        )

        # 降采样失败时直接返回错误。
        # 不要 fallback 返回原始 pcd，否则还是会把全量文件发给 client。
        return JSONResponse(
            {
                "ok": False,
                "reason": "pcd_downsample_failed",
                "error": str(exc),
                "scan_mode": str(SCAN_SESSION.get("mode", "2d")),
                "source_pcd": {
                    "name": pcd_path.name,
                    "size": int(source_size),
                },
            },
            status_code=500,
        )

    # 更新 session 状态。
    SCAN_SESSION["pcd_transfer_state"] = "ready"
    SCAN_SESSION["pcd_file"] = {
        "name": downsampled_path.name,
        "size": int(downsampled_size),
        "source_name": pcd_path.name,
        "source_size": int(source_size),
        "voxel_size": float(voxel_size),
        "downsampled": True,
    }

    logger.info(
        "scan pcd download ready source=%s source_size=%s output=%s output_size=%s meta=%s",
        pcd_path.name,
        source_size,
        downsampled_path.name,
        downsampled_size,
        downsample_meta,
    )

    # 返回的是降采样后的 PCD，不是原始全量 PCD。
    return FileResponse(
        downsampled_path,
        media_type="application/octet-stream",
        filename=downsampled_path.name,
        headers={
            "Content-Length": str(downsampled_size),

            # 返回给 client 的实际文件信息。
            "X-Scan-PCD-Name": downsampled_path.name,
            "X-Scan-PCD-Size": str(downsampled_size),

            # 原始 PCD 信息，方便前端展示压缩比例。
            "X-Scan-PCD-Source-Name": pcd_path.name,
            "X-Scan-PCD-Source-Size": str(source_size),

            # 明确告诉 client 这是降采样后的文件。
            "X-Scan-PCD-Downsampled": "true",
            "X-Scan-PCD-Voxel-Size": f"{float(voxel_size):.3f}",
        },
    )


# 重置扫描接口：清空扫描会话统计与传输状态。
@app.post("/scan/reset")
async def reset_scan() -> dict:
    _reset_scan_session()
    return {"ok": True, "scan_summary": _scan_summary()}


# 短时移动接口：执行一段持续时间的速度命令，到时自动停车。
@app.post("/control/move")
async def move(cmd: MoveCommand) -> dict:
    if not ros.enabled or ros.bridge is None:
        return {"ok": False, "msg": "ros bridge unavailable"}

    # 直接向 cmd_vel 发布；用 command_seq 防止旧的 move 延时停车覆盖新命令。
    global motion_command_seq
    motion_command_seq += 1
    command_seq = motion_command_seq
    CONTROL_TARGET["velocity"] = float(cmd.velocity)
    CONTROL_TARGET["yaw_rate"] = float(cmd.yaw_rate)
    CONTROL_TARGET["updated_at"] = time.time()
    _publish_control_command(float(cmd.velocity), float(cmd.yaw_rate), "api_move")
    await asyncio.sleep(cmd.duration)
    if command_seq == motion_command_seq:
        CONTROL_TARGET["velocity"] = 0.0
        CONTROL_TARGET["yaw_rate"] = 0.0
        CONTROL_TARGET["updated_at"] = time.time()
        _publish_control_command(0.0, 0.0, "api_move_stop")
    return {"ok": True, "msg": "ros cmd_vel applied", "state": {"pose": ros.bridge.latest_pose()}}


# 设置持续控制目标接口：由后台发布循环保持目标速度，超时后自动停车。
@app.post("/control/target")
async def set_control_target(cmd: ControlTargetRequest) -> dict:
    CONTROL_TARGET["velocity"] = float(cmd.velocity)
    CONTROL_TARGET["yaw_rate"] = float(cmd.yaw_rate)
    CONTROL_TARGET["updated_at"] = time.time()
    if ros.enabled and ros.bridge is not None:
        _publish_control_command(float(cmd.velocity), float(cmd.yaw_rate), "api_target")
        return {"ok": True, "msg": "control target applied", "state": {"pose": ros.bridge.latest_pose()}}
    return {"ok": False, "msg": "ros bridge unavailable", "state": {}}


# 立即停车接口：更新控制序号并发布零速度命令。
@app.post("/control/stop")
async def stop() -> dict:
    global motion_command_seq
    motion_command_seq += 1
    CONTROL_TARGET["velocity"] = 0.0
    CONTROL_TARGET["yaw_rate"] = 0.0
    CONTROL_TARGET["updated_at"] = time.time()
    _publish_control_command(0.0, 0.0, "api_stop")
    return {"ok": True}

# 实时数据 WebSocket：只订阅核心 pose/grid 主题，做流控后推送给前端。
@app.websocket("/ws/stream")
async def ws_stream(websocket: WebSocket) -> None:
    global latest_occupancy_grid

    # 接受连接后，为该客户端创建独立的发送队列和订阅任务。
    await websocket.accept()
    ws_id = id(websocket)
    ws_clients.add(ws_id)
    SERVER_RUNTIME["active_ws_connections_peak"] = max(int(SERVER_RUNTIME["active_ws_connections_peak"]), len(ws_clients))
    logger.info("ws connected id=%s clients=%s", ws_id, len(ws_clients))
    tasks = []
    outbound_queue: asyncio.Queue = asyncio.Queue(maxsize=CONFIG.ws_queue_size * 2)
    last_sent_at: dict[str, float] = {}

    ws_overflow_local = 0
    ws_near_capacity_local = 0
    ws_last_warn_local = 0.0
    last_client_activity = time.time()
    closed_by_server_timeout = False
    disconnect_reason = "client_disconnect"
    disconnect_detail = ""

    # 队列接近容量上限时限频输出告警，避免日志过多。
    def maybe_warn_capacity(fill_ratio: float, reason: str) -> None:
        nonlocal ws_last_warn_local
        now = time.time()
        if now - ws_last_warn_local >= QUEUE_WARN_INTERVAL_SEC:
            logger.warning(
                "ws queue near limit id=%s reason=%s fill=%.2f qsize=%s max=%s overflow_total=%s",
                ws_id,
                reason,
                fill_ratio,
                outbound_queue.qsize(),
                outbound_queue.maxsize,
                ws_overflow_local,
            )
            ws_last_warn_local = now
            SERVER_RUNTIME["ws_last_warn_at"] = now

    # 非阻塞入队：队列满时丢弃最旧消息，优先保持连接实时性。
    def enqueue_nonblocking(item: tuple[str, dict | str], reason: str) -> None:
        nonlocal ws_overflow_local, ws_near_capacity_local
        while True:
            try:
                if outbound_queue.maxsize > 0:
                    fill_ratio = outbound_queue.qsize() / outbound_queue.maxsize
                    if fill_ratio >= QUEUE_NEAR_CAPACITY_RATIO:
                        ws_near_capacity_local += 1
                        SERVER_RUNTIME["ws_near_capacity_total"] += 1
                        maybe_warn_capacity(fill_ratio, reason)
                outbound_queue.put_nowait(item)
                return
            except asyncio.QueueFull:
                ws_overflow_local += 1
                SERVER_RUNTIME["ws_overflow_total"] += 1
                maybe_warn_capacity(1.0, reason)
                try:
                    outbound_queue.get_nowait()
                except asyncio.QueueEmpty:
                    continue

    # 从 TopicBus 订阅单个主题，按主题策略限频/抽样后放入发送队列。
    async def enqueue_topic(topic: str) -> None:
        global latest_occupancy_grid

        async for message in bus.subscribe(topic):
            now = time.time()
            min_interval = TOPIC_MIN_INTERVAL_SEC.get(topic, 0.0)
            if min_interval > 0:
                last = last_sent_at.get(topic, 0.0)
                if now - last < min_interval:
                    continue
                last_sent_at[topic] = now

            elif topic == "/map/grid":
                latest_occupancy_grid = _occupancy_payload_to_grid(message["payload"])

            outbound_message = message

            packed = _pack_message(outbound_message)
            enqueue_nonblocking(("json", packed), reason=topic)

    # 监控客户端活跃度；长时间无收发时主动断开，释放服务端资源。
    async def monitor_client_idle() -> None:
        nonlocal closed_by_server_timeout, disconnect_reason, disconnect_detail
        while True:
            await asyncio.sleep(2.0)
            idle_sec = time.time() - last_client_activity
            if idle_sec > CLIENT_IDLE_TIMEOUT_SEC:
                SERVER_RUNTIME["forced_disconnect_total"] += 1
                closed_by_server_timeout = True
                disconnect_reason = "idle_timeout"
                disconnect_detail = f"idle_sec={idle_sec:.1f}"
                logger.warning("ws idle timeout id=%s idle_sec=%.1f, force disconnect", ws_id, idle_sec)
                try:
                    await websocket.close(code=4001, reason="idle_timeout")
                except Exception as exc:  # noqa: BLE001
                    logger.info("ws close ignored id=%s reason=idle_timeout detail=%s", ws_id, exc)
                return

    # 从发送队列取消息并写入 WebSocket，统一处理断开和 close 后发送异常。
    async def send_outbound() -> None:
        nonlocal last_client_activity, disconnect_reason, disconnect_detail
        while True:
            message_type, payload = await outbound_queue.get()
            try:
                if message_type == "json":
                    await websocket.send_json(payload)
                    last_client_activity = time.time()
                elif message_type == "text":
                    await websocket.send_text(payload)
                    last_client_activity = time.time()
            except WebSocketDisconnect:
                disconnect_reason = "client_disconnect"
                disconnect_detail = f"send_{message_type}"
                logger.info("ws send interrupted id=%s reason=%s detail=%s", ws_id, disconnect_reason, disconnect_detail)
                raise
            except RuntimeError as exc:
                if _is_websocket_send_after_close_error(exc):
                    disconnect_reason = "send_after_close"
                    disconnect_detail = f"send_{message_type}"
                    logger.info("ws send skipped after close id=%s reason=%s detail=%s", ws_id, disconnect_reason, disconnect_detail)
                    raise WebSocketDisconnect() from exc
                raise

    # 接收客户端 keepalive；收到 ping 时回复 pong，并刷新活跃时间。
    async def receive_keepalive() -> None:
        nonlocal last_client_activity, disconnect_reason, disconnect_detail
        while True:
            try:
                client_msg = await websocket.receive_text()
            except WebSocketDisconnect:
                disconnect_reason = "client_disconnect"
                disconnect_detail = "receive_text"
                raise
            last_client_activity = time.time()
            if client_msg == "ping":
                enqueue_nonblocking(("text", "pong"), reason="keepalive")

    try:
        # 为每个主题创建一个订阅任务，再创建发送、保活接收和空闲监控任务。
        for topic in STREAM_TOPICS:
            tasks.append(asyncio.create_task(enqueue_topic(topic)))
        tasks.append(asyncio.create_task(send_outbound()))
        tasks.append(asyncio.create_task(receive_keepalive()))
        tasks.append(asyncio.create_task(monitor_client_idle()))
        done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_EXCEPTION)
        for finished in done:
            exc = finished.exception()
            if exc is not None and not isinstance(exc, WebSocketDisconnect):
                raise exc
    except WebSocketDisconnect:
        if closed_by_server_timeout:
            logger.warning("ws disconnected id=%s reason=%s detail=%s", ws_id, disconnect_reason, disconnect_detail or "server_close")
        else:
            logger.warning("ws disconnected id=%s reason=%s detail=%s", ws_id, disconnect_reason, disconnect_detail or "client_close")
    except asyncio.CancelledError:
        logger.info("ws task cancelled id=%s", ws_id)
        raise
    finally:
        # 无论正常断开还是异常退出，都取消所有子任务并清理客户端状态。
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        ws_clients.discard(ws_id)
        if ws_overflow_local > 0:
            logger.warning("ws disconnected with overflow id=%s dropped=%s near_capacity=%s", ws_id, ws_overflow_local, ws_near_capacity_local)
        try:
            await websocket.close()
        except Exception:  # noqa: BLE001
            pass
