from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Any

try:
    import serial  # type: ignore
except Exception:  # noqa: BLE001
    serial = None


logger = logging.getLogger("autodrive.server.ultrasonic")


@dataclass
class UltrasonicSafetySnapshot:
    enabled: bool = False
    mode: str = "disabled"
    blocked: bool = False
    reason: str = ""
    distance_m: float | None = None
    sensor_distances_m: list[float] = field(default_factory=list)
    consecutive_faults: int = 0
    consecutive_clear: int = 0
    last_valid_at: float = 0.0
    last_error: str = ""
    last_raw_hex: str = ""
    transport_ready: bool = False

    def as_dict(self) -> dict[str, Any]:
        return {
            "enabled": bool(self.enabled),
            "mode": self.mode,
            "blocked": bool(self.blocked),
            "reason": self.reason,
            "distance_m": self.distance_m,
            "sensor_distances_m": list(self.sensor_distances_m),
            "consecutive_faults": int(self.consecutive_faults),
            "consecutive_clear": int(self.consecutive_clear),
            "last_valid_at": float(self.last_valid_at),
            "last_error": self.last_error,
            "last_raw_hex": self.last_raw_hex,
            "transport_ready": bool(self.transport_ready),
        }


def _copy_snapshot(snapshot: UltrasonicSafetySnapshot) -> UltrasonicSafetySnapshot:
    return UltrasonicSafetySnapshot(
        enabled=bool(snapshot.enabled),
        mode=str(snapshot.mode),
        blocked=bool(snapshot.blocked),
        reason=str(snapshot.reason),
        distance_m=None if snapshot.distance_m is None else float(snapshot.distance_m),
        sensor_distances_m=[float(value) for value in snapshot.sensor_distances_m],
        consecutive_faults=int(snapshot.consecutive_faults),
        consecutive_clear=int(snapshot.consecutive_clear),
        last_valid_at=float(snapshot.last_valid_at),
        last_error=str(snapshot.last_error),
        last_raw_hex=str(snapshot.last_raw_hex),
        transport_ready=bool(snapshot.transport_ready),
    )


class UltrasonicSafetyRuntime:
    def __init__(self, config: Any, serial_factory: Any | None = None, time_fn: Any | None = None) -> None:
        self.config = config
        self._serial_factory = serial_factory if serial_factory is not None else (serial.Serial if serial is not None else None)
        self._time = time_fn if callable(time_fn) else time.time
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        mode = str(getattr(config, "mode", "disabled") or "disabled")
        enabled = bool(getattr(config, "enabled", False)) and mode != "disabled"
        self._snapshot = UltrasonicSafetySnapshot(enabled=enabled, mode=mode)
        self._last_valid_distance_m: float | None = None
        self._pending_jump_distance_m: float | None = None
        self._last_serial_error_at = 0.0

    def start(self) -> None:
        if not bool(getattr(self.config, "enabled", False)):
            return
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_loop, name="ultrasonic-safety", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None

    def snapshot(self) -> UltrasonicSafetySnapshot:
        with self._lock:
            return _copy_snapshot(self._snapshot)

    def gate_command(self, velocity: float, yaw_rate: float, now: float | None = None) -> tuple[float, float, str]:
        velocity = float(velocity)
        yaw_rate = float(yaw_rate)
        if abs(velocity) <= 1e-9 and abs(yaw_rate) <= 1e-9:
            return 0.0, 0.0, "clear"
        snap = self.snapshot()
        if not snap.enabled:
            return velocity, yaw_rate, "clear"
        current_time = self._time() if now is None else float(now)
        stale_timeout = max(0.05, float(getattr(self.config, "stale_data_timeout_sec", 0.4) or 0.4))
        if snap.last_valid_at <= 0.0 or current_time - snap.last_valid_at > stale_timeout:
            return 0.0, 0.0, "sensor_stale"
        if snap.blocked:
            return 0.0, 0.0, snap.reason or "blocked"
        return velocity, yaw_rate, "clear"

    def update_from_measurement(
        self,
        distances_m: list[float] | None,
        raw: bytes = b"",
        error: str = "",
        now: float | None = None,
        transport_ready: bool = True,
    ) -> UltrasonicSafetySnapshot:
        current_time = self._time() if now is None else float(now)
        with self._lock:
            snapshot = self._snapshot
            snapshot.enabled = bool(getattr(self.config, "enabled", False))
            snapshot.mode = str(getattr(self.config, "mode", snapshot.mode) or snapshot.mode)
            snapshot.transport_ready = bool(transport_ready)
            snapshot.last_raw_hex = raw.hex() if raw else snapshot.last_raw_hex
            if distances_m:
                return self._accept_measurement_locked(distances_m, current_time, raw)
            return self._register_fault_locked(error=error or "read_failed", current_time=current_time, raw=raw, transport_ready=transport_ready)

    def _run_loop(self) -> None:
        if self._serial_factory is None:
            self.update_from_measurement(None, error="pyserial_unavailable", transport_ready=False)
            logger.warning("ultrasonic safety enabled but pyserial unavailable")
            return
        port = str(getattr(self.config, "port", "/dev/ttyUSB0") or "/dev/ttyUSB0")
        baud_rate = int(getattr(self.config, "baud_rate", 115200) or 115200)
        warmup_sec = max(0.0, float(getattr(self.config, "warmup_sec", 2.0) or 2.0))
        poll_interval_sec = max(0.02, float(getattr(self.config, "poll_interval_sec", 0.08) or 0.08))
        response_timeout_sec = max(0.01, float(getattr(self.config, "response_timeout_sec", 0.05) or 0.05))
        serial_conn = None
        try:
            serial_conn = self._serial_factory(port, baud_rate, timeout=0)
            logger.info("ultrasonic serial opened port=%s baud=%s mode=%s", port, baud_rate, getattr(self.config, "mode", "disabled"))
            time.sleep(warmup_sec)
            if hasattr(serial_conn, "reset_input_buffer"):
                serial_conn.reset_input_buffer()
            if hasattr(serial_conn, "reset_output_buffer"):
                serial_conn.reset_output_buffer()
            while not self._stop_event.is_set():
                raw = self._poll_serial(serial_conn, response_timeout_sec=response_timeout_sec)
                distances_m, error = self._parse_payload(raw)
                self.update_from_measurement(distances_m, raw=raw, error=error or "", transport_ready=True)
                self._stop_event.wait(poll_interval_sec)
        except Exception as exc:  # noqa: BLE001
            self.update_from_measurement(None, error=f"serial_error:{exc}", transport_ready=False)
            logger.warning("ultrasonic serial loop failed port=%s err=%s", port, exc)
        finally:
            if serial_conn is not None:
                try:
                    serial_conn.close()
                except Exception:  # noqa: BLE001
                    pass

    def _poll_serial(self, serial_conn: Any, response_timeout_sec: float) -> bytes:
        trigger_bytes = bytes([int(getattr(self.config, "trigger_byte", 0xFF) or 0xFF) & 0xFF])
        serial_conn.write(trigger_bytes)
        if hasattr(serial_conn, "flush"):
            serial_conn.flush()
        payload = bytearray()
        deadline = self._time() + response_timeout_sec
        expected_bytes = max(4, int(getattr(self.config, "frame_length", 4) or 4) * max(1, int(getattr(self.config, "sensor_count", 1) or 1)))
        while self._time() < deadline:
            waiting = int(getattr(serial_conn, "in_waiting", 0) or 0)
            if waiting > 0:
                payload.extend(serial_conn.read(waiting))
                if len(payload) >= expected_bytes:
                    break
            time.sleep(0.001)
        if hasattr(serial_conn, "reset_input_buffer"):
            serial_conn.reset_input_buffer()
        return bytes(payload)

    def _parse_payload(self, raw: bytes) -> tuple[list[float] | None, str | None]:
        frame_length = max(4, int(getattr(self.config, "frame_length", 4) or 4))
        sensor_count = max(1, int(getattr(self.config, "sensor_count", 1) or 1))
        if len(raw) < frame_length:
            return None, "short_frame"
        distances_m: list[float] = []
        for offset in range(0, min(len(raw), frame_length * sensor_count), frame_length):
            frame = raw[offset : offset + frame_length]
            if len(frame) < frame_length:
                break
            if frame[0] != 0xFF:
                return None, "bad_header"
            checksum = (frame[0] + frame[1] + frame[2]) & 0xFF
            if checksum != frame[3]:
                return None, "bad_checksum"
            distance_mm = (int(frame[1]) << 8) | int(frame[2])
            if distance_mm >= 0xFFFD:
                return None, "sensor_no_echo"
            distance_m = float(distance_mm) / 1000.0
            if distance_m <= 0.0:
                return None, "zero_distance"
            if distance_m > max(0.1, float(getattr(self.config, "max_valid_distance_m", 4.0) or 4.0)):
                return None, "distance_out_of_range"
            distances_m.append(distance_m)
        if not distances_m:
            return None, "no_valid_distance"
        return distances_m, None

    def _accept_measurement_locked(self, distances_m: list[float], current_time: float, raw: bytes) -> UltrasonicSafetySnapshot:
        snapshot = self._snapshot
        min_distance = min(float(distance) for distance in distances_m)
        sudden_jump_m = max(0.0, float(getattr(self.config, "sudden_jump_m", 0.45) or 0.45))
        if self._last_valid_distance_m is not None and abs(min_distance - self._last_valid_distance_m) >= sudden_jump_m:
            if self._pending_jump_distance_m is not None and abs(self._pending_jump_distance_m - min_distance) <= 0.05:
                self._pending_jump_distance_m = None
            else:
                self._pending_jump_distance_m = min_distance
                return self._register_fault_locked(error="sudden_jump_filtered", current_time=current_time, raw=raw, transport_ready=True)
        else:
            self._pending_jump_distance_m = None
        self._last_valid_distance_m = min_distance
        snapshot.distance_m = min_distance
        snapshot.sensor_distances_m = [float(distance) for distance in distances_m]
        snapshot.consecutive_faults = 0
        snapshot.last_valid_at = current_time
        snapshot.last_error = ""
        snapshot.last_raw_hex = raw.hex() if raw else snapshot.last_raw_hex
        danger_distance_m = max(0.05, float(getattr(self.config, "danger_distance_m", 0.35) or 0.35))
        resume_distance_m = max(danger_distance_m, float(getattr(self.config, "resume_distance_m", 0.45) or 0.45))
        recover_count = max(1, int(getattr(self.config, "recover_count", 2) or 2))
        if min_distance < danger_distance_m:
            snapshot.blocked = True
            snapshot.reason = "distance"
            snapshot.consecutive_clear = 0
        elif snapshot.blocked:
            if min_distance >= resume_distance_m:
                snapshot.consecutive_clear += 1
                if snapshot.consecutive_clear >= recover_count:
                    snapshot.blocked = False
                    snapshot.reason = ""
                    snapshot.consecutive_clear = 0
            else:
                snapshot.consecutive_clear = 0
        else:
            snapshot.consecutive_clear = 0
        return _copy_snapshot(snapshot)

    def _register_fault_locked(self, error: str, current_time: float, raw: bytes, transport_ready: bool) -> UltrasonicSafetySnapshot:
        snapshot = self._snapshot
        snapshot.transport_ready = bool(transport_ready)
        snapshot.last_error = str(error)
        snapshot.last_raw_hex = raw.hex() if raw else snapshot.last_raw_hex
        snapshot.consecutive_faults += 1
        snapshot.consecutive_clear = 0
        fault_trip_count = max(1, int(getattr(self.config, "fault_trip_count", 3) or 3))
        if snapshot.consecutive_faults >= fault_trip_count:
            snapshot.blocked = True
            snapshot.reason = "sensor_fault"
        return _copy_snapshot(snapshot)
