#!/usr/bin/env python3
"""Persistent head/wrist RealSense service for task-local same-frame perception."""
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
import os
from pathlib import Path
import pickle
import signal
import socket
import struct
import sys
import threading
import time
import traceback
from typing import Any


OBJECT_LOCATOR_ROOT = Path("/home/deepcybo/agentic_skills/atomic_skills/object_locator")
sys.path.insert(0, str(OBJECT_LOCATOR_ROOT / "src"))
from object_locator.realsense_camera import RealSenseCamera, list_realsense_devices  # noqa: E402


SOCKET_ENV = "TASK_PICK_TUBE_WRIST_CAMERA_SOCKET"
DEFAULT_SERIALS = {
    "head": "348522072761",
    "left": "347622074336",
    "right": "337322072568",
}
_HEADER = struct.Struct("!Q")


def _receive_exact(connection: socket.socket, size: int) -> bytes:
    chunks: list[bytes] = []
    while size:
        chunk = connection.recv(size)
        if not chunk:
            raise ConnectionError("camera service connection closed")
        chunks.append(chunk)
        size -= len(chunk)
    return b"".join(chunks)


def _receive(connection: socket.socket) -> Any:
    size = _HEADER.unpack(_receive_exact(connection, _HEADER.size))[0]
    if size > 256 * 1024 * 1024:
        raise ValueError(f"oversized camera message: {size}")
    return pickle.loads(_receive_exact(connection, size))


def _send(connection: socket.socket, payload: Any) -> None:
    encoded = pickle.dumps(payload, protocol=pickle.HIGHEST_PROTOCOL)
    connection.sendall(_HEADER.pack(len(encoded)) + encoded)


@dataclass
class CameraState:
    side: str
    serial: str
    lock: threading.Lock = field(default_factory=threading.Lock)
    frame: Any = None
    captured_monotonic: float = 0.0
    generation: int = 0
    error: str | None = None
    reset_count: int = 0
    restart_count: int = 0
    status: str = "starting"


def _serial_is_available(serial: str) -> bool:
    try:
        return any(device.serial_number == serial for device in list_realsense_devices())
    except Exception:
        return False


def _camera_worker(
    state: CameraState,
    stop: threading.Event,
    *,
    width: int,
    height: int,
    fps: int,
    warmup_frames: int,
    frame_timeout_ms: int,
) -> None:
    # Recovery stages: 0=initial/healthy open, 1=clean pipeline reopen,
    # 2=hardware reset followed by reopen.  Never request a hardware reset
    # while the serial is absent from USB enumeration.
    recovery_stage = 0
    missing_logged = False
    while not stop.is_set():
        if not _serial_is_available(state.serial):
            message = f"RealSense serial_number={state.serial} is not USB-enumerated"
            with state.lock:
                state.error = message
                state.frame = None
                state.captured_monotonic = 0.0
                state.status = "waiting_for_usb"
            if not missing_logged:
                print(
                    f"[realsense-cache] {state.side} disappeared from USB; "
                    "waiting for re-enumeration without hardware-reset spam",
                    file=sys.stderr,
                    flush=True,
                )
                missing_logged = True
            recovery_stage = 1
            stop.wait(1.0)
            continue

        if missing_logged:
            print(
                f"[realsense-cache] {state.side} re-enumerated; attempting clean pipeline reopen",
                file=sys.stderr,
                flush=True,
            )
            missing_logged = False
            recovery_stage = 1

        camera: RealSenseCamera | None = None
        use_hardware_reset = recovery_stage == 2
        try:
            if use_hardware_reset:
                with state.lock:
                    state.reset_count += 1
                    reset_count = state.reset_count
                    state.status = "hardware_reset"
                print(
                    f"[realsense-cache] {state.side} hardware reset attempt={reset_count}",
                    file=sys.stderr,
                    flush=True,
                )
            else:
                with state.lock:
                    state.status = "opening"
            camera = RealSenseCamera(
                width=width,
                height=height,
                fps=fps,
                serial_number=state.serial,
                visual_preset=4,
                reset_on_start=use_hardware_reset,
                reset_wait_s=5.0,
            )
            camera.start()
            camera.warmup(warmup_frames, timeout_ms=frame_timeout_ms, retries=1)
            with state.lock:
                state.error = None
                state.status = "streaming"
            recovery_stage = 0
            while not stop.is_set():
                frame = camera.capture(timeout_ms=frame_timeout_ms, retries=1)
                with state.lock:
                    state.frame = frame
                    state.captured_monotonic = time.monotonic()
                    state.generation += 1
                    state.error = None
                    state.status = "streaming"
        except Exception:
            error = traceback.format_exc(limit=3)
            with state.lock:
                state.error = error
                state.frame = None
                state.captured_monotonic = 0.0
                state.restart_count += 1
                restart_count = state.restart_count
                state.status = "recovering"
            if _serial_is_available(state.serial):
                if recovery_stage in (0, 2):
                    recovery_stage = 1
                    next_action = "clean pipeline reopen"
                else:
                    recovery_stage = 2
                    next_action = "one hardware reset"
                delay_sec = min(8.0, 1.0 + float(min(restart_count, 7)))
                print(
                    f"[realsense-cache] {state.side} stream failed restart={restart_count}; "
                    f"next={next_action} after {delay_sec:.1f}s",
                    file=sys.stderr,
                    flush=True,
                )
                stop.wait(delay_sec)
            else:
                # The next loop enters the quiet USB re-enumeration wait.
                recovery_stage = 1
        finally:
            if camera is not None:
                try:
                    camera.stop()
                except Exception:
                    # A USB disconnect can make pipeline.stop() fail too.  The
                    # worker must stay alive so it can wait for re-enumeration.
                    pass


def _snapshot(state: CameraState) -> dict[str, Any]:
    with state.lock:
        age_ms = None
        if state.captured_monotonic > 0:
            age_ms = (time.monotonic() - state.captured_monotonic) * 1000.0
        return {
            "side": state.side,
            "serial": state.serial,
            "ready": state.frame is not None,
            "age_ms": age_ms,
            "generation": state.generation,
            "error": state.error,
            "reset_count": state.reset_count,
            "restart_count": state.restart_count,
            "status": state.status,
        }


def serve(args: argparse.Namespace) -> int:
    if args.ready_file:
        args.ready_file.unlink(missing_ok=True)
    configured = {
        "head": args.head_serial,
        "left": args.left_serial,
        "right": args.right_serial,
    }
    required_names = set(args.required_camera or ["head"])
    discovery_timeout_sec = max(0.0, float(args.device_discovery_timeout_sec))
    discovery_poll_sec = max(0.05, float(args.device_discovery_poll_sec))
    discovery_deadline = time.monotonic() + discovery_timeout_sec
    last_missing_required: list[str] | None = None
    while True:
        available = {device.serial_number for device in list_realsense_devices()}
        missing_required = sorted(
            name for name in required_names if configured.get(name) not in available
        )
        if not missing_required:
            break
        if missing_required != last_missing_required:
            print(
                "[realsense-cache] waiting for required devices "
                f"missing={missing_required} available={sorted(available)}",
                file=sys.stderr,
                flush=True,
            )
            last_missing_required = missing_required
        if time.monotonic() >= discovery_deadline:
            raise RuntimeError(
                "required RealSense devices are unavailable after discovery retries; "
                f"missing_names={missing_required}, configured={configured}, "
                f"available={sorted(available)}. An unenumerated USB device cannot be "
                "hardware-reset through librealsense; reconnect or power-cycle it."
            )
        time.sleep(min(discovery_poll_sec, max(0.0, discovery_deadline - time.monotonic())))

    missing_names = sorted(name for name, serial in configured.items() if serial not in available)
    states = {
        name: CameraState(name, serial)
        for name, serial in configured.items()
        if serial in available
    }
    stop = threading.Event()
    threads = [
        threading.Thread(
            target=_camera_worker,
            args=(state, stop),
            kwargs={
                "width": args.width,
                "height": args.height,
                "fps": args.fps,
                "warmup_frames": args.warmup_frames,
                "frame_timeout_ms": args.frame_timeout_ms,
            },
            daemon=True,
        )
        for state in states.values()
    ]
    for thread in threads:
        thread.start()
    deadline = time.monotonic() + args.startup_timeout_sec
    while time.monotonic() < deadline:
        if all(_snapshot(state)["ready"] for state in states.values()):
            break
        time.sleep(0.05)
    else:
        stop.set()
        raise RuntimeError(f"persistent RealSense warmup failed: {[ _snapshot(s) for s in states.values() ]}")

    args.socket.parent.mkdir(parents=True, exist_ok=True)
    args.socket.unlink(missing_ok=True)
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(args.socket))
    server.listen(8)

    def request_stop(_signum, _frame) -> None:
        stop.set()
        server.close()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    if args.ready_file:
        args.ready_file.parent.mkdir(parents=True, exist_ok=True)
        args.ready_file.write_text(
            json.dumps(
                {
                    "pid": os.getpid(),
                    "cameras": {name: _snapshot(state) for name, state in states.items()},
                    "missing_cameras": missing_names,
                },
                indent=2,
            ),
            encoding="utf-8",
        )
    print(
        f"[realsense-cache] ready={sorted(states)} missing={missing_names}",
        file=sys.stderr,
        flush=True,
    )
    try:
        while not stop.is_set():
            try:
                connection, _ = server.accept()
            except OSError:
                if stop.is_set():
                    break
                raise
            with connection:
                try:
                    request = _receive(connection)
                    operation = request.get("op")
                    if operation == "health":
                        response = {"ok": True, "cameras": {side: _snapshot(state) for side, state in states.items()}}
                    elif operation == "frame":
                        side = str(request["side"])
                        state = states.get(side)
                        if state is None:
                            raise RuntimeError(
                                f"requested camera {side!r} is not available; "
                                f"ready={sorted(states)}, missing={missing_names}"
                            )
                        max_age_ms = float(request.get("max_age_ms", 500.0))
                        after_generation = int(request.get("after_generation", -1))
                        deadline = time.monotonic() + float(request.get("timeout_sec", 3.0))
                        while True:
                            with state.lock:
                                age_ms = (time.monotonic() - state.captured_monotonic) * 1000.0 if state.captured_monotonic else float("inf")
                                if state.frame is not None and state.generation > after_generation and age_ms <= max_age_ms:
                                    frame = state.frame
                                    response = {
                                        "ok": True,
                                        "side": side,
                                        "generation": state.generation,
                                        "age_ms": age_ms,
                                        "reset_count": state.reset_count,
                                        "captured_monotonic": state.captured_monotonic,
                                        "color_bgr": frame.color_bgr,
                                        "depth_m": frame.depth_m,
                                        "intrinsics": frame.intrinsics.to_dict(),
                                        "camera_timestamp_ms": frame.timestamp_ms,
                                    }
                                    break
                                error = state.error
                            if time.monotonic() >= deadline:
                                response = {"ok": False, "error": f"no fresh {side} frame <= {max_age_ms:g}ms", "camera_error": error, "health": _snapshot(state)}
                                break
                            time.sleep(0.01)
                    elif operation == "flush":
                        generations = {}
                        for side, state in states.items():
                            with state.lock:
                                generations[side] = state.generation
                                state.frame = None
                                state.captured_monotonic = 0.0
                        response = {"ok": True, "discarded_through_generation": generations}
                    else:
                        raise ValueError(f"unsupported operation: {operation!r}")
                except Exception as exc:
                    response = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
                _send(connection, response)
    finally:
        stop.set()
        server.close()
        for thread in threads:
            thread.join(timeout=5.0)
        args.socket.unlink(missing_ok=True)
    return 0


class WristCameraClient:
    def __init__(self, socket_path: Path | str | None = None, timeout_sec: float = 5.0) -> None:
        raw = str(socket_path) if socket_path is not None else os.environ.get(SOCKET_ENV, "")
        if not raw:
            raise RuntimeError(f"{SOCKET_ENV} is not set")
        self.socket_path = Path(raw)
        self.timeout_sec = timeout_sec

    def request(self, payload: dict[str, Any]) -> dict[str, Any]:
        connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        connection.settimeout(self.timeout_sec)
        try:
            connection.connect(str(self.socket_path))
            _send(connection, payload)
            response = _receive(connection)
        finally:
            connection.close()
        if not response.get("ok"):
            raise RuntimeError(f"wrist camera service failed: {response}")
        return response

    def frame(self, side: str, *, max_age_ms: float = 500.0, after_generation: int = -1) -> dict[str, Any]:
        return self.request({"op": "frame", "side": side, "max_age_ms": max_age_ms, "after_generation": after_generation, "timeout_sec": self.timeout_sec})

    def health(self) -> dict[str, Any]:
        return self.request({"op": "health"})

    def flush(self) -> dict[str, Any]:
        return self.request({"op": "flush"})


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serve", action="store_true")
    parser.add_argument("--socket", type=Path, required=True)
    parser.add_argument("--ready-file", type=Path)
    parser.add_argument("--head-serial", default=DEFAULT_SERIALS["head"])
    parser.add_argument("--left-serial", default=DEFAULT_SERIALS["left"])
    parser.add_argument("--right-serial", default=DEFAULT_SERIALS["right"])
    parser.add_argument(
        "--required-camera",
        action="append",
        choices=("head", "left", "right"),
        default=None,
        help="Camera that must exist for service startup; repeat as needed. Defaults to head only.",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--warmup-frames", type=int, default=30)
    parser.add_argument("--frame-timeout-ms", type=int, default=3000)
    parser.add_argument("--startup-timeout-sec", type=float, default=30.0)
    parser.add_argument(
        "--device-discovery-timeout-sec",
        type=float,
        default=15.0,
        help="Wait this long for required serials to enumerate before startup fails.",
    )
    parser.add_argument(
        "--device-discovery-poll-sec",
        type=float,
        default=0.5,
        help="Polling interval while waiting for required RealSense serials.",
    )
    return parser


if __name__ == "__main__":
    parsed = build_parser().parse_args()
    if not parsed.serve:
        raise SystemExit("--serve is required")
    raise SystemExit(serve(parsed))
