from __future__ import annotations

from pathlib import Path
import socket
import sys
import threading
from types import SimpleNamespace

import pytest


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))
from wrist_camera_service import CameraState, WristCameraClient, _camera_worker, _receive, _send  # noqa: E402
import wrist_camera_service  # noqa: E402


def test_client_health_and_flush_protocol(tmp_path: Path) -> None:
    socket_path = tmp_path / "camera.sock"
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(socket_path))
    server.listen(2)

    def serve() -> None:
        for _ in range(2):
            connection, _address = server.accept()
            with connection:
                request = _receive(connection)
                if request["op"] == "health":
                    _send(connection, {"ok": True, "cameras": {"left": {"ready": True}, "right": {"ready": True}}})
                else:
                    _send(connection, {"ok": True, "discarded_through_generation": {"left": 2, "right": 3}})
        server.close()

    thread = threading.Thread(target=serve)
    thread.start()
    client = WristCameraClient(socket_path)
    assert client.health()["cameras"]["left"]["ready"]
    assert client.flush()["discarded_through_generation"]["right"] == 3
    thread.join()


def test_service_fails_when_required_head_is_missing(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(wrist_camera_service, "list_realsense_devices", lambda: [SimpleNamespace(serial_number="right")])
    args = SimpleNamespace(
        head_serial="head",
        left_serial="left",
        right_serial="right",
        required_camera=["head"],
        device_discovery_timeout_sec=0.0,
        device_discovery_poll_sec=0.05,
        ready_file=tmp_path / "ready.json",
    )
    with pytest.raises(RuntimeError, match="required RealSense devices are unavailable after discovery retries"):
        wrist_camera_service.serve(args)
    assert not args.ready_file.exists()


def test_missing_usb_serial_waits_without_hardware_reset_spam(monkeypatch: pytest.MonkeyPatch) -> None:
    stop = threading.Event()
    constructors: list[object] = []

    monkeypatch.setattr(wrist_camera_service, "_serial_is_available", lambda _serial: False)
    monkeypatch.setattr(
        wrist_camera_service,
        "RealSenseCamera",
        lambda *args, **kwargs: constructors.append((args, kwargs)),
    )
    state = CameraState(side="left", serial="missing-left")
    thread = threading.Thread(
        target=_camera_worker,
        args=(state, stop),
        kwargs={
            "width": 640,
            "height": 480,
            "fps": 30,
            "warmup_frames": 1,
            "frame_timeout_ms": 10,
        },
    )
    thread.start()
    assert threading.Event().wait(0.05) is False
    stop.set()
    thread.join(timeout=1.0)

    assert not thread.is_alive()
    assert constructors == []
    assert state.status == "waiting_for_usb"
    assert state.reset_count == 0
