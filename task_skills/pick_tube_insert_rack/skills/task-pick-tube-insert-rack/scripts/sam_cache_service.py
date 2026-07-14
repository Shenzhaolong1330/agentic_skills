#!/usr/bin/env python3
"""Task-local persistent SAM service and detector-compatible client.

The service keeps one SAM model resident for the duration of the full task.
Only segmentation is delegated; camera capture, VLM, depth, and calibration
remain in the existing object-locator flow.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import pickle
import signal
import socket
import struct
import sys
import time
from typing import Any

import cv2
import numpy as np


OBJECT_LOCATOR_ROOT = Path("/home/deepcybo/agentic_skills/atomic_skills/object_locator")
sys.path.insert(0, str(OBJECT_LOCATOR_ROOT / "src"))

from object_locator.grounded_sam_detector import (  # noqa: E402
    GroundedSamConfig,
    GroundedSamDetector,
    bbox_from_mask,
    estimate_sample_bottle_keypoints,
)
from object_locator.models import BoundingBox, DetectionResult  # noqa: E402


SOCKET_ENV = "TASK_PICK_TUBE_SAM_SOCKET"
_HEADER = struct.Struct("!Q")


def _receive_exact(connection: socket.socket, size: int) -> bytes:
    chunks: list[bytes] = []
    remaining = size
    while remaining:
        chunk = connection.recv(remaining)
        if not chunk:
            raise ConnectionError("SAM cache connection closed before response completed")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def _receive_message(connection: socket.socket) -> Any:
    size = _HEADER.unpack(_receive_exact(connection, _HEADER.size))[0]
    if size > 128 * 1024 * 1024:
        raise ValueError(f"refusing oversized SAM cache message: {size} bytes")
    return pickle.loads(_receive_exact(connection, size))


def _send_message(connection: socket.socket, payload: Any) -> None:
    encoded = pickle.dumps(payload, protocol=pickle.HIGHEST_PROTOCOL)
    connection.sendall(_HEADER.pack(len(encoded)))
    connection.sendall(encoded)


class CachedSamRefiner:
    """GroundedSamDetector-compatible SAM refinement backed by the service."""

    def __init__(
        self,
        config: GroundedSamConfig,
        socket_path: Path | None = None,
        *,
        connect_timeout_sec: float = 20.0,
        inference_timeout_sec: float = 120.0,
    ) -> None:
        self.config = config
        raw_socket = str(socket_path) if socket_path is not None else os.environ.get(SOCKET_ENV, "")
        if not raw_socket:
            raise RuntimeError(f"{SOCKET_ENV} is not set")
        self.socket_path = Path(raw_socket)
        self.connect_timeout_sec = float(connect_timeout_sec)
        self.inference_timeout_sec = float(inference_timeout_sec)

    @staticmethod
    def _bgr_to_pil(image_bgr: np.ndarray):
        from PIL import Image

        return Image.fromarray(cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB))

    def _segment_bgr_with_sam(
        self,
        image_bgr: np.ndarray,
        bbox: BoundingBox,
    ) -> tuple[np.ndarray | None, float | None]:
        deadline = time.monotonic() + self.connect_timeout_sec
        last_error: OSError | None = None
        while time.monotonic() < deadline:
            connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            try:
                connection.settimeout(min(1.0, max(0.1, deadline - time.monotonic())))
                connection.connect(str(self.socket_path))
                connection.settimeout(self.inference_timeout_sec)
                _send_message(
                    connection,
                    {
                        "op": "segment",
                        "sam_model": self.config.sam_model,
                        "image_bgr": np.ascontiguousarray(image_bgr),
                        "bbox": bbox.to_dict(),
                    },
                )
                response = _receive_message(connection)
                if not response.get("ok"):
                    raise RuntimeError(f"persistent SAM service failed: {response.get('error')}")
                return response.get("mask"), response.get("score")
            except (FileNotFoundError, ConnectionRefusedError, socket.timeout) as exc:
                last_error = exc
                time.sleep(0.05)
            finally:
                connection.close()
        raise RuntimeError(
            f"persistent SAM service unavailable at {self.socket_path} after "
            f"{self.connect_timeout_sec:.1f}s: {last_error}"
        )

    def _segment_with_sam(self, pil_image, bbox: BoundingBox):
        image_rgb = np.asarray(pil_image.convert("RGB"))
        return self._segment_bgr_with_sam(cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR), bbox)

    def refine_detection_with_sam(
        self,
        image_bgr: np.ndarray,
        detection: DetectionResult,
    ) -> DetectionResult:
        if not detection.found:
            return detection
        notes = [detection.notes] if detection.notes else []
        notes.append("VLM bbox segmented by persistent SAM cache")
        mask, sam_score = self._segment_bgr_with_sam(image_bgr, detection.bbox)
        if mask is not None:
            notes.append(f"SAM mask area={int(mask.sum())} px")

        bbox = detection.bbox
        head_px = None
        tail_px = None
        if mask is not None and int(mask.sum()) >= self.config.min_mask_area_px:
            if self.config.refine_bbox_with_mask:
                refined = bbox_from_mask(mask)
                if refined is not None:
                    bbox = refined
            head_px, tail_px = estimate_sample_bottle_keypoints(
                image_bgr,
                mask,
                bbox,
                cap_endpoint_rule=self.config.cap_endpoint_rule,
                cap_dark_threshold=self.config.cap_dark_threshold,
                cap_min_area_px=self.config.cap_min_area_px,
            )
            if head_px and tail_px:
                notes.append(
                    "head/tail estimated from SAM mask principal axis "
                    f"and {self.config.cap_endpoint_rule} cap rule"
                )
            else:
                notes.append("head/tail unavailable from SAM mask")

        confidence = detection.confidence
        if sam_score is not None:
            confidence = float(min(1.0, 0.7 * confidence + 0.3 * sam_score))
        return DetectionResult(
            found=detection.found,
            label=detection.label,
            confidence=confidence,
            bbox=bbox,
            notes="; ".join(notes),
            source="vlm+sam-cache",
            head_px=head_px,
            tail_px=tail_px,
            mask=mask,
        )


def serve(socket_path: Path, ready_file: Path | None, sam_model: str, device: str) -> int:
    socket_path.parent.mkdir(parents=True, exist_ok=True)
    socket_path.unlink(missing_ok=True)
    if ready_file is not None:
        ready_file.parent.mkdir(parents=True, exist_ok=True)
        ready_file.unlink(missing_ok=True)

    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(socket_path))
    server.listen(8)
    stop_requested = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stop_requested
        stop_requested = True
        server.close()

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)

    detector = GroundedSamDetector(GroundedSamConfig(sam_model=sam_model, device=device))
    started = time.perf_counter()
    detector._load_torch()
    detector._load_sam()
    load_sec = time.perf_counter() - started
    print(
        f"[sam-cache] ready model={sam_model} device={detector._device} load_sec={load_sec:.3f}",
        file=sys.stderr,
        flush=True,
    )
    if ready_file is not None:
        ready_file.write_text(
            f'{{"pid": {os.getpid()}, "model": "{sam_model}", "load_sec": {load_sec:.6f}}}\n',
            encoding="utf-8",
        )

    request_count = 0
    try:
        while not stop_requested:
            try:
                connection, _ = server.accept()
            except OSError:
                if stop_requested:
                    break
                raise
            with connection:
                try:
                    request = _receive_message(connection)
                    if request.get("op") != "segment":
                        raise ValueError(f"unsupported SAM cache operation: {request.get('op')!r}")
                    if request.get("sam_model") != sam_model:
                        raise ValueError(
                            f"SAM cache model mismatch: service={sam_model!r}, "
                            f"request={request.get('sam_model')!r}"
                        )
                    bbox = BoundingBox(**request["bbox"])
                    image_bgr = np.asarray(request["image_bgr"], dtype=np.uint8)
                    inference_started = time.perf_counter()
                    mask, score = detector._segment_with_sam(detector._bgr_to_pil(image_bgr), bbox)
                    request_count += 1
                    print(
                        f"[sam-cache] request={request_count} "
                        f"inference_sec={time.perf_counter() - inference_started:.3f}",
                        file=sys.stderr,
                        flush=True,
                    )
                    _send_message(connection, {"ok": True, "mask": mask, "score": score})
                except Exception as exc:
                    try:
                        _send_message(connection, {"ok": False, "error": repr(exc)})
                    except OSError:
                        pass
    finally:
        server.close()
        socket_path.unlink(missing_ok=True)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serve", action="store_true", required=True)
    parser.add_argument("--socket", type=Path, required=True)
    parser.add_argument("--ready-file", type=Path, default=None)
    parser.add_argument("--sam-model", default="facebook/sam-vit-base")
    parser.add_argument("--device", default="auto")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return serve(args.socket, args.ready_file, args.sam_model, args.device)


if __name__ == "__main__":
    raise SystemExit(main())
