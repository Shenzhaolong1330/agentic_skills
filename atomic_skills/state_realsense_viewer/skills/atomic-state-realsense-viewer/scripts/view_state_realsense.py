#!/usr/bin/env python3
"""Inspect dual-Franka state and capture RealSense RGB-D snapshots."""

from __future__ import annotations

import argparse
from datetime import datetime
import importlib.util
import json
import os
from pathlib import Path
import sys
from typing import Any

import numpy as np

SKILL_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = SKILL_ROOT.parents[2]
REPO_ROOT = Path(__file__).resolve().parents[5]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "state_realsense_viewer" / "runs" / "latest"
DEFAULT_CLIENT_PATH = Path(
    "/home/deepcybo/agentic_skills/atomic_skills/dual_franka_p2p/skills/"
    "atomic-motion-franka-move-to-pose/dual_franka_robotiq_rpc_client.py"
)
DEFAULT_OBJECT_LOCATOR_SRC = Path(
    os.environ.get(
        "OBJECT_LOCATOR_SRC",
        "/home/deepcybo/agentic_skills/atomic_skills/object_locator/src",
    )
)
FALLBACK_PYTHON = Path(os.environ.get("STATE_REALSENSE_PYTHON", "/home/deepcybo/miniconda3/bin/python"))

from scripts.hardware_preflight import evaluate_operation

CAMERAS: dict[str, dict[str, Any]] = {
    "head": {"serial_number": "348522072761", "width": 640, "height": 480, "fps": 30},
    "left_wrist": {"serial_number": "347622074336", "width": 640, "height": 480, "fps": 30},
    "right_wrist": {"serial_number": "337322072568", "width": 640, "height": 480, "fps": 30},
}


def _load_rpc_client(client_path: Path):
    _reexec_with_fallback_python_if_missing(["zerorpc"])
    spec = importlib.util.spec_from_file_location("dual_franka_rpc_client_viewer", client_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load RPC client from {client_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_realsense(object_locator_src: Path):
    _reexec_with_fallback_python_if_missing(["pyrealsense2", "cv2"])
    sys.path.insert(0, str(object_locator_src))
    try:
        from object_locator.realsense_camera import RealSenseCamera, list_realsense_devices
    except ImportError as exc:
        raise RuntimeError(
            "failed to import object_locator.realsense_camera; "
            f"check OBJECT_LOCATOR_SRC or --object-locator-src: {object_locator_src}"
        ) from exc
    return RealSenseCamera, list_realsense_devices


def _reexec_with_fallback_python_if_missing(module_names: list[str]) -> None:
    missing = [
        module_name
        for module_name in module_names
        if importlib.util.find_spec(module_name) is None
    ]
    if not missing:
        return
    fallback = FALLBACK_PYTHON.expanduser()
    if not fallback.exists() or Path(sys.executable).resolve() == fallback.resolve():
        return
    os.execv(str(fallback), [str(fallback), *sys.argv])


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _write_json(path: Path, data: Any, *, compact: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(_jsonable(data), ensure_ascii=False, default=str, indent=None if compact else 2)
        + "\n",
        encoding="utf-8",
    )


def _selected_cameras(values: list[str]) -> list[str]:
    selected: list[str] = []
    raw_values = values or ["all"]
    for value in raw_values:
        for item in value.split(","):
            name = item.strip()
            if not name:
                continue
            if name == "all":
                for camera_name in CAMERAS:
                    if camera_name not in selected:
                        selected.append(camera_name)
                continue
            if name not in CAMERAS:
                raise ValueError(f"unknown camera {name!r}; choose from all,{','.join(CAMERAS)}")
            if name not in selected:
                selected.append(name)
    return selected


def _depth_to_bgr(depth_m: np.ndarray) -> np.ndarray:
    import cv2

    valid = depth_m[np.isfinite(depth_m) & (depth_m > 0)]
    if valid.size == 0:
        normalized = np.zeros(depth_m.shape, dtype=np.uint8)
    else:
        near, far = np.percentile(valid, [1, 99])
        if far <= near:
            far = near + 1e-3
        clipped = np.clip(depth_m, near, far)
        normalized = ((clipped - near) * (255.0 / (far - near))).astype(np.uint8)
        normalized[~np.isfinite(depth_m) | (depth_m <= 0)] = 0
    return cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)


def _make_panel(color_bgr: np.ndarray, depth_bgr: np.ndarray, label: str) -> np.ndarray:
    import cv2

    h = max(color_bgr.shape[0], depth_bgr.shape[0])
    if color_bgr.shape[:2] != depth_bgr.shape[:2]:
        depth_bgr = cv2.resize(depth_bgr, (color_bgr.shape[1], color_bgr.shape[0]))
        h = color_bgr.shape[0]
    header = np.full((36, color_bgr.shape[1] * 2, 3), 32, dtype=np.uint8)
    cv2.putText(header, label, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (245, 245, 245), 2)
    panel = np.hstack([color_bgr, depth_bgr])
    if panel.shape[0] != h:
        panel = cv2.resize(panel, (panel.shape[1], h))
    return np.vstack([header, panel])


def _capture_camera(name: str, camera_cfg: dict[str, Any], args: argparse.Namespace, output_dir: Path) -> dict[str, Any]:
    import cv2

    RealSenseCamera, _ = _load_realsense(args.object_locator_src.expanduser())
    result: dict[str, Any] = {
        "camera": name,
        "serial_number": camera_cfg["serial_number"],
        "width": camera_cfg["width"],
        "height": camera_cfg["height"],
        "fps": camera_cfg["fps"],
    }
    with RealSenseCamera(
        width=int(camera_cfg["width"]),
        height=int(camera_cfg["height"]),
        fps=int(camera_cfg["fps"]),
        serial_number=str(camera_cfg["serial_number"]),
        visual_preset=args.visual_preset,
        reset_on_start=args.reset_realsense,
        reset_wait_s=args.reset_wait_sec,
    ) as camera:
        frame = camera.capture(
            warmup_frames=args.warmup_frames,
            timeout_ms=args.frame_timeout_ms,
            retries=args.capture_retries,
        )

    rgb_path = output_dir / f"{name}_rgb.jpg"
    depth_path = output_dir / f"{name}_depth.jpg"
    panel_path = output_dir / f"{name}_panel.jpg"
    depth_bgr = _depth_to_bgr(frame.depth_m)
    panel = _make_panel(frame.color_bgr, depth_bgr, f"{name} sn={camera_cfg['serial_number']}")
    cv2.imwrite(str(rgb_path), frame.color_bgr)
    cv2.imwrite(str(depth_path), depth_bgr)
    cv2.imwrite(str(panel_path), panel)

    valid_depth = frame.depth_m[np.isfinite(frame.depth_m) & (frame.depth_m > 0)]
    result.update(
        {
            "timestamp_ms": frame.timestamp_ms,
            "rgb_image": str(rgb_path),
            "depth_image": str(depth_path),
            "panel_image": str(panel_path),
            "intrinsics": frame.intrinsics.to_dict(),
            "valid_depth_ratio": float(valid_depth.size / frame.depth_m.size),
            "depth_median_m": float(np.median(valid_depth)) if valid_depth.size else None,
        }
    )
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("mock", "dry_run", "from_artifacts", "live"), default="dry_run")
    parser.add_argument("--hardware-allowed", action="store_true", help="Allow access to real cameras/RPC in live mode.")
    parser.add_argument("--execute", action="store_true", help="Allow camera reset when --reset-realsense is selected.")
    parser.add_argument("--camera", action="append", help="all, head, left_wrist, right_wrist; may be repeated or comma-separated.")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--timestamp-dir", action="store_true", help="Create a timestamped subdirectory under --output-dir.")
    parser.add_argument("--no-state", action="store_true")
    parser.add_argument("--no-images", action="store_true")
    parser.add_argument("--print-state", action="store_true")
    parser.add_argument("--list-devices", action="store_true")
    parser.add_argument("--object-locator-src", type=Path, default=DEFAULT_OBJECT_LOCATOR_SRC)
    parser.add_argument("--server-host", default=os.environ.get("FRANKA_RPC_HOST", "172.16.0.1"))
    parser.add_argument("--server-port", type=int, default=int(os.environ.get("FRANKA_RPC_PORT", "4242")))
    parser.add_argument("--rpc-timeout-sec", type=float, default=float(os.environ.get("FRANKA_RPC_TIMEOUT_SEC", "30")))
    parser.add_argument("--warmup-frames", type=int, default=15)
    parser.add_argument("--frame-timeout-ms", type=int, default=20000)
    parser.add_argument("--capture-retries", type=int, default=5)
    parser.add_argument("--visual-preset", type=int, default=4)
    parser.add_argument("--reset-realsense", action="store_true")
    parser.add_argument("--reset-wait-sec", type=float, default=5.0)
    parser.add_argument("--compact", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    operation = "viewer-reset" if args.reset_realsense else "viewer"
    if args.mode != "live":
        print(json.dumps({"mode": args.mode, "planned_only": True, "operation": operation}))
        return 0
    _, _, decision = evaluate_operation(
        operation,
        mode=args.mode,
        hardware_allowed=args.hardware_allowed,
        execute=args.execute,
        manifest_path=REPO_ROOT / "skill_manifest.json",
    )
    if not decision.allowed:
        print(json.dumps({"mode": args.mode, "gate_decision": decision.to_dict()}))
        return 1
    output_dir = args.output_dir.expanduser()
    if args.timestamp_dir:
        output_dir = output_dir / datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir.mkdir(parents=True, exist_ok=True)

    summary: dict[str, Any] = {
        "output_dir": str(output_dir),
        "server": f"{args.server_host}:{args.server_port}",
        "state": None,
        "devices": None,
        "captures": {},
        "errors": {},
    }

    if args.list_devices:
        _, list_realsense_devices = _load_realsense(args.object_locator_src.expanduser())
        summary["devices"] = [device.to_dict() for device in list_realsense_devices()]

    if not args.no_state:
        try:
            rpc = _load_rpc_client(DEFAULT_CLIENT_PATH)
            client = rpc.DualFrankaRobotiqRpcClient(
                ip=args.server_host,
                port=args.server_port,
                timeout=args.rpc_timeout_sec,
            )
            try:
                state = client.get_full_state()
            finally:
                client.close()
            state_path = output_dir / "state.json"
            _write_json(state_path, state, compact=args.compact)
            summary["state"] = {"ok": True, "path": str(state_path)}
            if args.print_state:
                print(json.dumps(_jsonable(state), ensure_ascii=False, default=str, indent=None if args.compact else 2))
        except Exception as exc:
            summary["state"] = {"ok": False, "error": str(exc)}
            summary["errors"]["state"] = str(exc)

    if not args.no_images:
        for name in _selected_cameras(args.camera):
            try:
                summary["captures"][name] = _capture_camera(name, CAMERAS[name], args, output_dir)
            except Exception as exc:
                summary["captures"][name] = {"ok": False, "error": str(exc)}
                summary["errors"][name] = str(exc)

    summary_path = output_dir / "summary.json"
    _write_json(summary_path, summary, compact=args.compact)
    print(json.dumps(_jsonable(summary), ensure_ascii=False, default=str, indent=None if args.compact else 2))
    return 1 if summary["errors"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
