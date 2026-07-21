#!/usr/bin/env python3
"""Run tube insertion from a manually prepared gripper hold.

Starting assumption: the user has already placed the tube in one gripper. This
tool detects which gripper is closed, moves that arm above the rack, uses the
wrist camera to align over an unoccupied hole, then inserts, releases, and
retracts.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import traceback
import time
from typing import Any, Mapping


SCRIPT_DIR = Path(__file__).resolve().parent
ANY_POSE_DIR = Path("/home/deepcybo/agentic_skills/atomic_skills/object_locator")
SKILL_PATH = SCRIPT_DIR / "tube_insertion_skill.py"
DEFAULT_RACK_CONFIG = ANY_POSE_DIR / "config_rack_center_vlm.yaml"
DEFAULT_LEFT_HOLE_CONFIG = ANY_POSE_DIR / "config_rack_empty_hole_left_wrist_vlm.yaml"
DEFAULT_RIGHT_HOLE_CONFIG = ANY_POSE_DIR / "config_rack_empty_hole_right_wrist_vlm.yaml"
DEFAULT_LOG_DIR = SCRIPT_DIR.parent / "logs"
LOG_FILE: Path | None = None
LOG_LOCK = threading.Lock()


@dataclass(frozen=True)
class StageCommand:
    name: str
    argv: list[str]


STAGE_DESCRIPTIONS = {
    "observe_rack": "移动持管手到料架上方观察位，并用腕部相机观察料架",
    "move_above_hole_from_wrist": "根据腕部相机识别未占用孔，移动持管手到孔上方",
    "insert_release_retract": "从当前位置向下插入，释放夹爪，竖直上提脱离后将持管臂复位",
}


def _load_skill_module():
    spec = importlib.util.spec_from_file_location("tube_insertion_skill_for_full_flow", SKILL_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load insertion skill from {SKILL_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


skill = _load_skill_module()


def _str(value: Any) -> str:
    return str(value)


def default_log_file() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return DEFAULT_LOG_DIR / f"manual_grip_to_insert_{stamp}.log"


def set_log_file(path: Path, *, truncate: bool = True) -> None:
    global LOG_FILE
    LOG_FILE = path
    LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
    if truncate:
        LOG_FILE.write_text("", encoding="utf-8")
    else:
        LOG_FILE.touch()


def _write_log_file(text: str) -> None:
    if LOG_FILE is None:
        return
    with LOG_LOCK:
        with LOG_FILE.open("a", encoding="utf-8") as file:
            file.write(text)
            if not text.endswith("\n"):
                file.write("\n")


def log(message: str) -> None:
    line = f"[manual-grip-insert] {message}"
    print(line, file=sys.stderr, flush=True)
    _write_log_file(line)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server-host", default=os.environ.get("FRANKA_RPC_HOST", "172.16.0.1"))
    parser.add_argument("--server-port", type=int, default=int(os.environ.get("FRANKA_RPC_PORT", "4242")))
    parser.add_argument("--rpc-timeout-sec", type=float, default=float(os.environ.get("FRANKA_RPC_TIMEOUT_SEC", "30")))
    parser.add_argument("--any-pose-dir", type=Path, default=ANY_POSE_DIR)
    parser.add_argument(
        "--artifact-dir",
        type=Path,
        default=None,
        help="Directory for per-insertion artifacts such as raw wrist VLM responses.",
    )
    parser.add_argument("--execute", action="store_true", help="Actually send robot and gripper commands.")
    parser.add_argument("--compact", action="store_true")
    parser.add_argument(
        "--log-file",
        type=Path,
        default=None,
        help="Write wrapper logs and child stderr to this file. Defaults to logs/manual_grip_to_insert_<timestamp>.log.",
    )

    parser.add_argument("--holder-side", choices=("auto", "left", "right"), default="auto")
    parser.add_argument("--min-gripper-closed-fraction", type=float, default=0.35)
    parser.add_argument("--min-gripper-margin", type=float, default=0.20)

    parser.add_argument(
        "--rack-plane-z-base-m",
        type=float,
        default=None,
        help=(
            "Optional rack plane z override in base coordinates. If omitted, rack and "
            "hole height come from object-locator depth estimates."
        ),
    )
    parser.add_argument("--rack-config", type=Path, default=DEFAULT_RACK_CONFIG)
    parser.add_argument(
        "--rack-result-json",
        type=Path,
        default=None,
        help="Cached initial head-camera rack detection; skips repeated rack perception.",
    )
    parser.add_argument("--left-hole-config", type=Path, default=DEFAULT_LEFT_HOLE_CONFIG)
    parser.add_argument("--right-hole-config", type=Path, default=DEFAULT_RIGHT_HOLE_CONFIG)
    parser.add_argument("--wrist-perception-mode", choices=("cached-grid", "legacy-vlm"), default="legacy-vlm")
    parser.add_argument("--rack-grid-json", type=Path, default=None)
    parser.add_argument("--target-slot-id", default=None)
    parser.add_argument("--wrist-camera-socket", type=Path, default=None)
    parser.add_argument("--wrist-perception-report", type=Path, default=None)

    parser.add_argument("--observe-height-m", type=float, default=0.12)
    parser.add_argument("--observe-rate-hz", type=float, default=50.0)
    parser.add_argument("--observe-max-translation-speed", type=float, default=0.18)
    parser.add_argument("--observe-max-rotation-speed", type=float, default=0.5)
    parser.add_argument("--observe-max-translation-step", type=float, default=0.01)
    parser.add_argument("--observe-max-rotation-step", type=float, default=0.05)
    parser.add_argument("--observe-position-tolerance-m", type=float, default=0.008)
    parser.add_argument("--observe-rotation-tolerance-rad", type=float, default=0.10)
    parser.add_argument("--observe-max-correction-iters", type=int, default=10)
    parser.add_argument(
        "--non-holder-yield-mode",
        choices=("home", "outward-y"),
        default="home",
    )
    parser.add_argument("--yield-home-duration-sec", type=float, default=4.0)
    parser.add_argument("--yield-home-rate-hz", type=float, default=50.0)
    parser.add_argument("--yield-distance-m", type=float, default=0.12)

    parser.add_argument("--hole-standoff-m", type=float, default=0.08)
    parser.add_argument("--wrist-rate-hz", type=float, default=50.0)
    parser.add_argument("--wrist-max-translation-speed", type=float, default=0.2)
    parser.add_argument("--wrist-max-rotation-speed", type=float, default=0.8)
    parser.add_argument("--wrist-max-translation-step", type=float, default=0.05)
    parser.add_argument("--wrist-max-rotation-step", type=float, default=0.3)
    parser.add_argument("--wrist-position-tolerance-m", type=float, default=0.008)
    parser.add_argument("--wrist-rotation-tolerance-rad", type=float, default=0.035)
    parser.add_argument("--wrist-max-correction-iters", type=int, default=6)
    parser.add_argument("--wrist-max-posture-correction-iters", type=int, default=4)

    parser.add_argument("--insert-depth-m", type=float, default=0.055)
    parser.add_argument(
        "--near-insert-depth-tolerance-m",
        type=float,
        default=0.008,
        help=(
            "Accept a force-safe insertion within this additional depth slack as "
            "completion-with-warning; no insertion retry is performed."
        ),
    )
    parser.add_argument("--insert-rate-hz", type=float, default=50.0)
    parser.add_argument("--insert-max-translation-speed", type=float, default=0.01)
    parser.add_argument("--insert-max-translation-step", type=float, default=0.001)
    parser.add_argument("--insert-position-tolerance-m", type=float, default=0.006)
    parser.add_argument("--insert-settle-time-sec", type=float, default=0.8)
    parser.add_argument(
        "--insert-max-correction-iters",
        type=int,
        default=3,
        help="Maximum force-monitored P2P tolerance corrections after the initial descent.",
    )
    parser.add_argument(
        "--no-monitor-insert-force",
        action="store_false",
        dest="monitor_insert_force",
        default=True,
        help="Disable stepwise force monitoring during insertion descent.",
    )
    parser.add_argument("--insert-force-step-m", type=float, default=0.0005)
    parser.add_argument("--max-insert-force-delta-n", type=float, default=12.0)
    parser.add_argument("--max-insert-lateral-force-delta-n", type=float, default=4.0)
    parser.add_argument("--seated-axial-force-delta-n", type=float, default=6.0)
    parser.add_argument("--seated-min-depth-m", type=float, default=0.003)
    parser.add_argument("--retract-after-release-m", type=float, default=0.06)
    parser.add_argument(
        "--no-home-after-insert",
        action="store_false",
        dest="home_after_insert",
        default=True,
        help="After release, only retract vertically and do not return the insertion arm home.",
    )
    parser.add_argument("--home-after-insert-duration-sec", type=float, default=4.0)
    parser.add_argument("--home-after-insert-rate-hz", type=float, default=50.0)
    parser.add_argument("--retract-max-translation-speed", type=float, default=0.05)
    parser.add_argument("--retract-max-rotation-speed", type=float, default=0.2)
    parser.add_argument("--retract-max-translation-step", type=float, default=0.002)
    parser.add_argument("--retract-max-rotation-step", type=float, default=0.01)
    parser.add_argument("--retract-settle-time-sec", type=float, default=0.5)
    parser.add_argument(
        "--no-release-after-insert",
        action="store_false",
        dest="release_after_insert",
        default=True,
        help="Do not open the holder gripper after the insert descent attempt.",
    )
    return parser


def choose_holder_side_from_gripper_fractions(
    fractions: Mapping[str, float],
    *,
    requested_side: str,
    min_closed_fraction: float,
    min_margin: float,
) -> dict[str, Any]:
    if requested_side in ("left", "right"):
        return {
            "status": "ok",
            "side": requested_side,
            "fractions": dict(fractions),
            "reason": f"explicit holder side: {requested_side}",
        }

    left = float(fractions.get("left", 0.0))
    right = float(fractions.get("right", 0.0))
    threshold = float(min_closed_fraction)
    margin = float(min_margin)
    if left < threshold and right < threshold:
        return {
            "status": "not_found",
            "side": None,
            "fractions": {"left": left, "right": right},
            "reason": f"neither gripper closed above {threshold:g}",
        }
    if left >= threshold and right >= threshold and abs(left - right) < margin:
        return {
            "status": "ambiguous",
            "side": None,
            "fractions": {"left": left, "right": right},
            "reason": f"both grippers closed within margin {margin:g}",
        }
    side = "left" if left > right else "right"
    return {
        "status": "ok",
        "side": side,
        "fractions": {"left": left, "right": right},
        "reason": "selected more-closed gripper",
    }


def resolve_rack_plane_z_base_m(args: argparse.Namespace, *, holder_side: str) -> float | None:
    del holder_side
    if args.rack_plane_z_base_m is None:
        return None
    return float(args.rack_plane_z_base_m)


def read_gripper_fractions(args: argparse.Namespace) -> dict[str, float]:
    log(f"读取夹爪状态: server={args.server_host}:{args.server_port}, timeout={args.rpc_timeout_sec:g}s")
    client = skill.DualFrankaRobotiqRpcClient(
        ip=args.server_host,
        port=args.server_port,
        timeout=args.rpc_timeout_sec,
    )
    try:
        observation, _poses = skill.read_ee_poses(client)
    finally:
        client.close()
    return {
        side: float(skill.gripper_closed_fraction(observation[skill.side_arm_key(side)]))
        for side in ("left", "right")
    }


def _common_rpc_args(args: argparse.Namespace) -> list[str]:
    return [
        "--server-host",
        args.server_host,
        "--server-port",
        _str(args.server_port),
        "--rpc-timeout-sec",
        _str(args.rpc_timeout_sec),
    ]


def _maybe_execute(args: argparse.Namespace) -> list[str]:
    return ["--execute"] if args.execute else []


def build_stage_commands(args: argparse.Namespace, *, holder_side: str) -> list[StageCommand]:
    hole_config = args.left_hole_config if holder_side == "left" else args.right_hole_config
    rack_plane_z_base_m = resolve_rack_plane_z_base_m(args, holder_side=holder_side)
    artifact_dir = args.artifact_dir
    wrist_vlm_response = None if artifact_dir is None else artifact_dir / "wrist_empty_hole_vlm_response.json"
    rack_plane_args = (
        []
        if rack_plane_z_base_m is None
        else ["--rack-plane-z-base-m", _str(rack_plane_z_base_m)]
    )
    wrist_geometry_args = (
        ["--hole-plane-z-source", "detected-depth"]
        if rack_plane_z_base_m is None
        else ["--hole-plane-z-source", "current-standoff", *rack_plane_args]
    )
    observe = StageCommand(
        name="observe_rack",
        argv=[
            sys.executable,
            _str(SCRIPT_DIR / "tube_insertion_skill.py"),
            *_common_rpc_args(args),
            "--any-pose-dir",
            _str(args.any_pose_dir),
            *([] if artifact_dir is None else ["--artifact-dir", _str(artifact_dir / "observe_rack")]),
            "--holder-side",
            holder_side,
            *_maybe_execute(args),
            "--observe-height-m",
            _str(args.observe_height_m),
            *rack_plane_args,
            "--rack-config",
            _str(args.rack_config),
            *(
                []
                if args.rack_result_json is None
                else ["--rack-result-json", _str(args.rack_result_json)]
            ),
            "--left-hole-config",
            _str(args.left_hole_config),
            "--right-hole-config",
            _str(args.right_hole_config),
            "--roll-target-rad",
            _str(float(__import__("math").pi)),
            "--pitch-zero-rad",
            "0",
            "--rate-hz",
            _str(args.observe_rate_hz),
            "--max-translation-speed",
            _str(args.observe_max_translation_speed),
            "--max-rotation-speed",
            _str(args.observe_max_rotation_speed),
            "--max-translation-step",
            _str(args.observe_max_translation_step),
            "--max-rotation-step",
            _str(args.observe_max_rotation_step),
            "--position-tolerance-m",
            _str(args.observe_position_tolerance_m),
            "--rotation-tolerance-rad",
            _str(args.observe_rotation_tolerance_rad),
            "--max-correction-iters",
            _str(args.observe_max_correction_iters),
            "--non-holder-yield-mode",
            args.non_holder_yield_mode,
            "--yield-home-duration-sec",
            _str(args.yield_home_duration_sec),
            "--yield-home-rate-hz",
            _str(args.yield_home_rate_hz),
            "--yield-distance-m",
            _str(args.yield_distance_m),
            "--stop-after-observe",
        ],
    )
    wrist = StageCommand(
        name="move_above_hole_from_wrist",
        argv=[
            sys.executable,
            _str(SCRIPT_DIR / "move_above_hole_from_wrist.py"),
            *_common_rpc_args(args),
            "--any-pose-dir",
            _str(args.any_pose_dir),
            *([] if artifact_dir is None else ["--artifact-dir", _str(artifact_dir / "move_above_hole")]),
            "--side",
            holder_side,
            *_maybe_execute(args),
            "--standoff-m",
            _str(args.hole_standoff_m),
            *wrist_geometry_args,
            "--hole-config",
            _str(hole_config),
            "--rate-hz",
            _str(args.wrist_rate_hz),
            "--max-translation-speed",
            _str(args.wrist_max_translation_speed),
            "--max-rotation-speed",
            _str(args.wrist_max_rotation_speed),
            "--max-translation-step",
            _str(args.wrist_max_translation_step),
            "--max-rotation-step",
            _str(args.wrist_max_rotation_step),
            "--position-tolerance-m",
            _str(args.wrist_position_tolerance_m),
            "--rotation-tolerance-rad",
            _str(args.wrist_rotation_tolerance_rad),
            "--max-correction-iters",
            _str(args.wrist_max_correction_iters),
            "--max-posture-correction-iters",
            _str(args.wrist_max_posture_correction_iters),
            "--wrist-perception-mode",
            args.wrist_perception_mode,
            *([] if args.rack_grid_json is None else ["--rack-grid-json", _str(args.rack_grid_json)]),
            *([] if args.target_slot_id is None else ["--target-slot-id", args.target_slot_id]),
            *([] if args.wrist_camera_socket is None else ["--wrist-camera-socket", _str(args.wrist_camera_socket)]),
            *([] if args.wrist_perception_report is None else ["--wrist-perception-report", _str(args.wrist_perception_report)]),
            *([] if wrist_vlm_response is None else ["--wrist-vlm-response", _str(wrist_vlm_response)]),
        ],
    )
    insert_argv = [
        sys.executable,
        _str(SCRIPT_DIR / "insert_down_from_current_pose.py"),
        *_common_rpc_args(args),
        "--side",
        holder_side,
        *_maybe_execute(args),
        "--insert-depth-m",
        _str(args.insert_depth_m),
        "--near-insert-depth-tolerance-m",
        _str(args.near_insert_depth_tolerance_m),
        "--retract-after-release-m",
        _str(args.retract_after_release_m),
        "--rate-hz",
        _str(args.insert_rate_hz),
        "--max-translation-speed",
        _str(args.insert_max_translation_speed),
        "--max-translation-step",
        _str(args.insert_max_translation_step),
        "--position-tolerance-m",
        _str(args.insert_position_tolerance_m),
        "--settle-time-sec",
        _str(args.insert_settle_time_sec),
        "--max-correction-iters",
        _str(args.insert_max_correction_iters),
        "--retract-max-translation-speed",
        _str(args.retract_max_translation_speed),
        "--retract-max-rotation-speed",
        _str(args.retract_max_rotation_speed),
        "--retract-max-translation-step",
        _str(args.retract_max_translation_step),
        "--retract-max-rotation-step",
        _str(args.retract_max_rotation_step),
        "--retract-settle-time-sec",
        _str(args.retract_settle_time_sec),
    ]
    if args.monitor_insert_force:
        insert_argv.extend(
            [
                "--monitor-force-during-insert",
                "--insert-force-step-m",
                _str(args.insert_force_step_m),
                "--max-insert-force-delta-n",
                _str(args.max_insert_force_delta_n),
                "--max-insert-lateral-force-delta-n",
                _str(args.max_insert_lateral_force_delta_n),
                "--seated-axial-force-delta-n",
                _str(args.seated_axial_force_delta_n),
                "--seated-min-depth-m",
                _str(args.seated_min_depth_m),
            ]
        )
    if args.release_after_insert:
        insert_argv.append("--release-after-insert")
    if args.home_after_insert:
        insert_argv.extend(
            [
                "--home-after-release",
                "--home-duration-sec",
                _str(args.home_after_insert_duration_sec),
                "--home-rate-hz",
                _str(args.home_after_insert_rate_hz),
            ]
        )
    insert = StageCommand(name="insert_release_retract", argv=insert_argv)
    return [observe, wrist, insert]


def _last_json_object(text: str) -> dict[str, Any] | None:
    decoder = json.JSONDecoder()
    result: dict[str, Any] | None = None
    for index, char in enumerate(text):
        if char != "{":
            continue
        try:
            value, end = decoder.raw_decode(text[index:])
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict) and not text[index + end :].strip():
            result = value
    return result


def _tail_text(text: str, *, max_lines: int = 80, max_chars: int = 12000) -> str:
    stripped = text.strip()
    if not stripped:
        return ""
    lines = stripped.splitlines()
    tail = "\n".join(lines[-max_lines:])
    if len(tail) > max_chars:
        tail = tail[-max_chars:]
    return tail


def _format_pose(value: Any) -> str:
    if not isinstance(value, list | tuple) or len(value) < 3:
        return str(value)
    try:
        xyz = [float(value[index]) for index in range(3)]
    except (TypeError, ValueError):
        return str(value)
    return f"x={xyz[0]:+.4f}, y={xyz[1]:+.4f}, z={xyz[2]:+.4f}"


def _argv_value(argv: list[str], option: str) -> str | None:
    try:
        index = argv.index(option)
    except ValueError:
        return None
    if index + 1 >= len(argv):
        return None
    return argv[index + 1]


def _holder_side_from_command(command: StageCommand) -> str | None:
    return _argv_value(command.argv, "--holder-side") or _argv_value(command.argv, "--side")


def _stage_target_for_side(stage: Mapping[str, Any], side: str | None) -> Any:
    if side in ("left", "right"):
        return stage.get(f"{side}_target")
    return stage.get("left_target") or stage.get("right_target")


def _log_report_targets(command: StageCommand, report: Mapping[str, Any] | None) -> None:
    if not report:
        log(f"{command.name}: 没有解析到子脚本 JSON report")
        return

    side = _holder_side_from_command(command)
    if command.name == "observe_rack":
        rack = _mapping(report.get("rack"))
        position = rack.get("position_base_for_motion_m") or rack.get("position_base_m")
        if position is not None:
            log(f"{command.name}: 料架目标位置 base({_format_pose(position)})")
        for stage in report.get("stages", []):
            stage_mapping = _mapping(stage)
            if stage_mapping.get("stage") == "move_holder_to_wrist_observation_pose":
                target = _stage_target_for_side(stage_mapping, side)
                if target is not None:
                    log(f"{command.name}: 观察位目标 base({_format_pose(target)})")
                break
        return

    if command.name == "move_above_hole_from_wrist":
        hole = _mapping(report.get("hole")).get("position_base_m")
        target = report.get("target_pose")
        if hole is not None:
            log(f"{command.name}: 孔位目标 base({_format_pose(hole)})")
        if target is not None:
            log(f"{command.name}: 孔上方移动目标 base({_format_pose(target)})")
        return

    if command.name == "insert_release_retract":
        targets = report.get("insert_target_poses")
        if isinstance(targets, list):
            for index, target in enumerate(targets, start=1):
                log(f"{command.name}: 插入段 {index} 目标 base({_format_pose(target)})")
        retract_target = report.get("retract_target_pose")
        if retract_target is not None:
            log(f"{command.name}: 释放后撤回目标 base({_format_pose(retract_target)})")


def _iter_p2p_final_errors(value: Any, *, seen: set[int] | None = None) -> list[dict[str, Any]]:
    if seen is None:
        seen = set()
    if not isinstance(value, Mapping):
        if isinstance(value, list):
            errors: list[dict[str, Any]] = []
            for item in value:
                errors.extend(_iter_p2p_final_errors(item, seen=seen))
            return errors
        return []

    object_id = id(value)
    if object_id in seen:
        return []
    seen.add(object_id)

    errors: list[dict[str, Any]] = []
    result = _mapping(value.get("result"))
    final_error = _mapping(result.get("final_error"))
    stage_name = value.get("stage")
    if final_error and isinstance(stage_name, str):
        errors.append(
            {
                "stage": stage_name,
                "ok": result.get("ok"),
                "final_error": dict(final_error),
            }
        )

    for child in value.values():
        errors.extend(_iter_p2p_final_errors(child, seen=seen))
    return errors


def _log_p2p_final_errors(command: StageCommand, report: Mapping[str, Any] | None) -> None:
    if not report:
        return
    errors = _iter_p2p_final_errors(report)
    if not errors:
        log(f"{command.name}: 未找到 P2P final_error")
        return
    for index, error in enumerate(errors, start=1):
        log(
            f"{command.name}: P2P {index} {error['stage']} final_error="
            f"{json.dumps(error, ensure_ascii=False, default=str)}"
        )


def _run_command_capture_stdout_stream_stderr(argv: list[str]) -> tuple[int, str, str]:
    process = subprocess.Popen(
        argv,
        cwd=str(SCRIPT_DIR),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    stdout_lines: list[str] = []
    stderr_lines: list[str] = []

    def read_stdout() -> None:
        assert process.stdout is not None
        for line in process.stdout:
            stdout_lines.append(line)

    def read_stderr() -> None:
        assert process.stderr is not None
        for line in process.stderr:
            stderr_lines.append(line)
            print(line, file=sys.stderr, end="", flush=True)
            _write_log_file(line.rstrip("\n"))

    stdout_thread = threading.Thread(target=read_stdout, daemon=True)
    stderr_thread = threading.Thread(target=read_stderr, daemon=True)
    stdout_thread.start()
    stderr_thread.start()
    returncode = process.wait()
    stdout_thread.join()
    stderr_thread.join()
    return int(returncode), "".join(stdout_lines), "".join(stderr_lines)


def print_report(report: Mapping[str, Any], *, compact: bool) -> None:
    text = json.dumps(report, indent=None if compact else 2, ensure_ascii=False, default=str)
    _write_log_file("[manual-grip-insert] final report:")
    _write_log_file(text)
    print(text)


def run_stage(command: StageCommand) -> dict[str, Any]:
    stage_started = time.monotonic()
    description = STAGE_DESCRIPTIONS.get(command.name, command.name)
    log(f"开始阶段 {command.name}: {description}")
    log("执行命令: " + " ".join(command.argv))
    returncode, stdout, stderr = _run_command_capture_stdout_stream_stderr(command.argv)
    report = _last_json_object(stdout)
    log(f"结束阶段 {command.name}: returncode={returncode}")
    _log_report_targets(command, report)
    _log_p2p_final_errors(command, report)
    if report:
        stopped_reason = report.get("stopped_reason")
        if stopped_reason:
            log(f"{command.name}: stopped_reason={stopped_reason}")
    if returncode != 0:
        stdout_tail = _tail_text(stdout)
        stderr_tail = _tail_text(stderr)
        if stdout_tail:
            log(f"{command.name}: stdout tail:\n{stdout_tail}")
        if stderr_tail:
            log(f"{command.name}: stderr tail:\n{stderr_tail}")

    stage_report = {
        "name": command.name,
        "argv": command.argv,
        "returncode": returncode,
        "report": report,
        "elapsed_sec": time.monotonic() - stage_started,
    }
    if returncode != 0:
        stage_report["stdout_tail"] = _tail_text(stdout)
        stage_report["stderr_tail"] = _tail_text(stderr)
    return stage_report


def _mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def _finite_float(value: Any) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def head_rack_plane_z_from_observe_stage(
    stage_report: Mapping[str, Any],
) -> tuple[float | None, str | None]:
    observe_report = _mapping(stage_report.get("report"))
    rack = _mapping(observe_report.get("rack"))

    explicit_plane_z = _finite_float(rack.get("rack_plane_z_base_m"))
    if explicit_plane_z is not None:
        return explicit_plane_z, "rack.rack_plane_z_base_m"

    for field_name in ("position_base_for_motion_m", "position_base_m"):
        position = rack.get(field_name)
        if isinstance(position, (list, tuple)) and len(position) >= 3:
            position_z = _finite_float(position[2])
            if position_z is not None:
                return position_z, f"rack.{field_name}[2]"
    return None, None


def wrist_command_with_head_rack_fallback_z(command: StageCommand, plane_z_base_m: float) -> StageCommand:
    if command.name != "move_above_hole_from_wrist":
        raise ValueError(f"rack plane can only be applied to the wrist stage, got {command.name!r}")
    plane_z = _finite_float(plane_z_base_m)
    if plane_z is None:
        raise ValueError(f"rack plane z must be finite, got {plane_z_base_m!r}")

    argv = list(command.argv)
    source_option = "--hole-plane-z-source"
    try:
        source_index = argv.index(source_option)
    except ValueError as exc:
        raise ValueError(f"wrist command is missing {source_option}") from exc
    if source_index + 1 >= len(argv):
        raise ValueError(f"wrist command has no value after {source_option}")
    argv[source_index + 1] = "detected-depth"

    plane_option = "--rack-plane-z-base-m"
    if plane_option in argv:
        plane_index = argv.index(plane_option)
        if plane_index + 1 >= len(argv):
            raise ValueError(f"wrist command has no value after {plane_option}")
        argv[plane_index + 1] = _str(plane_z)
    else:
        argv[source_index + 2 : source_index + 2] = [plane_option, _str(plane_z)]
    return StageCommand(name=command.name, argv=argv)


def _final_error_from_stage(stage: Mapping[str, Any], side: str) -> dict[str, Any] | None:
    result = _mapping(stage.get("result"))
    final_error = _mapping(result.get("final_error"))
    if not final_error:
        return None

    side_prefix = f"{side}_"
    filtered = {
        key: value
        for key, value in final_error.items()
        if key.startswith(side_prefix) or key.startswith("max_")
    }
    return filtered or dict(final_error)


def _wrist_hole_error(stage_report: Mapping[str, Any], side: str) -> dict[str, Any] | None:
    report = _mapping(stage_report.get("report"))
    arrival_check = _mapping(report.get("move_arrival_check"))
    error = {
        key: arrival_check[key]
        for key in ("xy_error_m", "z_error_m", "rotation_error_rad")
        if key in arrival_check
    }
    final_error = _final_error_from_stage(_mapping(report.get("move_stage")), side)
    if final_error:
        error["final_error"] = final_error
    if error:
        return error

    tcp_xy_error = report.get("tcp_xy_error_to_hole_m")
    if tcp_xy_error is not None:
        return {"tcp_xy_error_to_hole_m": tcp_xy_error}
    return None


def _insert_down_error(stage_report: Mapping[str, Any], side: str) -> dict[str, Any] | None:
    report = _mapping(stage_report.get("report"))
    stages = report.get("stages", [])
    if not isinstance(stages, list):
        return None
    segment_errors: list[dict[str, Any]] = []
    for stage in stages:
        stage_mapping = _mapping(stage)
        if str(stage_mapping.get("stage", "")).startswith(f"guarded_insert_{side}_vertical"):
            samples = stage_mapping.get("samples", [])
            last_sample = _mapping(samples[-1]) if isinstance(samples, list) and samples else {}
            return {
                "guarded_insert": {
                    "ok": stage_mapping.get("ok"),
                    "stopped_reason": stage_mapping.get("stopped_reason"),
                    "insert_success_kind": stage_mapping.get("insert_success_kind"),
                    "sample_count": len(samples) if isinstance(samples, list) else None,
                    "last_sample": last_sample,
                }
            }
        if str(stage_mapping.get("stage", "")).startswith(f"insert_{side}_straight_down"):
            segment_errors.append(
                {
                    "stage": stage_mapping.get("stage"),
                    "final_error": _final_error_from_stage(stage_mapping, side),
                }
            )
    if not segment_errors:
        return None
    return {
        "segments": segment_errors,
        "final_error": segment_errors[-1]["final_error"],
    }


def _print_error(label: str, error: Mapping[str, Any] | None) -> None:
    value: Mapping[str, Any] = error if error is not None else {"unavailable": True}
    print(f"{label}: {json.dumps(value, ensure_ascii=False, default=str)}", flush=True)


def maybe_print_requested_error(stage_report: Mapping[str, Any], side: str) -> None:
    name = stage_report.get("name")
    if name == "move_above_hole_from_wrist":
        _print_error("到达识别孔上方error", _wrist_hole_error(stage_report, side))
    elif name == "insert_release_retract":
        _print_error("向下插入error", _insert_down_error(stage_report, side))


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.insert_max_correction_iters < 0:
        raise ValueError("--insert-max-correction-iters must be non-negative")
    if not math.isfinite(args.insert_settle_time_sec) or args.insert_settle_time_sec < 0.0:
        raise ValueError("--insert-settle-time-sec must be a non-negative finite value")
    set_log_file(
        args.log_file if args.log_file is not None else default_log_file(),
        truncate=args.log_file is None,
    )
    log(
        "启动手动夹持插管流程: "
        f"execute={bool(args.execute)}, holder_side={args.holder_side}, "
        f"server={args.server_host}:{args.server_port}"
    )
    log(f"日志文件: {LOG_FILE}")
    report: dict[str, Any] = {
        "execute": bool(args.execute),
        "server": f"{args.server_host}:{args.server_port}",
        "log_file": str(LOG_FILE),
        "rack_plane_z_base_m_requested": None
        if args.rack_plane_z_base_m is None
        else float(args.rack_plane_z_base_m),
        "requested_holder_side": args.holder_side,
        "stages": [],
    }

    if not args.execute:
        planned_holder_side = args.holder_side if args.holder_side in ("left", "right") else "left"
        report["planned_only"] = True
        report["holder"] = {
            "status": "planned_only",
            "side": planned_holder_side,
            "reason": "execute=false; no gripper state was read and no hardware command was run",
        }
        report["stages"] = [
            {"name": command.name, "argv": command.argv, "returncode": None, "planned_only": True}
            for command in build_stage_commands(args, holder_side=planned_holder_side)
        ]
        report["completion_flag"] = False
        report["physical_verified"] = False
        report["stopped_reason"] = "planned_only_requires_execute_for_hardware"
        print_report(report, compact=args.compact)
        return 0

    try:
        fractions = read_gripper_fractions(args)
    except Exception:
        report["stopped_reason"] = "read_gripper_fractions_failed"
        log("读取夹爪状态失败，完整异常如下:")
        traceback_text = traceback.format_exc()
        print(traceback_text, file=sys.stderr, end="", flush=True)
        _write_log_file(traceback_text.rstrip("\n"))
        print_report(report, compact=args.compact)
        return 1
    log(
        "夹爪闭合比例: "
        f"left={fractions.get('left', 0.0):.3f}, right={fractions.get('right', 0.0):.3f}"
    )
    decision = choose_holder_side_from_gripper_fractions(
        fractions,
        requested_side=args.holder_side,
        min_closed_fraction=args.min_gripper_closed_fraction,
        min_margin=args.min_gripper_margin,
    )
    report["holder"] = decision
    log(
        "夹持手选择结果: "
        f"status={decision['status']}, side={decision['side']}, reason={decision['reason']}"
    )
    if decision["status"] != "ok" or decision["side"] is None:
        report["stopped_reason"] = "holder_side_detection_failed"
        print_report(report, compact=args.compact)
        return 2

    report["rack_plane_z_base_m"] = resolve_rack_plane_z_base_m(args, holder_side=decision["side"])
    report["rack_plane_z_source"] = (
        "cli" if args.rack_plane_z_base_m is not None else "wrist_depth_then_head_fallback_pending"
    )
    if report["rack_plane_z_base_m"] is None:
        log("腕部角点深度优先；若不可靠，将回退到同次头部识别的料架平面 Z")
    else:
        log(
            "使用显式料架平面高度: "
            f"z={report['rack_plane_z_base_m']:+.6f}m, source={report['rack_plane_z_source']}"
        )

    commands = build_stage_commands(args, holder_side=decision["side"])
    insertion_attempts = 0
    for command_index in range(len(commands)):
        command = commands[command_index]
        try:
            stage_report = run_stage(command)
        except Exception:
            report["stopped_reason"] = f"{command.name}_exception"
            log(f"阶段 {command.name} 抛出异常，完整异常如下:")
            traceback_text = traceback.format_exc()
            print(traceback_text, file=sys.stderr, end="", flush=True)
            _write_log_file(traceback_text.rstrip("\n"))
            print_report(report, compact=args.compact)
            return 1
        if command.name == "insert_release_retract":
            insertion_attempts += 1
            stage_report["attempt"] = insertion_attempts
        report["stages"].append(stage_report)
        maybe_print_requested_error(stage_report, decision["side"])
        if stage_report["returncode"] != 0:
            report["stopped_reason"] = f"{command.name}_failed"
            print_report(report, compact=args.compact)
            return int(stage_report["returncode"])

        if command.name == "insert_release_retract":
            insert_report = _mapping(stage_report.get("report"))
            completion_status = insert_report.get("completion_status", "complete")
            report["completion_status"] = completion_status
            report["completion_flag"] = bool(insert_report.get("completion_flag", True))
            if completion_status == "completed_with_insert_tolerance_warning":
                report["insert_tolerance_warning"] = insert_report.get("insert_stage_warning")
                report["achieved_insert_depth_m"] = insert_report.get("achieved_insert_depth_m")
                report["release_confirmed_open"] = insert_report.get("release_confirmed_open")
                log(
                    "下插最终位姿容差未满足，但深度已达到且释放/上提/回 home 完成；"
                    "按带警告完成处理"
                )

        if command.name == "move_above_hole_from_wrist":
            wrist_report = _mapping(stage_report.get("report"))
            wrist_hole = _mapping(wrist_report.get("hole"))
            wrist_plane = _mapping(wrist_hole.get("hole_plane"))
            used_source = wrist_plane.get("source")
            if used_source is not None:
                report["hole_plane_z_source_used"] = used_source
                report["head_rack_plane_fallback_used"] = bool(wrist_plane.get("fallback_used", False))
                log(
                    "腕部孔位深度来源: "
                    f"source={used_source}, "
                    f"head_fallback_used={report['head_rack_plane_fallback_used']}"
                )

        if command.name == "observe_rack" and args.rack_plane_z_base_m is None:
            head_rack_plane_z, head_rack_plane_source = head_rack_plane_z_from_observe_stage(stage_report)
            if head_rack_plane_z is None:
                report["rack_plane_z_source"] = "head_detection_fallback_unavailable"
                log("头部识别报告中没有可用的 rack base Z；继续尝试腕部深度，但无法进行平面回退")
            else:
                report["rack_plane_z_base_m"] = float(head_rack_plane_z)
                report["rack_plane_z_source"] = "head_detection_fallback_available"
                report["rack_plane_z_report_field"] = head_rack_plane_source
                log(
                    "已准备腕部深度失败时使用的头部料架平面: "
                    f"z={head_rack_plane_z:+.6f}m, source={head_rack_plane_source}"
                )
                wrist_index = command_index + 1
                if wrist_index >= len(commands) or commands[wrist_index].name != "move_above_hole_from_wrist":
                    report["stopped_reason"] = "wrist_stage_missing_after_observe"
                    log("observe_rack 后没有找到腕部定位阶段")
                    print_report(report, compact=args.compact)
                    return 2
                commands[wrist_index] = wrist_command_with_head_rack_fallback_z(
                    commands[wrist_index],
                    head_rack_plane_z,
                )

    report.setdefault("completion_status", "complete")
    report.setdefault("completion_flag", True)
    log("流程完成")
    print_report(report, compact=args.compact)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
