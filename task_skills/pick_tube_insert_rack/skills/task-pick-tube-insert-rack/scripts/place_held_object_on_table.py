#!/usr/bin/env python3
"""Move the active arm to a base-frame table point, release, and retract."""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any, Mapping

import numpy as np
from scipy.spatial.transform import Rotation as R


RPC_CLIENT_PATH = Path(
    os.environ.get(
        "DUAL_FRANKA_RPC_CLIENT_PATH",
        "/home/deepcybo/agentic_skills/atomic_skills/dual_franka_p2p/skills/"
        "atomic-motion-franka-move-to-pose/dual_franka_robotiq_rpc_client.py",
    )
)


def _load_rpc_module():
    spec = importlib.util.spec_from_file_location("dual_franka_rpc_client_place", RPC_CLIENT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load RPC client from {RPC_CLIENT_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def parse_xyz(value: str) -> list[float]:
    try:
        parsed = json.loads(value)
    except json.JSONDecodeError as exc:
        raise argparse.ArgumentTypeError(f"xyz must be JSON [x,y,z]: {exc}") from exc
    vector = np.asarray(parsed, dtype=float).reshape(-1)
    if vector.size != 3 or not np.all(np.isfinite(vector)):
        raise argparse.ArgumentTypeError(f"xyz must contain 3 finite numbers, got {parsed!r}")
    return vector.tolist()


def load_place_xyz(result_json: Path) -> list[float]:
    payload = json.loads(result_json.expanduser().read_text(encoding="utf-8"))
    if not payload.get("found"):
        raise ValueError(f"placement locator did not find an empty tabletop area: {result_json}")
    position_base = payload.get("position_base")
    if not isinstance(position_base, Mapping) or not position_base.get("available"):
        raise ValueError(f"placement locator has no available position_base: {result_json}")
    points_base = payload.get("points_base")
    point = points_base.get("bbox_center") if isinstance(points_base, Mapping) else None
    if not isinstance(point, Mapping):
        raise ValueError(f"placement locator has no points_base.bbox_center: {result_json}")
    frame_point: Any = point
    if not all(axis in frame_point for axis in ("x_m", "y_m", "z_m")):
        frame_point = point.get("base")
    if not isinstance(frame_point, Mapping):
        raise ValueError(f"placement bbox_center has no base-frame xyz: {result_json}")
    values = [float(frame_point[axis]) for axis in ("x_m", "y_m", "z_m")]
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"placement bbox_center contains non-finite xyz: {values!r}")
    return values


def _side_key(side: str) -> str:
    return f"{side}_arm"


def _pose_from_observation(rpc: Any, observation: Mapping[str, Any], side: str) -> np.ndarray:
    return np.asarray(rpc._pose_from_side_observation(observation, _side_key(side)), dtype=float)


def _vertical_down_rotvec(current_pose: np.ndarray, down_pitch_rad: float) -> np.ndarray:
    """Keep the current roll/yaw while setting the TCP pitch to the down posture."""
    pose = np.asarray(current_pose, dtype=float).reshape(6)
    rpy = R.from_rotvec(pose[3:]).as_euler("xyz")
    return R.from_euler("xyz", [rpy[0], float(down_pitch_rad), rpy[2]]).as_rotvec()


def _move(
    client: Any,
    *,
    side: str,
    active_target: np.ndarray,
    current_left: np.ndarray,
    current_right: np.ndarray,
    args: argparse.Namespace,
    stage: str,
) -> dict[str, Any]:
    left_target = current_left.copy()
    right_target = current_right.copy()
    if side == "left":
        left_target = active_target.copy()
    else:
        right_target = active_target.copy()
    report: dict[str, Any] = {
        "stage": stage,
        "side": side,
        "left_target": left_target.tolist(),
        "right_target": right_target.tolist(),
        "execute": bool(args.execute),
    }
    if not args.execute:
        report["planned_only"] = True
        return report
    result = client.dual_robot_move_to_ee_pose(
        left_target.tolist(),
        right_target.tolist(),
        delta=False,
        wait=False,
        smooth=True,
        rate_hz=args.rate_hz,
        max_translation_speed=args.max_translation_speed,
        max_rotation_speed=args.max_rotation_speed,
        max_translation_step=args.max_translation_step,
        max_rotation_step=args.max_rotation_step,
        settle_time_sec=args.settle_time_sec,
        position_tolerance_m=args.position_tolerance_m,
        rotation_tolerance_rad=args.rotation_tolerance_rad,
        max_correction_iters=args.max_correction_iters,
        max_steps=args.max_steps,
    )
    report["result"] = result
    if not isinstance(result, Mapping) or not bool(result.get("ok")):
        raise RuntimeError(f"{stage} failed: {result}")
    return report


def _move_with_failure_release(
    client: Any,
    *,
    side: str,
    active_target: np.ndarray,
    current_left: np.ndarray,
    current_right: np.ndarray,
    args: argparse.Namespace,
    stage: str,
    report: dict[str, Any],
) -> dict[str, Any]:
    """Release the held object if a placement move fails, then re-raise the failure."""
    try:
        return _move(
            client,
            side=side,
            active_target=active_target,
            current_left=current_left,
            current_right=current_right,
            args=args,
            stage=stage,
        )
    except Exception as exc:
        if not (args.execute and args.release_on_motion_failure):
            raise
        release_report: dict[str, Any] = {
            "stage": "open_gripper_after_motion_failure",
            "side": side,
            "failed_stage": stage,
            "reason": "placement motion failed; releasing held object as requested",
        }
        try:
            open_result = client.open_gripper(_side_key(side))
            release_report["result"] = open_result
            release_report["ok"] = not (
                isinstance(open_result, Mapping) and open_result.get("ok") is False
            )
        except Exception as release_exc:
            release_report["ok"] = False
            release_report["error"] = repr(release_exc)
        if args.release_sleep_sec > 0.0:
            time.sleep(args.release_sleep_sec)
        if args.go_home_after_motion_failure:
            try:
                home_result = client.go_home(
                    "both",
                    args.failure_home_duration_sec,
                    args.failure_home_rate_hz,
                )
                release_report["go_home_result"] = home_result
                release_report["go_home_ok"] = not (
                    isinstance(home_result, Mapping) and home_result.get("ok") is False
                )
            except Exception as home_exc:
                release_report["go_home_ok"] = False
                release_report["go_home_error"] = repr(home_exc)
        report["stages"].append(release_report)
        report["released_on_motion_failure"] = bool(release_report.get("ok"))
        report["go_home_after_motion_failure"] = bool(release_report.get("go_home_ok"))
        print(
            "open_gripper_after_motion_failure:",
            json.dumps(release_report, indent=None if args.compact else 2, ensure_ascii=False, default=str),
        )
        raise


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    place_group = parser.add_mutually_exclusive_group(required=True)
    place_group.add_argument("--place-xyz", type=parse_xyz, help="TCP xyz in robot base frame at release.")
    place_group.add_argument("--place-result-json", type=Path, help="Object-locator result for an empty tabletop area.")
    parser.add_argument("--place-z-offset-m", type=float, default=0.10, help="Height added above a VLM table-surface point.")
    parser.add_argument(
        "--down-pitch-rad",
        type=float,
        default=0.0,
        help="TCP pitch used for the vertical-down placement posture; 0 matches the grasping down posture.",
    )
    parser.add_argument("--retract-distance-m", type=float, default=0.10)
    parser.add_argument("--keep-current-rotation", action="store_true", help="Preserve the extraction TCP orientation during placement.")
    parser.add_argument("--max-release-rise-m", type=float, default=None,
                        help="Reject release targets above current TCP by more than this distance, before any placement motion.")
    parser.add_argument("--release-sleep-sec", type=float, default=0.6)
    parser.add_argument("--server-host", default=os.environ.get("FRANKA_RPC_HOST", "172.16.0.1"))
    parser.add_argument("--server-port", type=int, default=int(os.environ.get("FRANKA_RPC_PORT", "4242")))
    parser.add_argument("--rpc-timeout-sec", type=float, default=float(os.environ.get("FRANKA_RPC_TIMEOUT_SEC", "30")))
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--max-translation-speed", type=float, default=0.04)
    parser.add_argument("--max-rotation-speed", type=float, default=0.20)
    parser.add_argument("--max-translation-step", type=float, default=0.001)
    parser.add_argument("--max-rotation-step", type=float, default=0.012)
    parser.add_argument("--settle-time-sec", type=float, default=1.0)
    parser.add_argument("--position-tolerance-m", type=float, default=0.005)
    parser.add_argument("--rotation-tolerance-rad", type=float, default=0.035)
    parser.add_argument("--max-correction-iters", type=int, default=2)
    parser.add_argument("--max-steps", type=int, default=3000)
    parser.add_argument(
        "--release-on-motion-failure",
        action="store_true",
        help="Open the active gripper if a placement P2P move fails, then return a failure status.",
    )
    parser.add_argument(
        "--go-home-after-motion-failure",
        action="store_true",
        help="After releasing on a placement motion failure, move both arms to home.",
    )
    parser.add_argument("--failure-home-duration-sec", type=float, default=5.0)
    parser.add_argument("--failure-home-rate-hz", type=float, default=50.0)
    parser.add_argument(
        "--go-home-after-success",
        action="store_true",
        help="After successful release and retract, move both arms to home.",
    )
    parser.add_argument("--success-home-duration-sec", type=float, default=5.0)
    parser.add_argument("--success-home-rate-hz", type=float, default=50.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--compact", action="store_true")
    return parser


def _validate_args(args: argparse.Namespace) -> None:
    for name in (
        "retract_distance_m",
        "release_sleep_sec",
        "place_z_offset_m",
        "down_pitch_rad",
        "rate_hz",
        "max_translation_speed",
        "max_rotation_speed",
        "max_translation_step",
        "max_rotation_step",
        "settle_time_sec",
        "position_tolerance_m",
        "rotation_tolerance_rad",
        "failure_home_duration_sec",
        "failure_home_rate_hz",
        "success_home_duration_sec",
        "success_home_rate_hz",
    ):
        value = float(getattr(args, name))
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"--{name.replace('_', '-')} must be finite and non-negative")
    if args.rate_hz <= 0.0 or args.max_translation_speed <= 0.0 or args.max_rotation_speed <= 0.0:
        raise ValueError("motion rate and speeds must be positive")
    if args.failure_home_rate_hz <= 0.0:
        raise ValueError("--failure-home-rate-hz must be positive")
    if args.success_home_rate_hz <= 0.0:
        raise ValueError("--success-home-rate-hz must be positive")
    if args.max_correction_iters < 0 or args.max_steps <= 0:
        raise ValueError("--max-correction-iters must be non-negative and --max-steps positive")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    _validate_args(args)
    if args.max_release_rise_m is not None and (not math.isfinite(args.max_release_rise_m) or args.max_release_rise_m < 0):
        raise ValueError("--max-release-rise-m must be finite and non-negative")
    place_xyz_values = args.place_xyz
    detected_surface_xyz: list[float] | None = None
    place_source = "cli"
    if args.place_result_json is not None:
        detected_surface_xyz = load_place_xyz(args.place_result_json)
        place_xyz_values = detected_surface_xyz
        place_source = "object_locator"
    place_xyz = np.asarray(place_xyz_values, dtype=float)
    if args.place_result_json is not None:
        place_xyz[2] += float(args.place_z_offset_m)
    report: dict[str, Any] = {
        "execute": bool(args.execute),
        "side": args.side,
        "place_xyz_base_m": place_xyz.tolist(),
        "place_source": place_source,
        "place_result_json": None if args.place_result_json is None else str(args.place_result_json.expanduser()),
        "place_point_source": "points_base.bbox_center.base" if detected_surface_xyz is not None else "cli",
        "detected_surface_xyz_base_m": detected_surface_xyz,
        "place_z_offset_m": float(args.place_z_offset_m) if args.place_result_json is not None else 0.0,
        "placement_orientation": "keep_current" if args.keep_current_rotation else "vertical_down",
        "down_pitch_rad": float(args.down_pitch_rad),
        "retract_distance_m": float(args.retract_distance_m),
        "stages": [],
    }
    if not args.execute:
        report["planned_only"] = True
        report["stages"] = ["move_xy_keep_current_z", "move_to_release_xyz", "open_gripper"]
        if args.retract_distance_m > 0:
            report["stages"].append("retract_up")
        print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False))
        return 0

    rpc = _load_rpc_module()
    client = rpc.DualFrankaRobotiqRpcClient(
        ip=args.server_host,
        port=args.server_port,
        timeout=args.rpc_timeout_sec,
    )
    try:
        report["ping"] = client.ping()
        client.step(None)
        observation = client.get_observation()
        left_current = _pose_from_observation(rpc, observation, "left")
        right_current = _pose_from_observation(rpc, observation, "right")
        active_current = left_current if args.side == "left" else right_current
        if args.max_release_rise_m is not None and place_xyz[2] - active_current[2] > args.max_release_rise_m:
            raise ValueError(f"Release target would rise by {place_xyz[2] - active_current[2]:.4f} m; "
                             "placement cancelled before motion or gripper release")
        vertical_down_rotvec = (active_current[3:].copy() if args.keep_current_rotation else
                                _vertical_down_rotvec(active_current, args.down_pitch_rad))
        report["vertical_down_rotvec"] = vertical_down_rotvec.tolist()

        xy_target = active_current.copy()
        xy_target[:2] = place_xyz[:2]
        xy_target[3:] = vertical_down_rotvec
        report["stages"].append(
            _move_with_failure_release(
                client,
                side=args.side,
                active_target=xy_target,
                current_left=left_current,
                current_right=right_current,
                args=args,
                stage="move_xy_keep_current_z",
                report=report,
            )
        )

        client.step(None)
        observation = client.get_observation()
        left_current = _pose_from_observation(rpc, observation, "left")
        right_current = _pose_from_observation(rpc, observation, "right")
        active_current = left_current if args.side == "left" else right_current
        release_target = active_current.copy()
        release_target[:3] = place_xyz
        release_target[3:] = vertical_down_rotvec
        report["stages"].append(
            _move_with_failure_release(
                client,
                side=args.side,
                active_target=release_target,
                current_left=left_current,
                current_right=right_current,
                args=args,
                stage="move_to_release_xyz",
                report=report,
            )
        )

        open_result = client.open_gripper(_side_key(args.side))
        report["stages"].append({"stage": "open_gripper", "result": open_result})
        if isinstance(open_result, Mapping) and open_result.get("ok") is False:
            raise RuntimeError(f"open gripper failed: {open_result}")
        if args.release_sleep_sec > 0.0:
            time.sleep(args.release_sleep_sec)

        if args.retract_distance_m > 0:
            client.step(None)
            observation = client.get_observation()
            left_current = _pose_from_observation(rpc, observation, "left")
            right_current = _pose_from_observation(rpc, observation, "right")
            active_current = left_current if args.side == "left" else right_current
            retract_target = active_current.copy()
            retract_target[2] += float(args.retract_distance_m)
            report["stages"].append(
                _move_with_failure_release(
                    client,
                    side=args.side,
                    active_target=retract_target,
                    current_left=left_current,
                    current_right=right_current,
                    args=args,
                    stage="retract_up",
                    report=report,
                )
            )
        if args.go_home_after_success:
            home_result = client.go_home(
                "both",
                args.success_home_duration_sec,
                args.success_home_rate_hz,
            )
            report["stages"].append(
                {
                    "stage": "go_home_after_success",
                    "result": home_result,
                }
            )
            if isinstance(home_result, Mapping) and home_result.get("ok") is False:
                raise RuntimeError(f"go_home_after_success failed: {home_result}")
        report["completion_flag"] = True
        report["physical_verified"] = True
        print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
        return 0
    finally:
        client.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
