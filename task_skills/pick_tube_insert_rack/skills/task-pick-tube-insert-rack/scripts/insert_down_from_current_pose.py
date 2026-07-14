#!/usr/bin/env python3
"""Insert straight down from the current pose without running vision.

Use this after the holder arm has already been moved above the selected hole.
The tool preserves the current x/y and orientation, moves only in base z,
optionally opens the gripper, retracts vertically for clearance, and can then
return the released arm to its stored single-arm home.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
from pathlib import Path
import sys
import time
from typing import Any, Mapping

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
SKILL_PATH = SCRIPT_DIR / "tube_insertion_skill.py"
DEFAULT_FIRST_INSERT_SEGMENT_M = 0.015


def _load_skill_module():
    spec = importlib.util.spec_from_file_location("tube_insertion_skill_for_insert_down", SKILL_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load insertion skill from {SKILL_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


skill = _load_skill_module()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server-host", default=os.environ.get("FRANKA_RPC_HOST", "172.16.0.1"))
    parser.add_argument("--server-port", type=int, default=int(os.environ.get("FRANKA_RPC_PORT", "4242")))
    parser.add_argument("--rpc-timeout-sec", type=float, default=float(os.environ.get("FRANKA_RPC_TIMEOUT_SEC", "30")))
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--execute", action="store_true", help="Actually send motion and gripper commands.")
    parser.add_argument("--insert-depth-m", type=float, default=0.01, help="Total downward base-z TCP motion.")
    parser.add_argument(
        "--release-after-insert",
        action="store_true",
        help="Open the selected gripper after the downward move attempt, even if P2P reports failure.",
    )
    parser.add_argument("--after-release-sleep-sec", type=float, default=0.5)
    parser.add_argument(
        "--retract-after-release-m",
        type=float,
        default=0.0,
        help="If positive, move upward by this distance after release.",
    )
    parser.add_argument(
        "--home-after-release",
        action="store_true",
        help="After the vertical clearance retract, return the released arm to its stored single-arm home.",
    )
    parser.add_argument("--home-duration-sec", type=float, default=4.0)
    parser.add_argument("--home-rate-hz", type=float, default=50.0)
    parser.add_argument(
        "--continue-on-p2p-failure",
        action="store_true",
        help="Continue to release/retract even if the P2P stage reports ok=false.",
    )
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--max-translation-speed", type=float, default=0.006)
    parser.add_argument("--max-rotation-speed", type=float, default=0.06)
    parser.add_argument("--max-translation-step", type=float, default=0.0005)
    parser.add_argument("--max-rotation-step", type=float, default=0.003)
    parser.add_argument("--settle-time-sec", type=float, default=0.8)
    parser.add_argument("--retract-max-translation-speed", type=float, default=0.05)
    parser.add_argument("--retract-max-rotation-speed", type=float, default=0.2)
    parser.add_argument("--retract-max-translation-step", type=float, default=0.002)
    parser.add_argument("--retract-max-rotation-step", type=float, default=0.01)
    parser.add_argument("--retract-settle-time-sec", type=float, default=0.5)
    parser.add_argument("--position-tolerance-m", type=float, default=0.004)
    parser.add_argument("--rotation-tolerance-rad", type=float, default=0.05)
    parser.add_argument("--max-correction-iters", type=int, default=2)
    parser.add_argument("--max-steps", type=int, default=5000)
    parser.add_argument("--compact", action="store_true")
    return parser


def _move_args(args: argparse.Namespace, *, motion_profile: str = "insert") -> argparse.Namespace:
    if motion_profile not in ("insert", "retract"):
        raise ValueError(f"unsupported motion profile: {motion_profile!r}")
    retract = motion_profile == "retract"
    return argparse.Namespace(
        rate_hz=float(args.rate_hz),
        max_translation_speed=float(
            args.retract_max_translation_speed if retract else args.max_translation_speed
        ),
        max_rotation_speed=float(
            args.retract_max_rotation_speed if retract else args.max_rotation_speed
        ),
        max_translation_step=float(
            args.retract_max_translation_step if retract else args.max_translation_step
        ),
        max_rotation_step=float(
            args.retract_max_rotation_step if retract else args.max_rotation_step
        ),
        settle_time_sec=float(args.retract_settle_time_sec if retract else args.settle_time_sec),
        position_tolerance_m=float(args.position_tolerance_m),
        rotation_tolerance_rad=float(args.rotation_tolerance_rad),
        max_correction_iters=int(args.max_correction_iters),
        max_steps=int(args.max_steps),
    )


def _force_summary(observation: Mapping[str, Any], side: str) -> dict[str, Any]:
    side_obs = observation.get(f"{side}_arm", {}) if isinstance(observation, Mapping) else {}
    robot_state = side_obs.get("robot_state", {}) if isinstance(side_obs, Mapping) else {}
    wrench = robot_state.get("wrench", {}) if isinstance(robot_state, Mapping) else {}
    force = wrench.get("force")
    if force is None:
        return {"force": None, "force_norm": None}
    arr = np.asarray(force, dtype=float).reshape(3)
    return {"force": arr.tolist(), "force_norm": float(np.linalg.norm(arr))}


def _gripper_closed_fraction(observation: Mapping[str, Any], side: str) -> float | None:
    side_obs = observation.get(f"{side}_arm", {}) if isinstance(observation, Mapping) else {}
    gripper = side_obs.get("gripper", {}) if isinstance(side_obs, Mapping) else {}
    try:
        position = float(gripper["position"])
        open_position = float(gripper["open_position"])
        closed_position = float(gripper["closed_position"])
    except (KeyError, TypeError, ValueError):
        return None
    span = closed_position - open_position
    if not np.isfinite(span) or abs(span) < 1e-9:
        return None
    return float(np.clip((position - open_position) / span, 0.0, 1.0))


def insertion_warning_is_completed(
    *,
    achieved_depth_m: float,
    requested_depth_m: float,
    position_tolerance_m: float,
    release_after_insert: bool,
    release_confirmed_open: bool,
    cleanup_path_completed: bool,
) -> bool:
    depth_floor = max(0.0, float(requested_depth_m) - float(position_tolerance_m))
    return bool(
        np.isfinite(achieved_depth_m)
        and achieved_depth_m >= depth_floor
        and release_after_insert
        and release_confirmed_open
        and cleanup_path_completed
    )


def compute_insert_down_target_pose(*, current_pose: np.ndarray, insert_depth_m: float) -> np.ndarray:
    depth = float(insert_depth_m)
    if not np.isfinite(depth) or depth < 0.0:
        raise ValueError(f"insert_depth_m must be finite and non-negative, got {insert_depth_m!r}")
    target = np.asarray(current_pose, dtype=float).reshape(6).copy()
    target[2] -= depth
    return target


def compute_retract_up_target_pose(*, current_pose: np.ndarray, retract_m: float) -> np.ndarray:
    distance = float(retract_m)
    if not np.isfinite(distance) or distance < 0.0:
        raise ValueError(f"retract_m must be finite and non-negative, got {retract_m!r}")
    target = np.asarray(current_pose, dtype=float).reshape(6).copy()
    target[2] += distance
    return target


def _move_side(
    *,
    client: Any,
    side: str,
    poses: Mapping[str, np.ndarray],
    target: np.ndarray,
    args: argparse.Namespace,
    stage_name: str,
    motion_profile: str = "insert",
) -> dict[str, Any]:
    return skill.move_dual_absolute(
        client=client,
        side=side,
        side_target=target,
        all_current=poses,
        args=_move_args(args, motion_profile=motion_profile),
        stage_name=stage_name,
        execute=bool(args.execute),
    )


def _stage_ok(stage: Mapping[str, Any], *, execute: bool) -> bool:
    if not execute:
        return True
    result = stage.get("result", {}) if isinstance(stage, Mapping) else {}
    return isinstance(result, Mapping) and bool(result.get("ok"))


def should_abort_before_release(
    *,
    insert_stage_ok: bool,
    release_after_insert: bool,
    continue_on_p2p_failure: bool,
) -> bool:
    if insert_stage_ok:
        return False
    # Treat --release-after-insert as a cleanup guarantee. A failed P2P move
    # must not bypass the requested release/retract/home sequence.
    return not bool(release_after_insert or continue_on_p2p_failure)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    insert_depth = float(args.insert_depth_m)
    if not np.isfinite(insert_depth) or insert_depth < 0.0:
        raise ValueError(f"insert_depth_m must be finite and non-negative, got {args.insert_depth_m!r}")
    if args.retract_after_release_m > 0.0 and not args.release_after_insert:
        raise ValueError("--retract-after-release-m requires --release-after-insert")
    if args.home_after_release and not args.release_after_insert:
        raise ValueError("--home-after-release requires --release-after-insert")
    if args.home_after_release and args.retract_after_release_m <= 0.0:
        raise ValueError("--home-after-release requires a positive --retract-after-release-m clearance")
    if not np.isfinite(args.home_duration_sec) or args.home_duration_sec <= 0.0:
        raise ValueError("--home-duration-sec must be a positive finite value")
    if not np.isfinite(args.home_rate_hz) or args.home_rate_hz <= 0.0:
        raise ValueError("--home-rate-hz must be a positive finite value")
    for option_name in (
        "retract_max_translation_speed",
        "retract_max_rotation_speed",
        "retract_max_translation_step",
        "retract_max_rotation_step",
        "retract_settle_time_sec",
    ):
        option_value = float(getattr(args, option_name))
        if not np.isfinite(option_value) or option_value <= 0.0:
            raise ValueError(f"--{option_name.replace('_', '-')} must be a positive finite value")

    if not args.execute:
        segment_depths = [
            min(insert_depth, DEFAULT_FIRST_INSERT_SEGMENT_M),
            max(0.0, insert_depth - DEFAULT_FIRST_INSERT_SEGMENT_M),
        ]
        planned_stages = [
            {"name": f"insert_{args.side}_straight_down_segment_{index}", "depth_m": depth, "planned_only": True}
            for index, depth in enumerate(segment_depths, start=1)
            if depth > 0.0
        ]
        if args.release_after_insert and args.retract_after_release_m > 0.0:
            planned_stages.append(
                {
                    "name": f"retract_{args.side}_after_release_for_clearance",
                    "distance_m": float(args.retract_after_release_m),
                    "max_translation_speed_mps": float(args.retract_max_translation_speed),
                    "planned_only": True,
                }
            )
        if args.home_after_release:
            planned_stages.append(
                {
                    "name": f"home_{args.side}_after_release",
                    "mode": "single_arm_home",
                    "planned_only": True,
                }
            )
        report = {
            "execute": False,
            "planned_only": True,
            "requires_execute_for_rpc": True,
            "requires_execute_for_motion": True,
            "requires_execute_for_gripper": bool(args.release_after_insert),
            "abnormal_robot_state_detected": False,
            "side": args.side,
            "insert_depth_m": insert_depth,
            "insert_segment_depths_m": segment_depths,
            "release_after_insert": bool(args.release_after_insert),
            "retract_after_release_m": float(args.retract_after_release_m),
            "retract_motion": {
                "max_translation_speed_mps": float(args.retract_max_translation_speed),
                "max_rotation_speed_radps": float(args.retract_max_rotation_speed),
                "max_translation_step_m": float(args.retract_max_translation_step),
                "max_rotation_step_rad": float(args.retract_max_rotation_step),
                "settle_time_sec": float(args.retract_settle_time_sec),
            },
            "home_after_release": bool(args.home_after_release),
            "server": f"{args.server_host}:{args.server_port}",
            "stages": planned_stages,
            "release_gripper": (
                {"skipped": True, "reason": "planned_only_requires_execute"}
                if args.release_after_insert
                else {"skipped": True, "reason": "release_after_insert=false"}
            ),
            "completion_flag": False,
            "physical_verified": False,
            "stopped_reason": "planned_only_requires_execute_for_hardware",
        }
        print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
        return 0

    client = skill.DualFrankaRobotiqRpcClient(
        ip=args.server_host,
        port=args.server_port,
        timeout=args.rpc_timeout_sec,
    )
    try:
        report: dict[str, Any] = {
            "execute": bool(args.execute),
            "side": args.side,
            "insert_depth_m": insert_depth,
            "insert_segment_depths_m": [
                min(insert_depth, DEFAULT_FIRST_INSERT_SEGMENT_M),
                max(0.0, insert_depth - DEFAULT_FIRST_INSERT_SEGMENT_M),
            ],
            "release_after_insert": bool(args.release_after_insert),
            "retract_after_release_m": float(args.retract_after_release_m),
            "retract_motion": {
                "max_translation_speed_mps": float(args.retract_max_translation_speed),
                "max_rotation_speed_radps": float(args.retract_max_rotation_speed),
                "max_translation_step_m": float(args.retract_max_translation_step),
                "max_rotation_step_rad": float(args.retract_max_rotation_step),
                "settle_time_sec": float(args.retract_settle_time_sec),
            },
            "home_after_release": bool(args.home_after_release),
            "server": f"{args.server_host}:{args.server_port}",
            "stages": [],
        }
        report["ping"] = client.ping()

        observation, poses = skill.read_ee_poses(client)
        current = poses[args.side].copy()
        report["start_pose"] = current.tolist()
        report["start_force"] = _force_summary(observation, args.side)

        remaining_depth = insert_depth
        segment_depths = [
            min(remaining_depth, DEFAULT_FIRST_INSERT_SEGMENT_M),
            max(0.0, remaining_depth - DEFAULT_FIRST_INSERT_SEGMENT_M),
        ]
        insert_stage_ok = True
        insert_targets: list[list[float]] = []
        for index, segment_depth in enumerate(segment_depths, start=1):
            if segment_depth <= 0.0:
                continue
            insert_target = compute_insert_down_target_pose(
                current_pose=poses[args.side],
                insert_depth_m=segment_depth,
            )
            insert_targets.append(insert_target.tolist())
            insert_stage = _move_side(
                client=client,
                side=args.side,
                poses=poses,
                target=insert_target,
                args=args,
                stage_name=(
                    f"insert_{args.side}_straight_down_segment_{index}_"
                    f"{segment_depth:.3f}m_from_current_pose"
                ),
            )
            report["stages"].append(insert_stage)
            stage_ok = _stage_ok(insert_stage, execute=bool(args.execute))
            insert_stage_ok = bool(insert_stage_ok and stage_ok)
            if not stage_ok:
                break
            observation, poses = skill.read_ee_poses(client)

        report["insert_target_poses"] = insert_targets
        report["insert_target_pose"] = insert_targets[-1] if insert_targets else current.tolist()
        report["insert_stage_ok"] = bool(insert_stage_ok)
        if not insert_stage_ok:
            report["insert_stage_warning"] = "insert_down_stage_failed"
            if args.release_after_insert:
                report["continued_after_insert_failure_for_release"] = True
        if should_abort_before_release(
            insert_stage_ok=insert_stage_ok,
            release_after_insert=args.release_after_insert,
            continue_on_p2p_failure=args.continue_on_p2p_failure,
        ):
            report["stopped_reason"] = "insert_down_stage_failed"
            print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
            return 3

        observation, poses = skill.read_ee_poses(client)
        after_insert = poses[args.side].copy()
        report["after_insert_pose"] = after_insert.tolist()
        achieved_insert_depth_m = float(current[2] - after_insert[2])
        report["achieved_insert_depth_m"] = achieved_insert_depth_m
        report["insert_depth_reached"] = bool(
            achieved_insert_depth_m
            >= max(0.0, insert_depth - float(args.position_tolerance_m))
        )
        report["after_insert_force"] = _force_summary(observation, args.side)

        if args.release_after_insert:
            if args.execute:
                report["release_gripper"] = client.open_gripper(f"{args.side}_arm")
                if args.after_release_sleep_sec > 0.0:
                    time.sleep(float(args.after_release_sleep_sec))
            else:
                report["release_gripper"] = {"skipped": True, "reason": "dry run; pass --execute"}

            observation, poses = skill.read_ee_poses(client)
            if args.retract_after_release_m > 0.0:
                retract_target = compute_retract_up_target_pose(
                    current_pose=poses[args.side],
                    retract_m=args.retract_after_release_m,
                )
                report["retract_target_pose"] = retract_target.tolist()
                retract_stage = _move_side(
                    client=client,
                    side=args.side,
                    poses=poses,
                    target=retract_target,
                    args=args,
                    stage_name=f"retract_{args.side}_after_release",
                    motion_profile="retract",
                )
                report["stages"].append(retract_stage)
                retract_ok = _stage_ok(retract_stage, execute=bool(args.execute))
                if not retract_ok and (args.home_after_release or not args.continue_on_p2p_failure):
                    report["stopped_reason"] = "retract_after_release_failed"
                    print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
                    return 3

            if args.home_after_release:
                home_stage = skill.move_side_to_home(
                    client=client,
                    side=args.side,
                    duration_sec=args.home_duration_sec,
                    rate_hz=args.home_rate_hz,
                    execute=bool(args.execute),
                    stage_name=f"home_{args.side}_after_release",
                )
                report["stages"].append(home_stage)
                if not skill.p2p_stage_succeeded(home_stage):
                    report["stopped_reason"] = "home_after_release_failed"
                    print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
                    return 3

        final_observation, final_poses = skill.read_ee_poses(client)
        report["final_pose"] = final_poses[args.side].tolist()
        report["final_force"] = _force_summary(final_observation, args.side)
        final_closed_fraction = _gripper_closed_fraction(final_observation, args.side)
        report["final_gripper_closed_fraction"] = final_closed_fraction
        report["release_confirmed_open"] = bool(
            final_closed_fraction is not None and final_closed_fraction <= 0.20
        )
        if not insert_stage_ok:
            cleanup_path_completed = bool(args.release_after_insert)
            report["release_cleanup_path_completed"] = cleanup_path_completed
            if insertion_warning_is_completed(
                achieved_depth_m=achieved_insert_depth_m,
                requested_depth_m=insert_depth,
                position_tolerance_m=args.position_tolerance_m,
                release_after_insert=bool(args.release_after_insert),
                release_confirmed_open=bool(report["release_confirmed_open"]),
                cleanup_path_completed=cleanup_path_completed,
            ):
                report["completion_status"] = "completed_with_insert_tolerance_warning"
                report["completion_flag"] = True
                report["physical_verified"] = False
                report["insert_stage_warning"] = (
                    "P2P final pose tolerance was not met, but requested downward depth was reached "
                    "and release/retract/home cleanup completed"
                )
                report["stopped_reason"] = None
                print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
                return 0
            report["stopped_reason"] = "insert_down_stage_failed_after_release_cleanup"
            print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
            return 3
        report["completion_status"] = "complete"
        report["completion_flag"] = True
        print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
        return 0
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
