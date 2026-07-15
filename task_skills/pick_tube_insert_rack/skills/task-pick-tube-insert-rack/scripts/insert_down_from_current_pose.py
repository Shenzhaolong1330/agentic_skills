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
from scipy.spatial.transform import Rotation as R


SCRIPT_DIR = Path(__file__).resolve().parent
SKILL_PATH = SCRIPT_DIR / "tube_insertion_skill.py"
DEFAULT_FIRST_INSERT_SEGMENT_M = 0.015
DEFAULT_NEAR_INSERT_DEPTH_TOLERANCE_M = 0.008


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
        "--near-insert-depth-tolerance-m",
        type=float,
        default=DEFAULT_NEAR_INSERT_DEPTH_TOLERANCE_M,
        help=(
            "Additional depth slack for a force-monitored insertion that completed without a collision. "
            "Such an attempt is released/retracted with a warning instead of retried."
        ),
    )
    parser.add_argument(
        "--monitor-force-during-insert",
        action="store_true",
        help="Descend in small Cartesian steps and inspect wrist force after every step.",
    )
    parser.add_argument("--insert-force-step-m", type=float, default=0.0005)
    parser.add_argument("--max-insert-force-delta-n", type=float, default=12.0)
    parser.add_argument("--max-insert-lateral-force-delta-n", type=float, default=4.0)
    parser.add_argument("--seated-axial-force-delta-n", type=float, default=6.0)
    parser.add_argument("--seated-min-depth-m", type=float, default=0.003)
    parser.add_argument(
        "--release-after-insert",
        action="store_true",
        help="Open the selected gripper after the downward move attempt, even if P2P reports failure.",
    )
    parser.add_argument("--after-release-sleep-sec", type=float, default=0.5)
    parser.add_argument("--retract-after-release-m", type=float, default=0.0)
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
        help="Continue to release even if the downward P2P stage reports ok=false.",
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
    parser.add_argument(
        "--max-correction-iters",
        type=int,
        default=3,
        help="Maximum force-monitored tolerance-correction passes after the initial descent.",
    )
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
    near_depth_tolerance_m: float | None = None,
    release_after_insert: bool,
    release_confirmed_open: bool,
) -> bool:
    effective_tolerance = float(position_tolerance_m)
    if near_depth_tolerance_m is not None:
        effective_tolerance = max(effective_tolerance, float(near_depth_tolerance_m))
    depth_floor = max(0.0, float(requested_depth_m) - effective_tolerance)
    return bool(
        np.isfinite(achieved_depth_m)
        and achieved_depth_m >= depth_floor
        and release_after_insert
        and release_confirmed_open
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
    # The normal release/retract/home sequence is reserved for a confirmed
    # insertion. An actual insertion failure must stop here without upward
    # cleanup motion.
    return not bool(release_after_insert or continue_on_p2p_failure)


def insertion_depth_reached(
    *, achieved_depth_m: float, requested_depth_m: float, position_tolerance_m: float
) -> bool:
    depth_floor = max(0.0, float(requested_depth_m) - float(position_tolerance_m))
    return bool(np.isfinite(achieved_depth_m) and achieved_depth_m >= depth_floor)


def insertion_depth_near_reached(
    *,
    achieved_depth_m: float,
    requested_depth_m: float,
    position_tolerance_m: float,
    near_depth_tolerance_m: float,
) -> bool:
    """Accept a near-complete descent when no collision guard fired.

    The strict P2P tolerance remains available for diagnostics.  This second
    gate is intentionally used only as a completion-with-warning condition;
    it must not turn a collision or a substantially short insertion into a
    successful insertion.
    """
    effective_tolerance = max(float(position_tolerance_m), float(near_depth_tolerance_m))
    return insertion_depth_reached(
        achieved_depth_m=achieved_depth_m,
        requested_depth_m=requested_depth_m,
        position_tolerance_m=effective_tolerance,
    )


def classify_insert_force(
    *,
    force_delta: np.ndarray,
    achieved_depth_m: float,
    max_total_delta_n: float,
    max_lateral_delta_n: float,
    seated_axial_delta_n: float,
    seated_min_depth_m: float,
) -> dict[str, Any]:
    """Classify guarded vertical insertion feedback in the base frame.

    A mostly axial contact after a small amount of insertion is treated as the
    tube seating in the rack.  A large lateral load, or a hard contact before
    the tube has entered the hole, is treated as an insertion collision.
    """
    delta = np.asarray(force_delta, dtype=float).reshape(3)
    total = float(np.linalg.norm(delta))
    lateral = float(np.linalg.norm(delta[:2]))
    axial = float(abs(delta[2]))
    axial_fraction = float(axial / total) if total > 1e-9 else 0.0
    seated = bool(
        achieved_depth_m >= seated_min_depth_m
        and axial >= seated_axial_delta_n
        and lateral <= max_lateral_delta_n
        and axial_fraction >= 0.8
    )
    if lateral > max_lateral_delta_n:
        classification = "lateral_collision"
    elif total > max_total_delta_n:
        classification = "hard_collision"
    elif achieved_depth_m < seated_min_depth_m and axial >= seated_axial_delta_n:
        classification = "early_axial_collision"
    elif seated:
        classification = "seated_axial_contact"
    else:
        classification = "continue"
    return {
        "classification": classification,
        "force_delta_n": delta.tolist(),
        "force_delta_norm_n": total,
        "lateral_force_delta_n": lateral,
        "axial_force_delta_n": axial,
        "axial_fraction": axial_fraction,
    }


def pose_error_to_target(*, current_pose: np.ndarray, target_pose: np.ndarray) -> dict[str, Any]:
    current = np.asarray(current_pose, dtype=float).reshape(6)
    target = np.asarray(target_pose, dtype=float).reshape(6)
    translation_delta = target[:3] - current[:3]
    rotation_delta = (
        R.from_rotvec(target[3:]) * R.from_rotvec(current[3:]).inv()
    ).as_rotvec()
    return {
        "translation_delta": translation_delta,
        "rotation_delta": rotation_delta,
        "translation_error_m": float(np.linalg.norm(translation_delta)),
        "rotation_error_rad": float(np.linalg.norm(rotation_delta)),
    }


def pose_error_within_tolerance(
    error: Mapping[str, Any], *, position_tolerance_m: float, rotation_tolerance_rad: float
) -> bool:
    return bool(
        float(error["translation_error_m"]) <= float(position_tolerance_m)
        and float(error["rotation_error_rad"]) <= float(rotation_tolerance_rad)
    )


def guarded_motion_step_limits(args: argparse.Namespace) -> tuple[float, float]:
    """Return per-cycle limits consistent with both configured speed and step caps."""
    rate_hz = float(args.rate_hz)
    translation_step = min(
        float(args.insert_force_step_m),
        float(args.max_translation_step),
        float(args.max_translation_speed) / rate_hz,
    )
    rotation_step = min(
        float(args.max_rotation_step),
        float(args.max_rotation_speed) / rate_hz,
    )
    return translation_step, rotation_step


def guarded_vertical_insert(
    *,
    client: Any,
    side: str,
    start_pose: np.ndarray,
    base_force: np.ndarray,
    depth_m: float,
    args: argparse.Namespace,
) -> dict[str, Any]:
    """Force-monitored absolute-pose descent with P2P-style settle/corrections."""
    start = np.asarray(start_pose, dtype=float).reshape(6)
    target = compute_insert_down_target_pose(current_pose=start, insert_depth_m=depth_m)
    base_force = np.asarray(base_force, dtype=float).reshape(3)
    action_key = f"{side}_arm"
    period_sec = 1.0 / float(args.rate_hz)
    translation_step, rotation_step = guarded_motion_step_limits(args)
    max_correction_iters = int(args.max_correction_iters)
    max_steps = int(args.max_steps)
    samples: list[dict[str, Any]] = []
    correction_reports: list[dict[str, Any]] = []
    last_step: Any = None
    total_steps = 0
    current = start.copy()

    def sample_feedback(*, correction_index: int, sample_kind: str) -> tuple[np.ndarray, dict[str, Any]]:
        observation, poses = skill.read_ee_poses(client)
        pose = np.asarray(poses[side], dtype=float).reshape(6)
        force = np.asarray(
            observation[action_key]["robot_state"]["wrench"]["force"], dtype=float
        ).reshape(3)
        achieved_depth = float(start[2] - pose[2])
        force_state = classify_insert_force(
            force_delta=force - base_force,
            achieved_depth_m=achieved_depth,
            max_total_delta_n=float(args.max_insert_force_delta_n),
            max_lateral_delta_n=float(args.max_insert_lateral_force_delta_n),
            seated_axial_delta_n=float(args.seated_axial_force_delta_n),
            seated_min_depth_m=float(args.seated_min_depth_m),
        )
        samples.append(
            {
                "index": len(samples) + 1,
                "correction_index": correction_index,
                "sample_kind": sample_kind,
                "pose": pose.tolist(),
                "achieved_depth_m": achieved_depth,
                "force": force.tolist(),
                **force_state,
            }
        )
        return pose, force_state

    def force_terminal_result(
        *, force_state: Mapping[str, Any], correction_index: int
    ) -> dict[str, Any] | None:
        classification = str(force_state["classification"])
        common = {
            "stage": f"guarded_insert_{side}_vertical",
            "execute": True,
            "target_pose": target.tolist(),
            "samples": samples,
            "correction_reports": correction_reports,
            "max_correction_iters": max_correction_iters,
            "correction_index": correction_index,
            "total_motion_steps": total_steps,
            "last_step": last_step,
        }
        if classification == "seated_axial_contact":
            return {
                **common,
                "ok": True,
                "insert_success_kind": classification,
                "contact_detected": True,
            }
        if classification in ("lateral_collision", "hard_collision", "early_axial_collision"):
            return {
                **common,
                "ok": False,
                "stopped_reason": classification,
                "force_guard_failure": True,
                "contact_detected": True,
            }
        return None

    for correction_index in range(max_correction_iters + 1):
        start_error = pose_error_to_target(current_pose=current, target_pose=target)
        if pose_error_within_tolerance(
            start_error,
            position_tolerance_m=float(args.position_tolerance_m),
            rotation_tolerance_rad=float(args.rotation_tolerance_rad),
        ):
            correction_reports.append(
                {
                    "correction_index": correction_index,
                    "skipped": True,
                    "reason": "already_within_tolerance",
                    "final_error": {
                        key: value.tolist() if isinstance(value, np.ndarray) else value
                        for key, value in start_error.items()
                    },
                }
            )
            break

        translation_error = np.asarray(start_error["translation_delta"], dtype=float)
        rotation_error = np.asarray(start_error["rotation_delta"], dtype=float)
        translation_steps = int(
            np.ceil(float(start_error["translation_error_m"]) / translation_step)
        )
        rotation_steps = int(np.ceil(float(start_error["rotation_error_rad"]) / rotation_step))
        planned_steps = max(1, translation_steps, rotation_steps)
        remaining_step_budget = max_steps - total_steps
        if remaining_step_budget <= 0:
            break
        executed_steps = min(planned_steps, remaining_step_budget)
        translation_command = translation_error / float(planned_steps)
        rotation_command = rotation_error / float(planned_steps)
        pass_report: dict[str, Any] = {
            "correction_index": correction_index,
            "planned_steps": planned_steps,
            "executed_steps": executed_steps,
            "translation_step_limit_m": translation_step,
            "rotation_step_limit_rad": rotation_step,
            "start_error": {
                key: value.tolist() if isinstance(value, np.ndarray) else value
                for key, value in start_error.items()
            },
        }
        deadline = time.monotonic()
        for step_index in range(executed_steps):
            last_step = client.step(
                {
                    action_key: {
                        "motion": {
                            "translation": translation_command.tolist(),
                            "rotation_rotvec": rotation_command.tolist(),
                        }
                    }
                }
            )
            total_steps += 1
            if step_index + 1 < executed_steps:
                deadline += period_sec
                time.sleep(max(0.0, deadline - time.monotonic()))
            current, force_state = sample_feedback(
                correction_index=correction_index, sample_kind="motion"
            )
            terminal = force_terminal_result(
                force_state=force_state, correction_index=correction_index
            )
            if terminal is not None:
                pass_report["stopped_at_step"] = step_index + 1
                correction_reports.append(pass_report)
                terminal["correction_reports"] = correction_reports
                return terminal

        if float(args.settle_time_sec) > 0.0:
            time.sleep(float(args.settle_time_sec))
        current, force_state = sample_feedback(
            correction_index=correction_index, sample_kind="settle"
        )
        final_error = pose_error_to_target(current_pose=current, target_pose=target)
        pass_report["settle_time_sec"] = float(args.settle_time_sec)
        pass_report["final_error"] = {
            key: value.tolist() if isinstance(value, np.ndarray) else value
            for key, value in final_error.items()
        }
        pass_report["within_tolerance"] = pose_error_within_tolerance(
            final_error,
            position_tolerance_m=float(args.position_tolerance_m),
            rotation_tolerance_rad=float(args.rotation_tolerance_rad),
        )
        correction_reports.append(pass_report)
        terminal = force_terminal_result(
            force_state=force_state, correction_index=correction_index
        )
        if terminal is not None:
            terminal["correction_reports"] = correction_reports
            return terminal
        if pass_report["within_tolerance"]:
            break

    final_error = pose_error_to_target(current_pose=current, target_pose=target)
    pose_reached = pose_error_within_tolerance(
        final_error,
        position_tolerance_m=float(args.position_tolerance_m),
        rotation_tolerance_rad=float(args.rotation_tolerance_rad),
    )
    achieved_depth = float(start[2] - current[2])
    reached = insertion_depth_reached(
        achieved_depth_m=achieved_depth,
        requested_depth_m=float(depth_m),
        position_tolerance_m=float(args.position_tolerance_m),
    )
    near_reached = insertion_depth_near_reached(
        achieved_depth_m=achieved_depth,
        requested_depth_m=float(depth_m),
        position_tolerance_m=float(args.position_tolerance_m),
        near_depth_tolerance_m=float(args.near_insert_depth_tolerance_m),
    )
    accepted = bool(reached or near_reached)
    return {
        "stage": f"guarded_insert_{side}_vertical",
        "execute": True,
        "ok": pose_reached,
        "target_pose": target.tolist(),
        "insert_success_kind": (
            "commanded_pose_reached"
            if pose_reached
            else "commanded_depth_reached"
            if reached
            else "commanded_depth_near_reached"
            if near_reached
            else None
        ),
        "stopped_reason": (
            None
            if pose_reached
            else "guarded_pose_tolerance_not_reached"
            if accepted
            else "guarded_depth_not_reached"
        ),
        "force_guard_failure": not accepted,
        "insert_depth_reached": bool(reached),
        "insert_depth_near_reached": bool(near_reached),
        "pose_tolerance_reached": bool(pose_reached),
        "final_error": {
            key: value.tolist() if isinstance(value, np.ndarray) else value
            for key, value in final_error.items()
        },
        "translation_step_limit_m": translation_step,
        "rotation_step_limit_rad": rotation_step,
        "settle_time_sec": float(args.settle_time_sec),
        "max_correction_iters": max_correction_iters,
        "correction_reports": correction_reports,
        "samples": samples,
        "last_step": last_step,
        "total_motion_steps": total_steps,
        "achieved_depth_m": achieved_depth,
    }


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
    for option_name in (
        "insert_force_step_m",
        "max_insert_force_delta_n",
        "max_insert_lateral_force_delta_n",
        "seated_axial_force_delta_n",
        "seated_min_depth_m",
        "near_insert_depth_tolerance_m",
        "rate_hz",
        "max_translation_speed",
        "max_rotation_speed",
        "max_translation_step",
        "max_rotation_step",
        "position_tolerance_m",
        "rotation_tolerance_rad",
    ):
        option_value = float(getattr(args, option_name))
        if not np.isfinite(option_value) or option_value <= 0.0:
            raise ValueError(f"--{option_name.replace('_', '-')} must be a positive finite value")
    if not np.isfinite(args.settle_time_sec) or args.settle_time_sec < 0.0:
        raise ValueError("--settle-time-sec must be a non-negative finite value")
    if args.max_correction_iters < 0:
        raise ValueError("--max-correction-iters must be non-negative")
    if args.max_steps <= 0:
        raise ValueError("--max-steps must be positive")
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
        if args.monitor_force_during_insert:
            planned_stages = [
                {
                    "name": f"guarded_insert_{args.side}_vertical",
                    "depth_m": insert_depth,
                    "step_m": float(args.insert_force_step_m),
                    "effective_translation_step_m": guarded_motion_step_limits(args)[0],
                    "settle_time_sec": float(args.settle_time_sec),
                    "max_correction_iters": int(args.max_correction_iters),
                    "planned_only": True,
                }
            ]
        else:
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
            "near_insert_depth_tolerance_m": float(args.near_insert_depth_tolerance_m),
            "monitor_force_during_insert": bool(args.monitor_force_during_insert),
            "insert_force_guard": {
                "step_m": float(args.insert_force_step_m),
                "effective_translation_step_m": guarded_motion_step_limits(args)[0],
                "max_total_delta_n": float(args.max_insert_force_delta_n),
                "max_lateral_delta_n": float(args.max_insert_lateral_force_delta_n),
                "seated_axial_delta_n": float(args.seated_axial_force_delta_n),
                "seated_min_depth_m": float(args.seated_min_depth_m),
                "settle_time_sec": float(args.settle_time_sec),
                "max_correction_iters": int(args.max_correction_iters),
            },
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
            "near_insert_depth_tolerance_m": float(args.near_insert_depth_tolerance_m),
            "monitor_force_during_insert": bool(args.monitor_force_during_insert),
            "insert_force_guard": {
                "step_m": float(args.insert_force_step_m),
                "effective_translation_step_m": guarded_motion_step_limits(args)[0],
                "max_total_delta_n": float(args.max_insert_force_delta_n),
                "max_lateral_delta_n": float(args.max_insert_lateral_force_delta_n),
                "seated_axial_delta_n": float(args.seated_axial_force_delta_n),
                "seated_min_depth_m": float(args.seated_min_depth_m),
                "settle_time_sec": float(args.settle_time_sec),
                "max_correction_iters": int(args.max_correction_iters),
            },
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
        if args.monitor_force_during_insert and report["start_force"]["force"] is None:
            raise RuntimeError("force-monitored insertion requires wrist wrench.force feedback")

        insert_stage_ok = True
        insert_targets: list[list[float]] = []
        guarded_success_kind: str | None = None
        guarded_force_failure = False
        if args.monitor_force_during_insert:
            guarded_stage = guarded_vertical_insert(
                client=client,
                side=args.side,
                start_pose=current,
                base_force=np.asarray(report["start_force"]["force"], dtype=float),
                depth_m=insert_depth,
                args=args,
            )
            report["stages"].append(guarded_stage)
            insert_stage_ok = bool(guarded_stage.get("ok"))
            guarded_success_kind = guarded_stage.get("insert_success_kind")
            guarded_force_failure = bool(guarded_stage.get("force_guard_failure"))
        else:
            remaining_depth = insert_depth
            segment_depths = [
                min(remaining_depth, DEFAULT_FIRST_INSERT_SEGMENT_M),
                max(0.0, remaining_depth - DEFAULT_FIRST_INSERT_SEGMENT_M),
            ]
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
        observation, poses = skill.read_ee_poses(client)
        after_insert = poses[args.side].copy()
        report["after_insert_pose"] = after_insert.tolist()
        achieved_insert_depth_m = float(current[2] - after_insert[2])
        report["achieved_insert_depth_m"] = achieved_insert_depth_m
        report["insert_depth_reached"] = insertion_depth_reached(
            achieved_depth_m=achieved_insert_depth_m,
            requested_depth_m=insert_depth,
            position_tolerance_m=float(args.position_tolerance_m),
        )
        report["insert_depth_near_reached"] = insertion_depth_near_reached(
            achieved_depth_m=achieved_insert_depth_m,
            requested_depth_m=insert_depth,
            position_tolerance_m=float(args.position_tolerance_m),
            near_depth_tolerance_m=float(args.near_insert_depth_tolerance_m),
        )
        report["insert_success_kind"] = guarded_success_kind
        report["insert_success_confirmed"] = bool(
            not guarded_force_failure
            and (
                report["insert_depth_reached"]
                or report["insert_depth_near_reached"]
                or guarded_success_kind == "seated_axial_contact"
            )
        )
        report["after_insert_force"] = _force_summary(observation, args.side)
        release_after_success = bool(
            args.release_after_insert and report["insert_success_confirmed"]
        )
        report["release_after_insert_executed"] = release_after_success

        if should_abort_before_release(
            insert_stage_ok=insert_stage_ok,
            release_after_insert=release_after_success,
            continue_on_p2p_failure=False,
        ):
            report["stopped_reason"] = "insert_down_stage_failed"
            print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
            return 3

        if release_after_success:
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
                if not retract_ok:
                    report.setdefault("cleanup_warnings", []).append(
                        "retract_after_release_p2p_failed"
                    )

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
                    report.setdefault("cleanup_warnings", []).append(
                        "home_after_release_p2p_failed"
                    )

        final_observation, final_poses = skill.read_ee_poses(client)
        report["final_pose"] = final_poses[args.side].tolist()
        report["final_force"] = _force_summary(final_observation, args.side)
        final_closed_fraction = _gripper_closed_fraction(final_observation, args.side)
        report["final_gripper_closed_fraction"] = final_closed_fraction
        report["release_confirmed_open"] = bool(
            final_closed_fraction is not None and final_closed_fraction <= 0.20
        )
        if not insert_stage_ok:
            if insertion_warning_is_completed(
                achieved_depth_m=achieved_insert_depth_m,
                requested_depth_m=insert_depth,
                position_tolerance_m=args.position_tolerance_m,
                near_depth_tolerance_m=args.near_insert_depth_tolerance_m,
                release_after_insert=bool(args.release_after_insert),
                release_confirmed_open=bool(report["release_confirmed_open"]),
            ):
                report["completion_status"] = "completed_with_insert_tolerance_warning"
                report["completion_flag"] = True
                report["physical_verified"] = False
                report["insert_stage_warning"] = (
                    "Strict P2P/depth tolerance was not met, but insertion was near-complete without "
                    "a force-guard collision; release/retract/home cleanup was attempted"
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
