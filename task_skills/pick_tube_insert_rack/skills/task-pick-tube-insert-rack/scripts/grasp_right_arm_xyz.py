#!/usr/bin/env python3
"""Single-arm grasp helper driven by an absolute torso-frame xyz target."""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation as R

DEFAULT_CLIENT_PATH = Path(
    os.environ.get(
        "DUAL_FRANKA_RPC_CLIENT_PATH",
        "/home/deepcybo/agentic_skills/atomic_skills/dual_franka_p2p/skills/"
        "atomic-motion-franka-move-to-pose/dual_franka_robotiq_rpc_client.py",
    )
)

RESULT_GRASP_POINTS = (
    "bbox_center",
    "head",
    "tail",
    "tail_to_head_1_5",
    "tail_to_head_1_4",
    "tail_to_head_1_3",
)

_RPC_CLIENT_PATH = DEFAULT_CLIENT_PATH
_RPC_CLIENT_SPEC = importlib.util.spec_from_file_location("dual_franka_rpc_client_grasp", _RPC_CLIENT_PATH)
if _RPC_CLIENT_SPEC is None or _RPC_CLIENT_SPEC.loader is None:
    raise RuntimeError(f"Failed to load RPC client module from {_RPC_CLIENT_PATH}")
franka_rpc = importlib.util.module_from_spec(_RPC_CLIENT_SPEC)
_RPC_CLIENT_SPEC.loader.exec_module(franka_rpc)

DualFrankaRobotiqRpcClient = franka_rpc.DualFrankaRobotiqRpcClient
_absolute_target_to_delta = franka_rpc._absolute_target_to_delta
_pose_from_side_observation = franka_rpc._pose_from_side_observation

np.set_printoptions(precision=6, suppress=True)


def parse_xyz(value: str) -> np.ndarray:
    try:
        parsed = json.loads(value)
    except json.JSONDecodeError as exc:
        raise argparse.ArgumentTypeError(f"xyz must be a JSON list of 3 numbers: {exc}") from exc
    xyz = np.asarray(parsed, dtype=float).reshape(-1)
    if xyz.size != 3 or not np.all(np.isfinite(xyz)):
        raise argparse.ArgumentTypeError(f"xyz must contain exactly 3 finite numbers, got {parsed!r}")
    return xyz


def parse_vector3(value: str) -> np.ndarray:
    try:
        parsed = json.loads(value)
    except json.JSONDecodeError as exc:
        raise argparse.ArgumentTypeError(f"vector must be a JSON list of 3 numbers: {exc}") from exc
    vector = np.asarray(parsed, dtype=float).reshape(-1)
    if vector.size != 3 or not np.all(np.isfinite(vector)):
        raise argparse.ArgumentTypeError(f"vector must contain exactly 3 finite numbers, got {parsed!r}")
    norm = float(np.linalg.norm(vector))
    if norm <= 1e-9:
        raise argparse.ArgumentTypeError(f"vector must have non-zero length, got {parsed!r}")
    return vector / norm


def parse_arm_side(value: str) -> str:
    normalized = value.strip().lower()
    aliases = {
        "l": "left_arm",
        "left": "left_arm",
        "left_arm": "left_arm",
        "r": "right_arm",
        "right": "right_arm",
        "right_arm": "right_arm",
    }
    try:
        return aliases[normalized]
    except KeyError as exc:
        raise argparse.ArgumentTypeError("arm must be one of: left, left_arm, right, right_arm") from exc


def parse_nonnegative_int(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"expected an integer, got {value!r}") from exc
    if parsed < 0:
        raise argparse.ArgumentTypeError(f"expected a non-negative integer, got {parsed}")
    return parsed


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Move one Franka arm to xyz in stages, point gripper down, then close the gripper.",
    )
    parser.add_argument("--xyz", type=parse_xyz, help='Target xyz, e.g. "[0.44, -0.11, -0.23]".')
    parser.add_argument("--arm", type=parse_arm_side, default="right_arm", help="Grasping arm: left or right.")
    parser.add_argument(
        "--result-json",
        type=Path,
        help="object-locator result JSON. Uses --result-grasp-point for xyz and points_base.head-tail for orientation.",
    )
    parser.add_argument(
        "--result-grasp-point",
        choices=RESULT_GRASP_POINTS,
        default="tail",
        help="points_base key to use as grasp xyz when --result-json is passed.",
    )
    parser.add_argument(
        "--result-base-frame",
        choices=("base", "baseright", "baseleft"),
        default="base",
        help="Coordinate frame to read from result JSON points_base when multiple base extrinsics are present.",
    )
    parser.add_argument(
        "--no-result-orientation",
        action="store_true",
        help="When --result-json is passed, use only its xyz target and do not align gripper yaw to points_base.head-tail.",
    )
    parser.add_argument("--server-host", default=os.environ.get("FRANKA_RPC_HOST", "172.16.0.1"))
    parser.add_argument("--server-port", type=int, default=int(os.environ.get("FRANKA_RPC_PORT", "4242")))
    parser.add_argument("--rpc-timeout-sec", type=float, default=float(os.environ.get("FRANKA_RPC_TIMEOUT_SEC", "30")))
    parser.add_argument("--rate_hz", "--rate-hz", dest="rate_hz", type=float, default=50.0)
    parser.add_argument("--max_translation_speed", "--max-translation-speed", dest="max_translation_speed", type=float, default=0.04)
    parser.add_argument("--max_rotation_speed", "--max-rotation-speed", dest="max_rotation_speed", type=float, default=0.20)
    parser.add_argument("--max_translation_step", "--max-translation-step", dest="max_translation_step", type=float, default=0.001)
    parser.add_argument("--max_rotation_step", "--max-rotation-step", dest="max_rotation_step", type=float, default=0.012)
    parser.add_argument("--settle_time_sec", "--settle-time-sec", dest="settle_time_sec", type=float, default=2.0)
    parser.add_argument(
        "--approach-max-translation-speed",
        type=float,
        default=None,
        help="Stage3 grasp descent speed. Defaults to --max-translation-speed when omitted.",
    )
    parser.add_argument(
        "--approach-max-rotation-speed",
        type=float,
        default=None,
        help="Stage3 grasp descent rotation speed. Defaults to --max-rotation-speed when omitted.",
    )
    parser.add_argument(
        "--approach-max-translation-step",
        type=float,
        default=None,
        help="Stage3 grasp descent translation step. Defaults to --max-translation-step when omitted.",
    )
    parser.add_argument(
        "--approach-max-rotation-step",
        type=float,
        default=None,
        help="Stage3 grasp descent rotation step. Defaults to --max-rotation-step when omitted.",
    )
    parser.add_argument(
        "--approach-settle-time-sec",
        type=float,
        default=None,
        help="Stage3 grasp descent settle time. Defaults to --settle-time-sec when omitted.",
    )
    parser.add_argument(
        "--grasp-arrival-observed-z-offset-m",
        type=float,
        default=0.0,
        help=(
            "Offset added only to the active arm's observed z when checking stage3 grasp arrival. "
            "This does not change the commanded target or any other stage."
        ),
    )
    parser.add_argument(
        "--grasp-target-z-offset-m",
        type=float,
        default=0.0,
        help=(
            "Offset added to the commanded grasp TCP z after loading --xyz/--result-json. "
            "Use a positive value to account for fingers extending below the TCP."
        ),
    )
    parser.add_argument("--position_tolerance_m", "--position-tolerance-m", dest="position_tolerance_m", type=float, default=0.003)
    parser.add_argument("--rotation_tolerance_rad", "--rotation-tolerance-rad", dest="rotation_tolerance_rad", type=float, default=0.03)
    parser.add_argument(
        "--transition-rotation-tolerance-rad",
        type=float,
        help="Stage4-only rotation tolerance. Defaults to --rotation-tolerance-rad when omitted.",
    )
    parser.add_argument("--max_correction_iters", "--max-correction-iters", dest="max_correction_iters", type=int, default=2)
    parser.add_argument(
        "--p2p-retries",
        type=parse_nonnegative_int,
        default=1,
        help="Full same-target P2P retries after an attempt finishes outside the configured pose tolerances.",
    )
    parser.add_argument(
        "--recover-stalled-controller",
        action="store_true",
        help=(
            "Before grasping, restart the active arm Cartesian controller when a failed P2P attempt "
            "produces essentially no observed motion, then retry the same absolute target."
        ),
    )
    parser.add_argument(
        "--controller-stall-translation-epsilon-m",
        type=float,
        default=0.0005,
        help="Maximum observed translation counted as no progress for controller-stall recovery.",
    )
    parser.add_argument(
        "--controller-stall-rotation-epsilon-rad",
        type=float,
        default=0.005,
        help="Maximum observed rotation counted as no progress for controller-stall recovery.",
    )
    parser.add_argument(
        "--controller-recovery-settle-time-sec",
        type=float,
        default=1.0,
        help="Wait after recover_robot restarts the selected Cartesian controller.",
    )
    parser.add_argument(
        "--max-stalled-correction-iters",
        type=parse_nonnegative_int,
        default=1,
        help=(
            "Return early from an active-arm P2P attempt after this many consecutive no-progress "
            "observations, so controller recovery does not wait for every tolerance correction. "
            "Use 0 to disable early stall return."
        ),
    )
    parser.add_argument("--max_steps", "--max-steps", dest="max_steps", type=int, default=3000)
    parser.add_argument("--down_pitch_rad", "--down-pitch-rad", dest="down_pitch_rad", type=float, default=0.0)
    parser.add_argument(
        "--orientation-vector",
        type=parse_vector3,
        help='Base-frame object direction vector to align gripper with, e.g. "[0.08, 0.99, 0.12]".',
    )
    parser.add_argument(
        "--gripper-parallel-axis",
        choices=("x", "y"),
        default="x",
        help="End-effector local axis to make parallel with --orientation-vector in the base-frame xy plane.",
    )
    parser.add_argument(
        "--directional-orientation",
        action="store_true",
        help="Match orientation-vector direction exactly instead of allowing a 180 degree parallel flip.",
    )
    parser.add_argument(
        "--keep-current-rotation",
        action="store_true",
        help="Keep the active arm's current end-effector rotation instead of pitching down or aligning to orientation.",
    )
    parser.add_argument("--skip-z", action="store_true", help="Do not move to target z before closing.")
    parser.add_argument("--no-close", action="store_true", help="Run all motion stages but do not close the gripper.")
    parser.add_argument(
        "--gripper-close-timeout-sec",
        type=float,
        default=3.0,
        help="Wait this long for gripper position feedback after each close command.",
    )
    parser.add_argument(
        "--gripper-close-poll-sec",
        type=float,
        default=0.1,
        help="Polling period while confirming that a close command physically moved the gripper.",
    )
    parser.add_argument(
        "--gripper-close-min-fraction",
        type=float,
        default=0.35,
        help="Minimum normalized closed position required before continuing (0=open, 1=fully closed).",
    )
    parser.add_argument(
        "--gripper-close-retries",
        type=parse_nonnegative_int,
        default=1,
        help="Close-command retries after feedback does not reach --gripper-close-min-fraction.",
    )
    parser.add_argument("--after-close-sleep-sec", type=float, default=0.5)
    parser.add_argument(
        "--after-partner-close-sleep-sec",
        "--after-left-close-sleep-sec",
        dest="after_partner_close_sleep_sec",
        type=float,
        default=0.5,
    )
    parser.add_argument(
        "--after-active-open-sleep-sec",
        "--after-right-open-sleep-sec",
        dest="after_active_open_sleep_sec",
        type=float,
        default=0.5,
    )
    parser.add_argument("--transition-json", type=Path, default=Path(__file__).resolve().with_name("transition.json"))
    parser.add_argument("--no-return-transition", action="store_true", help="Do not move back to transition.json after grasp.")
    parser.add_argument(
        "--go-home-before-transition",
        action="store_true",
        help="Move both arms to the server home pose after grasping and before the transition; grippers are unchanged.",
    )
    parser.add_argument("--pre-transition-home-duration-sec", type=float, default=5.0)
    parser.add_argument("--pre-transition-home-rate-hz", type=float, default=50.0)
    parser.add_argument(
        "--no-transfer-release",
        action="store_true",
        help="Do not close the partner gripper and open the grasping gripper after returning to transition.",
    )
    parser.add_argument(
        "--stop-after-partner-close",
        action="store_true",
        help=(
            "After returning to transition, close the partner gripper, open the active grasping "
            "gripper, then stop."
        ),
    )
    parser.add_argument(
        "--stop-before-partner-close",
        action="store_true",
        help="After returning to transition, stop before closing the partner gripper.",
    )
    parser.add_argument("--no-reanchor", action="store_true", help="Do not call step(None) before each stage.")
    parser.add_argument("--execute", action="store_true", help="Actually connect RPC, move robot, and control gripper. Default is plan-only.")
    parser.add_argument("--compact", action="store_true", help="Print compact JSON for result payloads.")
    return parser


def read_ee_poses(client: DualFrankaRobotiqRpcClient) -> tuple[np.ndarray, np.ndarray]:
    obs = client.get_observation()
    left = np.asarray(_pose_from_side_observation(obs, "left_arm"), dtype=float)
    right = np.asarray(_pose_from_side_observation(obs, "right_arm"), dtype=float)
    return left, right


def gripper_state_from_observation(observation: Any, side: str) -> dict[str, Any]:
    if not isinstance(observation, dict):
        return {}
    side_observation = observation.get(side)
    if not isinstance(side_observation, dict):
        return {}
    gripper = side_observation.get("gripper")
    if isinstance(gripper, dict):
        return dict(gripper)
    sensors = side_observation.get("sensors")
    if isinstance(sensors, dict) and isinstance(sensors.get("robotiq"), dict):
        return dict(sensors["robotiq"])
    return {}


def gripper_closed_fraction(state: dict[str, Any]) -> float | None:
    try:
        position = float(state["position"])
        open_position = float(state.get("open_position", 0.0))
        closed_position = float(state.get("closed_position", 0.7929))
    except (KeyError, TypeError, ValueError):
        return None
    values = (position, open_position, closed_position)
    if not all(math.isfinite(value) for value in values):
        return None
    span = closed_position - open_position
    if abs(span) <= 1e-9:
        return None
    return float(np.clip((position - open_position) / span, 0.0, 1.0))


def gripper_state_summary(state: dict[str, Any]) -> dict[str, Any]:
    keys = (
        "position",
        "velocity",
        "enabled",
        "last_command_position",
        "last_command_source",
        "open_position",
        "closed_position",
        "stamp",
    )
    return {key: state[key] for key in keys if key in state}


def close_gripper_and_confirm(
    client: DualFrankaRobotiqRpcClient,
    side: str,
    *,
    timeout_sec: float,
    poll_sec: float,
    min_closed_fraction: float,
    retries: int,
) -> tuple[Any, dict[str, Any]]:
    """Send close and wait for physical position feedback before allowing later actions."""
    initial_state = gripper_state_from_observation(client.get_observation(), side)
    initial_fraction = gripper_closed_fraction(initial_state)
    final_state = initial_state
    final_fraction = initial_fraction
    last_result: Any = None

    for attempt in range(1, retries + 2):
        last_result = client.close_gripper(side)
        deadline = time.monotonic() + timeout_sec
        while True:
            final_state = gripper_state_from_observation(client.get_observation(), side)
            final_fraction = gripper_closed_fraction(final_state)
            enabled = final_state.get("enabled")
            if (
                enabled is not False
                and final_fraction is not None
                and final_fraction >= min_closed_fraction
            ):
                return last_result, {
                    "ok": True,
                    "side": side,
                    "attempts": attempt,
                    "min_closed_fraction": min_closed_fraction,
                    "initial_closed_fraction": initial_fraction,
                    "final_closed_fraction": final_fraction,
                    "initial_state": gripper_state_summary(initial_state),
                    "final_state": gripper_state_summary(final_state),
                }
            now = time.monotonic()
            if now >= deadline:
                break
            time.sleep(min(poll_sec, deadline - now))

    return last_result, {
        "ok": False,
        "side": side,
        "attempts": retries + 1,
        "min_closed_fraction": min_closed_fraction,
        "initial_closed_fraction": initial_fraction,
        "final_closed_fraction": final_fraction,
        "initial_state": gripper_state_summary(initial_state),
        "final_state": gripper_state_summary(final_state),
        "reason": "gripper feedback did not reach the minimum closed fraction",
    }


def print_pose(name: str, pose: np.ndarray) -> None:
    print(f"{name}: {np.array2string(np.asarray(pose), precision=6, suppress_small=True)}")


def short_side(side: str) -> str:
    return side[:-4] if side.endswith("_arm") else side


def other_side(side: str) -> str:
    if side == "left_arm":
        return "right_arm"
    if side == "right_arm":
        return "left_arm"
    raise ValueError(f"Unexpected arm side: {side!r}")


def pose_for_side(left_pose: np.ndarray, right_pose: np.ndarray, side: str) -> np.ndarray:
    if side == "left_arm":
        return left_pose
    if side == "right_arm":
        return right_pose
    raise ValueError(f"Unexpected arm side: {side!r}")


def target_pair_for_active_side(
    active_side: str,
    active_target: np.ndarray,
    partner_target: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    if active_side == "left_arm":
        return active_target, partner_target
    if active_side == "right_arm":
        return partner_target, active_target
    raise ValueError(f"Unexpected arm side: {active_side!r}")


def _xyz_from_point(value: Any, name: str) -> np.ndarray:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a JSON object.")
    xyz = np.asarray([value.get("x_m"), value.get("y_m"), value.get("z_m")], dtype=float)
    if xyz.size != 3 or not np.all(np.isfinite(xyz)):
        raise ValueError(f"{name} must contain x_m/y_m/z_m finite numbers.")
    return xyz


def _point_for_result_base_frame(point: Any, name: str, frame: str) -> dict[str, Any]:
    if not isinstance(point, dict):
        raise ValueError(f"{name} must be a JSON object.")

    if all(axis in point for axis in ("x_m", "y_m", "z_m")):
        if frame != "base":
            raise ValueError(
                f"{name} uses the old flat x_m/y_m/z_m format and does not contain {frame!r}. "
                "Use --result-base-frame base for this JSON."
            )
        return point

    frame_point = point.get(frame)
    if not isinstance(frame_point, dict):
        available = ", ".join(sorted(k for k, v in point.items() if isinstance(v, dict))) or "none"
        raise ValueError(f"{name} does not contain frame {frame!r}. Available point frames: {available}.")
    return frame_point


def load_target_from_result_json(path: Path, frame: str, grasp_point: str) -> tuple[np.ndarray, np.ndarray | None]:
    result_json = path.expanduser()
    data = json.loads(result_json.read_text(encoding="utf-8"))
    points_base = data.get("points_base")
    if not isinstance(points_base, dict) or not points_base.get("available"):
        reason = points_base.get("reason") if isinstance(points_base, dict) else "missing points_base"
        raise ValueError(f"{result_json} does not contain available points_base ({reason}).")

    if grasp_point not in RESULT_GRASP_POINTS:
        raise ValueError(f"grasp_point must be one of {RESULT_GRASP_POINTS}, got {grasp_point!r}.")

    grasp_xyz = _xyz_from_point(
        _point_for_result_base_frame(points_base.get(grasp_point), f"points_base.{grasp_point}", frame),
        f"points_base.{grasp_point}.{frame}",
    )
    try:
        tail_xyz = _xyz_from_point(
            _point_for_result_base_frame(points_base.get("tail"), "points_base.tail", frame),
            f"points_base.tail.{frame}",
        )
        head_xyz = _xyz_from_point(
            _point_for_result_base_frame(points_base.get("head"), "points_base.head", frame),
            f"points_base.head.{frame}",
        )
    except ValueError:
        return grasp_xyz, None
    orientation_vector = head_xyz - tail_xyz
    norm = float(np.linalg.norm(orientation_vector))
    if norm <= 1e-9:
        return grasp_xyz, None
    return grasp_xyz, orientation_vector / norm


def pose_error(current: np.ndarray, target: np.ndarray) -> dict[str, Any]:
    residual = np.asarray(_absolute_target_to_delta(current.tolist(), target.tolist()), dtype=float)
    return {
        "raw_current_minus_target": (current - target).tolist(),
        "residual_command_to_target": residual.tolist(),
        "translation_norm_m": float(np.linalg.norm(residual[:3])),
        "rotation_norm_rad": float(np.linalg.norm(residual[3:])),
    }


def gripper_down_rotvec(current_pose: np.ndarray, down_pitch_rad: float) -> np.ndarray:
    rpy = R.from_rotvec(current_pose[3:]).as_euler("xyz")
    return R.from_euler("xyz", [rpy[0], down_pitch_rad, rpy[2]]).as_rotvec()


def _nearest_angle(angle: float, reference: float, period: float) -> float:
    return reference + ((angle - reference + period * 0.5) % period - period * 0.5)


def _axis_xy_for_yaw(roll: float, pitch: float, yaw: float, axis: str) -> np.ndarray:
    axis_index = 0 if axis == "x" else 1
    axis_world = R.from_euler("xyz", [roll, pitch, yaw]).as_matrix()[:, axis_index]
    axis_xy = axis_world[:2]
    norm = float(np.linalg.norm(axis_xy))
    if norm <= 1e-9:
        raise ValueError(f"End-effector {axis}-axis projection into base xy is too small.")
    return axis_xy / norm


def gripper_parallel_rotvec(
    current_pose: np.ndarray,
    orientation_vector_base: np.ndarray,
    down_pitch_rad: float,
    parallel_axis: str,
    directional: bool,
) -> tuple[np.ndarray, dict[str, Any]]:
    vector = np.asarray(orientation_vector_base, dtype=float).reshape(3)
    vector = vector / float(np.linalg.norm(vector))
    vector_xy = vector[:2]
    vector_xy_norm = float(np.linalg.norm(vector_xy))
    if vector_xy_norm <= 1e-6:
        raise ValueError(f"orientation_vector has no usable xy direction: {vector.tolist()}")

    target_xy = vector_xy / vector_xy_norm
    target_yaw = math.atan2(float(target_xy[1]), float(target_xy[0]))
    rpy = R.from_rotvec(current_pose[3:]).as_euler("xyz")
    roll = float(rpy[0])
    current_yaw = float(rpy[2])
    period = 2.0 * math.pi if directional else math.pi

    offsets = [0.0] if parallel_axis == "x" else [math.pi * 0.5, -math.pi * 0.5]
    best: tuple[float, float, float, np.ndarray] | None = None
    for offset in offsets:
        yaw = _nearest_angle(target_yaw + offset, current_yaw, period)
        axis_xy = _axis_xy_for_yaw(roll, down_pitch_rad, yaw, parallel_axis)
        dot = float(np.dot(axis_xy, target_xy))
        dot = max(-1.0, min(1.0, dot))
        parallel_score = dot if directional else abs(dot)
        yaw_delta = abs(_nearest_angle(yaw, current_yaw, 2.0 * math.pi) - current_yaw)
        candidate = (parallel_score, -yaw_delta, yaw, axis_xy)
        if best is None or candidate[:2] > best[:2]:
            best = candidate

    if best is None:
        raise RuntimeError("Failed to compute gripper orientation alignment.")

    score, _negative_yaw_delta, yaw, axis_xy = best
    rotvec = R.from_euler("xyz", [roll, down_pitch_rad, yaw]).as_rotvec()
    alignment = {
        "orientation_vector_base": vector.tolist(),
        "target_xy_unit": target_xy.tolist(),
        "target_yaw_rad": target_yaw,
        "chosen_yaw_rad": yaw,
        "current_yaw_rad": current_yaw,
        "parallel_axis": parallel_axis,
        "directional": directional,
        "axis_xy_unit": axis_xy.tolist(),
        "parallel_score": score,
    }
    return rotvec, alignment


def _xyz_rotvec_from_mapping(value: Any, name: str) -> np.ndarray:
    if isinstance(value, dict):
        value = value.get("xyz_rotvec")
    pose = np.asarray(value, dtype=float).reshape(-1)
    if pose.size != 6 or not np.all(np.isfinite(pose)):
        raise ValueError(f"{name} must contain xyz_rotvec with 6 finite numbers.")
    return pose


def _transition_targets_from_mapping(value: Any, name: str) -> tuple[np.ndarray, np.ndarray]:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a JSON object.")
    left = _xyz_rotvec_from_mapping(value.get("left_arm"), f"{name}.left_arm")
    right = _xyz_rotvec_from_mapping(value.get("right_arm"), f"{name}.right_arm")
    return left, right


def load_transition_targets(path: Path, active_side: str) -> tuple[np.ndarray, np.ndarray, str]:
    data = json.loads(path.expanduser().read_text(encoding="utf-8"))
    transition_states = data.get("transition_states")
    if isinstance(transition_states, dict):
        for state_key in (active_side, short_side(active_side)):
            if state_key in transition_states:
                left, right = _transition_targets_from_mapping(
                    transition_states[state_key],
                    f"transition_states.{state_key}",
                )
                return left, right, f"transition_states.{state_key}"

    summary = data.get("summary", {})
    left, right = _transition_targets_from_mapping(summary, "summary")
    return left, right, "summary"


def reanchor(client: DualFrankaRobotiqRpcClient, enabled: bool) -> None:
    if not enabled:
        return
    client.step(None)
    time.sleep(0.2)


def move_stage(
    client: DualFrankaRobotiqRpcClient,
    name: str,
    left_target: np.ndarray,
    right_target: np.ndarray,
    args: argparse.Namespace,
    *,
    position_tolerance_m: float | None = None,
    rotation_tolerance_rad: float | None = None,
    max_translation_speed: float | None = None,
    max_rotation_speed: float | None = None,
    max_translation_step: float | None = None,
    max_rotation_step: float | None = None,
    settle_time_sec: float | None = None,
    arrival_observed_z_offset_side: str | None = None,
    arrival_observed_z_offset_m: float = 0.0,
    recover_stalled_side: str | None = None,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    effective_position_tolerance_m = float(
        args.position_tolerance_m if position_tolerance_m is None else position_tolerance_m
    )
    effective_rotation_tolerance_rad = float(
        args.rotation_tolerance_rad if rotation_tolerance_rad is None else rotation_tolerance_rad
    )
    effective_max_translation_speed = float(
        args.max_translation_speed if max_translation_speed is None else max_translation_speed
    )
    effective_max_rotation_speed = float(
        args.max_rotation_speed if max_rotation_speed is None else max_rotation_speed
    )
    effective_max_translation_step = float(
        args.max_translation_step if max_translation_step is None else max_translation_step
    )
    effective_max_rotation_step = float(
        args.max_rotation_step if max_rotation_step is None else max_rotation_step
    )
    effective_settle_time_sec = float(
        args.settle_time_sec if settle_time_sec is None else settle_time_sec
    )
    effective_arrival_observed_z_offset_m = float(arrival_observed_z_offset_m)
    if arrival_observed_z_offset_side not in (None, "left_arm", "right_arm"):
        raise ValueError(
            "arrival_observed_z_offset_side must be None, 'left_arm', or 'right_arm', "
            f"got {arrival_observed_z_offset_side!r}"
        )
    if not math.isfinite(effective_arrival_observed_z_offset_m):
        raise ValueError("arrival_observed_z_offset_m must be finite")
    if recover_stalled_side not in (None, "left_arm", "right_arm"):
        raise ValueError(
            "recover_stalled_side must be None, 'left_arm', or 'right_arm', "
            f"got {recover_stalled_side!r}"
        )
    print(f"\n=== {name} ===")
    print_pose("left_target", left_target)
    print_pose("right_target", right_target)
    print(
        "stage tolerances:",
        f"position={effective_position_tolerance_m:.6f} m",
        f"rotation={effective_rotation_tolerance_rad:.6f} rad",
    )
    print(
        "stage motion:",
        f"translation_speed={effective_max_translation_speed:.3f} m/s",
        f"rotation_speed={effective_max_rotation_speed:.3f} rad/s",
        f"translation_step={effective_max_translation_step:.4f} m",
        f"rotation_step={effective_max_rotation_step:.4f} rad",
        f"settle={effective_settle_time_sec:.2f} s",
    )
    regular_retries_remaining = int(getattr(args, "p2p_retries", 1))
    controller_recovery_available = bool(
        getattr(args, "recover_stalled_controller", False) and recover_stalled_side is not None
    )
    stall_detection_enabled = bool(
        getattr(args, "recover_stalled_controller", False)
        and recover_stalled_side is not None
        and int(args.max_stalled_correction_iters) > 0
    )
    max_possible_attempts = 1 + regular_retries_remaining + int(controller_recovery_available)
    attempt_index = 0

    while True:
        attempt_number = attempt_index + 1
        if attempt_index > 0:
            reanchor(client, enabled=not bool(getattr(args, "no_reanchor", False)))
        print(f"p2p attempt: {attempt_number}/up-to-{max_possible_attempts}")

        left_before, right_before = read_ee_poses(client)

        result = client.dual_robot_move_to_ee_pose(
            left_target.tolist(),
            right_target.tolist(),
            delta=False,
            wait=False,
            smooth=True,
            rate_hz=args.rate_hz,
            max_translation_speed=effective_max_translation_speed,
            max_rotation_speed=effective_max_rotation_speed,
            max_translation_step=effective_max_translation_step,
            max_rotation_step=effective_max_rotation_step,
            settle_time_sec=effective_settle_time_sec,
            position_tolerance_m=effective_position_tolerance_m,
            rotation_tolerance_rad=effective_rotation_tolerance_rad,
            max_correction_iters=args.max_correction_iters,
            max_steps=args.max_steps,
            stall_detection_side=recover_stalled_side if stall_detection_enabled else None,
            stall_translation_epsilon_m=float(args.controller_stall_translation_epsilon_m),
            stall_rotation_epsilon_rad=float(args.controller_stall_rotation_epsilon_rad),
            max_stalled_correction_iters=(
                int(args.max_stalled_correction_iters) if stall_detection_enabled else 0
            ),
        )
        left_current, right_current = read_ee_poses(client)
        left_arrival = left_current.copy()
        right_arrival = right_current.copy()
        if arrival_observed_z_offset_side == "left_arm":
            left_arrival[2] += effective_arrival_observed_z_offset_m
        elif arrival_observed_z_offset_side == "right_arm":
            right_arrival[2] += effective_arrival_observed_z_offset_m
        left_error = pose_error(left_arrival, left_target)
        right_error = pose_error(right_arrival, right_target)
        print("p2p pose-tolerance ok (RPC returned):", result.get("ok"))
        if result.get("stalled") is not None:
            print(
                "p2p stall detection:",
                json.dumps(result.get("stalled"), indent=None if args.compact else 2, ensure_ascii=False),
            )
        print(
            "final_error:",
            json.dumps(
                result.get("final_error"),
                indent=None if args.compact else 2,
                ensure_ascii=False,
                default=str,
            ),
        )
        print_pose("left_current", left_current)
        print_pose("right_current", right_current)
        if arrival_observed_z_offset_side is not None and effective_arrival_observed_z_offset_m != 0.0:
            raw_z = left_current[2] if arrival_observed_z_offset_side == "left_arm" else right_current[2]
            print(
                "arrival observed-z adjustment:",
                f"side={arrival_observed_z_offset_side}",
                f"raw_z={raw_z:.6f} m",
                f"offset={effective_arrival_observed_z_offset_m:+.6f} m",
                f"adjusted_z={raw_z + effective_arrival_observed_z_offset_m:.6f} m",
            )
        print("left residual:", json.dumps(left_error, indent=None if args.compact else 2, ensure_ascii=False))
        print("right residual:", json.dumps(right_error, indent=None if args.compact else 2, ensure_ascii=False))
        result_ok = bool(result.get("ok"))
        residual_ok = (
            left_error["translation_norm_m"] <= effective_position_tolerance_m
            and left_error["rotation_norm_rad"] <= effective_rotation_tolerance_rad
            and right_error["translation_norm_m"] <= effective_position_tolerance_m
            and right_error["rotation_norm_rad"] <= effective_rotation_tolerance_rad
        )
        if residual_ok:
            if not result_ok:
                print(f"{name}: accepting RPC ok=false because observed residual is within configured tolerance")
            return left_current, right_current, result

        controller_recovered = False
        if controller_recovery_available:
            before = pose_for_side(left_before, right_before, recover_stalled_side)
            after = pose_for_side(left_current, right_current, recover_stalled_side)
            observed_motion = pose_error(before, after)
            translation_epsilon_m = float(args.controller_stall_translation_epsilon_m)
            rotation_epsilon_rad = float(args.controller_stall_rotation_epsilon_rad)
            stalled = (
                observed_motion["translation_norm_m"] <= translation_epsilon_m
                and observed_motion["rotation_norm_rad"] <= rotation_epsilon_rad
            )
            print(
                f"{name}: observed attempt motion for {recover_stalled_side}:",
                f"translation={observed_motion['translation_norm_m']:.6f} m",
                f"rotation={observed_motion['rotation_norm_rad']:.6f} rad",
                f"stalled={stalled}",
            )
            if stalled:
                print(
                    f"{name}: controller made no measurable progress; recovering "
                    f"{recover_stalled_side} before retry"
                )
                recovery = client.recover_robot(recover_stalled_side)
                print(
                    "controller recovery:",
                    json.dumps(recovery, indent=None if args.compact else 2, ensure_ascii=False, default=str),
                )
                if not isinstance(recovery, dict) or not bool(recovery.get("ok")):
                    raise RuntimeError(
                        f"{name}: {recover_stalled_side} controller recovery failed; "
                        "motion stopped before any gripper action"
                    )
                controller_recovered = True
                controller_recovery_available = False
                recovery_settle = float(args.controller_recovery_settle_time_sec)
                if recovery_settle > 0.0:
                    time.sleep(recovery_settle)

        if controller_recovered:
            print(
                f"{name}: retrying the same absolute target after controller recovery; "
                f"regular retries still available={regular_retries_remaining}"
            )
            attempt_index += 1
            continue

        if regular_retries_remaining > 0:
            regular_retries_remaining -= 1
            print(
                f"{name}: residual exceeds tolerance after P2P attempt {attempt_number}; "
                "retrying the same absolute target from the latest pose"
            )
            attempt_index += 1
            continue

        raise RuntimeError(
            f"{name} failed after {attempt_number} P2P attempt(s); "
            f"motion stopped before any subsequent gripper action "
            f"(p2p_ok={result_ok}, residual_ok={residual_ok})"
        )


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not math.isfinite(args.grasp_arrival_observed_z_offset_m):
        raise ValueError("--grasp-arrival-observed-z-offset-m must be finite")
    if not math.isfinite(args.grasp_target_z_offset_m):
        raise ValueError("--grasp-target-z-offset-m must be finite")
    for name in (
        "controller_stall_translation_epsilon_m",
        "controller_stall_rotation_epsilon_rad",
        "controller_recovery_settle_time_sec",
        "gripper_close_timeout_sec",
        "gripper_close_poll_sec",
    ):
        value = float(getattr(args, name))
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"--{name.replace('_', '-')} must be a non-negative finite value")
    if args.gripper_close_poll_sec <= 0.0:
        raise ValueError("--gripper-close-poll-sec must be positive")
    if not math.isfinite(args.gripper_close_min_fraction) or not 0.0 <= args.gripper_close_min_fraction <= 1.0:
        raise ValueError("--gripper-close-min-fraction must be a finite value in [0, 1]")
    active_side = args.arm
    partner_side = other_side(active_side)
    active_label = short_side(active_side)
    partner_label = short_side(partner_side)

    if args.result_json is not None:
        json_xyz, json_orientation_vector = load_target_from_result_json(
            args.result_json,
            args.result_base_frame,
            args.result_grasp_point,
        )
        if args.xyz is None:
            args.xyz = json_xyz
        if args.orientation_vector is None and json_orientation_vector is not None and not args.no_result_orientation:
            args.orientation_vector = json_orientation_vector

    if args.xyz is None:
        raise ValueError("No target xyz available. Pass --xyz or --result-json.")
    raw_grasp_xyz = np.asarray(args.xyz, dtype=float).reshape(3).copy()
    args.xyz = raw_grasp_xyz.copy()
    args.xyz[2] += float(args.grasp_target_z_offset_m)

    if not args.execute:
        plan_report = {
            "ok": True,
            "execute": False,
            "planned_only": True,
            "active_arm": active_side,
            "partner_arm": partner_side,
            "target_xyz": args.xyz.tolist() if hasattr(args.xyz, "tolist") else list(args.xyz),
            "raw_grasp_xyz": raw_grasp_xyz.tolist(),
            "grasp_target_z_offset_m": float(args.grasp_target_z_offset_m),
            "result_json": None if args.result_json is None else str(args.result_json.expanduser()),
            "result_base_frame": args.result_base_frame,
            "result_grasp_point": args.result_grasp_point,
            "orientation_vector": None if args.orientation_vector is None else args.orientation_vector.tolist(),
            "grasp_arrival_observed_z_offset_m": float(args.grasp_arrival_observed_z_offset_m),
            "stages": [
                "stage1_move_xy_keep_z",
                "stage2_align_or_pitch_gripper",
                "stage3_move_z_to_target",
                "stage4_go_home_without_gripper_action" if args.go_home_before_transition else "stage4_go_home_skipped",
                "stage4_return_transition" if not args.no_return_transition else "stage4_skipped",
            ],
            "would_close_gripper": not bool(args.no_close),
            "would_transfer_release": not bool(args.no_transfer_release or args.no_close),
            "would_stop_before_partner_close": bool(args.stop_before_partner_close),
            "would_stop_after_transfer_release": bool(args.stop_after_partner_close),
            "requires_execute_for_rpc": True,
            "abnormal_robot_state_detected": False,
        }
        print(json.dumps(plan_report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
        return 0

    client = DualFrankaRobotiqRpcClient(
        ip=args.server_host,
        port=args.server_port,
        timeout=args.rpc_timeout_sec,
    )
    try:
        print("server:", f"{args.server_host}:{args.server_port}")
        print("ping:", client.ping())
        if args.result_json is not None:
            print("result_json:", str(args.result_json.expanduser()))
            print("result_base_frame:", args.result_base_frame)
            print("result_grasp_point:", args.result_grasp_point)
        print("active_arm:", active_side)
        print("partner_arm:", partner_side)
        print_pose("target_xyz", args.xyz)
        print_pose("raw_grasp_xyz", raw_grasp_xyz)
        print("grasp_target_z_offset_m:", float(args.grasp_target_z_offset_m))

        left_current, right_current = read_ee_poses(client)
        print_pose("left0", left_current)
        print_pose("right0", right_current)

        reanchor(client, enabled=not args.no_reanchor)
        left_current, right_current = read_ee_poses(client)
        active_current = pose_for_side(left_current, right_current, active_side)
        partner_current = pose_for_side(left_current, right_current, partner_side)
        stage1_active = np.array(
            [
                args.xyz[0],
                args.xyz[1],
                active_current[2],
                active_current[3],
                active_current[4],
                active_current[5],
            ],
            dtype=float,
        )
        left_target, right_target = target_pair_for_active_side(active_side, stage1_active, partner_current)
        left_current, right_current, _ = move_stage(
            client,
            f"stage1_{active_label}_move_xy_keep_z",
            left_target,
            right_target,
            args,
            recover_stalled_side=active_side,
        )

        reanchor(client, enabled=not args.no_reanchor)
        left_current, right_current = read_ee_poses(client)
        active_current = pose_for_side(left_current, right_current, active_side)
        partner_current = pose_for_side(left_current, right_current, partner_side)
        if args.keep_current_rotation:
            down_rotvec = active_current[3:].copy()
            stage2_name = "stage2_keep_current_rotation"
        elif args.orientation_vector is None:
            down_rotvec = gripper_down_rotvec(active_current, args.down_pitch_rad)
            stage2_name = "stage2_pitch_gripper_down"
        else:
            down_rotvec, alignment = gripper_parallel_rotvec(
                active_current,
                args.orientation_vector,
                args.down_pitch_rad,
                args.gripper_parallel_axis,
                args.directional_orientation,
            )
            print_pose("orientation_vector_base", args.orientation_vector)
            print("orientation alignment:", json.dumps(alignment, indent=None if args.compact else 2, ensure_ascii=False))
            stage2_name = "stage2_align_gripper_to_orientation"
        stage2_active = np.array([args.xyz[0], args.xyz[1], active_current[2], *down_rotvec], dtype=float)
        left_target, right_target = target_pair_for_active_side(active_side, stage2_active, partner_current)
        left_current, right_current, _ = move_stage(
            client,
            f"{stage2_name}_{active_label}",
            left_target,
            right_target,
            args,
            recover_stalled_side=active_side,
        )

        if not args.skip_z:
            reanchor(client, enabled=not args.no_reanchor)
            left_current, right_current = read_ee_poses(client)
            partner_current = pose_for_side(left_current, right_current, partner_side)
            stage3_active = np.array([args.xyz[0], args.xyz[1], args.xyz[2], *down_rotvec], dtype=float)
            left_target, right_target = target_pair_for_active_side(active_side, stage3_active, partner_current)
            left_current, right_current, _ = move_stage(
                client,
                f"stage3_{active_label}_move_z_to_target",
                left_target,
                right_target,
                args,
                max_translation_speed=args.approach_max_translation_speed,
                max_rotation_speed=args.approach_max_rotation_speed,
                max_translation_step=args.approach_max_translation_step,
                max_rotation_step=args.approach_max_rotation_step,
                settle_time_sec=args.approach_settle_time_sec,
                arrival_observed_z_offset_side=active_side,
                arrival_observed_z_offset_m=args.grasp_arrival_observed_z_offset_m,
                recover_stalled_side=active_side,
            )

        final_left, final_right = read_ee_poses(client)
        final_active = pose_for_side(final_left, final_right, active_side)
        print("\n=== final ===")
        print_pose("left_final", final_left)
        print_pose("right_final", final_right)
        xyz_error = final_active[:3] - args.xyz
        print_pose(f"{active_label} xyz current-target", xyz_error)
        print(f"{active_label} xyz error norm m:", float(np.linalg.norm(xyz_error)))
        if args.grasp_arrival_observed_z_offset_m != 0.0:
            arrival_adjusted_active = final_active.copy()
            arrival_adjusted_active[2] += args.grasp_arrival_observed_z_offset_m
            arrival_xyz_error = arrival_adjusted_active[:3] - args.xyz
            print_pose(f"{active_label} arrival-adjusted xyz current-target", arrival_xyz_error)
            print(
                f"{active_label} arrival-adjusted xyz error norm m:",
                float(np.linalg.norm(arrival_xyz_error)),
            )

        if args.no_close:
            print("gripper close skipped by --no-close")
        else:
            close_result, close_confirmation = close_gripper_and_confirm(
                client,
                active_side,
                timeout_sec=args.gripper_close_timeout_sec,
                poll_sec=args.gripper_close_poll_sec,
                min_closed_fraction=args.gripper_close_min_fraction,
                retries=args.gripper_close_retries,
            )
            print(
                f"close_{active_label}_gripper:",
                json.dumps(close_result, indent=None if args.compact else 2, ensure_ascii=False, default=str),
            )
            print(
                f"close_{active_label}_gripper_confirmation:",
                json.dumps(close_confirmation, indent=None if args.compact else 2, ensure_ascii=False, default=str),
            )
            if not close_confirmation["ok"]:
                raise RuntimeError(
                    f"close_{active_label}_gripper did not produce confirmed physical closure; "
                    "motion stopped before lifting the tube"
                )
            if args.after_close_sleep_sec > 0:
                time.sleep(args.after_close_sleep_sec)

        if args.no_return_transition:
            print("return transition skipped by --no-return-transition")
            return 0

        if args.go_home_before_transition:
            print("\n=== pre_transition_go_home ===")
            print("moving both arms home; gripper states are unchanged")
            home_result = client.go_home(
                "both",
                args.pre_transition_home_duration_sec,
                args.pre_transition_home_rate_hz,
            )
            print(
                "go_home_both:",
                json.dumps(home_result, indent=None if args.compact else 2, ensure_ascii=False, default=str),
            )
            if isinstance(home_result, dict) and home_result.get("ok") is False:
                raise RuntimeError(f"pre-transition go_home failed: {home_result}")

        transition_left, transition_right, transition_source = load_transition_targets(args.transition_json, active_side)
        print("\n=== transition_target ===")
        print("transition_json:", str(args.transition_json))
        print("transition_source:", transition_source)
        print_pose("transition_left", transition_left)
        print_pose("transition_right", transition_right)
        reanchor(client, enabled=not args.no_reanchor)
        move_stage(
            client,
            "stage4_return_transition",
            transition_left,
            transition_right,
            args,
            rotation_tolerance_rad=args.transition_rotation_tolerance_rad,
        )

        if args.no_close:
            print(f"{partner_label} close/{active_label} open skipped by --no-close")
        elif args.no_transfer_release:
            print(f"{partner_label} close/{active_label} open skipped by --no-transfer-release")
        else:
            if args.stop_before_partner_close:
                print(
                    f"stopped before close_{partner_label}_gripper "
                    "by --stop-before-partner-close"
                )
                return 0
            partner_close_result, partner_close_confirmation = close_gripper_and_confirm(
                client,
                partner_side,
                timeout_sec=args.gripper_close_timeout_sec,
                poll_sec=args.gripper_close_poll_sec,
                min_closed_fraction=args.gripper_close_min_fraction,
                retries=args.gripper_close_retries,
            )
            print(
                f"close_{partner_label}_gripper:",
                json.dumps(partner_close_result, indent=None if args.compact else 2, ensure_ascii=False, default=str),
            )
            print(
                f"close_{partner_label}_gripper_confirmation:",
                json.dumps(
                    partner_close_confirmation,
                    indent=None if args.compact else 2,
                    ensure_ascii=False,
                    default=str,
                ),
            )
            if not partner_close_confirmation["ok"]:
                raise RuntimeError(
                    f"close_{partner_label}_gripper did not produce confirmed physical closure; "
                    f"retaining the tube in {active_label}_gripper"
                )
            if args.after_partner_close_sleep_sec > 0:
                time.sleep(args.after_partner_close_sleep_sec)

            active_open_result = client.open_gripper(active_side)
            print(
                f"open_{active_label}_gripper:",
                json.dumps(active_open_result, indent=None if args.compact else 2, ensure_ascii=False, default=str),
            )
            if args.after_active_open_sleep_sec > 0:
                time.sleep(args.after_active_open_sleep_sec)
            if args.stop_after_partner_close:
                print(
                    f"stopped after close_{partner_label}_gripper and open_{active_label}_gripper "
                    "by --stop-after-partner-close"
                )
                return 0
        return 0
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
