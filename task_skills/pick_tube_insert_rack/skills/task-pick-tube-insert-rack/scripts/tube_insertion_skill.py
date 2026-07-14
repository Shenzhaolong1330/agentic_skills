#!/usr/bin/env python3
"""Insert a held test tube into a rack hole.

This tool is intentionally conservative. It can run vision and planning in
dry-run mode, but it only sends robot motion when --execute is set. The final
insertion descent has a second explicit --execute-insertion gate. The holder
arm must be supplied explicitly with --holder-side.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Any, Mapping

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R


ANY_POSE_DIR = Path("/home/deepcybo/agentic_skills/atomic_skills/object_locator")
RPC_CLIENT_PATH = Path(
    os.environ.get(
        "DUAL_FRANKA_RPC_CLIENT_PATH",
        "/home/deepcybo/agentic_skills/atomic_skills/dual_franka_p2p/skills/"
        "atomic-motion-franka-move-to-pose/dual_franka_robotiq_rpc_client.py",
    )
)

SIDE_TO_ARM = {"left": "left_arm", "right": "right_arm"}
SIDE_TO_CAMERA = {"left": "left_wrist", "right": "right_wrist"}
OPPOSITE_SIDE = {"left": "right", "right": "left"}
SIDE_TO_BASE_T_FLANGE = {
    "left": ANY_POSE_DIR / "calibration" / "base_T_left_flange.yaml",
    "right": ANY_POSE_DIR / "calibration" / "base_T_flange.yaml",
}
DEFAULT_HOLE_CONFIG = {
    "left": ANY_POSE_DIR / "config_rack_empty_hole_left_wrist_vlm.yaml",
    "right": ANY_POSE_DIR / "config_rack_empty_hole_right_wrist_vlm.yaml",
}


def _load_rpc_module():
    spec = importlib.util.spec_from_file_location("dual_franka_rpc_client_for_insertion", RPC_CLIENT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load RPC client from {RPC_CLIENT_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


franka_rpc = _load_rpc_module()
DualFrankaRobotiqRpcClient = franka_rpc.DualFrankaRobotiqRpcClient
_pose_from_side_observation = franka_rpc._pose_from_side_observation


def as_vector3(value: Any, name: str) -> np.ndarray:
    vector = np.asarray(value, dtype=float).reshape(-1)
    if vector.size != 3 or not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain exactly 3 finite numbers, got {value!r}")
    return vector


def as_pose6(value: Any, name: str) -> np.ndarray:
    pose = np.asarray(value, dtype=float).reshape(-1)
    if pose.size != 6 or not np.all(np.isfinite(pose)):
        raise ValueError(f"{name} must contain exactly 6 finite numbers, got {value!r}")
    return pose


def parse_json_vector3(value: str) -> np.ndarray:
    try:
        parsed = json.loads(value)
    except json.JSONDecodeError as exc:
        raise argparse.ArgumentTypeError(f"expected JSON list of 3 numbers: {exc}") from exc
    try:
        return as_vector3(parsed, "vector")
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def normalize_vector(vector: np.ndarray, name: str) -> np.ndarray:
    vector = as_vector3(vector, name)
    norm = float(np.linalg.norm(vector))
    if norm <= 1e-9:
        raise ValueError(f"{name} must have non-zero length")
    return vector / norm


def rotation_between_vectors(source: np.ndarray, target: np.ndarray) -> R:
    source_unit = normalize_vector(source, "source")
    target_unit = normalize_vector(target, "target")
    cross = np.cross(source_unit, target_unit)
    cross_norm = float(np.linalg.norm(cross))
    dot = float(np.clip(np.dot(source_unit, target_unit), -1.0, 1.0))
    if cross_norm <= 1e-9:
        if dot > 0.0:
            return R.identity()
        helper = np.array([1.0, 0.0, 0.0], dtype=float)
        if abs(float(np.dot(source_unit, helper))) > 0.9:
            helper = np.array([0.0, 1.0, 0.0], dtype=float)
        axis = normalize_vector(np.cross(source_unit, helper), "opposite_axis")
        return R.from_rotvec(axis * np.pi)
    axis = cross / cross_norm
    angle = float(np.arctan2(cross_norm, dot))
    return R.from_rotvec(axis * angle)


def align_tcp_orientation_to_tube_axis(
    *,
    current_tcp_rotvec: np.ndarray,
    tube_axis_base: np.ndarray,
    desired_tube_axis_base: np.ndarray,
) -> np.ndarray:
    """Rotate the current TCP orientation so the inferred tube axis matches target.

    The tube is assumed rigidly grasped. The alignment rotation is applied in
    base frame before the current TCP orientation, so the inferred base-frame
    tube axis moves with the end effector.
    """
    current_rotation = R.from_rotvec(np.asarray(current_tcp_rotvec, dtype=float).reshape(3))
    align_rotation = rotation_between_vectors(tube_axis_base, desired_tube_axis_base)
    return (align_rotation * current_rotation).as_rotvec()


def nearest_equivalent_angle(angle: float, reference: float, period: float = 2.0 * np.pi) -> float:
    return float(reference + ((float(angle) - float(reference) + period * 0.5) % period - period * 0.5))


def tcp_pitch_zero_rotvec(
    holder_pose: np.ndarray,
    pitch_rad: float = 0.0,
    roll_rad: float | None = None,
) -> np.ndarray:
    """Match grasping's down posture by preserving yaw and setting pitch.

    By default this preserves the current roll, matching the grasping helper.
    Passing roll_rad=np.pi removes small roll drift while choosing the nearest
    equivalent pi branch to avoid a large flip.
    """
    pose = as_pose6(holder_pose, "holder_pose")
    rpy = R.from_rotvec(pose[3:]).as_euler("xyz")
    roll = float(rpy[0]) if roll_rad is None else nearest_equivalent_angle(float(roll_rad), float(rpy[0]))
    return R.from_euler("xyz", [roll, float(pitch_rad), float(rpy[2])]).as_rotvec()


def pose6_to_matrix(pose6: np.ndarray) -> np.ndarray:
    pose = as_pose6(pose6, "pose6")
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = R.from_rotvec(pose[3:]).as_matrix()
    matrix[:3, 3] = pose[:3]
    return matrix


def compute_tip_target(
    *,
    hole_base_m: np.ndarray,
    insertion_axis_base: np.ndarray,
    standoff_m: float,
    insertion_depth_m: float,
) -> np.ndarray:
    """Return a desired tube-tip position in base frame.

    insertion_axis_base points in the direction of insertion. A positive
    standoff moves opposite that axis, i.e. above the hole before insertion.
    A positive insertion_depth moves into the hole along that axis.
    """
    hole = as_vector3(hole_base_m, "hole_base_m")
    axis = normalize_vector(insertion_axis_base, "insertion_axis_base")
    return hole - axis * float(standoff_m) + axis * float(insertion_depth_m)


def compute_tcp_plane_target(
    *,
    hole_base_m: np.ndarray,
    insertion_axis_base: np.ndarray,
    standoff_m: float,
    insertion_depth_m: float,
) -> np.ndarray:
    """Return a desired TCP position relative to the hole plane in base frame."""
    return compute_tip_target(
        hole_base_m=hole_base_m,
        insertion_axis_base=insertion_axis_base,
        standoff_m=standoff_m,
        insertion_depth_m=insertion_depth_m,
    )


def compute_tcp_plane_target_pose(
    *,
    holder_pose: np.ndarray,
    hole_base_m: np.ndarray,
    insertion_axis_base: np.ndarray,
    standoff_m: float,
    insertion_depth_m: float,
    tcp_orientation_rotvec: np.ndarray,
) -> np.ndarray:
    tcp_target = compute_tcp_plane_target(
        hole_base_m=hole_base_m,
        insertion_axis_base=insertion_axis_base,
        standoff_m=standoff_m,
        insertion_depth_m=insertion_depth_m,
    )
    pose = as_pose6(holder_pose, "holder_pose").copy()
    pose[:3] = tcp_target
    pose[3:] = np.asarray(tcp_orientation_rotvec, dtype=float).reshape(3)
    return pose


def compute_tcp_target_for_tip(
    *,
    desired_tip_base_m: np.ndarray,
    tcp_to_tip_m: np.ndarray,
    tcp_orientation_rotvec: np.ndarray,
) -> np.ndarray:
    desired_tip = as_vector3(desired_tip_base_m, "desired_tip_base_m")
    tip_offset = as_vector3(tcp_to_tip_m, "tcp_to_tip_m")
    rot = R.from_rotvec(np.asarray(tcp_orientation_rotvec, dtype=float).reshape(3))
    return desired_tip - rot.as_matrix() @ tip_offset


def compute_rack_observation_pose(
    *,
    rack_base_m: np.ndarray,
    holder_pose: np.ndarray,
    tcp_orientation_rotvec: np.ndarray,
    observe_offset_m: np.ndarray,
    observe_height_m: float,
) -> np.ndarray:
    pose = as_pose6(holder_pose, "holder_pose").copy()
    rack = as_vector3(rack_base_m, "rack_base_m")
    offset = as_vector3(observe_offset_m, "observe_offset_m")
    pose[:3] = rack + offset
    pose[2] = rack[2] + float(observe_height_m)
    pose[3:] = np.asarray(tcp_orientation_rotvec, dtype=float).reshape(3)
    return pose


def apply_rack_plane_z_for_motion(
    point_base_m: np.ndarray,
    *,
    rack_plane_z_base_m: float | None,
) -> np.ndarray:
    """Use a manually calibrated rack plane z while preserving detected xy."""
    point = as_vector3(point_base_m, "point_base_m").copy()
    if rack_plane_z_base_m is None:
        return point
    plane_z = float(rack_plane_z_base_m)
    if not np.isfinite(plane_z):
        raise ValueError(f"rack_plane_z_base_m must be finite, got {rack_plane_z_base_m!r}")
    point[2] = plane_z
    return point


def compute_tip_target_pose(
    *,
    holder_pose: np.ndarray,
    hole_base_m: np.ndarray,
    insertion_axis_base: np.ndarray,
    standoff_m: float,
    insertion_depth_m: float,
    tcp_to_tip_m: np.ndarray,
    tcp_orientation_rotvec: np.ndarray,
) -> np.ndarray:
    tip_target = compute_tip_target(
        hole_base_m=hole_base_m,
        insertion_axis_base=insertion_axis_base,
        standoff_m=standoff_m,
        insertion_depth_m=insertion_depth_m,
    )
    tcp_xyz = compute_tcp_target_for_tip(
        desired_tip_base_m=tip_target,
        tcp_to_tip_m=tcp_to_tip_m,
        tcp_orientation_rotvec=tcp_orientation_rotvec,
    )
    pose = as_pose6(holder_pose, "holder_pose").copy()
    pose[:3] = tcp_xyz
    pose[3:] = np.asarray(tcp_orientation_rotvec, dtype=float).reshape(3)
    return pose


def gripper_closed_fraction(side_obs: Mapping[str, Any]) -> float:
    gripper = side_obs.get("gripper", {}) if isinstance(side_obs, Mapping) else {}
    if not isinstance(gripper, Mapping):
        return 0.0
    position = gripper.get("position")
    open_position = gripper.get("open_position", 0.0)
    closed_position = gripper.get("closed_position", 0.7929)
    try:
        position_f = float(position)
        open_f = float(open_position)
        closed_f = float(closed_position)
    except (TypeError, ValueError):
        return 0.0
    denom = closed_f - open_f
    if abs(denom) <= 1e-9:
        return 0.0
    return float(np.clip((position_f - open_f) / denom, 0.0, 1.0))


def load_flange_T_camera(any_pose_dir: Path, side: str) -> np.ndarray:
    camera_name = SIDE_TO_CAMERA[side]
    extrinsics_path = any_pose_dir / "calibration" / "extrinsics.yaml"
    raw = yaml.safe_load(extrinsics_path.read_text(encoding="utf-8")) or {}
    camera = raw["cameras"][camera_name]
    if "flange_T_camera" in camera:
        return np.asarray(camera["flange_T_camera"]["matrix"], dtype=float).reshape(4, 4)
    if "camera_T_flange" in camera:
        return np.linalg.inv(np.asarray(camera["camera_T_flange"]["matrix"], dtype=float).reshape(4, 4))
    raise ValueError(f"{camera_name} does not define flange_T_camera")


def tube_tip_axis_from_tcp_offset(*, current_tcp_rotvec: np.ndarray, tcp_to_tip_m: np.ndarray) -> np.ndarray:
    tcp_to_tip = normalize_vector(tcp_to_tip_m, "tcp_to_tip_m")
    return normalize_vector(R.from_rotvec(np.asarray(current_tcp_rotvec, dtype=float).reshape(3)).apply(tcp_to_tip), "tube_tip_axis_base")


def should_run_wrist_hole_detection(*, execute: bool, hole_result_json: Path | None) -> bool:
    return bool(execute or hole_result_json is not None)


def load_result_json(path: Path) -> dict[str, Any]:
    return json.loads(path.expanduser().read_text(encoding="utf-8"))


def read_ee_poses(client: DualFrankaRobotiqRpcClient) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    observation = client.get_observation()
    poses = {
        "left": np.asarray(_pose_from_side_observation(observation, "left_arm"), dtype=float),
        "right": np.asarray(_pose_from_side_observation(observation, "right_arm"), dtype=float),
    }
    return observation, poses


def write_base_T_flange_files(any_pose_dir: Path, observation: Mapping[str, Any]) -> None:
    for side, arm_key in SIDE_TO_ARM.items():
        side_obs = observation[arm_key]
        eef_pose = side_obs["robot_state"]["eef_pose"]
        payload = {
            "base_T_flange": {
                "translation_m": [float(v) for v in eef_pose["position"]],
                "rotation_quat_xyzw": [float(v) for v in eef_pose["orientation_xyzw"]],
            }
        }
        path = SIDE_TO_BASE_T_FLANGE[side]
        if any_pose_dir != ANY_POSE_DIR:
            path = any_pose_dir / "calibration" / path.name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")


def _object_locator_invocation(
    any_pose_dir: Path,
    config_path: Path,
    *,
    extra_args: list[str] | None = None,
) -> tuple[list[str], dict[str, str]]:
    config = config_path
    if not config.is_absolute():
        config = any_pose_dir / config
    executable = any_pose_dir / ".venv" / "bin" / "object-locator"
    if not executable.exists():
        raise FileNotFoundError(f"object-locator executable does not exist: {executable}")
    sam_socket = os.environ.get("TASK_PICK_TUBE_SAM_SOCKET")
    if sam_socket:
        wrapper = Path(__file__).with_name("object_locator_with_sam_cache.py")
        python = any_pose_dir / ".venv" / "bin" / "python"
        if not wrapper.exists() or not python.exists():
            raise FileNotFoundError(
                f"persistent SAM wrapper is unavailable: wrapper={wrapper}, python={python}"
            )
        command = [str(python), str(wrapper), "--config", str(config), "--json"]
    else:
        command = [str(executable), "--config", str(config), "--json"]
    if extra_args:
        command.extend(extra_args)
    env = dict(os.environ)
    env.setdefault("HF_HUB_DISABLE_XET", "1")
    env_path = any_pose_dir / ".env"
    if env_path.exists():
        try:
            from dotenv import dotenv_values

            for key, value in dotenv_values(env_path).items():
                if value is not None:
                    env[key] = value
        except ImportError:
            for line in env_path.read_text(encoding="utf-8").splitlines():
                stripped = line.strip()
                if not stripped or stripped.startswith("#") or "=" not in stripped:
                    continue
                key, value = stripped.split("=", 1)
                env[key.strip()] = value.strip().strip("'\"")
    return command, env


def _parse_object_locator_result(
    *,
    command: list[str],
    returncode: int,
    stdout: str,
    stderr: str,
) -> dict[str, Any]:
    if returncode != 0:
        raise RuntimeError(
            "object-locator failed\n"
            f"command: {' '.join(command)}\n"
            f"stdout:\n{stdout}\n"
            f"stderr:\n{stderr}"
        )
    try:
        return json.loads(stdout)
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"object-locator did not return JSON: {stdout[:500]}") from exc


def _capture_watchdog_seconds(name: str, default: float) -> float:
    raw = os.environ.get(name)
    value = default if raw is None else float(raw)
    if not np.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be a positive finite number, got {raw!r}")
    return float(value)


def _capture_watchdog_attempts(name: str, default: int) -> int:
    raw = os.environ.get(name)
    value = default if raw is None else int(raw)
    if value < 1:
        raise ValueError(f"{name} must be >= 1, got {raw!r}")
    return int(value)


def recover_object_locator_after_camera_reset(
    any_pose_dir: Path,
    config_path: Path,
    capture_ready_file: Path,
) -> tuple[PendingObjectLocator, dict[str, Any]]:
    """Retry capture with forced hardware reset, tolerating delayed UVC release."""
    recovery_sec = _capture_watchdog_seconds("REALSENSE_RESET_CAPTURE_TIMEOUT_SEC", 15.0)
    cooldown_sec = _capture_watchdog_seconds("REALSENSE_RESET_COOLDOWN_SEC", 2.0)
    max_attempts = _capture_watchdog_attempts("REALSENSE_RESET_MAX_ATTEMPTS", 3)
    last_error: Exception | None = None
    for attempt in range(1, max_attempts + 1):
        print(
            f"[tube-insertion] waiting {cooldown_sec:.1f}s for RealSense USB handle release "
            f"before reset attempt {attempt}/{max_attempts}",
            file=sys.stderr,
            flush=True,
        )
        time.sleep(cooldown_sec)
        pending = start_object_locator_after_capture_signal(
            any_pose_dir,
            config_path,
            capture_ready_file,
            extra_args=["--reset-realsense"],
        )
        try:
            capture_status = wait_for_object_locator_capture(
                pending,
                timeout_sec=recovery_sec,
            )
        except Exception as exc:
            last_error = exc
            cancel_pending_object_locator(pending)
            print(
                f"[tube-insertion] RealSense reset attempt {attempt}/{max_attempts} failed: {exc}",
                file=sys.stderr,
                flush=True,
            )
            continue
        print(
            f"[tube-insertion] RealSense recovered on reset attempt {attempt}/{max_attempts}; "
            "continuing inference",
            file=sys.stderr,
            flush=True,
        )
        return pending, capture_status
    raise RuntimeError(
        f"RealSense did not recover after {max_attempts} forced reset attempt(s) "
        f"for config={config_path}: {last_error}"
    ) from last_error


def run_object_locator(any_pose_dir: Path, config_path: Path) -> dict[str, Any]:
    """Run locator with a camera-only watchdog and one forced-reset retry.

    The short watchdog covers only RGB-D capture. Once capture_ready is written,
    VLM/SAM inference may take its normal configured amount of time.
    """
    watchdog_sec = _capture_watchdog_seconds("REALSENSE_CAPTURE_WATCHDOG_SEC", 3.0)
    signal_dir = Path("/tmp/agentic_skills_runs/object_locator_capture_watchdog")
    signal_file = signal_dir / f"capture_ready_{os.getpid()}_{time.time_ns()}.json"

    def start(*, reset_realsense: bool) -> PendingObjectLocator:
        extra_args = ["--reset-realsense"] if reset_realsense else None
        return start_object_locator_after_capture_signal(
            any_pose_dir,
            config_path,
            signal_file,
            extra_args=extra_args,
        )

    total_started = time.monotonic()
    reset_started: float | None = None
    capture_status: dict[str, Any] | None = None
    reset_attempted = False
    pending = start(reset_realsense=False)
    try:
        try:
            capture_status = wait_for_object_locator_capture(pending, timeout_sec=watchdog_sec)
        except TimeoutError:
            reset_attempted = True
            reset_started = time.monotonic()
            cancel_pending_object_locator(pending)
            print(
                f"[tube-insertion] RealSense produced no frame within {watchdog_sec:.1f}s; "
                f"forcing camera reset and retry for config={config_path}",
                file=sys.stderr,
                flush=True,
            )
            pending, capture_status = recover_object_locator_after_camera_reset(
                any_pose_dir,
                config_path,
                signal_file,
            )
        inference_started = time.monotonic()
        result = finish_pending_object_locator(pending)
        timings = {
            "capture": None if capture_status is None else float(capture_status.get("wait_sec", 0.0)),
            "reset_recovery": 0.0 if reset_started is None else inference_started - reset_started,
            "inference_after_capture": time.monotonic() - inference_started,
            "total": time.monotonic() - total_started,
            "reset_attempted": reset_attempted,
        }
        result["task_timings_sec"] = timings
        return result
    finally:
        signal_file.unlink(missing_ok=True)


@dataclass
class PendingObjectLocator:
    process: subprocess.Popen[str]
    command: list[str]
    capture_ready_file: Path
    started_monotonic: float


def start_object_locator_after_capture_signal(
    any_pose_dir: Path,
    config_path: Path,
    capture_ready_file: Path,
    *,
    extra_args: list[str] | None = None,
) -> PendingObjectLocator:
    capture_ready_file.parent.mkdir(parents=True, exist_ok=True)
    capture_ready_file.unlink(missing_ok=True)
    invocation_args = ["--capture-ready-file", str(capture_ready_file)]
    if extra_args:
        invocation_args.extend(extra_args)
    command, env = _object_locator_invocation(
        any_pose_dir,
        config_path,
        extra_args=invocation_args,
    )
    process = subprocess.Popen(
        command,
        cwd=str(any_pose_dir),
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return PendingObjectLocator(
        process=process,
        command=command,
        capture_ready_file=capture_ready_file,
        started_monotonic=time.monotonic(),
    )


def wait_for_object_locator_capture(
    pending: PendingObjectLocator,
    *,
    timeout_sec: float,
) -> dict[str, Any]:
    timeout = float(timeout_sec)
    if not np.isfinite(timeout) or timeout <= 0.0:
        raise ValueError(f"timeout_sec must be a positive finite value, got {timeout_sec!r}")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pending.capture_ready_file.exists():
            try:
                payload = json.loads(pending.capture_ready_file.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                raise RuntimeError(
                    f"invalid object-locator capture-ready file: {pending.capture_ready_file}"
                ) from exc
            return {
                "wait_sec": time.monotonic() - pending.started_monotonic,
                "signal": payload,
            }
        returncode = pending.process.poll()
        if returncode is not None:
            stdout, stderr = pending.process.communicate()
            _parse_object_locator_result(
                command=pending.command,
                returncode=int(returncode),
                stdout=stdout,
                stderr=stderr,
            )
            raise RuntimeError("object-locator exited without writing its capture-ready signal")
        time.sleep(0.05)
    raise TimeoutError(
        f"object-locator did not finish camera capture within {timeout:.1f}s: "
        f"{' '.join(pending.command)}"
    )


def finish_pending_object_locator(pending: PendingObjectLocator) -> dict[str, Any]:
    stdout, stderr = pending.process.communicate()
    return _parse_object_locator_result(
        command=pending.command,
        returncode=int(pending.process.returncode),
        stdout=stdout,
        stderr=stderr,
    )


def cancel_pending_object_locator(pending: PendingObjectLocator) -> None:
    if pending.process.poll() is None:
        pending.process.terminate()
        try:
            pending.process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            pending.process.kill()
            pending.process.wait(timeout=3.0)
    pending.process.communicate()


def side_arm_key(side: str) -> str:
    if side not in SIDE_TO_ARM:
        raise ValueError(f"side must be one of {sorted(SIDE_TO_ARM)}, got {side!r}")
    return SIDE_TO_ARM[side]


def compute_non_holder_yield_pose(
    *,
    holder_side: str,
    non_holder_pose: np.ndarray,
    distance_m: float,
) -> np.ndarray:
    non_holder_side = OPPOSITE_SIDE.get(holder_side)
    if non_holder_side is None:
        raise ValueError(f"holder_side must be one of {sorted(OPPOSITE_SIDE)}, got {holder_side!r}")
    distance = float(distance_m)
    if not np.isfinite(distance) or distance < 0.0:
        raise ValueError(f"distance_m must be a finite non-negative value, got {distance_m!r}")

    target = as_pose6(non_holder_pose, "non_holder_pose").copy()
    outward_y_sign = 1.0 if non_holder_side == "left" else -1.0
    target[1] += outward_y_sign * distance
    return target


def move_side_to_home(
    *,
    client: DualFrankaRobotiqRpcClient,
    side: str,
    duration_sec: float,
    rate_hz: float,
    execute: bool,
    stage_name: str,
) -> dict[str, Any]:
    side_arm_key(side)
    duration = float(duration_sec)
    rate = float(rate_hz)
    if not np.isfinite(duration) or duration <= 0.0:
        raise ValueError(f"duration_sec must be a positive finite value, got {duration_sec!r}")
    if not np.isfinite(rate) or rate <= 0.0:
        raise ValueError(f"rate_hz must be a positive finite value, got {rate_hz!r}")

    stage: dict[str, Any] = {
        "stage": stage_name,
        "mode": "single_arm_home",
        "side": side,
        "execute": bool(execute),
        "duration_sec": duration,
        "rate_hz": rate,
    }
    if not execute:
        stage["planned_only"] = True
        return stage
    try:
        home_result = client.go_home(side, duration, rate)
    except Exception as exc:
        stage["result"] = {"ok": False, "error": f"single-arm go_home failed: {exc}"}
        return stage

    stage["result"] = home_result
    if not isinstance(home_result, Mapping) or not bool(home_result.get("ok")):
        return stage

    # Joint-home switches the Cartesian controller back on with its previous
    # equilibrium target. Re-anchor both Cartesian targets before the next P2P
    # so the non-holder arm is not pulled back toward its pre-home pose.
    try:
        anchor_observation = client.reset()
        anchor_poses = {
            side: _pose_from_side_observation(anchor_observation, side_arm_key(side))
            for side in ("left", "right")
        }
        stage["cartesian_target_reanchor"] = {
            "ok": True,
            "method": "rpc_reset_target_to_current",
            "poses": anchor_poses,
        }
    except Exception as exc:
        stage["cartesian_target_reanchor"] = {
            "ok": False,
            "method": "rpc_reset_target_to_current",
            "error": str(exc),
        }
        failed_result = dict(home_result)
        failed_result["ok"] = False
        failed_result["error"] = "single-arm home succeeded but Cartesian target re-anchor failed"
        stage["result"] = failed_result
    return stage


def move_non_holder_to_home(
    *,
    client: DualFrankaRobotiqRpcClient,
    holder_side: str,
    duration_sec: float,
    rate_hz: float,
    execute: bool,
) -> dict[str, Any]:
    non_holder_side = OPPOSITE_SIDE.get(holder_side)
    if non_holder_side is None:
        raise ValueError(f"holder_side must be one of {sorted(OPPOSITE_SIDE)}, got {holder_side!r}")
    return move_side_to_home(
        client=client,
        side=non_holder_side,
        duration_sec=duration_sec,
        rate_hz=rate_hz,
        execute=execute,
        stage_name="yield_non_holder_arm",
    )


def compute_retract_pose(
    *,
    holder_pose: np.ndarray,
    insertion_axis_base: np.ndarray,
    retract_distance_m: float,
) -> np.ndarray:
    """Move the holder TCP away from the hole plane while keeping orientation."""
    distance = float(retract_distance_m)
    if not np.isfinite(distance) or distance < 0.0:
        raise ValueError(f"retract_distance_m must be a finite non-negative value, got {retract_distance_m!r}")

    pose = as_pose6(holder_pose, "holder_pose").copy()
    axis = normalize_vector(insertion_axis_base, "insertion_axis_base")
    pose[:3] -= axis * distance
    return pose


def compute_post_observe_vertical_correction_pose(
    *,
    current_pose: np.ndarray,
    tcp_orientation_rotvec: np.ndarray,
) -> np.ndarray:
    """Correct TCP posture in place after the observation move."""
    pose = as_pose6(current_pose, "current_pose").copy()
    pose[3:] = np.asarray(tcp_orientation_rotvec, dtype=float).reshape(3)
    return pose


def move_dual_absolute(
    *,
    client: DualFrankaRobotiqRpcClient,
    side: str,
    side_target: np.ndarray,
    all_current: Mapping[str, np.ndarray],
    args: argparse.Namespace,
    stage_name: str,
    execute: bool,
) -> dict[str, Any]:
    left_target = np.asarray(all_current["left"], dtype=float).copy()
    right_target = np.asarray(all_current["right"], dtype=float).copy()
    if side == "left":
        left_target = np.asarray(side_target, dtype=float)
    else:
        right_target = np.asarray(side_target, dtype=float)

    plan = {
        "stage": stage_name,
        "execute": bool(execute),
        "left_target": left_target.tolist(),
        "right_target": right_target.tolist(),
    }
    if not execute:
        return plan

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
    return {**plan, "result": result}


def p2p_stage_succeeded(stage: Mapping[str, Any]) -> bool:
    if not bool(stage.get("execute")):
        return True
    result = stage.get("result")
    return isinstance(result, Mapping) and bool(result.get("ok"))


def protected_insert(
    *,
    client: DualFrankaRobotiqRpcClient,
    side: str,
    insertion_axis_base: np.ndarray,
    depth_m: float,
    step_m: float,
    rate_hz: float,
    max_force_delta_n: float,
    execute: bool,
) -> dict[str, Any]:
    axis = normalize_vector(insertion_axis_base, "insertion_axis_base")
    depth_m = max(0.0, float(depth_m))
    step_m = max(1e-5, abs(float(step_m)))
    steps = int(np.ceil(depth_m / step_m)) if depth_m > 0.0 else 0
    action_key = side_arm_key(side)
    plan = {
        "stage": "protected_insert",
        "execute": bool(execute),
        "side": side,
        "axis_base": axis.tolist(),
        "depth_m": depth_m,
        "step_m": step_m,
        "steps": steps,
        "max_force_delta_n": float(max_force_delta_n),
    }
    if not execute or steps == 0:
        return plan

    observation, _poses = read_ee_poses(client)
    base_force = np.asarray(observation[action_key]["robot_state"]["wrench"]["force"], dtype=float)
    period = 1.0 / max(1e-6, float(rate_hz))
    travelled = 0.0
    samples: list[dict[str, Any]] = []
    for index in range(steps):
        remaining = depth_m - travelled
        current_step = min(step_m, remaining)
        translation = axis * current_step
        result = client.step(
            {
                action_key: {
                    "motion": {
                        "translation": translation.tolist(),
                        "rotation_rotvec": [0.0, 0.0, 0.0],
                    }
                }
            }
        )
        travelled += current_step
        observation, _poses = read_ee_poses(client)
        force = np.asarray(observation[action_key]["robot_state"]["wrench"]["force"], dtype=float)
        force_delta = float(np.linalg.norm(force - base_force))
        samples.append({"index": index + 1, "travelled_m": travelled, "force_delta_n": force_delta})
        if force_delta > max_force_delta_n:
            return {**plan, "ok": False, "stopped_reason": "force_delta_limit", "samples": samples, "last_step": result}
        time.sleep(period)
    return {**plan, "ok": True, "travelled_m": travelled, "samples": samples}


def _require_position_base(result: Mapping[str, Any], name: str) -> np.ndarray:
    position_base = result.get("position_base", {})
    if not isinstance(position_base, Mapping) or not position_base.get("available"):
        reason = position_base.get("reason") if isinstance(position_base, Mapping) else "missing position_base"
        raise RuntimeError(f"{name} has no available position_base: {reason}")
    return as_vector3(
        [position_base.get("x_m"), position_base.get("y_m"), position_base.get("z_m")],
        f"{name}.position_base",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server-host", default=os.environ.get("FRANKA_RPC_HOST", "172.16.0.1"))
    parser.add_argument("--server-port", type=int, default=int(os.environ.get("FRANKA_RPC_PORT", "4242")))
    parser.add_argument("--rpc-timeout-sec", type=float, default=float(os.environ.get("FRANKA_RPC_TIMEOUT_SEC", "30")))
    parser.add_argument("--any-pose-dir", type=Path, default=ANY_POSE_DIR)
    parser.add_argument(
        "--artifact-dir",
        type=Path,
        default=Path("/tmp/agentic_skills_runs/legacy_tube_insertion_skill"),
        help="Runtime artifacts/calibration output directory used unless --write-runtime-calibration-to-repo is set.",
    )
    parser.add_argument(
        "--write-runtime-calibration-to-repo",
        action="store_true",
        help="Explicitly allow writing live base_T_flange calibration files under --any-pose-dir.",
    )
    parser.add_argument("--rack-config", type=Path, default=ANY_POSE_DIR / "config_rack_center_vlm.yaml")
    parser.add_argument(
        "--rack-result-json",
        type=Path,
        default=None,
        help="Cached initial rack localization; skip head-camera capture and rack VLM inference.",
    )
    parser.add_argument(
        "--no-parallel-rack-localization",
        action="store_false",
        dest="parallel_rack_localization",
        default=True,
        help="Run rack localization serially after robot preparation instead of overlapping VLM inference.",
    )
    parser.add_argument(
        "--rack-capture-ready-timeout-sec",
        type=float,
        default=3.0,
        help="Head-camera no-frame watchdog before a forced RealSense reset (default: 3 seconds).",
    )
    parser.add_argument("--left-hole-config", type=Path, default=DEFAULT_HOLE_CONFIG["left"])
    parser.add_argument("--right-hole-config", type=Path, default=DEFAULT_HOLE_CONFIG["right"])
    parser.add_argument("--hole-result-json", type=Path, default=None)
    parser.add_argument("--holder-side", choices=("left", "right"), required=True)
    parser.add_argument("--observe-height-m", type=float, default=0.12)
    parser.add_argument(
        "--rack-plane-z-base-m",
        type=float,
        default=None,
        help=(
            "Manual rack/hole plane z in base frame. When set, rack and hole motion targets use this "
            "z instead of the depth-estimated rack/hole z while preserving detected xy."
        ),
    )
    parser.add_argument("--observe-offset", type=parse_json_vector3, default=np.zeros(3))
    parser.add_argument("--insertion-axis-base", type=parse_json_vector3, default=np.array([0.0, 0.0, -1.0]))
    parser.add_argument("--preinsert-standoff-m", type=float, default=0.04)
    parser.add_argument("--insert-depth-m", type=float, default=0.05)
    parser.add_argument("--insert-step-m", type=float, default=0.001)
    parser.add_argument("--insert-rate-hz", type=float, default=10.0)
    parser.add_argument("--max-force-delta-n", type=float, default=8.0)
    parser.add_argument("--tcp-to-tube-tip", type=parse_json_vector3, default=None)
    parser.add_argument(
        "--aligned-tcp-rotvec",
        type=parse_json_vector3,
        default=None,
        help="Explicit base-frame TCP rotvec to use for insertion alignment, bypassing tube-axis inference.",
    )
    parser.add_argument(
        "--align-tube-axis",
        action="store_true",
        help="Use the fixed TCP-to-tip offset for orientation instead of the default dynamic pitch=0 posture.",
    )
    parser.add_argument(
        "--pitch-zero-rad",
        type=float,
        default=0.0,
        help="Euler xyz pitch used by the default dynamic pitch-zero TCP posture.",
    )
    parser.add_argument(
        "--roll-target-rad",
        type=float,
        default=None,
        help=(
            "Optional Euler xyz roll target used by the default dynamic pitch-zero TCP posture. "
            "Use 3.141592653589793 to force the TCP z-axis vertical downward while preserving yaw."
        ),
    )
    parser.add_argument(
        "--no-yield-non-holder-arm",
        action="store_false",
        dest="yield_non_holder_arm",
        default=True,
        help="Do not move the arm that is not holding the tube to a clearance pose before rack localization.",
    )
    parser.add_argument(
        "--non-holder-yield-mode",
        choices=("home", "outward-y"),
        default="home",
        help=(
            "How to clear the non-holder arm before rack localization. home uses the server's "
            "stored single-arm joint home; outward-y preserves the previous relative Cartesian move."
        ),
    )
    parser.add_argument(
        "--yield-home-duration-sec",
        type=float,
        default=5.0,
        help="Duration of the non-holder single-arm home trajectory.",
    )
    parser.add_argument(
        "--yield-home-rate-hz",
        type=float,
        default=50.0,
        help="Command rate requested for the non-holder single-arm home trajectory.",
    )
    parser.add_argument(
        "--yield-distance-m",
        type=float,
        default=0.12,
        help="Outward y-axis distance used only when --non-holder-yield-mode=outward-y.",
    )
    parser.add_argument(
        "--no-align-tube-axis",
        action="store_true",
        help="Keep the current TCP orientation instead of aligning the detected tube axis to the hole axis.",
    )
    parser.add_argument("--execute", action="store_true", help="Allow P2P moves to observation and pre-insert poses.")
    parser.add_argument("--execute-insertion", action="store_true", help="Allow the final guarded insertion descent.")
    parser.add_argument(
        "--finish-after-insert",
        action="store_true",
        help="After a successful executed insertion, open the holder gripper and retract the holder arm.",
    )
    parser.add_argument("--after-release-sleep-sec", type=float, default=0.5)
    parser.add_argument(
        "--finish-retract-distance-m",
        type=float,
        default=0.12,
        help="Distance to move the holder TCP opposite the insertion axis after releasing the tube.",
    )
    parser.add_argument("--stop-after-observe", action="store_true", help="Stop after the rack observation move.")
    parser.add_argument("--skip-vision", action="store_true", help="Only validate imports/CLI; do not run cameras.")
    parser.add_argument("--rate_hz", "--rate-hz", dest="rate_hz", type=float, default=50.0)
    parser.add_argument("--max_translation_speed", "--max-translation-speed", dest="max_translation_speed", type=float, default=0.03)
    parser.add_argument("--max_rotation_speed", "--max-rotation-speed", dest="max_rotation_speed", type=float, default=0.20)
    parser.add_argument("--max_translation_step", "--max-translation-step", dest="max_translation_step", type=float, default=0.001)
    parser.add_argument("--max_rotation_step", "--max-rotation-step", dest="max_rotation_step", type=float, default=0.012)
    parser.add_argument("--settle_time_sec", "--settle-time-sec", dest="settle_time_sec", type=float, default=1.0)
    parser.add_argument("--position_tolerance_m", "--position-tolerance-m", dest="position_tolerance_m", type=float, default=0.003)
    parser.add_argument("--rotation_tolerance_rad", "--rotation-tolerance-rad", dest="rotation_tolerance_rad", type=float, default=0.03)
    parser.add_argument("--max_correction_iters", "--max-correction-iters", dest="max_correction_iters", type=int, default=1)
    parser.add_argument("--max_steps", "--max-steps", dest="max_steps", type=int, default=3000)
    parser.add_argument("--compact", action="store_true")
    return parser


def validate_args(args: argparse.Namespace) -> None:
    if args.execute_insertion and not args.execute:
        raise ValueError("--execute-insertion requires --execute")
    if args.finish_after_insert and not (args.execute and args.execute_insertion):
        raise ValueError("--finish-after-insert requires --execute and --execute-insertion")
    if not np.isfinite(args.yield_home_duration_sec) or args.yield_home_duration_sec <= 0.0:
        raise ValueError("--yield-home-duration-sec must be a positive finite value")
    if not np.isfinite(args.yield_home_rate_hz) or args.yield_home_rate_hz <= 0.0:
        raise ValueError("--yield-home-rate-hz must be a positive finite value")
    if not np.isfinite(args.rack_capture_ready_timeout_sec) or args.rack_capture_ready_timeout_sec <= 0.0:
        raise ValueError("--rack-capture-ready-timeout-sec must be a positive finite value")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    validate_args(args)

    if args.skip_vision:
        print(json.dumps({"ok": True, "message": "imports and CLI are valid"}, ensure_ascii=False))
        return 0

    runtime_calibration_dir = args.any_pose_dir if args.write_runtime_calibration_to_repo else args.artifact_dir / "object_locator_runtime"
    if not args.execute:
        report = {
            "execute": False,
            "execute_insertion": False,
            "planned_only": True,
            "requires_execute_for_rpc": True,
            "requires_execute_for_camera": args.hole_result_json is None,
            "requires_execute_for_motion": True,
            "requires_execute_for_gripper": bool(args.finish_after_insert),
            "abnormal_robot_state_detected": False,
            "holder": {"side": args.holder_side, "source": "explicit_holder_side"},
            "server": f"{args.server_host}:{args.server_port}",
            "artifact_dir": str(args.artifact_dir),
            "runtime_calibration_dir": str(runtime_calibration_dir),
            "rack_localization_concurrency": {
                "enabled": bool(args.parallel_rack_localization),
                "mode": "capture_then_parallel_inference" if args.parallel_rack_localization else "serial",
            },
            "stages": [
                {"stage": "observe_rack", "planned_only": True},
                {"stage": "locate_hole", "planned_only": True, "source": "artifact" if args.hole_result_json else "requires_live_camera"},
                {"stage": "preinsert_align", "planned_only": True},
                {"stage": "insert_descend", "planned_only": True, "requires_execute_insertion": True},
            ],
            "completion_flag": False,
            "physical_verified": False,
            "stopped_reason": "planned_only_requires_execute_for_rpc_camera_and_motion",
        }
        print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
        return 0

    pending_rack_locator: PendingObjectLocator | None = None
    rack_capture_ready_file: Path | None = None
    client = DualFrankaRobotiqRpcClient(ip=args.server_host, port=args.server_port, timeout=args.rpc_timeout_sec)
    try:
        report: dict[str, Any] = {
            "execute": bool(args.execute),
            "execute_insertion": bool(args.execute_insertion),
            "server": f"{args.server_host}:{args.server_port}",
            "stages": [],
        }
        report["ping"] = client.ping()

        observation, poses = read_ee_poses(client)
        runtime_calibration_dir.mkdir(parents=True, exist_ok=True)
        write_base_T_flange_files(runtime_calibration_dir, observation)
        report["runtime_calibration_dir"] = str(runtime_calibration_dir)

        holder_side = args.holder_side
        report["holder"] = {
            "status": "ok",
            "side": holder_side,
            "source": "explicit_holder_side",
            "reason": "holder side was selected by the caller before this stage",
        }
        holder_pose = poses[holder_side]
        tcp_to_tip = args.tcp_to_tube_tip
        if tcp_to_tip is None and args.align_tube_axis and args.aligned_tcp_rotvec is None and not args.no_align_tube_axis:
            raise RuntimeError(
                "Cannot align the tube axis without a fixed TCP-to-tip offset. Pass "
                "--tcp-to-tube-tip '[x,y,z]' or --aligned-tcp-rotvec '[rx,ry,rz]'."
            )
        report["tcp_to_tube_tip_m"] = None if tcp_to_tip is None else tcp_to_tip.tolist()

        desired_tube_axis_base = normalize_vector(args.insertion_axis_base, "insertion_axis_base")
        tube_axis_base = None
        axis_source = "dynamic_pitch_zero"
        if args.align_tube_axis:
            axis_source = "tcp_to_tip"
        if args.align_tube_axis and args.aligned_tcp_rotvec is None and not args.no_align_tube_axis:
            tube_axis_base = tube_tip_axis_from_tcp_offset(
                current_tcp_rotvec=holder_pose[3:],
                tcp_to_tip_m=tcp_to_tip,
            )
        preinsert_tcp_rotvec = holder_pose[3:].copy()
        axis_alignment: dict[str, Any] = {
            "enabled": not bool(args.no_align_tube_axis),
            "axis_source": axis_source,
            "desired_tube_axis_base": desired_tube_axis_base.tolist(),
        }
        if args.aligned_tcp_rotvec is not None:
            preinsert_tcp_rotvec = np.asarray(args.aligned_tcp_rotvec, dtype=float).reshape(3)
            axis_alignment.update(
                {
                    "enabled": True,
                    "axis_source": "explicit_tcp_rotvec",
                    "tcp_rotvec_before": holder_pose[3:].tolist(),
                    "tcp_rotvec_after": preinsert_tcp_rotvec.tolist(),
                }
            )
        elif args.no_align_tube_axis:
            axis_alignment["reason"] = "disabled by --no-align-tube-axis"
        elif not args.align_tube_axis:
            preinsert_tcp_rotvec = tcp_pitch_zero_rotvec(
                holder_pose,
                pitch_rad=args.pitch_zero_rad,
                roll_rad=args.roll_target_rad,
            )
            axis_alignment.update(
                {
                    "enabled": True,
                    "axis_source": "dynamic_pitch_zero",
                    "roll_target_rad": None if args.roll_target_rad is None else float(args.roll_target_rad),
                    "pitch_rad": float(args.pitch_zero_rad),
                    "tcp_rotvec_before": holder_pose[3:].tolist(),
                    "tcp_rotvec_after": preinsert_tcp_rotvec.tolist(),
                    "euler_xyz_before": R.from_rotvec(holder_pose[3:]).as_euler("xyz").tolist(),
                    "euler_xyz_after": R.from_rotvec(preinsert_tcp_rotvec).as_euler("xyz").tolist(),
                }
            )
        elif tube_axis_base is None:
            raise RuntimeError(
                "Cannot align tube axis from the configured TCP-to-tip offset."
            )
        else:
            preinsert_tcp_rotvec = align_tcp_orientation_to_tube_axis(
                current_tcp_rotvec=holder_pose[3:],
                tube_axis_base=tube_axis_base,
                desired_tube_axis_base=desired_tube_axis_base,
            )
            aligned_axis_base = R.from_rotvec(preinsert_tcp_rotvec).apply(
                R.from_rotvec(holder_pose[3:]).inv().apply(tube_axis_base)
            )
            axis_alignment.update(
                {
                    "tube_axis_base_before": tube_axis_base.tolist(),
                    "tube_axis_base_after": aligned_axis_base.tolist(),
                    "tcp_rotvec_before": holder_pose[3:].tolist(),
                    "tcp_rotvec_after": preinsert_tcp_rotvec.tolist(),
                }
            )
        report["axis_alignment"] = axis_alignment

        cached_rack = args.rack_result_json is not None
        rack_concurrency: dict[str, Any] = {
            "enabled": bool(args.parallel_rack_localization and not cached_rack),
            "mode": (
                "cached_initial_artifact"
                if cached_rack
                else "capture_then_parallel_inference"
                if args.parallel_rack_localization
                else "serial"
            ),
        }
        report["rack_localization_concurrency"] = rack_concurrency
        if cached_rack:
            rack_concurrency.update(
                {
                    "rack_result_json": str(args.rack_result_json),
                    "head_camera_capture_skipped": True,
                }
            )
            print(
                f"[tube-insertion] using cached initial rack detection: {args.rack_result_json}",
                file=sys.stderr,
                flush=True,
            )
        elif args.parallel_rack_localization:
            rack_capture_ready_file = args.artifact_dir / (
                f"rack_capture_ready_{os.getpid()}_{time.time_ns()}.json"
            )
            print(
                "[tube-insertion] capturing head-camera rack frame before robot preparation",
                file=sys.stderr,
                flush=True,
            )
            pending_rack_locator = start_object_locator_after_capture_signal(
                args.any_pose_dir,
                args.rack_config,
                rack_capture_ready_file,
            )
            try:
                capture_status = wait_for_object_locator_capture(
                    pending_rack_locator,
                    timeout_sec=args.rack_capture_ready_timeout_sec,
                )
            except TimeoutError:
                cancel_pending_object_locator(pending_rack_locator)
                print(
                    f"[tube-insertion] head RealSense produced no frame within "
                    f"{args.rack_capture_ready_timeout_sec:.1f}s; forcing reset",
                    file=sys.stderr,
                    flush=True,
                )
                pending_rack_locator, capture_status = recover_object_locator_after_camera_reset(
                    args.any_pose_dir,
                    args.rack_config,
                    rack_capture_ready_file,
                )
            rack_concurrency.update(
                {
                    "capture_ready_before_robot_motion": True,
                    "capture_wait_sec": float(capture_status["wait_sec"]),
                    "capture_signal": capture_status["signal"],
                }
            )
            print(
                "[tube-insertion] head-camera frame fixed; rack inference now overlaps robot preparation",
                file=sys.stderr,
                flush=True,
            )

        if args.yield_non_holder_arm:
            observation, poses = read_ee_poses(client)
            non_holder_side = OPPOSITE_SIDE[holder_side]
            if args.non_holder_yield_mode == "home":
                yield_stage = move_non_holder_to_home(
                    client=client,
                    holder_side=holder_side,
                    duration_sec=args.yield_home_duration_sec,
                    rate_hz=args.yield_home_rate_hz,
                    execute=args.execute,
                )
            else:
                yield_pose = compute_non_holder_yield_pose(
                    holder_side=holder_side,
                    non_holder_pose=poses[non_holder_side],
                    distance_m=args.yield_distance_m,
                )
                yield_stage = move_dual_absolute(
                    client=client,
                    side=non_holder_side,
                    side_target=yield_pose,
                    all_current=poses,
                    args=args,
                    stage_name="yield_non_holder_arm",
                    execute=args.execute,
                )
                yield_stage["mode"] = "outward_y"
            report["stages"].append(yield_stage)
            if not p2p_stage_succeeded(yield_stage):
                report["stopped_reason"] = "yield_non_holder_arm_failed"
                print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
                return 3
        else:
            report["stages"].append(
                {
                    "stage": "yield_non_holder_arm",
                    "execute": False,
                    "skipped": True,
                    "reason": "disabled by --no-yield-non-holder-arm",
                }
            )

        observation, poses = read_ee_poses(client)
        holder_pose = poses[holder_side]
        aligned_holder_pose = holder_pose.copy()
        aligned_holder_pose[3:] = preinsert_tcp_rotvec
        align_stage = move_dual_absolute(
            client=client,
            side=holder_side,
            side_target=aligned_holder_pose,
            all_current=poses,
            args=args,
            stage_name="align_holder_tcp_vertical",
            execute=args.execute,
        )
        report["stages"].append(align_stage)
        if not p2p_stage_succeeded(align_stage):
            report["stopped_reason"] = "align_holder_tcp_vertical_failed"
            print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
            return 3

        if args.execute:
            observation, poses = read_ee_poses(client)
            holder_pose = poses[holder_side]
            axis_alignment["tcp_rotvec_after_align_actual"] = holder_pose[3:].tolist()
            axis_alignment["posture_target_reused_after_align"] = True

        if args.rack_result_json is not None:
            rack_result = load_result_json(args.rack_result_json)
        elif pending_rack_locator is None:
            rack_result = run_object_locator(args.any_pose_dir, args.rack_config)
        else:
            post_preparation_wait_started = time.monotonic()
            rack_result = finish_pending_object_locator(pending_rack_locator)
            rack_concurrency.update(
                {
                    "post_preparation_wait_sec": time.monotonic() - post_preparation_wait_started,
                    "total_locator_sec": time.monotonic() - pending_rack_locator.started_monotonic,
                }
            )
            pending_rack_locator = None
            rack_capture_ready_file.unlink(missing_ok=True)
            rack_capture_ready_file = None
        rack_base = _require_position_base(rack_result, "rack")
        rack_base_for_motion = apply_rack_plane_z_for_motion(
            rack_base,
            rack_plane_z_base_m=args.rack_plane_z_base_m,
        )
        report["rack"] = {
            "source": "cached_initial_artifact" if args.rack_result_json is not None else "live_head_detection",
            "rack_result_json": None if args.rack_result_json is None else str(args.rack_result_json),
            "position_base_m": rack_base.tolist(),
            "position_base_for_motion_m": rack_base_for_motion.tolist(),
            "rack_plane_z_base_m": None if args.rack_plane_z_base_m is None else float(args.rack_plane_z_base_m),
            "run_id": rack_result.get("run_id"),
        }

        observation, poses = read_ee_poses(client)
        holder_pose = poses[holder_side]
        observe_pose = compute_rack_observation_pose(
            rack_base_m=rack_base_for_motion,
            holder_pose=holder_pose,
            tcp_orientation_rotvec=preinsert_tcp_rotvec,
            observe_offset_m=args.observe_offset,
            observe_height_m=args.observe_height_m,
        )
        observe_stage = move_dual_absolute(
            client=client,
            side=holder_side,
            side_target=observe_pose,
            all_current=poses,
            args=args,
            stage_name="move_holder_to_wrist_observation_pose",
            execute=args.execute,
        )
        report["stages"].append(observe_stage)
        if not p2p_stage_succeeded(observe_stage):
            report["stopped_reason"] = "move_holder_to_wrist_observation_pose_failed"
            print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
            return 3

        if args.execute:
            observation, poses = read_ee_poses(client)
            holder_pose = poses[holder_side]
            axis_alignment["tcp_rotvec_after_observe_before_correction_actual"] = holder_pose[3:].tolist()
            post_observe_correction_pose = compute_post_observe_vertical_correction_pose(
                current_pose=holder_pose,
                tcp_orientation_rotvec=preinsert_tcp_rotvec,
            )
            post_observe_correction_stage = move_dual_absolute(
                client=client,
                side=holder_side,
                side_target=post_observe_correction_pose,
                all_current=poses,
                args=args,
                stage_name="correct_holder_tcp_vertical_after_observe",
                execute=args.execute,
            )
            report["stages"].append(post_observe_correction_stage)
            if not p2p_stage_succeeded(post_observe_correction_stage):
                report["stopped_reason"] = "correct_holder_tcp_vertical_after_observe_failed"
                print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
                return 3

            observation, poses = read_ee_poses(client)
            holder_pose = poses[holder_side]
            axis_alignment["tcp_rotvec_after_observe_actual"] = holder_pose[3:].tolist()
            axis_alignment["posture_target_reused_after_observe"] = True

        if args.stop_after_observe:
            report["hole"] = {"skipped": True, "reason": "stopped after observation stage by --stop-after-observe"}
            print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
            return 0

        if not should_run_wrist_hole_detection(execute=args.execute, hole_result_json=args.hole_result_json):
            report["hole"] = {
                "skipped": True,
                "reason": "dry-run did not move wrist camera to rack observation pose; run with --execute or pass --hole-result-json",
            }
            print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
            return 0

        observation, poses = read_ee_poses(client)
        if args.execute:
            axis_alignment["tcp_rotvec_after_observe_actual"] = poses[holder_side][3:].tolist()
            axis_alignment["posture_target_reused_after_observe"] = True
        write_base_T_flange_files(args.any_pose_dir, observation)
        if args.hole_result_json is not None:
            hole_result = load_result_json(args.hole_result_json)
        else:
            hole_result = run_object_locator(
                args.any_pose_dir,
                args.left_hole_config if holder_side == "left" else args.right_hole_config,
            )
        hole_base = _require_position_base(hole_result, "hole")
        hole_base_for_motion = apply_rack_plane_z_for_motion(
            hole_base,
            rack_plane_z_base_m=args.rack_plane_z_base_m,
        )
        report["hole"] = {
            "position_base_m": hole_base.tolist(),
            "position_base_for_motion_m": hole_base_for_motion.tolist(),
            "rack_plane_z_base_m": None if args.rack_plane_z_base_m is None else float(args.rack_plane_z_base_m),
            "run_id": hole_result.get("run_id"),
        }

        holder_pose = poses[holder_side]
        preinsert_tcp_target = compute_tcp_plane_target(
            hole_base_m=hole_base_for_motion,
            insertion_axis_base=args.insertion_axis_base,
            standoff_m=args.preinsert_standoff_m,
            insertion_depth_m=0.0,
        )
        preinsert_pose = compute_tcp_plane_target_pose(
            holder_pose=holder_pose,
            hole_base_m=hole_base_for_motion,
            insertion_axis_base=args.insertion_axis_base,
            standoff_m=args.preinsert_standoff_m,
            insertion_depth_m=0.0,
            tcp_orientation_rotvec=preinsert_tcp_rotvec,
        )
        report["preinsert"] = {
            "tcp_target_base_m": preinsert_tcp_target.tolist(),
            "tcp_target_pose": preinsert_pose.tolist(),
        }
        final_tcp_target = compute_tcp_plane_target(
            hole_base_m=hole_base_for_motion,
            insertion_axis_base=args.insertion_axis_base,
            standoff_m=args.preinsert_standoff_m,
            insertion_depth_m=args.insert_depth_m,
        )
        report["final_tcp_target_after_insert_m"] = final_tcp_target.tolist()
        preinsert_stage = move_dual_absolute(
            client=client,
            side=holder_side,
            side_target=preinsert_pose,
            all_current=poses,
            args=args,
            stage_name="move_holder_to_preinsert_pose",
            execute=args.execute,
        )
        report["stages"].append(preinsert_stage)
        if not p2p_stage_succeeded(preinsert_stage):
            report["stopped_reason"] = "move_holder_to_preinsert_pose_failed"
            print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
            return 3

        observation, poses = read_ee_poses(client)
        holder_pose = poses[holder_side]
        if args.execute:
            axis_alignment["tcp_rotvec_after_preinsert_actual"] = holder_pose[3:].tolist()
            axis_alignment["posture_target_reused_after_preinsert"] = True
        insert_pose = compute_tcp_plane_target_pose(
            holder_pose=holder_pose,
            hole_base_m=hole_base_for_motion,
            insertion_axis_base=args.insertion_axis_base,
            standoff_m=args.preinsert_standoff_m,
            insertion_depth_m=args.insert_depth_m,
            tcp_orientation_rotvec=preinsert_tcp_rotvec,
        )
        report["insert"] = {
            "tcp_target_base_m": final_tcp_target.tolist(),
            "tcp_target_pose": insert_pose.tolist(),
        }
        insert_stage = move_dual_absolute(
            client=client,
            side=holder_side,
            side_target=insert_pose,
            all_current=poses,
            args=args,
            stage_name="move_holder_to_insert_pose",
            execute=args.execute_insertion,
        )
        report["stages"].append(insert_stage)
        if not p2p_stage_succeeded(insert_stage):
            report["stopped_reason"] = "move_holder_to_insert_pose_failed"
            print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
            return 3

        if args.finish_after_insert:
            release_result = client.open_gripper(side_arm_key(holder_side))
            finish_report: dict[str, Any] = {
                "release_holder_gripper": release_result,
                "retract_distance_m": float(args.finish_retract_distance_m),
            }
            if args.after_release_sleep_sec > 0:
                time.sleep(args.after_release_sleep_sec)

            observation, poses = read_ee_poses(client)
            holder_pose = poses[holder_side]
            retract_pose = compute_retract_pose(
                holder_pose=holder_pose,
                insertion_axis_base=args.insertion_axis_base,
                retract_distance_m=args.finish_retract_distance_m,
            )
            retract_stage = move_dual_absolute(
                client=client,
                side=holder_side,
                side_target=retract_pose,
                all_current=poses,
                args=args,
                stage_name="retract_holder_after_release",
                execute=True,
            )
            report["stages"].append(retract_stage)
            finish_report["retract_target_pose"] = retract_pose.tolist()
            finish_report["retract_stage"] = retract_stage
            report["finish"] = finish_report
            if not p2p_stage_succeeded(retract_stage):
                report["stopped_reason"] = "retract_holder_after_release_failed"
                print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
                return 3

        print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
        return 0
    finally:
        if pending_rack_locator is not None:
            cancel_pending_object_locator(pending_rack_locator)
        if rack_capture_ready_file is not None:
            rack_capture_ready_file.unlink(missing_ok=True)
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
