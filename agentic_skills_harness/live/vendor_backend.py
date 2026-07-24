"""Fixed Dual-Franka vendor binding used only by the S10H operator runner.

Importing this module is hardware-free.  The bundled ZeroRPC client is loaded
from one repository-relative path only after the runner has passed every
operator and hardware authorization gate.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import importlib.util
import ipaddress
import json
import math
from pathlib import Path
import re
from types import ModuleType
from typing import Any

from agentic_skills_harness.live.fingerprint import digest_json
from agentic_skills_harness.meta_operations.motion import MoveRelativeContract, MoveToPoseContract
from agentic_skills_harness.safety.estop_guard import EStopState
from agentic_skills_harness.safety.held_object_guard import HeldObjectState
from agentic_skills_harness.safety.motion_limits import MotionLimits
from agentic_skills_harness.safety.recovery_guard import RecoveryGuard
from agentic_skills_harness.safety.workspace import WorkspaceConfig


FIXED_BACKEND_ID = "dual_franka_robotiq_zerorpc.s10.v1"
FIXED_CLIENT_RELATIVE_PATH = Path(
    "atomic_skills/dual_franka_p2p/skills/"
    "atomic-motion-franka-move-to-pose/dual_franka_robotiq_rpc_client.py"
)
FIXED_RPC_PORT = 4242
EXPECTED_ROBOT_TYPE = "franka_dual_arm"
EXPECTED_GRIPPER_TYPE = "dual_robotiq_2f_85"
EXPECTED_ARM_IDS = ("left_arm", "right_arm")
EXPECTED_PROTOCOL_PREFIX = "zerorpc-nero_compatible@sha256:"
_CONTROLLER_PATTERN = re.compile(
    r"dual_franka_robotiq_rpc_server@(?P<host>[0-9]{1,3}(?:\.[0-9]{1,3}){3}):(?P<port>[0-9]{1,5})"
)
_SIDES = {"left": "left_arm", "right": "right_arm"}
_UNKNOWN = "UNKNOWN"


class VendorBindingError(ValueError):
    """The local hardware configuration does not match the fixed binding."""


class VendorCapabilityUnsupported(RuntimeError):
    """The audited vendor API does not expose a required primitive."""


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _finite_vector(value: Any, length: int, name: str) -> list[float]:
    if isinstance(value, (str, bytes, bytearray)) or not isinstance(value, Sequence):
        raise VendorBindingError(f"{name}_must_be_vector")
    if len(value) < length:
        raise VendorBindingError(f"{name}_must_have_{length}_values")
    result = [float(item) for item in value[:length]]
    if not all(math.isfinite(item) for item in result):
        raise VendorBindingError(f"{name}_must_be_finite")
    return result


def _as_mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def _quat_to_rotvec(quaternion: Any) -> list[float]:
    x, y, z, w = _finite_vector(quaternion, 4, "orientation_xyzw")
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise VendorBindingError("orientation_xyzw_zero_norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    sin_half = math.sqrt(x * x + y * y + z * z)
    if sin_half <= 1e-12:
        return [0.0, 0.0, 0.0]
    angle = 2.0 * math.atan2(sin_half, w)
    scale = angle / sin_half
    return [x * scale, y * scale, z * scale]


def _side_state(observation: Mapping[str, Any], side: str) -> Mapping[str, Any]:
    return _as_mapping(observation.get(_SIDES[side]))


def _robot_state(observation: Mapping[str, Any], side: str) -> Mapping[str, Any]:
    side_value = _side_state(observation, side)
    nested = _as_mapping(side_value.get("robot_state"))
    return nested or side_value


def _extract_pose(observation: Mapping[str, Any], side: str) -> list[float]:
    side_value = _side_state(observation, side)
    state = _robot_state(observation, side)
    for container in (side_value, state):
        if "end_pose" in container:
            return _finite_vector(container["end_pose"], 6, f"{side}_end_pose")
    eef_pose = _as_mapping(state.get("eef_pose"))
    if eef_pose:
        return _finite_vector(eef_pose.get("position"), 3, f"{side}_eef_position") + _quat_to_rotvec(
            eef_pose.get("orientation_xyzw")
        )
    axes = ("x", "y", "z", "rx", "ry", "rz")
    flat = [observation.get(f"{side}_ee_pose.{axis}") for axis in axes]
    if all(item is not None for item in flat):
        return _finite_vector(flat, 6, f"{side}_flat_end_pose")
    raise VendorBindingError(f"{side}_end_pose_unavailable")


def _extract_joints(observation: Mapping[str, Any], side: str) -> list[float] | str:
    state = _robot_state(observation, side)
    if state.get("joint_positions") is not None:
        return _finite_vector(state["joint_positions"], 7, f"{side}_joint_positions")
    flat = [observation.get(f"{side}_joint_{index}.pos") for index in range(1, 8)]
    if all(item is not None for item in flat):
        return _finite_vector(flat, 7, f"{side}_flat_joint_positions")
    return _UNKNOWN


def _extract_gripper(observation: Mapping[str, Any], side: str) -> Mapping[str, Any] | str:
    value = _side_state(observation, side).get("gripper")
    if isinstance(value, Mapping):
        return dict(value)
    if value is not None:
        return {"position": value}
    for suffix in ("gripper_state_norm", "gripper_cmd", "gripper_cmd_bin"):
        flat = observation.get(f"{side}_{suffix}")
        if flat is not None:
            return {"normalized": flat, "source": suffix}
    return _UNKNOWN


def _nested_value(observation: Mapping[str, Any], side: str, *names: str) -> Any:
    side_value = _side_state(observation, side)
    state = _robot_state(observation, side)
    for container in (state, side_value, observation):
        for name in names:
            if name in container:
                return container[name]
    return _UNKNOWN


@dataclass(frozen=True)
class DualFrankaVendorBinding:
    """Validated, non-user-selectable binding metadata."""

    repo_root: Path
    host: str
    port: int
    timeout_s: float
    client_path: Path
    client_sha256: str
    hardware_fingerprint_digest: str

    @classmethod
    def from_config(
        cls,
        repo_root: Path,
        config: Mapping[str, Any],
    ) -> "DualFrankaVendorBinding":
        repo = repo_root.resolve()
        hardware = _as_mapping(config.get("hardware"))
        if hardware.get("robot_type") != EXPECTED_ROBOT_TYPE:
            raise VendorBindingError("fixed_backend_robot_type_mismatch")
        if hardware.get("gripper_type") != EXPECTED_GRIPPER_TYPE:
            raise VendorBindingError("fixed_backend_gripper_type_mismatch")
        if tuple(hardware.get("arm_ids", ())) != EXPECTED_ARM_IDS:
            raise VendorBindingError("fixed_backend_arm_ids_mismatch")
        protocol = hardware.get("rpc_protocol_version")
        if not isinstance(protocol, str) or not protocol.startswith(EXPECTED_PROTOCOL_PREFIX):
            raise VendorBindingError("fixed_backend_rpc_protocol_mismatch")

        controller = hardware.get("controller_type")
        match = _CONTROLLER_PATTERN.fullmatch(str(controller))
        if match is None:
            raise VendorBindingError("fixed_backend_controller_type_mismatch")
        address = ipaddress.ip_address(match.group("host"))
        if address.version != 4 or not address.is_private:
            raise VendorBindingError("fixed_backend_private_ipv4_required")
        port = int(match.group("port"))
        if port != FIXED_RPC_PORT:
            raise VendorBindingError("fixed_backend_rpc_port_mismatch")

        timeout_s = float(_as_mapping(config.get("limits")).get("max_timeout_s", 30.0))
        if not math.isfinite(timeout_s) or timeout_s <= 0.0 or timeout_s > 30.0:
            raise VendorBindingError("fixed_backend_timeout_out_of_range")

        client_candidate = repo / FIXED_CLIENT_RELATIVE_PATH
        client_path = client_candidate.resolve()
        if (
            not client_path.is_file()
            or client_candidate.is_symlink()
            or not client_path.is_relative_to(repo)
        ):
            raise VendorBindingError("fixed_backend_client_missing_or_untrusted")

        required_fingerprint_fields = (
            "workspace_digest",
            "calibration_hash",
            "reset_config_digest",
        )
        if any(not isinstance(hardware.get(name), str) or not hardware[name] for name in required_fingerprint_fields):
            raise VendorBindingError("fixed_backend_hardware_fingerprint_incomplete")
        expected_workspace_digest = "sha256:" + digest_json(config.get("workspace", {}))
        if hardware["workspace_digest"] != expected_workspace_digest:
            raise VendorBindingError("fixed_backend_workspace_digest_mismatch")
        if _as_mapping(config.get("acceptance")).get("calibration_hash") != hardware["calibration_hash"]:
            raise VendorBindingError("fixed_backend_calibration_hash_mismatch")

        public_fingerprint = {
            "robot_type": hardware["robot_type"],
            "controller_binding": "dual_franka_robotiq_rpc_server:private-ipv4:4242",
            "rpc_protocol_version": protocol,
            "arm_ids": list(EXPECTED_ARM_IDS),
            "gripper_type": hardware["gripper_type"],
            "workspace_digest": hardware["workspace_digest"],
            "calibration_hash": hardware["calibration_hash"],
            "reset_config_digest": hardware["reset_config_digest"],
        }
        return cls(
            repo,
            str(address),
            port,
            timeout_s,
            client_path,
            _sha256_file(client_path),
            digest_json(public_fingerprint),
        )

    @property
    def adapter_digest(self) -> str:
        backend_path = Path(__file__).resolve()
        return digest_json(
            {
                "backend_id": FIXED_BACKEND_ID,
                "backend_sha256": _sha256_file(backend_path),
                "client_relative_path": FIXED_CLIENT_RELATIVE_PATH.as_posix(),
                "client_sha256": self.client_sha256,
                "rpc_port": self.port,
            }
        )

    def public_dict(self) -> dict[str, Any]:
        return {
            "backend_id": FIXED_BACKEND_ID,
            "adapter_digest": self.adapter_digest,
            "client_relative_path": FIXED_CLIENT_RELATIVE_PATH.as_posix(),
            "client_sha256": self.client_sha256,
            "endpoint": "private-ipv4:4242",
            "timeout_s": self.timeout_s,
            "hardware_fingerprint_digest": self.hardware_fingerprint_digest,
        }


def _load_fixed_client_module(binding: DualFrankaVendorBinding) -> ModuleType:
    module_name = "_agentic_s10_fixed_dual_franka_rpc_client"
    spec = importlib.util.spec_from_file_location(module_name, binding.client_path)
    if spec is None or spec.loader is None:
        raise VendorBindingError("fixed_backend_client_loader_unavailable")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _production_client_factory(binding: DualFrankaVendorBinding) -> Any:
    module = _load_fixed_client_module(binding)
    client_class = getattr(module, "DualFrankaRobotiqRpcClient", None)
    if not callable(client_class):
        raise VendorBindingError("fixed_backend_client_class_unavailable")
    return client_class(ip=binding.host, port=binding.port, timeout=binding.timeout_s)


class FixedDualFrankaVendorBackend:
    """Narrow S10H backend with no arbitrary RPC method or argument passthrough."""

    def __init__(
        self,
        binding: DualFrankaVendorBinding,
        *,
        client_factory: Callable[[DualFrankaVendorBinding], Any] | None = None,
    ) -> None:
        self.binding = binding
        self._client_factory = client_factory or _production_client_factory
        self._client: Any | None = None
        self._backend_calls = 0
        self._rpc_connection_count = 0

    @classmethod
    def from_config(
        cls,
        repo_root: Path,
        config: Mapping[str, Any],
    ) -> "FixedDualFrankaVendorBackend":
        return cls(DualFrankaVendorBinding.from_config(repo_root, config))

    @property
    def backend_calls(self) -> int:
        return self._backend_calls

    @property
    def rpc_connection_count(self) -> int:
        return self._rpc_connection_count

    def _connected_client(self) -> Any:
        if self._client is None:
            self._client = self._client_factory(self.binding)
            self._rpc_connection_count += 1
        return self._client

    def _invoke(self, method_name: str, *args: Any, **kwargs: Any) -> Any:
        client = self._connected_client()
        method = getattr(client, method_name, None)
        if not callable(method):
            raise VendorCapabilityUnsupported(f"vendor_method_unavailable:{method_name}")
        self._backend_calls += 1
        return method(*args, **kwargs)

    def ping(self) -> Mapping[str, Any]:
        result = self._invoke("ping")
        if not isinstance(result, Mapping) or result.get("ok") is not True:
            raise RuntimeError("vendor_ping_failed")
        return dict(result)

    def read_robot_state(self) -> dict[str, Any]:
        observation = self._invoke("get_observation")
        if not isinstance(observation, Mapping):
            raise RuntimeError("vendor_observation_not_mapping")
        errors: list[str] = []
        side_values: dict[str, dict[str, Any]] = {}
        for side in ("left", "right"):
            try:
                pose: list[float] | str = _extract_pose(observation, side)
            except VendorBindingError as exc:
                pose = _UNKNOWN
                errors.append(str(exc))
            try:
                joints: list[float] | str = _extract_joints(observation, side)
            except VendorBindingError as exc:
                joints = _UNKNOWN
                errors.append(str(exc))
            side_values[side] = {
                "joint_positions": joints,
                "ee_pose": pose,
                "gripper_state": _extract_gripper(observation, side),
                "wrench": _nested_value(observation, side, "wrench", "external_wrench"),
                "controller_state": _nested_value(observation, side, "controller_state"),
                "current_errors": _nested_value(observation, side, "current_errors", "errors"),
                "robot_mode": _nested_value(observation, side, "robot_mode"),
                "motion_state": _nested_value(observation, side, "motion_state", "is_moving"),
                "estop_state": _nested_value(observation, side, "estop_state", "estop"),
            }
        normalized: dict[str, Any] = {
            "ok": True,
            "source": FIXED_BACKEND_ID,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "left": side_values["left"],
            "right": side_values["right"],
            "errors": errors,
        }
        known = []
        for side in ("left", "right"):
            known.extend(
                normalized[side][name] != _UNKNOWN
                for name in (
                    "joint_positions",
                    "ee_pose",
                    "gripper_state",
                    "wrench",
                    "controller_state",
                    "current_errors",
                    "robot_mode",
                    "motion_state",
                    "estop_state",
                )
            )
        normalized["data_integrity"] = "OK" if all(known) else "PARTIAL"
        normalized["raw_observation"] = json.loads(json.dumps(observation, allow_nan=False))
        return normalized

    def command_gripper(self, *, side: str, operation: str) -> Any:
        if side not in _SIDES:
            raise VendorBindingError("gripper_side_enum")
        methods = {
            "open": "open_gripper",
            "close": "close_gripper",
            "initialize": "reactivate_gripper",
        }
        if operation == "status":
            return self.read_robot_state()[side]["gripper_state"]
        if operation not in methods:
            raise VendorBindingError("gripper_operation_enum")
        return self._invoke(methods[operation], _SIDES[side])

    def move_to_pose(
        self,
        *,
        side: str,
        pose: Mapping[str, Any],
        workspace: WorkspaceConfig,
        limits: MotionLimits,
    ) -> Any:
        if side not in _SIDES:
            raise VendorBindingError("motion_side_enum")
        validated = MoveToPoseContract(workspace, limits).validate(
            pose,
            side=side,
            translation_speed_m_s=limits.max_translation_speed_m_s,
            rotation_speed_rad_s=limits.max_rotation_speed_rad_s,
            timeout_s=limits.max_timeout_s,
        )
        current = self.read_robot_state()
        if current["left"]["ee_pose"] == _UNKNOWN or current["right"]["ee_pose"] == _UNKNOWN:
            raise VendorBindingError("current_end_pose_unavailable")
        left_target = list(current["left"]["ee_pose"])
        right_target = list(current["right"]["ee_pose"])
        target = validated["xyz_m"] + validated["rotvec_rad"]
        if side == "left":
            left_target = target
        else:
            right_target = target
        result = self._invoke(
            "dual_robot_move_to_ee_pose",
            left_target,
            right_target,
            delta=False,
            wait=True,
            smooth=True,
            max_translation_speed=limits.max_translation_speed_m_s,
            max_rotation_speed=limits.max_rotation_speed_rad_s,
            max_translation_step=limits.max_translation_step_m,
            max_rotation_step=limits.max_rotation_step_rad,
            position_tolerance_m=0.005,
            rotation_tolerance_rad=0.087266,
        )
        if isinstance(result, Mapping) and result.get("ok") is False:
            raise RuntimeError("vendor_motion_verification_failed")
        return result

    def move_relative(
        self,
        *,
        side: str,
        delta_xyz_m: Sequence[float],
        delta_rotvec_rad: Sequence[float],
        reference_frame: str,
        workspace: WorkspaceConfig,
        limits: MotionLimits,
    ) -> Any:
        if side not in _SIDES:
            raise VendorBindingError("motion_side_enum")
        current = self.read_robot_state()
        if current[side]["ee_pose"] == _UNKNOWN:
            raise VendorBindingError("current_end_pose_unavailable")
        current_pose = {
            "frame": workspace.robot_base_frame,
            "xyz_m": list(current[side]["ee_pose"][:3]),
            "rotvec_rad": list(current[side]["ee_pose"][3:]),
        }
        target = MoveRelativeContract(MoveToPoseContract(workspace, limits)).compute_target(
            current_pose,
            delta_xyz_m=delta_xyz_m,
            delta_rotvec_rad=delta_rotvec_rad,
            reference_frame=reference_frame,
            side=side,
            freshness_s=0.0,
        )
        return self.move_to_pose(side=side, pose=target, workspace=workspace, limits=limits)

    def safe_stop(self) -> dict[str, Any]:
        return {
            "ok": False,
            "status": "CAPABILITY_UNSUPPORTED",
            "verified": False,
            "reason": "vendor_cancel_hold_servo_stop_api_unavailable",
            "backend_calls": 0,
        }

    def verify_grasp(self) -> dict[str, Any]:
        return {
            "status": "CAPABILITY_UNSUPPORTED",
            "verified": False,
            "evidence_types": [],
            "confidence": None,
            "limited": True,
            "reason": "independent_grasp_evidence_unavailable",
            "backend_calls": 0,
        }

    def recover_fault(
        self,
        *,
        side: str,
        estop_state: EStopState | str,
        held_object_state: HeldObjectState | str,
        current_error_known: bool,
        allowed_as_recovery: bool,
    ) -> Any:
        if side not in {*_SIDES, "both"}:
            raise VendorBindingError("recovery_side_enum")
        allowed, reason = RecoveryGuard().allow(
            estop_state=estop_state,
            held_object_state=held_object_state,
            current_error_known=current_error_known,
            allowed_as_recovery=allowed_as_recovery,
        )
        if not allowed:
            raise PermissionError(reason)
        return self._invoke("recover_robot", _SIDES.get(side, "both"))

    def reset_home(
        self,
        *,
        estop_state: EStopState | str,
        held_object_state: HeldObjectState | str,
        allowed_as_recovery: bool,
    ) -> Any:
        allowed, reason = RecoveryGuard().allow(
            estop_state=estop_state,
            held_object_state=held_object_state,
            current_error_known=True,
            allowed_as_recovery=allowed_as_recovery,
        )
        if not allowed:
            raise PermissionError(reason)
        return self._invoke("go_home", "both", None, None)

    def close(self) -> None:
        if self._client is None:
            return
        close = getattr(self._client, "close", None)
        try:
            if callable(close):
                close()
        finally:
            self._client = None

    def __enter__(self) -> "FixedDualFrankaVendorBackend":
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.close()
