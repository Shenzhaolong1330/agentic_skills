from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Callable, Mapping

import numpy as np
from scipy.spatial.transform import Rotation

from ..safety.motion_limits import MotionLimits
from ..safety.workspace import WorkspaceConfig


def _vec(value: Any, name: str, size: int = 3) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)): raise ValueError(f"{name}_invalid")
    return result


@dataclass(frozen=True)
class MoveToPoseContract:
    workspace: WorkspaceConfig = WorkspaceConfig()
    limits: MotionLimits = MotionLimits()

    def validate(self, pose: Mapping[str, Any], *, side: str, translation_speed_m_s: float = 0.01, rotation_speed_rad_s: float = 0.05, timeout_s: float = 10.0) -> dict[str, Any]:
        if side not in {"left", "right", "both"}: raise ValueError("side_enum")
        frame = pose.get("frame")
        xyz = _vec(pose.get("xyz_m"), "xyz_m")
        rotvec = _vec(pose.get("rotvec_rad", (0, 0, 0)), "rotvec_rad")
        if not isinstance(pose.get("source"), str) or not pose["source"]: raise ValueError("pose_source_required")
        self.workspace.validate_pose(frame=frame, xyz_m=xyz.tolist(), side=side)
        self.limits.validate(translation_speed_m_s=translation_speed_m_s, rotation_speed_rad_s=rotation_speed_rad_s, timeout_s=timeout_s)
        return {"frame": frame, "side": side, "xyz_m": xyz.tolist(), "rotvec_rad": rotvec.tolist(), "timeout_s": timeout_s, "translation_speed_m_s": translation_speed_m_s, "rotation_speed_rad_s": rotation_speed_rad_s}


@dataclass(frozen=True)
class MoveRelativeContract:
    move_to_pose: MoveToPoseContract = MoveToPoseContract()

    def compute_target(self, current_pose: Mapping[str, Any], *, delta_xyz_m: Any, delta_rotvec_rad: Any, reference_frame: str, side: str, freshness_s: float | None, max_age_s: float = 1.0) -> dict[str, Any]:
        if reference_frame not in {"base", "tool"}: raise ValueError("relative_reference_frame_required")
        if freshness_s is None or not math.isfinite(float(freshness_s)) or freshness_s > max_age_s: raise ValueError("current_pose_stale")
        frame = current_pose.get("frame")
        if frame != self.move_to_pose.workspace.robot_base_frame: raise ValueError("current_pose_base_frame_required")
        current_xyz = _vec(current_pose.get("xyz_m"), "current_xyz_m")
        delta_xyz = _vec(delta_xyz_m, "delta_xyz_m")
        delta_rot = _vec(delta_rotvec_rad, "delta_rotvec_rad")
        if np.linalg.norm(delta_xyz) > self.move_to_pose.limits.max_relative_displacement_m:
            raise ValueError("relative_translation_limit_exceeded")
        if np.linalg.norm(delta_rot) > self.move_to_pose.limits.max_rotation_step_rad: raise ValueError("relative_rotation_limit_exceeded")
        current_rot = Rotation.from_rotvec(_vec(current_pose.get("rotvec_rad", (0, 0, 0)), "current_rotvec_rad"))
        target_xyz = current_xyz + (current_rot.apply(delta_xyz) if reference_frame == "tool" else delta_xyz)
        target_rot = current_rot * Rotation.from_rotvec(delta_rot) if reference_frame == "tool" else Rotation.from_rotvec(delta_rot) * current_rot
        target = {"frame": self.move_to_pose.workspace.robot_base_frame, "xyz_m": target_xyz.tolist(), "rotvec_rad": target_rot.as_rotvec().tolist(), "source": "motion.move_relative.composed"}
        self.move_to_pose.validate(target, side=side)
        return target

    def execute(self, observe_current: Callable[[], Mapping[str, Any]], dispatch_move_to_pose: Callable[[Mapping[str, Any]], Any], **kwargs: Any) -> Any:
        current = observe_current()
        target = self.compute_target(current, **kwargs)
        return dispatch_move_to_pose(target)


@dataclass(frozen=True)
class GuardedMoveContract:
    def validate(self, *, max_distance_m: float, max_speed_m_s: float, max_force_n: float, max_torque_nm: float, timeout_s: float, contact_direction: Any, contact_success_condition: str, retreat_policy: str, controller: Any | None = None) -> dict[str, Any]:
        required = (max_distance_m, max_speed_m_s, max_force_n, max_torque_nm, timeout_s)
        if any(not math.isfinite(float(value)) or float(value) <= 0 for value in required) or not contact_direction or not contact_success_condition or not retreat_policy: raise ValueError("guarded_move_limits_and_contact_contract_required")
        if controller is None or not callable(getattr(controller, "guarded_move", None)) or not any(callable(getattr(controller, name, None)) for name in ("stop", "hold", "cancel_motion")):
            return {"status": "CAPABILITY_UNSUPPORTED", "live_readiness": "DISABLED", "reason": "realtime_guarded_stop_api_unavailable"}
        return {"status": "IMPLEMENTATION_READY", "live_readiness": "HARDWARE_ACCEPTANCE_PENDING", "max_distance_m": max_distance_m, "max_speed_m_s": max_speed_m_s, "max_force_n": max_force_n, "max_torque_nm": max_torque_nm, "timeout_s": timeout_s}
