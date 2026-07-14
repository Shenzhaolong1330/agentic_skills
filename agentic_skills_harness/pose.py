from __future__ import annotations

import math
from typing import Any

from .types import Error6DoF, Pose6D

DEFAULT_TRANSLATION_THRESHOLD_M = 0.005
# 原任务只明确 5 mm 平移要求；旋转阈值是新增默认配置，约等于 5 degrees，可 CLI override。
DEFAULT_ROTATION_THRESHOLD_RAD = 0.0872664626


def validate_xyz_m(value: Any, name: str = "xyz_m") -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{name} must be a length-3 sequence")
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} contains non-finite values")
    return result


def validate_rotvec_rad(value: Any, name: str = "rotvec_rad") -> list[float]:
    if value is None:
        return [0.0, 0.0, 0.0]
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{name} must be a length-3 sequence")
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} contains non-finite values")
    return result


def normalize_pose6d(value: Any, *, frame: str = "base", source: str = "unknown") -> Pose6D:
    if isinstance(value, Pose6D):
        return value
    if isinstance(value, dict):
        xyz = value.get("xyz_m") or [value.get("x_m"), value.get("y_m"), value.get("z_m")]
        return Pose6D(
            frame=str(value.get("frame") or frame),
            xyz_m=validate_xyz_m(xyz),
            rotvec_rad=None if value.get("rotvec_rad") is None else validate_rotvec_rad(value.get("rotvec_rad")),
            rpy_rad=value.get("rpy_rad"),
            quaternion_xyzw=value.get("quaternion_xyzw"),
            source=str(value.get("source") or source),
            timestamp=value.get("timestamp"),
            confidence=value.get("confidence"),
            quality=value.get("quality"),
        )
    if isinstance(value, (list, tuple)) and len(value) in (3, 6):
        xyz = validate_xyz_m(value[:3])
        rotvec = validate_rotvec_rad(value[3:6]) if len(value) == 6 else None
        return Pose6D(frame=frame, xyz_m=xyz, rotvec_rad=rotvec, source=source)
    raise ValueError(f"cannot normalize pose from {type(value).__name__}")


def build_error_6dof(
    *,
    translation_m: float,
    rotation_rad: float,
    translation_threshold_m: float = DEFAULT_TRANSLATION_THRESHOLD_M,
    rotation_threshold_rad: float = DEFAULT_ROTATION_THRESHOLD_RAD,
    source: str = "computed",
) -> Error6DoF:
    translation = abs(float(translation_m))
    rotation = abs(float(rotation_rad))
    return Error6DoF(
        translation_m=translation,
        rotation_rad=rotation,
        translation_threshold_m=float(translation_threshold_m),
        rotation_threshold_rad=float(rotation_threshold_rad),
        within_threshold=translation <= float(translation_threshold_m) and rotation <= float(rotation_threshold_rad),
        source=source,
    )


def within_6dof_threshold(error: Error6DoF | dict[str, Any]) -> bool:
    if isinstance(error, Error6DoF):
        return bool(error.within_threshold)
    return bool(error.get("within_threshold"))
