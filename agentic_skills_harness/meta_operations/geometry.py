from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Iterable, Mapping

import numpy as np
from scipy.spatial.transform import Rotation


class TransformPoseError(ValueError):
    pass


def _finite(values: Iterable[float], name: str) -> np.ndarray:
    result = np.asarray(list(values), dtype=float)
    if result.ndim != 1 or not np.all(np.isfinite(result)):
        raise TransformPoseError(f"{name}_non_finite")
    return result


@dataclass(frozen=True)
class Transform:
    source_frame: str
    target_frame: str
    translation_m: tuple[float, float, float]
    rotvec_rad: tuple[float, float, float] = (0.0, 0.0, 0.0)
    calibration_hash: str = ""

    def matrix(self) -> np.ndarray:
        translation = _finite(self.translation_m, "translation")
        rotvec = _finite(self.rotvec_rad, "rotvec")
        if translation.size != 3 or rotvec.size != 3: raise TransformPoseError("transform_shape")
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
        matrix[:3, 3] = translation
        return matrix

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "Transform":
        return cls(str(data["source_frame"]), str(data["target_frame"]), tuple(_finite(data["translation_m"], "translation").tolist()), tuple(_finite(data.get("rotvec_rad", (0, 0, 0)), "rotvec").tolist()), str(data.get("calibration_hash", "")))


@dataclass(frozen=True)
class TransformChain:
    transforms: tuple[Transform, ...]
    calibration_hash: str

    def matrix(self, source_frame: str, target_frame: str, *, expected_calibration_hash: str) -> np.ndarray:
        if expected_calibration_hash != self.calibration_hash: raise TransformPoseError("calibration_hash_mismatch")
        if source_frame == target_frame: return np.eye(4)
        current = source_frame
        result = np.eye(4)
        for item in self.transforms:
            if item.calibration_hash and item.calibration_hash != expected_calibration_hash: raise TransformPoseError("calibration_hash_mismatch")
            if item.source_frame == current:
                result = item.matrix() @ result
                current = item.target_frame
            elif item.target_frame == current:
                result = np.linalg.inv(item.matrix()) @ result
                current = item.source_frame
            if current == target_frame: return result
        raise TransformPoseError("frame_chain_unavailable")


def _pose_matrix(pose: Mapping[str, Any]) -> tuple[np.ndarray, str]:
    frame = pose.get("frame")
    if not isinstance(frame, str) or not frame: raise TransformPoseError("source_frame_required")
    xyz = _finite(pose.get("xyz_m", ()), "pose")
    if xyz.size != 3: raise TransformPoseError("pose_shape")
    if pose.get("rotvec_rad") is not None:
        rotvec = _finite(pose["rotvec_rad"], "rotvec")
        if rotvec.size != 3: raise TransformPoseError("rotation_shape")
        rotation = Rotation.from_rotvec(rotvec)
    elif pose.get("quaternion_xyzw") is not None:
        quat = _finite(pose["quaternion_xyzw"], "quaternion")
        if quat.size != 4 or np.linalg.norm(quat) == 0: raise TransformPoseError("quaternion_invalid")
        rotation = Rotation.from_quat(quat / np.linalg.norm(quat))
    elif pose.get("rpy_rad") is not None:
        rpy = _finite(pose["rpy_rad"], "rpy")
        if rpy.size != 3: raise TransformPoseError("rpy_shape")
        rotation = Rotation.from_euler("xyz", rpy)
    else:
        rotation = Rotation.identity()
    matrix = np.eye(4)
    matrix[:3, :3] = rotation.as_matrix()
    matrix[:3, 3] = xyz
    return matrix, frame


def transform_pose(*, pose: Mapping[str, Any], target_frame: str, chain: TransformChain, expected_calibration_hash: str) -> dict[str, Any]:
    if not isinstance(pose, Mapping) or not isinstance(target_frame, str) or not target_frame: raise TransformPoseError("pose_and_target_frame_required")
    matrix, source_frame = _pose_matrix(pose)
    transform = chain.matrix(source_frame, target_frame, expected_calibration_hash=expected_calibration_hash)
    output = transform @ matrix
    rotation = Rotation.from_matrix(output[:3, :3])
    return {"frame": target_frame, "source_frame": source_frame, "xyz_m": output[:3, 3].tolist(), "rotvec_rad": rotation.as_rotvec().tolist(), "calibration_hash": expected_calibration_hash, "deterministic": True}
