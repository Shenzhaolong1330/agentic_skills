from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from typing import Any, Mapping


def _digest(value: Any) -> str:
    payload = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


@dataclass(frozen=True)
class HardwareFingerprint:
    """A non-secret identity for the hardware/configuration under test."""

    robot_type: str
    controller_type: str
    rpc_protocol_version: str
    arm_ids_hash: str
    camera_serial_hash: str | None
    gripper_type: str
    workspace_digest: str
    calibration_hash: str
    reset_config_digest: str

    def __post_init__(self) -> None:
        for name in ("robot_type", "controller_type", "rpc_protocol_version", "arm_ids_hash", "gripper_type", "workspace_digest", "calibration_hash", "reset_config_digest"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} must be a non-empty string")
        if self.camera_serial_hash is not None and (not isinstance(self.camera_serial_hash, str) or not self.camera_serial_hash.strip()):
            raise ValueError("camera_serial_hash must be null or non-empty")

    @classmethod
    def from_config(cls, data: Mapping[str, Any]) -> "HardwareFingerprint":
        allowed = {"robot_type", "controller_type", "rpc_protocol_version", "arm_ids", "arm_ids_hash", "camera_serial", "camera_serial_hash", "gripper_type", "workspace_digest", "calibration_hash", "reset_config_digest"}
        unknown = set(data) - allowed
        if unknown:
            raise ValueError(f"unknown hardware fingerprint fields: {sorted(unknown)}")
        arms = data.get("arm_ids")
        arm_hash = data.get("arm_ids_hash") or (_digest(sorted(arms)) if isinstance(arms, list) else None)
        camera_hash = data.get("camera_serial_hash")
        if camera_hash is None and data.get("camera_serial") is not None:
            camera_hash = _digest(str(data["camera_serial"]))
        if arms is not None and not isinstance(arms, list):
            raise ValueError("arm_ids must be an array when supplied")
        if arm_hash is None:
            raise ValueError("arm_ids_hash or non-empty arm_ids is required")
        return cls(data["robot_type"], data["controller_type"], data["rpc_protocol_version"], arm_hash, camera_hash, data["gripper_type"], data["workspace_digest"], data["calibration_hash"], data["reset_config_digest"])

    def to_dict(self) -> dict[str, Any]:
        return {"robot_type": self.robot_type, "controller_type": self.controller_type, "rpc_protocol_version": self.rpc_protocol_version, "arm_ids_hash": self.arm_ids_hash, "camera_serial_hash": self.camera_serial_hash, "gripper_type": self.gripper_type, "workspace_digest": self.workspace_digest, "calibration_hash": self.calibration_hash, "reset_config_digest": self.reset_config_digest}

    @property
    def digest(self) -> str:
        return _digest(self.to_dict())


def digest_json(value: Any) -> str:
    return _digest(value)
