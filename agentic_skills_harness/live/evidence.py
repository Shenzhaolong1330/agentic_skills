from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Mapping

from .fingerprint import HardwareFingerprint, digest_json


class EvidenceMismatch(ValueError):
    pass


@dataclass(frozen=True)
class AcceptanceEvidence:
    capability_id: str
    capability_version: str
    adapter_digest: str
    input_schema_digest: str
    output_schema_digest: str
    hardware_fingerprint_digest: str
    workspace_digest: str
    calibration_hash: str
    acceptance_level: str
    passed: bool
    automatic_checks: Mapping[str, Any]
    operator_checks: Mapping[str, Any]
    started_at: str
    ended_at: str
    expires_at: str | None = None
    revoked: bool = False
    real_hardware: bool = False

    def __post_init__(self) -> None:
        if not self.capability_id or not self.capability_version or not self.adapter_digest or not self.input_schema_digest or not self.output_schema_digest or not self.hardware_fingerprint_digest or not self.workspace_digest or not self.calibration_hash:
            raise EvidenceMismatch("acceptance evidence identity fields are required")
        if self.acceptance_level not in {"H0_CONFIG", "H1_READ_ONLY", "H2_GRIPPER_EMPTY", "H3_MOTION_P2P", "H4_MOTION_RELATIVE", "H5_STOP", "H6_GRASP_VERIFICATION", "H7_RECOVERY", "H8_RESET_HOME"}:
            raise EvidenceMismatch("invalid acceptance level")

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "AcceptanceEvidence":
        allowed = {"capability_id", "capability_version", "adapter_digest", "input_schema_digest", "output_schema_digest", "hardware_fingerprint_digest", "workspace_digest", "calibration_hash", "acceptance_level", "passed", "automatic_checks", "operator_checks", "started_at", "ended_at", "expires_at", "revoked", "real_hardware"}
        unknown = set(data) - allowed
        if unknown:
            raise EvidenceMismatch(f"unknown acceptance evidence fields: {sorted(unknown)}")
        return cls(**dict(data))

    def to_dict(self) -> dict[str, Any]:
        return dict(self.__dict__)

    @property
    def digest(self) -> str:
        return digest_json(self.to_dict())

    def usable(self, *, now: str | None = None) -> bool:
        if not self.passed or self.revoked:
            return False
        if not self.expires_at:
            return True
        current = datetime.fromisoformat((now or datetime.now(timezone.utc).isoformat()).replace("Z", "+00:00"))
        return current < datetime.fromisoformat(self.expires_at.replace("Z", "+00:00"))


def hardware_fingerprint_from_config(data: Mapping[str, Any]) -> HardwareFingerprint:
    return HardwareFingerprint.from_config(data)
