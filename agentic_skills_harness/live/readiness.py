from __future__ import annotations

from dataclasses import dataclass, replace
from datetime import datetime, timezone
from enum import Enum
from typing import Any, Iterable, Mapping

from .fingerprint import digest_json


class ReadinessState(str, Enum):
    DISABLED = "DISABLED"
    CONTRACT_ONLY = "CONTRACT_ONLY"
    IMPLEMENTATION_READY = "IMPLEMENTATION_READY"
    HARDWARE_ACCEPTANCE_PENDING = "HARDWARE_ACCEPTANCE_PENDING"
    VALIDATED_READ_ONLY = "VALIDATED_READ_ONLY"
    VALIDATED_ACTION = "VALIDATED_ACTION"
    VALIDATED_RECOVERY = "VALIDATED_RECOVERY"
    REJECTED = "REJECTED"


class VerificationMaturity(str, Enum):
    NONE = "NONE"
    OUTPUT_ONLY = "OUTPUT_ONLY"
    LIMITED = "LIMITED"
    INDEPENDENT_EVIDENCE = "INDEPENDENT_EVIDENCE"
    PHYSICALLY_VALIDATED = "PHYSICALLY_VALIDATED"


class ReadinessError(ValueError):
    pass


_LEVEL_RANK = {"H0_CONFIG": 0, "H1_READ_ONLY": 1, "H2_GRIPPER_EMPTY": 2, "H3_MOTION_P2P": 3, "H4_MOTION_RELATIVE": 4, "H5_STOP": 5, "H6_GRASP_VERIFICATION": 6, "H7_RECOVERY": 7, "H8_RESET_HOME": 8}
_STATE_LEVEL = {ReadinessState.VALIDATED_READ_ONLY: 1, ReadinessState.VALIDATED_ACTION: 6, ReadinessState.VALIDATED_RECOVERY: 7}


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass(frozen=True)
class CapabilityReadiness:
    capability_id: str
    capability_version: str
    adapter_id: str
    adapter_digest: str
    input_schema_digest: str
    output_schema_digest: str
    risk_class: str
    readiness_state: ReadinessState = ReadinessState.CONTRACT_ONLY
    required_acceptance_level: str = "H0_CONFIG"
    accepted_hardware_fingerprints: tuple[str, ...] = ()
    accepted_calibration_hashes: tuple[str, ...] = ()
    limitations: tuple[str, ...] = ()
    updated_at: str = ""
    verification_maturity: VerificationMaturity = VerificationMaturity.NONE
    workspace_digest: str = ""

    def __post_init__(self) -> None:
        for name in ("capability_id", "capability_version", "adapter_id", "adapter_digest", "input_schema_digest", "output_schema_digest", "risk_class", "required_acceptance_level"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ReadinessError(f"{name} must be non-empty")
        if self.workspace_digest and not isinstance(self.workspace_digest, str):
            raise ReadinessError("workspace_digest must be a string")
        if self.required_acceptance_level not in _LEVEL_RANK:
            raise ReadinessError(f"unknown acceptance level: {self.required_acceptance_level}")
        state = self.readiness_state if isinstance(self.readiness_state, ReadinessState) else ReadinessState(self.readiness_state)
        object.__setattr__(self, "readiness_state", state)
        maturity = self.verification_maturity if isinstance(self.verification_maturity, VerificationMaturity) else VerificationMaturity(self.verification_maturity)
        object.__setattr__(self, "verification_maturity", maturity)
        object.__setattr__(self, "accepted_hardware_fingerprints", tuple(self.accepted_hardware_fingerprints))
        object.__setattr__(self, "accepted_calibration_hashes", tuple(self.accepted_calibration_hashes))
        object.__setattr__(self, "limitations", tuple(str(item) for item in self.limitations))
        object.__setattr__(self, "updated_at", self.updated_at or _now())
        if state in _STATE_LEVEL and self.verification_maturity != VerificationMaturity.PHYSICALLY_VALIDATED:
            raise ReadinessError("validated readiness requires PHYSICALLY_VALIDATED maturity")
        if state in _STATE_LEVEL and not self.accepted_hardware_fingerprints:
            raise ReadinessError("validated readiness requires hardware evidence")
        if state in _STATE_LEVEL and not self.workspace_digest:
            raise ReadinessError("validated readiness requires workspace evidence")

    @property
    def live_validated(self) -> bool:
        return self.readiness_state in _STATE_LEVEL

    def to_dict(self) -> dict[str, Any]:
        return {"capability_id": self.capability_id, "capability_version": self.capability_version, "adapter_id": self.adapter_id, "adapter_digest": self.adapter_digest, "input_schema_digest": self.input_schema_digest, "output_schema_digest": self.output_schema_digest, "risk_class": self.risk_class, "readiness_state": self.readiness_state.value, "required_acceptance_level": self.required_acceptance_level, "accepted_hardware_fingerprints": list(self.accepted_hardware_fingerprints), "accepted_calibration_hashes": list(self.accepted_calibration_hashes), "limitations": list(self.limitations), "updated_at": self.updated_at, "verification_maturity": self.verification_maturity.value, "workspace_digest": self.workspace_digest}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "CapabilityReadiness":
        allowed = {"capability_id", "capability_version", "adapter_id", "adapter_digest", "input_schema_digest", "output_schema_digest", "risk_class", "readiness_state", "required_acceptance_level", "accepted_hardware_fingerprints", "accepted_calibration_hashes", "limitations", "updated_at", "verification_maturity"}
        unknown = set(data) - allowed
        if unknown:
            raise ReadinessError(f"unknown readiness fields: {sorted(unknown)}")
        return cls(**dict(data))

    def can_accept(self, *, acceptance_level: str, hardware_fingerprint_digest: str, calibration_hash: str, capability_version: str, adapter_digest: str, input_schema_digest: str, output_schema_digest: str, workspace_digest: str | None = None) -> tuple[bool, str]:
        if capability_version != self.capability_version:
            return False, "capability_version_mismatch"
        if adapter_digest != self.adapter_digest:
            return False, "adapter_digest_mismatch"
        if input_schema_digest != self.input_schema_digest or output_schema_digest != self.output_schema_digest:
            return False, "schema_digest_mismatch"
        if _LEVEL_RANK.get(acceptance_level, -1) < _LEVEL_RANK[self.required_acceptance_level]:
            return False, "acceptance_level_insufficient"
        if self.readiness_state in {ReadinessState.DISABLED, ReadinessState.CONTRACT_ONLY, ReadinessState.REJECTED}:
            return False, f"readiness_{self.readiness_state.value.lower()}"
        if self.readiness_state not in _STATE_LEVEL:
            return False, "hardware_acceptance_pending"
        if hardware_fingerprint_digest not in self.accepted_hardware_fingerprints:
            return False, "hardware_fingerprint_mismatch"
        if calibration_hash not in self.accepted_calibration_hashes:
            return False, "calibration_mismatch"
        if self.workspace_digest and workspace_digest != self.workspace_digest:
            return False, "workspace_mismatch"
        return True, "accepted"


class ReadinessRegistry:
    def __init__(self, values: Iterable[CapabilityReadiness] = ()) -> None:
        self._values: dict[str, CapabilityReadiness] = {}
        for value in values:
            self.put(value)

    def put(self, value: CapabilityReadiness) -> None:
        if not isinstance(value, CapabilityReadiness):
            raise TypeError("readiness registry accepts CapabilityReadiness")
        self._values[value.capability_id] = value

    def get(self, capability_id: str) -> CapabilityReadiness | None:
        return self._values.get(capability_id)

    def require(self, capability_id: str) -> CapabilityReadiness:
        value = self.get(capability_id)
        if value is None:
            raise ReadinessError(f"readiness not found: {capability_id}")
        return value

    def list(self) -> tuple[CapabilityReadiness, ...]:
        return tuple(self._values[key] for key in sorted(self._values))

    def can_live(self, capability_id: str, **kwargs: Any) -> tuple[bool, str]:
        return self.require(capability_id).can_accept(**kwargs)

    def to_dict(self) -> dict[str, Any]:
        return {item.capability_id: item.to_dict() for item in self.list()}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "ReadinessRegistry":
        return cls(CapabilityReadiness.from_dict(value) for value in data.values())

    @classmethod
    def from_capability_registry(cls, capability_registry: Any) -> "ReadinessRegistry":
        """Derive conservative, unvalidated readiness from the manifest."""
        values = []
        for capability in capability_registry.list(include_internal=True, include_legacy=True):
            input_digest = digest_json(capability_registry.resolve_input_schema(capability.capability_id))
            output_digest = digest_json(capability_registry.resolve_output_schema(capability.capability_id))
            adapter_digest = digest_json({"adapter_id": capability.adapter_id or "unsupported", "path": capability.path, "capability_version": capability.capability_version})
            state = ReadinessState(capability.live_readiness)
            values.append(CapabilityReadiness(capability.capability_id, capability.capability_version, capability.adapter_id or "unsupported", adapter_digest, input_digest, output_digest, capability.risk_class.value, state, capability.required_acceptance_level, (), (), (capability.notes,), verification_maturity=VerificationMaturity(capability.verification_maturity)))
        return cls(values)

    def digest(self) -> str:
        return digest_json(self.to_dict())

    def transition(self, capability_id: str, new_state: ReadinessState | str, *, evidence: Mapping[str, Any] | None = None) -> CapabilityReadiness:
        current = self.require(capability_id)
        target = new_state if isinstance(new_state, ReadinessState) else ReadinessState(new_state)
        allowed = {
            ReadinessState.DISABLED: {ReadinessState.CONTRACT_ONLY},
            ReadinessState.CONTRACT_ONLY: {ReadinessState.IMPLEMENTATION_READY, ReadinessState.DISABLED},
            ReadinessState.IMPLEMENTATION_READY: {ReadinessState.HARDWARE_ACCEPTANCE_PENDING, ReadinessState.CONTRACT_ONLY, ReadinessState.DISABLED},
            ReadinessState.HARDWARE_ACCEPTANCE_PENDING: {ReadinessState.VALIDATED_READ_ONLY, ReadinessState.VALIDATED_ACTION, ReadinessState.VALIDATED_RECOVERY, ReadinessState.REJECTED},
            ReadinessState.VALIDATED_READ_ONLY: {ReadinessState.REJECTED},
            ReadinessState.VALIDATED_ACTION: {ReadinessState.REJECTED},
            ReadinessState.VALIDATED_RECOVERY: {ReadinessState.REJECTED},
            ReadinessState.REJECTED: {ReadinessState.CONTRACT_ONLY},
        }
        if target not in allowed[current.readiness_state]:
            raise ReadinessError(f"illegal readiness transition: {current.readiness_state.value}->{target.value}")
        if target in _STATE_LEVEL:
            if not evidence or evidence.get("real_hardware") is not True or evidence.get("passed") is not True:
                raise ReadinessError("validated transition requires passed real hardware evidence")
            if evidence.get("capability_id") != current.capability_id or evidence.get("capability_version") != current.capability_version or evidence.get("adapter_digest") != current.adapter_digest:
                raise ReadinessError("acceptance identity mismatch")
            if evidence.get("input_schema_digest") != current.input_schema_digest or evidence.get("output_schema_digest") != current.output_schema_digest:
                raise ReadinessError("acceptance schema mismatch")
            updated = replace(current, readiness_state=target, verification_maturity=VerificationMaturity.PHYSICALLY_VALIDATED, accepted_hardware_fingerprints=(str(evidence["hardware_fingerprint_digest"]),), accepted_calibration_hashes=(str(evidence["calibration_hash"]),), workspace_digest=str(evidence.get("workspace_digest", current.workspace_digest)))
        else:
            updated = replace(current, readiness_state=target)
        self.put(updated)
        return updated
