from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping

from ..contracts.serialization import ContractValidationError, ensure_jsonable, ensure_utc_timestamp, reject_unknown, require_bool, require_number, require_object, require_string, stable_dumps, to_plain
from .clock import SystemClock, iso


_FORBIDDEN_KEYS = {"code", "script", "expression", "callback", "eval", "exec", "__import__"}


def _reject_code_fields(value: Any, path: str = "value") -> None:
    if isinstance(value, dict):
        for key, item in value.items():
            if isinstance(key, str) and key.lower() in _FORBIDDEN_KEYS:
                raise ContractValidationError(f"{path}.{key} is not permitted in world data")
            _reject_code_fields(item, f"{path}.{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            _reject_code_fields(item, f"{path}[{index}]")


class FactStatus(str, Enum):
    OBSERVED = "OBSERVED"
    TENTATIVE = "TENTATIVE"
    VERIFIED = "VERIFIED"
    INVALIDATED = "INVALIDATED"


@dataclass(frozen=True)
class EntityRef:
    entity_id: str
    entity_type: str
    attributes: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "entity_id", require_string(self.entity_id, "entity_id", non_empty=True))
        object.__setattr__(self, "entity_type", require_string(self.entity_type, "entity_type", non_empty=True))
        attributes = ensure_jsonable(require_object(dict(self.attributes), "attributes"), "attributes")
        _reject_code_fields(attributes, "attributes")
        object.__setattr__(self, "attributes", attributes)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "EntityRef":
        reject_unknown(data, ("entity_id", "entity_type", "attributes"), ("entity_id", "entity_type"))
        return cls(data["entity_id"], data["entity_type"], data.get("attributes", {}))

    def to_dict(self) -> dict[str, Any]:
        return {"entity_id": self.entity_id, "entity_type": self.entity_type, "attributes": dict(self.attributes)}


@dataclass(frozen=True)
class WorldFact:
    fact_id: str
    subject: EntityRef
    predicate: str
    object: Any
    value: Any
    status: FactStatus
    confidence: float | None
    frame: str | None
    observed_at: str
    valid_until: str | None
    source_capability_id: str | None
    source_capability_version: str | None
    artifact_refs: tuple[str, ...]
    calibration_hash: str | None
    revision: int
    metadata: Mapping[str, Any] = field(default_factory=dict)
    freshness_ttl_s: float | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "fact_id", require_string(self.fact_id, "fact_id", non_empty=True))
        if not isinstance(self.subject, EntityRef):
            object.__setattr__(self, "subject", EntityRef.from_dict(self.subject))
        object.__setattr__(self, "predicate", require_string(self.predicate, "predicate", non_empty=True))
        object_value = ensure_jsonable(self.object, "object")
        fact_value = ensure_jsonable(self.value, "value")
        _reject_code_fields(object_value, "object")
        _reject_code_fields(fact_value, "value")
        object.__setattr__(self, "object", object_value)
        object.__setattr__(self, "value", fact_value)
        status = self.status if isinstance(self.status, FactStatus) else FactStatus(self.status)
        object.__setattr__(self, "status", status)
        if self.confidence is not None:
            confidence = require_number(self.confidence, "confidence", minimum=0.0)
            if confidence > 1.0:
                raise ContractValidationError("confidence must be at most 1.0")
            object.__setattr__(self, "confidence", confidence)
        if self.frame is not None:
            object.__setattr__(self, "frame", require_string(self.frame, "frame", non_empty=True))
        object.__setattr__(self, "observed_at", ensure_utc_timestamp(self.observed_at, "observed_at"))
        if self.valid_until is not None:
            object.__setattr__(self, "valid_until", ensure_utc_timestamp(self.valid_until, "valid_until"))
            if self._as_datetime(self.valid_until) < self._as_datetime(self.observed_at):
                raise ContractValidationError("valid_until cannot be earlier than observed_at")
        for name in ("source_capability_id", "source_capability_version", "calibration_hash"):
            if getattr(self, name) is not None:
                object.__setattr__(self, name, require_string(getattr(self, name), name, non_empty=True))
        refs = tuple(self.artifact_refs)
        if any(not isinstance(item, str) or not item.strip() for item in refs):
            raise ContractValidationError("artifact_refs must contain non-empty strings")
        object.__setattr__(self, "artifact_refs", refs)
        if isinstance(self.revision, bool) or not isinstance(self.revision, int) or self.revision < 0:
            raise ContractValidationError("revision must be a non-negative integer")
        metadata = ensure_jsonable(require_object(dict(self.metadata), "metadata"), "metadata")
        _reject_code_fields(metadata, "metadata")
        object.__setattr__(self, "metadata", metadata)
        if self.freshness_ttl_s is not None:
            object.__setattr__(self, "freshness_ttl_s", require_number(self.freshness_ttl_s, "freshness_ttl_s", positive=True))

    @staticmethod
    def _as_datetime(value: str):
        from datetime import datetime
        return datetime.fromisoformat(value[:-1] + "+00:00" if value.endswith("Z") else value)

    def is_fresh(self, now: str | None = None) -> bool:
        if self.status == FactStatus.INVALIDATED:
            return False
        current = self._as_datetime(now or iso(SystemClock().now()))
        if self.valid_until is not None:
            return current <= self._as_datetime(self.valid_until)
        if self.freshness_ttl_s is not None:
            return (current - self._as_datetime(self.observed_at)).total_seconds() <= self.freshness_ttl_s
        return True

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "WorldFact":
        names = ("fact_id", "subject", "predicate", "object", "value", "status", "confidence", "frame", "observed_at", "valid_until", "source_capability_id", "source_capability_version", "artifact_refs", "calibration_hash", "revision", "metadata", "freshness_ttl_s")
        reject_unknown(data, names, names[:-2])
        return cls(data["fact_id"], EntityRef.from_dict(data["subject"]), data["predicate"], data["object"], data["value"], data["status"], data["confidence"], data["frame"], data["observed_at"], data["valid_until"], data["source_capability_id"], data["source_capability_version"], tuple(data["artifact_refs"]), data["calibration_hash"], data["revision"], data.get("metadata", {}), data.get("freshness_ttl_s"))

    def to_dict(self) -> dict[str, Any]:
        return {"fact_id": self.fact_id, "subject": self.subject.to_dict(), "predicate": self.predicate, "object": self.object, "value": self.value, "status": self.status.value, "confidence": self.confidence, "frame": self.frame, "observed_at": self.observed_at, "valid_until": self.valid_until, "source_capability_id": self.source_capability_id, "source_capability_version": self.source_capability_version, "artifact_refs": list(self.artifact_refs), "calibration_hash": self.calibration_hash, "revision": self.revision, "metadata": dict(self.metadata), "freshness_ttl_s": self.freshness_ttl_s}

    def to_json(self) -> str:
        return stable_dumps(self.to_dict())
