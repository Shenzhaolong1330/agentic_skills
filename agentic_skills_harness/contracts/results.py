from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

from .enums import ActionStatus, ErrorSeverity
from .errors import ErrorInfo
from .serialization import (
    ContractValidationError,
    ensure_jsonable,
    ensure_utc_timestamp,
    reject_unknown,
    require_bool,
    require_list,
    require_number,
    require_object,
    require_string,
    stable_dumps,
    to_plain,
    utc_now_iso,
)


def _error_list(value: Any, name: str) -> tuple[ErrorInfo, ...]:
    result: list[ErrorInfo] = []
    for item in require_list(value, name):
        if not isinstance(item, dict):
            raise ContractValidationError(f"{name} items must be ErrorInfo objects")
        result.append(ErrorInfo.from_dict(item))
    return tuple(result)


def _json_object(value: Any, name: str) -> dict[str, Any]:
    return ensure_jsonable(require_object(value, name), name)


@dataclass(frozen=True)
class VerificationResult:
    verified: bool
    predicate_id: str
    method: str
    evidence: tuple[Any, ...]
    confidence: float | None
    checked_at: str
    errors: tuple[ErrorInfo, ...] = ()
    warnings: tuple[ErrorInfo, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "verified", require_bool(self.verified, "verified"))
        object.__setattr__(self, "predicate_id", require_string(self.predicate_id, "predicate_id", non_empty=True))
        object.__setattr__(self, "method", require_string(self.method, "method"))
        evidence = tuple(self.evidence) if isinstance(self.evidence, (list, tuple)) else self.evidence
        if not isinstance(evidence, tuple):
            raise ContractValidationError("evidence must be an array")
        object.__setattr__(self, "evidence", tuple(ensure_jsonable(item, "evidence") for item in evidence))
        if self.confidence is not None:
            object.__setattr__(self, "confidence", require_number(self.confidence, "confidence", minimum=0.0))
            if self.confidence > 1.0:
                raise ContractValidationError("confidence must be at most 1.0")
        object.__setattr__(self, "checked_at", ensure_utc_timestamp(self.checked_at, "checked_at"))
        object.__setattr__(self, "errors", tuple(self.errors))
        object.__setattr__(self, "warnings", tuple(self.warnings))
        if any(not isinstance(item, ErrorInfo) for item in (*self.errors, *self.warnings)):
            raise ContractValidationError("errors and warnings must contain ErrorInfo values")
        if self.verified and (not self.method.strip() or not self.evidence):
            raise ContractValidationError("verified results require method and evidence")
        if self.verified and self.errors:
            raise ContractValidationError("a result with errors cannot be verified")

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "VerificationResult":
        names = ("verified", "predicate_id", "method", "evidence", "confidence", "checked_at", "errors", "warnings")
        reject_unknown(data, names, names)
        return cls(data["verified"], data["predicate_id"], data["method"], tuple(require_list(data["evidence"], "evidence")), data["confidence"], data["checked_at"], _error_list(data["errors"], "errors"), _error_list(data["warnings"], "warnings"))

    def to_dict(self) -> dict[str, Any]:
        return to_plain(self)

    def to_json(self) -> str:
        return stable_dumps(self.to_dict())


@dataclass(frozen=True)
class ActionResult:
    capability_id: str
    status: ActionStatus
    command_accepted: bool
    command_executed: bool
    planned_only: bool
    controller_target_reached: bool | None
    effect_observed: bool | None
    goal_verified: bool | None
    verification: VerificationResult | None
    outputs: dict[str, Any]
    errors: tuple[ErrorInfo, ...]
    warnings: tuple[ErrorInfo, ...]
    metrics: dict[str, Any]
    artifacts: dict[str, Any]
    started_at: str
    ended_at: str | None

    def __post_init__(self) -> None:
        object.__setattr__(self, "capability_id", require_string(self.capability_id, "capability_id", non_empty=True))
        if not isinstance(self.status, ActionStatus):
            object.__setattr__(self, "status", ActionStatus(self.status))
        for name in ("command_accepted", "command_executed", "planned_only"):
            object.__setattr__(self, name, require_bool(getattr(self, name), name))
        for name in ("controller_target_reached", "effect_observed", "goal_verified"):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, require_bool(value, name))
        object.__setattr__(self, "outputs", _json_object(self.outputs, "outputs"))
        object.__setattr__(self, "metrics", _json_object(self.metrics, "metrics"))
        object.__setattr__(self, "artifacts", _json_object(self.artifacts, "artifacts"))
        object.__setattr__(self, "errors", tuple(self.errors))
        object.__setattr__(self, "warnings", tuple(self.warnings))
        if any(not isinstance(item, ErrorInfo) for item in (*self.errors, *self.warnings)):
            raise ContractValidationError("errors and warnings must contain ErrorInfo values")
        object.__setattr__(self, "started_at", ensure_utc_timestamp(self.started_at, "started_at"))
        if self.ended_at is not None:
            object.__setattr__(self, "ended_at", ensure_utc_timestamp(self.ended_at, "ended_at"))
        if self.verification is not None and not isinstance(self.verification, VerificationResult):
            raise ContractValidationError("verification must be VerificationResult or null")
        if self.planned_only and self.command_executed:
            raise ContractValidationError("planned_only results cannot execute a command")
        if self.status == ActionStatus.PLANNED_ONLY and not self.planned_only:
            raise ContractValidationError("PLANNED_ONLY status requires planned_only=true")
        if self.status == ActionStatus.DENIED and self.command_executed:
            raise ContractValidationError("DENIED results cannot execute a command")
        if self.status == ActionStatus.TIMEOUT and self.effect_observed is True:
            raise ContractValidationError("TIMEOUT cannot assert effect_observed=true")
        if self.goal_verified is True and (self.verification is None or not self.verification.verified):
            raise ContractValidationError("goal_verified=true requires a verified VerificationResult")
        if self.status == ActionStatus.SUCCEEDED and any(error.severity in (ErrorSeverity.FATAL, ErrorSeverity.UNSAFE) for error in self.errors):
            raise ContractValidationError("fatal or unsafe errors cannot produce SUCCEEDED")

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ActionResult":
        names = ("capability_id", "status", "command_accepted", "command_executed", "planned_only", "controller_target_reached", "effect_observed", "goal_verified", "verification", "outputs", "errors", "warnings", "metrics", "artifacts", "started_at", "ended_at")
        reject_unknown(data, names, names)
        verification = None if data["verification"] is None else VerificationResult.from_dict(data["verification"])
        return cls(data["capability_id"], data["status"], data["command_accepted"], data["command_executed"], data["planned_only"], data["controller_target_reached"], data["effect_observed"], data["goal_verified"], verification, data["outputs"], _error_list(data["errors"], "errors"), _error_list(data["warnings"], "warnings"), data["metrics"], data["artifacts"], data["started_at"], data["ended_at"])

    @classmethod
    def from_command_result(cls, capability_id: str, returncode: int, *, command_accepted: bool = True, command_executed: bool = True, started_at: str | None = None, ended_at: str | None = None) -> "ActionResult":
        """Convert process status conservatively; no physical success is inferred."""
        status = ActionStatus.SUCCEEDED if returncode == 0 else ActionStatus.FAILED
        errors: tuple[ErrorInfo, ...] = () if returncode == 0 else (ErrorInfo(code="CAPABILITY_EXECUTION_FAILED", message="capability process returned a non-zero status", details={"returncode": returncode}, source="command"),)
        return cls(capability_id, status, command_accepted, command_executed, False, None, None, False, None, {}, errors, (), {"returncode": returncode}, {}, started_at or utc_now_iso(), ended_at)

    def to_dict(self) -> dict[str, Any]:
        return to_plain(self)

    def to_json(self) -> str:
        return stable_dumps(self.to_dict())


@dataclass(frozen=True)
class ObservationResult:
    capability_id: str
    ok: bool
    observations: dict[str, Any]
    captured_at: str
    freshness_ttl_s: float | None
    source: str
    errors: tuple[ErrorInfo, ...]
    warnings: tuple[ErrorInfo, ...]
    artifacts: dict[str, Any]

    def __post_init__(self) -> None:
        object.__setattr__(self, "capability_id", require_string(self.capability_id, "capability_id", non_empty=True))
        object.__setattr__(self, "ok", require_bool(self.ok, "ok"))
        object.__setattr__(self, "observations", _json_object(self.observations, "observations"))
        object.__setattr__(self, "captured_at", ensure_utc_timestamp(self.captured_at, "captured_at"))
        if self.freshness_ttl_s is not None:
            object.__setattr__(self, "freshness_ttl_s", require_number(self.freshness_ttl_s, "freshness_ttl_s", positive=True))
        object.__setattr__(self, "source", require_string(self.source, "source", non_empty=True))
        object.__setattr__(self, "errors", tuple(self.errors))
        object.__setattr__(self, "warnings", tuple(self.warnings))
        object.__setattr__(self, "artifacts", _json_object(self.artifacts, "artifacts"))

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ObservationResult":
        names = ("capability_id", "ok", "observations", "captured_at", "freshness_ttl_s", "source", "errors", "warnings", "artifacts")
        reject_unknown(data, names, names)
        return cls(data["capability_id"], data["ok"], data["observations"], data["captured_at"], data["freshness_ttl_s"], data["source"], _error_list(data["errors"], "errors"), _error_list(data["warnings"], "warnings"), data["artifacts"])

    def to_dict(self) -> dict[str, Any]:
        return to_plain(self)

    def to_json(self) -> str:
        return stable_dumps(self.to_dict())
