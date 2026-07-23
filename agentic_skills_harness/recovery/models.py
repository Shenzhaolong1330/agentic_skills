from __future__ import annotations

"""Serializable contracts used by the bounded recovery runtime.

The recovery layer deliberately contains data, policy, and decisions only.  It
does not contain adapters, backends, executable paths, or callbacks.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping

from ..contracts.enums import ErrorCategory, ErrorCode, ErrorSeverity, RiskClass
from ..contracts.errors import ErrorInfo
from ..contracts.errors import ERROR_CATALOG
from ..contracts.serialization import (
    ContractValidationError,
    ensure_jsonable,
    reject_unknown,
    require_bool,
    require_number,
    require_string,
    stable_dumps,
    utc_now_iso,
)


class RecoveryStrategyId(str, Enum):
    REOBSERVE = "REOBSERVE"
    RETRY_SAME_PARAMETERS = "RETRY_SAME_PARAMETERS"
    SAFE_RETREAT = "SAFE_RETREAT"
    ALTERNATE_CANDIDATE = "ALTERNATE_CANDIDATE"
    ALTERNATE_ARM = "ALTERNATE_ARM"
    RECOMPILE_REMAINDER = "RECOMPILE_REMAINDER"
    CONTROLLER_RECOVERY = "CONTROLLER_RECOVERY"
    RESET_HOME = "RESET_HOME"
    REQUEST_HUMAN = "REQUEST_HUMAN"
    ABORT = "ABORT"


class RecoveryDisposition(str, Enum):
    LOCAL_RETRY = "LOCAL_RETRY"
    RECOVERY_SUBGRAPH = "RECOVERY_SUBGRAPH"
    REPLAN_REMAINDER = "REPLAN_REMAINDER"
    NEEDS_HUMAN = "NEEDS_HUMAN"
    TERMINATE = "TERMINATE"


class HeldObjectEvidence(str, Enum):
    NONE_CONFIRMED = "NONE_CONFIRMED"
    HOLDING_CONFIRMED = "HOLDING_CONFIRMED"
    HOLDING_TENTATIVE = "HOLDING_TENTATIVE"
    UNKNOWN = "UNKNOWN"


class RecoveryResultStatus(str, Enum):
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    EXHAUSTED = "EXHAUSTED"
    NEEDS_HUMAN = "NEEDS_HUMAN"
    TERMINATED = "TERMINATED"


_FORBIDDEN_FIELDS = {
    "hardware_allowed",
    "execute",
    "executable",
    "argv",
    "env",
    "environment",
    "adapter",
    "adapter_id",
    "backend",
    "shell",
    "script",
    "subprocess",
    "expression",
    "code",
    "callback",
    "function",
}


def _reject_runtime_fields(value: Any, path: str = "$") -> None:
    if isinstance(value, Mapping):
        for key, item in value.items():
            if str(key).lower() in _FORBIDDEN_FIELDS:
                raise ContractValidationError(f"forbidden recovery runtime field: {path}.{key}")
            _reject_runtime_fields(item, f"{path}.{key}")
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            _reject_runtime_fields(item, f"{path}[{index}]")


def _json(value: Any, name: str) -> dict[str, Any]:
    if hasattr(value, "to_dict"):
        value = value.to_dict()
    if not isinstance(value, Mapping):
        raise ContractValidationError(f"{name} must be an object")
    result = ensure_jsonable(dict(value), name)
    _reject_runtime_fields(result, name)
    return result


def _tuple_strings(value: Any, name: str) -> tuple[str, ...]:
    result = tuple(str(item) for item in (value or ()))
    if any(not item.strip() for item in result):
        raise ContractValidationError(f"{name} must contain non-empty strings")
    return result


def _error(value: ErrorInfo | Mapping[str, Any]) -> ErrorInfo:
    if isinstance(value, ErrorInfo):
        return value
    data = dict(value)
    if {"category", "severity", "retryable", "safe_to_replan", "requires_human", "state_invalidated", "details", "source"}.issubset(data):
        return ErrorInfo.from_dict(data)
    code = data.get("code", "UNKNOWN_ERROR")
    try:
        catalog = ERROR_CATALOG[code if hasattr(code, "value") else ErrorCode(str(code))]
    except (KeyError, ValueError):
        catalog = ERROR_CATALOG[ErrorCode.UNKNOWN_ERROR]
    return ErrorInfo(code=code, category=data.get("category", catalog["category"]), severity=data.get("severity", catalog["severity"]), message=str(data.get("message", code)), retryable=bool(data.get("retryable", catalog["retryable"])), safe_to_replan=bool(data.get("safe_to_replan", catalog["safe_to_replan"])), requires_human=bool(data.get("requires_human", catalog["requires_human"])), state_invalidated=bool(data.get("state_invalidated", False)), details=dict(data.get("details", {})), source=str(data.get("source", "recovery")))


@dataclass(frozen=True)
class RecoveryStrategy:
    strategy_id: RecoveryStrategyId | str
    disposition: RecoveryDisposition | str
    priority: int = 0
    trigger_error_codes: tuple[str, ...] = ()
    trigger_error_categories: tuple[str, ...] = ()
    allowed_node_kinds: tuple[str, ...] = ()
    allowed_capability_ids: tuple[str, ...] = ()
    required_capability_ids: tuple[str, ...] = ()
    forbidden_when: Mapping[str, Any] = field(default_factory=dict)
    requires_reobserve: bool = False
    requires_safe_retreat: bool = False
    invalidates: tuple[str, ...] = ()
    max_attempts: int = 1
    risk_class: RiskClass | str = RiskClass.NONE
    template_id: str | None = None
    notes: str = ""

    def __post_init__(self) -> None:
        sid = self.strategy_id if isinstance(self.strategy_id, RecoveryStrategyId) else RecoveryStrategyId(str(self.strategy_id))
        disposition = self.disposition if isinstance(self.disposition, RecoveryDisposition) else RecoveryDisposition(str(self.disposition))
        object.__setattr__(self, "strategy_id", sid)
        object.__setattr__(self, "disposition", disposition)
        if isinstance(self.priority, bool) or not isinstance(self.priority, int):
            raise ContractValidationError("strategy priority must be an integer")
        for name in ("trigger_error_codes", "trigger_error_categories", "allowed_node_kinds", "allowed_capability_ids", "required_capability_ids", "invalidates"):
            object.__setattr__(self, name, _tuple_strings(getattr(self, name), name))
        forbidden = _json(self.forbidden_when, "forbidden_when")
        object.__setattr__(self, "forbidden_when", forbidden)
        object.__setattr__(self, "requires_reobserve", require_bool(self.requires_reobserve, "requires_reobserve"))
        object.__setattr__(self, "requires_safe_retreat", require_bool(self.requires_safe_retreat, "requires_safe_retreat"))
        if isinstance(self.max_attempts, bool) or not isinstance(self.max_attempts, int) or self.max_attempts < 1 or self.max_attempts > 1024:
            raise ContractValidationError("strategy max_attempts must be from 1 to 1024")
        risk = self.risk_class if isinstance(self.risk_class, RiskClass) else RiskClass(str(self.risk_class))
        object.__setattr__(self, "risk_class", risk)
        if self.template_id is not None:
            object.__setattr__(self, "template_id", require_string(self.template_id, "template_id", non_empty=True))
        object.__setattr__(self, "notes", require_string(self.notes, "notes"))
        if disposition == RecoveryDisposition.LOCAL_RETRY and sid != RecoveryStrategyId.RETRY_SAME_PARAMETERS:
            raise ContractValidationError("LOCAL_RETRY is reserved for RETRY_SAME_PARAMETERS")
        if sid == RecoveryStrategyId.RETRY_SAME_PARAMETERS and disposition != RecoveryDisposition.LOCAL_RETRY:
            raise ContractValidationError("RETRY_SAME_PARAMETERS must use LOCAL_RETRY")

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "RecoveryStrategy":
        allowed = {
            "strategy_id", "disposition", "priority", "trigger_error_codes", "trigger_error_categories",
            "allowed_node_kinds", "allowed_capability_ids", "required_capability_ids", "forbidden_when",
            "requires_reobserve", "requires_safe_retreat", "invalidates", "max_attempts", "risk_class",
            "template_id", "notes",
        }
        reject_unknown(data, allowed, ("strategy_id", "disposition"))
        return cls(
            data["strategy_id"], data["disposition"], data.get("priority", 0),
            tuple(data.get("trigger_error_codes", ())), tuple(data.get("trigger_error_categories", ())),
            tuple(data.get("allowed_node_kinds", ())), tuple(data.get("allowed_capability_ids", ())),
            tuple(data.get("required_capability_ids", ())), data.get("forbidden_when", {}),
            data.get("requires_reobserve", False), data.get("requires_safe_retreat", False),
            tuple(data.get("invalidates", ())), data.get("max_attempts", 1), data.get("risk_class", RiskClass.NONE.value),
            data.get("template_id"), data.get("notes", ""),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "strategy_id": self.strategy_id.value,
            "disposition": self.disposition.value,
            "priority": self.priority,
            "trigger_error_codes": list(self.trigger_error_codes),
            "trigger_error_categories": list(self.trigger_error_categories),
            "allowed_node_kinds": list(self.allowed_node_kinds),
            "allowed_capability_ids": list(self.allowed_capability_ids),
            "required_capability_ids": list(self.required_capability_ids),
            "forbidden_when": dict(self.forbidden_when),
            "requires_reobserve": self.requires_reobserve,
            "requires_safe_retreat": self.requires_safe_retreat,
            "invalidates": list(self.invalidates),
            "max_attempts": self.max_attempts,
            "risk_class": self.risk_class.value,
            "template_id": self.template_id,
            "notes": self.notes,
        }


@dataclass(frozen=True)
class RecoveryContext:
    run_id: str
    graph_id: str
    goal_id: str
    plan_hash: str
    failed_node_id: str
    failed_node_kind: str
    failed_capability_id: str | None
    error: ErrorInfo | Mapping[str, Any]
    world_snapshot: Mapping[str, Any]
    held_object_evidence: HeldObjectEvidence | str = HeldObjectEvidence.UNKNOWN
    completed_node_ids: tuple[str, ...] = ()
    completed_valid_effects: tuple[Mapping[str, Any], ...] = ()
    invalidated_effects: tuple[Mapping[str, Any], ...] = ()
    remaining_budget: Mapping[str, Any] = field(default_factory=dict)
    original_envelope: Mapping[str, Any] = field(default_factory=dict)
    mode: str = "mock"
    latest_progress_fingerprint: str | None = None
    recovery_history: tuple[Mapping[str, Any], ...] = ()
    replan_history: tuple[Mapping[str, Any], ...] = ()

    def __post_init__(self) -> None:
        for name in ("run_id", "graph_id", "goal_id", "plan_hash", "failed_node_id", "failed_node_kind", "mode"):
            object.__setattr__(self, name, require_string(getattr(self, name), name, non_empty=True))
        if self.failed_capability_id is not None:
            object.__setattr__(self, "failed_capability_id", require_string(self.failed_capability_id, "failed_capability_id", non_empty=True))
        error = _error(self.error)
        object.__setattr__(self, "error", error)
        object.__setattr__(self, "world_snapshot", _json(self.world_snapshot, "world_snapshot"))
        held = self.held_object_evidence if isinstance(self.held_object_evidence, HeldObjectEvidence) else HeldObjectEvidence(str(self.held_object_evidence))
        object.__setattr__(self, "held_object_evidence", held)
        object.__setattr__(self, "completed_node_ids", _tuple_strings(self.completed_node_ids, "completed_node_ids"))
        object.__setattr__(self, "completed_valid_effects", tuple(_json(item, "completed_valid_effect") for item in self.completed_valid_effects))
        object.__setattr__(self, "invalidated_effects", tuple(_json(item, "invalidated_effect") for item in self.invalidated_effects))
        object.__setattr__(self, "remaining_budget", _json(self.remaining_budget, "remaining_budget"))
        object.__setattr__(self, "original_envelope", _json(self.original_envelope, "original_envelope"))
        object.__setattr__(self, "latest_progress_fingerprint", None if self.latest_progress_fingerprint is None else require_string(self.latest_progress_fingerprint, "latest_progress_fingerprint", non_empty=True))
        object.__setattr__(self, "recovery_history", tuple(_json(item, "recovery_history") for item in self.recovery_history))
        object.__setattr__(self, "replan_history", tuple(_json(item, "replan_history") for item in self.replan_history))

    @property
    def error_info(self) -> ErrorInfo:
        return _error(self.error)

    def to_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id, "graph_id": self.graph_id, "goal_id": self.goal_id, "plan_hash": self.plan_hash,
            "failed_node_id": self.failed_node_id, "failed_node_kind": self.failed_node_kind,
            "failed_capability_id": self.failed_capability_id, "error": self.error_info.to_dict(),
            "world_snapshot": dict(self.world_snapshot), "held_object_evidence": self.held_object_evidence.value,
            "completed_node_ids": list(self.completed_node_ids), "completed_valid_effects": [dict(item) for item in self.completed_valid_effects],
            "invalidated_effects": [dict(item) for item in self.invalidated_effects], "remaining_budget": dict(self.remaining_budget),
            "original_envelope": dict(self.original_envelope), "mode": self.mode,
            "latest_progress_fingerprint": self.latest_progress_fingerprint,
            "recovery_history": [dict(item) for item in self.recovery_history], "replan_history": [dict(item) for item in self.replan_history],
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "RecoveryContext":
        allowed = {"run_id", "graph_id", "goal_id", "plan_hash", "failed_node_id", "failed_node_kind", "failed_capability_id", "error", "world_snapshot", "held_object_evidence", "completed_node_ids", "completed_valid_effects", "invalidated_effects", "remaining_budget", "original_envelope", "mode", "latest_progress_fingerprint", "recovery_history", "replan_history"}
        reject_unknown(data, allowed, ("run_id", "graph_id", "goal_id", "plan_hash", "failed_node_id", "failed_node_kind", "error", "world_snapshot", "original_envelope"))
        return cls(data["run_id"], data["graph_id"], data["goal_id"], data["plan_hash"], data["failed_node_id"], data["failed_node_kind"], data.get("failed_capability_id"), data["error"], data["world_snapshot"], data.get("held_object_evidence", HeldObjectEvidence.UNKNOWN.value), tuple(data.get("completed_node_ids", ())), tuple(data.get("completed_valid_effects", ())), tuple(data.get("invalidated_effects", ())), data.get("remaining_budget", {}), data["original_envelope"], data.get("mode", "mock"), data.get("latest_progress_fingerprint"), tuple(data.get("recovery_history", ())), tuple(data.get("replan_history", ())))


@dataclass(frozen=True)
class RecoveryCandidate:
    strategy: RecoveryStrategy
    eligible: bool
    reasons: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return {"strategy": self.strategy.to_dict(), "eligible": self.eligible, "reasons": list(self.reasons)}


@dataclass(frozen=True)
class RecoveryDecision:
    decision_id: str
    selected_strategy: RecoveryStrategy | None
    eligible_candidates: tuple[RecoveryCandidate, ...]
    rejected_candidates: tuple[RecoveryCandidate, ...]
    reason: str
    remaining_budget: Mapping[str, Any]
    requires_reobserve: bool = False
    requires_human: bool = False
    terminal_if_failed: bool = True
    selected_at: str = field(default_factory=utc_now_iso)

    def __post_init__(self) -> None:
        object.__setattr__(self, "decision_id", require_string(self.decision_id, "decision_id", non_empty=True))
        object.__setattr__(self, "reason", require_string(self.reason, "reason", non_empty=True))
        object.__setattr__(self, "remaining_budget", _json(self.remaining_budget, "remaining_budget"))
        object.__setattr__(self, "requires_reobserve", require_bool(self.requires_reobserve, "requires_reobserve"))
        object.__setattr__(self, "requires_human", require_bool(self.requires_human, "requires_human"))
        object.__setattr__(self, "terminal_if_failed", require_bool(self.terminal_if_failed, "terminal_if_failed"))
        object.__setattr__(self, "eligible_candidates", tuple(self.eligible_candidates))
        object.__setattr__(self, "rejected_candidates", tuple(self.rejected_candidates))

    def to_dict(self) -> dict[str, Any]:
        return {"decision_id": self.decision_id, "selected_strategy": None if self.selected_strategy is None else self.selected_strategy.to_dict(), "eligible_candidates": [item.to_dict() for item in self.eligible_candidates], "rejected_candidates": [item.to_dict() for item in self.rejected_candidates], "reason": self.reason, "remaining_budget": dict(self.remaining_budget), "requires_reobserve": self.requires_reobserve, "requires_human": self.requires_human, "terminal_if_failed": self.terminal_if_failed, "selected_at": self.selected_at}


@dataclass(frozen=True)
class RecoveryAttempt:
    attempt_id: str
    decision_id: str
    strategy_id: RecoveryStrategyId | str
    status: str
    started_at: str = field(default_factory=utc_now_iso)
    ended_at: str | None = None
    error: ErrorInfo | Mapping[str, Any] | None = None
    result_ref: str | None = None
    progress_fingerprint: str | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "attempt_id", require_string(self.attempt_id, "attempt_id", non_empty=True))
        object.__setattr__(self, "decision_id", require_string(self.decision_id, "decision_id", non_empty=True))
        sid = self.strategy_id if isinstance(self.strategy_id, RecoveryStrategyId) else RecoveryStrategyId(str(self.strategy_id))
        object.__setattr__(self, "strategy_id", sid)
        object.__setattr__(self, "status", require_string(self.status, "status", non_empty=True))
        if self.error is not None and not isinstance(self.error, ErrorInfo):
            object.__setattr__(self, "error", ErrorInfo.from_dict(dict(self.error)))

    def to_dict(self) -> dict[str, Any]:
        return {"attempt_id": self.attempt_id, "decision_id": self.decision_id, "strategy_id": self.strategy_id.value, "status": self.status, "started_at": self.started_at, "ended_at": self.ended_at, "error": None if self.error is None else (self.error.to_dict() if isinstance(self.error, ErrorInfo) else dict(self.error)), "result_ref": self.result_ref, "progress_fingerprint": self.progress_fingerprint}


@dataclass(frozen=True)
class RecoveryResult:
    attempt_id: str
    status: RecoveryResultStatus | str
    disposition: RecoveryDisposition | str
    strategy_id: RecoveryStrategyId | str
    graph_result: Mapping[str, Any] | None = None
    error: ErrorInfo | Mapping[str, Any] | None = None
    requires_reobserve: bool = False
    resumed: bool = False
    progress_fingerprint: str | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "attempt_id", require_string(self.attempt_id, "attempt_id", non_empty=True))
        object.__setattr__(self, "status", self.status if isinstance(self.status, RecoveryResultStatus) else RecoveryResultStatus(str(self.status)))
        object.__setattr__(self, "disposition", self.disposition if isinstance(self.disposition, RecoveryDisposition) else RecoveryDisposition(str(self.disposition)))
        object.__setattr__(self, "strategy_id", self.strategy_id if isinstance(self.strategy_id, RecoveryStrategyId) else RecoveryStrategyId(str(self.strategy_id)))
        if self.graph_result is not None:
            object.__setattr__(self, "graph_result", _json(self.graph_result, "graph_result"))
        if self.error is not None and not isinstance(self.error, ErrorInfo):
            object.__setattr__(self, "error", ErrorInfo.from_dict(dict(self.error)))
        object.__setattr__(self, "requires_reobserve", require_bool(self.requires_reobserve, "requires_reobserve"))
        object.__setattr__(self, "resumed", require_bool(self.resumed, "resumed"))

    def to_dict(self) -> dict[str, Any]:
        return {"attempt_id": self.attempt_id, "status": self.status.value, "disposition": self.disposition.value, "strategy_id": self.strategy_id.value, "graph_result": None if self.graph_result is None else dict(self.graph_result), "error": None if self.error is None else (self.error.to_dict() if isinstance(self.error, ErrorInfo) else dict(self.error)), "requires_reobserve": self.requires_reobserve, "resumed": self.resumed, "progress_fingerprint": self.progress_fingerprint}


@dataclass(frozen=True)
class PlanLineageEntry:
    lineage_index: int
    plan_hash: str
    parent_plan_hash: str
    reason: str
    failure_node_id: str
    error_code: str
    recovery_strategy_id: str
    created_at: str = field(default_factory=utc_now_iso)
    remaining_budget_digest: str = ""
    world_state_digest: str = ""

    def __post_init__(self) -> None:
        if isinstance(self.lineage_index, bool) or not isinstance(self.lineage_index, int) or self.lineage_index < 0:
            raise ContractValidationError("lineage_index must be a non-negative integer")
        for name in ("plan_hash", "parent_plan_hash", "reason", "failure_node_id", "error_code", "recovery_strategy_id"):
            object.__setattr__(self, name, require_string(getattr(self, name), name, non_empty=True))

    def to_dict(self) -> dict[str, Any]:
        return {"lineage_index": self.lineage_index, "plan_hash": self.plan_hash, "parent_plan_hash": self.parent_plan_hash, "reason": self.reason, "failure_node_id": self.failure_node_id, "error_code": self.error_code, "recovery_strategy_id": self.recovery_strategy_id, "created_at": self.created_at, "remaining_budget_digest": self.remaining_budget_digest, "world_state_digest": self.world_state_digest}


@dataclass(frozen=True)
class PlanLineage:
    root_plan_hash: str
    entries: tuple[PlanLineageEntry | Mapping[str, Any], ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "root_plan_hash", require_string(self.root_plan_hash, "root_plan_hash", non_empty=True))
        values = tuple(item if isinstance(item, PlanLineageEntry) else PlanLineageEntry(**dict(item)) for item in self.entries)
        previous = self.root_plan_hash
        expected_index = 0
        for entry in values:
            if entry.lineage_index != expected_index:
                raise ContractValidationError("lineage indexes must be contiguous and monotonic")
            if entry.parent_plan_hash != previous:
                raise ContractValidationError("lineage entry parent does not match current plan")
            if entry.plan_hash == entry.parent_plan_hash:
                raise ContractValidationError("lineage replacement must change plan hash")
            previous = entry.plan_hash
            expected_index += 1
        object.__setattr__(self, "entries", values)

    @property
    def current_plan_hash(self) -> str:
        return self.entries[-1].plan_hash if self.entries else self.root_plan_hash

    def append(self, entry: PlanLineageEntry) -> "PlanLineage":
        return PlanLineage(self.root_plan_hash, (*self.entries, entry))

    def to_dict(self) -> dict[str, Any]:
        return {"root_plan_hash": self.root_plan_hash, "entries": [item.to_dict() for item in self.entries]}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "PlanLineage":
        reject_unknown(data, {"root_plan_hash", "entries"}, ("root_plan_hash",))
        return cls(data["root_plan_hash"], tuple(data.get("entries", ())))


@dataclass(frozen=True)
class ReplanRequest:
    request_id: str
    root_goal_spec: Mapping[str, Any]
    original_envelope: Mapping[str, Any]
    current_world_snapshot: Mapping[str, Any]
    root_plan_hash: str
    current_plan_hash: str
    completed_node_ids: tuple[str, ...]
    completed_valid_effects: tuple[Mapping[str, Any], ...]
    invalidated_effects: tuple[Mapping[str, Any], ...]
    failed_node_id: str
    failure_error: ErrorInfo | Mapping[str, Any]
    remaining_budget: Mapping[str, Any]
    recovery_history: tuple[Mapping[str, Any], ...] = ()
    plan_lineage: PlanLineage | Mapping[str, Any] = field(default_factory=lambda: PlanLineage("root"))
    reason: str = ""

    def __post_init__(self) -> None:
        for name in ("request_id", "root_plan_hash", "current_plan_hash", "failed_node_id", "reason"):
            object.__setattr__(self, name, require_string(getattr(self, name), name, non_empty=name != "reason"))
        for name in ("root_goal_spec", "original_envelope", "current_world_snapshot", "remaining_budget"):
            object.__setattr__(self, name, _json(getattr(self, name), name))
        object.__setattr__(self, "completed_node_ids", _tuple_strings(self.completed_node_ids, "completed_node_ids"))
        object.__setattr__(self, "completed_valid_effects", tuple(_json(item, "completed_valid_effect") for item in self.completed_valid_effects))
        object.__setattr__(self, "invalidated_effects", tuple(_json(item, "invalidated_effect") for item in self.invalidated_effects))
        object.__setattr__(self, "recovery_history", tuple(_json(item, "recovery_history") for item in self.recovery_history))
        error = _error(self.failure_error)
        object.__setattr__(self, "failure_error", error)
        lineage = self.plan_lineage if isinstance(self.plan_lineage, PlanLineage) else PlanLineage.from_dict(self.plan_lineage)
        object.__setattr__(self, "plan_lineage", lineage)

    def to_dict(self) -> dict[str, Any]:
        return {"request_id": self.request_id, "root_goal_spec": dict(self.root_goal_spec), "original_envelope": dict(self.original_envelope), "current_world_snapshot": dict(self.current_world_snapshot), "root_plan_hash": self.root_plan_hash, "current_plan_hash": self.current_plan_hash, "completed_node_ids": list(self.completed_node_ids), "completed_valid_effects": [dict(item) for item in self.completed_valid_effects], "invalidated_effects": [dict(item) for item in self.invalidated_effects], "failed_node_id": self.failed_node_id, "failure_error": self.failure_error.to_dict(), "remaining_budget": dict(self.remaining_budget), "recovery_history": [dict(item) for item in self.recovery_history], "plan_lineage": self.plan_lineage.to_dict(), "reason": self.reason}


@dataclass(frozen=True)
class ReplanResult:
    request_id: str
    status: str
    replacement_goal_spec: Mapping[str, Any] | None = None
    replacement_envelope: Mapping[str, Any] | None = None
    replacement_task_graph: Mapping[str, Any] | None = None
    compilation_report: Mapping[str, Any] | None = None
    compiled_graph: Any | None = None
    lineage_entry: PlanLineageEntry | Mapping[str, Any] | None = None
    errors: tuple[Mapping[str, Any] | str, ...] = ()
    warnings: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "request_id", require_string(self.request_id, "request_id", non_empty=True))
        object.__setattr__(self, "status", require_string(self.status, "status", non_empty=True))
        for name in ("replacement_goal_spec", "replacement_envelope", "replacement_task_graph", "compilation_report"):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, _json(value, name))
        if self.lineage_entry is not None and not isinstance(self.lineage_entry, PlanLineageEntry):
            object.__setattr__(self, "lineage_entry", PlanLineageEntry(**dict(self.lineage_entry)))
        object.__setattr__(self, "errors", tuple(item if isinstance(item, str) else _json(item, "error") for item in self.errors))
        object.__setattr__(self, "warnings", tuple(str(item) for item in self.warnings))

    def to_dict(self) -> dict[str, Any]:
        return {"request_id": self.request_id, "status": self.status, "replacement_goal_spec": self.replacement_goal_spec, "replacement_envelope": self.replacement_envelope, "replacement_task_graph": self.replacement_task_graph, "compilation_report": self.compilation_report, "compiled_graph": self.compiled_graph.to_dict() if hasattr(self.compiled_graph, "to_dict") else self.compiled_graph, "lineage_entry": None if self.lineage_entry is None else (self.lineage_entry.to_dict() if isinstance(self.lineage_entry, PlanLineageEntry) else dict(self.lineage_entry)), "errors": [item if isinstance(item, str) else dict(item) for item in self.errors], "warnings": list(self.warnings)}
