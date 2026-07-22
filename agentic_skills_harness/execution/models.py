from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping

from ..contracts.enums import ErrorCode, NodeStatus, TaskTerminalState
from ..contracts.errors import ErrorInfo
from ..contracts.serialization import ensure_jsonable, reject_unknown, stable_dumps, to_plain, utc_now_iso


class ExecutionOutcome(str, Enum):
    GOAL_VERIFIED = "GOAL_VERIFIED"
    GRAPH_COMPLETED_UNVERIFIED = "GRAPH_COMPLETED_UNVERIFIED"
    PLAN_COMPLETED = "PLAN_COMPLETED"
    FAILED = "FAILED"
    NEEDS_HUMAN = "NEEDS_HUMAN"
    UNSAFE = "UNSAFE"
    TIMEOUT = "TIMEOUT"
    BUDGET_EXCEEDED = "BUDGET_EXCEEDED"
    NO_PROGRESS = "NO_PROGRESS"
    CANCELLED = "CANCELLED"
    REPLAN_REQUIRED = "REPLAN_REQUIRED"


class ExecutionScope(str, Enum):
    NONE = "NONE"
    SIMULATED = "SIMULATED"
    ARTIFACT_REPLAY = "ARTIFACT_REPLAY"
    PHYSICAL = "PHYSICAL"


class NodeExecutionDisposition(str, Enum):
    PENDING = "PENDING"
    READY = "READY"
    RUNNING = "RUNNING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    SKIPPED = "SKIPPED"
    CANCELLED = "CANCELLED"


def _status(value: NodeStatus | str) -> NodeStatus:
    return value if isinstance(value, NodeStatus) else NodeStatus(str(value))


@dataclass
class NodeAttempt:
    attempt_id: str
    node_id: str
    attempt_index: int
    status: NodeStatus | str = NodeStatus.RUNNING
    resolved_arguments_digest: str = ""
    started_at: str = field(default_factory=utc_now_iso)
    ended_at: str | None = None
    dispatch_called: bool = False
    verification_called: bool = False
    result_ref: str | None = None
    error: ErrorInfo | dict[str, Any] | None = None

    def __post_init__(self) -> None:
        self.status = _status(self.status)
        if self.error is not None and not isinstance(self.error, ErrorInfo):
            self.error = ErrorInfo.from_dict(dict(self.error))

    def to_dict(self) -> dict[str, Any]:
        return {"attempt_id": self.attempt_id, "node_id": self.node_id, "attempt_index": self.attempt_index, "status": self.status.value, "resolved_arguments_digest": self.resolved_arguments_digest, "started_at": self.started_at, "ended_at": self.ended_at, "dispatch_called": self.dispatch_called, "verification_called": self.verification_called, "result_ref": self.result_ref, "error": None if self.error is None else (self.error.to_dict() if isinstance(self.error, ErrorInfo) else dict(self.error))}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "NodeAttempt":
        reject_unknown(data, {"attempt_id", "node_id", "attempt_index", "status", "resolved_arguments_digest", "started_at", "ended_at", "dispatch_called", "verification_called", "result_ref", "error"}, ("attempt_id", "node_id", "attempt_index", "status"))
        return cls(data["attempt_id"], data["node_id"], data["attempt_index"], data["status"], data.get("resolved_arguments_digest", ""), data.get("started_at", utc_now_iso()), data.get("ended_at"), data.get("dispatch_called", False), data.get("verification_called", False), data.get("result_ref"), data.get("error"))


@dataclass
class NodeRuntimeRecord:
    node_id: str
    kind: str
    status: NodeStatus | str = NodeStatus.PENDING
    visit_count: int = 0
    attempt_count: int = 0
    latest_attempt_id: str | None = None
    latest_error_code: str | None = None
    latest_dispatch_result_ref: str | None = None
    latest_verification_result_ref: str | None = None
    output_ref: str | None = None
    started_at: str | None = None
    ended_at: str | None = None
    resource_requirements: tuple[dict[str, Any], ...] = ()
    warnings: list[str] = field(default_factory=list)
    attempts: list[NodeAttempt] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.status = _status(self.status)
        self.resource_requirements = tuple(dict(item) for item in self.resource_requirements)

    def to_dict(self, *, include_attempts: bool = True) -> dict[str, Any]:
        value = {"node_id": self.node_id, "kind": self.kind, "status": self.status.value, "visit_count": self.visit_count, "attempt_count": self.attempt_count, "latest_attempt_id": self.latest_attempt_id, "latest_error_code": self.latest_error_code, "latest_dispatch_result_ref": self.latest_dispatch_result_ref, "latest_verification_result_ref": self.latest_verification_result_ref, "output_ref": self.output_ref, "started_at": self.started_at, "ended_at": self.ended_at, "resource_requirements": [dict(item) for item in self.resource_requirements], "warnings": list(self.warnings)}
        if include_attempts:
            value["attempts"] = [item.to_dict() for item in self.attempts]
        return value

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "NodeRuntimeRecord":
        allowed = {"node_id", "kind", "status", "visit_count", "attempt_count", "latest_attempt_id", "latest_error_code", "latest_dispatch_result_ref", "latest_verification_result_ref", "output_ref", "started_at", "ended_at", "resource_requirements", "warnings", "attempts"}
        reject_unknown(data, allowed, ("node_id", "kind", "status"))
        return cls(data["node_id"], data["kind"], data["status"], data.get("visit_count", 0), data.get("attempt_count", 0), data.get("latest_attempt_id"), data.get("latest_error_code"), data.get("latest_dispatch_result_ref"), data.get("latest_verification_result_ref"), data.get("output_ref"), data.get("started_at"), data.get("ended_at"), tuple(data.get("resource_requirements", ())), list(data.get("warnings", ())), [NodeAttempt.from_dict(item) for item in data.get("attempts", ())])


@dataclass
class BudgetUsage:
    nodes_started: int = 0
    unique_nodes_completed: int = 0
    tool_calls: int = 0
    elapsed_s: float = 0.0
    replans: int = 0
    recovery_actions: int = 0
    same_error_retries: int = 0
    node_visits: dict[str, int] = field(default_factory=dict)
    edge_traversals: dict[str, int] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {"nodes_started": self.nodes_started, "unique_nodes_completed": self.unique_nodes_completed, "tool_calls": self.tool_calls, "elapsed_s": self.elapsed_s, "replans": self.replans, "recovery_actions": self.recovery_actions, "same_error_retries": self.same_error_retries, "node_visits": dict(sorted(self.node_visits.items())), "edge_traversals": dict(sorted(self.edge_traversals.items()))}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "BudgetUsage":
        return cls(data.get("nodes_started", 0), data.get("unique_nodes_completed", 0), data.get("tool_calls", 0), float(data.get("elapsed_s", 0.0)), data.get("replans", 0), data.get("recovery_actions", 0), data.get("same_error_retries", 0), {str(k): int(v) for k, v in dict(data.get("node_visits", {})).items()}, {str(k): int(v) for k, v in dict(data.get("edge_traversals", {})).items()})


@dataclass
class TaskExecutionResult:
    run_id: str
    graph_id: str
    goal_id: str
    plan_hash: str
    mode: str
    terminal_state: str
    outcome: ExecutionOutcome | str
    execution_scope: ExecutionScope | str
    graph_completed: bool
    goal_verified: bool
    physical_goal_verified: bool = False
    physical_execution_performed: bool = False
    planned_only: bool = True
    node_records: tuple[NodeRuntimeRecord, ...] = ()
    budget_usage: BudgetUsage = field(default_factory=BudgetUsage)
    world_snapshot_ref: str | None = None
    event_log_ref: str | None = None
    checkpoint_ref: str | None = None
    errors: tuple[ErrorInfo | dict[str, Any], ...] = ()
    warnings: tuple[str, ...] = ()
    artifacts: dict[str, Any] = field(default_factory=dict)
    started_at: str = field(default_factory=utc_now_iso)
    ended_at: str = field(default_factory=utc_now_iso)

    def __post_init__(self) -> None:
        self.outcome = self.outcome if isinstance(self.outcome, ExecutionOutcome) else ExecutionOutcome(str(self.outcome))
        self.execution_scope = self.execution_scope if isinstance(self.execution_scope, ExecutionScope) else ExecutionScope(str(self.execution_scope))
        self.node_records = tuple(item if isinstance(item, NodeRuntimeRecord) else NodeRuntimeRecord.from_dict(item) for item in self.node_records)
        self.budget_usage = self.budget_usage if isinstance(self.budget_usage, BudgetUsage) else BudgetUsage.from_dict(self.budget_usage)
        self.errors = tuple(item if isinstance(item, ErrorInfo) else ErrorInfo.from_dict(item) for item in self.errors)
        self.artifacts = ensure_jsonable(dict(self.artifacts), "artifacts")
        # S7 is intentionally unable to claim physical execution.
        self.physical_execution_performed = False
        self.physical_goal_verified = False

    def to_dict(self) -> dict[str, Any]:
        return {"run_id": self.run_id, "graph_id": self.graph_id, "goal_id": self.goal_id, "plan_hash": self.plan_hash, "mode": self.mode, "terminal_state": self.terminal_state, "outcome": self.outcome.value, "execution_scope": self.execution_scope.value, "graph_completed": self.graph_completed, "goal_verified": self.goal_verified, "physical_goal_verified": False, "physical_execution_performed": False, "planned_only": self.planned_only, "node_records": [item.to_dict() for item in self.node_records], "budget_usage": self.budget_usage.to_dict(), "world_snapshot_ref": self.world_snapshot_ref, "event_log_ref": self.event_log_ref, "checkpoint_ref": self.checkpoint_ref, "errors": [item.to_dict() for item in self.errors], "warnings": list(self.warnings), "artifacts": dict(self.artifacts), "started_at": self.started_at, "ended_at": self.ended_at}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "TaskExecutionResult":
        return cls(data["run_id"], data["graph_id"], data["goal_id"], data["plan_hash"], data["mode"], data["terminal_state"], data["outcome"], data["execution_scope"], data["graph_completed"], data["goal_verified"], False, False, data.get("planned_only", True), tuple(data.get("node_records", ())), BudgetUsage.from_dict(data.get("budget_usage", {})), data.get("world_snapshot_ref"), data.get("event_log_ref"), data.get("checkpoint_ref"), tuple(data.get("errors", ())), tuple(data.get("warnings", ())), data.get("artifacts", {}), data["started_at"], data["ended_at"])

    def to_json(self) -> str:
        return stable_dumps(self.to_dict())
