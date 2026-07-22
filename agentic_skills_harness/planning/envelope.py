from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping

from ..contracts.enums import ResourceMode, RiskClass
from ..contracts.resources import ResourceRequirement
from ..contracts.serialization import ContractValidationError, ensure_jsonable, reject_unknown, require_bool, require_number, require_string
from .policies import reject_execution_fields, require_id


class TargetMode(str, Enum):
    MOCK = "mock"
    DRY_RUN = "dry_run"
    FROM_ARTIFACTS = "from_artifacts"
    LIVE = "live"


@dataclass(frozen=True)
class WorkspaceConstraint:
    workspace_id: str
    frame: str
    min_xyz_m: tuple[float, float, float]
    max_xyz_m: tuple[float, float, float]
    allowed_entity_types: tuple[str, ...] = ()
    description: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "workspace_id", require_id(self.workspace_id, "workspace_id"))
        object.__setattr__(self, "frame", require_string(self.frame, "frame", non_empty=True))
        mins = tuple(float(item) for item in self.min_xyz_m)
        maxs = tuple(float(item) for item in self.max_xyz_m)
        if len(mins) != 3 or len(maxs) != 3 or any(item >= upper for item, upper in zip(mins, maxs)):
            raise ContractValidationError("workspace min_xyz_m must be less than max_xyz_m on every axis")
        for value in (*mins, *maxs):
            require_number(value, "workspace coordinate")
        object.__setattr__(self, "min_xyz_m", mins)
        object.__setattr__(self, "max_xyz_m", maxs)
        types = tuple(self.allowed_entity_types)
        if any(not isinstance(item, str) or not item.strip() for item in types):
            raise ContractValidationError("allowed_entity_types must contain non-empty strings")
        object.__setattr__(self, "allowed_entity_types", types)
        object.__setattr__(self, "description", require_string(self.description, "description"))

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "WorkspaceConstraint":
        allowed = ("workspace_id", "frame", "min_xyz_m", "max_xyz_m", "allowed_entity_types", "description")
        reject_unknown(data, allowed, ("workspace_id", "frame", "min_xyz_m", "max_xyz_m"))
        return cls(data["workspace_id"], data["frame"], tuple(data["min_xyz_m"]), tuple(data["max_xyz_m"]), tuple(data.get("allowed_entity_types", ())), data.get("description", ""))

    def to_dict(self) -> dict[str, Any]:
        return {"workspace_id": self.workspace_id, "frame": self.frame, "min_xyz_m": list(self.min_xyz_m), "max_xyz_m": list(self.max_xyz_m), "allowed_entity_types": list(self.allowed_entity_types), "description": self.description}


@dataclass(frozen=True)
class ApprovalPolicy:
    required_for_risk_classes: tuple[str, ...] = ()
    estop_requires_human: bool = True
    ambiguity_requires_human: bool = True
    description: str = ""

    def __post_init__(self) -> None:
        values = tuple(item.value if isinstance(item, RiskClass) else str(item) for item in self.required_for_risk_classes)
        unknown = set(values) - {item.value for item in RiskClass}
        if unknown:
            raise ContractValidationError(f"unknown approval risk class: {sorted(unknown)}")
        object.__setattr__(self, "required_for_risk_classes", values)
        object.__setattr__(self, "estop_requires_human", require_bool(self.estop_requires_human, "estop_requires_human"))
        object.__setattr__(self, "ambiguity_requires_human", require_bool(self.ambiguity_requires_human, "ambiguity_requires_human"))
        object.__setattr__(self, "description", require_string(self.description, "description"))

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "ApprovalPolicy":
        reject_unknown(data, ("required_for_risk_classes", "estop_requires_human", "ambiguity_requires_human", "description"))
        return cls(tuple(data.get("required_for_risk_classes", ())), data.get("estop_requires_human", True), data.get("ambiguity_requires_human", True), data.get("description", ""))

    def to_dict(self) -> dict[str, Any]:
        return {"required_for_risk_classes": list(self.required_for_risk_classes), "estop_requires_human": self.estop_requires_human, "ambiguity_requires_human": self.ambiguity_requires_human, "description": self.description}


@dataclass(frozen=True)
class ExecutionEnvelope:
    envelope_id: str
    target_mode: TargetMode | str
    allowed_capabilities: tuple[str, ...] = ()
    forbidden_capabilities: tuple[str, ...] = ()
    risk_ceiling: RiskClass | str = RiskClass.READ_ONLY_HARDWARE
    allowed_resources: tuple[str, ...] = ()
    workspace_constraints: tuple[WorkspaceConstraint, ...] = ()
    max_nodes: int = 64
    max_depth: int = 32
    max_elapsed_s: float = 300.0
    max_tool_calls: int = 64
    max_replans: int = 0
    max_recovery_actions: int = 0
    max_same_error_retries: int = 0
    no_progress_limit: int = 3
    approval_policy: ApprovalPolicy | Mapping[str, Any] = field(default_factory=ApprovalPolicy)
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "envelope_id", require_id(self.envelope_id, "envelope_id"))
        object.__setattr__(self, "target_mode", self.target_mode if isinstance(self.target_mode, TargetMode) else TargetMode(str(self.target_mode)))
        for name in ("allowed_capabilities", "forbidden_capabilities", "allowed_resources"):
            values = tuple(getattr(self, name))
            if any(not isinstance(item, str) or not item.strip() for item in values):
                raise ContractValidationError(f"{name} must contain non-empty strings")
            object.__setattr__(self, name, values)
        object.__setattr__(self, "risk_ceiling", self.risk_ceiling if isinstance(self.risk_ceiling, RiskClass) else RiskClass(str(self.risk_ceiling)))
        object.__setattr__(self, "workspace_constraints", tuple(item if isinstance(item, WorkspaceConstraint) else WorkspaceConstraint.from_dict(item) for item in self.workspace_constraints))
        for name in ("max_nodes", "max_depth", "max_tool_calls", "max_replans", "max_recovery_actions", "max_same_error_retries", "no_progress_limit"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < (1 if name in {"max_nodes", "max_depth", "max_tool_calls", "no_progress_limit"} else 0):
                raise ContractValidationError(f"{name} must be a bounded integer")
            object.__setattr__(self, name, value)
        object.__setattr__(self, "max_elapsed_s", require_number(self.max_elapsed_s, "max_elapsed_s", positive=True))
        policy = self.approval_policy if isinstance(self.approval_policy, ApprovalPolicy) else ApprovalPolicy.from_dict(self.approval_policy)
        object.__setattr__(self, "approval_policy", policy)
        metadata = ensure_jsonable(dict(self.metadata), "metadata")
        reject_execution_fields(metadata, "metadata")
        object.__setattr__(self, "metadata", metadata)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "ExecutionEnvelope":
        allowed = ("envelope_id", "target_mode", "allowed_capabilities", "forbidden_capabilities", "risk_ceiling", "allowed_resources", "workspace_constraints", "max_nodes", "max_depth", "max_elapsed_s", "max_tool_calls", "max_replans", "max_recovery_actions", "max_same_error_retries", "no_progress_limit", "approval_policy", "metadata")
        reject_unknown(data, allowed, ("envelope_id", "target_mode", "risk_ceiling"))
        reject_execution_fields(data)
        return cls(data["envelope_id"], data["target_mode"], tuple(data.get("allowed_capabilities", ())), tuple(data.get("forbidden_capabilities", ())), data["risk_ceiling"], tuple(data.get("allowed_resources", ())), tuple(data.get("workspace_constraints", ())), data.get("max_nodes", 64), data.get("max_depth", 32), data.get("max_elapsed_s", 300.0), data.get("max_tool_calls", 64), data.get("max_replans", 0), data.get("max_recovery_actions", 0), data.get("max_same_error_retries", 0), data.get("no_progress_limit", 3), data.get("approval_policy", {}), data.get("metadata", {}))

    def to_dict(self) -> dict[str, Any]:
        return {"envelope_id": self.envelope_id, "target_mode": self.target_mode.value, "allowed_capabilities": list(self.allowed_capabilities), "forbidden_capabilities": list(self.forbidden_capabilities), "risk_ceiling": self.risk_ceiling.value, "allowed_resources": list(self.allowed_resources), "workspace_constraints": [item.to_dict() for item in self.workspace_constraints], "max_nodes": self.max_nodes, "max_depth": self.max_depth, "max_elapsed_s": self.max_elapsed_s, "max_tool_calls": self.max_tool_calls, "max_replans": self.max_replans, "max_recovery_actions": self.max_recovery_actions, "max_same_error_retries": self.max_same_error_retries, "no_progress_limit": self.no_progress_limit, "approval_policy": self.approval_policy.to_dict(), "metadata": dict(self.metadata)}
