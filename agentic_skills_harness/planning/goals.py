from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping

from ..contracts.serialization import ContractValidationError, ensure_jsonable, reject_unknown, require_bool, require_number, require_string
from ..world.predicates import PredicateSpec
from .policies import reject_execution_fields, require_id


class GoalKind(str, Enum):
    OBSERVATION = "OBSERVATION"
    COMPUTE = "COMPUTE"
    PHYSICAL_STATE_CHANGE = "PHYSICAL_STATE_CHANGE"


TERMINAL_STATES = {"FAILED", "NEEDS_HUMAN", "UNSAFE", "TIMEOUT", "BUDGET_EXCEEDED", "NO_PROGRESS", "CANCELLED"}


@dataclass(frozen=True)
class EntitySpec:
    entity_id: str
    entity_type: str
    constraints: Mapping[str, Any] = field(default_factory=dict)
    attributes: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "entity_id", require_id(self.entity_id, "entity_id"))
        object.__setattr__(self, "entity_type", require_string(self.entity_type, "entity_type", non_empty=True))
        constraints = ensure_jsonable(dict(self.constraints), "constraints")
        attributes = ensure_jsonable(dict(self.attributes), "attributes")
        reject_execution_fields(constraints, "constraints")
        reject_execution_fields(attributes, "attributes")
        object.__setattr__(self, "constraints", constraints)
        object.__setattr__(self, "attributes", attributes)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "EntitySpec":
        reject_unknown(data, ("entity_id", "entity_type", "constraints", "attributes"), ("entity_id", "entity_type"))
        return cls(data["entity_id"], data["entity_type"], data.get("constraints", {}), data.get("attributes", {}))

    def to_dict(self) -> dict[str, Any]:
        return {"entity_id": self.entity_id, "entity_type": self.entity_type, "constraints": dict(self.constraints), "attributes": dict(self.attributes)}


@dataclass(frozen=True)
class EvidenceRequirement:
    evidence_id: str
    fact_selector: Mapping[str, Any]
    required_status: str = "VERIFIED"
    freshness_required: bool = True
    minimum_confidence: float = 0.0
    frame_requirement: str | None = None
    description: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "evidence_id", require_id(self.evidence_id, "evidence_id"))
        selector = ensure_jsonable(dict(self.fact_selector), "fact_selector")
        reject_execution_fields(selector, "fact_selector")
        object.__setattr__(self, "fact_selector", selector)
        object.__setattr__(self, "required_status", require_string(self.required_status, "required_status", non_empty=True).upper())
        object.__setattr__(self, "freshness_required", require_bool(self.freshness_required, "freshness_required"))
        confidence = require_number(self.minimum_confidence, "minimum_confidence", minimum=0.0)
        if confidence > 1.0:
            raise ContractValidationError("minimum_confidence must be at most 1")
        object.__setattr__(self, "minimum_confidence", confidence)
        if self.frame_requirement is not None:
            object.__setattr__(self, "frame_requirement", require_string(self.frame_requirement, "frame_requirement", non_empty=True))
        object.__setattr__(self, "description", require_string(self.description, "description"))

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "EvidenceRequirement":
        allowed = ("evidence_id", "fact_selector", "required_status", "freshness_required", "minimum_confidence", "frame_requirement", "description")
        reject_unknown(data, allowed, ("evidence_id", "fact_selector"))
        return cls(data["evidence_id"], data["fact_selector"], data.get("required_status", "VERIFIED"), data.get("freshness_required", True), data.get("minimum_confidence", 0.0), data.get("frame_requirement"), data.get("description", ""))

    def to_dict(self) -> dict[str, Any]:
        return {"evidence_id": self.evidence_id, "fact_selector": dict(self.fact_selector), "required_status": self.required_status, "freshness_required": self.freshness_required, "minimum_confidence": self.minimum_confidence, "frame_requirement": self.frame_requirement, "description": self.description}


@dataclass(frozen=True)
class FailurePredicate:
    predicate: PredicateSpec
    terminal_state: str
    reason: str

    def __post_init__(self) -> None:
        predicate = self.predicate if isinstance(self.predicate, PredicateSpec) else PredicateSpec.from_dict(self.predicate)
        object.__setattr__(self, "predicate", predicate)
        terminal = require_string(self.terminal_state, "terminal_state", non_empty=True).upper()
        if terminal not in TERMINAL_STATES:
            raise ContractValidationError(f"invalid terminal_state: {terminal}")
        object.__setattr__(self, "terminal_state", terminal)
        object.__setattr__(self, "reason", require_string(self.reason, "reason", non_empty=True))

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "FailurePredicate":
        reject_unknown(data, ("predicate", "terminal_state", "reason"), ("predicate", "terminal_state", "reason"))
        return cls(PredicateSpec.from_dict(data["predicate"]), data["terminal_state"], data["reason"])

    def to_dict(self) -> dict[str, Any]:
        return {"predicate": self.predicate.to_dict(), "terminal_state": self.terminal_state, "reason": self.reason}


@dataclass(frozen=True)
class Ambiguity:
    ambiguity_id: str
    description: str
    blocking: bool
    options: tuple[str, ...] = ()
    selected_resolution: str | None = None
    source: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "ambiguity_id", require_id(self.ambiguity_id, "ambiguity_id"))
        object.__setattr__(self, "description", require_string(self.description, "description", non_empty=True))
        object.__setattr__(self, "blocking", require_bool(self.blocking, "blocking"))
        values = tuple(self.options)
        if any(not isinstance(item, str) or not item.strip() for item in values):
            raise ContractValidationError("ambiguity options must be non-empty strings")
        object.__setattr__(self, "options", values)
        if self.selected_resolution is not None:
            object.__setattr__(self, "selected_resolution", require_string(self.selected_resolution, "selected_resolution", non_empty=True))
        object.__setattr__(self, "source", require_string(self.source, "source"))

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "Ambiguity":
        allowed = ("ambiguity_id", "description", "blocking", "options", "selected_resolution", "source")
        reject_unknown(data, allowed, ("ambiguity_id", "description", "blocking"))
        return cls(data["ambiguity_id"], data["description"], data["blocking"], tuple(data.get("options", ())), data.get("selected_resolution"), data.get("source", ""))

    def to_dict(self) -> dict[str, Any]:
        return {"ambiguity_id": self.ambiguity_id, "description": self.description, "blocking": self.blocking, "options": list(self.options), "selected_resolution": self.selected_resolution, "source": self.source}


@dataclass(frozen=True)
class GoalSpec:
    goal_id: str
    goal_kind: GoalKind | str
    description: str
    entities: tuple[EntitySpec, ...] = ()
    initial_assumptions: tuple[Mapping[str, Any], ...] = ()
    success_predicate: PredicateSpec | Mapping[str, Any] = field(default_factory=lambda: PredicateSpec("exists", ({"predicate": "goal.placeholder"},)))
    failure_predicates: tuple[FailurePredicate, ...] = ()
    required_evidence: tuple[EvidenceRequirement, ...] = ()
    ambiguities: tuple[Ambiguity, ...] = ()
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "goal_id", require_id(self.goal_id, "goal_id"))
        object.__setattr__(self, "goal_kind", self.goal_kind if isinstance(self.goal_kind, GoalKind) else GoalKind(str(self.goal_kind)))
        object.__setattr__(self, "description", require_string(self.description, "description", non_empty=True))
        object.__setattr__(self, "entities", tuple(item if isinstance(item, EntitySpec) else EntitySpec.from_dict(item) for item in self.entities))
        assumptions = tuple(ensure_jsonable(dict(item), "initial_assumption") for item in self.initial_assumptions)
        for item in assumptions:
            reject_execution_fields(item, "initial_assumptions")
        object.__setattr__(self, "initial_assumptions", assumptions)
        predicate = self.success_predicate if isinstance(self.success_predicate, PredicateSpec) else PredicateSpec.from_dict(self.success_predicate)
        object.__setattr__(self, "success_predicate", predicate)
        object.__setattr__(self, "failure_predicates", tuple(item if isinstance(item, FailurePredicate) else FailurePredicate.from_dict(item) for item in self.failure_predicates))
        object.__setattr__(self, "required_evidence", tuple(item if isinstance(item, EvidenceRequirement) else EvidenceRequirement.from_dict(item) for item in self.required_evidence))
        object.__setattr__(self, "ambiguities", tuple(item if isinstance(item, Ambiguity) else Ambiguity.from_dict(item) for item in self.ambiguities))
        metadata = ensure_jsonable(dict(self.metadata), "metadata")
        reject_execution_fields(metadata, "metadata")
        object.__setattr__(self, "metadata", metadata)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "GoalSpec":
        allowed = ("goal_id", "goal_kind", "description", "entities", "initial_assumptions", "success_predicate", "failure_predicates", "required_evidence", "ambiguities", "metadata")
        reject_unknown(data, allowed, ("goal_id", "goal_kind", "description", "success_predicate"))
        reject_execution_fields(data)
        return cls(data["goal_id"], data["goal_kind"], data["description"], tuple(data.get("entities", ())), tuple(data.get("initial_assumptions", ())), data["success_predicate"], tuple(data.get("failure_predicates", ())), tuple(data.get("required_evidence", ())), tuple(data.get("ambiguities", ())), data.get("metadata", {}))

    def to_dict(self) -> dict[str, Any]:
        return {"goal_id": self.goal_id, "goal_kind": self.goal_kind.value, "description": self.description, "entities": [item.to_dict() for item in self.entities], "initial_assumptions": [dict(item) for item in self.initial_assumptions], "success_predicate": self.success_predicate.to_dict(), "failure_predicates": [item.to_dict() for item in self.failure_predicates], "required_evidence": [item.to_dict() for item in self.required_evidence], "ambiguities": [item.to_dict() for item in self.ambiguities], "metadata": dict(self.metadata)}
