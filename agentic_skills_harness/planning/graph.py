from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping

from ..contracts.resources import ResourceRequirement
from ..contracts.serialization import ContractValidationError, ensure_jsonable, reject_unknown, require_bool, require_number, require_string
from ..world.predicates import PredicateSpec
from .bindings import InputBinding
from .policies import reject_execution_fields, require_id


class NodeKind(str, Enum):
    OBSERVE = "OBSERVE"
    COMPUTE = "COMPUTE"
    CHECK = "CHECK"
    ACT = "ACT"
    VERIFY = "VERIFY"
    RECOVER = "RECOVER"
    APPROVAL = "APPROVAL"
    HUMAN_ACTION = "HUMAN_ACTION"


class EdgeCondition(str, Enum):
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"
    PREDICATE_TRUE = "PREDICATE_TRUE"
    PREDICATE_FALSE = "PREDICATE_FALSE"
    DEFAULT = "DEFAULT"


@dataclass(frozen=True)
class RetryPolicy:
    max_attempts: int = 1
    retry_on_error_codes: tuple[str, ...] = ()
    backoff_policy: Mapping[str, Any] = field(default_factory=lambda: {"kind": "none"})
    parameter_adjustment_policy: str | None = None

    def __post_init__(self) -> None:
        if isinstance(self.max_attempts, bool) or not isinstance(self.max_attempts, int) or not 1 <= self.max_attempts <= 64:
            raise ContractValidationError("max_attempts must be an integer from 1 to 64")
        codes = tuple(self.retry_on_error_codes)
        if any(not isinstance(item, str) or not item.strip() for item in codes):
            raise ContractValidationError("retry_on_error_codes must contain strings")
        object.__setattr__(self, "retry_on_error_codes", codes)
        backoff = ensure_jsonable(dict(self.backoff_policy), "backoff_policy")
        reject_execution_fields(backoff, "backoff_policy")
        kind = backoff.get("kind", "none")
        if kind not in {"none", "fixed", "exponential"}:
            raise ContractValidationError("backoff kind must be none, fixed, or exponential")
        if "max_delay_s" in backoff:
            require_number(backoff["max_delay_s"], "backoff max_delay_s", minimum=0.0)
        object.__setattr__(self, "backoff_policy", backoff)
        if self.parameter_adjustment_policy is not None:
            object.__setattr__(self, "parameter_adjustment_policy", require_string(self.parameter_adjustment_policy, "parameter_adjustment_policy", non_empty=True))

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "RetryPolicy":
        allowed = ("max_attempts", "retry_on_error_codes", "backoff_policy", "parameter_adjustment_policy")
        reject_unknown(data, allowed, ("max_attempts",))
        return cls(data["max_attempts"], tuple(data.get("retry_on_error_codes", ())), data.get("backoff_policy", {"kind": "none"}), data.get("parameter_adjustment_policy"))

    def to_dict(self) -> dict[str, Any]:
        return {"max_attempts": self.max_attempts, "retry_on_error_codes": list(self.retry_on_error_codes), "backoff_policy": dict(self.backoff_policy), "parameter_adjustment_policy": self.parameter_adjustment_policy}


@dataclass(frozen=True)
class ExpectedEffect:
    effect_id: str
    predicate: PredicateSpec | Mapping[str, Any]
    verification_required: bool = True
    state_status_on_unverified: str = "TENTATIVE"

    def __post_init__(self) -> None:
        object.__setattr__(self, "effect_id", require_id(self.effect_id, "effect_id"))
        predicate = self.predicate if isinstance(self.predicate, PredicateSpec) else PredicateSpec.from_dict(self.predicate)
        object.__setattr__(self, "predicate", predicate)
        object.__setattr__(self, "verification_required", require_bool(self.verification_required, "verification_required"))
        status = require_string(self.state_status_on_unverified, "state_status_on_unverified", non_empty=True).upper()
        if status not in {"TENTATIVE", "OBSERVED", "VERIFIED", "INVALIDATED"}:
            raise ContractValidationError("invalid state_status_on_unverified")
        if status == "VERIFIED":
            raise ContractValidationError("graph cannot create VERIFIED effects before verification")
        object.__setattr__(self, "state_status_on_unverified", status)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "ExpectedEffect":
        allowed = ("effect_id", "predicate", "verification_required", "state_status_on_unverified")
        reject_unknown(data, allowed, ("effect_id", "predicate"))
        return cls(data["effect_id"], data["predicate"], data.get("verification_required", True), data.get("state_status_on_unverified", "TENTATIVE"))

    def to_dict(self) -> dict[str, Any]:
        return {"effect_id": self.effect_id, "predicate": self.predicate.to_dict(), "verification_required": self.verification_required, "state_status_on_unverified": self.state_status_on_unverified}


@dataclass(frozen=True)
class GraphNode:
    node_id: str
    kind: NodeKind | str
    capability_id: str | None = None
    arguments: Mapping[str, Any] = field(default_factory=dict)
    input_bindings: tuple[InputBinding, ...] = ()
    preconditions: tuple[PredicateSpec, ...] = ()
    expected_effects: tuple[ExpectedEffect, ...] = ()
    verifier: Mapping[str, Any] | None = None
    resource_requirements: tuple[ResourceRequirement, ...] = ()
    timeout_s: float | None = None
    retry_policy: RetryPolicy = field(default_factory=RetryPolicy)
    max_visits: int = 1
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "node_id", require_id(self.node_id, "node_id"))
        object.__setattr__(self, "kind", self.kind if isinstance(self.kind, NodeKind) else NodeKind(str(self.kind)))
        if self.capability_id is not None:
            object.__setattr__(self, "capability_id", require_id(self.capability_id, "capability_id"))
        arguments = ensure_jsonable(dict(self.arguments), "arguments")
        reject_execution_fields(arguments, "arguments")
        object.__setattr__(self, "arguments", arguments)
        bindings = tuple(item if isinstance(item, InputBinding) else InputBinding.from_dict(item) for item in self.input_bindings)
        if len({item.binding_id for item in bindings}) != len(bindings):
            raise ContractValidationError("duplicate input binding IDs")
        object.__setattr__(self, "input_bindings", bindings)
        predicates = tuple(item if isinstance(item, PredicateSpec) else PredicateSpec.from_dict(item) for item in self.preconditions)
        object.__setattr__(self, "preconditions", predicates)
        object.__setattr__(self, "expected_effects", tuple(item if isinstance(item, ExpectedEffect) else ExpectedEffect.from_dict(item) for item in self.expected_effects))
        if self.verifier is not None:
            verifier = ensure_jsonable(dict(self.verifier), "verifier")
            reject_execution_fields(verifier, "verifier")
            object.__setattr__(self, "verifier", verifier)
        object.__setattr__(self, "resource_requirements", tuple(item if isinstance(item, ResourceRequirement) else ResourceRequirement.from_dict(item) for item in self.resource_requirements))
        if self.timeout_s is not None:
            object.__setattr__(self, "timeout_s", require_number(self.timeout_s, "timeout_s", positive=True))
        retry = self.retry_policy if isinstance(self.retry_policy, RetryPolicy) else RetryPolicy.from_dict(self.retry_policy)
        object.__setattr__(self, "retry_policy", retry)
        if isinstance(self.max_visits, bool) or not isinstance(self.max_visits, int) or self.max_visits < 1 or self.max_visits > 1024:
            raise ContractValidationError("max_visits must be from 1 to 1024")
        metadata = ensure_jsonable(dict(self.metadata), "metadata")
        reject_execution_fields(metadata, "metadata")
        object.__setattr__(self, "metadata", metadata)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "GraphNode":
        allowed = ("node_id", "kind", "capability_id", "arguments", "input_bindings", "preconditions", "expected_effects", "verifier", "resource_requirements", "timeout_s", "retry_policy", "max_visits", "metadata")
        reject_unknown(data, allowed, ("node_id", "kind"))
        reject_execution_fields(data)
        return cls(data["node_id"], data["kind"], data.get("capability_id"), data.get("arguments", {}), tuple(data.get("input_bindings", ())), tuple(data.get("preconditions", ())), tuple(data.get("expected_effects", ())), data.get("verifier"), tuple(data.get("resource_requirements", ())), data.get("timeout_s"), RetryPolicy.from_dict(data["retry_policy"]) if "retry_policy" in data else RetryPolicy(), data.get("max_visits", 1), data.get("metadata", {}))

    def to_dict(self) -> dict[str, Any]:
        return {"node_id": self.node_id, "kind": self.kind.value, "capability_id": self.capability_id, "arguments": dict(self.arguments), "input_bindings": [item.to_dict() for item in self.input_bindings], "preconditions": [item.to_dict() for item in self.preconditions], "expected_effects": [item.to_dict() for item in self.expected_effects], "verifier": None if self.verifier is None else dict(self.verifier), "resource_requirements": [item.to_dict() for item in self.resource_requirements], "timeout_s": self.timeout_s, "retry_policy": self.retry_policy.to_dict(), "max_visits": self.max_visits, "metadata": dict(self.metadata)}


@dataclass(frozen=True)
class GraphEdge:
    edge_id: str
    source_node_id: str
    target_node_id: str
    condition: EdgeCondition | str = EdgeCondition.DEFAULT
    error_codes: tuple[str, ...] = ()
    max_traversals: int | None = None
    priority: int = 0

    def __post_init__(self) -> None:
        object.__setattr__(self, "edge_id", require_id(self.edge_id, "edge_id"))
        object.__setattr__(self, "source_node_id", require_id(self.source_node_id, "source_node_id"))
        object.__setattr__(self, "target_node_id", require_id(self.target_node_id, "target_node_id"))
        object.__setattr__(self, "condition", self.condition if isinstance(self.condition, EdgeCondition) else EdgeCondition(str(self.condition)))
        codes = tuple(self.error_codes)
        if any(not isinstance(item, str) or not item.strip() for item in codes):
            raise ContractValidationError("edge error_codes must contain strings")
        object.__setattr__(self, "error_codes", codes)
        if self.max_traversals is not None and (isinstance(self.max_traversals, bool) or not isinstance(self.max_traversals, int) or self.max_traversals < 1 or self.max_traversals > 1024):
            raise ContractValidationError("max_traversals must be from 1 to 1024")
        if isinstance(self.priority, bool) or not isinstance(self.priority, int):
            raise ContractValidationError("edge priority must be an integer")

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "GraphEdge":
        allowed = ("edge_id", "source_node_id", "target_node_id", "condition", "error_codes", "max_traversals", "priority")
        reject_unknown(data, allowed, ("edge_id", "source_node_id", "target_node_id", "condition"))
        return cls(data["edge_id"], data["source_node_id"], data["target_node_id"], data["condition"], tuple(data.get("error_codes", ())), data.get("max_traversals"), data.get("priority", 0))

    def to_dict(self) -> dict[str, Any]:
        return {"edge_id": self.edge_id, "source_node_id": self.source_node_id, "target_node_id": self.target_node_id, "condition": self.condition.value, "error_codes": list(self.error_codes), "max_traversals": self.max_traversals, "priority": self.priority}


@dataclass(frozen=True)
class TaskGraph:
    graph_id: str
    goal_id: str
    entry_node_id: str
    nodes: tuple[GraphNode, ...]
    edges: tuple[GraphEdge, ...] = ()
    terminal_nodes: tuple[str, ...] = ()
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "graph_id", require_id(self.graph_id, "graph_id"))
        object.__setattr__(self, "goal_id", require_id(self.goal_id, "goal_id"))
        object.__setattr__(self, "entry_node_id", require_id(self.entry_node_id, "entry_node_id"))
        nodes = tuple(item if isinstance(item, GraphNode) else GraphNode.from_dict(item) for item in self.nodes)
        if not nodes:
            raise ContractValidationError("task graph must contain nodes")
        if len({item.node_id for item in nodes}) != len(nodes):
            raise ContractValidationError("duplicate node IDs")
        object.__setattr__(self, "nodes", nodes)
        edges = tuple(item if isinstance(item, GraphEdge) else GraphEdge.from_dict(item) for item in self.edges)
        if len({item.edge_id for item in edges}) != len(edges):
            raise ContractValidationError("duplicate edge IDs")
        object.__setattr__(self, "edges", edges)
        terminals = tuple(self.terminal_nodes)
        if any(not isinstance(item, str) or not item.strip() for item in terminals):
            raise ContractValidationError("terminal_nodes must contain IDs")
        object.__setattr__(self, "terminal_nodes", terminals)
        metadata = ensure_jsonable(dict(self.metadata), "graph metadata")
        reject_execution_fields(metadata, "metadata")
        object.__setattr__(self, "metadata", metadata)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "TaskGraph":
        allowed = ("graph_id", "goal_id", "entry_node_id", "nodes", "edges", "terminal_nodes", "metadata")
        reject_unknown(data, allowed, ("graph_id", "goal_id", "entry_node_id", "nodes"))
        reject_execution_fields(data)
        return cls(data["graph_id"], data["goal_id"], data["entry_node_id"], tuple(data["nodes"]), tuple(data.get("edges", ())), tuple(data.get("terminal_nodes", ())), data.get("metadata", {}))

    def to_dict(self) -> dict[str, Any]:
        return {"graph_id": self.graph_id, "goal_id": self.goal_id, "entry_node_id": self.entry_node_id, "nodes": [item.to_dict() for item in self.nodes], "edges": [item.to_dict() for item in self.edges], "terminal_nodes": list(self.terminal_nodes), "metadata": dict(self.metadata)}
