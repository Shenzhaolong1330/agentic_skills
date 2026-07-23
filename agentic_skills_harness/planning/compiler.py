from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass, field
from math import isfinite
from pathlib import Path
from typing import Any, Iterable, Mapping

from ..capability import CapabilityContract
from ..contracts.enums import CapabilityKind, ErrorCode, ResourceMode, RiskClass
from ..contracts.serialization import ContractValidationError, reject_unknown, stable_dumps, utc_now_iso
from ..registry import CapabilityRegistry
from ..schema_validation import validate_json
from .bindings import BindingSource, InputBinding
from .canonical import digest
from .diagnostics import CompilationIssue, CompilationReport, IssueSeverity
from .envelope import ExecutionEnvelope, TargetMode
from .goals import GoalKind, GoalSpec
from .graph import EdgeCondition, GraphEdge, GraphNode, NodeKind, TaskGraph
from .policies import reject_execution_fields


RISK_ORDER = {
    RiskClass.NONE: 0,
    RiskClass.READ_ONLY_HARDWARE: 1,
    RiskClass.MOTION: 2,
    RiskClass.GRIPPER: 3,
    RiskClass.CONTACT: 4,
    RiskClass.RELEASE: 5,
    RiskClass.RECOVERY: 6,
    RiskClass.HIGH_RISK: 7,
}


_TASK_CONTEXT_TOKEN = object()


@dataclass(frozen=True)
class TaskCompilationContext:
    """Non-JSON authority for a trusted first-party TaskDefinition.

    An external graph can mention an internal capability ID, but the compiler
    accepts it only when this object was created by ``trusted``.  The authority
    token is intentionally not serializable and is never read from graph or
    envelope JSON.
    """

    task_id: str
    task_version: str
    internal_capability_allowlist: tuple[str, ...] = ()
    _authority: object | None = field(default=None, repr=False, compare=False)

    @classmethod
    def trusted(cls, task_id: str, task_version: str, internal_capability_allowlist: Iterable[str]) -> "TaskCompilationContext":
        return cls(str(task_id), str(task_version), tuple(sorted(set(str(item) for item in internal_capability_allowlist))), _TASK_CONTEXT_TOKEN)

    @property
    def is_trusted(self) -> bool:
        return self._authority is _TASK_CONTEXT_TOKEN


@dataclass(frozen=True)
class CompiledNode:
    node_id: str
    kind: str
    capability_id: str | None
    capability_version: str | None
    dispatch_status: str
    execution_disposition: str
    risk_class: str | None
    resources: tuple[dict[str, Any], ...]
    arguments: dict[str, Any]
    input_bindings: tuple[dict[str, Any], ...]
    preconditions: tuple[dict[str, Any], ...]
    expected_effects: tuple[dict[str, Any], ...]
    verifier_contract: dict[str, Any] | None
    timeout_s: float | None
    retry_policy: dict[str, Any]
    max_visits: int
    input_schema_digest: str | None
    output_schema_digest: str | None
    metadata: dict[str, Any] = field(default_factory=dict)
    fact_projections: tuple[dict[str, Any], ...] = ()

    def to_dict(self) -> dict[str, Any]:
        value = {
            "node_id": self.node_id, "kind": self.kind, "capability_id": self.capability_id,
            "capability_version": self.capability_version, "dispatch_status": self.dispatch_status,
            "execution_disposition": self.execution_disposition, "risk_class": self.risk_class,
            "resources": [dict(item) for item in self.resources], "arguments": dict(self.arguments),
            "input_bindings": [dict(item) for item in self.input_bindings],
            "preconditions": [dict(item) for item in self.preconditions],
            "expected_effects": [dict(item) for item in self.expected_effects],
            "verifier_contract": None if self.verifier_contract is None else dict(self.verifier_contract),
            "timeout_s": self.timeout_s, "retry_policy": dict(self.retry_policy), "max_visits": self.max_visits,
            "input_schema_digest": self.input_schema_digest, "output_schema_digest": self.output_schema_digest,
        }
        if self.metadata:
            value["metadata"] = dict(self.metadata)
        if self.fact_projections:
            value["fact_projections"] = [dict(item) for item in self.fact_projections]
        return value

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "CompiledNode":
        allowed = {"node_id", "kind", "capability_id", "capability_version", "dispatch_status", "execution_disposition", "risk_class", "resources", "arguments", "input_bindings", "preconditions", "expected_effects", "verifier_contract", "timeout_s", "retry_policy", "max_visits", "input_schema_digest", "output_schema_digest", "metadata", "fact_projections"}
        reject_unknown(data, allowed, ("node_id", "kind", "dispatch_status", "execution_disposition"))
        return cls(
            node_id=data["node_id"], kind=data["kind"], capability_id=data.get("capability_id"), capability_version=data.get("capability_version"),
            dispatch_status=data["dispatch_status"], execution_disposition=data["execution_disposition"], risk_class=data.get("risk_class"),
            resources=tuple(data.get("resources", ())), arguments=dict(data.get("arguments", {})), input_bindings=tuple(data.get("input_bindings", ())),
            preconditions=tuple(data.get("preconditions", ())), expected_effects=tuple(data.get("expected_effects", ())), verifier_contract=data.get("verifier_contract"),
            timeout_s=data.get("timeout_s"), retry_policy=dict(data.get("retry_policy", {"max_attempts": 1, "retry_on_error_codes": [], "backoff_policy": {"kind": "none"}, "parameter_adjustment_policy": None})),
            max_visits=data.get("max_visits", 1), input_schema_digest=data.get("input_schema_digest"), output_schema_digest=data.get("output_schema_digest"),
            metadata=dict(data.get("metadata", {})), fact_projections=tuple(data.get("fact_projections", ())),
        )


@dataclass(frozen=True)
class CompiledEdge:
    edge_id: str
    source_node_id: str
    target_node_id: str
    condition: str
    error_codes: tuple[str, ...]
    max_traversals: int | None
    priority: int

    def to_dict(self) -> dict[str, Any]:
        return {"edge_id": self.edge_id, "source_node_id": self.source_node_id, "target_node_id": self.target_node_id, "condition": self.condition, "error_codes": list(self.error_codes), "max_traversals": self.max_traversals, "priority": self.priority}


@dataclass(frozen=True)
class CompiledTaskGraph:
    compiled_graph_version: str
    graph_id: str
    goal_id: str
    target_mode: str
    manifest_version: str
    manifest_digest: str
    capability_index_digest: str
    goal_digest: str
    envelope_digest: str
    source_graph_digest: str
    plan_hash: str
    compiled_nodes: tuple[CompiledNode, ...]
    compiled_edges: tuple[CompiledEdge, ...]
    terminal_nodes: tuple[str, ...]
    budgets: dict[str, Any]
    workspace_constraints: tuple[dict[str, Any], ...]
    warnings: tuple[dict[str, Any], ...]
    compiled_at: str
    goal_predicate: dict[str, Any] | None = None
    goal_kind: str | None = None

    def _hash_payload(self) -> dict[str, Any]:
        payload = self.to_dict()
        payload.pop("plan_hash", None)
        payload.pop("compiled_at", None)
        return payload

    def to_dict(self) -> dict[str, Any]:
        value = {
            "compiled_graph_version": self.compiled_graph_version, "graph_id": self.graph_id, "goal_id": self.goal_id,
            "target_mode": self.target_mode, "manifest_version": self.manifest_version, "manifest_digest": self.manifest_digest,
            "capability_index_digest": self.capability_index_digest, "goal_digest": self.goal_digest,
            "envelope_digest": self.envelope_digest, "source_graph_digest": self.source_graph_digest,
            "plan_hash": self.plan_hash, "compiled_nodes": [item.to_dict() for item in self.compiled_nodes],
            "compiled_edges": [item.to_dict() for item in self.compiled_edges], "terminal_nodes": list(self.terminal_nodes),
            "budgets": dict(self.budgets), "workspace_constraints": [dict(item) for item in self.workspace_constraints],
            "warnings": [dict(item) for item in self.warnings], "compiled_at": self.compiled_at,
        }
        if self.goal_predicate is not None:
            value["goal_predicate"] = dict(self.goal_predicate)
        if self.goal_kind is not None:
            value["goal_kind"] = self.goal_kind
        return value

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "CompiledTaskGraph":
        allowed = {"compiled_graph_version", "graph_id", "goal_id", "target_mode", "manifest_version", "manifest_digest", "capability_index_digest", "goal_digest", "envelope_digest", "source_graph_digest", "plan_hash", "compiled_nodes", "compiled_edges", "terminal_nodes", "budgets", "workspace_constraints", "warnings", "compiled_at", "goal_predicate", "goal_kind"}
        reject_unknown(data, allowed, ("compiled_graph_version", "graph_id", "goal_id", "target_mode", "manifest_version", "manifest_digest", "capability_index_digest", "goal_digest", "envelope_digest", "source_graph_digest", "plan_hash", "compiled_nodes", "compiled_edges", "terminal_nodes", "budgets", "compiled_at"))
        edges = tuple(CompiledEdge(item["edge_id"], item["source_node_id"], item["target_node_id"], item["condition"], tuple(item.get("error_codes", ())), item.get("max_traversals"), item.get("priority", 0)) for item in data["compiled_edges"])
        return cls(
            compiled_graph_version=data["compiled_graph_version"], graph_id=data["graph_id"], goal_id=data["goal_id"], target_mode=data["target_mode"],
            manifest_version=data["manifest_version"], manifest_digest=data["manifest_digest"], capability_index_digest=data["capability_index_digest"],
            goal_digest=data["goal_digest"], envelope_digest=data["envelope_digest"], source_graph_digest=data["source_graph_digest"], plan_hash=data["plan_hash"],
            compiled_nodes=tuple(CompiledNode.from_dict(item) for item in data["compiled_nodes"]), compiled_edges=edges, terminal_nodes=tuple(data["terminal_nodes"]),
            budgets=dict(data["budgets"]), workspace_constraints=tuple(data.get("workspace_constraints", ())), warnings=tuple(data.get("warnings", ())), compiled_at=data["compiled_at"],
            goal_predicate=None if data.get("goal_predicate") is None else dict(data["goal_predicate"]), goal_kind=data.get("goal_kind"),
        )


class TaskGraphCompiler:
    """Purely static graph compiler; it never imports or calls dispatch code."""

    def __init__(self, capability_registry: CapabilityRegistry, *, manifest: Mapping[str, Any] | None = None, manifest_path: str | Path | None = None) -> None:
        self.registry = capability_registry
        self.manifest = dict(manifest) if manifest is not None else None
        self.manifest_path = Path(manifest_path).resolve() if manifest_path is not None else None

    def compile(self, goal: GoalSpec | Mapping[str, Any], envelope: ExecutionEnvelope | Mapping[str, Any], graph: TaskGraph | Mapping[str, Any], *, compilation_context: TaskCompilationContext | None = None) -> CompilationReport:
        issues: list[CompilationIssue] = []
        raw_goal = goal.to_dict() if isinstance(goal, GoalSpec) else goal
        raw_envelope = envelope.to_dict() if isinstance(envelope, ExecutionEnvelope) else envelope
        raw_graph = graph.to_dict() if isinstance(graph, TaskGraph) else graph
        for label, value in (("goal", raw_goal), ("envelope", raw_envelope), ("graph", raw_graph)):
            try:
                reject_execution_fields(value, label)
            except Exception as exc:
                self._add(issues, "FORBIDDEN_EXECUTION_FIELD", f"$.{label}", str(exc))
        goal_model = self._model(raw_goal, GoalSpec, "INVALID_GOAL_SPEC", "goal", issues)
        envelope_model = self._model(raw_envelope, ExecutionEnvelope, "INVALID_EXECUTION_ENVELOPE", "envelope", issues)
        graph_model = self._model(raw_graph, TaskGraph, "INVALID_TASK_GRAPH", "graph", issues)
        if goal_model is None or envelope_model is None or graph_model is None:
            return CompilationReport(None, tuple(issues))
        self._schema_validate(goal_model.to_dict(), "schemas/planning/goal_spec.schema.json", "INVALID_GOAL_SPEC", issues)
        self._schema_validate(envelope_model.to_dict(), "schemas/planning/execution_envelope.schema.json", "INVALID_EXECUTION_ENVELOPE", issues)
        self._schema_validate(graph_model.to_dict(), "schemas/planning/task_graph.schema.json", "INVALID_TASK_GRAPH", issues)
        self._validate_ambiguities(goal_model, issues)
        self._validate_ids_and_reachability(goal_model, graph_model, issues)
        self._validate_bindings(graph_model, issues)
        self._validate_capabilities(goal_model, envelope_model, graph_model, issues, compilation_context)
        self._validate_resources(envelope_model, graph_model, issues)
        self._validate_budgets(envelope_model, graph_model, issues)
        self._validate_cycles(envelope_model, graph_model, issues)
        self._validate_estop(graph_model, issues)
        self._validate_observability(goal_model, graph_model, issues)
        errors = [item for item in issues if item.is_error]
        if errors:
            return CompilationReport(None, tuple(issues))
        compiled = self._compile_graph(goal_model, envelope_model, graph_model, issues)
        return CompilationReport(compiled, tuple(issues))

    def _model(self, value: Any, model_type: type[Any], code: str, path: str, issues: list[CompilationIssue]) -> Any | None:
        if not isinstance(value, Mapping):
            self._add(issues, code, f"$.{path}", f"{path} must be an object")
            return None
        try:
            return model_type.from_dict(value)
        except Exception as exc:
            self._add(issues, code, f"$.{path}", str(exc))
            return None

    def _schema_validate(self, value: Mapping[str, Any], ref: str, code: str, issues: list[CompilationIssue]) -> None:
        try:
            errors = validate_json(value, ref, self.registry.repo_root)
        except Exception as exc:
            self._add(issues, code, "$", f"schema validation unavailable: {exc}")
            return
        for error in errors[:32]:
            self._add(issues, code, "$" + "".join(f"[{item!r}]" for item in error.get("path", ())), error.get("message", "schema validation failed"))

    @staticmethod
    def _add(issues: list[CompilationIssue], code: str, path: str, message: str, *, node_id: str | None = None, capability_id: str | None = None, severity: IssueSeverity = IssueSeverity.ERROR, details: Mapping[str, Any] | None = None) -> None:
        issues.append(CompilationIssue(code, severity, path, message, node_id, capability_id, dict(details or {})))

    def _validate_ambiguities(self, goal: GoalSpec, issues: list[CompilationIssue]) -> None:
        for item in goal.ambiguities:
            if item.blocking and not item.selected_resolution:
                self._add(issues, "BLOCKING_AMBIGUITY", f"$.ambiguities[{item.ambiguity_id}]", "blocking ambiguity has no selected_resolution")
            elif not item.blocking and not item.selected_resolution:
                self._add(issues, "NON_BLOCKING_AMBIGUITY_RESOLVED_CONSERVATIVELY", f"$.ambiguities[{item.ambiguity_id}]", "non-blocking ambiguity uses conservative unresolved resolution", severity=IssueSeverity.WARNING)

    def _validate_ids_and_reachability(self, goal: GoalSpec, graph: TaskGraph, issues: list[CompilationIssue]) -> None:
        if graph.goal_id != goal.goal_id:
            self._add(issues, "INVALID_TASK_GRAPH", "$.goal_id", "graph.goal_id must match goal.goal_id")
        node_ids = {item.node_id for item in graph.nodes}
        if graph.entry_node_id not in node_ids:
            self._add(issues, "DANGLING_NODE_REFERENCE", "$.entry_node_id", "entry node does not exist")
        for terminal in graph.terminal_nodes:
            if terminal not in node_ids:
                self._add(issues, "DANGLING_NODE_REFERENCE", "$.terminal_nodes", f"terminal node does not exist: {terminal}")
        outgoing = defaultdict(list)
        for edge in graph.edges:
            if edge.source_node_id not in node_ids or edge.target_node_id not in node_ids:
                self._add(issues, "DANGLING_NODE_REFERENCE", f"$.edges[{edge.edge_id}]", "edge references a missing node")
            outgoing[edge.source_node_id].append(edge)
        reachable = self._reachable(graph.entry_node_id, outgoing)
        for node in graph.nodes:
            if node.node_id not in reachable:
                self._add(issues, "UNREACHABLE_NODE", f"$.nodes[{node.node_id}]", "node is not reachable from entry_node_id", node_id=node.node_id)
        if graph.terminal_nodes and not any(item in reachable for item in graph.terminal_nodes):
            self._add(issues, "NO_TERMINAL_PATH", "$.terminal_nodes", "no terminal node is reachable")
        if not graph.terminal_nodes:
            self._add(issues, "NO_TERMINAL_PATH", "$.terminal_nodes", "task graph must declare at least one terminal node")
        for source, edges in outgoing.items():
            ordered = sorted(edges, key=lambda item: (item.priority, item.edge_id))
            for index, edge in enumerate(ordered):
                if index and edge.condition == ordered[index - 1].condition and edge.priority == ordered[index - 1].priority:
                    self._add(issues, "NON_DETERMINISTIC_GRAPH", f"$.edges[{edge.edge_id}]", "same source condition has tied priority")

    @staticmethod
    def _reachable(entry: str, outgoing: Mapping[str, Iterable[GraphEdge]]) -> set[str]:
        seen: set[str] = set()
        queue = deque([entry])
        while queue:
            current = queue.popleft()
            if current in seen:
                continue
            seen.add(current)
            queue.extend(item.target_node_id for item in outgoing.get(current, ()))
        return seen

    def _validate_bindings(self, graph: TaskGraph, issues: list[CompilationIssue]) -> None:
        node_ids = {item.node_id for item in graph.nodes}
        positions = {item.node_id: index for index, item in enumerate(graph.nodes)}
        for node in graph.nodes:
            seen: set[str] = set()
            for binding in node.input_bindings:
                if binding.binding_id in seen:
                    self._add(issues, "INVALID_TASK_GRAPH", f"$.nodes[{node.node_id}].input_bindings", "duplicate binding ID", node_id=node.node_id)
                seen.add(binding.binding_id)
                if binding.source_type == BindingSource.NODE_OUTPUT:
                    if binding.source_node_id not in node_ids:
                        self._add(issues, "DANGLING_OUTPUT_BINDING", f"$.nodes[{node.node_id}].input_bindings[{binding.binding_id}]", "source node does not exist", node_id=node.node_id)
                    elif positions[binding.source_node_id] >= positions[node.node_id]:
                        self._add(issues, "DANGLING_OUTPUT_BINDING", f"$.nodes[{node.node_id}].input_bindings[{binding.binding_id}]", "source node must occur before target node", node_id=node.node_id)
                if binding.source_type == BindingSource.WORLD_FACT and not binding.world_fact_selector:
                    self._add(issues, "DANGLING_OUTPUT_BINDING", f"$.nodes[{node.node_id}].input_bindings[{binding.binding_id}]", "world fact selector is empty", node_id=node.node_id)
                if binding.target_argument_path and not binding.target_argument_path.startswith("/"):
                    self._add(issues, "INVALID_TASK_GRAPH", f"$.nodes[{node.node_id}].input_bindings[{binding.binding_id}]", "target argument path is not a JSON Pointer", node_id=node.node_id)

    def _validate_capabilities(self, goal: GoalSpec, envelope: ExecutionEnvelope, graph: TaskGraph, issues: list[CompilationIssue], compilation_context: TaskCompilationContext | None = None) -> None:
        allowed = set(envelope.allowed_capabilities) or {item.capability_id for item in self.registry.list()}
        forbidden = set(envelope.forbidden_capabilities)
        for node in graph.nodes:
            if node.kind in {NodeKind.CHECK, NodeKind.APPROVAL, NodeKind.HUMAN_ACTION} and node.capability_id is not None:
                self._add(issues, "NODE_KIND_MISMATCH", f"$.nodes[{node.node_id}]", f"{node.kind.value} cannot reference a capability", node_id=node.node_id)
            if node.capability_id is None:
                if node.kind in {NodeKind.OBSERVE, NodeKind.ACT, NodeKind.RECOVER}:
                    self._add(issues, "UNKNOWN_CAPABILITY", f"$.nodes[{node.node_id}].capability_id", "node kind requires a capability", node_id=node.node_id)
                continue
            capability = self.registry.get(node.capability_id)
            if capability is None:
                self._add(issues, "UNKNOWN_CAPABILITY", f"$.nodes[{node.node_id}].capability_id", "unknown capability", node_id=node.node_id, capability_id=node.capability_id)
                continue
            trusted_internal = capability.visibility == "internal" and compilation_context is not None and compilation_context.is_trusted and capability.capability_id in compilation_context.internal_capability_allowlist
            if capability.visibility != "public" and not trusted_internal:
                self._add(issues, "CAPABILITY_NOT_PUBLIC", f"$.nodes[{node.node_id}].capability_id", "internal and legacy capabilities require trusted TaskCompilationContext", node_id=node.node_id, capability_id=node.capability_id)
            if capability.capability_id in forbidden:
                self._add(issues, "CAPABILITY_FORBIDDEN", f"$.nodes[{node.node_id}].capability_id", "capability is explicitly forbidden", node_id=node.node_id, capability_id=node.capability_id)
            elif capability.capability_id not in allowed and not trusted_internal:
                self._add(issues, "CAPABILITY_NOT_ALLOWED", f"$.nodes[{node.node_id}].capability_id", "capability is outside the envelope allowlist", node_id=node.node_id, capability_id=node.capability_id)
            if capability.dispatch_support == "unsupported":
                self._add(issues, "CAPABILITY_UNSUPPORTED", f"$.nodes[{node.node_id}].capability_id", "capability has no safe first-party adapter", node_id=node.node_id, capability_id=node.capability_id)
            mode_status = capability.mode_support.get(envelope.target_mode.value, "unsupported")
            if envelope.target_mode == TargetMode.LIVE and mode_status != "supported":
                self._add(issues, "CAPABILITY_PLAN_ONLY_FOR_LIVE", f"$.nodes[{node.node_id}].capability_id", "live compilation is disabled or not validated for this capability", node_id=node.node_id, capability_id=node.capability_id)
            elif envelope.target_mode == TargetMode.FROM_ARTIFACTS and mode_status != "supported":
                self._add(issues, "CAPABILITY_UNSUPPORTED", f"$.nodes[{node.node_id}].capability_id", "capability does not support from_artifacts", node_id=node.node_id, capability_id=node.capability_id)
            if RISK_ORDER[capability.risk_class] > RISK_ORDER[envelope.risk_ceiling]:
                self._add(issues, "RISK_CEILING_EXCEEDED", f"$.nodes[{node.node_id}].capability_id", "capability risk exceeds envelope ceiling", node_id=node.node_id, capability_id=node.capability_id)
            self._validate_kind(node, capability, issues)
            self._validate_arguments(node, capability, issues)
            self._validate_workspace(node, capability, envelope, issues)
            if node.timeout_s is not None and node.timeout_s > capability.timeout_s:
                self._add(issues, "TIMEOUT_EXCEEDS_CAPABILITY", f"$.nodes[{node.node_id}].timeout_s", "node timeout exceeds capability contract", node_id=node.node_id, capability_id=node.capability_id)
            if node.retry_policy.max_attempts > envelope.max_same_error_retries + 1:
                self._add(issues, "RETRY_EXCEEDS_ENVELOPE", f"$.nodes[{node.node_id}].retry_policy.max_attempts", "retry attempts exceed envelope budget", node_id=node.node_id, capability_id=node.capability_id)
            if node.kind in {NodeKind.ACT, NodeKind.RECOVER} and self._physical(capability):
                if node.verifier is None:
                    self._add(issues, "MISSING_VERIFIER", f"$.nodes[{node.node_id}].verifier", "physical action must declare a verifier", node_id=node.node_id, capability_id=node.capability_id)
                elif self._weak_verifier(node.verifier):
                    self._add(issues, "VERIFIER_WEAKENS_CONTRACT", f"$.nodes[{node.node_id}].verifier", "returncode or output schema cannot prove a physical effect", node_id=node.node_id, capability_id=node.capability_id)
                if node.verifier is not None and capability.verifier.required_for_physical_success and capability.verifier.physical_verification_limited and node.verifier.get("mode") == "complete_physical":
                    self._add(issues, "VERIFIER_WEAKENS_CONTRACT", f"$.nodes[{node.node_id}].verifier", "graph cannot upgrade a limited capability verifier", node_id=node.node_id, capability_id=node.capability_id)
        if goal.goal_kind == GoalKind.PHYSICAL_STATE_CHANGE and not goal.required_evidence:
            self._add(issues, "PHYSICAL_GOAL_WITHOUT_EVIDENCE", "$.required_evidence", "physical goal requires evidence")

    @staticmethod
    def _physical(capability: CapabilityContract) -> bool:
        return bool(capability.physical_side_effects or capability.moves_robot or capability.controls_gripper)

    def _validate_kind(self, node: GraphNode, capability: CapabilityContract, issues: list[CompilationIssue]) -> None:
        kind = capability.kind
        valid = True
        if node.kind == NodeKind.OBSERVE:
            valid = kind == CapabilityKind.OBSERVATION
        elif node.kind == NodeKind.COMPUTE:
            valid = kind == CapabilityKind.COMPUTE and not capability.requires_hardware
        elif node.kind == NodeKind.ACT:
            valid = kind in {CapabilityKind.ACTION, CapabilityKind.PROCEDURE, CapabilityKind.TASK}
        elif node.kind == NodeKind.RECOVER:
            valid = kind == CapabilityKind.RECOVERY and capability.allowed_as_recovery
            if not capability.allowed_as_recovery:
                self._add(issues, "RECOVERY_NOT_ALLOWED", f"$.nodes[{node.node_id}].capability_id", "capability is not allowed as recovery", node_id=node.node_id, capability_id=capability.capability_id)
        elif node.kind == NodeKind.VERIFY:
            valid = kind in {CapabilityKind.OBSERVATION, CapabilityKind.COMPUTE}
        if not valid:
            self._add(issues, "NODE_KIND_MISMATCH", f"$.nodes[{node.node_id}].kind", f"{node.kind.value} is incompatible with {kind.value}", node_id=node.node_id, capability_id=capability.capability_id)

    def _validate_arguments(self, node: GraphNode, capability: CapabilityContract, issues: list[CompilationIssue]) -> None:
        arguments = dict(node.arguments)
        dynamic: set[str] = set()
        for binding in node.input_bindings:
            if binding.target_argument_path.startswith("/") and binding.target_argument_path.count("/") == 1:
                key = binding.target_argument_path[1:]
                if binding.source_type == BindingSource.LITERAL:
                    arguments[key] = binding.literal
                else:
                    dynamic.add(key)
        try:
            schema = self.registry.resolve_input_schema(capability.capability_id)
            candidate = dict(arguments)
            for key in dynamic:
                if key not in candidate:
                    candidate[key] = self._schema_placeholder(schema.get("properties", {}).get(key, {}))
            errors = validate_json(candidate, capability.input_schema_ref, self.registry.repo_root)
        except Exception as exc:
            self._add(issues, "ARGUMENT_SCHEMA_INVALID", f"$.nodes[{node.node_id}].arguments", str(exc), node_id=node.node_id, capability_id=capability.capability_id)
            return
        for error in errors:
            path = "$.nodes[%s].arguments%s" % (node.node_id, "".join(f"[{item!r}]" for item in error.get("path", ())))
            self._add(issues, "ARGUMENT_SCHEMA_INVALID", path, error.get("message", "invalid capability arguments"), node_id=node.node_id, capability_id=capability.capability_id)

    @staticmethod
    def _schema_placeholder(schema: Mapping[str, Any]) -> Any:
        if "enum" in schema and schema["enum"]:
            return schema["enum"][0]
        kind = schema.get("type")
        if isinstance(kind, list):
            kind = next((item for item in kind if item != "null"), "string")
        if kind == "array":
            count = int(schema.get("minItems", 0))
            return [TaskGraphCompiler._schema_placeholder(schema.get("items", {})) for _ in range(count)]
        if kind == "object":
            result = {}
            for key in schema.get("required", ()):
                result[key] = TaskGraphCompiler._schema_placeholder(schema.get("properties", {}).get(key, {}))
            return result
        if kind == "number" or kind == "integer":
            return 0
        if kind == "boolean":
            return False
        return "dynamic"

    def _validate_workspace(self, node: GraphNode, capability: CapabilityContract, envelope: ExecutionEnvelope, issues: list[CompilationIssue]) -> None:
        if "xyz_m" not in node.arguments and not any(item.target_argument_path.endswith("/xyz_m") for item in node.input_bindings):
            if capability.capability_id == "motion.move_to_pose":
                self._add(issues, "FRAME_REQUIRED", f"$.nodes[{node.node_id}].arguments.frame", "motion pose must declare a frame", node_id=node.node_id, capability_id=capability.capability_id)
            return
        frame = node.arguments.get("frame")
        if not isinstance(frame, str) or not frame:
            if not any(item.target_argument_path.endswith("/frame") for item in node.input_bindings):
                self._add(issues, "FRAME_REQUIRED", f"$.nodes[{node.node_id}].arguments.frame", "pose frame is required", node_id=node.node_id, capability_id=capability.capability_id)
        dynamic_pose = any(item.target_argument_path.endswith("/xyz_m") for item in node.input_bindings) and any(item.source_type != BindingSource.LITERAL for item in node.input_bindings if item.target_argument_path.endswith("/xyz_m"))
        if dynamic_pose:
            guards = [item for item in node.input_bindings if item.target_argument_path.endswith("/xyz_m")]
            guarded = any(item.constraints.get("workspace_id") and item.constraints.get("runtime_check") is True for item in guards)
            if not guarded or not node.preconditions:
                self._add(issues, "DYNAMIC_POSE_WITHOUT_WORKSPACE_GUARD", f"$.nodes[{node.node_id}].input_bindings", "dynamic pose requires workspace_id, runtime_check, and a precondition", node_id=node.node_id, capability_id=capability.capability_id)
            return
        xyz = node.arguments.get("xyz_m")
        if not isinstance(xyz, list) or len(xyz) != 3 or any(isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(float(value)) for value in xyz):
            return
        matches = [item for item in envelope.workspace_constraints if item.frame == frame]
        if not matches:
            self._add(issues, "WORKSPACE_VIOLATION", f"$.nodes[{node.node_id}].arguments", "literal pose has no matching envelope workspace", node_id=node.node_id, capability_id=capability.capability_id)
            return
        if not any(all(lower <= float(value) <= upper for value, lower, upper in zip(xyz, item.min_xyz_m, item.max_xyz_m)) for item in matches):
            self._add(issues, "WORKSPACE_VIOLATION", f"$.nodes[{node.node_id}].arguments.xyz_m", "literal pose is outside every matching workspace", node_id=node.node_id, capability_id=capability.capability_id)

    def _validate_resources(self, envelope: ExecutionEnvelope, graph: TaskGraph, issues: list[CompilationIssue]) -> None:
        for node in graph.nodes:
            if node.capability_id is None:
                continue
            capability = self.registry.get(node.capability_id)
            if capability is None:
                continue
            declared = {item.resource_id: item for item in node.resource_requirements}
            for required in capability.resources:
                item = declared.get(required.resource_id)
                if item is None or (required.mode == ResourceMode.EXCLUSIVE and item.mode != ResourceMode.EXCLUSIVE):
                    self._add(issues, "RESOURCE_REQUIREMENT_MISSING", f"$.nodes[{node.node_id}].resource_requirements", f"capability resource is missing or weakened: {required.resource_id}", node_id=node.node_id, capability_id=node.capability_id)
            if envelope.allowed_resources and any(item.resource_id not in envelope.allowed_resources for item in capability.resources):
                self._add(issues, "RESOURCE_CONFLICT", f"$.nodes[{node.node_id}].resource_requirements", "capability requires a resource outside the envelope", node_id=node.node_id, capability_id=node.capability_id)
        groups: dict[str, list[GraphNode]] = defaultdict(list)
        for node in graph.nodes:
            group = node.metadata.get("parallel_group")
            if group:
                groups[str(group)].append(node)
        for group, nodes in groups.items():
            for index, left in enumerate(nodes):
                left_resources = {item.resource_id: item.mode for item in left.resource_requirements}
                for right in nodes[index + 1:]:
                    right_resources = {item.resource_id: item.mode for item in right.resource_requirements}
                    for resource in set(left_resources) & set(right_resources):
                        if ResourceMode.EXCLUSIVE in {left_resources[resource], right_resources[resource]}:
                            self._add(issues, "RESOURCE_CONFLICT", f"$.metadata.parallel_group[{group}]", f"parallel nodes conflict on exclusive resource {resource}", node_id=right.node_id)

    def _validate_budgets(self, envelope: ExecutionEnvelope, graph: TaskGraph, issues: list[CompilationIssue]) -> None:
        if len(graph.nodes) > envelope.max_nodes:
            self._add(issues, "GRAPH_NODE_BUDGET_EXCEEDED", "$.nodes", "graph exceeds max_nodes")
        tool_calls = sum(1 for item in graph.nodes if item.capability_id is not None)
        if tool_calls > envelope.max_tool_calls:
            self._add(issues, "TOOL_CALL_BUDGET_EXCEEDED", "$.nodes", "graph exceeds max_tool_calls")
        recoveries = sum(item.kind == NodeKind.RECOVER for item in graph.nodes)
        if recoveries > envelope.max_recovery_actions:
            self._add(issues, "RECOVERY_BUDGET_EXCEEDED", "$.nodes", "graph exceeds max_recovery_actions")
        depth = self._max_depth(graph)
        if depth > envelope.max_depth:
            self._add(issues, "GRAPH_DEPTH_BUDGET_EXCEEDED", "$.nodes", "graph depth exceeds max_depth")

    @staticmethod
    def _max_depth(graph: TaskGraph) -> int:
        outgoing = defaultdict(list)
        for edge in graph.edges:
            outgoing[edge.source_node_id].append(edge.target_node_id)
        best = 0
        queue = deque([(graph.entry_node_id, 1, frozenset())])
        while queue:
            node, depth, path = queue.popleft()
            best = max(best, depth)
            if node in path:
                continue
            for child in outgoing.get(node, ()):
                queue.append((child, depth + 1, path | {node}))
        return best

    def _validate_cycles(self, envelope: ExecutionEnvelope, graph: TaskGraph, issues: list[CompilationIssue]) -> None:
        outgoing = defaultdict(list)
        for edge in graph.edges:
            outgoing[edge.source_node_id].append(edge)
        nodes = {item.node_id: item for item in graph.nodes}
        stack: list[str] = []
        seen: set[tuple[str, tuple[str, ...]]] = set()

        def visit(node_id: str) -> None:
            if node_id in stack:
                cycle = stack[stack.index(node_id):]
                cycle_set = set(cycle)
                for source in cycle:
                    for edge in outgoing.get(source, ()):
                        if edge.target_node_id in cycle_set and edge.max_traversals is None:
                            self._add(issues, "UNBOUNDED_CYCLE", f"$.edges[{edge.edge_id}]", "cycle edge requires finite max_traversals", node_id=source)
                if not any(edge.target_node_id not in cycle_set for source in cycle for edge in outgoing.get(source, ())):
                    self._add(issues, "NO_TERMINAL_PATH", f"$.nodes[{node_id}]", "cycle has no exit path", node_id=node_id)
                return
            if len(stack) > envelope.max_depth + 1:
                return
            stack.append(node_id)
            for edge in outgoing.get(node_id, ()):
                visit(edge.target_node_id)
            stack.pop()

        visit(graph.entry_node_id)
        del seen, nodes

    def _validate_estop(self, graph: TaskGraph, issues: list[CompilationIssue]) -> None:
        nodes = {item.node_id: item for item in graph.nodes}
        outgoing = defaultdict(list)
        for edge in graph.edges:
            outgoing[edge.source_node_id].append(edge)
        for edge in graph.edges:
            if ErrorCode.ROBOT_ESTOP_OR_UNSAFE.value not in edge.error_codes:
                continue
            target = nodes.get(edge.target_node_id)
            if target is None or target.kind == NodeKind.RECOVER:
                self._add(issues, "ESTOP_AUTO_RECOVERY_FORBIDDEN", f"$.edges[{edge.edge_id}]", "E-stop cannot route to automatic recovery", node_id=edge.source_node_id)
            elif target.kind not in {NodeKind.HUMAN_ACTION, NodeKind.APPROVAL} and edge.target_node_id not in graph.terminal_nodes:
                self._add(issues, "ESTOP_AUTO_RECOVERY_FORBIDDEN", f"$.edges[{edge.edge_id}]", "E-stop must route to human action or a human/unsafe terminal", node_id=edge.source_node_id)

    def _validate_observability(self, goal: GoalSpec, graph: TaskGraph, issues: list[CompilationIssue]) -> None:
        predicate = goal.success_predicate.to_dict()
        if self._contains_returncode(predicate):
            self._add(issues, "SUCCESS_PREDICATE_NOT_OBSERVABLE", "$.success_predicate", "success predicate cannot be based on action returncode")
        for evidence in goal.required_evidence:
            if evidence.required_status in {"TENTATIVE", "INVALIDATED"}:
                self._add(issues, "SUCCESS_PREDICATE_NOT_OBSERVABLE", f"$.required_evidence[{evidence.evidence_id}]", "required evidence cannot be tentative or invalidated")
        if goal.goal_kind == GoalKind.PHYSICAL_STATE_CHANGE:
            if not any(item.kind == NodeKind.VERIFY for item in graph.nodes):
                self._add(issues, "SUCCESS_PREDICATE_NOT_OBSERVABLE", "$.nodes", "physical goal requires a VERIFY node")
            outgoing = defaultdict(list)
            for edge in graph.edges:
                outgoing[edge.source_node_id].append(edge)
            verify_ids = {item.node_id for item in graph.nodes if item.kind == NodeKind.VERIFY}
            for node in graph.nodes:
                capability = self.registry.get(node.capability_id) if node.capability_id else None
                if node.kind == NodeKind.ACT and capability is not None and self._physical(capability):
                    if not self._can_reach_any(node.node_id, verify_ids, outgoing):
                        self._add(issues, "MISSING_VERIFIER", f"$.nodes[{node.node_id}]", "physical action has no VERIFY path", node_id=node.node_id)

    @staticmethod
    def _contains_returncode(value: Any) -> bool:
        if isinstance(value, dict):
            return any(str(key).lower() in {"returncode", "command_returncode", "action_result", "command_result"} or TaskGraphCompiler._contains_returncode(item) for key, item in value.items())
        if isinstance(value, list):
            return any(TaskGraphCompiler._contains_returncode(item) for item in value)
        return False

    @staticmethod
    def _weak_verifier(verifier: Mapping[str, Any]) -> bool:
        value = stable_dumps(verifier).lower()
        return any(token in value for token in ("returncode", "action_result", "command_result", "output_schema_only"))

    @staticmethod
    def _can_reach_any(start: str, targets: set[str], outgoing: Mapping[str, Iterable[GraphEdge]]) -> bool:
        queue = deque([start])
        seen: set[str] = set()
        while queue:
            current = queue.popleft()
            if current in targets:
                return True
            if current in seen:
                continue
            seen.add(current)
            queue.extend(edge.target_node_id for edge in outgoing.get(current, ()))
        return False

    def _compile_graph(self, goal: GoalSpec, envelope: ExecutionEnvelope, graph: TaskGraph, issues: list[CompilationIssue]) -> CompiledTaskGraph:
        compiled_nodes: list[CompiledNode] = []
        capability_index = []
        for capability in self.registry.list(include_internal=True, include_legacy=True):
            capability_index.append({"capability_id": capability.capability_id, "capability_version": capability.capability_version, "input_schema_ref": capability.input_schema_ref, "output_schema_ref": capability.output_schema_ref})
        for node in graph.nodes:
            capability = self.registry.get(node.capability_id) if node.capability_id else None
            compiled_nodes.append(CompiledNode(
                node_id=node.node_id, kind=node.kind.value, capability_id=node.capability_id,
                capability_version=capability.capability_version if capability else None,
                dispatch_status=capability.dispatch_support if capability else "none",
                execution_disposition="PLAN_ONLY" if capability and capability.dispatch_support == "plan_only" else "STATIC" if capability is None else "SUPPORTED",
                risk_class=capability.risk_class.value if capability else None,
                resources=tuple(item.to_dict() for item in (node.resource_requirements or (capability.resources if capability else ()))),
                arguments=dict(node.arguments), input_bindings=tuple(item.to_dict() for item in node.input_bindings),
                preconditions=tuple(item.to_dict() for item in node.preconditions), expected_effects=tuple(item.to_dict() for item in node.expected_effects),
                verifier_contract=None if capability is None else {"capability": capability.verifier.to_dict(), "graph": None if node.verifier is None else dict(node.verifier)},
                timeout_s=node.timeout_s if node.timeout_s is not None else capability.timeout_s if capability else None,
                retry_policy=node.retry_policy.to_dict(), max_visits=node.max_visits,
                input_schema_digest=digest(self.registry.resolve_input_schema(capability.capability_id)) if capability else None,
                output_schema_digest=digest(self.registry.resolve_output_schema(capability.capability_id)) if capability else None,
                metadata=dict(node.metadata), fact_projections=tuple(node.metadata.get("fact_projections", ())),
            ))
        compiled_edges = tuple(CompiledEdge(item.edge_id, item.source_node_id, item.target_node_id, item.condition.value, item.error_codes, item.max_traversals, item.priority) for item in graph.edges)
        manifest = self.manifest or {"version": "0.2.x", "capabilities": capability_index}
        manifest_version = str(manifest.get("version", "0.2.x"))
        goal_digest = digest(goal.to_dict())
        envelope_digest = digest(envelope.to_dict())
        source_digest = digest(graph.to_dict())
        base = CompiledTaskGraph(
            compiled_graph_version="1.0.0", graph_id=graph.graph_id, goal_id=goal.goal_id, target_mode=envelope.target_mode.value,
            manifest_version=manifest_version, manifest_digest=digest(manifest), capability_index_digest=digest(capability_index),
            goal_digest=goal_digest, envelope_digest=envelope_digest, source_graph_digest=source_digest, plan_hash="",
            compiled_nodes=tuple(compiled_nodes), compiled_edges=compiled_edges, terminal_nodes=graph.terminal_nodes,
            budgets={"max_nodes": envelope.max_nodes, "max_depth": envelope.max_depth, "max_elapsed_s": envelope.max_elapsed_s, "max_tool_calls": envelope.max_tool_calls, "max_replans": envelope.max_replans, "max_recovery_actions": envelope.max_recovery_actions, "max_same_error_retries": envelope.max_same_error_retries, "no_progress_limit": envelope.no_progress_limit},
            workspace_constraints=tuple(item.to_dict() for item in envelope.workspace_constraints),
            warnings=tuple(item.to_dict() for item in issues if item.severity == IssueSeverity.WARNING), compiled_at=utc_now_iso(),
            goal_predicate=goal.success_predicate.to_dict(), goal_kind=goal.goal_kind.value,
        )
        return CompiledTaskGraph(**{**base.__dict__, "plan_hash": digest(base._hash_payload())})
