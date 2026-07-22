from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, replace
from pathlib import Path
import time
import uuid
from typing import Any, Mapping

from ..contracts.enums import ActionStatus, ErrorCode, ErrorSeverity, NodeStatus, TaskTerminalState
from ..contracts.errors import ErrorInfo
from ..contracts.results import ActionResult, ObservationResult, VerificationResult
from ..contracts.resources import ResourceRequirement
from ..contracts.serialization import stable_dumps, utc_now_iso
from ..dispatch import CapabilityDispatcher, DispatchContext, DispatchRequest, NoExecutionBackend
from ..dispatch.error_mapping import make_error
from ..planning.canonical import digest
from ..planning.compiler import CompiledEdge, CompiledNode, CompiledTaskGraph
from ..verification.engine import VerifierEngine
from ..world.invalidation import InvalidationEngine
from ..world.models import EntityRef, FactStatus, WorldFact
from ..world.predicates import PredicateEngine, PredicateResult, PredicateSpec
from ..world.snapshot import WorldSnapshot
from ..world.store import WorldStateStore
from ..types import SkillMode
from .bindings import RuntimeBindingResolver, json_pointer_get
from .budgets import BudgetTracker
from .cancellation import CancellationToken
from .checkpoint import CheckpointStore
from .errors import ExecutionError
from .events import ArtifactStore, EventStore, EventType
from .models import BudgetUsage, ExecutionOutcome, ExecutionScope, NodeAttempt, NodeRuntimeRecord, TaskExecutionResult
from .preflight import ExecutionPreflightValidator
from .progress import ProgressDetector
from .resources import ResourceLockManager
from .state_machine import NodeStateMachine


@dataclass
class _NodeRun:
    error: ErrorInfo | None = None
    predicate: PredicateResult | None = None
    verification: VerificationResult | None = None
    progressed: bool = False
    planned_only: bool = False
    output: Any = None


class GraphExecutor:
    """Deterministic, single-active-node executor for CompiledTaskGraph only."""

    def __init__(
        self,
        compiled_graph: CompiledTaskGraph,
        *,
        registry: Any,
        manifest: Mapping[str, Any],
        dispatcher: CapabilityDispatcher | Any | None = None,
        mode: str | None = None,
        artifact_dir: str | Path = "/tmp/agentic_skills_runs",
        world_state: WorldStateStore | None = None,
        predicate_engine: PredicateEngine | None = None,
        verifier_engine: VerifierEngine | None = None,
        invalidation_engine: InvalidationEngine | None = None,
        cancellation: CancellationToken | None = None,
        monotonic_clock: Any | None = None,
        utc_clock: Any | None = None,
        from_artifacts_root: str | Path | None = None,
        run_id: str | None = None,
        resume: bool = False,
    ) -> None:
        if not isinstance(compiled_graph, CompiledTaskGraph):
            raise TypeError("GraphExecutor accepts only CompiledTaskGraph")
        self.graph = compiled_graph
        self.registry = registry
        self.manifest = dict(manifest)
        self.mode = mode or compiled_graph.target_mode
        self.artifacts = ArtifactStore(artifact_dir)
        self.checkpoints = CheckpointStore(self.artifacts.root)
        self.utc_clock = utc_clock or utc_now_iso
        self.monotonic_clock = monotonic_clock or time.monotonic
        checkpoint_run_id = None
        if resume and self.checkpoints.path.exists():
            try:
                checkpoint_run_id = self.checkpoints.load().get("run_id")
            except Exception:
                checkpoint_run_id = None
        self.run_id = run_id or checkpoint_run_id or f"run_{uuid.uuid4().hex}"
        self.event_store = EventStore(self.artifacts.root, run_id=self.run_id, graph_id=self.graph.graph_id, clock=self.utc_clock)
        self.world = world_state or WorldStateStore()
        self.predicates = predicate_engine or PredicateEngine()
        self.invalidation = invalidation_engine or InvalidationEngine(self.world)
        self.dispatcher = dispatcher or CapabilityDispatcher(self.registry, backend=NoExecutionBackend(), world_state=self.world)
        self.verifier = verifier_engine or VerifierEngine(self.registry, dispatcher=self.dispatcher, predicate_engine=self.predicates)
        self.cancel_token = cancellation or CancellationToken()
        self.resource_locks = ResourceLockManager()
        self.budget = BudgetTracker(self.graph.budgets, monotonic_clock=self.monotonic_clock)
        self.bindings = RuntimeBindingResolver(registry=self.registry)
        self.progress = ProgressDetector(int(self.graph.budgets.get("no_progress_limit", 3)))
        self.from_artifacts_root = str(from_artifacts_root) if from_artifacts_root is not None else None
        self.node_records = {node.node_id: NodeRuntimeRecord(node.node_id, node.kind, resource_requirements=node.resources) for node in self.graph.compiled_nodes}
        self.outputs: dict[str, Any] = {}
        self.results: dict[str, Any] = {}
        self.verifications: dict[str, VerificationResult] = {}
        self.frontier: str | None = None
        self.started_monotonic: float | None = None
        self.started_at: str = self.utc_clock() if callable(self.utc_clock) else utc_now_iso()
        self.ended_at: str = self.started_at
        self._terminal_state: str = TaskTerminalState.FAILED.value
        self._outcome: ExecutionOutcome = ExecutionOutcome.FAILED
        self._goal_verification: VerificationResult | None = None
        self._warnings: list[str] = []
        self._errors: list[ErrorInfo] = []
        self._world_snapshot_ref: str | None = None
        self._planned_only = self.mode == "dry_run"
        self._restored = resume
        self.artifacts.write_json("execution_context.json", {"run_id": self.run_id, "graph_id": self.graph.graph_id, "goal_id": self.graph.goal_id, "mode": self.mode, "physical_execution_performed": False, "hardware_allowed": False, "execute": False})
        self.artifacts.write_json("compiled_graph_snapshot.json", self.graph.to_dict())
        self.artifacts.write_json("manifest_snapshot.json", self.manifest)

    @property
    def node_map(self) -> dict[str, CompiledNode]:
        return {node.node_id: node for node in self.graph.compiled_nodes}

    def _now(self) -> str:
        return self.utc_clock() if callable(self.utc_clock) else utc_now_iso()

    def _append(self, event: EventType, *, node_id: str | None = None, attempt_id: str | None = None, payload: Mapping[str, Any] | None = None, causal: dict[str, Any] | None = None) -> dict[str, Any]:
        return self.event_store.append(event, node_id=node_id, attempt_id=attempt_id, payload=payload, causal_event_id=None if causal is None else causal.get("event_id"))

    def _set_state(self, record: NodeRuntimeRecord, status: NodeStatus, *, node_id: str, attempt_id: str | None = None) -> None:
        machine = NodeStateMachine(record.status)
        machine.transition(status)
        record.status = machine.status
        if status == NodeStatus.READY:
            self._append(EventType.NODE_READY, node_id=node_id, attempt_id=attempt_id)

    def _resource_requirements(self, node: CompiledNode) -> tuple[ResourceRequirement, ...]:
        return tuple(ResourceRequirement.from_dict(item) for item in node.resources)

    def _check_cancelled(self) -> None:
        if self.cancel_token.is_cancelled:
            raise ExecutionError(ErrorCode.EXECUTION_INTERRUPTED, self.cancel_token.reason or "execution cancelled", {"reason": self.cancel_token.reason})

    def _world_digest(self) -> str:
        return digest(self.world.snapshot().to_dict())

    def _checkpoint(self) -> str:
        self._append(EventType.CHECKPOINT_WRITTEN, payload={"event_sequence_before_write": self.event_store.sequence})
        world_ref = self.artifacts.write_json(f"world/snapshots/revision_{self.world.current_revision}.json", self.world.snapshot().to_dict())
        self._world_snapshot_ref = world_ref
        payload = {
            "checkpoint_version": "1.0.0", "run_id": self.run_id, "graph_id": self.graph.graph_id, "goal_id": self.graph.goal_id,
            "plan_hash": self.graph.plan_hash, "manifest_digest": self.graph.manifest_digest, "capability_index_digest": self.graph.capability_index_digest,
            "compiled_graph_version": self.graph.compiled_graph_version, "mode": self.mode, "event_sequence": self.event_store.sequence, "event_digest": self.event_store.last_digest,
            "node_records": [item.to_dict() for item in self.node_records.values()], "edge_traversal_counts": dict(self.budget.usage.edge_traversals), "node_visit_counts": dict(self.budget.usage.node_visits),
            "budget_usage": self.budget.usage.to_dict(), "world_snapshot": self.world.snapshot().to_dict(), "world_snapshot_ref": world_ref,
            "node_output_refs": {key: value for key, value in ((node_id, record.output_ref) for node_id, record in self.node_records.items()) if value},
            "dispatch_result_refs": {key: value for key, value in ((node_id, record.latest_dispatch_result_ref) for node_id, record in self.node_records.items()) if value},
            "verification_result_refs": {key: value for key, value in ((node_id, record.latest_verification_result_ref) for node_id, record in self.node_records.items()) if value},
            "progress_fingerprints": self.progress.to_dict(), "terminal_state": self._terminal_state, "frontier": self.frontier, "created_at": self._now(), "outcome": self._outcome.value,
        }
        return self.checkpoints.write(payload)

    def _restore(self) -> None:
        checkpoint = self.checkpoints.load()
        required = {"run_id", "graph_id", "goal_id", "plan_hash", "manifest_digest", "capability_index_digest", "compiled_graph_version", "mode", "event_sequence", "event_digest", "node_records", "budget_usage", "world_snapshot", "terminal_state"}
        if not required.issubset(checkpoint):
            raise ExecutionError(ErrorCode.CHECKPOINT_CORRUPT, "checkpoint is missing required fields")
        identity = (("run_id", self.run_id), ("graph_id", self.graph.graph_id), ("goal_id", self.graph.goal_id), ("plan_hash", self.graph.plan_hash), ("manifest_digest", self.graph.manifest_digest), ("capability_index_digest", self.graph.capability_index_digest), ("compiled_graph_version", self.graph.compiled_graph_version), ("mode", self.mode))
        mismatch = {key: (expected, checkpoint.get(key)) for key, expected in identity if checkpoint.get(key) != expected}
        if mismatch:
            raise ExecutionError(ErrorCode.CHECKPOINT_MISMATCH, "checkpoint identity does not match compiled graph", {"mismatch": mismatch})
        events = self.event_store.read_verify()
        if checkpoint["event_sequence"] != len(events) or checkpoint["event_digest"] != (events[-1]["event_digest"] if events else None):
            raise ExecutionError(ErrorCode.CHECKPOINT_MISMATCH, "checkpoint and event log are inconsistent")
        self.node_records = {item.node_id: item for item in (NodeRuntimeRecord.from_dict(value) for value in checkpoint["node_records"])}
        if set(self.node_records) != set(self.node_map):
            raise ExecutionError(ErrorCode.CHECKPOINT_CORRUPT, "checkpoint node set does not match graph")
        self.budget.restore(BudgetUsage.from_dict(checkpoint["budget_usage"]))
        snapshot = WorldSnapshot.from_dict(checkpoint["world_snapshot"])
        self.world.restore(snapshot)
        self.progress = ProgressDetector.from_dict(checkpoint.get("progress_fingerprints", {}), int(self.graph.budgets.get("no_progress_limit", 3)))
        self.frontier = checkpoint.get("frontier")
        self._terminal_state = checkpoint.get("terminal_state", TaskTerminalState.FAILED.value)
        self._outcome = ExecutionOutcome(checkpoint.get("outcome", ExecutionOutcome.FAILED.value))
        self.resource_locks.clear_for_resume()
        for node_id, record in self.node_records.items():
            if record.output_ref:
                self.outputs[node_id] = self.artifacts.read_json(record.output_ref)
            if record.latest_dispatch_result_ref:
                self.results[node_id] = self.artifacts.read_json(record.latest_dispatch_result_ref)
        for record in self.node_records.values():
            if record.status == NodeStatus.RUNNING:
                # A checkpoint never trusts an old lock or an unknown in-flight action.
                if record.latest_dispatch_result_ref:
                    record.status = NodeStatus.FAILED
                    record.latest_error_code = ErrorCode.EXECUTION_INTERRUPTED.value
                else:
                    record.status = NodeStatus.READY
        self._append(EventType.CHECKPOINT_RESTORED, payload={"event_sequence": checkpoint["event_sequence"]})

    def _write_attempt_json(self, node_id: str, attempt_id: str, name: str, value: Any) -> str:
        return self.artifacts.write_json(f"nodes/{node_id}/attempts/{attempt_id}/{name}.json", value)

    def _make_attempt(self, record: NodeRuntimeRecord, node: CompiledNode, arguments_digest: str) -> NodeAttempt:
        record.attempt_count += 1
        attempt = NodeAttempt(f"attempt_{uuid.uuid4().hex}", node.node_id, record.attempt_count, NodeStatus.RUNNING, arguments_digest, self._now())
        record.latest_attempt_id = attempt.attempt_id
        record.attempts.append(attempt)
        record.started_at = record.started_at or attempt.started_at
        return attempt

    def _result_output(self, result: Any) -> Any:
        if isinstance(result, ObservationResult):
            return result.observations
        if isinstance(result, ActionResult):
            return result.outputs
        return result

    def _result_errors(self, result: Any) -> tuple[ErrorInfo, ...]:
        return tuple(getattr(result, "errors", ()))

    def _project_observations(self, node: CompiledNode, result: Any, attempt: NodeAttempt) -> bool:
        projections = node.fact_projections or tuple(node.metadata.get("fact_projections", ()))
        if not projections or not isinstance(result, ObservationResult):
            return False
        source = result.observations
        changed = False
        for projection in projections:
            spec = dict(projection)
            if not spec.get("fact_id") or not spec.get("predicate") or not isinstance(spec.get("subject"), dict):
                self._warnings.append(f"ignored incomplete fact projection for {node.node_id}")
                continue
            try:
                value = json_pointer_get(source, spec.get("value_path", ""))
                status = FactStatus(str(spec.get("status", "OBSERVED")))
                if status == FactStatus.VERIFIED:
                    status = FactStatus.OBSERVED
                capability = self.registry.get(node.capability_id or "")
                fact = WorldFact(
                    fact_id=str(spec["fact_id"]), subject=EntityRef.from_dict(spec["subject"]), predicate=str(spec["predicate"]), object=deepcopy(spec.get("object", value)), value=deepcopy(value), status=status,
                    confidence=spec.get("confidence", result.artifacts.get("confidence")), frame=spec.get("frame"), observed_at=result.captured_at, valid_until=spec.get("valid_until"), source_capability_id=node.capability_id,
                    source_capability_version=None if capability is None else capability.capability_version, artifact_refs=tuple(spec.get("artifact_refs", ())), calibration_hash=spec.get("calibration_hash"), revision=0, metadata={"observation": True, "node_id": node.node_id}, freshness_ttl_s=spec.get("freshness_ttl_s", result.freshness_ttl_s),
                )
                stored = self.world.add_fact(fact)
                self._append(EventType.WORLD_STATE_UPDATED, node_id=node.node_id, attempt_id=attempt.attempt_id, payload={"fact_id": stored.fact_id, "revision": stored.revision, "status": stored.status.value})
                changed = True
            except Exception as exc:
                self._warnings.append(f"fact projection failed for {node.node_id}: {exc}")
        return changed

    def _project_tentative_effects(self, node: CompiledNode, result: ActionResult, attempt: NodeAttempt) -> bool:
        if self.mode == "dry_run" or result.planned_only or not result.command_executed:
            return False
        changed = False
        for expected in node.expected_effects:
            spec = dict(expected.get("predicate", {}))
            operands = spec.get("operands", [])
            selector = operands[0] if operands and isinstance(operands[0], dict) else {"entity_id": "unknown", "entity_type": "unknown"}
            subject_id = selector.get("entity_id", selector.get("subject_id", "unknown"))
            subject_type = selector.get("entity_type", "simulated_entity")
            fact = WorldFact(f"effect:{node.node_id}:{expected.get('effect_id', 'effect')}", EntityRef(str(subject_id), str(subject_type)), f"expected.{expected.get('effect_id', 'effect')}", spec, {"predicate": spec}, FactStatus.TENTATIVE, None, None, self._now(), None, node.capability_id, node.capability_version, (), None, 0, {"expected_effect": True, "node_id": node.node_id}, None)
            stored = self.world.add_fact(fact)
            self._append(EventType.WORLD_STATE_UPDATED, node_id=node.node_id, attempt_id=attempt.attempt_id, payload={"fact_id": stored.fact_id, "status": stored.status.value, "effect": "TENTATIVE"})
            changed = True
        return changed

    def _evaluate_preconditions(self, node: CompiledNode, *, attempt: NodeAttempt) -> tuple[bool, PredicateResult | None]:
        specs = list(node.preconditions)
        if node.kind == "CHECK" and not specs and isinstance(node.metadata.get("predicate"), dict):
            specs = [node.metadata["predicate"]]
        if not specs:
            return True, None
        results = [self.predicates.evaluate(item, self.world, now=self._now()) for item in specs]
        satisfied = all(item.satisfied for item in results)
        result = results[0] if len(results) == 1 else PredicateResult(satisfied, f"all:{node.node_id}", "all", tuple(fact for item in results for fact in item.evidence_fact_ids), "all preconditions satisfied" if satisfied else "precondition false", tuple(error for item in results for error in item.errors), self._now())
        self._append(EventType.PRECONDITION_EVALUATED, node_id=node.node_id, attempt_id=attempt.attempt_id, payload=result.to_dict())
        return satisfied, result

    def _compute(self, node: CompiledNode, arguments: dict[str, Any]) -> Any:
        operation = str(node.metadata.get("operation", arguments.get("operation", "literal")))
        if operation == "literal":
            return deepcopy(node.metadata.get("value", arguments.get("value", arguments)))
        if operation == "copy":
            return deepcopy(json_pointer_get(arguments, str(node.metadata.get("source_path", "/value"))))
        if operation == "select":
            return deepcopy(json_pointer_get(arguments, str(node.metadata.get("source_path", "/value"))))
        if operation == "merge-json-object":
            values = node.metadata.get("values", arguments.get("values", []))
            if not isinstance(values, list) or any(not isinstance(value, dict) for value in values):
                raise ExecutionError(ErrorCode.INVALID_INPUT, "merge-json-object requires an array of objects")
            merged: dict[str, Any] = {}
            for value in values:
                merged.update(deepcopy(value))
            return merged
        raise ExecutionError(ErrorCode.CAPABILITY_UNSUPPORTED, "unsupported builtin compute operation", {"operation": operation})

    def _human_request(self, node: CompiledNode, attempt: NodeAttempt) -> str:
        request = {"request_version": "1.0.0", "run_id": self.run_id, "graph_id": self.graph.graph_id, "node_id": node.node_id, "attempt_id": attempt.attempt_id, "kind": node.kind, "action": node.metadata.get("human_action", node.metadata.get("request", "human confirmation or action is required")), "instructions": node.metadata.get("instructions", "Complete the requested action, then start a new bounded execution."), "created_at": self._now()}
        ref = self.artifacts.write_json("human_action_request.json", request)
        self._append(EventType.HUMAN_ACTION_REQUESTED, node_id=node.node_id, attempt_id=attempt.attempt_id, payload={"request_ref": ref})
        return ref

    def _execute_node(self, node: CompiledNode, record: NodeRuntimeRecord) -> _NodeRun:
        self._check_cancelled()
        self.budget.start_node(node.node_id, node.max_visits, self.started_monotonic or self.monotonic_clock())
        record.visit_count += 1
        self._set_state(record, NodeStatus.RUNNING, node_id=node.node_id)
        self._append(EventType.NODE_STARTED, node_id=node.node_id)
        attempt = self._make_attempt(record, node, "")
        result = _NodeRun()
        owner = f"{self.run_id}:{node.node_id}:{attempt.attempt_id}"
        acquired = False
        try:
            arguments, arguments_digest = self.bindings.resolve(node, outputs=self.outputs, world=self.world, now=self._now())
            attempt.resolved_arguments_digest = arguments_digest
            self._write_attempt_json(node.node_id, attempt.attempt_id, "resolved_arguments", arguments)
            self._append(EventType.INPUTS_RESOLVED, node_id=node.node_id, attempt_id=attempt.attempt_id, payload={"resolved_arguments_digest": arguments_digest})
            ok, predicate = self._evaluate_preconditions(node, attempt=attempt)
            result.predicate = predicate
            if node.kind == "CHECK":
                result.output = predicate.to_dict() if predicate else {"satisfied": True}
                result.predicate = predicate or PredicateResult(True, f"check:{node.node_id}", "check", (), "check passed", (), self._now())
                self._append(EventType.PREDICATE_EVALUATED, node_id=node.node_id, attempt_id=attempt.attempt_id, payload=result.predicate.to_dict())
                return result
            if not ok:
                result.error = make_error(ErrorCode.PRECONDITION_FALSE, "runtime precondition evaluated false", details={"node_id": node.node_id}, source="predicate_engine")
                return result
            if node.kind in {"APPROVAL", "HUMAN_ACTION"}:
                self._human_request(node, attempt)
                record.status = NodeStatus.CANCELLED
                result.error = make_error(ErrorCode.HUMAN_ACTION_REQUIRED, "human action is required", source="graph_executor")
                return result
            if node.kind == "COMPUTE":
                result.output = self._compute(node, arguments)
                return result
            if node.kind == "VERIFY":
                attempt.verification_called = True
                graph_verifier = (node.verifier_contract or {}).get("graph")
                predicate_spec = node.metadata.get("goal_predicate") or (graph_verifier.get("predicate") if isinstance(graph_verifier, dict) else None)
                if predicate_spec is not None and isinstance(predicate_spec, dict):
                    verification = self.verifier.verify_goal(predicate_spec, self.world)
                else:
                    prior = next((value for value in reversed(list(self.results.values())) if isinstance(value, dict) and value.get("capability_id")), None)
                    capability = self.registry.get(prior.get("capability_id")) if prior else None
                    verification = VerificationResult(False, node.node_id, "missing_verifier", (), None, self._now(), (make_error(ErrorCode.VERIFICATION_FAILED, "VERIFY node has no usable verifier evidence"),), ()) if capability is None else self.verifier.verify_effect(capability, ActionResult.from_dict(prior) if prior.get("status") else prior, self.world)  # type: ignore[arg-type]
                result.verification = verification
                self.verifications[node.node_id] = verification
                if verification.verified:
                    for fact_id in verification.evidence:
                        if isinstance(fact_id, str):
                            fact = self.world.get_fact(fact_id, include_invalidated=True)
                            if fact is not None and fact.status == FactStatus.TENTATIVE:
                                self.world.add_fact(replace(fact, status=FactStatus.VERIFIED))
                ref = self._write_attempt_json(node.node_id, attempt.attempt_id, "verification_result", verification.to_dict())
                record.latest_verification_result_ref = ref
                self._append(EventType.VERIFICATION_COMPLETED, node_id=node.node_id, attempt_id=attempt.attempt_id, payload={"verification_ref": ref, "verified": verification.verified})
                if self.mode in {"dry_run"}:
                    result.planned_only = True
                    result.output = verification.to_dict()
                    return result
                if not verification.verified:
                    result.error = verification.errors[0] if verification.errors else make_error(ErrorCode.VERIFICATION_FAILED, "verification failed")
                result.output = verification.to_dict()
                return result
            requirements = self._resource_requirements(node)
            if requirements:
                acquired = True
                locked = self.resource_locks.acquire(requirements, owner=owner)
                self._append(EventType.RESOURCE_ACQUIRED, node_id=node.node_id, attempt_id=attempt.attempt_id, payload={"resources": list(locked)})
            self._check_cancelled()
            if node.capability_id is None:
                raise ExecutionError(ErrorCode.CAPABILITY_NOT_FOUND, "runtime node requires a capability")
            if node.kind == "RECOVER":
                self.budget.add_recovery_action()
            attempt.dispatch_called = True
            self.budget.add_tool_call(self.started_monotonic or self.monotonic_clock())
            self._append(EventType.DISPATCH_REQUESTED, node_id=node.node_id, attempt_id=attempt.attempt_id, payload={"capability_id": node.capability_id, "arguments_digest": arguments_digest})
            context = DispatchContext(mode=SkillMode(self.mode), hardware_allowed=False, execute=False, artifact_dir=self.artifacts.root, fixture_roots=() if self.from_artifacts_root is None else (self.from_artifacts_root,), recovery=node.kind == "RECOVER", metadata={"run_id": self.run_id, "node_id": node.node_id})
            dispatch_request = DispatchRequest(node.capability_id, arguments, request_id=f"{self.run_id}:{attempt.attempt_id}", artifact_refs=tuple(node.metadata.get("artifact_refs", ())))
            dispatch_started = self.monotonic_clock()
            dispatched = self.dispatcher.dispatch(dispatch_request, context)
            self._check_cancelled()
            self.budget.check(self.started_monotonic or dispatch_started)
            ref = self._write_attempt_json(node.node_id, attempt.attempt_id, "dispatch_result", dispatched.to_dict())
            record.latest_dispatch_result_ref = ref
            self.results[node.node_id] = dispatched.to_dict()
            self._append(EventType.DISPATCH_COMPLETED, node_id=node.node_id, attempt_id=attempt.attempt_id, payload={"dispatch_result_ref": ref, "command_executed": getattr(dispatched, "command_executed", False)})
            if node.timeout_s is not None and self.monotonic_clock() - dispatch_started > node.timeout_s:
                result.error = make_error(ErrorCode.NODE_TIMEOUT, "compiled node timeout exceeded", details={"timeout_s": node.timeout_s})
                return result
            if isinstance(dispatched, ActionResult) and dispatched.command_executed:
                capability = self.registry.get(node.capability_id)
                if capability is not None:
                    invalidation = self.invalidation.invalidate_for_capability(capability, dispatched, arguments=arguments)
                    if invalidation.invalidated_fact_ids:
                        self._append(EventType.STATE_INVALIDATED, node_id=node.node_id, attempt_id=attempt.attempt_id, payload=invalidation.to_dict())
            if node.kind == "OBSERVE":
                result.output = self._result_output(dispatched)
                result.planned_only = bool(getattr(dispatched, "artifacts", {}).get("planned_only", False))
                if isinstance(dispatched, ObservationResult) and not dispatched.ok and not result.planned_only:
                    result.error = self._result_errors(dispatched)[0] if self._result_errors(dispatched) else make_error(ErrorCode.CAPABILITY_EXECUTION_FAILED, "observation failed")
                elif isinstance(dispatched, ObservationResult):
                    result.progressed = self._project_observations(node, dispatched, attempt)
            elif node.kind in {"ACT", "RECOVER"}:
                if not isinstance(dispatched, ActionResult):
                    result.error = make_error(ErrorCode.CAPABILITY_EXECUTION_FAILED, "action node did not return an ActionResult")
                else:
                    result.output = dispatched.outputs
                    result.planned_only = dispatched.planned_only
                    result.progressed = self._project_tentative_effects(node, dispatched, attempt)
                    if dispatched.errors:
                        result.error = dispatched.errors[0]
            else:
                result.output = self._result_output(dispatched)
                errors = self._result_errors(dispatched)
                if errors:
                    result.error = errors[0]
            if node.timeout_s is not None and getattr(dispatched, "metrics", {}).get("timed_out") is True:
                result.error = make_error(ErrorCode.NODE_TIMEOUT, "node timeout reported by dispatcher")
            return result
        except ExecutionError as exc:
            result.error = exc.to_error_info()
            return result
        except Exception as exc:
            result.error = make_error(ErrorCode.UNKNOWN_ERROR, f"node execution failed: {exc}", source="graph_executor")
            return result
        finally:
            if acquired:
                released = self.resource_locks.release_owner(owner)
                self._append(EventType.RESOURCE_RELEASED, node_id=node.node_id, attempt_id=attempt.attempt_id, payload={"resources": list(released)})

    def _can_retry(self, node: CompiledNode, record: NodeRuntimeRecord, error: ErrorInfo) -> bool:
        policy = dict(node.retry_policy)
        if policy.get("parameter_adjustment_policy"):
            return False
        if record.attempt_count >= int(policy.get("max_attempts", 1)):
            return False
        if error.code.value not in tuple(policy.get("retry_on_error_codes", ())):
            return False
        if not error.retryable or error.requires_human or error.severity in {ErrorSeverity.FATAL, ErrorSeverity.UNSAFE}:
            return False
        if self.budget.usage.same_error_retries >= int(self.graph.budgets.get("max_same_error_retries", 0)):
            return False
        return True

    def _route(self, node: CompiledNode, *, success: bool, predicate: PredicateResult | None, error_code: str | None) -> str | None:
        edges = [edge for edge in self.graph.compiled_edges if edge.source_node_id == node.node_id]
        exact: list[CompiledEdge] = []
        if predicate is not None:
            wanted = "PREDICATE_TRUE" if predicate.satisfied else "PREDICATE_FALSE"
            exact = [edge for edge in edges if edge.condition == wanted]
        if not exact:
            wanted = "SUCCESS" if success else "FAILURE"
            exact = [edge for edge in edges if edge.condition == wanted and (not edge.error_codes or (error_code in edge.error_codes))]
        if not exact:
            exact = [edge for edge in edges if edge.condition == "DEFAULT"]
        if not exact:
            return None
        exact.sort(key=lambda item: (-item.priority, item.edge_id))
        if len(exact) > 1 and exact[0].priority == exact[1].priority:
            raise ExecutionError(ErrorCode.NON_DETERMINISTIC_GRAPH, "multiple matching edges have the same priority", {"node_id": node.node_id, "edge_ids": [item.edge_id for item in exact]})
        selected = exact[0]
        self.budget.traverse_edge(selected.edge_id, selected.max_traversals)
        self._append(EventType.EDGE_TRAVERSED, node_id=node.node_id, payload={"edge_id": selected.edge_id, "target_node_id": selected.target_node_id, "condition": selected.condition})
        return selected.target_node_id

    def _finish(self, *, outcome: ExecutionOutcome, terminal_state: str, graph_completed: bool, goal_verified: bool, error: ErrorInfo | None = None) -> TaskExecutionResult:
        self._outcome = outcome
        self._terminal_state = terminal_state
        if error is not None:
            self._errors.append(error)
        self.ended_at = self._now()
        self.resource_locks.release_owner(self.run_id)
        self._append(EventType.TASK_TERMINATED, payload={"outcome": outcome.value, "terminal_state": terminal_state})
        self._checkpoint()
        result = TaskExecutionResult(self.run_id, self.graph.graph_id, self.graph.goal_id, self.graph.plan_hash, self.mode, terminal_state, outcome, ExecutionScope.NONE if self.mode == "dry_run" else ExecutionScope.SIMULATED if self.mode == "mock" else ExecutionScope.ARTIFACT_REPLAY, graph_completed, goal_verified, False, False, self._planned_only, tuple(self.node_records.values()), self.budget.usage, self._world_snapshot_ref, "events.jsonl", "checkpoint.json", tuple(self._errors), tuple(self._warnings), {"artifact_dir": str(self.artifacts.root.name)}, self.started_at, self.ended_at)
        self.artifacts.write_json("task_execution_result.json", result.to_dict())
        return result

    def run(self) -> TaskExecutionResult:
        self.started_monotonic = self.monotonic_clock()
        try:
            if self._restored:
                self._restore()
            else:
                self._append(EventType.TASK_CREATED, payload={"mode": self.mode, "plan_hash": self.graph.plan_hash})
            preflight = ExecutionPreflightValidator(self.graph, registry=self.registry, manifest=self.manifest, mode=self.mode).validate()
            if not preflight.ok:
                self._append(EventType.EXECUTION_PREFLIGHT_FAILED, payload=preflight.to_dict())
                return self._finish(outcome=ExecutionOutcome.FAILED, terminal_state=TaskTerminalState.FAILED.value, graph_completed=False, goal_verified=False, error=preflight.errors[0].to_error_info())
            self._append(EventType.EXECUTION_PREFLIGHT_PASSED, payload={"mode": self.mode})
            if self._restored and any(item.get("event_type") == EventType.TASK_TERMINATED.value for item in self.event_store.events()):
                return self._finish(outcome=self._outcome, terminal_state=self._terminal_state, graph_completed=self._terminal_state == TaskTerminalState.SUCCEEDED.value, goal_verified=bool(self._goal_verification and self._goal_verification.verified))
            current = self.frontier or next((node.node_id for node in self.graph.compiled_nodes if self.node_records[node.node_id].status in {NodeStatus.PENDING, NodeStatus.READY, NodeStatus.RUNNING}), None)
            current = current or self.graph.compiled_nodes[0].node_id
            while current is not None:
                self._check_cancelled()
                self.budget.check(self.started_monotonic)
                node = self.node_map[current]
                record = self.node_records[current]
                if record.status == NodeStatus.SUCCEEDED:
                    current = self._route(node, success=True, predicate=None, error_code=None)
                    continue
                if record.status == NodeStatus.PENDING:
                    self._set_state(record, NodeStatus.READY, node_id=current)
                if record.status == NodeStatus.RUNNING:
                    record.status = NodeStatus.READY
                outcome = self._execute_node(node, record)
                attempt = record.attempts[-1]
                if outcome.error is None:
                    self._set_state(record, NodeStatus.SUCCEEDED, node_id=node.node_id, attempt_id=attempt.attempt_id)
                    attempt.status = NodeStatus.SUCCEEDED
                    attempt.ended_at = self._now()
                    record.ended_at = attempt.ended_at
                    output = {} if outcome.output is None else outcome.output
                    ref = self._write_attempt_json(node.node_id, attempt.attempt_id, "output", output)
                    attempt.result_ref = ref
                    record.output_ref = ref
                    self.outputs[node.node_id] = output
                    self.budget.complete_node(node.node_id)
                    self._append(EventType.NODE_SUCCEEDED, node_id=node.node_id, attempt_id=attempt.attempt_id, payload={"output_ref": ref, "planned_only": outcome.planned_only})
                    self._checkpoint()
                    if current in self.graph.terminal_nodes:
                        goal_verified = False
                        if self.graph.goal_predicate is not None and self.mode != "dry_run":
                            self._goal_verification = self.verifier.verify_goal(self.graph.goal_predicate, self.world)
                            goal_verified = self._goal_verification.verified
                        if goal_verified:
                            return self._finish(outcome=ExecutionOutcome.GOAL_VERIFIED, terminal_state=TaskTerminalState.SUCCEEDED.value, graph_completed=True, goal_verified=True)
                        return self._finish(outcome=ExecutionOutcome.PLAN_COMPLETED if self.mode == "dry_run" else ExecutionOutcome.GRAPH_COMPLETED_UNVERIFIED, terminal_state=TaskTerminalState.SUCCEEDED.value, graph_completed=True, goal_verified=False)
                    next_node = self._route(node, success=True, predicate=outcome.predicate if node.kind == "CHECK" else None, error_code=None)
                    if next_node is None:
                        return self._finish(outcome=ExecutionOutcome.FAILED, terminal_state=TaskTerminalState.FAILED.value, graph_completed=False, goal_verified=False, error=make_error(ErrorCode.NO_TERMINAL_PATH, "no matching terminal path"))
                    self.frontier = next_node
                    current = next_node
                    continue
                error = outcome.error
                attempt.status = NodeStatus.CANCELLED if record.status == NodeStatus.CANCELLED else NodeStatus.FAILED
                attempt.ended_at = self._now()
                attempt.error = error
                record.ended_at = attempt.ended_at
                record.latest_error_code = error.code.value
                self._append(EventType.NODE_FAILED, node_id=node.node_id, attempt_id=attempt.attempt_id, payload={"error": error.to_dict()})
                if record.status != NodeStatus.CANCELLED:
                    self._set_state(record, NodeStatus.FAILED, node_id=node.node_id, attempt_id=attempt.attempt_id)
                fingerprint = ProgressDetector.fingerprint(frontier=current, arguments_digest=attempt.resolved_arguments_digest, world_digest=self._world_digest(), error_code=error.code.value, goal_satisfied=None if self._goal_verification is None else self._goal_verification.verified)
                no_progress = self.progress.observe(fingerprint, progressed=outcome.progressed)
                if no_progress:
                    self._append(EventType.NO_PROGRESS_DETECTED, node_id=current, attempt_id=attempt.attempt_id, payload={"fingerprint": fingerprint, "repeat_count": self.progress.repeat_count})
                    return self._finish(outcome=ExecutionOutcome.NO_PROGRESS, terminal_state=TaskTerminalState.NO_PROGRESS.value, graph_completed=False, goal_verified=False, error=make_error(ErrorCode.NO_PROGRESS, "no semantic progress detected"))
                if node.kind in {"APPROVAL", "HUMAN_ACTION"} or error.requires_human:
                    if record.status != NodeStatus.CANCELLED:
                        self._set_state(record, NodeStatus.FAILED, node_id=node.node_id, attempt_id=attempt.attempt_id)
                    return self._finish(outcome=ExecutionOutcome.NEEDS_HUMAN, terminal_state=TaskTerminalState.NEEDS_HUMAN.value, graph_completed=False, goal_verified=False, error=error)
                if error.code == ErrorCode.REPLAN_REQUIRED or dict(node.retry_policy).get("parameter_adjustment_policy"):
                    return self._finish(outcome=ExecutionOutcome.REPLAN_REQUIRED, terminal_state=TaskTerminalState.FAILED.value, graph_completed=False, goal_verified=False, error=make_error(ErrorCode.REPLAN_REQUIRED, "compiled retry policy requires replanning"))
                if self._can_retry(node, record, error):
                    self.budget.add_retry()
                    self._set_state(record, NodeStatus.READY, node_id=node.node_id, attempt_id=attempt.attempt_id)
                    self._append(EventType.NODE_RETRY_SCHEDULED, node_id=node.node_id, attempt_id=attempt.attempt_id, payload={"error_code": error.code.value, "attempt": record.attempt_count + 1})
                    self._checkpoint()
                    continue
                if error.severity == ErrorSeverity.UNSAFE:
                    return self._finish(outcome=ExecutionOutcome.UNSAFE, terminal_state=TaskTerminalState.UNSAFE.value, graph_completed=False, goal_verified=False, error=error)
                if error.code in {ErrorCode.NODE_TIMEOUT, ErrorCode.TASK_TIMEOUT, ErrorCode.MOTION_TIMEOUT}:
                    return self._finish(outcome=ExecutionOutcome.TIMEOUT, terminal_state=TaskTerminalState.TIMEOUT.value, graph_completed=False, goal_verified=False, error=error)
                try:
                    next_node = self._route(node, success=False, predicate=None, error_code=error.code.value)
                except ExecutionError as exc:
                    return self._finish(outcome=ExecutionOutcome.FAILED, terminal_state=TaskTerminalState.FAILED.value, graph_completed=False, goal_verified=False, error=exc.to_error_info())
                if next_node is None:
                    return self._finish(outcome=ExecutionOutcome.FAILED, terminal_state=TaskTerminalState.FAILED.value, graph_completed=False, goal_verified=False, error=error)
                self.frontier = next_node
                current = next_node
        except ExecutionError as exc:
            if exc.code == ErrorCode.EXECUTION_INTERRUPTED and self.cancel_token.is_cancelled:
                self._append(EventType.TASK_CANCELLED, payload={"reason": self.cancel_token.reason})
                return self._finish(outcome=ExecutionOutcome.CANCELLED, terminal_state=TaskTerminalState.CANCELLED.value, graph_completed=False, goal_verified=False, error=make_error(ErrorCode.EXECUTION_INTERRUPTED, self.cancel_token.reason or "execution cancelled"))
            if exc.code in {ErrorCode.BUDGET_EXCEEDED, ErrorCode.TASK_TIMEOUT}:
                return self._finish(outcome=ExecutionOutcome.TIMEOUT if exc.code == ErrorCode.TASK_TIMEOUT else ExecutionOutcome.BUDGET_EXCEEDED, terminal_state=TaskTerminalState.TIMEOUT.value if exc.code == ErrorCode.TASK_TIMEOUT else TaskTerminalState.BUDGET_EXCEEDED.value, graph_completed=False, goal_verified=False, error=exc.to_error_info())
            return self._finish(outcome=ExecutionOutcome.FAILED, terminal_state=TaskTerminalState.FAILED.value, graph_completed=False, goal_verified=False, error=exc.to_error_info())
        except Exception as exc:
            return self._finish(outcome=ExecutionOutcome.FAILED, terminal_state=TaskTerminalState.FAILED.value, graph_completed=False, goal_verified=False, error=make_error(ErrorCode.UNKNOWN_ERROR, str(exc), source="graph_executor"))
        return self._finish(outcome=ExecutionOutcome.FAILED, terminal_state=TaskTerminalState.FAILED.value, graph_completed=False, goal_verified=False, error=make_error(ErrorCode.NO_TERMINAL_PATH, "executor reached an empty frontier"))
