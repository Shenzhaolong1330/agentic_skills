from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any, Callable, Mapping
import uuid

from ..contracts.enums import ErrorCode, TaskTerminalState
from ..contracts.errors import ErrorInfo
from ..dispatch.error_mapping import make_error
from ..execution.events import EventType
from ..execution.executor import GraphExecutor
from ..execution.models import ExecutionOutcome, TaskExecutionResult
from ..planning.canonical import digest
from ..planning.goals import GoalKind, GoalSpec
from ..planning.compiler import CompiledTaskGraph
from .budgets import remaining_budget
from .lineage import append_lineage
from .models import (
    HeldObjectEvidence,
    PlanLineage,
    RecoveryAttempt,
    RecoveryContext,
    RecoveryDisposition,
    RecoveryResultStatus,
    ReplanRequest,
    ReplanResult,
    RecoveryResult,
)
from .replanning import NoReplanner, compile_replan, make_replan_request_id
from .registry import RecoveryPolicyRegistry
from .selector import RecoverySelector
from .templates import RecoveryTemplateRegistry


@dataclass(frozen=True)
class ResilientTaskResult:
    root_plan_hash: str
    final_plan_hash: str
    plan_lineage: PlanLineage
    recovery_attempts: tuple[RecoveryAttempt, ...]
    replan_attempts: tuple[ReplanResult, ...]
    recovery_exhausted: bool
    final_execution_result: TaskExecutionResult

    @property
    def outcome(self) -> ExecutionOutcome:
        return self.final_execution_result.outcome

    def to_dict(self) -> dict[str, Any]:
        return {
            "root_plan_hash": self.root_plan_hash,
            "final_plan_hash": self.final_plan_hash,
            "plan_lineage": self.plan_lineage.to_dict(),
            "recovery_attempts": [item.to_dict() for item in self.recovery_attempts],
            "replan_attempts": [item.to_dict() for item in self.replan_attempts],
            "recovery_exhausted": self.recovery_exhausted,
            "final_execution_result": self.final_execution_result.to_dict(),
        }


class RecoveryOrchestrator:
    """Compose recovery around GraphExecutor without duplicating its scheduler."""

    def __init__(
        self,
        *,
        selector: RecoverySelector,
        compiler: Any | None = None,
        template_registry: RecoveryTemplateRegistry | None = None,
        replanner: Any | None = None,
        executor_factory: Callable[..., Any] | None = None,
        max_iterations: int = 16,
    ) -> None:
        self.selector = selector
        self.compiler = compiler
        self.templates = template_registry or RecoveryTemplateRegistry(getattr(selector.registry, "capability_registry", None))
        self.replanner = replanner or NoReplanner()
        self.executor_factory = executor_factory or GraphExecutor
        if isinstance(max_iterations, bool) or not isinstance(max_iterations, int) or max_iterations < 1 or max_iterations > 1024:
            raise ValueError("max_iterations must be from 1 to 1024")
        self.max_iterations = max_iterations

    def _new_executor(self, graph: CompiledTaskGraph, kwargs: Mapping[str, Any]) -> Any:
        values = dict(kwargs)
        try:
            return self.executor_factory(graph, **values)
        except TypeError:
            # Small fake executors used by contract tests commonly accept only
            # the graph.  This fallback does not alter production behavior.
            return self.executor_factory(graph)

    @staticmethod
    def _default_envelope(graph: CompiledTaskGraph) -> dict[str, Any]:
        return {
            "envelope_id": f"compiled_{graph.graph_id}",
            "target_mode": graph.target_mode,
            "allowed_capabilities": sorted({node.capability_id for node in graph.compiled_nodes if node.capability_id}),
            "forbidden_capabilities": [],
            "risk_ceiling": "HIGH_RISK",
            "allowed_resources": sorted({resource.get("resource_id") for node in graph.compiled_nodes for resource in node.resources if resource.get("resource_id")}),
            "workspace_constraints": list(graph.workspace_constraints),
            **dict(graph.budgets),
            "approval_policy": {},
            "metadata": {},
        }

    @staticmethod
    def _default_goal(graph: CompiledTaskGraph) -> dict[str, Any]:
        return {
            "goal_id": graph.goal_id,
            "goal_kind": graph.goal_kind or GoalKind.COMPUTE.value,
            "description": f"goal for {graph.goal_id}",
            "success_predicate": graph.goal_predicate or {"predicate_id": "goal.exists", "operator": "exists", "args": [{"predicate": "goal.placeholder"}]},
            "entities": [], "initial_assumptions": [], "failure_predicates": [], "required_evidence": [], "ambiguities": [], "metadata": {},
        }

    @staticmethod
    def _held_evidence(executor: Any) -> HeldObjectEvidence:
        try:
            facts = executor.world.query(predicate="holding", include_invalidated=False)
            facts += executor.world.query(predicate="relation.holding", include_invalidated=False)
        except Exception:
            return HeldObjectEvidence.UNKNOWN
        if not facts:
            return HeldObjectEvidence.UNKNOWN
        if any(fact.status.value == "VERIFIED" and fact.value is True for fact in facts):
            return HeldObjectEvidence.HOLDING_CONFIRMED
        if any(fact.status.value == "TENTATIVE" and fact.value is True for fact in facts):
            return HeldObjectEvidence.HOLDING_TENTATIVE
        if all(fact.status.value == "VERIFIED" and fact.value is False for fact in facts):
            return HeldObjectEvidence.NONE_CONFIRMED
        return HeldObjectEvidence.UNKNOWN

    def _context(self, result: TaskExecutionResult, executor: Any, *, original_envelope: Mapping[str, Any], root_goal: Mapping[str, Any], lineage: PlanLineage, history: tuple[Mapping[str, Any], ...], replan_history: tuple[Mapping[str, Any], ...]) -> RecoveryContext:
        failed_record = next((record for record in reversed(result.node_records) if record.latest_error_code), None)
        error = result.errors[-1] if result.errors else make_error(ErrorCode.UNKNOWN_ERROR, "graph execution failed", source="recovery_orchestrator")
        failed_node_id = failed_record.node_id if failed_record else "unknown"
        failed_node_kind = failed_record.kind if failed_record else "UNKNOWN"
        world_snapshot = executor.world.snapshot().to_dict() if hasattr(executor, "world") else {}
        usage = result.budget_usage.to_dict()
        budget = getattr(executor, "graph", None).budgets if getattr(executor, "graph", None) is not None else {}
        return RecoveryContext(
            run_id=result.run_id, graph_id=result.graph_id, goal_id=result.goal_id, plan_hash=result.plan_hash,
            failed_node_id=failed_node_id, failed_node_kind=failed_node_kind,
            failed_capability_id=(next((node.capability_id for node in executor.graph.compiled_nodes if node.node_id == failed_node_id), None) if hasattr(executor, "graph") else None),
            error=error, world_snapshot=world_snapshot, held_object_evidence=self._held_evidence(executor),
            completed_node_ids=tuple(record.node_id for record in result.node_records if record.status.value == "SUCCEEDED"),
            completed_valid_effects=(), invalidated_effects=(), remaining_budget=remaining_budget(budget, usage),
            original_envelope=original_envelope, mode=result.mode,
            latest_progress_fingerprint=(getattr(executor, "progress", None).previous if getattr(executor, "progress", None) is not None else None),
            recovery_history=history, replan_history=replan_history,
        )

    @staticmethod
    def _terminalize(result: TaskExecutionResult, outcome: ExecutionOutcome, error: ErrorInfo) -> TaskExecutionResult:
        terminal = {
            ExecutionOutcome.NEEDS_HUMAN: TaskTerminalState.NEEDS_HUMAN.value,
            ExecutionOutcome.UNSAFE: TaskTerminalState.UNSAFE.value,
            ExecutionOutcome.NO_PROGRESS: TaskTerminalState.NO_PROGRESS.value,
            ExecutionOutcome.BUDGET_EXCEEDED: TaskTerminalState.BUDGET_EXCEEDED.value,
        }.get(outcome, TaskTerminalState.FAILED.value)
        return replace(result, outcome=outcome, terminal_state=terminal, errors=tuple((*result.errors, error)))

    @staticmethod
    def _append_event(executor: Any, event: EventType, payload: Mapping[str, Any]) -> None:
        store = getattr(executor, "event_store", None)
        if store is not None:
            store.append(event, payload=dict(payload))

    @staticmethod
    def _checkpoint(executor: Any, state: Mapping[str, Any]) -> None:
        store = getattr(executor, "checkpoints", None)
        if store is None:
            return
        try:
            payload = store.load()
        except Exception:
            payload = {"run_id": getattr(executor, "run_id", "unknown"), "graph_id": getattr(getattr(executor, "graph", None), "graph_id", "unknown"), "plan_hash": getattr(getattr(executor, "graph", None), "plan_hash", "unknown")}
        payload["recovery_state"] = dict(state)
        payload.pop("checkpoint_digest", None)
        store.write(payload)

    def run(
        self,
        compiled_graph: CompiledTaskGraph,
        *,
        executor_kwargs: Mapping[str, Any] | None = None,
        original_envelope: Mapping[str, Any] | None = None,
        root_goal_spec: Mapping[str, Any] | None = None,
    ) -> ResilientTaskResult:
        kwargs = dict(executor_kwargs or {})
        envelope = dict(original_envelope or self._default_envelope(compiled_graph))
        goal = dict(root_goal_spec or self._default_goal(compiled_graph))
        lineage = PlanLineage(compiled_graph.plan_hash)
        history: list[Mapping[str, Any]] = []
        replan_history: list[Mapping[str, Any]] = []
        attempts: list[RecoveryAttempt] = []
        replans: list[Any] = []
        current_graph = compiled_graph
        final_result: TaskExecutionResult | None = None
        exhausted = False
        for iteration in range(self.max_iterations):
            executor = self._new_executor(current_graph, kwargs)
            final_result = executor.run()
            if final_result.outcome in {ExecutionOutcome.GOAL_VERIFIED, ExecutionOutcome.GRAPH_COMPLETED_UNVERIFIED, ExecutionOutcome.PLAN_COMPLETED}:
                break
            context = self._context(final_result, executor, original_envelope=envelope, root_goal=goal, lineage=lineage, history=tuple(history), replan_history=tuple(replan_history))
            self._append_event(executor, EventType.RECOVERY_CONTEXT_CREATED, context.to_dict())
            decision = self.selector.select(context)
            self._append_event(executor, EventType.RECOVERY_CANDIDATES_EVALUATED, {"decision_id": decision.decision_id, "eligible": [item.to_dict() for item in decision.eligible_candidates], "rejected": [item.to_dict() for item in decision.rejected_candidates]})
            if decision.selected_strategy is None:
                self._append_event(executor, EventType.RECOVERY_EXHAUSTED, {"decision_id": decision.decision_id, "reason": decision.reason})
                exhausted = True
                final_result = self._terminalize(final_result, ExecutionOutcome.NEEDS_HUMAN, make_error(ErrorCode.HUMAN_ACTION_REQUIRED, decision.reason, source="recovery_selector"))
                break
            strategy = decision.selected_strategy
            self._append_event(executor, EventType.RECOVERY_SELECTED, {"decision_id": decision.decision_id, "strategy": strategy.to_dict()})
            attempt = RecoveryAttempt(f"recovery_{uuid.uuid4().hex}", decision.decision_id, strategy.strategy_id, "STARTED")
            attempts.append(attempt)
            self._append_event(executor, EventType.RECOVERY_STARTED, attempt.to_dict())
            history.append({"attempt_id": attempt.attempt_id, "decision_id": decision.decision_id, "strategy_id": strategy.strategy_id.value, "error_code": context.error_info.code.value, "progress_fingerprint": context.latest_progress_fingerprint, "world_state_digest": digest(context.world_snapshot)})
            self._checkpoint(executor, {"recovery_attempts": [item.to_dict() for item in attempts], "replan_attempts": [item.to_dict() for item in replans], "plan_lineage": lineage.to_dict(), "active_plan_hash": current_graph.plan_hash})
            if strategy.disposition == RecoveryDisposition.NEEDS_HUMAN:
                final_result = self._terminalize(final_result, ExecutionOutcome.NEEDS_HUMAN, make_error(ErrorCode.HUMAN_ACTION_REQUIRED, "recovery requires human action", source="recovery_orchestrator"))
                self._append_event(executor, EventType.RECOVERY_COMPLETED, {"attempt_id": attempt.attempt_id, "status": RecoveryResultStatus.NEEDS_HUMAN.value})
                break
            if strategy.disposition == RecoveryDisposition.TERMINATE:
                exhausted = True
                self._append_event(executor, EventType.RECOVERY_COMPLETED, {"attempt_id": attempt.attempt_id, "status": RecoveryResultStatus.TERMINATED.value})
                break
            if strategy.disposition == RecoveryDisposition.REPLAN_REMAINDER:
                request = ReplanRequest(
                    request_id=f"replan_{uuid.uuid4().hex}", root_goal_spec=goal, original_envelope=envelope,
                    current_world_snapshot=context.world_snapshot, root_plan_hash=lineage.root_plan_hash, current_plan_hash=current_graph.plan_hash,
                    completed_node_ids=context.completed_node_ids, completed_valid_effects=context.completed_valid_effects, invalidated_effects=context.invalidated_effects,
                    failed_node_id=context.failed_node_id, failure_error=context.error_info, remaining_budget=context.remaining_budget,
                    recovery_history=tuple(history), plan_lineage=lineage, reason=str(strategy.strategy_id.value),
                )
                self._append_event(executor, EventType.REPLAN_REQUESTED, request.to_dict())
                proposal = self.replanner.replan(request)
                compiled = compile_replan(request, proposal, compiler=self.compiler, internal_allowlist=envelope.get("metadata", {}).get("internal_capability_allowlist", ())) if self.compiler is not None else ReplanResult(request.request_id, "REPLAN_REJECTED", errors=("no compiler configured",))
                replans.append(compiled)
                replan_history.append(compiled.to_dict())
                if compiled.status != "COMPILED" or compiled.compiled_graph is None:
                    self._append_event(executor, EventType.REPLAN_REJECTED, compiled.to_dict())
                    continue
                lineage = append_lineage(lineage, plan_hash=compiled.compiled_graph.plan_hash, reason=request.reason, failure_node_id=context.failed_node_id, error_code=context.error_info.code.value, recovery_strategy_id=strategy.strategy_id.value, remaining_budget=context.remaining_budget, world_state=context.world_snapshot)
                current_graph = compiled.compiled_graph
                self._append_event(executor, EventType.REPLAN_COMPILED, compiled.to_dict())
                self._append_event(executor, EventType.PLAN_REPLACED, {"plan_hash": current_graph.plan_hash, "parent_plan_hash": lineage.entries[-1].parent_plan_hash})
                self._append_event(executor, EventType.PLAN_LINEAGE_UPDATED, lineage.to_dict())
                continue
            if strategy.disposition == RecoveryDisposition.RECOVERY_SUBGRAPH:
                graph = self.templates.build(strategy, context)
                if self.compiler is None:
                    self._append_event(executor, EventType.RECOVERY_FAILED, {"attempt_id": attempt.attempt_id, "reason": "no compiler configured"})
                    exhausted = True
                    break
                recovery_goal = {**goal, "goal_id": graph.goal_id, "goal_kind": GoalKind.COMPUTE.value, "description": "bounded recovery subgraph", "success_predicate": {"predicate_id": "recovery.completed", "operator": "exists", "args": [{"predicate": "recovery.completed"}]}}
                report = self.compiler.compile(recovery_goal, envelope, graph)
                self._append_event(executor, EventType.RECOVERY_SUBGRAPH_COMPILED, report.to_dict())
                if not report.ok or report.compiled_graph is None:
                    self._append_event(executor, EventType.RECOVERY_FAILED, {"attempt_id": attempt.attempt_id, "reason": "recovery subgraph compilation failed"})
                    exhausted = True
                    break
                recovery_executor = self._new_executor(report.compiled_graph, kwargs)
                recovery_result = recovery_executor.run()
                if recovery_result.outcome not in {ExecutionOutcome.GOAL_VERIFIED, ExecutionOutcome.GRAPH_COMPLETED_UNVERIFIED, ExecutionOutcome.PLAN_COMPLETED}:
                    self._append_event(executor, EventType.RECOVERY_FAILED, {"attempt_id": attempt.attempt_id, "result": recovery_result.to_dict()})
                    exhausted = True
                    break
                self._append_event(executor, EventType.RECOVERY_COMPLETED, {"attempt_id": attempt.attempt_id, "result": recovery_result.to_dict()})
                continue
            # LOCAL_RETRY is intentionally delegated back to GraphExecutor on
            # the next bounded iteration; its own retry policy remains the only
            # scheduler for a node.
            self._append_event(executor, EventType.RECOVERY_COMPLETED, {"attempt_id": attempt.attempt_id, "status": RecoveryResultStatus.COMPLETED.value})
        else:
            exhausted = True
        assert final_result is not None
        return ResilientTaskResult(compiled_graph.plan_hash, current_graph.plan_hash, lineage, tuple(attempts), tuple(replans), exhausted, final_result)


__all__ = ["RecoveryOrchestrator", "ResilientTaskResult"]
