from __future__ import annotations

"""Bounded, first-party remainder replanning interfaces."""

from dataclasses import dataclass
from typing import Any, Mapping, Protocol, Sequence

from ..contracts.enums import RiskClass
from ..planning.canonical import digest
from ..planning.compiler import RISK_ORDER
from ..planning.envelope import ExecutionEnvelope
from ..planning.goals import GoalSpec
from ..planning.graph import TaskGraph
from .models import ReplanRequest, ReplanResult


class RemainderReplanner(Protocol):
    def replan(self, request: ReplanRequest) -> ReplanResult: ...


class NoReplanner:
    def replan(self, request: ReplanRequest) -> ReplanResult:
        return ReplanResult(request.request_id, "REPLAN_REJECTED", errors=("no first-party replanner is registered",))


class DeterministicFixtureReplanner:
    """Select a pre-registered replacement fixture by stable reason key."""

    def __init__(self, fixtures: Mapping[str, Mapping[str, Any]]) -> None:
        self.fixtures = {str(key): dict(value) for key, value in fixtures.items()}

    def replan(self, request: ReplanRequest) -> ReplanResult:
        keys = (request.reason, request.failure_error.code.value, "default")
        fixture = next((self.fixtures[key] for key in keys if key in self.fixtures), None)
        if fixture is None:
            return ReplanResult(request.request_id, "REPLAN_REJECTED", errors=("no deterministic fixture matches the replan reason",))
        required = {"goal", "envelope", "graph"}
        if not required.issubset(fixture):
            return ReplanResult(request.request_id, "REPLAN_REJECTED", errors=("fixture is missing goal, envelope, or graph",))
        return ReplanResult(request.request_id, "PROPOSED", replacement_goal_spec=fixture["goal"], replacement_envelope=fixture["envelope"], replacement_task_graph=fixture["graph"])


class TaskDefinitionReplanner:
    """Adapter for a trusted task definition's deterministic builder."""

    def __init__(self, task_definition: Any) -> None:
        self.task_definition = task_definition

    def replan(self, request: ReplanRequest) -> ReplanResult:
        builder = getattr(self.task_definition, "replan_remainder", None)
        if not callable(builder):
            return ReplanResult(request.request_id, "REPLAN_REJECTED", errors=("task definition has no deterministic remainder replanner",))
        value = builder(request)
        if isinstance(value, ReplanResult):
            return value
        if not isinstance(value, Mapping):
            return ReplanResult(request.request_id, "REPLAN_REJECTED", errors=("task replanner returned a non-object",))
        return ReplanResult(request.request_id, "PROPOSED", replacement_goal_spec=value.get("goal"), replacement_envelope=value.get("envelope"), replacement_task_graph=value.get("graph"))


def _as_envelope(value: ExecutionEnvelope | Mapping[str, Any]) -> ExecutionEnvelope:
    return value if isinstance(value, ExecutionEnvelope) else ExecutionEnvelope.from_dict(value)


def _workspace_contains(original: ExecutionEnvelope, replacement: ExecutionEnvelope) -> bool:
    if not replacement.workspace_constraints:
        return True
    if not original.workspace_constraints:
        return False
    for candidate in replacement.workspace_constraints:
        matches = [item for item in original.workspace_constraints if item.frame == candidate.frame]
        if not matches:
            return False
        if not any(all(left <= inner and outer <= right for inner, left, outer, right in zip(candidate.min_xyz_m, item.min_xyz_m, candidate.max_xyz_m, item.max_xyz_m)) for item in matches):
            return False
    return True


def validate_replan_monotonicity(
    original_envelope: ExecutionEnvelope | Mapping[str, Any],
    replacement_envelope: ExecutionEnvelope | Mapping[str, Any],
    *,
    original_goal: GoalSpec | Mapping[str, Any] | None = None,
    replacement_goal: GoalSpec | Mapping[str, Any] | None = None,
    original_plan_hash: str | None = None,
    replacement_plan_hash: str | None = None,
    completed_valid_effects: Sequence[Mapping[str, Any]] = (),
    internal_allowlist: Sequence[str] = (),
) -> tuple[str, ...]:
    original = _as_envelope(original_envelope)
    replacement = _as_envelope(replacement_envelope)
    errors: list[str] = []
    if replacement.target_mode != original.target_mode:
        errors.append("target mode changed")
    if not set(replacement.allowed_capabilities).issubset(set(original.allowed_capabilities)):
        errors.append("allowed capabilities expanded")
    if not set(replacement.forbidden_capabilities).issuperset(set(original.forbidden_capabilities)):
        errors.append("forbidden capabilities were removed")
    if RISK_ORDER[replacement.risk_ceiling] > RISK_ORDER[original.risk_ceiling]:
        errors.append("risk ceiling increased")
    if not _workspace_contains(original, replacement):
        errors.append("workspace expanded")
    if not set(replacement.allowed_resources).issubset(set(original.allowed_resources)):
        errors.append("allowed resources expanded")
    for name in ("max_elapsed_s", "max_tool_calls", "max_replans", "max_recovery_actions", "max_same_error_retries", "max_nodes", "max_depth"):
        if getattr(replacement, name) > getattr(original, name):
            errors.append(f"budget increased: {name}")
    if not set(replacement.metadata.get("internal_capability_allowlist", ())).issubset(set(internal_allowlist)):
        errors.append("internal capability allowlist is not trusted")
    if original_goal is not None and replacement_goal is not None:
        old_goal = original_goal if isinstance(original_goal, GoalSpec) else GoalSpec.from_dict(original_goal)
        new_goal = replacement_goal if isinstance(replacement_goal, GoalSpec) else GoalSpec.from_dict(replacement_goal)
        old_blocking = {item.ambiguity_id for item in old_goal.ambiguities if item.blocking and not item.selected_resolution}
        new_blocking = {item.ambiguity_id for item in new_goal.ambiguities if item.blocking and not item.selected_resolution}
        if not old_blocking.issubset(new_blocking):
            errors.append("blocking ambiguity was ignored")
    if original_plan_hash is not None and replacement_plan_hash is not None and original_plan_hash == replacement_plan_hash:
        errors.append("no-op replan has the same plan hash")
    del completed_valid_effects  # Effects are checked by task graph policy where their predicates are known.
    return tuple(sorted(set(errors)))


def compile_replan(
    request: ReplanRequest,
    result: ReplanResult,
    *,
    compiler: Any,
    internal_allowlist: Sequence[str] = (),
) -> ReplanResult:
    if result.status not in {"PROPOSED", "COMPILED"}:
        return result
    if result.replacement_goal_spec is None or result.replacement_envelope is None or result.replacement_task_graph is None:
        return ReplanResult(request.request_id, "REPLAN_REJECTED", errors=("replacement is incomplete",))
    report = compiler.compile(result.replacement_goal_spec, result.replacement_envelope, result.replacement_task_graph)
    if not report.ok or report.compiled_graph is None:
        return ReplanResult(request.request_id, "REPLAN_REJECTED", replacement_goal_spec=result.replacement_goal_spec, replacement_envelope=result.replacement_envelope, replacement_task_graph=result.replacement_task_graph, compilation_report=report.to_dict(), errors=tuple(item.message for item in report.errors))
    monotonic_errors = validate_replan_monotonicity(request.original_envelope, result.replacement_envelope, original_goal=request.root_goal_spec, replacement_goal=result.replacement_goal_spec, original_plan_hash=request.current_plan_hash, replacement_plan_hash=report.compiled_graph.plan_hash, completed_valid_effects=request.completed_valid_effects, internal_allowlist=internal_allowlist)
    if monotonic_errors:
        return ReplanResult(request.request_id, "REPLAN_REJECTED", replacement_goal_spec=result.replacement_goal_spec, replacement_envelope=result.replacement_envelope, replacement_task_graph=result.replacement_task_graph, compilation_report=report.to_dict(), errors=monotonic_errors, warnings=tuple(item.message for item in report.warnings))
    return ReplanResult(request.request_id, "COMPILED", replacement_goal_spec=result.replacement_goal_spec, replacement_envelope=result.replacement_envelope, replacement_task_graph=result.replacement_task_graph, compilation_report=report.to_dict(), compiled_graph=report.compiled_graph, warnings=tuple(item.message for item in report.warnings))


def make_replan_request_id(request: ReplanRequest) -> str:
    return "replan_" + digest(request.to_dict())[:24]
