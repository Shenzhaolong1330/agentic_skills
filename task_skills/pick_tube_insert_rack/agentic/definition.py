from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Mapping

from agentic_skills_harness.planning import ExecutionEnvelope, GoalSpec, TaskCompilationContext, TaskGraph, TaskGraphCompiler

from .capabilities import COMPUTE_CAPABILITIES, OBSERVE_CAPABILITY, ACTION_CAPABILITIES, TaskCapabilityRegistry, internal_capabilities
from .compatibility import legacy_output_mapper
from .envelope import canonical_envelope
from .goals import canonical_goal_spec
from .graph import canonical_task_graph
from .recovery import TASK_RECOVERY_POLICY_IDS


@dataclass(frozen=True)
class TaskDefinition:
    task_id: str
    task_version: str
    description: str
    goal_builder: Callable[..., GoalSpec]
    envelope_builder: Callable[..., ExecutionEnvelope]
    graph_builder: Callable[..., TaskGraph]
    internal_capability_allowlist: tuple[str, ...]
    recovery_policy_ids: tuple[str, ...]
    legacy_output_mapper: Callable[..., dict[str, Any]]
    supported_modes: tuple[str, ...] = ("mock", "dry_run", "from_artifacts")

    def __post_init__(self) -> None:
        if not self.task_id or not self.task_version or not self.description:
            raise ValueError("TaskDefinition identifiers and description are required")
        if not self.internal_capability_allowlist:
            raise ValueError("TaskDefinition must declare internal capabilities")
        if "live" in self.supported_modes:
            raise ValueError("TaskDefinition cannot enable live mode in S9")

    def build_goal(self, **kwargs: Any) -> GoalSpec:
        return self.goal_builder(**kwargs)

    def build_envelope(self, *, mode: str = "mock", **kwargs: Any) -> ExecutionEnvelope:
        return self.envelope_builder(mode=mode, **kwargs)

    def build_graph(self, **kwargs: Any) -> TaskGraph:
        return self.graph_builder(**kwargs)

    def compilation_context(self) -> TaskCompilationContext:
        return TaskCompilationContext.trusted(self.task_id, self.task_version, self.internal_capability_allowlist)

    def compile(self, registry: Any, manifest: Mapping[str, Any], *, mode: str = "mock", graph_id: str | None = None) -> tuple[Any, TaskCapabilityRegistry, GoalSpec, ExecutionEnvelope, TaskGraph, TaskCompilationContext]:
        if mode not in self.supported_modes:
            raise ValueError(f"unsupported task mode: {mode}")
        task_registry = TaskCapabilityRegistry(registry, internal_capabilities())
        goal = self.build_goal()
        envelope = self.build_envelope(mode=mode)
        graph = self.build_graph(graph_id=graph_id or "pick_tube_insert_rack.single_tube.graph", goal_id=goal.goal_id)
        compiler = TaskGraphCompiler(task_registry, manifest=manifest)
        report = compiler.compile(goal, envelope, graph, compilation_context=self.compilation_context())
        return report, task_registry, goal, envelope, graph, self.compilation_context()


class TaskDefinitionRegistry:
    def __init__(self, definitions: tuple[TaskDefinition, ...] = (), *, production: bool = True) -> None:
        self.production = production
        self._definitions: dict[str, TaskDefinition] = {}
        for definition in definitions:
            self.register(definition, first_party=True)

    def register(self, definition: TaskDefinition, *, first_party: bool = False) -> TaskDefinition:
        if self.production and not first_party:
            raise PermissionError("production TaskDefinitions must be first-party registered")
        if definition.task_id in self._definitions:
            raise ValueError(f"duplicate task definition: {definition.task_id}")
        self._definitions[definition.task_id] = definition
        return definition

    def get(self, task_id: str) -> TaskDefinition | None:
        return self._definitions.get(task_id)

    def require(self, task_id: str) -> TaskDefinition:
        value = self.get(task_id)
        if value is None:
            raise KeyError(task_id)
        return value

    def list(self) -> tuple[TaskDefinition, ...]:
        return tuple(self._definitions[key] for key in sorted(self._definitions))


PICK_TUBE_INSERT_RACK = TaskDefinition(
    task_id="pick_tube_insert_rack", task_version="1.0.0",
    description="Canonical offline single-tube pick, insertion, release, and evidence verification.",
    goal_builder=canonical_goal_spec, envelope_builder=canonical_envelope, graph_builder=canonical_task_graph,
    internal_capability_allowlist=(*COMPUTE_CAPABILITIES, OBSERVE_CAPABILITY, *ACTION_CAPABILITIES.values()),
    recovery_policy_ids=TASK_RECOVERY_POLICY_IDS, legacy_output_mapper=legacy_output_mapper,
)


def default_task_definition_registry() -> TaskDefinitionRegistry:
    return TaskDefinitionRegistry((PICK_TUBE_INSERT_RACK,), production=True)


__all__ = ["PICK_TUBE_INSERT_RACK", "TaskDefinition", "TaskDefinitionRegistry", "default_task_definition_registry"]
