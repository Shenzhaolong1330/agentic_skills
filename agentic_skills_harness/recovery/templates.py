from __future__ import annotations

"""Fixed recovery subgraph templates.

Templates produce ordinary :class:`TaskGraph` values.  They never dispatch a
capability themselves; callers must compile and execute the result through
the normal S6/S7 runtime.
"""

from dataclasses import dataclass
from typing import Any, Mapping

from ..planning.graph import EdgeCondition, GraphEdge, GraphNode, NodeKind, TaskGraph
from ..planning.goals import GoalKind, GoalSpec
from ..planning.envelope import ExecutionEnvelope
from ..world.predicates import PredicateSpec
from .models import RecoveryContext, RecoveryStrategy, RecoveryStrategyId


def _capability_id(context: RecoveryContext, capability_registry: Any | None, *, kind: str | None = None) -> str | None:
    metadata = context.original_envelope.get("metadata", {})
    preferred = metadata.get("reobserve_capability_id") if isinstance(metadata, Mapping) else None
    if preferred and capability_registry is not None and capability_registry.get(preferred) is not None:
        return str(preferred)
    if capability_registry is None:
        return None
    for item in capability_registry.list(include_internal=True, include_legacy=True):
        if kind is None or getattr(item.kind, "value", item.kind) == kind:
            return item.capability_id
    return None


def _linear(graph_id: str, goal_id: str, nodes: list[GraphNode], *, metadata: Mapping[str, Any] | None = None) -> TaskGraph:
    edges = tuple(GraphEdge(f"edge_{index:02d}", nodes[index].node_id, nodes[index + 1].node_id, EdgeCondition.DEFAULT, priority=0) for index in range(len(nodes) - 1))
    return TaskGraph(graph_id, goal_id, nodes[0].node_id, tuple(nodes), edges, (nodes[-1].node_id,), dict(metadata or {}))


@dataclass(frozen=True)
class RecoveryTemplateRegistry:
    capability_registry: Any | None = None

    TEMPLATE_IDS = (
        "reobserve", "safe-retreat-then-reobserve", "controller-recovery-then-reobserve",
        "reset-home-then-reobserve", "request-human", "abort", "recompile-remainder",
    )

    def list(self) -> tuple[str, ...]:
        return self.TEMPLATE_IDS

    def build(self, strategy: RecoveryStrategy | str, context: RecoveryContext | Mapping[str, Any]) -> TaskGraph:
        item = strategy if isinstance(strategy, RecoveryStrategy) else RecoveryStrategy.from_dict(strategy)
        context_value = context if isinstance(context, RecoveryContext) else RecoveryContext.from_dict(context)
        template_id = item.template_id or item.strategy_id.value.lower()
        goal_id = f"recovery_goal_{context_value.run_id}"
        graph_id = f"recovery_graph_{context_value.run_id}_{item.strategy_id.value.lower()}"
        reobserve_id = _capability_id(context_value, self.capability_registry, kind="observation")
        nodes: list[GraphNode] = []
        if template_id == "reobserve":
            nodes.append(self._observe_node(reobserve_id, "reobserve", required=True))
        elif template_id == "safe-retreat-then-reobserve":
            nodes.append(GraphNode("safe_retreat", NodeKind.HUMAN_ACTION, metadata={"recovery_action": "SAFE_RETREAT", "plan_only": True, "requires_reobserve": True}))
            nodes.append(self._observe_node(reobserve_id, "reobserve", required=True))
        elif template_id in {"controller-recovery-then-reobserve", "reset-home-then-reobserve"}:
            capability_id = next(iter(item.required_capability_ids), None)
            if capability_id is not None and self.capability_registry is not None and self.capability_registry.get(capability_id) is not None:
                nodes.append(GraphNode("recovery_action", NodeKind.RECOVER, capability_id=capability_id, arguments={}, metadata={"recovery_strategy_id": item.strategy_id.value, "plan_only": True, "invalidates": ["robot.pose", "robot.state", "gripper.state", "relation.holding", "object.dynamic_pose"], "requires_reobserve": True}))
            else:
                nodes.append(GraphNode("recovery_action", NodeKind.HUMAN_ACTION, metadata={"recovery_action": item.strategy_id.value, "plan_only": True, "invalidates": ["robot.pose", "robot.state", "gripper.state", "relation.holding", "object.dynamic_pose"], "requires_reobserve": True}))
            nodes.append(self._observe_node(reobserve_id, "reobserve_after_reset", required=True))
        elif template_id == "recompile-remainder":
            nodes.append(GraphNode("replan_boundary", NodeKind.CHECK, metadata={"recovery_action": "RECOMPILE_REMAINDER", "requires_reobserve": item.requires_reobserve, "plan_only": True}))
        elif template_id == "request-human":
            nodes.append(GraphNode("request_human", NodeKind.HUMAN_ACTION, metadata={"human_action": "recovery_review", "instructions": "Review the failure and provide a fresh bounded run.", "plan_only": True}))
        elif template_id == "abort":
            nodes.append(GraphNode("abort", NodeKind.CHECK, metadata={"recovery_action": "ABORT", "terminal": True, "plan_only": True}))
        else:
            raise ValueError(f"unknown recovery template: {template_id}")
        return _linear(graph_id, goal_id, nodes, metadata={"recovery_template_id": template_id, "source_plan_hash": context_value.plan_hash, "requires_reobserve": item.requires_reobserve, "plan_only": True})

    @staticmethod
    def _observe_node(capability_id: str | None, node_id: str, *, required: bool) -> GraphNode:
        if capability_id:
            return GraphNode(node_id, NodeKind.OBSERVE, capability_id=capability_id, arguments={}, metadata={"recovery_action": "REOBSERVE", "required": required, "fresh_observation": True, "plan_only": True})
        return GraphNode(node_id, NodeKind.HUMAN_ACTION, metadata={"recovery_action": "REOBSERVE", "required": required, "fresh_observation": True, "plan_only": True, "instructions": "Obtain a fresh observation before resuming."})

    def compile(self, strategy: RecoveryStrategy | str, context: RecoveryContext | Mapping[str, Any], *, compiler: Any, goal: GoalSpec | Mapping[str, Any], envelope: ExecutionEnvelope | Mapping[str, Any]) -> Any:
        graph = self.build(strategy, context)
        return compiler.compile(goal, envelope, graph)
