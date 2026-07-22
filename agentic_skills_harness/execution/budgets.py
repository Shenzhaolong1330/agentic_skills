from __future__ import annotations

import math
from typing import Any, Mapping

from ..contracts.enums import ErrorCode
from .models import BudgetUsage
from .errors import ExecutionError


class BudgetTracker:
    def __init__(self, budget: Mapping[str, Any], *, monotonic_clock: Any) -> None:
        self.budget = {str(key): value for key, value in dict(budget).items()}
        self.clock = monotonic_clock
        self.usage = BudgetUsage()
        for key, value in self.budget.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)) or float(value) < 0:
                raise ValueError(f"budget {key} must be finite and non-negative")

    def elapsed(self, started_monotonic: float) -> float:
        self.usage.elapsed_s = max(0.0, float(self.clock()) - started_monotonic)
        return self.usage.elapsed_s

    def check(self, started_monotonic: float, *, node_id: str | None = None, edge_id: str | None = None) -> None:
        elapsed = self.elapsed(started_monotonic)
        if elapsed > float(self.budget.get("max_elapsed_s", float("inf"))):
            raise ExecutionError(ErrorCode.TASK_TIMEOUT, "task elapsed-time budget exceeded", {"elapsed_s": elapsed})
        if self.usage.tool_calls >= int(self.budget.get("max_tool_calls", 2**63 - 1)):
            raise ExecutionError(ErrorCode.BUDGET_EXCEEDED, "tool-call budget exceeded", {"tool_calls": self.usage.tool_calls})
        if self.usage.recovery_actions > int(self.budget.get("max_recovery_actions", 2**63 - 1)):
            raise ExecutionError(ErrorCode.BUDGET_EXCEEDED, "recovery-action budget exceeded", {"recovery_actions": self.usage.recovery_actions})
        if self.usage.same_error_retries > int(self.budget.get("max_same_error_retries", 2**63 - 1)):
            raise ExecutionError(ErrorCode.BUDGET_EXCEEDED, "same-error retry budget exceeded", {"same_error_retries": self.usage.same_error_retries})
        if node_id is not None and self.usage.node_visits.get(node_id, 0) > 0:
            pass
        if edge_id is not None and self.usage.edge_traversals.get(edge_id, 0) < 0:
            raise ExecutionError(ErrorCode.BUDGET_EXCEEDED, "negative edge traversal count")

    def start_node(self, node_id: str, max_visits: int, started_monotonic: float) -> None:
        self.check(started_monotonic)
        if self.usage.nodes_started >= int(self.budget.get("max_nodes", 2**63 - 1)):
            raise ExecutionError(ErrorCode.BUDGET_EXCEEDED, "node budget exceeded", {"nodes_started": self.usage.nodes_started})
        visits = self.usage.node_visits.get(node_id, 0) + 1
        if visits > max_visits:
            raise ExecutionError(ErrorCode.BUDGET_EXCEEDED, "node visit budget exceeded", {"node_id": node_id, "visits": visits, "max_visits": max_visits})
        self.usage.node_visits[node_id] = visits
        self.usage.nodes_started += 1

    def complete_node(self, node_id: str) -> None:
        self.usage.unique_nodes_completed += 1

    def add_tool_call(self, started_monotonic: float) -> None:
        self.check(started_monotonic)
        if self.usage.tool_calls + 1 > int(self.budget.get("max_tool_calls", 2**63 - 1)):
            raise ExecutionError(ErrorCode.BUDGET_EXCEEDED, "tool-call budget exceeded")
        self.usage.tool_calls += 1

    def add_recovery_action(self) -> None:
        self.usage.recovery_actions += 1
        if self.usage.recovery_actions > int(self.budget.get("max_recovery_actions", 2**63 - 1)):
            raise ExecutionError(ErrorCode.BUDGET_EXCEEDED, "recovery-action budget exceeded")

    def add_retry(self) -> None:
        self.usage.same_error_retries += 1
        if self.usage.same_error_retries > int(self.budget.get("max_same_error_retries", 2**63 - 1)):
            raise ExecutionError(ErrorCode.BUDGET_EXCEEDED, "same-error retry budget exceeded")

    def traverse_edge(self, edge_id: str, maximum: int | None) -> None:
        count = self.usage.edge_traversals.get(edge_id, 0) + 1
        if maximum is not None and count > maximum:
            raise ExecutionError(ErrorCode.BUDGET_EXCEEDED, "edge traversal budget exceeded", {"edge_id": edge_id, "traversals": count, "max_traversals": maximum})
        self.usage.edge_traversals[edge_id] = count

    def restore(self, usage: BudgetUsage) -> None:
        if usage.nodes_started < 0 or usage.tool_calls < 0 or usage.unique_nodes_completed < 0 or any(value < 0 for value in usage.node_visits.values()) or any(value < 0 for value in usage.edge_traversals.values()):
            raise ExecutionError(ErrorCode.CHECKPOINT_CORRUPT, "checkpoint contains negative budget counters")
        self.usage = usage
