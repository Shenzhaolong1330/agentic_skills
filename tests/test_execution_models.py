from __future__ import annotations

from dataclasses import replace

import pytest

from agentic_skills_harness.contracts.enums import NodeStatus
from agentic_skills_harness.execution.models import (
    BudgetUsage, ExecutionOutcome, ExecutionScope, NodeAttempt, NodeRuntimeRecord, TaskExecutionResult,
)


def test_runtime_models_round_trip_and_physical_invariants():
    record = NodeRuntimeRecord("n1", "OBSERVE", status=NodeStatus.SUCCEEDED, visit_count=1, attempt_count=1, attempts=[NodeAttempt("a1", "n1", 1, NodeStatus.SUCCEEDED)])
    result = TaskExecutionResult("r1", "g1", "goal1", "hash", "dry_run", "SUCCEEDED", ExecutionOutcome.PLAN_COMPLETED, ExecutionScope.NONE, True, False, True, True, True, (record,), BudgetUsage(nodes_started=1))
    decoded = TaskExecutionResult.from_dict(result.to_dict())
    assert decoded.to_dict() == result.to_dict()
    assert decoded.physical_execution_performed is False
    assert decoded.physical_goal_verified is False


def test_budget_usage_has_stable_counter_order():
    usage = BudgetUsage(node_visits={"b": 2, "a": 1}, edge_traversals={"e2": 2, "e1": 1})
    assert list(usage.to_dict()["node_visits"]) == ["a", "b"]
    assert BudgetUsage.from_dict(usage.to_dict()).to_dict() == usage.to_dict()


@pytest.mark.parametrize("outcome", list(ExecutionOutcome))
def test_all_execution_outcomes_are_serializable(outcome):
    assert outcome.value == str(outcome.value)
