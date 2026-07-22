from __future__ import annotations

import pytest

from agentic_skills_harness.contracts.enums import ErrorCode
from agentic_skills_harness.execution.budgets import BudgetTracker
from agentic_skills_harness.execution.errors import ExecutionError


class Clock:
    value = 0.0

    def __call__(self):
        return self.value


def test_budget_boundaries_are_enforced_before_dispatch():
    clock = Clock()
    tracker = BudgetTracker({"max_nodes": 1, "max_tool_calls": 1, "max_elapsed_s": 10, "max_recovery_actions": 0, "max_same_error_retries": 0}, monotonic_clock=clock)
    tracker.start_node("n", 1, 0.0)
    tracker.add_tool_call(0.0)
    with pytest.raises(ExecutionError) as exc:
        tracker.add_tool_call(0.0)
    assert exc.value.code == ErrorCode.BUDGET_EXCEEDED


def test_elapsed_timeout_uses_injected_monotonic_clock():
    clock = Clock()
    tracker = BudgetTracker({"max_elapsed_s": 1, "max_nodes": 4, "max_tool_calls": 4}, monotonic_clock=clock)
    clock.value = 1.1
    with pytest.raises(ExecutionError) as exc:
        tracker.check(0.0)
    assert exc.value.code == ErrorCode.TASK_TIMEOUT
