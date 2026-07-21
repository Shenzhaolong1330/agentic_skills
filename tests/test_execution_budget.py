from __future__ import annotations

import math
import pytest

from agentic_skills_harness.contracts import ExecutionBudget
from agentic_skills_harness.contracts.serialization import ContractValidationError


def test_budget_defaults_and_round_trip():
    budget = ExecutionBudget()
    assert ExecutionBudget.from_dict(budget.to_dict()).to_dict() == budget.to_dict()


@pytest.mark.parametrize("field", ["max_nodes", "max_depth", "max_elapsed_s", "max_tool_calls"])
def test_budget_positive_limits(field):
    values = ExecutionBudget().to_dict()
    values[field] = 0
    with pytest.raises(ContractValidationError):
        ExecutionBudget.from_dict(values)


@pytest.mark.parametrize("field", ["max_replans", "max_recovery_actions", "max_same_error_retries", "no_progress_limit"])
def test_budget_nonnegative_limits(field):
    values = ExecutionBudget().to_dict()
    values[field] = -1
    with pytest.raises(ContractValidationError):
        ExecutionBudget.from_dict(values)


def test_budget_rejects_non_finite():
    values = ExecutionBudget().to_dict()
    values["max_elapsed_s"] = math.inf
    with pytest.raises(ContractValidationError):
        ExecutionBudget.from_dict(values)
