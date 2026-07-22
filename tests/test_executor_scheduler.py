from __future__ import annotations

import pytest

from agentic_skills_harness.execution.scheduler import DeterministicScheduler
from agentic_skills_harness.execution.errors import ExecutionError
from agentic_skills_harness.planning.compiler import CompiledEdge


def test_scheduler_prefers_priority_then_canonical_id():
    scheduler = DeterministicScheduler()
    edges = [CompiledEdge("b", "n", "b", "SUCCESS", (), None, 1), CompiledEdge("a", "n", "a", "SUCCESS", (), None, 2)]
    assert scheduler.choose(edges, condition="SUCCESS").edge_id == "a"


def test_scheduler_fails_on_tied_matches():
    scheduler = DeterministicScheduler()
    edges = [CompiledEdge("a", "n", "a", "SUCCESS", (), None, 1), CompiledEdge("b", "n", "b", "SUCCESS", (), None, 1)]
    with pytest.raises(ExecutionError):
        scheduler.choose(edges, condition="SUCCESS")
