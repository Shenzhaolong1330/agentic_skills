from __future__ import annotations

import pytest

from agentic_skills_harness.contracts.enums import NodeStatus
from agentic_skills_harness.execution.errors import InvalidStateTransition
from agentic_skills_harness.execution.state_machine import NodeStateMachine


def test_valid_lifecycle_and_bounded_retry_transition():
    machine = NodeStateMachine()
    for state in (NodeStatus.READY, NodeStatus.RUNNING, NodeStatus.FAILED, NodeStatus.READY, NodeStatus.RUNNING, NodeStatus.SUCCEEDED):
        machine.transition(state)
    assert machine.status is NodeStatus.SUCCEEDED


@pytest.mark.parametrize("source,target", [(NodeStatus.SUCCEEDED, NodeStatus.RUNNING), (NodeStatus.CANCELLED, NodeStatus.RUNNING), (NodeStatus.SKIPPED, NodeStatus.RUNNING), (NodeStatus.PENDING, NodeStatus.RUNNING)])
def test_invalid_lifecycle_transition_is_rejected(source, target):
    machine = NodeStateMachine(source)
    with pytest.raises(InvalidStateTransition):
        machine.transition(target)
