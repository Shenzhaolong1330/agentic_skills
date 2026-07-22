from __future__ import annotations

from ..contracts.enums import NodeStatus
from .errors import InvalidStateTransition


ALLOWED_TRANSITIONS = {
    NodeStatus.PENDING: {NodeStatus.READY, NodeStatus.SKIPPED, NodeStatus.CANCELLED},
    NodeStatus.READY: {NodeStatus.RUNNING, NodeStatus.CANCELLED},
    NodeStatus.RUNNING: {NodeStatus.SUCCEEDED, NodeStatus.FAILED, NodeStatus.CANCELLED},
    NodeStatus.FAILED: {NodeStatus.READY},
    NodeStatus.SUCCEEDED: set(),
    NodeStatus.SKIPPED: set(),
    NodeStatus.CANCELLED: set(),
}


class NodeStateMachine:
    def __init__(self, status: NodeStatus | str = NodeStatus.PENDING) -> None:
        self.status = status if isinstance(status, NodeStatus) else NodeStatus(status)

    def transition(self, target: NodeStatus | str) -> NodeStatus:
        next_status = target if isinstance(target, NodeStatus) else NodeStatus(target)
        if next_status not in ALLOWED_TRANSITIONS[self.status]:
            raise InvalidStateTransition(self.status.value, next_status.value)
        self.status = next_status
        return self.status

    def can_transition(self, target: NodeStatus | str) -> bool:
        next_status = target if isinstance(target, NodeStatus) else NodeStatus(target)
        return next_status in ALLOWED_TRANSITIONS[self.status]
