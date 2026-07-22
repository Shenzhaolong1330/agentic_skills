from __future__ import annotations

from typing import Iterable

from ..planning.compiler import CompiledEdge
from .errors import ExecutionError
from ..contracts.enums import ErrorCode


class DeterministicScheduler:
    """Selects one next edge; it never starts concurrent work."""

    def choose(self, edges: Iterable[CompiledEdge], *, condition: str, error_code: str | None = None) -> CompiledEdge | None:
        candidates = [edge for edge in edges if edge.condition == condition and (not edge.error_codes or error_code in edge.error_codes)]
        if not candidates:
            return None
        candidates.sort(key=lambda edge: (-edge.priority, edge.edge_id))
        if len(candidates) > 1 and candidates[0].priority == candidates[1].priority:
            raise ExecutionError(ErrorCode.NON_DETERMINISTIC_GRAPH, "matching edges have tied priority")
        return candidates[0]
