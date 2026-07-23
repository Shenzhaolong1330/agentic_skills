from __future__ import annotations

from typing import Any, Mapping

from ..planning.canonical import digest
from .models import PlanLineage, PlanLineageEntry


def append_lineage(
    lineage: PlanLineage,
    *,
    plan_hash: str,
    reason: str,
    failure_node_id: str,
    error_code: str,
    recovery_strategy_id: str,
    remaining_budget: Mapping[str, Any],
    world_state: Mapping[str, Any],
    created_at: str | None = None,
) -> PlanLineage:
    entry = PlanLineageEntry(
        lineage_index=len(lineage.entries),
        plan_hash=plan_hash,
        parent_plan_hash=lineage.current_plan_hash,
        reason=reason,
        failure_node_id=failure_node_id,
        error_code=error_code,
        recovery_strategy_id=recovery_strategy_id,
        **({"created_at": created_at} if created_at is not None else {}),
        remaining_budget_digest=digest(remaining_budget),
        world_state_digest=digest(world_state),
    )
    return lineage.append(entry)


__all__ = ["PlanLineage", "PlanLineageEntry", "append_lineage"]
