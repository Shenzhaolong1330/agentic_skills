from __future__ import annotations

from typing import Any, Mapping


def legacy_output_mapper(result: Any, *, selected_arm: str | None = None, selected_hole: str | None = None) -> dict[str, Any]:
    outcome = getattr(result, "outcome", None)
    outcome_value = getattr(outcome, "value", outcome)
    return {
        "task_state": "complete" if outcome_value in {"GOAL_VERIFIED", "GRAPH_COMPLETED_UNVERIFIED", "PLAN_COMPLETED"} else "failed",
        "completion_flag": outcome_value in {"GOAL_VERIFIED", "GRAPH_COMPLETED_UNVERIFIED", "PLAN_COMPLETED"},
        "physical_verified": False,
        "physical_goal_verified": False,
        "physical_execution_performed": False,
        "stopped_reason": None if outcome_value in {"GOAL_VERIFIED", "GRAPH_COMPLETED_UNVERIFIED", "PLAN_COMPLETED"} else str(outcome_value or "FAILED"),
        "selected_arm": selected_arm,
        "selected_hole": selected_hole,
        "retry_count": int(getattr(getattr(result, "final_execution_result", result), "budget_usage", {}).same_error_retries) if getattr(getattr(result, "final_execution_result", result), "budget_usage", None) is not None else 0,
        "reset_recovery": False,
        "execution_scope": getattr(getattr(result, "final_execution_result", result), "execution_scope", "NONE").value if hasattr(getattr(getattr(result, "final_execution_result", result), "execution_scope", "NONE"), "value") else getattr(getattr(result, "final_execution_result", result), "execution_scope", "NONE"),
        "graph_id": getattr(getattr(result, "final_execution_result", result), "graph_id", None),
        "plan_hash": getattr(getattr(result, "final_execution_result", result), "plan_hash", None),
        "goal_verified": bool(getattr(getattr(result, "final_execution_result", result), "goal_verified", False)),
        "recovery_attempts": len(getattr(result, "recovery_attempts", ())),
        "replan_attempts": len(getattr(result, "replan_attempts", ())),
        "plan_lineage": getattr(result, "plan_lineage", None).to_dict() if getattr(result, "plan_lineage", None) is not None else None,
        "task_execution_result_ref": "task_execution_result.json",
    }


__all__ = ["legacy_output_mapper"]
