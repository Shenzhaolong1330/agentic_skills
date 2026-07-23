from __future__ import annotations

from typing import Any, Mapping


def remaining_budget(budget: Mapping[str, Any], usage: Mapping[str, Any]) -> dict[str, Any]:
    """Return non-negative residual counters without restoring consumed budget."""

    result: dict[str, Any] = {}
    pairs = (
        ("max_replans", "replans"), ("max_recovery_actions", "recovery_actions"),
        ("max_same_error_retries", "same_error_retries"), ("max_nodes", "nodes_started"),
        ("max_tool_calls", "tool_calls"),
    )
    for limit_key, usage_key in pairs:
        try:
            result[usage_key] = max(0, int(budget.get(limit_key, 0)) - int(usage.get(usage_key, 0)))
        except (TypeError, ValueError):
            result[usage_key] = 0
    return result


__all__ = ["remaining_budget"]
