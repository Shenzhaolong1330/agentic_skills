from __future__ import annotations

from typing import Any, Iterable, Mapping


def readiness_summary(values: Iterable[Any]) -> dict[str, int]:
    summary: dict[str, int] = {}
    for value in values:
        state = getattr(getattr(value, "readiness_state", None), "value", getattr(value, "readiness_state", "UNKNOWN"))
        summary[state] = summary.get(state, 0) + 1
    for state in ("VALIDATED_READ_ONLY", "VALIDATED_ACTION", "VALIDATED_RECOVERY", "HARDWARE_ACCEPTANCE_PENDING", "IMPLEMENTATION_READY", "CONTRACT_ONLY", "DISABLED", "REJECTED"):
        summary.setdefault(state, 0)
    return summary


def offline_safety_summary(*, malicious_payload_count: int, rejected_count: int, backend_calls: int, graph_live_enabled: bool = False) -> dict[str, Any]:
    return {"malicious_payload_count": malicious_payload_count, "malicious_payload_rejected_count": rejected_count, "rejection_rate": (rejected_count / malicious_payload_count) if malicious_payload_count else 1.0, "real_hardware_calls": 0, "graph_live_enabled": graph_live_enabled, "backend_calls_after_rejection": backend_calls}
