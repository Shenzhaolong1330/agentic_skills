from __future__ import annotations

from typing import Any

from agentic_skills_harness.contracts.enums import ActionStatus
from agentic_skills_harness.contracts.results import ActionResult, ObservationResult
from agentic_skills_harness.contracts.serialization import utc_now_iso

from .capabilities import ACTION_CAPABILITIES, OBSERVE_CAPABILITY


class PickTubeOfflineDispatcher:
    """Deterministic fake dispatcher for mock/dry_run/from_artifacts only."""

    def __init__(self, *, fixture: dict[str, Any] | None = None) -> None:
        self.fixture = dict(fixture or {})
        self.calls: list[str] = []

    def dispatch(self, request: Any, context: Any = None) -> ActionResult | ObservationResult:
        capability_id = request.capability_id if hasattr(request, "capability_id") else str(request["capability_id"])
        self.calls.append(capability_id)
        now = utc_now_iso()
        if capability_id == OBSERVE_CAPABILITY:
            return ObservationResult(capability_id, True, {"fixture": self.fixture.get("name", "default"), "fresh": True, "confidence": 1.0}, now, 300.0, "task_fixture", (), (), {"fixture": True})
        if capability_id in ACTION_CAPABILITIES.values():
            mode = getattr(getattr(context, "mode", None), "value", getattr(context, "mode", "mock"))
            planned = mode == "dry_run"
            return ActionResult(capability_id, ActionStatus.PLANNED_ONLY if planned else ActionStatus.SUCCEEDED, True, False, planned, None, None, False, None, {"simulated": not planned, "planned_only": planned}, (), (), {}, {"fixture": True, "planned_only": planned}, now, now)
        return ActionResult(capability_id, ActionStatus.SUCCEEDED, True, False, True, None, None, False, None, {"simulated": True}, (), (), {}, {"fixture": True}, now, now)


__all__ = ["PickTubeOfflineDispatcher"]
