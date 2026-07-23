from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any


class GripperOperation(str, Enum):
    OPEN = "open"
    CLOSE = "close"
    INITIALIZE = "initialize"
    STATUS = "status"


@dataclass(frozen=True)
class GripperCommand:
    def validate(self, *, operation: GripperOperation | str, side: str) -> dict[str, str]:
        operation = operation if isinstance(operation, GripperOperation) else GripperOperation(operation)
        if side not in {"left", "right"}: raise ValueError("side_enum")
        return {"operation": operation.value, "side": side, "may_release": str(operation == GripperOperation.OPEN).lower()}

    def execute(self, controller: Any, *, operation: GripperOperation | str, side: str) -> dict[str, Any]:
        command = self.validate(operation=operation, side=side)
        method = getattr(controller, command["operation"], None)
        if not callable(method): return {"ok": False, "status": "CAPABILITY_UNSUPPORTED", "holding": "UNKNOWN"}
        result = method(side)
        return {"ok": True, "status": "SUCCEEDED", "result": result, "holding": "UNKNOWN" if command["operation"] == "close" else "UNKNOWN", "may_release": command["may_release"] == "true"}
