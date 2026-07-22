from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class IssueSeverity(str, Enum):
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"


@dataclass(frozen=True)
class CompilationIssue:
    code: str
    severity: IssueSeverity | str
    path: str
    message: str
    node_id: str | None = None
    capability_id: str | None = None
    details: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "severity", self.severity if isinstance(self.severity, IssueSeverity) else IssueSeverity(str(self.severity)))

    @property
    def is_error(self) -> bool:
        return self.severity == IssueSeverity.ERROR

    def to_dict(self) -> dict[str, Any]:
        return {"code": self.code, "severity": self.severity.value, "path": self.path, "message": self.message, "node_id": self.node_id, "capability_id": self.capability_id, "details": dict(self.details)}


@dataclass(frozen=True)
class CompilationReport:
    compiled_graph: Any | None
    issues: tuple[CompilationIssue, ...] = ()

    @property
    def ok(self) -> bool:
        return self.compiled_graph is not None and not any(item.is_error for item in self.issues)

    @property
    def executable(self) -> bool:
        return False

    def to_dict(self) -> dict[str, Any]:
        return {"ok": self.ok, "executable": False, "issues": [item.to_dict() for item in self.issues], "compiled_graph": None if self.compiled_graph is None else self.compiled_graph.to_dict()}

