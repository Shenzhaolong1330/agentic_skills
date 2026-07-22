from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

from ..planning.canonical import digest


@dataclass
class ProgressDetector:
    limit: int = 3
    previous: str | None = None
    repeat_count: int = 0
    fingerprints: list[str] = field(default_factory=list)

    @staticmethod
    def fingerprint(*, frontier: str | None, arguments_digest: str, world_digest: str, error_code: str | None, goal_satisfied: bool | None) -> str:
        return digest({"frontier": frontier, "arguments_digest": arguments_digest, "world_digest": world_digest, "error_code": error_code, "goal_satisfied": goal_satisfied})

    def observe(self, fingerprint: str, *, progressed: bool = False) -> bool:
        if progressed or self.previous != fingerprint:
            self.previous = fingerprint
            self.repeat_count = 0
        else:
            self.repeat_count += 1
        self.fingerprints.append(fingerprint)
        return self.limit > 0 and self.repeat_count >= self.limit

    def to_dict(self) -> dict[str, Any]:
        return {"previous": self.previous, "repeat_count": self.repeat_count, "fingerprints": list(self.fingerprints[-32:])}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any], limit: int) -> "ProgressDetector":
        return cls(limit, data.get("previous"), int(data.get("repeat_count", 0)), list(data.get("fingerprints", ())))
