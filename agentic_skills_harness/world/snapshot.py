from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from ..contracts.serialization import reject_unknown, stable_dumps
from .models import WorldFact


@dataclass(frozen=True)
class WorldSnapshot:
    revision: int
    facts: tuple[WorldFact, ...]

    def __post_init__(self) -> None:
        if isinstance(self.revision, bool) or not isinstance(self.revision, int) or self.revision < 0:
            raise ValueError("snapshot revision must be a non-negative integer")
        object.__setattr__(self, "facts", tuple(sorted(self.facts, key=lambda fact: (fact.fact_id, fact.revision))))

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "WorldSnapshot":
        reject_unknown(data, ("revision", "facts"), ("revision", "facts"))
        return cls(data["revision"], tuple(WorldFact.from_dict(item) for item in data["facts"]))

    def to_dict(self) -> dict[str, Any]:
        return {"revision": self.revision, "facts": [fact.to_dict() for fact in self.facts]}

    def to_json(self) -> str:
        return stable_dumps(self.to_dict())
