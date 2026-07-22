from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

from ..contracts.serialization import ensure_jsonable, require_object, require_string
from ..world.predicates import PredicateSpec


@dataclass(frozen=True)
class VerificationRequest:
    capability_id: str
    predicate: PredicateSpec | None = None
    evidence: tuple[Any, ...] = ()
    arguments: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "capability_id", require_string(self.capability_id, "capability_id", non_empty=True))
        if self.predicate is not None and not isinstance(self.predicate, PredicateSpec):
            object.__setattr__(self, "predicate", PredicateSpec.from_dict(self.predicate))
        object.__setattr__(self, "evidence", tuple(ensure_jsonable(item, "evidence") for item in self.evidence))
        object.__setattr__(self, "arguments", ensure_jsonable(require_object(dict(self.arguments), "arguments"), "arguments"))
