from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .enums import ResourceMode
from .serialization import ContractValidationError, reject_unknown, require_bool, require_enum, require_list, require_string, to_plain


@dataclass(frozen=True)
class ResourceRequirement:
    resource_id: str
    mode: ResourceMode
    description: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "resource_id", require_string(self.resource_id, "resource_id", non_empty=True))
        object.__setattr__(self, "mode", require_enum(self.mode, ResourceMode, "mode"))
        object.__setattr__(self, "description", require_string(self.description, "description", non_empty=True))

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ResourceRequirement":
        reject_unknown(data, ("resource_id", "mode", "description"), ("resource_id", "mode", "description"))
        return cls(data["resource_id"], data["mode"], data["description"])

    def to_dict(self) -> dict[str, Any]:
        return {"resource_id": self.resource_id, "mode": self.mode.value, "description": self.description}


@dataclass(frozen=True)
class StateInvalidation:
    keys: tuple[str, ...]
    reason: str
    scope: str
    requires_reobserve: bool

    def __post_init__(self) -> None:
        keys = tuple(self.keys) if isinstance(self.keys, (list, tuple)) else self.keys
        if not keys or any(not isinstance(key, str) or not key.strip() for key in keys):
            raise ContractValidationError("keys must be a non-empty array of non-empty strings")
        object.__setattr__(self, "keys", keys)
        object.__setattr__(self, "reason", require_string(self.reason, "reason", non_empty=True))
        object.__setattr__(self, "scope", require_string(self.scope, "scope", non_empty=True))
        object.__setattr__(self, "requires_reobserve", require_bool(self.requires_reobserve, "requires_reobserve"))

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "StateInvalidation":
        reject_unknown(data, ("keys", "reason", "scope", "requires_reobserve"), ("keys", "reason", "scope", "requires_reobserve"))
        return cls(tuple(require_list(data["keys"], "keys")), data["reason"], data["scope"], data["requires_reobserve"])

    def to_dict(self) -> dict[str, Any]:
        return to_plain(self)
