from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping

from ..contracts.serialization import ContractValidationError, ensure_jsonable, reject_unknown, require_string
from .policies import reject_execution_fields, require_id


class BindingSource(str, Enum):
    LITERAL = "LITERAL"
    NODE_OUTPUT = "NODE_OUTPUT"
    WORLD_FACT = "WORLD_FACT"


def _pointer(value: str, field: str) -> str:
    if not isinstance(value, str) or (value and not value.startswith("/")) or "[" in value or "." in value:
        raise ContractValidationError(f"{field} must be a JSON Pointer")
    return value


@dataclass(frozen=True)
class InputBinding:
    binding_id: str
    target_argument_path: str
    source_type: BindingSource | str
    source_node_id: str | None = None
    source_output_path: str | None = None
    world_fact_selector: Mapping[str, Any] | None = None
    literal: Any = None
    expected_schema_ref: str | None = None
    constraints: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "binding_id", require_id(self.binding_id, "binding_id"))
        object.__setattr__(self, "target_argument_path", _pointer(self.target_argument_path, "target_argument_path"))
        source = self.source_type if isinstance(self.source_type, BindingSource) else BindingSource(str(self.source_type))
        object.__setattr__(self, "source_type", source)
        if source == BindingSource.NODE_OUTPUT:
            if not self.source_node_id or not self.source_output_path:
                raise ContractValidationError("NODE_OUTPUT binding requires source_node_id and source_output_path")
            object.__setattr__(self, "source_node_id", require_id(self.source_node_id, "source_node_id"))
            object.__setattr__(self, "source_output_path", _pointer(self.source_output_path, "source_output_path"))
        elif source == BindingSource.WORLD_FACT:
            if not isinstance(self.world_fact_selector, Mapping) or not self.world_fact_selector:
                raise ContractValidationError("WORLD_FACT binding requires world_fact_selector")
            selector = ensure_jsonable(dict(self.world_fact_selector), "world_fact_selector")
            reject_execution_fields(selector, "world_fact_selector")
            object.__setattr__(self, "world_fact_selector", selector)
        elif source == BindingSource.LITERAL:
            object.__setattr__(self, "literal", ensure_jsonable(self.literal, "literal"))
        if self.expected_schema_ref is not None:
            object.__setattr__(self, "expected_schema_ref", require_string(self.expected_schema_ref, "expected_schema_ref", non_empty=True))
        constraints = ensure_jsonable(dict(self.constraints), "binding constraints")
        reject_execution_fields(constraints, "constraints")
        object.__setattr__(self, "constraints", constraints)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "InputBinding":
        allowed = ("binding_id", "target_argument_path", "source_type", "source_node_id", "source_output_path", "world_fact_selector", "literal", "expected_schema_ref", "constraints")
        reject_unknown(data, allowed, ("binding_id", "target_argument_path", "source_type"))
        return cls(data["binding_id"], data["target_argument_path"], data["source_type"], data.get("source_node_id"), data.get("source_output_path"), data.get("world_fact_selector"), data.get("literal"), data.get("expected_schema_ref"), data.get("constraints", {}))

    def to_dict(self) -> dict[str, Any]:
        return {"binding_id": self.binding_id, "target_argument_path": self.target_argument_path, "source_type": self.source_type.value, "source_node_id": self.source_node_id, "source_output_path": self.source_output_path, "world_fact_selector": None if self.world_fact_selector is None else dict(self.world_fact_selector), "literal": self.literal, "expected_schema_ref": self.expected_schema_ref, "constraints": dict(self.constraints)}
