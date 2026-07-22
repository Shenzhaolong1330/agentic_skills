from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping
import re

from ..contracts.serialization import ContractValidationError, ensure_jsonable, require_bool, require_object, require_string, stable_dumps, utc_now_iso
from ..types import SkillContext, SkillMode


FORBIDDEN_REQUEST_KEYS = {
    "command", "argv", "executable", "script", "shell", "cwd", "env", "environment",
    "adapter", "adapter_id", "backend", "python_path", "reset_script", "client_path",
    "extra_args", "passthrough_args",
}
_SHELL_META = re.compile(r"(?:[;&|<>`]|\$\(|\x00)")


def _walk_security(value: Any, path: str = "$") -> None:
    if isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str):
                raise ContractValidationError(f"{path} keys must be strings")
            if key.lower() in FORBIDDEN_REQUEST_KEYS:
                raise ContractValidationError(f"forbidden dispatch binding field: {path}.{key}")
            _walk_security(item, f"{path}.{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            _walk_security(item, f"{path}[{index}]")
    elif isinstance(value, str) and _SHELL_META.search(value):
        raise ContractValidationError(f"shell metacharacter or null byte in dispatch input at {path}")


@dataclass(frozen=True)
class DispatchRequest:
    capability_id: str
    arguments: Mapping[str, Any] = field(default_factory=dict)
    request_id: str = ""
    artifact_refs: tuple[str, ...] = ()
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "capability_id", require_string(self.capability_id, "capability_id", non_empty=True))
        arguments = ensure_jsonable(require_object(dict(self.arguments), "arguments"), "arguments")
        metadata = ensure_jsonable(require_object(dict(self.metadata), "metadata"), "metadata")
        if len(stable_dumps(metadata).encode("utf-8")) > 8192:
            raise ContractValidationError("metadata exceeds 8192 bytes")
        refs = tuple(self.artifact_refs)
        if any(not isinstance(item, str) or not item.strip() for item in refs):
            raise ContractValidationError("artifact_refs must contain non-empty strings")
        _walk_security(arguments, "$.arguments")
        _walk_security(metadata, "$.metadata")
        _walk_security(list(refs), "$.artifact_refs")
        object.__setattr__(self, "arguments", arguments)
        object.__setattr__(self, "metadata", metadata)
        object.__setattr__(self, "artifact_refs", refs)
        if not self.request_id:
            object.__setattr__(self, "request_id", f"req_{utc_now_iso().replace(':', '').replace('+00:00', 'Z')}")
        else:
            object.__setattr__(self, "request_id", require_string(self.request_id, "request_id", non_empty=True))

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "DispatchRequest":
        if not isinstance(data, Mapping):
            raise ContractValidationError("DispatchRequest must be an object")
        allowed = {"capability_id", "arguments", "request_id", "artifact_refs", "metadata"}
        unknown = sorted(set(data) - allowed)
        if unknown:
            raise ContractValidationError(f"unknown DispatchRequest fields: {unknown}")
        return cls(data["capability_id"], data.get("arguments", {}), data.get("request_id", ""), tuple(data.get("artifact_refs", ())), data.get("metadata", {}))

    def to_dict(self) -> dict[str, Any]:
        return {"capability_id": self.capability_id, "arguments": dict(self.arguments), "request_id": self.request_id, "artifact_refs": list(self.artifact_refs), "metadata": dict(self.metadata)}


@dataclass(frozen=True)
class DispatchContext:
    mode: SkillMode | str = SkillMode.MOCK
    hardware_allowed: bool = False
    execute: bool = False
    artifact_dir: str | Path = "/tmp/agentic_skills_runs"
    fixture_roots: tuple[str | Path, ...] = ()
    recovery: bool = False
    allow_internal: bool = False
    metadata: Mapping[str, Any] = field(default_factory=dict)
    trace_dir: str | Path | None = None

    def __post_init__(self) -> None:
        mode = self.mode if isinstance(self.mode, SkillMode) else SkillMode(str(self.mode))
        object.__setattr__(self, "mode", mode)
        object.__setattr__(self, "hardware_allowed", require_bool(self.hardware_allowed, "hardware_allowed"))
        object.__setattr__(self, "execute", require_bool(self.execute, "execute"))
        object.__setattr__(self, "artifact_dir", str(Path(self.artifact_dir).expanduser()))
        object.__setattr__(self, "fixture_roots", tuple(str(Path(item).expanduser()) for item in self.fixture_roots))
        object.__setattr__(self, "metadata", ensure_jsonable(require_object(dict(self.metadata), "metadata"), "metadata"))
        if len(stable_dumps(self.metadata).encode("utf-8")) > 8192:
            raise ContractValidationError("context metadata exceeds 8192 bytes")

    @classmethod
    def from_skill_context(cls, context: SkillContext) -> "DispatchContext":
        return cls(mode=context.mode, hardware_allowed=context.hardware_allowed, execute=context.execute, artifact_dir=context.artifact_dir or "/tmp/agentic_skills_runs", metadata={"run_id": context.run_id})


@dataclass(frozen=True)
class InvocationPlan:
    request_id: str
    capability_id: str
    adapter_id: str
    mode: str
    executable: str
    argv: tuple[str, ...]
    cwd: str
    timeout_s: float
    artifact_dir: str
    requires_hardware: bool
    side_effects: tuple[str, ...]
    gate_decision: Mapping[str, Any]
    planned_only: bool = False

    def to_public_dict(self) -> dict[str, Any]:
        executable_name = Path(self.executable).name
        public_argv = [executable_name if index == 0 else (Path(value).name if value == self.executable else value) for index, value in enumerate(self.argv)]
        return {
            "request_id": self.request_id, "capability_id": self.capability_id, "adapter_id": self.adapter_id,
            "mode": self.mode, "argv": public_argv, "timeout_s": self.timeout_s,
            "artifact_dir": Path(self.artifact_dir).name, "requires_hardware": self.requires_hardware,
            "side_effects": list(self.side_effects), "gate_decision": dict(self.gate_decision), "planned_only": self.planned_only,
        }

    def to_dict(self) -> dict[str, Any]:
        return {"request_id": self.request_id, "capability_id": self.capability_id, "adapter_id": self.adapter_id, "mode": self.mode, "executable": self.executable, "argv": list(self.argv), "cwd": self.cwd, "timeout_s": self.timeout_s, "artifact_dir": self.artifact_dir, "requires_hardware": self.requires_hardware, "side_effects": list(self.side_effects), "gate_decision": dict(self.gate_decision), "planned_only": self.planned_only}
