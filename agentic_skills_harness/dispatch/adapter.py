from __future__ import annotations

from abc import ABC, abstractmethod
import json
from pathlib import Path
import sys
from typing import Any, Protocol

from ..capability import CapabilityContract
from .models import DispatchContext, DispatchRequest, InvocationPlan


class CapabilityAdapter(Protocol):
    adapter_id: str

    def supports(self, capability: CapabilityContract) -> bool: ...
    def build_plan(self, request: DispatchRequest, capability: CapabilityContract, context: DispatchContext) -> InvocationPlan: ...


class UnsupportedAdapter:
    adapter_id = "unsupported"

    def supports(self, capability: CapabilityContract) -> bool:
        return False

    def build_plan(self, request: DispatchRequest, capability: CapabilityContract, context: DispatchContext) -> InvocationPlan:
        raise ValueError(f"capability is unsupported: {capability.capability_id}")


class FixedEntrypointAdapter:
    """Builds argv from a fixed first-party binding and typed known fields."""

    adapter_id = "fixed_entrypoint"

    def __init__(self, repo_root: str | Path) -> None:
        self.repo_root = Path(repo_root).resolve()

    def supports(self, capability: CapabilityContract) -> bool:
        return capability.type in {"python", "shell"} and bool(capability.path)

    def _binding(self, capability: CapabilityContract) -> tuple[str, list[str]]:
        path = (self.repo_root / capability.path).resolve()
        try:
            path.relative_to(self.repo_root)
        except ValueError as exc:
            raise ValueError("entrypoint binding escapes repository") from exc
        if capability.type == "python" and path.suffix == ".py":
            return str(path), [sys.executable, str(path)]
        return str(path), [str(path)]

    @staticmethod
    def _typed_args(arguments: dict[str, Any]) -> list[str]:
        values: list[str] = []
        mapping = (("operation", "--command"), ("side", "--side"), ("holder_side", "--holder-side"), ("frame", "--frame"), ("source", "--source"))
        for key, flag in mapping:
            if key in arguments:
                values.extend([flag, str(arguments[key])])
        if "xyz_m" in arguments:
            values.extend(["--xyz", ",".join(str(value) for value in arguments["xyz_m"])])
        if "rotvec_rad" in arguments and arguments["rotvec_rad"] is not None:
            values.extend(["--rotvec", ",".join(str(value) for value in arguments["rotvec_rad"])])
        return values

    def build_plan(self, request: DispatchRequest, capability: CapabilityContract, context: DispatchContext) -> InvocationPlan:
        executable, argv = self._binding(capability)
        argv.extend(self._typed_args(dict(request.arguments)))
        if context.mode.value != "mock":
            argv.extend(["--mode", context.mode.value])
        if context.hardware_allowed:
            argv.append("--hardware-allowed")
        if context.execute and capability.execute_flag and capability.execute_flag not in argv:
            argv.append(capability.execute_flag)
        return InvocationPlan(
            request_id=request.request_id, capability_id=capability.capability_id, adapter_id=self.adapter_id,
            mode=context.mode.value, executable=executable, argv=tuple(argv), cwd=str(self.repo_root), timeout_s=capability.timeout_s,
            artifact_dir=str(Path(context.artifact_dir).resolve()), requires_hardware=capability.requires_hardware,
            side_effects=capability.physical_side_effects, gate_decision={}, planned_only=capability.dispatch_support == "plan_only",
        )
