from __future__ import annotations

from typing import Any

from ..capability import CapabilityContract
from ..registry import CapabilityNotFoundError, CapabilityRegistry
from .adapter import CapabilityAdapter, UnsupportedAdapter


class AdapterNotFoundError(KeyError):
    pass


class AdapterRegistry:
    """First-party capability-to-adapter bindings; never user selectable."""

    def __init__(self, capability_registry: CapabilityRegistry, adapters: dict[str, CapabilityAdapter] | None = None) -> None:
        self.capability_registry = capability_registry
        self._by_id: dict[str, CapabilityAdapter] = dict(adapters or {})
        for capability_id in self._by_id:
            capability_registry.require(capability_id)

    def register(self, capability_id: str, adapter: CapabilityAdapter) -> None:
        self.capability_registry.require(capability_id)
        if capability_id in self._by_id:
            raise ValueError(f"adapter already registered: {capability_id}")
        self._by_id[capability_id] = adapter

    def get(self, capability_id: str) -> CapabilityAdapter | None:
        return self._by_id.get(capability_id)

    def require(self, capability_id: str) -> CapabilityAdapter:
        if self.capability_registry.get(capability_id) is None:
            raise CapabilityNotFoundError(capability_id)
        adapter = self.get(capability_id)
        if adapter is None:
            raise AdapterNotFoundError(capability_id)
        return adapter

    def validate_against_capability_registry(self) -> list[str]:
        errors: list[str] = []
        for capability_id, adapter in self._by_id.items():
            capability = self.capability_registry.get(capability_id)
            if capability is None:
                errors.append(f"unknown capability: {capability_id}")
            elif not adapter.supports(capability) and not isinstance(adapter, UnsupportedAdapter):
                errors.append(f"adapter does not support {capability_id}")
        return errors

    def list_support(self, *, include_internal: bool = False) -> tuple[dict[str, Any], ...]:
        capabilities = self.capability_registry.list(include_internal=include_internal, include_legacy=include_internal)
        result = []
        for capability in capabilities:
            adapter = self.get(capability.capability_id)
            status = "UNSUPPORTED" if adapter is None or isinstance(adapter, UnsupportedAdapter) else capability.dispatch_support.upper()
            result.append({"capability_id": capability.capability_id, "visibility": capability.visibility, "adapter_id": getattr(adapter, "adapter_id", None), "dispatch_support": status})
        return tuple(result)
