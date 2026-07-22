"""Typed, capability-bound dispatch primitives."""

from .adapter import CapabilityAdapter, FixedEntrypointAdapter, UnsupportedAdapter
from .adapter_registry import AdapterRegistry
from .backends import BackendResult, ExecutionBackend, FakeBackend, NoExecutionBackend, SubprocessBackend
from .dispatcher import CapabilityDispatcher, DispatchError
from .models import DispatchContext, DispatchRequest, InvocationPlan
from .path_policy import SafePathPolicy, UnsafePathError

__all__ = [
    "AdapterRegistry", "BackendResult", "CapabilityAdapter", "CapabilityDispatcher",
    "DispatchContext", "DispatchError", "DispatchRequest", "ExecutionBackend",
    "FakeBackend", "FixedEntrypointAdapter", "InvocationPlan", "NoExecutionBackend",
    "SafePathPolicy", "SubprocessBackend", "UnsupportedAdapter", "UnsafePathError",
]
