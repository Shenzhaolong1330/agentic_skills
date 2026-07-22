"""First-party, capability-specific dispatch bindings.

The modules in this package deliberately contain no generic shell or script
execution surface.  A binding is selected by the registry, never by a
DispatchRequest.
"""

from .fixed import FirstPartyFixedAdapter, build_fixed_adapter_registry
from .presets import PresetCatalog, PresetSpec, default_preset_catalog

__all__ = [
    "FirstPartyFixedAdapter",
    "PresetCatalog",
    "PresetSpec",
    "build_fixed_adapter_registry",
    "default_preset_catalog",
]
