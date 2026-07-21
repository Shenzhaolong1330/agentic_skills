# Capability Registry (S3)

`CapabilityRegistry` is read-only metadata access. It loads and validates the
manifest, creates frozen `CapabilityContract` values, sorts IDs
deterministically, filters by kind/visibility/risk/hardware/recovery flags,
and resolves local input/output schemas.

The registry does not execute an entrypoint, import a robot client, access a
camera, start a service, invoke a shell, modify the manifest, or connect to a
network. Schema loading rejects URLs, `file://`, absolute paths, and `..`
escapes; local JSON Pointer fragments are supported.

Stable entry points are:

```python
from agentic_skills_harness.manifest import load_capability_registry

registry = load_capability_registry("skill_manifest.json")
capability = registry.require("motion.move_to_pose")
public = registry.list()
observations = registry.list(kind="observation")
input_schema = registry.resolve_input_schema(capability.capability_id)
```

S4 Dispatcher can depend on `CapabilityContract`, `CapabilityRegistry`, local
schema validation, `ErrorInfo`, `ActionResult`, and the existing S1
`HardwareGate`. Dispatcher, dynamic planning, world state, TaskGraph, and
runtime verifier execution are intentionally not implemented here.
