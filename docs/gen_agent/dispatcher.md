# Typed Capability Dispatcher (S4)

`CapabilityDispatcher` accepts only a capability ID, schema-valid JSON-object
arguments, artifact references, and a `DispatchContext`. Callers cannot provide
commands, argv, executables, scripts, environments, adapters, backends, or
verifier implementations. The AdapterRegistry selects a first-party fixed
binding; the CapabilityRegistry only provides metadata and schemas and never
executes.

`mock`, `dry_run`, and `from_artifacts` never call `SubprocessBackend`. `live`
requires `hardware_allowed`, and physical actions also require `execute`.
Subprocess execution uses fixed argv with `shell=False`; tests use
`FakeBackend`. Artifact paths are constrained to explicit roots and reject
traversal, URLs, null bytes, and symlink escapes. Output schema success is not
physical success, and each dispatch can write a redacted trace.

Current manifest entries are explicitly unsupported until a safe first-party
binding is registered; the inventory is not made “complete” by granting an
arbitrary adapter.
## Fixed Adapter boundary

The dispatcher uses a closed first-party adapter registry. Requests cannot select an adapter/backend or provide execution paths. Hardware entries are plan-only in this phase, and live dispatch rejects capabilities whose manifest mode support is not supported.
