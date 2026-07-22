# ExecutionEnvelope

`ExecutionEnvelope` bounds compilation with target mode, capability allow/deny lists, an S2 risk ceiling, resources, axis-aligned frame-specific workspaces, node/depth/time/tool-call budgets, replan/recovery/retry/no-progress limits, and approval metadata.

The envelope is not a hardware authorization mechanism. It has no `hardware_allowed`, `execute`, operator token, environment, executable, argv, adapter, backend, script, or cwd field. `APPROVAL` is an optional workflow node and cannot bypass HardwareGate or modify authorization.

Empty allowlists resolve to the public registry set only; forbidden capabilities always win. Internal, unsupported, or live-unvalidated hardware capabilities are rejected by the compiler.
