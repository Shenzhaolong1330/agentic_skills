# Static TaskGraph Compiler

`TaskGraphCompiler` validates GoalSpec, ExecutionEnvelope, and TaskGraph locally before any execution stage. It checks capability visibility/disposition/mode, schemas, risk, arguments, frames and workspace bounds, resources, verifiers, evidence observability, retries, recovery, budgets, reachability, deterministic edges, cycles, and E-stop routing.

The compiler never calls `CapabilityDispatcher`, `Adapter.build_plan`, a backend, a verifier capability, subprocess, a camera, or robot RPC. `CompiledTaskGraph` contains contract metadata and normalized arguments only; it contains no executable, argv, env, adapter, backend, `hardware_allowed`, or `execute` field. Its hash is canonical and excludes `compiled_at`, so identical inputs have identical hashes.

This is not a Graph Executor. World State and Verifier are not driven by the compiled graph until a later executor phase.
# S7 handoff

The compiler emits plan, manifest, capability-index, input-schema, and output-schema digests. The Executor rechecks these values and the current Registry disposition before any Dispatcher call.
