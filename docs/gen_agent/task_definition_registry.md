# TaskDefinition and registry

Task-specific orchestration is registered as a `TaskDefinition`, not embedded
in a generic executor. A definition supplies a stable task id/version,
`GoalSpec`, `ExecutionEnvelope`, internal capability allowlist, explicit
`TaskGraph`, recovery policy, supported offline modes, and a compatibility
mapper for legacy results.

`TaskDefinitionRegistry` resolves only checked-in definitions. The
`pick_tube_insert_rack` definition compiles through a trusted
`TaskCompilationContext`; internal compute, fixture observation, and plan
actions are private to that task and cannot be requested through the public
capability registry. The graph is deliberately expanded into observable,
computational, action-plan, and verification nodes so that each stage has a
checkpoint and recovery boundary.

The task CLI and legacy runner route `mock`, `dry_run`, and `from_artifacts`
through this definition. The legacy output shape is retained by an explicit
compatibility mapper. Live mode remains outside this migration and is rejected
by the agentic task CLI.
