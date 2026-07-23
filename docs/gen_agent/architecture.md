# Gen-Agent architecture (S0-S9)

## 范围

本阶段只冻结通用机器人 Agent 的安全基线，并收敛当前仓库的真机硬件入口。它不提前实现完整的通用规划系统，也不改变现有 `atomic`、`procedure`、`task` 三层 skill 分层。

后续流程中，Codex 负责理解用户目标、读取 skill 契约并生成结构化计划；Harness 负责授权判定、执行边界、trace 记录、健康检查和恢复约束。Codex 不应直接获得任意 Shell 执行能力，也不应绕过 Harness 直接拼接低层真机命令。

## 本阶段稳定边界

- `SkillContext` 保留 `mode`、`execute` 和 `hardware_allowed`。
- 所有硬件能力经过 `HardwareGate` 的共享判定函数。
- `CommandRunner` 只在 live gate 允许后调用命令；非 live 硬件能力只写计划。
- recovery 仍由 manifest entrypoint 和 `allowed_as_recovery` 共同约束。
- trace 至少记录上下文、manifest 快照、命令计划和 gate decision。
- mock、dry-run、artifact 回放以及本阶段的测试均不访问真实设备。

## S2/S3 contract boundary

`contracts/` contains task-independent `ActionResult`, `ObservationResult`,
`VerificationResult`, `ErrorInfo`, `ExecutionBudget`, resource requirements,
and invalidation declarations. Legacy task-specific results remain available
to the current runner and are not silently converted.

`CapabilityRegistry` reads Manifest v0.2 and local schemas only. It is not a
dispatcher and does not import, start, or invoke an entrypoint. `CAPABILITY_INDEX.md`
is a deterministic public view of the same manifest entries.

## S4/S5 boundary

`CapabilityDispatcher` accepts typed `DispatchRequest` values and selects fixed
adapters internally. It performs visibility, schema, path, gate, timeout,
output, error, result, and trace handling; it does not perform top-level goal
verification. `WorldStateStore` records provenance-bearing facts,
`PredicateEngine` evaluates a closed predicate language, and
`InvalidationEngine` applies platform reset/recovery invalidation.

`VerifierEngine` distinguishes output structure, controller arrival, observed
effect, and explicit goal verification. Limited physical verifiers remain
limited; no output schema or command completion is physical success.

## 后续方向（非 S0/S1 实现内容）

S6 GoalSpec/TaskGraph Compiler, S7 Graph Executor, S8 Recovery Engine, and
S9 offline task migration are implemented under the same Registry, Gate,
Dispatcher, and verifier boundaries. Natural-language Planner, Memory, and
true physical acceptance remain future work; later layers must not gain
arbitrary command execution.
## S4.5/S6 boundary

Fixed adapters can produce reviewable dry-run InvocationPlans but do not imply live support. GoalSpec, ExecutionEnvelope, and TaskGraph are authorization-neutral contracts. The static compiler produces no commands and performs no execution; S7 and later runtime stages are not part of this phase.
# S7 runtime boundary

The bounded Graph Executor consumes only `CompiledTaskGraph`, calls capabilities only through `CapabilityDispatcher`, and persists hash-chained events plus atomic checkpoints. S7 supports mock, dry-run, and artifact replay only; live and physical execution remain disabled.
# S8 recovery boundary

The Recovery Engine consumes typed failure context and a committed remainder,
selects only first-party strategies through policy/platform guards, and
executes recovery templates as ordinary bounded graphs. Remainder replanning
must pass monotonicity checks before replacing a plan. Recovery events and
lineage are nested in the same checkpoint and hash chain. No recovery path
constructs or dispatches arbitrary shell/code, and offline recovery never
performs physical execution.

# S9 task boundary

Task definitions supply task-scoped private capabilities, GoalSpec,
ExecutionEnvelope, explicit graph, recovery policy, and compatibility mapping.
The migrated pick-tube task is an offline fixture implementation; the legacy
live entrypoint remains independently gated and is not enabled by the new
runtime.
