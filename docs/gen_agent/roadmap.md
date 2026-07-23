# Gen-Agent roadmap

当前分支冻结了通用机器人 Agent 的安全基线，并收敛当前仓库的真机硬件入口。S2/S3 增加了通用 contract、Manifest v0.2 和只读 Registry，但不改变现有 `atomic`、`procedure`、`task` 三层 skill 分层。

## 阶段状态

- S0: PASS — baseline and hardware authorization model frozen.
- S1: PASS — live hardware access is gated by `hardware_allowed`.
- S2: implemented in this commit — generic execution types, structured errors, and schemas.
- S3: implemented in this commit — Manifest v0.2, read-only Registry, and public capability index.
- S4: implemented — typed Dispatcher, fixed AdapterRegistry, safe path policy,
  fake/no-execution backends, output/error mapping, and trace.
- S5: implemented — World State, bounded predicates, state invalidation, and
  effect/goal verifier foundations.

后续流程中，Codex 负责理解用户目标、读取 capability 契约并生成结构化请求；Harness 负责授权判定、执行边界、trace 记录、World State、验证和恢复约束。Codex 不应直接获得任意 Shell 执行能力，也不应绕过 Harness 直接拼接低层真机命令。

S6 GoalSpec/TaskGraph Compiler、S7 Graph Executor、S8 Recovery Engine 和
S9 离线 task migration 已实现；Planner、Memory 和完整物理验收仍未实现。
## S4.5 and S6 status

S4.5 fixed Adapter plans, S6 static contracts/compiler, S7 Graph Executor,
S8 Recovery Engine, and S9 task migration are implemented offline. No
hardware capability is live-supported or real-hardware validated. S11 Codex
Planner and Memory remain future work.
# S7 status

S7 adds the bounded, recoverable, auditable CompiledTaskGraph Executor. S8
adds deterministic recovery strategy selection, policy guards, recovery
subgraphs, remainder replanning, and plan lineage. S9 migrates the
`pick_tube_insert_rack` task to a registered offline TaskDefinition with
explicit GoalSpec/TaskGraph nodes and a compatibility mapper. Codex Planner,
Memory, and true physical acceptance remain outside this branch.
