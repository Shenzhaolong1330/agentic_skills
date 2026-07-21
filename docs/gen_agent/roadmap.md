# Gen-Agent roadmap

当前分支冻结了通用机器人 Agent 的安全基线，并收敛当前仓库的真机硬件入口。S2/S3 增加了通用 contract、Manifest v0.2 和只读 Registry，但不改变现有 `atomic`、`procedure`、`task` 三层 skill 分层。

## 阶段状态

- S0: PASS — baseline and hardware authorization model frozen.
- S1: PASS — live hardware access is gated by `hardware_allowed`.
- S2: implemented in this commit — generic execution types, structured errors, and schemas.
- S3: implemented in this commit — Manifest v0.2, read-only Registry, and public capability index.
- S4: not implemented — Dispatcher.
- S5: not implemented — planning/world-state integration.

后续流程中，Codex 负责理解用户目标、读取 capability 契约并生成结构化计划；Harness 负责授权判定、执行边界、trace 记录、健康检查和恢复约束。Codex 不应直接获得任意 Shell 执行能力，也不应绕过 Harness 直接拼接低层真机命令。

后续阶段可在不破坏本基线的前提下引入固定 Dispatcher、world state、task graph 和 memory。它们必须复用当前 Registry、Gate 与 Manifest 契约，不能让 Planner 改写安全不变量或获得任意命令执行能力。
