# Gen-Agent S0/S1 架构基线

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

## 后续方向（非 S0/S1 实现内容）

后续阶段可在不破坏本基线的前提下引入 capability registry、固定 dispatcher、world state、task graph 和 memory。它们必须复用当前 Gate 与 manifest 契约，不能让 Planner 改写安全不变量或获得任意命令执行能力。

