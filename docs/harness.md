# Harness 设计

## 目标

`agentic_skills_harness` 提供 task/procedure/atomic skill 的统一上下文、manifest、HardwareGate、trace、命令规划、机器人健康检查和自动 reset recovery。Codex 应先读取 `SKILL_INDEX.md` 和 `skill_manifest.json`，再读取具体 SKILL。

## Modes

- `mock`: 使用 mock fixture，不打开相机、不连接机器人、不执行 reset。
- `dry_run`: 生成 command plan，不执行硬件；可用 `--mock-robot-health abnormal` 验证 reset plan。
- `from_artifacts`: 读取用户提供的 JSON artifact，不执行硬件。
- `live`: 只有 `HardwareGate` 通过才允许 subprocess 调用硬件 entrypoint。

## SkillContext / SkillResult / TaskResult

`SkillContext` 包含 `run_id`、`mode`、`execute`、`hardware_allowed`、artifact dir、manifest path、robot server、阈值、reset attempt、held object state。`StageResult` 和 `TaskResult` 会写入 trace，失败时也必须写 `task_result.json`。

## HardwareGate

live 硬件 gate 必须满足：

1. `context.mode == live`
2. `context.hardware_allowed is true`
3. 有移动、夹爪、reset、home、控制器恢复或其他物理状态改变时，`context.execute is true`
4. manifest entrypoint 允许该用途；recovery 还需要 `allowed_as_recovery=true`

只读相机、状态和 health 能力不强制 `execute`。所有判定由同一个无 I/O 授权函数完成，拒绝原因是稳定的 gate reason。

mock/dry_run/from_artifacts 对硬件 entrypoint 返回 `planned_only=true`，不是错误。

## RobotHealthMonitor

健康状态枚举：`UNKNOWN`、`READY`、`WARNING`、`ABNORMAL`、`FAULT`、`ESTOP_OR_UNSAFE`、`UNREACHABLE`。live health check 使用 `atomic-state-dual-franka-reset` 的 `check_and_reset.py status` entrypoint，但必须通过 gate；mock/dry_run/from_artifacts 不连接 RPC。

## ResetRecoveryController

reset 没有被禁用，但只能作为 manifest 允许的 recovery entrypoint。`AUTO_RESET_RECOVERY` 在 abnormal robot state 或 reset guard 需要 reset 时触发。mock 返回 `MOCK_RESET_OK`；dry_run 写 reset command plan 并返回 `PLANNED_ONLY`；live gate 通过后才允许 reset。持管阶段 reset 后默认 abort，原因是 reset 可能移动双臂或改变夹爪/持物状态，需要对象状态重验证。E-stop/unsafe 状态不自动恢复。

## Trace

默认 artifact root：`/tmp/agentic_skills_runs/<run_id>/`。写入：

- `context.json`
- `manifest_snapshot.json`
- `robot_health_preflight.json`
- `reset_recovery_<index>.json`
- `stage_<index>_<STATE>.json`
- `command_plan.json`
- `task_result.json`

运行期 calibration/state 不应写入 repo，除非用户显式把 artifact_dir 指到 repo 内。

## 新增 skill manifest entry

新增 entrypoint 时必须声明 `requires_hardware`、`opens_camera`、`connects_robot_rpc`、`moves_robot`、`controls_gripper`、`default_safe_to_run`、`allowed_as_recovery`、`execute_flag` 和 side effects。这样 Codex 后续不需要全仓搜索即可安全路由。
