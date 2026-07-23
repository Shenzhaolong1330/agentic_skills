# Schemas

本目录定义 harness 的机器可读契约。新 contract 使用 JSON Schema
Draft 2020-12，并由 `agentic_skills_harness.schema_validation` 通过仓库内
相对引用安全加载；schema loader 不访问网络。

- `common.schema.json`: `Pose6D`、`Error6DoF`、`DetectionResult`、`MotionResult`、`GripperResult`、`RobotHealthStatus`、`ResetRecoveryResult`、`StageResult`、`SkillContext`、`TaskResult`。
- `task_pick_tube_insert_rack.schema.json`: `pick_tube_insert_rack` 的状态机、输入和输出字段。
- `error_info.schema.json`、`verification_result.schema.json`、`action_result.schema.json`、`observation_result.schema.json`：通用执行结果和结构化错误。
- `execution_budget.schema.json`、`resource_requirement.schema.json`、`state_invalidation.schema.json`：预算、资源和状态失效 contract。
- `capability_manifest.schema.json`：Capability Manifest v0.2 的外部契约。
- `capabilities/`：实际 manifest entrypoint 使用的最小输入 contract；物理输出统一使用通用 ActionResult，并不表示本阶段已执行物理 verifier。
- `world/`: Draft 2020-12 `EntityRef`, `WorldFact`, `WorldSnapshot`, bounded
  `PredicateSpec`/`PredicateResult`, `VerificationRequest`, and audit schemas.

默认 6DoF 阈值为平移 `0.005 m`、旋转 `0.0872664626 rad`。原任务只明确 5 mm 平移要求，旋转阈值约等于 5 degrees，可由 runner CLI 覆盖。
## Planning schemas

schemas/planning contains Draft 2020-12 GoalSpec, ExecutionEnvelope, InputBinding, TaskGraph, diagnostics, and compiled-plan contracts. They are local-only schemas with closed object roots; execution-control fields are rejected by the typed models and compiler. A compiled graph is descriptive and contains no executable command.
# S7 schemas

Runtime records, attempts, budgets, events, checkpoints, task results, cancellation requests, and human action requests are defined under `schemas/execution/`. Runtime artifacts are canonical JSON and relative to an artifact directory.

# S8/S9 schemas

Recovery context, strategy, decision, attempt/result, replan request/result,
plan lineage, and task definitions are defined under `schemas/recovery/`.
These schemas contain descriptive, serializable data only; executable code,
callbacks, shell commands, adapter objects, and hardware handles are not
valid fields.
