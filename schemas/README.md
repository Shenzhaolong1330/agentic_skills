# Schemas

本目录定义 task harness 的机器可读契约。

- `common.schema.json`: `Pose6D`、`Error6DoF`、`DetectionResult`、`MotionResult`、`GripperResult`、`RobotHealthStatus`、`ResetRecoveryResult`、`StageResult`、`SkillContext`、`TaskResult`。
- `task_pick_tube_insert_rack.schema.json`: `pick_tube_insert_rack` 的状态机、输入和输出字段。

默认 6DoF 阈值为平移 `0.005 m`、旋转 `0.0872664626 rad`。原任务只明确 5 mm 平移要求，旋转阈值约等于 5 degrees，可由 runner CLI 覆盖。
