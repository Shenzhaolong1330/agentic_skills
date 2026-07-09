# Conventions

## 核心决策

这个仓库先按“一个可运行项目一个目录”的方式组织，不做提前抽象。

当前 `atomic_skills/object_locator` 已经是完整项目，所以保持自包含。按分层语义，它属于 atomic perception skill：

```text
atomic_skills/object_locator/
├── src/object_locator/                         # 核心代码
├── tests/                                      # 项目测试
├── calibration/                                # 项目标定
├── config*.yaml                                # 项目配置
└── skills/atomic-perception-locate-object-3d/  # Codex 使用说明
```

## Skill 分层

- Atomic skill: 一个底层能力，例如识别定位、点到点运动、夹爪开合。
- Procedure skill: 固定动作流程，例如固定 handover、回 home、固定扫描。
- Task skill: 组合感知、运动和夹爪完成任务，例如抓起 vial、放入试管架。

组合 skill 只编排原子能力，不复制原子能力的实现。

## Skill 格式

一个 Codex skill 只放 Codex 需要的操作信息，不放核心算法。

推荐最小结构：

```text
skill-name/
├── SKILL.md
├── references/
│   ├── config-schema.md
│   ├── output-format.md
│   └── debugging.md
└── scripts/
    └── run_once.sh
```

`SKILL.md` 应该说明：

- 什么时候使用这个 skill。
- 项目根目录怎么定位。
- 默认运行什么命令。
- 输出文件在哪里。
- 常见失败看哪些 debug 文件。
- 改代码后跑哪些测试。

## 命名

Codex skill 文件夹指 `*/skills/<skill-name>/`，目录名必须和 `SKILL.md` frontmatter `name` 一致。

- 新增物理 skill 按类型分组：`atomic_skills/`、`procedure_skills/`、`task_skills/`。
- 分组下的 skill 目录用 `snake_case`，不加 `_skill` 后缀，例如 `object_locator`、`franka_motion`、`handover_fixed`、`pick_vial`。
- Atomic skill folder: `atomic_skills/<domain_or_capability>`，例如 `atomic_skills/object_locator`、`atomic_skills/franka_motion`。
- Procedure skill folder: `procedure_skills/<procedure_name>`，例如 `procedure_skills/handover_fixed`、`procedure_skills/go_home`。
- Task skill folder: `task_skills/<verb_object>` 或 `task_skills/<verb_object_to_target>`，例如 `task_skills/pick_vial`、`task_skills/place_vial_in_rack`。
- Python 包用 `snake_case`，例如 `object_locator`、`franka_motion`、`pick_vial`。
- Codex skill 目录和 frontmatter `name` 用带层级前缀的 `kebab-case`。
- Atomic perception: `atomic-perception-<capability>`，例如 `atomic-perception-locate-object-3d`。
- Atomic motion: `atomic-motion-<robot>-<capability>`，例如 `atomic-motion-franka-move-to-pose`。
- Atomic gripper: `atomic-gripper-<robot>-<capability>`，例如 `atomic-gripper-franka-close`。
- Procedure: `procedure-<robot-or-domain>-<routine>`，例如 `procedure-franka-handover-fixed`。
- Task: `task-<verb-object>` 或 `task-<verb-object-to-target>`，例如 `task-pick-vial`、`task-place-vial-in-rack`。

## 什么时候新增顶层目录

只有真的需要共享时才新增顶层目录：

- 新增 atomic skill 时，再建 `atomic_skills/`。
- 新增固定流程时，再建 `procedure_skills/`。
- 新增组合任务时，再建 `task_skills/`。
- 第二个项目复用了同一段代码，再建共享包。
- 出现 workspace 级测试或共享配置模板时，再建对应目录。

在此之前，配置、测试、脚本、标定都放回各自项目目录。

## 机器人安全

真实机器人执行必须默认保守：

- 默认 dry-run，不默认真实运动。
- 真实执行必须显式参数，例如 `--execute` 或 `execution.mode: real`。
- 轨迹发送前必须检查 workspace、速度、加速度、夹爪状态和碰撞风险。
- 坐标必须写清坐标系和单位，默认米和弧度。
- 使用视觉定位结果前，必须确认 base 坐标可用且标定匹配当前相机和机器人。

实验输出、`.env`、API key、模型权重、`.venv`、`runs/` 默认不提交。

## Git

- 外层 `agentic_skills` 管理 workspace 规范、文档和 submodule 指针。
- 成熟 skill project 可以保留独立 git repo；当前 `atomic_skills/object_locator` 就是独立 repo。
- 内层项目先提交、测试、推送，外层 repo 再更新文档或指针。
- Commit 前至少运行 `git status --short`，代码改动运行项目自己的测试。
- Commit message 推荐：`workspace: ...`、`atomic(<name>): ...`、`procedure(<name>): ...`、`task(<name>): ...`。
