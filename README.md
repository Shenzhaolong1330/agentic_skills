# Agentic Physical Skills

这个仓库用于开发 Codex 可调用的物理世界 skill。目前先保持最小结构，避免空目录和重复文档。

## Skill 分层

这里把物理 skill 分成三类：

| 类型 | 含义 | 例子 | 放哪里 |
| --- | --- | --- | --- |
| Atomic skill | 只做一个底层能力，输入输出稳定，可独立测试 | 识别定位、点到点运动、夹爪开合 | `atomic_skills/` |
| Procedure skill | 固定动作流程，不依赖复杂感知决策 | 固定 handover、回 home、固定扫描位姿 | `procedure_skills/` |
| Task skill | 组合感知、运动、夹爪和异常处理完成任务 | 抓起 vial、放入试管架、识别后递交 | `task_skills/` |

判断标准：

- 只完成一个能力，例如“定位目标 3D 点”或“移动到某个位姿”，就是 atomic skill。
- 固定点到点动作序列，例如 handover，不需要视觉决策，属于 procedure skill。
- 需要调用多个原子能力并处理状态机、重试、安全检查，例如 `task-pick-vial = locate + plan + move + grasp + lift`，属于 task skill。

## 当前结构

```text
agentic_skills/
├── README.md
├── docs/
│   └── conventions.md
└── atomic_skills/
    └── object_locator/
        ├── README.md
        ├── RUNNING.md
        ├── pyproject.toml
        ├── config*.yaml
        ├── calibration/
        ├── src/object_locator/
        ├── tests/
        └── skills/atomic-perception-locate-object-3d/
```

当前已有：

- `atomic_skills/object_locator`: atomic perception skill，负责 RealSense / VLM / Grounded-SAM / depth 的目标或部位 3D 定位。
- `atomic_skills/dual_franka_p2p`: atomic motion skill，负责双 Franka 末端点到点 base-frame xyz+rotvec 运动。
- `atomic_skills/dual_franka_gripper`: atomic gripper skill，负责双 Franka Robotiq 夹爪开、闭、初始化和状态读取。
- `procedure_skills/dual_franka_handover_transition`: procedure skill，负责两只手在标定 transition 位姿进行交接。
- `procedure_skills/robot_reset_home`: procedure skill，在 `le_nero` 环境运行 `robot-reset`，让机械臂回到原位。
- `task_skills/pick_tube_insert_rack`: task skill，负责定位试管、选择抓取手、抓取并插入试管架。

未来新增目录时使用按类型分组的命名：

```text
atomic_skills/
├── object_locator/         # atomic perception: locate object/keypoint in 3D
└── franka_motion/          # atomic motion: move-to-pose, move-to-joint, gripper

procedure_skills/
└── handover_fixed/         # procedure: fixed point-to-point handover

task_skills/
└── pick_vial/              # task: locate vial + move + grasp + lift
```

## 放置规则

- 核心实现放项目自己的 `src/`。例如 RealSense、VLM、Grounded-SAM、depth 反投影和坐标变换都放在 `atomic_skills/object_locator/src/object_locator/`。
- Codex skill 包装放项目自己的 `skills/`。例如 `atomic_skills/object_locator/skills/atomic-perception-locate-object-3d/` 只负责告诉 Codex 怎么运行、调试和解释输出。
- 顶层暂时不建空目录。等真的新增对应 skill 时，再创建 `atomic_skills/`、`procedure_skills/` 或 `task_skills/`。
- 如果以后有 Franka 运动规划，放在 `atomic_skills/franka_motion/`，里面自己放 `src/`、`tests/`、`skills/`。
- 如果以后有“视觉定位 + Franka 抓取”的组合 skill，放在 `task_skills/pick_vial/`，只做 orchestration。

## 命名规则

这里的 Codex skill 文件夹指 `*/skills/<skill-name>/`，不是外层物理能力项目目录。

物理能力项目目录命名：

- 类型分组目录固定为 `atomic_skills/`、`procedure_skills/`、`task_skills/`。
- 分组下的 skill 目录用 `snake_case`，不加 `_skill` 后缀，例如 `object_locator`、`franka_motion`、`handover_fixed`、`pick_vial`。
- Python 包用 `snake_case`，通常和 skill 目录一致，例如 `object_locator`、`franka_motion`、`pick_vial`。
- Codex skill 目录和 `SKILL.md` frontmatter 里的 `name` 用带层级前缀的 `kebab-case`，目录名必须和 `name` 一致。

三类目录命名：

- Atomic skill folder: `atomic_skills/<domain_or_capability>`，例如 `atomic_skills/object_locator`、`atomic_skills/franka_motion`、`atomic_skills/franka_gripper`。
- Procedure skill folder: `procedure_skills/<procedure_name>`，例如 `procedure_skills/handover_fixed`、`procedure_skills/go_home`、`procedure_skills/scan_workspace`。
- Task skill folder: `task_skills/<verb_object>` 或 `task_skills/<verb_object_to_target>`，例如 `task_skills/pick_vial`、`task_skills/place_vial_in_rack`、`task_skills/pick_and_handover_vial`。

Codex skill 文件夹命名：

```text
skills/<layer>-<domain>-<capability>/
```

`layer` 固定为：

- `atomic`: 原子能力。
- `procedure`: 固定流程。
- `task`: 组合任务。

推荐格式：

- Atomic perception: `atomic-perception-<capability>`，例如 `atomic-perception-locate-object-3d`、`atomic-perception-locate-rack-hole`。
- Atomic motion: `atomic-motion-<robot>-<capability>`，例如 `atomic-motion-franka-move-to-pose`、`atomic-motion-franka-move-to-joint`。
- Atomic gripper: `atomic-gripper-<robot>-<capability>`，例如 `atomic-gripper-franka-open`、`atomic-gripper-franka-close`。
- Procedure: `procedure-<robot-or-domain>-<routine>`，例如 `procedure-franka-handover-fixed`、`procedure-franka-go-home`。
- Task: `task-<verb-object>` 或 `task-<verb-object-to-target>`，例如 `task-pick-vial`、`task-place-vial-in-rack`、`task-pick-and-handover-vial`。

## 使用方式

Codex 使用 skill 时按从高到低选择：

1. 用户要完成完整任务时，优先使用 task skill，例如“抓起 vial”使用 `task-pick-vial`。
2. 用户要执行固定流程时，使用 procedure skill，例如“执行固定 handover”使用 `procedure-franka-handover-fixed`。
3. 用户只要单个能力时，使用 atomic skill，例如“定位 vial”使用 `atomic-perception-locate-object-3d`，“移动到某个位姿”使用 `atomic-motion-franka-move-to-pose`。

组合 skill 不重复实现原子能力，只编排它们：

```text
task-pick-vial
├── call atomic-perception-locate-object-3d
├── compute grasp/pregrasp/lift poses
├── call atomic-motion-franka-move-to-pose
├── call atomic-gripper-franka-close
└── call atomic-motion-franka-move-to-pose
```

真实机器人执行默认必须 dry-run。任何 procedure 或 task skill 要真实运动，都必须显式启用执行参数，并在执行前完成坐标系、标定、workspace、速度和碰撞检查。

## 现有项目

- [`atomic_skills/object_locator/`](atomic_skills/object_locator/README.md): RealSense D435i RGB-D 目标/部位定位，支持 `grounded_sam`、`color`、`vlm`，可输出相机坐标系和机器人 base 坐标系下的位置。
- [`atomic_skills/dual_franka_p2p/`](atomic_skills/dual_franka_p2p/): 双 Franka 点到点末端位姿运动。
- [`atomic_skills/dual_franka_gripper/`](atomic_skills/dual_franka_gripper/): 双 Franka Robotiq 夹爪开闭。
- [`procedure_skills/dual_franka_handover_transition/`](procedure_skills/dual_franka_handover_transition/): 双手交接 transition 流程。
- [`procedure_skills/robot_reset_home/`](procedure_skills/robot_reset_home/): 激活 `le_nero` 并运行 `robot-reset` 复位机械臂。
- [`task_skills/pick_tube_insert_rack/`](task_skills/pick_tube_insert_rack/): 试管抓取并插入试管架任务。
- [`atomic_skills/object_locator/RUNNING.md`](atomic_skills/object_locator/RUNNING.md): 当前机器的运行笔记和 smoke test 命令。
- [`docs/conventions.md`](docs/conventions.md): 团队约定、skill 格式和机器人安全边界。

## Git 管理标准

这个 workspace 采用“外层 workspace repo + 内层 skill project repo”的管理方式：

- 外层 `agentic_skills` repo 管理团队规范、README、目录约定，以及未来的 submodule 指针。
- 每个成熟 skill project 可以保留自己的 git repo，例如 `atomic_skills/object_locator` 当前来自 `https://github.com/Shenzhaolong1330/any_pose_skill`。
- 不删除内层项目的 `.git`，除非团队明确决定把它迁成 monorepo 子目录。
- 外层提交不直接混入内层项目的大量源码改动；内层项目先在自己的 repo 里提交、测试、推送，再由外层 repo 更新指针或文档。
- 不要在内层 repo 仍有未提交改动时更新外层 submodule/gitlink 指针。

首次把一个独立 skill repo 纳入 workspace 时，推荐用 submodule：

```bash
git submodule add -b main <repo-url> atomic_skills/<skill-folder>
git commit -m "workspace: add <skill-folder> submodule"
```

如果该目录已经存在且有自己的 `.git`，先在内层 repo 完成提交和推送，再决定是否转成 submodule 指针。

推荐提交流程：

```bash
# 1. 先处理具体 skill project
cd atomic_skills/object_locator
git status --short
python -m pytest -q
git add <changed-files>
git commit -m "atomic(object-locator): describe the change"
git push

# 2. 再回到 workspace 管理外层文档或 submodule 指针
cd ../..
git status --short
git add README.md docs/conventions.md atomic_skills/object_locator
git commit -m "workspace: update object locator skill"
```

如果某个新 skill 没有独立远端，并且团队确认它只属于这个 workspace，可以直接作为普通目录提交到外层 repo；一旦它需要独立发布、独立版本或复用，再拆成独立 repo/submodule。

Commit message 建议：

- `workspace: ...`：外层结构、README、规范、submodule 指针。
- `atomic(<name>): ...`：原子 skill，例如 `atomic(object-locator): add tail anchor depth sampling`。
- `procedure(<name>): ...`：固定流程，例如 `procedure(handover-fixed): add retreat pose`。
- `task(<name>): ...`：组合任务，例如 `task(pick-vial): add grasp retry state`。
- `docs: ...`、`test: ...`、`fix: ...` 可以用于很小的横向改动。

分支命名建议：

- `workspace/<topic>`
- `atomic/<skill-name>/<topic>`
- `procedure/<skill-name>/<topic>`
- `task/<skill-name>/<topic>`

不要提交：

- `.env`、API key 和其他凭据。
- `.venv/`、`__pycache__/`、`.pytest_cache/`。
- `runs/`、实验图片、`.npy`、`.bag`、模型权重、数据集。
- 未脱敏的真实机器人/相机私密标定，除非团队明确要求纳入版本管理。

真实机器人相关改动提交前至少确认：

```bash
git status --short
python -m pytest -q
```

涉及真实执行、轨迹规划、坐标变换、标定格式的改动，需要在 PR/commit 描述里写清 dry-run 结果、坐标系、单位和安全边界。

## 快速检查

```bash
cd atomic_skills/object_locator
source .venv/bin/activate
python -m pytest -q
```

## Harness / Manifest / Auto Reset Recovery

- [`SKILL_INDEX.md`](SKILL_INDEX.md): 仓库级 skill 路由索引，供 Codex 预加载，避免每次全仓搜索。
- [`skill_manifest.json`](skill_manifest.json): 机器可读 manifest，声明 entrypoint、硬件副作用和 safety gates。
- [`schemas/`](schemas/): `Pose6D`、`Error6DoF`、`RobotHealthStatus`、`ResetRecoveryResult`、`TaskResult` 等契约。
- [`agentic_skills_harness/`](agentic_skills_harness/): manifest loader、HardwareGate、trace writer、command runner、robot health monitor、ResetRecoveryController。
- [`docs/harness.md`](docs/harness.md): harness 模式、trace、reset recovery 和新增 manifest entry 的说明。

`task_pick_tube_insert_rack_runner.py` 是 `pick_tube_insert_rack` 推荐入口。默认使用 `mock`/`dry_run`，不会打开相机、连接机器人、移动机械臂或控制夹爪。live 任务必须显式给出 `--hardware-allowed --execute`；只读诊断只需 `--hardware-allowed`。reset 功能没有禁用；live 模式下 abnormal robot state 会自动进入受控 `AUTO_RESET_RECOVERY`。reset 仍是真机动作，必须通过 HardwareGate，并且所有 reset 都写入 trace。持管阶段 reset 后默认 abort，等待对象状态重验证。
