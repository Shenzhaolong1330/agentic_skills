# agentic_skills Skill Index

> 这是给 Codex 和任务 harness 预加载的仓库级索引。先读本文件、`CAPABILITY_INDEX.md` 和 `skill_manifest.json`，再进入具体 SKILL，避免每次重新全仓搜索。

Capability contract details and public routing metadata are in [`CAPABILITY_INDEX.md`](CAPABILITY_INDEX.md). The index is generated from the manifest and does not expose internal entrypoint commands or paths.

S4 dispatch uses only typed capability IDs and arguments; the system selects a
fixed adapter and the current actual manifest inventory is plan-only/unsupported
until each binding is independently reviewed. S5 World State, bounded
predicates, invalidation, and effect/goal verification are documented in
`docs/gen_agent/`.

## 全局安全原则

- 默认模式是 `mock` 或 `dry_run`，不会控制真实机器人、夹爪、相机或 ROS。
- 统一 task runner 和固定 shell wrapper 的 live 真机执行必须经过共享 `HardwareGate`：硬件访问显式给出 `--hardware-allowed`；移动、夹爪、reset/home 和其他物理副作用再给出 `--execute`，并且 manifest 允许该 entrypoint。
- `task-pick-tube-insert-rack` 的 single-flow shell wrapper 也受同一 Gate 约束：至少使用 `--mode live --hardware-allowed --execute`。授权失败发生在 SAM、RealSense、RPC 和机器人脚本启动前。
- reset 没有被禁用。live 模式下 abnormal robot state 会进入 `AUTO_RESET_RECOVERY`，由 reset guard 或 reset procedure 受控执行。
- reset 可能移动双臂、打开夹爪或改变持物状态，因此每次 reset 必须写 trace。持管阶段 reset 成功后默认 `ABORT`，要求重新验证对象状态。
- Codex 实现、审查、测试阶段不得直接运行 reset、P2P、gripper、object-locator live camera、launch/server 或任何带 `--execute` 的命令。

## 关键 Skill

| skill name | layer | path | purpose | safe default | hardware side effects | entrypoints | dependencies | reset/recovery policy |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `atomic-perception-locate-object-3d` | atomic | `atomic_skills/object_locator/skills/atomic-perception-locate-object-3d/` | RealSense/VLM/Grounded-SAM/depth 目标或部位 3D 定位 | 不由 task 默认 live 运行；mock/dry-run 使用 artifact/plan | live 会打开 RealSense，可调用 VLM 网络 | `object-locator --config ... --json` | RealSense、OpenRouter、depth calibration | 感知失败不触发 reset；相机/RPC 异常只写 stage failure |
| `atomic-motion-franka-move-to-pose` | atomic | `atomic_skills/dual_franka_p2p/skills/atomic-motion-franka-move-to-pose/` | 双 Franka base-frame xyz+rotvec P2P | dry-run plan；真实运动必须 `--mode live --hardware-allowed --execute` | live 会连接 RPC 并移动机械臂 | `scripts/move_to_pose.py` | RPC server | motion/RPC robot abnormal 触发 task `AUTO_RESET_RECOVERY` |
| `atomic-gripper-franka-open-close` | atomic | `atomic_skills/dual_franka_gripper/skills/atomic-gripper-franka-open-close/` | Robotiq open/close/initialize/status | status 需 `--mode live --hardware-allowed`；open/close 再加 `--execute` | live 会连接 RPC 并控制夹爪 | `scripts/gripper_control.py` | P2P RPC client | gripper/RPC abnormal 触发 task `AUTO_RESET_RECOVERY` |
| `atomic-state-dual-franka-reset` | atomic | `atomic_skills/dual_franka_reset/skills/atomic-state-dual-franka-reset/` | 读取双臂健康状态并按 guard 条件执行 reset | status 需 `--mode live --hardware-allowed`；真实 reset 必须再加 `--execute` 且 gate 通过 | status 连接 RPC；ensure/reset 可能移动双臂、清 fault、改变夹爪/持物状态 | `scripts/check_and_reset.py status|ensure` | P2P RPC client、robot_reset_home | `allowed_as_recovery=true`，task abnormal recovery 首选 |
| `atomic-state-realsense-viewer` | atomic | `atomic_skills/state_realsense_viewer/skills/atomic-state-realsense-viewer/` | 保存当前状态和 RealSense 图像供监督诊断 | 不作为默认 task stage | live 会读取 RPC 和打开相机 | `scripts/view_state_realsense.py` | object_locator camera utilities | 只做诊断，不自动 reset |
| `procedure-franka-handover-transition` | procedure | `procedure_skills/dual_franka_handover_transition/skills/procedure-franka-handover-transition/` | 固定 transition pose 双臂交接 | dry-run plan；真实交接必须 `--mode live --hardware-allowed --execute` | live 会移动双臂并控制夹爪 | `scripts/handover_transition.py` | P2P、gripper | abnormal 交给 task reset recovery；交接成功仍需物理验证增强 |
| `procedure-robot-reset-home` | procedure | `procedure_skills/robot_reset_home/skills/procedure-robot-reset-home/` | le_nero `robot-reset` 回 home | 不是默认入口；live 必须 `--mode live --hardware-allowed --execute` | 会移动双臂，可能打开夹爪 | `scripts/run_robot_reset.sh` | le_nero/dual_arm_teleop | `allowed_as_recovery=true` fallback；不绕过 HardwareGate |
| `task-pick-tube-insert-rack` | task | `task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/` | 试管抓取、交接、识别试管架和空孔、插入 | 推荐 `python scripts/task_pick_tube_insert_rack_runner.py --mode mock`；单根试管 dry-run 用 `scripts/run_single_pick_tube_insert_rack.sh --dry-run` | live 会组合感知、运动、夹爪、reset recovery | `scripts/task_pick_tube_insert_rack_runner.py`；单根试管真实执行入口是 `scripts/run_single_pick_tube_insert_rack.sh --mode live --hardware-allowed --execute` | 上述 atomic/procedure skills | runner 内置 `AUTO_RESET_RECOVERY`；single-flow 错误后按 wrapper reset policy 执行 |

## Codex 路由建议

1. 完整多根试管插入任务优先调用 `task-pick-tube-insert-rack` runner。
2. 单根试管“识别 -> 抓取 -> 插入”优先调用 `run_single_pick_tube_insert_rack.sh`；live 真机必须显式使用 `--mode live --hardware-allowed --execute`。
3. 低层脚本只作为 manifest entrypoint，不从 Codex 直接拼接执行真机。
4. 先审查 `command_plan.json` 和 trace，再考虑 live。
5. reset 是 task runtime recovery state，不是 shell 脚本里的无条件前置动作。
