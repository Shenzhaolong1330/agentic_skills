# 硬件入口 inventory

本表是外层仓库的受控入口清单。`GATED_PUBLIC` 和 `READ_ONLY_GATED` 是可由公开文档推荐的固定入口；`INTERNAL_BEHIND_GATE` 只能由已通过 Harness Gate 的固定 wrapper/runner 调用；`UNSUPPORTED` 与 `LEGACY_DISABLED` 不得作为 live 入口推荐。最终验收要求所有公开入口都有受控分类。

| 路径 | skill | entrypoint | 分类 | 副作用/只读 | gate 接入 | live 参数 | recovery | 公开文档 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `scripts/run_gen_agent_acceptance.py` | gen-agent | acceptance | UNSUPPORTED | 不访问硬件 | 静态/测试验收 | 无 | 否 | 是 |
| `atomic_skills/object_locator` gitlink | atomic-perception-locate-object-3d | object-locator-json | INTERNAL_BEHIND_GATE | 相机，只读 | task `CommandRunner` + manifest | `live`, `--hardware-allowed`；任务感知由 task `execute` 统一 gate | 否 | 仅说明为内部依赖 |
| `atomic_skills/dual_franka_p2p/.../move_to_pose.py` | atomic-motion-franka-move-to-pose | move_to_pose | GATED_PUBLIC | RPC + 移动 | 脚本调用共享 Gate | `--mode live --hardware-allowed --execute` | 否 | 是 |
| `atomic_skills/dual_franka_gripper/.../gripper_control.py` | atomic-gripper-franka-open-close | gripper_control | GATED_PUBLIC | status 只读；open/close 有副作用 | 脚本调用共享 Gate | status: `--mode live --hardware-allowed`；动作再加 `--execute` | 否 | 是 |
| `atomic_skills/dual_franka_reset/.../check_and_reset.py` | atomic-state-dual-franka-reset | check_and_reset_status | READ_ONLY_GATED | RPC 状态，只读 | 脚本调用共享 Gate | `--mode live --hardware-allowed` | 是 | 是 |
| 同上 | atomic-state-dual-franka-reset | check_and_reset_ensure | GATED_PUBLIC | reset、移动、夹爪 | 脚本调用共享 Gate + recovery manifest | `--mode live --hardware-allowed --execute` | 是 | 是 |
| `atomic_skills/state_realsense_viewer/.../view_state_realsense.py` | atomic-state-realsense-viewer | view_state_realsense | READ_ONLY_GATED | 相机/RPC 观测 | 脚本调用共享 Gate | `--mode live --hardware-allowed` | 否 | 是 |
| `procedure_skills/dual_franka_handover_transition/.../handover_transition.py` | procedure-franka-handover-transition | handover_transition | GATED_PUBLIC | 移动 + 夹爪 | 脚本调用共享 Gate | `--mode live --hardware-allowed --execute` | 否 | 是 |
| `procedure_skills/robot_reset_home/.../run_robot_reset.sh` | procedure-robot-reset-home | run_robot_reset | GATED_PUBLIC | reset/home、移动、夹爪 | 固定 recovery preflight + manifest | `--mode live --hardware-allowed --execute` | 是 | 是 |
| 同目录 `run_robot_recover.sh` | procedure-robot-reset-home | run_robot_recover | GATED_PUBLIC | 控制器恢复，物理状态可变 | 固定 recovery preflight + manifest | `--mode live --hardware-allowed --execute` | 是 | 是 |
| 同目录 `run_robot_go_home.sh` | procedure-robot-reset-home | run_robot_go_home | GATED_PUBLIC | home 移动 | 固定 home preflight + manifest | `--mode live --hardware-allowed --execute` | 否 | 是 |
| `task_skills/.../task_pick_tube_insert_rack_runner.py` | task-pick-tube-insert-rack | task_runner | GATED_PUBLIC | 感知、RPC、移动、夹爪 | task entrypoint 预检 + `CommandRunner` | live task: `--hardware-allowed --execute` | 内部 recovery 另检 manifest | 是 |
| `task_skills/.../run_single_pick_tube_insert_rack.sh` | task-pick-tube-insert-rack | single-flow | GATED_PUBLIC | SAM/相机服务、感知、移动、夹爪 | 启动服务前固定 task preflight | `--mode live --hardware-allowed --execute` | reset wrapper recovery gate | 是 |
| `task_skills/.../run_full_pick_tube_insert_rack.sh` | task-pick-tube-insert-rack | full-flow | GATED_PUBLIC | SAM/相机服务、感知、移动、夹爪 | 启动服务前固定 task preflight | `--mode live --hardware-allowed --execute` | reset wrapper recovery gate | 是 |
| `task_skills/.../locate_then_grasp_by_tail_side.py` + `grasp_right_arm_xyz.sh` | task-pick-tube-insert-rack | task_live_grasp_handover | INTERNAL_BEHIND_GATE | RPC、移动、夹爪 | 固定 `grasp` preflight；仅由 task wrapper 调用 | `--mode live --hardware-allowed --execute`（内部） | 否 | 否；不作为 Codex 直接入口 |
| `task_skills/.../run_manual_grip_to_insert.py` | task-pick-tube-insert-rack | task_live_insert_flow | INTERNAL_BEHIND_GATE | 相机、RPC、移动、夹爪 | 固定 `insert` preflight；仅由 single/full wrapper 调用 | `--mode live --hardware-allowed --execute`（内部） | 否 | 否；不作为 Codex 直接入口 |
| `task_skills/.../sam_cache_service.py` + `wrist_camera_service.py` | task-pick-tube-insert-rack | internal hardware services | INTERNAL_BEHIND_GATE | SAM/RealSense 后台服务 | 只由 single/full wrapper 在 task preflight 后启动 | 继承已通过的 wrapper live gate | 否 | 否 |
| task-owned `tube_insertion_skill.py`、`move_above_hole_from_wrist.py`、`insert_down_from_current_pose.py` | task-pick-tube-insert-rack | internal motion stages | INTERNAL_BEHIND_GATE | RPC、相机、移动、夹爪 | 仅由已 gated 的 insert flow 调用 | 继承 `insert` wrapper gate | 否 | 否 |
| `task_skills/.../grasp_right_arm_xyz.py`、`run_manual_grip_to_insert.py`、`locate_all_tubes_once.py` 等 | task-pick-tube-insert-rack | task stages | INTERNAL_BEHIND_GATE | 依次包含相机/RPC/移动/夹爪 | 仅由 task runner 或 fixed wrapper 调用 | 不公开 standalone live | 否 | 否 |

低层脚本仍可能被操作系统用户直接启动；本 inventory 不声称 OS 级权限隔离已经存在。公开 workflow 的安全边界是固定 wrapper/runner 在硬件初始化前执行共享 Gate。
