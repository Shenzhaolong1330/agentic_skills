# Capability Index

> Generated from `skill_manifest.json` v0.2.0. Only `visibility=public` entries are listed.
> This is metadata for routing and review; the Registry never executes a capability.
> S9 task-private capabilities are intentionally omitted and remain behind a trusted TaskCompilationContext.

## `gripper.command`

- version: `1.0.0`
- kind: `action`
- description: Robotiq gripper open/close/initialize/status. Entrypoint: gripper_control.
- input contract: `schemas/capabilities/gripper_command.input.schema.json`
- output contract: `schemas/action_result.schema.json`
- requires hardware: `true`
- risk class: `GRIPPER`
- physical side effects: controls gripper, changes gripper state, may change held-state
- resources: robot.dual_franka.grippers (exclusive)
- verifier: `task_specific`; physical verification limited: `true`
- dispatch support: `plan_only`; adapter binding: `fixed.gripper.command.v1`
- allowed as recovery: `false`

## `gripper.observe_status`

- version: `1.0.0`
- kind: `observation`
- description: Robotiq gripper open/close/initialize/status. Entrypoint: gripper_status.
- input contract: `schemas/capabilities/empty_input.schema.json`
- output contract: `schemas/observation_result.schema.json`
- requires hardware: `true`
- risk class: `READ_ONLY_HARDWARE`
- physical side effects: none declared
- resources: robot.dual_franka.grippers (shared)
- verifier: `output_schema`; physical verification limited: `false`
- dispatch support: `plan_only`; adapter binding: `fixed.gripper.observe_status.v1`
- allowed as recovery: `false`

## `motion.go_home`

- version: `1.0.0`
- kind: `action`
- description: Run le_nero robot-reset to return robot home. Entrypoint: run_robot_go_home.
- input contract: `schemas/capabilities/empty_input.schema.json`
- output contract: `schemas/action_result.schema.json`
- requires hardware: `true`
- risk class: `MOTION`
- physical side effects: moves robot arms, may move both arms
- resources: robot.dual_franka.shared_workspace (exclusive)
- verifier: `task_specific`; physical verification limited: `true`
- dispatch support: `plan_only`; adapter binding: `fixed.motion.go_home.v1`
- allowed as recovery: `false`

## `motion.move_to_pose`

- version: `1.0.0`
- kind: `action`
- description: Dual Franka base-frame xyz+rotvec P2P motion. Entrypoint: move_to_pose.
- input contract: `schemas/capabilities/motion_move_to_pose.input.schema.json`
- output contract: `schemas/action_result.schema.json`
- requires hardware: `true`
- risk class: `MOTION`
- physical side effects: moves robot arms
- resources: robot.dual_franka.shared_workspace (exclusive)
- verifier: `task_specific`; physical verification limited: `true`
- dispatch support: `plan_only`; adapter binding: `fixed.motion.move_to_pose.v1`
- allowed as recovery: `false`

## `perception.locate_object_3d`

- version: `1.0.0`
- kind: `observation`
- description: RealSense/VLM/Grounded-SAM/depth object localization. Entrypoint: object-locator-json.
- input contract: `schemas/capabilities/empty_input.schema.json`
- output contract: `schemas/observation_result.schema.json`
- requires hardware: `true`
- risk class: `READ_ONLY_HARDWARE`
- physical side effects: none declared
- resources: camera.realsense (shared)
- verifier: `output_schema`; physical verification limited: `false`
- dispatch support: `unsupported`; adapter binding: `none`
- allowed as recovery: `false`

## `procedure.handover_transition`

- version: `1.0.0`
- kind: `procedure`
- description: Fixed dual-Franka handover transition. Entrypoint: handover_transition.
- input contract: `schemas/capabilities/handover_transition.input.schema.json`
- output contract: `schemas/action_result.schema.json`
- requires hardware: `true`
- risk class: `HIGH_RISK`
- physical side effects: moves robot arms, controls gripper, changes gripper state, may change held-state
- resources: robot.dual_franka.shared_workspace (exclusive)
- verifier: `task_specific`; physical verification limited: `true`
- dispatch support: `plan_only`; adapter binding: `fixed.procedure.handover_transition.v1`
- allowed as recovery: `false`

## `recovery.robot_recover`

- version: `1.0.0`
- kind: `recovery`
- description: Run le_nero robot-reset to return robot home. Entrypoint: run_robot_recover.
- input contract: `schemas/capabilities/empty_input.schema.json`
- output contract: `schemas/action_result.schema.json`
- requires hardware: `true`
- risk class: `RECOVERY`
- physical side effects: changes controller state, changes gripper state, may change held-state, may move both arms, may open gripper
- resources: robot.dual_franka.shared_workspace (exclusive)
- verifier: `task_specific`; physical verification limited: `true`
- dispatch support: `plan_only`; adapter binding: `fixed.recovery.robot_recover.v1`
- allowed as recovery: `true`

## `recovery.robot_reset`

- version: `1.0.0`
- kind: `recovery`
- description: Run le_nero robot-reset to return robot home. Entrypoint: run_robot_reset.
- input contract: `schemas/capabilities/empty_input.schema.json`
- output contract: `schemas/action_result.schema.json`
- requires hardware: `true`
- risk class: `RECOVERY`
- physical side effects: moves robot arms, controls gripper, may clear controller faults, may change held-state, changes gripper state, may move both arms, may open gripper
- resources: robot.dual_franka.shared_workspace (exclusive)
- verifier: `task_specific`; physical verification limited: `true`
- dispatch support: `plan_only`; adapter binding: `fixed.recovery.robot_reset.v1`
- allowed as recovery: `true`

## `robot.observe_health`

- version: `1.0.0`
- kind: `observation`
- description: Dual Franka health check and guarded reset. Entrypoint: check_and_reset_status.
- input contract: `schemas/capabilities/empty_input.schema.json`
- output contract: `schemas/observation_result.schema.json`
- requires hardware: `true`
- risk class: `READ_ONLY_HARDWARE`
- physical side effects: none declared
- resources: robot.dual_franka.state (shared)
- verifier: `output_schema`; physical verification limited: `false`
- dispatch support: `plan_only`; adapter binding: `fixed.robot.observe_health.v1`
- allowed as recovery: `true`

## `robot.recover_reset_home`

- version: `1.0.0`
- kind: `recovery`
- description: Dual Franka health check and guarded reset. Entrypoint: check_and_reset_ensure.
- input contract: `schemas/capabilities/empty_input.schema.json`
- output contract: `schemas/action_result.schema.json`
- requires hardware: `true`
- risk class: `RECOVERY`
- physical side effects: moves robot arms, controls gripper, may clear controller faults, may change held-state, changes gripper state, may move both arms, may open gripper
- resources: robot.dual_franka.shared_workspace (exclusive)
- verifier: `task_specific`; physical verification limited: `true`
- dispatch support: `plan_only`; adapter binding: `fixed.robot.recover_reset_home.v1`
- allowed as recovery: `true`

## `state.capture_realsense`

- version: `1.0.0`
- kind: `observation`
- description: Capture state and RealSense images for supervision/debugging. Entrypoint: view_state_realsense.
- input contract: `schemas/capabilities/empty_input.schema.json`
- output contract: `schemas/observation_result.schema.json`
- requires hardware: `true`
- risk class: `READ_ONLY_HARDWARE`
- physical side effects: none declared
- resources: camera.realsense (shared), robot.dual_franka.state (shared)
- verifier: `output_schema`; physical verification limited: `false`
- dispatch support: `plan_only`; adapter binding: `fixed.state.capture_realsense.v1`
- allowed as recovery: `false`

## `state.reset_realsense`

- version: `1.0.0`
- kind: `action`
- description: Capture state and RealSense images for supervision/debugging. Entrypoint: view_state_realsense_reset.
- input contract: `schemas/capabilities/empty_input.schema.json`
- output contract: `schemas/action_result.schema.json`
- requires hardware: `true`
- risk class: `HIGH_RISK`
- physical side effects: changes camera device state
- resources: camera.realsense (exclusive)
- verifier: `task_specific`; physical verification limited: `true`
- dispatch support: `plan_only`; adapter binding: `fixed.state.reset_realsense.v1`
- allowed as recovery: `false`

## `task.pick_insert.full_flow`

- version: `1.0.0`
- kind: `task`
- description: Task-level pick tube, handover, locate rack/hole, insert, release. Entrypoint: full_flow.
- input contract: `schemas/capabilities/task_run.input.schema.json`
- output contract: `schemas/action_result.schema.json`
- requires hardware: `true`
- risk class: `HIGH_RISK`
- physical side effects: moves robot arms, controls gripper, composes hardware capabilities, changes gripper state, may change held-state
- resources: robot.dual_franka.shared_workspace (exclusive), camera.realsense (exclusive)
- verifier: `task_specific`; physical verification limited: `true`
- dispatch support: `unsupported`; adapter binding: `none`
- allowed as recovery: `false`

## `task.pick_insert.run`

- version: `1.0.0`
- kind: `task`
- description: Task-level pick tube, handover, locate rack/hole, insert, release. Entrypoint: task_runner.
- input contract: `schemas/capabilities/task_run.input.schema.json`
- output contract: `schemas/action_result.schema.json`
- requires hardware: `true`
- risk class: `HIGH_RISK`
- physical side effects: moves robot arms, controls gripper, composes hardware capabilities, changes gripper state, may change held-state
- resources: robot.dual_franka.shared_workspace (exclusive), camera.realsense (exclusive)
- verifier: `task_specific`; physical verification limited: `true`
- dispatch support: `unsupported`; adapter binding: `none`
- allowed as recovery: `false`

## `task.pick_insert.single_flow`

- version: `1.0.0`
- kind: `task`
- description: Task-level pick tube, handover, locate rack/hole, insert, release. Entrypoint: single_flow.
- input contract: `schemas/capabilities/task_run.input.schema.json`
- output contract: `schemas/action_result.schema.json`
- requires hardware: `true`
- risk class: `HIGH_RISK`
- physical side effects: moves robot arms, controls gripper, composes hardware capabilities, changes gripper state, may change held-state
- resources: robot.dual_franka.shared_workspace (exclusive), camera.realsense (exclusive)
- verifier: `task_specific`; physical verification limited: `true`
- dispatch support: `unsupported`; adapter binding: `none`
- allowed as recovery: `false`
