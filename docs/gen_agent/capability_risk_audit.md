# Capability Risk and Verification Audit

This is a static, offline audit of every manifest capability. Entrypoints are read as text only; no camera, RPC, service, or hardware command is started.

- status: **PASS**
- capabilities audited: 18/18
- warnings: 9
- errors: 0
- automatic corrections: 0

| Capability | Visibility | Risk | Motion | Gripper | Hardware | Verifier | Limited | Severity |
| --- | --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `gripper.command` | public | GRIPPER | false | true | true | task_specific | true | WARNING |
| `gripper.observe_status` | public | READ_ONLY_HARDWARE | false | false | true | output_schema | false | WARNING |
| `internal.task.grasp_handover` | internal | HIGH_RISK | true | true | true | task_specific | true | PASS |
| `internal.task.insert_flow` | internal | CONTACT | true | true | true | task_specific | true | PASS |
| `internal.task.locate_object` | internal | READ_ONLY_HARDWARE | false | false | true | output_schema | false | WARNING |
| `motion.go_home` | public | MOTION | true | false | true | task_specific | true | WARNING |
| `motion.move_to_pose` | public | MOTION | true | false | true | task_specific | true | WARNING |
| `perception.locate_object_3d` | public | READ_ONLY_HARDWARE | false | false | true | output_schema | false | PASS |
| `procedure.handover_transition` | public | HIGH_RISK | true | true | true | task_specific | true | PASS |
| `recovery.robot_recover` | public | RECOVERY | false | false | true | task_specific | true | WARNING |
| `recovery.robot_reset` | public | RECOVERY | true | true | true | task_specific | true | PASS |
| `robot.observe_health` | public | READ_ONLY_HARDWARE | false | false | true | output_schema | false | WARNING |
| `robot.recover_reset_home` | public | RECOVERY | true | true | true | task_specific | true | PASS |
| `state.capture_realsense` | public | READ_ONLY_HARDWARE | false | false | true | output_schema | false | WARNING |
| `state.reset_realsense` | public | HIGH_RISK | false | false | true | task_specific | true | WARNING |
| `task.pick_insert.full_flow` | public | HIGH_RISK | true | true | true | task_specific | true | PASS |
| `task.pick_insert.run` | public | HIGH_RISK | true | true | true | task_specific | true | PASS |
| `task.pick_insert.single_flow` | public | HIGH_RISK | true | true | true | task_specific | true | PASS |

## Conservative corrections

- None.

## Verifier interpretation

`output_schema` proves only output structure. A command return code, controller completion, or schema-valid output never proves a physical effect or top-level goal. Physical capabilities with incomplete independent evidence remain `physical_verification_limited=true` and require a later verifier.
