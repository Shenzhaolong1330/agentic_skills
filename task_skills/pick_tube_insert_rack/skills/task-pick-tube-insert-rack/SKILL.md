---
name: task-pick-tube-insert-rack
description: Task-level harness for locating a loose test tube, selecting the tail-side arm, grasping and handing over through a fixed transition, locating the rack and highest-confidence empty hole, inserting the tube, and handling abnormal robot state through gated AUTO_RESET_RECOVERY.
---

# Pick Tube Insert Rack

Use this task skill for the full task-level orchestration. It is not an atomic P2P, gripper, perception, or reset command.

## Safe Default Entry

```bash
python3 /home/deepcybo/agentic_skills/task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/task_pick_tube_insert_rack_runner.py \
  --mode mock
```

Dry-run planning entry:

```bash
python3 /home/deepcybo/agentic_skills/task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/task_pick_tube_insert_rack_runner.py \
  --mode dry_run \
  --artifact-dir /tmp/agentic_skills_pick_tube_insert_rack_dry_run
```

Safe shell wrapper for the complete task defaults to `dry_run`:

```bash
/home/deepcybo/agentic_skills/task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/run_full_pick_tube_insert_rack.sh \
  --dry-run \
  --artifact-dir /tmp/agentic_skills_pick_tube_insert_rack_full
```

In explicitly gated live mode, this wrapper runs tube localization, tail-side
arm selection, grasp and transition handover, then rack/hole localization,
insertion, release, and retract. It does not run an unconditional reset. Use
`--help` to inspect the live gates and per-stage passthrough arguments.

## Live Entry

Live mode may open cameras, move robot arms, control grippers, and run reset recovery. It must pass `HardwareGate`:

```bash
python3 /home/deepcybo/agentic_skills/task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/task_pick_tube_insert_rack_runner.py \
  --mode live \
  --execute \
  --auto-reset-on-abnormal
```

Do not run live mode during implementation or review.

## Auto Reset Recovery Policy

Reset is not disabled. In live mode, `auto_reset_on_abnormal` is on by default. If preflight health, motion, gripper, RPC, or reset-guard output indicates abnormal robot state, the runner enters `AUTO_RESET_RECOVERY`.

Recovery rules:

- `mock`: no real reset; returns `MOCK_RESET_OK` and after-health `READY`.
- `dry_run`: no real reset; records reset command plan and returns `PLANNED_ONLY`.
- `from_artifacts`: reads health/reset artifacts and never executes hardware.
- `live`: executes reset only after `HardwareGate` passes and manifest marks the reset entrypoint `allowed_as_recovery=true`.
- After reset, the runner performs `RESET_RECOVERY_VERIFY`.
- If reset happens before holding a tube, successful recovery may retry/resume.
- If reset happens while holding a tube, reset is still allowed, but `held_object_risk=true`; default result is `ABORT` with `reset_while_holding_tube_requires_reverification` unless future verified resume logic is explicitly enabled.
- Reset may move both arms or change gripper/object state, so every reset writes `reset_recovery_<index>.json`.

## Dependencies

- `atomic-perception-locate-object-3d`
- `atomic-motion-franka-move-to-pose`
- `atomic-gripper-franka-open-close`
- `atomic-state-dual-franka-reset`
- `procedure-franka-handover-transition`
- `procedure-robot-reset-home`
- `atomic-state-realsense-viewer` for supervision diagnostics only

Read repository-level `SKILL_INDEX.md` and `skill_manifest.json` before routing.

## Inputs And Outputs

Schemas:

- `/home/deepcybo/agentic_skills/schemas/common.schema.json`
- `/home/deepcybo/agentic_skills/schemas/task_pick_tube_insert_rack.schema.json`

Key task outputs include `tube_pose`, `tube_xyz`, `tube_mouth_direction`, `tube_body_axis`, `selected_arm`, `opposite_arm`, `pregrasp_pose`, `grasp_pose`, `handover_pose`, `rack_pose`, `empty_hole_candidates`, `selected_hole_pose`, `hole_confidence`, `wrist_camera_pose`, `error_xyz`, `error_rot`, `error_6dof`, `retry_count`, `task_state`, `held_object_state`, `robot_health`, `reset_recovery`, `completion_flag`, and `physical_verified`.

Default 6DoF threshold:

- translation <= `0.005 m`
- rotation <= `0.0872664626 rad`, about 5 degrees, configurable by CLI

## State Machine

| State | Purpose | Failure / recovery |
| --- | --- | --- |
| `INIT` | create context, load manifest, initialize trace | abort if manifest invalid |
| `PREFLIGHT_ROBOT_HEALTH_CHECK` | check health or plan check | abnormal enters `AUTO_RESET_RECOVERY`; estop/unsafe aborts |
| `AUTO_RESET_RECOVERY` | gated automatic reset recovery | mock/dry-run never execute; live requires gate |
| `RESET_RECOVERY_VERIFY` | check health after reset | still abnormal aborts; held-object reset defaults abort |
| `LOCATE_TUBE` | locate loose tube | perception failure aborts |
| `SELECT_ARM` | choose tail-side arm | uncertain side aborts/supervision required |
| `MOVE_TO_PREGRASP` | plan/execute P2P above body | abnormal triggers recovery; error threshold retries |
| `ALIGN_GRASP_POSE` | roll/pitch vertical, yaw policy | posture/motion failure retries then aborts |
| `GRASP_TUBE_BODY` | close selected gripper | abnormal triggers recovery; failed grasp aborts |
| `HANDOVER` | fixed transition and partner grip | no active release unless partner close is confirmed |
| `LOCATE_RACK` | locate rack with head camera | perception failure aborts |
| `NON_HOLDER_HOME` | move unused arm safe/home | abnormal triggers recovery; held-object reset aborts after reset |
| `HOLDER_ALIGN_RACK_ABOVE` | align holder over rack | abnormal triggers recovery |
| `WRIST_LOCATE_HOLE` | locate empty holes with wrist camera | no candidate aborts |
| `SELECT_HOLE` | choose highest confidence hole | low confidence aborts |
| `INSERT_ALIGN_REFINE` | refine up to configured retries | 6DoF threshold failure aborts |
| `INSERT_DESCEND` | descend z into hole | failure blocks release |
| `RELEASE` | release only after insert success | unresolved abnormal blocks release |
| `RETURN_HOME` | return holder safe/home | abnormal triggers recovery |
| `COMPLETE` | write final result | `completion_flag=true` |
| `ABORT` | write stopped_reason and task result | non-zero exit |

## Trace And Supervision

Artifacts are written under `artifact_dir`, default `/tmp/agentic_skills_runs/<run_id>/`:

- `context.json`
- `manifest_snapshot.json`
- `robot_health_preflight.json`
- `reset_recovery_<index>.json`
- `stage_<index>_<STATE>.json`
- `command_plan.json`
- `task_result.json`

## Current Limits

- Real grasp success still needs stronger gripper width/fraction and object-present validation.
- Real handover lacks contact/force confirmation.
- Real insertion still needs visual/force closed-loop confirmation.
- Existing low-level grasp orientation aligns a gripper axis parallel to tube axis; exact yaw-perpendicular-to-body-normal semantics require further calibration.
- Multi-candidate empty-hole support is wrapped in task logic; native object_locator candidate output should be added later for live perception.
- Live health, reset recovery, tube localization, grasp/handover, and insertion subprocesses are dispatched by `CommandRunner` only after their manifest entrypoint passes `HardwareGate`.
- A successful live return still reports warnings for the current contact/force verification and retract-versus-full-home limitations.
