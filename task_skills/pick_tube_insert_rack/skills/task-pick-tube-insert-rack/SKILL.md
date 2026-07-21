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

Single-tube shell wrapper for one recognize -> grasp -> insert attempt also
defaults to `dry_run`:

```bash
/home/deepcybo/agentic_skills/task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/run_single_pick_tube_insert_rack.sh \
  --dry-run \
  --artifact-dir /tmp/agentic_skills_pick_tube_insert_rack_single
```

In explicitly gated live mode, the full wrapper captures the head-camera RGB-D frame
once, inventories all safely localized loose tubes plus the fixed rack, sorts
tubes by initial image x from left to right, and then repeats tail-side arm
selection, grasp, transition handover, cached-slot local correction,
insertion, release, and retract for each cached tube. The cached rack pose is
reused for every observation move. Live mode defaults to the proven per-insert
`--wrist-perception-mode legacy-vlm`, which uses the existing wrist
object-locator VLM+SAM empty-hole method. Treat `cached-grid` as an experimental
opt-in only. The
atomic perception and motion skills are not modified. It
does not run an unconditional reset. Use `--help` to inspect the live gates and
per-stage passthrough arguments.

After model loading and the complete initial tube/rack inventory, pause before
the first robot motion and require the operator to press Enter. Print the RGB
and inventory artifact paths for inspection. Ctrl-C or closed stdin aborts with
no grasp motion. Require an interactive terminal by default; use
`--no-wait-before-motion` only for explicitly intended automation.

For exactly one physical tube insertion attempt, prefer
`run_single_pick_tube_insert_rack.sh`. Its live gate is script-level:
`--mode live --execute`. It does not read `AGENTIC_SKILLS_HARDWARE_TOKEN`, does
not require `--hardware-allowed`, and does not call the task runner
`HardwareGate`. It still opens live cameras, moves the robot, controls grippers,
and may run the configured reset script after a failed grasp/handover or
insertion stage. It uses the same persistent SAM service, persistent RealSense
service, rack cache, wrist-hole perception, reset policy, and per-stage
passthrough arguments as the full wrapper. It passes `--max-tubes 1` to the
initial inventory; in that mode the VLM prompt asks for only the leftmost
complete loose tabletop tube and does not request a full tube inventory. The
live perception stage emits only one executable tube result before grasping and
inserting it. Unlike the full wrapper, single-flow does not pause for Enter
before motion by default; add `--wait-before-motion` when an operator inspection
gate is needed. Default artifacts are
written under `/tmp/agentic_skills_runs/single_pick_tube_insert_rack_<stamp>/`,
with `single_flow.log`, `tube_detection.json`, `rack_detection.json`, and
`insertion_tube_01/wrist_empty_hole_vlm_response.json` when wrist VLM is used.
A failed grasp/handover or insertion triggers the configured full robot reset
before the wrapper exits with failure accounting; there is no "next tube" to
continue to.

During a multi-tube live run, a nonzero insertion subprocess is recorded in its
per-tube log and does not terminate the left-to-right loop; the wrapper processes
the remaining cached tubes and reports the aggregate failure count at the end.
If the insertion P2P misses final-pose tolerance but reaches the requested
downward depth and confirms gripper release plus retract/home cleanup, it is
reported as `completed_with_insert_tolerance_warning` rather than a stopped task.

By default, a nonzero grasp/handover or insertion subprocess triggers the
existing full `procedure-robot-reset-home` workflow before the loop advances to
the next cached tube. Full reset opens both grippers and returns both arms Home,
so a held tube may be released; each attempt is saved as
`reset_after_tube_NN_<stage>.log`. The loop resumes only after reset returns
success. A failed reset stops further robot commands. Use
`--no-reset-after-error` only when an operator is providing another recovery
path.

The live wrapper also starts one task-local persistent `facebook/sam-vit-base`
service before the initial capture. Model loading overlaps the initial camera/VLM
work, and the same resident model refines every initial tube box and every later
wrist-camera empty-hole box. This avoids reloading SAM for each object-locator
subprocess while leaving the atomic object-locator unchanged. The service is
stopped automatically when the wrapper exits; `sam_cache_ready.json` records its
PID and preload time. If the cache is unavailable or returns an error, perception
fails closed instead of continuing without the requested SAM refinement.

At program startup, also start one task-local persistent RealSense service for
the head camera and every currently enumerated wrist camera. Warm each available
camera for 30 frames and keep acquiring RGB-D in the background. Read the
initial inventory from a fresh head frame and run the legacy wrist VLM+SAM hole
detector on a fresh frame from the matching wrist without reopening the device.
Require frames to be no older than 500 ms. After robot reset, discard cached
frames and wait for one new frame from each available camera. Keep the head
camera mandatory; record a physically absent wrist camera in
`realsense_cache_ready.json` and fail immediately if that missing side is later
requested.

Only when explicitly selecting cached-grid, require both wrist RealSense devices
at persistent-service startup. Cached-grid live execution fails before motion unless
left serial `347622074336` and right serial `337322072568` both produce fresh
frames. Each insertion consumes a frame no older than 500 ms, projects the
reserved cached slot into it, performs local circular-hole and occupancy checks,
and uses resident SAM for refinement. Low confidence, unreliable depth, or
uncertain occupancy falls back to VLM+SAM on that exact frame. A corrected XY
more than 15 mm from the cached slot is rejected. After robot reset, retain the
service, flush both old frames, and require fresh frames from both sides.

The initial multi-tube inventory is written to `tube_detection.json` and the
same-frame rack localization to `rack_detection.json` by default. Do not build a
rack grid in the default legacy path. Experimental cached-grid mode fits the
fixed 3x4 holes with at least 8 observed centers and at most 4 px RMS, then writes
`rack_grid.json` and `rack_grid_overlay.jpg`. Number slots row-major in the head
image as `r1c1` through `r3c4`. Track confirmed slots as
`empty -> reserved -> occupied/unknown`; never reuse `unknown` during the run.
Write each wrist decision and timing breakdown to
`wrist_perception_tube_NN.json`. Compatible
per-tube object-locator JSON files under `tubes/`. If transparent
head/tail endpoints lack depth, the existing grasp wrapper falls back to the
calibrated bbox center while retaining pixel head/tail for arm selection.
Candidates without either a calibrated bbox center or pixel orientation abort
the inventory; partial inventory requires the explicit
`--locator-arg --allow-rejected-candidates` override.

All task-level rack and wrist-hole `object-locator` calls use a camera capture
watchdog. If no RGB-D capture-ready signal arrives within 3 seconds, the task
terminates that locator process, waits for its USB handle to be released,
hardware-resets only the RealSense selected by that locator config, and retries.
Reset recovery waits 2 seconds for UVC handle release and permits up to 3 reset
attempts; each attempt gets 15 seconds to produce a frame. VLM/SAM inference time
starts after capture and is not limited by the 3-second watchdog. Advanced
overrides are available through `REALSENSE_CAPTURE_WATCHDOG_SEC`,
`REALSENSE_RESET_COOLDOWN_SEC`, `REALSENSE_RESET_CAPTURE_TIMEOUT_SEC`, and
`REALSENSE_RESET_MAX_ATTEMPTS`.

Use a strict `0.035 rad` wrist P2P rotation tolerance, matching the vertical
roll/pitch gate. Keep at most four additional in-place posture corrections and
retain the 20 mm segmented descent, force checks, settle times, and motion
speeds. Reports include monotonic capture/reset/inference, XY approach, posture
correction, and per-descent-segment timings.

Before the first full live run, validate detection without robot motion:

```bash
scripts/run_full_pick_tube_insert_rack.sh --mode live --execute \
  --stop-after-inventory --artifact-dir /tmp/tube_inventory_check
```

## Live Entry

The Python task runner live mode may open cameras, move robot arms, control
grippers, and run reset recovery. It must pass `HardwareGate`:

```bash
python3 /home/deepcybo/agentic_skills/task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/task_pick_tube_insert_rack_runner.py \
  --mode live \
  --execute \
  --auto-reset-on-abnormal
```

Do not run live mode during implementation or review.

Single-tube live shell wrapper entry, without operator-token `HardwareGate`:

```bash
/home/deepcybo/agentic_skills/task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/run_single_pick_tube_insert_rack.sh \
  --mode live \
  --execute
```

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
- In the Python task runner, live health, reset recovery, tube localization, grasp/handover, and insertion subprocesses are dispatched by `CommandRunner` only after their manifest entrypoint passes `HardwareGate`. The single-flow shell wrapper is documented separately above and does not use the operator-token `HardwareGate`.
- A successful live return still reports warnings for the current contact/force verification and retract-versus-full-home limitations.
