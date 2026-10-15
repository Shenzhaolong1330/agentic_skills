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

During a multi-tube live run, a failed grasp/handover or insertion now stops the
loop without an automatic reset or advancing to another cached tube. The default
`--no-reset-after-error` also stops; it no longer means continuing without reset.
`--reset-after-error` explicitly enables the legacy full-reset-and-advance policy.
Full reset opens both grippers and returns both arms Home.

### Continuous correction used by the console

The generic console calls the Python grasp script directly with
`--p2p-continuous-corrections --p2p-retries 9`. This uses **one atomic call**:
one initial trajectory and at most nine subsequent bounded correction rounds.
It synchronizes the command reference only once and preserves accumulated
compensation within the stage. It returns immediately when all four measured
position/rotation errors pass. It never restarts the whole target after the
continuous budget is exhausted.

The console explicitly selects 3 mm / 0.10 rad arrival tolerances, honoring the
user-selected rotation limit, and two seconds
settling per round with the existing 25 mm / 0.20 rad compensation caps.
The separate precision probe defaults to 0.03 rad for stricter research; failing
that stricter test must not be reported as a pass.
`p2p_correction_progress` logs the nominal-goal residuals, historical best and
plateau/improvement status on every round. A plateau warns and spends the
remaining correction budget. A component outside arrival tolerance that grows
more than 0.5 mm or 0.005 rad above its historical best stops the stage; this
noise margin does not count as arrival or allow accumulated small divergence.

Every feedback packet must have fresh, healthy native diagnostics. This mode
selects `feedback_mode="read_after_step"` because the field server returns raw
observations from `step`; one read-only observation after each acknowledged
command supplies native health. Its count must equal the acknowledged command.
The atomic client also checks session identity and saved-target continuity.
Faults, unexpected commands, missing feedback and uncertain RPC outcomes stop
without replay. Reaching the correction or compensation limit while still
outside tolerance prevents subsequent descent, gripper or handover stages.
Continuous mode cannot restart controllers or apply a synthetic observed-Z
arrival offset. Existing standalone/shell callers remain on their legacy policy
unless they explicitly select the new flag.

### Bounded same-target P2P retries

The shared grasp shell defaults to `--p2p-retries 9`: each motion stage may call
P2P at most **10 times total**, including any explicitly enabled controller-recovery
retry. It returns immediately once all four measured pose errors pass their
tolerances; ten is a ceiling, not a required number of calls. Direct Python calls
retain the compatible default `--p2p-retries 2` (**3 calls total**). Pass
`--p2p-retries 0` for a single call. Through the tail-side launcher, use
`--grasp-arg=--p2p-retries --grasp-arg=9` to explicitly select the ten-call ceiling.
The older full-task live bridge uses this shell. The generic console grasp adapter
uses the continuous policy above through direct Python invocation.

After each completed call, read both measured poses and compare all four position
and rotation errors with the original stage tolerances. Proceed only when all
four pass. The shared shell enables `--warn-and-retry-stall`: a stalled call, or
less than 2% progress without increasing error, emits `WARNING: p2p_stall_retry:`
and can retry the **same absolute target** within the remaining budget. This
requires fresh native diagnostics for both arms, explicitly reporting a healthy
robot mode, no active errors and a progressing sensor clock. Missing diagnostics
are not evidence of health. Each retry rechecks feedback before sending motion.

Without this flag, direct Python calls keep the stricter existing policy: the
worst `error / tolerance` must decrease by at least 2% relative to the previous
attempt (the first call uses its initial measured poses as the baseline).
Configure that threshold with `--p2p-retry-min-progress-ratio 0.02`.
In warning mode, an increase exceeding the normalized 0.5 mm / 0.005 rad noise
margin still stops the legacy outer loop. Reported robot/collision faults, stale or missing required
feedback, and RPC errors still stop the stage. Reaching the call limit without
arrival also stops; a warning never authorizes subsequent gripper or motion
stages at an out-of-tolerance pose.

This also permits a new call after `tracking_compensation_limit` under the same
retry policy. Each atomic P2P call synchronizes its command reference
to the latest pose; its previous compensation is not carried over. Keep the same
target, speeds, step sizes, tolerances, and 25 mm / 0.20 rad compensation limits.
Repeated calls are a bounded convergence attempt, not a guarantee of success.

The normal shell keeps controller recovery disabled. Single-arm stages detect
three consecutive low-motion observations inside each call; the dual-arm handover
uses the outer loop's measured convergence check after each call.
Explicit legacy `--recover-stalled-controller`
remains separate and consumes the same total call budget. Legacy
`--continue-on-tolerance-miss` is honored only with `--p2p-retries 0`; it cannot
turn exhaustion of this retry loop into success. Logs print each attempt number,
both arms' residuals and the normalized convergence score. Native telemetry, when
present, must be at most 250 ms old and report idle/move without active errors;
older pose-only servers still require the existing external health preflight
and cannot use warning-based stall retries.

Grasping now waits for fresh closure feedback stable for 0.5 seconds before lifting.
The shared grasp shell sets `--transition-position-tolerance-m 0.005` for the
handover pose separately from the general position tolerance. The 00:27 and 00:34
runs on 2026-09-20 both accepted roughly 11–13 mm residuals under the previous
15 mm stage4 tolerance; their calibration targets were identical. Rotation tolerance
remains 0.06 rad. The bounded retry policy also applies to the handover motion.
After transfer, donor opening must be confirmed (closed fraction at most 0.1 for
0.3 seconds of fresh feedback) before retreat or insertion. The full multi-tube
wrapper passes the receiver from `handover_tube_N.json` to insertion explicitly;
it no longer guesses the holder from two transient gripper positions. A missing
handover report or unconfirmed opening stops insertion.

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
watchdog. If no RGB-D capture-ready signal arrives within 30 seconds, the task
terminates that locator process, waits for its USB handle to be released,
tries one clean reopen, then at most one hardware reset by default of only the
selected RealSense. Recovery waits 2 seconds for UVC handle release; each attempt
gets 30 seconds to produce a frame. Per-serial process locks prevent a second owner
from opening/resetting the same camera; hardware resets are at least 60 seconds apart.
Do not repeatedly set a preset that is already active. VLM/SAM inference time
starts after capture and is not limited by the capture watchdog. Advanced
overrides are available through `REALSENSE_CAPTURE_WATCHDOG_SEC`,
`REALSENSE_RESET_COOLDOWN_SEC`, `REALSENSE_RESET_CAPTURE_TIMEOUT_SEC`, and
`REALSENSE_RESET_MAX_ATTEMPTS`.
See [RealSense operation guide](../../../../docs/realsense_operation.md) for ownership,
bounded recovery, USB diagnostics, and local-only capture checks.

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

Reverse rack-to-table orange-cap flow:

```bash
/home/deepcybo/agentic_skills/task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/run_pick_yellow_cap_to_table.sh \
  --mode live \
  --execute \
  --holder-side right
```

This first locates the rack from the head camera, then moves only the selected
arm directly to the rack observation position with the vertical TCP posture in
the same P2P target. It then locates the orange cap from that wrist camera. The
wrist config uses `depth.position_anchor: bbox_corners`; the four
corner 3D points are averaged for `points_base.bbox_center`. `--place-xyz` is
optional: by default a second head-camera VLM call selects an unobstructed
blank tabletop area. The detected table surface z is raised by
`--place-z-offset-m` (default 0.10 m) to obtain the TCP release height. Use
`--place-xyz` only when overriding the VLM-selected point.

After the active wrist arm grasps the cap, it lifts 10 cm, returns through the
partner-side transition state, hands the cap to the other gripper, and the
partner arm performs the table placement and release. The placement helper
uses the locator's `points_base.bbox_center.base` 3D point, changes the
partner TCP to the vertical-down posture before the XY move, and then descends
to the z-offset release point.

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
- The full insertion wrapper now orients the right holder's configured local jaw-opening axis perpendicular to the rack long axis. It derives that axis from the cached rack grid when available, otherwise from the cached rack bbox long edge; use --jaw-opening-axis y or --rack-axis-base '[x, y, z]' when the local TCP convention or rack direction needs an explicit override. The signed branch defaults to --jaw-perpendicular-turn ccw; use cw to select the opposite branch.
- The actual Robotiq TCP local axis used for jaw opening must still be validated once against the robot's flange/TCP calibration; for the current grasp/TCP convention the default is local y, while local x is used to align the gripper body with the tube.
- Multi-candidate empty-hole support is wrapped in task logic; native object_locator candidate output should be added later for live perception.
- In the Python task runner, live health, reset recovery, tube localization, grasp/handover, and insertion subprocesses are dispatched by `CommandRunner` only after their manifest entrypoint passes `HardwareGate`. The single-flow shell wrapper is documented separately above and does not use the operator-token `HardwareGate`.
- A successful live return still reports warnings for the current contact/force verification and retract-versus-full-home limitations.
