---
name: atomic-motion-franka-move-to-pose
description: Move one or both dual-Franka end effectors point-to-point to absolute torso-frame xyz+rotvec poses using the bundled dual_franka_robotiq RPC client. Use only for atomic P2P robot motion before higher-level grasp, handover, insertion, or recovery routines.
---

# Dual Franka P2P Motion

Use this skill when the user asks for point-to-point Cartesian motion on the dual Franka system.

Bundled RPC client:

```bash
/home/deepcybo/agent_ws/agentic_skills/atomic_skills/dual_franka_p2p/skills/atomic-motion-franka-move-to-pose/dual_franka_robotiq_rpc_client.py
```

RPC defaults:

- `FRANKA_RPC_HOST` or `--server-host`, default `172.16.0.1`
- `FRANKA_RPC_PORT` or `--server-port`, default `4242`
- units: meters and radians
- pose format: `[x_m, y_m, z_m, rx, ry, rz]` in the shared `torso` frame (not either native arm-base frame)

## Atomic P2P

Dry-run plan:

```bash
python3 /home/deepcybo/agent_ws/agentic_skills/atomic_skills/dual_franka_p2p/skills/atomic-motion-franka-move-to-pose/scripts/move_to_pose.py \
  --left-pose '[0.5268, 0.0149, 0.0489, 1.7531, -1.8206, 0.8700]' \
  --right-pose '[0.5310, -0.0281, 0.0235, -1.8203, -1.7759, -0.9711]'
```

Real motion requires `--execute`:

```bash
python3 /home/deepcybo/agent_ws/agentic_skills/atomic_skills/dual_franka_p2p/skills/atomic-motion-franka-move-to-pose/scripts/move_to_pose.py \
  --right-pose '[0.5310, -0.0281, 0.0235, -1.8203, -1.7759, -0.9711]' \
  --max-translation-speed 0.04 \
  --max-rotation-speed 0.20 \
  --execute
```

If only one arm pose is passed, the other arm's current measured pose is used as
its nominal goal. In continuous mode, an already-arrived stationary arm keeps
its existing saved command compensation and receives explicit zero increments.

## Select the P2P controller

Standalone scripts and Python callers default to `segmented` when
`FRANKA_P2P_CONTROLLER` is unset. Select `continuous_servo` using one of:

- Python API: `client.dual_robot_move_to_ee_pose(..., controller="continuous_servo")`.
- Atomic CLI: add `--controller continuous_servo` to `scripts/move_to_pose.py`.
- Child-process environment: `FRANKA_P2P_CONTROLLER=continuous_servo`.

An explicit API/CLI controller overrides the environment. Only `segmented` and
`continuous_servo` are valid. Continuous mode requires `smooth=True` and real
timed dispatch; it rejects `smooth=False` and `sleep=False`.

The console selects `continuous_servo` by default for free-space P2P subprocesses
through this environment variable. To roll those commands back to the retained
implementation, set `"p2p_controller": "segmented"` in
`/home/deepcybo/agent_ws/robot_agent_lab/configs/runtime.json` before a new console
task. The console routes contact insertion and Home/reset procedures through
their existing policies; changing the free-space P2P controller does not replace
those procedures. The `p2p_controller_selected` event records the actual route.

## Continuous feedback controller

`continuous_servo` runs a timed outer feedback loop targeting 20 Hz for console
calls (at least 50 ms between dispatches; a lower requested rate is respected). Each successful
raw `step` acknowledgement is followed by one fresh native observation before
the next proposal. There is no per-correction settling pause, `step(None)`
reanchor, or outer same-target retry. The nominal goal stays fixed and command
compensation starts from the server's confirmed saved target.

If that saved target is outside the goal's compensation limits, a quintic nominal
approach first moves the saved command toward the goal using the caller's speed
and step limits. Translation and rotation are checked independently: an
orientation approach retains valid saved translation compensation, and vice
versa. Comparisons use the same numerical roundoff allowance as the servo math.
The slow feedback phase then uses at most **2 mm/s** translation
and **0.01 rad/s** rotation, with bounded acceleration and command steps. Its
rotation compensation limit remains at most **0.20 rad** around the nominal
goal. The standalone translation compensation limit defaults to **25 mm**, but
accepts a finite caller limit or `max_tracking_translation_bias_m=None` to remove
the fixed translation radius. The console explicitly sets
`FRANKA_P2P_TRANSLATION_BIAS_LIMIT_M=none`, overriding legacy numeric arguments
for its continuous free-space subprocesses. The effective result field
`servo_config.max_translation_bias_m` is then JSON `null`, not infinity.
Speed, acceleration, step, timing and feedback checks still apply. The controller
never treats reaching any remaining compensation limit as arrival.

Without a fixed translation radius, near-goal correction retains the saved
command even if its bias exceeds 25 mm. Far-position nominal approach translates
the previous saved-minus-measured offset along with the new goal. The 25 mm
nominal-approach selection distance is a trajectory-mode threshold, not a
correction cap. A fixed observation corridor includes the nominal path and the
finite distance allowed by slow speed and time budget; it is not a whole-arm
collision-clearance proof.

Arrival uses the actual source-bound native torso pose. Effective tolerances are
`min(caller_position_tolerance, 0.003 m)` and
`min(caller_rotation_tolerance, 0.10 rad)`: this mode can tighten a caller's
tolerance but cannot loosen it. Once the measured pose first enters tolerance,
the command smoothly brakes and retains its compensation. Completion then
requires a stationary hold lasting at least 0.5 seconds plus a fresh final
verification. Losing tolerance after braking fails; the integrator does not
restart automatically.

The slow servo has a **30-second budget including cap waits**; the final read-only hold has
a **6-second limit**. `max_steps` (default 3000) also bounds the total number of
nominal and servo commands. These are ceilings, not a promise to converge or a
30-second limit for the entire task. Legacy `max_correction_iters` and
`settle_time_sec` do not create repeated correction rounds in this mode.

If a bounded command stops at its compensation cap before arrival, emit
`p2p_servo_cap_warning` and keep checking fresh native feedback within that same
budget. When all commands are stationary, this is read-only: do not send zero
steps or change the saved targets/count. A later feasible inward correction may
resume the same loop. Faults or external commands still fail immediately; a
latched arrival that is lost never restarts. Budget exhaustion reports remaining
errors, saved biases and caps, without declaring the target physically unreachable.

Both arms must provide fresh source-bound native poses, complete healthy native
diagnostics and advancing timestamps/sequences. Every command verifies the raw
ACK, unchanged server session, exact next command count, and matching saved
targets before committing the internal proposal. Missing feedback, faults,
unexpected commands, limits and uncertain RPC outcomes raise without replay or
an automatic retry. Existing caller feedback guards still run; the legacy
`correction_guard` is called only after verified final arrival in this mode.
Events `p2p_servo_start`, `p2p_servo_progress`, `p2p_servo_arrived` and
`p2p_servo_failed` expose the selected controller, errors, bounds and timing.

This is an integrated continuous outer-loop candidate. Offline tests and local
precision probes do not establish that a complete grasp/handover/insertion task
has been verified with this controller. Resource ownership, independent
controller receipt checks and physical task verification remain caller duties.

## Legacy segmented correction and retries

The following behavior applies to `controller="segmented"`. The task flag
`--p2p-continuous-corrections` retains its older meaning: one atomic call with
discrete trajectory/measurement/settle rounds. It does **not** select the new
20 Hz feedback controller. The initial trajectory plus `max_correction_iters=9`
permits at most ten trajectory/measurement rounds. Return as soon as both arms pass position and
rotation tolerances. Keep the nominal goal fixed and accumulate only bounded
command compensation; a new atomic call synchronizes to measured pose and loses
that compensation.

The Python API accepts `stall_policy="warn"`, a required
`feedback_guard(observation)`, and optional `correction_guard(final_error)`.
The feedback guard must reject missing/stale native health, faults and collision
indications. It runs on the initial observation, synchronization reply, every
streamed acknowledgement and each correction observation. With the field server,
set `feedback_mode="read_after_step"`: its raw `step` reply has no native health.
After a successful reply, read `get_observation()` once and require its command
count to match the acknowledged step. A failed read never causes motion replay.
The default `step_reply` mode is for servers with complete reply observations.
Warning mode also verifies the
server session, step count and both saved command targets. Any inconsistent or
uncertain RPC result ends the call without replay.

`correction_guard` runs at every measured correction boundary, including arrival
and budget exhaustion. The task uses per-component historical-best errors and
0.5 mm / 0.005 rad noise margins to distinguish a plateau from growing error.
These margins do **not** enlarge arrival tolerances. A stall warns and continues
inside the same call; faults, significant divergence, compensation limits or
budget exhaustion still prevent out-of-tolerance completion.

The task entry point is
[task-pick-tube-insert-rack](../../../../task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/SKILL.md#legacy-segmented-correction-within-one-call):
`--p2p-continuous-corrections --p2p-retries 9`. These flags remain available when
the console is rolled back to `segmented`. The console grasp profile uses
3 mm / 0.10 rad arrival tolerances (the user-selected rotation limit) and
25 mm / 0.20 rad compensation caps. The isolated precision probe defaults to the
stricter 0.03 rad research target and reports that result separately.

Existing callers retain `stall_policy="stop"`. The standalone atomic CLI still
makes one call. Legacy task outer retries remain available without
`--p2p-continuous-corrections`;
each new call synchronizes its saved reference and starts compensation again.
Never repeat a relative delta or replay an RPC whose outcome is unknown.

## Safety Checks

Before `--execute`, verify:

- pose is in the shared `torso` frame, not camera or native arm-base frame
- target is inside the calibrated workspace
- speed and step limits are conservative for nearby fixtures
- the non-moving arm's current pose is collision-free
- `client.ping()` succeeds against the intended robot server

## Verification

Syntax check:

```bash
python3 -m py_compile \
  /home/deepcybo/agent_ws/agentic_skills/atomic_skills/dual_franka_p2p/skills/atomic-motion-franka-move-to-pose/scripts/move_to_pose.py \
  /home/deepcybo/agent_ws/agentic_skills/atomic_skills/dual_franka_p2p/skills/atomic-motion-franka-move-to-pose/dual_franka_robotiq_rpc_client.py
```

Dry-run does not move the robot.
