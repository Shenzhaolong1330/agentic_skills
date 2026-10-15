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

If only one arm pose is passed, the other arm holds its current end-effector pose.

## Continuous correction and legacy retries

For the console grasp path, keep one atomic call alive for the entire correction
budget. The initial trajectory plus `max_correction_iters=9` permits at most ten
trajectory/measurement rounds. Return as soon as both arms pass position and
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
[task-pick-tube-insert-rack](../../../../task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/SKILL.md#continuous-correction-used-by-the-console):
`--p2p-continuous-corrections --p2p-retries 9`. The generic console explicitly uses
3 mm / 0.03 rad arrival tolerances and 25 mm / 0.20 rad compensation caps.

Existing callers retain `stall_policy="stop"`. The standalone atomic CLI still
makes one call. Legacy task outer retries remain available without the new flag;
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
