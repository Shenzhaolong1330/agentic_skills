---
name: atomic-motion-franka-move-to-pose
description: Move one or both dual-Franka end effectors point-to-point to absolute base-frame xyz+rotvec poses using the bundled dual_franka_robotiq RPC client. Use only for atomic P2P robot motion before higher-level grasp, handover, insertion, or recovery routines.
---

# Dual Franka P2P Motion

Use this skill when the user asks for point-to-point Cartesian motion on the dual Franka system.

Bundled RPC client:

```bash
/home/deepcybo/agentic_skills/atomic_skills/dual_franka_p2p/skills/atomic-motion-franka-move-to-pose/dual_franka_robotiq_rpc_client.py
```

RPC defaults:

- `FRANKA_RPC_HOST` or `--server-host`, default `172.16.0.1`
- `FRANKA_RPC_PORT` or `--server-port`, default `4242`
- units: meters and radians
- pose format: `[x_m, y_m, z_m, rx, ry, rz]` in robot base frame

## Atomic P2P

Dry-run plan:

```bash
python3 /home/deepcybo/agentic_skills/atomic_skills/dual_franka_p2p/skills/atomic-motion-franka-move-to-pose/scripts/move_to_pose.py \
  --mode dry_run \
  --left-pose '[0.5268, 0.0149, 0.0489, 1.7531, -1.8206, 0.8700]' \
  --right-pose '[0.5310, -0.0281, 0.0235, -1.8203, -1.7759, -0.9711]'
```

Real motion requires both `--hardware-allowed` and `--execute`:

```bash
python3 /home/deepcybo/agentic_skills/atomic_skills/dual_franka_p2p/skills/atomic-motion-franka-move-to-pose/scripts/move_to_pose.py \
  --mode live \
  --hardware-allowed \
  --right-pose '[0.5310, -0.0281, 0.0235, -1.8203, -1.7759, -0.9711]' \
  --max-translation-speed 0.04 \
  --max-rotation-speed 0.20 \
  --execute
```

If only one arm pose is passed, the other arm holds its current end-effector pose.

## Safety Checks

Before `--execute`, verify:

- pose is in the intended base frame, not camera frame
- target is inside the calibrated workspace
- speed and step limits are conservative for nearby fixtures
- the non-moving arm's current pose is collision-free
- `client.ping()` succeeds against the intended robot server

## Verification

Syntax check:

```bash
python3 -m py_compile \
  /home/deepcybo/agentic_skills/atomic_skills/dual_franka_p2p/skills/atomic-motion-franka-move-to-pose/scripts/move_to_pose.py \
  /home/deepcybo/agentic_skills/atomic_skills/dual_franka_p2p/skills/atomic-motion-franka-move-to-pose/dual_franka_robotiq_rpc_client.py
```

Dry-run does not move the robot.
