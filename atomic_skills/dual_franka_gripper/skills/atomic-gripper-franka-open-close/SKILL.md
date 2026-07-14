---
name: atomic-gripper-franka-open-close
description: Open, close, initialize, or inspect the left/right Robotiq grippers on the dual Franka robot via the existing RPC client. Use for atomic gripper actions inside grasp, handover, insertion, reset, or manual recovery workflows.
---

# Dual Franka Gripper

Use this skill for only gripper state/action, not arm motion.

RPC client comes from the P2P atomic skill:

```bash
/home/deepcybo/agentic_skills/atomic_skills/dual_franka_p2p/skills/atomic-motion-franka-move-to-pose/dual_franka_robotiq_rpc_client.py
```

RPC defaults:

- `FRANKA_RPC_HOST` or `--server-host`, default `172.16.0.1`
- `FRANKA_RPC_PORT` or `--server-port`, default `4242`
- sides: `left`, `right`, `both`

## Status

Status is read-only and does not need `--execute`:

```bash
python3 /home/deepcybo/agentic_skills/atomic_skills/dual_franka_gripper/skills/atomic-gripper-franka-open-close/scripts/gripper_control.py \
  status --side both
```

## Open / Close

Dry-run plan:

```bash
python3 /home/deepcybo/agentic_skills/atomic_skills/dual_franka_gripper/skills/atomic-gripper-franka-open-close/scripts/gripper_control.py \
  close --side right
```

Real command:

```bash
python3 /home/deepcybo/agentic_skills/atomic_skills/dual_franka_gripper/skills/atomic-gripper-franka-open-close/scripts/gripper_control.py \
  close --side right --execute
```

Open:

```bash
python3 /home/deepcybo/agentic_skills/atomic_skills/dual_franka_gripper/skills/atomic-gripper-franka-open-close/scripts/gripper_control.py \
  open --side left --execute
```

Initialize/reactivate:

```bash
python3 /home/deepcybo/agentic_skills/atomic_skills/dual_franka_gripper/skills/atomic-gripper-franka-open-close/scripts/gripper_control.py \
  initialize --side both --execute
```

## Safety Checks

Before closing, verify the target object or fixture is in the gripper and fingers will not clamp cables, rack edges, or the other arm.

Before opening, verify the object is supported by the receiving hand, rack, or table.

## Verification

```bash
python3 -m py_compile \
  /home/deepcybo/agentic_skills/atomic_skills/dual_franka_gripper/skills/atomic-gripper-franka-open-close/scripts/gripper_control.py
```
