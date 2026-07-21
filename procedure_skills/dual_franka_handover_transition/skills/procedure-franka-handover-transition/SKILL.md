---
name: procedure-franka-handover-transition
description: Execute or plan the fixed dual-Franka two-hand handover transition: move both arms to the calibrated transition pose for the active holding arm, close the partner gripper, and open the active gripper.
---

# Dual Franka Handover Transition

Use this skill when the object is already held by one hand and must be transferred to the other hand at the calibrated transition pose.

This is a procedure skill: it composes existing atomic P2P motion and gripper open/close. It does not do new perception or dynamic planning.

RPC client comes from the P2P atomic skill:

```bash
/home/deepcybo/agentic_skills/atomic_skills/dual_franka_p2p/skills/atomic-motion-franka-move-to-pose/dual_franka_robotiq_rpc_client.py
```

Bundled transition config:

```bash
/home/deepcybo/agentic_skills/procedure_skills/dual_franka_handover_transition/config/transition.json
```

`transition.json` has arm-specific entries under `transition_states.right_arm` and `transition_states.left_arm`. The active arm means the arm currently holding the object before transfer.

## Plan Transition

Dry-run plan, no robot motion:

```bash
python3 /home/deepcybo/agentic_skills/procedure_skills/dual_franka_handover_transition/skills/procedure-franka-handover-transition/scripts/handover_transition.py \
  --mode dry_run --active-arm right
```

## Execute Transition

Real execution requires both `--hardware-allowed` and `--execute`:

```bash
python3 /home/deepcybo/agentic_skills/procedure_skills/dual_franka_handover_transition/skills/procedure-franka-handover-transition/scripts/handover_transition.py \
  --active-arm right \
  --mode live --hardware-allowed \
  --execute
```

Execution sequence:

1. load transition target for `--active-arm`
2. reanchor with `client.step(None)`
3. move both arms to the transition target
4. close the partner gripper
5. open the active gripper

To move to transition without the gripper transfer:

```bash
--no-transfer-release
```

## Safety Checks

Before `--execute`, verify:

- the active arm is actually holding the object
- both grippers are initialized and responding
- transition pose matches the current fixture and object length
- the transition path is clear for both arms
- the partner gripper can close before the active gripper opens

## Verification

```bash
python3 -m py_compile \
  /home/deepcybo/agentic_skills/procedure_skills/dual_franka_handover_transition/skills/procedure-franka-handover-transition/scripts/handover_transition.py
```
