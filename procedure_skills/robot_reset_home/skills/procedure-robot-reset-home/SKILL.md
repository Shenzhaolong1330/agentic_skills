---
name: procedure-robot-reset-home
description: Reset, recover, or move the configured dual Franka robot to its saved Home pose through guarded le_nero wrappers. Use when the user asks to run robot-reset, clear a Franka controller fault without returning Home, return one or both arms Home without changing gripper state, or restore the robot to its initial pose.
---

# Robot Reset Home

Choose the narrowest command that matches the requested operation.

## Full Reset

> **Safety warning:** Avoid running a full reset while either gripper is holding a test tube. The default Franka reset configuration opens the grippers, so the tube can fall, break, spill, or damage nearby equipment. Before resetting, place the tube in a safe location or complete a safe handover. If the tube must remain held, prefer `run_robot_go_home.sh` or `run_robot_recover.sh` as appropriate because those wrappers preserve the gripper state.

Run the configured `robot-reset` workflow:

```bash
/home/deepcybo/agentic_skills/procedure_skills/robot_reset_home/skills/procedure-robot-reset-home/scripts/run_robot_reset.sh \
  --mode live --hardware-allowed --execute
```

What it does:

1. sources `/home/deepcybo/miniconda3/etc/profile.d/conda.sh`
2. activates `le_nero`
3. changes to `/home/deepcybo/Le-nero/dual_arm_teleop`
4. runs `robot-reset`

`robot-reset` still internally loads its default `scripts/config/record_cfg.yaml` to select `record.robot_type`. Reset behavior such as `reset_go_home`, `go_home_duration_sec`, and `reset_opens_grippers` lives in the robot detail config, for Franka:

```bash
/home/deepcybo/Le-nero/dual_arm_teleop/scripts/config/robots/franka_config.yaml
```

Override config only if needed:

```bash
/home/deepcybo/agentic_skills/procedure_skills/robot_reset_home/skills/procedure-robot-reset-home/scripts/run_robot_reset.sh \
  --mode live --hardware-allowed --execute \
  --config /home/deepcybo/Le-nero/dual_arm_teleop/scripts/config/record_cfg.yaml
```

## Go Home Without Changing Grippers

Move both arms to the RPC server's saved Home pose. Do not open or close either gripper:

```bash
/home/deepcybo/agentic_skills/procedure_skills/robot_reset_home/skills/procedure-robot-reset-home/scripts/run_robot_go_home.sh \
  --mode live --hardware-allowed --execute
```

Move only one arm or tune the trajectory:

```bash
/home/deepcybo/agentic_skills/procedure_skills/robot_reset_home/skills/procedure-robot-reset-home/scripts/run_robot_go_home.sh \
  --mode live --hardware-allowed --execute \
  --side left_arm \
  --duration-sec 5 \
  --rate-hz 50
```

This command does not perform controller error recovery. If the arm is faulted, `go-home` can fail; recover first, verify the state, and then request Home motion.

## Recover Without Going Home

Request Franka controller error recovery for both arms without commanding a Home trajectory or changing either gripper:

```bash
/home/deepcybo/agentic_skills/procedure_skills/robot_reset_home/skills/procedure-robot-reset-home/scripts/run_robot_recover.sh \
  --mode live --hardware-allowed --execute
```

Recover one arm only:

```bash
/home/deepcybo/agentic_skills/procedure_skills/robot_reset_home/skills/procedure-robot-reset-home/scripts/run_robot_recover.sh \
  --mode live --hardware-allowed --execute \
  --side right_arm
```

Both RPC wrappers default to `FRANKA_RPC_HOST` / `FRANKA_RPC_PORT`, falling back to `172.16.0.1:4242`. Use `--server-host` and `--server-port` to override them.

## Command Differences

- `run_robot_reset.sh`: run the configured reset workflow; it may move both arms Home and open grippers.
- `run_robot_go_home.sh`: move the selected arm(s) Home; preserve gripper state.
- `run_robot_recover.sh`: request controller error recovery; do not command Home or grippers.

Safety checks before running:

- Robot workspace is clear.
- Neither gripper is holding a test tube or another object that could fall when a full reset opens the grippers.
- The correct robot server/config is active.
- Emergency stop is reachable.
- E-stop or another unsafe state is not automatically cleared. After E-stop, use only read-only diagnosis or request manual intervention; reset and Home are physical operations, not E-stop recovery.

Verification:

```bash
bash -n \
  /home/deepcybo/agentic_skills/procedure_skills/robot_reset_home/skills/procedure-robot-reset-home/scripts/run_robot_reset.sh \
  /home/deepcybo/agentic_skills/procedure_skills/robot_reset_home/skills/procedure-robot-reset-home/scripts/run_robot_go_home.sh \
  /home/deepcybo/agentic_skills/procedure_skills/robot_reset_home/skills/procedure-robot-reset-home/scripts/run_robot_recover.sh
```
