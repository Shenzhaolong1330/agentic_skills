---
name: atomic-state-dual-franka-reset
description: Inspect dual-Franka RPC telemetry and native Franka diagnostic fields to decide whether either arm needs reset, then conditionally run the existing robot-reset command with post-reset verification. Use when Codex needs to check whether the left or right Franka is abnormal, diagnose current_errors or robot_mode, recover a faulted dual-Franka setup, or avoid an unnecessary reset before another robot skill.
---

# Dual Franka Conditional Reset

Inspect first. Run `robot-reset` only for an explicit current fault, and verify state again afterward.

Bundled script:

```bash
/home/deepcybo/agentic_skills/atomic_skills/dual_franka_reset/skills/atomic-state-dual-franka-reset/scripts/check_and_reset.py
```

Defaults:

- RPC: `FRANKA_RPC_HOST` / `FRANKA_RPC_PORT`, default `172.16.0.1:4242`
- RPC client: the existing `dual_franka_robotiq_rpc_client.py`
- reset config: `/home/deepcybo/Le-nero/dual_arm_teleop/scripts/config/record_cfg.yaml`
- reset runner: the existing `procedure-robot-reset-home` wrapper

## Check Status

Run the read-only check first:

```bash
python3 /home/deepcybo/agentic_skills/atomic_skills/dual_franka_reset/skills/atomic-state-dual-franka-reset/scripts/check_and_reset.py \
  --mode live --hardware-allowed status
```

Interpret `classification`:

- `healthy`: both arms have valid telemetry and explicit healthy diagnostic evidence.
- `needs_reset`: `current_errors` is active, `needs_reset`/equivalent is true, or `robot_mode` is `REFLEX`. Real reset additionally requires `auto_reset_allowed: true`, which proves both arms have complete telemetry and resolved current diagnostics.
- `manual_intervention`: `robot_mode` is `USER_STOPPED`; clear the stop condition manually. Concurrent reset reasons remain visible but do not authorize motion.
- `recovering`: `robot_mode` is `AUTOMATIC_ERROR_RECOVERY`; wait and check again. Concurrent reset reasons remain visible but do not authorize motion.
- `unknown`: communication, telemetry, or diagnostic evidence is insufficient. Do not auto-reset.

Do not use `last_motion_errors` alone to trigger reset; it describes the previous motion, not necessarily a current fault.

The currently deployed ZeroRPC observation contains joint, pose, wrench, and gripper telemetry but no native `current_errors` or `robot_mode`. In strict mode this correctly returns `unknown` instead of claiming the arms are healthy. Use `--allow-telemetry-only` only with `status` when a connectivity/telemetry check is sufficient; it is rejected by `ensure` and cannot rule out a Franka controller fault.

## Reset Only If Needed

Preview the conditional action without moving the robot:

```bash
python3 /home/deepcybo/agentic_skills/atomic_skills/dual_franka_reset/skills/atomic-state-dual-franka-reset/scripts/check_and_reset.py \
  --mode dry_run ensure
```

After completing the safety checks, allow real execution:

```bash
python3 /home/deepcybo/agentic_skills/atomic_skills/dual_franka_reset/skills/atomic-state-dual-franka-reset/scripts/check_and_reset.py \
  --mode live --hardware-allowed --execute ensure
```

`--mode live --hardware-allowed --execute ensure` runs `robot-reset` only when the pre-check is `needs_reset`. It then reconnects to RPC and writes `status_after` into the JSON report. The manifest must also mark the reset entrypoint as recovery-allowed.

Before planning or executing, the script resolves `robot_ip` and `robot_port` from the reset config (including `scripts/config/robots/franka_config.yaml` when needed) and requires them to exactly match the RPC endpoint that was inspected. A mismatch is a hard failure so one robot's fault cannot reset another robot.

Exit codes:

- `0`: healthy, or reset completed and post-check is healthy
- `2`: `status` found reset is needed, or `ensure` only planned it because live hardware authorization or `--execute` was omitted
- `3`: status is `unknown`, `manual_intervention`, or `recovering`, or fault evidence exists without complete dual-arm telemetry and diagnostics; no reset was attempted
- `1`: RPC, reset command, or post-reset verification failed

## Safety

Before `--execute`, verify all of the following:

- Both arm workspaces and the home paths are clear.
- Neither gripper holds an unsupported object.
- The selected config targets the intended dual-Franka server.
- The emergency stop is reachable.

With the current Franka config, `robot-reset` moves both arms home and opens both grippers. It is not a motion-free error-clear operation. The dual-Franka RPC server separately exposes `recover_robot`/`recover`, which is the dedicated error-recovery path, but this skill follows the requested `robot-reset` workflow and reports a failure if the fault remains.

An E-stop or unsafe state is never automatically cleared. After E-stop, only read-only diagnosis or a request for manual intervention is permitted. Reset and Home are physical state changes, not E-stop recovery.

If `robot-reset` times out, treat the robot as still active until physically verified: the script terminates the local process group, but a trajectory already accepted by the remote server may continue. Use the emergency stop when motion is unsafe.

## Verification

```bash
python3 -m py_compile \
  /home/deepcybo/agentic_skills/atomic_skills/dual_franka_reset/skills/atomic-state-dual-franka-reset/scripts/check_and_reset.py

python3 -m unittest discover -s \
  /home/deepcybo/agentic_skills/atomic_skills/dual_franka_reset/tests -v
```
