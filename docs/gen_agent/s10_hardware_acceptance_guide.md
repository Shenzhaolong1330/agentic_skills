# S10H hardware acceptance guide

S10H is operator-only. Do not run it from Codex, CI, pytest, or automation.
S10A generated no real acceptance result and did not access hardware.

The operator runner is bound to
`dual_franka_robotiq_zerorpc.s10.v1` and
`realsense_object_locator.s10.v1`. It can load only the bundled
`dual_franka_robotiq_rpc_client.py`, accepts no backend/client/executable
selector, requires the configured private IPv4 controller on fixed port 4242,
and validates the expected robot, arm, gripper, protocol, workspace,
calibration, and reset identities. H0 performs only static validation and
never loads that client.

The scene backend similarly fixes the three RealSense serials, viewer script,
object-locator executable, head-camera target, calibration sources, and output
layout. It has two closed locator modes:

- `grounded-sam` maps only to
  `config_rack_center_grounded_sam.yaml` and performs local inference.
- `openrouter-vlm` maps only to `config_rack_center_vlm.yaml`. The existing
  object locator obtains `OPENROUTER_API_KEY` from its local `.env` or process
  environment; the runner never reads, accepts, records, or prints that key.

Neither mode accepts a caller-provided config path, model, target, camera
serial, executable, environment map, or extra arguments.

## Prepare and run

1. Generate a local config:

   `conda run -n agentic-gen-agent python scripts/generate_s10_acceptance_config.py`

2. Fill only the local config with non-secret model/fingerprint hashes,
   workspace, calibration, fixed safe pose IDs, and conservative limits. Keep
   both grippers empty, clear the shared workspace, verify E-stop access,
   camera/gripper/robot identity, RPC target, calibration, recording, and
   absence of another controller. Set
   `acceptance.scene_locator_mode` to either `grounded-sam` or
   `openrouter-vlm`; `grounded-sam` is the default.

3. Validate H0 and audit:

   `conda run -n agentic-gen-agent python scripts/run_s10_hardware_acceptance.py audit`

   Expected status is `CONFIG_VALID`, with
   `vendor_binding.status=FIXED_VENDOR_CONFIG_VALID` and
   `scene_binding.status=FIXED_SCENE_CONFIG_VALID`, followed by vendor and
   scene backend statuses of
   `PASS_IMPLEMENTATION_READY_OFFLINE_AUDITED`. H0 hashes and checks both
   fixed scene presets, but reports the selected mode separately. It also
   checks the hardware fingerprint, fixed RPC type/private endpoint/port,
   arm/gripper/camera identity, workspace digest, and calibration-source
   binding. There must be no RPC connection, camera access, model inference,
   network request, gripper command, or robot motion.

4. In the physical operator terminal, run levels in order: `read-only`,
   `gripper-empty`, `motion-p2p`, `motion-relative`, `safe-stop`,
   `grasp-verification`, `fault-recovery`, and finally `reset-home`, with
   `--hardware-allowed` and (for side effects) `--execute`. The runner refuses
   non-interactive or automation contexts and never accepts arbitrary pose,
   executable, adapter, backend, script, or client arguments. Config paths are
   restricted to `config/local` or `config/templates`.

For H1, choose one fixed mode. The config default is used when the option is
omitted:

`conda run -n agentic-gen-agent python scripts/run_s10_hardware_acceptance.py read-only --hardware-allowed --scene-locator-mode grounded-sam`

or:

`conda run -n agentic-gen-agent python scripts/run_s10_hardware_acceptance.py read-only --hardware-allowed --scene-locator-mode openrouter-vlm`

H1 samples vendor robot/gripper state ten times, split around scene capture so
the no-motion comparison covers the camera/locator interval. It captures five
RGB-D samples from each of `head`, `left_wrist`, and `right_wrist`, then runs
five head-camera localizations of the fixed black test tube rack. Automatic
checks require exact sample counts, strictly increasing timestamps, non-empty
depth, no stale output, successful detection, confidence in `[0,1]`, an
available `base` frame, the fixed calibration hash, no motion, no gripper
commands, and no physical action calls. No camera reset is requested. A
process failure is recorded and is not retried by the S10 runner.

The operator must confirm the three image sources are correct and not swapped,
the rack is the detected object, the reported base-frame position is
reasonable, and neither arm moved. Each mode has a different adapter digest;
an H1 artifact validates only the mode it records. To compare both modes, run
them as two separate H1 artifacts and review each separately.

H2 uses empty grippers. H3 uses only fixed safe acceptance poses and
conservative speed. H4 composes bounded relative targets and delegates them to
absolute P2P; it never sends an arbitrary relative RPC. H5 records
`skipped_not_validated` because the vendor API has no real stop primitive and
never substitutes reset. H6 records `skipped_not_validated` because there is
no independent grasp verifier. H7/H8 have distinct audited vendor methods,
but the runner rejects both while E-stop, held-object, or current-error
evidence is unknown; never create a collision or unsafe fault to make H7
runnable.

Any failed level stops higher levels; do not retry automatically or raise
limits. Every real probe artifact initially has `passed: false` and
`operator_review_required`; automatic checks never self-promote a capability.
Artifacts belong under `/tmp/agentic_skills_gen_agent/S10/hardware/`:
fingerprint, plan, events, observations, steps, result, and operator review.
Send the artifact directory or `acceptance_result.json` back for offline
analysis. Promotion emits a reviewable patch and never changes the manifest or
auto-promotes readiness.
