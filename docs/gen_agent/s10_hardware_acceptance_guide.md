# S10H hardware acceptance guide

S10H is operator-only. Do not run it from Codex, CI, pytest, or automation.
S10A generated no real acceptance result and did not access hardware.

## Prepare and run

1. Generate a local config:

   `conda run -n agentic-gen-agent python scripts/generate_s10_acceptance_config.py`

2. Fill only the local config with non-secret model/fingerprint hashes,
   workspace, calibration, fixed safe pose IDs, and conservative limits. Keep
   both grippers empty, clear the shared workspace, verify E-stop access,
   camera/gripper/robot identity, RPC target, calibration, recording, and
   absence of another controller.

3. Validate H0 and audit:

   `conda run -n agentic-gen-agent python scripts/run_s10_hardware_acceptance.py audit`

4. In the physical operator terminal, run levels in order: `read-only`,
   `gripper-empty`, `motion-p2p`, `motion-relative`, `safe-stop`,
   `grasp-verification`, `fault-recovery`, and finally `reset-home`, with
   `--hardware-allowed` and (for side effects) `--execute`. The runner refuses
   non-interactive or automation contexts and never accepts arbitrary pose,
   executable, adapter, backend, script, or config arguments.

H1 samples state/cameras/localization and checks no motion. H2 uses empty
grippers. H3 uses only fixed safe acceptance poses and conservative speed. H4
checks base/tool direction and stale-pose rejection. H5 runs only when a real
stop API exists. H6 uses low-value objects and reports false positives/false
negatives. H7 uses only an existing benign recoverable fault; never create a
collision or unsafe fault. H8 is last, with empty grippers and clear home path.

Any failed level stops higher levels; do not retry automatically or raise
limits. Artifacts belong under `/tmp/agentic_skills_gen_agent/S10/hardware/`:
fingerprint, plan, events, observations, steps, result, and operator review.
Send the artifact directory or `acceptance_result.json` back for offline
analysis. Promotion emits a reviewable patch and never changes the manifest or
auto-promotes readiness.
