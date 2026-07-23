# S10A offline acceptance

Run:

`conda run -n agentic-gen-agent python scripts/run_gen_agent_acceptance.py --phase s10-offline --repo-root . --output-dir /tmp/agentic_skills_gen_agent/S10/offline`

The phase performs environment/prerequisite checks, S8/S9 regression, static
implementation audit, readiness/schema/policy tests, meta-operation contract
tests, acceptance CLI/security scans, compileall, index and documentation
checks. Fake backends and temporary artifacts are allowed; real camera, RPC,
reset, motion, gripper, ROS, vendor SDK, and live manifest entrypoints are not.

The successful S10A status is
`PASS_IMPLEMENTATION_HARDWARE_ACCEPTANCE_PENDING`. It means code and offline
contracts are ready for operator acceptance, not that a robot was validated.
