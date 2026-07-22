# S4/S5 acceptance

```bash
conda run -n agentic-gen-agent python scripts/run_gen_agent_acceptance.py \
  --phase s4-s5 --repo-root . \
  --output-dir /tmp/agentic_skills_gen_agent/S04_S05/final
```

The offline runner checks S2/S3, the all-capability static audit, manifest,
index and schema validation, dispatcher/world/verifier tests, security scans,
generic-module task terms, compileability, regressions, and hardware counters.
It uses fake backends, temporary artifacts, and injected clocks only; it never
performs real-camera, RPC, reset, motion, gripper, or true physical
verification.

S6 GoalSpec/TaskGraph Compiler, S7 Graph Executor, Planner, Recovery Graph,
and Memory remain unimplemented.
