# S8/S9 acceptance

Run the complete offline acceptance with:

```bash
conda run -n agentic-gen-agent python scripts/run_gen_agent_acceptance.py \
  --phase s8-s9 --output-dir /tmp/agentic_skills_gen_agent/S08_S09/final
```

The report covers the environment gate, the S7 regression suite, recovery
selection and rejection, platform safety guards, bounded retry/termination,
remainder replanning monotonicity, lineage/checkpoint/event integration,
task-definition compilation and migration audit, legacy compatibility, and
repeated deterministic runs. It also scans the task entrypoints to ensure
there is no live option in the offline agentic CLI and no physical execution
in the tested paths.

The required result is `status: PASS` in `acceptance.json`, with zero failed
pytest tests and zero physical executions. The known S7 insertion regression
is fixed by allowing the compatibility workflow to proceed in offline mode
while retaining the live hardware gate; the workflow still does not perform a
lift-only insertion retry. S10, not S8/S9, is the stage for real Adapter and
pre-hardware audit work.
