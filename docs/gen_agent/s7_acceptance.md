# S7 acceptance

Run the offline acceptance suite with:

```text
conda run -n agentic-gen-agent python scripts/run_gen_agent_acceptance.py --phase s7 --repo-root . --output-dir /tmp/agentic_skills_gen_agent/S07/final
```

The suite runs the S4.5/S6 acceptance, environment and schema checks, contract/lifecycle/binding/dispatcher/world/verifier/resource/budget/event/checkpoint/CLI tests, 25 valid bounded scenarios, invalid and malicious path fixtures, ten resume scenarios, and 100 deterministic runs. It also scans for live CLI exposure, direct adapter/backend calls from Executor code, path escapes, and hardware calls.

The expected known baseline failure remains `tests/test_insertion_retry_logic.py::InsertionRetryLogicTests::test_flow_does_not_retry_insert_by_lifting`; it must not be hidden or weakened. S7 must perform zero physical execution and report zero physical goal verification.
