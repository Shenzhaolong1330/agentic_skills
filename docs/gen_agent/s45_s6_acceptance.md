# S4.5 + S6 acceptance

Acceptance is offline and hardware-free. It audits all manifest capabilities, fixed Adapter security, plan-only live rejection, schema contracts, valid and invalid graph fixtures, malicious-field rejection, workspace/verifier/cycle/resource policies, deterministic hashes, CLI output, performance, and S0–S5 regressions.

The current known baseline failure remains `test_flow_does_not_retry_insert_by_lifting`; it is not a new regression. No real camera, RPC, reset, motion, or gripper operation is permitted. A passing dry-run plan is not a live-support claim and no Adapter has been accepted on a real robot in this phase.

S7 Graph Executor, S8 Recovery Engine, S11 Codex Planner, and long-term Memory are not implemented.
