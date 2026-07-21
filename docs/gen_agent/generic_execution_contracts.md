# Generic execution contracts (S2)

`agentic_skills_harness.contracts` is independent of any one task package or
hardware client. It defines `NodeStatus`, `TaskTerminalState`, capability and
risk enums, structured results, budgets, resources, and state invalidation.
The older `TaskState`, `StageResult`, and `TaskResult` remain in
`agentic_skills_harness.types` as legacy compatibility for the existing runner;
they are not replaced in this phase.

An `ActionResult` deliberately keeps five observations separate:

| Question | Field |
| --- | --- |
| Was the request accepted? | `command_accepted` |
| Did the command run? | `command_executed` |
| Did the controller reach its target? | `controller_target_reached` |
| Was a physical effect observed? | `effect_observed` |
| Was the top-level goal verified? | `goal_verified` plus `verification` |

Process return code is only process-level evidence. A zero return code does not
populate controller, effect, or goal fields. Plan-only and denied results are
also explicit, so a caller cannot confuse authorization with execution.

All contract values have strict `from_dict` decoders, deterministic JSON
serialization, UTC timestamps, and JSON-only details/artifacts. This phase
defines budgets but does not enforce them at runtime.
