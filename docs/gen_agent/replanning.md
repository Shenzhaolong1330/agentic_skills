# S8 remainder replanning

`ReplanRequest` identifies the failed node, committed facts, invalidated
facts, remaining graph, budget snapshot, failure class, and the current
execution envelope. `RemainderReplanner` returns a candidate remainder; the
compiler then produces a normal `CompiledTaskGraph`.

`validate_replan_monotonicity` enforces that a replan cannot silently broaden
authority. The candidate must preserve the mode, remain inside the original
workspace and resources, stay within remaining budgets, keep the risk class at
or below the original, use a subset of the capability allowlist, and retain
or strengthen forbidden-capability restrictions. Ambiguous blockers,
repeated no-op hashes, invalid graphs, and budget regressions are rejected.

The runtime records `REPLAN_REQUESTED`, `REPLAN_COMPILED` or
`REPLAN_REJECTED`, followed by `PLAN_REPLACED` only after the replacement has
passed validation. A rejected replan leaves the prior plan authoritative and
terminates or requests human intervention according to the policy.

The checked-in fixture replanner is deterministic and offline. It exists for
contract and acceptance coverage; it is not a language-model planner and it
does not execute a returned command or script.
