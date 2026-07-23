# TaskGraph contract

Task graphs are static data graphs with `OBSERVE`, `COMPUTE`, `CHECK`, `ACT`, `VERIFY`, `RECOVER`, `APPROVAL`, and `HUMAN_ACTION` nodes. Edges have closed conditions, deterministic priorities, error routing, and optional finite traversal limits.

Arguments are typed capability arguments. `InputBinding` supports only literals, prior node JSON-pointer outputs, and S5 world fact selectors. It does not support Python attribute access, JSONPath code, eval, executable paths, argv, environment, adapters, or backends.

Physical actions must carry a verifier and have a later verification path. Expected effects cannot be marked `VERIFIED` by a graph. Recovery is bounded and must use a capability explicitly allowed as recovery; E-stop routes to human handling and never to automatic reset or home.
# Runtime note

Task graphs are static planning contracts. The Executor does not accept or recompile raw TaskGraph values; only the compiler's digest-bound `CompiledTaskGraph` can enter S7 runtime.
# S8 recovery graphs

Recovery templates are ordinary explicit task graphs with observe, compute,
action-plan, and verify nodes. They do not embed executable code and are
executed by the same GraphExecutor as the task remainder.
