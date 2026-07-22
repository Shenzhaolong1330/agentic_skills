# Verifier Engine (S5)

`VerifierEngine` separates `verify_effect` from `verify_goal`. An
`output_schema` verifier proves only structured output. A zero return code,
`command_executed`, or `controller_target_reached` never proves a physical
effect. Physical actions with `physical_verification_limited=true` remain
unverified without independent evidence. Unregistered task-specific verifiers
return `task_specific_verifier_not_registered`; capability verifiers are
bounded and can target only observation/check capabilities.

Goal verification requires an explicit `PredicateSpec` over current,
non-stale, non-invalidated, non-tentative facts. Fatal and unsafe errors block
goal verification. Effect and goal updates are separate immutable operations.
